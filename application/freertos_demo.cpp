// ============================================================================
// freertos_demo.cpp —— FreeRTOS 任务调度中心（多任务架构）
//
// 职责:
//   实现 freertos_demo() 调度入口，把飞控工作拆成三个 FreeRTOS 任务：
//     controlTask      控制任务（最高优先级，核1，1kHz 固定周期）
//                      readIMU -> step -> estimate -> control -> sendMotors
//     commTask         通信任务（中优先级，核0，1kHz 轮询）
//                      readRC / readWebRC / CLI / processMavlink
//     housekeepingTask 后台任务（低优先级，核0，200Hz）
//                      电池电压 / 飞行日志 / 参数回写 NVS / LED
//
// 关键逻辑:
//   - freertos_demo() 创建 start_task，start_task 再派生三个工作任务后删除自身
//     （沿用"开始任务统一创建"的组织方式）；
//   - ESP-IDF 的调度器在 app_main 之前已启动（app_main 本身就运行在 FreeRTOS
//     主任务里），因此这里不需要也不可以调用 vTaskStartScheduler()；
//   - 控制任务用 vTaskDelayUntil 保证 1kHz 固定周期，PID 的 dt 由 step() 提供；
//   - 慢速/阻塞操作（NVS 写、HTTP、CLI 标定命令）不再阻塞控制环；
//   - 任务间同步（定义在本文件，声明见 globals.h）：
//       targetMutex   互斥保护控制目标块（attitudeTarget/ratesTarget/
//                     thrustTarget/ratesExtra/armed）：控制任务在 control()
//                     全程持有，MAVLink SET_ATTITUDE_TARGET 写入前持有
//                     （带优先级继承）；
//       controlPaused pause() 置位，控制任务暂停 estimate/control/sendMotors，
//                     供电机测试等地面命令独占 PWM；
//       其余共享量（controlRoll 等单 float、mode、batteryVoltage、channels[]）
//       为对齐 32bit 访问，ESP32 上读写天然原子，不加锁；多字结构的"组撕裂"
//       （如遥测读到半新半旧的姿态）只影响显示/日志，不影响控制正确性。
//
// 输入/输出:
//   freertos_demo() 无参数；由 app_main() 在 setup() 完成后调用一次。
//
// 边界情况与潜在风险:
//   - 栈大小单位为字节（ESP-IDF），与 STM32 裸机模板的"字"不同，勿照抄数值；
//   - 控制任务看门狗安全：vTaskDelayUntil 每周期让出；若 IMU 读取阻塞过长，
//     该任务 5s 不让出会触发 ESP_TASK_WDT 复位（阈值见 sdkconfig）；
//   - 必须在 setup() 完成、所有模块就绪后再调用 freertos_demo()，否则任务会
//     访问未初始化的外设（电机/IMU/遥控）；
//   - CLI 的参数写入（PID 增益等）与控制任务并发访问仍存在（单字长原子，
//     不会撕裂，但可能"半生效"一帧），与原单循环+HTTP 回调方案的风险等同。
// ============================================================================

#include "globals.h"
#include "freertos_demo.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// 控制周期 1ms、comm 的 vTaskDelay(1) 与 HK 的 pdMS_TO_TICKS() 都按"1 tick = 1ms"写死。
// 若 sdkconfig 把 tick 改成别的值，"1kHz 控制环"会静默变味（100Hz tick → 10ms 周期），
// 编译期直接拦下，避免上机才发现 PID 全乱。
#if defined(CONFIG_FREERTOS_HZ) && (CONFIG_FREERTOS_HZ != 1000)
#error "freertos_demo.cpp 假定 CONFIG_FREERTOS_HZ=1000（1 tick = 1 ms），请同步修改控制周期"
#endif

// ============================================================
// 任务间同步原语（声明见 globals.h）
// ============================================================
SemaphoreHandle_t targetMutex = NULL;
volatile bool controlPaused = false;
volatile bool attitudeResetRequested = false;

// ============================================================
// 任务配置
// ============================================================
#define START_TASK_PRIO      1
#define START_TASK_STACK     4096
#define START_TASK_CORE      0

#define CONTROL_TASK_PRIO    10  // 控制任务：最高应用优先级，保证 1kHz 节拍
#define CONTROL_TASK_STACK   8192
#define CONTROL_TASK_CORE    1   // 核1，避开核0上的 WiFi 协议栈
#define CONTROL_PERIOD_MS    1   // 控制周期 1ms（1kHz）

#define COMM_TASK_PRIO       5
#define COMM_TASK_STACK      8192
#define COMM_TASK_CORE       0

#define HK_TASK_PRIO         2
#define HK_TASK_STACK        4096
#define HK_TASK_CORE         0
#define HK_PERIOD_MS         5   // 200Hz：需高于日志采样率 LOG_RATE(100Hz)

// ============================================================
// 控制任务（1kHz 固定周期）：感知 -> 估计 -> 控制 -> 执行
// 环内顺序与原单循环一致；estimate/control/sendMotors 的数据流不跨任务，
// 因此控制核心链路内部无需任何锁。
// ============================================================
static void controlTask(void *pvParameters) {
	TickType_t lastWake = xTaskGetTickCount();
	while (true) {
		readIMU();               // 1) 采样 IMU（拿到最新 gyro/acc）
		step();                  // 2) 推进时间基准（更新 t/dt，全系统唯一时间源）
		if (attitudeResetRequested) { // CLI reset 请求：在本任务内执行，避免与 estimate 竞态
			attitude = Quaternion();
			resetGyroBiasTracker(); // 清空在线零偏微调并重新开始静止计数
			attitudeResetRequested = false;
		}
		if (!controlPaused) {    // pause() 期间跳过控制输出，让 CLI 独占 PWM
			estimate();          // 3) 姿态/速度估计（依赖 IMU 与时间）
			if (xSemaphoreTake(targetMutex, portMAX_DELAY) == pdTRUE) {
				control();       // 4) 控制链（含 failsafe，输出 motors[]）
				xSemaphoreGive(targetMutex);
			}
			sendMotors();        // 5) 下发电机 PWM
		}
		vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
	}
}

// ============================================================
// 通信任务（1kHz 轮询）：遥控 / 网页遥控 / CLI / MAVLink
// readRC 逐字节解析 SBUS，必须高频轮询才不丢帧；阻塞型 CLI 命令
// （标定、电机测试）在此任务执行，不再拖慢控制环。
// ============================================================
static void commTask(void *pvParameters) {
	while (true) {
		readRC();                     // 采样遥控（字节流状态机）
#if WEB_RC_ENABLED
		readWebRC();                  // 刷新网页遥控在线标志
		processConsoleCommandQueue(); // 执行网页控制台排队的命令
#endif
		handleInput();                // 处理串口 CLI 输入
#if WIFI_ENABLED
		processMavlink();             // MAVLink 遥测收发（内部限速）
#endif
		vTaskDelay(1);                // 1kHz 轮询
	}
}

// ============================================================
// 后台任务（200Hz）：电池 / 日志 / 参数回写 / LED
// 全部是"慢而可延迟"的工作：NVS 写 flash 耗时较长，移到此任务后
// 不会再像原单循环那样周期性阻塞控制环。
// ============================================================
static void housekeepingTask(void *pvParameters) {
	while (true) {
		updateBatteryVoltage();  // 刷新电池电压（供 failsafe 与显示，20ms 内有效）
		logData();               // 飞行日志采样（内部按 LOG_RATE=100Hz 限速，仅解锁时）
		syncParameters();        // 参数回写 NVS（1Hz、未飞行时，内部限速）
		updateLED();             // 刷新状态灯
		vTaskDelay(pdMS_TO_TICKS(HK_PERIOD_MS));
	}
}

// ============================================================
// 开始任务：统一创建全部工作任务，完成后删除自身
// ============================================================
static void startTask(void *pvParameters) {
	// 互斥量必须在任何任务使用之前创建（此时三个工作任务尚未启动）
	targetMutex = xSemaphoreCreateMutex();
	configASSERT(targetMutex);

	// 创建失败不能静默：没有控制任务 = 电机永远收不到指令，没有通信任务 = CLI 死等
	// （pause() 依赖控制任务推进 t）。内存不足属于启动期配置错误，直接重启并留下日志。
	if (xTaskCreatePinnedToCore(controlTask, "control", CONTROL_TASK_STACK,
	                            NULL, CONTROL_TASK_PRIO, NULL, CONTROL_TASK_CORE) != pdPASS ||
	    xTaskCreatePinnedToCore(commTask, "comm", COMM_TASK_STACK,
	                            NULL, COMM_TASK_PRIO, NULL, COMM_TASK_CORE) != pdPASS ||
	    xTaskCreatePinnedToCore(housekeepingTask, "housekeeping", HK_TASK_STACK,
	                            NULL, HK_TASK_PRIO, NULL, HK_TASK_CORE) != pdPASS) {
		print("任务创建失败（内存不足？），重启\n");
		vTaskDelay(pdMS_TO_TICKS(200));
		esp_restart();
	}

	vTaskDelete(NULL); // 删除 start_task 自身，回收栈空间
}

// ============================================================
// 任务调度总入口：创建开始任务
// 注意：ESP-IDF 中调度器在 app_main 之前已启动，app_main 本身就运行在
// FreeRTOS 主任务里，因此这里不需要也不可以调用 vTaskStartScheduler()
// ============================================================
void freertos_demo() {
	xTaskCreatePinnedToCore(startTask, "start", START_TASK_STACK,
	                        NULL, START_TASK_PRIO, NULL, START_TASK_CORE);
}
