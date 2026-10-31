// ============================================================================
// main.cpp —— 程序入口（任务调度见 application/freertos_demo.cpp）
//
// 职责:
//   1) 定义全局状态变量（t/dt、控制输入、姿态等）；
//   2) 初始化控制台 UART（UART0，115200）；
//   3) 执行一次性初始化 setup()；
//   4) 调用 freertos_demo() 启动任务调度（control/comm/housekeeping 三任务，
//      任务结构与同步机制见 freertos_demo.cpp 头注释）。
//
// 边界情况与潜在风险:
//   - 全局变量在此定义（而非在 globals.h），其余模块通过 extern 引用；
//     新增全局量时需同步在 globals.h 声明。
//   - 初始化顺序不能随意调换：setupMotors() 必须在 sendMotors() 之前，
//     setupParameters() 必须在依赖参数的各模块之前。
//   - 若 IMU 初始化失败（imuOK=false），程序不会中止，但会禁止解锁。
// ============================================================================

// hhh-flight main entry point - ESP-IDF port
// Entry point: app_main() -> init -> freertos_demo()

#include "globals.h"
#include "freertos_demo.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

// ============================================================
// 全局变量定义
// ============================================================
float t = NAN;
float dt;
float controlRoll, controlPitch, controlYaw, controlThrottle;
float controlMode = NAN;
Vector gyro;
Vector acc;
Vector rates;
Quaternion attitude;
bool landed;

// Dummy byte for UART reading (used in IMU calibration)
uint8_t dummyByte = 0;

// ============================================================
// 控制台 UART 初始化（UART0，115200-8N1，无流控）
// 说明：只配置参数与引脚（沿用默认引脚），实际驱动安装后会由 cli.cpp 读写。
// ============================================================
static void initConsoleUART() {
	uart_config_t uart_config = {};
	uart_config.baud_rate = 115200;
	uart_config.data_bits = UART_DATA_8_BITS;
	uart_config.parity = UART_PARITY_DISABLE;
	uart_config.stop_bits = UART_STOP_BITS_1;
	uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
	uart_config.source_clk = UART_SCLK_APB;

	uart_driver_install(UART_NUM_0, 512, 512, 0, NULL, 0);
	uart_param_config(UART_NUM_0, &uart_config);
	uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
	             UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

// ============================================================
// Setup：一次性初始化（顺序有依赖，勿随意调换；必须在 freertos_demo() 之前完成）
// ============================================================
static void setup() {
	print("程序开始初始化！\n");
	// 编译时间戳：烧录后用于确认板子里跑的到底是不是刚编出来的固件
	// （排查"改了代码但行为没变"时，这行是唯一可靠的判据）
	print("固件编译时间: %s %s\n", __DATE__, __TIME__);
	disableBrownOut();   // 关闭欠压复位，避免电机启动瞬间掉压导致重启
	setupParameters();   // 最先初始化：后续模块的配置都来自参数
	setupLED();
	setupMotors();
	setLED(true);        // 初始化期间常亮
#if WIFI_ENABLED
	setupWiFi();         // 建立 UDP/ESP-NOW 链路
#endif
#if WEB_RC_ENABLED
	setupWebRC();        // 启动网页遥控 HTTP 服务
#endif
	setupIMU();          // 初始化 IMU（失败会置 imuOK=false，禁止解锁）
	setupRC();           // 初始化遥控串口
	setLED(false);
	print("程序初始化完成！\n");
	print("================================\n");
}

// ============================================================
// ESP-IDF 入口：初始化 -> 启动任务调度 -> 退出主任务
// ============================================================
extern "C" void app_main() {
	// 初始化 NVS（参数系统与 WiFi 都依赖）；损坏/版本不符时擦除后重建
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		nvs_flash_erase();
		nvs_flash_init();
	}

	// 初始化控制台 UART（UART0）
	initConsoleUART();

	// 一次性初始化（必须在任务启动前完成，避免并发初始化）
	setup();

	// 启动 FreeRTOS 任务调度（见 freertos_demo.cpp）
	freertos_demo();

	// app_main 主任务使命完成，删除自身（调度器继续运行）
	vTaskDelete(NULL);
}
