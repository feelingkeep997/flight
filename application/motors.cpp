// ============================================================================
// motors.cpp —— 电机 PWM 输出（LEDC 外设，支持 MOSFET 直驱 / 电调两种模式）
//
// 职责:
//   把 control.cpp 算出的归一化推力 motors[4]（0~1）转换成 4 路硬件 PWM 输出。
//   使用 ESP-IDF 的 LEDC（LED PWM 控制器）外设——注意 LEDC 只是外设名称，
//   与板载 LED 指示灯无关。
//
// 关键逻辑:
//   - setupMotors() 配置一个共用 LEDC 定时器（4 通道同频同分辨率）+ 4 个通道，
//     通道号与电机下标一一对应；配置前先把引脚拉低，确保上电不转；
//   - getDutyCycle() 做单位换算，分两种模式：
//       pwmMax >= 0：电调模式，先映射成脉宽 µs 再换算成占空比计数（value==0 用 pwmStop 停机脉宽）；
//       pwmMax <  0：纯占空比模式（MOSFET 直驱），0~1 直接线性对应 0~满量程；
//   - sendMotors() 每控制周期下发一次占空比。
//
// 输入/输出:
//   输入：全局 motors[4]（由 controlTorque 写入）、pwm* 参数（可由参数系统改写）。
//   输出：4 路 GPIO 上的 PWM 波形。motorsActive() 供 estimate.cpp 判断是否离地。
//
// 重要参数:
//   pwmFrequency  默认 25kHz（高于人耳听阈，避免啸叫）
//   pwmResolution 默认 10bit（占空比 0~1023）
//   pwmStop/pwmMin/pwmMax  电调模式的停机/最小/最大脉宽（µs）；pwmMax=-1 表示直驱模式
//
// 边界情况与潜在风险:
//   - 修改 MOT_PWM_FREQ/RES 或 MOT_PIN_* 参数会通过回调重新调用 setupMotors()，
//     重复配置 LEDC 定时器/通道属幂等操作，但会短暂打断输出。
//   - motors_initialized 未置位前 sendMotors() 直接返回，防止初始化前写通道。
//   - getDutyCycle / setupMotors 对 pwmFrequency(<=0) 与 pwmResolution(不在 1~14)
//     都做了钳位兜底，参数被写坏时不会除零或移位溢出。
//   - 电机下标顺序（RL/RR/FR/FL）与混控符号必须与机架实际转向一致，接错会翻转。
//   - testMotor() 会占用通信任务 3 秒（内部用 pause 走时间），期间控制逻辑不推进；
//     已解锁时直接拒绝执行，避免"控制环停摆 + 单电机常转"导致翻机。
// ============================================================================

#include "globals.h"
#include "cf_math.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 4 路电机的归一化推力，取值 0.0 ~ 1.0（0 = 停转，1 = 满油门）。
// 下标使用下方 MOTOR_* 常量；由 control.cpp 的 controlTorque() 写入混合结果，
// 再由 sendMotors() 转换为 PWM 占空比输出。
float motors[4];

// 4 路电机的 PWM 输出引脚，顺序与 MOTOR_* 下标一一对应，来自板级配置 BOARD_MOTOR_PINS
int motorPins[4] = BOARD_MOTOR_PINS;

// PWM 参数（可通过参数系统 MOT_PWM_* 修改，修改后需重新调用 setupMotors() 生效）
int pwmFrequency = 25000; // PWM 频率 Hz：25 kHz 高于人耳听阈，避免电机发出可闻啸叫
int pwmResolution = 10;   // PWM 分辨率 bit：10 bit => 占空比计数范围 0 ~ 1023
int pwmStop = 0;          // 电调模式下「停机」脉宽（µs）
int pwmMin = 0;           // 电调模式下最小脉宽（µs）
int pwmMax = -1;          // 电调模式下最大脉宽（µs）；-1 = 纯占空比模式（MOSFET 直驱）

// 电机下标常量：数值同时作为 LEDC 通道号使用（见 setupMotors() 中的 channel = i）
const int MOTOR_REAR_LEFT = 0;
const int MOTOR_REAR_RIGHT = 1;
const int MOTOR_FRONT_RIGHT = 2;
const int MOTOR_FRONT_LEFT = 3;

static ledc_timer_bit_t resolution_bits = LEDC_TIMER_10_BIT; // 定时器分辨率缓存（记录当前配置值，供调试参考）
static bool motors_initialized = false;                      // LEDC 通道是否已配置完成，防止初始化前写占空比

// 毫秒级延时封装（换算为 FreeRTOS tick），仅用于非实时路径（如电机测试）
void delay_ms(uint32_t ms) {
	vTaskDelay(pdMS_TO_TICKS(ms));
}

// PWM 分辨率合法区间（ESP32 LEDC 低速模式支持 1~14 bit）。
// MOT_PWM_RES 是可写参数，写成 0/负数/过大都会让 (1 << res) 溢出或得到 0 满量程，
// 因此所有用到分辨率的地方都先过这一层钳位。
static int safePwmResolution() {
	return (pwmResolution >= 1 && pwmResolution <= 14) ? pwmResolution : 10;
}

// 初始化电机 PWM：配置 LEDC 定时器与 4 路通道，并先把引脚拉低确保上电不转
void setupMotors() {
	print("Setup Motors\n");

	// ---- 配置 LEDC 定时器（4 路通道共用，保证频率与分辨率一致）----
	int resBits = safePwmResolution();
	int freqHz = (pwmFrequency > 0) ? pwmFrequency : 25000; // MOT_PWM_FREQ=0 会让 LEDC 拒绝配置

	ledc_timer_config_t timer_conf = {};
	timer_conf.speed_mode = LEDC_LOW_SPEED_MODE;
	timer_conf.duty_resolution = (ledc_timer_bit_t)resBits;
	timer_conf.timer_num = LEDC_TIMER_0;
	timer_conf.freq_hz = freqHz;
	timer_conf.clk_cfg = LEDC_AUTO_CLK;
	esp_err_t ret = ledc_timer_config(&timer_conf);
	if (ret != ESP_OK) {
		print("LED timer config failed: %d\n", ret);
	}

	resolution_bits = (ledc_timer_bit_t)resBits;

	// ---- 逐路配置 4 个 LEDC 通道（通道号 == 电机下标）----
	for (int i = 0; i < 4; i++) {
		// 先将 GPIO 设为输出并置低：在 LEDC 接管引脚前保证电机处于停转状态
		gpio_set_direction((gpio_num_t)motorPins[i], GPIO_MODE_OUTPUT);
		gpio_set_level((gpio_num_t)motorPins[i], 0);

		ledc_channel_config_t chan_conf = {};
		chan_conf.speed_mode = LEDC_LOW_SPEED_MODE;
		chan_conf.channel = (ledc_channel_t)i;
		chan_conf.timer_sel = LEDC_TIMER_0;
		chan_conf.intr_type = LEDC_INTR_DISABLE;
		chan_conf.gpio_num = motorPins[i];
		chan_conf.duty = 0; // 初始占空比 0 = 停转
		chan_conf.hpoint = 0;

		ret = ledc_channel_config(&chan_conf);
		print("  motor%d pin=%d ledc=%s\n", i, motorPins[i], ret == ESP_OK ? "OK" : "FAIL");
	}

	motors_initialized = true;
	sendMotors(); // 立即下发一次 0 占空比，使输出状态确定
	print("Motors initialized\n");
}

// 将归一化推力（0 ~ 1）换算为 LEDC 占空比计数值（0 ~ 2^pwmResolution - 1）
// 电调模式下需要用 pwmFrequency 把脉宽换算成占空比，而 MOT_PWM_FREQ 是可写参数：
// 一旦被写成 0（或负数）就会除零/得到负周期，因此这里做下限保护。
int getDutyCycle(float value) {
	value = std::clamp(value, 0.0f, 1.0f);
	const int resBits = safePwmResolution();
	const float fullScale = (float)((1 << resBits) - 1);
	if (pwmMax >= 0) { // 电调（ESC）模式：以脉宽 µs 表示油门
		// 先把 0~1 映射到 pwmMin~pwmMax 脉宽；value==0（停转）时改用 pwmStop 停机脉宽
		float pwm = mapf(value, 0, 1, (float)pwmMin, (float)pwmMax);
		if (value == 0) pwm = pwmStop;
		// 再把脉宽 µs 换算为占空比：单个 PWM 周期 = 1e6 / pwmFrequency µs
		// pwmFrequency 非法时退化为 50Hz（标准电调刷新率），避免除零
		int freqHz = (pwmFrequency > 0) ? pwmFrequency : 50;
		float periodUs = 1000000.0f / (float)freqHz;
		float duty = mapf(pwm, 0, periodUs, 0, fullScale);
		return (int)round(std::clamp(duty, 0.0f, fullScale));
	} else { // 纯占空比模式（MOSFET 直驱）：0 ~ 1 直接线性对应 0 ~ 满量程
		return (int)round(value * fullScale);
	}
}

// 将 motors[] 当前值下发到 4 路 PWM 输出（每个控制周期调用一次）
void sendMotors() {
	if (!motors_initialized) return; // 通道尚未配置完成，直接返回避免无效写
	for (int i = 0; i < 4; i++) {
		ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i, getDutyCycle(motors[i]));
		ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i); // 使新占空比立即生效
	}
}

// 是否存在任一电机输出非零；供 estimate.cpp 判定飞行器是否已离地（landed 估计）使用
bool motorsActive() {
	return motors[0] != 0 || motors[1] != 0 || motors[2] != 0 || motors[3] != 0;
}

// 单电机测试：以 30% 油门转动 3 秒后停转。
// 由 CLI 命令 mfr/mfl/mrr/mrl 调用，用于装机后桨叶转向/接线检查。
// 多任务改造：必须先置 controlPaused 挂起控制任务，否则控制环每 1ms 就会把
// motors[] 覆盖掉（未解锁时清零），测试电机根本不会转。
// 安全约束：已解锁时直接拒绝执行。旧版不做任何检查，一旦在解锁状态下误敲
// mfr/mfl，控制环被挂起 3 秒、failsafe 同时停摆，而单个电机以 30% 常转 ——
// 飞机会立刻翻掉。测试必须在上锁状态下进行。
void testMotor(int n) {
	if (armed) {
		print("已解锁状态禁止测试电机，请先 disarm\n");
		return;
	}
	if (n < 0 || n > 3) {
		print("电机编号非法: %d（应为 0~3）\n", n);
		return;
	}
	print("Testing motor %d\n", n);
	controlPaused = true;   // 挂起控制任务输出，独占 motors[]/PWM（pause() 结束时恢复原值）
	motors[n] = 0.3f;
	delay_ms(50);
	sendMotors();
	pause(3);
	motors[n] = 0;
	sendMotors();
	controlPaused = false;  // 显式清零：pause() 只是恢复调用前的值，这里由本函数负责收尾
	print("Done\n");
}
