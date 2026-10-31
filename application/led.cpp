// ============================================================================
// led.cpp —— 板载状态指示灯
//
// 职责:
//   用一颗 GPIO 驱动的 LED 表达飞行器状态：
//     未解锁 + 电量告警 → 8Hz 快闪
//     未解锁 + 正常     → 常灭
//     已解锁 + 有告警   → 8Hz 快闪（倒置 / 遥控丢失 / 网页失联 / 低电量）
//     已解锁 + 正常     → 1Hz 慢闪
//
// 关键逻辑:
//   - 闪烁不依赖定时器，而是在每个控制周期由 updateLED() 用 esp_timer 时间取模决定亮灭；
//   - 通过 BOARD_LED_ENABLED / BOARD_LED_INVERTED 适配不同板卡（共阴/共阳、有无 LED）；
//   - 板卡未启用 LED 时，本文件在编译期退化为一批空实现（见文件末 #else 分支）。
//
// 输入/输出:
//   输入：armed / batteryVoltage / thrustTarget / isInverted / controlTime / t /
//         webRCEnabled / useWebRC / isUsingWebRC()；
//   输出：GPIO 电平（无返回值）。
//
// 重要参数:
//   BLINK_PERIOD(500000us) 慢闪半周期；BLINK_FAST_PERIOD(62500us) 快闪半周期。
//   BOARD_LED_PIN / BOARD_LED_INVERTED 板级宏（见 board_config.h）。
//
// 边界情况与潜在风险:
//   - now_us() 把 esp_timer 的 int64 微秒截断成 uint32，约 71.6 分钟回绕一次；
//     取模闪烁相位会因此跳变一次，属可接受的视觉抖动。
//   - setLED 内部缓存 state，重复调用同值不会重复写 GPIO；但若外部直接改 GPIO 会失同步。
//   - 电量告警阈值区分"飞行中"与"地面"（见 batteryAlertActive），避免地面待机误闪。
// ============================================================================

#include "globals.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#if BOARD_LED_ENABLED

#define BLINK_PERIOD      500000  // 慢闪半周期 500ms -> 1Hz
#define BLINK_FAST_PERIOD  62500  // 快闪半周期 62.5ms -> 8Hz

// 取当前时间（微秒，截断为 32 位）。见文件头回绕说明。
static uint32_t now_us() {
	return (uint32_t)(esp_timer_get_time());
}

// 初始化 LED GPIO（输出模式），并按极性点亮/熄灭到"灭"
void setupLED() {
	gpio_config_t io_conf = {};
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = (1ULL << BOARD_LED_PIN);
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
	gpio_config(&io_conf);

	// 初始熄灭（BOARD_LED_INVERTED 表示低电平点亮）
	gpio_set_level((gpio_num_t)BOARD_LED_PIN, BOARD_LED_INVERTED ? 1 : 0);
}

// 设置 LED 亮/灭（on=true 点亮）。内部缓存状态，重复调用不重复写 GPIO。
void setLED(bool on) {
	static bool state = false;
	if (on == state) return;
	gpio_set_level((gpio_num_t)BOARD_LED_PIN, (on ^ BOARD_LED_INVERTED) ? 1 : 0);
	state = on;
}

// 以慢闪节奏闪烁一次（按当前时间取模决定亮灭）
void blinkLED() {
	setLED(now_us() / BLINK_PERIOD % 2);
}

// 是否处于电量告警：飞行中阈值更低（3.3V），地面阈值更高（3.5V）。
// 未接电池检测（<= 0.5V）时不告警。
bool batteryAlertActive() {
	if (batteryVoltage <= VBAT_ABSENT_THRESHOLD) return false;
	bool flying = armed && thrustTarget >= 0.15f;
	if (flying) return batteryVoltage < VBAT_LOW_THRESHOLD;
	else        return batteryVoltage < VBAT_WARN_THRESHOLD;
}

// 是否存在任何需要快闪的告警：倒置 / 遥控丢失 / 网页失联 / 低电量
bool ledAlertActive() {
	if (isInverted) return true;
	if (controlTime != 0 && armed && (t - controlTime > rcLossTimeout)) return true;
#if WEB_RC_ENABLED
	if (webRCEnabled && useWebRC && !isUsingWebRC()) return true;
#endif
	if (batteryAlertActive()) return true;
	return false;
}

// 每控制周期刷新 LED 状态（决定常灭 / 慢闪 / 快闪）
void updateLED() {
	if (!armed) { // 未解锁
		if (batteryAlertActive()) {
			setLED(now_us() / BLINK_FAST_PERIOD % 2); // 电量告警快闪
		} else {
			setLED(false); // 正常常灭
		}
		return;
	}
	if (ledAlertActive()) {
		setLED(now_us() / BLINK_FAST_PERIOD % 2); // 告警快闪
	} else {
		setLED(now_us() / BLINK_PERIOD % 2);      // 正常慢闪
	}
}

#else
// ---- 板卡未启用 LED：全部退化为空实现，保证链接不报错 ----

void setupLED() {}
void setLED(bool on) { (void)on; }
void blinkLED() {}
void updateLED() {}

#endif
