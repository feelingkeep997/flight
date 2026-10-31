// ============================================================================
// util.cpp —— 通用工具函数（数学映射、有效性判断、字符串切分、限速器、延时门）
//
// 职责:
//   提供与硬件无关的杂项工具：
//     - mapf()            线性区间映射（遥控/油门标定的基础）
//     - invalid()/valid() 浮点有效性判断（NAN/INF）
//     - wrapAngle()       角度归一化到 (-π, π]
//     - disableBrownOut() 关闭欠压复位（电池瞬时压降会导致 ESP32 意外重启）
//     - splitString()     CLI 命令行按空格切分
//     - Rate / Delay      两个轻量定时辅助类
//
// 关键逻辑:
//   - wrapAngle 用 fmodf 取模后再折到 (-π, π]，用于偏航误差计算，避免 ±π 跳变；
//   - disableBrownOut 分芯片处理：经典 ESP32 直接清寄存器位，S3/C3 调 esp_brownout_disable()。
//
// 输入/输出:
//   均为无状态纯函数，除 Rate/Delay 持有各自的时间戳状态。
//
// 边界情况与潜在风险:
//   - mapf 不做除零保护：in_max == in_min 时结果 INF/NAN，调用方需保证区间有效。
//   - splitString 就地修改传入字符串（strtok 会写 '\0'），且最多切出 3 段；
//     第 3 段保留其后的原始内容（含空格），因此"p NAME VALUE"的值可含空格。
//   - Rate 依赖全局 t，若 t 长时间不推进（如控制循环卡死）则永不触发。
//   - disableBrownOut 关闭欠压保护后，供电不足时会表现为随机崩溃/花屏而非干净复位，
//     排查供电问题时需知悉这一点。
// ============================================================================

#include "globals.h"
#include <cmath>

#include <cstring>
#include "esp_timer.h"
#include "soc/rtc_cntl_reg.h"

#ifdef CONFIG_IDF_TARGET_ESP32
#include "soc/soc.h"
#else
#include "esp_private/brownout.h"
#endif

// 标准重力加速度（m/s²）。acc 以 m/s² 为单位，垂直速度估计用它扣除重力分量。
const float ONE_G = 9.80665;

// 线性映射：把 x 从 [in_min,in_max] 线性映射到 [out_min,out_max]。
// 用于遥控通道标定、油门曲线等。不做区间有效性检查（见文件头风险）。
float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// 是否为无效浮点（NAN 或 INF）
bool invalid(float x) {
	return !isfinite(x);
}

// 是否为有效浮点
bool valid(float x) {
	return isfinite(x);
}

// 把角度归一化到 (-π, π]，用于偏航误差、角度差计算，避免跨越 ±π 时出现 2π 跳变
float wrapAngle(float angle) {
	angle = fmodf(angle, 2 * (float)M_PI);
	if (angle > M_PI) {
		angle -= 2 * (float)M_PI;
	} else if (angle < -(float)M_PI) {
		angle += 2 * (float)M_PI;
	}
	return angle;
}

// 关闭欠压复位（brownout reset）。电机启动瞬间的电压跌落会触发该复位，
// 导致飞行中意外重启，故在启动流程中调用一次禁用它。
void disableBrownOut() {
#ifdef CONFIG_IDF_TARGET_ESP32
	// 经典 ESP32：直接清 RTC_CNTL 中的欠压使能位
	REG_CLR_BIT(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ENA);
#else
	// S3/C3：调用 IDF 提供的接口
	esp_brownout_disable();
#endif
}

// 按空格/制表符把 str 切成最多 3 段。
// 参数:
//   str     输入字符串，会被就地修改（strtok 会写入 '\0'）
//   token0  第 1 段（命令名）
//   token1  第 2 段（第一个参数）
//   token2  第 3 段（剩余全部内容，可含空格）
//   maxLen  各输出缓冲区的容量，内部按 maxLen-1 截断并补 '\0'
// 注意：token2 用 strtok(NULL, "") 取得，会保留其后的空格，适合"p NAME VALUE"形式。
void splitString(char* str, char* token0, char* token1, char* token2, int maxLen) {
	// 跳过前导空白
	while (*str == ' ' || *str == '\t') str++;

	token0[0] = token1[0] = token2[0] = '\0';

	char* tok = strtok(str, " \t");
	if (tok) {
		strncpy(token0, tok, maxLen - 1);
		token0[maxLen - 1] = '\0';
	}
	tok = strtok(NULL, " \t");
	if (tok) {
		strncpy(token1, tok, maxLen - 1);
		token1[maxLen - 1] = '\0';
	}
	tok = strtok(NULL, "");
	if (tok) {
		strncpy(token2, tok, maxLen - 1);
		token2[maxLen - 1] = '\0';
	}
}

// Rate::operator bool —— 限速器：按设定频率返回 true。
// 用法：static Rate rate(10); if (rate) { ... }  // 每秒最多执行 10 次
// 依赖全局时间 t；首次调用时 last=0，若 t 已大于 1/rate 会立即触发一次。
Rate::operator bool() {
	if (t - last >= 1.0f / rate) {
		last = t;
		return true;
	}
	return false;
}

// Delay::update —— 布尔信号延时确认（防抖）：输入持续为 true 达 delay 秒后才返回 true。
// 一旦输入变 false 立即复位计时。用于"遥控丢失持续 N 秒才触发失控"这类判定。
bool Delay::update(bool on) {
	if (!on) {
		start = NAN;
		return false;
	} else if (isnan(start)) {
		start = t;
	}
	return t - start >= delay;
}
