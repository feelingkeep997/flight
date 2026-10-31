// ============================================================================
// time.cpp —— 全局时间基准与主循环节拍
//
// 职责:
//   维护全局时间 t（秒，自系统启动以来的单调时间）与相邻两次 step() 的间隔 dt，
//   并统计主循环实际运行频率 loopRate。整套控制/滤波/PID 都依赖这两个量。
//
// 关键逻辑:
//   step() 用 esp_timer_get_time()（微秒，单调递增、不受网络对时影响）换算成秒，
//   更新 dt 与 t；若 dt 非正（首帧或复位）则强制为 0，避免除零/负时间。
//   computeLoopRate() 用 1 秒滑动窗口统计 step() 调用次数，得到真实循环频率。
//
// 输入/输出:
//   无参数。副作用是更新全局 t / dt / loopRate。
//
// 边界情况与潜在风险:
//   - esp_timer_get_time() 返回 int64，这里先转 float 再除 1e6；运行极久（数天）后
//     float 精度下降会让 dt 出现抖动，长航时任务需留意。
//   - dt 只在"非正"时归零，不做上限钳位；若某次循环被长时间阻塞（如 HTTP 处理），
//     dt 会变成一个大值——上层 PID 用 dtMax 兜底、速度积分则直接乘 dt，需知悉。
//   - loopRate 为整数计数（每秒更新一次），首秒内为 0。
//   - t 的初值为 NAN（定义在 main.cpp），因此第一次 step() 得到的 dt 为 NAN，
//     被 !(dt > 0) 判定为 0，属预期行为。
// ============================================================================

#include "globals.h"
#include "esp_timer.h"

float loopRate = 0.0f; // 主循环实测频率（Hz），由 computeLoopRate() 每秒刷新一次

// 推进全局时间：更新 t 与 dt。由 main.cpp 的 loop() 以及 pause() 等需要"走时间"的地方调用。
void step() {
	float now = (float)esp_timer_get_time() / 1000000.0f; // 微秒 -> 秒
	dt = now - t;
	t = now;

	if (!(dt > 0)) { // 首帧（t 为 NAN）或时间回退：视作 0，保证下游不出现除零/负 dt
		dt = 0;
	}

	computeLoopRate();
}

// 用 1 秒滑动窗口统计 step() 被调用的次数，得到实际主循环频率（Hz）
void computeLoopRate() {
	static float windowStart = 0;
	static uint32_t rate = 0;
	rate++;
	if (t - windowStart >= 1) { // 满 1 秒即刷新一次 loopRate 并重置计数
		loopRate = rate;
		windowStart = t;
		rate = 0;
	}
}
