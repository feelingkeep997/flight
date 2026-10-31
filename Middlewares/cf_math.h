// ============================================================================
// cf_math.h —— 飞控全局数学辅助（纯内联、无硬件依赖）
//
// 职责:
//   1) 提供角度/弧度互换的两个便捷函数 degrees() / radians()；
//   2) 集中引入 C++ 标准数学与算法库，使其它模块只需 #include "cf_math.h"
//      即可使用 std::clamp / std::min / std::max / std::abs / std::isfinite 等。
//
// 关键逻辑:
//   degrees/radians 为线性换算，M_PI 由 <cmath> 提供（依赖 GNU 扩展宏）。
//
// 输入/输出:
//   degrees(rad) -> 角度值；radians(deg) -> 弧度值。
//   两者均为 inline，无内部状态、无副作用，可在任意上下文安全调用。
//
// 边界情况与潜在风险:
//   - 不做入参范围检查：传入 NAN/INF 时返回值同样是 NAN/INF，调用方需自行判有效
//     （项目里惯用 valid()/invalid() 做判断）。
//   - 精度为 float（约 7 位有效十进制数字），不适合高精度长期积分/大地测量场景。
// ============================================================================

#pragma once

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <algorithm>

// 弧度 -> 度。用于把内部弧度表示换算成人类可读的度数（打印/日志）。
inline float degrees(float rad) { return rad * 180.0f / static_cast<float>(M_PI); }

// 度 -> 弧度。用于把配置/参数中的度数（如 CTL_TILT_MAX）换算成内部弧度。
inline float radians(float deg) { return deg * static_cast<float>(M_PI) / 180.0f; }
