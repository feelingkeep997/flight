// ============================================================================
// pid.h —— 单路 PID 控制器（带积分抗饱和与 D 项低通 / D-on-measurement）
//
// 职责:
//   提供飞控中最基础的闭环控制单元。本项目共实例化 6 个 PID：
//     - 角速率环：rollRatePID / pitchRatePID / yawRatePID（误差 = 角速度 rad/s）
//     - 姿态环：  rollPID / pitchPID / yawPID（误差 = 姿态角/旋转矢量，rad）
//   全部在 control.cpp 中构造，增益可由参数系统（CTL_*_P/I/D）在线改写。
//
// 关键逻辑:
//   dt 由全局时间基准 t 与上一次调用时刻 prevTime 之差得到；
//   仅在 0 < dt < dtMax 时才做积分与微分（异常 dt 视为一次断点，清零 I/D 防爆），
//   输出 = p*error + clamp(i*integral, ±windup) + d*derivative。
//   I 项为双重抗饱和：积分"本体"先按 ±windup/|i| 限幅，输出贡献再按 ±windup 限幅。
//   只压输出贡献时，integral 本体仍可无界增长，误差反向后需先"退饱和"才出钳，
//   表现为松杆后缓慢反打，故本体也要限。
//   D 项默认对误差微分；给出 measurement 时切换为 D-on-measurement（只对测量值微分）。
//
// 输入/输出:
//   update(error)              -> 控制量（float，单位随使用场景而定），D 对误差微分。
//   update(error, measurement) -> 同上，但 D 只对 measurement 微分（消除微分冲击）。
//   内部维护 prevError / prevMeasurement / prevTime / integral / derivative / lpf 状态。
//
// 重要参数:
//   p, i, d   比例/积分/微分增益。
//   windup    积分项限幅（抗积分饱和）；为 0 时 I 项被完全钳死（等于关闭积分）。
//   dAlpha    D 项低通滤波系数（1 = 不过滤；越小越平滑但相位滞后越大）。
//   dtMax     单步允许的最大时间间隔（s），默认 0.1；超过则认为控制断流并复位 I/D。
//
// 边界处理与注意事项:
//   - 依赖全局 t（外部时钟），必须保证每个控制周期都调用 update()；否则 dt 累积过大会
//     触发 dt <= 0 || dt >= dtMax 分支，清零 I/D 并复位微分滤波器。
//   - reset() 把 prevTime/prevError/prevMeasurement 置 NAN，下一帧 dt 为 NAN，会再次走
//     复位分支，属于刻意设计（保证复位后第一帧不产生 I/D 冲击）。
//   - 微分项对噪声敏感，故默认经 lpf 平滑；若把 dAlpha 设为 1 需自行确保输入干净。
//   - measurement 必须与 error 同单位、同正方向（内环传角速率 rad/s，不能误传角度）；
//     传错单位不会崩溃，但 D 项会按错误量纲工作。measurement 传 NAN 即退回原行为。
//   - integral 本体限幅为 windup/|i|：i == 0 时本体恒为 0（此时积分本来也不起作用）。
//     windup 为负或非有限值时按 0 处理；error 非有限时复位控制器并返回 0。
//   - 未做输出限幅，饱和保护由调用方（controlTorque 的 desaturate/clamp）负责。
// ============================================================================

#pragma once

#include "lpf.h"
#include "cf_math.h"

// 全局时间基准（定义在 time.cpp，由 step() 每个控制周期推进）
extern float t;

class PID {
public:
	float p, i, d;      // 比例、积分、微分增益
	float windup;       // 积分项限幅（抗积分饱和）
	float dtMax;        // 允许的最大 dt（s），超过即判定控制断流

	float derivative = 0; // 最近一次计算出的微分项（已滤波）
	float integral = 0;   // 累积积分项

	// 条件积分（抗饱和）开关：置 true 时本帧只算 P/D、不累加积分。
	// 由调用方在执行机构已经顶到限幅时置位（见 control.cpp 的 controlTorque），
	// 用法是"本帧发现饱和 → 下一帧冻结"，避免"机体跟不动 → 积分越涨越大 →
	// 差动拉满 → 单侧翘起翻机"的正反馈。
	bool freezeIntegral = false;

	LowPassFilter<float> lpf; // D 项低通滤波器

	PID(float p, float i, float d, float windup = 0, float dAlpha = 1, float dtMax = 0.1) :
		p(p), i(i), d(d), windup(windup), dtMax(dtMax), lpf(dAlpha) {}

	// 送入当前误差，返回 PID 输出（同时推进内部状态）。
	// measurement 省略 = D 对误差微分（原行为）；给出该轴实测值（单位与 error 一致，
	// 如角速率 rad/s）= D-on-measurement，setpoint 跳变不再产生微分冲击。
	// 两种用法可在运行中切换，切换后首帧 D 为 0（不冲击）。
	float update(float error, float measurement = NAN) {
		if (!std::isfinite(error)) { // 非有限误差不得污染积分和微分状态
			reset();
			return 0;
		}

		const float windupLimit = std::isfinite(windup) ? std::max(windup, 0.0f) : 0.0f;
		float dt = t - prevTime; // 与上次调用的时间差

		if (dt > 0 && dt < dtMax) {
			// I：累积后对"本体"限幅（输出 clamp 只压贡献，本体无界会延长退饱和时间）
			// freezeIntegral=true（上一帧执行机构已饱和）时跳过累加，只保留当前值，
			// 退饱和后再继续积分 —— 这就是条件积分式抗饱和。
			if (!freezeIntegral) {
				integral += error * dt;                              // 累积积分
				float integralLimit = (i != 0) ? windupLimit / std::fabs(i) : 0;
				integral = std::clamp(integral, -integralLimit, integralLimit);
			}

			// D：有测量值时只对测量值微分（d(error)/dt = -d(measurement)/dt）。
			// prevMeasurement 未初始化（首帧 / 刚切换用法）时本帧不出 D，避免 NAN 污染 lpf。
			float dInput;
			if (std::isfinite(measurement)) {
				dInput = std::isfinite(prevMeasurement) ? -(measurement - prevMeasurement) : 0;
			} else {
				dInput = std::isfinite(prevError) ? error - prevError : 0;
			}
			derivative = lpf.update(dInput / dt);                    // 求微分并低通
		} else {
			// 首次调用（prevTime=NAN）或时间异常跳变：本帧不做 I/D，避免数值爆炸
			integral = 0;
			derivative = 0;
			lpf.reset(); // 同时丢弃断流前的滤波状态，恢复后不复用旧微分
		}

		prevError = error;
		prevMeasurement = measurement;
		prevTime = t;

		return p * error + std::clamp(i * integral, -windupLimit, windupLimit) + d * derivative;
	}

	// 复位控制器状态（用于解锁/切换模式/失控保护等场景，避免残留积分造成突跳）
	void reset() {
		prevError = NAN;
		prevMeasurement = NAN;
		prevTime = NAN;
		integral = 0;
		derivative = 0;
		lpf.reset();
	}

private:
	float prevError = NAN;       // 上一次误差（首次为 NAN，用于识别"首帧"）
	float prevMeasurement = NAN; // 上一次测量值（仅 D-on-measurement 路径使用）
	float prevTime = NAN;        // 上一次调用时间（首次为 NAN）
};
