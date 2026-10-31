// ============================================================================
// lpf.h —— 滤波器库（模板实现），提供两种可直接用于飞控的平滑/估计器：
//   1) LowPassFilter<T> —— 一阶低通（指数加权移动平均 EWMA），实现最轻量、alpha 固定；
//   2) KalmanFilter<T>  —— 一维卡尔曼滤波，用过程/测量噪声模型实时权衡
//                          "信任预测"与"信任观测"，比固定 alpha 更抗噪、滞后更小。
//
// ======================== 一、LowPassFilter =================================
//
// 职责:
//   对随时间输入的信号做平滑，抑制高频噪声（陀螺仪噪声、D 项微分噪声、
//   垂直加速度振动等）。模板参数 T 需支持 - 和 * 以及 += 运算，
//   项目中实际用到 T = float（标量）与 T = Vector（向量）两种。
//
// 关键逻辑:
//   output += alpha * (input - output)
//   即 output = (1-alpha)*output + alpha*input，alpha 越大越"跟随输入"。
//   首次 update() 时直接以输入作为初值（避免从 0 缓慢爬升的启动瞬态）。
//
// 输入/输出:
//   update(input) -> 滤波后的新值，同时更新内部 output。
//   alpha 为公开成员，可被参数系统（如 EST_RATES_LPF_A）在运行时直接改写。
//
// 重要参数:
//   alpha       平滑系数，取值 [0, 1]；0 保持当前输出，1 表示直通（不滤波）。
//   cutOffFreq  目标截止频率（Hz），配合采样周期 dt 由 setCutOffFrequency 反算 alpha。
//
// 边界处理与注意事项:
//   - 每次 update 都把 alpha 限制到 [0, 1]；NAN/INF 回退为 1（直通），避免输出发散。
//   - 直通分支同步 output 和初始化状态，切回滤波时从最后一次输入继续，不会跳变。
//   - setCutOffFrequency 的频率或 dt 非正、非有限时回退为直通。
//   - reset() 只清除 initialized 标志，不清零 output；下次 update 会重新以输入初始化。
//   - T = Vector 时依赖 Vector 的 -= / * 运算符（见 vector.h），未做单位归一。
// ============================================================================

#pragma once

#include <algorithm>
#include <cmath>

template <typename T>
class LowPassFilter {
public:
	float alpha; // 平滑系数，1 表示禁用滤波（直通）
	T output;

	LowPassFilter(float alpha): alpha(sanitizeAlpha(alpha)) {};

	// 送入一个新样本，返回滤波结果（并同步更新 output）
	T update(const T input) {
		alpha = sanitizeAlpha(alpha); // 参数可被运行时直接改写，每次更新都校验

		if (!initialized) { // 首个样本：以输入为初值，避免从零缓慢爬升
			output = input;
			initialized = true;
		}

		if (alpha == 1) { // 直通也同步内部状态，切回滤波时不会重新初始化
			output = input;
			return output;
		}

		return output += alpha * (input - output);
	}

	// 由截止频率与采样周期反算 alpha（一阶 RC 离散化公式）
	// 参数无效时退回直通，避免产生 NAN 或锁死输出。
	void setCutOffFrequency(float cutOffFreq, float dt) {
		if (!std::isfinite(cutOffFreq) || !std::isfinite(dt) || cutOffFreq <= 0 || dt <= 0) {
			alpha = 1;
			return;
		}
		alpha = sanitizeAlpha(1.0f - std::exp(-6.28318530717958647692f * cutOffFreq * dt));
	}

	// 复位为"未初始化"状态；注意不会清零 output（见文件头风险说明）
	void reset() {
		initialized = false;
	}

private:
	static float sanitizeAlpha(float value) {
		return std::isfinite(value) ? std::clamp(value, 0.0f, 1.0f) : 1.0f;
	}

	bool initialized = false; // 是否已用首个样本完成初始化
};

// ============================================================================
// 二、KalmanFilter —— 一维卡尔曼滤波（标量，或对 Vector 逐轴共用同一套参数）
//
// 适用场景:
//   观测噪声较大、又希望"该跟的时候跟得快、该稳的时候稳得住"的场合，例如电池
//   电压/电流、温度、速度估计，以及 IMU 单轴的慢变零偏跟踪。相比 LowPassFilter
//   的固定 alpha，卡尔曼会随"当前估计有多可信"自动调整增益：刚启动/久未观测时
//   增益大（快速收敛），估计稳定后增益小（强力抑噪、输出平滑）。
//
// 模型（恒定状态 + 标量观测，最简一维形式）:
//   状态 x        待估计量（本实现假设每步预测值等于上一步，即 A = 1）
//   过程噪声 q    系统内在不确定度（模型越不可信/信号变化越快 → q 越大）
//   测量噪声 r    传感器噪声方差（噪声越大 → r 越大，越少采信观测）
//


// 数学公式（标准一维卡尔曼；本实现状态转移 A=1、观测矩阵 H=1、无控制量）:
//   符号约定:
//     x̂ₖ   第 k 步状态估计          zₖ   第 k 步观测（即 update 的入参 input）
//     Pₖ   估计误差协方差           Q=q  过程噪声方差   R=r  测量噪声方差
//     Kₖ   卡尔曼增益              上标 ⁻ 表示"预测值/预测协方差"
//
//   ① 预测（时间更新，A=1）:
//        x̂ₖ⁻ = x̂ₖ₋₁                     状态预测：恒等模型，预测值 = 上一估计
//        Pₖ⁻ = Pₖ₋₁ + Q                 协方差预测：叠加过程噪声 Q=过程噪声方差
//   ② 更新（测量更新，H=1）:
//        Kₖ   = Pₖ⁻ / (Pₖ⁻ + R)         增益（0 < K < 1，衡量该信观测多少）
//        x̂ₖ   = x̂ₖ⁻ + Kₖ · (zₖ - x̂ₖ⁻)    用观测修正状态
//        Pₖ   = (1 - Kₖ) · Pₖ⁻          更新协方差：观测带来信息，不确定度下降
//
//   为书写简洁，代码里把"预测值"直接记作 x、P（即 x̂ₖ⁻→x，Pₖ⁻→P），
//   于是 ① ② 与下方 update() 的 1)~4) 一一对应：
//   1) 预测: P = P + q           ← Pₖ⁻ = Pₖ₋₁ + Q
//   2) 增益: K = P / (P + r)     ← Kₖ   = Pₖ⁻ / (Pₖ⁻ + R)
//   3) 更新: x = x + K * (z - x) ← x̂ₖ   = x̂ₖ⁻ + Kₖ·(zₖ - x̂ₖ⁻)
//   4) 修正: P = (1 - K) * P     ← Pₖ   = (1 - Kₖ)·Pₖ⁻
//



// 与 LowPassFilter 的关系:
//   更新式形式完全一致，区别只是这里的 alpha 换成了实时计算的 K；若 K 恒为常数，
//   本滤波器即退化为 LowPassFilter。
//
// 输入/输出:
//   update(input) -> 滤波后的新值，同时更新内部状态 output 与协方差。
//   q、r 为公开成员，可被参数系统在运行时直接改写。
//
// 调参经验（q、r 都取方差量级，真正决定稳态行为的是比值 r/q）:
//   - r/q 越大 → 越平滑、响应越慢；r/q 越小 → 越跟随、噪声越明显。
//   - 常见做法：r 取"传感器实测噪声方差"，q 取"每步状态变化量的方差"。
//
// 边界处理与注意事项:
//   - q/r 的负值和非有限值在 update 时按 0 处理；r == 0 表示完全信任测量值。
//   - P、q、r 全为 0 时增益取 0，保持当前估计，避免 0 / 0；中间量用 double 防溢出。
//   - T = Vector 时三轴共用同一个 P/q/r（假设各轴噪声同分布）；若各轴噪声差异大，
//     应为每轴各建一个 KalmanFilter<float>。
//   - T 需支持 +(T)、-(T) 和 *(float)（工程内 float 与 Vector 均满足）。
//   - reset() 只重置状态与协方差，不改变 q/r。
// ============================================================================
template <typename T>
class KalmanFilter {
public:
	T output; // 状态估计 x（即滤波输出）
	float q;  // 过程噪声方差
	float r;  // 测量噪声方差

	// initial: 初始状态估计；q/r 含义见上。估计误差协方差 P 以 1.0 起步（中等不确定度）
	KalmanFilter(T initial, float q = 0.001f, float r = 0.1f):
		output(initial), q(q), r(r) {}

	// 送入一个观测样本，返回滤波后的状态估计（并同步更新 output 与协方差 P）
	T update(const T input) {
		// q/r 可被运行时改写；负值和非有限值均按 0 处理，r == 0 表示无测量噪声。
		q = std::isfinite(q) ? std::max(q, 0.0f) : 0.0f;
		r = std::isfinite(r) ? std::max(r, 0.0f) : 0.0f;
		if (!std::isfinite(errorCovariance) || errorCovariance < 0) {
			errorCovariance = 1.0f;
		}

		// 用 double 计算中间量，避免 P + q 或 P + r 在 float 中溢出。
		const double predictedCovariance = static_cast<double>(errorCovariance) + q;
		const double denominator = predictedCovariance + r;
		const float K = denominator > 0 ? static_cast<float>(predictedCovariance / denominator) : 0.0f;

		output = output + (input - output) * K;
		errorCovariance = denominator > 0
			? static_cast<float>(predictedCovariance * r / denominator)
			: 0.0f; // P、q、r 全为 0 时保持当前估计，避免 0 / 0。

		return output;
	}

	// 复位：把状态置为 value，并把协方差恢复到初始 1.0（q/r 保持不变）
	void reset(T value) {
		output = value;
		errorCovariance = 1.0f;
	}

private:
	float errorCovariance = 1.0f; // 估计误差协方差 P
};
