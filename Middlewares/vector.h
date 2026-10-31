// ============================================================================
// vector.h —— 轻量三维向量库（飞控姿态/角速度/加速度的统一数据类型）
//
// 职责:
//   为全工程提供 Vector 类型（float x,y,z），承载陀螺仪角速度、加速度、
//   旋转矢量、力矩、姿态欧拉角等几乎所有三维量；并提供常用运算与几何工具。
//
// 关键逻辑:
//   - 基本运算：+ - * /（含与标量、与向量的逐元素运算）、== != += -=；
//   - 有效性判断：finite()/valid()/invalid() 基于 std::isfinite，invalidate() 置 NAN，
//     用于表达"本帧无目标/未激活"（如 attitudeTarget、ratesTarget 在 RAW/ACRO 模式下失效）；
//   - 几何工具：dot / cross / angleBetween / rotationVectorBetween。
//
// 输入/输出:
//   全部为值语义（除 normalize() 原地修改）。rotationVectorBetween(a,b) 返回把 a 转到 b
//   所需的最小旋转矢量（方向=转轴，模长=夹角，单位 rad），是姿态误差控制的核心。
//
// 边界情况与潜在风险:
//   - normalize() 不做零向量保护：对零向量调用会产生 NAN/INF。调用前须自行判断 norm()。
//   - operator/ 与逐元素除法均不检查除数为 0。
//   - == 为浮点精确比较，仅适合判断"是否被 invalidate() 置成 NAN"之类的场景，
//     不要用于比较两个近似相等的物理量。
//   - rotationVectorBetween 对零向量/平行向量做了保护（返回 0 或 π），
//     但对反向向量会临时构造一个垂直轴，结果可能不唯一（属几何固有性质）。
//   - 内部一律使用 float，长链条运算注意累积误差。
// ============================================================================

#pragma once

#include "cf_math.h"

class Vector {
public:
	float x, y, z;

	Vector(): x(0), y(0), z(0) {};

	Vector(float x, float y, float z): x(x), y(y), z(z) {};

	// 是否为零向量（精确比较）
	bool zero() const {
		return x == 0 && y == 0 && z == 0;
	}

	// 三个分量是否均为有限值（非 NAN/INF）
	bool finite() const {
		return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
	}

	bool valid() const {
		return finite();
	}

	bool invalid() const {
		return !valid();
	}

	// 把向量标记为"无效"（三分量置 NAN）。用于表达"无目标/未激活"状态，
	// 下游用 invalid() 判断后跳过对应控制环节。
	void invalidate() {
		x = NAN;
		y = NAN;
		z = NAN;
	}

	// 欧几里得范数（模长）
	float norm() const {
		return sqrt(x * x + y * y + z * z);
	}

	// 原地归一化。注意：对零向量调用会得到 NAN（无保护）
	void normalize() {
		float n = norm();
		x /= n;
		y /= n;
		z /= n;
	}

	Vector operator + (const float b) const {
		return Vector(x + b, y + b, z + b);
	}

	Vector operator * (const float b) const {
		return Vector(x * b, y * b, z * b);
	}

	Vector operator / (const float b) const {
		return Vector(x / b, y / b, z / b);
	}

	Vector operator + (const Vector& b) const {
		return Vector(x + b.x, y + b.y, z + b.z);
	}

	Vector operator - (const Vector& b) const {
		return Vector(x - b.x, y - b.y, z - b.z);
	}

	Vector& operator += (const Vector& b) {
		return *this = *this + b;
	}

	Vector& operator -= (const Vector& b) {
		return *this = *this - b;
	}

	// Element-wise multiplication
	Vector operator * (const Vector& b) const {
		return Vector(x * b.x, y * b.y, z * b.z);
	}

	// Element-wise division
	Vector operator / (const Vector& b) const {
		return Vector(x / b.x, y / b.y, z / b.z);
	}

	bool operator == (const Vector& b) const {
		return x == b.x && y == b.y && z == b.z;
	}

	bool operator != (const Vector& b) const {
		return !(*this == b);
	}

	// 点积（内积）：a·b = |a||b|cosθ
	static float dot(const Vector& a, const Vector& b) {
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}

	// 叉积（外积）：结果垂直于 a、b 构成的平面，方向由右手定则决定
	static Vector cross(const Vector& a, const Vector& b) {
		return Vector(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
	}

	// 两向量夹角（rad），内部对 cos 做 [-1,1] 钳位以规避浮点误差导致的 acos 域外
	static float angleBetween(const Vector& a, const Vector& b) {
		return acos(std::clamp(dot(a, b) / (a.norm() * b.norm()), -1.0f, 1.0f));
	}

	// 返回"把 a 转到 b"的最小旋转矢量（方向=转轴，模长=夹角 rad）。
	// 姿态误差控制的核心：controlAttitude() 用它对 upTarget/upActual 求误差。
	// 特殊情况处理：
	//   - a 或 b 近似零向量：返回 0（无法定义旋转）；
	//   - a、b 平行同向：返回 0；
	//   - a、b 平行反向：绕任一垂直轴旋转 π（轴不唯一，取与 X 或 Y 轴叉积得到的轴）。
	static Vector rotationVectorBetween(const Vector& a, const Vector& b) {
		float an = a.norm();
		float bn = b.norm();
		if (an < 1e-6 || bn < 1e-6) {
			return Vector(0, 0, 0);
		}
		Vector direction = cross(a, b);
		if (direction.norm() < 1e-6) { // 平行
			if (dot(a, b) > 0) { // 同向
				return Vector(0, 0, 0);
			}
			// 反向：任选一个与 a 垂直的轴，转 π
			Vector perp = cross(a, Vector(1, 0, 0));
			if (perp.norm() < 1e-6) {
				perp = cross(a, Vector(0, 1, 0));
			}
			perp.normalize();
			return perp * (float)M_PI;
		}
		direction.normalize();
		float angle = angleBetween(a, b);
		return direction * angle;
	}
};

// 标量在左的运算符（使 2.0f * v 与 v * 2.0f 等价）
inline Vector operator * (const float a, const Vector& b) { return b * a; }
inline Vector operator + (const float a, const Vector& b) { return b + a; }
