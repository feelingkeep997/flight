// ============================================================================
// quaternion.h —— 轻量旋转四元数库（姿态表示与旋转运算）
//
// 职责:
//   用单位四元数 (w,x,y,z) 表示机体姿态，并提供构造/旋转/换算等运算。
//   相比欧拉角可避免万向锁，是姿态解算（estimate.cpp）与控制（control.cpp）的基础。
//
// 关键逻辑:
//   - 约定：四元数描述"机体坐标系 -> 世界坐标系"的旋转；
//   - fromEuler 使用 ZYX 内旋（yaw-pitch-roll）顺序，与 toEuler 互为逆运算；
//   - rotate() 用于把增量旋转（如陀螺仪积分）叠加到当前姿态上；
//   - conjugate(v) = q·v·q⁻¹ 把向量从机体系旋到世界系；
//     conjugateInversed(v) = q⁻¹·v·q 反向，把世界系向量旋到机体系；
//     rotateVector(v,q) 等价于 conjugateInversed(v)，即 **世界系 → 机体系**
//     （它的行内注释曾写反，已改正，详见函数处说明）；
//   - 所有构造/运算后应保持单位化（rotate/between 默认 normalize=true）。
//
// 输入/输出:
//   全部为值语义。fromRotationVector 输入旋转矢量（模长=弧度）返回四元数；
//   toEuler() 返回 Vector(roll, pitch, yaw)（rad）；getRoll/Pitch/Yaw 为便捷取值。
//
// 边界情况与潜在风险:
//   - fromAxisAngle 未做零轴保护：axis.norm()==0 时会产生 NAN/INF。
//     （fromRotationVector 已对零旋转矢量做短路保护，可安全使用。）
//   - toAxisAngle/toRotationVector 在 w≈1（零旋转）或 sin(angle/2)≈0 时数值不稳定，
//     故 toRotationVector 对恒等四元数做了特判。
//   - toEuler 在俯仰接近 ±90° 时（|sarg|≈1）进入奇异分支，roll 被强制置 0，
//     此时 yaw/roll 不可分辨（万向锁），仅 yaw 组合有意义。
//   - 直接改写 w/x/y/z（如 MAVLink 的 SET_ATTITUDE_TARGET）后须保证单位化。
//   - 精度为 float，长时间积分需依赖外部（加速度计/level 修正）抑制漂移。
// ============================================================================

#pragma once

#include "vector.h"

class Quaternion {
public:
	float w, x, y, z;

	Quaternion(): w(1), x(0), y(0), z(0) {}; // 默认恒等旋转

	Quaternion(float w, float x, float y, float z): w(w), x(x), y(y), z(z) {};

	// 由转轴 + 转角构造。注意 axis 需为单位向量，且不可为零向量
	static Quaternion fromAxisAngle(const Vector& axis, float angle) {
		float halfAngle = angle * 0.5;
		float sin2 = sin(halfAngle);
		float cos2 = cos(halfAngle);
		float sinNorm = sin2 / axis.norm();
		return Quaternion(cos2, axis.x * sinNorm, axis.y * sinNorm, axis.z * sinNorm);
	}

	// 由旋转矢量构造（方向为转轴，模长为转角 rad）；零矢量安全返回恒等四元数
	static Quaternion fromRotationVector(const Vector& rotation) {
		if (rotation.zero()) {
			return Quaternion();
		}
		return Quaternion::fromAxisAngle(rotation, rotation.norm());
	}

	// 由欧拉角 (roll,pitch,yaw) 构造，使用 ZYX 内旋顺序，与 toEuler 互逆
	static Quaternion fromEuler(const Vector& euler) {
		float cx = cos(euler.x / 2);
		float cy = cos(euler.y / 2);
		float cz = cos(euler.z / 2);
		float sx = sin(euler.x / 2);
		float sy = sin(euler.y / 2);
		float sz = sin(euler.z / 2);

		return Quaternion(
			cx * cy * cz + sx * sy * sz,
			sx * cy * cz - cx * sy * sz,
			cx * sy * cz + sx * cy * sz,
			cx * cy * sz - sx * sy * cz);
	}

	// 构造把向量 u 旋转到向量 v 的最短旋转四元数（内部会归一化）
	static Quaternion fromBetweenVectors(const Vector& u, const Vector& v) {
		float dot = u.x * v.x + u.y * v.y + u.z * v.z;
		float w1 = u.y * v.z - u.z * v.y;
		float w2 = u.z * v.x - u.x * v.z;
		float w3 = u.x * v.y - u.y * v.x;

		Quaternion ret(
			dot + sqrt(dot * dot + w1 * w1 + w2 * w2 + w3 * w3),
			w1,
			w2,
			w3);
		ret.normalize();
		return ret;
	}

	// 各分量是否均为有限值（非 NAN/INF）
	bool finite() const {
		return std::isfinite(w) && std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
	}

	bool valid() const {
		return finite();
	}

	bool invalid() const {
		return !valid();
	}

	// 标记为无效（置 NAN），表达"无姿态目标"（RAW/ACRO 模式下会用到）
	void invalidate() {
		w = NAN;
		x = NAN;
		y = NAN;
		z = NAN;
	}

	// 四元数模长（单位四元数应恒为 1）
	float norm() const {
		return sqrt(w * w + x * x + y * y + z * z);
	}

	// 原地归一化。注意：零四元数调用会产生 NAN（无保护）
	void normalize() {
		float n = norm();
		w /= n;
		x /= n;
		y /= n;
		z /= n;
	}

	// 转成 (转轴, 转角)。注意：零旋转（angle≈0）时 sin(angle/2)≈0，除法会失稳
	void toAxisAngle(Vector& axis, float& angle) const {
		angle = acos(w) * 2;
		axis.x = x / sin(angle / 2);
		axis.y = y / sin(angle / 2);
		axis.z = z / sin(angle / 2);
	}

	// 转成旋转矢量（轴 * 角度）。对恒等四元数特判返回零矢量，规避 toAxisAngle 的除零
	Vector toRotationVector() const {
		if (w == 1 && x == 0 && y == 0 && z == 0) return Vector(0, 0, 0);
		float angle;
		Vector axis;
		toAxisAngle(axis, angle);
		return angle * axis;
	}

	// 转成欧拉角 Vector(roll, pitch, yaw)，单位 rad，与 fromEuler 互逆。
	// 俯仰接近 ±90° 时进入奇异分支（万向锁），此时 roll 置 0，仅 yaw 有意义。
	Vector toEuler() const {
		Vector euler;
		float sqx = x * x;
		float sqy = y * y;
		float sqz = z * z;
		float sqw = w * w;
		float sarg = -2 * (x * z - w * y) / (sqx + sqy + sqz + sqw);
		if (sarg <= -0.99999) { // 俯仰 ≈ -90°，奇异
			euler.x = 0;
			euler.y = -0.5f * (float)M_PI;
			euler.z = -2 * atan2(y, x);
		} else if (sarg >= 0.99999) { // 俯仰 ≈ +90°，奇异
			euler.x = 0;
			euler.y = 0.5f * (float)M_PI;
			euler.z = 2 * atan2(y, x);
		} else {
			euler.x = atan2(2 * (y * z + w * x), sqw - sqx - sqy + sqz); // roll
			euler.y = asin(sarg);                                       // pitch
			euler.z = atan2(2 * (x * y + w * z), sqw + sqx - sqy - sqz); // yaw
		}
		return euler;
	}

	// ---- 便捷取角（内部走 toEuler，注意其奇异点特性）----
	float getRoll() const {
		return toEuler().x;
	}

	float getPitch() const {
		return toEuler().y;
	}

	// 仅取偏航角。control.cpp 用它构造"保持当前机头朝向"的定高/失控目标姿态
	float getYaw() const {
		return toEuler().z;
	}

	// 单独改写某一欧拉角分量（其余分量保持不变），内部会重建四元数
	void setRoll(float roll) {
		Vector euler = toEuler();
		*this = Quaternion::fromEuler(Vector(roll, euler.y, euler.z));
	}

	void setPitch(float pitch) {
		Vector euler = toEuler();
		*this = Quaternion::fromEuler(Vector(euler.x, pitch, euler.z));
	}

	void setYaw(float yaw) {
		Vector euler = toEuler();
		*this = Quaternion::fromEuler(Vector(euler.x, euler.y, yaw));
	}

	// 四元数乘法（复合旋转）：a * b 表示先施加 b 再施加 a
	Quaternion operator * (const Quaternion& q) const {
		return Quaternion(
			w * q.w - x * q.x - y * q.y - z * q.z,
			w * q.x + x * q.w + y * q.z - z * q.y,
			w * q.y + y * q.w + z * q.x - x * q.z,
			w * q.z + z * q.w + x * q.y - y * q.x);
	}

	bool operator == (const Quaternion& q) const {
		return w == q.w && x == q.x && y == q.y && z == q.z;
	}

	bool operator != (const Quaternion& q) const {
		return !(*this == q);
	}

	// 逆（单位四元数时等价于共轭）。注意：零四元数会得到 NAN
	Quaternion inversed() const {
		float normSqInv = 1 / (w * w + x * x + y * y + z * z);
		return Quaternion(
			w * normSqInv,
			-x * normSqInv,
			-y * normSqInv,
			-z * normSqInv);
	}

	// 把向量 v 由"本四元数描述的坐标系"变换到世界系：res = q * v * q⁻¹
	Vector conjugate(const Vector& v) const {
		Quaternion qv(0, v.x, v.y, v.z);
		Quaternion res = (*this) * qv * inversed();
		return Vector(res.x, res.y, res.z);
	}

	// 反向变换：把世界系向量变到机体系：res = q⁻¹ * v * q
	Vector conjugateInversed(const Vector& v) const {
		Quaternion qv(0, v.x, v.y, v.z);
		Quaternion res = inversed() * qv * (*this);
		return Vector(res.x, res.y, res.z);
	}

	// 复合旋转 a*b 并（默认）归一化，用于把增量旋转叠加到当前姿态
	static Quaternion rotate(const Quaternion& a, const Quaternion& b, const bool normalize = true) {
		Quaternion rotated = a * b;
		if (normalize) {
			rotated.normalize();
		}
		return rotated;
	}

	// 用四元数 q 旋转向量 v，等价于 q.conjugateInversed(v)，即 **世界系 -> 机体系**。
	// ⚠️ 本行注释原先写成"机体系 -> 世界系"，与实现（q⁻¹·v·q）相反，已改正。
	//    工程里的实际用法也都是按"世界系→机体系"来的：
	//      controlAttitude()/applyAcc()/applyLevel()/invertedFailsafe() 都是
	//      rotateVector(Vector(0,0,1), attitude)，得到"世界 up 在机体系下的表示"，
	//      其 z 分量正好等于 cos(倾角)——这是判倒置、算 tilt 的依据。
	//    要做机体系→世界系请用 conjugate()，例如 estimateVelZ() 里的
	//      accWorld = attitude.conjugate(acc)。
	//    调用点里那些写着"机体 Z 轴在世界系方向"的行内注释同样是反的，看实现别看注释。
	static Vector rotateVector(const Vector& v, const Quaternion& q) {
		return q.conjugateInversed(v);
	}

	// 求 a 到 b 的差值四元数（b 相对 a 的旋转）
	static Quaternion between(const Quaternion& a, const Quaternion& b, const bool normalize = true) {
		Quaternion q = a * b.inversed();
		if (normalize) {
			q.normalize();
		}
		return q;
	}
};
