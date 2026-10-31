// ============================================================================
// control.cpp —— 飞行控制主链（模式解释 → 姿态 → 角速率 → 力矩混合）
//
// 职责:
//   把"遥控/网页/MAVLink 的归一化输入"翻译成"4 路电机的推力指令"。控制链自上而下：
//     interpretControls()  解析模式与杆量，生成 attitudeTarget / ratesTarget / thrustTarget
//     failsafe()           各类保护（遥控丢失/倒置/低电量），可强制切 AUTO 并下降
//     controlAttitude()    姿态环：姿态误差 -> 期望角速率（+ 前馈 ratesExtra）
//     controlRates()       角速率环：角速率误差 -> 期望力矩
//     controlTorque()      控制分配：力矩+推力 -> 四电机输出，并做饱和缩放
//
// 关键逻辑:
//   - 飞行模式由 modeChannel 分三档映射到 flightModes[3]，取值 RAW/ACRO/STAB/ALTHOLD/AUTO；
//   - 解锁/上锁手势：油门最低 + 偏航打满（右满解锁 / 左满上锁）；
//   - STAB/ALTHOLD：姿态目标 = 杆量 * tiltMax（叠加 trim），偏航保持/按杆转；
//   - ACRO：直接给角速率目标，姿态目标失效；
//   - RAW：直接给力矩目标，绕过姿态/速率环（调试用，危险）；
//   - ALTHOLD：油门杆变为"升降速率指令"，见 controlAltitude()。
//
// 输入/输出:
//   输入：全局 controlRoll/Pitch/Yaw/Throttle/Mode（[-1,1] / [0,1] / [0,1]）、
//         attitude / rates（来自估计）、armed / mode。
//   输出：motors[4]（0~1 归一化推力）、thrustTarget、ratesTarget、torqueTarget 等。
//
// 重要参数（均可由参数系统在线改写，见 parameters.cpp）:
//   *_RATE_P/I/D  角速率环增益；*_P/I/D 姿态环增益；*_RATE_MAX 各轴最大角速率；
//   TILT_MAX      自稳模式最大倾角；MOT_THR_MIN/MAX 油门映射范围；ALT_* 定高参数。
//
// 边界情况与潜在风险:
//   - 控制链各环节都用 invalid() 做"未激活"判断；若上游忘了 invalidate，会带着旧目标飞行。
//   - mode == AUTO 时 interpretControls() 直接返回，模式通道**不会**覆盖 AUTO；
//     这是失控保护能正常下降的前提（见 interpretControls 注释）。退出 AUTO 只能靠
//     autoFailsafe()（杆量变化）或网页遥控按钮。
//   - controlTorque 的怠速闩锁 idleLatch：解锁但未推油时电机保持 0，推过油后才允许
//     收油到怠速；上锁即复位。测试电机(mfr/mfl/...)依赖 controlPaused 独占 PWM，
//     与本函数不冲突，但已解锁时不要测试（姿态会失控）。
//   - desaturate 只做等比缩放，可能把总推力一并压低（大机动时会掉高度），属取舍。
//   - 电机下标顺序（FL/FR/RL/RR）与混控符号必须与机架转向一致，接错会导致翻转。
// ============================================================================

#include "globals.h"
#include "cf_math.h"
#include <cstdio>
#include <cstring>

// 前向声明：电机输出饱和缩放（定义在文件末尾）
static void desaturate(float& a, float& b, float& c, float& d);

// ============== 角速率环 PID（内环，误差单位 rad/s）==============
// 针对 118mm 轴距微型四轴的整定值
#define PITCHRATE_P 0.06
#define PITCHRATE_I 0.1
#define PITCHRATE_D 0.001
#define PITCHRATE_I_LIM 0.3
#define ROLLRATE_P PITCHRATE_P
#define ROLLRATE_I PITCHRATE_I
#define ROLLRATE_D PITCHRATE_D
#define ROLLRATE_I_LIM PITCHRATE_I_LIM
#define YAWRATE_P 0.3
#define YAWRATE_I 0.01
#define YAWRATE_D 0.01
#define YAWRATE_I_LIM 0.3

// 角速率环积分限幅随推力缩放（2026-10-29）：
// 原来 roll/pitch 的积分输出限幅固定 0.3，而 20% 油门对应的总推力只有 0.226 ——
// 积分单独一项就能把差动顶到满量程。起飞瞬间机体还在地面/低速跟不动，误差迟迟
// 不收敛 → 积分涨满 → 带着满差动离地 → 单侧翘起翻机（RAW 模式无环路所以不翻）。
// 改成"积分输出 ≤ 当前推力的 25%"，低推力下积分最多只能产生 1/4 推力的差动，
// 抬不起任何一侧；推力升高后限幅自动放大，不影响飞行中的配平能力。
#define RATE_I_LIM_RATIO 0.25f
#define RATE_I_LIM_MAX   0.15f

// ============== 姿态环 PID（外环，误差单位 rad）==============
#define ROLL_P 6
#define ROLL_I 0
#define ROLL_D 0
#define ROLL_I_LIM radians(5.0f)
#define PITCH_P ROLL_P
#define PITCH_I ROLL_I
#define PITCH_D ROLL_D
#define PITCH_I_LIM ROLL_I_LIM
#define YAW_P 3

// ============== 限幅与映射常量 ==============
#define PITCHRATE_MAX radians(360)  // 俯仰最大角速率
#define ROLLRATE_MAX radians(360)   // 横滚最大角速率
#define YAWRATE_MAX radians(300)    // 偏航最大角速率
#define TILT_MAX radians(30)        // 自稳模式最大倾角（安全上限）
#define ARM_THROTTLE_LIMIT   0.05f  // 网页解锁时允许的最大油门（防止带油门解锁）
#define RATES_D_LPF_ALPHA 0.2       // 角速率环 D 项低通系数

float motThrMin = 0.10f; // 怠速推力下限（电机不停转，保证可控）
float motThrMax = 0.9f;  // 推力上限（留 10% 余量给姿态控制）

// 三轴力矩相对平均推力的最大比例（见 controlTorque 的"低推力差动限幅"）。
// 0.8 → 最低一路仍保有 20% 推力，绝不出现"一路被钳到 0、整机只剩单侧推力"。
// 调小 = 更平顺但低油门响应更软；调大 = 响应快但地面容易单侧翘起。
#define DIFF_LIMIT_RATIO 0.8f

// 飞行模式常量：RAW 直通力矩，ACRO 角速率，STAB 自稳，ALTHOLD 近似定高，AUTO 自动/失控下降
const int RAW = 0, ACRO = 1, STAB = 2, ALTHOLD = 3, AUTO = 4;
int mode = STAB;                        // 当前飞行模式
bool armed = false;                     // 是否已解锁
int flightModes[] = {STAB, STAB, STAB}; // 模式通道三档对应的模式（可由 CTL_FLT_MODE_* 配置）

// PID 实例：角速率环（带 D 项低通）+ 姿态环
PID rollRatePID(ROLLRATE_P, ROLLRATE_I, ROLLRATE_D, ROLLRATE_I_LIM, RATES_D_LPF_ALPHA);
PID pitchRatePID(PITCHRATE_P, PITCHRATE_I, PITCHRATE_D, PITCHRATE_I_LIM, RATES_D_LPF_ALPHA);
// 【2026-10-22 修正】原先漏传 windup（默认 0），integral 输出被 clamp 成 ±0，
//   导致 YAWRATE_I 完全失效、YAWRATE_I_LIM 定义后从未被引用；同步补上 D 项低通。
PID yawRatePID(YAWRATE_P, YAWRATE_I, YAWRATE_D, YAWRATE_I_LIM, RATES_D_LPF_ALPHA);
PID rollPID(ROLL_P, ROLL_I, ROLL_D, ROLL_I_LIM);
PID pitchPID(PITCH_P, PITCH_I, PITCH_D, PITCH_I_LIM);
PID yawPID(YAW_P, 0, 0);
Vector maxRate(ROLLRATE_MAX, PITCHRATE_MAX, YAWRATE_MAX); // 各轴最大角速率
float tiltMax = TILT_MAX;                                  // 自稳最大倾角

Quaternion attitudeTarget; // 目标姿态
Vector ratesTarget;        // 目标角速率
Vector ratesExtra;         // 角速率前馈（偏航杆量，直接叠加到速率环输出）
Vector torqueTarget;       // 目标力矩
float thrustTarget;        // 目标推力（0~1）

float trimRoll  = 0.0f; // 横滚配平（消除重心偏移导致的悬停倾斜）
float trimPitch = 0.0f; // 俯仰配平

// ============== ALTHOLD 近似定高参数（无气压计，基于加速度计垂直速度）==============
// 油门中位死区内 = 保持当前状态（松杆悬停）；死区外 = 升降速率指令（油门=垂直速度）。
// 悬停推力由 I 项在线学习：切入定高时以当前杆位油门为初值，松杆后向真实悬停油门收敛。
// 注意：速度估计带泄漏，长时间（>30s）会缓慢漂移，属预期行为。
float altHoldP = 0.08f;        // ALT_P：速度误差→推力增益（推力/每(m/s)误差）；过大垂直振荡，过小响应迟钝
float altHoldI = 0.3f;         // ALT_I：悬停推力学习速率（推力/每(m/s·s)误差）；过大振荡，过小松杆后缓慢沉浮
float altHoldRateMax = 1.5f;   // ALT_RATE_MAX：满杆升降速率（m/s）
float altHoldDeadband = 0.08f; // ALT_DEADBAND：油门中位保持死区（0~0.5），越大越难误触发升降
float hoverThrustEst = NAN;    // 学习到的悬停推力（NAN=未激活）；离开 ALTHOLD 即作废，下次切入重学

void control() {
	interpretControls();
#if WEB_RC_ENABLED
	interpretWebRC();
#endif
	failsafe();
	controlAttitude();
	controlRates();
	controlTorque();
}

void interpretControls() {
	// 模式通道 → 飞行模式。
	// 【关键】AUTO 由失控保护（descend）接管，此处**不得覆盖**。
	// 旧版无条件执行三档映射，导致下一周期 mode 立刻被模式通道改回 STAB，
	// descend() 的 `if (mode != AUTO)` 于是每帧都成立：PID 被反复 reset、
	// thrustTarget 被反复重新钳到 ALTHOLD_HOVER_THRUST(0.7)，再减去 dt/descendTime
	// 又被钳回去 —— 推力永远停在 0.7，失控保护变成了"永久悬停"，飞机根本降不下来。
	// 退出 AUTO 的唯一途径是 autoFailsafe()（杆量变化 → 人工接管）或网页遥控按钮。
	if (mode != AUTO) {
		if (controlMode < 0.25) mode = flightModes[0];
		else if (controlMode <= 0.75) mode = flightModes[1];
		else if (controlMode > 0.75) mode = flightModes[2];
	}

	if (mode == AUTO) return;

	// 【2026-10-29 新增 · 解锁边沿复位全部 PID】
	// 姿态环/角速率环在"未解锁"和"推力 < motThrMin"时都是直接 return（见
	// controlAttitude/controlRates），积分项**不会**被清零。于是上一次解锁周期里
	// 在地面被钳住、长时间大误差累积出来的积分会原样留到下一次起飞：
	// 重新推油门的第一帧就带着满量程差动输出，表现就是"一推油单侧翘起 → 翻"。
	// 这里在上锁/解锁发生切换的那一帧统一复位，保证每次起飞都从干净状态开始。
	// （armed 也可能被网页遥控 / failsafe / CLI 改写，所以用静态比较而非只看手势。）
	static bool lastArmedState = false;
	if (armed != lastArmedState) {
		rollPID.reset();
		pitchPID.reset();
		yawPID.reset();
		rollRatePID.reset();
		pitchRatePID.reset();
		yawRatePID.reset();
		ratesTarget = Vector(0, 0, 0);
		lastArmedState = armed;
	}

#if WEB_RC_ENABLED
	if (!isUsingWebRC()) {
#endif
		static bool armWarnNotified = false;
		if (controlThrottle < 0.05 && controlYaw > 0.95) {
			if (!imuOK) {
				if (!armWarnNotified) {
					printNonBlocking("IMU故障，禁止解锁！\n");
#if WEB_RC_ENABLED
					setWebRCWarn("IMU故障 禁止解锁");
#endif
					armWarnNotified = true;
				}
			} else if (batteryVoltage > VBAT_ABSENT_THRESHOLD && batteryVoltage < VBAT_WARN_THRESHOLD) {
				if (!armWarnNotified) {
					printNonBlocking("电量低(%.2fV)，禁止解锁\n", batteryVoltage);
#if WEB_RC_ENABLED
					char warnBuf[64];
					snprintf(warnBuf, sizeof(warnBuf), "电量低(%.2fV) 禁止解锁", batteryVoltage);
					setWebRCWarn(warnBuf);
#endif
					armWarnNotified = true;
				}
			} else {
				armed = true;
				armWarnNotified = false;
			}
		}
		if (controlThrottle < 0.05 && controlYaw < -0.95) armed = false;
#if WEB_RC_ENABLED
	}
#endif

	// 偏航小杆量死区：小于 10% 视为不转，避免手抖引起缓慢偏航
	if (std::fabs(controlYaw) < 0.1) controlYaw = 0;

	// 油门死区映射：<0.05 视为收油（推力 0）；否则线性映射到 [motThrMin, motThrMax]
	if (controlThrottle < 0.05f) {
		thrustTarget = 0.0f;
	} else {
		thrustTarget = mapf(controlThrottle, 0.05f, 1.0f, motThrMin, motThrMax);
	}

	// STAB / ALTHOLD：生成姿态目标（杆量 → 倾角，叠加 trim；偏航保持或按杆转）
	if (mode == STAB || mode == ALTHOLD) {
		float yawTarget = attitudeTarget.getYaw();
		// 未解锁 / 目标偏航无效 / 有偏航杆量时，把目标偏航重置为当前偏航（避免突变）
		if (!armed || invalid(yawTarget) || controlYaw != 0) yawTarget = attitude.getYaw();
		attitudeTarget = Quaternion::fromEuler(Vector(controlRoll * tiltMax + trimRoll, controlPitch * tiltMax + trimPitch, yawTarget));
		ratesExtra = Vector(0, 0, -controlYaw * maxRate.z); // 偏航杆量作为角速率前馈
	}

	if (mode == ALTHOLD) {
		controlAltitude(); // 油门=升降速率，覆盖上面的直通油门
	} else {
		hoverThrustEst = NAN; // 离开定高（含失控降落的 AUTO）即作废估计，下次切入重学
	}

	// ACRO：直接给角速率目标，姿态目标置为无效（跳过姿态环）
	if (mode == ACRO) {
		attitudeTarget.invalidate();
		ratesTarget.x = controlRoll * maxRate.x;
		ratesTarget.y = controlPitch * maxRate.y;
		ratesTarget.z = -controlYaw * maxRate.z;
	}

	// RAW：直接给力矩目标，速率目标置无效（跳过姿态环与速率环）。仅用于调试，飞行中极危险
	if (mode == RAW) {
		attitudeTarget.invalidate();
		ratesTarget.invalidate();
		torqueTarget = Vector(controlRoll, controlPitch, -controlYaw) * 0.1f;
	}
}

// ALTHOLD 近似定高：油门杆 → 期望垂直速度，P 阻尼 + I 学习悬停推力
// 悬停推力估计由 interpretControls 在离开 ALTHOLD/上锁/油门到底时置 NAN，切入即重学
void controlAltitude() {
	if (!armed || controlThrottle < 0.05f) {
		hoverThrustEst = NAN;
		return; // thrustTarget 已由底部死区置 0
	}

	// 首次进入：以当前杆位对应的直通油门为悬停推力初值，切换瞬间推力无跳变；
	// 之后由 I 项继续在线学习
	if (invalid(hoverThrustEst)) {
		hoverThrustEst = mapf(controlThrottle, 0.05f, 1.0f, motThrMin, motThrMax);
		printNonBlocking("ALTHOLD: 进入定高，初始悬停推力 %.2f\n", hoverThrustEst);
	}

	// 油门中位死区内 = 保持（目标速度 0）；死区外线性映射为升降速率指令
	float velTarget = 0;
	if (controlThrottle > 0.5f + altHoldDeadband) {
		velTarget = mapf(controlThrottle, 0.5f + altHoldDeadband, 1.0f, 0, altHoldRateMax);
	} else if (controlThrottle < 0.5f - altHoldDeadband) {
		velTarget = mapf(controlThrottle, 0.05f, 0.5f - altHoldDeadband, -altHoldRateMax, 0);
	}

	float velError = velTarget - velZ;

	// I：悬停推力在线学习（松杆 velTarget=0 时持续收敛到真实悬停油门）
	hoverThrustEst += altHoldI * velError * dt;
	hoverThrustEst = std::clamp(hoverThrustEst, motThrMin, motThrMax);

	thrustTarget = std::clamp(hoverThrustEst + altHoldP * velError, 0.0f, motThrMax);
}

// 姿态环：把姿态误差（机体系下的旋转矢量）转成期望角速率。
// 输入：attitude（实测）、attitudeTarget（目标）、ratesExtra（偏航前馈）
// 输出：ratesTarget.x/y/z
// 跳过条件：未解锁 / 目标姿态无效（RAW/ACRO）/ 推力低于怠速（视为未起飞）
void controlAttitude() {
	if (!armed || attitudeTarget.invalid() || thrustTarget < motThrMin) return;
	const Vector up(0, 0, 1);
	// rotateVector(v,q) = q⁻¹·v·q = 世界系→机体系（见 quaternion.h 注释）。
	// 所以 upActual/upTarget 都是"世界 up 轴在（目标）机体系下的表示"，
	// 两者的差值旋转矢量天然落在机体系，正好对应 ratesTarget 的坐标系。
	Vector upActual = Quaternion::rotateVector(up, attitude);        // 世界 up 在实测机体系下
	Vector upTarget = Quaternion::rotateVector(up, attitudeTarget);  // 世界 up 在目标机体系下
	Vector error = Vector::rotationVectorBetween(upTarget, upActual); // 姿态误差旋转矢量（rad，机体系）
	ratesTarget.x = rollPID.update(error.x) + ratesExtra.x;
	ratesTarget.y = pitchPID.update(error.y) + ratesExtra.y;
	// 偏航单独用角度差（wrapAngle 避免 ±π 跳变）
	float yawError = wrapAngle(attitudeTarget.getYaw() - attitude.getYaw());
	ratesTarget.z = yawPID.update(yawError) + ratesExtra.z;
}

// 角速率环：把角速率误差转成期望力矩。
// 输入：rates（实测，已滤波）、ratesTarget（目标）
// 输出：torqueTarget.x/y/z
// 跳过条件：未解锁 / 速率目标无效（RAW 模式）/ 推力低于怠速
void controlRates() {
	if (!armed || ratesTarget.invalid() || thrustTarget < motThrMin) return;
	// 积分限幅随推力缩放（见 RATE_I_LIM_RATIO 处的说明）：低推力时积分顶不出
	// 足以抬起/压低一侧机臂的差动，从根上消除"起飞瞬间被积分掀翻"。
	float iLim = std::clamp(thrustTarget * RATE_I_LIM_RATIO, 0.0f, RATE_I_LIM_MAX);
	rollRatePID.windup = iLim;
	pitchRatePID.windup = iLim;
	Vector error = ratesTarget - rates;
	// 内环用 D-on-measurement：D 项只对实测角速率微分，ratesTarget 跳变（模式切换、
	// 偏航杆量前馈）不再产生微分冲击；I 项仍按误差累积。
	torqueTarget.x = rollRatePID.update(error.x, rates.x);
	torqueTarget.y = pitchRatePID.update(error.y, rates.y);
	torqueTarget.z = yawRatePID.update(error.z, rates.z);
}

// 控制分配：把（推力 + 力矩）混合成 4 路电机输出。
// 混控矩阵基于 X 型四轴：对角两桨同向，相邻两桨反向；
// 力矩正负号决定"哪个电机加速/减速"从而产生对应方向的力矩。
//
// 怠速逻辑（idleLatch）：
//   旧版只要 armed 且 thrustTarget < motThrMin，四路就统一钉在 motThrMin(10%)。
//   解锁手势本身要求油门最低，于是"一解锁桨就转"——在地面上相当危险；
//   而且 descend() 把推力递减到 0 的最后一秒也被这个分支顶住，桨停不下来。
//   现在加一个"本次解锁周期内是否真的推过油"的闩锁：
//     - 上锁 → 闩锁复位，电机强制为 0；
//     - 解锁但还没推油 → 电机保持 0（桨不转，可以安全地拿着飞机）；
//     - 推油越过 motThrMin → 闩锁置位，之后收油到底时保留怠速（保证空中可控）。
//   注意：这同时让 motorsActive() 在"解锁未推油"时为 false，estimate.cpp 的
//   landed 判据因此仍然成立（电机不转 + 加速度≈1g → landed）。
void controlTorque() {
	if (!torqueTarget.valid()) {
		memset(motors, 0, sizeof(motors)); // 防止无效控制量时沿用上一周期的不对称输出
		return;
	}

	static bool idleLatch = false; // 本次解锁周期内是否曾经给过油（跨帧保持）

	if (!armed) {
		idleLatch = false;
		memset(motors, 0, sizeof(motors));
		return;
	}
	if (thrustTarget >= motThrMin) idleLatch = true;

	if (!idleLatch) { // 解锁但油门在最低位：保持停转，不做怠速
		memset(motors, 0, sizeof(motors));
		return;
	}
	if (thrustTarget < motThrMin) { // 已起飞后收油到底：保持最小转速（空中不能停桨）
		for (int i = 0; i < 4; i++) motors[i] = motThrMin;
		return;
	}
	// 【2026-10-29 新增 · 低推力差动限幅】
	// 地面低油门（远低于悬停推力）时，机体被地面"粘住"不跟随指令，姿态环/角速率环
	// 的误差迟迟不收敛 → 积分把差动推到满量程 → desaturate + clamp 把某一路钉在 0。
	// 一旦有一路完全停转，整机就只剩单侧推力：只要出现一点升力趋势，机体立刻单侧
	// 翘起→翻，而且积分还一直顶在那儿，表现为"推到 20% 就翘翻"。
	// 把三轴力矩各自限制在平均推力的 80% 以内，四路里最低那一路也保有 20% 推力，
	// 绝不出现"一路 0"的极端分配。
	// 影响面：悬停推力≈0.5 时上限 0.4，远大于正常机动的需求（内环 P=0.06，
	// 5°/s 误差才 0.0003 量级），只有低油门大误差时才会咬到，不改变飞行手感。
	float diffLimit = thrustTarget * DIFF_LIMIT_RATIO;
	torqueTarget.x = std::clamp(torqueTarget.x, -diffLimit, diffLimit);
	torqueTarget.y = std::clamp(torqueTarget.y, -diffLimit, diffLimit);
	torqueTarget.z = std::clamp(torqueTarget.z, -diffLimit, diffLimit);

	// 条件积分（抗饱和）：本帧某个轴已经被限幅压住 → 下一帧冻结该轴的积分累加。
	// 配合 controlRates() 里随推力缩放的积分限幅，彻底断掉"机体跟不动 → 积分涨满 →
	// 带着满差动离地 → 单侧翘起翻机"这条正反馈链。
	rollRatePID.freezeIntegral  = std::fabs(torqueTarget.x) >= diffLimit - 1e-6f;
	pitchRatePID.freezeIntegral = std::fabs(torqueTarget.y) >= diffLimit - 1e-6f;
	yawRatePID.freezeIntegral   = std::fabs(torqueTarget.z) >= diffLimit - 1e-6f;

	// 混控矩阵（X 型，对角两桨同向，相邻反向）
	// 【2026-10-29 台架实测修正 · 横滚符号（第二次，以此为准）】
	//   本机实测机体系为右手系 x 前 / y 左 / z 上，证据（拆桨手摆 + trace）：
	//     静止   acc(0, 0, ~9.8)  rpy(0,0,0)
	//     右倾   acc.y = +3.4    roll = +20°     → +Y 朝左、+Z 朝上
	//     机头下压 acc.x = -4.5   pitch = +27°    → +X 朝机头（IMU 安装正确）
	//   在该坐标系下"绕 +x 正向旋转"= 右倾（右侧下沉），所以机体右倾时
	//   controlAttitude() 算出的 error.x < 0。2026-10-18 的写法
	//   （前右 = T + tx）会把右侧推力**减小**，等于越纠越右倾：
	//   台架 20% 油门右倾 20° 实测 left=0.475/0.246、right=0.000/0.167，
	//   差动被拉到一路钳 0 —— 这正是"推到 20% 就翘起来翻掉 + 一直右偏"的成因。
	//   现在把 tx 整体取反：tx > 0 → 右侧两路推力增加 → 抬右，与"绕 +x 负向"对齐。
	//   俯仰（ty）/ 偏航（tz）经同一组台架数据验证方向正确，未改动：
	//     机头下压时 front=0.305+0.399 > rear=0.184+0.000（前桨抬机头）✓
	motors[MOTOR_FRONT_LEFT]  = thrustTarget + torqueTarget.x - torqueTarget.y + torqueTarget.z;
	motors[MOTOR_FRONT_RIGHT] = thrustTarget - torqueTarget.x - torqueTarget.y - torqueTarget.z;
	motors[MOTOR_REAR_LEFT]   = thrustTarget + torqueTarget.x + torqueTarget.y - torqueTarget.z;
	motors[MOTOR_REAR_RIGHT]  = thrustTarget - torqueTarget.x + torqueTarget.y + torqueTarget.z;
	desaturate(motors[MOTOR_FRONT_LEFT], motors[MOTOR_FRONT_RIGHT], motors[MOTOR_REAR_LEFT], motors[MOTOR_REAR_RIGHT]);
	// 最终钳位到 [0,1]，防止数值越界
	motors[0] = std::clamp(motors[0], 0.0f, 1.0f);
	motors[1] = std::clamp(motors[1], 0.0f, 1.0f);
	motors[2] = std::clamp(motors[2], 0.0f, 1.0f);
	motors[3] = std::clamp(motors[3], 0.0f, 1.0f);
}

// 饱和缩放：当某路电机超出 [0,1] 时，围绕四路均值等比压缩"差分量"，
// 尽量保留总推力（均值）同时把最大/最小路拉回合法区间。
// 算法：分别按"超出上限"和"低于下限"计算允许的缩放系数，取更严格者。
static void desaturate(float& a, float& b, float& c, float& d) {
	float avg = (a + b + c + d) * 0.25f; // 总推力对应的均值
	float maxVal = std::max(std::max(a, b), std::max(c, d));
	float minVal = std::min(std::min(a, b), std::min(c, d));
	float scale = 1.0f;
	if (maxVal > 1.0f && maxVal > avg) scale = std::min(scale, (1.0f - avg) / (maxVal - avg));
	if (minVal < 0.0f && avg > minVal) scale = std::min(scale, avg / (avg - minVal));
	if (scale < 1.0f) { // 只在需要时缩放，避免无谓的浮点误差
		a = avg + (a - avg) * scale;
		b = avg + (b - avg) * scale;
		c = avg + (c - avg) * scale;
		d = avg + (d - avg) * scale;
	}
}

// 返回当前飞行模式的名称字符串（用于日志/网页显示）
const char* getModeName() {
	switch (mode) {
		case 0: return "RAW";
		case 1: return "ACRO";
		case 2: return "STAB";
		case 3: return "ALTHOLD";
		case 4: return "AUTO";
		default: return "UNKNOWN";
	}
}

#if WEB_RC_ENABLED
// 处理网页遥控的"按钮上升沿"事件（解锁/上锁/急停/切模式）。
// 输入：webRCButtons 位掩码（由 web_rc.cpp 的 HTTP 接口更新）
// 仅处理上升沿（本次按下且上次未按），避免长按重复触发。
// 与遥控杆量无关：杆量已在 web_rc.cpp 里直接写入 controlRoll/Pitch/Yaw/Throttle。
void interpretWebRC() {
	if (!isUsingWebRC()) return;
	static uint16_t lastWebRCButtons = 0;
	uint16_t risingEdge = webRCButtons & ~lastWebRCButtons; // 本次新按下的位
	lastWebRCButtons = webRCButtons;

	static bool lastArmedState = false;
	if (armed != lastArmedState) {
		printNonBlocking(armed ? "Web RC: 已解锁\n" : "Web RC: 已上锁\n");
		lastArmedState = armed;
	}

	if (risingEdge & 0x0001) {
		if (!imuOK) {
			setWebRCWarn("IMU故障 禁止解锁");
		} else if (batteryVoltage > VBAT_ABSENT_THRESHOLD && batteryVoltage < VBAT_WARN_THRESHOLD) {
			char warnBuf[64];
			snprintf(warnBuf, sizeof(warnBuf), "电量低(%.2fV) 禁止解锁", batteryVoltage);
			setWebRCWarn(warnBuf);
		} else if (controlThrottle > ARM_THROTTLE_LIMIT) {
			setWebRCWarn("油门过高，无法解锁");
		} else {
			armed = true;
			webRCWarnMsg[0] = '\0';
		}
	}
	if (risingEdge & 0x0002) { armed = false; mode = STAB; }
	if (risingEdge & 0x0004) { armed = false; thrustTarget = 0.0f; mode = STAB; }
	if (risingEdge & 0x0040) mode = STAB;
	if (risingEdge & 0x0080) mode = ACRO;
	// 按钮8：ALTHOLD定高模式（上升沿）。近似定高：油门中位=松杆悬停，推/拉=升降速率
	if (risingEdge & 0x0100) mode = ALTHOLD;

	static int lastMode = STAB;
	if (mode != lastMode) {
		printNonBlocking("Web RC: 模式切换到 %s\n", getModeName());
		lastMode = mode;
	}
}
#endif
