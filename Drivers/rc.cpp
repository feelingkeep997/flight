// ============================================================================
// rc.cpp —— 遥控接收机（SBUS，串口）
//
// 职责:
//   从 UART 读取遥控接收机的串口数据，解析出各通道原始值，并归一化成飞控使用的
//   controlRoll/Pitch/Yaw/Throttle/Mode（[-1,1] / [0,1]），同时刷新 controlTime
//   供"遥控丢失保护"判断。
//
// 关键逻辑:
//   - setupRC() 安装 UART 并按 SBUS 物理层配置：100000 baud / 8 数据位 / 偶校验 /
//     2 停止位 / RX 电平反相（SBUS 是反相串口，ESP32 可用 uart_set_line_inverse 内部取反）；
//   - readRC() 由通信任务以 1kHz 调用，每次把 UART 里积压的字节全部喂进状态机
//     （最多 64 字节，避免饿死同任务里的其它轮询）；
//   - 状态机按 25 字节定长收帧：0x0F 起始 → 22 字节通道数据 → 1 字节标志 → 0x00 结束；
//     结束字节或标志位不对就整帧丢弃并重新同步；
//   - 一帧合法后 sbusUnpack() 按 11bit 小端位序解出 16 个通道，再 normalizeRC()
//     做"两段式"归一化（以中位为分界，左右行程分别映射），最后写入控制量。
//
// 输入/输出:
//   输入：UART（引脚 BOARD_RC_RX_PIN / rcRxPin）。
//   输出：channels[16]、controlRoll/Pitch/Yaw/Throttle/Mode、controlTime。
//
// 重要参数:
//   rcProtocol(0=SBUS；1=CRSF 未实现，setupRC 会强制回退到 0 并告警)
//   rcBaud(仅 CRSF 用；SBUS 波特率由协议固定为 100000，不受此参数影响)
//   rcInverted(1=RX 内部反相，接收机未自带反相器时用；0=信号已是正相)
//   channelZero[16] = 通道中位（参数 RC_MID_*），channelMax[16] = 通道最大（RC_MAX_*）
//     最小值按对称行程推导为 2*mid - max，由 calibrateRC() 实测写入
//
// 控制权归属（重要）:
//   物理遥控与网页遥控都会写 controlRoll/Pitch/Yaw/Throttle。规则是
//   **"只要有合法的物理遥控帧，物理遥控就是主控"**——遥控器在手边、延迟最低、
//   失联可被 rcLossFailsafe 捕获，比浏览器可靠得多。想用网页遥控飞行，
//   请关闭/不绑定发射机。两路同时在线时会打印一次提示。
//
// 边界情况与潜在风险:
//   - 收到的帧若标志位带 failsafe/frame_lost，或前 4 个通道超出 11bit 合理量程，
//     整帧丢弃且**不更新 controlTime**，从而让 rcLossFailsafe 能按超时接管。
//   - channelZero/channelMax 会被合法性校验（中位 100~2000、最大 > 中位且 <= 2100），
//     不合法就回退到 SBUS 默认 992/1811，避免 mapf 分母为 0 或负数把输出打飞。
//   - 第 5 通道（模式）超出量程时 controlMode 置 NAN；interpretControls 对 NAN 的
//     三档比较全为假，mode 保持不变（接收机没接模式开关时不会乱切模式）。
//   - calibrateRC() 期间 rcCalibrating=true，normalizeRC() 只更新 channels[] 不写控制量，
//     避免标定时乱打杆把控制量灌进飞控。标定前必须解锁状态为 false（内部会拒绝）。
//   - controlTime 只在成功解析一帧后更新；若接收机无信号，它保持不变，
//     从而使 rcLossFailsafe 能按 t - controlTime 判断超时。
// ============================================================================

#include "globals.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>
#include <cmath>

// ---- SBUS 物理层与帧结构 ----
#define SBUS_BAUD          100000  // 协议固定，不可调
#define SBUS_FRAME_LEN     25      // 1 起始 + 22 通道 + 1 标志 + 1 结束
#define SBUS_START_BYTE    0x0F
#define SBUS_END_BYTE      0x00
#define SBUS_FLAG_DIGITAL_CH17 0x01
#define SBUS_FLAG_DIGITAL_CH18 0x02
#define SBUS_FLAG_FRAME_LOST   0x04  // 接收机与发射机之间丢帧（本帧数据仍可用，但连续丢要警惕）
#define SBUS_FLAG_FAILSAFE     0x08  // 接收机进入保护值输出（数据不是飞手的，必须丢弃）
#define SBUS_CHANNEL_BITS  11
#define SBUS_CHANNEL_MASK  0x7FF
#define SBUS_CHANNELS      16

// SBUS 通道量程：中位 992，行程 ±690 -> 172 ~ 1811
#define SBUS_CH_MID        992.0f
#define SBUS_CH_MAX        1811.0f
// 合法性窗口：超出即认为该帧是噪声/错位（11bit 理论上限 2047，留一点余量）
#define SBUS_CH_PLAUSIBLE_MIN 100.0f
#define SBUS_CH_PLAUSIBLE_MAX 2047.0f

// readRC() 单次调用最多消费的字节数：足够吃掉一整帧(25)，又不会长时间独占通信任务
#define RC_READ_BUDGET     64

// rcSignalOK() 的判定时长（s），与参数 SF_RC_LOSS_TIME 的默认值保持一致。
// 注意：真正触发失控保护的是 safety.cpp 的 rcLossFailsafe()（用 rcLossTimeout 参数），
//       这里只用于诊断显示，两者不要混用。
#define RC_SIGNAL_TIMEOUT  1.0f

// 默认协议取板级宏 BOARD_RC_PROTOCOL（见 board_config.h），当前三套板卡都是 0=SBUS。
// 运行时也可用参数 RC_PROTOCOL 改写，但 CRSF 未实现，setupRC() 会强制回退到 0。
int rcProtocol = BOARD_RC_PROTOCOL; // 0=SBUS（已实现），1=CRSF（未实现）
int rcBaud = 420000; // 仅 CRSF 使用；SBUS 固定 100000，不受此参数影响
int rcInverted = 1;  // 1 = RX 内部反相（标准 SBUS 接收机需要）
int rcTxPin = -1;
int rcRxPin = BOARD_RC_RX_PIN;
uint16_t channels[16];
float controlTime = 0;
// 通道中位 / 最大值。最小值按对称行程推导为 2*mid - max（见 channelRange）。
// 默认给 SBUS 标准量程；实测标定请用 CLI `cr`。
float channelZero[16] = {
	SBUS_CH_MID, SBUS_CH_MID, SBUS_CH_MID, SBUS_CH_MID, SBUS_CH_MID, SBUS_CH_MID, SBUS_CH_MID, SBUS_CH_MID,
	SBUS_CH_MID, SBUS_CH_MID, SBUS_CH_MID, SBUS_CH_MID, SBUS_CH_MID, SBUS_CH_MID, SBUS_CH_MID, SBUS_CH_MID
};
float channelMax[16] = {
	SBUS_CH_MAX, SBUS_CH_MAX, SBUS_CH_MAX, SBUS_CH_MAX, SBUS_CH_MAX, SBUS_CH_MAX, SBUS_CH_MAX, SBUS_CH_MAX,
	SBUS_CH_MAX, SBUS_CH_MAX, SBUS_CH_MAX, SBUS_CH_MAX, SBUS_CH_MAX, SBUS_CH_MAX, SBUS_CH_MAX, SBUS_CH_MAX
};
float rollChannel, pitchChannel, throttleChannel, yawChannel, modeChannel;

static uart_port_t rcUartPort = (uart_port_t)BOARD_RC_UART_NUM;
static bool rcInitialized = false;
static bool rcCalibrating = false;   // 标定期间不写控制量
static uint32_t rcGoodFrames = 0;    // 累计合法帧数（诊断用）
static uint32_t rcBadFrames = 0;     // 累计丢弃帧数（同步失败/标志位异常/量程异常）
static bool rcDualSourceWarned = false; // "物理遥控与网页遥控同时在线"只提示一次

// SBUS 状态机：定长收帧
static uint8_t sbusFrame[SBUS_FRAME_LEN];
static int sbusIdx = 0;

// 把 22 字节的紧凑位流解成 16 个 11bit 通道（小端位序）。
// 通道 i 的起始比特 = i*11，落在数据区（frame[1..22]）内；
// 取值时最多需要跨 3 个字节（bitOff 最大为 7，7+11=18 > 16），
// i=15 时读到 frame[23]（标志字节），仍在数组内且被 11bit 掩码截断，无越界。
static void sbusUnpack(const uint8_t* frame, uint16_t* out) {
	for (int i = 0; i < SBUS_CHANNELS; i++) {
		int bit = i * SBUS_CHANNEL_BITS;
		int byteIdx = 1 + (bit >> 3);
		int bitOff = bit & 7;
		uint32_t v = (uint32_t)frame[byteIdx] | ((uint32_t)frame[byteIdx + 1] << 8);
		if (bitOff) v |= ((uint32_t)frame[byteIdx + 2] << 16);
		out[i] = (uint16_t)((v >> bitOff) & SBUS_CHANNEL_MASK);
	}
}

// 取第 i 通道的 [lo, mid, hi] 三段标定点。
// mid/hi 来自参数（RC_MID_* / RC_MAX_*），lo 按"对称行程"推导 = 2*mid - hi。
// 参数被写坏时回退到 SBUS 默认值：否则 mapf 的分母可能为 0 或负数，输出直接跑飞。
static void channelRange(int i, float& lo, float& mid, float& hi) {
	mid = channelZero[i];
	hi = channelMax[i];
	if (!std::isfinite(mid) || !std::isfinite(hi) ||
	    mid < SBUS_CH_PLAUSIBLE_MIN || mid > SBUS_CH_MAX ||
	    hi <= mid || hi > SBUS_CH_PLAUSIBLE_MAX) {
		mid = SBUS_CH_MID;
		hi = SBUS_CH_MAX;
	}
	lo = 2.0f * mid - hi;
	if (lo < 0.0f) lo = 0.0f;
}

// 两段式映射：以中位为分界，[lo,mid] -> [outLo,outMid]，[mid,hi] -> [outMid,outHi]。
// 左右行程不等长（配平偏移、摇杆机械误差）时也不会让输出整体偏置。
static float mapCentered(float raw, float lo, float mid, float hi, float outLo, float outHi) {
	float outMid = (outLo + outHi) * 0.5f;
	float v = (raw <= mid) ? mapf(raw, lo, mid, outLo, outMid)
	                       : mapf(raw, mid, hi, outMid, outHi);
	return std::clamp(v, std::min(outLo, outHi), std::max(outLo, outHi));
}

void setupRC() {
	print("Setting up RC receiver...\n");

	// CRSF 分支历史上只是占位实现（收字节、攒够 10 个就丢掉，从不更新任何通道），
	// 而 rcProtocol 的默认值偏偏是 1，于是"遥控接了但完全没反应"成了静默故障。
	// 这里强制回退到已实现的 SBUS 并明确告警。
	if (rcProtocol != 0) {
		print("警告: RC_PROTOCOL=%d (CRSF) 尚未实现，已回退到 SBUS\n", rcProtocol);
		rcProtocol = 0;
	}

	// SBUS 的物理层是协议规定死的：100000 baud、8E2、逻辑反相。
	// 刻意不使用 rcBaud —— 那是 CRSF 的 420000，若沿用 NVS 里的旧值会一个字节都收不到。
	uart_config_t uart_config = {};
	uart_config.baud_rate = SBUS_BAUD;
	uart_config.data_bits = UART_DATA_8_BITS;
	uart_config.parity = UART_PARITY_EVEN;
	uart_config.stop_bits = UART_STOP_BITS_2;
	uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
	uart_config.source_clk = UART_SCLK_APB;

	esp_err_t ret = uart_driver_install(rcUartPort, 512, 0, 0, NULL, 0);
	if (ret != ESP_OK) {
		print("Failed to install RC UART driver: %s\n", esp_err_to_name(ret));
		return;
	}

	ret = uart_param_config(rcUartPort, &uart_config);
	if (ret != ESP_OK) {
		print("Failed to configure RC UART params: %s\n", esp_err_to_name(ret));
		return;
	}

	ret = uart_set_pin(rcUartPort, rcTxPin, rcRxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	if (ret != ESP_OK) {
		print("Failed to set RC UART pins: %s\n", esp_err_to_name(ret));
		return;
	}

	// SBUS 是反相串口。多数接收机输出的是原始反相信号，需要 ESP32 在 GPIO 矩阵里取反；
	// 若板子上已经有反相电路（或接收机输出了正相 TTL），把参数 RC_INVERTED 设为 0。
	if (rcInverted) {
		// ESP-IDF 里的名字是 UART_SIGNAL_RXD_INV（不是 UART_SIGNAL_RX_INV）
		ret = uart_set_line_inverse(rcUartPort, UART_SIGNAL_RXD_INV);
		if (ret != ESP_OK) print("Failed to set RX inverse: %s\n", esp_err_to_name(ret));
	} else {
		uart_set_line_inverse(rcUartPort, UART_SIGNAL_INV_DISABLE);
	}

	uart_flush_input(rcUartPort); // 丢掉上电期间积累的电平噪声/半截帧

	rcInitialized = true;
	print("RC receiver initialized: SBUS %d baud 8E2 on UART%d, RX=GPIO%d, inverted=%d\n",
	      SBUS_BAUD, (int)rcUartPort, rcRxPin, rcInverted);
	print("提示: 首次使用请先执行 `cr` 标定通道行程\n");
}

// 处理一帧收齐的 SBUS 数据：校验结束字节与标志位，解包，量程合理性检查，再归一化。
// 返回 true 表示这是一帧可以采信的飞手数据。
static bool sbusHandleFrame(const uint8_t* frame) {
	// 结束字节不符：说明起始字节 0x0F 是从数据里误认的，丢帧重新同步
	if (frame[24] != SBUS_END_BYTE) return false;

	uint8_t flags = frame[23];
	// failsafe 帧输出的是接收机预设的保护值，不是飞手意图，必须丢弃
	if (flags & SBUS_FLAG_FAILSAFE) return false;
	// 丢帧标志：接收机自己都没把握，同样丢弃（宁可触发 rcLoss 也不要假数据）
	if (flags & SBUS_FLAG_FRAME_LOST) return false;

	uint16_t decoded[SBUS_CHANNELS];
	sbusUnpack(frame, decoded);

	// 量程合理性：前 4 个通道（滚/俯/油/偏）必须落在 11bit 有效区间内，
	// 否则视为位流错位（例如波特率/反相配置不对时会解出各种离谱值）
	for (int i = 0; i < 4; i++) {
		if (decoded[i] < (uint16_t)SBUS_CH_PLAUSIBLE_MIN ||
		    decoded[i] > (uint16_t)SBUS_CH_PLAUSIBLE_MAX) return false;
	}

	memcpy(channels, decoded, sizeof(channels));
	return true;
}

// 从 UART 读取并解析遥控数据。返回 true 表示本次成功解析出至少一帧合法数据。
// 由通信任务以 1kHz 调用；单次最多消费 RC_READ_BUDGET 字节。
bool readRC() {
	if (!rcInitialized) return false;

	bool gotFrame = false;
	int budget = RC_READ_BUDGET;
	uint8_t byte;

	while (budget-- > 0 && uart_read_bytes(rcUartPort, &byte, 1, 0) == 1) {
		if (sbusIdx == 0) {
			// 只在起始字节处同步；数据区里的 0x0F 不会被误当帧头，
			// 因为一旦同步成功就是定长 25 字节，中途不再判断内容
			if (byte != SBUS_START_BYTE) continue;
			sbusFrame[sbusIdx++] = byte;
			continue;
		}

		sbusFrame[sbusIdx++] = byte;
		if (sbusIdx < SBUS_FRAME_LEN) continue;

		sbusIdx = 0; // 一帧收齐，无论成功与否都回到同步态
		if (sbusHandleFrame(sbusFrame)) {
			rcGoodFrames++;
			normalizeRC(); // 内部会刷新 controlTime
			gotFrame = true;
		} else {
			rcBadFrames++;
		}
	}

	// 物理遥控与网页遥控同时在线：物理遥控优先（见文件头"控制权归属"），提示一次即可。
	// 注意复位条件用 rcSignalOK() 而不是 "!gotFrame"：SBUS 只有约 70Hz 而本函数跑 1kHz，
	// 绝大多数调用本来就拿不到帧，用 !gotFrame 会让提示每秒刷十几遍。
#if WEB_RC_ENABLED
	if (gotFrame) {
		if (!rcDualSourceWarned && isUsingWebRC()) {
			rcDualSourceWarned = true;
			print("提示: 物理遥控与网页遥控同时在线，已切换为物理遥控主控\n");
		}
	} else if (!rcSignalOK()) {
		rcDualSourceWarned = false; // 物理遥控真正掉线后，才允许下次重新提示
	}
#endif

	return gotFrame;
}

// 把 channels[] 原始值按标定值归一化成飞控控制量，并刷新 controlTime。
// 映射：roll/pitch/yaw → [-1,1]（两段式，以中位为界）；throttle → [0,1]；mode → [0,1]。
//
// 注意 mode 通道映射到 [0,1] 而不是旧版的 [0,2]：interpretControls() 用
//   controlMode < 0.25 / <= 0.75 / > 0.75 三档判断，取值域必须是 [0,1]。
//   旧版映射到 [0,2] 时，三档开关的中位(≈0.83)和高位(≈2.0)都会落进 "> 0.75"，
//   导致 flightModes[1] 永远选不到——这是个静默的功能缺失。
void normalizeRC() {
	if (rcCalibrating) return; // 标定期间只更新 channels[]，不把乱打的杆量灌进控制链

	float lo, mid, hi;

	channelRange(0, lo, mid, hi);
	rollChannel = mapCentered((float)channels[0], lo, mid, hi, -1.0f, 1.0f);

	channelRange(1, lo, mid, hi);
	pitchChannel = mapCentered((float)channels[1], lo, mid, hi, -1.0f, 1.0f);

	// 油门是单向行程：从最小值到最大值映射到 [0,1]，不能用中位分段
	channelRange(2, lo, mid, hi);
	throttleChannel = std::clamp(mapf((float)channels[2], lo, hi, 0.0f, 1.0f), 0.0f, 1.0f);

	channelRange(3, lo, mid, hi);
	yawChannel = mapCentered((float)channels[3], lo, mid, hi, -1.0f, 1.0f);

	// 模式通道：接收机没接第 5 通道时读数会在量程外，此时置 NAN，
	// interpretControls 的三档比较对 NAN 全为假，mode 保持不变（不会被乱切）
	channelRange(4, lo, mid, hi);
	float rawMode = (float)channels[4];
	if (rawMode >= lo && rawMode <= hi) {
		modeChannel = std::clamp(mapf(rawMode, lo, hi, 0.0f, 1.0f), 0.0f, 1.0f);
	} else {
		modeChannel = NAN;
	}

	// 写入飞控控制量（物理遥控为主控，见文件头）
	controlRoll = rollChannel;
	controlPitch = pitchChannel;
	controlYaw = yawChannel;
	controlThrottle = throttleChannel;
	controlMode = modeChannel;

	controlTime = t; // 记录最近一次收到合法遥控数据的时间（供丢失保护判断）
}

// 遥控标定：10 秒内把每个通道都推到最小/中位/最大，实测写入 channelZero(中位) 与 channelMax(最大)。
// 在通信任务里执行，期间自己调 readRC() 收帧（通信任务被本函数占住，不会重入）。
// 安全约束：必须在上锁状态下执行（解锁时拒绝）；标定期间不写控制量。
void calibrateRC() {
	if (armed) {
		print("标定RC前请先上锁（disarm）\n");
		return;
	}
	if (!rcInitialized) {
		print("RC 串口未初始化，无法标定\n");
		return;
	}

	print("Calibrating RC...\n");
	print("10 秒内把每个摇杆/开关都推到 最小 -> 中位 -> 最大 各停一下\n");

	const int CAL_SECONDS = 10;
	float mn[16], mx[16];
	for (int i = 0; i < 16; i++) { mn[i] = 1e9f; mx[i] = -1e9f; }
	uint32_t frames = 0;

	rcCalibrating = true;
	int64_t startUs = esp_timer_get_time();
	while (esp_timer_get_time() - startUs < (int64_t)CAL_SECONDS * 1000000) {
		// readRC 内部会跑状态机；合法帧已经把 channels[] 更新好了
		if (readRC()) {
			frames++;
			for (int i = 0; i < 16; i++) {
				float v = (float)channels[i];
				if (v < mn[i]) mn[i] = v;
				if (v > mx[i]) mx[i] = v;
			}
		}
		vTaskDelay(pdMS_TO_TICKS(2));
	}
	rcCalibrating = false;

	if (frames == 0) {
		print("标定失败：10 秒内一帧合法 SBUS 都没收到\n");
		print("请检查：接线(RX)、接收机是否对码、RC_INVERTED 是否需要改为 0\n");
		return;
	}

	int applied = 0;
	for (int i = 0; i < 16; i++) {
		// 行程太小说明这个通道没动过（或没接），保留原值，避免把没标定的通道写成垃圾
		if (mx[i] - mn[i] < 200.0f) continue;
		if (mn[i] < SBUS_CH_PLAUSIBLE_MIN || mx[i] > SBUS_CH_PLAUSIBLE_MAX) continue;
		channelMax[i] = mx[i];
		channelZero[i] = (mn[i] + mx[i]) * 0.5f; // 中位取实测行程的中点
		applied++;
	}

	print("标定完成：收到 %u 帧，写入 %d 个通道（其余行程不足已跳过）\n", (unsigned)frames, applied);
	printRCCalibration();
	print("参数 RC_MID_*/RC_MAX_* 会在落地后由 syncParameters 自动存进 NVS\n");
}

void printRCCalibration() {
	print("RC Calibration (mid / max / derived min):\n");
	for (int i = 0; i < 5; i++) {
		float lo, mid, hi;
		channelRange(i, lo, mid, hi);
		print("  CH%d mid=%.0f max=%.0f min=%.0f\n", i, mid, hi, lo);
	}
	print("RC frames: good=%u bad=%u\n", (unsigned)rcGoodFrames, (unsigned)rcBadFrames);
	print("RC signal: %s (controlTime=%.2f, t=%.2f)\n",
	      rcSignalOK() ? "OK" : "NO SIGNAL", controlTime, t);
}

// 是否有"活的"物理遥控信号：收到过合法帧且未超时。
// 供诊断与外部判断控制权归属使用；失控保护本身走 t - controlTime（见 rcLossFailsafe）。
bool rcSignalOK() {
	return rcGoodFrames > 0 && controlTime > 0.0f && (t - controlTime) < RC_SIGNAL_TIMEOUT;
}
