// ============================================================================
// mavlink.cpp —— MAVLink 协议应用层（遥测上行 / 指令下行）
//
// 职责:
//   在 wifi.cpp（或 espnow.cpp）提供的 sendWiFi()/receiveWiFi() 字节通道之上，
//   用 mavlink_compat.h 里的精简 MAVLink v1 实现完成：
//     - 上行遥测：心跳、扩展状态、姿态四元数、RC 通道、电机输出、原始 IMU、控制台文本；
//     - 下行指令：手动控制、参数读写（列表/读取/设置）、任务列表、串口控制台、姿态目标、命令长包。
//
// 关键逻辑:
//   - processMavlink() 每周期调用，内部先发后收；
//   - 发送按两条限速通道：telemetrySlow(2Hz，心跳/状态) 与 telemetryFast(板级 10Hz，
//     姿态/RC/电机/IMU)；fast 通道仅在 mavlinkConnected 后才发；
//   - 接收用 mavlink_parse_char 逐字节组包，收到首个包即置 mavlinkConnected=true，
//     并据此把 UDP/ESP-NOW 的对端地址学习为实际来源（见 wifi.cpp/espnow.cpp）；
//   - 控制台文本（mavlinkPrint）先缓存到 2048 字节缓冲，再按 70 字节/包以 SERIAL_CONTROL 发出。
//
// 输入/输出:
//   输入：全局 armed/mode/attitude/rates/acc/gyro/channels/motors/t；来自地面站的 MAVLink 帧。
//   输出：MAVLink 帧（经 sendWiFi）；对 controlRoll/Pitch/Yaw/Throttle、mode、armed、参数的修改。
//
// 重要参数:
//   mavlinkSysId（MAV_SYS_ID，默认 1）；telemetrySlow.rate（MAV_RATE_SLOW=2）；
//   telemetryFast.rate（MAV_RATE_FAST，板级 BOARD_MAVLINK_TELEM_FAST_HZ）。
//
// 边界情况与潜在风险:
//   - 姿态/角速率/加速度在打包时对 y、z 分量取了负号（坐标系与 MAVLink 惯例的差异），
//     地面站看到的朝向依赖于这一组符号，改动需与地面站一并核对。
//   - 本实现为"精简 MAVLink v1"，消息 ID 有冲突（如 SCALED_IMU 与 ACTUATOR_CONTROL_TARGET
//     都写作 26），CRC 种子表也只覆盖用到的消息，兼容性有限。
//   - handleMavlink 的 SET_ATTITUDE_TARGET 只在 mode==AUTO 时生效，且会直接改写 armed，
//     存在被地面站误触发的风险；MAV_CMD_COMPONENT_ARM_DISARM 也要求油门处于最低。
//   - 参数请求会逐条回包，参数表较大时会短时间内产生大量报文（可能挤占链路）。
//   - mavlinkPrintBuffer 为固定 2048 字节，超长文本会被静默丢弃（不会截断提示）。
//   - 仅在 WIFI_ENABLED 时编译；WIFI 关闭时本文件为空。
// ============================================================================

#include "globals.h"

#if WIFI_ENABLED

#include "mavlink_compat.h"
#include <string.h>
#include <stdio.h>

int mavlinkSysId = 1;
Rate telemetrySlow(2);
Rate telemetryFast(BOARD_MAVLINK_TELEM_FAST_HZ);

bool mavlinkConnected = false;
static char mavlinkPrintBuffer[2048];
static int mavlinkPrintLen = 0;

extern uint16_t channels[16];
extern float motors[4];

void sendMessage(const mavlink_message_t *msg);
void handleMavlink(const mavlink_message_t *msg);
void sendMavlinkPrint();
void sendMavlink();
void receiveMavlink();

// MAVLink 处理主入口：每个控制周期调用一次（先发遥测，再收指令）
void processMavlink() {
	sendMavlink();
	receiveMavlink();
}

// 按限速通道发送各类遥测报文。
// slow 通道（2Hz）：心跳 + 扩展状态；fast 通道（板级 10Hz）：姿态/RC/电机/IMU。
// fast 通道仅在已与地面站建立联系（mavlinkConnected）后才发送，避免无谓流量。
void sendMavlink() {
	sendMavlinkPrint(); // 先补发缓冲中的控制台文本

	mavlink_message_t msg;
	uint32_t time_ms = (uint32_t)(t * 1000);

	if (telemetrySlow) {
		// 心跳：携带解锁状态与当前模式
		mavlink_msg_heartbeat_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &msg,
			MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC,
			(armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0) |
			((mode == STAB) ? MAV_MODE_FLAG_STABILIZE_ENABLED : 0) |
			((mode == AUTO) ? MAV_MODE_FLAG_AUTO_ENABLED : MAV_MODE_FLAG_MANUAL_INPUT_ENABLED),
			mode, MAV_STATE_STANDBY);
		sendMessage(&msg);

		if (!mavlinkConnected) return; // 未建立联系前只发心跳

		// 扩展状态：是否在地面
		mavlink_msg_extended_sys_state_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &msg,
			MAV_VTOL_STATE_UNDEFINED, landed ? MAV_LANDED_STATE_ON_GROUND : MAV_LANDED_STATE_IN_AIR);
		sendMessage(&msg);
	}

	if (telemetryFast && mavlinkConnected) {
		// 姿态四元数（注意 y/z 取负，用于匹配 MAVLink 坐标系）
		const float offset[] = {0, 0, 0, 0};
		mavlink_msg_attitude_quaternion_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &msg,
			time_ms, attitude.w, attitude.x, -attitude.y, -attitude.z, rates.x, -rates.y, -rates.z, offset);
		sendMessage(&msg);

		// RC 原始通道（仅在收到过通道数据时才发）
		mavlink_msg_rc_channels_raw_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &msg,
			(uint32_t)(controlTime * 1000), 0,
			channels[0], channels[1], channels[2], channels[3],
			channels[4], channels[5], channels[6], channels[7], UINT8_MAX);
		if (channels[0] != 0) sendMessage(&msg);

		// 电机输出（把 4 路归一化推力塞进 8 元控制量数组）
		float controls[8] = {0};
		memcpy(controls, motors, sizeof(motors));
		mavlink_msg_actuator_control_target_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &msg, time_ms, 0, controls);
		sendMessage(&msg);

		// 原始 IMU（加速度以 mg 为单位，y/z 取负；角速度以 mrad/s 为单位）
		mavlink_msg_scaled_imu_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &msg, time_ms,
			(int16_t)(acc.x / ONE_G * 1000), (int16_t)(-acc.y / ONE_G * 1000), (int16_t)(-acc.z / ONE_G * 1000),
			(int16_t)(gyro.x * 1000), (int16_t)(-gyro.y * 1000), (int16_t)(-gyro.z * 1000),
			0, 0, 0, 0);
		sendMessage(&msg);
	}
}

// 把一条已构造好的 MAVLink 消息序列化后经字节通道发出（UDP 或 ESP-NOW）
void sendMessage(const mavlink_message_t *msg) {
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	int len = mavlink_msg_to_send_buffer(buf, msg);
	sendWiFi(buf, len);
}

// 接收并解析地面站发来的数据。收到任意数据即视为"已连接"（mavlinkConnected=true），
// 这会触发发送端学习对端地址，并解锁 fast 遥测通道。
void receiveMavlink() {
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	int len = receiveWiFi(buf, MAVLINK_MAX_PACKET_LEN);
	if (len) mavlinkConnected = true;

	mavlink_message_t msg;
	static mavlink_status_t status = {}; // 跨帧保持解析状态
	for (int i = 0; i < len; i++) {
		if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
			handleMavlink(&msg); // 组包完成，分发处理
		}
	}
}

// 按消息 ID 分发处理下行指令。
// 处理的报文：MANUAL_CONTROL(69)、PARAM_REQUEST_LIST(21)、PARAM_REQUEST_READ(20)、
// PARAM_SET(23)、MISSION_REQUEST_LIST(43)、SERIAL_CONTROL(126)、
// SET_ATTITUDE_TARGET(82)、COMMAND_LONG(76)。
// 注意：多处通过 target_system 过滤非本机报文。
void handleMavlink(const mavlink_message_t *msg) {
	if (msg->msgid == 69) { // MAVLINK_MSG_ID_MANUAL_CONTROL = 69
		mavlink_manual_control_t m;
		mavlink_msg_manual_control_decode(msg, &m);
		if (m.target && m.target != mavlinkSysId) return;
		controlThrottle = m.z / 1000.0f;
		controlPitch = m.x / 1000.0f;
		controlRoll = m.y / 1000.0f;
		controlYaw = m.r / 1000.0f;
		controlMode = NAN;
		controlTime = t;
	}

	if (msg->msgid == MAVLINK_MSG_ID_PARAM_REQUEST_LIST) {
		mavlink_param_request_list_t m;
		mavlink_msg_param_request_list_decode(msg, &m);
		if (m.target_system && m.target_system != mavlinkSysId) return;
		mavlink_message_t out;
		for (int i = 0; i < parametersCount(); i++) {
			mavlink_msg_param_value_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &out,
				getParameterName(i), getParameter(i), MAV_PARAM_TYPE_REAL32, parametersCount(), i);
			sendMessage(&out);
		}
	}

	if (msg->msgid == MAVLINK_MSG_ID_PARAM_REQUEST_READ) {
		mavlink_param_request_read_t m;
		mavlink_msg_param_request_read_decode(msg, &m);
		if (m.target_system && m.target_system != mavlinkSysId) return;
		char name[17] = {0};
		strlcpy(name, m.param_id, sizeof(name));
		float value = strlen(name) == 0 ? getParameter(m.param_index) : getParameter(name);
		mavlink_message_t out;
		mavlink_msg_param_value_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &out,
			name, value, MAV_PARAM_TYPE_REAL32, parametersCount(), m.param_index);
		sendMessage(&out);
	}

	if (msg->msgid == MAVLINK_MSG_ID_PARAM_SET) {
		mavlink_param_set_t m;
		mavlink_msg_param_set_decode(msg, &m);
		if (m.target_system && m.target_system != mavlinkSysId) return;
		bool success = setParameter(m.param_id, m.param_value);
		if (!success) return;
		mavlink_message_t out;
		mavlink_msg_param_value_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &out,
			m.param_id, getParameter(m.param_id), MAV_PARAM_TYPE_REAL32, parametersCount(), 0);
		sendMessage(&out);
	}

	if (msg->msgid == MAVLINK_MSG_ID_MISSION_REQUEST_LIST) {
		mavlink_mission_request_list_t m;
		mavlink_msg_mission_request_list_decode(msg, &m);
		if (m.target_system && m.target_system != mavlinkSysId) return;
		mavlink_message_t out;
		mavlink_msg_mission_count_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &out, 0, 0, 0, MAV_MISSION_TYPE_MISSION, 0);
		sendMessage(&out);
	}

	if (msg->msgid == MAVLINK_MSG_ID_SERIAL_CONTROL) {
		mavlink_serial_control_t m;
		mavlink_msg_serial_control_decode(msg, &m);
		char data[71] = {0};
		strlcpy(data, (const char *)m.data, m.count);
		doCommand(data, true);
	}

	if (msg->msgid == MAVLINK_MSG_ID_SET_ATTITUDE_TARGET) {
		if (mode != AUTO) return;
		mavlink_set_attitude_target_t m;
		mavlink_msg_set_attitude_target_decode(msg, &m);
		if (m.target_system && m.target_system != mavlinkSysId) return;
		// 目标块被控制任务整周期消费，写入须持 targetMutex（多任务架构，见 main.cpp）
		if (xSemaphoreTake(targetMutex, pdMS_TO_TICKS(10)) != pdTRUE) return;
		ratesTarget.x = m.body_roll_rate;
		ratesTarget.y = -m.body_pitch_rate;
		ratesTarget.z = -m.body_yaw_rate;
		attitudeTarget.w = m.q[0];
		attitudeTarget.x = m.q[1];
		attitudeTarget.y = -m.q[2];
		attitudeTarget.z = -m.q[3];
		thrustTarget = m.thrust;
		ratesExtra = Vector(0, 0, 0);
		if (m.type_mask & ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE) attitudeTarget.invalidate();
		armed = m.thrust > 0;
		xSemaphoreGive(targetMutex);
	}

	if (msg->msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
		mavlink_command_long_t m;
		mavlink_msg_command_long_decode(msg, &m);
		if (m.target_system && m.target_system != mavlinkSysId) return;
		bool accepted = false;

		if (m.command == MAV_CMD_COMPONENT_ARM_DISARM) {
			if (m.param1 && controlThrottle > 0.05) return;
			accepted = true;
			armed = (int)m.param1 == 1;
		}

		if (m.command == MAV_CMD_DO_SET_MODE) {
			if (m.param2 < 0 || m.param2 > AUTO) return;
			accepted = true;
			mode = (int)m.param2;
		}

		mavlink_message_t ack;
		mavlink_msg_command_ack_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &ack,
			m.command, accepted ? MAV_RESULT_ACCEPTED : MAV_RESULT_UNSUPPORTED, UINT8_MAX, 0, msg->sysid, msg->compid);
		sendMessage(&ack);
	}
}

// 把控制台文本追加到发送缓冲（由 print() 调用）。
// 注意：缓冲固定 2048 字节，超出部分会被静默丢弃（不会截断已有内容）。
void mavlinkPrint(const char* str) {
	int slen = strlen(str);
	if (mavlinkPrintLen + slen < (int)sizeof(mavlinkPrintBuffer) - 1) {
		memcpy(mavlinkPrintBuffer + mavlinkPrintLen, str, slen);
		mavlinkPrintLen += slen;
		mavlinkPrintBuffer[mavlinkPrintLen] = '\0';
	}
}

// 把缓冲中的控制台文本按 70 字节/包以 SERIAL_CONTROL 报文发出，发完清空缓冲。
// 多包时除最后一片外都带 MULTI 标志，供地面站拼装。
void sendMavlinkPrint() {
	if (mavlinkPrintLen == 0) return;
	const char *str = mavlinkPrintBuffer;
	int totalLen = mavlinkPrintLen;
	for (int i = 0; i < totalLen; i += 70) {
		int chunk = (totalLen - i < 70) ? (totalLen - i) : 70;
		mavlink_message_t msg;
		mavlink_msg_serial_control_pack(mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &msg,
			SERIAL_CONTROL_DEV_SHELL,
			(i + 70 < totalLen) ? SERIAL_CONTROL_FLAG_MULTI : 0,
			0, 0, chunk, (const uint8_t*)(str + i), 0, 0);
		sendMessage(&msg);
	}
	mavlinkPrintLen = 0;
	mavlinkPrintBuffer[0] = '\0';
}

#endif // WIFI_ENABLED
