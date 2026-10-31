// ============================================================================
// mavlink_compat.h —— 精简版 MAVLink v1 实现（仅覆盖本项目用到的消息）
//
// 职责:
//   不依赖官方 mavlink 库，自行实现 MAVLink v1 的组包/解包、CRC 与消息 pack/decode。
//   供 mavlink.cpp 使用，也可被 espnow.cpp 引用（取 MAVLINK_MAX_PACKET_LEN）。
//
// 关键逻辑:
//   - 帧格式：0xFE | len | seq | sysid | compid | msgid | payload | crc(2)
//     （MAVLink v1，帧头 6 字节 + 载荷 + 2 字节 CRC）；
//   - CRC 采用 CRC-16/MCRF4XX（查表 mavlink_crc_x25_table），并按消息 ID 取种子；
//   - mavlink_parse_char() 是逐字节状态机，跨调用保存状态（mavlink_status_t）。
//
// 输入/输出:
//   *_pack()  把字段写入 mavlink_message_t（不序列化）；
//   mavlink_msg_to_send_buffer() 把消息序列化成字节流；
//   mavlink_parse_char() 逐字节输入，组包完成时返回 true 并填充 mavlink_message_t。
//
// 边界情况与潜在风险:
//   - 消息 ID 存在冲突/占位：SCALED_IMU(26) 与 ACTUATOR_CONTROL_TARGET(26)、
//     SET_ACTUATOR_CONTROL_TARGET(26) 相同；且 mavlink_msg_scaled_imu_pack 里
//     msgid 直接硬编码为 26。与标准地面站交互时需以实际用到的消息为准。
//   - CRC 种子表只覆盖"本项目用到的消息"，未列出的消息种子返回 0，会导致 CRC 校验失败。
//   - pack 函数只填 msg->payload 与 len，不填 seq；seq 由调用方/静态变量维护（当前未递增）。
//   - 载荷字段布局按 MAVLink 的"字段重排"规则手工排列，改动消息定义需同步改 pack 与 decode。
//   - decode 函数按固定偏移读取 payload，若收到的消息 len 不足会读到越界数据（未做长度校验）。
//   - 若要获得完整协议支持，应替换为官方 mavlink/c_library_v2。
// ============================================================================

#pragma once

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

// 最大整包长度（帧头 6 + 载荷最大 255 + CRC 2，留余量）
#define MAVLINK_MAX_PACKET_LEN 280
// 解析通道编号（本实现只有一条通道，保留参数是为了与官方 API 形参一致）
#define MAVLINK_COMM_0 0

// ---------- 消息 ID（注意：精简实现存在 ID 冲突，见文件头风险说明） ----------
#define MAVLINK_MSG_ID_HEARTBEAT              0    // 心跳（在线状态/模式/解锁标志）
#define MAVLINK_MSG_ID_EXTENDED_SYS_STATE     245  // 扩展状态：在地面/空中
#define MAVLINK_MSG_ID_ATTITUDE_QUATERNION    31   // 姿态四元数 + 三轴角速率
#define MAVLINK_MSG_ID_RC_CHANNELS_RAW        35   // 遥控原始通道值
#define MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET 26  // 执行器控制量（这里装 4 路电机输出）
#define MAVLINK_MSG_ID_SCALED_IMU             26   // 原始 IMU —— 注意：与上一条冲突！
#define MAVLINK_MSG_ID_PARAM_VALUE            22   // 参数值（参数协议的回包）
#define MAVLINK_MSG_ID_PARAM_REQUEST_LIST     21   // 请求列出全部参数
#define MAVLINK_MSG_ID_PARAM_REQUEST_READ     20   // 请求读取单个参数
#define MAVLINK_MSG_ID_PARAM_SET              23   // 设置（写入）参数
#define MAVLINK_MSG_ID_MISSION_REQUEST_LIST   43   // 请求任务列表
#define MAVLINK_MSG_ID_MISSION_COUNT          44   // 任务总数（本机固定回 0）
#define MAVLINK_MSG_ID_SERIAL_CONTROL         126  // 串口控制（双向承载控制台文本/命令）
#define MAVLINK_MSG_ID_SET_ATTITUDE_TARGET    82   // 地面站下发目标姿态/推力
#define MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET 26 // —— 同样占用 26（冲突）
#define MAVLINK_MSG_ID_LOG_REQUEST_DATA       119  // 请求日志数据段
#define MAVLINK_MSG_ID_LOG_DATA               120  // 日志数据段
#define MAVLINK_MSG_ID_COMMAND_LONG           76   // 通用长命令（解锁/上锁、切模式）
#define MAVLINK_MSG_ID_AUTOPILOT_VERSION      148  // 自驾仪版本/能力
#define MAVLINK_MSG_ID_COMMAND_ACK            77   // 命令应答（接受/不支持）

#define MAVLINK_MSG_ID_SCALED_IMU_REAL        26   // "真实" SCALED_IMU 的占位别名

// ---------- MAVLink 枚举（仅收录本项目用到的取值） ----------
#define MAV_TYPE_QUADROTOR 2        // 机型：四旋翼
#define MAV_AUTOPILOT_GENERIC 3     // 自驾仪类型：通用
#define MAV_COMP_ID_AUTOPILOT1 1    // 组件 ID：1 号自驾仪
#define MAV_MODE_FLAG_SAFETY_ARMED 128     // 模式标志位：已解锁
#define MAV_MODE_FLAG_STABILIZE_ENABLED 4  // 模式标志位：自稳使能
#define MAV_MODE_FLAG_AUTO_ENABLED 16      // 模式标志位：自动模式使能
#define MAV_MODE_FLAG_MANUAL_INPUT_ENABLED 64 // 模式标志位：接受手动输入
#define MAV_STATE_STANDBY 3         // 系统状态：待命
#define MAV_VTOL_STATE_UNDEFINED 0  // VTOL 状态：不适用/未定义
#define MAV_LANDED_STATE_ON_GROUND 1 // 着陆状态：在地面
#define MAV_LANDED_STATE_IN_AIR 2    // 着陆状态：在空中
#define MAV_PARAM_TYPE_REAL32 9     // 参数类型：32 位浮点
#define MAV_MISSION_TYPE_MISSION 0  // 任务类型：普通任务
#define SERIAL_CONTROL_DEV_SHELL 4  // SERIAL_CONTROL 设备：shell（控制台）
#define SERIAL_CONTROL_FLAG_MULTI 2 // SERIAL_CONTROL 标志：后面还有分片
#define ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE 7 // SET_ATTITUDE_TARGET 掩码位：忽略姿态
#define MAV_CMD_REQUEST_MESSAGE 512      // 命令：请求指定消息
#define MAV_CMD_COMPONENT_ARM_DISARM 400 // 命令：解锁(1)/上锁(0)
#define MAV_CMD_DO_SET_MODE 176          // 命令：设置飞行模式
#define MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT 2 // 能力位：支持浮点参数
#define MAV_PROTOCOL_CAPABILITY_MAVLINK2 1    // 能力位：支持 MAVLink v2
#define MAV_RESULT_ACCEPTED 0      // 命令结果：已接受
#define MAV_RESULT_UNSUPPORTED 1   // 命令结果：不支持

// ---------- 关键字段长度 ----------
#define MAVLINK_MSG_ID_PARAM_SET_FIELD_PARAM_ID_LEN 16          // 参数名字段长（含结尾 0）
#define MAVLINK_MSG_ID_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN 16
#define MAVLINK_MSG_ID_SERIAL_CONTROL_FIELD_DATA_LEN 70         // 控制台文本每包 70 字节

// MAVLink v1 消息对象：pack 的写入目标 / parse 的解析输出
typedef struct {
	uint16_t len;                                 // 载荷字节数
	uint8_t incompat_flags;                       // 不兼容标志（v2 才用，v1 恒 0）
	uint8_t compat_flags;                         // 兼容标志（v2 才用）
	uint8_t seq;                                  // 包序号（本实现未递增）
	uint8_t sysid;                                // 系统 ID：哪架飞机
	uint8_t compid;                               // 组件 ID：机上哪个组件
	uint8_t msgid;                                // 消息 ID：这条是什么消息
	uint8_t payload[MAVLINK_MAX_PACKET_LEN - 12]; // 载荷（小端，按消息定义排布）
	uint16_t checksum;                            // CRC 字段（本实现序列化时现算，此处未用）
} mavlink_message_t;

// 逐字节解析器的跨调用状态：每个通道一份， mavlink_parse_char 每次调用都带进来
typedef struct {
	uint8_t msg_received;                     // 是否收到过完整包
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];   // 组包缓冲区
	uint16_t buffer_pos;                      // 当前写入位置
	uint8_t current_rx_seq;                   // 最近收到的 seq（可做丢包检测，未使用）
	uint8_t parse_state;                      // 状态机：0=找帧头 1=读长度 2=收剩余字节
	uint8_t magic;                            // 帧头字节（应为 0xFE）
	uint16_t packet_len;                      // 本包总长 = 载荷长 + 8
	uint16_t packet_rx_drop_count;            // 丢包/错包计数（本实现未维护）
	uint16_t crc_offset;                      // CRC 偏移（本实现未使用）
} mavlink_status_t;

// CRC table (MAVLink CRC-16/MCRF4XX)
static const uint16_t mavlink_crc_x25_table[256] = {
	0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
	0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
	0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
	0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
	0x2462,0x3443,0x0420,0x1401,0x64e6,0x7487,0x4444,0x5425,
	0xa5ca,0xb5eb,0x8588,0x95a9,0xe56e,0xf54f,0xc52c,0xd50d,
	0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
	0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
	0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
	0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
	0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
	0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
	0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
	0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
	0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
	0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
	0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
	0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
	0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
	0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
	0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
	0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
	0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
	0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
	0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
	0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
	0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
	0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
	0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
	0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
	0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
	0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
};

// 按消息 ID 取 CRC 种子：MAVLink 规定每种消息有专属种子，作为 CRC 的初值参与校验，
// 目的是防止"载荷被错位解析成另一种消息"也能通过校验。
// 注意：这张精简表只列了本项目用到的消息；未列出的消息返回 0，
//       与标准地面站互发那些消息时 CRC 校验必然失败（官方库有完整 256 项种子表）。
static inline uint16_t mavlink_crc_seed(uint8_t msgid) {
	switch (msgid) {
		case MAVLINK_MSG_ID_HEARTBEAT: return 50;
		case MAVLINK_MSG_ID_PARAM_VALUE: return 220;
		case MAVLINK_MSG_ID_PARAM_SET: return 168;
		case MAVLINK_MSG_ID_PARAM_REQUEST_READ: return 214;
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: return 159;
		case MAVLINK_MSG_ID_ATTITUDE_QUATERNION: return 246;
		case MAVLINK_MSG_ID_COMMAND_LONG: return 152;
		case MAVLINK_MSG_ID_COMMAND_ACK: return 9;
		case MAVLINK_MSG_ID_SERIAL_CONTROL: return 189;
		case MAVLINK_MSG_ID_MISSION_COUNT: return 114;
		case MAVLINK_MSG_ID_LOG_DATA: return 134;
		case MAVLINK_MSG_ID_AUTOPILOT_VERSION: return 231;
		default: return 0;
	}
}

// CRC-16/MCRF4XX 逐字节累加（查表法）：crc = (crc<<8) ^ table[(crc>>8) ^ byte]
// 参数 crc 为累加初值（发送/接收时都传入消息种子 mavlink_crc_seed(msgid)）
static inline uint16_t mavlink_crc_accumulate(const uint8_t *data, uint16_t len, uint16_t crc) {
	for (uint16_t i = 0; i < len; i++) {
		crc = (crc << 8) ^ mavlink_crc_x25_table[((crc >> 8) ^ data[i]) & 0xFF];
	}
	return crc;
}

// 把消息序列化成可发送的字节流（MAVLink v1 帧）
// 帧格式：0xFE | len | seq | sysid | compid | msgid | payload | crc(低字节在前)
// CRC 计算范围：从 len 字节到载荷末尾（共 5+len 字节），初值 = 本消息的种子
// 返回值：写入 buf 的总字节数 = 6(帧头) + len(载荷) + 2(CRC)
static inline uint16_t mavlink_msg_to_send_buffer(uint8_t *buf, const mavlink_message_t *msg) {
	uint16_t payload_len = msg->len;
	buf[0] = 0xFE; // MAVLink v1 帧头（magic）
	buf[1] = (uint8_t)payload_len; // 载荷长度
	buf[2] = msg->seq;   // 包序号
	buf[3] = msg->sysid; // 系统 ID
	buf[4] = msg->compid;// 组件 ID
	buf[5] = msg->msgid; // 消息 ID
	memcpy(&buf[6], msg->payload, payload_len); // 载荷原样拷贝
	// CRC 覆盖 buf[1..5+payload_len]，种子按消息 ID 取
	uint16_t crc = mavlink_crc_accumulate(&buf[1], 5 + payload_len, mavlink_crc_seed(msg->msgid));
	buf[6 + payload_len] = (uint8_t)(crc & 0xFF);      // CRC 低字节在前（小端）
	buf[6 + payload_len + 1] = (uint8_t)(crc >> 8);    // CRC 高字节
	return 6 + payload_len + 2; // 帧头 + 载荷 + CRC
}

// 逐字节解析器：每收到 1 个字节调用一次；组包完成且 CRC 校验通过时
// 返回 true 并填充 msg（供上层按 msgid 分发），否则返回 false。
// 状态机流程（status 跨调用保存进度）：
//   状态0：等待帧头 0xFE；
//   状态1：读第 2 字节 = 载荷长度，推算整包长 = len + 8（6 帧头 + 2 CRC）；
//   状态2：继续接收直到收满整包，然后计算 CRC 并比对：
//          一致 → 填充 msg 返回 true；不一致 → 整包静默丢弃，回到状态0重找帧头。
// 注意：本实现不做"半包纠错/重同步"（官方库会在 CRC 失败后尝试从包内再找帧头）；
//       chan 参数保留以兼容官方 API，本实现只有一条通道。
static inline bool mavlink_parse_char(uint8_t chan, uint8_t c, mavlink_message_t *msg, mavlink_status_t *status) {
	(void)chan; // 通道号未使用
	if (status->parse_state == 0) {
		if (c == 0xFE) {
			status->buffer[0] = c;
			status->parse_state = 1;
			status->buffer_pos = 1;
		}
	} else if (status->parse_state == 1) {
		status->buffer[1] = c;
		status->packet_len = c + 8; // payload + 6 header + 2 crc
		status->parse_state = 2;
		status->buffer_pos = 2;
	} else {
		status->buffer[status->buffer_pos++] = c;
		if (status->buffer_pos >= status->packet_len) {
			// Complete packet
			uint16_t payload_len = status->buffer[1];
			uint16_t crc = mavlink_crc_accumulate(&status->buffer[1], 5 + payload_len, mavlink_crc_seed(status->buffer[5]));
			uint16_t recv_crc = status->buffer[6 + payload_len] | (status->buffer[6 + payload_len + 1] << 8);
			if (crc == recv_crc) {
				msg->len = payload_len;
				msg->seq = status->buffer[2];
				msg->sysid = status->buffer[3];
				msg->compid = status->buffer[4];
				msg->msgid = status->buffer[5];
				memcpy(msg->payload, &status->buffer[6], payload_len);
				status->parse_state = 0;
				status->msg_received = 1;
				return true;
			}
			status->parse_state = 0;
		}
	}
	return false;
}

// ============================================================================
// 消息打包函数（pack）—— 把字段写入 mavlink_message_t（只填载荷，不序列化）
//
// 所有 pack 函数的共同约定：
//   1) 写入 msgid/sysid/compid 到消息头，msg->len = 该消息载荷的固定字节数；
//   2) 载荷按 MAVLink 规范"字段重排"后的顺序排列（大字段在前、小字段在后），
//      多字节字段一律小端 —— 代码里用 memcpy 直接拷贝原生内存（ESP32 为小端）；
//   3) 只填 payload，不填 seq（本实现未递增）也不算 CRC，序列化交给
//      mavlink_msg_to_send_buffer()；
//   4) 个别函数会先 memset 清零载荷（如 heartbeat/autopilot_version），其余只写已用字段。
//
// 各消息载荷布局（字段顺序必须与下方 decode 保持一致，否则解出的值错位）：
//   heartbeat(9)             type|autopilot|base_mode|custom_mode|system_status|mavlink_version
//   extended_sys_state(2)    vtol_state|landed_state
//   attitude_quaternion(32)  time_ms|q0..q3|rollspeed|pitchspeed|yawspeed（全 float）
//   rc_channels_raw(22)      time_ms|port|chan1..8(u16)|rssi
//   actuator_control_target(41) time_ms|group|controls[8](float)
//   scaled_imu(24)           time_ms|xacc|yacc|zacc|xgyro|ygyro|zgyro|xmag|ymag|zmag|temp（int16）
//                            ⚠ msgid 硬编码 26，与 ACTUATOR_CONTROL_TARGET 冲突
//   param_value(25)          param_value(float)|param_type|param_count|param_index|param_id[16]
//   mission_count(5)         target_sys|target_comp|count(u16)|mission_type
//   serial_control(79)       device|flags|timeout(u16)|baudrate(u32)|count|data[70]
//   log_data(90)             id(u16)|ofs(u32)|count|data[90]
//   autopilot_version(60)    capabilities(u64)|flight_sw|middleware_sw|os_sw|board_version|
//                            flight_custom|middleware_custom|os_custom|vendor_id|product_id|uid
//   command_ack(9)           command(u16)|result|progress|result_param2(i32)|target_sys
// ============================================================================

static inline void mavlink_msg_heartbeat_pack(uint8_t sysid, uint8_t compid, mavlink_message_t *msg,
	uint8_t type, uint8_t autopilot, uint8_t base_mode, uint8_t custom_mode, uint8_t system_status) {
	msg->msgid = MAVLINK_MSG_ID_HEARTBEAT;
	msg->sysid = sysid;
	msg->compid = compid;
	msg->len = 9;
	memset(msg->payload, 0, sizeof(msg->payload));
	msg->payload[0] = type;
	msg->payload[1] = autopilot;
	msg->payload[2] = base_mode;
	msg->payload[3] = custom_mode;
	msg->payload[4] = system_status;
	msg->payload[5] = 3; // mavlink version 3
}

static inline void mavlink_msg_extended_sys_state_pack(uint8_t sysid, uint8_t compid, mavlink_message_t *msg,
	uint8_t vtol_state, uint8_t landed_state) {
	msg->msgid = MAVLINK_MSG_ID_EXTENDED_SYS_STATE;
	msg->sysid = sysid;
	msg->compid = compid;
	msg->len = 2;
	msg->payload[0] = vtol_state;
	msg->payload[1] = landed_state;
}

static inline void mavlink_msg_attitude_quaternion_pack(uint8_t sysid, uint8_t compid, mavlink_message_t *msg,
	uint32_t time_ms, float q0, float q1, float q2, float q3, float rollspeed, float pitchspeed, float yawspeed,
	const float *offset) {
	(void)offset;
	msg->msgid = MAVLINK_MSG_ID_ATTITUDE_QUATERNION;
	msg->sysid = sysid;
	msg->compid = compid;
	msg->len = 32;
	uint8_t *p = msg->payload;
	memcpy(p, &time_ms, 4); p += 4;
	memcpy(p, &q0, 4); p += 4;
	memcpy(p, &q1, 4); p += 4;
	memcpy(p, &q2, 4); p += 4;
	memcpy(p, &q3, 4); p += 4;
	memcpy(p, &rollspeed, 4); p += 4;
	memcpy(p, &pitchspeed, 4); p += 4;
	memcpy(p, &yawspeed, 4); p += 4;
}

static inline void mavlink_msg_rc_channels_raw_pack(uint8_t sysid, uint8_t compid, mavlink_message_t *msg,
	uint32_t time_ms, uint8_t port, uint16_t c0, uint16_t c1, uint16_t c2, uint16_t c3,
	uint16_t c4, uint16_t c5, uint16_t c6, uint16_t c7, uint8_t rssi) {
	msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS_RAW;
	msg->sysid = sysid;
	msg->compid = compid;
	msg->len = 22;
	uint8_t *p = msg->payload;
	memcpy(p, &time_ms, 4); p += 4;
	*p++ = port;
	memcpy(p, &c0, 2); p += 2;
	memcpy(p, &c1, 2); p += 2;
	memcpy(p, &c2, 2); p += 2;
	memcpy(p, &c3, 2); p += 2;
	memcpy(p, &c4, 2); p += 2;
	memcpy(p, &c5, 2); p += 2;
	memcpy(p, &c6, 2); p += 2;
	memcpy(p, &c7, 2); p += 2;
	*p = rssi;
}

static inline void mavlink_msg_actuator_control_target_pack(uint8_t sysid, uint8_t compid, mavlink_message_t *msg,
	uint32_t time_ms, uint8_t group, const float *controls) {
	msg->msgid = MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET;
	msg->sysid = sysid;
	msg->compid = compid;
	msg->len = 41;
	uint8_t *p = msg->payload;
	memcpy(p, &time_ms, 4); p += 4;
	*p++ = group;
	memcpy(p, controls, 32);
}

static inline void mavlink_msg_scaled_imu_pack(uint8_t sysid, uint8_t compid, mavlink_message_t *msg,
	uint32_t time_ms, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro,
	int16_t xmag, int16_t ymag, int16_t zmag, int16_t temp) {
	msg->msgid = 26;
	msg->sysid = sysid;
	msg->compid = compid;
	msg->len = 24;
	uint8_t *p = msg->payload;
	memcpy(p, &time_ms, 4); p += 4;
	memcpy(p, &xacc, 2); p += 2;
	memcpy(p, &yacc, 2); p += 2;
	memcpy(p, &zacc, 2); p += 2;
	memcpy(p, &xgyro, 2); p += 2;
	memcpy(p, &ygyro, 2); p += 2;
	memcpy(p, &zgyro, 2); p += 2;
	memcpy(p, &xmag, 2); p += 2;
	memcpy(p, &ymag, 2); p += 2;
	memcpy(p, &zmag, 2); p += 2;
	memcpy(p, &temp, 2);
}

static inline void mavlink_msg_param_value_pack(uint8_t sysid, uint8_t compid, mavlink_message_t *msg,
	const char *param_id, float param_value, uint8_t param_type, uint16_t param_count, uint16_t param_index) {
	msg->msgid = MAVLINK_MSG_ID_PARAM_VALUE;
	msg->sysid = sysid;
	msg->compid = compid;
	msg->len = 25;
	uint8_t *p = msg->payload;
	memcpy(p, &param_value, 4); p += 4;
	memcpy(p, &param_type, 1); p += 1;
	memcpy(p, &param_count, 2); p += 2;
	memcpy(p, &param_index, 2); p += 2;
	memset(p, 0, 16);
	strncpy((char*)p, param_id, 16);
}

static inline void mavlink_msg_mission_count_pack(uint8_t sysid, uint8_t compid, mavlink_message_t *msg,
	uint8_t target_sys, uint8_t target_comp, uint16_t count, uint8_t mission_type, uint8_t opaque) {
	(void)opaque;
	msg->msgid = MAVLINK_MSG_ID_MISSION_COUNT;
	msg->sysid = sysid;
	msg->compid = compid;
	msg->len = 5;
	uint8_t *p = msg->payload;
	*p++ = target_sys;
	*p++ = target_comp;
	memcpy(p, &count, 2); p += 2;
	*p = mission_type;
}

static inline void mavlink_msg_serial_control_pack(uint8_t sysid, uint8_t compid, mavlink_message_t *msg,
	uint8_t device, uint8_t flags, uint16_t timeout, uint32_t baudrate, uint8_t count, const uint8_t *data, uint8_t a, uint8_t b) {
	(void)a; (void)b;
	msg->msgid = MAVLINK_MSG_ID_SERIAL_CONTROL;
	msg->sysid = sysid;
	msg->compid = compid;
	msg->len = 79;
	uint8_t *p = msg->payload;
	*p++ = device;
	*p++ = flags;
	memcpy(p, &timeout, 2); p += 2;
	memcpy(p, &baudrate, 4); p += 4;
	*p++ = count;
	memset(p, 0, 70);
	memcpy(p, data, count);
}

static inline void mavlink_msg_log_data_pack(uint8_t sysid, uint8_t compid, mavlink_message_t *msg,
	uint16_t id, uint32_t ofs, uint8_t count, const uint8_t *data) {
	msg->msgid = MAVLINK_MSG_ID_LOG_DATA;
	msg->sysid = sysid;
	msg->compid = compid;
	msg->len = 90;
	uint8_t *p = msg->payload;
	memcpy(p, &id, 2); p += 2;
	memcpy(p, &ofs, 4); p += 4;
	*p++ = count;
	memset(p, 0, 90);
	memcpy(p, data, count);
}

static inline void mavlink_msg_autopilot_version_pack(uint8_t sysid, uint8_t compid, mavlink_message_t *msg,
	uint64_t capabilities, uint32_t flight_sw_version, uint32_t middleware_sw_version, uint32_t os_sw_version,
	uint32_t board_version, uint16_t flight_custom_version, uint16_t middleware_custom_version,
	uint16_t os_custom_version, uint16_t vendor_id, uint16_t product_id, uint64_t uid, uint8_t uid2) {
	(void)uid2;
	msg->msgid = MAVLINK_MSG_ID_AUTOPILOT_VERSION;
	msg->sysid = sysid;
	msg->compid = compid;
	msg->len = 60;
	uint8_t *p = msg->payload;
	memset(p, 0, 60);
	memcpy(p, &capabilities, 8); p += 8;
	memcpy(p, &flight_sw_version, 4); p += 4;
	memcpy(p, &middleware_sw_version, 4); p += 4;
	memcpy(p, &os_sw_version, 4); p += 4;
	memcpy(p, &board_version, 4); p += 4;
	memcpy(p, &flight_custom_version, 2); p += 2;
	memcpy(p, &middleware_custom_version, 2); p += 2;
	memcpy(p, &os_custom_version, 2); p += 2;
	memcpy(p, &vendor_id, 2); p += 2;
	memcpy(p, &product_id, 2); p += 2;
	memcpy(p, &uid, 8);
}

static inline void mavlink_msg_command_ack_pack(uint8_t sysid, uint8_t compid, mavlink_message_t *msg,
	uint16_t command, uint8_t result, uint8_t progress, int32_t result_param2, uint8_t target_sys, uint8_t target_comp) {
	msg->msgid = MAVLINK_MSG_ID_COMMAND_ACK;
	msg->sysid = sysid;
	msg->compid = compid;
	msg->len = 9;
	uint8_t *p = msg->payload;
	memcpy(p, &command, 2); p += 2;
	*p++ = result;
	*p++ = progress;
	memcpy(p, &result_param2, 4); p += 4;
	*p = target_sys;
}

// ============================================================================
// 消息解码函数（decode）—— 从 msg->payload 按固定偏移读出字段填入结构体
//
// 共同约定：
//   1) 每种下行消息配一个 XXX_t 结构体 + 一个 mavlink_msg_XXX_decode()；
//   2) 字段顺序/字节序与 pack 完全一致（小端，memcpy 直接按内存偏移读）；
//   3) ⚠ 本实现不校验 msg->len，载荷不足时会越界读（依赖 CRC 先过滤坏包）；
//   4) 解码后由 mavlink.cpp 的 handleMavlink() 按 msgid 分发处理。
//
// 两个易踩的偏移坑（都是官方"字段重排"导致，并非按声明顺序排列）：
//   - param_set：target_system 在载荷末尾偏移 21，而不是开头；
//   - manual_control：x/y/z/r 各 int16（-1000~1000），buttons 在偏移 8，
//     target 在载荷末尾偏移 10。
// ============================================================================

// ---- 各下行消息的解码目标结构体 ----
typedef struct { uint8_t target_system; uint16_t param_index; char param_id[16]; } mavlink_param_request_read_t;          // 读单个参数：目标系统/参数下标/参数名
typedef struct { uint8_t target_system; } mavlink_param_request_list_t;                                                   // 请求列出全部参数
typedef struct { uint8_t target_system; float param_value; uint8_t param_type; char param_id[16]; } mavlink_param_set_t;  // 设置参数：目标/新值/类型/参数名
typedef struct { uint8_t target_system; uint8_t target_component; } mavlink_mission_request_list_t;                       // 请求任务列表
typedef struct { uint8_t target_system; uint16_t count; uint8_t mission_type; } mavlink_mission_count_t;                  // 任务总数（本机回包用）
typedef struct { uint8_t target_system; uint8_t device; uint16_t flags; uint16_t timeout; uint32_t baudrate; uint8_t count; uint8_t data[70]; } mavlink_serial_control_t; // 串口控制：承载控制台文本/命令
typedef struct { uint8_t target_system; uint8_t target_component; uint8_t type_mask; float q[4]; float body_roll_rate; float body_pitch_rate; float body_yaw_rate; float thrust; } mavlink_set_attitude_target_t; // 目标姿态四元数/角速率/推力
typedef struct { uint8_t target_system; uint8_t target_component; uint8_t group_mlx; float controls[8]; } mavlink_set_actuator_control_target_t;  // 目标执行器控制量（8 通道）
typedef struct { uint16_t id; uint32_t ofs; uint16_t count; } mavlink_log_request_data_t;                                 // 请求日志：日志ID/偏移/字节数
typedef struct { uint8_t target_system; uint16_t command; float param1; float param2; float param3; float param4; float param5; float param6; float param7; } mavlink_command_long_t; // 通用长命令：命令号 + 7 个参数
typedef struct { int16_t x, y, z, r; uint16_t buttons; int16_t target; } mavlink_manual_control_t;                        // 手动控制：x/y/z/r ∈ [-1000,1000]，buttons 按位，target 系统号

static inline void mavlink_msg_param_request_list_decode(const mavlink_message_t *msg, mavlink_param_request_list_t *m) {
	m->target_system = msg->payload[0];
}

static inline void mavlink_msg_param_request_read_decode(const mavlink_message_t *msg, mavlink_param_request_read_t *m) {
	m->target_system = msg->payload[0];
	m->param_index = msg->payload[1] | (msg->payload[2] << 8);
	memset(m->param_id, 0, 16);
	memcpy(m->param_id, &msg->payload[3], 16);
}

static inline void mavlink_msg_param_set_decode(const mavlink_message_t *msg, mavlink_param_set_t *m) {
	memcpy(&m->param_value, &msg->payload[0], 4);
	m->param_type = msg->payload[4];
	memset(m->param_id, 0, 16);
	memcpy(m->param_id, &msg->payload[5], 16);
	m->target_system = msg->payload[21];
}

static inline void mavlink_msg_mission_request_list_decode(const mavlink_message_t *msg, mavlink_mission_request_list_t *m) {
	m->target_system = msg->payload[0];
	m->target_component = msg->payload[1];
}

static inline void mavlink_msg_serial_control_decode(const mavlink_message_t *msg, mavlink_serial_control_t *m) {
	m->device = msg->payload[0];
	m->flags = msg->payload[1] | (msg->payload[2] << 8);
	memcpy(&m->timeout, &msg->payload[3], 2);
	memcpy(&m->baudrate, &msg->payload[5], 4);
	m->count = msg->payload[9];
	memcpy(m->data, &msg->payload[10], 70);
	m->target_system = 0;
}

static inline void mavlink_msg_set_attitude_target_decode(const mavlink_message_t *msg, mavlink_set_attitude_target_t *m) {
	m->target_system = msg->payload[0];
	m->target_component = msg->payload[1];
	m->type_mask = msg->payload[2];
	memcpy(m->q, &msg->payload[4], 16);
	memcpy(&m->body_roll_rate, &msg->payload[20], 4);
	memcpy(&m->body_pitch_rate, &msg->payload[24], 4);
	memcpy(&m->body_yaw_rate, &msg->payload[28], 4);
	memcpy(&m->thrust, &msg->payload[32], 4);
}

static inline void mavlink_msg_set_actuator_control_target_decode(const mavlink_message_t *msg, mavlink_set_actuator_control_target_t *m) {
	memcpy(m->controls, &msg->payload[5], 32);
	m->target_system = msg->payload[0];
	m->target_component = msg->payload[1];
}

static inline void mavlink_msg_log_request_data_decode(const mavlink_message_t *msg, mavlink_log_request_data_t *m) {
	memcpy(&m->id, &msg->payload[0], 2);
	memcpy(&m->ofs, &msg->payload[2], 4);
	memcpy(&m->count, &msg->payload[6], 2);
}

static inline void mavlink_msg_command_long_decode(const mavlink_message_t *msg, mavlink_command_long_t *m) {
	memcpy(&m->param1, &msg->payload[0], 4);
	memcpy(&m->param2, &msg->payload[4], 4);
	memcpy(&m->param3, &msg->payload[8], 4);
	memcpy(&m->param4, &msg->payload[12], 4);
	memcpy(&m->param5, &msg->payload[16], 4);
	memcpy(&m->param6, &msg->payload[20], 4);
	memcpy(&m->param7, &msg->payload[24], 4);
	m->command = msg->payload[28] | (msg->payload[29] << 8);
	m->target_system = msg->payload[30];
}

static inline void mavlink_msg_manual_control_decode(const mavlink_message_t *msg, mavlink_manual_control_t *m) {
	memcpy(&m->x, &msg->payload[0], 2);
	memcpy(&m->y, &msg->payload[2], 2);
	memcpy(&m->z, &msg->payload[4], 2);
	memcpy(&m->r, &msg->payload[6], 2);
	m->buttons = msg->payload[8] | (msg->payload[9] << 8);
	memcpy(&m->target, &msg->payload[10], 2);
}
