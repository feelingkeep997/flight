// ESP-NOW 通讯（MAVLink 传输层替代 UDP 版本）- ESP-IDF 移植版
// 当 ESPNOW_ENABLED 时，本文件完全取代 wifi.cpp 中的 UDP/AP/HTTP 链路：
// MAVLink 仍由 mavlink.cpp 打包/解包，本文件只提供 sendWiFi()/receiveWiFi()
// 两个接口，对外行为与 UDP 版本一致，因此 mavlink.cpp 无需任何改动。
//
// 要点：
//  - ESP-NOW 跑在 WiFi 射频上，但仍需 esp_wifi_init + STA 模式 + start（不连路由器、不开 AP）。
//  - 单包载荷上限约 250 字节，而 MAVLINK_MAX_PACKET_LEN=280，故长 MAVLink 帧需分片/重组。
//  - 目标（地面站）MAC 默认广播；首次收到数据包后学习为单播（与 UDP 行为一致）。
//
// 编译开关:
//   需要 WIFI_ENABLED && ESPNOW_ENABLED 同时成立。当前工程中 ESPNOW_ENABLED
//   **没有任何地方定义**（不在 board_config.h、CMakeLists、sdkconfig 中），
//   因此本文件默认**不参与编译**，实际走的是 wifi.cpp 的 UDP 实现。
//   若要启用 ESP-NOW，需显式定义 ESPNOW_ENABLED（并注意与 wifi.cpp 的 setupWiFi 重名，
//   二者只能编译其一）。
//
// 边界情况与潜在风险:
//   - 单包上限 250 字节，而 MAVLINK_MAX_PACKET_LEN=280，故实现了 4 字节分片头
//     （magic 0xCF 0xA5 + total + index）做分片/重组；分片丢失会导致整帧丢弃。
//   - 接收回调运行在 WiFi 任务上下文，只做入队（rx_queue，深度 8）；
//     队列满时丢弃最旧帧，避免阻塞射频任务。
//   - 重组依赖"收到 index==total-1 即认为完整"，不校验是否收齐所有分片。
//   - 对端 MAC 学习后即注册为 peer，未做合法性校验（同一信道内任何设备都可被学习）。
//   - printWiFiInfo/configWiFi 在此模式下与 UDP 版本语义不同（ap/sta 配置无效）。

#include "globals.h"
#include "mavlink_compat.h"

#if WIFI_ENABLED && ESPNOW_ENABLED

#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_idf_version.h"
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#define ESPNOW_WIFI_CHANNEL 1              // 与地面站必须一致的 WiFi 信道
#define ESPNOW_MTU 250                     // ESP32/S3 ESP-NOW 单包最大 payload
#define ESPNOW_FRAG_HDR 4                  // 分片头：magic(2) + total(1) + index(1)
#define ESPNOW_FRAG_PAYLOAD (ESPNOW_MTU - ESPNOW_FRAG_HDR) // 246
#define ESPNOW_FRAG_MAGIC0 0xCF
#define ESPNOW_FRAG_MAGIC1 0xA5

static bool espnow_initialized = false;
// 目标 MAC：默认广播；首次收到包后学习为单播
static uint8_t peer_addr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static bool peer_known = false;

// 接收队列：完整 MAVLink 帧（字节流）入队，receiveWiFi() 非阻塞取走
static QueueHandle_t rx_queue = NULL;
typedef struct { uint8_t *data; int len; } rx_item_t;

// 分片重组缓冲
static uint8_t reasm_buf[MAVLINK_MAX_PACKET_LEN];
static int reasm_len = 0;
static int reasm_total = 0;

// 把一条完整帧入队（堆拷贝；队列满则丢弃最旧，避免阻塞主循环）
static void enqueue_frame(const uint8_t *data, int len) {
	if (len <= 0 || len > MAVLINK_MAX_PACKET_LEN) return;
	uint8_t *buf = (uint8_t *)malloc(len);
	if (!buf) return;
	memcpy(buf, data, len);
	rx_item_t item = { buf, len };
	if (rx_queue && xQueueSend(rx_queue, &item, 0) != pdTRUE) {
		rx_item_t drop;
		if (xQueueReceive(rx_queue, &drop, 0) == pdTRUE) free(drop.data);
		xQueueSend(rx_queue, &item, 0);
	}
}

// 新增/更新对端（ESP-NOW 发送前必须已 add_peer；重复添加返回错误属正常）
static void ensure_peer(const uint8_t *mac) {
	esp_now_peer_info_t peer = {};
	memcpy(peer.peer_addr, mac, 6);
	peer.channel = ESPNOW_WIFI_CHANNEL;
	peer.encrypt = false;
	esp_now_add_peer(&peer);
}

// 处理一帧收到的数据：分片则重组，否则整包即一条 MAVLink 帧
static void handle_incoming(const uint8_t *mac, const uint8_t *data, int len) {
	if (mac) {
		if (!peer_known || memcmp(peer_addr, mac, 6) != 0) {
			memcpy(peer_addr, mac, 6);
			ensure_peer(mac); // 学到单播地址后注册为对端，便于回传
		}
		peer_known = true;
	}

	// 分片帧：以 magic 开头
	if (len >= ESPNOW_FRAG_HDR && data[0] == ESPNOW_FRAG_MAGIC0 && data[1] == ESPNOW_FRAG_MAGIC1) {
		int total = data[2];
		int index = data[3];
		const uint8_t *payload = data + ESPNOW_FRAG_HDR;
		int plen = len - ESPNOW_FRAG_HDR;
		if (index == 0) { reasm_len = 0; reasm_total = total; }
		if (plen > 0 && reasm_len + plen <= MAVLINK_MAX_PACKET_LEN) {
			memcpy(reasm_buf + reasm_len, payload, plen);
			reasm_len += plen;
		}
		if (index == total - 1) { // 收到最后一片即认为重组完成
			enqueue_frame(reasm_buf, reasm_len);
			reasm_len = 0; reasm_total = 0;
		}
		return;
	}
	// 未分片：整包即为一条 MAVLink 帧（首字节 0xFE，不会与 magic 冲突）
	enqueue_frame(data, len);
}

// ESP-NOW 接收回调：IDF >= 4.4 使用 esp_now_recv_info_t 新签名，旧版用 mac 指针签名
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
	const uint8_t *mac = (info && info->src_addr) ? info->src_addr : NULL;
	handle_incoming(mac, data, len);
}
#else
static void espnow_recv_cb(const uint8_t *mac, const uint8_t *data, int len) {
	handle_incoming(mac, data, len);
}
#endif

void setupWiFi() {
	print("Setup ESP-NOW\n");

	esp_netif_init();
	esp_event_loop_create_default();

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	esp_wifi_init(&cfg);
	esp_wifi_set_mode(WIFI_MODE_STA);          // ESP-NOW 只需射频，不连路由器
	esp_wifi_start();
	esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

	esp_err_t ret = esp_now_init();
	if (ret != ESP_OK) {
		print("ESP-NOW init failed: %d\n", ret);
		return;
	}
	esp_now_register_recv_cb(espnow_recv_cb);

	ensure_peer(peer_addr); // 注册广播对端，未学习 MAC 前也能外发

	// 打印本机 MAC，方便地面站配置（广播无需此值；单播需填入）
	uint8_t self_mac[6];
	if (esp_wifi_get_mac(WIFI_IF_STA, self_mac) == ESP_OK) {
		print("ESP-NOW MAC: %02X:%02X:%02X:%02X:%02X:%02X (channel %d)\n",
		      self_mac[0], self_mac[1], self_mac[2], self_mac[3], self_mac[4], self_mac[5],
		      ESPNOW_WIFI_CHANNEL);
	}

	rx_queue = xQueueCreate(8, sizeof(rx_item_t));

	espnow_initialized = true;
	print("ESP-NOW ready\n");
}

// 发送一帧（长帧自动分片）
void sendWiFi(const uint8_t *buf, int len) {
	if (!espnow_initialized || len <= 0) return;

	if (len <= ESPNOW_FRAG_PAYLOAD) {
		esp_now_send(peer_addr, buf, len);
		return;
	}
	int total = (len + ESPNOW_FRAG_PAYLOAD - 1) / ESPNOW_FRAG_PAYLOAD;
	uint8_t pkt[ESPNOW_MTU];
	for (int i = 0; i < total; i++) {
		pkt[0] = ESPNOW_FRAG_MAGIC0;
		pkt[1] = ESPNOW_FRAG_MAGIC1;
		pkt[2] = (uint8_t)total;
		pkt[3] = (uint8_t)i;
		int off = i * ESPNOW_FRAG_PAYLOAD;
		int plen = (len - off < ESPNOW_FRAG_PAYLOAD) ? (len - off) : ESPNOW_FRAG_PAYLOAD;
		memcpy(pkt + ESPNOW_FRAG_HDR, buf + off, plen);
		esp_now_send(peer_addr, pkt, ESPNOW_FRAG_HDR + plen);
	}
}

// 非阻塞取走一帧收到的 MAVLink 字节流
int receiveWiFi(uint8_t *buf, int len) {
	if (!espnow_initialized || !rx_queue) return 0;
	rx_item_t item;
	if (xQueueReceive(rx_queue, &item, 0) == pdTRUE) {
		int n = (item.len < len) ? item.len : len;
		memcpy(buf, item.data, n);
		free(item.data);
		return n;
	}
	return 0;
}

void printWiFiInfo() {
	print("Mode: ESP-NOW (channel %d)\n", ESPNOW_WIFI_CHANNEL);
	print("Peer: %02X:%02X:%02X:%02X:%02X:%02X%s\n",
	      peer_addr[0], peer_addr[1], peer_addr[2], peer_addr[3], peer_addr[4], peer_addr[5],
	      peer_known ? " (learned)" : " (broadcast)");
	print("MAVLink connected: %d\n", mavlinkConnected);
}

// ESP-NOW 模式下 ap/sta 命令无意义
void configWiFi(bool ap, const char *ssid, const char *password) {
	print("ESP-NOW mode: ap/sta config not applicable\n");
}

#endif // WIFI_ENABLED && ESPNOW_ENABLED
