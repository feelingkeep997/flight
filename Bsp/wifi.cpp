// WiFi 通讯配置 - ESP-IDF 移植版
// 基于 esp_wifi + lwIP socket 的 WiFi 与 UDP 实现。
// 负责三件事：
//   1) 建立 WiFi 链路（AP 热点 或 STA 连接路由器）
//   2) 建立 UDP 套接字，作为 MAVLink 地面站通讯的收发通道（见 mavlink.cpp）
//   3) 提供 SSID/密码的配置与状态查询（CLI 命令 ap/sta/wifi）
// 编译开关 WIFI_ENABLED 来自 board_config.h；关闭时本文件整体不参与编译。
//
// 边界情况与潜在风险:
//   - UDP 无连接：默认向广播地址发送，收到地面站数据后才把对端学习为单播；
//     若地面站更换 IP，需等其再次发包才会重新定向。
//   - AP 模式下 sendWiFi() 始终广播；STA 模式下未连上路由器则直接放弃发送。
//   - ssid/pass 暂存缓冲为 64 字节，写入 esp_wifi 配置结构时用 strlcpy 按目标字段
//     实际容量截断（ap.ssid 只有 32 字节），因此超长 SSID 会被静默截短而不是越界。
//   - configWiFi() 只写 NVS，不支持热切换，需重启生效；且不校验长度，超长部分在
//     setupWiFi() 里才被截断。
//   - receiveWiFi() 为非阻塞（MSG_DONTWAIT），需由主循环高频调用。

#include "globals.h"

#if WIFI_ENABLED

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_err.h"
#include "lwip/sockets.h"
#include "lwip/ip4_addr.h"
#include "nvs_flash.h"
#include <string.h>

// NVS 读写接口（实现在 main/parameters.cpp）
// 注意：不要在这里 extern nvs_storage —— 它在 parameters.cpp 中是 static（内部链接），
//       声明一个 extern 只是"碰巧"没被引用所以链接期不报错，一旦有人用它就是未定义符号。
//       需要读写字符串请走下面这两个封装。
extern void nvsGetString(const char* key, char* out, size_t outLen, const char* defaultVal);
extern void nvsPutString(const char* key, const char* val);

// WiFi 工作模式：0=关闭 1=AP(自身发热点) 2=STA(连路由器)
// 可通过参数 WIFI_MODE 修改，修改后需重启生效
const int W_DISABLED = 0, W_AP = 1, W_STA = 2;
int wifiMode = W_AP;      // 默认 AP：无路由器环境下也能连接地面站
// AP 加密方式（参数 WIFI_AP_AUTH，重启生效）：0=开放热点（无密码），1=WPA2-PSK。
// 默认开放仅用于排查 iPhone 等客户端的 WPA2 握手兼容问题；
// 串口执行 `p WIFI_AP_AUTH 1` 后 reboot 即可恢复密码保护。
int wifiApAuth = 0;
int udpLocalPort = 14550; // 本地 UDP 监听端口（MAVLink 默认端口），参数 WIFI_LOC_PORT
int udpRemotePort = 14550;// 远端 UDP 目标端口（地面站端口），参数 WIFI_REM_PORT

static int udp_sock = -1;                        // UDP 套接字描述符，-1 表示未创建
static struct sockaddr_in udp_remote_addr;       // 当前通讯对端地址：随收到的数据包自动更新
static bool wifi_initialized = false;            // setupWiFi() 是否已成功完成（决定收发是否放行）
static bool wifi_connected = false;              // STA 模式下是否已获取 IP（仅置位，断开时不复位）
static esp_netif_t* ap_netif = NULL;             // AP 模式的网络接口
static esp_netif_t* sta_netif = NULL;            // STA 模式的网络接口

// 打印某个 ESP-IDF 调用的执行结果；ESP_ERR_INVALID_STATE 视为正常，
// 因为 esp_netif_init()/事件循环等属于幂等初始化，重复调用会返回该状态码。
static void printEspResult(const char* step, esp_err_t ret) {
	if (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE) {
		print("%s OK\n", step);
	} else {
		print("%s failed: %s (%d)\n", step, esp_err_to_name(ret), (int)ret);
	}
}

// WiFi/IP 事件回调：打印连接状态，并在 STA 启动后主动发起连接
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
	if (event_base == WIFI_EVENT) {
		if (event_id == WIFI_EVENT_AP_STACONNECTED) {
			print("WiFi: station connected\n");     // 有设备接入本机热点（AP 模式）
		} else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
			print("WiFi: station disconnected\n");
		} else if (event_id == WIFI_EVENT_STA_START) {
			esp_wifi_connect();                     // STA 启动后立即连接目标路由器（异步）
		}
	} else if (event_base == IP_EVENT) {
		if (event_id == IP_EVENT_STA_GOT_IP) {
			wifi_connected = true;
			print("WiFi: got IP\n");                // 获取到 IP 才算真正连通
		}
	}
}

// 初始化 WiFi 与 UDP 通讯（由 main.cpp 启动流程调用一次）
void setupWiFi() {
	print("Setup Wi-Fi\n");

	// ---- 初始化 TCP/IP 协议栈 ----
	// Initialize TCP/IP adapter
	printEspResult("esp_netif_init", esp_netif_init());

	// 创建默认事件循环（若已存在会返回 ESP_ERR_INVALID_STATE，属正常）
	// Create default event loop if not already created
	printEspResult("esp_event_loop_create_default", esp_event_loop_create_default());

	// ---- 按模式创建对应的网络接口 ----
	// Create WiFi interface
	if (wifiMode == W_AP) {
		ap_netif = esp_netif_create_default_wifi_ap();
		if (!ap_netif) print("WiFi AP netif create failed\n");
	} else if (wifiMode == W_STA) {
		sta_netif = esp_netif_create_default_wifi_sta();
		if (!sta_netif) print("WiFi STA netif create failed\n");
	}
	// 注：wifiMode == W_DISABLED 时两个分支都不执行，仅完成协议栈与事件初始化，
	//     不调用 esp_wifi_set_mode/start，因此 WiFi 不会真正启动。

	// ---- 初始化 WiFi 驱动并注册事件回调 ----
	// Init WiFi with default config
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	printEspResult("esp_wifi_init", esp_wifi_init(&cfg));

	// Register event handlers（WIFI_EVENT 收全部事件，IP_EVENT 只关心拿到 IP）
	printEspResult("wifi_event_register", esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
	printEspResult("ip_event_register", esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

	// 凭证暂存区：esp_wifi 中 ssid 字段上限 32 字节、password 上限 64 字节。
	// 注意：下面用 strcpy 拷入配置结构，若 NVS 中存有超过上限的名称会越界，慎用超长 SSID。
	char ssid[64], pass[64];

	if (wifiMode == W_AP) {
		// ---- AP 模式：从 NVS 读取热点名称/密码，首次上电使用默认值 ----
		// SSID/密码都改用新的 NVS 键：旧固件可能已把历史凭证写进 WIFI_AP_SSID /
		// WIFI_AP_PASS，若继续读旧键，默认值永远不会生效（表现为热点名改不掉或
		// 默认密码提示错误）。换键后旧值自然失效，之后 CLI 的
		// `ap <ssid> <password>` 仍可正常修改并保存。
		nvsGetString("WIFI_AP_SSID2", ssid, sizeof(ssid), "feelingkeep");
		nvsGetString("WIFI_AP_PASS2", pass, sizeof(pass), "12345678");

		wifi_config_t wifi_config = {};
		// 用 strlcpy 而不是 strcpy：esp_wifi 里 ap.ssid 只有 32 字节、ap.password 64 字节，
		// 而 CLI 的 `ap <ssid> <password>` 命令不校验长度，NVS 里可能存着超长字符串，
		// strcpy 会直接踩穿 wifi_config_t 里后面的字段（含 authmode/pmf_cfg），
		// 表现为"热点起不来"或更隐蔽的配置错乱。
		strlcpy((char*)wifi_config.ap.ssid, ssid, sizeof(wifi_config.ap.ssid));
		wifi_config.ap.ssid_len = strlen((char*)wifi_config.ap.ssid);   // 显式给出 SSID 长度，兼容含 0x00 结尾的名字
		wifi_config.ap.channel = 1;               // 固定信道 1，便于地面站快速发现
		wifi_config.ap.max_connection = 4;        // 最多允许 4 个客户端接入
		if (wifiApAuth == 0) {
			// 开放热点：authmode 必须为 OPEN 且密码为空，否则 esp_wifi_set_config 会拒绝
			wifi_config.ap.authmode = WIFI_AUTH_OPEN;
			wifi_config.ap.password[0] = '\0';
		} else {
			wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
			strlcpy((char*)wifi_config.ap.password, pass, sizeof(wifi_config.ap.password));
		}
		wifi_config.ap.pmf_cfg.required = false;  // 关闭 PMF 强制要求，兼容旧设备连接

		// 固定本机 IP 为 192.168.4.1：DHCP 服务先停再改地址，改完重新启动
		if (ap_netif) {
			esp_netif_ip_info_t ip_info = {};
			IP4_ADDR(&ip_info.ip, 192, 168, 4, 1);
			IP4_ADDR(&ip_info.gw, 192, 168, 4, 1);
			IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
			printEspResult("AP DHCP stop", esp_netif_dhcps_stop(ap_netif));
			printEspResult("AP set IP 192.168.4.1", esp_netif_set_ip_info(ap_netif, &ip_info));
			printEspResult("AP DHCP start", esp_netif_dhcps_start(ap_netif));
		}

		printEspResult("esp_wifi_set_mode(AP)", esp_wifi_set_mode(WIFI_MODE_AP));
		printEspResult("esp_wifi_set_config(AP)", esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
		// 关闭省电模式：避免 sleep 造成遥测丢包与遥控延迟
		printEspResult("esp_wifi_set_ps(NONE)", esp_wifi_set_ps(WIFI_PS_NONE));
		printEspResult("esp_wifi_start", esp_wifi_start());
		print("WiFi AP started: %s (%s)\n", ssid,
			wifiApAuth == 0 ? "OPEN, no password" : "WPA2-PSK");
	} else if (wifiMode == W_STA) {
		// ---- STA 模式：连接外部路由器，凭证来自 NVS（默认空 = 无凭证） ----
		nvsGetString("WIFI_STA_SSID", ssid, sizeof(ssid), "");
		nvsGetString("WIFI_STA_PASS", pass, sizeof(pass), "");

		wifi_config_t wifi_config = {};
		// 同 AP 分支：strlcpy 防超长 SSID/密码越界。
		// threshold.authmode 保持零初始化(= WIFI_AUTH_OPEN)，否则 STA 会拒绝连接开放热点。
		strlcpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
		strlcpy((char*)wifi_config.sta.password, pass, sizeof(wifi_config.sta.password));

		printEspResult("esp_wifi_set_mode(STA)", esp_wifi_set_mode(WIFI_MODE_STA));
		printEspResult("esp_wifi_set_config(STA)", esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
		printEspResult("esp_wifi_start", esp_wifi_start()); // 启动后由 STA_START 事件触发连接
		print("WiFi STA connecting to: %s\n", ssid);
	}

	// ---- 创建 UDP 套接字（AP/STA 共用同一条收发通道）----
	// Setup UDP socket
	udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (udp_sock < 0) {
		print("UDP socket creation failed\n");
		return; // 套接字创建失败，保持 wifi_initialized=false，收发接口将直接返回
	}

	// 绑定本机所有网卡的 udpLocalPort，用于接收地面站指令
	struct sockaddr_in local_addr = {};
	local_addr.sin_family = AF_INET;
	local_addr.sin_port = htons(udpLocalPort);
	local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	if (bind(udp_sock, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
		print("UDP bind failed\n");
	}

	// 默认远端口标为广播地址：地面站在未知 IP 时也能收到遥测；
	// 一旦收到地面站数据包，对端地址会被更新为实际来源（见 receiveWiFi）
	// Default broadcast remote
	udp_remote_addr.sin_family = AF_INET;
	udp_remote_addr.sin_port = htons(udpRemotePort);
	udp_remote_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);

	// 允许向广播地址发送
	// Enable broadcast
	int broadcastEnable = 1;
	setsockopt(udp_sock, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable));

	wifi_initialized = true;
}

// 通过 UDP 发送数据（遥测上行），由 mavlink.cpp 的 sendMavlink() 调用
void sendWiFi(const uint8_t *buf, int len) {
	if (!wifi_initialized || udp_sock < 0) return;

	// STA 模式下未连上路由器时无链路可用，直接放弃发送
	// Check if we have clients (AP mode) or are connected (STA mode)
	wifi_ap_record_t ap_info;
	bool has_link = (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK); // 仅 STA 模式有意义

	wifi_mode_t mode;
	esp_wifi_get_mode(&mode);
	if (mode == WIFI_MODE_AP) {
		esp_netif_ip_info_t ip_info;
		esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_AP_DEF"), &ip_info);
		// AP 模式下始终使用广播发送（ip_info 仅作调试参考，未参与判断）
	} else if (!has_link) {
		return; // STA 未连接路由器：放弃本次发送
	}

	sendto(udp_sock, buf, len, 0, (struct sockaddr*)&udp_remote_addr, sizeof(udp_remote_addr));
}

// 非阻塞接收 UDP 数据（指令下行），返回收到的字节数，无数据返回 0
int receiveWiFi(uint8_t *buf, int len) {
	if (!wifi_initialized || udp_sock < 0) return 0;

	struct sockaddr_in src_addr;
	socklen_t addr_len = sizeof(src_addr);
	// MSG_DONTWAIT：无数据立即返回 -1，不阻塞主控制循环
	int received = recvfrom(udp_sock, buf, len, MSG_DONTWAIT, (struct sockaddr*)&src_addr, &addr_len);
	if (received > 0) {
		// 学习对端地址：把遥测/回复定向到最近发来数据的地面站，而非一直广播
		// Update remote IP
		udp_remote_addr.sin_addr = src_addr.sin_addr;
		udp_remote_addr.sin_port = src_addr.sin_port;
	}
	return received > 0 ? received : 0;
}

// 打印当前 WiFi 状态（CLI 命令 wifi）；注意打印的 SSID 等取自驱动实际配置
void printWiFiInfo() {
	wifi_mode_t mode;
	esp_wifi_get_mode(&mode);

	if (mode == WIFI_MODE_AP) {
		print("Mode: Access Point (AP)\n");
		wifi_config_t conf;
		esp_wifi_get_config(WIFI_IF_AP, &conf);
		print("SSID: %s\n", conf.ap.ssid);
		print("Security: %s\n", conf.ap.authmode == WIFI_AUTH_OPEN ? "Open (no password)" : "WPA2-PSK");
		if (conf.ap.authmode == WIFI_AUTH_OPEN) {
			print("Password: (none)\n");
		} else {
			// 仅本地串口可见；用方括号包住明文，便于发现尾随空格等不可见字符
			print("Password: [%s]\n", conf.ap.password);
		}
		wifi_sta_list_t sta_list;
		esp_wifi_ap_get_sta_list(&sta_list);
		print("Clients: %d\n", sta_list.num); // 当前接入的客户端数量

		esp_netif_ip_info_t ip_info;
		esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_AP_DEF"), &ip_info);
		print("IP: " IPSTR "\n", IP2STR(&ip_info.ip));
	} else if (mode == WIFI_MODE_STA) {
		print("Mode: Client (STA)\n");
		print("Connected: %d\n", wifi_connected); // 是否已从路由器获取 IP
	} else {
		print("Mode: Disabled\n");
		return; // 未启用时不打印后续通讯状态
	}
	print("MAVLink connected: %d\n", mavlinkConnected); // 是否已收到过地面站数据
}

// 去掉字符串两端空白：串口终端/复制粘贴可能在命令末尾带入空格，
// 若原样存进 NVS，手机按可见字符输入就会一直提示"密码错误"。
static void trimWhitespace(char* s) {
	char* start = s;
	while (*start == ' ' || *start == '\t') start++;
	if (start != s) memmove(s, start, strlen(start) + 1);
	size_t len = strlen(s);
	while (len > 0 && (s[len - 1] == ' ' || s[len - 1] == '\t')) s[--len] = '\0';
}

// 保存 WiFi 凭证到 NVS（CLI 命令 ap/sta），重启后生效
void configWiFi(bool ap, const char *ssid, const char *password) {
	char s[64], p[64];
	strlcpy(s, ssid ? ssid : "", sizeof(s));
	strlcpy(p, password ? password : "", sizeof(p));
	trimWhitespace(s);
	trimWhitespace(p);

	if (ap) {
		// 先校验再写：避免把空 SSID 或非法密码写进 NVS，导致下次开机热点起不来
		if (s[0] == '\0') {
			print("SSID 不能为空，未保存\n");
			return;
		}
		if (strlen(p) < 8 || strlen(p) > 63) {
			print("AP 密码长度须为 8~63 字符，未保存\n");
			return;
		}
		nvsPutString("WIFI_AP_SSID2", s);
		nvsPutString("WIFI_AP_PASS2", p);
	} else {
		nvsPutString("WIFI_STA_SSID", s);
		nvsPutString("WIFI_STA_PASS", p);
	}
	// 运行中的 WiFi 配置不便热切换，改为提示用户重启
	print("✓ 重启后生效 Reboot to apply\n");
}

#endif // WIFI_ENABLED
