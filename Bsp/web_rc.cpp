// ============================================================================
// web_rc.cpp —— 网页遥控与网页控制台（esp_http_server）
//
// 职责:
//   在设备上跑一个 HTTP 服务器（默认 80 端口），提供：
//     - 网页遥控页面（GET / 等，内容为 web_rc_html.h 中内嵌的 HTML/JS）；
//     - 遥控数据接口 POST /web_rc（JSON：t=1 杆量、t=2 按钮、t=4 心跳）；
//     - 状态查询 GET /web_rc/status（电压、模式、各轴输出）；
//     - 网页控制台 GET /console、POST /console/cmd|enable|disable。
//   手机连上设备的 AP 热点后即可在浏览器里操作飞行器，并查看串口日志。
//
// 关键逻辑:
//   - 遥控数据以 JSON 字符串手工解析（findJsonValue），不走 JSON 库；
//   - 杆量经 死区 → 归一化 → 缩放 后写入全局 controlRoll/Pitch/Yaw/Throttle，
//     并刷新 webRCLastUpdate；isWebRCEnabled() 以 WEB_RC_ONLINE_TIMEOUT_MS(8s，
//     与 safety.cpp 的失联保护共用) 判断"遥控是否在线"；
//   - 按钮用位掩码 webRCButtons 表达，interpretWebRC()（control.cpp）处理上升沿；
//   - 控制台日志写入环形缓冲 consoleBuf，命令进环形队列 consoleCmdQueue，
//     由主循环的 processConsoleCommandQueue() 取出执行（避免在 HTTP 任务里跑控制逻辑）。
//
// 输入/输出:
//   输入：HTTP 请求（JSON body / query）；输出：HTTP 响应（HTML/JSON）+ 对全局控制量的写入。
//
// 重要参数:
//   WEB_RC_ONLINE_TIMEOUT_MS(8000，见 globals.h，与失联保护共用) 遥控在线超时；
//   webRCStickScale(0.85)/webRCYawScale(0.68) 杆量缩放；
//   stickDeadzone/throttleDeadzone(0.06)；CONSOLE_* 控制台缓冲与队列容量。
//
// 边界情况与潜在风险:
//   - JSON 解析是"字符串查找 + atof"，格式稍有偏差就会解析失败或得到 0（非严格校验）；
//   - HTTP 任务与主控制循环在不同任务中并发访问全局变量（controlRoll 等），
//     无锁保护，属"最后写入者生效"，极端时序下可能出现半新半旧的一帧；
//   - 未做任何鉴权：同一热点内任何设备都能发指令，仅适合本地调试/演示场景；
//   - max_open_sockets=4 且超时被调小，并发请求多时可能拒绝连接（已启用 LRU 回收）；
//   - 控制台命令经队列（容量 4）转交主循环，队列满时 HTTP 返回 500；
//   - 仅在 WEB_RC_ENABLED 时编译；关闭时 setupWebRC 只打印一行提示。
// ============================================================================

#include "globals.h"

#if WEB_RC_ENABLED

#include "esp_http_server.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "cf_math.h"
#include "web_rc_html.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// 在线判定超时统一取 globals.h 的 WEB_RC_ONLINE_TIMEOUT_MS，
// 与 safety.cpp 的 webRCLossFailsafe() 共用同一阈值（详见 globals.h 注释）。

bool webRCEnabled    = false;
bool useWebRC        = false;
bool webRCUpdated    = false;
bool webConsoleEnabled = false;
char webRCWarnMsg[64] = "";
uint16_t webRCButtons    = 0;
unsigned long webRCLastUpdate = 0;

float webRCRoll     = 0.0f;
float webRCPitch    = 0.0f;
float webRCYaw      = 0.0f;
float webRCThrottle = 0.0f;

static float webRCThrottleScale = 1.0f;
static float webRCStickScale    = 0.85f;
static float webRCYawScale      = 0.68f;
static float stickDeadzone    = 0.06f;
static float throttleDeadzone = 0.06f;

static const float THROTTLE_MAX = 100.0f;
static const float STICK_MAX    = 30.0f;
static const float RAW_MAX      = 100.0f;

static float lastValidThrottle = 0.0f;
static float lastValidRoll     = 0.0f;
static float lastValidPitch    = 0.0f;
static float lastValidYaw      = 0.0f;

static int lastProcType = 0;
static int lastProcButtonIdx = -1;
static int lastProcButtonState = -1;
static unsigned long lastWebRCRequestLog = 0;

// Console buffer
#define CONSOLE_LINES    BOARD_CONSOLE_LINES
#define CONSOLE_LINE_LEN BOARD_CONSOLE_LINE_LEN
static char consoleBuf[CONSOLE_LINES][CONSOLE_LINE_LEN];
static int  consoleTail   = 0;
static int  consoleFilled = 0;
static int  consoleTotal  = 0;

#define CONSOLE_CMD_QUEUE_SIZE 4
#define CONSOLE_CMD_LEN 64
static char consoleCmdQueue[CONSOLE_CMD_QUEUE_SIZE][CONSOLE_CMD_LEN];
static int  consoleCmdHead  = 0;
static int  consoleCmdTail  = 0;
static int  consoleCmdCount = 0;

static httpd_handle_t server = NULL;

static unsigned long now_ms() { return (unsigned long)(esp_timer_get_time() / 1000); }

static const char* httpMethodName(int method) {
	switch (method) {
		case HTTP_GET: return "GET";
		case HTTP_POST: return "POST";
		case HTTP_PUT: return "PUT";
		case HTTP_DELETE: return "DELETE";
		default: return "?";
	}
}

static void logHttpRequest(httpd_req_t *req, const char* tag) {
	char host[64] = "-";
	char peer[48] = "?";
	httpd_req_get_hdr_value_str(req, "Host", host, sizeof(host));

	int sockfd = httpd_req_to_sockfd(req);
	struct sockaddr_storage source_addr = {};
	socklen_t addr_len = sizeof(source_addr);
	if (sockfd >= 0 && getpeername(sockfd, (struct sockaddr*)&source_addr, &addr_len) == 0) {
		if (source_addr.ss_family == AF_INET) {
			struct sockaddr_in* addr = (struct sockaddr_in*)&source_addr;
			inet_ntoa_r(addr->sin_addr, peer, sizeof(peer));
		}
	}

	print("HTTP %s %s %s from %s host=%s\n", tag, httpMethodName(req->method), req->uri, peer, host);
}

static void logEspResult(const char* step, esp_err_t ret) {
	if (ret == ESP_OK) {
		print("%s OK\n", step);
	} else {
		print("%s failed: %s (%d)\n", step, esp_err_to_name(ret), (int)ret);
	}
}

static esp_err_t sendWebRCPage(httpd_req_t *req) {
	httpd_resp_set_type(req, "text/html");
	httpd_resp_set_hdr(req, "Cache-Control", "no-store");

	const char* html = webRCIndexHtml;
	size_t remaining = strlen(html);
	size_t offset = 0;
	while (remaining > 0) {
		size_t chunk = remaining > 1024 ? 1024 : remaining;
		esp_err_t ret = httpd_resp_send_chunk(req, html + offset, chunk);
		if (ret != ESP_OK) {
			print("HTTP page send failed at %u/%u: %s (%d)\n",
				(unsigned)offset, (unsigned)(offset + remaining), esp_err_to_name(ret), (int)ret);
			return ret;
		}
		offset += chunk;
		remaining -= chunk;
	}

	esp_err_t ret = httpd_resp_send_chunk(req, NULL, 0);
	if (ret != ESP_OK) {
		print("HTTP page send finish failed: %s (%d)\n", esp_err_to_name(ret), (int)ret);
	}
	return ret;
}

// 把一条控制台命令放入环形队列（由 HTTP 任务调用）。
// 队列满时返回 false（调用方据此回 500）。命令会被主循环的
// processConsoleCommandQueue() 取出执行，避免在 HTTP 任务里跑控制逻辑。
bool enqueueConsoleCmd(const char* cmd) {
	if (!cmd || !*cmd) return false;
	if (consoleCmdCount >= CONSOLE_CMD_QUEUE_SIZE) return false;
	strncpy(consoleCmdQueue[consoleCmdTail], cmd, CONSOLE_CMD_LEN - 1);
	consoleCmdQueue[consoleCmdTail][CONSOLE_CMD_LEN - 1] = '\0';
	consoleCmdTail = (consoleCmdTail + 1) % CONSOLE_CMD_QUEUE_SIZE;
	consoleCmdCount++;
	return true;
}

// 从队列取出一条命令并执行（由主循环每周期调用）。一次只处理一条，避免长时间占用主循环。
void processConsoleCommandQueue() {
	if (consoleCmdCount <= 0) return;
	char cmd[CONSOLE_CMD_LEN];
	strncpy(cmd, consoleCmdQueue[consoleCmdHead], CONSOLE_CMD_LEN);
	consoleCmdHead = (consoleCmdHead + 1) % CONSOLE_CMD_QUEUE_SIZE;
	consoleCmdCount--;
	doCommand(cmd, false);
}

// 把一段文本按行写入控制台环形缓冲（供网页控制台轮询读取）。
// 每行截断到 CONSOLE_LINE_LEN-1 字节；缓冲写满后覆盖最旧行。
void webLog(const char* msg) {
	const char* start = msg;
	while (*start) {
		const char* end = strchr(start, '\n');
		int len = end ? (int)(end - start) : (int)strlen(start);
		if (len > 0) {
			int copy = (len < CONSOLE_LINE_LEN - 1) ? len : (CONSOLE_LINE_LEN - 1);
			strncpy(consoleBuf[consoleTail], start, copy);
			consoleBuf[consoleTail][copy] = '\0';
			consoleTail = (consoleTail + 1) % CONSOLE_LINES;
			if (consoleFilled < CONSOLE_LINES) consoleFilled++;
			consoleTotal++;
		}
		if (!end) break;
		start = end + 1;
	}
}

// 摇杆死区处理：|norm| < deadzone 视为 0；否则把 [deadzone,1] 重新线性拉伸到 [0,1]，
// 保证死区外仍能输出满量程。
static float applyDeadzone(float norm, float deadzone) {
	if (fabsf(norm) < deadzone) return 0.0f;
	float sign = (norm > 0.0f) ? 1.0f : -1.0f;
	return sign * (fabsf(norm) - deadzone) / (1.0f - deadzone);
}

// 处理一个摇杆轴（横滚/俯仰/偏航）：非法值回退到上次有效值，其余做归一化+死区。
// 返回值为 -STICK_MAX ~ +STICK_MAX 区间的"百分制"中间量（后续再缩放成控制量）。
// lastValid 为引用，用于在收到异常值时保持上一帧输出，避免突变。
static float processAxis(float raw, float& lastValid) {
	if (isnan(raw) || isinf(raw) || fabsf(raw) > 1000.0f) return lastValid; // 非法值保护
	float norm = std::clamp(raw, -RAW_MAX, RAW_MAX) / RAW_MAX;             // 归一化到 [-1,1]
	norm = applyDeadzone(norm, stickDeadzone);
	lastValid = norm * STICK_MAX;
	return lastValid;
}

// 处理油门轴：原始 [-100,100] 先映射到 [0,100]，再过死区与缩放。
// 非法值同样回退到上次有效值。返回 0 ~ THROTTLE_MAX 的百分制油门。
static float processThrottle(float raw) {
	if (isnan(raw) || isinf(raw) || fabsf(raw) > 1000.0f) return lastValidThrottle;
	raw = std::clamp(raw, -RAW_MAX, RAW_MAX);
	float pct = (raw + RAW_MAX) / (2.0f * RAW_MAX) * THROTTLE_MAX;
	if (pct < throttleDeadzone * THROTTLE_MAX) pct = 0.0f; // 油门低位死区（可真正收油到 0）
	pct = std::clamp(pct * webRCThrottleScale, 0.0f, THROTTLE_MAX);
	lastValidThrottle = pct;
	return pct;
}

// 处理一条遥控 JSON 报文并更新控制量。
// 输入：roll/pitch/yaw（原始 -100~100）、throttle（原始 -100~100）、buttons（按钮位掩码）
// 处理链：油门 死区+缩放 → 各轴 死区+归一化+缩放 → 写入全局控制量
// 副作用：更新 webRCThrottle/Roll/Pitch/Yaw/Buttons、webRCLastUpdate、webRCUpdated，
//         并写 controlRoll/Pitch/Yaw/Throttle、controlMode(NAN)、controlTime。
static void setWebRCInput(float roll, float pitch, float yaw, float throttle, uint16_t buttons) {
	float pThrottle = processThrottle(throttle);
	float pYaw      = processAxis(yaw,   lastValidYaw);
	float pPitch    = processAxis(pitch, lastValidPitch);
	float pRoll     = processAxis(roll,  lastValidRoll);

	webRCThrottle = pThrottle;
	webRCYaw      = pYaw;
	webRCPitch    = pPitch;
	webRCRoll     = pRoll;
	webRCButtons  = buttons;
	webRCLastUpdate = now_ms();
	webRCUpdated  = true;

	// 缩放并钳位到飞控输入范围：横滚/俯仰用 STICK_MAX，偏航另有缩放
	controlRoll     = std::clamp(pRoll  * webRCStickScale / STICK_MAX, -1.0f, 1.0f);
	controlPitch    = std::clamp(pPitch * webRCStickScale / STICK_MAX, -1.0f, 1.0f);
	controlYaw      = std::clamp(pYaw   * webRCYawScale   / STICK_MAX, -1.0f, 1.0f);
	controlThrottle = pThrottle / THROTTLE_MAX;
	controlMode     = NAN; // 网页遥控不提供模式通道，置 NAN 交由按钮切换
	controlTime     = t;
}

static const char* findJsonValue(const char* json, const char* key) {
	const char* p = strstr(json, key);
	if (!p) return NULL;
	p = strchr(p, ':');
	if (!p) return NULL;
	p++;
	while (*p == ' ' || *p == '"') p++;
	return p;
}

// 解析网页遥控的 JSON 报文并分发。
// 协议（精简 JSON，手工解析）：
//   t=1 杆量：{ "t":1, "th":油门, "r":横滚, "p":俯仰, "y":偏航 }
//   t=2 按钮：{ "t":2, "b":按钮下标, "s":按下/松开 }（按下置位、松开清位）
//   t=4 心跳：{ "t":4 }（仅刷新在线时间戳）
// 返回 true 表示识别到合法报文（不代表数值一定有效）。
static bool handleJSONProtocol(const char* body) {
	if (!body || !*body || !strstr(body, "{")) return false;
	const char* typePos = strstr(body, "\"t\":");
	if (!typePos) return false;
	int type = atoi(typePos + 4);
	const char* v;

	lastProcType = type;
	lastProcButtonIdx = -1;
	lastProcButtonState = -1;

	switch (type) {
		case 1: {
			float th = 0, r = 0, p = 0, y = 0;
			if ((v = findJsonValue(body, "\"th\""))) th = atof(v);
			if ((v = findJsonValue(body, "\"r\"")))  r  = atof(v);
			if ((v = findJsonValue(body, "\"p\"")))  p  = atof(v);
			if ((v = findJsonValue(body, "\"y\"")))  y  = atof(v);
			setWebRCInput(r, p, y, th, webRCButtons);
			break;
		}
		case 2: {
			int idx = 0, state = 0;
			if ((v = findJsonValue(body, "\"b\"")))  idx   = atoi(v);
			if ((v = findJsonValue(body, "\"s\"")))  state = atoi(v);
			if (idx >= 0 && idx < 16) {
				if (state) webRCButtons |=  (1 << idx);
				else       webRCButtons &= ~(1 << idx);
				webRCLastUpdate = now_ms();
				webRCUpdated = true;
				lastProcButtonIdx = idx;
				lastProcButtonState = state;
			}
			break;
		}
		case 4:
			webRCLastUpdate = now_ms();
			webRCUpdated = true;
			break;
	}
	return true;
}

void setWebRCWarn(const char* msg) {
	strncpy(webRCWarnMsg, msg, sizeof(webRCWarnMsg) - 1);
	webRCWarnMsg[sizeof(webRCWarnMsg) - 1] = '\0';
}

// 网页遥控是否"在线"：最近 WEB_RC_ONLINE_TIMEOUT_MS(8s) 内收到过任何遥控/心跳报文。
// 该阈值与 safety.cpp 的 webRCLossFailsafe() 完全一致，保证"判定失联"与"触发下降"同时发生。
// 注意：返回 true 不代表控制量一定有效（可能只是心跳），有效性由调用方结合场景判断。
bool isWebRCEnabled() {
	return webRCUpdated && (now_ms() - webRCLastUpdate < WEB_RC_ONLINE_TIMEOUT_MS);
}

// 网页遥控是否正在实际接管控制（已启用开关 且 在线）
bool isUsingWebRC() {
	return useWebRC && isWebRCEnabled();
}

// HTTP handler: GET / - serve HTML
static esp_err_t handler_root(httpd_req_t *req) {
	logHttpRequest(req, "page");
	return sendWebRCPage(req);
}

// HTTP 处理：POST /web_rc 与 /web_rc/heartbeat
// 流程：读取 body → handleJSONProtocol 解析 → 返回状态 JSON（含 mode/armed 及可选告警）。
// 日志限速 2s，避免高频遥控请求把串口/网页控制台刷爆。
static esp_err_t handler_web_rc(httpd_req_t *req) {
	unsigned long now = now_ms();
	if (now - lastWebRCRequestLog > 2000) {
		logHttpRequest(req, "api");
		lastWebRCRequestLog = now;
	}

	char body[512];
	int received = httpd_req_recv(req, body, sizeof(body) - 1);
	if (received <= 0) {
		httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "{\"e\":\"no data\"}");
		return ESP_FAIL;
	}
	body[received] = '\0';

	if (handleJSONProtocol(body)) {
		char resp[320];
		bool deliverWarn = (lastProcType == 2 || lastProcType == 4) && webRCWarnMsg[0];
		if (deliverWarn) {
			if (lastProcButtonIdx >= 0) {
				snprintf(resp, sizeof(resp),
					"{\"s\":\"ok\",\"m\":%d,\"arm\":%d,\"rt\":%d,\"bi\":%d,\"bs\":%d,\"warn\":\"%s\"}",
					mode, (int)armed, lastProcType, lastProcButtonIdx, lastProcButtonState, webRCWarnMsg);
			} else {
				snprintf(resp, sizeof(resp),
					"{\"s\":\"ok\",\"m\":%d,\"arm\":%d,\"rt\":%d,\"warn\":\"%s\"}",
					mode, (int)armed, lastProcType, webRCWarnMsg);
			}
			webRCWarnMsg[0] = '\0';
		} else {
			if (lastProcButtonIdx >= 0) {
				snprintf(resp, sizeof(resp),
					"{\"s\":\"ok\",\"m\":%d,\"arm\":%d,\"rt\":%d,\"bi\":%d,\"bs\":%d}",
					mode, (int)armed, lastProcType, lastProcButtonIdx, lastProcButtonState);
			} else {
				snprintf(resp, sizeof(resp),
					"{\"s\":\"ok\",\"m\":%d,\"arm\":%d,\"rt\":%d}",
					mode, (int)armed, lastProcType);
			}
		}
		httpd_resp_set_type(req, "application/json");
		httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
	} else {
		httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "{\"e\":\"parse failed\"}");
	}
	return ESP_OK;
}

// HTTP handler: GET /web_rc/status
static esp_err_t handler_status(httpd_req_t *req) {
	// 高频状态轮询不打日志，避免 getpeername 系统调用开销拖慢 httpd 任务
	float vbat = readBatteryVoltage();
	if (isnan(vbat) || vbat < 0.0f) vbat = 0.0f;
	char json[384];
	snprintf(json, sizeof(json),
		"{\"enabled\":%s,\"active\":%s,\"voltage\":%.2f,\"throttle\":%.1f,\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f}",
		isWebRCEnabled() ? "true" : "false",
		isUsingWebRC()   ? "true" : "false",
		vbat, webRCThrottle, webRCRoll, webRCPitch, webRCYaw);
	httpd_resp_set_type(req, "application/json");
	httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
	return ESP_OK;
}

// HTTP handler: GET /console
//
// 【缓冲区安全】此处的 JSON 是手工拼装的，历史上出现过栈溢出：
//   旧写法用 `pos += snprintf(json + pos, sizeof(json) - pos, ...)`，而 snprintf
//   返回的是"若不截断本应写入的长度"，一旦截断 pos 就会超过 sizeof(json)；
//   随后 `sizeof(json) - pos` 以 size_t 计算发生回绕，变成一个巨大的长度参数，
//   下一次 snprintf 直接写穿栈（httpd 任务栈溢出 → 复位或更糟）。
//   现在统一走 jsonPut/jsonPutStr 两个带钳位的追加函数：pos 永不超过 cap，
//   且 cap = sizeof(json)-1 始终给结尾 '\0' 留位。
static esp_err_t handler_console_get(httpd_req_t *req) {
	// 高频控制台轮询不打日志，避免 getpeername 开销
	char query[64] = "";
	httpd_req_get_url_query_str(req, query, sizeof(query));
	int since = -1, limit = 20;
	char param[16];
	if (httpd_query_key_value(query, "since", param, sizeof(param)) == ESP_OK) since = atoi(param);
	if (httpd_query_key_value(query, "limit", param, sizeof(param)) == ESP_OK) limit = atoi(param);
	if (limit <= 0) limit = 20;
	if (limit > CONSOLE_LINES) limit = CONSOLE_LINES;

	int availableFrom = (consoleTotal > consoleFilled) ? (consoleTotal - consoleFilled) : 0;
	int sendFrom = (since >= 0) ? since : availableFrom;
	if (sendFrom < availableFrom) sendFrom = availableFrom;
	if (sendFrom > consoleTotal) sendFrom = consoleTotal;
	int sendTo = (consoleTotal < sendFrom + limit) ? consoleTotal : (sendFrom + limit);

	char json[4096];
	const int cap = (int)sizeof(json) - 1; // 预留 1 字节给结尾 '\0'
	int pos = 0;

	// 追加单个字符：越界即丢弃（不会写穿缓冲区）
	auto jsonPut = [&](char c) { if (pos < cap) json[pos++] = c; };
	// 追加字符串：越界即截断
	auto jsonPutStr = [&](const char* s) { while (*s && pos < cap) json[pos++] = *s++; };

	char head[96];
	snprintf(head, sizeof(head), "{\"total\":%d,\"next\":%d,\"has_more\":%s,\"lines\":[",
		consoleTotal, sendTo, (sendTo < consoleTotal) ? "true" : "false");
	jsonPutStr(head);

	bool first = true;
	for (int i = sendFrom; i < sendTo; i++) {
		int idx = i % CONSOLE_LINES;
		if (!first) jsonPut(',');
		first = false;
		jsonPut('"');
		// 转义 JSON 特殊字符；留 2 字节余量，保证最长转义序列 "\\\\" 不会把
		// 结尾的引号/中括号挤出缓冲区
		for (int c = 0; consoleBuf[idx][c] && pos < cap - 2; c++) {
			char ch = consoleBuf[idx][c];
			if (ch == '\\')      { jsonPut('\\'); jsonPut('\\'); }
			else if (ch == '"')  { jsonPut('\\'); jsonPut('"'); }
			else if (ch == '\n') { jsonPut('\\'); jsonPut('n'); }
			else if (ch == '\r') { jsonPut('\\'); jsonPut('r'); }
			else if (ch == '\t') { jsonPut('\\'); jsonPut('t'); }
			else if ((unsigned char)ch < 0x20) { /* 其余控制字符直接丢弃，避免产出非法 JSON */ }
			else jsonPut(ch);
		}
		jsonPut('"');
	}
	jsonPut(']');
	jsonPut('}');
	json[pos] = '\0';

	httpd_resp_set_type(req, "application/json");
	httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
	return ESP_OK;
}

// HTTP handler: POST /console/cmd
static esp_err_t handler_console_cmd(httpd_req_t *req) {
	logHttpRequest(req, "console-cmd");
	char body[CONSOLE_CMD_LEN + 32];
	int received = httpd_req_recv(req, body, sizeof(body) - 1);
	if (received <= 0) {
		httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "{\"ok\":0,\"e\":\"empty\"}");
		return ESP_FAIL;
	}
	body[received] = '\0';
	// Trim
	char* p = body;
	while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
	if (*p == '\0') {
		httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "{\"ok\":0,\"e\":\"empty command\"}");
		return ESP_FAIL;
	}
	char logbuf[CONSOLE_CMD_LEN + 64];
	snprintf(logbuf, sizeof(logbuf), "> %.*s", (int)(sizeof(logbuf) - 3), p);
	webLog(logbuf);
	if (!enqueueConsoleCmd(p)) {
		webLog("! command queue is full");
		httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "{\"ok\":0,\"e\":\"queue full\"}");
		return ESP_FAIL;
	}
	httpd_resp_set_type(req, "application/json");
	httpd_resp_send(req, "{\"ok\":1,\"queued\":1}", HTTPD_RESP_USE_STRLEN);
	return ESP_OK;
}

// HTTP handler: GET /log.txt?n=40
// 返回内存黑匣子的精简摘要（9 列、一行约 62 字符），专为手机截图设计：
// 网页控制台在手机上不能滚动/复制，直接看这个纯文本页最省事。
static esp_err_t handler_log_txt(httpd_req_t *req) {
	char query[64] = "";
	httpd_req_get_url_query_str(req, query, sizeof(query));
	int n = 40;
	char param[16];
	if (httpd_query_key_value(query, "n", param, sizeof(param)) == ESP_OK) n = atoi(param);
	if (n <= 0) n = 40;
	if (n > 100) n = 100;

	static char txt[8192];
	int len = buildLogBrief(txt, (int)sizeof(txt), n);
	httpd_resp_set_type(req, "text/plain; charset=utf-8");
	httpd_resp_set_hdr(req, "Cache-Control", "no-store");
	httpd_resp_send(req, txt, len);
	return ESP_OK;
}

// HTTP handler: GET /vib.txt
// 返回最近一次 CLI `vib` 的振动自检报告（纯文本，4 行）。
// 用途：手机上网页控制台不能复制/不便滚动，用 Safari 打开这个纯文本页
// 就能正常滑动、长按全选复制或整页截图，方便把数据发出来。
static esp_err_t handler_vib_txt(httpd_req_t *req) {
	httpd_resp_set_type(req, "text/plain; charset=utf-8");
	httpd_resp_set_hdr(req, "Cache-Control", "no-store");
	httpd_resp_send(req, getVibReport(), HTTPD_RESP_USE_STRLEN);
	return ESP_OK;
}

// HTTP handler: POST /console/enable and /console/disable
static esp_err_t handler_console_enable(httpd_req_t *req) {
	logHttpRequest(req, "console-enable");
	webConsoleEnabled = true;
	httpd_resp_set_type(req, "application/json");
	httpd_resp_send(req, "{\"ok\":1}", HTTPD_RESP_USE_STRLEN);
	return ESP_OK;
}

static esp_err_t handler_console_disable(httpd_req_t *req) {
	logHttpRequest(req, "console-disable");
	webConsoleEnabled = false;
	httpd_resp_set_type(req, "application/json");
	httpd_resp_send(req, "{\"ok\":1}", HTTPD_RESP_USE_STRLEN);
	return ESP_OK;
}

static esp_err_t handler_not_found(httpd_req_t *req, httpd_err_code_t err) {
	(void)err;
	logHttpRequest(req, "not-found");
	if (req->method == HTTP_GET) {
		httpd_resp_set_status(req, "200 OK");
		return sendWebRCPage(req);
	}

	httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "not found");
	return ESP_FAIL;
}

static void registerUri(const char* name, const char* uri, httpd_method_t method, esp_err_t (*handler)(httpd_req_t *)) {
	httpd_uri_t item = {};
	item.uri = uri;
	item.method = method;
	item.handler = handler;
	esp_err_t ret = httpd_register_uri_handler(server, &item);
	char step[96];
	snprintf(step, sizeof(step), "register %s %s %s", name, httpMethodName(method), uri);
	logEspResult(step, ret);
}

// 启动 HTTP 服务器并注册全部 URI。
// 配置要点：端口 80、栈 8KB、并发 socket 上限 4、启用 LRU 回收、
// 缩短收发超时以快速释放半开连接（AP 直连场景并发低，防半开连接堆积）。
void setupWebRC() {
	print("Setup WEB RC\n");
	lastValidThrottle = 0.0f;
	lastValidRoll = lastValidPitch = lastValidYaw = 0.0f;

	httpd_config_t config = HTTPD_DEFAULT_CONFIG();
	config.server_port = 80;
	config.max_uri_handlers = 20;
	config.stack_size = 8192;
	config.send_wait_timeout = 5;   // 缩短发送超时：客户端不读则快速释放，避免 httpd socket 长时间挂起
	config.recv_wait_timeout = 2;   // 缩短接收超时：半开连接快速回收，避免占用有限的并发槽
	config.lru_purge_enable = true;
	config.max_open_sockets = 4;    // AP 直连场景并发低，限制并发槽防止半开连接堆积拖垮后续请求
	config.uri_match_fn = httpd_uri_match_wildcard;

	esp_err_t ret = httpd_start(&server, &config);
	if (ret != ESP_OK) {
		print("Web RC server start failed: %s (%d)\n", esp_err_to_name(ret), (int)ret);
		return;
	}
	print("Web RC HTTP server listening on port %d\n", config.server_port);

	registerUri("root", "/", HTTP_GET, handler_root);
	registerUri("index", "/index.html", HTTP_GET, handler_root);
	registerUri("web page", "/web_rc", HTTP_GET, handler_root);
	registerUri("web page slash", "/web_rc/", HTTP_GET, handler_root);
	registerUri("apple hotspot", "/hotspot-detect.html", HTTP_GET, handler_root);
	registerUri("apple success", "/library/test/success.html", HTTP_GET, handler_root);
	registerUri("success txt", "/success.txt", HTTP_GET, handler_root);
	registerUri("android probe", "/generate_204", HTTP_GET, handler_root);
	registerUri("web rc", "/web_rc", HTTP_POST, handler_web_rc);
	registerUri("heartbeat", "/web_rc/heartbeat", HTTP_POST, handler_web_rc);
	registerUri("status", "/web_rc/status", HTTP_GET, handler_status);
	registerUri("console", "/console", HTTP_GET, handler_console_get);
	registerUri("console cmd", "/console/cmd", HTTP_POST, handler_console_cmd);
	registerUri("log txt", "/log.txt", HTTP_GET, handler_log_txt);
	registerUri("vib txt", "/vib.txt", HTTP_GET, handler_vib_txt);
	registerUri("console enable", "/console/enable", HTTP_POST, handler_console_enable);
	registerUri("console disable", "/console/disable", HTTP_POST, handler_console_disable);
	logEspResult("register 404 handler", httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, handler_not_found));

	print("✓ Web RC 已启动，访问地址: http://192.168.4.1\n");
}

void readWebRC() {
	// esp_http_server handles requests asynchronously
	if (isWebRCEnabled()) {
		webRCEnabled = useWebRC = true;
	} else {
		webRCEnabled = useWebRC = false;
	}
}

#else
void setupWebRC() { print("Web RC已禁用\n"); }
void readWebRC()  {}
void processConsoleCommandQueue() {}
#endif
