// ============================================================================
// cli.cpp —— 串口命令行（CLI）与控制台输出
//
// 职责:
//   在 UART0（115200）上提供一个简单命令行，用于地面调试：
//   查看/修改参数、标定 IMU/遥控、测试电机、切换飞行模式、解锁上锁、查看状态、导出日志等。
//   同时 print() 是全工程统一的输出入口，会同时把文本送到串口、MAVLink 和网页控制台。
//
// 关键逻辑:
//   - handleInput() 每周期非阻塞读取串口字节，按行（\n 或 \r）触发 doCommand()；
//   - doCommand() 用 splitString 切成"命令 + 参数0 + 参数1"后逐条匹配（命令名转小写）；
//   - print() 用 vsnprintf 格式化后写 UART0，并按编译开关转发给 MAVLink / 网页控制台；
//   - pause() 在等待期间继续 step()/handleInput()/processMavlink()/readWebRC()，
//     使"等待"期间时间与通讯不中断（但控制环不推进）。
//
// 输入/输出:
//   输入：UART0 上的文本命令；输出：文本响应（经 print）。
//
// 边界情况与潜在风险:
//   - print() 使用 1000 字节栈缓冲，单条日志过长会被截断。
//   - handleInput() 的输入行缓冲为 200 字节，超长输入会被丢弃多余字符（不会报错）。
//   - `preset`（重置参数）会擦除 NVS 并重启，注意会连带清除 WiFi 凭证。
//   - `arm` 命令只检查 imuOK，不检查电量/油门，属调试命令，飞行前慎用。
//   - pause() 期间不执行 estimate()/control()/sendMotors()，即控制环停摆；
//     因此 pause 只适合地面使用（如电机测试）。pause() 内部仍会调 handleInput()
//     以保持交互，但 doCommand 有 cliBusy 重入闩锁，命令执行期间收到的输入会被丢弃。
//   - `plot` 绘图与 `trace` 遥测**默认关闭**：上电串口上只有 CLI/日志文本。
//     需要看波形时 `plot on`（地面 50Hz，解锁后固件自动降到 10Hz），
//     需要人可读数据行时 `trace on`（5Hz）。用完记得关，别让它常开占带宽。
//   - CLI 与网页控制台共用 doCommand()，两者命令集完全一致。
// ============================================================================

#include "globals.h"
#include "cf_math.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <ctype.h>

#define CLI_UART_NUM 0
#define CLI_BUF_SIZE 256
#define CLI_MAX_LINE 200

static char cliInputLine[CLI_MAX_LINE];
static int cliInputLen = 0;
static bool cliMotdShown = false;
// doCommand 执行期间置位。pause() 会在等待期间继续调 handleInput() 以保持 CLI 交互，
// 于是形成 doCommand → testMotor → pause → handleInput → doCommand 的递归：
// cliInputLine/cliInputLen 是文件级单例，重入会把外层命令的半截输入冲掉，
// 还可能在电机测试途中嵌套执行第二条命令（例如再触发一次 testMotor）。
// 加这个闩锁后，命令执行期间收到的字节一律丢弃，不再嵌套分发。
static bool cliBusy = false;

static const char* motd =
"CLI命令菜单，输入相应命令，回车后执行:\n"
"help - 帮助\n"
"p - 显示所有参数\n"
"p <name> - 显示指定参数\n"
"p <name> <value> - 设置参数\n"
"preset - 重置参数存储\n"
"mfr, mfl, mrr, mrl - 测试马达\n"
"ca - 六面校准加速度计(按提示翻转,共6面约1分钟)\n"
"ca1 - 快速校准加速度计(仅水平静置,阻塞约4秒)\n"
"cg - 只校准陀螺仪零偏(静置,阻塞约1秒)\n"
"vib [秒] - 振动自检(默认2秒), 结果同时见 http://192.168.4.1/vib.txt\n"
"ps - 显示姿态\n"
"cr - 校准RC遥控器\n"
"rc - 显示RC遥控数据\n"
"plot [on|off] - IMU串口绘图开关(默认off,开启后50Hz/解锁后10Hz)\n"
"hh - 开启串口数据打印(同 plot on)\n"
"HH - 关闭串口数据打印(同 plot off)\n"
"trace [on|off] - IMU串口可读遥测开关(默认off,开启后5Hz)\n"
"quiet - 一键关闭全部数据输出(trace+plot)\n"
"wifi - 显示WiFi信息\n"
"ap <ssid> <password> - 配置AP模式\n"
"p WIFI_AP_AUTH 0|1 - AP加密(0=开放,1=WPA2,重启生效)\n"
"sta <ssid> <password> - 配置STA模式\n"
"raw/stab/acro/auto - 飞行模式\n"
"arm - 解锁无人机\n"
"disarm - 锁定无人机\n"
"psq - 显示姿态四元数\n"
"imu - 显示IMU数据\n"
"time - 显示时间\n"
"mot - 显示motor输出\n"
"sys - 显示系统info\n"
"log dump - 导出全部日志; log tail [n] - 导出最近n行(默认60)\n"
"reboot - 重启\n"
"reset - 重置姿态\n";

// 统一输出入口：格式化后写 UART0，并按编译开关转发给 MAVLink 与网页控制台。
// 注意：栈缓冲固定 1000 字节，超长内容会被 vsnprintf 截断。

// 只写 UART0 且绝不阻塞：TX 环形缓冲余量不足时整帧丢弃并返回 false。
// 供 1kHz 控制任务（绘图/遥测/failsafe 日志）使用——控制环里宁丢日志，不丢节拍。
bool consoleWriteNoBlock(const char* buf, int len) {
	if (len <= 0) return true;
	size_t freeSize = 0;
	if (uart_get_tx_buffer_free_size((uart_port_t)CLI_UART_NUM, &freeSize) != ESP_OK) return false;
	if (freeSize < (size_t)len) return false;
	return uart_write_bytes((uart_port_t)CLI_UART_NUM, buf, len) == len;
}

// 公共输出路径。blocking=true 为普通日志语义（等 TX 环有空位，保证不丢）；
// blocking=false 为控制任务语义（拥塞即丢，绝不阻塞调用者）。
static void printToSinks(const char* buf, bool blocking) {
	if (blocking) {
		uart_write_bytes((uart_port_t)CLI_UART_NUM, buf, strlen(buf));
	} else {
		consoleWriteNoBlock(buf, (int)strlen(buf));
	}

#if WIFI_ENABLED
	mavlinkPrint(buf); // 转发到 MAVLink 串口控制台（内部只做缓冲，不阻塞）
#endif
#if WEB_RC_ENABLED
	if (webConsoleEnabled) webLog(buf); // 转发到网页控制台（内部只做环形缓冲）
#endif
}

void print(const char* format, ...) {
	char buf[1000];
	va_list args;
	va_start(args, format);
	vsnprintf(buf, sizeof(buf), format, args);
	va_end(args);
	printToSinks(buf, true);
}

// 控制任务专用：串口拥塞时丢弃本行，绝不让 UART 阻塞 1kHz 控制环
void printNonBlocking(const char* format, ...) {
	char buf[1000];
	va_list args;
	va_start(args, format);
	vsnprintf(buf, sizeof(buf), format, args);
	va_end(args);
	printToSinks(buf, false);
}

// 阻塞等待指定时长（秒）。置位 controlPaused 让控制任务跳过 estimate/control/sendMotors，
// 期间 CLI 可独占 PWM（电机测试等），时间基准 t 仍由控制任务的 step() 推进。
// 仅限地面使用（挂起期间 failsafe 与电机输出全部停摆）。
// 注意：本函数在通信任务中执行；processMavlink/readWebRC 已由各自任务独立运行，无需在此轮询。
// 结束时恢复调用前的 controlPaused 值而不是无条件清零——testMotor() 会先自行置位再调
// 本函数，无条件清零会把外层的语义吃掉（虽然现在两者恰好一致，但不要依赖这种巧合）。
void pause(float duration) {
	bool wasPaused = controlPaused;
	controlPaused = true;
	float start = t;
	while (t - start < duration) {
		handleInput(); // 保持 CLI 交互（命令执行期间收到的字节会被 cliBusy 闩锁丢弃）
		vTaskDelay(pdMS_TO_TICKS(10));
	}
	controlPaused = wasPaused;
}

void doCommand(const char* str, bool echo) {
	char buf[CLI_MAX_LINE];
	strncpy(buf, str, CLI_MAX_LINE - 1);
	buf[CLI_MAX_LINE - 1] = '\0';

	char command[64], arg0[64], arg1[128];
	splitString(buf, command, arg0, arg1, sizeof(command));
	if (command[0] == '\0') return;

	if (echo) {
		print("> %s\n", str);
	}

	// hh / HH：快速开关 IMU 串口数据打印（等价于 plot on / plot off）。
	// 必须在下方 lowercase 之前判断——命令统一转小写后 hh 与 HH 将无法区分。
	if (strcmp(command, "hh") == 0 || strcmp(command, "HH") == 0) {
		bool on = (command[0] == 'h');
		setImuPlot(on);
		print("IMU plot %s\n", on ? "on" : "off");
		return;
	}

	// lowercase command
	for (char* p = command; *p; p++) *p = tolower(*p);

	// 重入闩锁：整条命令执行期间置位，阻止 pause()→handleInput() 递归再次分发命令。
	// 用 RAII 风格的手工配对，保证任何提前 return 都不会把闩锁留在置位状态。
	struct BusyGuard {
		BusyGuard() { cliBusy = true; }
		~BusyGuard() { cliBusy = false; }
	} busyGuard;

	if (strcmp(command, "help") == 0 || strcmp(command, "motd") == 0) {
		print("%s\n", motd);
	} else if (strcmp(command, "p") == 0 && arg0[0] == '\0') {
		printParameters();
	} else if (strcmp(command, "p") == 0 && arg0[0] != '\0' && arg1[0] == '\0') {
		print("%s = %g\n", arg0, getParameter(arg0));
	} else if (strcmp(command, "p") == 0) {
		bool success = setParameter(arg0, atof(arg1));
		if (success) {
			print("%s = %g\n", arg0, getParameter(arg0));
		} else {
			print("Parameter not found: %s\n", arg0);
		}
	} else if (strcmp(command, "preset") == 0) {
		resetParameters();
	} else if (strcmp(command, "time") == 0) {
		print("Time: %f\n", t);
		print("Loop rate: %.0f\n", loopRate);
		print("dt: %f\n", dt);
	} else if (strcmp(command, "ps") == 0) {
		Vector a = attitude.toEuler();
		print("roll: %f pitch: %f yaw: %f\n", degrees(a.x), degrees(a.y), degrees(a.z));
	} else if (strcmp(command, "psq") == 0) {
		print("qw: %f qx: %f qy: %f qz: %f\n", attitude.w, attitude.x, attitude.y, attitude.z);
	} else if (strcmp(command, "imu") == 0) {
		printIMUInfo();
		printIMUCalibration();
		print("landed: %d\n", landed);
	} else if (strcmp(command, "arm") == 0) {
		if (!imuOK) { print("IMU故障，禁止解锁！\n"); }
		else armed = true;
	} else if (strcmp(command, "disarm") == 0) {
		armed = false;
	} else if (strcmp(command, "raw") == 0) {
		mode = RAW;
	} else if (strcmp(command, "stab") == 0) {
		mode = STAB;
	} else if (strcmp(command, "acro") == 0) {
		mode = ACRO;
	} else if (strcmp(command, "auto") == 0) {
		mode = AUTO;
	} else if (strcmp(command, "rc") == 0) {
		print("channels: ");
		for (int i = 0; i < 16; i++) print("%u ", channels[i]);
		print("\nroll: %g pitch: %g yaw: %g throttle: %g mode: %g\n",
			controlRoll, controlPitch, controlYaw, controlThrottle, controlMode);
		print("time: %.1f\n", controlTime);
		print("mode: %s\n", getModeName());
		print("armed: %d\n", armed);
		printRCCalibration(); // 标定值 + 收帧统计 + 物理遥控信号状态
	} else if (strcmp(command, "wifi") == 0) {
#if WIFI_ENABLED
		printWiFiInfo();
#endif
	} else if (strcmp(command, "ap") == 0) {
#if WIFI_ENABLED
		configWiFi(true, arg0, arg1);
#endif
	} else if (strcmp(command, "sta") == 0) {
#if WIFI_ENABLED
		configWiFi(false, arg0, arg1);
#endif
	} else if (strcmp(command, "mot") == 0) {
		print("front-right %g front-left %g rear-right %g rear-left %g\n",
			motors[MOTOR_FRONT_RIGHT], motors[MOTOR_FRONT_LEFT], motors[MOTOR_REAR_RIGHT], motors[MOTOR_REAR_LEFT]);
	} else if (strcmp(command, "log") == 0) {
		printLogHeader();
		if (strcmp(arg0, "dump") == 0) {
			printLogData();          // 全部已写入的行（环形，顺序非时间序）
		} else if (strcmp(arg0, "tail") == 0) {
			int n = (arg1[0] != '\0') ? atoi(arg1) : 60; // 默认最近 0.6 秒（60 行 @100Hz）
			printLogTail(n);         // 按时间顺序导出最近 n 行——事故现场用
		}
	} else if (strcmp(command, "cr") == 0) {
		calibrateRC();
	} else if (strcmp(command, "cg") == 0) {
		calibrateGyroOnce();
	} else if (strcmp(command, "ca") == 0) {
		calibrateAccel6();
	} else if (strcmp(command, "ca1") == 0) {
		calibrateAccel();
	} else if (strcmp(command, "vib") == 0) {
		// 振动自检：默认 2 秒。输出 4 行摘要，同时挂在 /vib.txt 供手机复制/截图。
		measureVibration(arg0[0] != '\0' ? atoi(arg0) : 2);
	} else if (strcmp(command, "plot") == 0) {
		// IMU 串口调试绘图开关。默认关闭（上电不发数值行，不刷屏）；
		// 手动 `plot on` 后才输出，频率地面 50Hz、解锁后自动 10Hz，
		// 避免数值行一直占带宽、把 failsafe / 定高 / HTTP 的 print() 挤掉。
		if (arg0[0] == '\0') {
			print("IMU plot: %s\n", getImuPlot() ? "on" : "off");
		} else if (strcasecmp(arg0, "on") == 0 || strcmp(arg0, "1") == 0) {
			setImuPlot(true);
		} else if (strcasecmp(arg0, "off") == 0 || strcmp(arg0, "0") == 0) {
			setImuPlot(false);
		} else {
			print("用法: plot [on|off]\n");
		}
	} else if (strcmp(command, "trace") == 0) {
		// 串口可读遥测：低频（5Hz）一行带标签的数据，串口助手/网页控制台直接可读，
		// 不需要 VOFA+。与 plot 互不冲突，可同时开（但飞行前建议都关）。
		if (arg0[0] == '\0') {
			print("IMU trace: %s\n", getImuTrace() ? "on" : "off");
		} else if (strcasecmp(arg0, "on") == 0 || strcmp(arg0, "1") == 0) {
			setImuTrace(true);
		} else if (strcasecmp(arg0, "off") == 0 || strcmp(arg0, "0") == 0) {
			setImuTrace(false);
		} else {
			print("用法: trace [on|off]\n");
		}
	} else if (strcmp(command, "quiet") == 0) {
		// 一键静音：同时关掉 trace 与 plot 两路数据流，只保留命令回显。
		// 飞行前收尾用，等价于连发 `trace off` + `plot off`。
		setImuTrace(false);
		setImuPlot(false);
	} else if (strcmp(command, "mfr") == 0) {
		testMotor(MOTOR_FRONT_RIGHT);
	} else if (strcmp(command, "mfl") == 0) {
		testMotor(MOTOR_FRONT_LEFT);
	} else if (strcmp(command, "mrr") == 0) {
		testMotor(MOTOR_REAR_RIGHT);
	} else if (strcmp(command, "mrl") == 0) {
		testMotor(MOTOR_REAR_LEFT);
	} else if (strcmp(command, "sys") == 0) {
		print("Chip: ESP32\n");
		print("Free heap: %d\n", (int)esp_get_free_heap_size());
	} else if (strcmp(command, "reset") == 0) {
		// 多任务改造：attitude/零偏滤波器只允许控制任务写，这里置请求标志
		attitudeResetRequested = true;
		print("Attitude reset requested\n");
	} else if (strcmp(command, "reboot") == 0) {
		esp_restart();
	} else {
		print("Invalid command: %s\n", command);
	}
}

// 每周期非阻塞读取串口，累积到一行（\n 或 \r）后交给 doCommand 执行。
// 首次调用时打印一次命令菜单（motd）。输入行超过 CLI_MAX_LINE-1 的部分被丢弃。
// 命令执行期间（cliBusy，例如 testMotor 的 pause 循环里）收到的字节直接丢弃并清空
// 行缓冲，避免递归执行第二条命令、也避免半截输入污染外层的 cliInputLine。
void handleInput() {
	if (!cliMotdShown) {
		print("%s\n", motd);
		cliMotdShown = true;
	}

	uint8_t byte;
	while (uart_read_bytes((uart_port_t)CLI_UART_NUM, &byte, 1, 0) > 0) {
		if (cliBusy) { // 命令执行中：排空串口但不解析
			cliInputLen = 0;
			continue;
		}
		if (byte == '\n' || byte == '\r') {
			if (byte == '\r') {
				// also consume following \n if present
				uart_read_bytes((uart_port_t)CLI_UART_NUM, &byte, 1, 0);
			}
			cliInputLine[cliInputLen] = '\0';
			cliInputLen = 0; // 先清零再执行：doCommand 内部可能重入 handleInput
			doCommand(cliInputLine, false);
		} else if (cliInputLen < CLI_MAX_LINE - 1) {
			cliInputLine[cliInputLen++] = byte;
		}
	}
}
