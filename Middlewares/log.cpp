// ============================================================================
// log.cpp —— 内存飞行日志（黑匣子）
//
// 职责:
//   在解锁飞行期间以固定频率把关键状态采样到 RAM 环形缓冲区，供事后通过
//   CLI 命令 `log` / `log dump` 导出 CSV，用于分析振动、跟踪误差、姿态漂移等。
//
// 关键逻辑:
//   - 采样列由 logEntries[] 声明（列名 + 变量指针），打印表头与数据都遍历它；
//   - 缓冲区是固定大小的二维数组 logBuffer[LOG_SIZE][18]，写满后回绕覆盖最旧数据；
//   - logData() 仅在上锁(armed)时记录，并按 LOG_RATE(100Hz) 限速；
//   - attitudeEuler/attitudeTargetEuler 是四元数转欧拉角的中间缓存，列里记录的是它们。
//
// 输入/输出:
//   无参数。输出为串口 CSV 文本（printLogHeader / printLogData）。
//
// 重要参数:
//   LOG_RATE     采样频率 Hz（100），与 LOG_DURATION 相乘决定缓冲容量。
//   LOG_DURATION 记录时长 s，来自板级宏 BOARD_LOG_DURATION（ESP32/S3 为 8s）。
//   LOG_SIZE     = LOG_DURATION * LOG_RATE，缓冲区行数。
//
// 边界情况与潜在风险:
//   - 缓冲区为静态分配：LOG_SIZE*14*4 字节，ESP32 默认 8s*100Hz=800 行约 44KB，
//     属较大的常驻 RAM 占用，调大 LOG_DURATION 需评估内存余量。
//   - 列数上限硬编码为 18（logBuffer 第二维）：新增列超过 18 会越界写，必须先改这里。
//   - printLogData 以 logBuffer[i][0]==0（时间列为 0）判断"空槽"，若某次采样恰好
//     t 为 0 则该行被跳过（实际飞行中 t 远大于 0，不会触发）。
//   - 回绕后 dump 输出顺序不是时间序（从物理 0 行开始），分析时需自行按 t 排序。
// ============================================================================

#include "globals.h"

#define LOG_RATE 100                              // 采样频率 Hz
#define LOG_DURATION BOARD_LOG_DURATION           // 记录时长 s（板级宏）
#define LOG_SIZE (LOG_DURATION * LOG_RATE)        // 环形缓冲行数

Vector attitudeEuler;        // attitude 的欧拉角缓存（供日志记录）
Vector attitudeTargetEuler;  // attitudeTarget 的欧拉角缓存（供日志记录）

// 一条日志列：列名 + 指向被记录变量的指针
struct LogEntry {
	const char *name;
	float *value;
};

// 日志列定义：顺序即 CSV 列顺序（共 18 列，勿超 logBuffer 的第二维）
static LogEntry logEntries[] = {
	{"t", &t},
	{"rates.x", &rates.x},
	{"rates.y", &rates.y},
	{"rates.z", &rates.z},
	{"ratesTarget.x", &ratesTarget.x},
	{"ratesTarget.y", &ratesTarget.y},
	{"ratesTarget.z", &ratesTarget.z},
	{"attitude.x", &attitudeEuler.x},
	{"attitude.y", &attitudeEuler.y},
	{"attitude.z", &attitudeEuler.z},
	{"attitudeTarget.x", &attitudeTargetEuler.x},
	{"attitudeTarget.y", &attitudeTargetEuler.y},
	{"attitudeTarget.z", &attitudeTargetEuler.z},
	{"thrustTarget", &thrustTarget},
	// 四路电机输出（2026-10-23 新增）：翻机/失控分析的关键列——
	// 没有它就无法判断"是姿态先偏、还是某一路电机先被钳死"
	{"motors[RL]", &motors[MOTOR_REAR_LEFT]},
	{"motors[RR]", &motors[MOTOR_REAR_RIGHT]},
	{"motors[FR]", &motors[MOTOR_FRONT_RIGHT]},
	{"motors[FL]", &motors[MOTOR_FRONT_LEFT]}
};

static const int logColumns = sizeof(logEntries) / sizeof(logEntries[0]);
static float logBuffer[LOG_SIZE][18];  // 18 列上限（14 + 4 路电机；硬编码，新增列必须先改这里）
static int logPointer = 0;             // 环形写指针（printLogTail 需要读，故提到文件作用域）

// 把四元数姿态转换为欧拉角缓存（打印日志前调用）
void prepareLogData() {
	attitudeEuler = attitude.toEuler();
	attitudeTargetEuler = attitudeTarget.toEuler();
}

// 按 LOG_RATE 限速采样一帧；仅解锁时记录
void logData() {
	if (!armed) return;
	static Rate period(LOG_RATE);
	if (!period) return;

	prepareLogData();

	for (int i = 0; i < logColumns; i++) {
		logBuffer[logPointer][i] = *logEntries[i].value;
	}

	logPointer++;
	if (logPointer >= LOG_SIZE) { // 环形回绕
		logPointer = 0;
	}
}

// 打印 CSV 表头（列名）
void printLogHeader() {
	for (int i = 0; i < logColumns; i++) {
		print("%s%s", logEntries[i].name, i < logColumns - 1 ? "," : "\n");
	}
}

// 打印缓冲区中所有已写入的日志行（跳过空槽）
void printLogData() {
	for (int i = 0; i < LOG_SIZE; i++) {
		if (logBuffer[i][0] == 0) continue; // 空槽判断（见文件头风险）
		for (int j = 0; j < logColumns; j++) {
			print("%g%s", logBuffer[i][j], j < logColumns - 1 ? "," : "\n");
		}
	}
}

// 生成"精简摘要"文本（供网页 GET /log.txt 使用）。
// 每行只保留判断失控最关键的 9 个量，固定列宽、1~2 位小数，一行约 62 字符：
//   t | roll(°) | pitch(°) | rateX | targetRateX | mRL mRR mFR mFL
// 目的：手机竖屏一屏能看十几行，可直接截图，绕开"网页控制台不能滚动/复制"的限制。
// 列索引对应 logEntries 顺序：0=t 1=rates.x 4=ratesTarget.x 7=attitude.x 8=attitude.y
//                            14=motors[RL] 15=motors[RR] 16=motors[FR] 17=motors[FL]
int buildLogBrief(char* out, int cap, int rows) {
	if (!out || cap < 2) return 0;
	int pos = 0, w;

	// 纯数字 CSV：每行 9 个值，逗号分隔，无表头/无空格/无字母。
	// 列序固定（解析端按此顺序）：t, roll(°), pitch(°), rateX, tgtRX, RL, RR, FR, FL
	if (rows <= 0 || rows > LOG_SIZE) rows = LOG_SIZE;
	int start = (logPointer - rows + LOG_SIZE) % LOG_SIZE;
	for (int k = 0; k < rows; k++) {
		int i = (start + k) % LOG_SIZE;
		if (logBuffer[i][0] == 0) continue; // 空槽（未写过）
		w = snprintf(out + pos, (size_t)(cap - pos), "%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
		             logBuffer[i][0],
		             degrees(logBuffer[i][7]),  // roll  (°)
		             degrees(logBuffer[i][8]),  // pitch (°)
		             logBuffer[i][1],           // 实测滚转角速率 (rad/s)
		             logBuffer[i][4],           // 目标滚转角速率 (rad/s)
		             logBuffer[i][14], logBuffer[i][15], logBuffer[i][16], logBuffer[i][17]);
		if (w < 0 || w >= cap - pos) { out[cap - 1] = '\0'; return cap - 1; }
		pos += w;
	}
	out[pos] = '\0';
	return pos;
}

// 打印最近 rows 行（按时间顺序，从最旧到最新）。
// 用于事故现场：翻机/失控后立刻 `log tail`，只导出最后不到一秒的数据，
// 能直接看出"姿态先偏还是电机先钳位"。rows<=0 或超过容量时按容量处理。
void printLogTail(int rows) {
	if (rows <= 0 || rows > LOG_SIZE) rows = LOG_SIZE;
	// 从当前写指针往前数 rows 行（环形，注意取模）
	int start = (logPointer - rows + LOG_SIZE) % LOG_SIZE;
	for (int k = 0; k < rows; k++) {
		int i = (start + k) % LOG_SIZE;
		if (logBuffer[i][0] == 0) continue; // 空槽（未写过）
		for (int j = 0; j < logColumns; j++) {
			print("%g%s", logBuffer[i][j], j < logColumns - 1 ? "," : "\n");
		}
	}
}
