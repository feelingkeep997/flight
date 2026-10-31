// ============================================================================
// imu.cpp —— IMU 上层驱动（初始化、读取、去偏、标定、调试绘图）
//
// 职责:
//   在 mpu9250.cpp（SPI 寄存器层）之上，提供飞控需要的"已校准、已对齐坐标系"的
//   角速度 gyro 与加速度 acc，并负责芯片识别、零偏标定与串口调试绘图。
//
// 关键逻辑:
//   - setupIMU()/configureIMU() 初始化 SPI 并按 WHO_AM_I 识别芯片
//     （0x71/0x73=MPU9250/9255，0x70=MPU6500；本项目实际使用 MPU6500，寄存器兼容）；
//   - setupIMU() 末尾做一次上电陀螺零偏标定 calibrateGyroOnce()（约 1s，需静置）；
//   - readIMU() 每周期：读原始值 → 陀螺去偏（静态标定 gyroBias + 在线微调 gyroBiasTrim）→
//     减 accBias → 乘 accScale → 若配置了 imuRotation 则把向量旋到机体坐标系 →
//     写入全局 gyro / acc；
//   - 陀螺去偏是"静态标定 + 静止在线微调"两级：gyroBias 由上电/CLI 标定得到，
//     gyroBiasTrim 只在"未解锁 且 连续 1s 角速度低于 2.9°/s"时以极慢速率更新，
//     用于跟踪温漂。**绝不无条件跟踪角速度**——见下方 gyroBiasTrim 注释的历史教训。
//
// 输入/输出:
//   输入：SPI 硬件（经 mpu9250_* 接口）、参数（IMU_ROT_* / IMU_ACC_BIAS_* / IMU_ACC_SCALE_*）。
//   输出：全局 gyro（rad/s）、acc（m/s²）、imuOK 标志。
//
// 重要参数:
//   imuRotation  传感器到机体的安装旋转（欧拉角，参数 IMU_ROT_ROLL/PITCH/YAW）
//   accBias/accScale 加速度计零偏与比例修正
//   IMU_DEBUG_PLOT / IMU_DEBUG_PLOT_HZ  调试绘图编译开关与输出频率
//   imuPlotOn    调试绘图运行期开关，默认 false（上电不发数值行，需 `plot on`），CLI `plot on/off` 切换
//
// 边界情况与潜在风险:
//   - imuOK 为 false 时 readIMU() 直接返回，gyro/acc 保持上一次的值（不会清零），
//     因此解锁自检必须依赖 imuOK，否则会拿着陈旧数据飞行。
//   - calibrateGyroOnce() 会阻塞约 1s（200×5ms），采样写入局部变量后一次性写入 gyroBias，
//     必须在解锁前调用；calibrateAccel*() 会阻塞 3 秒+，同样仅限地面使用。
//   - calibrateAccel 假定"机体水平静止"，未做六面标定，故只能校正零偏不能校正比例。
//   - 调试绘图**默认关闭**：上电不再自动灌数值行，串口只有 CLI / 日志文本；
//     需要波形时手动 `plot on`（地面 50Hz≈40% 带宽，解锁后自动 10Hz≈8%）。
//     默认关闭的原因：一路 17 通道数值行会把 failsafe / 定高 / CLI 的 print()
//     挤在 512 字节 TX 环里排队甚至丢弃，且接普通串口助手时根本没法看。
//   - 调试绘图输出用 invalid(hoverThrustEst) 判断，未激活时发 -1 作为占位。
// ============================================================================

#include "globals.h"
#include "mpu9250.h"
#include <cmath>
#include "esp_log.h"
#include "driver/uart.h"   // 调试绘图需直接写控制台串口
#include <cstdio>          // snprintf
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// ============================================================
// FireWater 串口绘图（调试用）
// 数据格式（无标签形式）："ch0,ch1,...,chN\n"
// 必须以 '\n' 结尾，且每行只能有纯数值 + 逗号，FireWater 才会绘图
//
// 开关设计：编译期 IMU_DEBUG_PLOT 决定是否编进固件，运行期 imuPlotOn 决定是否真的发。
//   运行期默认**关闭**：上电不发任何数值行，串口上只有 CLI / 日志文本。
//   要用 VOFA+ 看波形时手动 `plot on`（地面 50Hz、解锁后自动 10Hz）。
//   默认关闭的原因：17 通道一行约 129 字节，50Hz 时 ≈6.5 kB/s，占 115200bps(≈11.5 kB/s)
//   的 ~56%，会把 failsafe / 定高 / CLI 的 print() 挤在 TX 环里排队甚至丢弃；
//   且数值行会打断串口助手里人可读的日志。
//   需要时 `plot on` 打开，`plot off` 立即关闭。
// ============================================================
#define IMU_DEBUG_PLOT      1    // 编译期总开关：1 = 编入绘图代码，0 = 完全不参与编译
#define IMU_DEBUG_PLOT_HZ   50   // 地面输出频率（Hz）
#define IMU_PLOT_ARMED_HZ   10   // 解锁后自动降到的频率（Hz），给飞行日志让带宽

#if IMU_DEBUG_PLOT
static bool imuPlotOn = false;  // 运行期开关，默认关闭（需 `plot on`，见上方说明）

// 把一帧 IMU 数据按 FireWater 格式写到控制台串口（UART0）
// 直接 uart_write_bytes 而不走 print()，避免同时灌到 MAVLink 和 Web 控制台
//
// 注意：画的是**去偏之后、控制环真正使用的** gyro/acc，不是芯片原始值。
//   早期版本画的是 mpu9250_read_data 的原始输出，因此当去偏环节把角速度
//   滤成高通（见 gyroBiasTrim 处的历史教训）时，VOFA+ 上看起来一切正常，
//   姿态却纹丝不动——调试波形必须反映控制器实际看到的量，否则等于没调。
static void imuPlot(const Vector& accel, const Vector& gyroRad) {
    if (!imuPlotOn) return;
    // ⚠️ 必须写成 "!(t - lastTime >= 周期)" 而不是 "t - lastTime < 周期"：
    // t 的初值是 NAN（main.cpp），而 controlTask 里 readIMU() 在 step() 之前调用，
    // 首帧拿到的 t 就是 NAN。若让 lastTime = NAN，此后所有比较恒为假（NAN 比较永远假），
    // 限速彻底失效 → 1kHz 灌 93 字节/帧 ≈ 93 kB/s，远超 115200bps(≈11.5 kB/s)，
    // 串口被压垮。写成 >= 的否定形式后，NAN 会走进 return 分支，天然安全。
    static float lastTime = 0.0f;
    // 解锁后自动降频：把 UART0 带宽让给 failsafe / 定高日志，但仍留一条粗波形，
    // 事后能看油门目标与姿态趋势（不至于完全瞎）。
    const float plotHz = armed ? (float)IMU_PLOT_ARMED_HZ : (float)IMU_DEBUG_PLOT_HZ;
    if (!(t - lastTime >= 1.0f / plotHz)) return; // 限速（NAN 安全）
    lastTime = t;

    char buf[256];
    // 通道顺序：acc xyz (m/s^2)、gyro xyz (deg/s)、姿态 roll/pitch/yaw (rad)、
    //           velZ (m/s)、thrustTarget (0~1)、hoverThrustEst (0~1，未激活发-1)、armed (0/1)、
    //           motor RL/RR/FR/FL (0~1 归一化推力，即 PWM 指令值，非实测转速)
    // 注意：不加任何标签/前缀，行首即第一个数值
    Vector euler = attitude.toEuler();
    // 17 个通道，逗号分隔、行尾 '\n'，与上方"通道顺序"说明一一对应：
    int n = snprintf(buf, sizeof(buf), "%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.1f,%.4f,%.4f,%.4f,%.4f\n",
                     accel.x,                                       // ch0 : 加速度 X (m/s^2)
                     accel.y,                                       // ch1 : 加速度 Y (m/s^2)
                     accel.z,                                       // ch2 : 加速度 Z (m/s^2)
                     gyroRad.x * 180.0f / M_PI,                     // ch3 : 角速度 X (deg/s)
                     gyroRad.y * 180.0f / M_PI,                     // ch4 : 角速度 Y (deg/s)
                     gyroRad.z * 180.0f / M_PI,                     // ch5 : 角速度 Z (deg/s)
                     euler.x,                                       // ch6 : 姿态 roll  (rad，绕机体 X)
                     euler.y,                                       // ch7 : 姿态 pitch (rad，绕机体 Y)
                     euler.z,                                       // ch8 : 姿态 yaw   (rad，绕机体 Z)
                     velZ,                                          // ch9 : 垂直速度 (m/s，向上为正)
                     thrustTarget,                                  // ch10: 油门目标 (0~1，已限幅)
                     invalid(hoverThrustEst) ? -1.0f : hoverThrustEst, // ch11: 悬停油门估计 (0~1；未激活发 -1)
                     armed ? 1.0f : 0.0f,                           // ch12: 解锁标志 (1=已解锁, 0=未解锁)
                     motors[MOTOR_REAR_LEFT],                       // ch13: 电机 后左 (0~1)
                     motors[MOTOR_REAR_RIGHT],                      // ch14: 电机 后右 (0~1)
                     motors[MOTOR_FRONT_RIGHT],                     // ch15: 电机 前右 (0~1)
                     motors[MOTOR_FRONT_LEFT]);                     // ch16: 电机 前左 (0~1)
    // 非阻塞写：控制任务 1kHz，TX 环被日志挤满时宁可丢这一帧绘图，也不能阻塞控制环
    if (n > 0 && n < (int)sizeof(buf)) consoleWriteNoBlock(buf, n);
}
#endif // IMU_DEBUG_PLOT

// 运行期开关（CLI `plot on/off` 调用）。IMU_DEBUG_PLOT=0 时为空实现，保证调用方无需条件编译。
void setImuPlot(bool on) {
#if IMU_DEBUG_PLOT
    imuPlotOn = on;
    print("IMU 调试绘图: %s (%dHz，解锁后自动 %dHz，17通道，含4路电机)\n",
          on ? "已开启" : "已关闭", IMU_DEBUG_PLOT_HZ, IMU_PLOT_ARMED_HZ);
#else
    (void)on;
    print("IMU 调试绘图未编入固件（IMU_DEBUG_PLOT=0）\n");
#endif
}

// 查询绘图开关状态（供 CLI 无参调用时显示）
bool getImuPlot() {
#if IMU_DEBUG_PLOT
    return imuPlotOn;
#else
    return false;
#endif
}

// ============================================================
// 串口可读遥测（给串口监视器看的，不依赖 VOFA+）
// 与 imuPlot 的区别：
//   - 走 print()，输出人能读懂的一行（带标签与单位），串口助手/网页控制台都能看；
//   - 频率低得多（IMU_TRACE_HZ，默认 5Hz，约 0.6 kB/s），不会挤掉 failsafe/定高的日志；
//   - 默认关闭，需要时 `trace on` 打开，`trace off` 关闭（不要再默认打开，
//     否则上电串口一直被数据行刷屏，CLI 交互和故障日志都看不清）。
// 与 imuPlot 一样画的是**控制环真正使用**的 acc/gyro（已去偏、已对齐机体系）。
// 限速同样写成 NAN 安全形式，理由见 imuPlot 里的注释。
// ============================================================
#define IMU_TRACE_HZ   5
static bool imuTraceOn = false;

static void imuTrace(const Vector& accel, const Vector& gyroRad) {
    if (!imuTraceOn) return;
    static float lastTime = 0.0f;
    if (!(t - lastTime >= 1.0f / IMU_TRACE_HZ)) return; // 限速（NAN 安全）
    lastTime = t;

    Vector euler = attitude.toEuler();
    // 非阻塞：本函数在控制任务里调用（1kHz），串口拥塞时丢弃这一行遥测
    printNonBlocking("[IMU] t=%6.1f acc(%5.2f %5.2f %5.2f) gyro(%7.2f %7.2f %7.2f) rpy(%6.1f %6.1f %6.1f) velZ=%5.2f thr=%.2f armed=%d\n",
          t,
          accel.x, accel.y, accel.z,                                        // m/s^2
          gyroRad.x * 180.0f / M_PI, gyroRad.y * 180.0f / M_PI, gyroRad.z * 180.0f / M_PI, // deg/s
          degrees(euler.x), degrees(euler.y), degrees(euler.z),             // deg
          velZ,                                                             // m/s
          thrustTarget,                                                     // 0~1
          armed ? 1 : 0);
}

// 运行期开关（CLI `trace on/off` 调用）
void setImuTrace(bool on) {
    imuTraceOn = on;
    print("IMU 串口遥测: %s (%dHz)\n", on ? "已开启" : "已关闭", IMU_TRACE_HZ);
}

// 查询串口遥测开关状态（供 CLI 无参调用时显示）
bool getImuTrace() { return imuTraceOn; }

// ============================================================
// 陀螺零偏：静态标定 gyroBias + 静止在线微调 gyroBiasTrim
//
// 【历史教训 · 不要改回去】
// 原实现在这里放了三个 KalmanFilter<float>(0, 1e-6f, 0.01f) 无条件跟踪 rawGyro，
// 然后用 `correctedGyro = rawGyro - gyroBias - filtered` 相减。稳态增益 K≈0.00995
// （时间常数约 100 个样本 = 0.1s），于是"信号减去它自己的低通"恰好构成一个
// 截止频率约 1.6Hz 的**高通滤波器**：
//   - 恒定 100°/s 的旋转，角速率反馈按 (1-K)^n 指数衰减（τ≈100ms），姿态角最多只能
//     积到 ω/(K·f) ≈ 10.05° 就停住不动：0.1s 积 6.3°(真值 10°)、0.3s 积 9.5°(真值 30°)、
//     1s 积 9.95°(真值 100°)，之后永远不再增长（数值仿真验证，见 docs 变更记录）；
//   - 任何持续角速度都被当成"零偏"吃掉，姿态环拿不到稳定的角速率反馈，
//     内环等效于开环 —— 这是本机重构后始终无法离地的首要原因。
// 正确做法：零偏只能在"确认静止"时估计。下面的 gyroBiasTrim 用
//   未解锁 + 角速度模长低于阈值 + 连续满足 1s 三重门控，任何真实转动都会立刻
//   清零计数，因此不可能把机动信号误当零偏。
// ============================================================
Vector gyroBiasTrim(0, 0, 0);   // 在线微调量（rad/s）：跟踪温漂，仅静止时更新
static int gyroStaticCount = 0; // 连续满足静止条件的样本数

#define GYRO_TRIM_STATIC_SAMPLES 1000   // 需连续 1000 帧（1kHz 下 1s）静止才开始微调
#define GYRO_TRIM_RATE_TH        0.09f  // rad/s ≈ 5.2°/s，低于此视为"没在转"。⚠️ 必须 > 微调上限对应的
                                         // 残余漂移：门控看的是去偏前的 rawGyro，若真实漂移超过此值，
                                         // 微调会永远学不到（2026-10-16 陀螺 Y 漂移事件的教训，原 0.05 太紧）
#define GYRO_TRIM_ALPHA          0.0005f // 微调低通系数，时间常数约 2s（1/alpha/1000Hz）
#define GYRO_TRIM_LIMIT          0.08f  // rad/s ≈ 4.6°/s，微调量硬限幅，防异常累积（原 0.035=2°/s 不够
                                         // 覆盖 MPU6500 Y 轴温漂，饱和后残留 2°/s 导致起飞持续上仰）

// 复位在线微调。gyroBiasTrim 是多字结构，控制任务的 readIMU() 每毫秒都在写它，
// 因此**跨任务调用时必须持有 imuMutex**（calibrateGyroOnce 已照此办理）；
// 控制任务自己调（CLI reset 走 attitudeResetRequested）则天然无竞争。
void resetGyroBiasTracker() {
    gyroBiasTrim = Vector(0, 0, 0);
    gyroStaticCount = 0;
}

bool imuOK = false;
Vector imuRotation(0, 0, 0);  // IMU 安装旋转（欧拉角 rad）：描述"传感器坐标系 -> 机体坐标系"的方向偏置；参数 IMU_ROT_ROLL/PITCH/YAW，默认 0 表示无旋转
Vector accBias(0, 0, 0);      // 加速度计零偏（m/s²）：每轴恒定偏移量，由 ca 校准写入（水平静止采样均值 - 重力）；参数 IMU_ACC_BIAS_*，默认 0
Vector accScale(1, 1, 1);     // 加速度计比例因子（无量纲）：每轴增益/灵敏度修正；本工程未做六面标定，故恒为 1；参数 IMU_ACC_SCALE_*
Vector gyroBias(0, 0, 0);     // 陀螺仪静态零偏（rad/s）：由 setupIMU() 上电标定或 CLI `cg` 写入

// ============================================================
// IMU 跨任务互斥（多任务架构引入）
// 控制任务以 1kHz 调 readIMU()，通信任务的 CLI 命令（ca/imu）也会直接
// 访问 SPI 并改写标定量——不加锁会出现 SPI 事务交错与标定量撕裂写入。
// setupIMU()（任务启动前）创建；未创建时退化为不加锁。
// ============================================================
static SemaphoreHandle_t imuMutex = NULL;
static inline void imuLock()   { if (imuMutex) xSemaphoreTake(imuMutex, portMAX_DELAY); }
static inline void imuUnlock() { if (imuMutex) xSemaphoreGive(imuMutex); }

// 初始化 IMU：底层 SPI 初始化 + 配置并识别芯片 + 上电陀螺零偏标定
void setupIMU() {
    print("Setting up MPU9250...\n");
    
    imuMutex = xSemaphoreCreateMutex(); // 必须在任何任务访问 IMU 前创建
    
    // 初始化 MPU9250（SPI 总线与设备）
    mpu9250_init();
    
    // 配置 IMU（量程、采样率、DLPF）并校验 WHO_AM_I
    configureIMU();

    // 上电做一次陀螺静态零偏标定（约 1s，200 次采样求均值）。
    // 此时任务尚未启动、机体必然静置，是最安全的标定窗口；
    // 之后 gyroBiasTrim 只负责跟踪温漂，不再承担"从零开始找零偏"的任务。
    if (imuOK) calibrateGyroOnce();
}

// 配置 IMU：设置陀螺 ±1000dps / 加速度 ±16g，识别 WHO_AM_I 并置 imuOK。
// 注意：识别失败时 imuOK=false，此后 readIMU() 不再更新数据。
void configureIMU() {
    print("Configuring MPU9250...\n");
    
    // 配置：陀螺 ±1000 deg/s，加速度 ±16g
    mpu9250_configure(MPU9250_GYRO_FSR_1000, MPU9250_ACCEL_FSR_16G);
    
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // 校验芯片身份
    // WHO_AM_I: 0x71/0x73 = MPU9250/MPU9255，0x70 = MPU6500
    // （本项目实际用的是 MPU6500，其加速度计/陀螺仪寄存器与 MPU9250 兼容）
    uint8_t whoami = mpu9250_read_reg(MPU_REG_WHO_AM_I);
    if (whoami == 0x71 || whoami == 0x73 || whoami == 0x70) {
        print("MPU9250/MPU6500 detected: WHO_AM_I=0x%02X\n", whoami);
        imuOK = true;
        // 安装旋转不在此层施加：readIMU() 会依据参数 imuRotation 把
        // gyro/acc 从传感器坐标系旋到机体坐标系（见下方 readIMU 注释）。
    } else {
        print("IMU not found! Expected 0x71/0x73 (MPU9250) or 0x70 (MPU6500), got 0x%02X\n", whoami);
        imuOK = false;
    }
}

// 读取一帧 IMU 数据并完成校准/对齐，写入全局 gyro / acc。
// 处理链：原始值 → 减静态零偏 gyroBias → 减静止微调量 gyroBiasTrim →
//         减 accBias → 乘 accScale → 坐标系旋转 → 调试绘图。
// 注意：imuOK=false 时直接返回（不清零数据）。
void readIMU() {
    if (!imuOK) return;
    imuLock(); // 与通信任务的 CLI 标定/查询互斥（SPI 事务不可交错）
    
    // 读取加速度计与陀螺仪原始数据
    Vector rawGyro, rawAcc;
    mpu9250_read_data(&rawGyro, &rawAcc);

    // ---- 陀螺零偏在线微调（只在确认静止时更新）----
    // 三重门控：未解锁 + 原始角速度模长低于 GYRO_TRIM_RATE_TH + 连续满足 1s。
    // 任一条件破掉立刻清零计数，因此手持移动、起飞、机动都不会污染零偏。
    if (!armed && rawGyro.norm() < GYRO_TRIM_RATE_TH) {
        if (gyroStaticCount < GYRO_TRIM_STATIC_SAMPLES) {
            gyroStaticCount++;
        } else {
            // 收敛目标是"扣除静态标定后的残余量"，不是 rawGyro：
            // readIMU 末尾是 rawGyro - gyroBias - gyroBiasTrim，gyroBias 已经被减过一次，
            // 若 trim 再收敛到 rawGyro，静止时输出会被顶成 -gyroBias（2026-10-18 修正）。
            gyroBiasTrim += ((rawGyro - gyroBias) - gyroBiasTrim) * GYRO_TRIM_ALPHA;
            // 硬限幅：即使门控被极端情况骗过，微调量也不可能大到影响姿态解算
            float trimNorm = gyroBiasTrim.norm();
            if (trimNorm > GYRO_TRIM_LIMIT) gyroBiasTrim = gyroBiasTrim * (GYRO_TRIM_LIMIT / trimNorm);
        }
    } else {
        gyroStaticCount = 0;
    }

    // 去偏：静态标定值 + 静止期学到的温漂微调量。这里只做"减去常值偏置"，
    //       绝不能出现"减去自己的低通"这类高通结构（见 gyroBiasTrim 处的历史教训）。
    Vector correctedGyro = rawGyro - gyroBias - gyroBiasTrim;
    Vector correctedAcc = rawAcc - accBias;
    
    // 应用加速度计比例修正
    correctedAcc.x *= accScale.x;
    correctedAcc.y *= accScale.y;
    correctedAcc.z *= accScale.z;
    
    // 若配置了安装旋转，则把两路向量旋到机体坐标系
    if (imuRotation.norm() > 0.001f) {
        Quaternion rotQuat = Quaternion::fromEuler(imuRotation);
        correctedGyro = rotQuat.conjugate(correctedGyro);
        correctedAcc = rotQuat.conjugate(correctedAcc);
    }
    
    gyro = correctedGyro;
    acc = correctedAcc;
    imuUnlock();

    // 输出一律放在解锁之后：UART 写在 TX 环满时会阻塞，持着 imuMutex 阻塞
    // 会把通信任务的 cg/ca 标定命令一起卡住（控制任务 1kHz，更不能在里面堵）。
#if IMU_DEBUG_PLOT
    imuPlot(acc, gyro); // FireWater 调试绘图：输出控制环真正使用的 acc / gyro（已去偏、已对齐机体）
#endif
    imuTrace(acc, gyro); // 同帧数据的人类可读版本（默认 5Hz）
}

// 陀螺仪一次性零偏标定：静止采样 200 次求均值作为 gyroBias。
// 多任务改造：采样写入局部变量（不再污染全局 gyro/acc），SPI 访问持 imuMutex。
// 标定完成后清空在线微调量并重新计数，避免"旧 trim + 新 bias"叠加成双倍偏置。
// 安全约束：已解锁时拒绝执行——飞行中转一下机体就会把真实角速度写进零偏。
void calibrateGyroOnce() {
    if (!imuOK) return;
    if (armed) { print("已解锁状态禁止标定陀螺，请先 disarm\n"); return; }
    
    print("Calibrating gyro once... keep still\n");
    Vector sum(0, 0, 0);
    int samples = 200;
    
    for (int i = 0; i < samples; i++) {
        Vector g, a;
        imuLock();
        mpu9250_read_data(&g, &a);
        imuUnlock();
        sum += g;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
    // gyroBias 与 gyroBiasTrim 都是多字结构，控制任务的 readIMU() 同时在读，
    // 必须整组在锁内改，否则会出现"新 bias + 旧 trim"的撕裂组合。
    imuLock();
    gyroBias = sum / samples;
    resetGyroBiasTracker();
    imuUnlock();

    print("Gyro bias: %f %f %f (rad/s) = %.3f %.3f %.3f (deg/s)\n",
          gyroBias.x, gyroBias.y, gyroBias.z,
          degrees(gyroBias.x), degrees(gyroBias.y), degrees(gyroBias.z));
}

// 加速度计标定：水平静止采样 500 次求均值，减去重力得到 accBias。
// 注意：会阻塞约 3s + 采样时间；仅限地面调用。假定机体水平，未做六面标定。
// 多任务改造：采样写入局部变量（不再与控制任务的 readIMU 争写全局 gyro/acc）。
// 安全约束：已解锁时拒绝执行——accBias 直接参与姿态修正与 velZ 积分，
//           飞行中写坏它等于给估计器灌一个恒定假加速度。
void calibrateAccel() {
    if (!imuOK) return;
    if (armed) { print("已解锁状态禁止标定加速度计，请先 disarm\n"); return; }
    
    print("Calibrating accel...\n");
    print("Place drone on flat surface and keep still...\n");
    
    print("Starting calibration in 3 seconds...\n");
    vTaskDelay(pdMS_TO_TICKS(3000)); // 留出时间摆平并静置
    
    Vector sum(0, 0, 0);
    int samples = 500;
    
    for (int i = 0; i < samples; i++) {
        Vector g, a;
        imuLock();
        mpu9250_read_data(&g, &a);
        imuUnlock();
        sum += a;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    
    imuLock();
    accBias = sum / samples - Vector(0, 0, ONE_G); // 减去重力（Z 轴）
    imuUnlock();
    print("Accel bias: %f %f %f\n", accBias.x, accBias.y, accBias.z);
}

// 与 calibrateAccel 相同的采样逻辑，但不等待 3 秒（供上电自动标定流程使用）。
// ⚠️ 当前**没有任何调用者**（setupIMU 只自动标陀螺，不自动标加计），属死代码；
//    保留是为了将来做"上电自动六面/水平标定"时有现成入口。同样加了 armed 保护。
void calibrateAccelOnce() {
    if (!imuOK) return;
    if (armed) { print("已解锁状态禁止标定加速度计，请先 disarm\n"); return; }
    
    print("Calibrating accel once...\n");
    Vector sum(0, 0, 0);
    int samples = 500;
    
    for (int i = 0; i < samples; i++) {
        Vector g, a;
        imuLock();
        mpu9250_read_data(&g, &a);
        imuUnlock();
        sum += a;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    
    imuLock();
    accBias = sum / samples - Vector(0, 0, ONE_G);
    imuUnlock();
    print("Accel bias: %f %f %f\n", accBias.x, accBias.y, accBias.z);
}

// ============================================================
// 六面加速度计校准（CLI `ca`）：六个正交姿态各采样一次，
// 每轴用 +g/-g 两面解出零偏与比例因子：
//   scale = (m+ - m-) / 2g      bias = (m+ + m-) / 2
// 比单位置法（现 `ca1`）多出比例校正：单位置只能校零偏，
// 芯片灵敏度误差(典型±1~3%)它无能为力，六面法一并解决。
// 前提：IMU 安装旋转参数 IMU_ROT_* 为 0（默认）。若配置过非 90°
// 倍数的旋转，六个摆放面与传感器轴不再对齐，本方法不成立。
// 采样逐帧持锁读 SPI（与控制任务 readIMU 互斥）；六面全部算完
// 后在锁内一次性写入 accBias/accScale，避免撕裂组合。
// 完整性校验：任一面模长偏离 1g 超 1.5 m/s²（摆放不到位/被干扰）
// 或某轴比例超 [0.70, 1.30] → 整体中止，不写入任何值。
// 全程约 6×(8s+1s) ≈ 1 分钟，期间阻塞通信任务（仅限地面未解锁使用）。
// ============================================================
#define ACCEL6_SETTLE_MS  8000  // 每面静置等待，给用户时间翻转摆放
#define ACCEL6_SAMPLES    500   // 每面采样次数（2ms 间隔 ≈ 1 秒）
#define ACCEL6_NORM_TOL   1.5f  // 面模长偏离 1g 的容忍度 (m/s²)
#define ACCEL6_SCALE_MIN  0.70f // 比例因子合理下限（正常应接近 1）
#define ACCEL6_SCALE_MAX  1.30f

void calibrateAccel6() {
    if (!imuOK) { print("IMU not initialized\n"); return; }
    if (armed) { print("已解锁状态禁止校准加速度计，请先 disarm\n"); return; }

    static const char* faces[6] = {
        "水平放置：机头朝前（正常飞行方向），底部朝下水平放在平坦表面，确保完全水平无倾斜",
        "机头朝上：保持机头朝前，将机头指向天空，尾部朝下并接触支撑面，与地面垂直",
        "机头朝下：保持机头朝前，将机头指向地面并接触支撑面，尾部朝上，与地面垂直",
        "右侧朝下：保持机头朝前，将飞行器右侧朝下接触支撑面，左侧机臂朝上，与地面垂直",
        "左侧朝下：保持机头朝前，将飞行器左侧朝下接触支撑面，右侧机臂朝上，与地面垂直",
        "倒置放置：保持机头朝前，将飞行器完全翻转，顶部朝下、底部朝上，整体呈水平倒置",
    };
    static const char* faceDone[6] = {
        "水平", "机头向上", "机头向下", "右侧向下", "左侧向下", "倒置",
    };

    print("===== 加速度计六面校准开始（共 6 面，每面静置 %d 秒）=====\n", ACCEL6_SETTLE_MS / 1000);
    Vector m[6];
    for (int i = 0; i < 6; i++) {
        print("%d/6 %s\n保持不动，%d 秒后开始校准；\n", i + 1, faces[i], ACCEL6_SETTLE_MS / 1000);
        vTaskDelay(pdMS_TO_TICKS(ACCEL6_SETTLE_MS));

        Vector sum(0, 0, 0);
        for (int s = 0; s < ACCEL6_SAMPLES; s++) {
            Vector g, a;
            imuLock();
            mpu9250_read_data(&g, &a);
            imuUnlock();
            sum += a;
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        m[i] = sum / ACCEL6_SAMPLES;
        print("%s校准完成。  [均值 %7.3f %7.3f %7.3f]\n", faceDone[i], m[i].x, m[i].y, m[i].z);
    }

    // 完整性校验：每个姿态下合加速度模长都应接近 1g
    for (int i = 0; i < 6; i++) {
        float n = m[i].norm();
        if (fabsf(n - ONE_G) > ACCEL6_NORM_TOL) {
            print("✗ 第 %d 面模长 %.2f 偏离 1g 过大（摆放不到位或采样被晃动），中止，未写入任何标定值\n", i + 1, n);
            return;
        }
    }

    // 每轴解算：在六个面均值中找该轴的 +g 面与 -g 面
    Vector bias(0, 0, 0), scale(1, 1, 1);
    float axisVal[6];
    for (int axis = 0; axis < 3; axis++) {
        for (int i = 0; i < 6; i++)
            axisVal[i] = (axis == 0) ? m[i].x : (axis == 1) ? m[i].y : m[i].z;
        int pIdx = 0, nIdx = 0;
        for (int i = 1; i < 6; i++) {
            if (axisVal[i] > axisVal[pIdx]) pIdx = i;
            if (axisVal[i] < axisVal[nIdx]) nIdx = i;
        }
        float s = (axisVal[pIdx] - axisVal[nIdx]) / (2.0f * ONE_G);
        float b = (axisVal[pIdx] + axisVal[nIdx]) * 0.5f;
        if (s < ACCEL6_SCALE_MIN || s > ACCEL6_SCALE_MAX) {
            print("✗ %c 轴比例 %.3f 超出 [%.2f, %.2f]，数据不可信，中止，未写入任何标定值\n",
                  "XYZ"[axis], s, ACCEL6_SCALE_MIN, ACCEL6_SCALE_MAX);
            return;
        }
        if (axis == 0)      { scale.x = s; bias.x = b; }
        else if (axis == 1) { scale.y = s; bias.y = b; }
        else                { scale.z = s; bias.z = b; }
    }

    imuLock();
    accBias = bias;
    accScale = scale;
    imuUnlock();

    print("✓ 六面校准全部完成！\n");
    print("Accel bias:  %7.4f %7.4f %7.4f (m/s^2)\n", bias.x, bias.y, bias.z);
    print("Accel scale: %7.4f %7.4f %7.4f\n", scale.x, scale.y, scale.z);
    print("将机身放正，执行 ps 查看水平结果（roll/pitch 应在 ±1° 内）\n");
}

// 打印当前 IMU 标定值（零偏、在线微调量与比例因子）
void printIMUCalibration() {
    print("Gyro bias: %f %f %f\n", gyroBias.x, gyroBias.y, gyroBias.z);
    print("Gyro trim: %f %f %f (online, static-only)\n", gyroBiasTrim.x, gyroBiasTrim.y, gyroBiasTrim.z);
    print("Accel bias: %f %f %f\n", accBias.x, accBias.y, accBias.z);
    print("Accel scale: %f %f %f\n", accScale.x, accScale.y, accScale.z);
}

// 打印 IMU 状态：WHO_AM_I 与一帧原始读数（CLI 命令 imu）
void printIMUInfo() {
    if (!imuOK) {
        print("IMU not initialized\n");
        return;
    }
    
    imuLock();
    uint8_t whoami = mpu9250_read_reg(MPU_REG_WHO_AM_I);
    Vector dummyGyro, dummyAcc;
    mpu9250_read_data(&dummyGyro, &dummyAcc);
    imuUnlock();
    print("MPU9250 WHO_AM_I: 0x%02X\n", whoami);
    // 注意：Vector 分量是 float，必须用 %f（原来的 %d 属于未定义行为，会打印垃圾值）
    print("Raw gyro (rad/s): %f %f %f\n", dummyGyro.x, dummyGyro.y, dummyGyro.z);
    print("Raw accel (m/s^2): %f %f %f\n", dummyAcc.x, dummyAcc.y, dummyAcc.z);
}

// ============================================================
// 振动自检（CLI `vib [秒]`，默认 2s，上限 10s）
// 目的：把"振动大不大"变成可对比的数字，方便换桨 / 加减震 / 紧螺丝前后各测一次。
// 采样对象是控制环真正使用的 gyro / acc（已去偏、已对齐机体），200Hz × N 秒。
// 输出 4 行：
//   第1行 工况（时长、推力、四路电机）
//   第2行 acc 模长 min/max/峰峰值/标准差 —— 振动强度。参考：拆桨 31% 油门 pp≈4.1、std≈1.1
//   第3行 gyro 三轴 std（振动噪声）与 mean（残余零偏），单位 deg/s
//   第4行 yaw 起止与漂移率 —— 最直接的"会不会自己转"
// 注意：
//   - 本函数阻塞（vTaskDelay），但控制环照常运行，已解锁时也可用；
//   - 数据同时保存，浏览器打开 http://192.168.4.1/vib.txt 可滚动/长按复制/整页截图，
//     绕开"网页控制台复制不了"的问题；
//   - std 只反映交流分量；mean 才是直流偏置，两者要分开看。
// ============================================================
static char vibReport[512] = "no data (run 'vib' first)\n";

const char* getVibReport() { return vibReport; }

void measureVibration(int seconds) {
    if (!imuOK) { print("IMU not initialized\n"); return; }
    if (seconds <= 0) seconds = 2;
    if (seconds > 10) seconds = 10;

    const int periodMs = 5;                                  // 200Hz：足够反映振动包络
    const int samples = seconds * 1000 / periodMs;

    float aMin = 1e9f, aMax = -1e9f, aSum = 0.0f, aSum2 = 0.0f;
    Vector gSum(0, 0, 0), gSum2(0, 0, 0);
    const Vector euler0 = attitude.toEuler();
    const float thrAtStart = thrustTarget;

    print("VIB measuring %ds ... keep the same condition\n", seconds);

    int n = 0;
    for (int i = 0; i < samples; i++) {
        Vector g, a;
        imuLock();          // Vector 是三 float，跨任务读要防撕裂
        g = gyro;
        a = acc;
        imuUnlock();

        const float an = a.norm();
        if (an < aMin) aMin = an;
        if (an > aMax) aMax = an;
        aSum += an;
        aSum2 += an * an;
        gSum += g;
        gSum2 += Vector(g.x * g.x, g.y * g.y, g.z * g.z);
        n++;
        vTaskDelay(pdMS_TO_TICKS(periodMs));
    }
    if (n <= 0) { print("VIB: no samples\n"); return; }

    const float nf = (float)n;
    const Vector euler1 = attitude.toEuler();
    const float aMean = aSum / nf;
    const float aStd = sqrtf(fmaxf(aSum2 / nf - aMean * aMean, 0.0f)); // 方差可能被浮点误差压成负

    const Vector gMean = gSum / nf;
    const Vector gStd(sqrtf(fmaxf(gSum2.x / nf - gMean.x * gMean.x, 0.0f)),
                      sqrtf(fmaxf(gSum2.y / nf - gMean.y * gMean.y, 0.0f)),
                      sqrtf(fmaxf(gSum2.z / nf - gMean.z * gMean.z, 0.0f)));
    const float yawDrift = degrees(wrapAngle(euler1.z - euler0.z)) / (float)seconds;

    snprintf(vibReport, sizeof(vibReport),
             "VIB %ds thr=%.2f mot %.2f/%.2f/%.2f/%.2f\n"
             "acc |a| min=%.2f max=%.2f pp=%.2f std=%.2f mean=%.2f\n"
             "gyro std x=%.1f y=%.1f z=%.1f | mean x=%.1f y=%.1f z=%.1f (deg/s)\n"
             "yaw %.1f -> %.1f  drift=%.2f deg/s\n",
             seconds, thrAtStart,
             motors[MOTOR_REAR_LEFT], motors[MOTOR_REAR_RIGHT],
             motors[MOTOR_FRONT_RIGHT], motors[MOTOR_FRONT_LEFT],
             aMin, aMax, aMax - aMin, aStd, aMean,
             degrees(gStd.x), degrees(gStd.y), degrees(gStd.z),
             degrees(gMean.x), degrees(gMean.y), degrees(gMean.z),
             degrees(euler0.z), degrees(euler1.z), yawDrift);

    print("%s", vibReport);
    print("(纯文本副本: http://192.168.4.1/vib.txt)\n");
}
