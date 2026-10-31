// ============================================================================
// mpu9250.h —— MPU9250/MPU6500 驱动接口与寄存器/量程定义
//
// 职责:
//   提供寄存器地址宏、量程（FSR）选项宏，以及 SPI 读写 / 配置 / 数据读取等接口声明。
//   供 imu.cpp 调用。
//
// 关键逻辑:
//   寄存器地址均为 7 位；SPI 传输时最高位区分读写（MSB=1 读，MSB=0 写）。
//   量程宏直接对应 GYRO_CONFIG / ACCEL_CONFIG 的低位编码，灵敏度 = 32768 / 量程。
//
// 输入/输出:
//   mpu9250_read_data() 输出 gyro(rad/s) 与 acc(m/s²)；
//   其余接口返回寄存器值或执行状态（部分为空实现）。
//
// 边界情况与潜在风险:
//   - 灵敏度换算（在 .cpp 中）必须与 configure 时选用的 FSR 宏保持一一对应。
//   - 本项目实际芯片为 MPU6500（WHO_AM_I=0x70），与 MPU9250 寄存器兼容，
//     但无内置磁力计（AK8963），相关寄存器/宏仅为兼容保留。
//   - 安装旋转不在本层处理，由 imu.cpp 依据参数 IMU_ROT_* 施加（见文件末尾说明）。
// ============================================================================

#pragma once

#include "vector.h"
#include "quaternion.h"
#include "board_config.h"

// 寄存器地址（均为 7 位地址；SPI 传输时最高位用于区分读/写：MSB=1 读，MSB=0 写）
#define MPU_REG_WHO_AM_I       0x75 // 芯片身份寄存器：只读，MPU9250 典型值 0x71（部分 0x73）
#define MPU_REG_PWR_MGMT_1     0x6B // 电源管理 1：选择时钟源、进入/退出睡眠模式
#define MPU_REG_PWR_MGMT_2     0x6C // 电源管理 2：逐轴使能加速度计/陀螺仪（0x00 = 全部开启）
#define MPU_REG_SMPLRT_DIV     0x19 // 采样率分频：DLPF 开启时 采样率 = 1kHz / (值 + 1)
#define MPU_REG_CONFIG         0x1A // 配置寄存器：bits[2:0] = 陀螺仪 DLPF_CFG（带宽/延迟）
#define MPU_REG_GYRO_CONFIG    0x1B // 陀螺仪配置：bits[4:3] 满量程 FSR，bits[1:0] FCHOICE_B（00=DLPF 开）
#define MPU_REG_ACCEL_CONFIG   0x1C // 加速度计配置：bits[4:3] 满量程 AFS_SEL
#define MPU_REG_ACCEL_CONFIG_2 0x1D // 加速度计配置 2：bit3 = accel_fchoice_b（0=DLPF 开），bits[2:0] = A_DLPF_CFG
#define MPU_REG_FIFO_EN        0x23 // FIFO 使能：选择哪些传感器数据写入 FIFO
#define MPU_REG_INT_PIN_CFG    0x37 // 中断引脚配置；也含旁路 AK8963（磁力计）I2C 的位
#define MPU_REG_USER_CTRL      0x6A // 用户控制：FIFO / I2C 主机 / SPI 模式使能
#define MPU_REG_FIFO_COUNT     0x72 // FIFO 计数（高字节）：当前 FIFO 中的有效字节数
#define MPU_REG_FIFO_R_L       0x74 // FIFO 读数据寄存器
#define MPU_REG_ACCEL_XOUT_H   0x3B // 加速度计数据起始寄存器（X 高字节）：连续 6 字节为 X/Y/Z
#define MPU_REG_GYRO_XOUT_H    0x43 // 陀螺仪数据起始寄存器（X 高字节）：连续 6 字节为 X/Y/Z

// 陀螺仪满量程范围（FSR）选项 —— 写入 MPU_REG_GYRO_CONFIG 的低 2 位
// 灵敏度 = 32768 / 量程，用于把原始计数换算成 deg/s
#define MPU9250_GYRO_FSR_250   0x00  // +/- 250 deg/s  -> 131.0 LSB/(deg/s)
#define MPU9250_GYRO_FSR_500   0x08  // +/- 500 deg/s  -> 65.5  LSB/(deg/s)
#define MPU9250_GYRO_FSR_1000  0x10  // +/- 1000 deg/s -> 32.8  LSB/(deg/s)
#define MPU9250_GYRO_FSR_2000  0x18  // +/- 2000 deg/s -> 16.4  LSB/(deg/s)

// 加速度计满量程范围（FSR）选项 —— 写入 MPU_REG_ACCEL_CONFIG 的低 2 位
// 灵敏度 = 32768 / 量程，用于把原始计数换算成 g
#define MPU9250_ACCEL_FSR_2G   0x00  // +/- 2g  -> 16384 LSB/g
#define MPU9250_ACCEL_FSR_4G   0x08  // +/- 4g  -> 8192  LSB/g
#define MPU9250_ACCEL_FSR_8G   0x10  // +/- 8g  -> 4096  LSB/g
#define MPU9250_ACCEL_FSR_16G  0x18  // +/- 16g -> 2048  LSB/g

// 陀螺仪 DLPF_CFG —— 写入 MPU_REG_CONFIG 的 bits[2:0]（数据手册 Register Map 表 4.3）
// DLPF 开启时内部采样 1kHz，输出率再被 SMPLRT_DIV 分频
#define MPU9250_DLPF_250HZ  0x00  // 带宽 250Hz，延迟 0.97ms（输出率变为 8kHz/(1+DIV)）
#define MPU9250_DLPF_184HZ  0x01  // 带宽 184Hz，延迟 2.9ms
#define MPU9250_DLPF_92HZ   0x02  // 带宽  92Hz，延迟 3.9ms
#define MPU9250_DLPF_41HZ   0x03  // 带宽  41Hz，延迟 5.9ms   <- 本项目使用
#define MPU9250_DLPF_20HZ   0x04  // 带宽  20Hz，延迟 9.9ms
#define MPU9250_DLPF_10HZ   0x05  // 带宽  10Hz，延迟 17.85ms
#define MPU9250_DLPF_5HZ    0x06  // 带宽   5Hz，延迟 33.48ms  <- 原错误配置（相位延迟过大，姿态环无法工作）
#define MPU9250_DLPF_3600HZ 0x07  // 带宽 3600Hz，延迟 0.17ms（输出率变为 8kHz/(1+DIV)）

// 加速度计 A_DLPF_CFG —— 写入 MPU_REG_ACCEL_CONFIG_2 的 bits[2:0]（accel_fchoice_b=0 时生效）
// 括号内为对应的 3dB 截止频率
#define MPU9250_ADLPF_460HZ 0x00  // 采样 460Hz (250Hz)，延迟 6.00ms <- 寄存器默认值（原代码未写此寄存器）
#define MPU9250_ADLPF_184HZ 0x01  // 采样 184Hz (120Hz)，延迟 5.80ms
#define MPU9250_ADLPF_92HZ  0x02  // 采样  92Hz  (60Hz)，延迟 7.80ms
#define MPU9250_ADLPF_41HZ  0x03  // 采样  41Hz  (27Hz)，延迟 11.90ms <- 本项目使用
#define MPU9250_ADLPF_20HZ  0x04  // 采样  20Hz (13.6Hz)，延迟 17.80ms
#define MPU9250_ADLPF_10HZ  0x05  // 采样  10Hz    (7Hz)，延迟 35.70ms
#define MPU9250_ADLPF_5HZ   0x06  // 采样   5Hz  (3.4Hz)，延迟 66.96ms

void mpu9250_init();                             // 初始化 SPI 总线与设备（CS/SCK/MOSI/MISO）并完成默认配置
uint8_t mpu9250_read_reg(uint8_t reg);           // 读单个寄存器：reg 的低 7 位为地址，返回该寄存器值
void mpu9250_write_reg(uint8_t reg, uint8_t value); // 写单个寄存器：把 value 写入地址为 reg 的寄存器
void mpu9250_read_data(Vector *gyro, Vector *acc);  // 读 accel/gyro 并换算：acc 单位 m/s^2，gyro 单位 rad/s
void mpu9250_configure(int gyroFSR, int accelFSR);  // 配置采样率、量程、DLPF 并唤醒设备（参数取上面的 FSR 宏）
// 注：安装旋转（传感器坐标系 -> 机体坐标系）由 imu.cpp 依据参数 IMU_ROT_* 统一施加，
//     本层不再提供 mpu9250_set_rotation()，避免"看似生效实则空转"的误导接口。
