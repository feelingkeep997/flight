// ============================================================================
// mpu9250.cpp —— MPU9250/MPU6500 寄存器层驱动（SPI）
//
// 职责:
//   直接通过 SPI 访问 MPU9250 的寄存器：初始化总线、读/写单寄存器、
//   突发读取加速度计+陀螺仪原始数据并换算成物理单位、配置量程/采样率/DLPF。
//
// 关键逻辑:
//   - MPU9250 的 SPI 时序：首字节为地址，最高位=1 表示读、=0 表示写；
//     读操作需发 1 个地址字节 + 1 个哑元字节，返回值在第二个字节；
//   - 传感器数据是一段连续寄存器块（ACCEL_XOUT_H 起 14 字节），
//     必须在一次 CS 拉低期间突发读完，否则数据会错位；
//   - 数据为大端序 16 位补码，按当前 FSR 灵敏度换算：
//     ±16g → 2048 LSB/g；±1000dps → 32.8 LSB/(deg/s)。
//
// 输入/输出:
//   输入：SPI 硬件（引脚由 board_config.h 的 BOARD_SPI_* 定义）。
//   输出：gyro（rad/s）、acc（m/s²），以及各配置寄存器的读写结果。
//
// 重要参数:
//   gyroFSR/accelFSR 取 mpu9250.h 中的 MPU9250_*_FSR_* 宏；
//   SMPLRT_DIV=0 → 1kHz 输出；CONFIG=MPU9250_DLPF_41HZ → 陀螺 41Hz/5.9ms；
//   ACCEL_CONFIG_2=MPU9250_ADLPF_41HZ → 加速度 41Hz/11.9ms。
//
// 边界情况与潜在风险:
//   - 灵敏度常数必须与 configure 时写入的 FSR 一致，否则换算出的物理量整体偏大/偏小。
//   - 加速度数据在 buffer[0..5]、陀螺在 buffer[8..13]（buffer[6..7] 是温度），
//     下标写错会读到温度或错位（注释中已标出历史坑点）。
//   - 安装旋转不在本层处理：由 imu.cpp 依据参数 IMU_ROT_* 施加（曾经的
//     mpu9250_set_rotation() 是只打日志的空壳，已删除以免误导）。
//   - 单寄存器读写每次都是一次 SPI 传输，频繁调用开销较大；读取数据请用突发接口。
//   - 若 SPI 初始化/挂载设备失败只打印错误不返回状态，调用方无法直接感知。
// ============================================================================

#include "mpu9250.h"
#include "board_config.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "mpu9250";

// 注意：寄存器地址宏（MPU_REG_*）与量程宏（MPU9250_*_FSR_*）统一由头文件 mpu9250.h 提供，此处不再重复定义

// Device handle
static spi_device_handle_t spiHandle = nullptr;

void mpu9250_init() {
    // 选择 SPI 主机：ESP32-S3 用 SPI2_HOST，其它芯片（如 ESP32）用 HSPI_HOST
#if CONFIG_IDF_TARGET_ESP32S3
    spi_host_device_t spiHost = SPI2_HOST;
#else
    spi_host_device_t spiHost = HSPI_HOST;
#endif
    
    // SPI 总线配置：指定 MOSI/MISO/SCK 三个引脚（来自板级配置宏）
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = BOARD_SPI_MOSI;   // 主出从入数据线引脚
    buscfg.miso_io_num = BOARD_SPI_MISO;   // 主入从出数据线引脚
    buscfg.sclk_io_num = BOARD_SPI_SCK;    // 时钟线引脚
    buscfg.quadwp_io_num = -1;             // 不使用 QSPI 写保护引脚
    buscfg.quadhd_io_num = -1;             // 不使用 QSPI 保持引脚
    buscfg.max_transfer_sz = 256;          // 单次传输最大字节数
    
    // 初始化 SPI 总线，使用自动分配的 DMA 通道
    esp_err_t ret = spi_bus_initialize(spiHost, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI");
        return;
    }
    
    // 单个 SPI 设备（MPU9250）的接口配置
    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = 10 * 1000 * 1000; // 通信速率 10 MHz
    devcfg.spics_io_num = BOARD_SPI_CS;        // 片选 CS 引脚（由硬件自动拉低/拉高）
    devcfg.queue_size = 1;                     // 传输队列深度为 1
    devcfg.flags = SPI_DEVICE_NO_DUMMY;        // 不在读操作前插入空时钟周期
    
    // 将 MPU9250 设备挂载到 SPI 总线，拿到设备句柄 spiHandle
    ret = spi_bus_add_device(spiHost, &devcfg, &spiHandle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device");
        return;
    }
    
    ESP_LOGI(TAG, "MPU9250 initialized on SPI");
}

// 读取 MPU9250 单个寄存器
// 参数 reg: 要读取的寄存器地址（只取低 7 位，最高位会被置 1 表示"读"）
// 返回: 该寄存器的值
uint8_t mpu9250_read_reg(uint8_t reg) {
    spi_transaction_t trans = {};
    // cmd[0]: 读指令 = 寄存器地址且最高位(第 7 位)置 1（MPU9250 SPI 规定 MSB=1 为读）
    // cmd[1]: 占位字节，读操作期间主机发出的哑元数据，从机在其后返回真值
    uint8_t cmd[2] = {(uint8_t)(reg | 0x80), 0};
    // 接收缓冲区必须与发送等长（2 字节），否则驱动会因 rxlength=16 越界写入 1 字节
    // resp[0] 是地址字节期间 MISO 上的无效数据，真正的寄存器值在 resp[1]
    uint8_t resp[2] = {0, 0};
    
    trans.tx_buffer = cmd;  // 发送缓冲区：读指令(2 字节)
    trans.rx_buffer = resp; // 接收缓冲区：2 字节（与发送缓冲区长度一致）
    trans.length = 16;   // 总传输位数 16 = 8 位指令 + 8 位返回数据（全双工，收发同时进行）
    trans.rxlength = 16; // 接收位数 16（与 length 一致，读全程都在收）
    
    // 执行本次 SPI 传输（硬件会自动拉低/拉高 CS）
    spi_device_transmit(spiHandle, &trans);
    
    return resp[1]; // 返回读到的寄存器内容（跳过地址阶段的无效字节）
}

// 向 MPU9250 单个寄存器写入一个字节
// 参数 reg:   目标寄存器地址（只取低 7 位，最高位会被清 0 表示"写"）
// 参数 value: 要写入寄存器的值
// MPU9250该怎么工作
void mpu9250_write_reg(uint8_t reg, uint8_t value) {
    spi_transaction_t trans = {};
    // cmd[0]: 写指令 = 寄存器地址且最高位(第 7 位)清 0（MPU9250 SPI 规定 MSB=0 为写）
    // cmd[1]: 紧随其后的就是要写入的数据字节，从机在收到该字节时将其锁存到 reg
    uint8_t cmd[2] = {(uint8_t)(reg & 0x7F), value};

    trans.tx_buffer = cmd; // 发送缓冲区：写指令 + 数据（2 字节）
    // 写操作不需要接收从机数据，因此不设置 rx_buffer（MISO 上的字节会被丢弃）
    trans.length = 16; // 总传输位数 16 = 8 位指令 + 8 位数据

    // 执行本次 SPI 传输（硬件会自动拉低/拉高 CS）
    spi_device_transmit(spiHandle, &trans);
}

// 一次性读取加速度计 + 陀螺仪的原始采样值并换算成物理单位
// 参数 gyro: 输出陀螺仪角速度，单位 rad/s
// 参数 acc:  输出加速度计加速度，单位 m/s^2
void mpu9250_read_data(Vector *gyro, Vector *acc) {
    // MPU9250 的传感器数据是连续的寄存器块，从 ACCEL_XOUT_H(0x3B) 开始依次排列：
    //   [0-1]ACCEL_X [2-3]ACCEL_Y [4-5]ACCEL_Z [6-7]TEMP
    //   [8-9]GYRO_X [10-11]GYRO_Y [12-13]GYRO_Z
    //   加速度 6 字节 + 温度 2 字节 + 陀螺仪 6 字节 = 共 14 字节
    // 这里只读前 14 字节（6 accel + 6 gyro + 2 temp），温度暂不使用 


    spi_transaction_t trans = {};

    // 突发读要求 CS 全程保持低电平，因此"指令 + 数据"必须在同一次传输中完成
    // 发送缓冲区：第 0 字节为读指令（最高位置 1 表示"读"，低 7 位为起始寄存器地址，
    //            从机收到后会从该地址起自动递增输出），其后 14 字节输出哑元，仅为产生时钟
    uint8_t tx_buf[15] = {0};
    tx_buf[0] = (uint8_t)(MPU_REG_ACCEL_XOUT_H | 0x80);

    // 接收缓冲区：第 0 字节是传指令期间 MISO 上的无效字节，真实数据从下标 1 开始
    uint8_t rx_buf[15] = {0};

    trans.tx_buffer = tx_buf; // 发送缓冲区：15 字节
    trans.rx_buffer = rx_buf; // 接收缓冲区：15 字节
    trans.length = 15 * 8;    // 发送位数 = 15 字节 * 8 位
    trans.rxlength = 15 * 8;  // 接收位数与发送位数一致（全程都在收）

    spi_device_transmit(spiHandle, &trans); // 单次传输，CS 全程保持低电平

    const uint8_t *buffer = rx_buf + 1; // 跳过指令字节，buffer[0..13] 即传感器数据
    
    // 解析加速度计：每轴 2 字节，大端序（高字节在前），组合成有符号 16 位补码
    // 先提升为 uint16_t 再左移，避免有符号移位/符号扩展带来的问题
    int16_t ax = (int16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
    int16_t ay = (int16_t)(((uint16_t)buffer[2] << 8) | buffer[3]);
    int16_t az = (int16_t)(((uint16_t)buffer[4] << 8) | buffer[5]);

    // 解析陀螺仪：同样大端序 16 位（buffer[6]、buffer[7] 为温度，未使用）
    // 注意：陀螺仪数据从下标 8 开始，此前写成 6 会导致读到温度/错位
    int16_t gx = (int16_t)(((uint16_t)buffer[8] << 8) | buffer[9]);
    int16_t gy = (int16_t)(((uint16_t)buffer[10] << 8) | buffer[11]);
    int16_t gz = (int16_t)(((uint16_t)buffer[12] << 8) | buffer[13]);

    // 换算为物理单位（灵敏度必须与 mpu9250_configure 实际写入的 FSR 一致）
    // Accel: +/- 16g      -> 2048 LSB/g            （对应 MPU9250_ACCEL_FSR_16G）
    // Gyro:  +/- 1000deg/s -> 32.8 LSB/(deg/s)     （对应 MPU9250_GYRO_FSR_1000）
    // 先把原始计数除以灵敏度得到 g，再由 1g = 9.81 m/s^2 转成国际单位
    acc->x = ax / 2048.0f * 9.81f; // m/s^2
    acc->y = ay / 2048.0f * 9.81f;
    acc->z = az / 2048.0f * 9.81f;

    // 先把原始计数除以灵敏度得到 deg/s，再乘 π/180 转成 rad/s
    gyro->x = gx / 32.8f * M_PI / 180.0f; // rad/s
    gyro->y = gy / 32.8f * M_PI / 180.0f;
    gyro->z = gz / 32.8f * M_PI / 180.0f;
}

// 配置 MPU9250 的工作参数：唤醒/时钟、采样率、量程、陀螺与加速度 DLPF
// 参数 gyroFSR:  陀螺仪满量程选择（见头文件 MPU9250_GYRO_FSR_*）
// 参数 accelFSR: 加速度计满量程选择（见头文件 MPU9250_ACCEL_FSR_*）
//
// 配置目标（与 1kHz 控制环匹配）：
//   SMPLRT_DIV = 0        -> 传感器输出率 1kHz（DLPF 开启时内部即 1kHz，不再分频）
//   陀螺  DLPF_CFG = 3    -> 带宽 41Hz，群延迟 5.9ms
//   加计  A_DLPF_CFG = 3  -> 采样 41Hz(3dB 27Hz)，群延迟 11.9ms
//
// 修改前的错误配置（务必不要改回去）：
//   SMPLRT_DIV = 7 -> 输出率只有 125Hz，控制环 1kHz 读到的是同一帧的 8 次重复；
//   CONFIG = 0x06  -> 陀螺 DLPF 5Hz / 延迟 33.48ms（注释里写的"约21Hz"在数据手册中不存在），
//                     33ms 的相位延迟相当于姿态环滞后 33 个控制周期，P=6 的外环必然振荡发散；
//   ACCEL_CONFIG_2 从未写入 -> 保持寄存器默认 0，即加速度计跑在 460Hz/6ms，
//                     电机振动几乎不滤波就进了积分，注释里"约21Hz 带宽"同样是错的。
//
// 写入顺序：先唤醒并选好时钟，再写各配置寄存器。芯片复位后处于 SLEEP 状态，
//   虽然睡眠下寄存器仍可写，但按数据手册推荐顺序（PWR_MGMT_1 先行）更稳妥。
void mpu9250_configure(int gyroFSR, int accelFSR) {
    // 电源管理 1：退出睡眠 + 选择 X 轴陀螺 PLL 作为时钟源（比内部 RC 振荡器稳定得多）
    mpu9250_write_reg(MPU_REG_PWR_MGMT_1, 0x01);
    vTaskDelay(pdMS_TO_TICKS(10)); // 等 PLL 锁定后再写配置

    // 电源管理 2：0x00 = 加速度计与陀螺仪三轴全部使能（复位默认值，此处显式写出以防被改过）
    mpu9250_write_reg(MPU_REG_PWR_MGMT_2, 0x00);

    // 采样率分频：DLPF 开启时输出率 = 1kHz / (1 + SMPLRT_DIV)，取 0 即 1kHz，与控制环同频
    mpu9250_write_reg(MPU_REG_SMPLRT_DIV, 0x00);

    // 陀螺仪 DLPF：41Hz 带宽 / 5.9ms 延迟。小四轴没有软件陷波，靠这一级压电机噪声已足够，
    // 且延迟远小于 125Hz 级别的环路周期，不会给姿态环引入明显相位滞后。
    mpu9250_write_reg(MPU_REG_CONFIG, MPU9250_DLPF_41HZ);

    // 配置陀螺仪满量程范围（写入 GYRO_CONFIG bits[4:3]，bits[1:0] FCHOICE_B=00 表示 DLPF 开启）
    // FSR = +/- 250: 0x00, +/- 500: 0x08, +/- 1000: 0x10, +/- 2000: 0x18
    mpu9250_write_reg(MPU_REG_GYRO_CONFIG, gyroFSR);

    // 配置加速度计满量程范围（写入 ACCEL_CONFIG bits[4:3]）
    // FSR = +/- 2g: 0x00, +/- 4g: 0x08, +/- 8g: 0x10, +/- 16g: 0x18
    mpu9250_write_reg(MPU_REG_ACCEL_CONFIG, accelFSR);

    // 加速度计 DLPF：bit3 accel_fchoice_b = 0（DLPF 开启），bits[2:0] = A_DLPF_CFG
    // 取 41Hz：加速度只用于极慢的姿态修正(accWeight=0.003)、落地判据与 velZ 积分，
    // 这三者都对延迟不敏感、对振动噪声非常敏感，故与陀螺取同一带宽、优先压噪。
    mpu9250_write_reg(MPU_REG_ACCEL_CONFIG_2, MPU9250_ADLPF_41HZ);

    // 等待滤波器与内部状态稳定
    vTaskDelay(pdMS_TO_TICKS(100));

    // 读取 WHO_AM_I 寄存器校验芯片身份
    uint8_t whoami = mpu9250_read_reg(MPU_REG_WHO_AM_I);
    // 0x71/0x73 = MPU9250/MPU9255，0x70 = MPU6500（三者加速度计/陀螺仪寄存器兼容）
    if (whoami == 0x71 || whoami == 0x73 || whoami == 0x70) {
        ESP_LOGI(TAG, "MPU9250/MPU6500 detected: WHO_AM_I=0x%02X", whoami);
    } else {
        // 读到非预期值说明未连接、接线错误、SPI 通信失败或芯片型号不符
        ESP_LOGE(TAG, "IMU not found! Expected 0x71/0x73 (MPU9250) or 0x70 (MPU6500), got 0x%02X", whoami);
    }

    // 诊断：回读刚写入的配置寄存器，验证 SPI 读写链路是否真正生效
    // 链路正常时应为 SMPLRT_DIV=0x00 CONFIG=0x03 GYRO=0x10 ACCEL=0x18 ACCEL2=0x03 PWR=0x01
    ESP_LOGI(TAG, "reg readback: DIV=0x%02X CONFIG=0x%02X GYRO=0x%02X ACCEL=0x%02X ACCEL2=0x%02X PWR=0x%02X",
             mpu9250_read_reg(MPU_REG_SMPLRT_DIV), mpu9250_read_reg(MPU_REG_CONFIG),
             mpu9250_read_reg(MPU_REG_GYRO_CONFIG), mpu9250_read_reg(MPU_REG_ACCEL_CONFIG),
             mpu9250_read_reg(MPU_REG_ACCEL_CONFIG_2), mpu9250_read_reg(MPU_REG_PWR_MGMT_1));
}
