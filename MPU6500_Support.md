# MPU6500 支持说明

## 硬件确认

本项目已更新为支持 **MPU6500** (MPU-6500) IMU 模块，适用于 **ESP32 30P** 开发板。

### 主要组件

1. **ESP32 30P 开发板** - 基于 ESP32-S3 芯片
2. **MPU6500 模块** - 9 轴运动处理传感器（陀螺仪 + 加速度计）
3. **ME6118A33PG** - 3.3V 低压差线性稳压器

## MPU6500 vs MPU6050 vs MPU9250

| 特性 | MPU6050 | MPU6500 | MPU9250 |
|------|---------|---------|---------|
| WHO_AM_I | 0x68 | **0x71/0x70** | 0x71/0x73 |
| 陀螺仪范围 | ±250/500/1000/2000 dps | **±250/500/1000/2000 dps** | ±250/500/1000/2000 dps |
| 加速度计范围 | ±2/4/8/16g | **±2/4/8/16g** | ±2/4/8/16g |
| 磁力计 | 无 | **无** | 有 (AK8963) |
| 功耗 | 低 | **更低** | 中 |
| 价格 | 低 | **中** | 高 |

## 代码变更

### 1. 驱动文件

- `components/drivers/mpu6500.cpp` - MPU6500 SPI 驱动实现
- `components/drivers/mpu6500.h` - 驱动头文件

### 2. IMU 传感器代码

- `components/sensors/imu.cpp` - 使用 MPU6500 API

关键检测代码：
```cpp
uint8_t whoami = mpu6500_read_reg(MPU_REG_WHO_AM_I);
if (whoami == 0x71 || whoami == 0x70) {
    print("MPU6500 detected: WHO_AM_I=0x%02X\n", whoami);
    imuOK = true;
}
```

### 3. 构建配置

- `components/drivers/CMakeLists.txt` - 指向 mpu6500.cpp
- `sdkconfig.defaults` - 目标芯片设置为 esp32s3

## 引脚配置 (ESP32-S3)

### SPI 连接 (默认 FSPI)

```
ESP32-S3      MPU6500
GPIO 12  →    SCK
GPIO 13  →    MISO  
GPIO 11  →    MOSI
GPIO 10  →    CS
```

这些引脚在 `board_config.h` 中定义：
```c
#define BOARD_SPI_SCK      12
#define BOARD_SPI_MISO     13
#define BOARD_SPI_MOSI     11
#define BOARD_SPI_CS       10
```

## 编译步骤

### 1. 清理之前的构建

```bash
cd /D/esp32/esp32project/hhh-flight
idf.py fullclean
```

### 2. 设置目标芯片

```bash
idf.py set-target esp32s3
```

### 3. 重新编译

```bash
idf.py build
```

### 4. 烧录和监控

```bash
idf.py -p /dev/ttyACM0 flash monitor
```

## 测试验证

### 成功检测输出

```
Setting up MPU6500...
Configuring MPU6500...
MPU6500 detected: WHO_AM_I=0x71
```

### 失败检测输出

```
MPU6500 not found! Expected 0x71 or 0x70, got 0xXX
```

如果检测到错误的 WHO_AM_I 值：
- 检查 SPI 接线
- 确认电源电压
- 检查 CS 引脚连接

## 调试命令

通过串口终端（115200 baud）：

```bash
# 查看 IMU 数据
imu

# 校准加速度计
ca

# 系统信息
sys
```

预期输出示例：
```
IMU data:
MPU6500 WHO_AM_I: 0x71
Raw gyro: 123 -45 678
Raw accel: 1000 2000 -15000
```

## 性能优化

### SPI 时钟频率

默认 10MHz，如果遇到通信问题可以降低：

```cpp
// 在 mpu6500.cpp 中修改
devcfg.clock_speed_hz = 5 * 1000 * 1000; // 5 MHz
```

### 采样率配置

默认 125Hz（1kHz / 8），可在 `configureIMU()` 中调整：

```cpp
mpu6500_write_reg(MPU_REG_SMPLRT_DIV, 3); // 1kHz / (1+3) = 250Hz
```

## 常见问题

### Q: MPU6500 无法检测？

A: 
1. 检查 SPI 接线是否正确
2. 确认 VCC 为 3.3V
3. 检查 CS 引脚是否连接
4. 尝试降低 SPI 时钟频率

### Q: 数据异常或漂移严重？

A:
1. 执行 `ca` 命令校准加速度计
2. 确保无人机放置在平稳表面上
3. 检查机械安装是否牢固

### Q: WiFi 工作不正常？

A:
1. 确认 `sdkconfig.defaults` 中启用了 PSRAM
2. 检查天线连接
3. 减少周围电磁干扰

## 参考资料

- [MPU6500 官方数据手册](https://invensense.tdk.com/wp-content/uploads/2024/01/PS-MPU-6500-datasheet1.pdf)
- [ESP32-S3 技术参考手册](https://www.espressif.com.cn/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf)
- [ESP-IDF SPI Master 组件文档](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripheral/spi_master.html)

## 版本历史

- **v1.0** (2026-09-19) - 初始 MPU6500 支持
  - 添加 mpu6500.cpp 驱动
  - 更新 imu.cpp 使用新 API
  - 配置 ESP32-S3 目标
  - 创建硬件连接指南

---

**注意**: 本固件针对 ESP32-S3 + MPU6500 配置优化。如使用其他硬件组合，请相应修改 `board_config.h` 中的引脚定义。
