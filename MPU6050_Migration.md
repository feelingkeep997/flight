# MPU9250 到 MPU6050 迁移说明

## 主要差异

### 1. WHO_AM_I 寄存器值
- **MPU9250**: 0x71 或 0x73
- **MPU6050**: 0x68

### 2. 陀螺仪灵敏度
- **MPU9250**: 16.4 counts/(deg/s) (±2000 dps) 或 32.7 counts/(deg/s) (±1000 dps)
- **MPU6050**: 16.4 counts/(deg/s) (±1000 dps)

### 3. 加速度计灵敏度
- **MPU9250**: 16384 counts/g (±16g)
- **MPU6050**: 16384 counts/g (±16g) - 相同

### 4. 寄存器差异
MPU6050 和 MPU9250 的主要寄存器地址相同，但部分配置寄存器有所不同：

| 功能 | MPU9250 | MPU6050 | 说明 |
|------|---------|---------|------|
| WHO_AM_I | 0x75 | 0x75 | 相同地址，不同返回值 |
| PWR_MGMT_1 | 0x6B | 0x6B | 相同 |
| SMPLRT_DIV | 0x19 | 0x19 | 相同 |
| GYRO_CONFIG | 0x1B | 0x1B | 相同 |
| ACCEL_CONFIG | 0x1C | 0x1C | 相同 |

## 代码变更

### 文件重命名
- `mpu9250.cpp` → `mpu6050.cpp`
- `mpu9250.h` → `mpu6050.h`

### 关键修改

#### 1. 数据读取函数
```cpp
// MPU6050: 使用 16.4 作为陀螺仪灵敏度
gyro->x = gx / 16.4f * M_PI / 180.0f; // rad/s (MPU6050 sensitivity)
```

#### 2. 配置函数
添加了专用的配置函数：
```cpp
void mpu6050_configure(int gyroFSR, int accelFSR) {
    // Configure sample rate
    mpu6050_write_reg(MPU_REG_SMPLRT_DIV, 7); // 1kHz / (1+7) = 125Hz
    
    // Configure gyroscope full scale range
    mpu6050_write_reg(MPU_REG_GYRO_CONFIG, gyroFSR);
    
    // Configure accelerometer full scale range
    mpu6050_write_reg(MPU_REG_ACCEL_CONFIG, accelFSR);
    
    // Configure DLPF (Digital Low Pass Filter)
    mpu6050_write_reg(MPU_REG_CONFIG, 0x06);
    
    // Power management - wake up device
    mpu6050_write_reg(MPU_REG_PWR_MGMT_1, 0x01);
}
```

#### 3. IMU 初始化
```cpp
void setupIMU() {
    print("Setting up MPU6050...\n");
    mpu6050_init();
    configureIMU();
}

void configureIMU() {
    print("Configuring MPU6050...\n");
    mpu6050_configure(MPU6050_GYRO_FSR_1000, MPU6050_ACCEL_FSR_16G);
    
    uint8_t whoami = mpu6050_read_reg(MPU_REG_WHO_AM_I);
    if (whoami == 0x68) {  // MPU6050 检测
        imuOK = true;
    }
}
```

## CMakeLists.txt 更新

```cmake
idf_component_register(
    SRCS "mpu6050.cpp"  # 改为 mpu6050
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer log freertos
)
```

## 硬件连接（SPI 模式）

| MPU6050 | ESP32 |
|---------|-------|
| VCC     | 3.3V  |
| GND     | GND   |
| SCK     | GPIO18 (HSPI CLK) |
| MOSI    | GPIO23 (HSPI MOSI) |
| MISO    | GPIO19 (HSPI MISO) |
| CS      | GPIO5 (HSPI SS) |
| INT     | 可选（未使用 SPI 模式）|

## 测试步骤

1. **检查连接**：确认 SPI 引脚连接正确
2. **编译并烧录**：运行 `idf.py build` 和 `idf.py flash`
3. **串口输出**：通过串口终端查看输出
   - 成功：`MPU6050 detected: WHO_AM_I=0x68`
   - 失败：`MPU6050 not found! Expected 0x68, got 0xXX`
4. **校准**：使用 `ca` 命令校准加速度计
5. **查看数据**：使用 `imu` 命令查看实时数据

## 常见问题

### 1. WHO_AM_I 返回错误值
- 检查电源电压（应为 3.3V）
- 检查 SPI 接线是否正确
- 尝试降低 SPI 时钟频率（从 10MHz 降到 5MHz）

### 2. 数据异常
- 重新校准 IMU
- 检查加速度计量程设置是否合适
- 确认 IMU 安装方向与参数一致

### 3. 无法检测到设备
- 确认芯片型号确实是 MPU6050
- 检查 CS 引脚电平（空闲时应为高）
- 使用万用表测量 VCC 和 GND

## 参考资源

- [MPU6050 数据手册](https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-6000A-Datasheet1.pdf)
- [ESP-IDF SPI Master 组件](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripheral/spi_master.html)
