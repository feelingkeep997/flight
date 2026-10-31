# hhh-flight - ESP-IDF Flight Controller

基于 ESP32 的穿越机飞控固件，使用 ESP-IDF 开发。

## 项目结构

采用 Keil 风格的分层目录（构建系统仍为 ESP-IDF / CMake，各层通过 `EXTRA_COMPONENT_DIRS` 注册为组件）：

```
hhh-flight/
├── User/                       # 入口层：app_main() 与主循环
│   └── main.cpp
├── application/                # 应用层：飞行控制逻辑
│   ├── control.cpp             # 姿态/高度控制、串级 PID
│   ├── estimate.cpp            # 姿态估计（陀螺积分 + 加速度修正）
│   ├── motors.cpp              # 电机混控与输出
│   ├── safety.cpp              # 失控保护与安全阈值
│   ├── led.cpp                 # LED 状态指示
│   └── parameters.cpp          # NVS 参数存取
├── Drivers/                    # 驱动层：具体外设
│   ├── mpu9250.cpp / .h        # MPU9250 SPI 驱动
│   ├── imu.cpp                 # IMU 数据读取与标定
│   ├── rc.cpp                  # 遥控接收机 (SBUS/CRSF)
│   └── battery.cpp             # 电池电压检测
├── Bsp/                        # 板级支持层
│   ├── board_config.h          # 引脚 / 特性宏定义
│   ├── wifi.cpp                # WiFi AP/STA 与 UDP 遥控
│   ├── web_rc.cpp              # Web 遥控接口
│   ├── web_rc_html.h           # 内嵌网页资源
│   └── espnow.cpp              # ESP-NOW 链路（默认未参与编译）
├── Middlewares/                # 中间件层：与硬件无关
│   ├── globals.h               # 全局变量 / 函数声明中心
│   ├── vector.h                # 向量数学
│   ├── quaternion.h            # 四元数数学
│   ├── pid.h                   # PID 控制器
│   ├── lpf.h                   # 低通滤波器
│   ├── cf_common.cpp / cf_math.h
│   ├── cli.cpp                 # 命令行接口
│   ├── log.cpp                 # 数据日志
│   ├── mavlink.cpp / mavlink_compat.h   # MAVLink 协议
│   └── time.cpp / util.cpp     # 时间与工具函数
├── docs/                       # 设计文档与仿真页面
├── CMakeLists.txt              # 顶层构建配置（注册各层为组件）
├── sdkconfig.defaults          # ESP-IDF 默认配置
└── partitions.csv              # Flash 分区表
```

### 分层依赖

各层单向依赖，下层不感知上层：

```
User  →  application  →  Drivers  →  Bsp  →  Middlewares
```

- `Middlewares` 不依赖任何硬件抽象，只做数学、协议与工具
- `Bsp` 负责板级差异（引脚、WiFi、Web 控制台）
- `Drivers` 是具体外设驱动，通过 `Bsp/board_config.h` 的宏适配不同芯片

## 支持的硬件

- **ESP32** (经典双核) - 默认配置
- **ESP32-C3** - WiFi 禁用模式
- **ESP32-S3** - 全功能配置

## 主要功能

### 飞行模式

模式常量定义在 `application/control.cpp`（`RAW=0, ACRO=1, STAB=2, ALTHOLD=3, AUTO=4`），默认 `STAB`。

| 模式 | 杆量含义 | 走哪几个环 |
|---|---|---|
| RAW | 直接给**力矩**（杆量 ×0.1），完全无辅助 | 都跳过，仅混控 |
| ACRO | 给**角速率**目标，姿态不自动回平（纯手动/特技） | 角速率环 |
| STAB | 给**姿态角**目标，松杆自动回平并锁头（默认模式） | 姿态环 + 角速率环 |
| ALTHOLD | 同 STAB，但油门杆变成**升降速率**指令，中位死区内松杆悬停 | 姿态环 + 角速率环 + 定高环 |
| AUTO | 仅供失控保护 `descend()` 与 MAVLink `SET_ATTITUDE_TARGET` 使用，遥控杆量不生效 | 由调用方决定 |

> ⚠️ 本节旧版把 ACRO 写成"自稳"、STAB 写成"定高"，与代码相反，已按上表改正。

### 传感器
- MPU9250 / MPU6500 IMU（陀螺仪 + 加速度计，SPI；无磁力计）
- SBUS 遥控接收（CRSF 尚未实现，`setupRC()` 会强制回退到 SBUS 并告警）
- 电池电压监测（ADC1，2:1 分压）
- **无气压计 / 无 ToF**，定高为加速度计近似

### 通信
- UART 命令行界面 (115200 baud)
- WiFi AP/STA 模式
- MAVLink 遥测
- Web 遥控器

## 编译环境

### ESP-IDF 要求
- ESP-IDF v4.4 或更高版本
- Xtensa toolchain (ESP32)
- RISC-V toolchain (ESP32-C3/S3)

### 安装 ESP-IDF

```bash
git clone --recursive https://github.com/espressif/esp-idf.git --branch v5.1
cd esp-idf
./install.sh esp32
source export.sh
```

### 编译项目

```bash
cd hhh-flight

# 对于 ESP32 (默认)
idf.py set-target esp32
idf.py build

# 对于 ESP32-C3
idf.py set-target esp32c3
idf.py build

# 对于 ESP32-S3
idf.py set-target esp32s3
idf.py build
```

### 烧录固件

```bash
# ESP32
idf.py -p /dev/ttyUSB0 flash monitor

# ESP32-C3
idf.py -p /dev/ttyACM0 flash monitor

# ESP32-S3
idf.py -p /dev/ttyACM0 flash monitor
```

## 命令行接口

上电后通过串口终端连接（115200 baud），可用命令：

```
help          - 显示帮助信息
p             - 显示所有参数
p <name>      - 显示指定参数
p <name> <v>  - 设置参数值
preset        - 重置所有参数
arm           - 解锁无人机
disarm        - 锁定无人机
raw/stab/acro/auto - 切换飞行模式
ca            - 校准加速度计
cr            - 校准遥控器
imu           - 显示 IMU 信息
log [dump]    - 查看日志
reboot        - 重启系统
sys           - 显示系统信息
```

## 引脚配置

### ESP32 (默认)
- **电机**: GPIO 12, 13, 15, 14
- **IMU SPI**: SCK=18, MISO=19, MOSI=23, CS=5
- **RC UART**: RX=GPIO 4, TX=-1
- **电池 ADC**: GPIO 36
- **LED**: GPIO 2

### ESP32-C3
- **电机**: GPIO 3, 10, 0, 1
- **IMU SPI**: SCK=4, MISO=5, MOSI=6, CS=7
- **RC UART**: RX=GPIO 9
- **电池 ADC**: GPIO 2
- **LED**: GPIO 8

### ESP32-S3
- **电机**: GPIO 4, 5, 6, 7
- **IMU SPI**: SCK=12, MISO=13, MOSI=11, CS=10
- **RC UART**: RX=GPIO 8
- **电池 ADC**: GPIO 1
- **LED**: GPIO 2

## 注意事项

1. **无气压计**：ALTHOLD 模式使用垂直速度积分近似定高，会随时间漂移
2. **WiFi 性能**：ESP32-C3 单核模式下 WiFi 会占用 CPU，建议关闭
3. **Flash 分区**：使用自定义分区表，预留空间存储参数和日志
4. **ADC 精度**：电池电压使用 ESP32 内部 ADC，需要外部分压电路

## 相关文档

- [`HARDWARE_SETUP.md`](HARDWARE_SETUP.md) — 硬件连接与接线说明
- [`MPU6050_Migration.md`](MPU6050_Migration.md) — 从 MPU6050 迁移到 MPU9250 的说明
- [`MPU6500_Support.md`](MPU6500_Support.md) — MPU6500 支持说明
- [`docs/学习方案.md`](docs/学习方案.md) — 代码学习路径
- [`docs/mixer-pid-sim.html`](docs/mixer-pid-sim.html) — 混控与 PID 仿真页面

## 开发指南

### 添加新的源文件

1. 把 `.cpp` / `.h` 放到对应层次的目录（例如 `Drivers/`）
2. 在该目录的 `CMakeLists.txt` 的 `SRCS` 中追加源文件
3. 跨层调用通过 `REQUIRES` 声明依赖，并保持 `User → application → Drivers → Bsp → Middlewares` 的单向依赖

以 `Drivers/` 为例：

```cmake
idf_component_register(
    SRCS
        "mpu9250.cpp"
        "imu.cpp"
        "rc.cpp"
        "battery.cpp"
        "new_driver.cpp"      # 新增
    INCLUDE_DIRS "."
    REQUIRES Middlewares Bsp driver esp_timer esp_adc freertos log
)
```

### 新增一个层 / 组件目录

在顶层 `CMakeLists.txt` 的 `EXTRA_COMPONENT_DIRS` 中追加目录名即可，ESP-IDF 会把它注册为组件：

```cmake
set(EXTRA_COMPONENT_DIRS
    Middlewares
    Bsp
    Drivers
    application
    User
    NewLayer          # 新增
)
```

### 调试技巧

1. 启用打印日志：修改 `board_config.h` 中的打印级别
2. 使用 JTAG 调试：配置 OpenOCD
3. 查看 FreeRTOS 任务：通过 CLI 命令 `sys`

## 许可证

本项目遵循 MIT 许可证。

## 致谢

- ESP32 by Espressif Systems
- Crazyflie firmware inspiration
- ESP-IDF development framework
