# 硬件连接指南（实际构建目标：经典 ESP32）

> **⚠️ 先读这一段**
> 本文件原先整篇按 **ESP32-S3** 写（GPIO12/13/11/10 接 SPI、GPIO4-7 接电机、`idf.py set-target esp32s3`），
> 但仓库里 `sdkconfig:220` 与 `build/config.env` 实测都是 **`CONFIG_IDF_TARGET="esp32"`（经典 ESP32）**，
> 编译出来的固件走的是 `Bsp/board_config.h` 最后的默认分支，引脚与 S3 完全不同。
> 照旧文档接线会一个外设都不通。下面所有表格已按**实际生效的经典 ESP32 分支**改正，
> S3 分支的引脚表挪到文末附录（`board_config.h` 里确实有这一段，换芯片时用得上）。
>
> 权威来源永远是 `Bsp/board_config.h`，本文件只是它的可读版本。改引脚请改那个头文件。

## 硬件清单

- **主控**: 经典 ESP32（双核 Xtensa LX6，160MHz），E1 Mini V1.3Y 板
- **IMU**: MPU6500（WHO_AM_I=0x70；驱动同时接受 0x71/0x73 = MPU9250/9255，寄存器兼容，无磁力计）
- **电机**: 4 × 8520 空心杯，MOSFET 直驱（`pwmMax = -1` → 纯占空比模式，不是电调脉宽模式）
- **遥控**: SBUS 接收机（CRSF 分支未实现，见下文）
- **电源**: ME6118A33PG 3.3V LDO
- **无气压计 / 无 ToF**：定高只能用加速度计近似（ALTHOLD）

## SPI（IMU）

| ESP32 GPIO | MPU6500 | 说明 |
|---|---|---|
| 18 | SCK  | SPI 时钟，主机 HSPI，**10 MHz** |
| 19 | MISO | 主入从出 |
| 23 | MOSI | 主出从入 |
| 5  | CS   | 片选，由 SPI 外设自动控制 |
| 3.3V | VCC | 供电 |
| GND | GND | 地 |

来源：`board_config.h` 默认分支的 `BOARD_SPI_SCK/MISO/MOSI/CS`，`Drivers/mpu9250.cpp` 的 `mpu9250_init()`。

## 电机 PWM（LEDC 低速模式，25 kHz，10 bit）

| 电机（下标） | ESP32 GPIO | ⚠️ strapping 引脚 |
|---|---|---|
| Rear Left (0)   | **12** | **是 · MTDI —— 最危险，见下** |
| Rear Right (1)  | 13 | 否 |
| Front Right (2) | **15** | **是 · MTDO** |
| Front Left (3)  | 14 | 否 |

`BOARD_MOTOR_PINS {12, 13, 15, 14}`，下标语义见 `application/motors.cpp` 的 `MOTOR_*` 常量。

**这两颗 strapping 引脚必须先搞清楚再上电：**

- **GPIO12 (MTDI)**：上电瞬间若为高电平，ROM 会把 SPI flash 供电电压设成 **1.8V**，
  而模块上的 flash 是 3.3V → **直接启动失败**（串口无输出或一片乱码）。
  MOSFET 直驱时栅极通常有下拉电阻，静态为低是安全的；但如果驱动电路带内部上拉、
  或栅极下拉缺失/开路，就会踩雷。**接线前先量一下 GPIO12 对地电阻。**
- **GPIO15 (MTDO)**：上电为高会打开 SDIO slave 时序并让 ROM 在 UART0 输出启动日志；
  为低则抑制启动日志。不会导致启动失败，但"看不到 boot log"往往就是它被电机电路拉低了。
- 附带一提，**GPIO2（板载 LED）也是 strapping 引脚**，上电必须为低或悬空才能进下载模式。
  若 LED 接法是 GPIO2 → 限流电阻 → 3.3V（灌电流），上电会被拉高，
  烧录时需要按住 BOOT 再按 RST。

## 遥控接收机（SBUS）

| ESP32 GPIO | 接收机 | 说明 |
|---|---|---|
| 4 | Signal | SBUS 信号输入（`BOARD_RC_RX_PIN`） |
| 5V 或 3.3V | VCC | 按接收机规格，多数 SBUS 接收机要 5V |
| GND | GND | 必须与 ESP32 共地 |

**SBUS 的物理层是协议规定死的，`Drivers/rc.cpp` 已按此配置：**

- **100000 baud、8 数据位、偶校验、2 停止位（8E2）** —— 不是常见的 8N1
- **逻辑反相**：SBUS 是反相串口。代码里用 `uart_set_line_inverse(UART_SIGNAL_RXD_INV)`
  在 ESP32 的 GPIO 矩阵内部取反，因此**接收机可以直接接 GPIO4，不需要外部反相电路**。
  如果你的板子上已经有反相电路（或接收机输出的是正相 TTL），把参数 `RC_INVERTED` 设成 `0`。
- 使用 **UART2**（`BOARD_RC_UART_NUM`），UART0 留给 CLI/日志。

**CRSF（ELRS）目前没有实现**：`rc.cpp` 里 CRSF 分支历史上只是个占位空壳，
`setupRC()` 现在会检测到 `RC_PROTOCOL != 0` 并**强制回退到 SBUS + 打印告警**，
不再静默失效。要用 ELRS 得自己补 CRSF 解析。

**首次使用必须标定**：串口敲 `cr`，10 秒内把每个摇杆/开关推到 最小 → 中位 → 最大 各停一下。
标定结果写入参数 `RC_MID_*` / `RC_MAX_*`，落地后自动存进 NVS。
用 `rc` 命令查看通道原始值、good/bad 帧计数与信号状态。

## 电池电压检测

| ESP32 GPIO | 分压输出 | 说明 |
|---|---|---|
| **36** | 分压中点 | ADC1_CHANNEL_0，input-only 引脚，正适合做检测 |

- 分压比按 **2:1** 硬编码（100k + 100k），见 `Drivers/battery.cpp` 的 `VBAT_DIVIDER_RATIO`。
  **如果你的电阻不是这个比例，读数会成比例错误，电量保护阈值全部失准** —— 改这个宏。
- 只能用 ESP32 的 **ADC1**（GPIO32~39）。ADC2 与 WiFi 冲突，不可用。
- 阈值（`Middlewares/globals.h`）：WARN 3.5V / LOW 3.3V / CRIT 3.0V / ABSENT 0.5V。
  飞行中跌破 3.0V → 强制下降；地面跌破 3.5V → 禁止解锁。

## LED 状态灯

| ESP32 GPIO | LED | 说明 |
|---|---|---|
| 2 | 阳极 (+) | `BOARD_LED_PIN`，`BOARD_LED_INVERTED = 0`（高电平点亮） |
| GND | 阴极 (-) | 串限流电阻 |

## 串口 / 调试

- **UART0，115200 8N1**，同时承载 CLI、`print()` 日志与 VOFA+/FireWater 绘图。
- Windows 上的 COM 号不固定（CH340 会随插拔/USB 口变化，见过 COM4、COM5、COM6），
  以设备管理器为准；VOFA+ 里选错口就是什么都收不到。
- **绘图/遥测默认是关的**：上电只打印初始化信息与 CLI 菜单，不刷数据行。
  要看波形先敲 `plot on`（VOFA+ 选 FireWater，17 通道，地面 **50Hz**、`arm` 之后自动降到 **10Hz**）；
  要看人可读数据行敲 `trace on`（5Hz，带标签）。`plot off` / `trace off` / `quiet` 关闭。
  一直常开会占掉大部分 115200 带宽（50Hz≈56%），把 failsafe / 定高 / HTTP 的日志挤掉。

## 编译和烧录

本仓库的工具链在 `D:\esp32\espreif\Espressif`，但 `idf_cmd_init.bat` 已损坏，
`idf.py` / `ninja` / `cmake` 都不在 PATH（只有 `IDF_PATH` 环境变量）。
**增量构建直接用 ninja 绝对路径**（在 `build/` 目录下执行）：

```bash
cd build
"D:/esp32/espreif/Espressif/tools/ninja/1.12.1/ninja.exe"
```

**坑**：改动任何 `CMakeLists.txt` 会让 ninja 自动重跑 cmake 配置，而它用的是系统
`C:\python\python.exe`（缺 `idf_component_manager`）从而失败。修法是手动重新配置：

```bash
cd build
cmake -G Ninja \
  -DPYTHON="D:/esp32/espreif/Espressif/python_env/idf5.1_py3.11_env/Scripts/python.exe" \
  -DIDF_PATH="D:/esp32/espreif/Espressif/frameworks/esp-idf-v5.1.6" ..
"D:/esp32/espreif/Espressif/tools/ninja/1.12.1/ninja.exe"
```

**目标芯片**：`CONFIG_IDF_TARGET="esp32"`（经典 ESP32）。
如果真的要换成 S3，除了 `idf.py set-target esp32s3`，还必须先给 `Drivers/battery.cpp`
补上 S3 的 ADC 实现 —— 它目前只认经典 ESP32 的 ADC1 通道，
S3 分支里的 `BOARD_VBAT_ADC_PIN 1` 不在映射表内，会直接 `#error` 编译失败。

## 故障排查

### MPU6500/9250 检测失败

串口输出 `IMU not found! Expected 0x71/0x73 (MPU9250) or 0x70 (MPU6500), got 0xXX`：

- `got 0x00` 或 `0xFF` → SPI 根本没通：查 CS/SCK/MISO/MOSI 接线、查 3.3V 供电、
  确认 CS 空闲时为高。
- 启动日志里有一行 `reg readback: DIV=0x00 CONFIG=0x03 GYRO=0x10 ACCEL=0x18 ACCEL2=0x03 PWR=0x01`，
  **用它判断 SPI 写链路是否真的生效**；读到全 0x00 说明写不进去。
- 试着把 `mpu9250_init()` 里的 `clock_speed_hz` 从 10MHz 降到 1MHz 排除信号完整性问题。

### 遥控收不到数据

`rc` 命令看 `good=` / `bad=` 帧计数：

- 两个都是 0 → 完全没有字节进来：查接线（GPIO4）、查接收机是否对码、查共地。
- `bad` 一直涨而 `good=0` → 收到字节但解不出帧：**十有八九是反相设置反了**，
  试 `p RC_INVERTED 0` 然后 `reboot`；也可能是波特率/校验位不对。
- 标定失败提示"10 秒内一帧合法 SBUS 都没收到"时同样先查 `RC_INVERTED`。

### 上电就重启 / 串口无输出

大概率是 **GPIO12 被拉高**导致 flash 电压被设成 1.8V。断开 GPIO12 上的电机驱动再上电验证。

### ADC 读数异常

- 先确认外部分压比是否真的是 2:1，不是就改 `VBAT_DIVIDER_RATIO`。
- 电机负载变化时读数会抖（代码未做多次平均），`safety.cpp` 里有 0.9s 去抖兜底。

## 调试命令速查

上电后串口终端（115200）会自动打印一次菜单。常用：

```
help          完整菜单
imu           IMU 状态 + 标定值（含陀螺零偏与在线微调量）
cg            只标定陀螺零偏（静置，约 1 秒）
ca            标定加速度计（水平静置，约 4 秒）
cr            标定遥控行程（10 秒内推满各通道）
rc            遥控通道原始值 + 收帧统计 + 信号状态
plot on|off   IMU 串口绘图开关（默认 off，开启后 50Hz / 解锁后 10Hz）
trace on|off  IMU 串口可读遥测开关（默认 off，5Hz）
quiet         一键关闭 plot + trace 两路数据输出
ps / psq      姿态欧拉角 / 四元数
mot           四路电机输出
p             全部参数；p <名> <值> 改参数（热生效，落地后自动存 NVS）
mfr/mfl/mrr/mrl  单电机测试（**已解锁时会被拒绝**）
arm / disarm  解锁 / 上锁
reset         姿态与陀螺微调量复位
sys           芯片与剩余堆
reboot        重启
```

## 参考资料

- [经典 ESP32 数据手册](https://www.espressif.com.cn/sites/default/files/documentation/esp32_datasheet_cn.pdf)（strapping 引脚见 "Strapping Pins" 一节）
- [MPU-6500 数据手册](https://invensense.tdk.com/wp-content/uploads/2024/01/PS-MPU-6500-datasheet1.pdf)（DLPF_CFG / A_DLPF_CFG 表在 Register Map 章节）
- [MPU-9250 寄存器映射](https://invensense.tdk.com/wp-content/uploads/2024/01/RM-MPU-9250A-00-v1.6.pdf)
- [ESP-IDF SPI Master 文档](https://docs.espressif.com/projects/esp-idf/en/v5.1.6/esp32/api-reference/peripheral/spi_master.html)
- [ESP-IDF UART 文档](https://docs.espressif.com/projects/esp-idf/en/v5.1.6/esp32/api-reference/peripherals/uart.html)

---

## 附录：ESP32-S3 分支引脚（当前**未**使用）

`board_config.h` 里有 `#elif defined(CONFIG_IDF_TARGET_ESP32S3)` 分支，换芯片时对照：

| 功能 | S3 GPIO |
|---|---|
| SPI SCK / MISO / MOSI / CS | 12 / 13 / 11 / 10 |
| 电机 RL / RR / FR / FL | 4 / 5 / 6 / 7 |
| RC 信号 | 8 |
| 电池 ADC | 1（⚠️ `battery.cpp` 尚不支持，会 `#error`） |
| I2C SDA / SCL | 35 / 36 |
| LED | 2 |
