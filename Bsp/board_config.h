// ============================================================================
// board_config.h —— 板级硬件配置（按芯片型号分发引脚与功能开关）
//
// 职责:
//   全工程唯一的"硬件抽象层"：根据 ESP-IDF 自动定义的 CONFIG_IDF_TARGET_* 宏，
//   为 ESP32C3 / ESP32S3 / ESP32（默认）三套板卡分别给出电机引脚、电池 ADC 引脚、
//   遥控串口、SPI（IMU）、I2C、LED、日志/控制台容量、以及 WiFi/Web RC 等功能开关。
//
// 关键逻辑:
//   编译期条件编译（#ifdef CONFIG_IDF_TARGET_ESP32S3 等）选择对应宏块；
//   未识别的芯片走最后的 fallback，保证工程仍可编译（功能按保守默认）。
//
// 输入/输出:
//   纯宏定义，无运行时行为。被几乎所有模块包含（经 globals.h）。
//
// 重要宏:
//   BOARD_MOTOR_PINS  4 路电机引脚，顺序对应 MOTOR_REAR_LEFT/RIGHT/FRONT_RIGHT/FRONT_LEFT
//   BOARD_VBAT_ADC_PIN 电池检测引脚（经典 ESP32 用 GPIO36，须是 ADC1 引脚之一）
//   BOARD_RC_RX_PIN / BOARD_RC_BAUD / BOARD_RC_PROTOCOL  遥控串口
//   BOARD_SPI_SCK/MISO/MOSI/CS  IMU 的 SPI 引脚
//   BOARD_LED_PIN / BOARD_LED_INVERTED  状态灯
//   BOARD_WIFI_ENABLED / BOARD_WEB_RC_ENABLED  功能总开关（C3 关闭，S3/ESP32 打开）
//   BOARD_LOG_DURATION / BOARD_CONSOLE_LINES  日志与控制台容量（影响 RAM 占用）
//
// 边界情况与潜在风险:
//   - 引脚号写错会直接导致外设不工作（甚至与 flash/PSRAM 引脚冲突造成启动失败），
//     改动前务必对照具体开发板原理图。
//   - C3 板卡的 BOARD_WIFI_ENABLED=0，因此 wifi.cpp/web_rc.cpp 整体不参与编译；
//     若在 C3 上使用网页遥控需先改此宏并核实内存余量。
//   - battery.cpp 目前只实现经典 ESP32 的 ADC1 路径：BOARD_VBAT_ADC_PIN 必须是
//     ESP32 的 ADC1 引脚（32/33/34/35/36/37/38/39），否则编译期直接报错。
//     C3/S3 段里定义的引脚（2 / 1）不在该范围内，故这两个板卡当前无法编译通过
//     （如需支持，请先在 battery.cpp 中补上对应芯片的 ADC 实现）。
//   - 文件末的 fallback 宏只在对应宏"未定义"时生效，用于兼容未知芯片。
// ============================================================================

// Board-level hardware configuration - ESP-IDF port
// ESP-IDF automatically defines CONFIG_IDF_TARGET_ESP32 / ESP32S3 / ESP32C3

#pragma once

#ifdef CONFIG_IDF_TARGET_ESP32C3
// ---------------------- ESP32C3 -------------------------
// 特点：引脚少、无 WiFi 功能开关（关闭 Web RC），日志缓冲较小

#define BOARD_MOTOR_PINS   {3, 10, 0, 1}
#define BOARD_VBAT_ADC_PIN 2
#define BOARD_RC_RX_PIN    9
#define BOARD_RC_TX_PIN    -1
#define BOARD_RC_PROTOCOL  0   // 0 = SBUS（rc.cpp 目前只实现了 SBUS）
#define BOARD_RC_BAUD      420000 // 仅 CRSF 用；SBUS 固定 100000
#define BOARD_RC_UART_NUM  1

#define BOARD_SPI_SCK      4
#define BOARD_SPI_MISO     5
#define BOARD_SPI_MOSI     6
#define BOARD_SPI_CS       7

#define BOARD_I2C_SDA          (-1)
#define BOARD_I2C_SCL          (-1)

#define BOARD_LED_ENABLED  1
#define BOARD_LED_PIN      8
#define BOARD_LED_INVERTED 1

#define BOARD_VBAT_ADC_SAMPLES       8
#define BOARD_LOG_DURATION           4
#define BOARD_CONSOLE_LINES          20
#define BOARD_CONSOLE_LINE_LEN       160
#define BOARD_MAVLINK_TELEM_FAST_HZ  5
#define BOARD_WIFI_ENABLED           0
#define BOARD_WEB_RC_ENABLED         0

#undef  EXPANSION_BOARD_ENABLED
#define EXPANSION_BOARD_ENABLED      0
#define BOARD_VL53_XSHUT_PIN   (-1)
#define BOARD_PMW_CS_PIN       (-1)

#elif defined(CONFIG_IDF_TARGET_ESP32S3)
// ---------------------- ESP32S3 -------------------------
// 特点：完整功能（WiFi + Web RC + 扩展板），日志缓冲较大

#define BOARD_MOTOR_PINS   {4, 5, 6, 7}
#define BOARD_VBAT_ADC_PIN 1
#define BOARD_RC_RX_PIN    8
#define BOARD_RC_TX_PIN    -1
#define BOARD_RC_PROTOCOL  0   // 0 = SBUS（rc.cpp 目前只实现了 SBUS）
#define BOARD_RC_BAUD      420000 // 仅 CRSF 用；SBUS 固定 100000
#define BOARD_RC_UART_NUM  2

#define BOARD_SPI_SCK      12
#define BOARD_SPI_MISO     13
#define BOARD_SPI_MOSI     11
#define BOARD_SPI_CS       10

#define BOARD_I2C_SDA          35
#define BOARD_I2C_SCL          36

#define BOARD_LED_ENABLED  1
#define BOARD_LED_PIN      2
#define BOARD_LED_INVERTED 0

#define BOARD_VBAT_ADC_SAMPLES       16
#define BOARD_LOG_DURATION           8
#define BOARD_CONSOLE_LINES          50
#define BOARD_CONSOLE_LINE_LEN       240
#define BOARD_MAVLINK_TELEM_FAST_HZ  10
#define BOARD_WIFI_ENABLED           1
#define BOARD_WEB_RC_ENABLED         1

#define EXPANSION_BOARD_ENABLED      1
#define BOARD_VL53_XSHUT_PIN   37
#define BOARD_PMW_CS_PIN       38

#else
// ---------------------- ESP32 (default) -------------------------
// 默认板卡（也是当前项目实际使用的一套：E1 Mini V1.3Y）
//
// ⚠ 电机引脚里有两颗是 ESP32 的 strapping（启动配置）引脚，接线/改版前务必核对：
//   GPIO12 = MTDI。上电瞬间若为高电平，ROM 会把 SPI flash 电压设成 1.8V，
//            而模块上的 flash 是 3.3V → 直接启动失败（串口一片乱码或不输出）。
//            用 MOSFET 直驱时栅极通常有下拉电阻，静态为低是安全的；但如果换成
//            带内部上拉的驱动电路、或栅极下拉缺失，就会踩这个坑。
//   GPIO15 = MTDO。上电为高会打开 SDIO slave 时序并让 ROM 在 UART0 输出启动日志；
//            为低则抑制启动日志。不会导致启动失败，但"看不到 boot log"往往就是它。
//   GPIO2  = 板载 LED。也是 strapping 引脚，上电必须为低或悬空才能进下载模式；
//            若 LED 接法是 GPIO2 → 限流电阻 → 3.3V（灌电流），上电会被拉高，
//            需要按住 BOOT 再按 RST 才能烧录。
// 这些是硬件事实，不能靠改软件规避；若你的板子电机确实接在 12/15 上，
// 请确认对应的下拉/驱动电路，别在没核实前改这里的引脚号。
#define BOARD_MOTOR_PINS   {12, 13, 15, 14}
#define BOARD_VBAT_ADC_PIN 36
#define BOARD_RC_RX_PIN    4
#define BOARD_RC_TX_PIN    -1
#define BOARD_RC_PROTOCOL  0   // 0 = SBUS（rc.cpp 目前只实现了 SBUS，CRSF 分支是空壳）
#define BOARD_RC_BAUD      420000 // 仅 CRSF 用；SBUS 固定 100000，见 rc.cpp
#define BOARD_RC_UART_NUM  2

#define BOARD_SPI_SCK      18
#define BOARD_SPI_MISO     19
#define BOARD_SPI_MOSI     23
#define BOARD_SPI_CS       5

#define BOARD_I2C_SDA      21
#define BOARD_I2C_SCL      22

#define BOARD_LED_ENABLED  1
#define BOARD_LED_PIN      2
#define BOARD_LED_INVERTED 0

#define BOARD_VBAT_ADC_SAMPLES       16
#define BOARD_LOG_DURATION           8
#define BOARD_CONSOLE_LINES          50
#define BOARD_CONSOLE_LINE_LEN       240
#define BOARD_MAVLINK_TELEM_FAST_HZ  10
#define BOARD_WIFI_ENABLED           1
#define BOARD_WEB_RC_ENABLED         1

#define EXPANSION_BOARD_ENABLED      1
#define BOARD_VL53_XSHUT_PIN   32
#define BOARD_PMW_CS_PIN       33

#endif

// ---- 未知芯片的兜底默认值（仅当对应宏尚未定义时生效）----
#if !defined(EXPANSION_BOARD_ENABLED)
#define EXPANSION_BOARD_ENABLED 1
#endif
#if !defined(BOARD_WIFI_ENABLED)
#define BOARD_WIFI_ENABLED   1
#endif
#if !defined(BOARD_WEB_RC_ENABLED)
#define BOARD_WEB_RC_ENABLED 1
#endif
#if !defined(BOARD_RC_UART_NUM)
#define BOARD_RC_UART_NUM 1
#endif
#if !defined(BOARD_RC_PROTOCOL)
#define BOARD_RC_PROTOCOL 0
#endif
