// ============================================================================
// battery.cpp —— 电池电压检测（仅支持经典 ESP32）
//
// 职责:
//   通过 ADC1 读取分压后的电池电压，供电量保护（safety.cpp）与网页/CLI 显示使用。
//
// 关键逻辑:
//   - 检测引脚由板级宏 BOARD_VBAT_ADC_PIN 决定（见 board_config.h），
//     在编译期换算成对应的 ADC1 通道，避免硬编码与板级配置脱节；
//   - 首次调用时配置 ADC1：12bit 宽度、12dB 衰减（量程约 0~3.3V），并做校准特征化；
//   - 读数用校准曲线换算成引脚电压（mV），再乘分压比还原成电池电压；
//   - updateBatteryVoltage() 每控制周期刷新全局 batteryVoltage。
//
// 输入/输出:
//   输入：ADC1 通道（由 BOARD_VBAT_ADC_PIN 推导）。
//   输出：batteryVoltage（V）。
//
// 重要参数:
//   BOARD_VBAT_ADC_PIN   电池分压输出所接的 GPIO（board_config.h，本项目为 36）
//   VBAT_DIVIDER_RATIO   分压比（默认 2.0，即 100k + 100k）
//
// 边界情况与潜在风险:
//   - 仅支持经典 ESP32 的 ADC1（GPIO32~39）。C3/S3 不再支持，换芯片需重写本文件。
//   - ADC1 通道与 GPIO 的对应关系是固定的，下方映射表只覆盖 ESP32 的 8 个 ADC1 引脚；
//     若 BOARD_VBAT_ADC_PIN 不在表中会直接编译失败（刻意设计，避免引脚写错后静默读到错误通道）。
//   - 分压比按 2:1 硬编码，若硬件电阻不同，读数会成比例错误（电量阈值判断随之失准）。
//   - 未做多次采样平均/滤波，电机负载变化时读数会有明显抖动；
//     safety.cpp 里的去抖（0.9s）部分缓解了这一问题。
// ============================================================================

#include "globals.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

// 电池分压比：100k + 100k = 2:1（引脚电压 = 电池电压 / 2）
#define VBAT_DIVIDER_RATIO 2.0f

// 经典 ESP32 的 ADC1 通道 ↔ GPIO 固定映射，按板级宏在编译期选定。
// 说明：ESP32 的 ADC1 只能采这 8 个引脚，其中 GPIO34~39 为 input-only（正适合做检测）。
#if BOARD_VBAT_ADC_PIN == 36
static const adc1_channel_t vbatChannel = ADC1_CHANNEL_0;
#elif BOARD_VBAT_ADC_PIN == 37
static const adc1_channel_t vbatChannel = ADC1_CHANNEL_1;
#elif BOARD_VBAT_ADC_PIN == 38
static const adc1_channel_t vbatChannel = ADC1_CHANNEL_2;
#elif BOARD_VBAT_ADC_PIN == 39
static const adc1_channel_t vbatChannel = ADC1_CHANNEL_3;
#elif BOARD_VBAT_ADC_PIN == 32
static const adc1_channel_t vbatChannel = ADC1_CHANNEL_4;
#elif BOARD_VBAT_ADC_PIN == 33
static const adc1_channel_t vbatChannel = ADC1_CHANNEL_5;
#elif BOARD_VBAT_ADC_PIN == 34
static const adc1_channel_t vbatChannel = ADC1_CHANNEL_6;
#elif BOARD_VBAT_ADC_PIN == 35
static const adc1_channel_t vbatChannel = ADC1_CHANNEL_7;
#else
#error "BOARD_VBAT_ADC_PIN 必须是 ESP32 ADC1 支持的引脚之一：32/33/34/35/36/37/38/39"
#endif

float batteryVoltage = NAN; // 最新电池电压（V）；NAN 表示无有效读数

// 刷新全局 batteryVoltage（每控制周期调用一次）
void updateBatteryVoltage() {
    batteryVoltage = readBatteryVoltage();
}

// 读取电池电压（V）。
// 处理链：ADC 原始值 → 校准曲线换算成引脚电压(mV) → 乘分压比还原电池电压。
float readBatteryVoltage() {
    static bool initialized = false;                     // ADC 是否已配置（只做一次）
    static esp_adc_cal_characteristics_t adc_chars;      // ADC 校准曲线参数

    if (!initialized) { // 首次调用时完成 ADC 配置（幂等）
        adc1_config_width(ADC_WIDTH_BIT_12);                          // 12bit 分辨率（0~4095）
        adc1_config_channel_atten(vbatChannel, ADC_ATTEN_DB_12);      // 12dB 衰减，量程约 0~3.3V

        // 特征化：建立"原始值 → 电压(mV)"的校准曲线
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12,
                                 0, &adc_chars);
        initialized = true;
    }

    int raw = adc1_get_raw(vbatChannel);

    // 用校准曲线换算成引脚电压（mV），比直接用 3.3V/4096 线性换算更准
    uint32_t mv = esp_adc_cal_raw_to_voltage(raw, &adc_chars);

    // 乘分压比还原电池端电压（V）
    return mv / 1000.0f * VBAT_DIVIDER_RATIO;
}
