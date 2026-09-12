#ifndef __CONFIG_H
#define __CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#define CONFIG_MAGIC      0x5AA55AA5UL
#define CONFIG_VERSION    10

/* 配置 Flash 存储地址 (使用 App Flash 最后一页, 0x0803F800) */
#define CONFIG_FLASH_ADDR  0x0803F800UL

/* 手柄型号索引 */
#define HANDLE_NUM      3
#define HANDLE_T12      0
#define HANDLE_T245     1
#define HANDLE_T210     2

typedef struct
{
    uint32_t magic;
    uint16_t version;

    /* 当前手柄型号 (见 HANDLE_*) */
    uint8_t  handle_type;

    /* 温度设置 (通用, 跨型号共享) */
    uint16_t default_temp;      /* 默认目标温度 °C */
    uint16_t sleep_temp;        /* 休眠温度 °C */

    /* PID 参数 (*100), 每型号一套
     * 分段控制: ΔT>50°C 全速, 30~50°C 用激进组(积分限幅 15%), ≤30°C 用保守组(限幅 30%)
     * 积分限幅为内置固定值, 不存入配置 */
    int16_t pid_agg_kp[HANDLE_NUM];   /* 激进 Kp (*100) */
    int16_t pid_agg_ki[HANDLE_NUM];   /* 激进 Ki (*100) */
    int16_t pid_agg_kd[HANDLE_NUM];   /* 激进 Kd (*100) */
    int16_t pid_cons_kp[HANDLE_NUM];  /* 保守 Kp (*100) */
    int16_t pid_cons_ki[HANDLE_NUM];  /* 保守 Ki (*100) */
    int16_t pid_cons_kd[HANDLE_NUM];  /* 保守 Kd (*100) */

    /* 每型号的温度上限 (°C) */
    uint16_t temp_max[HANDLE_NUM];    /* 常规设定上限 (T12=400, T245/T210=450) */
    uint16_t boost_max[HANDLE_NUM];   /* 大焊点自动加温(Boost)允许抬升到的上限
                                       * (须 ≥ temp_max, T12=450, 其余=450)
                                       * PID 计算按 boost_max 钳制有效设定值 */

    /* 多点温度校准偏移 (°C, -50 ~ +50), 每型号一套, 按温度区间插值
     * 6 温度点: 100/200/300/350/400/450, 细分常用焊接带 (300~350) */
    int8_t  cal_off_100[HANDLE_NUM];  /* 100°C 校准偏移 */
    int8_t  cal_off_200[HANDLE_NUM];  /* 200°C 校准偏移 */
    int8_t  cal_off_300[HANDLE_NUM];  /* 300°C 校准偏移 */
    int8_t  cal_off_350[HANDLE_NUM];  /* 350°C 校准偏移 */
    int8_t  cal_off_400[HANDLE_NUM];  /* 400°C 校准偏移 */
    int8_t  cal_off_450[HANDLE_NUM];  /* 450°C 校准偏移 */

    /* 系统设置 */
    uint16_t sleep_time;        /* 休眠时间 分钟, 0=禁用 */
    uint16_t shutdown_time;     /* 自动关机 分钟, 0=禁用 */
    uint8_t  buzzer_enable;     /* 0=关, 1=开 */
    uint8_t  brightness;        /* 0~255 */

    /* 温度校准点 (ADC 值), 每型号一套, 与偏移同温度点对齐 */
    uint16_t cal_adc_100[HANDLE_NUM];  /* 100°C 时的 ADC 值 */
    uint16_t cal_adc_200[HANDLE_NUM];  /* 200°C 时的 ADC 值 */
    uint16_t cal_adc_300[HANDLE_NUM];  /* 300°C 时的 ADC 值 */
    uint16_t cal_adc_350[HANDLE_NUM];  /* 350°C 时的 ADC 值 */
    uint16_t cal_adc_400[HANDLE_NUM];  /* 400°C 时的 ADC 值 */
    uint16_t cal_adc_450[HANDLE_NUM];  /* 450°C 时的 ADC 值 */

    uint16_t crc;

}config_t;

extern config_t g_config;

void Config_Init(void);
void Config_Load(void);
void Config_Save(void);
void Config_Reset(void);

/* 当前型号的脚柄索引 (越界回落到 T12) */
uint8_t  Config_GetHandleIdx(void);

/* 当前型号显示名 ("T12"/"T245"/"T210", 与索引对应) */
const char *Config_GetHandleName(void);

/* 型号校准点有效性 (严格单调递增 + 首点非 0) */
bool Config_IsHandleCalValid(uint8_t h);

/* 当前型号的温度上下限 (°C, 运行时按 handle_type 取值) */
uint16_t Config_GetTempMax(void);   /* 常规设定上限 */
uint16_t Config_GetBoostMax(void);  /* 有效设定(含 Boost 抬升)上限, 须 ≥ temp_max */

#endif