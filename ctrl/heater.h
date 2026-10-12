#ifndef __HEATER_H
#define __HEATER_H

#include <stdint.h>
#include <stdbool.h>

/*
 * 加热控制模块
 * 通过 PWM 控制加热芯功率
 * 功率范围: 0 ~ HEATER_POWER_MAX (对应 0% ~ 100%)
 */
#define HEATER_POWER_MAX  1000

/* 实际可达的最大功率(90% 占空比): 保留 ≥10% 关断窗供温度采样.
 * 对外报告"加热百分比"时应以本值为满量程 —— 否则会出现
 * "显示 100% 而实际占空比只有 90%"的失真(功率条/百分比数字直接读数). */
#define HEATER_POWER_LIMIT  (HEATER_POWER_MAX * 90 / 100)

/* 初始化加热器 (含 PWM 初始化) */
void Heater_Init(void);

/* 设置加热功率 0 ~ HEATER_POWER_MAX */
void Heater_SetPower(int16_t power);

/* 获取当前加热功率 */
int16_t Heater_GetPower(void);

/* 启用/禁用加热 */
void Heater_Enable(void);
void Heater_Disable(void);

#endif
