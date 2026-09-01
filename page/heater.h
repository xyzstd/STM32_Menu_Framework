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

/* 初始化加热器 (含 PWM 初始化) */
void Heater_Init(void);

/* 设置加热功率 0 ~ HEATER_POWER_MAX */
void Heater_SetPower(int16_t power);

/* 获取当前加热功率 */
int16_t Heater_GetPower(void);

/* 启用/禁用加热 */
void Heater_Enable(void);
void Heater_Disable(void);
bool Heater_IsEnabled(void);

#endif