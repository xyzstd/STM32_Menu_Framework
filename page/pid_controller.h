#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H

#include <stdint.h>

/* PID 输出范围: 0 ~ PID_OUTPUT_MAX (对应 0% ~ 100% 加热功率) */
#define PID_OUTPUT_MAX  1000

/* 积分限幅 (内置固定值, 不提供页面调节): PID 输出满量程的 30% */
#define PID_INTEGRAL_LIMIT  ((PID_OUTPUT_MAX * 30) / 100)

/* PID 参数存储格式: 实际值 = 存储值 / 100 */
#define PID_SCALE       100

/* 温度下限固定 100°C; 上限按型号在运行时由 Config_GetTempMax() 提供
 * (T12=400, T245/T210=450), 不再使用固定宏, 保证切型号后目标不越标称上限 */
#define TEMP_MIN        100

void PID_Controller_Init(void);
void PID_Controller_Reload(void);   /* 型号切换后按当前型号系数重载 PID 参数并复位 */
void PID_Controller_Reset(void);

/* 设置目标温度 (单位: °C) */
void PID_Controller_SetSetpoint(int16_t setpoint);
int16_t PID_Controller_GetSetpoint(void);

/* 获取有效设定值 (用户设定 + 自动加温抬升, 参与 PID 计算) */
int16_t PID_Controller_GetEffectiveSetpoint(void);

/* 设置当前实测温度 */
void PID_Controller_SetMeasured(int16_t measured);
int16_t PID_Controller_GetMeasured(void);

/*
 * 执行分段 PID 计算, 返回加热功率 0 ~ PID_OUTPUT_MAX
 * - ΔT > 50°C: 全速加热
 * - 30°C < ΔT ≤ 50°C: 激进 PID
 * - ΔT ≤ 30°C: 保守 PID
 * - 积分: 分离 + 内置 30% 限幅; 微分: 先行 + 不完全微分低通
 */
int16_t PID_Controller_Compute(void);

/* 设置 PID 参数 (激进组 / 保守组, 均为 *100) */
void PID_Controller_SetAggKp(int16_t kp);
void PID_Controller_SetAggKi(int16_t ki);
void PID_Controller_SetAggKd(int16_t kd);
void PID_Controller_SetConsKp(int16_t kp);
void PID_Controller_SetConsKi(int16_t ki);
void PID_Controller_SetConsKd(int16_t kd);

/* 获取上次输出值 */
int16_t PID_Controller_GetOutput(void);

/* 大焊点自动加温: 在设定值基础上叠加抬升量 (0~+, 单位 °C)
 * 抬升量只影响有效设定值(参与 PID/温度到达计算), 不修改用户原设定值 */
void PID_Controller_SetBoostOffset(int16_t offset);
int16_t PID_Controller_GetBoostOffset(void);

#endif