#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "heater.h"   /* HEATER_POWER_LIMIT: PID 输出满量程须与之相等 */

/* PID 输出范围: 0 ~ PID_OUTPUT_MAX, 即 0 ~ 加热层实际可达的最大功率.
 * 必须等于 HEATER_POWER_LIMIT(900) 而不是 HEATER_POWER_MAX(1000):
 * 加热层为保留 ≥10% 关断窗(供温度采样)会把功率钳到 900; 若 PID 仍以 1000 为
 * 满量程, 抗饱和就按"不可达的上限"计算 —— 积分可比实际出力多累积约 10% 占空比,
 * 退出饱和时需额外泄放, 表现为过冲. 本定义强制二者一致, 改一处即同步. */
#define PID_OUTPUT_MAX  HEATER_POWER_LIMIT

/* 积分限幅 (内置固定值, 不提供页面调节):
 *   保守区(ΔT≤30°C): PID 输出满量程的 30%
 *   激进区(30<ΔT≤50°C): 收紧到 15%, 让 agg_ki 生效又不致大误差饱和 */
#define PID_INTEGRAL_LIMIT      ((PID_OUTPUT_MAX * 30) / 100)
#define PID_INTEGRAL_LIMIT_AGG  ((PID_OUTPUT_MAX * 15) / 100)

/* PID 参数存储格式: 实际值 = 存储值 / 100 */
#define PID_SCALE       100

/* 温度下限固定 100°C; 上限按型号在运行时由 Config_GetTempMax() 提供
 * (T12/T245/T210 均为 450), 不再使用固定宏, 保证切型号后目标不越标称上限 */
#define TEMP_MIN        100

void PID_Controller_Init(void);
void PID_Controller_Reload(void);   /* 型号切换后按当前型号系数重载 PID 参数并复位 */
void PID_Controller_Reset(void);

/* 设置目标温度 (单位: °C) */
void PID_Controller_SetSetpoint(int16_t setpoint);
int16_t PID_Controller_GetSetpoint(void);

/* 设置当前实测温度 */
void PID_Controller_SetMeasured(int16_t measured);
int16_t PID_Controller_GetMeasured(void);

/*
 * 执行分段 PID 计算, 返回加热功率 0 ~ PID_OUTPUT_MAX
 * - ΔT > 50°C: 全速加热
 * - 30°C < ΔT ≤ 50°C: 激进 PID
 * - ΔT ≤ 30°C: 保守 PID
 * - 积分: 全速区清零, 激进区累计(限幅 15%), 保守区限幅 30%;
 *          微分: 先行 + 不完全微分低通
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

/* ============ 继电反馈自整定 (Aström–Hägglund) ============
 * 启动后, run_pid_cycle 每周期调用 PID_TuneAdvance(meas) 取得继电输出,
 * 温度围绕设定点等幅振荡, 4 个完整周期后由描述函数法 + Ziegler-Nichols
 * 得出保守组(稳态段) PID 参数. 整定期间跳过 PID/前馈/加热诊断.
 *
 * 进度单位: PID_TuneAdvance 内部按"半周期"计数(TUNE_HALF_REQUIRED 8 = 4 个
 * 完整周期). PID_TuneProgressCycles() 返回完整周期数(0~4), 页面的 "N/4"
 * 指示与"周期"字样即以此为准; 内部半周期计数不再对外暴露, 避免单位误用. */
typedef struct { int16_t kp; int16_t ki; int16_t kd; } PID_Gain_t;   /* *100 存储格式 */

void PID_TuneStart(int16_t setpoint);
int16_t PID_TuneAdvance(int16_t measured);  /* 返回加热功率 0~PID_OUTPUT_MAX */
bool PID_TuneActive(void);
bool PID_TuneFinish(void);                  /* 已结束(成功或失败) */
/* 最近一次整定是否成功. 结果被保存下来: PID_TuneAbort() 会清 finished/success,
 * 而页面在工作层已 Abort 之后仍需显示结果, 故单独留一份判定供 UI 使用,
 * 直到下次 PID_TuneStart 才刷新. */
bool PID_TuneLastSucceeded(void);
int16_t PID_TuneProgressCycles(void);       /* 已完成完整周期数 0 ~ PID_TuneCyclesRequired() */
int16_t PID_TuneCyclesRequired(void);       /* 整定所需的完整周期总数 (页面 "N/x" 的分母) */
void PID_TuneGetResult(PID_Gain_t *g);
void PID_TuneAbort(void);

#endif
