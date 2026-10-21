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
 *   小误差区(|ΔT|≤25°C): 满量程的 10%
 *   大误差区(|ΔT|≥35°C): 收紧到 5%
 * 2026-10-20 由 30%/15% 收紧: 稳态功率改由前馈底座(pid_hold)承担, 积分只需
 * 补模型误差; 限幅越小, 接近段累积越少, 过冲越小(仿真: 250°C 过冲 5.2→0.2). */
#define PID_INTEGRAL_LIMIT      ((PID_OUTPUT_MAX * 10) / 100)
#define PID_INTEGRAL_LIMIT_AGG  ((PID_OUTPUT_MAX * 5) / 100)

/* PID 参数存储格式: 实际值 = 存储值 / 100 */
#define PID_SCALE       100

/* 用户可编辑的 PID 增益上限 (*100) —— 单一来源:
 *   - 页面(page/pid.c)用它钳位用户可设范围与载入值;
 *   - 自整定写回(main/work_task.c)用它钳位整定结果.
 * 二者必须同源: 整定短暂可产出 Kd 达 3×10^4(经 tune_gain_sat 饱和到 32767),
 * 若原样存入配置, 用户一进 PID 页就会被页面上限静默钳掉, 且 2 位整数显示
 * 截断高位 —— 同源钳位后不会落盘超出页面可显示/可编辑范围的值. */
#define PID_KP_MAX      2000   /* 20.00, 覆盖默认 11.00 */
#define PID_KI_MAX      500    /* 5.00 */
#define PID_KD_MAX      1000   /* 10.00, 覆盖自整定可能产出的较大 Kd */

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
 * 执行 PID 计算, 返回输出 0 ~ PID_OUTPUT_MAX.
 * feedforward_floor 是工作层最终 PID+前馈输出中的前馈下限:
 * 前馈主导且误差为正时, 积分跟踪前馈下限(受积分限幅约束), 避免接管冲击.
 * - 增益为单组(原双组已于 2026-10-19 合并, 详见 .c 文件头注释);
 * - 积分限幅仍按 |误差| 调度: ≤25°C 给 10%, ≥35°C 收紧到 5%(稳态由前馈底座承担);
 * - 微分: 先行 + 不完全微分低通.
 */
int16_t PID_Controller_Compute(int16_t feedforward_floor);

/* 设置 PID 参数 (单组, 均为 *100) */
void PID_Controller_SetKp(int16_t kp);
void PID_Controller_SetKi(int16_t ki);
void PID_Controller_SetKd(int16_t kd);

/* 获取上次输出值 */
int16_t PID_Controller_GetOutput(void);

/* ============ 继电反馈自整定 (Aström–Hägglund) ============
 * 启动后, run_pid_cycle 每周期调用 PID_TuneAdvance(meas) 取得继电输出,
 * 温度围绕设定点等幅振荡, 4 个完整周期后由描述函数法 + Ziegler-Nichols
 * 得出 PID 参数并写入单组(原为"保守组", 单组化后即唯一的一组).
 * 整定期间跳过 PID/前馈/加热诊断.
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
