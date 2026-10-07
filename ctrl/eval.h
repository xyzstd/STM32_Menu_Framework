#ifndef __EVAL_H
#define __EVAL_H

#include <stdint.h>
#include <stdbool.h>

/* 负载响应评估: 恒温下真焊焊点, 记录 90s 温度曲线, 在线单遍提取
 *  最大跌落 / 恢复时间 / 过冲 / 尾段波动, 按规则给出"偏激/偏保守/合适"
 *  结论与 Kp 缩放建议(Scale), 由用户在 Slow PID 页手动应用.
 * 状态机纯计算无副作用, 由 work_task 每 PID 周期喂入实测温度. */

typedef enum{
    EVAL_IDLE = 0,   /* 未开始 */
    EVAL_REC,        /* 记录中 */
    EVAL_DONE,       /* 已出结果 */
}eval_state_t;

typedef struct{
    int16_t  drop;         /* 最大跌落 °C (基准 - 最低) */
    uint16_t recv_s;       /* 恢复时间 s (跌落到回到基准-5°C 内) */
    int16_t  overshoot;    /* 恢复后过冲 °C */
    int16_t  swing;        /* 尾段 10s 峰峰波动 °C */
    uint8_t  scale_x10;    /* 建议 Kp 缩放 ×10 (7/10/12), 0=无建议 */
    bool     load;         /* 是否检测到真实负载 (跌落 ≥ 8°C) */
}eval_result_t;

void         Eval_Reset(void);        /* 回 IDLE, 页面离场时调用 */
void         Eval_Start(void);        /* 开始记录 (调用方须先校验手柄/校准) */
void         Eval_Stop(void);         /* 提前结束 → 立即出结果 */
bool         Eval_IsActive(void);     /* 记录中 (work_task 据此抑制休眠) */
eval_state_t Eval_GetState(void);
uint16_t     Eval_GetElapsedSec(void);
eval_result_t Eval_GetResult(void);

/* 内部: 每 PID 周期由 work_task 喂入实测温度 (仅 REC 时采样) */
void eval_advance(int16_t temp);

#endif
