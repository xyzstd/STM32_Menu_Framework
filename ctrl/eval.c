#include "eval.h"

/* 负载响应评估状态机 (纯计算, 无 I/O, 可独立测试):
 * 以"起始温度"为基准, 恒温下用户焊焊点 → 实测跌出基准-5°C 视为负载,
 * 回到基准-5°C 内视为恢复; 恢复后最高点为过冲; 尾段 10s 用 50 样本
 * 环形窗算峰峰波动. 90s(450 tick @ 200ms)或手动提前结束时按规则判
 * 结论: 过冲>15°C 偏激 → Kp×0.7; 恢复>30s 或尾段波动>8°C 偏保守 →
 * Kp×1.2; 否则合适 → ×1.0. */

#define EVAL_DROP_HYS     5     /* °C: 低于基准此值判定为负载跌落 */
#define EVAL_LOAD_MIN     8     /* °C: 最大跌落不足此值判"未焊负载" */
#define EVAL_MAX_TICKS    450   /* 90s @ 200ms */
#define EVAL_TAIL_N       50    /* 尾段波动窗口: 10s @ 200ms */

#define EVAL_OS_LIMIT     15    /* 过冲上限 °C */
#define EVAL_RECV_S_LIMIT 30    /* 恢复时间上限 s */
#define EVAL_SWING_LIMIT  8     /* 尾段波动上限 °C */

static eval_state_t state;
static int16_t  base;        /* 起始基准温度 */
static int16_t  min_temp;    /* 全局最低 */
static bool     in_drop;     /* 处于跌落区 */
static bool     recovered;   /* 已回到基准附近 */
static uint16_t drop_tick;   /* 首次进入跌落区时刻 */
static uint16_t recv_tick;   /* 恢复时刻 */
static int16_t  post_max;    /* 恢复后最高 */
static int16_t  tail[EVAL_TAIL_N];   /* 尾段窗口环形缓冲 */
static uint8_t  tail_i, tail_cnt;
static uint16_t tick;
static eval_result_t result;

static void eval_finish(void);


static void eval_finish(void)
{
    int16_t tail_min = 0, tail_max = 0;
    uint8_t i;

    if (tail_cnt > 0)
    {
        tail_min = tail_max = tail[0];
        for (i = 1; i < tail_cnt; i++)
        {
            if (tail[i] < tail_min) tail_min = tail[i];
            if (tail[i] > tail_max) tail_max = tail[i];
        }
    }

    result.drop      = (tick > 0) ? (int16_t)(base - min_temp) : 0;  /* 无采样即结束: 防 0-32767 假跌落 */
    result.load      = (result.drop >= EVAL_LOAD_MIN);
    result.recv_s    = 0;
    result.overshoot = 0;
    result.swing     = (int16_t)(tail_max - tail_min);
    result.scale_x10 = 0;

    if (result.load)
    {
        uint16_t span = recovered ? (uint16_t)(recv_tick - drop_tick)
                                  : (uint16_t)(tick - drop_tick);
        result.recv_s = (uint16_t)(span / 5);      /* 200ms/格 → 秒 */
        result.overshoot = (int16_t)(post_max - base);
        if (result.overshoot > EVAL_OS_LIMIT)
            result.scale_x10 = 7;
        else if (result.recv_s > EVAL_RECV_S_LIMIT || result.swing > EVAL_SWING_LIMIT)
            result.scale_x10 = 12;
        else
            result.scale_x10 = 10;
    }

    state = EVAL_DONE;
}

void Eval_Reset(void)
{
    state = EVAL_IDLE;
    tick = 0;
    tail_i = 0;
    tail_cnt = 0;
}

void Eval_Start(void)
{
    Eval_Reset();
    state = EVAL_REC;
    base = 0;
    min_temp = 32767;
    in_drop = false;
    recovered = false;
    drop_tick = 0;
    recv_tick = 0;
    post_max = 0;
}

void Eval_Stop(void)
{
    if (state == EVAL_REC)
        eval_finish();
}

bool Eval_IsActive(void)
{
    return (state == EVAL_REC);
}

eval_state_t Eval_GetState(void)
{
    return state;
}

uint16_t Eval_GetElapsedSec(void)
{
    return (uint16_t)(tick / 5);
}

eval_result_t Eval_GetResult(void)
{
    return result;
}

void eval_advance(int16_t temp)
{
    if (state != EVAL_REC)
        return;

    tick++;

    /* 尾段窗口: 环形缓冲 50 样本 (10s) */
    tail[tail_i] = temp;
    tail_i = (uint8_t)((tail_i + 1) % EVAL_TAIL_N);
    if (tail_cnt < EVAL_TAIL_N)
        tail_cnt++;

    if (tick == 1)
    {
        base = temp;          /* 起始基准: 用户应先等恒温再开始 */
        min_temp = temp;
        post_max = temp;
        return;
    }

    if (temp < min_temp)
        min_temp = temp;

    if (!in_drop && temp < (int16_t)(base - EVAL_DROP_HYS))
    {
        in_drop = true;
        drop_tick = tick;
    }

    if (in_drop && !recovered && temp >= (int16_t)(base - EVAL_DROP_HYS))
    {
        recovered = true;
        recv_tick = tick;
    }

    if (recovered && temp > post_max)
        post_max = temp;

    if (tick >= EVAL_MAX_TICKS)
        eval_finish();
}
