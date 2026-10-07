#include "menu.h"
#include "oled.h"
#include "eval.h"
#include "work_task.h"
#include "config.h"
#include "buzzer.h"

/* 负载响应评估页: 进入后先等恒温, CLICK 开始记录 90s(期间真焊焊点),
 * 记录中 CLICK 可提前出结果; 结束后显示 跌落/恢复/过冲/尾段波动 与
 * Kp 缩放建议(手动到 Slow PID 页应用). LPRESS/离场丢弃本次数据.
 * 评估状态机在 ctrl/eval.c, 本页只做交互与显示. */

static void eval_start(void);
static menu_result_t eval_handler(const menu_cmd_t *cmd, void *ud);
static void eval_exit(void *ud);
static void eval_center(const char *s, uint8_t y);
static void eval_row_set(int16_t set, uint8_t y);
static void eval_row_now(int16_t meas, uint8_t y);
static menu_period_t eval_display(void *ud);

static void eval_start(void)
{
    /* 入口校验: 存在锁存故障(热电偶短路/开路等)或当前型号校准无效时,
     * 加热被 run_pid_cycle 拦截, 评估只会记录一条无意义的恒温线 → 拒绝启动并蜂鸣 */
    if (WorkTask_HasFault() || !Cal_IsHandleValid(Config_GetHandleIdx()))
    {
        Buzzer_Beep(BUZZER_EVENT_ALARM);
        return;
    }
    Eval_Start();
}

static menu_result_t eval_handler(const menu_cmd_t *cmd, void *ud)
{
    (void)ud;
    if (cmd->event == MENU_EVENT_CLICK)
    {
        switch (Eval_GetState())
        {
        case EVAL_IDLE: eval_start();  break;   /* 开始记录 */
        case EVAL_REC:  Eval_Stop();   break;   /* 提前出结果 */
        default:        Eval_Start();  break;   /* DONE: 重新测一轮 */
        }
        return MENU_RET_OK;
    }
    if (cmd->event == MENU_EVENT_LPRESS)
        return MENU_RET_DONE;
    return MENU_RET_OK;
}

static void eval_exit(void *ud)
{
    (void)ud;
    Eval_Reset();
}

/* 混合宽度水平居中: ASCII 8px/字符, 中文 16px/字符 */
static void eval_center(const char *s, uint8_t y)
{
    uint8_t w = 0;
    const char *p = s;
    while (*p)
    {
        if ((*p & 0x80) == 0) { w += 8;  p++; }
        else                  { w += 16; p += 3; }
    }
    OLED_ShowChinese((int16_t)(128 - (int16_t)w) / 2, y, s);
}

/* 第 1/2 行: 目标/当前温度
 * 数值列 x=48 与 Cur Cal 页 Meas 列、Auto Tune 页 SET/NOW 列统一对齐
 * (标签 2 汉字占 32px, x=8..40, 值列 x=48; 度符号 x=72, C x=80) */
static void eval_row_set(int16_t set, uint8_t y)
{
    OLED_ShowChinese(8, y, "设定");
    OLED_ShowNum(48, y, set, 3);
    OLED_ShowDegree(72, y);
    OLED_ShowChar(80, y, 'C', OLED_8X16);
}

static void eval_row_now(int16_t meas, uint8_t y)
{
    OLED_ShowChinese(8, y, "当前");
    OLED_ShowNum(48, y, meas, 3);
    OLED_ShowDegree(72, y);
    OLED_ShowChar(80, y, 'C', OLED_8X16);
}

static menu_period_t eval_display(void *ud)
{
    (void)ud;
    int16_t set  = WorkTask_GetRuntimeTemp();
    int16_t meas = WorkTask_GetMeasuredTemp();

    OLED_Clear();

    switch (Eval_GetState())
    {
    case EVAL_IDLE:
        eval_center("负载评估", 0);
        eval_row_set(set, 16);
        eval_row_now(meas, 32);
        OLED_ShowString(25, 56, "CLK: Start", OLED_6X8);   /* 6x8 10 字 78px 居中 */
        break;

    case EVAL_REC:
        eval_center("记录中", 0);
        eval_row_set(set, 16);
        eval_row_now(meas, 32);
        OLED_ShowChinese(8, 48, "时间");
        OLED_ShowNum(48, 48, Eval_GetElapsedSec(), 3);
        OLED_ShowChinese(72, 48, "秒/90");
        break;

    default: /* EVAL_DONE */
    {
        eval_result_t r = Eval_GetResult();
        const char *verdict;

        if (!r.load)                    verdict = "未焊接";
        else if (r.scale_x10 == 7)      verdict = "激进x0.7";
        else if (r.scale_x10 == 12)     verdict = "保守x1.2";
        else                            verdict = "平稳";
        eval_center(verdict, 0);

        OLED_ShowChinese(8, 16, "跌落");
        OLED_ShowNum(48, 16, r.drop, 3);
        OLED_ShowChar(72, 16, 'C', OLED_8X16);

        OLED_ShowChinese(8, 32, "恢复");
        OLED_ShowNum(48, 32, r.recv_s, 3);
        OLED_ShowChinese(72, 32, "秒");

        /* 无单位两列: 过冲 x8..40 值 x40..64 / 波动 x64..96 值 x96..120 */
        OLED_ShowChinese(8, 48, "过冲");
        OLED_ShowNum(40, 48, r.overshoot, 3);
        OLED_ShowChinese(64, 48, "波动");
        OLED_ShowNum(96, 48, r.swing, 3);
        break;
    }
    }

    OLED_UpdateDiff();
    return 0;
}

const menu_ops_t eval_ops = {
    .handler = eval_handler,
    .display = eval_display,
    .exit = eval_exit,
};
