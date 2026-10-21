#include "menu.h"
#include "oled.h"
#include "autotune.h"
#include "config.h"
#include "work_task.h"
#include "pid_controller.h"
#include "buzzer.h"

/* 继电自整定页: 进入即以当前运行时目标温度为设定点启动继电振荡,
 * 页内显示进度(整定目标/当前温度/已完成周期); 完成自动落盘(work_task 层
 * 写当前型号单组 + Config_Save + 蜂鸣), 本页只反馈结果.
 * 中途退出 → 中止未完成整定, 恢复原 PID 参数. */

static int16_t tune_setpoint = 0;    /* 进入时捕获的整定目标温度 (整定期间不变) */
static bool    tune_started = false; /* 本次进入是否成功启动整定 (区分"被拒未启动"与"已失败") */

/* 本页刷新周期: 整定要跑 40~160s, 期间温度与周期数都在变, 但**没有任何输入事件**.
 * 渲染调度只在"输入事件派发后"和"页面返回的周期到期时"重绘(见 main/ui_task.c),
 * 故返回 0 会让画面从进入那一刻冻结, 直到用户去拧旋钮. 200ms 足够跟上 1°C/s 的
 * 温度变化; 静止帧由 OLED_UpdateDiff 差量判脏, 无变化时零 I2C 推送, 代价极低. */
#define TUNE_REFRESH_MS  200

static void tune_enter(void *ud);
static menu_result_t tune_handler(const menu_cmd_t *cmd, void *ud);
static void tune_exit(void *ud);
static menu_period_t tune_display(void *ud);

static void tune_enter(void *ud)
{
    (void)ud;
    /* 入口校验: 存在锁存故障(热电偶短路/开路等)或当前型号校准数据无效时,
     * 加热会被 run_pid_cycle 拦截, 整定只会空转到超时 → 直接拒绝启动 */
    tune_started = (!WorkTask_HasFault() && Cal_IsHandleValid(Config_GetHandleIdx()));
    if (!tune_started)
    {
        Buzzer_Beep(BUZZER_EVENT_ALARM);
        return;
    }
    tune_setpoint = WorkTask_GetRuntimeTemp();   /* 以当前目标温度整定 */
    PID_TuneStart(tune_setpoint);
}

static menu_result_t tune_handler(const menu_cmd_t *cmd, void *ud)
{
    (void)ud;
    if (cmd->event == MENU_EVENT_CLICK || cmd->event == MENU_EVENT_LPRESS)
    {
        if (PID_TuneActive())
            PID_TuneAbort();   /* 主动退出: 中止未完成整定 */
        return MENU_RET_DONE;
    }
    return MENU_RET_STAY;
}

static void tune_exit(void *ud)
{
    (void)ud;
    if (PID_TuneActive())
        PID_TuneAbort();   /* 兜底: 异常离场也中止 */
}

static menu_period_t tune_display(void *ud)
{
    (void)ud;
    int16_t set   = tune_setpoint;      /* 整定目标温度 */
    int16_t meas  = PID_Controller_GetMeasured();
    int16_t prog  = PID_TuneProgressCycles();   /* 完整周期数 0~4 */
    bool    over  = !PID_TuneActive();  /* 已结束(完成或被中止) */

    OLED_Clear();

    /* 标题: 4 汉字 64px, 居中 x=32 */
    OLED_ShowChinese(32, 0, "自动整定");

    /* 第 1 行 (y=16-31): 整定目标温度 */
    /* 数值列 x=48 与 Cur Cal 页 Meas 列、Eval 页 SET/NOW 列统一对齐
     * (标签 2 汉字占 32px, x=8..40, 值列 x=48; 度符号 x=72, C x=80) */
    OLED_ShowChinese(8, 16, "设定");
    OLED_ShowNum(48, 16, set, 3);
    OLED_ShowDegree(72, 16);
    OLED_ShowChar(80, 16, 'C', OLED_8X16);

    /* 第 2 行 (y=32-47): 当前温度 */
    OLED_ShowChinese(8, 32, "当前");
    OLED_ShowNum(48, 32, meas, 3);
    OLED_ShowDegree(72, 32);
    OLED_ShowChar(80, 32, 'C', OLED_8X16);

    /* 第 3 行 (y=48-63): 状态 (统一 8x16, 与上行 16px 行距, 无重叠) */
    if (!tune_started)
        OLED_ShowChinese(8, 48, "无法整定");   /* 入口校验被拒 */
    else if (over)
    {
        /* 算法整定成功与 Flash 保存成功分开判定, 防止保存失败仍显示"完成保存". */
        if (PID_TuneLastSucceeded())
        {
            if (WorkTask_TuneSaveSucceeded())
                OLED_ShowChinese(8, 48, "完成保存");
            else
                OLED_ShowChinese(8, 48, "保存失败");
        }
        else
        {
            OLED_ShowChinese(8, 48, "失败");
        }
    }
    else
    {
        OLED_ShowChinese(8, 48, "周期");
        OLED_ShowNum(48, 48, prog, 1);
        OLED_ShowString(56, 48, "/", OLED_8X16);            /* "/" 非字库字形, 用字符串接口 */
        OLED_ShowNum(64, 48, PID_TuneCyclesRequired(), 1);  /* 分母取自控制器, 不再硬编码 */
    }

    OLED_UpdateDiff();
    return TUNE_REFRESH_MS;   /* 非 0: 整定期间无输入也要周期重绘(见上) */
}

const menu_ops_t autotune_ops = {
    .enter = tune_enter, .handler = tune_handler,
    .display = tune_display, .exit = tune_exit,
};
