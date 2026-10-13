#include "menu.h"
#include "oled.h"
#include "handle.h"
#include "config.h"
#include "work_task.h"
#include "confirm.h"
#include "heater.h"
#include "buzzer.h"
#include "cur_cal.h"
#include "toast.h"

/*
 * 手柄型号页: T12 / T245 / T210
 *
 * 用途: 切换当前烙铁手柄型号. 不同型号对应不同参数表
 * (校准曲线 / 温度上限 / PID 参数), 切换后自动保存.
 *
 * 交互 (单一项, 不做多选目录):
 *   - UP/DOWN : 循环切换当前型号 (仅预览, 不落地)
 *   - CLICK / LPRESS: 预览型号已更改(≠当前配置)时弹出确认页;
 *                     未更改时长按直接退出返回上层, 免确认页
 *
 * 确认页 (YES/NO, 由通用 confirm 模板处理, 显示当前选中的型号):
 *   - UP/DOWN : 切换 否/是
 *   - CLICK   : 选"是"→ 写配置+安全闭合后退出; 选"否"→ 取消返回重选
 *   - LPRESS  : 取消, 返回重选
 *
 * 安全: 切换通过 WorkTask_OnHandleChanged() 收敛运行状态——
 *       重载 PID、目标重新钳位到新型号上限.
 */

#define HANDLE_LABEL_NUM  3
static const char *handle_labels[HANDLE_LABEL_NUM] = {"T12", "T245", "T210"};

/* 链检进行中的轮询周期 (ms): 非阻塞链检需页面周期驱动取结论 */
#define HANDLE_CHK_PERIOD  50

static uint8_t   cur_idx = HANDLE_T12;   /* 当前预览/待选中的型号 */
static confirm_t cf;                 /* 确认页子状态机 */
static bool      chk_active;         /* 换头链检进行中(非阻塞) */

static void handle_enter(void *ud);
static void handle_exit(void *ud);
static menu_result_t handle_handler(const menu_cmd_t *cmd, void *ud);
static menu_period_t handle_display(void *ud);
static void handle_apply_switch(void);

static void handle_enter(void *ud)
{
    (void)ud;
    cur_idx       = Config_GetHandleIdx();   /* 以当前型号为初始选中 */
    cf.active = false;                   /* 初始不在确认态 */
    chk_active = false;                  /* 清残留链检状态 */

    /* 门1前移: 进入换头界面即断热(对齐鲍鱼焊台做法), 防止用户握着
     * 滚烫手柄停留在换头页. 取消/确认切换后由 handle_exit 恢复加热使能,
     * 引导进 Cur Cal 时由校准页加热抑制接管. */
    Heater_Disable();
}

static void handle_exit(void *ud)
{
    (void)ud;
    /* 链检未完成就离开本页: 中止脉冲并清测量占空比, 防随后 Heater_Enable 重放 */
    chk_active = false;
    WorkTask_PulseAbort();

    /* 离开换头页恢复加热使能: 进入页时已断热(门1), 取消/确认切换/引导进
     * Cur Cal 后由 PID 周期或校准页加热抑制接管. 存在锁存故障时不使能
     * (故障门在下一 PID 周期也强制断热, 这里避免残差功率输出窗口). */
    if (!WorkTask_HasFault())
    {
        Heater_SetPower(0);
        Heater_Enable();
    }
}


/* 链检通过后落地切换: 写配置 → 安全闭合 → 必要时引导标定 / 回 HOME.
 * 由 display 在轮询到结论后调用(链检期间已丢弃输入, cur_idx 不会被改). */
static void handle_apply_switch(void)
{
    bool saved;

    /* 确认切换: 写配置 + 触发安全闭合 */
    g_config.handle_type = cur_idx;
    saved = Config_Save();
    WorkTask_OnHandleChanged();
    if (saved)
        Toast_Saved();   /* 仅在确实落盘成功时弹框 */
    else
        Buzzer_Beep(BUZZER_EVENT_ALARM);   /* 失败必须告警, 否则用户以为已存下 */

#if !HW_SIM_MODE
    /* 切换到尚无冷态电流参考(未标定)的型号: 直接引导进 Cur Cal 完成冷态标定,
     * 之后细匹配保护才对新型号生效; 引导页已抑制加热, 热态头先显示 Cooling
     * 等降温. 模拟模式跳过: TypeCal 不落盘、参考值恒 0, 照此每次切型号都会被
     * 送进 Cur Cal 首次引导页(该页 cal_lock 有意忽略 UP/DOWN), 表现为旋转无反应. */
    if (TypeCal_GetRef(cur_idx) == 0)
    {
        CurCal_BootEnter();
        return;
    }
#endif

    /* 切换成功直接回 HOME: 切换是"完成态"(断热/PID 重载/目标钳位/Err4 自动重检
     * 均已收敛), 回 HOME 看温度继续焊接是用户自然下一步, 省一步导航.
     * Menu_SetCurrentId 自动 exit 本页(handle_exit 恢复加热使能). */
    Menu_SetCurrentId(MENU_HOME_ID);
}

static menu_result_t handle_handler(const menu_cmd_t *cmd, void *ud)
{
    (void)ud;

    /* ---- 链检进行中: 丢弃一切输入 ---- */
    /* 结论由 display 轮询取回. 此处忽略事件, 防止 cur_idx 在检测期间被改,
     * 造成"检测的是 A 型号、落地的是 B 型号". 链检约一个 PID 周期(200ms). */
    if (chk_active)
        return MENU_RET_OK;

    /* ---- 确认页处理 (优先) ---- */
    if (cf.active)
    {
        bool confirmed = false;
        (void)confirm_handle(cmd, &cf, &confirmed);
        if (confirmed)
        {
            /* 目标型号 ≠ 当前型号: 启动非阻塞链路检测(热电偶 → 发热芯测量脉冲
             * → 供电 + 目标型号冷态阻值匹配), 结论由 display 轮询后落地.
             * 按目标型号 cur_idx 判定: 换头场景下插入的正是目标头, 匹配通过才
             * 允许切换; 防止"选了型号却插着别的头"的误操作(如 T12 头被选成
             * T245 烧 450°C). */
            if (cur_idx != Config_GetHandleIdx())
            {
                if (!WorkTask_CheckChainStart(cur_idx))
                {
                    Buzzer_Beep(BUZZER_EVENT_ALARM);
                    cf.active = false;   /* 未受理(故障/自检中/已有测量): 退回重选 */
                    return MENU_RET_OK;
                }
                chk_active = true;       /* 进入链检: 结果由 display 轮询 */
                return MENU_RET_OK;
            }

            /* 同型号确认(未真正切换): 返回上一层 */
            return MENU_RET_DONE;
        }
        /* 选"否"/长按: 取消, 返回单型号视图重选; 其余仍在确认态 */
        return MENU_RET_OK;
    }

    /* ---- 单个型号视图 ---- */
    switch(cmd->event)
    {
        case MENU_EVENT_UP:
            cur_idx = (cur_idx + 1 >= HANDLE_LABEL_NUM) ? 0 : (uint8_t)(cur_idx + 1);
            break;
        case MENU_EVENT_DOWN:
            cur_idx = (cur_idx == 0) ? (uint8_t)(HANDLE_LABEL_NUM - 1) : (uint8_t)(cur_idx - 1);
            break;
        case MENU_EVENT_CLICK:
        case MENU_EVENT_LPRESS:
            /* 焦点已变化(预览≠当前配置) → CLICK 与长按均可弹确认页;
             * 型号未更改 → 长按直接退出返回上层, 免去无效确认 */
            if (cur_idx != Config_GetHandleIdx())
                confirm_open(&cf);
            else if (cmd->event == MENU_EVENT_LPRESS)
                return MENU_RET_DONE;
            break;
        default:
            break;
    }
    return MENU_RET_OK;
}

static menu_period_t handle_display(void *ud)
{
    (void)ud;
    menu_period_t period = 0;

    /* ---- 链检进行中: 轮询结论 ---- */
    /* 非阻塞链检的结果只能由周期渲染驱动取回(handler 仅在事件时被调用).
     * 期间保持确认页画面, 每 HANDLE_CHK_PERIOD 复询一次.
     * 结论就绪后在此落地: 本页 display 由 menu_page_display 调用, 后者已在调用
     * 前取好 item、调用后立即返回, 故此处发起跳转不会破坏渲染路径. */
    if (chk_active)
    {
        bool ok = false;
        work_pulse_t st = WorkTask_CheckChainPoll(&ok);

        if (st == WORK_PULSE_BUSY)
        {
            period = HANDLE_CHK_PERIOD;   /* 结果未就绪: 保持画面, 稍后再轮询 */
        }
        else
        {
            /* 必须同时处理 READY 与 IDLE: 后者表示脉冲被中止(锁存故障/关机等),
             * 若只认 READY, chk_active 将永远为真 —— 而 handler 在 chk_active 时
             * 丢弃一切输入, 用户会卡在确认页无法退出(需断电). */
            chk_active = false;
            if (st != WORK_PULSE_READY || !ok)
            {
                Buzzer_Beep(BUZZER_EVENT_ALARM);
                cf.active = false;        /* 未通过/被中止: 退回单型号视图重选 */
            }
            else
            {
                handle_apply_switch();    /* 通过: 落地切换(可能跳 Cur Cal / HOME) */
                /* 本跳转由 display 发起, 不在事件路径上(无人替本页调
                 * UI_RequestRefresh), 故返回一个短周期, 保证下一帧渲染新页面. */
                return HANDLE_CHK_PERIOD;
            }
        }
    }

    OLED_Clear();

    /* ---- 确认页显示 (标题带当前型号) ---- */
    if (cf.active)
    {
        /* 直接拼接显示, 不经 snprintf/stdio: 各段都是字面量, 宽度可在调用点算准.
         * 文案为 "切换到" + 型号 + "?", 分两步拼接(本页只有这一处, 不值得引入变参拼接).
         * confirm_display 仍负责水平居中, 故这里只传拼好的标题字符串 */
        char title[20];
        oled_pair_cat(title, sizeof(title), "切换到", handle_labels[cur_idx]);
        oled_pair_cat(title, sizeof(title), title, "?");
        confirm_display(&cf, title);
        OLED_UpdateDiff();
        return period;
    }

    /* ---- 单个型号视图: 显示当前选中, 切换预览 ---- */
    OLED_ShowChinese(8, 0, "手柄");   /* 标题 y0, 与其它页统一 */

    /* 当前型号大号反显框 (宽 40 容纳 "T245", 整行居中 x44..83):
     * 文字 8x16 在 18px 高框内上下各留 1px; 标题(0..15)/型号框(27..44)/
     * 提示(56)三行间距均衡 */
    OLED_ShowString(48, 28, handle_labels[cur_idx], OLED_8X16);
    OLED_ReverseRoundRect(44, 27, 40, 18, 6);

    /* 底部操作提示 (6x8, 统一"键名: 动作"形式, 两行各居中):
     * CLK: Switch = 型号已改动时按键弹确认页切换; HLD: Exit = 未改动时长按退出 */
    OLED_ShowString(21, 48, "CLK: Switch", OLED_6X8);   /* 11 字 86px x21..107 */
    OLED_ShowString(29, 56, "HLD: Exit", OLED_6X8);     /* 9 字 70px x29..99 */

    OLED_UpdateDiff();
    return period;
}

const menu_ops_t handle_ops = {
    .enter   = handle_enter,
    .handler = handle_handler,
    .display = handle_display,
    .exit    = handle_exit,
};
