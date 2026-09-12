#include <stdio.h>
#include "menu.h"
#include "oled.h"
#include "handle.h"
#include "config.h"
#include "work_task.h"
#include "confirm.h"

/*
 * 手柄型号页: T12 / T245 / T210
 *
 * 用途: 切换当前烙铁手柄型号. 不同型号对应不同参数表
 * (校准曲线 / 温度上限 / Boost 上限 / PID 参数), 切换后自动保存.
 *
 * 交互 (单一项, 不做多选目录):
 *   - UP/DOWN : 循环切换当前型号 (仅预览, 不落地)
 *   - LPRESS  : 返回上级必经确认页
 *
 * 确认页 (YES/NO, 由通用 confirm 模板处理, 显示当前选中的型号):
 *   - UP/DOWN : 切换 否/是
 *   - CLICK   : 选"是"→ 写配置+安全闭合后退出; 选"否"→ 取消返回重选
 *   - LPRESS  : 取消, 返回重选
 *
 * 安全: 切换通过 WorkTask_OnHandleChanged() 收敛运行状态——
 *       重载 PID、目标重新钳位到新型号上限、清 Boost 抬升.
 */

#define HANDLE_LABEL_NUM  3
static const char *handle_labels[HANDLE_LABEL_NUM] = {"T12", "T245", "T210"};

static uint8_t   cur_idx = HANDLE_T12;   /* 当前预览/待选中的型号 */
static confirm_t cf;                 /* 确认页子状态机 */

static void handle_enter(void *ud)
{
    (void)ud;
    cur_idx       = Config_GetHandleIdx();   /* 以当前型号为初始选中 */
    cf.active = false;                   /* 初始不在确认态 */
}

static menu_result_t handle_handler(const menu_cmd_t *cmd, void *ud)
{
    (void)ud;

    /* ---- 确认页处理 (优先) ---- */
    if (cf.active)
    {
        bool confirmed = false;
        (void)confirm_handle(cmd, &cf, &confirmed);
        if (confirmed)
        {
            /* 确认切换: 写配置 + 触发安全闭合, 然后退出返回上级 */
            g_config.handle_type = cur_idx;
            Config_Save();
            WorkTask_OnHandleChanged();
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
        case MENU_EVENT_LPRESS:
            /* 返回上级必经确认页 */
            confirm_open(&cf);
            break;
        default:
            break;
    }
    return MENU_RET_OK;
}

static menu_period_t handle_display(void *ud)
{
    (void)ud;
    OLED_Clear();

    /* ---- 确认页显示 (标题带当前型号) ---- */
    if (cf.active)
    {
        char buf[24];
        (void)snprintf(buf, sizeof(buf), "Switch to %s?", handle_labels[cur_idx]);
        confirm_display(&cf, buf);
        OLED_UpdateDiff();
        return 0;
    }

    /* ---- 单个型号视图: 显示当前选中, 切换预览 ---- */
    OLED_ShowString(8, 8, "Handle:", OLED_8X16);

    /* 当前型号大号反显框 (宽 40 容纳 "T245") */
    OLED_ShowString(28, 32, handle_labels[cur_idx], OLED_8X16);
    OLED_ReverseRoundRect(24, 30, 40, 18, 6);

    /* 底部提示 */
    OLED_ShowString(8, 54, "HOLD to Switch", OLED_6X8);

    OLED_UpdateDiff();
    return 0;
}

const menu_ops_t handle_ops = {
    .enter   = handle_enter,
    .handler = handle_handler,
    .display = handle_display,
    .exit    = NULL,
};
