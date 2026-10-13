#include "stm32f10x.h"
#include "timer.h"
#include "menu_display.h"
#include "menu.h"
#include "work_task.h"
#include "cur_cal.h"
#include "input.h"
#include "toast.h"

/* 渲染合并窗口(ms): 事件高频到达(旋转)时, 两次渲染之间至少间隔该时长.
 * 输入采样已迁入 TIM2 1ms 中断(Input_Scan), 渲染阻塞主循环不再拖延输入轮询,
 * 故合并窗口可明显小于旧值: 兼顾旋转跟手(高亮及时刷新)与合并高频事件(跳过中间帧).
 * 2ms = 编码器 4ms 轮询周期的一半 → 每个事件到达后渲染尽快响应,
 * 配合 Input_Process 每轮限流 2 事件, 事件按到达顺序逐帧渲染(每帧 ≤2 格),
 * 消除快速旋转"先不动后连续跳格"(4ms 窗口把一批事件合并成一次渲染).
 * 静止时零请求零渲染, 不消耗 CPU. */
#define UI_RENDER_COOLDOWN_MS  2

static uint16_t ui_refresh_period = 0;
static uint32_t ui_refresh_tick = 0;
static uint32_t ui_last_render_tick = 0;
static volatile bool ui_refresh_request = false;
static bool ui_fault_prev = false;   /* 上一轮故障态, 用于上升沿检测 */
static bool toast_pending_prev = false;   /* 上一轮提示框是否仍在显示 */


void UI_RequestRefresh(void)
{
    ui_refresh_request = true;
}


/* ==================== 保存提示框 ====================
 * 提示框本身已下移到页面层(page/toast.c): 它只依赖 OLED 与 tick, 且是"页面如何
 * 反馈"的视觉细节. 原先放在这里, 迫使 5 个页面 include "ui_task.h" 形成反向依赖.
 * 本文件只负责调度 —— 渲染后叠加、到期后再渲染一帧把框擦掉. */


void UI_Task_Init(void)
{
    /* 注册刷新请求 hook: 动画需要"再渲染一帧"时由 menu 层回调到这里.
     * 单向依赖 —— menu 层不再声明/引用本层的 UI_RequestRefresh(). */
    MenuDisplay_SetRefreshHook(UI_RequestRefresh);

    ui_refresh_period = MenuDisplay_Render();
    ui_refresh_tick = Timer_GetTick();
    ui_last_render_tick = ui_refresh_tick;
}


void UI_Task_Process(void)
{
    uint32_t now = Timer_GetTick();

    /* 锁存故障 UI 兜底: 任意页面发生故障(热电偶/过温/加热器)时回到常驻页 HOME
     * 统一处理. 蜂鸣报警已由 work_task 锁存点全局触发; HOME 显示故障码,
     * 长按走 WorkTask_Recover 重跑安全检测状态机. 仅上升沿触发一次. */
    {
        bool fault = WorkTask_HasFault();

        if (fault && !ui_fault_prev)
        {
            /* 调节态的清理不必在此手工做: 若当前停在单参数页上, Menu_SetCurrentId
             * 会经 menu_exit_handler 调该页 ops->exit(内含 param_exit 落盘)。 */
            Input_ClearEvents(); /* 丢弃原页面积压事件, 防 HOME 错位消费(误下钻/恢复/调温) */
            Input_ResyncEncoder(); /* 故障跳转非事件触发: 一并重同步编码器相位, 防跳转后第一格被残留吃掉 */
            Menu_SetCurrentId(MENU_HOME_ID);
            UI_RequestRefresh();
        }
        ui_fault_prev = fault;
    }

    /* 首次冷态标定引导: 上电自检通过且当前型号未标定时, 自动进入 Cur Cal
     * 引导页 (First Cal), 用户标定完成或跳过后清除引导标志.
     * 仅当处于 HOME 且无故障时触发一次(进入 Cur Cal 后当前页即离开 HOME,
     * 不会重复触发); 故障存在时让位给上方故障兜底处理. */
    if (WorkTask_IsBootCalPending() && !WorkTask_HasFault() &&
        Menu_GetCurrentId() == MENU_HOME_ID)
    {
        /* 与上方故障上升沿路径对称: 跳页前丢弃积压事件,
         * 防止引导切页后残留 UP/DOWN 意外切换 Tip Cal 预选型号(cal_lock=false)。
         * (调节态清理同故障路径, 由 Menu_SetCurrentId → menu_exit_handler 完成。) */
        Input_ClearEvents();
        Input_ResyncEncoder(); /* 引导跳转非事件触发: 一并重同步编码器相位 */
        CurCal_BootEnter();
        UI_RequestRefresh();
    }


    /* 提示框到期: Toast_Pending() 会顺手清掉到期状态, 这里发现"上一帧还在、
     * 现在没了"就请求一帧重绘, 把叠加框擦掉(页面重绘后 Toast_Overlay 不再画). */
    if (toast_pending_prev && !Toast_Pending())
        UI_RequestRefresh();
    toast_pending_prev = Toast_Pending();

    if(ui_refresh_request)
    {
        /* 合并帧: 距上次渲染不足窗口则保留请求稍后渲染一次最新状态,
         * 防止快速旋转时每格都全量重绘抢占输入; 到点消费并重绘. */
        if((now - ui_last_render_tick) < UI_RENDER_COOLDOWN_MS)
            return;
        ui_refresh_request = false;
        ui_last_render_tick = now;
        ui_refresh_tick = now;
        ui_refresh_period = MenuDisplay_Render();
        (void)Toast_Overlay();   /* 渲染后叠加保存框 */
    }
    else if(ui_refresh_period != 0)
    {
        if((now - ui_refresh_tick) >= ui_refresh_period)
        {
            ui_refresh_tick = now;
            ui_last_render_tick = now;
            ui_refresh_period = MenuDisplay_Render();
            (void)Toast_Overlay();   /* 渲染后叠加保存框 */
        }
    }
}
