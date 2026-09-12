#include "stm32f10x.h"
#include "timer.h"
#include "menu_display.h"

/* 渲染合并窗口(ms): 事件高频到达(旋转)时, 两次渲染之间至少间隔该时长.
 * 全屏缓冲重建 + 差量推送在真机上耗时可观, 若每 10ms 事件都立即渲染,
 * 渲染会抢占主循环, 拖慢下一轮输入扫描, 表现为"旋转时 UI 卡顿".
 * 合并后输入以 10ms 节拍照常推进状态, 屏幕以约 33fps 渲染最新一帧, 停手补帧. */
#define UI_RENDER_COOLDOWN_MS  30

static uint16_t ui_refresh_period = 0;
static uint32_t ui_refresh_tick = 0;
static uint32_t ui_last_render_tick = 0;
static volatile bool ui_refresh_request = false;


void UI_RequestRefresh(void)
{
    ui_refresh_request = true;
}


void UI_Task_Init(void)
{
    ui_refresh_period = UI_Render();
    ui_refresh_tick = Timer_GetTick();
    ui_last_render_tick = ui_refresh_tick;
}


void UI_Task_Process(void)
{
    uint32_t now = Timer_GetTick();

    if(ui_refresh_request)
    {
        /* 合并帧: 距上次渲染不足窗口则保留请求稍后渲染一次最新状态,
         * 防止快速旋转时每格都全量重绘抢占输入; 到点消费并重绘. */
        if((now - ui_last_render_tick) < UI_RENDER_COOLDOWN_MS)
            return;
        ui_refresh_request = false;
        ui_last_render_tick = now;
        ui_refresh_tick = now;
        ui_refresh_period = UI_Render();
    }
    else if(ui_refresh_period != 0)
    {
        if((now - ui_refresh_tick) >= ui_refresh_period)
        {
            ui_refresh_tick = now;
            ui_last_render_tick = now;
            ui_refresh_period = UI_Render();
        }
    }
}
