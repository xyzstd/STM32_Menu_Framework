#include "stm32f10x.h"
#include "timer.h"
#include "menu_display.h"

static uint16_t ui_refresh_period = 0;
static uint32_t ui_refresh_tick = 0;
static volatile bool ui_refresh_request = false;


void UI_RequestRefresh(void)
{
    ui_refresh_request = true;
}


void UI_Task_Init(void)
{
    ui_refresh_period = UI_Render();
    ui_refresh_tick = Timer_GetTick();
}


void UI_Task_Process(void)
{
    if(ui_refresh_request)
    {
        ui_refresh_request = false;
        ui_refresh_period = UI_Render();
        ui_refresh_tick = Timer_GetTick();
    }

    if(ui_refresh_period != 0)
    {
        if((Timer_GetTick() - ui_refresh_tick) >= ui_refresh_period)
        {
            ui_refresh_tick = Timer_GetTick();
            ui_refresh_period = UI_Render();
        }
    }
}
