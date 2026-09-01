#include "menu.h"
#include "oled.h"
#include "menu_display.h"
#include "temp.h"
#include "config.h"
#include "pid_controller.h"

/* 目标温度范围: 100°C ~ 450°C, 步进 5°C */
static int16_t g_target_temp = 300;

static bool g_dirty = false; /* 参数是否被修改 */

static menu_result_t temp_handler(const menu_cmd_t *cmd, void *user_data)
{
    (void)user_data;
    switch(cmd->event)
    {
        case MENU_EVENT_UP:
            /* 单参数页: 直接可调, 上限随当前型号 */
            g_target_temp += (int16_t)(5 * cmd->step);
            if (g_target_temp > (int16_t)Config_GetTempMax())
                g_target_temp = (int16_t)Config_GetTempMax();
            g_dirty = true;
            break;
        case MENU_EVENT_DOWN:
            g_target_temp -= (int16_t)(5 * cmd->step);
            if (g_target_temp < TEMP_MIN)
                g_target_temp = TEMP_MIN;
            g_dirty = true;
            break;
        case MENU_EVENT_CLICK:
        case MENU_EVENT_LPRESS:
            /* 单参数页: 直接可调, 点击/长按均退出(退出回调保存) */
            return MENU_RET_DONE;
        default:
            break;
    }
    return MENU_RET_OK;
}

static menu_period_t temp_display(void *user_data)
{
    (void)user_data;
    OLED_Clear();

    OLED_ShowString(16, 20, "Set:", OLED_8X16);
    OLED_ShowNum(56, 20, g_target_temp, 3);
    OLED_ShowDegree(88, 20);            /* °C 移到选中框右侧外, 避免与边框粘连 */
    OLED_ShowChar(96, 20, 'C', OLED_8X16);

    /* 单参数: 常驻实线框标识可调区 (x54..81, 数值 x56..79 两侧各 2px; 上下各 1px) */
    OLED_DrawRoundRect(54, 19, 28, 18, 6, OLED_COLOR_SET);

    OLED_Update();
    return 0;
}

static void temp_enter(void *user_data)
{
    (void)user_data;
    g_target_temp = (int16_t)g_config.default_temp;
    g_dirty = false;
}

static void temp_exit(void *user_data)
{
    (void)user_data;
    if(g_dirty)
    {
        /* 保存到配置 */
        g_config.default_temp = (uint16_t)g_target_temp;
        PID_Controller_SetSetpoint(g_target_temp);
        Config_Save();
        g_dirty = false;
    }
}

const menu_ops_t temp_ops = {
    .enter = temp_enter,
    .handler = temp_handler,
    .display = temp_display,
    .exit = temp_exit,
};

int16_t Temp_GetTarget(void)
{
    return g_target_temp;
}