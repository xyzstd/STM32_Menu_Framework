#include "menu.h"
#include "oled.h"
#include "menu_display.h"
#include "temp.h"
#include "config.h"
#include "pid_controller.h"
#include "param.h"

/*
 * 目标温度设置页 (单参数模板):
 *   范围 TEMP_MIN~当前型号上限, 步进 5°C.
 *
 * 因上限随当前型号(Config_GetTempMax)动态变化, param_t 的 max
 * 无法静态初始化, 故使用静态实例, 在 enter 时解析当前型号上限.
 * 页面内不会切换型号 (切换在 handle 页), 且返回本页会重新 enter,
 * 故进入时快照上限与实际行为一致.
 */

static param_t temp;

static void temp_load(param_t *it)
{
    it->val = (int16_t)g_config.default_temp;
}

static void temp_store(param_t *it)
{
    g_config.default_temp = (uint16_t)it->val;
    PID_Controller_SetSetpoint(it->val);
}

static void temp_setup(void)
{
    temp.min      = (int16_t)TEMP_MIN;
    temp.max      = (int16_t)Config_GetTempMax();
    temp.step     = 5;
    temp.toggle   = 0;
    temp.load     = temp_load;
    temp.store    = temp_store;
}

static void temp_enter(void *ud)
{
    (void)ud;
    temp_setup();
    param_enter(&temp);
}

static menu_period_t temp_display(void *ud)
{
    (void)ud;
    OLED_Clear();

    OLED_ShowString(16, 20, "Set:", OLED_8X16);
    OLED_ShowNum(56, 20, temp.val, 3);
    OLED_ShowDegree(88, 20);            /* °C 移到选中框右侧外, 避免与边框粘连 */
    OLED_ShowChar(96, 20, 'C', OLED_8X16);

    /* 单参数: 常驻实线框标识可调区 (x54..81, 数值 x56..79 两侧各 2px; 上下各 1px) */
    OLED_DrawRoundRect(54, 19, 28, 18, 6, OLED_COLOR_SET);

    OLED_UpdateDiff();
    return 0;
}

static menu_result_t temp_handler(const menu_cmd_t *cmd, void *ud){ (void)ud; return param_handler(cmd, &temp); }
static void temp_exit(void *ud){ (void)ud; param_exit(&temp); }

const menu_ops_t temp_ops = {
    .enter   = temp_enter,
    .handler = temp_handler,
    .display = temp_display,
    .exit    = temp_exit,
};
