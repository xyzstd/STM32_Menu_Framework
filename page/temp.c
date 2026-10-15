#include "menu.h"
#include "oled.h"
#include "temp.h"
#include "config.h"
#include "pid_controller.h"
#include "param.h"
#include "work_task.h"
#include "buzzer.h"

/*
 * 目标温度设置项 (单参数模板):
 *   范围 TEMP_MIN~当前型号上限, 步进 5°C.
 *
 * 本项是一个普通页(ops->display 非空): CLICK 由内核激活为 HANDLER 并调 enter,
 * 页内独立整页显示当前值; CLICK/LPRESS 由 param_handler 返回 DONE → 内核走
 * menu_exit_handler → exit 落盘并回列表。
 *
 * 因上限随当前型号(Config_GetTempMax)动态变化, param_cfg_t 的 max
 * 无法静态初始化, 故用非 const 的静态实例(落 .bss, 零初始化, 无 Flash 代价),
 * 在 enter 时解析当前型号上限.
 */

static param_cfg_t temp_cfg;
static param_t temp = { .cfg = &temp_cfg };

static void temp_load(param_t *it);
static void temp_store(param_t *it);
static void temp_setup(void);
static void temp_enter(void *ud);
static menu_result_t temp_handler(const menu_cmd_t *cmd, void *ud);
static void temp_exit(void *ud);
static menu_period_t temp_display(void *ud);

static void temp_load(param_t *it)
{
    it->val = (int16_t)g_config.default_temp;
}

static void temp_store(param_t *it)
{
    g_config.default_temp = (uint16_t)it->val;
    /* 同步运行时目标温度: 本项与 HOME 是同一温度的两条写入口, 只写 default_temp
     * 会让休眠/关机时 remember_last_temp 用旧 runtime_temp 覆写回落盘值, 静默
     * 丢弃本页设置 */
    WorkTask_SetRuntimeTemp(it->val);
    PID_Controller_SetSetpoint(it->val);
}

static void temp_setup(void)
{
    temp_cfg.min      = (int16_t)TEMP_MIN;
    temp_cfg.max      = (int16_t)Config_GetTempMax();
    temp_cfg.step     = 5;
    temp_cfg.toggle   = 0;
    temp_cfg.load     = temp_load;
    temp_cfg.store    = temp_store;
}

static void temp_enter(void *ud)
{
    (void)ud;
    temp_setup();
    param_enter(&temp);
}

static menu_result_t temp_handler(const menu_cmd_t *cmd, void *ud){ (void)ud; return param_handler(cmd, &temp); }

/* 目标温度是 HOME 高频操作, 退出落盘**不弹"已保存"框**以免遮挡读数;
 * 但落盘失败必须蜂鸣 —— 静默失败会让用户以为新设定已存下, 重启才发现回到旧值.
 * (仅抑制成功提示, 不抑制失败告警) */
static void temp_exit(void *ud)
{
    (void)ud;
    if (param_exit(&temp) == PARAM_EXIT_SAVE_FAILED)
        Buzzer_Beep(BUZZER_EVENT_ALARM);
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

const menu_ops_t temp_ops = {
    .enter   = temp_enter,
    .handler = temp_handler,
    .display = temp_display,
    .exit    = temp_exit,
};
