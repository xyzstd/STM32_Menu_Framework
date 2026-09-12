#include "menu.h"
#include "oled.h"
#include "menu_display.h"
#include "settings.h"
#include "config.h"
#include "buzzer.h"
#include "param.h"
#include "confirm.h"

/* 固件版本字符串 (信息页显示, 发布时更新) */
#define FW_VERSION_STR  "1.0.0"

/* ==================== 休眠时间 ==================== */
static void sleep_time_load(param_t *it){ it->val = (int16_t)g_config.sleep_time; }
static void sleep_time_store(param_t *it){ g_config.sleep_time = (uint16_t)it->val; }

static param_t sleep_time = {
    .min = 0, .max = 60, .step = 1,
    .load = sleep_time_load, .store = sleep_time_store,
};

static menu_period_t sleep_time_display(void *ud)
{
    (void)ud;
    int16_t v = sleep_time.val;
    OLED_Clear();

    OLED_ShowString(8, 18, "Sleep:", OLED_8X16);
    if (v == 0)
        OLED_ShowString(64, 18, "OFF", OLED_8X16);
    else {
        OLED_ShowNum(64, 18, v, 2);
        OLED_ShowString(96, 18, "min", OLED_8X16);   /* 移到框外, 避免与选中框边框重叠 */
    }

    /* 单参数: 常驻实线框标识可调区, 无选中/非选中切换 */
    OLED_DrawRoundRect(62, 17, 28, 18, 6, OLED_COLOR_SET);

    OLED_UpdateDiff();
    return 0;
}

static void sleep_time_enter(void *ud){ (void)ud; param_enter(&sleep_time); }
static menu_result_t sleep_time_handler(const menu_cmd_t *cmd, void *ud){ (void)ud; return param_handler(cmd, &sleep_time); }
static void sleep_time_exit(void *ud){ (void)ud; param_exit(&sleep_time); }

const menu_ops_t sleep_time_ops = {
    .enter = sleep_time_enter, .handler = sleep_time_handler,
    .display = sleep_time_display, .exit = sleep_time_exit,
};

/* ==================== 自动关机 ==================== */
static void shutdown_load(param_t *it){ it->val = (int16_t)g_config.shutdown_time; }
static void shutdown_store(param_t *it){ g_config.shutdown_time = (uint16_t)it->val; }

static param_t shutdown = {
    .min = 0, .max = 120, .step = 5,
    .load = shutdown_load, .store = shutdown_store,
};

static menu_period_t shutdown_display(void *ud)
{
    (void)ud;
    int16_t v = shutdown.val;
    OLED_Clear();

    OLED_ShowString(8, 18, "AutoOff", OLED_8X16);   /* 去冒号以容纳 "min", 避免超出右边界 */
    if (v == 0)
        OLED_ShowString(68, 18, "OFF", OLED_8X16);
    else {
        OLED_ShowNum(68, 18, v, 3);
        OLED_ShowString(100, 18, "min", OLED_8X16);   /* 移到框外, 避免与选中框边框重叠 */
    }

    /* 单参数: 常驻实线框标识可调区 */
    OLED_DrawRoundRect(66, 17, 28, 18, 6, OLED_COLOR_SET);

    OLED_UpdateDiff();
    return 0;
}

static void shutdown_enter(void *ud){ (void)ud; param_enter(&shutdown); }
static menu_result_t shutdown_handler(const menu_cmd_t *cmd, void *ud){ (void)ud; return param_handler(cmd, &shutdown); }
static void shutdown_exit(void *ud){ (void)ud; param_exit(&shutdown); }

const menu_ops_t shutdown_ops = {
    .enter = shutdown_enter, .handler = shutdown_handler,
    .display = shutdown_display, .exit = shutdown_exit,
};

/* ==================== 蜂鸣器 ==================== */
static void buzzer_load(param_t *it){ it->val = g_config.buzzer_enable ? 1 : 0; }
static void buzzer_store(param_t *it){ g_config.buzzer_enable = (uint8_t)it->val; }

static param_t buzzer = {
    .min = 0, .max = 1, .toggle = 1,
    .load = buzzer_load, .store = buzzer_store,
};

static menu_period_t buzzer_display(void *ud)
{
    (void)ud;
    OLED_Clear();

    OLED_ShowString(8, 18, "Buzzer:", OLED_8X16);
    if (buzzer.val)
        OLED_ShowString(80, 18, "ON", OLED_8X16);
    else
        OLED_ShowString(80, 18, "OFF", OLED_8X16);

    /* 单参数: 常驻实线框标识可调区, 无选中/非选中切换 */
    OLED_DrawRoundRect(78, 17, 28, 18, 6, OLED_COLOR_SET);

    OLED_UpdateDiff();
    return 0;
}

static void buzzer_enter(void *ud){ (void)ud; param_enter(&buzzer); }
static menu_result_t buzzer_handler(const menu_cmd_t *cmd, void *ud){ (void)ud; return param_handler(cmd, &buzzer); }
static void buzzer_exit(void *ud){ (void)ud; param_exit(&buzzer); }

const menu_ops_t buzzer_ops = {
    .enter = buzzer_enter, .handler = buzzer_handler,
    .display = buzzer_display, .exit = buzzer_exit,
};

/* ==================== 屏幕亮度 ==================== */
static void brightness_apply(param_t *it){ OLED_SetContrast((uint8_t)it->val); }
static void brightness_load(param_t *it){ it->val = (int16_t)g_config.brightness; }
static void brightness_store(param_t *it){ g_config.brightness = (uint8_t)it->val; }

static param_t brightness = {
    .min = 10, .max = 255, .step = 5,
    .load = brightness_load, .store = brightness_store,
    .on_change = brightness_apply,
};

static menu_period_t brightness_display(void *ud)
{
    (void)ud;
    int16_t v = brightness.val;
    OLED_Clear();

    OLED_ShowString(8, 18, "Bright:", OLED_8X16);
    OLED_ShowNum(80, 18, v, 3);

    /* 亮度指示条 */
    if (v > 0) {
        uint8_t bar_w = (uint8_t)((uint32_t)(OLED_WIDTH - 16) * v / 255);
        OLED_FillRect(8, 40, bar_w, 8, OLED_COLOR_SET);
    }

    /* 单参数: 常驻实线框标识可调区, 无选中/非选中切换 */
    OLED_DrawRoundRect(78, 17, 28, 18, 6, OLED_COLOR_SET);

    OLED_UpdateDiff();
    return 0;
}

static void brightness_enter(void *ud){ (void)ud; param_enter(&brightness); }
static menu_result_t brightness_handler(const menu_cmd_t *cmd, void *ud){ (void)ud; return param_handler(cmd, &brightness); }
static void brightness_exit(void *ud){ (void)ud; param_exit(&brightness); }

const menu_ops_t brightness_ops = {
    .enter = brightness_enter, .handler = brightness_handler,
    .display = brightness_display, .exit = brightness_exit,
};

/* ==================== 恢复默认 ==================== */
static confirm_t reset_cf;

static void reset_enter(void *ud)
{
    (void)ud;
    confirm_open(&reset_cf);   /* 默认停在 "NO" */
}

static menu_result_t reset_handler(const menu_cmd_t *cmd, void *ud)
{
    (void)ud;
    bool confirmed = false;
    menu_result_t r = confirm_handle(cmd, &reset_cf, &confirmed);
    if (confirmed)
    {
        Config_Reset();   /* 内部已写 Flash (恢复默认并保存) */
        /* 应用亮度默认值 */
        OLED_SetContrast(g_config.brightness);
        return MENU_RET_DONE;
    }
    if (r == MENU_RET_DONE)
        return MENU_RET_DONE;   /* 选 NO / 长按: 取消并退出 */
    return MENU_RET_OK;
}

static menu_period_t reset_display(void *ud)
{
    (void)ud;
    OLED_Clear();

    confirm_display(&reset_cf, "Reset to def?");

    OLED_UpdateDiff();
    return 0;
}

const menu_ops_t reset_ops = {
    .enter = reset_enter,
    .handler = reset_handler,
    .display = reset_display,
};

/* ==================== 版本信息 (只读页: 由 MENU_FLAG_PASSIVE_VIEW 内核裁决, 无需 handler) ==================== */
static menu_period_t version_display(void *ud)
{
    (void)ud;
    OLED_Clear();

    /* 各行错开, 避免 8x16 标题与 6x8 信息行发生像素重叠 */
    OLED_ShowString(8, 6, "Soldering Iron", OLED_8X16);   /* 行 6-21 */
    OLED_ShowString(8, 26, "FW: " FW_VERSION_STR, OLED_6X8);   /* 行 26-33 */
    OLED_ShowString(8, 36, "MCU: STM32F103RC", OLED_6X8);  /* 行 36-43 */
    OLED_ShowString(8, 46, "OLED: 128x64", OLED_6X8);      /* 行 46-53 */

    OLED_UpdateDiff();
    return 0;
}

const menu_ops_t version_ops = {
    .display = version_display,
};