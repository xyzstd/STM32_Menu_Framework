#include "menu.h"
#include "oled.h"
#include "menu_display.h"
#include "settings.h"
#include "config.h"
#include "buzzer.h"

/* ==================== 休眠时间 ==================== */
static bool g_slp_dirty = false;

static menu_result_t sleep_time_handler(const menu_cmd_t *cmd, void *ud)
{
    (void)ud;
    int16_t *v = (int16_t *)&g_config.sleep_time;
    switch(cmd->event)
    {
        case MENU_EVENT_UP:
            *v += (int16_t)(1 * cmd->step); if (*v > 60) *v = 60; g_slp_dirty = true;
            break;
        case MENU_EVENT_DOWN:
            *v -= (int16_t)(1 * cmd->step); if (*v < 0) *v = 0; g_slp_dirty = true;
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

static void sleep_time_exit(void *ud)
{
    (void)ud;
    if(g_slp_dirty){ Config_Save(); g_slp_dirty = false; }
}

static menu_period_t sleep_time_display(void *ud)
{
    (void)ud;
    OLED_Clear();

    OLED_ShowString(8, 18, "Sleep:", OLED_8X16);
    if (g_config.sleep_time == 0)
        OLED_ShowString(64, 18, "OFF", OLED_8X16);
    else {
        OLED_ShowNum(64, 18, g_config.sleep_time, 2);
        OLED_ShowString(96, 18, "min", OLED_8X16);   /* 移到框外, 避免与选中框边框重叠 */
    }

    /* 单参数: 常驻实线框标识可调区, 无选中/非选中切换 */
    OLED_DrawRoundRect(62, 17, 28, 18, 6, OLED_COLOR_SET);

    OLED_Update();
    return 0;
}

const menu_ops_t sleep_time_ops = {
    .enter = NULL,
    .handler = sleep_time_handler,
    .display = sleep_time_display,
    .exit = sleep_time_exit,
};

/* ==================== 自动关机 ==================== */
static bool g_shut_dirty = false;

static menu_result_t shutdown_handler(const menu_cmd_t *cmd, void *ud)
{
    (void)ud;
    int16_t *v = (int16_t *)&g_config.shutdown_time;
    switch(cmd->event)
    {
        case MENU_EVENT_UP:
            *v += (int16_t)(5 * cmd->step); if (*v > 120) *v = 120; g_shut_dirty = true;
            break;
        case MENU_EVENT_DOWN:
            *v -= (int16_t)(5 * cmd->step); if (*v < 0) *v = 0; g_shut_dirty = true;
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

static void shutdown_exit(void *ud)
{
    (void)ud;
    if(g_shut_dirty){ Config_Save(); g_shut_dirty = false; }
}

static menu_period_t shutdown_display(void *ud)
{
    (void)ud;
    OLED_Clear();

    OLED_ShowString(8, 18, "AutoOff", OLED_8X16);   /* 去冒号以容纳 "min", 避免超出右边界 */
    if (g_config.shutdown_time == 0)
        OLED_ShowString(68, 18, "OFF", OLED_8X16);
    else {
        OLED_ShowNum(68, 18, g_config.shutdown_time, 3);
        OLED_ShowString(100, 18, "min", OLED_8X16);   /* 移到框外, 避免与选中框边框重叠 */
    }

    /* 单参数: 常驻实线框标识可调区 */
    OLED_DrawRoundRect(66, 17, 28, 18, 6, OLED_COLOR_SET);

    OLED_Update();
    return 0;
}

const menu_ops_t shutdown_ops = {
    .handler = shutdown_handler,
    .display = shutdown_display,
    .exit = shutdown_exit,
};

/* ==================== 蜂鸣器 ==================== */
static bool g_buz_dirty = false;

static menu_result_t buzzer_handler(const menu_cmd_t *cmd, void *ud)
{
    (void)ud;
    switch(cmd->event)
    {
        case MENU_EVENT_UP:
        case MENU_EVENT_DOWN:
            /* 单参数页: 直接切换开/关 */
            g_config.buzzer_enable = g_config.buzzer_enable ? 0 : 1;
            g_buz_dirty = true;
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

static void buzzer_exit(void *ud)
{
    (void)ud;
    if(g_buz_dirty){ Config_Save(); g_buz_dirty = false; }
}

static menu_period_t buzzer_display(void *ud)
{
    (void)ud;
    OLED_Clear();

    OLED_ShowString(8, 18, "Buzzer:", OLED_8X16);
    if (g_config.buzzer_enable)
        OLED_ShowString(80, 18, "ON", OLED_8X16);
    else
        OLED_ShowString(80, 18, "OFF", OLED_8X16);

    /* 单参数: 常驻实线框标识可调区, 无选中/非选中切换 */
    OLED_DrawRoundRect(78, 17, 28, 18, 6, OLED_COLOR_SET);

    OLED_Update();
    return 0;
}

const menu_ops_t buzzer_ops = {
    .handler = buzzer_handler,
    .display = buzzer_display,
    .exit = buzzer_exit,
};

/* ==================== 屏幕亮度 ==================== */
static bool g_bri_dirty = false;

static menu_result_t brightness_handler(const menu_cmd_t *cmd, void *ud)
{
    (void)ud;
    int16_t v = (int16_t)g_config.brightness;
    switch(cmd->event)
    {
        case MENU_EVENT_UP:
            /* 单参数页: 直接可调 (int16_t 中间量防溢出) */
            v += (int16_t)(5 * cmd->step);
            if (v > 255) v = 255;
            g_config.brightness = (uint8_t)v;
            g_bri_dirty = true;
            break;
        case MENU_EVENT_DOWN:
            v -= (int16_t)(5 * cmd->step);
            if (v < 10) v = 10;
            g_config.brightness = (uint8_t)v;
            g_bri_dirty = true;
            break;
        case MENU_EVENT_CLICK:
        case MENU_EVENT_LPRESS:
            /* 单参数页: 直接可调, 点击/长按均退出(退出回调保存) */
            return MENU_RET_DONE;
        default:
            break;
    }
    /* 实时应用亮度 */
    OLED_SetContrast(g_config.brightness);
    return MENU_RET_OK;
}

static void brightness_exit(void *ud)
{
    (void)ud;
    if(g_bri_dirty){ Config_Save(); g_bri_dirty = false; }
}

static menu_period_t brightness_display(void *ud)
{
    (void)ud;
    OLED_Clear();

    OLED_ShowString(8, 18, "Bright:", OLED_8X16);
    OLED_ShowNum(80, 18, g_config.brightness, 3);

    /* 亮度指示条 */
    if (g_config.brightness > 0) {
        uint8_t bar_w = (uint8_t)((uint32_t)(OLED_WIDTH - 16) * g_config.brightness / 255);
        OLED_FillRect(8, 40, bar_w, 8, OLED_COLOR_SET);
    }

    /* 单参数: 常驻实线框标识可调区, 无选中/非选中切换 */
    OLED_DrawRoundRect(78, 17, 28, 18, 6, OLED_COLOR_SET);

    OLED_Update();
    return 0;
}

const menu_ops_t brightness_ops = {
    .handler = brightness_handler,
    .display = brightness_display,
    .exit = brightness_exit,
};

/* ==================== 恢复默认 ==================== */
static uint8_t g_reset_confirm = 0; /* 0=否, 1=是 */

static void reset_enter(void *ud)
{
    (void)ud;
    g_reset_confirm = 0;   /* 默认停在 "NO" */
}

static menu_result_t reset_handler(const menu_cmd_t *cmd, void *ud)
{
    (void)ud;
    switch(cmd->event)
    {
        case MENU_EVENT_UP:
        case MENU_EVENT_DOWN:
            g_reset_confirm = !g_reset_confirm;
            break;
        case MENU_EVENT_CLICK:
            if (g_reset_confirm == 0)
                return MENU_RET_DONE;   /* 选择 NO: 取消并退出 */
            Config_Reset();   /* 内部已写 Flash (恢复默认并保存) */
            /* 应用亮度默认值 */
            OLED_SetContrast(g_config.brightness);
            return MENU_RET_DONE;
        case MENU_EVENT_LPRESS:
            return MENU_RET_DONE;   /* 不确认直接退出 */
        default:
            break;
    }
    return MENU_RET_OK;
}

static menu_period_t reset_display(void *ud)
{
    (void)ud;
    OLED_Clear();

    OLED_ShowString(8, 16, "Reset to def?", OLED_8X16);

    /* 同时显示两个选项 */
    OLED_ShowString(32, 34, "NO", OLED_8X16);
    OLED_ShowString(80, 34, "YES", OLED_8X16);

    /* 选中项反显, 未选中项实线框; 文字两侧对称 2px, 上下各 1px */
    if (g_reset_confirm == 0)
    {
        OLED_ReverseRoundRect(30, 33, 20, 18, 6);
        OLED_DrawRoundRect(78, 33, 28, 18, 6, OLED_COLOR_SET);
    }
    else
    {
        OLED_DrawRoundRect(30, 33, 20, 18, 6, OLED_COLOR_SET);
        OLED_ReverseRoundRect(78, 33, 28, 18, 6);
    }

    OLED_Update();
    return 0;
}

const menu_ops_t reset_ops = {
    .enter = reset_enter,
    .handler = reset_handler,
    .display = reset_display,
};

/* ==================== 版本信息 ==================== */
static menu_result_t version_handler(const menu_cmd_t *cmd, void *ud)
{
    (void)ud;
    /* 功能页: 显示内容时点击/长按均可返回 */
    if(cmd->event == MENU_EVENT_CLICK || cmd->event == MENU_EVENT_LPRESS)
        return MENU_RET_DONE;
    return MENU_RET_OK;
}

static menu_period_t version_display(void *ud)
{
    (void)ud;
    OLED_Clear();

    /* 各行错开, 避免 8x16 标题与 6x8 信息行发生像素重叠 */
    OLED_ShowString(8, 6, "Soldering Iron", OLED_8X16);   /* 行 6-21 */
    OLED_ShowString(8, 26, "FW: V1.0.0", OLED_6X8);        /* 行 26-33 */
    OLED_ShowString(8, 36, "MCU: STM32F103RC", OLED_6X8);  /* 行 36-43 */
    OLED_ShowString(8, 46, "OLED: 128x64", OLED_6X8);      /* 行 46-53 */

    OLED_Update();
    return 0;
}

const menu_ops_t version_ops = {
    .handler = version_handler,
    .display = version_display,
};