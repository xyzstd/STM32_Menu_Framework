#include "menu.h"
#include "oled.h"
#include "menu_display.h"
#include "settings.h"
#include "config.h"
#include "buzzer.h"
#include "param.h"
#include "confirm.h"
#include "work_task.h"
#include "heater.h"
#include "toast.h"

/* 固件版本字符串 (信息页显示, 发布时更新) */
#define FW_VERSION_STR  "1.0.0"

static void sleep_time_load(param_t *it);
static void sleep_time_store(param_t *it);
static void sleep_time_enter(void *ud);
static menu_result_t sleep_time_handler(const menu_cmd_t *cmd, void *ud);
static void sleep_time_exit(void *ud);
static menu_period_t sleep_time_display(void *ud);
static void sleep_temp_load(param_t *it);
static void sleep_temp_store(param_t *it);
static void sleep_temp_enter(void *ud);
static menu_result_t sleep_temp_handler(const menu_cmd_t *cmd, void *ud);
static void sleep_temp_exit(void *ud);
static menu_period_t sleep_temp_display(void *ud);
static void shutdown_load(param_t *it);
static void shutdown_store(param_t *it);
static void shutdown_enter(void *ud);
static menu_result_t shutdown_handler(const menu_cmd_t *cmd, void *ud);
static void shutdown_exit(void *ud);
static menu_period_t shutdown_display(void *ud);
static void buzzer_load(param_t *it);
static void buzzer_store(param_t *it);
static void buzzer_enter(void *ud);
static menu_result_t buzzer_handler(const menu_cmd_t *cmd, void *ud);
static void buzzer_exit(void *ud);
static menu_period_t buzzer_display(void *ud);
static void brightness_apply(param_t *it);
static void brightness_load(param_t *it);
static void brightness_store(param_t *it);
static void brightness_enter(void *ud);
static menu_result_t brightness_handler(const menu_cmd_t *cmd, void *ud);
static void brightness_exit(void *ud);
static menu_period_t brightness_display(void *ud);
static void reset_enter(void *ud);
static menu_result_t reset_handler(const menu_cmd_t *cmd, void *ud);
static menu_period_t reset_display(void *ud);
static menu_period_t version_display(void *ud);

/* ==================== 休眠时间 ==================== */
static void sleep_time_load(param_t *it){ it->val = (int16_t)g_config.sleep_time; }
static void sleep_time_store(param_t *it){ g_config.sleep_time = (uint16_t)it->val; }

static param_t sleep_time = {
    .min = 0, .max = 60, .step = 1,
    .load = sleep_time_load, .store = sleep_time_store,
};

static void sleep_time_enter(void *ud){ (void)ud; param_enter(&sleep_time); }
static menu_result_t sleep_time_handler(const menu_cmd_t *cmd, void *ud){ (void)ud; return param_handler(cmd, &sleep_time); }
static void sleep_time_exit(void *ud){ (void)ud; if (param_exit(&sleep_time)) Toast_Saved(); }
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

const menu_ops_t sleep_time_ops = {
    .enter = sleep_time_enter, .handler = sleep_time_handler,
    .display = sleep_time_display, .exit = sleep_time_exit,
};

/* ==================== 休眠温度 (保温) ==================== */
static void sleep_temp_load(param_t *it){ it->val = (int16_t)g_config.sleep_temp; }
static void sleep_temp_store(param_t *it){ g_config.sleep_temp = (uint16_t)it->val; }

static param_t sleep_temp = {
    .min = 50, .max = 300, .step = 10,
    .load = sleep_temp_load, .store = sleep_temp_store,
};

static void sleep_temp_enter(void *ud){ (void)ud; param_enter(&sleep_temp); }
static menu_result_t sleep_temp_handler(const menu_cmd_t *cmd, void *ud){ (void)ud; return param_handler(cmd, &sleep_temp); }
static void sleep_temp_exit(void *ud){ (void)ud; if (param_exit(&sleep_temp)) Toast_Saved(); }
static menu_period_t sleep_temp_display(void *ud)
{
    (void)ud;
    int16_t v = sleep_temp.val;
    OLED_Clear();

    OLED_ShowString(8, 20, "Idle:", OLED_8X16);
    OLED_ShowNum(56, 20, v, 3);
    OLED_ShowDegree(88, 20);            /* °C 移到选中框右侧外, 避免与边框粘连 */
    OLED_ShowChar(96, 20, 'C', OLED_8X16);

    /* 单参数: 常驻实线框标识可调区 (x54..81, 数值 x56..79 两侧各 2px; 上下各 1px) */
    OLED_DrawRoundRect(54, 19, 28, 18, 6, OLED_COLOR_SET);

    OLED_UpdateDiff();
    return 0;
}

const menu_ops_t sleep_temp_ops = {
    .enter = sleep_temp_enter, .handler = sleep_temp_handler,
    .display = sleep_temp_display, .exit = sleep_temp_exit,
};

/* ==================== 自动关机 ==================== */
static void shutdown_load(param_t *it){ it->val = (int16_t)g_config.shutdown_time; }
static void shutdown_store(param_t *it){ g_config.shutdown_time = (uint16_t)it->val; }

static param_t shutdown = {
    .min = 0, .max = 120, .step = 5,
    .load = shutdown_load, .store = shutdown_store,
};

static void shutdown_enter(void *ud){ (void)ud; param_enter(&shutdown); }
static menu_result_t shutdown_handler(const menu_cmd_t *cmd, void *ud){ (void)ud; return param_handler(cmd, &shutdown); }
static void shutdown_exit(void *ud){ (void)ud; if (param_exit(&shutdown)) Toast_Saved(); }
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

const menu_ops_t shutdown_ops = {
    .enter = shutdown_enter, .handler = shutdown_handler,
    .display = shutdown_display, .exit = shutdown_exit,
};

/* ==================== 蜂鸣器 ==================== */
static void buzzer_load(param_t *it){ it->val = g_config.buzzer_enable ? 1 : 0; }
static void buzzer_store(param_t *it)
{
    g_config.buzzer_enable = (uint8_t)it->val;
    Buzzer_SetEnabled(g_config.buzzer_enable != 0);   /* 立即生效, 不必等重启 */
}

static param_t buzzer = {
    .min = 0, .max = 1, .toggle = 1,
    .load = buzzer_load, .store = buzzer_store,
};

static void buzzer_enter(void *ud){ (void)ud; param_enter(&buzzer); }
static menu_result_t buzzer_handler(const menu_cmd_t *cmd, void *ud){ (void)ud; return param_handler(cmd, &buzzer); }
static void buzzer_exit(void *ud){ (void)ud; if (param_exit(&buzzer)) Toast_Saved(); }
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

static void brightness_enter(void *ud){ (void)ud; param_enter(&brightness); }
static menu_result_t brightness_handler(const menu_cmd_t *cmd, void *ud){ (void)ud; return param_handler(cmd, &brightness); }
static void brightness_exit(void *ud){ (void)ud; if (param_exit(&brightness)) Toast_Saved(); }
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
        /* 配置已回到默认型号(T12)/上限(400)/PID 参数: 走型号切换的安全收敛链路,
         * 重载 PID、按新上限钳位目标、清 Boost 残留、对齐引导标志并重评型号故障.
         * 该函数开头断热(防切换瞬间残余功率), 此处按"离开换头页"同一方式恢复使能:
         * ACTIVE 态没有其他 Heater_Enable 路径, 不补会静默永久停加热. */
        WorkTask_OnHandleChanged();
        if (!WorkTask_HasFault())
            Heater_Enable();
        /* 应用亮度与蜂鸣器默认值 */
        OLED_SetContrast(g_config.brightness);
        Buzzer_SetEnabled(g_config.buzzer_enable != 0);
        Toast_Saved();   /* 落盘成功: 弹出"已保存"框 */
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

    confirm_display(&reset_cf, "恢复默认?");

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
    char ver[24];
    (void)ud;
    OLED_Clear();

    /* 4 行 16px 高铺满屏无重叠: y0/16/32/48
     * 标题 4 汉字 64px 居中; 信息行 ASCII 与汉字混合 (OLED_ShowChinese) */
    OLED_ShowChinese(32, 0, "智能烙铁");
    /* 版本号取自 FW_VERSION_STR 单点定义: 发布时只改宏, 不必再翻渲染代码.
     * 拼接用 oled_pair_cat 而非 snprintf: 避免为 2 处文案把 stdio 格式化
     * 机制整个拉进链接(实测约 2.3KB, 含随 printfa 一并引入的浮点支持). */
    oled_pair_cat(ver, sizeof(ver), "固件版本:", FW_VERSION_STR);
    OLED_ShowChinese(8, 16, ver);
    OLED_ShowChinese(8, 32, "芯片:F103RC");
    OLED_ShowChinese(8, 48, "屏幕:128x64");

    OLED_UpdateDiff();
    return 0;
}

const menu_ops_t version_ops = {
    .display = version_display,
};