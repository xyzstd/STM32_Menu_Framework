#include "menu.h"
#include "oled.h"
#include "menu_display.h"
#include "ota_boot.h"

/* 每 250ms 显示/检查一次 */
#define REFRESH_PERIOD_MS  250

/* 等待动画: 4 帧循环 */
#define ANIM_FRAMES  4
static uint8_t anim_frame = 0;

static menu_result_t ota_wait_handler(const menu_cmd_t *cmd, void *user_data)
{
    (void)cmd; (void)user_data;

    /* 功能页: 显示内容时点击/长按均可返回(点击/长按均退出等待) */
    if (cmd->event == MENU_EVENT_CLICK || cmd->event == MENU_EVENT_LPRESS)
        return MENU_RET_DONE;

    return MENU_RET_OK;
}

static menu_period_t ota_wait_display(void *user_data)
{
    (void)user_data;
    static const char *dots[ANIM_FRAMES] = {"   ", ".  ", ".. ", "..."};
    char buf[8];
    const char *d = dots[anim_frame];

    /* 检查 OTA 请求标志（串口 RX 中断里 OTA_SerialFeedRx 状态机设置）。*/
    if (OTA_GetAppRequest()) {
        OTA_SetAppRequest(false);
        OLED_Clear();
        /* 黑底白字提示条: 先白条→黑字→整体反显; 文本 y20..35 完全落在条内 y19..36 */
        OLED_FillRect(0, 19, OLED_WIDTH, 18, OLED_COLOR_SET);
        OLED_ShowString((OLED_WIDTH - 8 * 10) / 2, 20, "Connected!", OLED_8X16);
        OLED_ReverseArea(0, 19, OLED_WIDTH, 18);
        /* OLED_6X8 实际按 8px/字符步进, 故按 8*11 计算居中 */
        OLED_ShowString((OLED_WIDTH - 8 * 11) / 2, 42, "Resetting...", OLED_6X8);
        OLED_Update();
        OTA_EnterBootloader();  /* 写 noinit 魔数 + NVIC_SystemReset()，不返回 */
    }

    anim_frame = (uint8_t)((anim_frame + 1) % ANIM_FRAMES);

    OLED_Clear();

    /* status text */
    OLED_ShowString((OLED_WIDTH - 8 * 11) / 2, 20, "Waiting for", OLED_8X16);

    /* animated dots: OLED_6X8 实际按 8px/字符步进, 故按 8*8 计算居中 */
    buf[0] = 'h'; buf[1] = 'o'; buf[2] = 's'; buf[3] = 't';
    buf[4] = d[0]; buf[5] = d[1]; buf[6] = d[2]; buf[7] = '\0';
    OLED_ShowString((OLED_WIDTH - 8 * 8) / 2, 38, buf, OLED_6X8);
	OLED_Update();
    return REFRESH_PERIOD_MS;
}

static void ota_wait_enter(void *user_data)
{
    (void)user_data;
    OTA_SetAppRequest(false);
    anim_frame = 0;
}

static void ota_wait_exit(void *user_data)
{
    (void)user_data;
    OTA_SetAppRequest(false);
}

const menu_ops_t ota_wait_ops = {
    .enter   = ota_wait_enter,
    .handler = ota_wait_handler,
    .display = ota_wait_display,
    .exit    = ota_wait_exit,
};
