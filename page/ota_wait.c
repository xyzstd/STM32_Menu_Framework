#include "menu.h"
#include "oled.h"
#include "ota_boot.h"
#include "buzzer.h"

/* 每 250ms 显示/检查一次 */
#define REFRESH_PERIOD_MS  250

/* 等待动画: 4 帧循环 */
#define ANIM_FRAMES  4
static uint8_t anim_frame = 0;

/* 标志页写入失败(低压/写保护): 停在错误提示, 不再重画等待界面.
 * OTA_EnterBootloader 成功即复位不返回, 故"函数返回"本身就等于失败.
 * 仅在 OTA 模块启用时才有意义(关闭时只写不读 → 会触发 #550-D 警告). */
#if OTA_APP_SIDE_ENABLED
static bool boot_failed = false;
#endif

static menu_result_t ota_wait_handler(const menu_cmd_t *cmd, void *user_data);
static menu_period_t ota_wait_display(void *user_data);
static void ota_wait_enter(void *user_data);
static void ota_wait_exit(void *user_data);

/* 事件处理: 本页要"停留等待"上位机握手, 故 CLICK/UP/DOWN 一律不响应(返回 OK 留守),
 * 只有长按才退出. 与只读信息页(任意键即退)语义相反, 退出裁决权在本 handler. */
static menu_result_t ota_wait_handler(const menu_cmd_t *cmd, void *user_data)
{
    (void)user_data;
    if (cmd->event == MENU_EVENT_LPRESS)
        return MENU_RET_DONE;   /* 长按: 放弃等待, 返回列表 */
    return MENU_RET_OK;         /* CLICK/UP/DOWN: 留在本页继续等握手 */
}

static menu_period_t ota_wait_display(void *user_data)
{
    (void)user_data;
#if OTA_APP_SIDE_ENABLED
    static const char *const dots[ANIM_FRAMES] = {"   ", ".  ", ".. ", "..."};
    char buf[16];
    const char *d = dots[anim_frame];
#else
    /* OTA 模块已关闭: 不显示"等待主机"(永远等不到), 直接提示功能不可用,
     * 并给出退出方式 —— 与"写入失败"分支用同一套交互, 用户不会困在本页. */
    (void)anim_frame;
#endif

#if !OTA_APP_SIDE_ENABLED
    OLED_Clear();
    /* OTA 模块已关闭: 整页只居中显示一行"未启用"。
     * 原实现是"升级已关"(y=20) + "长按退出"(y=56) 两行 —— 后者把 16x16 汉字放在
     * y=56, 底部 8 行超出 64px 屏被静默裁掉(只剩上半截), 本就是坏的; 去掉后既解决
     * 裁切, 又让页面只剩一句结论。
     * 退出手势不必提示: 长按返回是全局一致的操作(本页 handler 亦只认 LPRESS)。
     * 字库已按需补"未/用"(见 bsp/oled_data.c), 措辞不必再迁就已有字形。 */
    OLED_ShowChinese((int16_t)(((int16_t)OLED_WIDTH - 3 * 16) / 2),   /* 3 汉字 48px 居中 */
                     (int16_t)(((int16_t)OLED_HEIGHT - 16) / 2),      /* 16px 行高垂直居中 */
                     "未启用");
    OLED_UpdateDiff();
    return REFRESH_PERIOD_MS;
#else
    /* 标志页写入失败: 只提示错误, 不再显示"等待主机"以免误导 */
    if (boot_failed)
    {
        OLED_Clear();
        OLED_ShowChinese(32, 20, "升级失败");   /* 4 汉字 64px, 居中 x=32 */
        OLED_UpdateDiff();
        return REFRESH_PERIOD_MS;
    }

    /* 检查 OTA 请求标志（串口 RX 中断里 OTA_SerialFeedRx 状态机设置）。*/
    if (OTA_GetAppRequest()) {
        OTA_SetAppRequest(false);
        OLED_Clear();
        /* 黑底白字提示条: 先白条→黑字→整体反显; 文本 y20..35 完全落在条内 y19..36 */
        OLED_FillRect(0, 19, OLED_WIDTH, 18, OLED_COLOR_SET);
        OLED_ShowChinese(40, 20, "已连接");
        OLED_ReverseArea(0, 19, OLED_WIDTH, 18);
        OLED_ShowChinese(32, 42, "正在重启");
        OLED_Update();
        OTA_EnterBootloader();  /* 成功: 写标志页 + NVIC_SystemReset(), 不返回 */
        /* 走到这里 = 标志页写入失败(低压/写保护), 未复位. 若不提示, 用户会看到
         * "正在重启"却停在原界面, 误以为已进入升级而实际上什么都没发生. */
        boot_failed = true;
        Buzzer_Beep(BUZZER_EVENT_ALARM);
        return REFRESH_PERIOD_MS;   /* 本帧先不重画, 下一帧进上面的错误分支 */
    }

    anim_frame = (uint8_t)((anim_frame + 1) % ANIM_FRAMES);

    OLED_Clear();

    /* status text */
    OLED_ShowChinese(32, 20, "等待主机");

    /* 状态行与动画点, 不再加"长按退出"提示: 长按返回是全局一致的操作,
     * 每个页面都重复一遍既占版面也不好看。 */
    oled_pair_cat(buf, sizeof(buf), "连接", d);
    OLED_ShowChinese(36, 38, buf);

    OLED_UpdateDiff();
    return REFRESH_PERIOD_MS;
#endif /* OTA_APP_SIDE_ENABLED */
}

static void ota_wait_enter(void *user_data)
{
    (void)user_data;
#if OTA_APP_SIDE_ENABLED
    OTA_SetAppRequest(false);
#endif
    anim_frame  = 0;
#if OTA_APP_SIDE_ENABLED
    boot_failed = false;   /* 重进本页清错误态, 允许再试一次 */
#endif
}

static void ota_wait_exit(void *user_data)
{
    (void)user_data;
#if OTA_APP_SIDE_ENABLED
    OTA_SetAppRequest(false);
#endif
}

const menu_ops_t ota_wait_ops = {
    .enter   = ota_wait_enter,
    .handler = ota_wait_handler,
    .display = ota_wait_display,
    .exit    = ota_wait_exit,
};
