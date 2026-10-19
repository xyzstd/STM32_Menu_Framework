#include "confirm.h"
#include "oled.h"

#define CONFIRM_NO  0
#define CONFIRM_YES 1

void confirm_open(confirm_t *c)
{
    c->active = true;
    c->choice = CONFIRM_NO;
}

menu_result_t confirm_handle(const menu_cmd_t *cmd, confirm_t *c, bool *confirmed)
{
    *confirmed = false;
    switch(cmd->event)
    {
    case MENU_EVENT_UP:
    case MENU_EVENT_DOWN:
        c->choice = (uint8_t)!c->choice;   /* 切换 否/是 */
        break;

    case MENU_EVENT_CLICK:
        if (c->choice == CONFIRM_YES)
        {
            *confirmed = true;       /* YES: 宿主执行写操作并退出 */
            c->active = false;
        }
        else
        {
            c->active = false;
            return MENU_RET_DONE;    /* 选 NO: 取消 */
        }
        break;

    case MENU_EVENT_LPRESS:
        c->active = false;
        return MENU_RET_DONE;        /* 取消 */
    default:
        break;
    }
    return MENU_RET_STAY;
}

void confirm_display(const confirm_t *c, const char *title)
{
    /* 混合宽度: ASCII 8px/字符, 中文 16px/字符 */
    uint8_t w = 0;
    const char *p = title;
    while (*p)
    {
        if ((*p & 0x80) == 0) { w += 8;  p++; }
        else                  { w += 16; p += 3; }
    }

    /* 标题水平居中, 顶部留白 12px */
    OLED_ShowChinese((OLED_WIDTH - (int16_t)w) / 2, 12, title);

    /* 按钮对(20+28=76px)整体居中: 起始 x26, 垂直均衡(下边距 10px) */
    OLED_ShowChinese(28, 37, "否");
    OLED_ShowChinese(80, 37, "是");

    /* 选中项反显, 未选中项实线框 */
    if (c->choice == CONFIRM_YES)
    {
        OLED_DrawRoundRect(26, 36, 20, 18, 6, OLED_COLOR_SET);
        OLED_ReverseRoundRect(74, 36, 28, 18, 6);
    }
    else
    {
        OLED_ReverseRoundRect(26, 36, 20, 18, 6);
        OLED_DrawRoundRect(74, 36, 28, 18, 6, OLED_COLOR_SET);
    }
}
