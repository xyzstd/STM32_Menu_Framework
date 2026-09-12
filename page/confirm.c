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
    return MENU_RET_OK;
}

void confirm_display(const confirm_t *c, const char *title)
{
    OLED_ShowString(8, 16, title, OLED_8X16);

    OLED_ShowString(32, 34, "NO", OLED_8X16);
    OLED_ShowString(80, 34, "YES", OLED_8X16);

    /* 选中项反显, 未选中项实线框 */
    if (c->choice == CONFIRM_YES)
    {
        OLED_DrawRoundRect(30, 33, 20, 18, 6, OLED_COLOR_SET);
        OLED_ReverseRoundRect(78, 33, 28, 18, 6);
    }
    else
    {
        OLED_ReverseRoundRect(30, 33, 20, 18, 6);
        OLED_DrawRoundRect(78, 33, 28, 18, 6, OLED_COLOR_SET);
    }
}