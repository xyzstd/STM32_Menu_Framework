#include "menu.h"
#include "oled.h"
#include "menu_display.h"
#include "menu_anim.h"

#define MENU_LINES_PER_PAGE (OLED_HEIGHT / OLED_8X16)
#define MENU_ITEM_HEIGHT OLED_8X16

#if MENU_ANIM_ENABLE
static int16_t menu_item_y(uint8_t id, uint8_t first, uint8_t page_start)
{
    return (int16_t)((id - page_start) * MENU_ITEM_HEIGHT);
}
#endif

/* 计算 UTF-8 字符串显示宽度: ASCII 8px, 中文 16px (与 OLED_ShowChinese 一致) */
static uint8_t menu_text_width(const char *str)
{
    uint8_t width = 0;

    while (str && *str)
    {
        uint8_t b = (uint8_t)*str;

        if (b < 0x80)
        {
            width += 8;
            str += 1;
        }
        else if ((b & 0xE0) == 0xC0)       str += 2;
        else if ((b & 0xF0) == 0xE0)       { width += 16; str += 3; }
        else if ((b & 0xF8) == 0xF0)       { width += 16; str += 4; }
        else                               str += 1;
    }

    return width;
}

static menu_period_t menu_list_display(void)
{
    uint8_t first, last;
    uint8_t selected = Menu_GetCurrentId();

    if (!Menu_GetSiblingRange(selected, &first, &last))
        return 0;

    uint8_t count = last - first + 1;
    uint8_t index = selected - first;
    uint8_t lines_per_page = OLED_HEIGHT / MENU_ITEM_HEIGHT;

    /* 循环滚动窗口: 按环滚动, 末项↔首项平滑衔接不跳变。
     * 短列表(<一屏)自然逐行显示; 长列表时选中项钉在最末行,
     * 窗口起始序号 = (index-(lines_per_page-1)) mod count, 随移动逐格旋转。 */
    int16_t base = 0;
    if (count > lines_per_page)
    {
        base = (int16_t)index - (int16_t)(lines_per_page - 1);
        base %= (int16_t)count;
        if (base < 0)
            base += (int16_t)count;
    }

    OLED_Clear();

    for (uint8_t i = 0; i < lines_per_page; i++)
    {
        int16_t v = base + (int16_t)i;
        uint8_t idx;

        if (count > lines_per_page)
        {
            /* 环内取模下标: 只有越过 count 边界一次才需回绕 */
            if (v >= (int16_t)count)
                v -= (int16_t)count;
            idx = (uint8_t)v;
        }
        else
        {
            if (v >= (int16_t)count)   /* 短列表: 末尾行留空 */
                break;
            idx = (uint8_t)v;
        }

        uint8_t id = first + idx;
        const menu_item_t *item = Menu_GetItem(id);

        if (!item)
            continue;

        int16_t y = (int16_t)i * MENU_ITEM_HEIGHT;

        if (item->name)
        {
            /* 文本水平居中: 根据文本宽度动态计算起始 X 坐标, 再左移 2px 微调 */
            uint8_t text_width = menu_text_width(item->name);
            int16_t x_center = (int16_t)((OLED_WIDTH - text_width) / 2) - 2;

            if (x_center < 0)
                x_center = 0;

            OLED_ShowChinese(x_center, y, item->name);
        }

        if (idx == index)
        {
            /* 选中框: 反显整行(黑底白字), 与文本 8x16 行高完全贴合 */
            OLED_ReverseRoundRect(1, y, OLED_WIDTH - 2, 16, 6);
        }
    }

    OLED_Update();

    return 0;
}

/* 页面渲染(handler 状态): 委托给当前项的 ops->display */
static menu_period_t menu_page_display(void)
{
    const menu_item_t *item = Menu_GetItem(Menu_GetCurrentId());

    if (item && item->ops && item->ops->display)
        return item->ops->display(item->user_data);

    return 0;
}

menu_period_t UI_Render(void)
{
    if (Menu_GetState() == MENU_STATE_NORMAL)
    {
        /* HOME 恒为 handler, 不会落到 NORMAL 列表; 此处不再特判 */
        menu_period_t ret = menu_list_display();

#if MENU_ANIM_ENABLE
        if (AnimNav_Advance())
            UI_RequestRefresh();
#endif

        return ret;
    }
    return menu_page_display();
}
