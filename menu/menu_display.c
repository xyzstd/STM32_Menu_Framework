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

/* 计算 UTF-8 字符串显示宽度: ASCII 8px, 多字节字符(中文/扩展)16px (与 OLED_ShowChinese 一致) */
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
        else if ((b & 0xE0) == 0xC0)       { width += 16; str += 2; }   /* 2 字节: 拉丁扩展/希腊 */
        else if ((b & 0xF0) == 0xE0)       { width += 16; str += 3; }   /* 3 字节: 中文 */
        else if ((b & 0xF8) == 0xF0)       { width += 16; str += 4; }
        else                               str += 1;
    }

    return width;
}

/* 环形滚动窗口状态: 窗口顶行项(相对 first 的下标) + 高亮所在行(0..lines-1)。
 * 高亮在窗口内逐格移动; 到达窗口边缘时才整体滚动一行; 首↔末由取模保证连续。
 * 不变量: 静止时 (win_top + win_cursor) % count == index。 */
static uint8_t win_first  = 0xFF;
static uint8_t win_count  = 0;
static uint8_t win_top    = 0;   /* 窗口第 0 行显示的项下标 */
static uint8_t win_cursor = 0;   /* 高亮所在行(0..lines-1) */
static uint8_t win_prev   = 0;   /* 上一帧选中下标, 用于判定移动方向(含环回) */


/* 环形窗口状态推进: 依当前选中下标(index)与上一帧(win_prev)更新 win_top/win_cursor。
 * 返回 true 为进入新列表/选中跳变(需全量重绘); false 为窗口内正常移动。 */
static bool menu_list_advance(uint8_t index, uint8_t first, uint8_t count)
{
    uint8_t lines_per_page = MENU_LINES_PER_PAGE;

    /* 进入新列表(或列表结构变化): 窗口顶行定格选中项, 高亮末行首行 */
    if (first != win_first || count != win_count)
    {
        /* 列表切换: 窗口顶行落到选中项, 保证高亮第一行必在可见窗口内。
         * 首次进入列表 index=0, 行为与 win_top=0 一致; 但"从子列表返回"时
         * 选中项 index 可能 ≥ 每页行数, 若仍定格 0 会滚出可见窗口而丢失反显。 */
        win_first  = first;
        win_count  = count;
        win_top    = index;
        win_cursor = 0;
        win_prev   = index;
        return true;
    }

    if (count > 1)
    {
        /* 移动方向仅 ±1, 首末处由环回值识别(跨界方向不歧义) */
        int8_t dir = 0;

        if      (win_prev == count - 1 && index == 0)          dir =  1;  /* 尾→首(下滚环回) */
        else if (win_prev == 0 && index == count - 1)          dir = -1;  /* 首→尾(上卷环回) */
        else if (index > win_prev)                             dir =  1;
        else if (index < win_prev)                             dir = -1;

        if (dir > 0)
        {
            /* 高亮逐格下移; 到底行则整窗滚一行, 内容自底部滑入 */
            if (win_cursor < lines_per_page - 1)
                win_cursor++;
            else
                win_top = (uint8_t)((win_top + 1) % count);
        }
        else if (dir < 0)
        {
            /* 高亮逐格上移; 到顶行则整窗反滚一行, 内容自顶部滑出 */
            if (win_cursor > 0)
                win_cursor--;
            else
                win_top = (uint8_t)((win_top + count - 1) % count);
        }
        win_prev = index;
    }
    else
    {
        /* 单元素列表: 无滚动 */
        win_prev   = index;
        win_top    = 0;
        win_cursor = 0;
    }

    /* 兜底防脱同步: 列表重入或选中跳变时, 令选中项落在窗口首行 */
    if (((win_top + win_cursor) % count) != index)
    {
        win_top    = index;
        win_cursor = 0;
        return true;
    }

    return false;
}


/* 列表渲染: 按已推进的窗口(win_top/win_cursor)绘制当前帧, 环形取模显示可见项 */
static void menu_list_render(uint8_t first, uint8_t count, uint8_t index)
{
    uint8_t lines_per_page = MENU_LINES_PER_PAGE;

    OLED_Clear();

    for (uint8_t r = 0; r < lines_per_page; r++)
    {
        uint8_t idx;

        if (count > lines_per_page)
            idx = (uint8_t)((win_top + r) % count);   /* 环形可见窗口 */
        else
        {
            if (r >= count)                          /* 短列表: 末尾行留空 */
                break;
            idx = r;
        }

        uint8_t id = first + idx;
        const menu_item_t *item = Menu_GetItem(id);

        if (!item)
            continue;

        int16_t y = (int16_t)r * MENU_ITEM_HEIGHT;

        if (item->name)
        {
            /* 文本在选中框内水平居中: 由文本宽度动态求起始 X, 再左移 2px 视觉微调 */
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

    OLED_UpdateDiff();      /* 差量推送: 仅高亮/滚动行所在页变化时更新 */
}


static menu_period_t menu_list_display(void)
{
    uint8_t first, last;
    uint8_t selected = Menu_GetCurrentId();

    if (!Menu_GetSiblingRange(selected, &first, &last))
    {
        /* 当前节点无兄弟列表(孤立/配置错误): 退出列表态, 清除窗口记录 */
        win_first = 0xFF;
        return 0;
    }

    uint8_t count = last - first + 1;
    uint8_t index = selected - first;

    menu_list_advance(index, first, count);   /* 更新环形窗口状态 */
    menu_list_render(first, count, index);    /* 绘制当前帧 */

    return 0;
}

/* 页面渲染(handler 状态): 委托给当前项的 ops->display */
static menu_period_t menu_page_display(void)
{
    const menu_item_t *item = Menu_GetItem(Menu_GetCurrentId());

    if (item && item->ops && item->ops->display)
        return item->ops->display(Menu_GetUserData(item));

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
