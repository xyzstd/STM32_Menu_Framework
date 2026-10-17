#include "menu.h"
#include "oled.h"
#include "menu_display.h"
#include "menu_anim.h"

#define MENU_LINES_PER_PAGE (OLED_HEIGHT / OLED_8X16)
#define MENU_ITEM_HEIGHT OLED_8X16

/* 环形滚动窗口状态: 窗口顶行项(相对 first 的下标) + 高亮所在行(0..lines-1)。
 * 高亮在窗口内逐格移动; 到达窗口边缘时才整体滚动一行; 首↔末由取模保证连续。
 * 不变量: 静止时 (win_top + win_cursor) % count == index。 */
static uint8_t win_first  = 0xFF;
static uint8_t win_count  = 0;
static uint8_t win_top    = 0;   /* 窗口第 0 行显示的项下标 */
static uint8_t win_cursor = 0;   /* 高亮所在行(0..lines-1) */
static uint8_t win_prev   = 0;   /* 上一帧选中下标, 用于判定移动方向(含环回) */
static bool    win_entry_pending = false;   /* 刚离开列表态(渲染过页面): 下次进入列表按"新进入"复位窗口 */

/* 刷新请求 hook(由应用层注册, 见 menu_display.h): 仅动画需要"本帧未完"时调用.
 * menu 层借此与调度方解耦, 不按名字依赖 UI_RequestRefresh().
 * 变量与消费点同受 MENU_ANIM_ENABLE 约束, 否则动画关闭时"只写不读"会告警. */
#if MENU_ANIM_ENABLE
static void (*menu_refresh_hook)(void) = 0;
#endif

void MenuDisplay_SetRefreshHook(void (*hook)(void))
{
#if MENU_ANIM_ENABLE
    menu_refresh_hook = hook;
#else
    (void)hook;   /* 动画关闭: 暂无消费方, 保留注册接口使调用方无需条件编译 */
#endif
}

/* 本文件的 static 函数均按依赖序定义(被调用者在前), 故无需前置声明.
 * 新增函数时放在调用者之前即可 —— 否则 AC5 会因隐式声明直接报错(硬失败, 不静默). */


/* 窗口复位: 顶行定格选中项, 高亮落在首行.
 * 进入新列表 / 列表结构变化 / 从页面态返回首项时调用 ——
 * 保证高亮必在可见窗口内(从子列表返回时选中项 index 可能 ≥ 每页行数,
 * 若仍定格 0 会滚出可见窗口而丢失反显). */
static void menu_window_reset(uint8_t index, uint8_t first, uint8_t count)
{
    win_first  = first;
    win_count  = count;
    win_top    = index;
    win_cursor = 0;
    win_prev   = index;
}

/* 选中项在两帧之间的移动方向: +1 下移 / -1 上移 / 0 未动.
 * 首末位置必须按"环回值"识别 —— count-1→0 是下滚环回、0→count-1 是上卷环回;
 * 只比大小会把这两者判成"跨 count-1 格的大跳", 动画方向就反了. */
static int8_t menu_move_dir(uint8_t prev, uint8_t index, uint8_t count)
{
    if (prev == count - 1 && index == 0)
        return 1;
    if (prev == 0 && index == count - 1)
        return -1;
    if (index > prev)
        return 1;
    if (index < prev)
        return -1;
    return 0;
}

/* 环形窗口状态推进: 依当前选中下标(index)与上一帧(win_prev)更新 win_top/win_cursor。 */
static void menu_list_advance(uint8_t index, uint8_t first, uint8_t count)
{
    uint8_t lines_per_page = MENU_LINES_PER_PAGE;

    /* 进入新列表(或列表结构变化): 窗口顶行定格选中项, 高亮末行首行 */
    if (first != win_first || count != win_count)
    {
        menu_window_reset(index, first, count);
        return;
    }

    if (count > 1)
    {
        int16_t jump = (int16_t)index - (int16_t)win_prev;
        /* 首末环回(|jump| == count-1): 仍走逐格滚动, 保留慢速环回的连贯动画 */
        bool    wrap = (jump == (int16_t)(count - 1) || jump == -(int16_t)(count - 1));

        /* 快速旋转跳变(两帧间选中项跨多格, 且非首末环回): 跳过逐格动画,
         * 保持高亮所在行不变, 窗口整体跟随选中项滑动, 消除"先到顶再掉回"的错位抖动. */
        if (!wrap && (jump > 1 || jump < -1))
        {
            int16_t top = (int16_t)index - (int16_t)win_cursor;

            while (top < 0)
                top += (int16_t)count;
            win_top    = (uint8_t)(top % (int16_t)count);
            win_prev   = index;
            return;
        }

        /* 移动方向仅 ±1, 首末处由环回值识别(跨界方向不歧义) */
        int8_t dir = menu_move_dir(win_prev, index, count);

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
    }
}


/* 行高 16px 的绘图布局 (靠左): [x8 圆点] [x22 菜单名] ...... [右侧内容] */
#define MENU_DOT_X      12    /* 圆点圆心 x */
#define MENU_TEXT_X     22
#define MENU_CHEV_X     114   /* 可进入三角 ▶ 起始列 */

/* 左侧状态圆点: 靶心 (r3~4 圆环 + r1 中心亮心).
 * 逐点绘制保证真圆, 反显不影响 */
static void menu_draw_dot(int16_t yc)
{
    for (int16_t j = -4; j <= 4; j++)
        for (int16_t i = -4; i <= 4; i++)
        {
            int16_t d = i * i + j * j;
            if (d <= 16 && (d >= 9 || d <= 1))
                OLED_DrawPixel(MENU_DOT_X + i, yc + j, OLED_COLOR_SET);
        }
}

/* 右侧可进入三角 ▶: 8px 高 4px 宽, 行内垂直居中 (逐列渐宽实心列) */
static void menu_draw_chevron(int16_t y)
{
    for (uint8_t i = 0; i < 4; i++)
        OLED_FillRect((int16_t)(MENU_CHEV_X + i), (int16_t)(y + 3 + i), 1,
                      (uint8_t)(9 - 2 * i), OLED_COLOR_SET);
}

/* 选中框反显(字节级): 与 OLED_ReverseRoundRect(1,y,OLED_WIDTH-2,16,6) 逐像素结果
 * 完全一致——按同款圆角跨度公式(与 OLED_FillRoundRect 相同的 dx 判定)逐行算出
 * left..right, 直接对显示缓冲按位 XOR, 免去 ~2000 次 DrawPixel 函数调用
 * (≈0.9ms/帧 → ≈0.05ms). 行高 16px 页对齐, 单行 = 2 个整页, 无跨页拆分. */
static void menu_reverse_row(int16_t y)
{
    const int16_t x      = 1;
    const int16_t width  = OLED_WIDTH - 2;
    const int16_t height = 16;
    const int16_t radius = 6;
    int16_t yy;

    for (yy = 0; yy < height; yy++)
    {
        int16_t left  = x;
        int16_t right = x + width - 1;
        /* 本行到最近圆心的垂直距离; < 0 表示不在圆角区(整行实心) */
        int16_t dy    = -1;

        if (yy < radius)
            dy = radius - 1 - yy;
        else if (yy >= height - radius)
            dy = yy - (height - radius);

        if (dy >= 0)
        {
            /* 圆角行: 由圆心距反解本行左右内缩量 dx */
            int16_t dx = 0;

            while ((dx + 1) * (dx + 1) + dy * dy <= radius * radius)
                dx++;

            left  = x + radius - dx;
            right = x + width - radius + dx - 1;
        }

        {
            uint8_t page = (uint8_t)((y + yy) >> 3);
            uint8_t mask = (uint8_t)(1u << ((y + yy) & 0x07));
            int16_t c;

            for (c = left; c <= right; c++)
                OLED_DisplayBuf[page][c] ^= mask;
        }
    }
}


/* 第 r 行显示列表里的第几项(相对 first). 长列表按环形窗口取模; 短列表直接顺序
 * 排, 返回值 >= count 表示该行无内容(末尾留空), 由调用方 break. */
static uint8_t menu_row_index(uint8_t r, uint8_t count)
{
    if (count > MENU_LINES_PER_PAGE)
        return (uint8_t)((win_top + r) % count);

    return r;
}

/* 列表渲染: 按已推进的窗口(win_top/win_cursor)绘制当前帧, 环形取模显示可见项 */
static void menu_list_render(uint8_t first, uint8_t count, uint8_t index)
{
    uint8_t lines_per_page = MENU_LINES_PER_PAGE;

    OLED_Clear();

    for (uint8_t r = 0; r < lines_per_page; r++)
    {
        uint8_t idx = menu_row_index(r, count);

        if (idx >= count)   /* 短列表: 末尾行留空 */
            break;

        uint8_t id = first + idx;
        const menu_item_t *item = Menu_GetItem(id);

        if (!item)
            continue;

        int16_t y = (int16_t)r * MENU_ITEM_HEIGHT;

        /* 左侧圆点: 导航态=靶心圆环 */
        menu_draw_dot((int16_t)(y + 8));

        /* 名字靠左 */
        if (item->name)
            OLED_ShowChinese(MENU_TEXT_X, y, item->name);

        if (item->child_num > 0 || item->ops)
            menu_draw_chevron(y);   /* 可进入: 子菜单/页面/动作 → 右侧三角 */

        if (idx == index)
        {
            /* 选中框: 反显整行(黑底白字). 字节级 XOR, 与 OLED_ReverseRoundRect 逐像素一致 */
            menu_reverse_row(y);
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

    /* 从页面态回到列表(常驻页/参数页/确认页)视为"新进入": 子菜单进入恒落首项(index==0),
     * 强制复位窗口到首行首项(进入始终指向第一项); 参数页返回落在自身项(index 可能非 0),
     * 保持原位窗口不清零, 高亮不动. 列表内连续导航不触发. */
    if (win_entry_pending)
    {
        win_entry_pending = false;
        if (index == 0)
        {
            menu_window_reset(index, first, count);
            menu_list_render(first, count, index);
            return 0;
        }
    }

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

menu_period_t MenuDisplay_Render(void)
{
    if (Menu_GetState() == MENU_STATE_NORMAL)
    {
        /* HOME 恒为 handler, 不会落到 NORMAL 列表; 此处不再特判 */
        menu_period_t ret = menu_list_display();

#if MENU_ANIM_ENABLE
        if (AnimNav_Advance() && menu_refresh_hook)
            menu_refresh_hook();   /* 动画未完: 请调度方再渲染一帧 */
#endif

        return ret;
    }
    win_entry_pending = true;   /* 离开列表态(渲染页面): 下次进入列表按新进入处理 */
    return menu_page_display();
}
