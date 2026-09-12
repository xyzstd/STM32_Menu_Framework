/*====================================================================
 * 菜单导航动画模块实现
 * 帧格式: 见 menu_anim.h。
 * 实现要点:
 *   - 动画状态收敛到单一结构体, 由本模块统一维护
 *   - step 从 1 起步: 跳过与静止画面相同的 t=0 帧,
 *     使"停住后再转一格"首帧立即可见移动, 不产生卡顿感
 *====================================================================*/
#include "menu_anim.h"

/* 动画运行状态 —— 单一数据源 */
static struct {
    AnimType  type;
    uint8_t   step;          /* 当前帧步进(1..ANIM_STEPS); == ANIM_STEPS 表示空闲 */
    int16_t   from_y;        /* ANIM_BAR: 起始行 Y */
    int16_t   to_y;          /* ANIM_BAR: 目标行 Y */
    int8_t    dir;           /* ANIM_PAGE: +1 向下翻页, -1 向上翻页 */
    uint8_t   prev_thumb_y;  /* 上次滚动条滑块 Y(翻页过渡) */
} anim;

/* 当前帧进度 —— 非线性缓动:
 * 归一化进度 t=step/ANIM_STEPS 经 easeOutBack 曲线映射,
 * 使动画"快启动、滑过目标后轻微回弹", 视觉更具弹性。 */

/* easeOutBack: f = 1 - c3*u^3 + c1*u^2, u = 1-t, c1=1.70158, c3=2.70158
 * 末尾带轻微过冲回弹(overshoot≈1.1), 即选中框滑过目标一点再弹回。
 * 入参 0..ANIM_UNIT; 返回值 0..~1.1*ANIM_UNIT(允许越过 1.0)。*/
static uint16_t anim_ease(uint16_t t01)
{
    const uint32_t C1_Q = 1742;   /* 1.70158 * ANIM_UNIT */
    const uint32_t C3_Q = 2766;   /* 2.70158 * ANIM_UNIT */
    uint32_t u   = ANIM_UNIT - t01;          /* u (Q) */
    uint32_t u2  = u * u / ANIM_UNIT;        /* u^2 (Q) */
    uint32_t u3  = u2 * u / ANIM_UNIT;       /* u^3 (Q) */
    uint32_t f   = ANIM_UNIT
                   - C3_Q * u3 / ANIM_UNIT
                   + C1_Q * u2 / ANIM_UNIT;
    return (uint16_t)f;
}

/* 当前帧缓动后的归一化进度(0..ANIM_UNIT) */
static uint16_t anim_eased01(void)
{
    uint16_t t01 = (uint16_t)((uint32_t)anim.step * ANIM_UNIT / ANIM_STEPS);
    return anim_ease(t01);
}

/* 当前帧缓动后的像素进度(0..ANIM_LINE_H) */
static int16_t anim_progress(void)
{
    return (int16_t)((uint32_t)anim_eased01() * ANIM_LINE_H / ANIM_UNIT);
}

/* ---- 控制接口 ---- */

void AnimNav_Stop(void)
{
    anim.step = ANIM_STEPS;
}

void AnimNav_StartBar(int16_t from_y, int16_t to_y)
{
    anim.step   = 1;
    anim.from_y = from_y;
    anim.to_y   = to_y;
}

void AnimNav_StartPage(int8_t dir)
{
    anim.step = 1;
    anim.dir  = dir;
}

int AnimNav_Advance(void)
{
    if (anim.step < ANIM_STEPS)
    {
        anim.step++;
        return 1;
    }
    anim.step = ANIM_STEPS;
    return 0;
}

/* ---- 查询接口 ---- */

AnimType AnimNav_Type(void)
{
    return anim.type;
}

/* 翻页动画: 内容尚未滑到位的垂直位移(0..ANIM_LINE_H) */
static int16_t anim_page_delta(void)
{
    return (int16_t)(ANIM_LINE_H - anim_progress());
}

int16_t AnimNav_PageOffset(void)
{
    if (anim.type != ANIM_PAGE)
        return 0;
    /* 向下翻页(dir=+1): 内容自底部(+LINE_H)上滑到原位; 向上翻页则相反 */
    return (int16_t)((int16_t)anim.dir * anim_page_delta());
}

int16_t AnimNav_BarY(int16_t base_y)
{
    switch (anim.type) {
    case ANIM_PAGE:
        return base_y + (int16_t)((int16_t)anim.dir * anim_page_delta());
    case ANIM_BAR:
        return base_y + (int16_t)((anim.to_y - anim.from_y) * (int16_t)anim_eased01() / ANIM_UNIT);
    default:
        return base_y;
    }
}

int16_t AnimNav_StepRatio(void)
{
    return (int16_t)anim_eased01();
}

/* ---- 滚动条过渡 ---- */

void AnimNav_SetThumbY(uint8_t y)
{
    anim.prev_thumb_y = y;
}

uint8_t AnimNav_GetThumbY(void)
{
    return anim.prev_thumb_y;
}

