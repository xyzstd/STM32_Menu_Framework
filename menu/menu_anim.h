#ifndef __MENU_ANIM_H
#define __MENU_ANIM_H

#include <stdint.h>

/*====================================================================
 * 菜单导航动画模块
 * 负责选中框滑动(ANIM_BAR)与整页滑入(ANIM_PAGE)的状态机。
 * 渲染层(menu_display.c)只调用本模块的控制接口和查询接口,
 * 不关心动画内部步进/插值细节。
 *====================================================================*/

#define ANIM_STEPS   8U        /* 每段动画帧数(20ms/帧 ≈ 160ms) */
#define ANIM_PERIOD  20U       /* 动画期间周期刷新间隔(ms) */
#define ANIM_LINE_H  16U       /* 菜单行高(像素), 须与渲染层 ITEM_H 一致 */
#define ANIM_UNIT    1024U     /* 归一化进度定点单位(1.0 = 1024) */

typedef enum{
    ANIM_NONE = 0,
    ANIM_BAR,      /* 选中框在行间滑动 */
    ANIM_PAGE,     /* 跨页: 整页内容从进入侧滑入 */
} AnimType;

/* ---- 控制接口 ---- */
void AnimNav_Stop(void);
void AnimNav_StartBar(int16_t from_y, int16_t to_y);
void AnimNav_StartPage(int8_t dir);
int  AnimNav_Advance(void);         /* 1=动画未结束(需周期刷新), 0=已结束 */

/* ---- 查询接口(渲染层读取) ---- */
AnimType  AnimNav_Type(void);
int16_t   AnimNav_PageOffset(void);   /* 跨页动画内容垂直偏移; 非跨页=0 */
int16_t   AnimNav_BarY(int16_t base_y); /* 选中框当前 Y(按动画类型插值) */
int16_t   AnimNav_StepRatio(void);    /* 当前帧缓动后归一化进度(0..ANIM_UNIT), 供按比例插值 */

/* ---- 滚动条过渡(翻页动画时滑块平滑) ---- */
void    AnimNav_SetThumbY(uint8_t y);
uint8_t AnimNav_GetThumbY(void);

#endif /* __MENU_ANIM_H */

