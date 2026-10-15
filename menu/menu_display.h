#ifndef __MENU_DISPLAY_H
#define __MENU_DISPLAY_H

#include "menu.h"

/* 菜单导航动画开关: 1=启用选中框滑动/整页滑动动画, 0=关闭(默认不启用).
 * ⚠️ 置 1 之前必须先把 menu/menu_anim.c 加回工程构建(它当前不在 .uvprojx 里),
 *    并完成渲染层接入 —— 详见 menu_anim.h 顶部的"若要启用"三步, 否则会链接失败
 *    或打开宏却毫无动画。 */
#define MENU_ANIM_ENABLE 0

/* 菜单渲染入口: 渲染当前帧, 返回本页要求的刷新周期(0=不周期刷新).
 * 归属 menu 层(它描述"菜单长什么样"), 由应用层的刷新调度按周期/事件调用.
 * 命名带 MenuDisplay_ 前缀而非 UI_: 后者看不出归属, 曾与 main/ui_task 混为一谈. */
menu_period_t MenuDisplay_Render(void);

/* 刷新请求 hook: 动画"本帧未完、需再渲染一帧"时, menu 层通过它通知调度方.
 * 这样 menu 层不必按名字声明/依赖应用层的 UI_RequestRefresh() —— 菜单内核与
 * 其视图层才能成对搬走(换项目时只需在新应用里注册一次 hook). */
void MenuDisplay_SetRefreshHook(void (*hook)(void));

#endif
