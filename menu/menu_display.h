#ifndef __MENU_DISPLAY_H
#define __MENU_DISPLAY_H

#include "menu.h"

/* 菜单导航动画开关: 1=启用选中框滑动/整页滑动动画, 0=关闭(默认不启用) */
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
