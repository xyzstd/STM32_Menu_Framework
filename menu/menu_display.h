#ifndef __MENU_DISPLAY_H
#define __MENU_DISPLAY_H

#include "menu.h"

/* 菜单导航动画开关: 1=启用选中框滑动/整页滑动动画, 0=关闭(默认不启用) */
#define MENU_ANIM_ENABLE 0

menu_period_t UI_Render(void);
void UI_RequestRefresh(void);

#endif
