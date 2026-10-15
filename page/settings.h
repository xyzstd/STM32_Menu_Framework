#ifndef __SETTINGS_H
#define __SETTINGS_H

#include "menu.h"

extern const menu_ops_t sleep_set_ops;   /* 休眠设置: 休眠时间/休眠温度/自动关机 三参一页 */
extern const menu_ops_t func_set_ops;    /* 功能设置: 蜂鸣器/屏幕亮度 两参一页 */
extern const menu_ops_t reset_ops;
extern const menu_ops_t version_ops;

#endif
