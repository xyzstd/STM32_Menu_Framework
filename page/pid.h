#ifndef __PID_H
#define __PID_H

#include "menu.h"

/* PID 参数子菜单两个子页面 (激进/保守) */
extern const menu_ops_t pid_agg_ops;   /* 激进参数: KP/KI/KD */
extern const menu_ops_t pid_cons_ops;  /* 保守参数: KP/KI/KD */

#endif
