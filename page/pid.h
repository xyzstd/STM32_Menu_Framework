#ifndef __PID_H
#define __PID_H

#include "menu.h"

/* PID 参数子菜单: 两个子页面(激进/保守), 各自以静态实例封闭在 pid.c,
 * 菜单表经 pid_agg_ops / pid_cons_ops 关联对应页面. */
extern const menu_ops_t pid_agg_ops;   /* 激进参数页 */
extern const menu_ops_t pid_cons_ops;  /* 保守参数页 */

#endif