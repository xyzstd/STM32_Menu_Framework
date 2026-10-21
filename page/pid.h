#ifndef __PID_H
#define __PID_H

#include "menu.h"

/* PID 参数页: 单组 KP/KI/KD, 静态实例封闭在 pid.c,
 * 菜单表经 pid_ops 关联本页. */
extern const menu_ops_t pid_ops;

#endif
