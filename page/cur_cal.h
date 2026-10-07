#ifndef __CUR_CAL_H
#define __CUR_CAL_H

#include "menu.h"

/* 本页在菜单表中的 ID (menu_table.c 索引 19, PID 子菜单末项) */
#define CUR_CAL_MENU_ID  19

extern const menu_ops_t cur_cal_ops;

/* 首次引导入口: 置引导模式并强制进入本页 (由 UI 层监视
 * WorkTask_IsBootCalPending 后调用). 引导模式下 CLICK 保存后 /
 * LPRESS 跳回 HOME 并清除引导标志. */
void CurCal_BootEnter(void);

#endif
