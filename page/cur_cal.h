#ifndef __CUR_CAL_H
#define __CUR_CAL_H

#include "menu.h"

/* 本页在菜单表中的 ID (menu_table.c 索引 15, PID 子菜单末项).
 * ⚠️ 必须与 menu_table.c 的索引保持一致: 增删/重排菜单项后若忘记同步本宏,
 * CurCal_BootEnter 的跳转会被 Menu_SetCurrentId 的范围检查**静默丢弃**
 * (首次冷态标定引导失效, 却不报错、不崩溃, 极难定位).
 * 由 build_tools/check_menu_table.py 第 13 项校验兜底. */
#define CUR_CAL_MENU_ID  14

extern const menu_ops_t cur_cal_ops;

/* 首次引导入口: 置引导模式并强制进入本页 (由 UI 层监视
 * WorkTask_IsBootCalPending 后调用). 引导模式下 CLICK 保存后 /
 * LPRESS 跳回 HOME 并清除引导标志. */
void CurCal_BootEnter(void);

#endif
