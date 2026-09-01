#ifndef __HANDLE_H
#define __HANDLE_H

#include "menu.h"

/* 手柄型号页 (T12 / T245 / T210).
 * 单一型号项: UP/DOWN 循环切换预览; 返回上级必经 YES/NO 确认页,
 * 确认页显示当前选中的型号, 选"是"确认切换(自动保存并触发安全闭合),
 * 选"否"/LPRESS 取消返回重选. (型号页专用的 ops, 内部状态机渲染单个视图与确认页) */
extern const menu_ops_t handle_ops;

#endif