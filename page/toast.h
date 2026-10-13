#ifndef __TOAST_H
#define __TOAST_H

#include <stdbool.h>

/* ==================== 浮动提示框 (页面层原语) ====================
 * 页面在"确实落盘成功"后调用 Toast_Saved(): 在已渲染的画面上叠加一个卡片式
 * 弹窗(白底圆角 + 内嵌细边框 + 对勾 + 黑字), 约 0.6s 后自动消失.
 *
 * 归属: 这是"页面如何反馈"的视觉细节, 故属页面层, 而不是 main/ui_task.
 * 原先放在 ui_task.c 里, 导致 5 个页面文件为了一句反馈去 include "ui_task.h",
 * 形成 page -> main 的反向依赖. 下移后该依赖消失.
 *
 * 用法(渲染调度方 main/ui_task):
 *   每次本帧渲染完成后调用 Toast_Overlay() 叠加; 它返回 true 表示"本帧仍显示",
 *   调度方据此决定是否在到期后再渲染一帧把框擦掉. */
void Toast_Saved(void);      /* 显示"已保存"(0.6s) */
bool Toast_Pending(void);    /* 提示是否仍待显示(未到期) */
bool Toast_Overlay(void);    /* 叠加绘制当前提示; true=本帧画了 */

#endif
