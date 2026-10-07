#ifndef __UI_TASK_H
#define __UI_TASK_H

void UI_Task_Init(void);
void UI_Task_Process(void);
void UI_RequestRefresh(void);

/* 说明: 原 UI_ShowSaved() 已下移为页面层的 Toast_Saved() (见 page/toast.h).
 * 提示框是"页面如何反馈"的视觉细节, 原先放在本层迫使 5 个页面 include
 * "ui_task.h" 形成 page -> main 反向依赖, 故移出. */

#endif
