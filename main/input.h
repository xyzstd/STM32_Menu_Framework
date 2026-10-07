#ifndef __INPUT_H
#define __INPUT_H

void Input_Init(void);
void Input_Process(void);   /* 主循环: 消费事件队列, 执行菜单逻辑 */
void Input_ClearEvents(void); /* 主循环: 清空事件队列(强制跳转前丢弃积压事件) */

#endif
