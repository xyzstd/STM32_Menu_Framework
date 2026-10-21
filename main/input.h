#ifndef __INPUT_H
#define __INPUT_H

#include <stdbool.h>

void Input_Init(void);
void Input_Process(void);   /* 主循环: 消费事件队列, 执行菜单逻辑 */
void Input_ClearEvents(void); /* 主循环: 清空事件队列(跳转后丢弃积压事件) */

/* 页面切换后的输入复位: 清空积压事件 + 重同步编码器半格相位。
 * 打包成一个调用, 调用点不会再漏掉一半、或把顺序写错。
 * ⚠️ 必须在**跳转之后**调用 —— 跳转会经 menu_exit_handler 调当前页的 exit, 参数页
 * dirty 时 param_exit → Config_Save() 要擦写 Flash 25~50ms, 期间 TIM2 照常把编码器/
 * 按键事件入队; 先清后跳会漏掉这一批, 它们会被目标页错位消费(误调温/误切型号)。 */
void Input_ResetForJump(void);

/* 队列中是否有待处理的按键事件(CLICK/LPRESS)。只读、不改队列、无副作用。
 * 用途: Input_Process 据此放宽本轮消费上限, 把排在按键前面的积压旋转快速排空
 * —— 按键**不插队**(保持 FIFO 顺序), 只是让它前面别堆东西。
 * 渲染层也可用它在按键待处理时让路(先响应输入, 画面稍后再刷)。 */
bool Input_HasKeyPending(void);

#endif
