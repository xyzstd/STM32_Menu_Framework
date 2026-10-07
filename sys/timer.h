#ifndef _TIMER_H
#define _TIMER_H

#include <stdint.h>
#include <stddef.h>   /* NULL */

void Timer_Init(void);

uint32_t Timer_GetTick(void);

/* 1ms 节拍回调: 由使用方在初始化时注册, 定时器中断每 1ms 调用一次.
 * 存在意义是分层 —— 定时器驱动不应知道"输入扫描"这回事. 原实现在
 * TIM2_IRQHandler 里直接调 Input_Scan(), 使 sys/ 反向依赖 main/.
 * 现由 main/input.c 自行注册, sys/ 只依赖本抽象.
 * 约束: 回调在中断上下文执行, 必须短小、不可阻塞、不可调耗时函数(如 Flash 擦写).
 * 传 NULL 表示注销. */
typedef void (*timer_tick_fn)(void);
void Timer_SetTickCallback(timer_tick_fn fn);

#endif
