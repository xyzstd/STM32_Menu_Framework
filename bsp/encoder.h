#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"
#include <stdbool.h>

void Encoder_Init(void);

/* 取走自上次调用以来累计的步数(正=正向/UP, 负=反向/DOWN); 读后清零.
 * 由 TIM4 硬件编码器接口计数, 每满一格(4 计数)算一步; input_scan 单上下文调用. */
int16_t Encoder_GetSteps(void);

/* 请求重同步: 丢弃未满一格的残留相位(计数基准不动). 供菜单跨页/跨列表跳转后
 * 调用, 防旧上下文的相位带入新页面吃掉新方向第一格. 主循环置位, 取样上下文执行. */
void Encoder_RequestResync(void);

/* 取走"相位锁死"请求: true = 已判定卡死并丢弃了残差相位, 调用方应作废积压的旋转事件.
 * 主循环上下文调用; 无请求时返回 false. */
bool Encoder_TakeStuck(void);

#endif
