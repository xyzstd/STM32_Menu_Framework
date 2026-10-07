#ifndef _PWM_H
#define _PWM_H

#include <stdbool.h>

/* PWM 占空比满刻度 (对应 ARR=999, 1000 级分辨率, 与 pwm.c 的 PWM_ARR_VALUE 一致) */
#define PWM_CCR_MAX   999U

void PWM_Init(void);
void PWM_SetCompare1(uint16_t);

/* 注入组采样相位 (ADC 中断逐帧交替): false = 导通段中点(采导通电流),
 * true = 关断段中点(采关断基准)。CC4 无引脚输出, 仅产生比较事件触发注入组。 */
void PWM_SetInjectPhase(bool off_phase);
bool PWM_GetInjectPhase(void);

#endif
