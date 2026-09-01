#ifndef _PWM_H
#define _PWM_H

/* PWM 占空比满刻度 (对应 ARR=999, 1000 级分辨率, 与 pwm.c 的 PWM_ARR_VALUE 一致) */
#define PWM_CCR_MAX   999U

void PWM_Init(void);
void PWM_SetCompare1(uint16_t);
	
#endif
