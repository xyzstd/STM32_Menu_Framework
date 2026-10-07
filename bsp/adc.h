#ifndef _AD_H
#define _AD_H

#include <stdint.h>

void AD_Init(void);
uint16_t AD_GetValue(uint8_t channel);
uint16_t AD_GetT12(void);
uint16_t AD_GetVin(void);
/* 差分后导通电流 = 导通相位采样 - 关断相位采样 (同一注入组, CC4 逐帧交替相位)。
 * 加热全关期间无导通采样, 返回 0(非陈旧值)。
 * 标志为消费式: 读一次清一次, 同一周期内重复读第二次会得 0。 */
uint16_t AD_GetCurrent(void);
int16_t AD_GetCpuTempC(void);   /* CPU 内部温度 °C×10 (冷端补偿参考) */

#endif