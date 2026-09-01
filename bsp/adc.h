#ifndef _AD_H
#define _AD_H

#include <stdint.h>

void AD_Init(void);
uint16_t AD_GetValue(uint8_t channel);
uint16_t AD_GetT12(void);
uint16_t AD_GetVin(void);
uint16_t AD_GetNTC(void);
uint16_t AD_GetCurrent(void);
uint16_t AD_GetCurrentOff(void);

#endif