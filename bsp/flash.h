#ifndef __FLASH_H
#define __FLASH_H

#include <stdint.h>

void Flash_Read(uint32_t addr,void *buf,uint16_t len);
void Flash_Write(uint32_t addr,const void *buf,uint16_t len);
void Flash_ErasePage(uint32_t addr);

#endif
