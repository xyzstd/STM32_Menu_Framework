#ifndef __FLASH_H
#define __FLASH_H

#include <stdint.h>
#include <stdbool.h>

void Flash_Read(uint32_t addr,void *buf,uint16_t len);

/* 落盘 API 返回 true=操作确实成功, false=失败(超时/写保护/编程错误).
 * 调用方对 false 必须如实上报, 不可当作成功 —— 见 flash.c 顶部关于
 * 粘滞错误标志为何会静默吞掉后续所有擦写的说明. */
bool Flash_Write(uint32_t addr,const void *buf,uint16_t len);
bool Flash_ErasePage(uint32_t addr);

#endif
