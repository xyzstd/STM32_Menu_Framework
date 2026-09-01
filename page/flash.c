#include "stm32f10x.h"   
#include "flash.h"
#include <string.h>

void Flash_Read(uint32_t addr,void *buf,uint16_t len)
{
    memcpy(buf,(const void *)addr,len);
}

void Flash_ErasePage(uint32_t addr)
{
    FLASH_Unlock();
    FLASH_ErasePage(addr);
    FLASH_Lock();
}

void Flash_Write(uint32_t addr,const void *buf,uint16_t len)
{
    uint16_t i;
    const uint8_t *src=buf;

    FLASH_Unlock();

    for(i=0;i<len;i+=2)
    {
        uint16_t data=src[i];

        if(i+1<len)
            data|=(uint16_t)src[i+1]<<8;
        else
            data|=0xFF00;

        FLASH_ProgramHalfWord(addr+i,data);
    }

    FLASH_Lock();
}
