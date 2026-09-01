#include "ota_boot.h"
#include "ota_config.h"
#include "stm32f10x.h"
#include <stdio.h>

static volatile bool s_app_request;
static uint8_t s_req_state;

void OTA_SetAppRequest(bool v) { s_app_request = v; }
bool OTA_GetAppRequest(void)    { return s_app_request; }

void OTA_SerialFeedRx(uint8_t b)
{
    switch (s_req_state) {
    case 0: if (b == OTA_APP_REQ_B0) s_req_state = 1; else s_req_state = 0; break;
    case 1: if (b == OTA_APP_REQ_B1) s_req_state = 2; else if (b == OTA_APP_REQ_B0) s_req_state = 1; else s_req_state = 0; break;
    case 2: if (b == OTA_APP_REQ_B2) { s_app_request = true; s_req_state = 0; }
            else if (b == OTA_APP_REQ_B0) s_req_state = 1; else s_req_state = 0;
        break;
    }
}

void OTA_EnterBootloader(void)
{
    printf("[APP] erasing flag page at 0x%08X...\r\n", (unsigned int)OTA_FLAG_FLASH_ADDR);
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    FLASH_ErasePage(OTA_FLAG_FLASH_ADDR);
    printf("[APP] writing magic 0x%08lX to Flash...\r\n", (unsigned long)OTA_FLAG_MAGIC);
    FLASH_ProgramWord(OTA_FLAG_FLASH_ADDR, OTA_FLAG_MAGIC);
    FLASH_Lock();
    printf("[APP] verify readback=0x%08lX\r\n", (unsigned long)(*(volatile uint32_t *)OTA_FLAG_FLASH_ADDR));
    /* 写魔数到 Flash 标志页（非易失，任意复位不丢）*/
    NVIC_SystemReset();
}
