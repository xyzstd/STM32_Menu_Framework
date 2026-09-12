#include "ota_boot.h"
#include "ota_config.h"
#include "stm32f10x.h"
#include <stdio.h>

static volatile bool app_request;
static uint8_t req_state;

void OTA_SetAppRequest(bool v) { app_request = v; }
bool OTA_GetAppRequest(void)    { return app_request; }

void OTA_SerialFeedRx(uint8_t b)
{
    switch (req_state) {
    case 0: if (b == OTA_APP_REQ_B0) req_state = 1; else req_state = 0; break;
    case 1: if (b == OTA_APP_REQ_B1) req_state = 2; else if (b == OTA_APP_REQ_B0) req_state = 1; else req_state = 0; break;
    case 2: if (b == OTA_APP_REQ_B2) { app_request = true; req_state = 0; }
            else if (b == OTA_APP_REQ_B0) req_state = 1; else req_state = 0;
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
