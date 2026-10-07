#include "ota_boot.h"
#include "ota_config.h"
#include "stm32f10x.h"

/* OTA 进入过程的上位机串口日志开关.
 * 默认关闭: 这三条 printf 是纯调试输出(用户看不到, 进入升级后马上复位),
 * 却是全工程最后一处把 stdio 格式化机制拉进链接的引用 —— 保留它们会让
 * 整份 printf 及其连带的浮点支持(实测约 2.3KB)常驻 Flash.
 * 需要用串口观察进入流程时置 1 即可, 其余代码不受影响. */
#define OTA_BOOT_DEBUG  0

#if OTA_BOOT_DEBUG
#include <stdio.h>
#endif

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
#if OTA_BOOT_DEBUG
    printf("[APP] erasing flag page at 0x%08X...\r\n", (unsigned int)OTA_FLAG_FLASH_ADDR);
#endif
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    FLASH_ErasePage(OTA_FLAG_FLASH_ADDR);
#if OTA_BOOT_DEBUG
    printf("[APP] writing magic 0x%08lX to Flash...\r\n", (unsigned long)OTA_FLAG_MAGIC);
#endif
    FLASH_ProgramWord(OTA_FLAG_FLASH_ADDR, OTA_FLAG_MAGIC);
    FLASH_Lock();
#if OTA_BOOT_DEBUG
    printf("[APP] verify readback=0x%08lX\r\n", (unsigned long)(*(volatile uint32_t *)OTA_FLAG_FLASH_ADDR));
#endif
    /* 写魔数到 Flash 标志页（非易失，任意复位不丢）*/
    NVIC_SystemReset();
}
