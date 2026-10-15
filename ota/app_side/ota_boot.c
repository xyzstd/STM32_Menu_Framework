#include "ota_boot.h"
#include "ota_config.h"

/* 本文件整体受 OTA_APP_SIDE_ENABLED 控制(见 ota_boot.h 说明):
 * 关闭时编译为空单元 —— 内容仍保留, 改开关 + 恢复编译即可原样复活.
 * 注意: 本文件已从 Keil 工程排除编译, 所以当前连空单元都不会产出;
 * 下面的 #if 是为"重新纳入工程但忘了改开关"这种情况兜底, 避免半开状态. */
#if OTA_APP_SIDE_ENABLED

#include "stm32f10x.h"

/* OTA 进入过程的上位机串口日志开关.
 * 默认关闭: 这三条 printf 是纯调试输出(用户看不到, 进入升级后马上复位),
 * 却是全工程最后一处把 stdio 格式化机制拉进链接的引用 —— 保留它们会让
 * 整份 printf 及其连带的浮点支持(实测约 2.3KB)常驻 Flash.
 * 需要用串口观察进入流程时置 1 即可, 其余代码不受影响. */
#define OTA_BOOT_DEBUG  0

/* 标志页写入重试次数: 单次失败常见于瞬时低压, 重试一次即可覆盖绝大多数情况 */
#define OTA_FLAG_WRITE_RETRY  2

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
    uint8_t retry;

    /* 擦+写标志页, **回读校验通过才复位**; 失败则返回(不复位).
     * 原先不检查 FLASH_ErasePage / FLASH_ProgramWord 的返回值就无条件复位: 低压或
     * 写保护导致写失败时魔数没写进去, Bootloader 读不到标志 → 直接回 App, 用户看到
     * "正在重启"却毫无变化, 且没有任何错误反馈(本仓库 config.c/flash.c 都检查返回值,
     * 这里是唯一漏点). 回读校验不能省 —— 编程操作报 FLASH_COMPLETE 不代表内容正确. */
    for (retry = 0; retry < OTA_FLAG_WRITE_RETRY; retry++)
    {
#if OTA_BOOT_DEBUG
        printf("[APP] erase+write flag page 0x%08X (try %u)...\r\n",
               (unsigned int)OTA_FLAG_FLASH_ADDR, (unsigned)retry);
#endif
        FLASH_Unlock();
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

        if (FLASH_ErasePage(OTA_FLAG_FLASH_ADDR) == FLASH_COMPLETE &&
            FLASH_ProgramWord(OTA_FLAG_FLASH_ADDR, OTA_FLAG_MAGIC) == FLASH_COMPLETE)
        {
            FLASH_Lock();
            if (*(volatile uint32_t *)OTA_FLAG_FLASH_ADDR == OTA_FLAG_MAGIC)
                break;   /* 已写入且回读一致 */
        }
        else
        {
            FLASH_Lock();
        }
    }

#if OTA_BOOT_DEBUG
    printf("[APP] verify readback=0x%08lX\r\n", (unsigned long)(*(volatile uint32_t *)OTA_FLAG_FLASH_ADDR));
#endif

    if (*(volatile uint32_t *)OTA_FLAG_FLASH_ADDR != OTA_FLAG_MAGIC)
        return;   /* 标志页写入失败: 不复位, 由调用方提示并留在原页 */

    /* 标志页已确认写入(非易失, 任意复位不丢) → 复位进 Bootloader, 不返回 */
    NVIC_SystemReset();
}

#endif /* OTA_APP_SIDE_ENABLED */
