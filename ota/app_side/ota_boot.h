#ifndef __OTA_BOOT_H
#define __OTA_BOOT_H

#include <stdint.h>
#include <stdbool.h>

/* 擦除 Flash 标志页 + 写魔数 + 软复位进入 Bootloader 升级模式（不返回）*/
void OTA_EnterBootloader(void);

/* 串口收到 OTA 请求 3 字节序列后由 Serial_HandleRx 设置 */
void OTA_SetAppRequest(bool v);
bool OTA_GetAppRequest(void);

/* Serial 识别序列时调用，把字节喂进状态机 */
void OTA_SerialFeedRx(uint8_t b);

#endif /* __OTA_BOOT_H */
