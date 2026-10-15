#ifndef __OTA_BOOT_H
#define __OTA_BOOT_H

#include <stdint.h>
#include <stdbool.h>

/*================ App 侧 OTA 模块总开关 ================
 * 本模块(ota/app_side/ota_boot.c)已从 Keil 工程排除编译, 源码保留在仓库里,
 * 由下面的宏决定"逻辑上是否参与构建":
 *   0 = 不参与构建(当前状态): ota_boot.c 编译为空单元, 调用点(main.c 注册回调、
 *       ota_wait.c 的握手跳转)一并消失, "固件升级"菜单页显示"功能未启用".
 *       ⚠️ 单独把这个宏改回 1 还不够 —— 必须同时在 Keil 工程树里右键
 *       ota_boot.c → Options → 取消勾选 "Include in Target Build"(恢复编译),
 *       否则会找不到 OTA_* 符号而链接失败.
 *   1 = 参与构建: 恢复串口触发 + 跳转 Bootloader 的完整行为.
 * 之所以用"显式宏"而不是靠 `#ifdef 文件被编译` 之类隐式判断, 是为了让
 * 两个文件(模块自身与调用方)共享同一事实来源, 不会各自为政.
 *
 * 说明: 上游真正的 Bootloader(Boot 区源码)本就不在本仓库中, 只有 App 侧
 * 支持代码(本文件 + ota_boot.c)与共享头 ota/ota_common.h. */
#ifndef OTA_APP_SIDE_ENABLED
#define OTA_APP_SIDE_ENABLED   0
#endif

#if OTA_APP_SIDE_ENABLED

/* 擦除 Flash 标志页 + 写魔数 + 软复位进入 Bootloader 升级模式（不返回）*/
void OTA_EnterBootloader(void);

/* 串口收到 OTA 请求 3 字节序列后由 Serial_HandleRx 设置 */
void OTA_SetAppRequest(bool v);
bool OTA_GetAppRequest(void);

/* Serial 识别序列时调用，把字节喂进状态机 */
void OTA_SerialFeedRx(uint8_t b);

#endif /* OTA_APP_SIDE_ENABLED */

#endif /* __OTA_BOOT_H */
