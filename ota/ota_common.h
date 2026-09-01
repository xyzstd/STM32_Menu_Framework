#ifndef __OTA_COMMON_H
#define __OTA_COMMON_H

/*================ OTA 共享常量 ================
 * 此文件包含 Bootloader 和 App 端都需要的常量定义，
 * 统一维护避免修改一处遗漏另一处。
 */

#include <stdint.h>

/* ---- 升级请求标志 (Flash 魔数，非易失) ----
 * App: 擦除标志页 → 写魔数 → 软复位
 * Boot: 读 Flash 标志页，若匹配 → 擦除 → 进入升级
 * 地址 0x08007800 = Boot 区最后一页 (page 15, 2KB)，Boot 代码约 6KB 不冲突
 */
#define OTA_FLAG_FLASH_ADDR  0x08007800UL
#define OTA_FLAG_MAGIC       0xA5A5A5A5UL

/* ---- App 区 Flash 基地址 ----
 * Boot 跳转、App 向量表重定位、上位机烧录都使用此地址，单一来源
 */
#define OTA_APP_FLASH_BASE   0x08008000UL

/* ---- 帧头检测 ---- */
#define OTA_FRAME_HDR_1  0xAA
#define OTA_FRAME_HDR_2  0x55

/* ---- 协议命令码 ---- */
typedef enum {
    OTA_CMD_START     = 0x01,
    OTA_CMD_START_ACK = 0x02,
    OTA_CMD_DATA      = 0x03,
    OTA_CMD_DATA_ACK  = 0x04,
    OTA_CMD_DATA_NACK = 0x05,
    OTA_CMD_END       = 0x06,
    OTA_CMD_END_ACK   = 0x07,
    OTA_CMD_ABORT     = 0x08
} ota_cmd_t;

/* ---- START_ACK / END_ACK 状态码 ---- */
#define OTA_ACK_OK       0
#define OTA_ACK_BUSY     1
#define OTA_ACK_ERROR    2

/* ---- 协议参数 ---- */
#define OTA_PKT_MAX_DATA       1024U
#define OTA_TIMEOUT_MS         30000U

/* ---- App 端触发序列 ----
 * 上位机发送此 3 字节序列触发 App 端进入 OTA 升级
 */
#define OTA_APP_REQ_B0   0xA5
#define OTA_APP_REQ_B1   0x5A
#define OTA_APP_REQ_B2   0x09

#endif /* __OTA_COMMON_H */
