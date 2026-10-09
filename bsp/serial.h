#ifndef _SERIAL_H
#define _SERIAL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>   /* NULL */

void Serial_Init(void);

/* 接收字节回调: 每个收到的字节在中断上下文里派发一份给注册者
 * (当前注册者是 OTA 请求序列识别). 这样串口驱动不必认识 OTA 协议 ——
 * 原实现直接 include ota_boot.h 并调 OTA_SerialFeedRx(), 形成 bsp -> ota 反依赖.
 * 约束: 回调在中断上下文执行, 必须短小、不可阻塞. 传 NULL 注销.
 * 注: 本驱动不维护接收缓冲、不识别协议; 历史内置的 512B 环形缓冲与
 *     Serial_Available/ReadByte/SendByte/SendBuf 因全工程零调用已移除. */
typedef void (*serial_rx_fn)(uint8_t byte);
void Serial_SetRxCallback(serial_rx_fn fn);

/* 中断服务例程 */
void Serial_RxIRQHandler(void);

#endif
