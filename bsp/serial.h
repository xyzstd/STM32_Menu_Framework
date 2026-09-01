#ifndef _SERIAL_H
#define _SERIAL_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

void Serial_Init(void);

/* TX */
void Serial_SendByte(uint8_t Byte);
void Serial_SendBuf(const uint8_t *buf, uint16_t len);

/* RX: 环形缓冲 + 中断 + OTA 请求识别 */
uint16_t Serial_Available(void);
uint8_t  Serial_ReadByte(void);

/* 中断服务例程 */
void Serial_RxIRQHandler(void);

#endif
