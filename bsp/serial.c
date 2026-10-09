#include "stm32f10x.h"                  // Device header
#include "serial.h"
#include "misc.h"

#define UART_BAUDRATE       115200U   /* OTA 通信固定波特率 */

/* 接收回调(注册者: ota_boot). volatile: 主循环注册, 中断随时读取.
 *
 * 本驱动只做"把每个收到的字节派发给注册者"——不维护接收缓冲、不识别协议.
 * 历史: 曾内置 512B 环形缓冲(rx_buf/head/tail)与 Serial_Available/ReadByte、
 * 以及 Serial_SendByte/SendBuf, 但全工程零调用(OTA 只需识别 3 字节请求序列,
 * 走本回调即可). 该缓冲只写不读: map 实证 rx_buf 常驻 .bss 512 字节
 * (占 ZI 约 16%), 每收一字节还白做一次入环/判满. 已整段移除, 接收路径
 * 收敛为"ISR 直接派发回调". 若日后需要通用 TX/RX 接口, 再按需加回. */
static volatile serial_rx_fn rx_cb = NULL;

void Serial_SetRxCallback(serial_rx_fn fn)
{
    rx_cb = fn;
}

void Serial_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = UART_BAUDRATE;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1, &USART_InitStructure);

    /* NVIC USART1 */
    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel                   = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

void Serial_RxIRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t b = (uint8_t)USART_ReceiveData(USART1);
        /* 直接派发给注册者(OTA 请求序列识别). 经回调而非直接调用:
         * 本驱动不认识 OTA 协议, 也不依赖 ota/ 的头文件. */
        if (rx_cb)
            rx_cb(b);
    }
}
