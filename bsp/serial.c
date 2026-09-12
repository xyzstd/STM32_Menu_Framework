#include "stm32f10x.h"                  // Device header
#include "serial.h"
#include "ota_boot.h"
#include "misc.h"

#define SERIAL_RX_BUF_SIZE  512U
#define UART_BAUDRATE       115200U   /* OTA 通信固定波特率 */
static volatile uint8_t  rx_buf[SERIAL_RX_BUF_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;

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

    rx_head = rx_tail = 0;
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

void Serial_SendByte(uint8_t Byte)
{
    USART_SendData(USART1, Byte);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
}

void Serial_SendBuf(const uint8_t *buf, uint16_t len)
{
    while (len--) Serial_SendByte(*buf++);
}

uint16_t Serial_Available(void)
{
    return (uint16_t)((rx_head - rx_tail + SERIAL_RX_BUF_SIZE) % SERIAL_RX_BUF_SIZE);
}

uint8_t Serial_ReadByte(void)
{
    uint8_t b = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1) % SERIAL_RX_BUF_SIZE;
    return b;
}

void Serial_RxIRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t b = (uint8_t)USART_ReceiveData(USART1);
        uint16_t next = (rx_head + 1) % SERIAL_RX_BUF_SIZE;
        if (next != rx_tail) {
            rx_buf[rx_head] = b;
            rx_head = next;
        }
        /* 同时喂进 OTA 请求状态机 */
        OTA_SerialFeedRx(b);
    }
}

int fputc(int ch, FILE *f)
{
    Serial_SendByte((uint8_t)ch);
    return ch;
}
