#ifndef __OLED_H
#define __OLED_H

#include <stdint.h>
#include "oled_data.h"


/* OLED�ֱ���
 * �޸�OLED_HEIGHT����֧��128x64 / 128x32
 */
#define OLED_WIDTH          128U
#define OLED_HEIGHT         64U

/* SSD1315ҳ�߶� */
#define OLED_PAGE_HEIGHT    8U

/* ҳ���� */
#define OLED_PAGE_NUM       (OLED_HEIGHT/OLED_PAGE_HEIGHT)


/* ��ͼ��ɫ */
#define OLED_COLOR_CLEAR    0U
#define OLED_COLOR_SET      1U
#define OLED_COLOR_XOR      2U


/* ����ߴ� */
#define OLED_8X16            16U
#define OLED_6X8             6U


/* OLED�Դ�
 *
 * SSD1306/SSD1315ҳģʽ:
 *
 * һ��byte��������8������
 *
 * OLED_DisplayBuf[page][x]
 *
 */
extern uint8_t OLED_DisplayBuf[OLED_PAGE_NUM][OLED_WIDTH];



/*================ ��ʼ�� =================*/

void OLED_Init(void);



/*================ ˢ�� =================*/

void OLED_Update(void);

void OLED_UpdateArea(int16_t x,int16_t y,
                     uint8_t width,uint8_t height);



/*================ ���� =================*/

void OLED_Clear(void);

void OLED_ClearArea(int16_t x,int16_t y,
                    uint8_t width,uint8_t height);


void OLED_FillArea(int16_t x,int16_t y,
                   uint8_t width,uint8_t height,
                   uint8_t color);



/*================ ���� =================*/

/*
 * ����ͼ�κ������յ��ô˽ӿ�
 *
 * page = y >> 3
 * bit  = y & 0x07
 */
void OLED_DrawPixel(int16_t x,int16_t y,
                    uint8_t color);



/*================ ͼ�� =================*/

void OLED_DrawLine(int16_t x0,int16_t y0,
                   int16_t x1,int16_t y1,
                   uint8_t color);


void OLED_DrawRect(int16_t x,int16_t y,
                   uint8_t width,uint8_t height,
                   uint8_t color);


void OLED_FillRect(int16_t x,int16_t y,
                   uint8_t width,uint8_t height,
                   uint8_t color);


void OLED_DrawRoundRect(int16_t x,int16_t y,
                        uint8_t width,uint8_t height,
                        uint8_t radius,
                        uint8_t color);



/*================ ��Ч =================*/

void OLED_ReverseArea(int16_t x,int16_t y,
                      uint8_t width,uint8_t height);

void OLED_FillRoundRect(int16_t x,int16_t y,
                        uint8_t width,uint8_t height,
                        uint8_t radius,
                        uint8_t color);

void OLED_ReverseRoundRect(int16_t x,int16_t y,
                           uint8_t width,uint8_t height,
                           uint8_t radius);


/*================ ͼƬ =================*/

void OLED_ShowImage(int16_t x,int16_t y,
                    uint8_t width,uint8_t height,
                    const uint8_t *image);



/*================ =================*/

void OLED_ShowChar(int16_t x,int16_t y,
                   char ch,uint8_t font);


void OLED_ShowString(int16_t x,int16_t y,
                     const char *str,
                     uint8_t font);

void OLED_ShowDegree(int16_t x,int16_t y);


void OLED_ShowNum(int16_t x,int16_t y,
                  int32_t num,
                  uint8_t len);


void OLED_Print(int16_t x,int16_t y,
                const char *format,...);


/*================ 16x32 =================*/

uint8_t OLED_Rev8(uint8_t b);

void OLED_ShowLargeChar(int16_t x,int16_t y,char ch);

void OLED_ShowLargeNum(int16_t x,int16_t y,int32_t num);


/*================ =================*/

void OLED_DrawProgressBar(int16_t x,int16_t y,
                          uint8_t width,uint8_t height,
                          uint8_t percent);


/*================ 中文 =================*/

void OLED_ShowChinese(int16_t x, int16_t y, const char *str);


/*================ 亮度控制 =================*/

void OLED_SetContrast(uint8_t val);


#endif
