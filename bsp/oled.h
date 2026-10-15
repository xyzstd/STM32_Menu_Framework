#ifndef __OLED_H
#define __OLED_H

#include <stdint.h>
#include "oled_data.h"


/* OLED 分辨率
 * 修改 OLED_HEIGHT 即可支持 128x64 / 128x32
 */
#define OLED_WIDTH          128U
#define OLED_HEIGHT         64U

/* SSD1315 页高度 */
#define OLED_PAGE_HEIGHT    8U

/* 页数量 */
#define OLED_PAGE_NUM       (OLED_HEIGHT/OLED_PAGE_HEIGHT)


/* 绘图颜色 */
#define OLED_COLOR_CLEAR    0U
#define OLED_COLOR_SET      1U
#define OLED_COLOR_XOR      2U


/* 字体尺寸 */
#define OLED_8X16            16U
#define OLED_6X8             6U


/* OLED 显存
 *
 * SSD1306/SSD1315 页模式:
 *
 * 一个 byte 代表纵向 8 个像素
 *
 * OLED_DisplayBuf[page][x]
 *
 */
extern uint8_t OLED_DisplayBuf[OLED_PAGE_NUM][OLED_WIDTH];



/*================ 初始化 =================*/

void OLED_Init(void);



/*================ 刷新 =================*/

void OLED_Update(void);

/* 页级差量标记: 比较指纹, 把内容变化的页记入脏页位图(不推送 I2C) */
void OLED_UpdateDiff(void);

/* 主循环每轮调用: 推送一页待推脏页(分帧推送, 单次阻塞 ≤1.5ms, 不卡输入) */
void OLED_FlushStep(void);



/*================ 清屏 =================*/

void OLED_Clear(void);

void OLED_ClearArea(int16_t x,int16_t y,
                    uint8_t width,uint8_t height);


void OLED_FillArea(int16_t x,int16_t y,
                   uint8_t width,uint8_t height,
                   uint8_t color);



/*================ 像素操作 =================*/

/*
 * 所有图形函数最终调用此接口
 *
 * page = y >> 3
 * bit  = y & 0x07
 */
void OLED_DrawPixel(int16_t x,int16_t y,
                    uint8_t color);



/*================ 图形 =================*/

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



/*================ 特效 =================*/

void OLED_ReverseArea(int16_t x,int16_t y,
                      uint8_t width,uint8_t height);

void OLED_FillRoundRect(int16_t x,int16_t y,
                        uint8_t width,uint8_t height,
                        uint8_t radius,
                        uint8_t color);

void OLED_ReverseRoundRect(int16_t x,int16_t y,
                           uint8_t width,uint8_t height,
                           uint8_t radius);


/*================ 图片 =================*/

void OLED_ShowImage(int16_t x,int16_t y,
                    uint8_t width,uint8_t height,
                    const uint8_t *image);



/*================ 字符 =================*/

void OLED_ShowChar(int16_t x,int16_t y,
                   char ch,uint8_t font);


void OLED_ShowString(int16_t x,int16_t y,
                     const char *str,
                     uint8_t font);

void OLED_ShowDegree(int16_t x,int16_t y);


void OLED_ShowNum(int16_t x,int16_t y,
                  int32_t num,
                  uint8_t len);


/*================ 16x32 =================*/

uint8_t OLED_Rev8(uint8_t b);

void OLED_ShowLargeChar(int16_t x,int16_t y,char ch);

void OLED_ShowLargeNum(int16_t x,int16_t y,int32_t num);

/* 大字号 "°C" (16x32), 紧随 OLED_ShowLargeNum 绘出的数字右侧 (同一 y)。
 * 字形取自 OLED_F16x32 表的第 10 项; 原由 page/home.c 直接写 OLED_DisplayBuf
 * 绘制(绕过本层封装), 现收进本层, 供 HOME 与"温度设置"页共用。 */
void OLED_ShowLargeDegC(int16_t x,int16_t y);


/*================ 进度条 =================*/

void OLED_DrawProgressBar(int16_t x,int16_t y,
                          uint8_t width,uint8_t height,
                          uint8_t percent);


/*================ 中文 =================*/

void OLED_ShowChinese(int16_t x, int16_t y, const char *str);

/* Concatenate two literal string segments into dst (NUL-terminated, truncated to
 * cap). Avoids snprintf/stdio for display text such as "切换到" + "T245?"; the
 * caller keeps its own centring math. */
void oled_pair_cat(char *dst, uint8_t cap, const char *head, const char *tail);


/*================ 亮度控制 =================*/

void OLED_SetContrast(uint8_t val);

#endif
