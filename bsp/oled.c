#include "oled.h"
#include "oled_data.h"
#include "stm32f10x.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

uint8_t OLED_DisplayBuf[OLED_PAGE_NUM][OLED_WIDTH];


/*================ I2C =================*/

static void oled_w_scl(uint8_t bit)
{
    GPIO_WriteBit(GPIOC,GPIO_Pin_8,(BitAction)bit);
}


static void oled_w_sda(uint8_t bit)
{
    GPIO_WriteBit(GPIOC,GPIO_Pin_9,(BitAction)bit);
}


static void oled_i2c_start(void)
{
    volatile uint32_t delay;
    oled_w_sda(1);
    for(delay=0;delay<10;delay++);
    oled_w_scl(1);
    for(delay=0;delay<10;delay++);
    oled_w_sda(0);
    for(delay=0;delay<10;delay++);
    oled_w_scl(0);
    for(delay=0;delay<10;delay++);
}


static void oled_i2c_stop(void)
{
    volatile uint32_t delay;
    oled_w_sda(0);
    for(delay=0;delay<10;delay++);
    oled_w_scl(1);
    for(delay=0;delay<10;delay++);
    oled_w_sda(1);
    for(delay=0;delay<10;delay++);
}


static void oled_i2c_send_byte(uint8_t byte)
{
    uint8_t i;

    for(i=0;i<8;i++)
    {
        oled_w_sda(byte&(0x80>>i));
        oled_w_scl(1);
        oled_w_scl(0);
    }

    oled_w_scl(1);
    oled_w_scl(0);
}


/*================ SSD1315 =================*/

void OLED_WriteCommand(uint8_t command)
{
    oled_i2c_start();

    oled_i2c_send_byte(0x78);
    oled_i2c_send_byte(0x00);
    oled_i2c_send_byte(command);

    oled_i2c_stop();
}


void OLED_WriteData(uint8_t *data,uint8_t count)
{
    uint8_t i;

    oled_i2c_start();

    oled_i2c_send_byte(0x78);
    oled_i2c_send_byte(0x40);

    for(i=0;i<count;i++)
        oled_i2c_send_byte(data[i]);

    oled_i2c_stop();
}



static void oled_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;


    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_GPIOC,
        ENABLE);


    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;


    GPIO_Init(GPIOC,&GPIO_InitStructure);


    oled_w_scl(1);
    oled_w_sda(1);
}



/*================ ��ʼ�� =================*/

void OLED_Init(void)
{
    oled_gpio_init();


    OLED_WriteCommand(0xAE);


    OLED_WriteCommand(0xD5);
    OLED_WriteCommand(0x80);


    OLED_WriteCommand(0xA8);
    OLED_WriteCommand(OLED_HEIGHT-1);


    OLED_WriteCommand(0xD3);
    OLED_WriteCommand(0x00);


    OLED_WriteCommand(0x40);


    OLED_WriteCommand(0x8D);
    OLED_WriteCommand(0x14);


    OLED_WriteCommand(0x20);
    OLED_WriteCommand(0x02);


    OLED_WriteCommand(0xA1);
    OLED_WriteCommand(0xC8);


    OLED_WriteCommand(0xDA);

#if OLED_HEIGHT==64
    OLED_WriteCommand(0x12);
#else
    OLED_WriteCommand(0x02);
#endif


    OLED_WriteCommand(0x81);
    OLED_WriteCommand(0xCF);


    OLED_WriteCommand(0xD9);
    OLED_WriteCommand(0xF1);


    OLED_WriteCommand(0xDB);
    OLED_WriteCommand(0x30);


    OLED_WriteCommand(0xA4);
    OLED_WriteCommand(0xA6);


    OLED_WriteCommand(0x2E);


    OLED_WriteCommand(0xAF);


    OLED_Clear();
    OLED_Update();
}

/*================ 亮度控制 =================*/

void OLED_SetContrast(uint8_t val)
{
    OLED_WriteCommand(0x81);
    OLED_WriteCommand(val);
}


/*================ Cursor =================*/

static void oled_set_cursor(uint8_t page,uint8_t x)
{
    OLED_WriteCommand(0xB0|page);
    OLED_WriteCommand(0x10|(x>>4));
    OLED_WriteCommand(0x00|(x&0x0F));
}


void OLED_Update(void)
{
    uint8_t page;

    for(page=0;page<OLED_PAGE_NUM;page++)
    {
        oled_set_cursor(page,0);
        OLED_WriteData(OLED_DisplayBuf[page],OLED_WIDTH);
    }
}


void OLED_UpdateArea(int16_t x,int16_t y,uint8_t width,uint8_t height)
{
    uint8_t page_start;
    uint8_t page_end;
    uint8_t page;


    if(width==0||height==0)
        return;


    if(x<0)
    {
        width+=x;
        x=0;
    }

    if(y<0)
    {
        height+=y;
        y=0;
    }


    if(x>=OLED_WIDTH||y>=OLED_HEIGHT)
        return;


    if(x+width>OLED_WIDTH)
        width=OLED_WIDTH-x;


    if(y+height>OLED_HEIGHT)
        height=OLED_HEIGHT-y;


    page_start=y>>3;
    page_end=(y+height-1)>>3;


    for(page=page_start;page<=page_end;page++)
    {
        oled_set_cursor(page,x);
        OLED_WriteData(&OLED_DisplayBuf[page][x],width);
    }
}



/*================ ��� =================*/

void OLED_Clear(void)
{
    uint16_t i,j;

    for(i=0;i<OLED_PAGE_NUM;i++)
    {
        for(j=0;j<OLED_WIDTH;j++)
            OLED_DisplayBuf[i][j]=0x00;
    }
}


void OLED_ClearArea(int16_t x,int16_t y,uint8_t width,uint8_t height)
{
    OLED_FillArea(x,y,width,height,OLED_COLOR_CLEAR);
}


void OLED_FillArea(int16_t x,int16_t y,uint8_t width,uint8_t height,uint8_t color)
{
    uint8_t i,j;


    for(j=0;j<height;j++)
    {
        for(i=0;i<width;i++)
            OLED_DrawPixel(x+i,y+j,color);
    }
}



/*================ λ�������� =================*/

/*
 * SSD1315ҳģʽ:
 *
 * page = y / 8
 *
 * bit  = y % 8
 *
 * һ��byte����8����������
 */
void OLED_DrawPixel(int16_t x,int16_t y,uint8_t color)
{
    uint8_t page;
    uint8_t bit;
    uint8_t mask;


    if(x<0||x>=OLED_WIDTH)
        return;


    if(y<0||y>=OLED_HEIGHT)
        return;


    page=y>>3;
    bit=y&0x07;
    mask=1<<bit;


    switch(color)
    {
    case OLED_COLOR_SET:

        OLED_DisplayBuf[page][x]|=mask;

        break;


    case OLED_COLOR_CLEAR:

        OLED_DisplayBuf[page][x]&=~mask;

        break;


    case OLED_COLOR_XOR:

        OLED_DisplayBuf[page][x]^=mask;

        break;


    default:

        break;
    }
}



void OLED_ReverseArea(int16_t x,int16_t y,uint8_t width,uint8_t height)
{
    uint8_t i,j;


    for(j=0;j<height;j++)
    {
        for(i=0;i<width;i++)
            OLED_DrawPixel(x+i,y+j,OLED_COLOR_XOR);
    }
}

/*================ ͼ�� =================*/

void OLED_DrawLine(int16_t x0,int16_t y0,int16_t x1,int16_t y1,uint8_t color)
{
    int16_t dx=abs(x1-x0);
    int16_t sx=x0<x1?1:-1;
    int16_t dy=-abs(y1-y0);
    int16_t sy=y0<y1?1:-1;
    int16_t err=dx+dy;
    int16_t e2;


    while(1)
    {
        OLED_DrawPixel(x0,y0,color);

        if(x0==x1&&y0==y1)
            break;


        e2=err<<1;


        if(e2>=dy)
        {
            err+=dy;
            x0+=sx;
        }


        if(e2<=dx)
        {
            err+=dx;
            y0+=sy;
        }
    }
}



void OLED_DrawRect(int16_t x,int16_t y,uint8_t width,uint8_t height,uint8_t color)
{
    if(width<2||height<2)
        return;


    OLED_DrawLine(x,y,x+width-1,y,color);

    OLED_DrawLine(
        x,
        y+height-1,
        x+width-1,
        y+height-1,
        color);


    OLED_DrawLine(
        x,
        y,
        x,
        y+height-1,
        color);


    OLED_DrawLine(
        x+width-1,
        y,
        x+width-1,
        y+height-1,
        color);
}



void OLED_FillRect(int16_t x,int16_t y,uint8_t width,uint8_t height,uint8_t color)
{
    uint8_t i;


    for(i=0;i<height;i++)
    {
        OLED_DrawLine(
            x,
            y+i,
            x+width-1,
            y+i,
            color);
    }
}



/*================ Բ�Ǿ��� =================*/

static void oled_draw_quarter_circle(int16_t cx,int16_t cy,uint8_t r,uint8_t corner,uint8_t color)
{
    int16_t x=0;
    int16_t y=r;
    int16_t d=3-2*r;


    while(y>=x)
    {
        switch(corner)
        {
        case 0:

            OLED_DrawPixel(cx-x,cy-y,color);
            OLED_DrawPixel(cx-y,cy-x,color);

            break;


        case 1:

            OLED_DrawPixel(cx+x,cy-y,color);
            OLED_DrawPixel(cx+y,cy-x,color);

            break;


        case 2:

            OLED_DrawPixel(cx-x,cy+y,color);
            OLED_DrawPixel(cx-y,cy+x,color);

            break;


        case 3:

            OLED_DrawPixel(cx+x,cy+y,color);
            OLED_DrawPixel(cx+y,cy+x,color);

            break;
        }


        if(d<0)
        {
            d+=4*x+6;
        }
        else
        {
            d+=4*(x-y)+10;
            y--;
        }

        x++;
    }
}



void OLED_DrawRoundRect(int16_t x,int16_t y,uint8_t width,uint8_t height,uint8_t radius,uint8_t color)
{
    if(radius==0)
    {
        OLED_DrawRect(x,y,width,height,color);
        return;
    }


    if(radius*2>width)
        radius=width/2;


    if(radius*2>height)
        radius=height/2;



    OLED_DrawLine(
        x+radius,
        y,
        x+width-radius-1,
        y,
        color);


    OLED_DrawLine(
        x+radius,
        y+height-1,
        x+width-radius-1,
        y+height-1,
        color);



    OLED_DrawLine(
        x,
        y+radius,
        x,
        y+height-radius-1,
        color);


    OLED_DrawLine(
        x+width-1,
        y+radius,
        x+width-1,
        y+height-radius-1,
        color);



    oled_draw_quarter_circle(
        x+radius,
        y+radius,
        radius,
        0,
        color);


    oled_draw_quarter_circle(
        x+width-radius-1,
        y+radius,
        radius,
        1,
        color);


    oled_draw_quarter_circle(
        x+radius,
        y+height-radius-1,
        radius,
        2,
        color);


    oled_draw_quarter_circle(
        x+width-radius-1,
        y+height-radius-1,
        radius,
        3,
        color);
}

void OLED_FillRoundRect(int16_t x,int16_t y,
                        uint8_t width,uint8_t height,
                        uint8_t radius,
                        uint8_t color)
{
    int16_t yy;

    if(width==0 || height==0)
        return;

    if(radius==0)
    {
        OLED_FillRect(x,y,width,height,color);
        return;
    }

    if(radius>width/2)
        radius=width/2;

    if(radius>height/2)
        radius=height/2;

    for(yy=0;yy<height;yy++)
    {
        int16_t left=x;
        int16_t right=x+width-1;

        if(yy<radius)
        {
            /* ���ϡ�����Բ�� */
            int16_t dy=radius-1-yy;
            int16_t dx=0;

            while((dx+1)*(dx+1)+dy*dy<=radius*radius)
                dx++;

            left=x+radius-dx;
            right=x+width-radius+dx-1;
        }
        else if(yy>=height-radius)
        {
            /* ���¡�����Բ�� */
            int16_t dy=yy-(height-radius);
            int16_t dx=0;

            while((dx+1)*(dx+1)+dy*dy<=radius*radius)
                dx++;

            left=x+radius-dx;
            right=x+width-radius+dx-1;
        }

        OLED_DrawLine(left,
                      y+yy,
                      right,
                      y+yy,
                      color);
    }
}

void OLED_ReverseRoundRect(int16_t x,int16_t y,
                           uint8_t width,uint8_t height,
                           uint8_t radius)
{
    OLED_FillRoundRect(x,
                       y,
                       width,
                       height,
                       radius,
                       OLED_COLOR_XOR);
}

/*================ ͼƬ =================*/

void OLED_ShowImage(int16_t x,int16_t y,uint8_t width,uint8_t height,const uint8_t *image)
{
    uint8_t row;
    uint8_t col;
    uint8_t data;


    for(row=0;row<height;row++)
    {
        for(col=0;col<width;col++)
        {
            data=image[(row>>3)*width+col];

            if(data&(1<<(row&0x07)))
                OLED_DrawPixel(x+col,y+row,OLED_COLOR_SET);
            else
                OLED_DrawPixel(x+col,y+row,OLED_COLOR_CLEAR);
        }
    }
}



/*================ �ַ� =================*/

void OLED_ShowChar(int16_t x,int16_t y,char ch,uint8_t font)
{
    /* 越界字符(控制符/非ASCII)替换为 '?', 防止字模数组越界读 */
    if(ch<' '||ch>'~')
        ch='?';

    if(font==OLED_8X16)
    {
        OLED_ShowImage(
            x,
            y,
            8,
            16,
            OLED_F8x16[ch-' ']);
    }
    else if(font==OLED_6X8)
    {
        OLED_ShowImage(
            x,
            y,
            6,
            8,
            OLED_F6x8[ch-' ']);
    }
}



void OLED_ShowString(int16_t x,int16_t y,const char *str,uint8_t font)
{
    while(*str)
    {
        OLED_ShowChar(x,y,*str,font);

        x+=8;
        str++;
    }
}


/* 画一个 8x16 内的小号圆环 "°" 符号, 位于字形上部 */
void OLED_ShowDegree(int16_t x,int16_t y)
{
    int16_t cx = x + 4;   /* 圆环中心 */
    int16_t cy = y + 2;
    int16_t py, px;

    for(py = cy - 2; py <= cy + 2; py++)
    {
        for(px = cx - 2; px <= cx + 2; px++)
        {
            int16_t dx = px - cx;
            int16_t dy = py - cy;
            int32_t d2 = dx*dx + dy*dy;

            /* 半径 1~2 的空心圆环 (1px 厚) */
            if(d2 >= 1 && d2 <= 4)
                OLED_DrawPixel(px, py, OLED_COLOR_SET);
        }
    }
}



/*================ ���� =================*/

static uint32_t oled_pow10(uint8_t n)
{
    uint32_t value=1;


    while(n--)
        value*=10;


    return value;
}



void OLED_ShowNum(int16_t x,int16_t y,int32_t num,uint8_t len)
{
    uint8_t i;
    uint8_t digit;


    if(num<0)
    {
        OLED_ShowChar(x,y,'-',OLED_8X16);

        x+=8;
        num=-num;
    }


    for(i=0;i<len;i++)
    {
        digit=(num/oled_pow10(len-i-1))%10;


        OLED_ShowChar(
            x+i*8,
            y,
            '0'+digit,
            OLED_8X16);
    }
}



/*================ printf =================*/

void OLED_Print(int16_t x,int16_t y,const char *format,...)
{
    char buf[32];

    va_list args;


    va_start(args,format);

    vsnprintf(buf,sizeof(buf),format,args);

    va_end(args);


    OLED_ShowString(
        x,
        y,
        buf,
        OLED_8X16);
}



/*================ 16x32 =================*/

/*
 * 字模取模方式：列行式、从上到下、从左到右、高位在前
 * 即每个字节 bit7 表示该列最上方像素、bit0 表示最下方像素。
 * 显示缓冲区约定 bit0 为页内最上方像素，故逐字节位反转后再写入。
 */
uint8_t OLED_Rev8(uint8_t b)
{
    b = (uint8_t)((b & 0xF0u) >> 4) | (uint8_t)((b & 0x0Fu) << 4);
    b = (uint8_t)((b & 0xCCu) >> 2) | (uint8_t)((b & 0x33u) << 2);
    b = (uint8_t)((b & 0xAAu) >> 1) | (uint8_t)((b & 0x55u) << 1);
    return b;
}

void OLED_ShowLargeChar(int16_t x,int16_t y,char ch)
{
    uint8_t idx;
    uint8_t page;
    uint8_t col;
    uint8_t data;
    uint8_t bit_offset = (uint8_t)y & 0x07;
    uint8_t page_base;
    uint8_t cols;

    /* 边界钳位: 越界坐标直接返回, 防止 OLED_DisplayBuf 越界写 */
    if(x<0 || x>=OLED_WIDTH || y<0 || y>=OLED_HEIGHT)
        return;

    if(ch>='0'&&ch<='9')
        idx=ch-'0';
    else if(ch=='-')
        idx=11;
    else
        return;

    page_base = (uint8_t)(y >> 3);
    cols = (x + 16 > OLED_WIDTH) ? (uint8_t)(OLED_WIDTH - x) : 16;

    /* 高位在前 -> 缓冲区低位在上的位反转 */
    for(page=0;page<4;page++)
    {
        if(page_base + page >= OLED_PAGE_NUM)
            break;
        for(col=0;col<cols;col++)
        {
            data=OLED_Rev8(OLED_F16x32[idx][page*16+col]);
            if(bit_offset == 0)
            {
                if(data)
                    OLED_DisplayBuf[page_base + page][x+col]|=data;
            }
            else
            {
                /* 非页对齐：低位部分写到当前页，高位部分溢出到下一页 */
                uint8_t lo_mask = (uint8_t)((1u << (8u - bit_offset)) - 1u);
                uint8_t hi_shift = 8u - bit_offset;
                if(data)
                {
                    OLED_DisplayBuf[page_base + page][x+col]|=(data & lo_mask)<<bit_offset;
                    if(page_base + page + 1 < OLED_PAGE_NUM)
                        OLED_DisplayBuf[page_base + page + 1][x+col]|=data>>hi_shift;
                }
            }
        }
    }
}


void OLED_ShowLargeNum(int16_t x,int16_t y,int32_t num)
{
    uint8_t buf[10];    /* int32 最多 10 位, 防止位数较多时越界写 */
    uint8_t len=0;
    uint8_t i;
    int16_t pos;
    uint8_t neg=0;


    if(num<0)
    {
        neg=1;
        num=-num;
    }


    do
    {
        buf[len++]=num%10;
        num/=10;
    }
    while(num>0);


    pos=x+16*(len+neg)-16;


    if(neg)
    {
        OLED_ShowLargeChar(pos,y,'-');
        pos-=16;
    }


    for(i=0;i<len;i++)
    {
        OLED_ShowLargeChar(pos,y,'0'+buf[len-1-i]);
        pos-=16;
    }
}



/*================ 中文 =================*/

/* 返回当前 UTF-8 字符的字节长度 (ASCII=1, 中文=3) */
static uint8_t oled_utf8_len(const char *str)
{
    uint8_t b = (uint8_t)str[0];

    if (b < 0x80)                 return 1;
    else if ((b & 0xE0) == 0xC0)  return 2;
    else if ((b & 0xF0) == 0xE0)  return 3;   /* CJK 中文 */
    else if ((b & 0xF8) == 0xF0)  return 4;
    return 1;
}

void OLED_ShowChinese(int16_t x, int16_t y, const char *str)
{
    uint8_t i, k, page, col;
    uint8_t data;
    uint16_t cnt;
    uint8_t bit_offset;
    uint8_t page_base;
    uint8_t cols;
    uint8_t n;

    if (!str || !*str)
        return;

    /* 边界钳位: 越界坐标直接返回, 防止 OLED_DisplayBuf 越界写 */
    if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT)
        return;

    cnt = OLED_CF16x16_COUNT;
    bit_offset = (uint8_t)y & 0x07;
    page_base = (uint8_t)(y >> 3);
    cols = (x + 16 > OLED_WIDTH) ? (uint8_t)(OLED_WIDTH - x) : 16;

    while (*str)
    {
        n = oled_utf8_len(str);

        if (n == 1)
        {
            /* ASCII 字符: 用 8x16 显示, 字符宽 8 */
            OLED_ShowChar(x, y, *str, OLED_8X16);
            x += 8;
            str += 1;
            continue;
        }

        /* 多字节 (中文): 按完整 UTF-8 码点在字库中查找 */
        for (i = 0; i < cnt; i++)
        {
            for (k = 0; k < n; k++)
            {
                if (OLED_CF16x16[i].Index[k] != str[k])
                    break;
            }
            if (k == n)
            {
                if (bit_offset == 0)
                {
                    /* 页对齐：直接写入 */
                    for (page = 0; page < 2; page++)
                    {
                        if (page_base + page >= OLED_PAGE_NUM)
                            break;
                        for (col = 0; col < cols; col++)
                        {
                            data = OLED_Rev8(OLED_CF16x16[i].Data[page * 16 + col]);
                            if (data)
                                OLED_DisplayBuf[page_base + page][x + col] |= data;
                        }
                    }
                }
                else
                {
                    /* 非页对齐：拆分写入跨页 */
                    uint8_t lo_mask = (uint8_t)((1u << (8u - bit_offset)) - 1u);
                    uint8_t hi_shift = 8u - bit_offset;

                    for (page = 0; page < 2; page++)
                    {
                        if (page_base + page >= OLED_PAGE_NUM)
                            break;
                        for (col = 0; col < cols; col++)
                        {
                            data = OLED_Rev8(OLED_CF16x16[i].Data[page * 16 + col]);
                            if (data)
                            {
                                /* 低位部分写到当前页的偏移位 */
                                OLED_DisplayBuf[page_base + page][x + col] |= (data & lo_mask) << bit_offset;
                                /* 高位部分溢出到下一页 */
                                if (page_base + page + 1 < OLED_PAGE_NUM)
                                    OLED_DisplayBuf[page_base + page + 1][x + col] |= data >> hi_shift;
                            }
                        }
                    }
                }
                break;
            }
        }
        x += 16;
        str += n;
    }
}


/*================ =================*/

void OLED_DrawProgressBar(int16_t x,int16_t y,
                          uint8_t width,uint8_t height,
                          uint8_t percent)
{
    uint8_t fill;
    uint8_t i;


    if(width<4||height<4)
        return;


    if(percent>100)
        percent=100;


    OLED_DrawRect(x,y,width,height,OLED_COLOR_SET);


    fill=(uint8_t)((uint32_t)(width-2)*percent/100);


    for(i=0;i<height-2;i++)
    {
        OLED_DrawLine(
            x+1,
            y+1+i,
            x+fill,
            y+1+i,
            OLED_COLOR_SET);
    }
}

