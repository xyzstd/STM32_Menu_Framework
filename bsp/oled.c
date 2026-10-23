#include "oled.h"
#include "oled_data.h"
#include "stm32f10x.h"
#include <stddef.h>   /* NULL */
#include <stdbool.h>
#include <stdlib.h>   /* abs */
#include <string.h>   /* memset (OLED_Clear 整屏清 0) */
#include "timer.h"    /* Timer_GetTick (兜底整屏重推计时) */

/* 不再包含 <stdio.h>/<stdarg.h>: 原 OLED_Print 走 vsnprintf, 是全工程最后一处
 * 把 stdio 格式化机制拉进链接的引用(连带浮点支持, 实测约 2.3KB).
 * 文案拼接改用 oled_pair_cat, 数值显示改用 OLED_ShowNum. */

uint8_t OLED_DisplayBuf[OLED_PAGE_NUM][OLED_WIDTH];

/* 差量刷屏快照: 记录最近一次推送到屏幕的每页 32bit 指纹, 供 OLED_UpdateDiff 判脏.
 * 以 8×4B 指纹取代整帧副本(省约 1KB RAM); 全推/局部推也同步本快照,
 * 保证混合调用一致不错漏. */
static uint32_t oled_snap_sum[OLED_PAGE_NUM];

/* 待推送脏页位图(bit n = 第 n 页缓冲已变化、等待推送) + 轮转起点.
 * OLED_UpdateDiff 只做"比较指纹 + 置位", 真正的 I2C 推送由主循环每轮
 * OLED_FlushStep 推一页完成 —— 把"一次整屏 8 页 ≈25ms 的同步阻塞"拆成
 * "每轮 1~2 页(单次 ≤6.4ms)", 让 Input_Process 在两次推送之间得到执行.
 * 否则整屏推送期间主循环停摆, 每轮只能消费 2 个输入事件(当时的上限), 消费速率掉到
 * ~140/s(低于编码器 250/s) → 队列积压 → 表现为"转了不动、过一会才恢复". */
static uint8_t oled_dirty = 0;
static uint8_t oled_flush_next = 0;

/* 兜底整屏重推周期 (ms). 即便 ACK 可检测瞬时丢写, 周期全推仍可覆盖上电瞬间
 * 的总线异常/控制器复位等状态失步. 内容不变则视觉无差异(不闪), 代价约
 * 8 页×3.2ms / 10s ≈ 0.26% 的 I2C 占用(每 bit 2.7µs = 1.2 高 + 1.5 低, 128B/页). */
#define OLED_FULL_REFRESH_MS  10000u
#define OLED_WRITE_RETRY_MS   50u     /* NACK 后退避, OLED 缺失时不在主循环每轮忙重试 */

/* 触发全局退避所需的"连续 NACK"次数.
 * 退避本意是防"屏幕缺失/总线挂了"时主循环每轮白耗 1.7ms 重试; 但若一有 NACK
 * 就全局退避 50ms, 一次瞬时干扰(软 I2C 总线受加热 PWM 串扰等)就会让**整屏**
 * 50ms 不更新 —— 表现为"画面突然卡一下". 故改为连续失败达阈值才退避: 单次瞬时
 * NACK 只丢掉那一页(下轮继续推其它页, 该页留脏稍后重试), 不影响整体刷新节奏. */
#define OLED_NACK_BACKOFF_AFTER  3u
static uint8_t oled_nack_streak = 0;

/* 软 I2C 半周期延时(volatile 空循环次数). 时序经反汇编核对(Release -O3):
 * 一次 volatile 空循环迭代 ≈ 11 周期 ≈ 0.15µs.
 * 原实现**只在 SCL 高电平段加延时**(12 次 ≈ 1.8µs), 低电平段直接进下一位的 SDA
 * 设置 —— 实测 tLOW ≈ 0.3µs, 远低于 I2C 规范最小 1.3µs(400kHz 快速模式), 等效时钟
 * 约 476kHz 也已超规范. 这是"偶发 NACK"的根因: 总线一旦受加热 PWM 串扰或温漂,
 * 从机就可能采样失败 → 丢写 → 该页不刷新(旧版无 ACK 检测时表现为像素残影).
 * 现高低电平各留合规宽度, 每位总时长仅增约 1/3, 分帧刷屏完全吸收. */
#define I2C_DLY_HIGH  8u    /* SCL 高 ≈1.2µs (规范 ≥0.6µs) */
#define I2C_DLY_LOW   10u   /* SCL 低 ≈1.5µs (规范 ≥1.3µs) */

static uint32_t oled_full_tick = 0;
static uint32_t oled_retry_tick = 0;
static bool oled_retry_wait = false;

static uint32_t oled_page_sum(uint8_t page);
static void oled_snap_sync(uint8_t page);

static void oled_w_scl(uint8_t bit);
static void oled_w_sda(uint8_t bit);
static void oled_i2c_start(void);
static void oled_i2c_stop(void);
static bool oled_i2c_send_byte(uint8_t byte);

static void oled_gpio_init(void);

static bool oled_set_cursor(uint8_t page,uint8_t x);

static void oled_draw_quarter_circle(int16_t cx,int16_t cy,uint8_t r,uint8_t corner,uint8_t color);
static uint32_t oled_pow10(uint8_t n);
static uint8_t oled_utf8_len(const char *str);

/* 页指纹: 必须是"位置敏感"的 —— 内容只要有一个字节变化就必须改变指纹.
 * 旧实现用逐字节无符号累加, 加法与字节顺序无关, 于是同页内字节多重集相同、
 * 仅位置不同的改动(字形水平移 1 像素、menu_reverse_row 交换两列等)指纹不变,
 * 该页会被判为"未变化"而永不推送, 表现为像素残影到下次整页刷新.
 * 曾改用"左旋 5 位 + 加常数": 位置敏感, 但左旋 32 次即恒等(5×32 ≡ 0 mod 32),
 * 故下标相差 32 整数倍的字节旋转权重相同, 交换二者仅靠进位区分 —— 实测相距
 * 32/64 列的字节交换仍有约 1/180 的碰撞率, 远高于 2^-32 的可忽略量级.
 * 改用 FNV-1a(异或后乘质数): 乘法把每个字节的贡献按位置彻底打散, 不存在 mod-32
 * 等价类; 单字节改动与任意换位都会改变指纹.
 * 成本: 每字节一次异或 + 一次乘法(32bit MUL, M3 单周期), 每帧最多 8×128 字节, 可忽略. */
static uint32_t oled_page_sum(uint8_t page)
{
    uint32_t s = 0x811C9DC5u;   /* FNV-1a 32bit 偏移基准 */
    uint8_t c;
    for(c=0;c<OLED_WIDTH;c++)
    {
        s ^= OLED_DisplayBuf[page][c];
        s *= 0x01000193u;       /* FNV-1a 质数: 逐字节按位置混合 */
    }
    return s;
}

static void oled_snap_sync(uint8_t page)
{
    oled_snap_sum[page] = oled_page_sum(page);
}


/*================ I2C =================*/

static void oled_w_scl(uint8_t bit)
{
    if (bit) GPIOC->BSRR = GPIO_Pin_8;
    else     GPIOC->BRR  = GPIO_Pin_8;
}


static void oled_w_sda(uint8_t bit)
{
    if (bit) GPIOC->BSRR = GPIO_Pin_9;
    else     GPIOC->BRR  = GPIO_Pin_9;
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


static bool oled_i2c_send_byte(uint8_t byte)
{
    uint8_t i;
    bool ack;
    volatile uint32_t delay;

    for(i=0;i<8;i++)
    {
        oled_w_sda(byte&(0x80>>i));
        oled_w_scl(1);
        for(delay=0;delay<I2C_DLY_HIGH;delay++);   /* 保持 SCL 高电平宽度 */
        oled_w_scl(0);
        for(delay=0;delay<I2C_DLY_LOW;delay++);    /* 补 SCL 低电平宽度(原缺失) */
    }

    /* 第9个时钟采 ACK: 开漏输出写 1 即释放 SDA, 从机应拉低应答.
     * 旧实现固定发时钟但不采样, 所以即使 NACK/屏幕掉线也被当作写成功. */
    oled_w_sda(1);
    for(delay=0;delay<4;delay++);
    oled_w_scl(1);
    for(delay=0;delay<I2C_DLY_HIGH;delay++);
    ack = (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9) == Bit_RESET);
    oled_w_scl(0);
    for(delay=0;delay<I2C_DLY_LOW;delay++);   /* 字节之间同样保证 tLOW */
    return ack;
}


/*================ SSD1315 =================*/

bool OLED_WriteCommand(uint8_t command)
{
    bool ack;
    oled_i2c_start();

    ack = oled_i2c_send_byte(0x78);
    if (ack) ack = oled_i2c_send_byte(0x00);
    if (ack) ack = oled_i2c_send_byte(command);

    oled_i2c_stop();
    return ack;
}


bool OLED_WriteData(uint8_t *data,uint8_t count)
{
    uint8_t i;
    bool ack;

    oled_i2c_start();

    ack = oled_i2c_send_byte(0x78);
    if (ack) ack = oled_i2c_send_byte(0x40);

    for(i=0; ack && i<count; i++)
        ack = oled_i2c_send_byte(data[i]);

    oled_i2c_stop();
    return ack;
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



/*================ 初始化 =================*/

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

static bool oled_set_cursor(uint8_t page,uint8_t x)
{
    if (!OLED_WriteCommand((uint8_t)(0xB0 | page)))
        return false;
    if (!OLED_WriteCommand((uint8_t)(0x10 | (x >> 4))))
        return false;
    return OLED_WriteCommand((uint8_t)(0x00 | (x & 0x0F)));
}


void OLED_Update(void)
{
    uint8_t page;

    for(page=0;page<OLED_PAGE_NUM;page++)
    {
        uint8_t mask = (uint8_t)(1u << page);
        if (oled_set_cursor(page, 0) &&
            OLED_WriteData(OLED_DisplayBuf[page], OLED_WIDTH))
        {
            oled_snap_sync(page);
            oled_dirty &= (uint8_t)~mask;
        }
        else
        {
            /* NACK: 快照不得前移, 保留脏位交给主循环重试. */
            oled_dirty |= mask;
        }
    }

    oled_full_tick = Timer_GetTick();   /* 兜底计时一并重启 */
}

/* 页级差量"标记": 按页指纹与快照比对, 把内容变化的页记入脏页位图.
 * 菜单/参数页每帧往往只改少数几页(高亮行、数值、功率条), 其余页一致则跳过;
 * 静止时逐页指纹全等 -> 无脏页 -> 零推送零闪烁. 指纹(32bit FNV-1a)代替整帧
 * 比较, 节省 ~1KB RAM.
 * 注意: 本函数**不再直接推送 I2C** —— 真正推送由主循环 OLED_FlushStep 每轮一页
 * 完成(见下). 调用方语义不变: 仍在"渲染完缓冲"后调用. */
void OLED_UpdateDiff(void)
{
    uint8_t  page;
    uint32_t now = Timer_GetTick();

    /* 周期兜底: 到期则整屏标脏(理由见 OLED_FULL_REFRESH_MS 说明).
     * 与下方指纹比较是"或"关系 —— 兜底只加推送, 不影响原有差量语义. */
    if ((uint32_t)(now - oled_full_tick) >= OLED_FULL_REFRESH_MS)
    {
        oled_full_tick = now;
        oled_dirty |= (uint8_t)((1u << OLED_PAGE_NUM) - 1u);
    }

    for(page=0;page<OLED_PAGE_NUM;page++)
    {
        if (oled_page_sum(page) != oled_snap_sum[page])
            oled_dirty |= (uint8_t)(1u << page);
    }
}


/* 脏页批量推送阈值: 脏页数 >= OLED_FLUSH_BATCH_MIN 时本轮推 2 页.
 * 用于上电首帧/列表切换这类整屏重绘 —— 单次阻塞 2x3.2ms 仍远小于 12ms 事件
 * 年龄阈值, 而整屏 8 页的完成时间从 26ms 减半到 13ms. 常规翻一格只脏 4 页,
 * 按 1 页推, 保持最小阻塞. */
#define OLED_FLUSH_BATCH_MIN   4u
#define OLED_FLUSH_BATCH       2u

static uint8_t oled_dirty_count(void)
{
    uint8_t v = oled_dirty;
    uint8_t n = 0;

    while (v)
    {
        n = (uint8_t)(n + (v & 1u));
        v >>= 1;
    }

    return n;
}

/* 主循环每轮调用: 若仍有待推页, 自轮转起点向后找一页推送.
 * 每轮推 1~2 页(脏页 ≥4 时 2 页) = 单次阻塞 ≤6.4ms, 仍小于 12ms 事件年龄阈值.
 * 只有 cursor 命令与整页数据均收到 ACK 才同步快照/清脏位；NACK 时快照保持旧值,
 * 脏页保留并轮转到下一页，50ms 后再重试，避免屏幕掉线时每轮都忙等.
 * 周期全刷仍保留, 兜住上电/复位导致的总线状态失步. */
void OLED_FlushStep(void)
{
    uint8_t i, page;
    uint8_t budget;
    uint32_t now = Timer_GetTick();

    if (oled_retry_wait)
    {
        if ((uint32_t)(now - oled_retry_tick) < OLED_WRITE_RETRY_MS)
            return;
        oled_retry_wait = false;
    }

    if (oled_dirty == 0)
        return;

    budget = (oled_dirty_count() >= OLED_FLUSH_BATCH_MIN) ? OLED_FLUSH_BATCH : 1u;

    while (budget--)
    {
        for (i = 0; i < OLED_PAGE_NUM; i++)
        {
            uint8_t mask;
            bool ok;

            page = (uint8_t)((oled_flush_next + i) % OLED_PAGE_NUM);
            mask = (uint8_t)(1u << page);
            if (!(oled_dirty & mask))
                continue;

            ok = oled_set_cursor(page, 0);
            if (ok)
                ok = OLED_WriteData(OLED_DisplayBuf[page], OLED_WIDTH);

            if (ok)
            {
                oled_snap_sync(page);
                oled_dirty &= (uint8_t)~mask;
                oled_flush_next = (uint8_t)((page + 1) % OLED_PAGE_NUM);
                oled_retry_wait = false;
                oled_nack_streak = 0;   /* 总线恢复正常: 清零连续失败计数 */
            }
            else
            {
                /* ACK 缺失: 不提交快照、不清脏位; 给总线恢复/屏幕重连留时间. */
                oled_dirty |= mask;
                oled_flush_next = (uint8_t)((page + 1) % OLED_PAGE_NUM);
                /* 只有**连续**失败才启用全局退避(见 OLED_NACK_BACKOFF_AFTER):
                 * 单次瞬时 NACK 若也退避 50ms, 整屏会跟着停摆 → "突然卡一下".
                 * 屏幕真的不在时, 前几次失败很快(地址字节即 NACK, 约几十 µs),
                 * 达到阈值后照样退避, 不会退化成每轮忙重试. */
                if (oled_nack_streak < 0xFFu)
                    oled_nack_streak++;
                if (oled_nack_streak >= OLED_NACK_BACKOFF_AFTER)
                {
                    oled_retry_tick = now;
                    oled_retry_wait = true;
                }
                return;   /* 失败即停: 不再推第二批 */
            }

            break;   /* 本页完成, 继续下一批 */
        }

        if (i >= OLED_PAGE_NUM)
            return;   /* 没有更多脏页 */
    }
}



/*================ 清屏 =================*/

void OLED_Clear(void)
{
    /* 整屏清 0: 用 memset 按字写(库/编译器走 word store), 比逐字节双循环快数倍.
     * 本函数在每次整屏重绘(菜单列表 / 各页面 display)都被调用, 属热路径. */
    memset(OLED_DisplayBuf, 0, sizeof(OLED_DisplayBuf));
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



/*================ 像素操作 =================*/

/*
 * SSD1315 页模式:
 *
 * page = y / 8
 *
 * bit  = y % 8
 *
 * 一个 byte 代表纵向 8 个像素
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

/*================ 图形 =================*/

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



/*================ 圆角矩形 =================*/

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
            /* 左上、右上圆角 */
            int16_t dy=radius-1-yy;
            int16_t dx=0;

            while((dx+1)*(dx+1)+dy*dy<=radius*radius)
                dx++;

            left=x+radius-dx;
            right=x+width-radius+dx-1;
        }
        else if(yy>=height-radius)
        {
            /* 左下、右下圆角 */
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

/*================ 图片 =================*/

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

            if(data&(0x80>>(row&0x07)))   /* 字模统一 MSB 在上: bit7=该列最上像素 */
                OLED_DrawPixel(x+col,y+row,OLED_COLOR_SET);
            else
                OLED_DrawPixel(x+col,y+row,OLED_COLOR_CLEAR);
        }
    }
}



/*================ 字符 =================*/

/* 紧凑 ASCII 字库(每行 = [字符, 字形字节], 见 oled_data.c 裁剪说明):
 * 逐行线性查找 ch 的字形, 命中返回字形数据指针.
 * 未命中 = 该字符被 G8 宏开关剔除(或新增文案忘了开开关), 退化为 '?'
 * 兜底字形 —— 显示问号, 失败可见, 且不影响其它字符的对位.
 * '?' 由构建期(check_ascii_font.py)保证在保留集内, 第二次查找必中;
 * tab[1] 只是理论不可达时的非空兜底(第 0 行恒为 ' '). */
static const uint8_t *oled_ascii_glyph(const uint8_t *tab, uint8_t row, uint8_t n, uint8_t ch)
{
    uint8_t i;
    uint16_t off;

    for(i = 0; i < n; i++)
    {
        off = (uint16_t)((uint16_t)i * row);
        if(tab[off] == ch)
            return &tab[off + 1];
    }
    for(i = 0; i < n; i++)
    {
        off = (uint16_t)((uint16_t)i * row);
        if(tab[off] == '?')
            return &tab[off + 1];
    }
    return &tab[1];
}

void OLED_ShowChar(int16_t x,int16_t y,char ch,uint8_t font)
{
    /* 越界字符(控制符/非ASCII)替换为 '?', 防止字库越界读 */
    if(ch<' '||ch>'~')
        ch='?';

    if(font==OLED_8X16)
    {
        OLED_ShowImage(
            x,
            y,
            8,
            16,
            oled_ascii_glyph((const uint8_t *)OLED_F8x16,
                             (uint8_t)sizeof(OLED_F8x16[0]),
                             OLED_F8x16_COUNT,
                             (uint8_t)ch));
    }
    else if(font==OLED_6X8)
    {
        OLED_ShowImage(
            x,
            y,
            6,
            8,
            oled_ascii_glyph((const uint8_t *)OLED_F6x8,
                             (uint8_t)sizeof(OLED_F6x8[0]),
                             OLED_F6x8_COUNT,
                             (uint8_t)ch));
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



/*================ 数值 =================*/

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
    uint32_t v;

    if(num < 0)
    {
        OLED_ShowChar(x,y,'-',OLED_8X16);
        x+=8;
        /* 无符号回绕取绝对值: 对 INT32_MIN 也定义良好, 消除 -num 的未定义行为 */
        v = 0u - (uint32_t)num;
    }
    else
    {
        v = (uint32_t)num;
    }

    for(i=0;i<len;i++)
    {
        digit=(uint8_t)((v/oled_pow10(len-i-1))%10);

        OLED_ShowChar(
            x+i*8,
            y,
            '0'+digit,
            OLED_8X16);
    }
}



/*================ 16x32 =================*/

/* 字模取模方向统一约定: 所有字模(ASCII/中文/大数字/图标)一律
 * 列行式、从上到下、从左到右、高位在前(bit7=该列最上像素),
 * 与常见取模工具(PCTOLCD 等)默认输出一致, 新增字模可直接粘贴.
 * 注意: 显示缓冲为低位在前(bit0=页内最上像素), 整字节写入前需
 * 经 OLED_Rev8 反转; 逐像素读取路径(OLED_ShowImage)则用 MSB 位序. */
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
    uint32_t v;

    if(num<0)
    {
        neg=1;
        /* 无符号回绕取绝对值: 对 INT32_MIN 也定义良好, 消除 -num 的未定义行为 */
        v = 0u - (uint32_t)num;
    }
    else
    {
        v = (uint32_t)num;
    }

    do
    {
        buf[len++]=(uint8_t)(v%10);
        v/=10;
    }
    while(v>0);


    /* 摆位: 从左端第 1 个槽开始, 每绘一个字符 x 右移 16 —— 这样"先绘的字符
     * 在左", 与人的阅读方向一致. 用满宽右对齐: 最右字符右边缘落在 x+16*span.
     * 起始 x = 末尾槽起点 - 16*(span-1) = x + 16*(len+neg) - 16*span,
     * 其中 span = len+neg(负号也占一格). */
    pos=x+16*(len+neg)-16*(len+neg);


    if(neg)
    {
        OLED_ShowLargeChar(pos,y,'-');
        pos+=16;
    }

    /* 位数顺序: buf[] 由上面的 do-while 生成 —— buf[0] 是最低位(个位),
     * buf[len-1] 才是最高位. 由左往右绘, 故须自最高位起取: buf[len-1-i].
     * 曾误用 buf[i] 配右起左行的摆位: 把个位摆在最左, 两位数以上整体倒序
     * (25 显示 "52"、350 显示 "053"、300 显示 "003"); 只有 1 位数
     * 因次序无关而正确, 掩盖了该 bug. */
    for(i=0;i<len;i++)
    {
        OLED_ShowLargeChar(pos,y,'0'+buf[len-1-i]);
        pos+=16;
    }
}

/* 大字号 °C (16x32): 字形是 OLED_F16x32 表的第 10 项。
 * 收进本层的理由: "°C" 用哪一号字形是本层字模表的约定, 让页面层自己去数索引并
 * 直接写 OLED_DisplayBuf, 既破坏封装又容易写错; HOME 与温度设置页都要画它。
 * 32 行高须按页对齐, 否则跨页拼接会错位 —— 不满足直接不画(调用点只传 y=16)。 */
void OLED_ShowLargeDegC(int16_t x,int16_t y)
{
    uint8_t page;
    uint8_t col;

    if(y < 0 || (y & 0x07) != 0 || y + 32 > OLED_HEIGHT)
        return;

    for(page = 0; page < 4; page++)
    {
        for(col = 0; col < 16; col++)
        {
            uint8_t data = OLED_Rev8(OLED_F16x32[10][page * 16 + col]);
            int16_t px   = (int16_t)(x + col);

            if(data && px >= 0 && px < OLED_WIDTH)
                OLED_DisplayBuf[(uint8_t)((y >> 3) + page)][px] |= data;
        }
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


/* 拼接两段字面量: 等价于 snprintf(dst, cap, "%s%s", head, tail) 但不引入 stdio.
 * 仅供"固定标签 + 固定小段变量"的显示文案使用(如 "切换到"+"T245?"、
 * "固件版本:"+"1.0.0"), 两段都是编译期字面量, 故结果的像素宽度
 * (中文 16px / ASCII 8px)在调用点即可算准 —— 调用方自行居中/摆位, 本函数只管拼串.
 * 越界即截断(始终以 '\0' 收尾), 逐字节拷贝避免额外 strlen 与 dest 初值依赖. */
void oled_pair_cat(char *dst, uint8_t cap, const char *head, const char *tail)
{
    uint8_t n = 0;

    if (dst == NULL || cap == 0)
        return;

    while (*head && n < (uint8_t)(cap - 1))
        dst[n++] = *head++;

    while (*tail && n < (uint8_t)(cap - 1))
        dst[n++] = *tail++;

    dst[n] = '\0';
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

