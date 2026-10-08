#include "stm32f10x.h"
#include <stdbool.h>
#include "encoder.h"

/*
 * 旋转编码器: TIM4 硬件编码器接口(TI12)计数.
 *
 * 为什么不用 GPIO 外部中断(EXTI): STM32F1 的 EXTI 没有数字滤波, 编码器信号线上的
 * 窄毛刺会直接变成一次中断; 信号稍差时就是高频中断风暴, 把主循环(按键消费/渲染)
 * 整个饿死 —— 表现为"按键没反应 + 计数疯狂乱跳". TIM4 的输入捕获滤波器
 * (TIM_ICFilter) 在硬件层就能滤掉窄毛刺, 计数稳定且不占中断.
 *
 * 引脚: PB6=A(TI1), PB7=B(TI2), 上拉输入; 编码器公共端接 GND.
 * 计数→步: 机械编码器每齿格 4 个正交计数, 每满 4 计数算一个菜单步.
 */

#define TIMx                TIM4
#define TIMx_CLK            RCC_APB1Periph_TIM4

#define ENC_TICKS_PER_STEP  4   /* 每齿格的正交计数(满 4 计数 = 一个菜单步) */

static uint16_t enc_last = 0;   /* 上次读取的计数器值 */
static int16_t  enc_acc  = 0;   /* 未满一格的余量计数 */

/* 跳转重同步请求: 主循环置位、取样上下文执行(见 Encoder_RequestResync).
 * 单字节标志, 主循环写/中断读, 无需临界区. */
static volatile bool enc_resync_req = false;

void Encoder_Init(void)
{
    GPIO_InitTypeDef       GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef      TIM_ICInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(TIMx_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;      /* 上拉输入, 编码器公共端接 GND */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler     = 0;
    TIM_TimeBaseStructure.TIM_Period        = 0xFFFF;   /* 16bit 满量程, 差分读取自然抗回绕 */
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

    /* 输入滤波: 硬件滤掉窄毛刺(关键抗噪手段, EXTI 方案没有这一层) */
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10;
    TIM_ICInitStructure.TIM_Channel  = TIM_Channel_1;
    TIM_ICInit(TIMx, &TIM_ICInitStructure);

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(TIMx, &TIM_ICInitStructure);

    TIM_EncoderInterfaceConfig(TIMx, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Falling);

    TIM_Cmd(TIMx, ENABLE);
    TIM_SetCounter(TIMx, 0);

    enc_last = 0;
    enc_acc  = 0;
}

/* 请求"重同步": 丢弃未满一格的残留相位(计数基准不动).
 * 供菜单跨页/跨列表跳转后调用 —— 旧上下文的半格残留若被带入新页面, 当残留与
 * 新转向反号时, 新方向第一格(4 计数)不足以补齐一格, 表现为"进子菜单/返回后
 * 第一格不动, 要再转一格". 置位在主循环, 复位在 Encoder_GetSteps 内(与计数读取
 * 同上下文)执行, 避免与中断竞争. */
void Encoder_RequestResync(void)
{
    enc_resync_req = true;
}

/* 取走自上次调用以来累计的步数(正=正向/UP, 负=反向/DOWN); 读后清零.
 * 仅由 input_scan(TIM2 中断)单上下文调用, 故无需临界区. */
int16_t Encoder_GetSteps(void)
{
    uint16_t now   = TIM_GetCounter(TIMx);
    int16_t  delta = (int16_t)(now - enc_last);   /* 差分: 16bit 回绕安全 */
    int16_t  sum;
    int16_t  steps;

    /* 跳转重同步: 只清残留相位(enc_acc), 不动计数基准 enc_last ——
     * 自上次取样以来的计数(delta)照常结算, 故"跳转后立刻转的第一格"不会被吞. */
    if (enc_resync_req)
    {
        enc_resync_req = false;
        enc_acc = 0;
    }

    if (delta == 0)
        return 0;

    enc_last = now;
    sum = (int16_t)(enc_acc + delta);

    /* 转向重锚(消除"转向卡格"): 反向时把旧方向的残留相位 r 沿新方向预支掉 ——
     * 锚点反射到当前位置, 新方向只需 4-|r| 计数即出一步, 而不是"先还清 |r|
     * 再走 4"(最坏 7 计数 ≈ 1.75 格空转, 即"转到底子后反向不动").
     * 只在反向"尚未跨过锚点"时反射: 判据取 sum 与 enc_acc 同号(残留仍在本侧).
     * 一旦反向计数越过锚点(sum 变号)即按净位移结算 —— 否则反射会得到与净位移
     * 反号的残差(快速反向时多走数格才出事件, 甚至多报/少报一步).
     * 残留为 0(正好停在格边界)时无残留可预支, 转向死区保持 1 格, 属固有下限. */
    if (enc_acc != 0 && ((delta > 0) != (enc_acc > 0)) &&
        ((sum > 0) == (enc_acc > 0)))
        enc_acc = (int16_t)(-sum);
    else
        enc_acc = sum;

    steps   = (int16_t)(enc_acc / ENC_TICKS_PER_STEP);         /* 满格数(向零截断) */
    enc_acc = (int16_t)(enc_acc - steps * ENC_TICKS_PER_STEP); /* 保留不足一格的余量 */

    return steps;
}
