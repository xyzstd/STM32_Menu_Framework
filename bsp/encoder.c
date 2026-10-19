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

/* 卡死自恢复阈值(原始计数): 正常转动时两次出步之间的累计原始计数不会超过 ~8;
 * 叠加换向反射与触点抖动, 最坏合法情况约 12. 取 16 留余量 —— 若累计到 16 仍无步,
 * 说明相位被反向抖动反复反射吃空(表现为"快速旋转时界面不动, 要反向才恢复").
 * 此时丢弃残差相位并请求上层作废积压旋转事件, 下一格即可正常出步, 无需反向.
 * 快速旋转下 16 计数约 60~100ms, 恢复延迟不可感知. */
#define ENC_STUCK_COUNTS    16

static uint16_t enc_last = 0;   /* 上次读取的计数器值 */
static int16_t  enc_acc  = 0;   /* 未满一格的余量计数 */
static uint16_t enc_stuck_acc = 0;            /* 自上次出步以来累计的原始计数(饱和) */
static volatile bool enc_stuck_req = false;   /* 相位锁死: 请求上层清事件队列 */

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
 * 仅由 input_scan(TIM2 中断)单上下文调用, 故无需临界区.
 * 重锚的规范语义按单位计数定义. 直接对聚合 delta 反射会令结果依赖 4ms 采样分块；
 * 这里最多处理首次反向计数以决定是否预支相位, 再把同方向余量作 O(1) 净结算.
 * 因而既与单位计数状态转移等价, 又不会因毛刺让 ISR 进入长循环. */
int16_t Encoder_GetSteps(void)
{
    uint16_t now   = TIM_GetCounter(TIMx);
    int16_t  delta = (int16_t)(now - enc_last);   /* 差分: 16bit 回绕安全 */
    int32_t  sum;
    int32_t  residual;
    int16_t  steps;
    int8_t   dir;

    /* 跳转重同步: 只清残留相位(enc_acc), 不动计数基准 enc_last ——
     * 自上次取样以来的计数(delta)照常结算, 故"跳转后立刻转的第一格"不会被吞. */
    if (enc_resync_req)
    {
        enc_resync_req = false;
        enc_acc = 0;
        enc_stuck_acc = 0;
    }

    if (delta == 0)
        return 0;

    enc_last = now;
    residual = enc_acc;
    steps = 0;
    dir = (delta > 0) ? 1 : -1;

    /* 累计原始计数(含反向抖动): 出步即清零, 用于卡死判定 */
    {
        int32_t ad = (delta > 0) ? delta : -(int32_t)delta;

        enc_stuck_acc = (ad > (int32_t)(0xFFFFu - enc_stuck_acc))
                      ? (uint16_t)0xFFFFu
                      : (uint16_t)(enc_stuck_acc + ad);
    }

    /* 转向预支: 若新方向与旧残差反号, 先只处理第一个单位计数.
     * sum 仍与旧残差同号=该单位尚未越锚, 反射相位; 随后残差已与新方向同号,
     * delta 剩余部分不可能再触发反射, 可直接合并. 若 sum 为 0, 按越锚分支处理.
     * 例 acc=+3、delta=-3: 首计数使 sum=+2→预支成 -2, 余量 -2 合计 -4,
     * 输出 -1 步; 无论该 -3 被分成 -1/-2 还是 -2/-1, 结果相同.
     * delta 由 int16_t 差分给出, 最大幅度 32768; 全程 int32_t 中间量, 无长循环. */
    if (residual != 0 && ((delta > 0) != (residual > 0)))
    {
        sum = residual + dir;
        if ((sum > 0) == (residual > 0))
            residual = -sum;   /* 越锚前反射, 减少换向死区 */
        else
            residual = sum;    /* 首个计数已到锚点/越锚: 不反射 */
        delta = (int16_t)(delta - dir);  /* 首个单位计数已纳入 residual */
    }

    residual += delta;
    steps = (int16_t)(residual / ENC_TICKS_PER_STEP);  /* C99 有符号除法向零截断 */
    residual -= (int32_t)steps * ENC_TICKS_PER_STEP;
    enc_acc = (int16_t)residual;

    if (steps != 0)
        enc_stuck_acc = 0;
    else if (enc_stuck_acc >= ENC_STUCK_COUNTS)
    {
        enc_acc = 0;            /* 丢弃残差相位: 下一格重新从 0 起算 */
        enc_stuck_acc = 0;
        enc_stuck_req = true;   /* 队列清空由主循环(消费者)执行 */
    }

    return steps;
}


/* 取走"相位锁死"请求(见 ENC_STUCK_COUNTS): true = 已判死并丢弃了残差相位,
 * 调用方应作废积压的旋转事件(旧事件对应的相位已失效)。主循环上下文调用。 */
bool Encoder_TakeStuck(void)
{
    if (!enc_stuck_req)
        return false;

    enc_stuck_req = false;
    return true;
}
#undef TIMx
#undef TIMx_CLK
