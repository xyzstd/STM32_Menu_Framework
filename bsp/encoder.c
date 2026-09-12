#include "encoder.h"

#define TIMx                TIM4
#define TIMx_CLK            RCC_APB1Periph_TIM4

void Encoder_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(TIMx_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10;
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInit(TIMx, &TIM_ICInitStructure);

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(TIMx, &TIM_ICInitStructure);

    TIM_EncoderInterfaceConfig(
        TIMx,
        TIM_EncoderMode_TI12,
        TIM_ICPolarity_Rising,
        TIM_ICPolarity_Falling);

    TIM_Cmd(TIMx, ENABLE);
	TIM_SetCounter(TIMx, 0);
}

int16_t Encoder_GetDiff(void)
{
    static uint16_t last = 0;

    uint16_t now = TIM_GetCounter(TIMx);

    int16_t diff = (int16_t)(now - last);

    last = now;

    return diff;
}

#undef TIMx
#undef TIMx_CLK 

