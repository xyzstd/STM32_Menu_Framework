#include "stm32f10x.h"                  // Device header
#include "timer.h"

#define TIMx                TIM2
#define TIMx_IRQn           TIM2_IRQn
#define TIMx_CLK            RCC_APB1Periph_TIM2

static volatile uint32_t sys_tick=0;

void Timer_Init(void)
{
    RCC_APB1PeriphClockCmd(TIMx_CLK, ENABLE);
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up ;
    TIM_TimeBaseInitStruct.TIM_Period = 1000 - 1;
    TIM_TimeBaseInitStruct.TIM_Prescaler = 72 -1;
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
    
    TIM_TimeBaseInit(TIMx,&TIM_TimeBaseInitStruct);
    TIM_ClearFlag(TIMx,TIM_FLAG_Update);
    TIM_ITConfig(TIMx,TIM_IT_Update,ENABLE);
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = TIMx_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStruct);

    TIM_Cmd(TIMx,ENABLE);
}

uint32_t Timer_GetTick(void)
{
    return sys_tick;
}

void TIM2_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET)
    {
        sys_tick++;

        TIM_ClearITPendingBit(
            TIM2,
            TIM_IT_Update
        );
    }
}

#undef TIMx                
#undef TIMx_IRQn           
#undef TIMx_CLK        
