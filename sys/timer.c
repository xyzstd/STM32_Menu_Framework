#include "stm32f10x.h"                  // Device header
#include "timer.h"

#define TIMx                TIM2
#define TIMx_IRQn           TIM2_IRQn
#define TIMx_CLK            RCC_APB1Periph_TIM2

static volatile uint32_t sys_tick=0;

/* 1ms 节拍回调 (注册者: main/input.c 的 Input_Scan).
 * volatile: 主循环可注册/注销, 中断随时读取. */
static volatile timer_tick_fn tick_cb = NULL;

void Timer_SetTickCallback(timer_tick_fn fn)
{
    tick_cb = fn;
}

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
    
    /* 中断优先级分组已由 main 在任何 NVIC_Init 之前统一设置 (NVIC_PriorityGroup_2),
     * 此处不再重复; 分组晚设会让先配置的中断编码出错, 故必须单点提前设置. */
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
        /* 派发给注册者(输入扫描按 1ms 采样, 自身每 4/5ms 才真正取样).
         * 通过回调而非直接调用: sys/ 不再依赖 main/ 的任何头文件. */
        if(tick_cb)
            tick_cb();
        TIM_ClearITPendingBit(
            TIM2,
            TIM_IT_Update
        );
    }
}

#undef TIMx                
#undef TIMx_IRQn           
#undef TIMx_CLK        
