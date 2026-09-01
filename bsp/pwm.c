#include "stm32f10x.h"                  // Device header
#include "pwm.h"

/* PWM 频率 = TIM1 时钟(72MHz) / ((PSR+1)*(ARR+1)) = 72M / (6*1000) = 12kHz
 * 分辨率 1000 级 (CCR 0~999). */
#define PWM_ARR_VALUE   999U            /* ARR: 周期计数值 (1000-1) */
#define PWM_PSR_VALUE   5U              /* PSR: 预分频值  (6-1)    */

/* 最大占空比 90% 的钳制已上移到 Heater 层 (power ≤ 900), 本层保持纯 0~999 线性映射。 */

void PWM_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct;
    
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;          /* TIM1_CH1 (PA8), 不用互补 CH1N */
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&GPIO_InitStruct);
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up ;
    TIM_TimeBaseInitStruct.TIM_Period = PWM_ARR_VALUE;
    TIM_TimeBaseInitStruct.TIM_Prescaler = PWM_PSR_VALUE;
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStruct);
    
    TIM_OCInitTypeDef TIM_OCInitStruct;
    TIM_OCStructInit(&TIM_OCInitStruct);
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse = 0; //CCR
    TIM_OC1Init(TIM1,&TIM_OCInitStruct);

    /* 规则组已改软件连续自由采样(adc.c), 不再需要 CH3 事件。
     * CH4 比较事件触发注入组(电流), 采样点=导通段中点。
     * (本标准库注入组外部触发仅支持 T1_TRGO/T1_CC4, 故用 CC4 而非 CC2。)
     * 实际定位放在 PWM_SetCompare1 里与 CCR1 同步更新, 这里仅置初值。不出波形。 */
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_Toggle;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Disable; /* 不出波形, 仅产生 CC 事件 */
    TIM_OCInitStruct.TIM_Pulse = 0;                             /* 初值, 实际由 SetCompare1 维护 */
    TIM_OC4Init(TIM1,&TIM_OCInitStruct);

    /* TIM1 高级定时器: 即使只用 CH1 非互补输出, 也必须开 MOE 主输出使能,
     * 否则 PWM 通道被 BDTR 的 MOE 位关死, PA8 无波形输出。 */
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    TIM_Cmd(TIM1,ENABLE);
}

void PWM_SetCompare1(uint16_t Compare)
{
    uint16_t ccr4;
	
	if(Compare>=PWM_CCR_MAX)Compare=PWM_CCR_MAX;
    TIM_SetCompare1(TIM1, Compare);

    /* 导通段 [0, CCR1] 中点 = CCR1/2 → CC4 事件触发注入组(电流), 导通中采样有过流电流 */
    ccr4 = (uint16_t)(Compare / 2);
    TIM_SetCompare4(TIM1, ccr4);
}
