#include "stm32f10x.h"                  // Device header
#include "pwm.h"

/* PWM 频率 = TIM1 时钟(72MHz) / ((PSR+1)*(ARR+1)) = 72M / (6*1000) = 12kHz
 * 分辨率 1000 级 (CCR 0~999). */
#define PWM_ARR_VALUE   999U            /* ARR: 周期计数值 (1000-1) */
#define PWM_PSR_VALUE   5U              /* PSR: 预分频值  (6-1)    */

/* 最大占空比 90% 的钳制已上移到 Heater 层 (power ≤ 900), 本层保持纯 0~999 线性映射。 */

/* 注入组采样相位: false = 导通段中点(采导通电流), true = 关断段中点(采关断基准)。
 * 由 ADC 的 JEOC 中断逐帧取反, 主循环只读。初值取关断相位: 保证第一帧先落到关断
 * 基准上, 免得 AD_GetCurrent 在基准尚未采到时把 0 当基准算出偏大的电流。
 * volatile 必须加: 该量在 ADC1_2_IRQHandler 里写、在 PWM_SetCompare1 里读,
 * 无 volatile 时编译器可把读取提升到循环外缓存, 主循环将长期用错相位 ——
 * 直接影响电流采样与过流判定。同类 ISR/主循环共享量在 adc.c/encoder.c 均已加。 */
static volatile bool inject_off_phase = true;

/* ADC 采样孔径 (注入通道采样时间 55.5 个 ADC 时钟): 采样开关在孔径末端断开,
 * 锁存到的就是那一刻的输入, 故触发点要比目标时刻提前一个孔径。ADC 时钟与 TIM1
 * 计数时钟同为 12MHz(均由 72MHz 分频), 故 1 个 ADC 时钟 = 1 个 PWM tick。 */
#define INJ_APERTURE_TICKS  56

/* 关断相位采样点距周期末的裕量: 越靠后, 运放自关断沿起可恢复的时间越长;
 * 留一段裕量避免孔径末端越过周期末(撞上下个周期的导通沿) */
#define INJ_TAIL_MARGIN     30

static uint16_t inject_ccr4(uint16_t compare, bool off_phase);

/* 注入触发点 CC4: 两相都以"锁存时刻"(孔径末端)锚定, 触发点 = 锁存时刻 − 孔径。
 *   导通相位: 锁存时刻取导通段 [0, CCR1] 中点 → 触发点 = CCR1/2 − 孔径;
 *   关断相位: 锁存时刻取关断窗末段         → 距周期末 INJ_TAIL_MARGIN。
 * 加热层已把功率钳到 90% → CCR1 ≤ 899, 关断窗宽 100 tick > 孔径 56 tick;
 * 关断相的两处钳位只在该前提下失效时兜底(API 允许 CCR1 取到 999)。 */
static uint16_t inject_ccr4(uint16_t compare, bool off_phase)
{
    int32_t ccr;

    if (!off_phase)
    {
        /* 导通相位: 目标锁存时刻 = 导通段 [0, CCR1] 中点, 触发点须再提前一个孔径
         * (与上方 INJ_APERTURE_TICKS 的规则一致; 原先漏了这一步 —— 锁存时刻实际
         * 偏晚一个孔径, 功率 <11.2%(CCR1<112) 时采样窗尾部越入关断段, 导通电流
         * 被低估 → 过流保护变钝). 下限 0 防负触发点; 无需上钳位: 前移一个孔径后
         * 窗口上界 = CCR1/2 ≤ CCR1, 恒不越出导通段. */
        ccr = (int32_t)compare / 2 - INJ_APERTURE_TICKS;
        if (ccr < 0)
            ccr = 0;

        return (uint16_t)ccr;
    }

    ccr = (int32_t)PWM_ARR_VALUE - INJ_APERTURE_TICKS - INJ_TAIL_MARGIN;

    if (ccr < (int32_t)compare + 1)                          /* 不得落回导通段 */
        ccr = (int32_t)compare + 1;
    if (ccr > (int32_t)PWM_ARR_VALUE - INJ_APERTURE_TICKS)   /* 孔径不得越过周期末 */
        ccr = (int32_t)PWM_ARR_VALUE - INJ_APERTURE_TICKS;

    return (uint16_t)ccr;
}

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
     * CH4 比较事件触发注入组(电流): 触发点在导通段中点与关断段中点之间逐帧交替,
     * 分别采"导通电流"与"关断基准"。触发点定位由 PWM_SetCompare1(随功率)与
     * PWM_SetInjectPhase(随相位)维护, 这里仅置初值。不出波形。
     * (本标准库注入组外部触发仅支持 T1_TRGO/T1_CC4, 故用 CC4 而非 CC2。) */
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
    if(Compare>=PWM_CCR_MAX)Compare=PWM_CCR_MAX;
    TIM_SetCompare1(TIM1, Compare);

    /* 功率变化时按当前相位重算 CC4(触发点位置随 CCR1 走), 保证注入采样
     * 始终落在正确的相位区间。与中断里的相位取反存在极窄的读写交错窗口,
     * 但注入值是"逐帧刷新"的单值, 交错至多让某一帧归错相位, 下一帧(≈166µs)
     * 即被覆盖, 远早于 200ms 的读取周期, 也不会造成误判过流。 */
    TIM_SetCompare4(TIM1, inject_ccr4(Compare, inject_off_phase));
}

void PWM_SetInjectPhase(bool off_phase)
{
    inject_off_phase = off_phase;
    /* CCR1 由加热层维护, 这里现取现算, 免去在两个模块间同步功率副本 */
    TIM_SetCompare4(TIM1, inject_ccr4(TIM_GetCapture1(TIM1), off_phase));
}

bool PWM_GetInjectPhase(void)
{
    return inject_off_phase;
}
