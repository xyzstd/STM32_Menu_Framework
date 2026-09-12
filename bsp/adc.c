#include "stm32f10x.h"                  // Device header

/*
 * ADC 采集: 双触发 — 规则组(自由采样) + 注入组(导通采, 电流)
 *
 *  规则组 (规则组外部触发 = None, 软件连续转换, 自由采样):
 *     CH0(PA0,T12热电偶), CH1(PA1,电流/关断), CH2(PA2,输入电压), CH6(PA6,NTC)
 *     → 连续扫描 + 循环 DMA → adc_buf, 采样噪声由 work_task 一阶低通 temp_filt 抑制
 *  注入组 (注入组外部触发 = TIM1 CC4, 导通段中点, 有过流电流):
 *     电流采样电阻(运放B输出) ADC 通道 → JEOC 中断读 JDR1 → adc_cur
 *
 *  规则组连续转换, 自由采样的 4 通道 DMA 缓存 (不锁 CC3 相位, 兼容 T12/T245);
 *  注入组被 CC4(导通)触发, 单通道电流采样, 中断取回。
 *  差分校准: 过流电流 = 导通电流(注入) - 关断电流(规则组电流通道),
 *              消去运放失调/共模/ADC 偏置。
 *  非阻塞: AD_Get* 返回缓存, 跨周期平均由调用方(work_task 一阶低通)完成。
 */

/* 采样通道表 (顺序 = DMA 缓存顺序) */
typedef struct
{
    uint8_t  adc_ch;      /* ADC 通道号 */
    uint8_t  chan_index;  /* ADC_Channel_x */
    uint16_t gpio_pin;    /* 对应 GPIO */
    uint8_t  report_slot; /* 在报告数组中的下标 */
} adc_chan_cfg_t;

#define ADC_CHANNEL_COUNT   4

/* 规则组过采样帧数: 连续自由采样下对同一通道 N 帧求平均, 抑制 PWM 开关沿/工频噪声。
 * 一轮 4 通道 ≈ 18.5µs(12MHz/55.5周期), N=8 → 均值窗口 ≈ 148µs, 延迟可忽略.
 * 缓存按 N×通道数展开成环形, 由循环 DMA 持续填充。 */
#define TEMP_OVERSAMPLE      8

/* 电流采样通道 (PA1 = ADC_Channel_1, 按实际接线可改)
 * 同时用于: 注入组=导通电流,  规则组=关断电流(差分校准) */
#define CUR_ADC_CH          ADC_Channel_1
#define CUR_ADC_PIN         GPIO_Pin_1
#define CUR_GPIOX           GPIOA

/* 缓存: DMA 每次完成一轮(N 帧)后环形填充, 每帧按通道表顺序填入 */
static volatile uint16_t adc_buf[ADC_CHANNEL_COUNT * TEMP_OVERSAMPLE];
/* 注入组(电流)最新采样值, 由 ADC1_2_IRQHandler 更新 (导通电流) */
static volatile uint16_t adc_cur;

static const adc_chan_cfg_t chan_cfgs[ADC_CHANNEL_COUNT] =
{
    { 0, ADC_Channel_0, GPIO_Pin_0, 0 }, /* 热电偶 T12 */
    { 1, ADC_Channel_1, GPIO_Pin_1, 1 }, /* 电流(关断)  */
    { 2, ADC_Channel_2, GPIO_Pin_2, 2 }, /* 输入电压    */
    { 6, ADC_Channel_6, GPIO_Pin_6, 3 }, /* NTC 手柄    */
};

void AD_Init(void)
{
    uint8_t i;

    /* 1. 使能 ADC / GPIOA / DMA1 时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    /* 2. GPIO 全设 AIN */
    {
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AIN;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStruct.GPIO_Pin = 0;
        for (i = 0; i < ADC_CHANNEL_COUNT; i++)
            GPIO_InitStruct.GPIO_Pin |= chan_cfgs[i].gpio_pin;
        GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* 电流采样引脚 (注入组, 运放B输出) */
        GPIO_InitStruct.GPIO_Pin = CUR_ADC_PIN;
        GPIO_Init(CUR_GPIOX, &GPIO_InitStruct);
    }

    /* 3. DMA1 通道1 ← ADC1, 循环模式, 目标 = 缓存 */
    {
        DMA_InitTypeDef DMA_InitStruct;
        DMA_DeInit(DMA1_Channel1);
        DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
        DMA_InitStruct.DMA_MemoryBaseAddr     = (uint32_t)adc_buf;
        DMA_InitStruct.DMA_DIR                = DMA_DIR_PeripheralSRC;
        DMA_InitStruct.DMA_BufferSize         = ADC_CHANNEL_COUNT * TEMP_OVERSAMPLE;
        DMA_InitStruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
        DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
        DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
        DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
        DMA_InitStruct.DMA_Mode               = DMA_Mode_Circular;
        DMA_InitStruct.DMA_Priority           = DMA_Priority_High;
        DMA_InitStruct.DMA_M2M                = DMA_M2M_Disable;
        DMA_Init(DMA1_Channel1, &DMA_InitStruct);
        DMA_Cmd(DMA1_Channel1, ENABLE);
    }

    /* 4. ADC1: 扫描 + 定时器触发 + DMA 请求 */
    {
        ADC_InitTypeDef ADC_InitStruct;
        ADC_InitStruct.ADC_Mode               = ADC_Mode_Independent;
        ADC_InitStruct.ADC_ScanConvMode       = ENABLE;
        ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;    /* 连续转换模式, 统一自由采样(兼容 T12/T245, 由软件低通滤波 temp_filt 抑噪) */
        ADC_InitStruct.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None;   /* 自由采样, 依赖软件启动一次后连续转换 */
        ADC_InitStruct.ADC_DataAlign          = ADC_DataAlign_Right;
        ADC_InitStruct.ADC_NbrOfChannel       = ADC_CHANNEL_COUNT;
        ADC_Init(ADC1, &ADC_InitStruct);

        /* 5. 各通道采样时间 + 在扫描顺序中的位置 */
        for (i = 0; i < ADC_CHANNEL_COUNT; i++)
            ADC_RegularChannelConfig(ADC1, chan_cfgs[i].adc_ch, i + 1, ADC_SampleTime_55Cycles5);

        ADC_DMACmd(ADC1, ENABLE);
        ADC_Cmd(ADC1, ENABLE);

        /* 校准 */
        ADC_ResetCalibration(ADC1);
        while (ADC_GetResetCalibrationStatus(ADC1));
        ADC_StartCalibration(ADC1);
        while (ADC_GetCalibrationStatus(ADC1));

        /* 自由采样: 无外部触发, 软件触发一次, 结合 ContinuousConvMode/循环 DMA 持续转换 */
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    }

    /* 5. 注入组: 电流通道, 外部触发 TIM1 CC4(导通中), 单通道注入 */
    {
        /* 注入通道序列第 1 位 = 电流 */
        ADC_InjectedChannelConfig(ADC1, CUR_ADC_CH, 1, ADC_SampleTime_55Cycles5);
        /* 注入组外部触发配置在 ADC_Init 之后单独设置 (本标准库 ADC_InitTypeDef 不含注入触发字段),
         * 触发源用 T1_CC4(导通中) —— 注入组支持 T1_TRGO/T1_CC4, 无 T1_CC2。 */
        ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_CC4);
        ADC_ExternalTrigInjectedConvCmd(ADC1, ENABLE);

        /* JEOC 中断: 注入转换完成后读回电流值 */
        ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);
        NVIC_InitTypeDef NVIC_InitStruct;
        NVIC_InitStruct.NVIC_IRQChannel                   = ADC1_2_IRQn;
        NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
        NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0;
        NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
        NVIC_Init(&NVIC_InitStruct);
    }
}

/* 读取指定 ADC 通道的 DMA 缓存值: 对该通道 N 帧过采样求平均(四舍五入), 抑制采样噪声
 * 循环 DMA 持续写入缓存, 读取期间可能跨越新老两轮 → 用 DMA 传输计数器(CNDTR)
 * 做一致性快照: 读完整轮后 CNDTR 未变说明期间无传输, 样本同轮, 结果可信;
 * 否则混入新老样本, 重读。正常读取仅亚微秒, 远小于单帧写入间隔(≈18.5µs), 首次即成功。 */
uint16_t AD_GetValue(uint8_t channel_adc)
{
    uint8_t i, ci = ADC_CHANNEL_COUNT;
    uint32_t sum;
    uint16_t v, vmin, vmax;

    for (i = 0; i < ADC_CHANNEL_COUNT; i++)
        if (chan_cfgs[i].adc_ch == channel_adc) { ci = i; break; }
    if (ci >= ADC_CHANNEL_COUNT) return 0;

    for (;;)
    {
        uint16_t before = DMA_GetCurrDataCounter(DMA1_Channel1);
        sum = 0;
        vmin = 0xFFFF;
        vmax = 0;
        for (i = 0; i < TEMP_OVERSAMPLE; i++)
        {
            v = adc_buf[i * ADC_CHANNEL_COUNT + ci];
            sum += v;
            if (v < vmin) vmin = v;
            if (v > vmax) vmax = v;
        }
        if (DMA_GetCurrDataCounter(DMA1_Channel1) == before)
            break;
    }

    /* 去极值平均: 丢弃一帧最大、一帧最小(偶发毛刺/PWM 开关沿), 对剩余帧求平均,
     * 再按比例放大回满采样数, 避免被单帧毛刺拉偏。 */
    sum -= (uint32_t)vmin + vmax;
    return (uint16_t)((sum + TEMP_OVERSAMPLE / 2) / (TEMP_OVERSAMPLE - 2));
}

/* 读取 T12 热电偶温度 ADC (PA0, CH0) */
uint16_t AD_GetT12(void)
{
    return AD_GetValue(0);
}

/* 读取输入电压 ADC (PA2, CH2) */
uint16_t AD_GetVin(void)
{
    return AD_GetValue(2);
}

/* 读取 NTC 手柄温度 ADC (PA6, CH6) */
uint16_t AD_GetNTC(void)
{
    return AD_GetValue(6);
}

/* 读取关断段电流 (规则组缓存里的电流通道, 自由采样) */
uint16_t AD_GetCurrentOff(void)
{
    return AD_GetValue(CUR_ADC_CH);
}

/* 差分校准后电流 = 导通电流(注入组, CC4) - 关断电流(规则组电流通道, 自由采样)
 * 消去运放失调/共模/ADC 偏置, 接近真实工作电流。
 * 导通值被偏置抬高时结果更大 → 过流判断更准；关断 ≥ 导通时钳到 0 防下溢。 */
uint16_t AD_GetCurrent(void)
{
    uint16_t i_off = AD_GetCurrentOff();
    uint16_t i_on  = adc_cur;
    return (i_on > i_off) ? (i_on - i_off) : 0;
}

/* 注入组 JEOC 中断: 导通中采样完成 → 读回电流值 (过流检测) */
void ADC1_2_IRQHandler(void)
{
    if (ADC_GetITStatus(ADC1, ADC_IT_JEOC) != RESET)
    {
        adc_cur = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
        ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);
    }
}