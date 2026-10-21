#include "stm32f10x.h"                  // Device header
#include <stdbool.h>
#include "pwm.h"

/*
 * ADC 采集: 双触发 — 规则组(自由采样) + 注入组(相位锁定, 电流)
 *
 *  规则组 (规则组外部触发 = None, 软件连续转换, 自由采样):
 *     CH0(PA0,T12热电偶), CH2(PA2,输入电压), CH16(CPU 内部温度, 冷端补偿参考)
 *     → 连续扫描 + 循环 DMA → adc_buf, 采样噪声由 work_task 一阶低通 temp_filt 抑制
 *  注入组 (注入组外部触发 = TIM1 CC4, 单通道电流):
 *     电流采样电阻(运放B输出) ADC 通道 → JEOC 中断读 JDR1
 *     CC4 触发点由 pwm.c 逐帧在"导通段中点"与"关断段中点"之间交替, 中断按当前
 *     相位把采样分派到 adc_cur(导通电流) / adc_cur_off(关断基准)。
 *
 *  差分校准: 过流电流 = 导通相位采样 - 关断相位采样, 消去运放失调/共模/ADC 偏置。
 *              两路都由 CC4 相位锁定, 与 PWM 严格同步, 不依赖规则组的自由采样相位。
 *  非阻塞: AD_Get* 返回缓存, 跨周期平均由调用方(work_task 一阶低通)完成。
 */

/* 采样通道表 (顺序 = DMA 缓存顺序) */
typedef struct
{
    uint8_t  adc_ch;      /* ADC 通道号 */
    uint16_t gpio_pin;    /* 对应 GPIO */
} adc_chan_cfg_t;

#define ADC_CHANNEL_COUNT   3

/* 规则组过采样帧数: 连续自由采样下对同一通道 N 帧求平均, 抑制 PWM 开关沿/工频噪声。
 * 一轮 3 通道 ≈ 32µs(68+68+252 个 12MHz 时钟, CH16 用长采样), N=8 → 均值窗口 ≈ 0.26ms,
 * 延迟可忽略. 缓存按 N×通道数展开成环形, 由循环 DMA 持续填充。 */
#define TEMP_OVERSAMPLE      8

/* 一致性快照最大尝试次数: 正常读取亚微秒级, 首次即成功; 上限只为防止
 * DMA/ADC 异常停摆时 CNDTR 恒变而无限空转(会拖到 IWDG 复位整机)。 */
#define ADC_SNAPSHOT_RETRY   8

/* 电流采样通道 (PA1 = ADC_Channel_1, 按实际接线可改)
 * 只走注入组(CC4 相位锁定): 导通段中点采导通电流, 关断段中点采关断基准 */
#define CUR_ADC_CH          ADC_Channel_1
#define CUR_ADC_PIN         GPIO_Pin_1
#define CUR_GPIOX           GPIOA

/* 缓存: DMA 每次完成一轮(N 帧)后环形填充, 每帧按通道表顺序填入 */
static volatile uint16_t adc_buf[ADC_CHANNEL_COUNT * TEMP_OVERSAMPLE];
/* 注入组(电流)最新采样值, 由 ADC1_2_IRQHandler 按触发相位分别更新 */
static volatile uint16_t adc_cur;       /* 导通段中点采样 → 导通电流 */
static volatile uint16_t adc_cur_off;   /* 关断段中点采样 → 关断基准(运放失调/偏置) */
/* 自上次读取以来是否收到过新的导通采样。断热(PWM CCR=0)期间 CC4 不产生导通采样,
 * adc_cur 会停留在停机前的陈旧值, 靠此标志让 AD_GetCurrent 区分"无导通"与"有电流"。 */
static volatile bool adc_cur_fresh;

static const adc_chan_cfg_t chan_cfgs[ADC_CHANNEL_COUNT] =
{
    { 0,  GPIO_Pin_0 }, /* 热电偶 T12 */
    { 2,  GPIO_Pin_2 }, /* 输入电压    */
    { 16, 0          }, /* CPU 内部温度(冷端补偿参考, 无外部引脚) */
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

        /* 5. 各通道采样时间 + 在扫描顺序中的位置.
         * 内部温度传感器建议长采样(≥239.5 周期)以获得稳定读数, 其余通道 55.5. */
        for (i = 0; i < ADC_CHANNEL_COUNT; i++)
            ADC_RegularChannelConfig(ADC1, chan_cfgs[i].adc_ch, i + 1,
                (chan_cfgs[i].adc_ch == ADC_Channel_16) ? ADC_SampleTime_239Cycles5
                                                        : ADC_SampleTime_55Cycles5);

        ADC_DMACmd(ADC1, ENABLE);
        ADC_TempSensorVrefintCmd(ENABLE);   /* 使能内部温度传感器 + VREFINT (先于 ADC 上电) */
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
 * 否则混入新老样本, 重读。正常读取仅亚微秒, 远小于单帧写入间隔(≈18.5µs), 首次即成功。
 * 重读设上限(ADC_SNAPSHOT_RETRY): DMA/ADC 停摆时 CNDTR 恒变, 无上限会死等拖到
 * IWDG 复位; 达到上限即接受最近一轮样本(可能跨轮混样, 但远优于整机复位)。 */
uint16_t AD_GetValue(uint8_t channel_adc)
{
    uint8_t i, ci = ADC_CHANNEL_COUNT;
    uint8_t retry;
    uint32_t sum;
    uint16_t v, vmin, vmax;

    for (i = 0; i < ADC_CHANNEL_COUNT; i++)
        if (chan_cfgs[i].adc_ch == channel_adc) { ci = i; break; }
    if (ci >= ADC_CHANNEL_COUNT) return 0;

    for (retry = 0; retry < ADC_SNAPSHOT_RETRY; retry++)
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

    /* 去极值平均: 丢弃一帧最大、一帧最小(偶发毛刺/PWM 开关沿), 对剩余 N-2 帧求均值
     * (四舍五入), 避免被单帧毛刺拉偏。四舍五入加的是"半个除数"(N-2)/2 = 3, 而非
     * N/2 = 4 —— 后者会让余数在 2/6 处就进位, 比正确四舍五入平均偏高 1/6 LSB。 */
    sum -= (uint32_t)vmin + vmax;
    return (uint16_t)((sum + (TEMP_OVERSAMPLE - 2) / 2) / (TEMP_OVERSAMPLE - 2));
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

/* 读取 CPU 内部温度 (°C×10). 走规则组 CH16 缓存, 同 8 帧去极值平均.
 * STM32F1 内部温度传感器: Vsense = adc*3300/4096 (mV), 25°C 时 V25≈1430mV,
 * 正温度系数 ≈4.3mV/°C → T = (Vsense−1430)/4.3 + 25.
 * 定点: T_x10 = 250 + (adc*330000/4096 − 143000)/43.
 * 注意: 绝对精度受 V25 芯片间散布(±0.09V → ±21°C)限制, 冷端补偿应使用
 * 相对变化量(见 work_task 冷端补偿), 而非本函数的绝对读数. */
int16_t AD_GetCpuTempC(void)
{
    int32_t adc   = (int32_t)AD_GetValue(ADC_Channel_16);
    int32_t t_x10 = 250 + (((adc * 330000) / 4096 - 143000) / 43);

    if (t_x10 < 0)      t_x10 = 0;    /* 钳 0~125°C */
    if (t_x10 > 1250)   t_x10 = 1250;
    return (int16_t)t_x10;
}

/* 差分校准后电流 = 导通相位采样 - 关断相位采样 (同一注入组, CC4 逐帧交替相位)
 * 两路都与 PWM 相位严格锁定, 消去运放失调/共模/ADC 偏置, 接近真实工作电流。
 * 导通值被偏置抬高时结果更大 → 过流判断更准；关断 ≥ 导通时钳到 0 防下溢。
 *
 * 新鲜度: 加热全关(PWM CCR=0 → 导通相位触发点为 0)时不产生导通采样, adc_cur
 * 停留在停机前的陈旧值。若直接返回, 断热态会被读成"仍有电流", 过流判据失真。
 * 故先消费新鲜标志: 自上次读取以来无新导通采样即返回 0(物理上无导通→无电流),
 * 有采样则返回差分值。标志为"消费式"(读一次清一次), 每个 PID 周期读一次即够。 */
uint16_t AD_GetCurrent(void)
{
    uint16_t i_off, i_on;
    bool     fresh;
    uint32_t primask;

    /* 读-清须原子: 若注入中断恰在"读标志"与"清标志"之间置位(新导通采样到达),
     * 会把该次置位误清掉 → 本次读成"无电流". 关中断仅数条指令, 主循环上下文可忽略. */
    primask = __get_PRIMASK();
    __disable_irq();
    fresh = adc_cur_fresh;
    adc_cur_fresh = false;
    __set_PRIMASK(primask);

    if (!fresh)
        return 0;

    i_off = adc_cur_off;
    i_on  = adc_cur;
    return (i_on > i_off) ? (i_on - i_off) : 0;
}

/* 注入组 JEOC 中断: 一帧电流采样完成 → 按当前触发相位分派, 再把 CC4 换到另一相位。
 * 相位只由本中断改写(主循环仅读), 故"分派"与"刚触发的那一帧"始终对应。 */
void ADC1_2_IRQHandler(void)
{
    if (ADC_GetITStatus(ADC1, ADC_IT_JEOC) != RESET)
    {
        uint16_t v  = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
        bool     off = PWM_GetInjectPhase();

        if (off)
            adc_cur_off = v;        /* 关断段中点: 关断基准 */
        else
        {
            adc_cur = v;            /* 导通段中点: 导通电流 */
            adc_cur_fresh = true;   /* 标记"有新的导通采样", 供 AD_GetCurrent 判新鲜度 */
        }

        PWM_SetInjectPhase(!off);   /* 下一帧换到另一相位(导通↔关断) */

        ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);
    }
}
