#include "heater.h"
#include "pwm.h"
#include "stm32f10x.h"

/*
 * 加热控制模块
 *
 * 通过 TIM1 CH1 (PA8) PWM 输出控制加热芯
 * 功率映射: power(0~1000) → PWM CCR(0~999)
 *   PWM duty = power * 999 / 1000
 * 最大功率 90%: 在此层钳 power ≤ 900, 保留 ≥10% 关断窗供温度采样,
 *   PWM 层是 1000 级分辨率, 全功率窗内粒度完整。
 */

#define HEATER_MAX_POWER_PCT   90
#define HEATER_POWER_LIMIT     (HEATER_POWER_MAX * HEATER_MAX_POWER_PCT / 100) /* 900 */

static struct {
    int16_t power;      /* 当前功率 0~1000 */
    bool enabled;       /* 加热使能 */
} heater;

void Heater_Init(void)
{
    PWM_Init();
    heater.enabled = true;
    PWM_SetCompare1(0);
}

void Heater_SetPower(int16_t power)
{
    uint16_t ccr;

    if (power < 0)
        power = 0;
    if (power > HEATER_POWER_MAX)
        power = HEATER_POWER_MAX;
    /* 90% 上限: 保证 ≥10% 关断窗给温度采样 */
    if (power > HEATER_POWER_LIMIT)
        power = HEATER_POWER_LIMIT;

    heater.power = power;

    if (!heater.enabled)
    {
        PWM_SetCompare1(0);
        return;
    }

    /* 功率 0~1000 → CCR 0~999 (1000 级满刻度) */
    ccr = (uint16_t)((uint32_t)power * PWM_CCR_MAX / HEATER_POWER_MAX);
    PWM_SetCompare1(ccr);
}

int16_t Heater_GetPower(void)
{
    return heater.power;
}

void Heater_Enable(void)
{
    heater.enabled = true;
    /* 恢复之前的功率 */
    Heater_SetPower(heater.power);
}

void Heater_Disable(void)
{
    heater.enabled = false;
    PWM_SetCompare1(0);
}

bool Heater_IsEnabled(void)
{
    return heater.enabled;
}