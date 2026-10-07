#include "buzzer.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "timer.h"

/*
 * 蜂鸣器驱动实现
 *
 * 使用 GPIO 翻转驱动有源蜂鸣器。
 * 通过状态机管理不同事件的蜂鸣模式:
 *   - 温度到达: 短鸣 200ms
 *   - 按键点击: 短鸣 50ms
 *   - 报警: 连续鸣叫 500ms on / 500ms off, 重复3次
 */

/* ========== 硬件引脚配置 ========== */
#define BUZZER_GPIO_PORT    GPIOB
#define BUZZER_GPIO_PIN     GPIO_Pin_0
#define BUZZER_GPIO_CLK     RCC_APB2Periph_GPIOB

/* 蜂鸣器 on/off */
#define BUZZER_ON()    GPIO_SetBits(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN)
#define BUZZER_OFF()   GPIO_ResetBits(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN)

/* ========== 蜂鸣模式参数 ========== */
#define BUZZ_ON_MS_SHORT   50    /* 按键点击: 50ms */
#define BUZZ_ON_MS_NORMAL  200   /* 温度到达: 200ms */
#define BUZZ_ALARM_ON_MS   500   /* 报警: 500ms on */
#define BUZZ_ALARM_OFF_MS  500   /* 报警: 500ms off */
#define BUZZ_ALARM_REPEAT  3     /* 报警重复次数 */

/* ========== 状态机 ========== */

typedef enum {
    BUZZ_STATE_IDLE,          /* 空闲 */
    BUZZ_STATE_ON,            /* 正在鸣叫 */
    BUZZ_STATE_ALARM_ON,      /* 报警鸣叫 */
    BUZZ_STATE_ALARM_OFF,     /* 报警静音 */
} buzz_state_t;

static struct {
    buzz_state_t state;
    uint32_t     tick_start;   /* 进入当前状态的时间戳 */
    uint8_t      alarm_count;  /* 报警重复计数 */
    uint8_t      on_ms;        /* 本次 ON 鸣叫时长 (温度到达/按键点击不同) */
    bool         enabled;      /* 静音开关, 由 Buzzer_SetEnabled 下发 */
} buzz;

void Buzzer_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_APB2PeriphClockCmd(BUZZER_GPIO_CLK, ENABLE);

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Pin = BUZZER_GPIO_PIN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BUZZER_GPIO_PORT, &GPIO_InitStruct);

    BUZZER_OFF();
    buzz.state = BUZZ_STATE_IDLE;
    buzz.enabled = true;      /* 默认允许鸣叫 */
}

/* 静音开关: 由应用层在配置加载/变更时下发.
 * 原实现直接读 g_config.buzzer_enable, 使驱动依赖应用配置模块(config.h);
 * 改为被动接收一个布尔量后, 本驱动只依赖 GPIO 与 tick, 可独立使用/测试. */
void Buzzer_SetEnabled(bool en)
{
    buzz.enabled = en;
    if (!en)
    {
        BUZZER_OFF();                    /* 立即停止当前鸣叫 */
        buzz.state = BUZZ_STATE_IDLE;
    }
}

void Buzzer_Beep(buzzer_event_t event)
{
    /* 静音开关集中判断: 关闭时忽略一切蜂鸣(含报警), 调用方无需重复判断 */
    if (!buzz.enabled)
        return;

    /* 报警进行中忽略普通提示(按键滴声/温度到达), 防止报警段被截断;
     * 报警事件本身仍可覆盖重新报警 */
    if ((buzz.state == BUZZ_STATE_ALARM_ON || buzz.state == BUZZ_STATE_ALARM_OFF) &&
        event != BUZZER_EVENT_ALARM)
        return;

    uint32_t now = Timer_GetTick();

    switch (event)
    {
        case BUZZER_EVENT_TEMP_REACHED:
            buzz.state = BUZZ_STATE_ON;
            buzz.on_ms = BUZZ_ON_MS_NORMAL;   /* 温度到达: 长鸣 200ms */
            buzz.tick_start = now;
            BUZZER_ON();
            break;

        case BUZZER_EVENT_KEY_CLICK:
            buzz.state = BUZZ_STATE_ON;
            buzz.on_ms = BUZZ_ON_MS_SHORT;    /* 按键点击: 短鸣 50ms */
            buzz.tick_start = now;
            BUZZER_ON();
            break;

        case BUZZER_EVENT_ALARM:
            buzz.state = BUZZ_STATE_ALARM_ON;
            buzz.alarm_count = 0;
            buzz.tick_start = now;
            BUZZER_ON();
            break;

        default:
            break;
    }
}

void Buzzer_Process(void)
{
    uint32_t now = Timer_GetTick();
    uint32_t elapsed;

    switch (buzz.state)
    {
        case BUZZ_STATE_ON:
            elapsed = now - buzz.tick_start;
            if (elapsed >= buzz.on_ms)
            {
                BUZZER_OFF();
                buzz.state = BUZZ_STATE_IDLE;
            }
            break;

        case BUZZ_STATE_ALARM_ON:
            elapsed = now - buzz.tick_start;
            if (elapsed >= BUZZ_ALARM_ON_MS)
            {
                BUZZER_OFF();
                buzz.state = BUZZ_STATE_ALARM_OFF;
                buzz.tick_start = now;
            }
            break;

        case BUZZ_STATE_ALARM_OFF:
            elapsed = now - buzz.tick_start;
            if (elapsed >= BUZZ_ALARM_OFF_MS)
            {
                buzz.alarm_count++;
                if (buzz.alarm_count >= BUZZ_ALARM_REPEAT)
                {
                    buzz.state = BUZZ_STATE_IDLE;
                }
                else
                {
                    buzz.state = BUZZ_STATE_ALARM_ON;
                    buzz.tick_start = now;
                    BUZZER_ON();
                }
            }
            break;

        default:
            break;
    }
}