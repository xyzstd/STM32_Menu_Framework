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
} s_buzz;

void Buzzer_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_APB2PeriphClockCmd(BUZZER_GPIO_CLK, ENABLE);

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Pin = BUZZER_GPIO_PIN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BUZZER_GPIO_PORT, &GPIO_InitStruct);

    BUZZER_OFF();
    s_buzz.state = BUZZ_STATE_IDLE;
}

void Buzzer_Beep(buzzer_event_t event)
{
    uint32_t now = Timer_GetTick();

    switch (event)
    {
        case BUZZER_EVENT_TEMP_REACHED:
            s_buzz.state = BUZZ_STATE_ON;
            s_buzz.tick_start = now;
            BUZZER_ON();
            break;

        case BUZZER_EVENT_KEY_CLICK:
            s_buzz.state = BUZZ_STATE_ON;
            s_buzz.tick_start = now;
            BUZZER_ON();
            break;

        case BUZZER_EVENT_ALARM:
            s_buzz.state = BUZZ_STATE_ALARM_ON;
            s_buzz.tick_start = now;
            s_buzz.alarm_count = 0;
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

    switch (s_buzz.state)
    {
        case BUZZ_STATE_ON:
            elapsed = now - s_buzz.tick_start;
            if (elapsed >= BUZZ_ON_MS_NORMAL)
            {
                BUZZER_OFF();
                s_buzz.state = BUZZ_STATE_IDLE;
            }
            break;

        case BUZZ_STATE_ALARM_ON:
            elapsed = now - s_buzz.tick_start;
            if (elapsed >= BUZZ_ALARM_ON_MS)
            {
                BUZZER_OFF();
                s_buzz.state = BUZZ_STATE_ALARM_OFF;
                s_buzz.tick_start = now;
            }
            break;

        case BUZZ_STATE_ALARM_OFF:
            elapsed = now - s_buzz.tick_start;
            if (elapsed >= BUZZ_ALARM_OFF_MS)
            {
                s_buzz.alarm_count++;
                if (s_buzz.alarm_count >= BUZZ_ALARM_REPEAT)
                {
                    s_buzz.state = BUZZ_STATE_IDLE;
                }
                else
                {
                    s_buzz.state = BUZZ_STATE_ALARM_ON;
                    s_buzz.tick_start = now;
                    BUZZER_ON();
                }
            }
            break;

        default:
            break;
    }
}