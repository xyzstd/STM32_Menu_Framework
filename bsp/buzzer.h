#ifndef __BUZZER_H
#define __BUZZER_H

#include <stdint.h>
#include <stdbool.h>

/*
 * 蜂鸣器驱动
 *
 * 使用 GPIO 驱动有源蜂鸣器, 通过 TIM 定时器输出 PWM 或 GPIO 翻转。
 * 默认引脚: PB0 (可根据硬件修改)
 *
 * 引脚定义:
 *   BUZZER_GPIO_PORT   - GPIOB
 *   BUZZER_GPIO_PIN    - GPIO_Pin_0
 *   BUZZER_GPIO_CLK    - RCC_APB2Periph_GPIOB
 */

/* 蜂鸣器触发事件 */
typedef enum {
    BUZZER_EVENT_TEMP_REACHED,   /* 温度到达 */
    BUZZER_EVENT_KEY_CLICK,      /* 按键点击 */
    BUZZER_EVENT_ALARM,          /* 报警 */
} buzzer_event_t;

/* 初始化蜂鸣器 */
void Buzzer_Init(void);

/* 静音开关: 由应用层在配置加载/变更时下发(驱动不读应用配置).
 * 关闭时立即停止当前鸣叫, 并忽略后续一切蜂鸣(含报警). */
void Buzzer_SetEnabled(bool en);

/* 触发蜂鸣器事件 */
void Buzzer_Beep(buzzer_event_t event);

/* 在定时器中断或主循环中周期性调用, 处理蜂鸣器状态机 */
void Buzzer_Process(void);

#endif