#ifndef __KEY_H
#define __KEY_H

#include "stm32f10x.h"
#include <stdint.h>

#define ENABLE_DCLICK 0

enum{
    KEY_DELAY_TIME = 2,
    KEY_DOUBLE_TIME = 0,
    KEY_LONG_TIME = 40
};

// 按键事件类型
typedef enum {
    KEY_EVENT_NONE,              // 无事件
    KEY_EVENT_CLICK,             // 单击事件
    KEY_EVENT_DOUBLE_CLICK,      // 双击事件
    KEY_EVENT_LONG_PRESS         // 长按事件
} key_event_t;

// 按键状态
typedef enum {
    KEY_STATE_IDLE,              // 空闲状态
    KEY_STATE_DEBOUNCE,          // 消抖状态
    KEY_STATE_PRESSED,           // 按下状态
    KEY_STATE_WAIT_DOUBLE,   // 等待释放状态
    KEY_STATE_WAIT_RELEASE
} key_state_t;

// 按键结构体，存储按键状态及相关变量
typedef struct {
    void (*event_callback)(key_event_t event); // 事件回调函数
	GPIO_TypeDef * gpiox;
	uint16_t key_pin;       // 按键引脚
	key_state_t state;           // 当前按键状态
	uint8_t timer;         // 计时器
} key_info_t;

// 函数声明
void Key_Init(key_info_t *key,GPIO_TypeDef *gpio, uint16_t pin, void (*callback)(key_event_t));
void Key_Scan(key_info_t *key);

#endif
