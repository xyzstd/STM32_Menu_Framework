#ifndef __KEY_H
#define __KEY_H

#include "stm32f10x.h"
#include <stdint.h>

#define ENABLE_DCLICK 0

/* 计时阈值(单位: 扫描次数, 每 5ms 一次 —— 见 input.c 的 KEY_SCAN_MS) */
enum{
    KEY_DELAY_TIME     = 4,    /* 20ms: 按下/松手确认门槛 */
    KEY_DOUBLE_TIME    = 0,    /* 双击判定窗口(ENABLE_DCLICK=0 时不用) */
    KEY_LONG_TIME      = 80,  /* 400ms: 长按判定 */
    KEY_ARM_LEVEL_TIME = 30    /* 150ms: 上电装填需连续见到的稳定高电平次数 */
};

/* 按键事件类型 */
typedef enum {
    KEY_EVENT_NONE,                /* 无事件 */
    KEY_EVENT_CLICK,               /* 单击事件 */
    KEY_EVENT_DOUBLE_CLICK,        /* 双击事件 */
    KEY_EVENT_LONG_PRESS           /* 长按事件 */
} key_event_t;

/* 按键状态 */
typedef enum {
    KEY_STATE_ARM,                 /* 上电装填状态(等待引脚稳定为高) */
    KEY_STATE_IDLE,                /* 空闲状态 */
    KEY_STATE_DEBOUNCE,            /* 消抖状态(按下侧确认) */
    KEY_STATE_PRESSED,             /* 按下状态 */
    KEY_STATE_CLICK_JUDGE,         /* 松手后等待单击判定(ENABLE_DCLICK=0 时立即发 CLICK) */
    KEY_STATE_WAIT_RELEASE,        /* 长按后等待松手(仅回 IDLE, 无事件) */
} key_state_t;

/* 按键结构体, 存储按键状态及相关变量 */
typedef struct {
    void (*event_callback)(key_event_t event); /* 事件回调函数 */
    GPIO_TypeDef *gpiox;                       /* 按键所在 GPIO */
    /* 按键引脚: 存 **GPIO_Pin_x 掩码**(如 GPIO_Pin_8 = 0x0100), 必须 16 位.
     * 不可改成 uint8_t: 只有 GPIO_Pin_0..7(0x0001..0x0080) 装得下, Pin8~15
     * (0x0100..0x8000) 会被截断成 0 → GPIO_Init 不配置任何引脚, 且读回恒为低,
     * 表现为"按键一直按下". 全工程其它 GPIO 用法也都是掩码, 保持一致. */
    uint16_t key_pin;
    key_state_t state;                         /* 当前按键状态 */
    uint8_t timer;                             /* 计时器(扫描次数) */
    uint8_t rel_cnt;                           /* 释放侧连续高电平计数(松手确认) */
} key_info_t;

/* 函数声明 */
void Key_Init(key_info_t *key, GPIO_TypeDef *gpio, uint16_t pin, void (*callback)(key_event_t));
void Key_Scan(key_info_t *key);

#endif
