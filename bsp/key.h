#ifndef __KEY_H
#define __KEY_H

#include "stm32f10x.h"
#include <stdint.h>


#define KEY_SCAN_MS      5
/* All thresholds are in scan ticks; one tick = KEY_SCAN_MS (5 ms), which input.c
 * uses as the Key_Scan() period.
 * They count CONSECUTIVE samples agreeing with the level being waited for, so
 * contact bounce shorter than the threshold is discarded by construction. */
enum{
    KEY_PRESS_TICKS   = 5,    /* 25 ms of low level  -> press confirmed */
    KEY_RELEASE_TICKS = 5,    /* 25 ms of high level -> release confirmed */
    KEY_LONG_TICKS    = 80,   /* 400 ms held (from the confirmed press) -> LONG_PRESS */
    KEY_ARM_TICKS     = 40    /* 200 ms of high level before the key is armed */
};

/* Key events. NONE is never emitted; it only keeps a zeroed instance harmless. */
typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_EVENT_CLICK,        /* pressed then released before KEY_LONG_TICKS */
    KEY_EVENT_LONG_PRESS    /* held for KEY_LONG_TICKS; the release is swallowed */
} key_event_t;

/* Key states. Both edges are confirmed before anything is reported, so the
 * consumer never sees bounce, and each state does exactly one job. */
typedef enum {
    KEY_STATE_ARM = 0,      /* power-on: a key held through reset must not count */
    KEY_STATE_RELEASED,     /* released, waiting for a press */
    KEY_STATE_PRESSED,      /* press confirmed, measuring hold time */
    KEY_STATE_WAIT_RELEASE  /* long press reported, waiting for the release */
} key_state_t;

/* Key instance */
typedef struct {
    void (*event_cb)(key_event_t event);   /* event callback */
    GPIO_TypeDef *gpiox;                   /* GPIO port of the key */
    uint16_t key_pin;                      /* pin mask, GPIO_Pin_x */
    key_state_t state;                     /* current state */
    uint8_t stable;                        /* consecutive samples matching the level being waited for */
    uint8_t hold;                          /* samples since the press was confirmed */
} key_info_t;

/* API */
void Key_Init(key_info_t *key, GPIO_TypeDef *gpio, uint16_t pin, void (*event_cb)(key_event_t));
void Key_Scan(key_info_t *key);

#endif
