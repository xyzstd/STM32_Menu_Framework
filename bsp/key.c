#include "stm32f10x.h"
#include <stdbool.h>
#include "key.h"


static inline bool key_is_down(const key_info_t *key)
{
    return GPIO_ReadInputDataBit(key->gpiox, key->key_pin) == Bit_RESET;
}


static inline bool ticks_reached(uint8_t *ticks, uint8_t target)
{
    if(*ticks < target)
        (*ticks)++;

    return *ticks >= target;
}


void Key_Init(key_info_t *key, GPIO_TypeDef *gpio, uint16_t pin, void (*event_cb)(key_event_t))
{
    GPIO_InitTypeDef gpio_init_struct;

    if(!key || !event_cb)
        return;

    if(gpio == GPIOA)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    else if(gpio == GPIOB)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    else if(gpio == GPIOC)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    gpio_init_struct.GPIO_Mode  = GPIO_Mode_IPU;
    gpio_init_struct.GPIO_Pin   = pin;
    gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(gpio, &gpio_init_struct);

    key->state = KEY_STATE_ARM;
    key->stable = 0;
    key->hold = 0;
    key->gpiox = gpio;
    key->key_pin = pin;
    key->event_cb = event_cb;
}


void Key_Scan(key_info_t *key)
{
    bool down;

    if(!key)
        return;

    down = key_is_down(key);

    switch(key->state)
    {
        case KEY_STATE_ARM:
            if(down)
            {
                key->stable = 0;
                break;
            }
            if(ticks_reached(&key->stable, KEY_ARM_TICKS))
            {
                key->stable = 0;
                key->state = KEY_STATE_RELEASED;
            }
            break;

        case KEY_STATE_RELEASED:
            if(!down)
            {
                key->stable = 0;
                break;
            }
            if(ticks_reached(&key->stable, KEY_PRESS_TICKS))
            {
                key->stable = 0;
                key->hold = 0;
                key->state = KEY_STATE_PRESSED;
            }
            break;

        case KEY_STATE_PRESSED:
            if(down)
            {
                key->stable = 0;

                if(ticks_reached(&key->hold, KEY_LONG_TICKS))
                {
                    key->state = KEY_STATE_WAIT_RELEASE;
                    key->event_cb(KEY_EVENT_LONG_PRESS);
                }
                break;
            }
            if(ticks_reached(&key->stable, KEY_RELEASE_TICKS))
            {
                key->stable = 0;
                key->state = KEY_STATE_RELEASED;
                key->event_cb(KEY_EVENT_CLICK);
            }
            break;

        case KEY_STATE_WAIT_RELEASE:
            if(down)
            {
                key->stable = 0;
                break;
            }
            if(ticks_reached(&key->stable, KEY_RELEASE_TICKS))
            {
                key->stable = 0;
                key->state = KEY_STATE_RELEASED;
            }
            break;

        default:
            key->stable = 0;
            key->state = KEY_STATE_ARM;
            break;
    }
}
