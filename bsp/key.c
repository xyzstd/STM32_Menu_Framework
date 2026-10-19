#include "stm32f10x.h"
#include <stdbool.h>
#include "key.h"


void Key_Init(key_info_t *key, GPIO_TypeDef *gpio, uint16_t pin, void (*callback)(key_event_t))
{
    GPIO_InitTypeDef gpio_init_struct;

    if(!key)
        return;
    if(!callback)
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
    key->timer = 0;
    key->rel_cnt = 0;
    key->gpiox = gpio;
    key->key_pin = pin;
    key->event_callback = callback;
}


static bool is_key_pressed(key_info_t *key)
{
    return !GPIO_ReadInputDataBit(key->gpiox, key->key_pin);
}


static void update_timer(key_info_t *key)
{
    if(key->timer < 255)
        key->timer++;
}

static void reset_timer(key_info_t *key)
{
    key->timer = 0;
}


static void process_key_arm(key_info_t *key)
{
    update_timer(key);

    if(is_key_pressed(key))
    {
        reset_timer(key);
        return;
    }

    if(key->timer >= KEY_ARM_LEVEL_TIME)
        key->state = KEY_STATE_IDLE;
}

static void process_key_idle(key_info_t *key)
{
    if(is_key_pressed(key))
    {
        key->state = KEY_STATE_DEBOUNCE;
        reset_timer(key);
    }
}

static void process_key_debounce(key_info_t *key)
{
    if(is_key_pressed(key))
    {
        update_timer(key);

        if(key->timer >= KEY_DELAY_TIME)
        {
            key->state = KEY_STATE_PRESSED;
            reset_timer(key);
        }
    }
    else
    {
        key->state = KEY_STATE_IDLE;
    }
}


static void process_key_pressed(key_info_t *key)
{
    update_timer(key);

    if(is_key_pressed(key))
    {
        key->rel_cnt = 0;

        if(key->timer >= KEY_LONG_TIME)
        {
            key->event_callback(KEY_EVENT_LONG_PRESS);
            key->rel_cnt = 0;
            key->state = KEY_STATE_WAIT_RELEASE;
        }
    }
    else
    {
        key->rel_cnt++;
        if(key->rel_cnt >= KEY_DELAY_TIME+1)
        {
            key->rel_cnt = 0;
            key->state = KEY_STATE_CLICK_JUDGE;
            reset_timer(key);
        }
    }
}

static void process_key_wait_release(key_info_t *key)
{
    if(!is_key_pressed(key))
    {
        key->rel_cnt++;
        if(key->rel_cnt >= KEY_DELAY_TIME)
        {
            key->rel_cnt = 0;
            key->state = KEY_STATE_IDLE;
        }
    }
    else
    {
        key->rel_cnt = 0;
    }
}


static void process_key_click_judge(key_info_t *key)
{
#if ENABLE_DCLICK
    update_timer(key);

    if(is_key_pressed(key))
    {
        if(key->timer <= KEY_DOUBLE_TIME)
            key->event_callback(KEY_EVENT_DOUBLE_CLICK);
        key->state = KEY_STATE_WAIT_RELEASE;
        return;
    }

    if(key->timer <= KEY_DOUBLE_TIME)
        return;
#endif
    key->event_callback(KEY_EVENT_CLICK);
    key->state = KEY_STATE_IDLE;
}


void Key_Scan(key_info_t *key)
{
    if(!key)
        return;

    switch(key->state)
    {
        case KEY_STATE_ARM:
            process_key_arm(key);
            break;

        case KEY_STATE_IDLE:
            process_key_idle(key);
            break;

        case KEY_STATE_DEBOUNCE:
            process_key_debounce(key);
            break;

        case KEY_STATE_PRESSED:
            process_key_pressed(key);
            break;

        case KEY_STATE_WAIT_RELEASE:
            process_key_wait_release(key);
            break;

        case KEY_STATE_CLICK_JUDGE:
            process_key_click_judge(key);
            break;

        default:
            break;
    }
}
