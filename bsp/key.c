#include "stm32f10x.h"
#include "key.h"

void Key_Init(key_info_t *key, GPIO_TypeDef *gpio, uint16_t pin, void (*callback)(key_event_t))
{
	if(!key)return;
	if(!callback)return;
    if(gpio == GPIOA)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
    }
	else if(gpio == GPIOB)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	}
    else if(gpio == GPIOC)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
    }


    GPIO_InitTypeDef gpio_init_struct;

    gpio_init_struct.GPIO_Mode = GPIO_Mode_IPU;
    gpio_init_struct.GPIO_Pin = pin;
    gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;


    GPIO_Init(gpio,&gpio_init_struct);


    key->state = KEY_STATE_IDLE;
    key->timer = 0;
    key->gpiox = gpio;
    key->key_pin = pin;
    key->event_callback = callback;
}


static uint8_t is_key_pressed(key_info_t *key)
{
    return !GPIO_ReadInputDataBit(
        key->gpiox,
        key->key_pin
    );
}

// 定时器管理函数
static void update_timer(key_info_t *key)
{
    if (key->timer < 255) // 避免溢出
    {
        key->timer++;
    }
}

// 复位定时器
static void reset_timer(key_info_t *key)
{
    key->timer = 0;
}

// 状态机处理函数：空闲状态
static void process_key_idle(key_info_t *key)
{
    if (is_key_pressed(key))
    {
        key->state = KEY_STATE_DEBOUNCE; // 切换到消抖状态
        reset_timer(key);
    }
}

// 状态机处理函数：消抖状态
static void process_key_debounce(key_info_t *key)
{
    update_timer(key);
    if (key->timer >= KEY_DELAY_TIME)
    {
        if (is_key_pressed(key))
        {
            key->state = KEY_STATE_PRESSED; // 切换到按下状态
            reset_timer(key);
        }
        else
        {
            key->state = KEY_STATE_IDLE; // 返回空闲状态
            reset_timer(key);
        }
    }
}

// 状态机处理函数：按下状态
static void process_key_pressed(key_info_t *key)
{
    update_timer(key);
    if(is_key_pressed(key)){
            // 释放后判断按下时间是否长于 KEY_LONG_TIME
        if (key->timer >= KEY_LONG_TIME)
        {
            if(key->event_callback)
                key->event_callback(KEY_EVENT_LONG_PRESS); // 调用长按事件回调
            key->state = KEY_STATE_WAIT_RELEASE;
        }
    }
    else if (!is_key_pressed(key)) // 按键释放
    {
        key->state = KEY_STATE_WAIT_DOUBLE; // 切换到等待释放状态
        reset_timer(key);
    }
}
// 状态机处理函数：等待释放状态
static void process_pressed_release(key_info_t *key)
{
        if(!is_key_pressed(key))
        {
            key->state = KEY_STATE_IDLE; // 返回空闲状态
            reset_timer(key);
        }
}

// 状态机处理函数：判断是否双击
static void process_key_wait_double(key_info_t *key)
{
#if defined(ENABLE_DCLICK) && ENABLE_DCLICK==1
    update_timer(key);

    if (is_key_pressed(key)) // 检测到再次按下，可能是双击
    {
        if (key->timer <= KEY_DOUBLE_TIME)
        {
            if(key->event_callback)
                key->event_callback(KEY_EVENT_DOUBLE_CLICK); // 调用双击事件回调'
        }
        key->state = KEY_STATE_WAIT_RELEASE;
    }
    else if (key->timer > KEY_DOUBLE_TIME) // 超时，认为是单击
    {
#endif
		if(key->event_callback)
            key->event_callback(KEY_EVENT_CLICK); // 调用单击事件回调
        key->state = KEY_STATE_IDLE;
#if defined(ENABLE_DCLICK) && ENABLE_DCLICK==1	
    }
#endif
}

// 主按键扫描函数，状态机核心逻辑
void Key_Scan(key_info_t *key)
{
    if(!key)return;
    
    switch (key->state)
    {
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
        process_pressed_release(key);
        break;
    
    case KEY_STATE_WAIT_DOUBLE:
        process_key_wait_double(key);
        break;
    default:
        break;
    }
}
