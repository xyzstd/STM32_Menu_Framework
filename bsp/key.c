#include "stm32f10x.h"
#include "key.h"

static uint8_t is_key_pressed(key_info_t *key);
static void update_timer(key_info_t *key);
static void reset_timer(key_info_t *key);
static void process_key_arm(key_info_t *key);
static void process_key_idle(key_info_t *key);
static void process_key_debounce(key_info_t *key);
static void process_key_pressed(key_info_t *key);
static void process_pressed_release(key_info_t *key);
static void process_key_click_judge(key_info_t *key);


/* 按键状态机(单键), 由 Key_Scan 以固定间隔调用(见 input.c 的 KEY_SCAN_MS = 5ms):
 * 各计时阈值的单位都是"扫描次数" —— KEY_DELAY_TIME=4 即 20ms, KEY_LONG_TIME=120 即
 * 600ms, KEY_ARM_LEVEL_TIME=50 即 250ms. */
void Key_Init(key_info_t *key, GPIO_TypeDef *gpio, uint16_t pin, void (*callback)(key_event_t))
{
    GPIO_InitTypeDef gpio_init_struct;

    if (!key)
        return;
    if (!callback)
        return;

    if (gpio == GPIOA)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    else if (gpio == GPIOB)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    else if (gpio == GPIOC)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    gpio_init_struct.GPIO_Mode  = GPIO_Mode_IPU;
    gpio_init_struct.GPIO_Pin   = pin;
    gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(gpio, &gpio_init_struct);

    key->state = KEY_STATE_ARM;   /* 上电先装填: 等引脚稳定为高, 之后才进入正常按键逻辑 */
    key->timer = 0;
    key->rel_cnt = 0;
    key->gpiox = gpio;
    key->key_pin = pin;
    key->event_callback = callback;
}


static uint8_t is_key_pressed(key_info_t *key)
{
    return !GPIO_ReadInputDataBit(key->gpiox, key->key_pin);
}


/* 定时器管理: 每次扫描递增, 封顶 255 防 uint8_t 溢出 */
static void update_timer(key_info_t *key)
{
    if (key->timer < 255)
        key->timer++;
}

/* 复位定时器 */
static void reset_timer(key_info_t *key)
{
    key->timer = 0;
}

/* 状态机处理函数: 上电装填状态
 * 上电后必须先连续见到稳定高电平(KEY_ARM_LEVEL_TIME 次扫描)才允许进入正常按键逻辑.
 * 消除上电瞬间 PB8 因电源上升、按键线 RC 充电或 EMI 造成的低电平被误确认成一次按下
 * 再释放、从而自发产生 CLICK 的问题(表现为开机自动跳进菜单). 期间读到低电平即重新
 * 计时, 故与 RC 充电时间长短无关. */
static void process_key_arm(key_info_t *key)
{
    update_timer(key);

    if (is_key_pressed(key))   /* 仍为低电平: 继续等待 */
    {
        reset_timer(key);
        return;
    }

    if (key->timer >= KEY_ARM_LEVEL_TIME)
    {
        key->state = KEY_STATE_IDLE;   /* 引脚已稳定为高, 进入正常按键逻辑 */
        reset_timer(key);
    }
}

/* 状态机处理函数: 空闲状态 */
static void process_key_idle(key_info_t *key)
{
    if (is_key_pressed(key))
    {
        key->state = KEY_STATE_DEBOUNCE;   /* 切换到消抖状态 */
        reset_timer(key);
    }
}

/* 状态机处理函数: 消抖状态(按下侧确认, 与释放侧的 rel_cnt 门槛对称) */
static void process_key_debounce(key_info_t *key)
{
    update_timer(key);

    if (key->timer >= KEY_DELAY_TIME)
    {
        if (is_key_pressed(key))
        {
            key->state = KEY_STATE_PRESSED;   /* 确认按下 */
            reset_timer(key);
        }
        else
        {
            key->state = KEY_STATE_IDLE;      /* 抖动: 返回空闲状态 */
            reset_timer(key);
        }
    }
}

/* 状态机处理函数: 按下状态 */
static void process_key_pressed(key_info_t *key)
{
    update_timer(key);

    if (is_key_pressed(key))
    {
        key->rel_cnt = 0;   /* 仍按下: 清零松手确认计数(高电平毛刺被此处吸收) */

        if (key->timer >= KEY_LONG_TIME)   /* 按下时间超过 KEY_LONG_TIME: 判定为长按 */
        {
            key->event_callback(KEY_EVENT_LONG_PRESS);
            key->state = KEY_STATE_WAIT_RELEASE;
        }
    }
    else   /* 读到高电平: 需连续 KEY_DELAY_TIME 次确认才算真松手(与按下侧同款门槛) */
    {
        /* 释放侧此前零消抖: 一次高电平毛刺即判松手 → CLICK_JUDGE 在不看引脚的情况下
         * 立即发 CLICK, 而按键实际仍按着 → 再松手又发一次 CLICK(一次按压两个事件).
         * 改为要求连续 KEY_DELAY_TIME 次高电平; 确认前一直留在本态, timer 继续走,
         * 故长按计时不受影响(这是不复用 timer 做释放计数的原因). */
        key->rel_cnt++;
        if (key->rel_cnt >= KEY_DELAY_TIME)
        {
            key->rel_cnt = 0;
            key->state = KEY_STATE_CLICK_JUDGE;
            reset_timer(key);
        }
    }
}

/* 状态机处理函数: 长按后等待松手(无事件, 仅回空闲) */
static void process_pressed_release(key_info_t *key)
{
    if (!is_key_pressed(key))
    {
        key->state = KEY_STATE_IDLE;
        reset_timer(key);
    }
}

/* 状态机处理函数: 松手后等待单击判定
 * ENABLE_DCLICK=1 时按 KEY_DOUBLE_TIME 区分单击/双击; =0 时本态只看一眼即发 CLICK. */
static void process_key_click_judge(key_info_t *key)
{
#if defined(ENABLE_DCLICK) && ENABLE_DCLICK==1
    update_timer(key);

    if (is_key_pressed(key))   /* 再次按下: 可能是双击 */
    {
        if (key->timer <= KEY_DOUBLE_TIME)
            key->event_callback(KEY_EVENT_DOUBLE_CLICK);
        key->state = KEY_STATE_WAIT_RELEASE;
    }
    else if (key->timer > KEY_DOUBLE_TIME)   /* 超时: 认为是单击 */
    {
#endif
        key->event_callback(KEY_EVENT_CLICK);
        key->state = KEY_STATE_IDLE;
        reset_timer(key);
#if defined(ENABLE_DCLICK) && ENABLE_DCLICK==1
    }
#endif
}

/* 主按键扫描函数: 状态机核心逻辑 */
void Key_Scan(key_info_t *key)
{
    if (!key)
        return;

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

        case KEY_STATE_CLICK_JUDGE:
            process_key_click_judge(key);
            break;

        case KEY_STATE_ARM:
            process_key_arm(key);
            break;

        default:
            break;
    }
}
