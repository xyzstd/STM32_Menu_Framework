#include "stm32f10x.h"
#include "key.h"
#include "encoder.h"
#include "menu.h"
#include "ui_task.h"
#include "work_task.h"
#include "timer.h"

/* 一组 = 4 个硬件齿格: 每次扫描至多消费一组(一格), 残余兼容累计续攒.
 * 菜单导航与参数编辑共用同一套消费逻辑(无分档/窗口结算之分). */
#define ENC_GROUP_TICKS  4

static volatile uint32_t input_tick;

static key_info_t key;
static menu_cmd_t menu_cmd;

static int32_t encoder_acc = 0;


/* 统一事件出口: 唤醒 + 组装命令 + 分发 + 请求刷新 */
static void menu_notify(uint8_t event, uint8_t step)
{
    WorkTask_MarkActivity();
    menu_cmd.event = event;
    menu_cmd.step  = step;
    Menu_HandleEvent(&menu_cmd);
    UI_RequestRefresh();
}

static void on_key_event(key_event_t evt)
{
    if (evt == KEY_EVENT_CLICK)
        menu_notify(MENU_EVENT_CLICK, 0);
    else if (evt == KEY_EVENT_LONG_PRESS)
        menu_notify(MENU_EVENT_LPRESS, 0);
}


/* 换向感知累加: 当本次位移 diff 与残余 encoder_acc 异号时, 说明正在改变旋转方向.
 * 此时丢弃原方向的少量未消费残余, 从本次净位移重新起算, 反立即响应,
 * 避免"先还旧账"导致的换向滞后/丢步(菜单导航与参数调节共用). */
static int16_t feed_axis(int16_t acc_now)
{
    int16_t diff = Encoder_GetDiff();

    if (diff != 0 && ((acc_now > 0 && diff < 0) || (acc_now < 0 && diff > 0)))
        return diff;              /* 换向: 丢弃旧残余, 从本次位移全新起算 */
    return acc_now + diff;        /* 同向/静止: 正常累加 */
}

/* 编码器统一处理: 菜单与参数共用同一套"逐组消费"逻辑(无分档/窗口结算之分).
 * 每次扫描至多消费一组(if 而非 while), 残余留在 encoder_acc 下一帧续攒,
 * 高速时逐帧接力消化, 任何转速都不丢齿不跳步; 状态切换不触碰 acc,
 * 切换瞬间已转的齿格原样保留给下一状态消费, 对称统一.
 * 方向语义: 菜单正转→下移(DOWN), 参数正转→增大(UP), 故意相反. */
static void encoder_process(void)
{
    uint8_t state = Menu_GetState();

    encoder_acc = feed_axis(encoder_acc);   /* 硬件齿格持续累计; 换向丢弃旧残余, 反立响应 */

    if (encoder_acc >= ENC_GROUP_TICKS)
    {
        encoder_acc -= ENC_GROUP_TICKS;
        menu_notify(state == MENU_STATE_NORMAL ? MENU_EVENT_DOWN : MENU_EVENT_UP, 1);
    }
    else if (encoder_acc <= -ENC_GROUP_TICKS)
    {
        encoder_acc += ENC_GROUP_TICKS;
        menu_notify(state == MENU_STATE_NORMAL ? MENU_EVENT_UP : MENU_EVENT_DOWN, 1);
    }
}


void Input_Init(void)
{
    Encoder_Init();
    Key_Init(&key, GPIOB, GPIO_Pin_8, on_key_event);
}


void Input_Scan(void)
{
	static uint8_t cnt1 = 0;
	static uint8_t cnt2 = 0;
    if (Timer_GetTick() - input_tick >= 5)
    {
        input_tick = Timer_GetTick();
		cnt1++;
		cnt2++;
		if(cnt1>=2)
		{
			cnt1=0;
			Key_Scan(&key);
		}
		if(cnt2>=3)
		{
			cnt2=0;
			encoder_process();
		}
    }
}