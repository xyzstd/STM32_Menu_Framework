#include "stm32f10x.h"
#include "key.h"
#include "encoder.h"
#include "menu.h"
#include "ui_task.h"
#include "work_task.h"
#include "timer.h"

#define ENC_MENU_DIV  4

/* 调参消费周期(ms): 扫描仍是 10ms, 但在参数编辑时累计这么久才结算一次
 * 转速并发送步进. 10ms 内人手产生的齿格太少, 每帧结算档位几乎永远落在
 * 最低档, 快慢无法区分; 拉长窗口后累计齿格数才有足够分辨力. */
#define ENC_PARAM_CONSUME_MS  40   /* 4 次 10ms 扫描结算一次 */

static volatile uint32_t input_tick;
static key_info_t key;
static menu_cmd_t menu_cmd;
static int32_t encoder_acc = 0;
static uint32_t param_consume_tick = 0;
static uint8_t last_menu_state = MENU_STATE_NORMAL;


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


/* menu browsing: 每次扫描至多消费一格(if 而非 while), 让高亮每帧(10ms)恰好走一行.
 * 残余齿格留在 encoder_acc, 下一扫描继续消化, 无丢步/跳步. 正转→向下. */
static void encoder_menu_process(void)
{
    encoder_acc += Encoder_GetDiff();

    if (encoder_acc >= ENC_MENU_DIV)
    {
        encoder_acc -= ENC_MENU_DIV;
        menu_notify(MENU_EVENT_DOWN, 1);
    }
    else if (encoder_acc <= -ENC_MENU_DIV)
    {
        encoder_acc += ENC_MENU_DIV;
        menu_notify(MENU_EVENT_UP, 1);
    }
}


/* parameter editing: 按本次扫描消化掉的"4 齿格一组"组数分档.
 * 组数 1→步进1, 2→步进5, ≥3→步进10. */
static int16_t param_step_for_speed(int16_t det)
{
    if (det >= 3)
        return 10;   /* 计数 ≥12 */
    if (det == 2)
        return 5;    /* 计数 8 */
    return 1;        /* 计数 4 */
}

/* parameter editing: 累计齿格, 但把消费(结算转速/发步进)抬高到消费周期一次.
 * while 消化窗口内全部完整 4 齿格组, 避免快速旋转丢步, 也避免停手后拖影漂移.
 * 方向: 菜单向下(正转)时参数向上(增大). */
static void encoder_param_process(void)
{
    int16_t groups = 0;

    encoder_acc += Encoder_GetDiff();

    /* 未到消费周期: 只累计, 不结算 */
    if ((Timer_GetTick() - param_consume_tick) < ENC_PARAM_CONSUME_MS)
        return;

    param_consume_tick = Timer_GetTick();

    /* 消化窗口内全部完整 4 齿格组; 残留(<4)保留到下一周期, 不丢也不翻转符号 */
    while (encoder_acc >= 4 || encoder_acc <= -4)
    {
        if (encoder_acc >= 4)
        {
            encoder_acc -= 4;
            groups++;
        }
        else
        {
            encoder_acc += 4;
            groups--;
        }
    }

    if (!groups)
        return;

    /* 方向与菜单相反: 菜单向下(正转)时参数向上(增大); 步进由速度三档决定 */
    menu_notify((groups > 0) ? MENU_EVENT_UP : MENU_EVENT_DOWN,
                (uint8_t)param_step_for_speed(groups > 0 ? groups : -groups));
}


static void encoder_process(void)
{
    uint8_t state = Menu_GetState();

    /* 模式切换时清零累计/计时, 避免挂起残留 */
    if (state != last_menu_state)
    {
        encoder_acc = 0;
        param_consume_tick = Timer_GetTick();
        last_menu_state = state;
    }

    if (state == MENU_STATE_NORMAL)
        encoder_menu_process();
    else
        encoder_param_process();
}


void Input_Init(void)
{
    Encoder_Init();
    Key_Init(&key, GPIOB, GPIO_Pin_8, on_key_event);
}


void Input_Scan(void)
{
    if (Timer_GetTick() - input_tick >= 10)
    {
        input_tick = Timer_GetTick();

        Key_Scan(&key);
        encoder_process();
    }
}