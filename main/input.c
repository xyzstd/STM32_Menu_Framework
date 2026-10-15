#include "stm32f10x.h"
#include "key.h"
#include "encoder.h"
#include "input.h"
#include "menu.h"
#include "ui_task.h"
#include "work_task.h"
#include "timer.h"

/* 单次 4ms 编码器取样的物理合理上限。超过时把整批当作毛刺丢弃，而不把噪声
 * 截成若干合法步送入 UI，避免一次异常计数导致突然跳格。 */
#define ENC_MAX_EVENTS_PER_POLL  4

/* 采样分频(ms): 定时器节拍 1ms. */
#define KEY_SCAN_MS      5
#define ENC_POLL_MS      4

/* 单队列 SPSC：保留末尾槽位给按键事件；旋转事件达到阈值就丢弃最新值。
 * 旋转事件另带时间戳，主循环阻塞恢复后会丢弃过期旋转，避免陈旧动作突然补执行。 */
#define INPUT_EVT_QUEUE_SIZE       32u
#define INPUT_EVT_QUEUE_MASK       (INPUT_EVT_QUEUE_SIZE - 1u)
#define INPUT_KEY_RESERVED_SLOTS    4u
#define INPUT_ENCODER_MAX_AGE_MS   12u

/* 队列头尾用 uint8_t, 空槽法需留 1 格; 掩码求占用量要求容量为 2 的幂. */
typedef char input_evt_queue_size_must_be_valid[
    (INPUT_EVT_QUEUE_SIZE >= 2u &&
     INPUT_EVT_QUEUE_SIZE <  256u &&   /* uint8_t head/tail 无法表示容量 256 */
     (INPUT_EVT_QUEUE_SIZE & (INPUT_EVT_QUEUE_SIZE - 1u)) == 0u &&
     INPUT_EVT_QUEUE_SIZE > INPUT_KEY_RESERVED_SLOTS + 1u) ? 1 : -1];

typedef struct {
    uint16_t tick16;   /* Timer_GetTick 低 16 位; 事件年龄远小于 32768ms */
    int8_t   step;     /* 当前队列仅存 -1/0/+1 */
    uint8_t  event;
} input_evt_t;

typedef char input_evt_size_must_be_4[(sizeof(input_evt_t) == 4) ? 1 : -1];

static key_info_t key;

/* 1ms 节拍与分频计数: 仅由 input_scan 在 TIM2 中断内读写(单上下文, 故无需 volatile).
 * scan_tick 为自由运行的毫秒节拍, 差值比较天然抗 2^32 回绕. */
static uint32_t scan_tick = 0;   /* 1ms 节拍 */
static uint32_t key_tick  = 0;   /* 上次按键扫描时刻 */
static uint32_t enc_tick  = 0;   /* 上次编码器取样时刻 */


/* ==================== 事件队列 (SPSC 环形缓冲) ====================
 * 中断内只做采样入队(evt_enqueue), 主循环 Input_Process 出队执行菜单逻辑,
 * 把 Flash 保存等耗时操作整体挪出 1ms 定时器中断, 输入节拍不再被渲染/落盘拖慢.
 * 并发安全: 单生产者(中断)单消费者(主循环), head 仅消费者改, tail 仅生产者改,
 * 空 = head==tail, 满 = tail+1==head, 无锁; 消费者复制期间生产者不写该位置. */
static struct{
    input_evt_t buf[INPUT_EVT_QUEUE_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
}evt_q;

static void evt_enqueue(uint8_t event, int8_t step);
static bool evt_dequeue(input_evt_t *out);

static void menu_notify(menu_cmd_t *cmd);

static uint16_t menu_ctx(void);

static void on_key_event(key_event_t evt);

static void encoder_process(void);

static void input_scan(void);


static void evt_enqueue(uint8_t event, int8_t step)
{
    uint8_t next = (uint8_t)((evt_q.tail + 1u) % INPUT_EVT_QUEUE_SIZE);
    uint8_t used = (uint8_t)((uint8_t)(evt_q.tail - evt_q.head) & INPUT_EVT_QUEUE_MASK);
    bool is_key_event = (event == MENU_EVENT_CLICK || event == MENU_EVENT_LPRESS);

    /* 编码器不能占用预留槽: 主循环短时被 Flash/测量流程阻塞时，按键仍有位置入队. */
    if (!is_key_event && used >= INPUT_EVT_QUEUE_SIZE - 1 - INPUT_KEY_RESERVED_SLOTS)
        return;

    if (next == evt_q.head)
        return;   /* 极端情况下连按键预留槽也耗尽: 丢弃新事件，不改写消费者队列内容 */

    evt_q.buf[evt_q.tail].tick16 = (uint16_t)Timer_GetTick();
    evt_q.buf[evt_q.tail].step   = (int8_t)step;
    evt_q.buf[evt_q.tail].event  = event;
    evt_q.tail = next;
}

static bool evt_dequeue(input_evt_t *out)
{
    if(evt_q.head == evt_q.tail)
        return false;

    *out = evt_q.buf[evt_q.head];
    evt_q.head = (uint8_t)((evt_q.head + 1) % INPUT_EVT_QUEUE_SIZE);
    return true;
}


/* 事件出口(主循环上下文): 唤醒 + 方向映射 + 分发 + 请求刷新.
 * 编码器入队语义为原始方向: 正向=UP(增大), 反向=DOWN. 消费时映射:
 *   - 列表导航(NORMAL): 反转, 正转→DOWN(菜单下移, 原语义);
 *   - 页面态(HANDLER, 含列表内就地调节项): 原样, 正转=UP=增大. */
static void menu_notify(menu_cmd_t *cmd)
{
    uint8_t event = cmd->event;

    WorkTask_MarkActivity();

    if(Menu_GetState() == MENU_STATE_NORMAL &&
       (event == MENU_EVENT_UP || event == MENU_EVENT_DOWN))
        cmd->event = (event == MENU_EVENT_UP) ? MENU_EVENT_DOWN : MENU_EVENT_UP;

    Menu_EventDispatch(cmd);
    UI_RequestRefresh();
}


static void on_key_event(key_event_t evt)
{
    if(evt == KEY_EVENT_CLICK)
        evt_enqueue(MENU_EVENT_CLICK, 0);
    else if(evt == KEY_EVENT_LONG_PRESS)
        evt_enqueue(MENU_EVENT_LPRESS, 0);
}


/* 菜单"上下文"指纹: 高字节 = 内核状态(列表/页面), 低字节 = 页面或列表身份.
 *   - HANDLER 态: 身份 = 当前页面 id;
 *   - NORMAL  态: 身份 = 当前选中项的父级(同一列表内移动选择项, 身份不变).
 * 指纹变化即代表发生了跨页/跨列表跳转, 此时让编码器重同步(丢弃未满一格的残留
 * 相位), 避免旧上下文的相位被带入新页面 —— 否则残留与新转向反号时会吃掉新页面
 * 第一格(表现为"进子菜单/返回后第一格不动, 要再转一格"). */
static uint16_t menu_ctx(void)
{
    uint8_t id = Menu_GetCurrentId();

    if (Menu_GetState() == MENU_STATE_NORMAL)
    {
        const menu_item_t *item = Menu_GetItem(id);
        id = (item != NULL) ? item->parent : 0xFFU;   /* 同一列表 = 共同父级 */
        return (uint16_t)((uint16_t)MENU_STATE_NORMAL << 8 | id);
    }

    return (uint16_t)((uint16_t)MENU_STATE_HANDLER << 8 | id);
}


/* 编码器取样(1ms 中断内, 每 4ms 真正取一次): 取走 bsp/encoder.c 累计的步数, 每步入队
 * 一个方向事件. 正交计数、硬件去毛刺、以及"每格一步"的折算都在 encoder.c 完成,
 * 此处只做"步 → 事件"的摊平, 不再持有任何位置/相位状态.
 * 入队语义: 正向=UP(增大), 反向=DOWN; 具体去向(列表下移/参数增减)由消费端状态决定. */
static void encoder_process(void)
{
    int16_t n = Encoder_GetSteps();
    int16_t i;

    if (n == 0)
        return;

    /* 4ms 内出现超过物理合理上限的步数视为计数毛刺, 整批丢弃。
     * 旧逻辑把大脉冲截成 20 个合法事件, 仍会把明显异常变成一次大幅跳格. */
    if (n > ENC_MAX_EVENTS_PER_POLL || n < -ENC_MAX_EVENTS_PER_POLL)
        return;

    for (i = 0; i < n; i++)
        evt_enqueue(MENU_EVENT_UP, 1);
    for (i = 0; i > n; i--)
        evt_enqueue(MENU_EVENT_DOWN, 1);
}


void Input_Init(void)
{
    Encoder_Init();
    Key_Init(&key, GPIOB, GPIO_Pin_8, on_key_event);
    /* 把 1ms 采样挂到定时器节拍上: 由本层主动注册, 而不是让 sys/timer.c
     * 反过来 include input.h 并直接调 Input_Scan —— 后者会让 sys/ 依赖 main/. */
    Timer_SetTickCallback(input_scan);
}


/* TIM2 1ms 中断内调用: 只采样入队, 不执行任何菜单逻辑/耗时回调.
 * 按键 5ms 扫描, 编码器 4ms 轮询(快速旋转时逐格出事件, 画面跟手). */
static void input_scan(void)
{
    scan_tick++;   /* 1ms 节拍 (与 TIM2 中断调用频率一致) */

    if(scan_tick - key_tick >= KEY_SCAN_MS)
    {
        key_tick = scan_tick;
        Key_Scan(&key);
    }

    if(scan_tick - enc_tick >= ENC_POLL_MS)
    {
        enc_tick = scan_tick;
        encoder_process();
    }
}


/* 主循环每轮最多分发 2 个新鲜事件。编码器旧事件超龄即丢弃；否则在 Flash/长测量
 * 阻塞恢复后，队列会把过期旋转逐格补执行，表现为突然跳格，甚至在页面已变后误操作.
 * 按键事件有预留槽且不超龄丢弃；过期旋转在这里快速跳过，不占本轮 2 个有效事件额度.
 * 检测到跨页面/列表上下文后清空剩余队列，并重同步编码器半格相位，避免旧上下文积压输入带入新页面. */
void Input_Process(void)
{
    input_evt_t evt;
    menu_cmd_t cmd;
    uint8_t n = 0;

    while (n < 2 && evt_dequeue(&evt))
    {
        uint16_t now16 = (uint16_t)Timer_GetTick();
        uint16_t age16 = (uint16_t)(now16 - evt.tick16);
        uint16_t ctx_before;

        if ((evt.event == MENU_EVENT_UP || evt.event == MENU_EVENT_DOWN) &&
             age16 > INPUT_ENCODER_MAX_AGE_MS)
            continue;   /* 丢弃过期旋转，不让旧输入在阻塞恢复后突然执行 */

        cmd.event = (menu_event_t)evt.event;
        cmd.step  = evt.step;
        ctx_before = menu_ctx();

        n++;
        menu_notify(&cmd);

        if (menu_ctx() != ctx_before)
        {
            Input_ClearEvents();     /* 上下文已变: 丢弃仍排队的旧输入 */
            Input_ResyncEncoder();   /* 同时丢弃编码器未满一格的残留相位 */
        }
    }
}


/* 重同步编码器相位(丢弃未满一格的残留计数), 供菜单跳转后调用.
 * 与 Input_ClearEvents 同层暴露: 页面/调度层(如 page/cur_cal.c、main/ui_task.c)
 * 只依赖 input.h, 不直接触碰 bsp/encoder.h. */
void Input_ResyncEncoder(void)
{
    Encoder_RequestResync();
}


/* 清空事件队列: 页面强制跳转(如锁存故障跳回 HOME)前丢弃积压事件,
 * 防止原页面的 CLICK/LPRESS/UP/DOWN 被目标页错位消费(误下钻/误恢复/误调温).
 * 实现要点: 只推进消费者自己的读指针 head 到当前 tail, 绝不回写 tail.
 * 本队列是 SPSC 结构 —— head 仅消费者改、tail 仅生产者(1ms 中断)改. 旧实现把
 * 两者都清零, 若中断恰在两次写之间入队, tail 会被重置为 0 而"队尾槽已写"的
 * 事实仍在, 于是一个只有 1 个元素的新队列会从旧位置读出 N 个陈旧槽位当成
 * 新事件消费. 改为 head=tail 后该交错不可能发生: 最多丢弃已入队的事件. */
void Input_ClearEvents(void)
{
    evt_q.head = evt_q.tail;   /* 消费到队尾 = 逻辑清空; 生产者无感知 */
}
