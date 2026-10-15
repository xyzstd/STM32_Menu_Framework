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

/* 每轮主循环最多分发的事件数。
 * 常规 2: 配合渲染层 UI_RENDER_COOLDOWN_MS(2ms) 合并窗口, 快速旋转时逐格跟手。
 * 队列中已有按键待处理时放宽到 8: 按键**不插队** —— 队列里积压的旋转是"按键之前
 * 产生、且尚未显示"的输入(menu_notify 是 dispatch 与 UI_RequestRefresh 绑定的),
 * 让按键插到它们前面会把按键落到过期的选中项上(点错目标), 随后那几格旋转再落进
 * 新页面, 与"进子菜单卡一格"是同一类错位。所以只加速排空、不改顺序, 使按键延迟
 * 从 "N 轮" 降到 "约 1 轮"。放宽期间渲染仍受 2ms 合并窗口约束, 不会连刷。 */
#define INPUT_DRAIN_NORMAL  2u
#define INPUT_DRAIN_BURST   8u

/* 消费上限守卫: 放宽值必须严格大于常规值(否则"按键在场时加速排空"名不副实,
 * 等于没放宽), 且常规值至少为 1(否则队列永不被消费). 违反即负数组长度 = 编译失败. */
typedef char input_drain_budget_must_be_valid[
    (INPUT_DRAIN_NORMAL >= 1u && INPUT_DRAIN_NORMAL < INPUT_DRAIN_BURST) ? 1 : -1];

/* 队列头尾用 uint8_t, 空槽法需留 1 格; 掩码求占用量要求容量为 2 的幂.
 * 另加两条, 守卫"按键永不因编码器刷太快而进不了队"这一性质:
 *   - INPUT_KEY_RESERVED_SLOTS >= 1: 预留为 0 时下面的"容量 > 预留+1"依然成立,
 *     守卫形同虚设, 而编码器会一路填满队列, 按键在队满时照样被丢 —— 静默失效.
 *   - 容量 > 预留+1: 保证"编码器可占格数"为正, 且至少留 1 格空槽给满/空判定. */
typedef char input_evt_queue_size_must_be_valid[
    (INPUT_EVT_QUEUE_SIZE >= 2u &&
     INPUT_EVT_QUEUE_SIZE <  256u &&   /* uint8_t head/tail 无法表示容量 256 */
     (INPUT_EVT_QUEUE_SIZE & (INPUT_EVT_QUEUE_SIZE - 1u)) == 0u &&
     INPUT_KEY_RESERVED_SLOTS >= 1u &&
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


/* 主循环每轮分发事件: 常规上限 INPUT_DRAIN_NORMAL 个(配合渲染合并窗口逐格跟手),
 * 队列中已有按键待处理时放宽到 INPUT_DRAIN_BURST 个 —— 顺序完全不变, 只是把排在
 * 按键前面的积压旋转快速吐完(见 INPUT_DRAIN_* 处说明)。
 * 编码器旧事件超龄即丢弃；否则在 Flash/长测量
 * 阻塞恢复后，队列会把过期旋转逐格补执行，表现为突然跳格，甚至在页面已变后误操作.
 * 按键事件有预留槽且不超龄丢弃；过期旋转在这里快速跳过，不占本轮有效事件额度.
 * 检测到跨页面/列表上下文后清空剩余队列，并重同步编码器半格相位，避免旧上下文积压输入带入新页面. */
void Input_Process(void)
{
    input_evt_t evt;
    menu_cmd_t cmd;
    uint8_t n = 0;
    uint8_t budget = Input_HasKeyPending() ? INPUT_DRAIN_BURST : INPUT_DRAIN_NORMAL;

    while (n < budget && evt_dequeue(&evt))
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
            Input_ResetForJump();   /* 上下文已变: 清积压事件 + 重同步编码器相位 */
    }
}


/* 重同步编码器相位(丢弃未满一格的残留计数).
 * 常规调用路径是 Input_ResetForJump(它同时清队列) —— 单独用本函数只在"不需要
 * 丢事件、只想丢半格残留"的场合。与 Input_ClearEvents 同层暴露: 页面/调度层
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


/* 页面切换后的输入复位(见 input.h): 清队列 + 重同步编码器相位。
 * 只是把两件事打包, 本身不做事 —— 目的是让"跳转后复位"成为一个不可拆错的动作:
 * 原先 ui_task 故障跳转、CurCal_BootEnter、input.c 的 ctx 变化路径各自手写这两行,
 * 三份重复且顺序不一(前两处是"先清后跳", 会漏掉跳转落盘期间入队的事件)。 */
void Input_ResetForJump(void)
{
    Input_ClearEvents();
    Input_ResyncEncoder();
}


/* 队列中是否有按键事件待处理(见 input.h): 只读扫描, 不改 head/tail。
 * 并发安全: 快照 head 与 tail 后只读 [head, tail) 区间 —— 生产者(1ms 中断)是先写
 * buf[tail] 再推进 tail, 故该区间内的槽位必已写完; 扫描期间生产者只会往更后面追加,
 * 不会改写本区间。tail 快照之后新入队的按键本次看不到, 下一轮即见 —— 作为"是否放宽
 * 本轮消费上限"的启发式判据, 这个时间差无影响。 */
bool Input_HasKeyPending(void)
{
    uint8_t i   = evt_q.head;
    uint8_t end = evt_q.tail;

    while (i != end)
    {
        uint8_t e = evt_q.buf[i].event;

        if (e == MENU_EVENT_CLICK || e == MENU_EVENT_LPRESS)
            return true;

        i = (uint8_t)((i + 1u) % INPUT_EVT_QUEUE_SIZE);
    }

    return false;
}
