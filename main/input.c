#include "stm32f10x.h"
#include "key.h"
#include "encoder.h"
#include "input.h"
#include "menu.h"
#include "ui_task.h"
#include "work_task.h"
#include "timer.h"

/* 一组 = 4 个硬件计数 = 1 个齿格 = 1 次菜单步. 菜单导航与参数编辑共用同一套消费逻辑. */
#define ENC_GROUP_TICKS  4

/* 采样分频(ms): 定时器节拍 1ms. 注意与上面的"4 个硬件计数"量纲不同 ——
 * 前者是编码器计数, 后者是毫秒, 只是数值巧合相同. */
#define KEY_SCAN_MS      5   /* 按键扫描间隔 */
#define ENC_POLL_MS      4   /* 编码器轮询间隔 */

/* 事件队列容量: 中断(生产者)入队, 主循环(消费者)出队.
 * 12ms 轮询内极速旋转的事件数远小于此, 满则丢弃最新, 可接受. */
#define INPUT_EVT_QUEUE_SIZE  32

static key_info_t key;

/* 位置锚定齿格消费: enc_pos 为累计计数(相对开机), 锚点 enc_det_pos 只在位置
 * 真正走过一个完整齿格(±4 计数)后才推进, 每次推进消费对应方向的一个事件:
 *   - 正反对称: 前进/后退各满 4 计数出一事件, 行程一致, 快速往返事件密度稳定;
 *   - 抖动免疫: 停在齿格边界附近时位置小幅回摆(±2 计数)不跨锚点, 不会产生
 *     "零位移消费一格再弹回"的伪事件对(中点触发模型在该场景的固有缺陷);
 *   - 转向重锚: 残留相位中突然反向时, 方向翻转确认即贴锚, 消除"还清残留"死区. */
static int32_t enc_pos = 0;
static int32_t enc_det_pos = 0;
static int32_t enc_last_pos = 0;   /* 上一轮位置, 判运动方向 */
static int8_t  enc_dir = 0;        /* 上一轮运动方向: 1=正, -1=反, 0=静止 */
static uint8_t enc_static_cnt = 0; /* 位置连续静止的轮询轮数(静止吸合用) */

/* 1ms 节拍与分频计数: 仅由 input_scan 在 TIM2 中断内读写(单上下文, 故无需 volatile).
 * scan_tick 为自由运行的毫秒节拍, 差值比较天然抗 2^32 回绕. */
static uint32_t scan_tick = 0;   /* 1ms 节拍 */
static uint32_t key_tick  = 0;   /* 上次按键扫描时刻 */
static uint32_t enc_tick  = 0;   /* 上次编码器轮询时刻 */


/* ==================== 事件队列 (SPSC 环形缓冲) ====================
 * 中断内只做采样入队(evt_enqueue), 主循环 Input_Process 出队执行菜单逻辑,
 * 把 Flash 保存等耗时操作整体挪出 1ms 定时器中断, 输入节拍不再被渲染/落盘拖慢.
 * 并发安全: 单生产者(中断)单消费者(主循环), head 仅消费者改, tail 仅生产者改,
 * 空 = head==tail, 满 = tail+1==head, 无锁; 消费者复制期间生产者不写该位置. */
static struct{
    menu_cmd_t buf[INPUT_EVT_QUEUE_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
}evt_q;

static bool evt_enqueue(uint8_t event, int16_t step);
static bool evt_dequeue(menu_cmd_t *out);

static void menu_notify(menu_cmd_t *cmd);
static void on_key_event(key_event_t evt);

static void encoder_process(void);

static void input_scan(void);


static bool evt_enqueue(uint8_t event, int16_t step)
{
    uint8_t next = (uint8_t)((evt_q.tail + 1) % INPUT_EVT_QUEUE_SIZE);

    if(next == evt_q.head)
    {
        /* 满: 旋转事件(UP/DOWN)直接丢弃; 按键事件(CLICK/LPRESS)覆盖队尾旋转事件,
         * 保证高速旋转后立即按键不丢操作(丢键=页面操作无响应/乱跳感).
         * 竞态: 消费者正处理队尾槽时被替换, 单字节写完整, 结果仍是合法事件. */
        if(event == MENU_EVENT_CLICK || event == MENU_EVENT_LPRESS)
        {
            uint8_t last = (uint8_t)((evt_q.tail + INPUT_EVT_QUEUE_SIZE - 1) % INPUT_EVT_QUEUE_SIZE);

            if(evt_q.buf[last].event == MENU_EVENT_UP || evt_q.buf[last].event == MENU_EVENT_DOWN)
            {
                evt_q.buf[last].event = (menu_event_t)event;
                evt_q.buf[last].step  = step;
                return true;
            }
        }
        return false;   /* 仍满: 极端高速旋转或按键自身积压, 丢弃 */
    }

    evt_q.buf[evt_q.tail].event = (menu_event_t)event;
    evt_q.buf[evt_q.tail].step  = step;
    evt_q.tail = next;
    return true;
}

static bool evt_dequeue(menu_cmd_t *out)
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

    Menu_HandleEvent(cmd);
    UI_RequestRefresh();
}


static void on_key_event(key_event_t evt)
{
    if(evt == KEY_EVENT_CLICK)
        evt_enqueue(MENU_EVENT_CLICK, 0);
    else if(evt == KEY_EVENT_LONG_PRESS)
        evt_enqueue(MENU_EVENT_LPRESS, 0);
}


/* 编码器采样(中断内): 位置超出锚点一个完整齿格(4 计数)才出方向事件入队, 方向映射在消费端.
 * 转向重锚: 位置在残留相位(未满 4 计数)中突然反向时, 若仍等满 4 计数才触发,
 * 需先"还清"残留(最多 3 计数)再凑满 4 → 最坏 7 计数死区(≈1.75 齿格无响应, 即"突然转向卡").
 * 方向翻转确认(反向 ≥1 计数)立即把锚点贴到当前位置, 反向从零起计, 死区压缩到 ≤1.25 齿格.
 * 贴锚只置 delta=0 不触发事件, 慢速防抖(伪事件对吸收)保持不变.
 * 入队语义: 正向=UP(增大), 反向=DOWN; 具体去向(列表下移/参数增减)由消费时状态决定. */
static void encoder_process(void)
{
    int32_t delta, n;
    int8_t  dir;

    enc_pos += Encoder_GetDiff();                /* 硬件计数持续累计(差分读取无丢失) */
    delta = enc_pos - enc_det_pos;

    dir = (enc_pos > enc_last_pos) ? 1 : (enc_pos < enc_last_pos) ? -1 : 0;
    if (dir != 0 && enc_dir != 0 && dir != enc_dir &&
        !(delta >= ENC_GROUP_TICKS || delta <= -ENC_GROUP_TICKS))
    {
        /* 转向: 残留相位预支 —— 锚点沿原方向再推残留量(带符号),
         * 反向计数从 -|残留| 起计, 再走 4-|残留| 计数即出事件,
         * 死区由 1 齿格压到 ≤半格(残留≥2 时生效).
         * 注意: 残留必须经 enc_det_pos 偏移带入后续轮次, 不能只改本轮
         * delta (delta 每轮由 enc_pos-enc_det_pos 重算, 单轮取反会被覆盖).
         * 边界回摆(≤2 计数不跨新锚点 4 计数)仍无事件, 抖动免疫不变;
         * 残留=0 的转向(转满一格后立刻反向)无残留可预支, 死区保持 1 齿格
         * (满 4 计数锚定模型的固有下限, 突破需接受回摆伪事件, 不做). */
        enc_det_pos = enc_pos + delta;
        delta = -delta;
    }
    enc_dir = dir;
    enc_last_pos = enc_pos;

    /* 静止吸合(齿格吸附): 位置连续 2 个轮询周期(8ms)无变化即视为静止.
     * 若残留相位已走满大半齿格(|delta|==3, 即 3/4 格)停住, 立即兑现 1 个方向
     * 事件并贴锚, 消除"转了大半格停住但 UI 未动"的攒格延迟(用户"卡格"来源).
     * 阈值取 3 而非 2 的权衡:
     *   - |delta|<=2 的残留(≤半格)保留不动: 慢速逐格点动(每次 1~2 计数停)若被
     *     清零会永不触发(更卡); 编码器回摆抖动(±2 计数)若被吸合会放大成伪事件
     *     (破坏锚定模型抖动免疫);
     *   - 连续旋转(dir!=0)不触发吸合, 正常满 4 计数出事件路径不变. */
    if (dir == 0)
    {
        if (enc_static_cnt < 250)
            enc_static_cnt++;
        if (enc_static_cnt >= 2)
        {
            if (delta >= 3)
            {
                evt_enqueue(MENU_EVENT_UP, 1);
                enc_det_pos = enc_pos;
                delta = 0;
            }
            else if (delta <= -3)
            {
                evt_enqueue(MENU_EVENT_DOWN, 1);
                enc_det_pos = enc_pos;
                delta = 0;
            }
        }
    }
    else
    {
        enc_static_cnt = 0;
    }

    if (delta >= ENC_GROUP_TICKS)
    {
        n = delta / ENC_GROUP_TICKS;             /* 正向跨过的完整齿格数 */
        enc_det_pos += n * ENC_GROUP_TICKS;
        while (n-- > 0)
            evt_enqueue(MENU_EVENT_UP, 1);
    }
    else if (delta <= -ENC_GROUP_TICKS)
    {
        n = delta / ENC_GROUP_TICKS;             /* 反向跨过的完整齿格数(负, 向零截断) */
        enc_det_pos += n * ENC_GROUP_TICKS;
        while (n++ < 0)
            evt_enqueue(MENU_EVENT_DOWN, 1);
    }
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


/* 主循环事件消费: 每轮最多消费 2 个事件, 不一次清空队列.
 * 快速旋转时事件批量到达, 若 while 一次清空, 渲染合并窗口内会一次反映 N 格
 * (视觉"先不动后连续跳格"); 限流后事件按到达顺序摊平到后续各帧, 每帧 ≤2 格,
 * 跟手且吞吐(≈800/s)仍高于编码器物理事件率, 队列不积压不丢事件.
 * 菜单逻辑/Flash 落盘等耗时操作全部在主循环上下文执行, 不阻塞 1ms 输入节拍. */
void Input_Process(void)
{
    menu_cmd_t cmd;
    uint8_t n = 0;

    while (n < 2 && evt_dequeue(&cmd))
    {
        n++;
        menu_notify(&cmd);
    }
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
