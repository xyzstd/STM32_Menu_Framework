#include "menu.h"
#include "oled.h"
#include "pid.h"
#include "config.h"
#include "pid_controller.h"
#include "toast.h"
#include "buzzer.h"

/*
 * PID 参数设置: 单组参数 (KP/KI/KD)
 *
 * 2026-10-19 由双组(激进/保守)合并为单组. 静态分析与仿真显示保守组在误差
 * 25°C 以下被前馈完全遮蔽, 双组唯一收益是过冲段阻尼, 而保守组 Kp=0.50
 * 刹车过弱、过冲反而更大 —— 合并后到温更快、过冲更小、参数少一半.
 * 详见 docs/ 与 build_tools/pid_sim.py.
 *
 * 实际值 = 存储值 / 100 (KP/KI/KD)
 * 积分限幅为内置值 (按 |误差| 在 10%~5% 间调度), 不提供页面调节.
 *
 * 多参数编辑状态机由本页自行实现(非跨文件模板, 与 calibrate 同风格):
 *   导航态 UP/DOWN 环形移动 focus; CLICK 切换调参态;
 *   调参态 UP/DOWN 改当前字段(线性钳位); LPRESS 返回上一级.
 *
 * 编辑缓冲另存一份 enter 初值副本(base), 两处用到:
 *   ① 退出时与 val 比对判断"是否真的改了值" —— 只看"是否调过参"会把
 *      "转到边界又转回来"也当成改动, 白擦一次 Flash;
 *   ② 落盘失败时回滚, 避免内存按新增益工作而 Flash 仍是旧值(重启后悄悄变回去).
 */

/* KP/KI/KD 上限 (*100) 见 pid_controller.h(PID_KP_MAX/KI_MAX/KD_MAX) —— 与
 * 自整定写回(main/work_task.c)同源; 此处只留页面私有的步进与字段数. */
#define PID_STEP   10     /* 步进 0.10 */
#define PID_FIELD_NUM 3

/* 字段描述表: 名称/上限/步进 (下限恒 0, 线性钳位不回绕) */
static const char *const pid_labels[PID_FIELD_NUM] = {"KP:", "KI:", "KD:"};
static const int16_t pid_max[PID_FIELD_NUM]  = {PID_KP_MAX, PID_KI_MAX, PID_KD_MAX};
static const int16_t pid_step[PID_FIELD_NUM] = {PID_STEP, PID_STEP, PID_STEP};

/* 编辑状态机实例: 焦点/编辑态与编辑缓冲 */
typedef struct {
    int16_t val[PID_FIELD_NUM];   /* 编辑缓冲 */
    int16_t base[PID_FIELD_NUM];  /* enter 时初值: 判"是否真改了" + 落盘失败回滚 */
    uint8_t focus;                /* 聚焦字段 0..2 */
    uint8_t editing;              /* 0=导航态, 1=调参态 */
} pid_edit_t;

static pid_edit_t pe;

static menu_result_t pid_edit_handler(const menu_cmd_t *cmd, pid_edit_t *e);
static void pid_load(void);
static bool pid_store(void);
static void pid_edit_draw_row(int16_t y, const char *name, int16_t val);
static menu_period_t pid_edit_display(const pid_edit_t *e);
static void pid_enter(void *ud);
static menu_result_t pid_handler(const menu_cmd_t *cmd, void *ud);
static menu_period_t pid_display(void *ud);
static void pid_exit(void *ud);

/* 编辑状态机: 导航聚焦(环形) / 调参增减(线性钳位) / CLICK 切换 / LPRESS 退出 */
static menu_result_t pid_edit_handler(const menu_cmd_t *cmd, pid_edit_t *e)
{
    switch(cmd->event)
    {
    case MENU_EVENT_UP:
    case MENU_EVENT_DOWN:
        if (!e->editing)
        {
            if (cmd->event == MENU_EVENT_UP)
                e->focus = (uint8_t)((e->focus == PID_FIELD_NUM - 1) ? 0 : e->focus + 1);
            else
                e->focus = (uint8_t)((e->focus == 0) ? PID_FIELD_NUM - 1 : e->focus - 1);
        }
        else
        {
            if (cmd->event == MENU_EVENT_UP)
            {
                e->val[e->focus] += pid_step[e->focus];
                if (e->val[e->focus] > pid_max[e->focus]) e->val[e->focus] = pid_max[e->focus];
            }
            else
            {
                e->val[e->focus] -= pid_step[e->focus];
                if (e->val[e->focus] < 0) e->val[e->focus] = 0;
            }
        }
        break;

    case MENU_EVENT_CLICK:
        e->editing = (uint8_t)!e->editing;
        break;

    case MENU_EVENT_LPRESS:
        return MENU_RET_DONE;   /* 长按一律返回上一级 */
    default:
        break;
    }
    return MENU_RET_STAY;
}

/* ==================== 载入/落盘 ==================== */

static void pid_load(void)
{
    uint8_t h = Config_GetHandleIdx();
    uint8_t i;

    pe.val[0] = (int16_t)g_config.pid_kp[h];
    pe.val[1] = (int16_t)g_config.pid_ki[h];
    pe.val[2] = (int16_t)g_config.pid_kd[h];

    for (i = 0; i < PID_FIELD_NUM; i++)
        pe.base[i] = pe.val[i];
}

/* 落盘: 先写 RAM(缓存权威), 再 Config_Save; 失败则把配置与控制器一起回滚.
 * 返回 false 时调用方必须告警 —— 否则用户以为已存下, 重启后悄悄变回旧值. */
static bool pid_store(void)
{
    uint8_t h = Config_GetHandleIdx();
    uint8_t i;

    g_config.pid_kp[h] = pe.val[0];
    g_config.pid_ki[h] = pe.val[1];
    g_config.pid_kd[h] = pe.val[2];

    if (Config_Save())
    {
        PID_Controller_SetKp(pe.val[0]);
        PID_Controller_SetKi(pe.val[1]);
        PID_Controller_SetKd(pe.val[2]);
        return true;
    }

    /* 落盘失败: RAM 与控制器都退回 enter 时的值, 保证"运行中生效的"
     * 与"Flash 里存的"始终一致 */
    g_config.pid_kp[h] = pe.base[0];
    g_config.pid_ki[h] = pe.base[1];
    g_config.pid_kd[h] = pe.base[2];
    PID_Controller_SetKp(pe.base[0]);
    PID_Controller_SetKi(pe.base[1]);
    PID_Controller_SetKd(pe.base[2]);
    for (i = 0; i < PID_FIELD_NUM; i++)
        pe.val[i] = pe.base[i];
    return false;
}

/* ==================== display 布局 (本页私有) ==================== */

static void pid_edit_draw_row(int16_t y, const char *name, int16_t val)
{
    int16_t int_part = val / 100;
    int16_t dec_part = val % 100;

    OLED_ShowString(4, y, name, OLED_8X16);
    OLED_ShowNum(40, y, int_part, 2);
    OLED_ShowChar(56, y, '.', OLED_8X16);
    OLED_ShowNum(64, y, dec_part, 2);   /* dec=0 时补零显示 "00" */
}

static menu_period_t pid_edit_display(const pid_edit_t *e)
{
    /* 3 行布局 (垂直居中) */
    const uint8_t row_y[PID_FIELD_NUM] = {8, 24, 40};

    OLED_Clear();

    pid_edit_draw_row(row_y[0], pid_labels[0], e->val[0]);
    pid_edit_draw_row(row_y[1], pid_labels[1], e->val[1]);
    pid_edit_draw_row(row_y[2], pid_labels[2], e->val[2]);

    /* 右上角显示当前生效手柄型号徽标, 防调错别的型号参数;
     * 该型号校准点非法(退化映射)时反显警示, 提示需重新校准 */
    {
        const char *nm = Config_GetHandleName();
        uint8_t len = 0;
        while (nm[len]) len++;
        int16_t bx = (int16_t)(OLED_WIDTH - len * 8 - 2);
        OLED_ShowString(bx, 0, nm, OLED_6X8);
        if (!Cal_IsHandleValid(Config_GetHandleIdx()))
            OLED_ReverseRoundRect(bx, 0, len * 8, 8, 2);
    }

    /* 聚焦/选中框: 16px 高与行距一致, 相邻行框精确贴合不重叠 */
    if (e->editing)
        OLED_ReverseRoundRect(2, row_y[e->focus], 80, 16, 6);
    else
        OLED_DrawRoundRect(2, row_y[e->focus], 80, 16, 6, OLED_COLOR_SET);

    OLED_UpdateDiff();
    return 0;
}

/* ==================== 页面: 薄闭包绑定静态实例 ==================== */

static void pid_enter(void *ud)
{
    (void)ud;
    pid_load();
    pe.focus = 0;
    pe.editing = 0;
}

static menu_result_t pid_handler(const menu_cmd_t *cmd, void *ud)
{
    (void)ud;
    return pid_edit_handler(cmd, &pe);
}

static menu_period_t pid_display(void *ud)
{
    (void)ud;
    return pid_edit_display(&pe);
}

static void pid_exit(void *ud)
{
    uint8_t i;
    bool changed = false;
    (void)ud;

    /* 只有确实改动了值才落盘(见文件顶部说明) */
    for (i = 0; i < PID_FIELD_NUM; i++)
    {
        if (pe.val[i] != pe.base[i]) { changed = true; break; }
    }
    if (!changed)
        return;

    if (pid_store())
        Toast_Saved();   /* 仅在确实落盘成功时弹框: 失败却提示"已保存"会误导用户 */
    else
        Buzzer_Beep(BUZZER_EVENT_ALARM);   /* 失败必须告警, 否则用户以为已存下 */
}

const menu_ops_t pid_ops = {
    .enter = pid_enter, .handler = pid_handler,
    .display = pid_display, .exit = pid_exit,
};
