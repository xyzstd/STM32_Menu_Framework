#include "menu.h"
#include "oled.h"
#include "pid.h"
#include "config.h"
#include "pid_controller.h"
#include "toast.h"
#include "buzzer.h"

/*
 * PID 参数设置: 两个子页面
 *   - 激进参数 (ΔT 30~50°C 使用): KP/KI/KD
 *   - 保守参数 (ΔT ≤30°C 使用):  KP/KI/KD
 *
 * 实际值 = 存储值 / 100 (KP/KI/KD)
 * 积分限幅为内置固定值 (PID 输出满量程 30%), 不提供页面调节.
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

/* 编辑状态机实例: 焦点/编辑态/脏标记与编辑缓冲, 每组(激进/保守)一份 */
typedef struct {
    int16_t val[PID_FIELD_NUM];   /* 编辑缓冲 */
    int16_t base[PID_FIELD_NUM];  /* enter 时初值: 判"是否真改了" + 落盘失败回滚 */
    uint8_t focus;                /* 聚焦字段 0..2 */
    uint8_t editing;              /* 0=导航态, 1=调参态 */
} pid_edit_t;

static pid_edit_t agg;   /* 激进参数组 */
static pid_edit_t cons;  /* 保守参数组 */

static menu_result_t pid_edit_handler(const menu_cmd_t *cmd, pid_edit_t *e);
static void pid_agg_load(void);
static bool pid_agg_store(void);
static void pid_cons_load(void);
static bool pid_cons_store(void);
static void pid_edit_draw_row(int16_t y, const char *name, int16_t val);
static menu_period_t pid_edit_display(const pid_edit_t *e);
static void pid_agg_enter(void *ud);
static menu_result_t pid_agg_handler(const menu_cmd_t *cmd, void *ud);
static menu_period_t pid_agg_display(void *ud);
static void pid_agg_exit(void *ud);
static void pid_cons_enter(void *ud);
static menu_result_t pid_cons_handler(const menu_cmd_t *cmd, void *ud);
static menu_period_t pid_cons_display(void *ud);
static void pid_cons_exit(void *ud);

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

/* ==================== 激进参数: 载入/落盘 ==================== */

static void pid_agg_load(void)
{
    uint8_t h = Config_GetHandleIdx();
    agg.val[0] = (int16_t)g_config.pid_agg_kp[h];
    agg.val[1] = (int16_t)g_config.pid_agg_ki[h];
    agg.val[2] = (int16_t)g_config.pid_agg_kd[h];
    agg.base[0] = agg.val[0];
    agg.base[1] = agg.val[1];
    agg.base[2] = agg.val[2];
}

/* 写 g_config + 落盘; 返回 true = 已成功写入 Flash.
 * 失败时把 g_config / PID 控制器 / 编辑缓冲**一并回滚**到 enter 初值 ——
 * 不回滚的话运行时按新增益工作而 Flash 仍是旧值, 重启后悄悄变回去; 且重进
 * 本页 load 会读到新值, 表现成"已经保存了", 正是最误导人的静默数据丢失. */
static bool pid_agg_store(void)
{
    uint8_t h = Config_GetHandleIdx();
    uint8_t i;

    g_config.pid_agg_kp[h] = agg.val[0];
    g_config.pid_agg_ki[h] = agg.val[1];
    g_config.pid_agg_kd[h] = agg.val[2];
    PID_Controller_SetAggKp(agg.val[0]);
    PID_Controller_SetAggKi(agg.val[1]);
    PID_Controller_SetAggKd(agg.val[2]);

    if (Config_Save())
        return true;

    g_config.pid_agg_kp[h] = agg.base[0];
    g_config.pid_agg_ki[h] = agg.base[1];
    g_config.pid_agg_kd[h] = agg.base[2];
    PID_Controller_SetAggKp(agg.base[0]);
    PID_Controller_SetAggKi(agg.base[1]);
    PID_Controller_SetAggKd(agg.base[2]);
    for (i = 0; i < PID_FIELD_NUM; i++)
        agg.val[i] = agg.base[i];
    return false;
}

/* ==================== 保守参数 ==================== */

static void pid_cons_load(void)
{
    uint8_t h = Config_GetHandleIdx();
    cons.val[0] = (int16_t)g_config.pid_cons_kp[h];
    cons.val[1] = (int16_t)g_config.pid_cons_ki[h];
    cons.val[2] = (int16_t)g_config.pid_cons_kd[h];
    cons.base[0] = cons.val[0];
    cons.base[1] = cons.val[1];
    cons.base[2] = cons.val[2];
}

/* 同 pid_agg_store: 返回 true = 已落盘; 失败则回滚到 enter 初值. */
static bool pid_cons_store(void)
{
    uint8_t h = Config_GetHandleIdx();
    uint8_t i;

    g_config.pid_cons_kp[h] = cons.val[0];
    g_config.pid_cons_ki[h] = cons.val[1];
    g_config.pid_cons_kd[h] = cons.val[2];
    PID_Controller_SetConsKp(cons.val[0]);
    PID_Controller_SetConsKi(cons.val[1]);
    PID_Controller_SetConsKd(cons.val[2]);

    if (Config_Save())
        return true;

    g_config.pid_cons_kp[h] = cons.base[0];
    g_config.pid_cons_ki[h] = cons.base[1];
    g_config.pid_cons_kd[h] = cons.base[2];
    PID_Controller_SetConsKp(cons.base[0]);
    PID_Controller_SetConsKi(cons.base[1]);
    PID_Controller_SetConsKd(cons.base[2]);
    for (i = 0; i < PID_FIELD_NUM; i++)
        cons.val[i] = cons.base[i];
    return false;
}

/* ==================== 共享 display 布局 (本页私有) ==================== */

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

/* ==================== 激进参数页: 薄闭包绑定静态实例 ==================== */

static void pid_agg_enter(void *ud)
{
    (void)ud;
    pid_agg_load();
    agg.focus = 0;
    agg.editing = 0;
}
static menu_result_t pid_agg_handler(const menu_cmd_t *cmd, void *ud){ (void)ud; return pid_edit_handler(cmd, &agg); }
static menu_period_t pid_agg_display(void *ud){ (void)ud; return pid_edit_display(&agg); }
static void pid_agg_exit(void *ud)
{
    uint8_t i;
    bool changed = false;
    (void)ud;

    /* 只有确实改动了值才落盘(见文件顶部说明) */
    for (i = 0; i < PID_FIELD_NUM; i++)
    {
        if (agg.val[i] != agg.base[i]) { changed = true; break; }
    }
    if (!changed)
        return;

    if (pid_agg_store())
        Toast_Saved();   /* 仅在确实落盘成功时弹框: 失败却提示"已保存"会误导用户 */
    else
        Buzzer_Beep(BUZZER_EVENT_ALARM);   /* 失败必须告警, 否则用户以为已存下 */
}

const menu_ops_t pid_agg_ops = {
    .enter = pid_agg_enter, .handler = pid_agg_handler,
    .display = pid_agg_display, .exit = pid_agg_exit,
};

/* ==================== 保守参数页 ==================== */

static void pid_cons_enter(void *ud)
{
    (void)ud;
    pid_cons_load();
    cons.focus = 0;
    cons.editing = 0;
}
static menu_result_t pid_cons_handler(const menu_cmd_t *cmd, void *ud){ (void)ud; return pid_edit_handler(cmd, &cons); }
static menu_period_t pid_cons_display(void *ud){ (void)ud; return pid_edit_display(&cons); }
static void pid_cons_exit(void *ud)
{
    uint8_t i;
    bool changed = false;
    (void)ud;

    for (i = 0; i < PID_FIELD_NUM; i++)
    {
        if (cons.val[i] != cons.base[i]) { changed = true; break; }
    }
    if (!changed)
        return;

    if (pid_cons_store())
        Toast_Saved();
    else
        Buzzer_Beep(BUZZER_EVENT_ALARM);
}

const menu_ops_t pid_cons_ops = {
    .enter = pid_cons_enter, .handler = pid_cons_handler,
    .display = pid_cons_display, .exit = pid_cons_exit,
};
