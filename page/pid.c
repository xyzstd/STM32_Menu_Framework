#include "menu.h"
#include "oled.h"
#include "menu_display.h"
#include "pid.h"
#include "config.h"
#include "pid_controller.h"

/*
 * PID 参数设置: 两个子页面
 *   - 激进参数 (ΔT 30~50°C 使用): KP/KI/KD
 *   - 保守参数 (ΔT ≤30°C 使用):  KP/KI/KD
 *
 * 实际值 = 存储值 / 100 (KP/KI/KD)
 * 积分限幅为内置固定值 (PID 输出满量程 30%), 不提供页面调节.
 */

/* KP/KI/KD 上限 (*100) 与步进 */
#define PID_KP_MAX 2000   /* 20.00, 覆盖激进默认 11.00 */
#define PID_KI_MAX 500    /* 5.00 */
#define PID_KD_MAX 1000   /* 10.00, 覆盖保守默认 6.00 */
#define PID_STEP   10     /* 步进 0.10 */

/* ==================== 通用 3 参数编辑页 ==================== */

typedef struct {
    const char *label[3];  /* 参数名 */
    int16_t     val[3];    /* 编辑缓冲 (*100) */
    int16_t     max[3];    /* 上限 (*100) */
    uint8_t     focus;     /* 0..2 */
    bool        sel;       /* false=导航态, true=调参态 */
    bool        dirty;     /* 参数是否被修改 */
} pid_edit_t;

/* 激进组 / 保守组 编辑状态 */
static pid_edit_t g_agg = {
    .label = {"KP:", "KI:", "KD:"},
    .max   = {PID_KP_MAX, PID_KI_MAX, PID_KD_MAX},
    .focus = 0, .sel = false, .dirty = false,
};
static pid_edit_t g_cons = {
    .label = {"KP:", "KI:", "KD:"},
    .max   = {PID_KP_MAX, PID_KI_MAX, PID_KD_MAX},
    .focus = 0, .sel = false, .dirty = false,
};

static void pid_edit_reset(pid_edit_t *e)
{
    e->focus = 0;
    e->sel = false;
    e->dirty = false;
}

static menu_result_t pid_edit_handler(const menu_cmd_t *cmd, pid_edit_t *e)
{
    switch(cmd->event)
    {
        case MENU_EVENT_UP:
        case MENU_EVENT_DOWN:
            if(!e->sel)
            {
                /* 导航聚焦: 与菜单浏览同向 —— 正转(UP事件)向下一个参数, 反转(DOWN事件)回上一个参数 */
                if(cmd->event == MENU_EVENT_UP)
                    e->focus = (e->focus == 2) ? 0 : (uint8_t)(e->focus + 1);
                else
                    e->focus = (e->focus == 0) ? 2 : (uint8_t)(e->focus - 1);
            }
            else
            {
                /* 调参 */
                if(cmd->event == MENU_EVENT_UP)
                    e->val[e->focus] += (int16_t)(PID_STEP * cmd->step);
                else
                    e->val[e->focus] -= (int16_t)(PID_STEP * cmd->step);
                if(e->val[e->focus] < 0) e->val[e->focus] = 0;
                if(e->val[e->focus] > e->max[e->focus]) e->val[e->focus] = e->max[e->focus];
                e->dirty = true;
            }
            break;

        case MENU_EVENT_CLICK:
            e->sel = !e->sel;
            break;

        case MENU_EVENT_LPRESS:
            /* 无论是否在选中/调参态, 长按一律返回上一级 */
            return MENU_RET_DONE;

        default:
            break;
    }
    return MENU_RET_OK;
}

static void pid_edit_draw_row(int16_t y, const char *name, int16_t val)
{
    int16_t int_part = val / 100;
    int16_t dec_part = val % 100;

    OLED_ShowString(4, y, name, OLED_8X16);
    OLED_ShowNum(40, y, int_part, 2);
    OLED_ShowChar(56, y, '.', OLED_8X16);
    if(dec_part < 10)
        OLED_ShowNum(64, y, dec_part * 10, 2);
    else
        OLED_ShowNum(64, y, dec_part, 2);
}

static menu_period_t pid_edit_display(const pid_edit_t *e)
{
    /* 3 行布局 (垂直居中) */
    const uint8_t row_y[3] = {8, 24, 40};

    OLED_Clear();

    pid_edit_draw_row(row_y[0], e->label[0], e->val[0]);
    pid_edit_draw_row(row_y[1], e->label[1], e->val[1]);
    pid_edit_draw_row(row_y[2], e->label[2], e->val[2]);

    /* 右上角显示当前生效手柄型号徽标, 防调错别的型号参数;
     * 该型号校准点非法(退化映射)时反显警示, 提示需重新校准 */
    {
        const char *nm = Config_GetHandleName();
        uint8_t len = 0;
        while (nm[len]) len++;
        int16_t bx = (int16_t)(OLED_WIDTH - len * 8 - 2);
        OLED_ShowString(bx, 0, nm, OLED_6X8);
        if (!Config_IsHandleCalValid(Config_GetHandleIdx()))
            OLED_ReverseRoundRect(bx, 0, len * 8, 8, 2);
    }

    /* 聚焦/选中框: 行文本 y..y+15 上下各留 1px; 标签 x4 左留 2px, 数值右缘 x79 右留 2px */
    if(e->sel)
        OLED_ReverseRoundRect(2, row_y[e->focus] - 1, 80, 18, 6);
    else
        OLED_DrawRoundRect(2, row_y[e->focus] - 1, 80, 18, 6, OLED_COLOR_SET);

    OLED_Update();
    return 0;
}

/* ==================== 激进参数 ==================== */

static void pid_agg_enter(void *user_data)
{
    (void)user_data;
    uint8_t h = Config_GetHandleIdx();
    g_agg.val[0] = (int16_t)g_config.pid_agg_kp[h];
    g_agg.val[1] = (int16_t)g_config.pid_agg_ki[h];
    g_agg.val[2] = (int16_t)g_config.pid_agg_kd[h];
    pid_edit_reset(&g_agg);
}

static menu_result_t pid_agg_handler(const menu_cmd_t *cmd, void *user_data)
{
    (void)user_data;
    return pid_edit_handler(cmd, &g_agg);
}

static menu_period_t pid_agg_display(void *user_data)
{
    (void)user_data;
    return pid_edit_display(&g_agg);
}

static void pid_agg_exit(void *user_data)
{
    (void)user_data;
    if(g_agg.dirty)
    {
        uint8_t h = Config_GetHandleIdx();
        g_config.pid_agg_kp[h] = g_agg.val[0];
        g_config.pid_agg_ki[h] = g_agg.val[1];
        g_config.pid_agg_kd[h] = g_agg.val[2];
        PID_Controller_SetAggKp(g_agg.val[0]);
        PID_Controller_SetAggKi(g_agg.val[1]);
        PID_Controller_SetAggKd(g_agg.val[2]);
        Config_Save();
        g_agg.dirty = false;
    }
}

/* ==================== 保守参数 ==================== */

static void pid_cons_enter(void *user_data)
{
    (void)user_data;
    uint8_t h = Config_GetHandleIdx();
    g_cons.val[0] = (int16_t)g_config.pid_cons_kp[h];
    g_cons.val[1] = (int16_t)g_config.pid_cons_ki[h];
    g_cons.val[2] = (int16_t)g_config.pid_cons_kd[h];
    pid_edit_reset(&g_cons);
}

static menu_result_t pid_cons_handler(const menu_cmd_t *cmd, void *user_data)
{
    (void)user_data;
    return pid_edit_handler(cmd, &g_cons);
}

static menu_period_t pid_cons_display(void *user_data)
{
    (void)user_data;
    return pid_edit_display(&g_cons);
}

static void pid_cons_exit(void *user_data)
{
    (void)user_data;
    if(g_cons.dirty)
    {
        uint8_t h = Config_GetHandleIdx();
        g_config.pid_cons_kp[h] = g_cons.val[0];
        g_config.pid_cons_ki[h] = g_cons.val[1];
        g_config.pid_cons_kd[h] = g_cons.val[2];
        PID_Controller_SetConsKp(g_cons.val[0]);
        PID_Controller_SetConsKi(g_cons.val[1]);
        PID_Controller_SetConsKd(g_cons.val[2]);
        Config_Save();
        g_cons.dirty = false;
    }
}

/* ==================== ops ==================== */

const menu_ops_t pid_agg_ops = {
    .enter   = pid_agg_enter,
    .handler = pid_agg_handler,
    .display = pid_agg_display,
    .exit    = pid_agg_exit,
};

const menu_ops_t pid_cons_ops = {
    .enter   = pid_cons_enter,
    .handler = pid_cons_handler,
    .display = pid_cons_display,
    .exit    = pid_cons_exit,
};
