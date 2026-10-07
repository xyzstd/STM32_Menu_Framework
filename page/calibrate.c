#include "menu.h"
#include "oled.h"
#include "menu_display.h"
#include "calibrate.h"
#include "config.h"
#include "work_task.h"
#include "pid_controller.h"

/* 多点温度校准: 100/200/300/350/400/450°C 六个温度点各设一个偏移量
 * 实际温度 = 六点插值基准温度 + 按温度区间插值的偏移
 * 偏移量范围: -50 ~ +50 °C
 *
 * 使用流程:
 *   1. 进入本页后, 先用 UP/DOWN 导航切换聚焦点 (100/200/300/350/400/450), 此时不改变目标温度
 *   2. CLICK 选中当前聚焦点后, 才将其设为目标温度, 烙铁加热至该校准点
 *   3. 调参态 UP/DOWN 调整该点偏移; CLICK 或 LPRESS 返回导航态 (不退出页面)
 *   4. 等温度稳定后, 用万用表测烙铁头实际温度, 将"实测 - 设定"的差值直接填入偏移
 *      (实际温度 = 基准 + 偏移, 实测比设定高则填正值, 如设定350/实测355 → +5)
 *   5. 导航态 LPRESS 退出, 恢复进入前的目标温度 */

#define CAL_OFF_MAX   50
#define CAL_OFF_MIN   (-50)

/* 各校准点的目标温度 (°C) */
static const int16_t cal_points[CAL_POINT_NUM] = {100, 200, 300, 350, 400, 450};

static int16_t off[CAL_POINT_NUM]; /* 各温度点偏移量 */
static int16_t prev_setpoint;      /* 进入本页前的目标温度, 退出时恢复 */
static uint8_t focus = 0;          /* 当前聚焦的温度点 0..5 */
static bool sel = false;           /* false=导航态, true=调参态 */
static bool dirty = false;         /* 参数是否被修改 */

static const char *cal_names[CAL_POINT_NUM] = {"100:", "200:", "300:", "350:", "400:", "450:"};

static void cal_draw_row(uint8_t y, uint8_t idx);
static void cal_draw_live(uint8_t y);
static menu_result_t calibrate_handler(const menu_cmd_t *cmd, void *user_data);
static menu_period_t calibrate_display(void *user_data);
static void calibrate_enter(void *user_data);
static void calibrate_exit(void *user_data);

static void cal_draw_row(uint8_t y, uint8_t idx)
{
    int16_t v = off[idx];
    char buf[4];

    OLED_ShowString(6, y, cal_names[idx], OLED_6X8);
    buf[0] = (v < 0) ? '-' : '+';
    buf[1] = (char)('0' + ((v < 0 ? -v : v) / 10));
    buf[2] = (char)('0' + ((v < 0 ? -v : v) % 10));
    buf[3] = '\0';
    OLED_ShowString(40, y, buf, OLED_6X8);
}

/* 底部实时温度显示: 让用户对照实际温度调偏移 (T:235°C) */
static void cal_draw_live(uint8_t y)
{
    int16_t t = WorkTask_GetMeasuredTemp();
    char buf[4];
    uint8_t n;

    if (t < 0) t = 0;
    if (t > 999) t = 999;

    buf[0] = (char)('0' + t / 100);
    buf[1] = (char)('0' + (t / 10) % 10);
    buf[2] = (char)('0' + t % 10);
    buf[3] = '\0';
    n = (t >= 100) ? 3u : (t >= 10) ? 2u : 1u;

    /* 底部实时温度显示: 让用户对照实际温度调偏移 ("温度" 32px x4..36, 值列 x40) */
    OLED_ShowChinese(4, y, "温度");
    OLED_ShowString(40, y, buf + (3 - n), OLED_8X16);
    OLED_ShowDegree(40 + 8 * n, y);
    OLED_ShowChar(40 + 8 * n + 8, y, 'C', OLED_8X16);
}

static menu_result_t calibrate_handler(const menu_cmd_t *cmd, void *user_data)
{
    (void)user_data;
    switch(cmd->event)
    {
        case MENU_EVENT_UP:
        case MENU_EVENT_DOWN:
            if(!sel)
            {
                /* 导航态: 与菜单浏览同向 —— 正转(UP事件)向下一个校准点, 反转(DOWN事件)回上一个 */
                if(cmd->event == MENU_EVENT_UP)
                    focus = (focus == CAL_POINT_NUM - 1) ? 0
                                                             : (uint8_t)(focus + 1);
                else
                    focus = (focus == 0) ? (uint8_t)(CAL_POINT_NUM - 1)
                                             : (uint8_t)(focus - 1);
            }
            else
            {
                /* 调参态: 调整当前聚焦点偏移 */
                if(cmd->event == MENU_EVENT_UP)
                    off[focus] += cmd->step;
                else
                    off[focus] -= cmd->step;
                if (off[focus] > CAL_OFF_MAX) off[focus] = CAL_OFF_MAX;
                if (off[focus] < CAL_OFF_MIN) off[focus] = CAL_OFF_MIN;
                dirty = true;
            }
            break;
        case MENU_EVENT_CLICK:
            if(sel)
            {
                /* 退出调参 → 导航 */
                sel = false;
            }
            else
            {
                /* 进入调参: 选中后才把校准点设为目标温度, 烙铁加热至该校准点 */
                sel = true;
                PID_Controller_SetSetpoint(cal_points[focus]);
                PID_Controller_Reset();
            }
            break;
        case MENU_EVENT_LPRESS:
            if(sel)
            {
                /* 调参态长按: 改为非选中, 不直接退出 */
                sel = false;
            }
            else
            {
                /* 导航态长按: 退出页面 (exit 回调恢复目标温度) */
                return MENU_RET_DONE;
            }
            break;
        default:
            break;
    }
    return MENU_RET_OK;
}

static menu_period_t calibrate_display(void *user_data)
{
    (void)user_data;
    /* 紧凑布局: 6 行偏移 (6x8, y0 起每行 8px) + 底部实时温度 (8x16, y48) */
    const uint8_t row_y[CAL_POINT_NUM] = {0, 8, 16, 24, 32, 40};
    uint8_t i;

    OLED_Clear();

    for (i = 0; i < CAL_POINT_NUM; i++)
        cal_draw_row(row_y[i], i);

    cal_draw_live(48);

    /* 右上角显示当前生效手柄型号徽标, 防校准别的型号文件;
     * 该型号校准点非法(退化映射)时反显警示 */
    {
        const char *nm = Config_GetHandleName();
        uint8_t len = 0;
        while (nm[len]) len++;
        int16_t bx = (int16_t)(OLED_WIDTH - len * 8 - 2);
        OLED_ShowString(bx, 0, nm, OLED_6X8);
        if (!Cal_IsHandleValid(Config_GetHandleIdx()))
            OLED_ReverseRoundRect(bx, 0, len * 8, 8, 2);
    }

    /* 聚焦/选中框: 6x8 行文本 y..y+7 上下各留 1px, 故框起点 y (紧凑布局不再 -1),
     * 高度 10 覆盖整行; 标签 x6 左留 2px, 数值右缘 x61 右留 2px (OLED_6X8 实际按 8px 步进) */
    if(sel)
        OLED_ReverseRoundRect(4, row_y[focus], 60, 9, 4);
    else
        OLED_DrawRoundRect(4, row_y[focus], 60, 9, 4, OLED_COLOR_SET);

    OLED_UpdateDiff();
    return 0;
}

static void calibrate_enter(void *user_data)
{
    (void)user_data;
    uint8_t h = Config_GetHandleIdx();
    uint8_t i;

    for (i = 0; i < CAL_POINT_NUM; i++)
        off[i] = Cal_GetOff(i, h);   /* int8 → int16 隐式符号提升 */

    focus = 0;
    sel = false;
    dirty = false;

    /* 校准需长时间等温稳定, 期间抑制休眠/关机, 防止目标温度被入睡切走 */
    WorkTask_SetSleepSuppress(true);

    /* 记住进入前的目标温度, 退出时恢复 */
    prev_setpoint = PID_Controller_GetSetpoint();
    /* 不自动加热, 待用户 CLICK 选中校准点后才设置目标温度 */
}

static void calibrate_exit(void *user_data)
{
    (void)user_data;
    if(dirty)
    {
        uint8_t h = Config_GetHandleIdx();
        uint8_t i;

        for (i = 0; i < CAL_POINT_NUM; i++)
            Cal_SetOff(i, h, (int8_t)off[i]);   /* int16 → int8 需显式回缩 */
        /* 只写校准独立页, 不动用户设置. 返回值暂不向用户上报(本页无"已保存"
         * 提示, 见 F16); (void) 显式表明"有意忽略"而非漏检. */
        (void)Cal_Save();
        dirty = false;
    }

    /* 恢复进入前的目标温度 */
    PID_Controller_SetSetpoint(prev_setpoint);
    PID_Controller_Reset();
    WorkTask_SetSleepSuppress(false);   /* 退出校准, 恢复休眠/关机计时 */
}

const menu_ops_t calibrate_ops = {
    .enter = calibrate_enter,
    .handler = calibrate_handler,
    .display = calibrate_display,
    .exit = calibrate_exit,
};
