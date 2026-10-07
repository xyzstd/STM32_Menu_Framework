#include "menu.h"
#include "input.h"
#include "cur_cal.h"
#include "oled.h"
#include "work_task.h"
#include "config.h"
#include "buzzer.h"
#include "toast.h"

/* 电流标定页 (Cur Cal):
 * 自动实测当前插入烙铁头的"冷态发热芯测量脉冲导通电流" ADC, 供发热芯
 * 型号匹配检查 (type_cur_match) 作参考值. 进入后若烙铁头热态(超过
 * TYPE_CUR_COOL_MAX)则自动等待降温, 显示 "Cooling: XX°C", 冷态后才实测(阻塞约 200ms):
 *   - 存在锁存故障 → 拒绝测量并蜂鸣
 *   - UP/DOWN: 选择要标定的型号 (T12/T245/T210)
 *   - CLICK:   将实测值写入选中型号参考并落盘 (TypeCal_SetRef)
 *   - LPRESS:  退出; 换头标定需退出后重新进入以重新实测
 *
 * 首次引导模式 (CurCal_BootEnter, 标题 "First Cal"):
 * 上电自检通过且当前型号未标定时, UI 层自动进入本页, 引导用户先冷态
 * 标定再正常加热. 该模式下 CLICK 保存后 / LPRESS 均回 HOME 并清除
 * 引导标志 (WorkTask_BootCalDone), 退出回调统一清理. */

static const char cal_title[]  = "电流标定";
static const char boot_title[] = "首次标定";
static const char tip_title[]  = "换头标定";
static const char *const cal_names[HANDLE_NUM] = {"T12", "T245", "T210"};

/* 冷态标定温度门共用 TYPE_CUR_COOL_MAX (见 work_task.h):
 * 烙铁头高于此温度时阻值已偏移, 标定不可靠, 需等降温 */
/* 等降温时页面刷新周期 (ms) */
#define CUR_CAL_COOL_PERIOD  200

static uint8_t  cal_idx;      /* 当前选中的型号索引 */
static uint16_t meas_cur;     /* 本次实测脉冲电流 ADC */
static bool     meas_ok;      /* 本次测量是否有效 */
static bool     cal_entered;  /* 本次进入是否已实测 */
static bool     cal_boot;     /* 引导模式 (First Cal / Tip Cal) */
static bool     cal_lock;     /* 引导模式锁定型号: 首次(未标定)锁定, 插头不匹配放开 */

static const char *cur_cal_title(void);
static void cur_cal_measure(void);
static menu_result_t cur_cal_handler(const menu_cmd_t *cmd, void *ud);
static void cur_cal_enter(void *ud);
static void cur_cal_exit(void *ud);
static void cur_cal_center(const char *s, uint8_t y);
static menu_period_t cur_cal_display(void *ud);

/* 标题: 引导模式按是否锁定型号区分首次(First Cal)与插头不匹配(Tip Cal) */
static const char *cur_cal_title(void)
{
    if (!cal_boot)
        return cal_title;
    return cal_lock ? boot_title : tip_title;
}

static void cur_cal_measure(void)
{
    meas_ok  = false;
    meas_cur = 0;

    /* 存在锁存故障(热电偶/过温/加热器)时不应施加热芯脉冲 */
    if (WorkTask_HasFault())
    {
        Buzzer_Beep(BUZZER_EVENT_ALARM);
        return;
    }

    meas_cur = WorkTask_MeasurePulseCur();
    if (meas_cur == 0)
        Buzzer_Beep(BUZZER_EVENT_ALARM);   /* 非冷态: 阻值偏移, 标定不可靠 */
    else
        meas_ok = true;
}

static menu_result_t cur_cal_handler(const menu_cmd_t *cmd, void *ud)
{
    (void)ud;
    switch (cmd->event)
    {
    case MENU_EVENT_UP:
        if (!cal_lock)   /* 首次引导锁定当前型号; 插头不匹配引导允许确认型号 */
            cal_idx = (uint8_t)((cal_idx + 1) % HANDLE_NUM);
        break;
    case MENU_EVENT_DOWN:
        if (!cal_lock)
            cal_idx = (uint8_t)((cal_idx + HANDLE_NUM - 1) % HANDLE_NUM);
        break;
    case MENU_EVENT_CLICK:
        if (meas_ok)
        {
            /* 型号匹配校验: 拒绝"拿 T12 头填 T245/T210 参考值"等误操作
             * (两级保护, 判定规则见 WorkTask_TypeCurMatch); 不匹配拒保存并蜂鸣 */
            if (!WorkTask_TypeCurMatch(meas_cur, cal_idx))
            {
                Buzzer_Beep(BUZZER_EVENT_ALARM);
                break;
            }
            TypeCal_SetRef(cal_idx, meas_cur);
            Buzzer_Beep(BUZZER_EVENT_TEMP_REACHED);
            Toast_Saved();   /* 人性化反馈: 弹出"已保存"框 */
            /* 当前配置型号已标定 → 清除首次引导标志 (手动/引导路径均幂等;
             * 标定的是其他型号时不清, 引导仍待处理) */
            if (cal_idx == Config_GetHandleIdx())
                WorkTask_BootCalDone();
            if (cal_boot)
            {
                /* 跳 HOME 前清输入事件: 防残留旋转事件被 HOME 错位消费(误调温) */
                Input_ClearEvents();   /* 跳转前丢弃积压事件, 防目标页错位消费 */
                Menu_SetCurrentId(MENU_HOME_ID);   /* 引导模式: 标定完成回 HOME (exit 清标志) */
            }
        }
        else
        {
            Buzzer_Beep(BUZZER_EVENT_ALARM);   /* 测量无效: 拒绝保存 */
        }
        break;
    case MENU_EVENT_LPRESS:
        if (cal_boot)
        {
            /* 跳过引导: 当前型号仍无冷态电流参考 → 告警, 提示型号匹配保护
             * 未生效(细匹配无效 + 粗保护退化), 避免用户无感知降级 */
            if (TypeCal_GetRef(Config_GetHandleIdx()) == 0)
                Buzzer_Beep(BUZZER_EVENT_ALARM);
            /* 跳 HOME 前清输入事件: 防残留旋转事件被 HOME 错位消费(误调温) */
            Input_ClearEvents();   /* 跳转前丢弃积压事件, 防目标页错位消费 */
            Menu_SetCurrentId(MENU_HOME_ID);   /* 跳过引导: 回 HOME (exit 清标志) */
            return MENU_RET_OK;
        }
        return MENU_RET_DONE;
    default:
        break;
    }
    return MENU_RET_OK;
}

static void cur_cal_enter(void *ud)
{
    (void)ud;
    /* 进入即抑制加热: 热态烙铁头需断热才能自然降温到冷态(≤50°C)完成标定;
     * 退出(保存/跳过)时由 exit 恢复加热 */
    WorkTask_SetCalSuppress(true);
    WorkTask_SetSleepSuppress(true);   /* 等降温可能数分钟, 期间抑制休眠/自动关机 */
}

static void cur_cal_exit(void *ud)
{
    (void)ud;
    cal_entered = false;   /* 下次进入重新实测 */
    cal_lock    = false;   /* 重置锁定状态 */
    WorkTask_SetCalSuppress(false);   /* 恢复加热使能 */
    WorkTask_SetSleepSuppress(false);   /* 恢复休眠/关机计时 */

    if (cal_boot)
    {
        cal_boot = false;
        WorkTask_BootCalDone();   /* 无论标定完成还是跳过, 退出引导即清除 */
    }
}

/* 混合宽度水平居中: ASCII 8px/字符, 中文 16px/字符 */
static void cur_cal_center(const char *s, uint8_t y)
{
    uint8_t w = 0;
    const char *p = s;
    while (*p)
    {
        if ((*p & 0x80) == 0) { w += 8;  p++; }
        else                  { w += 16; p += 3; }
    }
    OLED_ShowChinese((int16_t)(128 - (int16_t)w) / 2, y, s);
}

static menu_period_t cur_cal_display(void *ud)
{
    (void)ud;
    uint16_t ref = TypeCal_GetRef(cal_idx);

    if (!cal_entered)
    {
        /* 热态等待降温: 显示实时温度, 冷态后才实测 */
        if (WorkTask_GetMeasuredTemp() > TYPE_CUR_COOL_MAX)
        {
            OLED_Clear();
            cur_cal_center(cur_cal_title(), 0);
            OLED_ShowChinese(8, 16, "降温");
            OLED_ShowNum(48, 16, (uint16_t)WorkTask_GetMeasuredTemp(), 3);
            OLED_ShowDegree(72, 16);            /* 度符号与其它状态页统一 */
            OLED_ShowChar(80, 16, 'C', OLED_8X16);
            OLED_ShowString(29, 56, "HLD: Skip", OLED_6X8);   /* 6x8 9 字 70px 居中 */
            OLED_UpdateDiff();
            return CUR_CAL_COOL_PERIOD;   /* 周期复查, 降温后自动开始实测 */
        }

        cal_entered = true;
        cur_cal_measure();
    }

    OLED_Clear();

    cur_cal_center(cur_cal_title(), 0);

    /* 第1行: 选中型号 + 已存参考值 (---- 表示未标定)
     * 型号最长 4 字符(T245/T210)占 32px, 冒号定在 x=40 不遮盖第4字符
     * (原 x=32 会盖掉 '5'/'0', 显示成 T24/T21); 参考值列 x=48 与 Meas 行对齐 */
    OLED_ShowString(8, 16, cal_names[cal_idx], OLED_8X16);
    OLED_ShowChar(40, 16, ':', OLED_8X16);
    if (ref == 0)
        OLED_ShowString(48, 16, "----", OLED_8X16);
    else
        OLED_ShowNum(48, 16, ref, 4);

    /* 第2行: 本次实测值 (---- 表示测量无效) */
    OLED_ShowChinese(8, 32, "实测");
    if (meas_ok)
        OLED_ShowNum(48, 32, meas_cur, 4);
    else
        OLED_ShowString(48, 32, "----", OLED_8X16);

    /* 操作提示 (6x8, 统一"键名: 动作"形式, 两行各居中):
     * CLK: Save = 保存实测参考值; 引导模式 HLD: Skip = 跳过标定
     * 保存成功反馈由 Toast_Saved 弹出框承担 (见 page/toast.h) */
    OLED_ShowString(29, 48, "CLK: Save", OLED_6X8);   /* 9 字 70px x29..99 */
    OLED_ShowString(29, 56, cal_boot ? "HLD: Skip" : "HLD: Exit", OLED_6X8);   /* 9 字 70px x29..99 */

    OLED_UpdateDiff();
    return 0;
}

void CurCal_BootEnter(void)
{
    cal_boot    = true;
    cal_entered = false;   /* 进入后重新实测 (自检冷态, 立即测) */
    if (TypeCal_GetRef(Config_GetHandleIdx()) == 0)
    {
        /* 首次引导 (First Cal): 当前型号未标定 → 锁定当前型号,
         * 防误切其他型号致引导标志被清、当前型号保护退化 */
        cal_lock = true;
        cal_idx  = Config_GetHandleIdx();
    }
    else
    {
        /* 插头不匹配引导 (Tip Cal): 当前型号已标定但自检实测电流不符
         * (插了别的头) → 按实测电流预选型号, 允许 UP/DOWN 确认后保存 */
        cal_lock = false;
        cal_idx  = WorkTask_MatchTipType(WorkTask_GetTipCur());
    }
    Input_ClearEvents();   /* 跳转前丢弃积压事件, 防目标页错位消费 */
    Menu_SetCurrentId(CUR_CAL_MENU_ID);
}

const menu_ops_t cur_cal_ops = {
    .enter   = cur_cal_enter,
    .handler = cur_cal_handler,
    .display = cur_cal_display,
    .exit    = cur_cal_exit,
};
