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
/* 型号显示名不在此另存一份: 统一用 Config_GetHandleNameByIdx()(见 bsp/config.h) */

/* 冷态标定温度门共用 TYPE_CUR_COOL_MAX (见 work_task.h):
 * 烙铁头高于此温度时阻值已偏移, 标定不可靠, 需等降温 */
/* 等降温时页面刷新周期 (ms) */
#define CUR_CAL_COOL_PERIOD  200
/* 测量脉冲进行中的轮询周期 (ms): 非阻塞测量需页面周期驱动取结果 */
#define CUR_CAL_MEAS_PERIOD   50

static uint8_t  cal_idx;      /* 当前选中的型号索引 */
static uint16_t meas_cur;     /* 本次实测脉冲电流 ADC */
static bool     meas_ok;      /* 本次测量是否有效 */
static bool     meas_busy;    /* 测量脉冲进行中(等待结果) */
static bool     cal_entered;  /* 本次进入是否已实测 */
static bool     cal_boot;     /* 引导模式 (First Cal / Tip Cal) */
static bool     cal_lock;     /* 引导模式锁定型号: 首次(未标定)锁定, 插头不匹配放开 */

static const char *cur_cal_title(void);
static void cur_cal_measure(void);
static bool cur_cal_poll_measure(void);
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

/* 发起测量脉冲(非阻塞): 只启动, 结果由 cur_cal_poll_measure 轮询取回.
 * 未受理(故障/自检未完成/非冷态/已有测量在跑)时蜂鸣, 并保持 meas_ok=false. */
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

    if (!WorkTask_PulseStart(true))
    {
        Buzzer_Beep(BUZZER_EVENT_ALARM);   /* 非冷态: 阻值偏移, 标定不可靠 */
        return;
    }
    meas_busy = true;
}

/* 轮询测量结果. 返回 true = 仍在测量中(本轮实测值暂显示为空). */
static bool cur_cal_poll_measure(void)
{
    uint16_t cur = 0;
    work_pulse_t st;

    if (!meas_busy)
        return false;

    st = WorkTask_PulsePoll(&cur);
    if (st == WORK_PULSE_BUSY)
        return true;               /* 结果未就绪: 下个周期再问 */

    /* READY = 取到结果; IDLE = 脉冲被中止(锁存故障/关机等).
     * 二者都必须收尾 —— 只认 READY 会让 meas_busy 永远为真, 页面以 50ms 周期
     * 无限重绘且标定永不落定. */
    meas_busy = false;
    if (st != WORK_PULSE_READY || cur == 0)
    {
        Buzzer_Beep(BUZZER_EVENT_ALARM);   /* 链路异常/被中止: 标定不可靠 */
    }
    else
    {
        meas_cur = cur;
        meas_ok  = true;
    }
    return false;
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
            /* 仅"确实落盘成功"才提示已保存(与 settings/param/pid 页同一口径):
             * 写失败却弹"已保存", 用户会以为参考值已生效, 重启后回到旧值且无从察觉. */
            if (TypeCal_SetRef(cal_idx, meas_cur))
            {
                Buzzer_Beep(BUZZER_EVENT_TEMP_REACHED);
                Toast_Saved();   /* 人性化反馈: 弹出"已保存"框 */
            }
            else
            {
                Buzzer_Beep(BUZZER_EVENT_ALARM);   /* 落盘失败: 不弹"已保存" */
                break;
            }
            /* 当前配置型号已标定 → 清除首次引导标志 (手动/引导路径均幂等;
             * 标定的是其他型号时不清, 引导仍待处理) */
            if (cal_idx == Config_GetHandleIdx())
                WorkTask_BootCalDone();
            if (cal_boot)
            {
                /* 跳转后由内核重同步编码器相位(见 input.c 的 menu_ctx) */
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
            /* 跳转后由内核重同步编码器相位(见 input.c 的 menu_ctx) */
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
    meas_busy   = false;
    WorkTask_PulseAbort();            /* 未完成的测量: 断热作废, 防残留占空比 */
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
    menu_period_t period = 0;

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

    /* 测量脉冲进行中: 周期轮询取结果; 未就绪则本轮后 50ms 再渲染一次.
     * 期间实测值保持空(meas_ok=false), 结果到达后下一帧自动显示. */
    if (cur_cal_poll_measure())
        period = CUR_CAL_MEAS_PERIOD;

    OLED_Clear();

    cur_cal_center(cur_cal_title(), 0);

    /* 第1行: 选中型号 + 已存参考值 (---- 表示未标定)
     * 型号最长 4 字符(T245/T210)占 32px, 冒号定在 x=40 不遮盖第4字符
     * (原 x=32 会盖掉 '5'/'0', 显示成 T24/T21); 参考值列 x=48 与 Meas 行对齐 */
    OLED_ShowString(8, 16, Config_GetHandleNameByIdx(cal_idx), OLED_8X16);
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
    return period;
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
    Menu_SetCurrentId(CUR_CAL_MENU_ID);
    /* 输入复位必须在**跳转之后**: 跳转会经 menu_exit_handler 调当前页的 exit, 若那
     * 是参数页且 dirty 会落盘(Flash 擦写 25~50ms), 期间 TIM2 照常把编码器/按键事件
     * 入队 —— 先清后跳会漏掉这一批, 它们会被本页错位消费(误切 Tip Cal 预选型号,
     * cal_lock=false 时尤其明显)。详见 Input_ResetForJump 的说明。 */
    Input_ResetForJump();
}

const menu_ops_t cur_cal_ops = {
    .enter   = cur_cal_enter,
    .handler = cur_cal_handler,
    .display = cur_cal_display,
    .exit    = cur_cal_exit,
};
