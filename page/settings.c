#include "menu.h"
#include "oled.h"
#include "settings.h"
#include "config.h"
#include "buzzer.h"
#include "confirm.h"
#include "work_task.h"
#include "heater.h"
#include "toast.h"

/* 固件版本字符串 (信息页显示, 发布时更新) */
#define FW_VERSION_STR  "1.0.0"

static void sleep_set_load(void);
static void sleep_set_enter(void *ud);
static menu_result_t sleep_set_handler(const menu_cmd_t *cmd, void *ud);
static menu_period_t sleep_set_display(void *ud);
static void sleep_set_exit(void *ud);
static void func_set_load(void);
static void func_set_enter(void *ud);
static menu_result_t func_set_handler(const menu_cmd_t *cmd, void *ud);
static menu_period_t func_set_display(void *ud);
static void func_set_exit(void *ud);
static void reset_enter(void *ud);
static menu_result_t reset_handler(const menu_cmd_t *cmd, void *ud);
static menu_period_t reset_display(void *ud);
static menu_period_t version_display(void *ud);

/* ==================== 休眠设置 (三参一页) ====================
 * 原为三页(休眠时间 / 休眠温度 / 自动关机), 现合并为一页多参调节.
 * 合并的理由不是"少翻两页", 而是三者存在**隐藏的强耦合**, 分页时用户无法察觉:
 *
 *   main/work_task.c 的 check_sleep_shutdown 用同一个 last_activity 判两个阈值,
 *   且"入睡"那一支带 return —— 于是
 *       shutdown_time >  sleep_time → 先保温、再关机 (唯一合理)
 *       shutdown_time <= sleep_time → 到点直接关机, **休眠(含保温温度)永不生效**
 *   而原实现对该关系零校验、UI 零提示: 用户把自动关机设成 10min、休眠设成
 *   30min, 现象只是"休眠不起作用", 极难自查.
 *
 * 合并后三个数同屏, 关系一眼可见; 冲突时页面底部直接给出文字诊断
 * (见 sleep_set_display) + 保存时蜂鸣, 不再存在静默的无效配置.
 *
 * 编辑状态机照 page/pid.c 的多参页(导航态环形移动 focus / CLICK 切调参态 /
 * LPRESS 退出); 范围与步进按字段索引, 三行各不相同. */

#define SLEEP_FIELD_NUM 3

/* 字段: 0=休眠时间(min) 1=休眠温度(°C) 2=自动关机(min).
 * 标签宽度上限 6 字符(x=4..52): 再宽会与数值框(x=58 起)重叠, 故第三行用
 * "Auto:" 而非原单页的 "AutoOff"(7 字符 = 56px 正好撞上框左边). */
/* 四个表**刻意不写长度**, 由初值推导元素数 —— 这是下面守卫能生效的前提:
 * 若写成 [SLEEP_FIELD_NUM], 元素数必然等于它, 守卫恒真、形同虚设, 而"加字段时
 * 漏给某个表补初值"会被静默补 0(合法 C), 编译期毫无提示. */
static const char *const slp_label[] = {"Sleep:", "Idle:", "Auto:"};
/* 范围取自 bsp/config.h 的单一来源: 与 config_clamp(载入收敛)同源,
 * 保证"页面能设出的值"必然通过载入校验. */
static const int16_t slp_min[]  = {SLEEP_TIME_MIN, SLEEP_TEMP_MIN, SHUTDOWN_TIME_MIN};
static const int16_t slp_max[]  = {SLEEP_TIME_MAX, SLEEP_TEMP_MAX, SHUTDOWN_TIME_MAX};
static const int16_t slp_step[] = {1, 10, 5};

/* 编辑状态机实例. base 留一份初值副本, 两处用到:
 *   ① 退出时与 val 比对, 判断"是否真的改动了值"(见 sleep_set_exit);
 *   ② 落盘失败时回滚 —— 不回滚会让运行时按新值工作而 Flash 仍是旧值, 重启后
 *      悄悄变回去, 是最误导人的静默数据丢失(与 param_exit 同口径).
 * 共 14 B(原三个 param_t 为 36 B). */
typedef struct {
    int16_t val[SLEEP_FIELD_NUM];    /* 编辑缓冲 */
    int16_t base[SLEEP_FIELD_NUM];   /* enter 时初值 */
    uint8_t focus;                   /* 聚焦字段 0..2 */
    uint8_t editing;                 /* 0=导航态, 1=调参态 */
} sleep_set_t;

static sleep_set_t slp;

/* 编译期守卫: 四个字段表必须与 SLEEP_FIELD_NUM 等长 —— 漏改任意一个都会让
 * 下面的索引访问越界(或读到静默补的 0). C99 下 const 数组成员不是整数常量
 * 表达式, 但"整个数组的元素个数"由 sizeof 得出, 是常量表达式, 故可用负数组
 * 长度断言(AC5 无 C11 _Static_assert).
 * 前提: 上面的表不能写死长度, 否则 sizeof 恒等于 SLEEP_FIELD_NUM, 守卫失效. */
typedef char slp_label_guard[(sizeof(slp_label) / sizeof(slp_label[0]) == SLEEP_FIELD_NUM) ? 1 : -1];
typedef char slp_min_guard  [(sizeof(slp_min)   / sizeof(slp_min[0])   == SLEEP_FIELD_NUM) ? 1 : -1];
typedef char slp_max_guard  [(sizeof(slp_max)   / sizeof(slp_max[0])   == SLEEP_FIELD_NUM) ? 1 : -1];
typedef char slp_step_guard [(sizeof(slp_step)  / sizeof(slp_step[0])  == SLEEP_FIELD_NUM) ? 1 : -1];

/* 取编辑缓冲判断冲突(而非 g_config): 用户调参过程中就能立刻看到诊断,
 * 不必等保存. 保存后 val 与 g_config 等价, exit 复用同一函数.
 *
 * 任一项为 0 表示该项被禁用, 不构成冲突:
 *   sleep=0    → 只关机不保温 (合理)
 *   shutdown=0 → 只保温不关机 (合理) */
static bool sleep_set_conflict(const int16_t *v)
{
    return (v[0] > 0 && v[2] > 0 && v[2] <= v[0]);
}

/* 十进位数. 用于右对齐显示 —— OLED_ShowNum 是**固定位数**(不足补前导零),
 * 直接写 3 位会把 60 显示成 "060". 休眠设置 / 功能设置两个多参页共用. */
static uint8_t set_digits(int16_t v)
{
    uint8_t n = 1;
    if (v < 0) v = 0;   /* 本页字段无负值, 防御性 */
    while (v >= 10) { v = (int16_t)(v / 10); n++; }
    return n;
}

/* 右对齐显示定长 ASCII: xr 为右边界(OLED_ShowString 步进恒为 8px). 两多参页共用 */
static void set_show_right(int16_t xr, int16_t y, const char *s)
{
    uint8_t n = 0;
    while (s[n]) n++;
    OLED_ShowString((int16_t)(xr - n * 8), y, s, OLED_8X16);
}

static void sleep_set_load(void)
{
    uint8_t i;

    slp.val[0] = (int16_t)g_config.sleep_time;
    slp.val[1] = (int16_t)g_config.sleep_temp;
    slp.val[2] = (int16_t)g_config.shutdown_time;

    /* 装载即钳位: 配置可能落在范围外(默认值改动 / 旧固件写入), 不钳会显示
     * 超范围值, 用户看到的初值就是错的. 与 param_enter 同一处理. */
    for (i = 0; i < SLEEP_FIELD_NUM; i++)
    {
        if (slp.val[i] < slp_min[i]) slp.val[i] = slp_min[i];
        if (slp.val[i] > slp_max[i]) slp.val[i] = slp_max[i];
        slp.base[i] = slp.val[i];
    }
}

static bool sleep_set_store(void)
{
    g_config.sleep_time    = (uint16_t)slp.val[0];
    g_config.sleep_temp    = (uint16_t)slp.val[1];
    g_config.shutdown_time = (uint16_t)slp.val[2];
    return Config_Save();
}

/* 编辑状态机: 导航聚焦(环形) / 调参增减(线性钳位) / CLICK 切换 / LPRESS 退出.
 * 步进乘 cmd->step 以保留编码器加速(原三页走 param_handler, 同样支持加速). */
static menu_result_t sleep_set_edit(const menu_cmd_t *cmd)
{
    switch (cmd->event)
    {
    case MENU_EVENT_UP:
    case MENU_EVENT_DOWN:
        if (!slp.editing)
        {
            if (cmd->event == MENU_EVENT_UP)
                slp.focus = (uint8_t)((slp.focus == SLEEP_FIELD_NUM - 1) ? 0 : slp.focus + 1);
            else
                slp.focus = (uint8_t)((slp.focus == 0) ? SLEEP_FIELD_NUM - 1 : slp.focus - 1);
        }
        else
        {
            int16_t step = (int16_t)(slp_step[slp.focus] * cmd->step);

            if (cmd->event == MENU_EVENT_UP)
            {
                slp.val[slp.focus] = (int16_t)(slp.val[slp.focus] + step);
                if (slp.val[slp.focus] > slp_max[slp.focus]) slp.val[slp.focus] = slp_max[slp.focus];
            }
            else
            {
                slp.val[slp.focus] = (int16_t)(slp.val[slp.focus] - step);
                if (slp.val[slp.focus] < slp_min[slp.focus]) slp.val[slp.focus] = slp_min[slp.focus];
            }
        }
        break;

    case MENU_EVENT_CLICK:
        slp.editing = (uint8_t)!slp.editing;
        break;

    case MENU_EVENT_LPRESS:
        return MENU_RET_DONE;   /* 长按一律返回上一级 */
    default:
        break;
    }
    return MENU_RET_OK;
}

static void sleep_set_enter(void *ud)
{
    (void)ud;
    sleep_set_load();
    slp.focus = 0;
    slp.editing = 0;
}

static menu_result_t sleep_set_handler(const menu_cmd_t *cmd, void *ud)
{
    (void)ud;
    return sleep_set_edit(cmd);
}

/* 三行布局, 每行 16px, 行距贴合不重叠 (row_y = 8/24/40, 底部 56 留诊断行).
 * 标签 x=4(≤6 字符), 数值右对齐到 92, 聚焦框 x=58 宽 36, 单位 x=100. */
static menu_period_t sleep_set_display(void *ud)
{
    uint8_t i;
    (void)ud;

    OLED_Clear();

    for (i = 0; i < SLEEP_FIELD_NUM; i++)
    {
        int16_t y = (int16_t)(8 + i * 16);
        int16_t v = slp.val[i];

        OLED_ShowString(4, y, slp_label[i], OLED_8X16);

        if (i == 1 && slp.val[0] == 0)
        {
            /* 休眠禁用 → 保温温度不会生效. 显示 "---" 而不是数值, 否则用户会
             * 以为它仍在起作用(实际永远不会被用作 setpoint). */
            set_show_right(92, y, "---");
        }
        else if (i != 1 && v == 0)
        {
            set_show_right(92, y, "OFF");   /* 时间项 0 = 禁用 */
        }
        else
        {
            uint8_t d = set_digits(v);
            OLED_ShowNum((int16_t)(92 - d * 8), y, v, d);

            if (i == 1)
            {
                OLED_ShowDegree(100, y);
                OLED_ShowChar(108, y, 'C', OLED_8X16);
            }
            else
                OLED_ShowString(100, y, "min", OLED_8X16);
        }
    }

    /* 聚焦框: 只框数值区, 单位留在框外(与原单参数页同一处理, 避免边框与单位粘连) */
    if (slp.editing)
        OLED_ReverseRoundRect(58, (int16_t)(8 + slp.focus * 16), 36, 16, 6);
    else
        OLED_DrawRoundRect(58, (int16_t)(8 + slp.focus * 16), 36, 16, 6, OLED_COLOR_SET);

    /* 冲突诊断行. 刻意用 ASCII 而非汉字: 汉字字库每字 32 B, 不为一句提示扩容
     * (见 build_tools/check_chars.py 的安全网约定). "Sleep too long" 的所有
     * 字符都在 ASCII 可达集内, 由 build_tools/check_ascii_font.py 兜底校验. */
    if (sleep_set_conflict(slp.val))
        OLED_ShowString(8, 56, "Sleep too long", OLED_6X8);

    OLED_UpdateDiff();
    return 0;
}

static void sleep_set_exit(void *ud)
{
    uint8_t i;
    bool changed = false;
    (void)ud;

    /* 只有"确实改动了值"才落盘. 不能只看"是否调过参": 转到上/下限后又转回来、
     * 或 UP 一下再 DOWN 回去, 值其实没变 —— 照落盘就是白擦一次 Flash(每次擦写
     * 都消耗寿命). 与 param_exit 的 val/base 比对同一口径. */
    for (i = 0; i < SLEEP_FIELD_NUM; i++)
    {
        if (slp.val[i] != slp.base[i]) { changed = true; break; }
    }
    if (!changed)
        return;   /* 未改动: 静默 */

    if (sleep_set_store())
    {
        /* 保存成功但配置冲突: 用蜂鸣代替"已保存"弹框 —— 弹"已保存"会让用户
         * 以为一切正常, 而页面上那行 "Sleep too long" 才是他需要看到的. */
        if (sleep_set_conflict(slp.val))
            Buzzer_Beep(BUZZER_EVENT_ALARM);
        else
            Toast_Saved();
    }
    else
    {
        /* 落盘失败: 回滚 RAM 副本, 让配置与 Flash 保持一致(与 param_exit 同口径) */
        for (i = 0; i < SLEEP_FIELD_NUM; i++)
        {
            slp.val[i] = slp.base[i];
        }
        g_config.sleep_time    = (uint16_t)slp.base[0];
        g_config.sleep_temp    = (uint16_t)slp.base[1];
        g_config.shutdown_time = (uint16_t)slp.base[2];
        Buzzer_Beep(BUZZER_EVENT_ALARM);
    }
}

const menu_ops_t sleep_set_ops = {
    .enter = sleep_set_enter, .handler = sleep_set_handler,
    .display = sleep_set_display, .exit = sleep_set_exit,
};

/* ==================== 功能设置 (两参一页) ====================
 * 原为两页(蜂鸣器 / 屏幕亮度), 现合并为一页多参调节, 与"休眠设置"同一形态.
 * 合并理由: 二者同属"人机反馈"类偏好, 同屏可见更顺手; 且亮度在调参时即可
 * 预览(写对比度), 与开关同屏更直观.
 *
 * 编辑状态机照 sleep_set_ops: 导航态环形移动 focus / CLICK 切调参态 / LPRESS 退出.
 * 字段 0 为开关型(UP/DOWN 在 min/max 间翻转), 字段 1 为数值型(步进 + 钳位 + 实时预览). */

#define FUNC_FIELD_NUM 2

/* 字段: 0=蜂鸣器(开关) 1=屏幕亮度(10~255, 与旧亮度页同范围).
 * 五个表刻意不写长度, 由初值推导元素数 —— 这是下面守卫能生效的前提(同 sleep_set). */
static const char *const fn_label[]  = {"Buzzer:", "Bright:"};
static const int16_t      fn_min[]    = {0, 10};   /* 亮度下限 10: 更低已几乎不可见 */
static const int16_t      fn_max[]    = {1, 255};
static const int16_t      fn_step[]   = {1, 5};
static const uint8_t      fn_toggle[] = {1, 0};    /* 1=开关型(在 min/max 间翻转) */

typedef char fn_label_guard [(sizeof(fn_label)  / sizeof(fn_label[0])  == FUNC_FIELD_NUM) ? 1 : -1];
typedef char fn_min_guard   [(sizeof(fn_min)    / sizeof(fn_min[0])    == FUNC_FIELD_NUM) ? 1 : -1];
typedef char fn_max_guard   [(sizeof(fn_max)    / sizeof(fn_max[0])    == FUNC_FIELD_NUM) ? 1 : -1];
typedef char fn_step_guard  [(sizeof(fn_step)   / sizeof(fn_step[0])   == FUNC_FIELD_NUM) ? 1 : -1];
typedef char fn_toggle_guard[(sizeof(fn_toggle) / sizeof(fn_toggle[0]) == FUNC_FIELD_NUM) ? 1 : -1];

/* 编辑状态机实例. base 用于退出时判"是否真改动"与落盘失败回滚(同 sleep_set). */
typedef struct {
    int16_t val[FUNC_FIELD_NUM];
    int16_t base[FUNC_FIELD_NUM];
    uint8_t focus;
    uint8_t editing;
} func_set_t;

static func_set_t fn;

static void func_set_load(void)
{
    uint8_t i;

    fn.val[0] = g_config.buzzer_enable ? 1 : 0;
    fn.val[1] = (int16_t)g_config.brightness;

    /* 装载即钳位: 配置可能落在范围外(默认值改动 / 旧固件写入), 不钳会显示
     * 超范围值, 用户看到的初值就是错的. 与 sleep_set_load 同一处理. */
    for (i = 0; i < FUNC_FIELD_NUM; i++)
    {
        if (fn.val[i] < fn_min[i]) fn.val[i] = fn_min[i];
        if (fn.val[i] > fn_max[i]) fn.val[i] = fn_max[i];
        fn.base[i] = fn.val[i];
    }
}

/* 落盘并立即生效: 蜂鸣器开关与亮度都不必等重启(与旧两页 store 的语义一致). */
static bool func_set_store(void)
{
    g_config.buzzer_enable = (uint8_t)fn.val[0];
    g_config.brightness    = (uint8_t)fn.val[1];
    Buzzer_SetEnabled(g_config.buzzer_enable != 0);
    OLED_SetContrast(g_config.brightness);
    return Config_Save();
}

/* 亮度实时预览(旧亮度页 on_change 语义): 调参时立刻看到效果, 不必退出页面.
 * 蜂鸣器开关不在此预览 —— 与旧蜂鸣器页一致, 落盘时才生效. */
static void func_set_preview(void)
{
    OLED_SetContrast((uint8_t)fn.val[1]);
}

static menu_result_t func_set_edit(const menu_cmd_t *cmd)
{
    switch (cmd->event)
    {
    case MENU_EVENT_UP:
    case MENU_EVENT_DOWN:
        if (!fn.editing)
        {
            if (cmd->event == MENU_EVENT_UP)
                fn.focus = (uint8_t)((fn.focus == FUNC_FIELD_NUM - 1) ? 0 : fn.focus + 1);
            else
                fn.focus = (uint8_t)((fn.focus == 0) ? FUNC_FIELD_NUM - 1 : fn.focus - 1);
        }
        else if (fn_toggle[fn.focus])
        {
            /* 开关型: 在 min/max 间翻转(与 param_handler 的 toggle 同语义) */
            fn.val[fn.focus] = (fn.val[fn.focus] == fn_min[fn.focus]) ? fn_max[fn.focus]
                                                                     : fn_min[fn.focus];
        }
        else
        {
            int16_t step = (int16_t)(fn_step[fn.focus] * cmd->step);

            if (cmd->event == MENU_EVENT_UP)
            {
                fn.val[fn.focus] = (int16_t)(fn.val[fn.focus] + step);
                if (fn.val[fn.focus] > fn_max[fn.focus]) fn.val[fn.focus] = fn_max[fn.focus];
            }
            else
            {
                fn.val[fn.focus] = (int16_t)(fn.val[fn.focus] - step);
                if (fn.val[fn.focus] < fn_min[fn.focus]) fn.val[fn.focus] = fn_min[fn.focus];
            }
            func_set_preview();   /* 亮度即时可见 */
        }
        break;

    case MENU_EVENT_CLICK:
        fn.editing = (uint8_t)!fn.editing;
        break;

    case MENU_EVENT_LPRESS:
        return MENU_RET_DONE;   /* 长按一律返回上一级 */
    default:
        break;
    }
    return MENU_RET_OK;
}

static void func_set_enter(void *ud)
{
    (void)ud;
    func_set_load();
    fn.focus = 0;
    fn.editing = 0;
}

static menu_result_t func_set_handler(const menu_cmd_t *cmd, void *ud)
{
    (void)ud;
    return func_set_edit(cmd);
}

/* 两行参数(y=8 / y=24) + 底部亮度条(y=46): 条长随亮度值同步变化.
 * 标签 x=4, 数值右对齐到 92, 聚焦框 x=58 宽 36(与 sleep_set 同一几何). */
static menu_period_t func_set_display(void *ud)
{
    uint8_t i;
    (void)ud;

    OLED_Clear();

    for (i = 0; i < FUNC_FIELD_NUM; i++)
    {
        int16_t y = (int16_t)(8 + i * 16);

        OLED_ShowString(4, y, fn_label[i], OLED_8X16);

        if (fn_toggle[i])
        {
            set_show_right(92, y, fn.val[i] ? "ON" : "OFF");
        }
        else
        {
            uint8_t d = set_digits(fn.val[i]);
            OLED_ShowNum((int16_t)(92 - d * 8), y, fn.val[i], d);
        }
    }

    /* 亮度指示条 (算法与旧亮度页一致: 满宽 = OLED_WIDTH-16) */
    if (fn.val[1] > 0)
        OLED_FillRect(8, 46, (uint8_t)((uint32_t)(OLED_WIDTH - 16) * fn.val[1] / 255), 10,
                      OLED_COLOR_SET);

    if (fn.editing)
        OLED_ReverseRoundRect(58, (int16_t)(8 + fn.focus * 16), 36, 16, 6);
    else
        OLED_DrawRoundRect(58, (int16_t)(8 + fn.focus * 16), 36, 16, 6, OLED_COLOR_SET);

    OLED_UpdateDiff();
    return 0;
}

static void func_set_exit(void *ud)
{
    uint8_t i;
    bool changed = false;
    (void)ud;

    /* 只有"确实改动了值"才落盘(转到上/下限又转回来、UP 一下再 DOWN 回去都不算),
     * 避免白擦一次 Flash. 与 sleep_set_exit / param_exit 同一口径. */
    for (i = 0; i < FUNC_FIELD_NUM; i++)
    {
        if (fn.val[i] != fn.base[i]) { changed = true; break; }
    }
    if (!changed)
        return;   /* 未改动: 静默 */

    if (func_set_store())
    {
        Toast_Saved();
    }
    else
    {
        /* 落盘失败: 回滚 RAM 副本与**已生效**的亮度/蜂鸣器, 使硬件与 Flash 一致.
         * 不回滚会让用户看到"设置生效了"而重启后变回去(最误导人的静默丢失). */
        for (i = 0; i < FUNC_FIELD_NUM; i++)
        {
            fn.val[i] = fn.base[i];
        }
        g_config.buzzer_enable = (uint8_t)fn.base[0];
        g_config.brightness    = (uint8_t)fn.base[1];
        Buzzer_SetEnabled(g_config.buzzer_enable != 0);
        OLED_SetContrast(g_config.brightness);
        Buzzer_Beep(BUZZER_EVENT_ALARM);
    }
}

const menu_ops_t func_set_ops = {
    .enter = func_set_enter, .handler = func_set_handler,
    .display = func_set_display, .exit = func_set_exit,
};

/* ==================== 恢复默认 ==================== */
static confirm_t reset_cf;

static void reset_enter(void *ud)
{
    (void)ud;
    confirm_open(&reset_cf);   /* 默认停在 "NO" */
}

static menu_result_t reset_handler(const menu_cmd_t *cmd, void *ud)
{
    (void)ud;
    bool confirmed = false;
    menu_result_t r = confirm_handle(cmd, &reset_cf, &confirmed);
    if (confirmed)
    {
        bool saved = Config_Reset();   /* 内部已写 Flash (恢复默认并保存) */
        /* 配置已回到默认型号(T12)/上限(450)/PID 参数: 走型号切换的安全收敛链路,
         * 重载 PID、按新上限钳位目标、对齐引导标志并重评型号故障.
         * 该函数开头断热(防切换瞬间残余功率), 此处按"离开换头页"同一方式恢复使能:
         * ACTIVE 态没有其他 Heater_Enable 路径, 不补会静默永久停加热. */
        WorkTask_OnHandleChanged();
        if (!WorkTask_HasFault())
            Heater_Enable();
        /* 应用亮度与蜂鸣器默认值 */
        OLED_SetContrast(g_config.brightness);
        Buzzer_SetEnabled(g_config.buzzer_enable != 0);
        /* 仅"确实落盘成功"才弹"已保存" —— 与 param_exit/pid 页同一口径;
         * 写失败却提示已保存, 会让用户以为设置已生效, 重启后却发现回到旧值. */
        if (saved)
            Toast_Saved();
        else
            Buzzer_Beep(BUZZER_EVENT_ALARM);
        return MENU_RET_DONE;
    }
    if (r == MENU_RET_DONE)
        return MENU_RET_DONE;   /* 选 NO / 长按: 取消并退出 */
    return MENU_RET_OK;
}

static menu_period_t reset_display(void *ud)
{
    (void)ud;
    OLED_Clear();

    confirm_display(&reset_cf, "恢复默认?");

    OLED_UpdateDiff();
    return 0;
}

const menu_ops_t reset_ops = {
    .enter = reset_enter,
    .handler = reset_handler,
    .display = reset_display,
};

/* ==================== 版本信息 (只读页: 任意 CLICK/LPRESS 退出, 由本页 handler 裁决) ==================== */
static menu_result_t version_handler(const menu_cmd_t *cmd, void *ud)
{
    (void)ud;
    if (cmd->event == MENU_EVENT_CLICK || cmd->event == MENU_EVENT_LPRESS)
        return MENU_RET_DONE;   /* 结束本页(叶子项) → 回父列表 */
    return MENU_RET_OK;         /* UP/DOWN: 只读页无内容可滚, 忽略留守 */
}

static menu_period_t version_display(void *ud)
{
    char ver[24];
    (void)ud;
    OLED_Clear();

    /* 4 行 16px 高铺满屏无重叠: y0/16/32/48
     * 标题 4 汉字 64px 居中; 信息行 ASCII 与汉字混合 (OLED_ShowChinese) */
    OLED_ShowChinese(32, 0, "智能烙铁");
    /* 版本号取自 FW_VERSION_STR 单点定义: 发布时只改宏, 不必再翻渲染代码.
     * 拼接用 oled_pair_cat 而非 snprintf: 避免为 2 处文案把 stdio 格式化
     * 机制整个拉进链接(实测约 2.3KB, 含随 printfa 一并引入的浮点支持). */
    oled_pair_cat(ver, sizeof(ver), "固件版本:", FW_VERSION_STR);
    OLED_ShowChinese(8, 16, ver);
    OLED_ShowChinese(8, 32, "芯片:F103RC");
    OLED_ShowChinese(8, 48, "屏幕:128x64");

    OLED_UpdateDiff();
    return 0;
}

const menu_ops_t version_ops = {
    .handler = version_handler,
    .display = version_display,
};
