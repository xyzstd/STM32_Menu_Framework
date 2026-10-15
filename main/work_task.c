#include "work_task.h"
#include "timer.h"
#include "adc.h"
#include "pid_controller.h"
#include "heater.h"
#include "config.h"
#include "buzzer.h"
#include "stm32f10x.h"

/*
 * 后台工作任务 (参考 Cai-Zi/STM32_T12_Controller)
 *
 * 全局后台任务, 负责:
 *   - 分段式 PID 温度闭环控制 (200ms 周期)
 *   - 输入电压检测 (PA2)
 *   - 运动检测 (震动开关, PC13)
 *   - 休眠/自动关机计时
 *   - 过温保护
 *   - 温度到达检测 + 蜂鸣器提示
 *   - 加热器故障检测
 *   - 热电偶短路/开路检测 (锁存断热)
 *   - 上电硬件链路自检 (热电偶/发热芯/电流/电压, 失败锁存故障码)
 */

/* ========== 常量 ========== */

#define PID_INTERVAL_MS          200    /* PID 控制周期 (ms) */
#define TEMP_REACHED_THRESHOLD   5      /* 温度到达阈值 */
/* 到达判定去抖: 需连续 N 个 PID 周期都落在阈值内才置"到达"(200ms/周期).
 * 单帧判定在阈值附近会被滤波残余抖动反复翻转 —— 虽已用 prev 去重避免重复鸣叫,
 * 但偶发的单帧越界仍会打断"已到达"状态并再触发一次蜂鸣. 取 2 个周期(400ms)
 * 既滤掉单帧毛刺, 又保持提示及时. */
#define TEMP_REACHED_CNT         2
#define OVER_TEMP_LIMIT          500    /* 过温保护阈值 (°C) */
/* 温度换算合理上界: 必须高于 OVER_TEMP_LIMIT. 否则 adc_to_temp 把读数钳到
 * OVER_TEMP_LIMIT 后, run_pid_cycle 的 `meas_temp > OVER_TEMP_LIMIT` 永不成立,
 * 过温保护形同虚设。 */
#define TEMP_SANITY_MAX          600    /* 温度读数上界 (°C), 须 > 过温阈值 */

/* 热电偶失效(短路/断线)检测: 温度 ADC 低于此阈值判短路/断线 → 锁存断热.
 * 校准原点: 0°C≈0, 25°C≈198; 烙铁正常工作温度远高于室温, 留裕量防误判.
 * 开路(运放饱和近满量程)由下方 THERMO_ADC_OPEN 对称检测. */
#define THERMO_ADC_MIN           50

/* 热电偶开路/未插检测阈值: 温度 ADC ≥ 此值(运放输出近饱和域)判开路 → 锁存断热.
 * 硬件要求: 校准后 450°C 点的 ADC 需显著低于此值(建议 ≤3950), 否则真实高温会被
 * 误判开路; 出厂占位(450°C=4095)为未标定状态, 真机必须重新校准后方可上 450°C. */
#define THERMO_ADC_OPEN          4000
#define THERMO_OPEN_COUNT        8      /* 连续 8 个 PID 周期(1.6s)达饱和才锁存, 吸收尖峰 */

/* 室温常量: 上电初始实测温度/诊断基准 (硬件无 NTC, 不再有手柄温度传感器) */
#define ROOM_TEMP                25

/* 输入电压: 分压比 11:1, 3.3V 参考 (换算见 calc_vin_x10, 全整数) */

/* 加热器故障检测: 仅当"已按满功率输出且温差仍大"时, 判"窗口内温升不足"为失效.
 * 窗口由 10s 放宽到 20s: 压在大型铜箔/地平面时温度可能被吸住, 短窗口内温升不足会
 * 误锁 heater_fault. 真失效(断芯)在窗口内同样必然温升不足, 且上电自检/热电偶开路
 * 会先一步兜底, 放宽窗口不削弱检测能力。 */
#define DIAG_CHECK_MS            20000
#define DIAG_TEMP_RISE_THRESHOLD 5

/* 运行时目标温度 (RAM, 不落 flash): HOME 页临时调解的温度保存在此,
 * 整个运行期间持续生效, 休眠唤醒时恢复到该值而非默认值. 掉电丢失. */
static int16_t  runtime_temp;

/* 冷端补偿基准: CPU 内部温度(°C×10). 上电瞬间冷端≈环境≈CPU,
 * 之后环境/冷端漂移用 CPU 温度相对变化量近似捕捉, 规避 V25 芯片间散布.
 * 基准在上电 2s、滤波收敛后延后重取 (避开开机瞬时值).
 * 仅真机模式使用(模拟模式无真实 ADC, 一并编译掉, 免生"未使用"告警). */
#if !HW_SIM_MODE
static int16_t  cpu_temp_ref;

/* CPU 温度一阶低通滤波值: 补偿只应携带慢变的环境/冷端漂移,
 * 滤除内部温度传感器采样噪声与 MCU 自热快变分量 (见 CPU_TEMP_FILT_K). */
static int16_t  cpu_temp_filt;
static uint32_t boot_tick;      /* 上电时刻 (冷端基准延后采样用) */
static bool     cpu_ref_ready;  /* 冷端基准已就绪(延后采样完成) */
#endif

/* 休眠/关机计时抑制: 校准等长流程置位, 期间不入睡(整定期间同样抑制, 见下方).
 * 仅真机模式使用(模拟模式整体禁用休眠/关机计时). */
#if !HW_SIM_MODE
static bool     sleep_suppress;
#endif

/* 校准页(Cur Cal)加热抑制: 页面期间只维持温度读数供"Cooling"显示,
 * 断热不控热, 让热态烙铁头自然降温到冷态后完成标定. */
static bool     cal_suppress;

/* 插头不匹配引导断热: 自检发现当前配置型号已标定、但插入头电流不符
 * (插了另一量级的头)时置位, 引导用户进 Cur Cal 标定新头前持续断热,
 * 标定完成或跳过后由 BootCalDone 清除. */
static bool     tip_guide;

/* 本次自检是否已由"型号不匹配(无害)"建立引导: HEAT_READ 置位, SELFTEST_VIN
 * 据此跳过 boot_cal_eval() —— 否则会按"当前配置型号"重判并清掉 tip_guide,
 * 使不匹配的烙铁头被加热. 每轮自检在 SELFTEST_THERMO 入口复位.
 * 仅真机模式使用(自检状态机整体在 #if !HW_SIM_MODE 内, 模拟模式无引用). */
#if !HW_SIM_MODE
static bool     type_guide_boot;
#endif

/* 温度 ADC 一阶低通滤波系数 (定点): K = 192/256 ≈ 0.75.
 * 高频噪声已由 ADC 8 帧过采样抑制, 此低通仅保留中频平滑 → K 提高到 0.75,
 * 90% 阶跃响应从约 3.3 个 PID 周期降到约 1.7 个, 相位延迟减半、更跟手. */
#define TEMP_FILT_K              192

/* 冷端补偿: CPU 内部温度一阶低通系数 (定点): K = 7/1024 ≈ 0.0068,
 * τ ≈ 29s @200ms 周期 —— 只保留分钟级的环境/冷端漂移分量,
 * 滤除采样噪声与 MCU 自热快变(自热上升沿被滤成缓慢趋势, 配合 K=1 不会
 * 引入读数虚高; 若真机对比发现长时间焊接读数偏正, 可再加衰减系数 K<1). */
#define CPU_TEMP_FILT_K           7

/* 冷端基准延后采样: 上电 2s、滤波收敛后重取 cpu_temp_ref, 避开开机瞬时值 */
#define CPU_REF_DELAY_MS          2000

/* ========== 硬件抽象开关 ==========
 *  HW_SIM_MODE 已提升为全局共享宏(定义于 config.h): 1=模拟, 0=真机.
 * 模拟模式: 无真实外设/手柄, 所有传感器输入由程序"假装"为硬件已就绪,
 *           便于跑通整条加热/PID/休眠逻辑, 屏幕会显示已连接手柄.
 * 真实模式: 读取真实 ADC / GPIO. 切换只需改 config.h 一处, 不触碰业务逻辑. */

/* 模拟加热响应: 每个 PID 周期(200ms)让仿真温度向设定值逼近的格数 */
#define SIM_HEAT_RATE             4

/* 过流保护阈值 (注入组电流 ADC 码):
 * 电流经采样电阻 R_s → 运放B → ADC。阈值按"最大允许电流对应的运放输出 ADC 码"设定,
 * 超过即判过流 → 断热。具体数值需按实际 R_s 与运放增益标定。 */
#define OVERCURRENT_LIMIT         3500

/* ========== 上电硬件链路自检 (SELFTEST) ==========
 * 冷态(开机未加热)一次性执行, 依次判定:
 *   1) 热电偶短路/开路 (ADC 区间判定)
 *   2) 发热芯断芯/短路: 施加热芯测量脉冲(10% 占空比), 读注入组导通电流
 *      (瞬时导通电流不随占空比变化, 只反映发热芯与电流链路健康);
 *      电流≈0 → 断芯或电流采样失效; 电流近饱和 → 发热芯短路
 *   3) 输入电压范围 (9V~30V)
 * 任一失败置 selftest_fault 锁存断热并记录故障码(UI 显示).
 * 判定的是链路完整性(ADC 区间/电流/电压), 与烙铁温度无关,
 * 故热态亦可用 WorkTask_Recover() 重新复测: 清除锁存 → 重跑本链路 → 通过才恢复. */
#define SELFTEST_PULSE_POWER     100    /* 测量脉冲: 10% 占空比 (0~1000), 200ms 温升可忽略 */
#define SELFTEST_CUR_MIN         30     /* 脉冲下电流 ADC < 此值 → 断芯/无电流 */
#define SELFTEST_CUR_MAX         3950   /* 脉冲下电流 ADC ≥ 此值 → 发热芯短路 */
#define SELFTEST_VIN_MIN_x10     90     /* 供电低于 9.0V 判电源异常 */
#define SELFTEST_VIN_MAX_x10     300    /* 供电高于 30.0V 判电源异常 */

/* 出厂经验门限(最后兜底): 非 T12 型号配置下, 实测脉冲电流低于此值按"T12 量级"拒加热.
 * T12 冷阻(≈8Ω)远高于 T245/T210(≈2/3Ω), 同占空比下导通电流仅约其 1/4.
 * 仅在"T12 与参与比对的型号都未标定"时才会用到(见 is_t12_scale); 那种状态 UI 会先
 * 引导 Cur Cal 标定, 正常使用到不了这里. 0 = 不拦截.
 * 注意: 它不再是"T12 未标定"时的唯一门限 —— 那会让粗保护退化为放行(已修). */
#define TYPE_CUR_COARSE_T12_MAX  0

/* ========== 发热芯型号匹配 (两级保护) ==========
 * 测量脉冲导通瞬间电流只由发热芯冷阻决定 (T12≈8Ω / T245≈2Ω / T210≈3Ω),
 * 故导通中点采样的电流 ADC 可区分型号.
 *
 * 第1级 细匹配: 当前型号已标定(参考值≠0)时, 实测须落在 参考值±30% 区间,
 *               裕量覆盖批次差异与手柄接口接触电阻; 不匹配 → 拒加热.
 * 第2级 粗保护: 防最危险场景"T12 头插进非 T12 型号(T245/T210)配置".
 *               T12 冷阻最高 → 同脉冲电流量级仅约 450°C 型号的 1/4, 故对
 *               非 T12 配置, 实测电流低于"T12 量级上限"即拒.
 *               门限取 T12 参考 ×1.8 (T12 头实测≤1.3×ref, 450°C 型号实测
 *               ≥2.8×ref, 1.8 落在两量级之间留足裕量); T12 未标定时退化为
 *               出厂经验门限 TYPE_CUR_COARSE_T12_MAX (0=待真机标定前不拦截).
 *
 * 保护依据(与温度上限无关): 温度换算曲线(cal_t)与 PID 增益都是按**配置型号**
 *               标定的; 插错头后读数与实际温度不再对应、增益也失配, 实际温度
 *               可能明显偏离目标. 三型号上限统一为 450 后这一点**并未改变** ——
 *               该保护依然有效, 只是不再以"450 超过 T12 额定"为由.
 *               另有实际收益: 及时告知"插错头", 而不是静默用错误曲线控温
 *               (那会让用户按错误读数焊接).
 *
 * 注: 三型号温度上限已统一为 450°C, 判据一律按"配置型号是否为 T12", 不再用
 *     原来的 "temp_max>400" —— 统一上限后该式恒真, 会让 T12 头在任何配置下被拒.
 *
 * [2026-09-22 升级] 三档判型取代单参考 ±30% 细匹配:
 *   以 T12 冷态参考为锚, 两个边界把 T12/T210/T245 三档切开, 判型结果与
 *   配置型号比对. 解决旧 ±30% 窗口对 T245/T210(阻值仅差 1.5 倍)裕量仅
 *   ~4.6% 不可靠的问题. 边界裕量: T12 侧 15%+, T210/T245 侧 20%+.
 *   分级处置: 危险(T12 量级进非 T12 配置) → 锁 Err4; 无害(450°C 级互插、
 *   T12 配置插 450°C 头) → 引导 Tip Cal 校正不锁死. */

#if !HW_SIM_MODE
static uint8_t classify_tip_cur(uint16_t cur);
static bool is_t12_scale(uint16_t cur, uint8_t idx);
#endif
static bool type_cur_match(uint16_t cur, uint8_t idx);

#if !HW_SIM_MODE
static int32_t spline_interp(const int32_t *x, const int32_t *y, uint8_t n, int32_t xv);
static int16_t cal_offset_interp(int16_t temp_base);
static int16_t adc_to_temp(uint16_t adc_val);
#endif

#if !HW_SIM_MODE
static uint16_t calc_vin_x10(uint16_t adc);
#endif
static void read_input_voltage(void);
static void read_motion_sensor(void);
#if !HW_SIM_MODE
static void remember_last_temp(void);
#endif
static void check_sleep_shutdown(uint32_t now);

static void latch_fault(bool *fault_flag);
static void check_heater_diagnostic(uint32_t now);

static void run_pid_cycle(void);

#if !HW_SIM_MODE
static void boot_cal_eval(void);
static void selftest_advance(void);
#endif

static void WorkTask_Retry(void);

/* 非阻塞测量脉冲推进: 两种模式都有实现(模拟模式在 Start 里直接置就绪) */
static bool pulse_step(void);


#if !HW_SIM_MODE
static uint8_t classify_tip_cur(uint16_t cur)
{
    uint16_t ref = TypeCal_GetRef(HANDLE_T12);
    if (ref == 0)
        return 0xFF;   /* T12 未标定: 无法归一化判型, 调用方退化粗保护 */
    if (cur < (uint16_t)((uint32_t)ref * 15 / 10))
        return HANDLE_T12;    /* <1.5×ref: T12 量级 (T12 头实测≤1.3×ref, 热态更低) */
    if (cur < (uint16_t)((uint32_t)ref * 33 / 10))
        return HANDLE_T210;   /* 1.5~3.3×ref: T210 量级 (2.67 居中) */
    return HANDLE_T245;       /* >3.3×ref: T245 量级 (4.0) */
}

/* "T12 量级"判定 (冷态/热态通用): 危险分流专用 —— 热态下 T245/T210 电流
 * 衰减不可判型, 但 T12 头电流仍远低于门限, 故该判定在热态依然可靠.
 * idx = 参与比对的 450°C 型号(用它的参考值反推门限).
 *   ① T12 自身已标定 → 门限 = 1.5×ref_T12 (T12 头实测 ≤1.3×ref, 留 15% 裕量).
 *   ② T12 未标定 → 改用 idx 自身参考反推: T12 冷阻(≈8Ω) 约为 T245(≈2Ω) 的 4 倍、
 *      T210(≈3Ω) 的 2.7 倍, 故同脉冲下电流仅约其 1/4 与 0.37. 取 idx 参考的 1/2
 *      作分界: T245 配置下 T12(0.25) 被判出、T210(0.67) 不误伤; T210 配置下
 *      T12(0.37) 被判出、T245(1.5) 不误伤 —— 两侧裕量均 ≥1.3 倍.
 *      (原实现在此直接返回 false, 使 T12 未标定时粗保护完全失效, 属"失败开放".)
 *   ③ idx 也未标定 → 退回出厂经验门限 TYPE_CUR_COARSE_T12_MAX(0=不拦截;
 *      该场景 UI 会先引导 Cur Cal 标定, 正常使用不会停留在这里). */
static bool is_t12_scale(uint16_t cur, uint8_t idx)
{
    uint32_t floor = (uint32_t)TypeCal_GetRef(HANDLE_T12) * 15 / 10;

    if (floor == 0)
    {
        uint32_t self = TypeCal_GetRef(idx);
        floor = (self != 0) ? (self / 2) : TYPE_CUR_COARSE_T12_MAX;
    }

    return floor != 0 && (uint32_t)cur < floor;
}
#endif

static bool type_cur_match(uint16_t cur, uint8_t idx)
{
#if !HW_SIM_MODE
    /* T12 参考已标定: 三档判型 (冷态权威判定) */
    uint8_t cls = classify_tip_cur(cur);
    if (cls != 0xFF)
    {
        /* 危险: 非 T12 型号配置下判为 T12 量级 → 必锁 (烧 T12 头).
         * 判据按"型号"而非温度上限 —— 三型号上限已统一为 450, 原 "temp_max>400"
         * 会恒真, 导致连 T12 配置也被拒(无法使用 T12 头). */
        if (cls == HANDLE_T12 && idx != HANDLE_T12)
            return false;
        /* 判型与配置型号不一致: 含 T12 配置插 450°C 头、T245/T210 互插
         * (无害但需校正) → 上层分流引导, 本层统一判不匹配 */
        if (cls != idx)
            return false;
        return true;
    }
    /* T12 未标定: 仍用 idx 自身参考反推粗门限 (详见 is_t12_scale) */
    if (idx != HANDLE_T12 && is_t12_scale(cur, idx))
        return false;
#endif
    return true;
}

/* 自检状态机 (仅上电阶段推进, 完成后常驻 DONE) */
typedef enum {
    SELFTEST_THERMO = 0,   /* 判定热电偶短路/开路 */
    SELFTEST_HEAT_ON,      /* 施加热芯测量脉冲 */
    SELFTEST_HEAT_READ,    /* 读电流判定断芯/短路, 断热 */
    SELFTEST_VIN,          /* 判定输入电压范围 */
    SELFTEST_DONE          /* 自检完成, 进入正常闭环 */
} selftest_state_t;

/* 自检故障码 (供 UI 显示, 0=通过) */
#define SELFTEST_CODE_NONE    0
#define SELFTEST_CODE_THERMO  1   /* 热电偶短路或开路 */
#define SELFTEST_CODE_HEATER  2   /* 发热芯断芯/短路或电流采样失效 */
#define SELFTEST_CODE_VIN     3   /* 输入电压超范围 */
#define SELFTEST_CODE_TYPE    4   /* 发热芯型号与配置不匹配 (未标定时不产生) */

/* ========== 内部状态 ========== */

static struct {
    work_state_t state;
    uint32_t     last_activity;
    uint32_t     last_pid_tick;
    int16_t      measured_temp;
    int16_t      power;
    bool         tune_save_succeeded; /* 最近一次算法成功整定是否已持久化 */
    bool         temp_reached;
    bool         temp_reached_prev;
    uint8_t      reached_cnt;        /* 到达判定去抖: 连续满足阈值的周期数 */

    /* 输入电压 */
    uint16_t     vin_adc;            /* 最新输入电压 ADC 值 */
    uint16_t     vin_voltage_x10;    /* 输入电压 *10 (如 240 = 24.0V) */

    /* 热电偶 */
    int32_t      temp_filt;          /* 温度 ADC 一阶低通滤波值 */
    uint8_t      thermo_open_cnt;    /* 开路饱和 ADC 连续计数 (防抖) */

    /* 上电硬件链路自检 */
    selftest_state_t selftest;       /* 自检状态机 */
    uint8_t          selftest_code;  /* 自检故障码, 0=通过 */
    bool             selftest_fault; /* 自检失败锁存 (经 WorkTask_Recover 重跑全链路通过才清) */
    bool             boot_cal_pending; /* 首次引导: 当前型号无冷态电流参考(未标定),
                                        * 自检通过后提示先冷态标定再加热 */
    uint16_t         tip_cur;          /* 插头不匹配引导: 自检实测脉冲电流, 供 Cur Cal 预选型号 */

    /* 运动检测 */
    bool         motion_detected;    /* 本次扫描是否检测到运动 */

    /* 加热器故障 */
    uint32_t     diag_tick;
    int16_t      diag_temp_last;

    /* 锁存保护故障 (置位后持续断热, 直到 WorkTask_Recover 手动恢复;
     * 过温只清锁存, 其余重跑自检链路, 全链路通过才恢复) */
    bool         heater_fault;      /* 加热器故障 (功率>0 但长时间无温升 / 过流) */
    bool         thermo_fault;      /* 热电偶短路/断线 */
    bool         overtemp_fault;    /* 过温 */
} work;


/* ========== 内部函数 ========== */

#if !HW_SIM_MODE
/* 单调三次样条(Fritsch-Carlson 保单调)插值, 替换分段线性插值.
 * 特性: 曲线精确穿过所有校准点, 各段连续且单调(不超调/不回弯),
 *       消除分段线性在段边界产生的"拐角", 使温度读数与温度到导
 *       数在整段都平滑连续, 减少微分项噪声. 所有运算 int32 无浮点.
 * 参数: x[] 升序, y[] 对应, 共 n 点(≤8); 超出 [x0,xn-1] 返回端点值. */
#define SPLINE_SCALE   1000
static int32_t spline_interp(const int32_t *x, const int32_t *y, uint8_t n, int32_t xv)
{
    int32_t m[8], t[8];
    uint8_t i, k;
    int32_t dx, u, s_int, s2, s3, h00, h10, h01, h11, v2, v3, yy;

    if (n < 2) return 0;
    if (xv <= x[0]) return y[0];
    if (xv >= x[n - 1]) return y[n - 1];

    /* 各段斜率 (y*SCALE / x) */
    for (i = 0; i + 1 < n; i++)
        m[i] = (y[i + 1] - y[i]) * SPLINE_SCALE / (x[i + 1] - x[i]);

    /* 节点切线 (保单调 Fritsch-Carlson 钳位) */
    t[0] = m[0];
    t[n - 1] = m[n - 2];
    for (k = 1; k + 1 < n; k++)
    {
        int32_t dx_l = x[k] - x[k - 1];
        int32_t dx_r = x[k + 1] - x[k];
        int32_t mlo;
        if (m[k - 1] == 0 || m[k] == 0)   /* 某侧平缓(极点)则切线归零, 防过冲 */
        {
            t[k] = 0;
            continue;
        }
        t[k] = (m[k - 1] * dx_r + m[k] * dx_l) / (dx_l + dx_r);
        if (t[k] < 0) t[k] = 0;
        mlo = (m[k - 1] < m[k]) ? m[k - 1] : m[k];
        if (t[k] > 3 * mlo) t[k] = 3 * mlo;   /* 保证段内单调 */
    }

    /* 定位所在段 */
    k = 0;
    for (i = 0; i + 1 < n; i++)
        if (xv <= x[i + 1]) { k = i; break; }

    /* 三次 Hermite 求值 */
    dx = x[k + 1] - x[k];
    u  = xv - x[k];
    s_int = u * SPLINE_SCALE / dx;
    s2 = s_int * s_int / SPLINE_SCALE;
    s3 = s2 * s_int / SPLINE_SCALE;
    h00 = 2 * s3 - 3 * s2 + SPLINE_SCALE;
    h10 = s3 - 2 * s2 + s_int;
    h01 = (-2 * s3) + (3 * s2);
    h11 = s3 - s2;
    v2 = dx * t[k] / SPLINE_SCALE;
    v3 = dx * t[k + 1] / SPLINE_SCALE;
    yy = (h00 * y[k] + h10 * v2 + h01 * y[k + 1] + h11 * v3) / SPLINE_SCALE;
    return yy;
}

/* 温度校准偏移: 200/300/350/400/450°C 五点偏移, 用单调三次样条平滑插值
 * 5 温度点细分常用焊接带 (300~350), 曲线平滑连续; 超范围时取端点偏移. */
static int16_t cal_offset_interp(int16_t temp_base)
{
    uint8_t h = Config_GetHandleIdx();
    /* xs 是常量: 用 static const 放 Flash(RO), 免去每次调用把它拷到栈(20 B).
     * ys 依赖运行时校准值, 必须留在栈上. */
    static const int32_t xs[5] = {200, 300, 350, 400, 450};
    int32_t ys[5] = {
        Cal_GetOff(0, h),
        Cal_GetOff(1, h),
        Cal_GetOff(2, h),
        Cal_GetOff(3, h),
        Cal_GetOff(4, h),
    };
    return (int16_t)spline_interp(xs, ys, 5, temp_base);
}

/* 使用校准点进行温度转换 (线性插值 + 多点偏移), 校准点按当前型号索引 */
static int16_t adc_to_temp(uint16_t adc_val)
{
    uint8_t h = Config_GetHandleIdx();
    int32_t temp;
    int32_t adc = (int32_t)adc_val;

    /* 校准点非法(被清零/Flash损坏)时退化为线性映射, 防止除零 */
    if (!Cal_IsHandleValid(h))
    {
        return (int16_t)(adc * 120 / 4095);
    }

    /* 使用多项校准点进行温度转换:
     * 校准点之间用单调三次样条平滑插值, 低于 200°C 用原点线性, 高于 450°C 用末段斜率外推 */
    if (adc <= Cal_GetAdc(0, h))
    {
        /* 低于 200°C(最低校准点): 原点线性外推 */
        temp = 200 * adc / Cal_GetAdc(0, h);
    }
    else if (adc <= Cal_GetAdc(4, h))
    {
        /* 200°C~450°C: 校准点区间单调三次样条平滑插值, 消除分段线性拐角,
         * 5 温度点 (200/300/350/400/450) 细分常用焊接带 */
        int32_t xs[5] = {
            Cal_GetAdc(0, h),
            Cal_GetAdc(1, h),
            Cal_GetAdc(2, h),
            Cal_GetAdc(3, h),
            Cal_GetAdc(4, h),
        };
        /* ys 是常量: 用 static const 放 Flash(RO), 免去每次调用拷到栈(20 B).
         * 本函数会调用 cal_offset_interp, 两者栈帧嵌套 —— 两处常量化后峰值栈共省 40 B.
         * xs 依赖运行时校准值, 必须留在栈上. */
        static const int32_t ys[5] = {200, 300, 350, 400, 450};
        temp = spline_interp(xs, ys, 5, (int32_t)adc);
    }
    else
    {
        /* 高于 450°C: 沿用 400~450 段斜率外推 */
        int32_t span = (int32_t)Cal_GetAdc(4, h) - Cal_GetAdc(3, h);
        if (span <= 0) span = 1;
        temp = 450 + (int32_t)(adc - Cal_GetAdc(4, h)) * 50 / span;
    }

    /* 叠加按温度区间插值的校准偏移 */
    temp += cal_offset_interp((int16_t)temp);
    if (temp < 0) temp = 0;
    if (temp > TEMP_SANITY_MAX) temp = TEMP_SANITY_MAX;

    return (int16_t)temp;
}
#endif /* !HW_SIM_MODE */

/* 计算输入电压 (分压比 11:1, 3.3V 参考) */
/* Vin = ADC * 3.3 / 4095 * 11, 输出 *10 (如 240 = 24.0V) */
#if !HW_SIM_MODE
static uint16_t calc_vin_x10(uint16_t adc)
{
    /* Vin *10 = adc * 33 * 11 / 4095 = adc * 363 / 4095 */
    /* 简化为: adc * 363 / 4095 */
    return (uint16_t)((uint32_t)adc * 363 / 4095);
}
#endif

/* 读取输入电压 */
static void read_input_voltage(void)
{
#if HW_SIM_MODE
    /* 模拟: 假定电源已接入, 恒为 24.0V */
    work.vin_voltage_x10 = 240;   /* 直接固定 24.0V, 避免二次取整误差 */
#else
    work.vin_adc = AD_GetVin();
    work.vin_voltage_x10 = calc_vin_x10(work.vin_adc);
#endif
}

/* 检查运动检测 (PC13 SLEEP 引脚, 震动开关) */
static void read_motion_sensor(void)
{
#if HW_SIM_MODE
    /* 模拟: 默认静止(不触发唤醒/复位休眠计时) */
    work.motion_detected = false;
#else
    /* PC13 配置为输入, 低电平有效 (震动开关闭合时 = 0)。
     * 注: 运动检测原接 PA8, 与 TIM1_CH1(PWM 加热)冲突, 已迁移到 PC13. */
    work.motion_detected = (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == Bit_RESET);
#endif
}

/* 记忆上次使用温度: 进入休眠/自动关机(会话结束)时, 把 HOME 运行时目标温度
 * 回写配置"开机温度"(default_temp), 下次上电由 WorkTask_Init 自动恢复.
 * 仅数值变化才落盘, 避免每次休眠都产生无谓的 Flash 擦写损耗.
 * 仅真机模式使用(模拟模式不进入休眠/关机). */
#if !HW_SIM_MODE
static void remember_last_temp(void)
{
    int16_t temp = runtime_temp;
    if (temp > (int16_t)Config_GetTempMax())
        temp = (int16_t)Config_GetTempMax();
    if (temp < TEMP_MIN)
        temp = TEMP_MIN;
    if (temp != (int16_t)g_config.default_temp)
    {
        g_config.default_temp = (uint16_t)temp;
        /* 记忆温度属"尽力而为": 落盘失败无 UI 可提示(此刻正入睡/关机),
         * RAM 值仍然正确, 下次入睡会再试. */
        (void)Config_Save();
    }
}
#endif

/* 检查休眠/关机计时 */
static void check_sleep_shutdown(uint32_t now)
{
#if HW_SIM_MODE
    /* 模拟联调: 禁用休眠/自动关机计时, 长时间挂机不切状态不落盘,
     * 便于稳定复现/排查其他 bug (真机行为不变).
     * 用 #else 分块而非提前 return: 提前 return 会让下方整段成为"不可达"告警(#111). */
    (void)now;
#else
    /* SHUTDOWN 为会话终态, 无需再计时; SLEEP 必须继续计时, 否则入睡后
     * 自动关机永不触发(原实现对"非 ACTIVE"一律返回, 休眠态即免疫关机)。 */
    if (work.state == WORK_STATE_SHUTDOWN)
        return;

    /* 整定/校准等受控长流程期间不入睡: 整定通常不足 2 分钟且用户在场观察,
     * 校准需长时间等温稳定 */
    if (PID_TuneActive() || sleep_suppress)
        return;

    /* 检测到运动 → 重置计时 */
    if (work.motion_detected)
    {
        work.last_activity = now;
        return;
    }

    uint32_t elapsed_ms = now - work.last_activity;
    uint32_t elapsed_sec = elapsed_ms / 1000;

    /* 入睡判定仅限 ACTIVE: SLEEP 态若仍走此分支, 会每帧重复置休眠温度并复位 PID */
    if (work.state == WORK_STATE_ACTIVE &&
        g_config.sleep_time > 0 && elapsed_sec >= (uint32_t)g_config.sleep_time * 60)
    {
        remember_last_temp();
        work.state = WORK_STATE_SLEEP;
        PID_Controller_SetSetpoint((int16_t)g_config.sleep_temp);
        PID_Controller_Reset();
        return;
    }

    if (g_config.shutdown_time > 0 && elapsed_sec >= (uint32_t)g_config.shutdown_time * 60)
    {
        remember_last_temp();
        work.state = WORK_STATE_SHUTDOWN;
        Heater_Disable();
        return;
    }
#endif
}

/* 锁存一类保护故障并立即断热告警 (热电偶短路/开路、过流、加热器失效共用):
 * 置故障位 → 断热 → 输出归零 → 告警. 置位后由 WorkTask_Recover 手动恢复.
 * 抽成一处, 避免各保护各写一份导致漏掉"置位/告警"等步骤. */
static void latch_fault(bool *fault_flag)
{
    *fault_flag = true;
    Heater_Disable();
    work.power = 0;
    Buzzer_Beep(BUZZER_EVENT_ALARM);
}

/* 加热器故障检测 */
static void check_heater_diagnostic(uint32_t now)
{
    if (work.power <= 0)
        return;

    int16_t setpoint = PID_Controller_GetSetpoint();
    int16_t diff = setpoint - work.measured_temp;
    if (diff < 50)
    {
        /* 不在诊断区: 刷新基准温度与计时起点, 使温差重新 ≥50 时重启计时窗口 */
        work.diag_temp_last = work.measured_temp;
        work.diag_tick = now;
        return;
    }

    /* 仅在闭环已把输出拉满(90% 上限)时才判定: 输出未满说明闭环尚未尽全力,
     * 此时"无温升"不能归因于加热器失效, 否则大负载/弱供电下会误锁 heater_fault。 */
    if (work.power < (HEATER_POWER_MAX * 9 / 10))
    {
        work.diag_temp_last = work.measured_temp;
        work.diag_tick = now;
        return;
    }

    if ((now - work.diag_tick) < DIAG_CHECK_MS)
        return;

    int16_t rise = work.measured_temp - work.diag_temp_last;
    if (rise < DIAG_TEMP_RISE_THRESHOLD)
        latch_fault(&work.heater_fault);

    work.diag_temp_last = work.measured_temp;
    work.diag_tick = now;   /* 检测完成复位计时: 缺失会导致诊断间隔退化为 200ms 误判故障 */
}

/* 上电硬件链路自检 (每 PID 周期推进一步, 仅真实硬件模式):
 * 依次判定 热电偶短路/开路 → 发热芯断芯/短路(测量脉冲+注入组电流) → 输入电压范围.
 * 任一失败置 selftest_fault 锁存断热并记录故障码(UI 显示), 故障码保留至
 * WorkTask_Recover 重跑且全链路通过才清零; 全部通过后进入正常 PID 闭环.
 * 测量脉冲 10% 占空比持续 1 个 PID 周期, 冷态温升 <1°C. */

/* 首次引导判定: 当前配置型号无冷态电流参考(未标定) → 置引导标志.
 * 调用时机: 自检全链路通过(real, 天然冷态) / WorkTask_Init(sim, 直接完成),
 * 即"即将进入正常加热"前一刻. 用户完成标定或明确跳过后经 BootCalDone 清除.
 * 仅真机模式使用(模拟模式在 WorkTask_Init 里直接跳过引导). */
#if !HW_SIM_MODE
static void boot_cal_eval(void)
{
    work.boot_cal_pending = (TypeCal_GetRef(Config_GetHandleIdx()) == 0);
    /* 链路重新验证通过(Recover 重跑自检): 恢复加热使能 —— 若不匹配引导
     * (tip_guide) 残留而头已换回匹配型号, 会持续断热导致焊台不加热 */
    tip_guide = false;
}
#endif

#if !HW_SIM_MODE
static void selftest_advance(void)
{
    switch (work.selftest)
    {
    case SELFTEST_THERMO:
        type_guide_boot = false;   /* 新一轮自检入口: 清除上一轮的型号引导标记 */
        /* 热电偶: 冷态读数应处于 [THERMO_ADC_MIN, THERMO_ADC_OPEN) 正常区间,
         * 短路(<50)或开路(≥4000)均为链路失效 → 锁存故障码 1 */
        {
            uint16_t adc = AD_GetT12();
            if (adc < THERMO_ADC_MIN || adc >= THERMO_ADC_OPEN)
            {
                work.selftest_code  = SELFTEST_CODE_THERMO;
                work.selftest_fault = true;
                work.selftest       = SELFTEST_DONE;
                return;
            }
        }
        work.selftest = SELFTEST_HEAT_ON;
        break;

    case SELFTEST_HEAT_ON:
        /* 施加热芯测量脉冲 (固定小占空比), 下一周期读注入组导通电流 */
        Heater_Enable();
        Heater_SetPower(SELFTEST_PULSE_POWER);
        work.selftest = SELFTEST_HEAT_READ;
        break;

    case SELFTEST_HEAT_READ:
        {
            uint16_t cur = AD_GetCurrent();
            Heater_Disable();
            work.power = 0;

            /* 电流≈0 → 断芯或电流采样失效; 近饱和 → 发热芯短路 → 锁存故障码 2 */
            if (cur < SELFTEST_CUR_MIN || cur >= SELFTEST_CUR_MAX)
            {
                work.selftest_code  = SELFTEST_CODE_HEATER;
                work.selftest_fault = true;
                work.selftest       = SELFTEST_DONE;
                return;
            }

            /* 型号匹配: 冷态脉冲电流三档判型须与当前配置型号一致 (T12 未标定
             * 退化粗保护); 不匹配时按危险度分流:
             *   危险(T12 量级头进非 T12 配置) → 锁 Err4 拒绝;
             *   无害(T12 配置插 450°C 头、T245/T210 互插) → 引导 Tip Cal 校正.
             * 仅冷态权威判定 (与 CheckChain 冷态分支一致): 热态(掉电重启或
             * Recover 重检时头未冷)下 450°C 头电流随温升衰减, 会被 is_t12_scale
             * 误判 T12 量级 → 误锁 Err4 并陷入"长按恢复-再误锁"循环直到头冷;
             * 故热态跳过判型, 由冷态重启/切换链检兜底. 注意自检期间
             * run_pid_cycle 未运行、measured_temp 为初值, 冷态判定须实测换算. */
            if (adc_to_temp(AD_GetT12()) <= TYPE_CUR_COOL_MAX &&
                !type_cur_match(cur, Config_GetHandleIdx()))
            {
                uint8_t idx = Config_GetHandleIdx();
                /* 危险场景: 非 T12 配置下实测为 T12 量级 → 锁 Err4, 必须换头或切换
                 * 型号, 不引导标定 —— 该配置的温度曲线与 PID 增益都按 T245/T210
                 * 标定, 插 T12 头时读数与实际不符且增益失配, 实际温度可能偏离到
                 * 危险区间; 此风险与温度上限是否 450 无关(三型号已统一), 故仍按
                 * "必须换头"处理. is_t12_scale 冷态/热态均可靠(电流量级远低于门限). */
                if (idx != HANDLE_T12 && is_t12_scale(cur, idx))
                {
                    work.selftest_code  = SELFTEST_CODE_TYPE;
                    work.selftest_fault = true;
                    work.selftest       = SELFTEST_DONE;
                    return;
                }
                /* 无害场景: 判型与配置型号不一致但无烧头风险 (T12 配置插 450°C
                 * 头、T245/T210 互插) → 不锁故障, 置引导 + 持续断热(tip_guide),
                 * 由 UI 层自动进 Cur Cal 引导用户标定实际插入头后再恢复加热 */
                work.selftest_code  = SELFTEST_CODE_NONE;
                work.selftest_fault = false;
                work.boot_cal_pending = true;
                tip_guide             = true;
                work.tip_cur          = cur;
                type_guide_boot       = true;
                /* 不在此收尾: 继续走 SELFTEST_VIN 做供电范围判定 —— 原实现
                 * 直接 SELFTEST_DONE+return, 会漏掉本次开机的 9~30V 检查. */
            }
        }
        work.selftest = SELFTEST_VIN;
        break;

    case SELFTEST_VIN:
        read_input_voltage();
        if (work.vin_voltage_x10 < SELFTEST_VIN_MIN_x10 ||
            work.vin_voltage_x10 > SELFTEST_VIN_MAX_x10)
        {
            work.selftest_code  = SELFTEST_CODE_VIN;
            work.selftest_fault = true;
        }
        else
        {
            /* 全链路通过: 解除自检故障锁存并清零故障码 —— 此前(含重跑期间)
             * 保留故障码, 避免 UI 从 Err 闪回正常状态造成误导 */
            work.selftest_fault = false;
            work.selftest_code  = SELFTEST_CODE_NONE;

            /* 自检天然处于冷态: 当前型号未标定 → 置首次引导标志,
             * 由 UI 层自动进 Cur Cal 引导先冷态标定, 再进入正常加热.
             * 但若本轮 HEAT_READ 已判出"型号不匹配(无害)"并建立了引导, 则保留其
             * boot_cal_pending/tip_guide, 不再 boot_cal_eval —— 否则会按"当前配置
             * 型号"重判(可能判为已标定)并清掉引导, 使不匹配的烙铁头被加热. */
            if (!type_guide_boot)
                boot_cal_eval();

            /* 恢复加热使能: 上面 SELFTEST_HEAT_READ 已 Heater_Disable(), 若此处不恢复,
             * 则当前型号已标定(boot_cal_pending=false)的设备自检通过后仍不加热 ——
             * 之后没有任何 Heater_Enable 路径可达(ACTIVE 态 MarkActivity 不走,
             * 也不会进 Cur Cal 页调 SetCalSuppress(false))。
             * 先清缓存功率再使能: 否则 Heater_Enable 会把自检残留的
             * SELFTEST_PULSE_POWER 重放到 PWM。
             * 仅在 boot_cal_pending=false 时使能: 未标定场景 UI 即将进 Cur Cal,
             * 那里会 SetCalSuppress(true) 主动断热, 此处不必(也不该)先开加热。 */
            if (!work.boot_cal_pending)
            {
                Heater_SetPower(0);
                if (!WorkTask_HasFault())
                    Heater_Enable();
            }
        }
        work.selftest = SELFTEST_DONE;
        break;

    default:
        break;
    }
}
#endif /* !HW_SIM_MODE */

/* 执行 PID 控制周期 */
static void run_pid_cycle(void)
{
    int16_t meas_temp;
    int16_t power;

    /* 1~2. 读取实测温度 */
#if HW_SIM_MODE
    /* 模拟: 温度随时间向设定值逼近, 模拟真实加热过程, 使 PID / 温度到达 /
      加热器故障检测都能正常工作 */
    {
        int16_t setpoint = PID_Controller_GetSetpoint();
        int16_t sim      = work.measured_temp;

        if (cal_suppress)
            sim = (int16_t)(sim - SIM_HEAT_RATE);   /* 校准抑制: 断热, 模拟自然降温 */
        else if (sim < setpoint)
            sim = (int16_t)(sim + SIM_HEAT_RATE);
        else if (sim > setpoint)
            sim = (int16_t)(sim - SIM_HEAT_RATE);
        if (sim < 25) sim = 25;
        if (sim > OVER_TEMP_LIMIT) sim = OVER_TEMP_LIMIT;
        meas_temp = sim;
    }
#else
    uint16_t adc_val = AD_GetT12();

    /* 热电偶失效检测: ADC 低于物理下限(短路/断线) → 锁存断热,
     * 防止读数偏低被当低温、满功率加热烧烙铁. */
    if (adc_val < THERMO_ADC_MIN)
    {
        latch_fault(&work.thermo_fault);
        return;
    }

    /* 热电偶开路/未插检测: ADC 达运放饱和域(≥4000)连续 8 个周期 → 锁存断热,
     * 与短路检测对称. 冷态正常读数远低于此值, 不会误判;
     * 校准后 450°C 点 ADC 必须显著低于阈值(见 THERMO_ADC_OPEN 注释). */
    if (adc_val >= THERMO_ADC_OPEN)
    {
        if (++work.thermo_open_cnt >= THERMO_OPEN_COUNT)
        {
            latch_fault(&work.thermo_fault);
            return;
        }
    }
    else
    {
        work.thermo_open_cnt = 0;
    }

    /* 一阶低通: 替代原 ADC 内部 5 次平均, 抑制单次置乱噪声 (k ≈ 0.75) */
    work.temp_filt = work.temp_filt + ((((int32_t)adc_val - work.temp_filt) * TEMP_FILT_K) >> 8);
    meas_temp = adc_to_temp((uint16_t)work.temp_filt);

    /* 冷端补偿 (CPU 内部温度相对漂移): F1 内部温度传感器绝对精度受 V25 散布
     * 限制(±20°C 量级), 故用"相对变化量"而非绝对温度——上电时冷端≈环境≈CPU,
     * 之后环境/冷端漂移被 CPU 温度变化近似捕捉. CPU 温度先经一阶低通
     * (τ≈29s), 只补偿慢变漂移, 滤除采样噪声与自热快变; 系数待真机实验标定.
     * 单位: AD_GetCpuTempC 返回 °C×10(cpu_temp_ref 亦为同尺度), 而 meas_temp
     * 为整数 °C —— 故差值必须 /10 后才可叠加, 否则补偿被放大 10 倍:
     * 例如 MCU 自热 +5.0°C 会给实测温度凭空加上 +50°C, 进而污染 PID、
     * 显示、加热器诊断与 500°C 过温锁存. */
    {
        int16_t cpu_now = AD_GetCpuTempC();
        cpu_temp_filt += ((int32_t)(cpu_now - cpu_temp_filt) * CPU_TEMP_FILT_K) >> 10;
        meas_temp += (int16_t)((cpu_temp_filt - cpu_temp_ref) / 10);
    }
#endif
    work.measured_temp = meas_temp;

    /* 3.4 校准页(Cur Cal)加热抑制 / 插头不匹配引导断热: 只读温显示冷却进度,
     * 断热不控热. 置于温度采样/热电偶失效检测之后(短路/开路保护仍生效),
     * 跳过 PID 输出与加热器诊断 —— 冷却等待 10s 无温升会误锁存 heater_fault */
    if (cal_suppress || tip_guide)
    {
        Heater_Disable();
        work.power = 0;
        return;
    }

    /* 3. 过温保护 (锁存: 直到手动清除才恢复, 防读数抖动在阈值附近反复开断) */
    if (meas_temp > OVER_TEMP_LIMIT)
    {
        work.overtemp_fault = true;
        Heater_Disable();
        work.power = 0;
        Buzzer_Beep(BUZZER_EVENT_ALARM);
        return;
    }

#if !HW_SIM_MODE
    /* 3.5 过流保护: AD_GetCurrent() = 导通相位采样 - 关断相位采样(同一注入组,
     * CC4 逐帧在导通/关断中点交替触发, 两路都与 PWM 相位锁定)
     * 差分校准后消去运放失调/偏置, 接近真实工作电流, 超过阈值判过流 → 断热。
     * 只在真实硬件模式有效(HW_SIM 下无电流采样, 恒 0, 不触发)。 */
    if (AD_GetCurrent() > OVERCURRENT_LIMIT)
    {
        /* 过流(短路/功率链路异常)锁存为加热器故障. 仅断热会让 PWM 恒 0, 而
         * ACTIVE 态没有 Heater_Enable 路径(MarkActivity 仅在非 ACTIVE 时使能),
         * 将静默永久停加热且 UI 无提示. 锁存后走 WorkTask_Recover 恢复. */
        latch_fault(&work.heater_fault);
        return;
    }
#endif

    /* 4. 更新 PID 实测温度 */
    PID_Controller_SetMeasured(meas_temp);

    /* 4.5 自整定模式: 继电反馈. 过温/过流保护已在上方执行; 整定中跳过
     * PID/前馈(由继电输出接管)与加热诊断(振荡温升会误报诊断). */
    if (PID_TuneActive())
    {
        work.power = PID_TuneAdvance(meas_temp);
        Heater_SetPower(work.power);

        if (PID_TuneFinish())
        {
            if (PID_TuneLastSucceeded())
            {
                PID_Gain_t tg;
                uint8_t h = Config_GetHandleIdx();
                PID_TuneGetResult(&tg);
                /* 钳位到页面可显示/可编辑上限(与 page/pid.c 同源): 整定短暂可产出
                 * Kd 达 3×10^4(经 tune_gain_sat 饱和到 32767), 原样存入会让用户
                 * 一进 PID 页就被页面上限静默钳掉, 且 2 位整数显示截断高位. */
                if (tg.kp > PID_KP_MAX) tg.kp = PID_KP_MAX;
                if (tg.ki > PID_KI_MAX) tg.ki = PID_KI_MAX;
                if (tg.kd > PID_KD_MAX) tg.kd = PID_KD_MAX;
                int16_t old_kp = g_config.pid_cons_kp[h];
                int16_t old_ki = g_config.pid_cons_ki[h];
                int16_t old_kd = g_config.pid_cons_kd[h];
                bool saved;

                g_config.pid_cons_kp[h] = tg.kp;   /* 写入当前型号保守组 */
                g_config.pid_cons_ki[h] = tg.ki;
                g_config.pid_cons_kd[h] = tg.kd;
                saved = Config_Save();
                work.tune_save_succeeded = saved;

                if (!saved)
                {
                    /* 保存失败时运行参数、RAM 配置均回滚到整定前快照.
                     * Config_Save 先擦页再写; 若失败发生在擦除之后, 旧 Flash 内容可能已
                     * 不完整, 故恢复旧值后尽力再写一次. 若 Flash 持续故障, 持久化状态仍
                     * 无法保证, 但本次运行绝不启用未保存的新增益, 且 UI 明确报告失败. */
                    g_config.pid_cons_kp[h] = old_kp;
                    g_config.pid_cons_ki[h] = old_ki;
                    g_config.pid_cons_kd[h] = old_kd;
                    (void)Config_Save();
                    Buzzer_Beep(BUZZER_EVENT_ALARM);
                }
                else
                {
                    Buzzer_Beep(BUZZER_EVENT_TEMP_REACHED);
                }

                /* 仅成功落盘的新参数可用于本次运行; 失败则从已回滚的旧配置重载.
                 * Reload 内含 Reset, 也清除整定期间遗留的微分历史, 避免首帧尖峰. */
                PID_Controller_Reload();
            }
            else
            {
                Buzzer_Beep(BUZZER_EVENT_ALARM);
            }
            PID_TuneAbort();   /* 复位状态机(keep: half_count 供进度显示,
                                * last_success 供页面显示成败) */
        }
        return;
    }

    /* 5. 前馈与 PID 协同: 前馈查表连续增加冷启动/大焊点功率, 并把实际前馈下限
     * 传入 PID, 让积分器在前馈主导时冻结正向累积. 高误差下仍计算激进 PID,
     * 不再在 50°C 处从约 600 直接切到 900; 前馈曲线在 50°C 为 600、100°C 为 900.
     * 参数组也在 25~35°C 误差范围渐变, 避免旧 30°C 边界的增益跳变. */
    {
        static const int16_t ff_d[5] = { 0, 10, 25, 50, 100 };
        static const int16_t ff_v[5] = { 0, 180, 350, 600, 900 };
        int16_t setpoint = PID_Controller_GetSetpoint();
        int16_t diff     = (int16_t)(setpoint - meas_temp);   /* >0 = 需要加热 */
        int16_t ff       = 0;

        if (diff >= ff_d[4])
            ff = ff_v[4];
        else if (diff > 0)
        {
            uint8_t k;
            for (k = 0; k < 4; k++)
                if (diff < ff_d[k + 1])
                    break;
            ff = (int16_t)(ff_v[k] +
                 (int32_t)(ff_v[k + 1] - ff_v[k]) * (diff - ff_d[k]) /
                 (ff_d[k + 1] - ff_d[k]));
        }

        power = PID_Controller_Compute(ff);
        if (power < ff)
            power = ff;
    }
    work.power = power;

    /* 6. 设置加热器功率 */
    Heater_SetPower(power);

    /* 7. 检测温度是否到达目标 */
    {
        int16_t setpoint = PID_Controller_GetSetpoint();
        int16_t diff = meas_temp - setpoint;
        if (diff < 0)
            diff = (int16_t)(-diff);

        /* 去抖: 连续 TEMP_REACHED_CNT 个周期在阈值内才置位; 一旦越界立即清零
         * (置位慢、清除快 —— 提示要稳, 退出要及时) */
        if (diff <= TEMP_REACHED_THRESHOLD)
        {
            if (work.reached_cnt < TEMP_REACHED_CNT)
                work.reached_cnt++;
        }
        else
        {
            work.reached_cnt = 0;
        }
        work.temp_reached = (work.reached_cnt >= TEMP_REACHED_CNT);

        if (work.temp_reached && !work.temp_reached_prev)
        {
            Buzzer_Beep(BUZZER_EVENT_TEMP_REACHED);
        }
        work.temp_reached_prev = work.temp_reached;
    }

    /* 8. 加热器故障检测 (自整定期间已在上方提前返回, 不在此列) */
    check_heater_diagnostic(Timer_GetTick());
}


/* ========== 接口实现 ========== */

void WorkTask_Init(void)
{
    /* 初始化运动检测引脚 (PC13, 震动开关, 上拉输入)
     * 原 PA8 被 TIM1_CH1(PWM 加热)占用, 两者复用同一引脚会互相覆盖配置,
     * 使加热 PWM 或运动检测失效, 故迁移到空闲的 PC13. */
    GPIO_InitTypeDef gpio;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    gpio.GPIO_Mode = GPIO_Mode_IPU;    /* 上拉输入, 开关闭合=低电平 */
    gpio.GPIO_Pin = GPIO_Pin_13;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpio);

    work.last_activity = Timer_GetTick();
    work.last_pid_tick = Timer_GetTick();
    work.measured_temp = ROOM_TEMP;
    work.power = 0;
    work.tune_save_succeeded = false;
    work.temp_reached = false;
    work.temp_reached_prev = false;
    work.reached_cnt = 0;
    work.vin_adc = 0;
    work.vin_voltage_x10 = 0;
    work.temp_filt = (int32_t)AD_GetT12();   /* 首次滤波以当前读数初始化, 避免从 0 爬升 */
    work.thermo_open_cnt = 0;
    work.selftest       = HW_SIM_MODE ? SELFTEST_DONE : SELFTEST_THERMO;
    work.selftest_code  = SELFTEST_CODE_NONE;
    work.selftest_fault = false;
    work.boot_cal_pending = false;
    work.motion_detected = false;
    work.diag_temp_last = work.measured_temp;
    work.thermo_fault  = false;
    work.overtemp_fault = false;
    /* 以下三项原先依赖"文件级 static 结构零初始化"碰巧成立(state=0=ACTIVE,
     * heater_fault/tip_cur=0); 显式写出, 与相邻字段一致, 免枚举顺序/存储类
     * 变化时读到脏值. */
    work.state         = WORK_STATE_ACTIVE;
    work.heater_fault  = false;
    work.tip_cur       = 0;
    work.diag_tick = Timer_GetTick();
#if !HW_SIM_MODE
    cpu_temp_filt = AD_GetCpuTempC();   /* 滤波以首次读数初始化, 避免从 0 爬升 */
    cpu_temp_ref  = cpu_temp_filt;      /* 临时基准: 上电 2s 后由 Process 延后重取 */
    boot_tick     = Timer_GetTick();
    cpu_ref_ready = false;
#endif

    PID_Controller_SetSetpoint((int16_t)g_config.default_temp);
    PID_Controller_Reset();

    /* 运行时目标温度初值 = 默认温度 (RAM, 掉电丢失) */
    runtime_temp = (int16_t)g_config.default_temp;

    /* 首次引导: 模拟模式自检直接完成, 此时即"即将进入正常加热"前一刻;
     * 真实模式由 selftest_advance 全链路通过时置位(引导不能早于自检结束,
     * 否则会与自检的测量脉冲冲突).
     * 模拟联调: 出厂参考未标定会置位引导标志, 开机即被 ui_task 拉进 First Cal
     * 引导页且型号锁定, 旋转/转向全部无响应(卡格) → 模拟模式跳过引导,
     * 直接进入正常加热联调; 真实模式行为不变. */
#if HW_SIM_MODE
    work.boot_cal_pending = false;
#endif
}

void WorkTask_Process(void)
{
    uint32_t now = Timer_GetTick();

    /* 1. 运动检测 (每帧) */
    read_motion_sensor();

#if !HW_SIM_MODE
    /* 1.5 休眠中检测到运动 → 唤醒 (恢复运行时目标温度并重新起控).
     * 仅 SLEEP 响应; SHUTDOWN 为自动关机, 需手动操作唤醒, 防误唤醒。
     * 注: check_sleep_shutdown 只做"入睡/关机"的单向推进, 不承担唤醒职责。 */
    if (work.motion_detected && work.state == WORK_STATE_SLEEP)
        WorkTask_MarkActivity();
#endif

    /* 2. 休眠/关机计时检测 (每帧) */
    check_sleep_shutdown(now);

    /* 3. PID 控制周期 (每 200ms): 传感器采样与温度闭环在同一次节拍内完成 */
    if ((now - work.last_pid_tick) < PID_INTERVAL_MS)
        return;

    work.last_pid_tick = now;

    read_input_voltage();

#if !HW_SIM_MODE
    /* 冷端基准延后采样: 上电 2s、滤波收敛后重取 cpu_temp_ref, 避开开机瞬时值.
     * 此后基准固定, 运行时只补偿相对漂移. */
    if (!cpu_ref_ready && (now - boot_tick) >= CPU_REF_DELAY_MS)
    {
        cpu_temp_ref   = cpu_temp_filt;
        cpu_ref_ready  = true;
    }
#endif

    /* 非阻塞测量脉冲推进(换头链检 / Cur Cal 共用): 置于闭环准入门之前 ——
     * 换头链检常用于"目标型号尚未标定"场景, 不能依赖 Cal_IsHandleValid 放行.
     * 返回 true = 脉冲进行中并独占加热器, 本轮跳过闭环/诊断,
     * 防止 PID 输出覆盖测量占空比. 完成即断热并置 READY, 由页面轮询取走. */
    if (pulse_step())
        return;

    if (work.state == WORK_STATE_SHUTDOWN)
    {
        Heater_Disable();
        work.power = 0;
        return;
    }

#if !HW_SIM_MODE
    /* 上电硬件链路自检: 完成前不进入 PID 闭环, 失败由 selftest_fault 持续断热 */
    if (work.selftest != SELFTEST_DONE)
    {
        selftest_advance();
        return;
    }
#endif

    /* 锁存保护故障(过温/热电偶短路/开路/加热器故障/自检失败): 持续断热, 直到手动恢复 */
    if (work.thermo_fault || work.overtemp_fault || work.heater_fault || work.selftest_fault)
    {
        Heater_Disable();
        work.power = 0;
        return;
    }

    /* 门2: 当前型号校准数据无效(未标定/数据损坏)时禁加热, 防止错误曲线
     * 读温偏差大导致失控. 出厂占位数据严格递增, 不受影响. */
    if (!Cal_IsHandleValid(Config_GetHandleIdx()))
    {
        Heater_Disable();
        work.power = 0;
        PID_Controller_Reset();
        return;
    }

    run_pid_cycle();
}

void WorkTask_MarkActivity(void)
{
    work.last_activity = Timer_GetTick();

    if (work.state != WORK_STATE_ACTIVE)
    {
        /* 唤醒必须显式恢复 ACTIVE: 本函数是唯一的唤醒入口, 而 work.state
         * 只在 check_sleep_shutdown 里被置为 SLEEP/SHUTDOWN. 若此处不落地,
         * state 将永久停留在 SLEEP —— 后果是两重的:
         *   1) 入睡判定要求 state==ACTIVE, 故自动关机从此永不触发(休眠变成单次);
         *   2) HOME 状态行里 SLEEP 优先级高于 OK/Heat, 于是一边满功率加热
         *      一边恒显示 "Sleep".
         * 注: 唤醒自 SHUTDOWN 也走此分支(自动关机后按任意键即恢复工作). */
        work.state = WORK_STATE_ACTIVE;

        work.diag_temp_last = ROOM_TEMP;
        /* 唤醒恢复运行时目标温度(HOME 临时调解值), 而非默认值 */
        PID_Controller_SetSetpoint(runtime_temp);
        PID_Controller_Reset();
        /* 存在锁存故障时不得恢复加热. Heater_Enable 会把 heater.power(上次功率,
         * 断热路径有意不清它)原样写回 PWM; 而重新断热的三处(SHUTDOWN/保护门/门2)
         * 都在 200ms PID 节拍门之内, 一旦在此写回, 陈旧功率会一直输出到下一个 PID
         * tick —— 最长 200ms、最大约 65W(≈13J), 且故障期间等于在"温度不可知"
         * 状态下加热. 与 SetCalSuppress/handle_exit/reset_handler 的既有规则一致. */
        if (!WorkTask_HasFault())
            Heater_Enable();
    }
}

/* 设置休眠/关机计时抑制 (校准等长流程置位, 退出时清除).
 * 模拟模式休眠/关机整体禁用, 该标志无作用, 一并编译掉免生"未使用"告警. */
void WorkTask_SetSleepSuppress(bool en)
{
#if !HW_SIM_MODE
    sleep_suppress = en;
#else
    (void)en;
#endif
}

/* 校准页(Cur Cal)加热抑制: 置位断热不控热(热态烙铁头自然降温到冷态完成
 * 标定); 清除恢复加热使能, 由下一 PID 周期重新起控. 存在锁存故障时保持
 * 断热(故障门在下一 PID 周期同样强制断热, 这里避免残差功率输出窗口). */
void WorkTask_SetCalSuppress(bool en)
{
    if (en)
    {
        cal_suppress = true;
        Heater_Disable();
    }
    else
    {
        cal_suppress = false;
        /* 恢复加热前刷新加热器诊断基准: 抑制期间烙铁头已自然降温到冷态,
         * 沿用进入前的热态基准会在恢复后立即误判 heater_fault (rise 为负
         * < 阈值, 与 Recover 过温路径 L1357 同一逻辑) */
        work.diag_tick      = Timer_GetTick();
        work.diag_temp_last = work.measured_temp;
        if (!WorkTask_HasFault())
            Heater_Enable();
    }
}

/* 运行时目标温度 (RAM): HOME 临时调解时保存, 只写内存不落 flash */
void WorkTask_SetRuntimeTemp(int16_t temp)
{
    if (temp > (int16_t)Config_GetTempMax()) temp = (int16_t)Config_GetTempMax();
    if (temp < TEMP_MIN)                     temp = TEMP_MIN;
    runtime_temp = temp;
}

int16_t WorkTask_GetRuntimeTemp(void)
{
    return runtime_temp;
}

/* 手柄型号切换后的安全闭环: 调用方须已更新 g_config.handle_type 并 Config_Save().
 * 此处重新收敛运行状态到新型号的安全边界:
 *   0. 切换瞬间显式断热(门1), 不依赖 PID Reset 隐式归零兜底;
 *   1. 重载现役 PID 参数(每型号独立) 并复位(清积分/微分);
 *   2. 运行时目标温度若超过新型号上限则钳位(三型号统一 450), 避免切到
 *      低上限型号后仍按旧目标持续加热超标称;
 *   3. 目标温度重新下发并复位 PID, 用新型号控制律重新起控. */
void WorkTask_OnHandleChanged(void)
{
    /* 门1: 切换瞬间显式断热, 切断任何残余功率路径 */
    Heater_Disable();

    /* 切换落地 = 已按目标型号链检通过: 清除插头不匹配引导断热残留
     * (不匹配引导期间用户直接切型号的场景), 避免切换后持续断热 */
    tip_guide = false;

    /* 引导标志与新型号对齐: 目标型号已标定 → 清除首次引导(残留标志会在回
     * HOME 后被 UI 层误拉进 Tip Cal); 目标未标定 → 置位, 由 handle 页引导
     * 进 Cur Cal 完成标定. 保持"pending == 当前型号未标定"恒成立.
     * 模拟模式恒清: TypeCal 不落盘、电流为桩值 → 参考值恒 0, 置位会让 UI 每次
     * 回 HOME 都被拉进首次引导页; 而该页在 cal_lock 下有意忽略 UP/DOWN, 表现为
     * "旋转编码器无反应/像卡死". 口径与 WorkTask_Init 的模拟模式跳过引导一致。 */
#if HW_SIM_MODE
    work.boot_cal_pending = false;
#else
    work.boot_cal_pending = (TypeCal_GetRef(Config_GetHandleIdx()) == 0);
#endif

    /* 先重载 PID(内部 Reset), 再用新型号上限钳位目标并复位 */
    PID_Controller_Reload();

    if (runtime_temp > (int16_t)Config_GetTempMax())
        runtime_temp = (int16_t)Config_GetTempMax();
    if (runtime_temp < TEMP_MIN)
        runtime_temp = TEMP_MIN;

    PID_Controller_SetSetpoint(runtime_temp);
    PID_Controller_Reset();

    /* 新型号校准点非法(会退化线性映射, 读温偏差大)时蜂鸣警示, 提示需重新校准 */
    if (!Cal_IsHandleValid(Config_GetHandleIdx()))
        Buzzer_Beep(BUZZER_EVENT_ALARM);

    /* 型号不匹配故障(Err4)专用恢复: 根因即"配置型号与插入头不符", 切换落地
     * 已按目标型号链检通过(CheckChain) → 自动重跑自检, 通过即清锁存恢复加热,
     * 无需回 HOME 长按. 其他故障(Err1/2/3)不在此列, 仍按统一规则回 HOME 长按
     * 重检恢复. 重跑期间 selftest_fault 保留(通过才清), 避免闪回误导. */
    if (work.selftest_fault && work.selftest_code == SELFTEST_CODE_TYPE)
        work.selftest = SELFTEST_THERMO;
}

work_state_t WorkTask_GetState(void)
{
    return work.state;
}

bool WorkTask_IsTempReached(void)
{
    return work.temp_reached;
}

int16_t WorkTask_GetMeasuredTemp(void)
{
    return work.measured_temp;
}

int16_t WorkTask_GetPower(void)
{
    return work.power;
}

bool WorkTask_TuneSaveSucceeded(void)
{
    return work.tune_save_succeeded;
}

uint8_t WorkTask_GetPowerPercent(void)
{
    /* 以"实际可达功率"HEATER_POWER_LIMIT(90%)为满量程. 加热层会把请求功率钳到
     * 该值(保留 ≥10% 关断窗供采样), PID 输出上限(PID_OUTPUT_MAX)也已与其绑定;
     * 若按 HEATER_POWER_MAX(100%)换算, 功率条最后 10% 行程对应的实际加热量
     * 毫无变化(死区), 且 100% 处占空比只有 90%. 按可达功率换算后,
     * 100% = 设备能给出的最大加热, 读数与实际出力一致. */
    uint32_t p = (uint32_t)work.power;
    if (p > HEATER_POWER_LIMIT)
        p = HEATER_POWER_LIMIT;
    return (uint8_t)(p * 100 / HEATER_POWER_LIMIT);
}

bool WorkTask_IsHeaterFault(void)
{
    return work.heater_fault;
}

/* 任一锁存保护故障(过温/热电偶失效/加热器故障/自检失败)是否处于锁定态 */
bool WorkTask_HasFault(void)
{
    return work.heater_fault || work.thermo_fault || work.overtemp_fault || work.selftest_fault;
}

/* 清除运行期锁存故障(过温/热电偶失效/加热器故障), 允许重新加热.
 * 注: 上电自检故障(selftest_fault)不在此清除 —— 需经 WorkTask_Recover
 * 重跑自检链路复核, 而非直接解除锁存. */
void WorkTask_ClearFaults(void)
{
    work.heater_fault   = false;
    work.thermo_fault   = false;
    work.overtemp_fault = false;
    work.diag_tick = Timer_GetTick();
    work.diag_temp_last = work.measured_temp;
}

/* ========== 新增接口 ========== */

uint16_t WorkTask_GetVinX10(void)
{
    return work.vin_voltage_x10;
}

bool WorkTask_IsMotionDetected(void)
{
    return work.motion_detected;
}

/* 上电硬件链路自检故障码 (0=通过, 非 0=对应链路失败, 见 SELFTEST_CODE_*) */
uint8_t WorkTask_GetSelftestCode(void)
{
    return work.selftest_code;
}

/* 统一锁存故障码 (0=无故障, 供 HOME 状态行显示 ErrN):
 * 自检锁存时返回自检码(1~4); 否则按运行中锁存故障映射
 * 1=热电偶失效 2=加热器故障 3=过温. 优先级: 自检 > 热电偶 > 加热器 > 过温.
 * 修复: 原 HOME 状态行仅查自检码, 运行中过温/加热器/热电偶短路锁存会
 * 错误落为 "NoHnd" 误导用户(实际为传感器/加热故障). */
uint8_t WorkTask_GetFaultCode(void)
{
    if (work.selftest_fault)
        return work.selftest_code;
    if (work.thermo_fault)
        return SELFTEST_CODE_THERMO;   /* 1 */
    if (work.heater_fault)
        return SELFTEST_CODE_HEATER;   /* 2 */
    if (work.overtemp_fault)
        return 3;                      /* 过温(与自检码 3 同值, 分阶段语义) */
    return 0;
}

/* 首次冷态电流标定引导是否待处理 (当前型号未标定, 自检通过后置位):
 * 由 UI 层监视, 自动进入 Cur Cal 页引导用户先冷态标定再正常加热 */
bool WorkTask_IsBootCalPending(void)
{
    return work.boot_cal_pending;
}

/* 首次引导结束 (用户已标定或明确跳过), 清除引导标志;
 * 同时清除插头不匹配引导断热 (tip_guide), 恢复加热使能 */
void WorkTask_BootCalDone(void)
{
    work.boot_cal_pending = false;
    tip_guide             = false;
    /* 引导期间持续断热, 头已降温到冷态; 恢复加热前刷新诊断基准,
     * 防恢复后误锁 heater_fault (与 SetCalSuppress 清除路径同逻辑) */
    work.diag_tick      = Timer_GetTick();
    work.diag_temp_last = work.measured_temp;
}

/* 插头不匹配引导: 自检实测脉冲电流 (供 Cur Cal 页预选插入的型号; 未触发时 0) */
uint16_t WorkTask_GetTipCur(void)
{
    return work.tip_cur;
}

/* 按实测脉冲电流匹配插入的发热芯型号 (Cur Cal 插头不匹配引导默认选中用):
 * 与各已标定参考取差值最小者; 候选均未标定 → 默认 450°C 量级型号(T245).
 * 仅 T12 配置插非 T12 量级头场景触发, 故排除当前配置型号. */
uint8_t WorkTask_MatchTipType(uint16_t cur)
{
    uint8_t  best   = HANDLE_T245;   /* 默认: 450°C 量级, 用户可 UP/DOWN 调整 */
    uint32_t best_d = 0xFFFFFFFFu;

    for (uint8_t i = 0; i < HANDLE_NUM; i++)
    {
        if (i == Config_GetHandleIdx())
            continue;                 /* 当前型号已知不匹配, 排除 */
        uint16_t ref = TypeCal_GetRef(i);
        if (ref == 0)
            continue;                 /* 未标定无参考, 跳过 */
        uint32_t d = (cur > ref) ? (uint32_t)(cur - ref) : (uint32_t)(ref - cur);
        if (d < best_d)
        {
            best_d = d;
            best   = i;
        }
    }
    return best;
}

/* 重新检测硬件链路后恢复 (链路类故障专用):
 * 清除运行期锁存故障并重置自检状态机, 下一 PID 周期由 WorkTask_Process 重跑
 * 热电偶/发热芯/电流/电压判定, 全链路通过才清零 selftest_fault/code 并恢复加热;
 * 链路仍异常则保留(必要时更新)故障码, 继续锁存断热. 不盲信解除锁存. */
static void WorkTask_Retry(void)
{
    WorkTask_ClearFaults();
#if !HW_SIM_MODE
    work.selftest = SELFTEST_THERMO;   /* 重跑自检链路; selftest_fault/code 保留至通过 */
#endif
}

/* 按故障类型恢复 (烙铁插好后长按确认, HOME 页调用):
 * 仅过温(链路本身完好, 自检已通过) → 只清过温锁存, 不重跑自检链路,
 * 等待降温后直接恢复, 避免"恢复→又锁存"的无效重检循环;
 * 其他故障(热电偶/加热器/自检链路) → 重跑自检链路, 全链路通过才恢复. */
void WorkTask_Recover(void)
{
    if (work.overtemp_fault && !work.thermo_fault && !work.heater_fault && !work.selftest_fault)
    {
        work.overtemp_fault = false;   /* 过温独立路径: 链路完好, 无需重检 */
        /* 刷新加热器诊断基准: 过温锁存期间烙铁可能已大幅冷却, 沿用旧基准
         * 会在恢复后立即误判 heater_fault (rise 为负 < 阈值) */
        work.diag_tick       = Timer_GetTick();
        work.diag_temp_last  = work.measured_temp;
        /* 恢复加热使能: 过温锁存时已 Heater_Disable(), 只清标志不会重新使能 ——
         * PID 之后算出的功率写不进 PWM, 表现为静默不加热, 且无温升会让
         * check_heater_diagnostic 再次误锁 heater_fault。
         * 先清缓存功率再使能, 避免重放锁存前的旧占空比。 */
        Heater_SetPower(0);
        if (!WorkTask_HasFault())
            Heater_Enable();
        return;
    }
    WorkTask_Retry();
}

/* ========== 非阻塞发热芯测量脉冲 ==========
 * 换头链检与 Cur Cal 标定共用: 施加热芯脉冲(10% 占空比), 下一个 PID 周期读
 * 注入组导通电流. 取代原 pulse_cur_raw() 的 busy-wait 200ms —— 期间主循环完全
 * 停摆(输入事件积压、UI 冻结), 改为状态机后由 PID 周期推进, 主循环照常运行.
 *
 * 生命周期: Start → (WAIT) → Poll 返回 READY → 取走结果.
 * 脉冲期间加热器由本状态机独占: pulse_step() 返回 true 时 WorkTask_Process 提前
 * 返回, 跳过 PID/诊断, 防止闭环输出覆盖测量占空比. 任一保护介入
 * (锁存故障/自检/关机)即中止, 结果作废. */

typedef enum {
    PULSE_IDLE = 0,   /* 未开始 / 已收尾 */
    PULSE_WAIT,       /* 脉冲已施加, 等待下个 PID 周期读电流 */
    PULSE_READY,      /* 结果就绪, 等调用方取走 */
} pulse_state_t;

static struct {
    pulse_state_t state;
    uint16_t cur;         /* 实测导通电流 ADC */
    uint32_t t0;          /* 施加脉冲的时刻: 保证到读电流间隔 ≥ 一个 PID 周期 */
    uint8_t  chain_idx;   /* 链检目标型号(异步期间保存) */
    bool     chain_mode;  /* true = 链检(READY 时一并给出结论) */
    bool     cold;        /* 发起时刻的冷态快照(结果一个周期后才判定) */
    bool     result_ok;   /* 链检结论 */
} pulse;


#if !HW_SIM_MODE
/* 链检后半段判定: 电流区间 → 型号匹配 → 供电, 与自检同源.
 * cold 用发起时刻的快照: 结果在一个 PID 周期后才判定, 期间温度可能变化. */
static bool chain_evaluate(uint8_t idx, uint16_t cur, bool cold)
{
    if (cur < SELFTEST_CUR_MIN || cur >= SELFTEST_CUR_MAX)
        return false;   /* 发热芯断芯/短路或电流链路失效 */

    /* 型号匹配: 冷态权威三档判型; 热态阻值偏移无法精确判型. 热态仅拦"真 T12
     * 头切向非 T12 配置"危险场景 —— 但 is_t12_scale 对热 450°C 头同样误判
     * (热 T245/T210 电流衰减至 <1.5×ref 会被当 T12), 故仅当当前配置为 T12
     * (插头曾按 T12 冷态验证过)时才可信拦截: 此时若真插 T12 头, 其电流恒远
     * 低于门限(热态更低), 拦截可靠; 若插的是热 450°C 头(T245→T12→T245 回转),
     * 误拒但失败关闭, 冷后可重试. 450°C 组内互插(T245↔T210)放行, 无烧头风险 */
    if (cold)
    {
        if (!type_cur_match(cur, idx))
            return false;   /* 冷态: 插的烙铁头与目标型号不匹配 */
    }
    else if (idx != HANDLE_T12 &&
             Config_GetHandleIdx() == HANDLE_T12 &&
             is_t12_scale(cur, idx))
    {
        return false;   /* 热态: 仅拦 T12 配置下切向非 T12 型号 */
    }

    read_input_voltage();
    if (work.vin_voltage_x10 < SELFTEST_VIN_MIN_x10 ||
        work.vin_voltage_x10 > SELFTEST_VIN_MAX_x10)
        return false;   /* 供电异常 */

    return true;
}
#endif


/* 每 PID 周期推进一步. 返回 true = 脉冲期间独占加热器, 调用方跳过本轮闭环.
 * 非阻塞要点: 只做"读一次电流 + 断热", 不等待 —— 等待由 PID 周期本身承担. */
static bool pulse_step(void)
{
    if (pulse.state == PULSE_IDLE)
        return false;

    /* 保护/自检/关机介入: 立即中止并断热, 结果作废(轮询方会看到 IDLE).
     * 过温并入此门: 脉冲期间 run_pid_cycle 被跳过, 而过温锁存判定在其中 ——
     * 不在此快检, 新发生的过温要等脉冲结束才被发现. 这里只提前断热,
     * 锁存仍交给下一 PID 周期的正式过温路径(判定/蜂鸣/记码口径不变). */
    if (work.state == WORK_STATE_SHUTDOWN ||
        work.selftest != SELFTEST_DONE ||
        WorkTask_HasFault() ||
        work.measured_temp > OVER_TEMP_LIMIT)
    {
        Heater_SetPower(0);
        Heater_Disable();
        work.power  = 0;
        pulse.state = PULSE_IDLE;
        return false;
    }

    if (pulse.state == PULSE_WAIT)
    {
        /* 施加脉冲到读电流必须间隔至少一个 PID 周期: Start 由页面在任意时刻发起,
         * 若恰落在 PID 节拍之前, 下一个节拍就取样会"只加热了极短时间" —— 电流尚未
         * 建立, 读数不可信. 自检路径(HEAT_ON→HEAT_READ)天然按节拍对齐, 无此问题.
         * 故此处按时间判定: 未满一个周期就继续独占加热器, 下个节拍再读. */
        if ((Timer_GetTick() - pulse.t0) < PID_INTERVAL_MS)
            return true;

        pulse.cur = AD_GetCurrent();
        Heater_SetPower(0);   /* 先清测量占空比再断热, 防后续恢复使能时重放 */
        Heater_Disable();
        work.power = 0;

#if HW_SIM_MODE
        if (pulse.chain_mode)
            pulse.result_ok = true;
#else
        if (pulse.chain_mode)
            pulse.result_ok = chain_evaluate(pulse.chain_idx, pulse.cur, pulse.cold);
#endif
        pulse.state = PULSE_READY;
        return true;   /* 本轮仍独占: 不让闭环在同一周期写回功率 */
    }

    return false;      /* PULSE_READY: 加热器已断, 闭环可正常运行 */
}


/* 换头链路检测 —— 非阻塞版(取代原同步 WorkTask_CheckChain 的 busy-wait):
 * CheckChainStart 做瞬时检查(准入/热电偶)并发起测量脉冲; 结果就绪后由
 * CheckChainPoll 给出结论. 型号匹配按 目标型号 idx 判定 (调用方传待切换型号,
 * 而非当前配置型号): 换头场景下当前插入的正是目标头, 若仍按旧型号参考检查会
 * 误拦合法切换; 反之目标头与所选型号不符(如 T12 头被选成 T245) → 拒绝切换.
 * 返回 false = 未受理(已有测量在跑 / 锁存故障 / 自检未完成 / 热电偶异常),
 * 调用方应立即拒绝本次操作. 模拟模式: 恒受理, 首次 Poll 即 READY 且结论为 true. */
bool WorkTask_CheckChainStart(uint8_t idx)
{
    if (pulse.state == PULSE_WAIT)
        return false;                 /* 已有测量正在进行 */
    /* READY(上次结果未被取走)视为可覆盖: 否则页面异常离开后本接口会永久拒绝 */

    pulse.chain_mode = true;
    pulse.chain_idx  = idx;
    pulse.result_ok  = false;

#if HW_SIM_MODE
    (void)idx;
    pulse.cold  = true;
    pulse.cur   = 1200;               /* 模拟: 恒有效, 便于联调 */
    pulse.state = PULSE_READY;        /* 无阻塞: 直接给结论 */
    return true;
#else
    if (work.selftest != SELFTEST_DONE)
        return false;
    if (WorkTask_HasFault())
        return false;                 /* 故障门: 不输出测量脉冲 */

    {
        uint16_t adc = AD_GetT12();
        if (adc < THERMO_ADC_MIN || adc >= THERMO_ADC_OPEN)
            return false;             /* 热电偶短路/开路 */
    }

    pulse.cold = (work.measured_temp <= TYPE_CUR_COOL_MAX);

    pulse.t0 = Timer_GetTick();
    Heater_Enable();
    Heater_SetPower(SELFTEST_PULSE_POWER);
    pulse.state = PULSE_WAIT;         /* 满一个 PID 周期后读电流 */
    return true;
#endif
}

/* 发起一次冷态发热芯测量脉冲(Cur Cal 标定用, 非阻塞).
 * require_cold=true 时非冷态直接拒绝 —— 阻值已随温度偏移, 标定不可靠.
 * 返回 false = 未受理(已有测量在跑 / 锁存故障 / 自检未完成 / 非冷态). */
bool WorkTask_PulseStart(bool require_cold)
{
    if (pulse.state == PULSE_WAIT)
        return false;                 /* 已有测量正在进行 */
    /* READY(上次结果未被取走)视为可覆盖: 否则页面异常离开后本接口会永久拒绝 */

    pulse.chain_mode = false;
    pulse.result_ok  = false;

#if HW_SIM_MODE
    (void)require_cold;
    pulse.cur   = 1200;               /* 模拟: 恒有效, 便于 Cur Cal 页联调 */
    pulse.state = PULSE_READY;
    return true;
#else
    if (work.selftest != SELFTEST_DONE)
        return false;
    if (WorkTask_HasFault())
        return false;                 /* 故障门: 不输出测量脉冲 */
    if (require_cold && work.measured_temp > TYPE_CUR_COOL_MAX)
        return false;                 /* 非冷态: 阻值已偏移, 标定不可靠 */

    pulse.t0 = Timer_GetTick();
    Heater_Enable();
    Heater_SetPower(SELFTEST_PULSE_POWER);
    pulse.state = PULSE_WAIT;         /* 满一个 PID 周期后读电流 */
    return true;
#endif
}


/* 轮询测量结果(换头链检与 Cur Cal 共用):
 *   IDLE  — 未开始 / 已收尾
 *   BUSY  — 脉冲进行中
 *   READY — 本次已给出结果(*cur_out / *ok_out 有效), 状态随即回到 IDLE.
 * READY 时加热器已在 pulse_step() 内断开, 调用方无需再处理. */
static work_pulse_t pulse_poll(uint16_t *cur_out, bool *ok_out)
{
    if (pulse.state == PULSE_WAIT)
        return WORK_PULSE_BUSY;
    if (pulse.state != PULSE_READY)
        return WORK_PULSE_IDLE;

    if (cur_out) *cur_out = pulse.cur;
    if (ok_out)  *ok_out  = pulse.result_ok;
    pulse.state = PULSE_IDLE;   /* 取走即收尾 */
    return WORK_PULSE_READY;
}

work_pulse_t WorkTask_PulsePoll(uint16_t *cur_out)
{
    return pulse_poll(cur_out, NULL);
}

work_pulse_t WorkTask_CheckChainPoll(bool *ok_out)
{
    return pulse_poll(NULL, ok_out);
}


/* 中止进行中的测量(页面退出等): 清测量占空比并断热, 结果作废.
 * 无测量时为空操作, 不影响正常加热使能. */
void WorkTask_PulseAbort(void)
{
    if (pulse.state == PULSE_IDLE)
        return;

    pulse.state = PULSE_IDLE;
    Heater_SetPower(0);   /* 防调用方随后 Heater_Enable 时重放测量占空比 */
    Heater_Disable();
    work.power = 0;
}

/* 发热芯型号匹配 (三档判型) 公开包装: Cur Cal 页保存参考值前校验用.
 * 与自检/切换确认共用同一判定, 拒绝"拿 T12 头填 T245/T210 参考值"等
 * 误操作: T12 参考已标定时按三档判型比对; 未标定时退化粗保护出厂门限
 * (模拟模式下不判型恒通过, 见 type_cur_match 注释). */
bool WorkTask_TypeCurMatch(uint16_t cur, uint8_t idx)
{
    return type_cur_match(cur, idx);
}
