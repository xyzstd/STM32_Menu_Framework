#include "pid_controller.h"
#include "config.h"

/*
 * 整数 PID 控制器 (单组参数)
 *
 * 2026-10-19 由双组(激进/保守)合并为单组: 静态分析与仿真显示, 保守组在
 * 误差 25°C 以下被前馈完全遮蔽(积分跟踪把 PID 输出钉在前馈值), 双组的唯一
 * 实际收益是过冲段阻尼; 而保守组 Kp=0.50 刹车过弱, 过冲反而比激进组更大.
 * 合并后默认取原激进组(11.00/0.50/1.00), 到温更快、过冲更小、菜单少 3 个参数.
 * 积分限幅仍按 |误差| 调度(10% → 5%), 与增益无关.
 *
 * 所有计算使用整数运算, 无浮点:
 *   - Kp/Ki/Kd 存储为 *100, 计算时除以 PID_SCALE(100)
 *   - 积分项: 积分分离 + 抗饱和限幅
 *   - 微分项: 微分先行(对实测温度微分, 消除设定值突变冲击)
 *             + 不完全微分(一阶低通, 抑制测量噪声放大)
 *   - 输出钳位到 0 ~ PID_OUTPUT_MAX
 */

/* 不完全微分低通系数 k: 192/256 ≈ 0.75 = 新值权重,
 * 即 d_filter = (1-k)*旧 + k*新。k 越小 D 越平滑(响应越慢),
 * k 越大越接近原始微分(测量噪声放大越明显)。 */
#define PID_D_FILTER       192

/* 积分限幅的调度区间: |误差| ≤ 25°C 用 10%, ≥ 35°C 收紧到 5%, 中间线性过渡.
 * 只影响积分, 与增益组无关(单组化后不再有增益混合带). */
#define PID_LIM_LOW        25
#define PID_LIM_HIGH       35

static struct {
    int16_t kp;
    int16_t ki;
    int16_t kd;

    int16_t setpoint;       /* 目标温度 °C */
    int16_t measured;       /* 实测温度 °C */

    int32_t integral;       /* 积分累加值 *100 */
    int32_t i_rem;          /* 积分增量的整除余数(±<100), 见 Compute 的"余数累加" */
    int16_t last_measured;  /* 上次实测温度, 微分先行用 */
    int32_t d_filter;       /* 微分低通滤波值 */
    bool     d_first;       /* 首次计算标志, 跳过首帧微分 */

    int16_t output;         /* 上次输出值 */
} pid;

static int16_t tune_gain_sat(int64_t v);


void PID_Controller_Init(void)
{
    PID_Controller_Reload();
}

/* 增益合法性钳位: Flash 里的值可能越界(旧固件写入 / 数据损坏), 载入即收敛到
 * 控制器允许范围. 不钳的后果不是失控(输出仍被加热层钳到 HEATER_POWER_LIMIT),
 * 而是控制质量异常且没有任何提示 —— 用户只会觉得"温度抖得厉害"却查不出原因.
 * 上限常量留在本模块: bsp/config.c 只负责存, 不必知道 PID 的合法范围. */
static inline int16_t pid_gain_clamp(int16_t v, int16_t max)
{
    if (v < 0)
        return 0;
    if (v > max)
        return max;
    return v;
}

/* 按当前型号从配置加载 PID 参数并复位状态.
 * 型号切换后调用, 使现役 PID 系数随型号切换而更新, 避免用错型号参数控温. */
void PID_Controller_Reload(void)
{
    uint8_t h = Config_GetHandleIdx();

    pid.kp = pid_gain_clamp(g_config.pid_kp[h], PID_KP_MAX);
    pid.ki = pid_gain_clamp(g_config.pid_ki[h], PID_KI_MAX);
    pid.kd = pid_gain_clamp(g_config.pid_kd[h], PID_KD_MAX);
    PID_Controller_Reset();
}

void PID_Controller_Reset(void)
{
    pid.output = 0;
    pid.integral = 0;
    pid.i_rem = 0;
    pid.d_filter = 0;
    pid.last_measured = 0;
    pid.d_first = true;   /* 下次计算跳过首帧微分, 避免冷启动冲击 */

    /* 复位即中止继电自整定: 手柄失连/型号切换/校准期间振荡已无意义 */
    PID_TuneAbort();
}

void PID_Controller_SetSetpoint(int16_t setpoint)
{
    /* 统一钳位到合法温度范围, 纵深防御: 即便某调用方漏钳/配置异常,
     * 也不会设出越界目标导致持续过热. 上限按当前型号运行时取值. */
    if (setpoint > (int16_t)Config_GetTempMax())
        setpoint = (int16_t)Config_GetTempMax();
    else if (setpoint < TEMP_MIN)
        setpoint = TEMP_MIN;
    pid.setpoint = setpoint;
}

int16_t PID_Controller_GetSetpoint(void)
{
    return pid.setpoint;
}

void PID_Controller_SetMeasured(int16_t measured)
{
    pid.measured = measured;
}

int16_t PID_Controller_GetMeasured(void)
{
    return pid.measured;
}

int16_t PID_Controller_Compute(int16_t feedforward_floor)
{
    int16_t error;
    int16_t abs_error;
    int16_t cur_kp, cur_ki, cur_kd;
    int16_t lim;
    int32_t p_term, d_term;
    int32_t output, candidate_integral;
    int32_t old_integral, old_rem, raw, inc, next_rem;

    error = (int16_t)(pid.setpoint - pid.measured);
    if (feedforward_floor < 0)
        feedforward_floor = 0;
    else if (feedforward_floor > PID_OUTPUT_MAX)
        feedforward_floor = PID_OUTPUT_MAX;

    /* 单组增益: 不再按误差切换参数组(双组已于 2026-10-19 合并) */
    cur_kp = pid.kp;
    cur_ki = pid.ki;
    cur_kd = pid.kd;

    abs_error = (error < 0) ? (int16_t)(-error) : error;
    if (abs_error <= PID_LIM_LOW)
        lim = PID_INTEGRAL_LIMIT;
    else if (abs_error >= PID_LIM_HIGH)
        lim = PID_INTEGRAL_LIMIT_AGG;
    else
        lim = (int16_t)(PID_INTEGRAL_LIMIT -
              ((PID_INTEGRAL_LIMIT - PID_INTEGRAL_LIMIT_AGG) *
               (abs_error - PID_LIM_LOW) / (PID_LIM_HIGH - PID_LIM_LOW)));

    p_term = (int32_t)cur_kp * error / PID_SCALE;

    /* 微分先行 + 一阶低通: 对实测温度求导, 设定值变化不直接产生冲击. */
    if (pid.d_first)
    {
        pid.d_first = false;
        pid.d_filter = 0;
    }
    else
    {
        int32_t d_new = (int32_t)(pid.last_measured - pid.measured) * cur_kd / PID_SCALE;
        pid.d_filter += (d_new - pid.d_filter) * PID_D_FILTER / 256;
    }
    pid.last_measured = pid.measured;
    d_term = pid.d_filter;

    /* 先形成候选积分, 再检查实际输出是否仍被前馈托底.
     * 前馈主导且误差仍要求加热时, 把积分直接跟踪到 ff-P-D (受积分限幅约束),
     * 使 PID 内部输出贴近外部实际命令; 前馈退出时避免积分器接管冲击. */
    if (pid.integral > lim)
        pid.integral = lim;
    else if (pid.integral < -lim)
        pid.integral = -lim;
    old_integral = pid.integral;
    old_rem = pid.i_rem;
    raw = (int32_t)cur_ki * error + old_rem;
    inc = raw / PID_SCALE;
    next_rem = raw - inc * PID_SCALE;
    candidate_integral = old_integral + inc;
    if (candidate_integral > lim)
        candidate_integral = lim;
    else if (candidate_integral < -lim)
        candidate_integral = -lim;

    output = p_term + candidate_integral + d_term;

    /* 条件积分 anti-windup:
     * ① 前馈主导且仍有正误差: 积分跟踪外部前馈命令(受限幅约束), 减小接管误差;
     * ② PID 已饱和时, 只冻结继续推向饱和方向的积分, 允许反方向误差退饱和. */
    if (error > 0 && output < feedforward_floor)
    {
        pid.integral = (int32_t)feedforward_floor - p_term - d_term;
        if (pid.integral > lim)
            pid.integral = lim;
        else if (pid.integral < -lim)
            pid.integral = -lim;
        pid.i_rem = 0;
    }
    else if ((output > PID_OUTPUT_MAX && inc > 0) || (output < 0 && inc < 0))
    {
        pid.integral = old_integral;
        pid.i_rem = old_rem;
    }
    else
    {
        pid.integral = candidate_integral;
        pid.i_rem = next_rem;
    }

    output = p_term + pid.integral + d_term;
    if (output > PID_OUTPUT_MAX)
        output = PID_OUTPUT_MAX;
    else if (output < 0)
        output = 0;

    pid.output = (int16_t)output;
    return (int16_t)output;
}

void PID_Controller_SetKp(int16_t kp)
{
    pid.kp = kp;
}

void PID_Controller_SetKi(int16_t ki)
{
    pid.ki = ki;
}

void PID_Controller_SetKd(int16_t kd)
{
    pid.kd = kd;
}

int16_t PID_Controller_GetOutput(void)
{
    return pid.output;
}

/* ============ 继电反馈自整定 ============
 *
 * 原理 (Aström–Hägglund 继电反馈 + Ziegler-Nichols):
 *   以"低于设定点满功率 / 高于设定点断电"的继电激励使温度围绕设定点等幅振荡,
 *   记录峰谷幅值 a 与临界周期 Tu. 由描述函数法求临界增益
 *     Ku = 4h/(π·a), h = 继电幅值(功率差的一半, 0~1000)
 *   再按 Z-N 继电整定式给出单组参数:
 *     Kp = 0.6·Ku,  Ki = Kp/(0.5·Tu),  Kd = Kp·Tu/8
 *   存储格式 *100 (与 PID_SCALE 一致).
 *
 * 定点细节:
 *   - 温度幅值 a 内部用 °C×10, Ku_x1000 = 4h×1000/(π·a_deg);
 *   - Tu 以"PID 周期数"计(1 周期=200ms), 换算进 Kp/Ki/Kd 公式.
 *   - 拐点检测: 温度差分变号 + 最小半周期迟滞(MIN_HALF)滤除噪声小振荡.
 */
#define TUNE_DUTY_ON        (PID_OUTPUT_MAX * 70 / 100)   /* 继电加热段输出 70% */
#define TUNE_T12_MAX        400      /* T12 整定温度上限: 比加热上限(450)低一档,
                                      * 给继电振荡峰值(设定点上方约 ±30°C)留安全余量 */
#define TUNE_HALF_REQUIRED  8        /* 8 个半周期 = 4 个完整周期 */
#define TUNE_CYCLES_REQUIRED (TUNE_HALF_REQUIRED / 2)     /* 对外汇报的完整周期数 */
#define TUNE_MIN_HALF       5        /* 半周期最短 5 周期(1s), 更短视为噪声 */
#define TUNE_AMP_MIN_X10    300      /* 收敛时振荡幅值 <3°C 判无效 */
#define TUNE_TIMEOUT        600      /* 单段上限 600 周期 = 120s: 预热不翻转/半周期过长时尽早放弃 */
#define TUNE_TOTAL_TIMEOUT  1500     /* 整定总时长兜底 1500 周期 = 300s: 正常 8 半周期约 40~160s,
                                      * 留足裕量; 仅防病理情况下无限拖长(整定期间休眠被抑制) */
#define TUNE_OVER_DELTA     80       /* 温度超出设定点 80°C 判异常中止 */

static struct {
    bool     active;
    int16_t  setpoint;
    uint8_t  half_count;
    int32_t  peak_sum;       /* °C×10 累加 */
    int32_t  valley_sum;     /* °C×10 累加 */
    uint32_t half_len_sum;   /* 半周期长度(周期数)累加 */
    uint16_t timer;          /* 当前半周期计时, 每接受一个半周期清零(用于量取半周期长度) */
    uint32_t total_timer;    /* 整定总计时(周期数), 全程不清零, 用于总时长超时 */
    int16_t  last_x10;       /* 上帧温度 °C×10 */
    int16_t  ext_x10;        /* 本半周期内跟踪到的极值 °C×10 (升段取最大/降段取最小) */
    int8_t   direction;      /* +1 升 / -1 降 / 0 未知 */
    bool     tracking;       /* 已捕获首个有效拐点 */
    bool     finished;
    bool     success;
    bool     last_success;   /* 结果留存: Abort 清 finished/success 后 UI 仍可显示成败 */
    PID_Gain_t gain;
} tune;

void PID_TuneStart(int16_t setpoint)
{
    /* 钳位到合法温度范围, 保证继电振荡安全。
     * 上限用型号温度上限: 振荡会在设定点上再叠 ±30°C 左右,
     * 若按更宽的上限钳位, 峰值会远超标称上限。
     * T12 再收紧一档到 TUNE_T12_MAX: T12 头按 450 整定时振荡峰值可达 ~480°C,
     * 已超出多数 T12 头的标称范围; 整定所得增益对全温区同样适用, 故用略低的
     * 激励温度换安全余量(加热上限仍是 450, 不受影响)。 */
    int16_t cap = (int16_t)Config_GetTempMax();

    if (Config_GetHandleIdx() == HANDLE_T12 && cap > TUNE_T12_MAX)
        cap = TUNE_T12_MAX;

    if (setpoint > cap)
        setpoint = cap;
    else if (setpoint < TEMP_MIN)
        setpoint = TEMP_MIN;

    tune.active     = true;
    tune.setpoint   = setpoint;
    tune.half_count = 0;
    tune.peak_sum   = 0;
    tune.valley_sum = 0;
    tune.half_len_sum = 0;
    tune.timer      = 0;
    tune.total_timer = 0;
    tune.last_x10   = 0;
    tune.ext_x10    = 0;
    tune.direction  = 0;
    tune.tracking   = false;
    tune.finished   = false;
    tune.success    = false;
    tune.last_success = false;   /* 新一轮开始: 刷新上次结果 */
    tune.gain.kp = tune.gain.ki = tune.gain.kd = 0;
}

void PID_TuneAbort(void)
{
    tune.active   = false;
    tune.finished = false;
    tune.success  = false;
    /* last_success 有意不清: 工作层在整定结束后会立即 Abort(), 页面随后还要
     * 显示"完成保存/失败", 故成败结论必须跨过 Abort 留存到下一轮开始. */
}

bool PID_TuneActive(void)  { return tune.active; }
bool PID_TuneFinish(void)  { return tune.finished; }
bool PID_TuneSucceeded(void) { return tune.finished && tune.success; }

bool PID_TuneLastSucceeded(void)
{
    return tune.last_success;
}

int16_t PID_TuneProgressCycles(void)
{
    return (int16_t)(tune.half_count / 2);   /* 半周期 → 完整周期 */
}

int16_t PID_TuneCyclesRequired(void)
{
    return (int16_t)TUNE_CYCLES_REQUIRED;    /* 页面 "N/x" 的分母, 单一来源 */
}

void PID_TuneGetResult(PID_Gain_t *g)
{
    if (g)
        *g = tune.gain;
}

/* 整定增益饱和: ×100 定点在 int16 里表示不下时钳到上限。
 * 不钳的话末尾的 (int16_t) 强转会回绕 —— 幅度可达 3×10^4 的 Kd 会变成负值,
 * 使 D 项符号翻转(温度上升反而加功率) → 正反馈失稳, 比"整定无效"更糟。
 * 失败方向定为"削到上限"而非"回绕", 保证 0 ≤ 增益 ≤ 32767。 */
static int16_t tune_gain_sat(int64_t v)
{
    if (v > 32767) return 32767;
    if (v < 0)     return 0;
    return (int16_t)v;
}

int16_t PID_TuneAdvance(int16_t measured)
{
    int16_t out;
    int16_t cur_x10 = (int16_t)(measured * 10);
    int8_t  dir     = 0;

    if (!tune.active)
        return 0;

    /* 1. 继电输出: 低于设定点加热, 高于断电 */
    out = (measured < tune.setpoint) ? TUNE_DUTY_ON : 0;

    /* 2. 拐点检测: 差分变号 + 最小半周期迟滞 */
    if (tune.last_x10 != 0)
    {
        int16_t diff = (int16_t)(cur_x10 - tune.last_x10);
        if (diff > 0)       dir = 1;
        else if (diff < 0)  dir = -1;

        if (dir != tune.direction && dir != 0 && tune.direction != 0)
        {
            /* 方向翻转: 结算刚结束那一段的**跟踪极值**(而不是翻转帧温度).
             * 真实峰谷落在两次采样之间, 用翻转帧温度会低估振荡幅值 a; 而
             * Ku = 4h/(πa) 与 a 成反比 —— 低估 a 会让 Ku 偏大, 整定出的增益偏激进. */
            if (tune.tracking && tune.timer >= TUNE_MIN_HALF)
            {
                if (tune.direction > 0)
                    tune.peak_sum   += tune.ext_x10;
                else
                    tune.valley_sum += tune.ext_x10;

                tune.half_len_sum += tune.timer;
                tune.half_count++;
                tune.timer = 0;
            }
            tune.tracking = true;
            tune.ext_x10  = cur_x10;   /* 新半周期: 极值从翻转点重新跟踪 */
        }
        else if (tune.tracking && dir != 0)
        {
            /* 同向推进: 继续跟踪本半周期极值(升段取最大 / 降段取最小) */
            if (dir > 0) { if (cur_x10 > tune.ext_x10) tune.ext_x10 = cur_x10; }
            else         { if (cur_x10 < tune.ext_x10) tune.ext_x10 = cur_x10; }
        }
    }

    tune.last_x10  = cur_x10;
    /* 温度恰好持平的周期(diff==0 → dir==0)保持原方向: 若写回 0, 拐点判据的
     * "tune.direction != 0"会失效, 紧随其后的真实反向被漏检, 导致半周期并段、
     * 极值错位; 极端情况下半周期数永远攒不够 8, 整定必然超时失败。 */
    if (dir != 0)
        tune.direction = dir;
    tune.timer++;
    tune.total_timer++;

    /* 3. 收敛判定: 8 个半周期 */
    if (tune.half_count >= TUNE_HALF_REQUIRED)
    {
        int32_t avg_peak   = tune.peak_sum   / (TUNE_HALF_REQUIRED / 2);
        int32_t avg_valley = tune.valley_sum / (TUNE_HALF_REQUIRED / 2);
        int32_t amp_x10    = (avg_peak - avg_valley) / 2;
        int32_t tu_cycles  = (int32_t)(tune.half_len_sum / TUNE_HALF_REQUIRED) * 2; /* 完整周期 */
        int64_t ku_x1000;

        if (amp_x10 >= TUNE_AMP_MIN_X10 && tu_cycles > 0 && avg_peak > tune.setpoint * 10 && avg_valley < tune.setpoint * 10)
        {
            /* Ku_x1000 = 4h×1000/(π·a_deg), a_deg = amp_x10/10
             *          = 4h×10000/(π·amp_x10)
             * 其中 h = 继电方波幅值(相对均值) = TUNE_DUTY_ON/2. 由 TUNE_DUTY_ON
             * 推导而非写死常数 —— 否则改 PID_OUTPUT_MAX 时此处不会同步, Ku 会整体偏移. */
            ku_x1000 = (int64_t)(4 * (TUNE_DUTY_ON / 2) * 10000) * 1000 / (3142 * amp_x10);
            tune.gain.kp = tune_gain_sat(60 * ku_x1000 / 1000);
            /* 控制器每 200ms 周期积分/微分一次, Z-N 连续公式需换算成"每周期"增益:
             *   Ki_cycle = Kp/(0.5·Tu_sec)×0.2 = 2·Kp_stored/tu_cycles
             *   Kd_cycle = Kp·Tu_sec/8/0.2    = Kp_stored·tu_cycles/8
             * 三项一律经 tune_gain_sat 饱和后存储: 尤其 Kd, 其量级可达 3×10^4
             * (Kd/Ts 把微分系数放大了 5 倍), 直接强转会回绕成负值. */
            tune.gain.ki = tune_gain_sat((int32_t)tune.gain.kp * 2 / tu_cycles);
            tune.gain.kd = tune_gain_sat((int32_t)tune.gain.kp * (int32_t)tu_cycles / 8);

            tune.success = true;
        }
        tune.finished = true;
        tune.last_success = tune.success;   /* 留存给 UI(Abort 后 survivor) */
        tune.active   = false;
        return 0;   /* 结束: 断电, 由上层写配置 */
    }

    /* 4. 异常中止: 单段超时 / 总时长超时 / 温度远超设定点。
     * 单段看 timer(每接受半周期清零, 量的是当前半周期或预热段);
     * 总时长看 total_timer(全程不清零) —— 只用 timer 时, 各段各自 <120s 而总耗时
     * 可以远超上限, 期间休眠被抑制, 用户会被长时间困在整定页。 */
    if (tune.timer > TUNE_TIMEOUT || tune.total_timer > TUNE_TOTAL_TIMEOUT ||
        measured > tune.setpoint + TUNE_OVER_DELTA)
    {
        tune.finished = true;
        tune.success  = false;
        tune.last_success = false;   /* 异常中止: UI 显示"失败" */
        tune.active   = false;
        return 0;
    }

    return out;
}
