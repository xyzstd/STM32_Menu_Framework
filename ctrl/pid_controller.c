#include "pid_controller.h"
#include "config.h"

/*
 * 分段式整数 PID 控制器
 *
 * 参考 Cai-Zi/STM32_T12_Controller 的分段控制策略:
 *   - ΔT > 50°C: 全速加热 (输出 = PID_OUTPUT_MAX)
 *   - 30°C < ΔT ≤ 50°C: 激进 PID (aggressive)
 *   - ΔT ≤ 30°C: 保守 PID (conservative)
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
#define PID_D_FILTER  192

static struct {
    /* 激进 PID 参数 */
    int16_t agg_kp;
    int16_t agg_ki;
    int16_t agg_kd;

    /* 保守 PID 参数 */
    int16_t cons_kp;
    int16_t cons_ki;
    int16_t cons_kd;

    int16_t setpoint;       /* 目标温度 °C */
    int16_t measured;       /* 实测温度 °C */

    int16_t boost_offset;   /* 大焊点自动加温抬升量 °C (0~+, 叠加到有效设定值) */

    int32_t integral;       /* 积分累加值 *100 */
    int32_t i_rem;          /* 积分增量的整除余数(±<100), 见 Compute 的"余数累加" */
    int16_t last_measured;  /* 上次实测温度, 微分先行用 */
    int32_t d_filter;       /* 微分低通滤波值 */
    bool     d_first;       /* 首次计算标志, 跳过首帧微分 */

    int16_t output;         /* 上次输出值 */
} pid;

static int16_t effective_setpoint(void);

static int16_t tune_gain_sat(int64_t v);


void PID_Controller_Init(void)
{
    PID_Controller_Reload();
}

/* 按当前型号从配置加载 PID 参数 (激进/保守) 并复位状态.
 * 型号切换后调用, 使现役 PID 系数随型号切换而更新, 避免用错型号参数控温. */
void PID_Controller_Reload(void)
{
    uint8_t h = Config_GetHandleIdx();

    pid.agg_kp  = g_config.pid_agg_kp[h];
    pid.agg_ki  = g_config.pid_agg_ki[h];
    pid.agg_kd  = g_config.pid_agg_kd[h];
    pid.cons_kp = g_config.pid_cons_kp[h];
    pid.cons_ki = g_config.pid_cons_ki[h];
    pid.cons_kd = g_config.pid_cons_kd[h];
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
    /* 用户手动设定目标时清除自动加温抬升, 避免异常叠加 */
    pid.boost_offset = 0;
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

/* 有效设定值 = 用户设定 + Boost 抬升, 按 Boost 上限钳位.
 * 对外读数与 Compute() 判据共用一份, 保证两者口径一致(如不读出 T12 的 460°C). */
static int16_t effective_setpoint(void)
{
    int16_t eff = (int16_t)(pid.setpoint + pid.boost_offset);
    if (eff > (int16_t)Config_GetBoostMax())
        eff = (int16_t)Config_GetBoostMax();
    return eff;
}

int16_t PID_Controller_GetEffectiveSetpoint(void)
{
    return effective_setpoint();
}

int16_t PID_Controller_Compute(void)
{
    int16_t error;
    int32_t p_term, i_term, d_term;
    int32_t output;
    int16_t cur_kp, cur_ki, cur_kd;

    /* 计算误差: 使用有效设定值(用户设定 + 自动加温抬升), 抬升不越 Boost 上限.
     * 注意: 钳位用 Boost 上限而非 temp_max, 否则用户设到接近 temp_max 时
     * Boost 抬升会被压死、T12(400/+450)在大焊点下无法继续抬温. */
    error = (int16_t)(effective_setpoint() - pid.measured);

    /* 分段控制 */
    if (error > 50)
    {
        /* 温差 > 50°C: 全速加热 */
        pid.integral = 0;                       /* 全速时清零积分 */
        pid.last_measured = pid.measured;       /* 持续记录实测温度, 使切回 PID 时微分先行项保持连续, 避免输出瞬间跌落 */
        pid.output = PID_OUTPUT_MAX;            /* 同步"上次输出", 使 GetOutput() 在全速段也不失真 */
        return PID_OUTPUT_MAX;
    }

    /* 选择 PID 参数集 */
    if (error > 30)
    {
        /* 30°C < 温差 ≤ 50°C: 激进 PID */
        cur_kp = pid.agg_kp;
        cur_ki = pid.agg_ki;
        cur_kd = pid.agg_kd;
    }
    else
    {
        /* 温差 ≤ 30°C: 保守 PID */
        cur_kp = pid.cons_kp;
        cur_ki = pid.cons_ki;
        cur_kd = pid.cons_kd;
    }

    /* 比例项: P = Kp * error / 100 */
    p_term = (int32_t)cur_kp * error / PID_SCALE;

    /* 积分项: I += Ki * error / 100, 积分分离 + 抗饱和.
     * 只有限幅值随分段变化:
     *   - 激进/过冲区(|ΔT|>30°C): 收紧到 15% 满量程, 大误差下不致饱和;
     *   - 保守区(ΔT≤30°C): 放宽到 30% 满量程, 保证稳态精度.
     * 全速区(ΔT>50°C)上方已清零积分, 不累计. */
    {
        int16_t lim = (error > 30 || error < -30) ? PID_INTEGRAL_LIMIT_AGG : PID_INTEGRAL_LIMIT;
        int32_t raw, inc;

        /* 余数累加: 直接写 (ki*error)/100 会在 |error| < 100/ki 时整除截断为 0
         * (ki 存 ×100, 保守组 0.60→60, 故 |error|<2 时积分完全停走), 稳态消不掉最后
         * 1~2°C 偏差. 这里把未除尽的余数留到下一周期, 使小误差也能缓慢累积. */
        raw = (int32_t)cur_ki * error + pid.i_rem;
        inc = raw / PID_SCALE;
        pid.i_rem = raw - inc * PID_SCALE;   /* 余数(与 raw 同号, 范围 ±<100) */
        pid.integral += inc;
        if (pid.integral > lim)
            pid.integral = lim;
        else if (pid.integral < -lim)
            pid.integral = -lim;
    }
    i_term = pid.integral;

    /* 微分项: 微分先行 + 不完全微分
     * 微分先行: 对实测温度微分 (D = Kd*(last_meas - meas)/100),
     * 设定值突变(调温/休眠唤醒)不进入微分, 消除微分冲击;
     * 不完全微分: 一阶低通滤波, 抑制测量噪声放大 */
    if (pid.d_first)
    {
        /* 首帧仅记录温度, 不产生微分 */
        pid.d_first = false;
        pid.d_filter = 0;
    }
    else
    {
        int32_t d_new = (int32_t)(pid.last_measured - pid.measured) * cur_kd / PID_SCALE;
        pid.d_filter += (d_new - pid.d_filter) * PID_D_FILTER / 256;
    }
    pid.last_measured = pid.measured;   /* 持续记录, 供下一帧微分 */
    d_term = pid.d_filter;

    /* 合成输出 + 抗饱和(integral clamping):
     * 若未钳输出已顶出 [0, PID_OUTPUT_MAX], 把积分回退到"使输出恰落在限值"的
     * 值, 阻止饱和期间积分继续朝饱和方向累积(退出饱和后过冲的主要来源).
     * 上方限幅只约束积分自身幅度, 防不住"比例+积分"合力越界, 故在此补齐. */
    output = p_term + i_term + d_term;

    if (output > PID_OUTPUT_MAX)
    {
        int32_t i_cap = PID_OUTPUT_MAX - p_term - d_term;
        if (pid.integral > i_cap)
            pid.integral = i_cap;
        output = PID_OUTPUT_MAX;
    }
    else if (output < 0)
    {
        int32_t i_floor = -p_term - d_term;
        if (pid.integral < i_floor)
            pid.integral = i_floor;
        output = 0;
    }

    pid.output = (int16_t)output;
    return (int16_t)output;
}

void PID_Controller_SetAggKp(int16_t kp)
{
    pid.agg_kp = kp;
}

void PID_Controller_SetAggKi(int16_t ki)
{
    pid.agg_ki = ki;
}

void PID_Controller_SetAggKd(int16_t kd)
{
    pid.agg_kd = kd;
}

void PID_Controller_SetConsKp(int16_t kp)
{
    pid.cons_kp = kp;
}

void PID_Controller_SetConsKi(int16_t ki)
{
    pid.cons_ki = ki;
}

void PID_Controller_SetConsKd(int16_t kd)
{
    pid.cons_kd = kd;
}

void PID_Controller_SetBoostOffset(int16_t offset)
{
    if (offset < 0) offset = 0;
    pid.boost_offset = offset;
}

int16_t PID_Controller_GetBoostOffset(void)
{
    return pid.boost_offset;
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
 *   再按 Z-N 继电整定式给出保守组(稳态段)参数:
 *     Kp = 0.6·Ku,  Ki = Kp/(0.5·Tu),  Kd = Kp·Tu/8
 *   存储格式 *100 (与 PID_SCALE 一致).
 *
 * 定点细节:
 *   - 温度幅值 a 内部用 °C×10, Ku_x1000 = 4h×1000/(π·a_deg);
 *   - Tu 以"PID 周期数"计(1 周期=200ms), 换算进 Kp/Ki/Kd 公式.
 *   - 拐点检测: 温度差分变号 + 最小半周期迟滞(MIN_HALF)滤除噪声小振荡.
 */
#define TUNE_DUTY_ON        (PID_OUTPUT_MAX * 70 / 100)   /* 继电加热段输出 70% */
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
     * 上限用型号温度上限(而非 Boost 上限): 振荡会在设定点上再叠 ±30°C 左右,
     * 若按 Boost 上限(如 T12 的 450)钳位, 峰值可达约 480°C, 远超标称上限。 */
    if (setpoint > (int16_t)Config_GetTempMax())
        setpoint = (int16_t)Config_GetTempMax();
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
            /* 方向翻转: 记录刚结束那段的极值(用翻转帧近似), 且要求间隔够长 */
            if (tune.tracking && tune.timer >= TUNE_MIN_HALF)
            {
                if (tune.direction > 0)
                    tune.peak_sum += cur_x10;
                else
                    tune.valley_sum += cur_x10;

                tune.half_len_sum += tune.timer;
                tune.half_count++;
                tune.timer = 0;
            }
            tune.tracking = true;
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
            /* Ku_x1000 = 4h×1000/(π·a_deg); h=350(0~700 方波幅值), a_deg=amp_x10/10
             *          = 14000×1000/(π·amp_x10) */
            ku_x1000 = (int64_t)14000000 * 1000 / (3142 * amp_x10);
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
