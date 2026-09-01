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

/* 不完全微分低通系数 α: 192/256 ≈ 0.75 (α 越大 D 越平滑, 响应越慢) */
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
    int16_t last_measured;  /* 上次实测温度, 微分先行用 */
    int32_t d_filter;       /* 微分低通滤波值 */
    bool     d_first;       /* 首次计算标志, 跳过首帧微分 */

    int16_t output;         /* 上次输出值 */
} s_pid;

void PID_Controller_Init(void)
{
    PID_Controller_Reload();
}

/* 按当前型号从配置加载 PID 参数 (激进/保守) 并复位状态.
 * 型号切换后调用, 使现役 PID 系数随型号切换而更新, 避免用错型号参数控温. */
void PID_Controller_Reload(void)
{
    uint8_t h = Config_GetHandleIdx();

    s_pid.agg_kp  = g_config.pid_agg_kp[h];
    s_pid.agg_ki  = g_config.pid_agg_ki[h];
    s_pid.agg_kd  = g_config.pid_agg_kd[h];
    s_pid.cons_kp = g_config.pid_cons_kp[h];
    s_pid.cons_ki = g_config.pid_cons_ki[h];
    s_pid.cons_kd = g_config.pid_cons_kd[h];
    PID_Controller_Reset();
}

void PID_Controller_Reset(void)
{
    s_pid.boost_offset = 0;
    s_pid.integral = 0;
    s_pid.d_filter = 0;
    s_pid.last_measured = 0;
    s_pid.d_first = true;   /* 下次计算跳过首帧微分, 避免冷启动冲击 */
    s_pid.output = 0;
}

void PID_Controller_SetSetpoint(int16_t setpoint)
{
    /* 用户手动设定目标时清除自动加温抬升, 避免异常叠加 */
    s_pid.boost_offset = 0;
    /* 统一钳位到合法温度范围, 纵深防御: 即便某调用方漏钳/配置异常,
     * 也不会设出越界目标导致持续过热. 上限按当前型号运行时取值. */
    if (setpoint > (int16_t)Config_GetTempMax())
        setpoint = (int16_t)Config_GetTempMax();
    else if (setpoint < TEMP_MIN)
        setpoint = TEMP_MIN;
    s_pid.setpoint = setpoint;
}

int16_t PID_Controller_GetSetpoint(void)
{
    return s_pid.setpoint;
}

void PID_Controller_SetMeasured(int16_t measured)
{
    s_pid.measured = measured;
}

int16_t PID_Controller_GetMeasured(void)
{
    return s_pid.measured;
}

int16_t PID_Controller_GetEffectiveSetpoint(void)
{
    return (int16_t)(s_pid.setpoint + s_pid.boost_offset);
}

int16_t PID_Controller_Compute(void)
{
    int16_t error;
    int32_t p_term, i_term, d_term;
    int32_t output;
    int16_t cur_kp, cur_ki, cur_kd;

    /* 计算误差: 使用有效设定值(用户设定 + 自动加温抬升), 抬升不越Boost上限.
     * 注意: 钳位用 Boost 上限而非 temp_max, 否则用户设到接近 temp_max 时
     * Boost 抬升会被压死、T12(400/+450)在大焊点下无法继续抬温. */
    {
        int16_t eff_set = (int16_t)(s_pid.setpoint + s_pid.boost_offset);
        if (eff_set > (int16_t)Config_GetBoostMax()) eff_set = (int16_t)Config_GetBoostMax();
        error = (int16_t)(eff_set - s_pid.measured);
    }

    /* 分段控制 */
    if (error > 50)
    {
        /* 温差 > 50°C: 全速加热 */
        s_pid.output = PID_OUTPUT_MAX;
        s_pid.integral = 0;         /* 全速时清零积分 */
        /* 持续记录实测温度, 使切回 PID 时微分先行项保持连续,
         * 避免输出瞬间跌落 */
        s_pid.last_measured = s_pid.measured;
        s_pid.d_first = false;
        return s_pid.output;
    }

    /* 选择 PID 参数集 */
    if (error > 30)
    {
        /* 30°C < 温差 ≤ 50°C: 激进 PID */
        cur_kp = s_pid.agg_kp;
        cur_ki = s_pid.agg_ki;
        cur_kd = s_pid.agg_kd;
    }
    else
    {
        /* 温差 ≤ 30°C: 保守 PID */
        cur_kp = s_pid.cons_kp;
        cur_ki = s_pid.cons_ki;
        cur_kd = s_pid.cons_kd;
    }

    /* 比例项: P = Kp * error / 100 */
    p_term = (int32_t)cur_kp * error / PID_SCALE;

    /* 积分项: I += Ki * error / 100
     * 积分分离: |error| > 30°C (激进/全速区) 时停止积分,
     * 避免积分在温差大时快速饱和, 抑制切换回保守 PID 时的超调;
     * 积分抗饱和: 积分值限制在 PID 输出满量程 30% 范围内 (内置固定限幅). */
    if (error > 30 || error < -30)
    {
        /* 误差过大时不累计积分 */
    }
    else
    {
        s_pid.integral += (int32_t)cur_ki * error / PID_SCALE;
        if (s_pid.integral > PID_INTEGRAL_LIMIT)
            s_pid.integral = PID_INTEGRAL_LIMIT;
        else if (s_pid.integral < -PID_INTEGRAL_LIMIT)
            s_pid.integral = -PID_INTEGRAL_LIMIT;
    }
    i_term = s_pid.integral;

    /* 微分项: 微分先行 + 不完全微分
     * 微分先行: 对实测温度微分 (D = Kd*(last_meas - meas)/100),
     * 设定值突变(调温/休眠唤醒)不进入微分, 消除微分冲击;
     * 不完全微分: 一阶低通滤波, 抑制测量噪声放大 */
    if (s_pid.d_first)
    {
        /* 首帧仅记录温度, 不产生微分 */
        s_pid.d_first = false;
        s_pid.d_filter = 0;
    }
    else
    {
        int32_t d_new = (int32_t)cur_kd * (s_pid.last_measured - s_pid.measured) / PID_SCALE;
        s_pid.d_filter += (d_new - s_pid.d_filter) * PID_D_FILTER / 256;
    }
    s_pid.last_measured = s_pid.measured;
    d_term = s_pid.d_filter;

    /* 合成输出 */
    output = p_term + i_term + d_term;

    /* 输出钳位 */
    if (output > PID_OUTPUT_MAX)
        output = PID_OUTPUT_MAX;
    else if (output < 0)
        output = 0;

    s_pid.output = (int16_t)output;
    return s_pid.output;
}

void PID_Controller_SetAggKp(int16_t kp)
{
    s_pid.agg_kp = kp;
}

void PID_Controller_SetAggKi(int16_t ki)
{
    s_pid.agg_ki = ki;
}

void PID_Controller_SetAggKd(int16_t kd)
{
    s_pid.agg_kd = kd;
}

void PID_Controller_SetConsKp(int16_t kp)
{
    s_pid.cons_kp = kp;
}

void PID_Controller_SetConsKi(int16_t ki)
{
    s_pid.cons_ki = ki;
}

void PID_Controller_SetConsKd(int16_t kd)
{
    s_pid.cons_kd = kd;
}

void PID_Controller_SetBoostOffset(int16_t offset)
{
    if (offset < 0) offset = 0;
    s_pid.boost_offset = offset;
}

int16_t PID_Controller_GetBoostOffset(void)
{
    return s_pid.boost_offset;
}

int16_t PID_Controller_GetOutput(void)
{
    return s_pid.output;
}