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
 *   - 手柄连接检测 (NTC, PA6)
 *   - 运动检测 (震动开关, PA8)
 *   - 休眠/自动关机计时
 *   - 过温保护
 *   - 温度到达检测 + 蜂鸣器提示
 *   - 加热器故障检测
 */

/* ========== 常量 ========== */

#define PID_INTERVAL_MS          200    /* PID 控制周期 (ms) */
#define TEMP_REACHED_THRESHOLD   5      /* 温度到达阈值 */
#define OVER_TEMP_LIMIT          500    /* 过温保护阈值 (°C) */

/* 热电偶失效(短路/断线)检测: 温度 ADC 低于此阈值判短路/断线 → 锁存断热.
 * 校准原点: 0°C≈0, 25°C≈198; 烙铁正常工作温度远高于室温, 留裕量防误判.
 * 开路(运放饱和近满量程)与 450°C+ 在 ADC 域重叠, 由加热器故障诊断兜底. */
#define THERMO_ADC_MIN           50

/* 输入电压: 分压比 11:1, 3.3V 参考 */
#define VIN_DIVIDER_RATIO        11
#define VIN_ADC_REF              3.3f   /* 不使用浮点, 用整数替代 */

/* 手柄连接检测: 直接对 NTC ADC 原始值区间判定 (跳过 adc→温度线性映射, 更抗边界误判)
 * 连接区间 800~3600: 10k NTC + 10k 上拉, beta=3950
 *   0°C ~ 3155,  25°C ~ 2047,  50°C ~ 1082 (均为分压 ADC 码)
 *   开路/未插 (上拉几乎满量程) ≈ 4095,  短路 ≈ 0
 * 保留 ±20% 裕量, 避开低温/高温/开路/短路边界 */
#define HANDLE_ADC_MIN            800
#define HANDLE_ADC_MAX            3600

/* 加热器故障检测 */
#define DIAG_CHECK_MS            10000
#define DIAG_TEMP_RISE_THRESHOLD 5

/* 大焊点自动加温 (Boost): 实测温度连续大幅低于设定值判定烙铁头热量不足
 * (压在大焊点/地平面/大面积铜箔上吸热), 自动抬升设定温度, 恢复后回落.
 *
 * 触发前提(防误判):
 *   - 该目标温度下曾真正"到达"(boost_enabled 锁存), 冷启动/调温/唤醒不算;
 *   - 只在工作态且加热器正常、供电充足时生效;
 *   - 持续 1.5s 才触发; 抬升量按"曲线"平滑爬升/回落, 不一次性跳满;
 *   - 单次最长 6s, 超时判定异常强制结束并进入冷却, 防反复抢功率. */
#define BOOST_SAG_THRESHOLD      30     /* 温差超过 30°C 判定为"压住大焊点" */
#define BOOST_SAG_MS             1500   /* 温差须持续 1.5s, 避免冷启动/温度到达误触发 */
#define BOOST_RISE_DEG           50     /* 抬升幅度 °C */
#define BOOST_RAMP_RATE          8      /* 每 PID 周期(200ms)抬升/回落 °C, 形成平滑升温曲线 */
#define BOOST_RECOVER_HYS        10     /* 回到设定值上方 10°C 内即认为已恢复 */
#define BOOST_MAX_MS             6000   /* 单次 Boost 最长 6s, 超时判定为异常强制结束 */
#define BOOST_COOLDOWN_MS        10000  /* 异常结束后至少冷却 10s 才允许再次触发 */
#define BOOST_VIN_MIN_x10        180    /* 供电低于 18.0V 不 Boost, 防止供电不足时误判加功率 */

static int16_t  last_setpoint;   /* 上一周期设定值, 检测调温/唤醒重置 */
static int16_t  boost_target;    /* 抬升后的有效目标温度 */
static bool     boost_enabled;   /* 该目标温度下曾真正到达过(锁存) */
static uint32_t boost_cooldown;  /* 冷却倒计时剩余 ms, 0=允许触发 */
static uint32_t boost_elapsed;   /* 本次 Boost 已持续 ms */

/* 运行时目标温度 (RAM, 不落 flash): HOME 页临时调解的温度保存在此,
 * 整个运行期间持续生效, 休眠唤醒时恢复到该值而非默认值. 掉电丢失. */
static int16_t  runtime_temp;

/* NTC 温度计算: 简化的 beta 方程 */
#define NTC_BETA                 3950
#define NTC_ROOM_TEMP            25      /* 室温 °C */

/* 温度 ADC 一阶低通滤波系数 (定点): K = 192/256 ≈ 0.75.
 * 高频噪声已由 ADC 8 帧过采样抑制, 此低通仅保留中频平滑 → K 提高到 0.75,
 * 90% 阶跃响应从约 3.3 个 PID 周期降到约 1.7 个, 相位延迟减半、更跟手. */
#define TEMP_FILT_K              192

/* ========== 硬件抽象开关 ==========
 *  HW_SIM_MODE = 1 : 模拟模式(无真实外设/手柄)。开发与 GUI 模拟时使用,
 *                    所有传感器输入由程序"假装"为硬件已就绪, 便于跑通整条
 *                    加热/PID/休眠逻辑, 屏幕会显示已连接手柄。
 *  HW_SIM_MODE = 0 : 真实硬件模式, 读取真实 ADC / GPIO。
 * 切换只需改这一个宏, 不触碰业务逻辑。 */
#define HW_SIM_MODE               1

/* 模拟加热响应: 每个 PID 周期(200ms)让仿真温度向设定值逼近的格数 */
#define SIM_HEAT_RATE             4

/* 过流保护阈值 (注入组电流 ADC 码):
 * 电流经采样电阻 R_s → 运放B → ADC。阈值按"最大允许电流对应的运放输出 ADC 码"设定,
 * 超过即判过流 → 断热。具体数值需按实际 R_s 与运放增益标定。 */
#define OVERCURRENT_LIMIT         3500

/* ========== 内部状态 ========== */

static struct {
    work_state_t state;
    uint32_t     last_activity;
    uint32_t     last_pid_tick;
    int16_t      measured_temp;
    int16_t      power;
    bool         temp_reached;
    bool         temp_reached_prev;

    /* 输入电压 */
    uint16_t     vin_adc;            /* 最新输入电压 ADC 值 */
    uint16_t     vin_voltage_x10;    /* 输入电压 *10 (如 240 = 24.0V) */

    /* 手柄连接 */
    uint16_t     ntc_adc;            /* 最新 NTC ADC 值 */
    int16_t      ntc_temp;           /* NTC 温度 °C */
    int32_t      temp_filt;          /* 温度 ADC 一阶低通滤波值 */
    bool         handle_connected;
    uint8_t      handle_err_count;

    /* 运动检测 */
    bool         motion_detected;    /* 本次扫描是否检测到运动 */

    /* 加热器故障 */
    uint32_t     diag_tick;
    int16_t      diag_temp_last;
    bool         heater_fault;

    /* 锁存保护故障 (置位后持续断热, 直到 WorkTask_ClearFaults 手动清除) */
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

/* 温度校准偏移: 100/200/300/350/400/450°C 六点偏移, 用单调三次样条平滑插值
 * 6 温度点细分常用焊接带 (300~350), 曲线平滑连续; 超范围时取端点偏移. */
static int16_t cal_offset_interp(int16_t temp_base)
{
    uint8_t h = Config_GetHandleIdx();
    int32_t xs[6] = {100, 200, 300, 350, 400, 450};
    int32_t ys[6] = {
        (int16_t)g_config.cal_off_100[h],
        (int16_t)g_config.cal_off_200[h],
        (int16_t)g_config.cal_off_300[h],
        (int16_t)g_config.cal_off_350[h],
        (int16_t)g_config.cal_off_400[h],
        (int16_t)g_config.cal_off_450[h],
    };
    return (int16_t)spline_interp(xs, ys, 6, temp_base);
}

/* 使用校准点进行温度转换 (线性插值 + 多点偏移), 校准点按当前型号索引 */
static int16_t adc_to_temp(uint16_t adc_val)
{
    uint8_t h = Config_GetHandleIdx();
    int32_t temp;
    int32_t adc = (int32_t)adc_val;

    /* 校准点非法(被清零/Flash损坏)时退化为线性映射, 防止除零 */
    if (g_config.cal_adc_100[h] == 0 ||
        g_config.cal_adc_200[h] <= g_config.cal_adc_100[h] ||
        g_config.cal_adc_300[h] <= g_config.cal_adc_200[h] ||
        g_config.cal_adc_350[h] <= g_config.cal_adc_300[h] ||
        g_config.cal_adc_400[h] <= g_config.cal_adc_350[h] ||
        g_config.cal_adc_450[h] <= g_config.cal_adc_400[h])
    {
        return (int16_t)(adc * 120 / 4095);
    }

    /* 使用多项校准点进行温度转换:
     * 校准点之间用单调三次样条平滑插值, 低于 100°C 用原点线性, 高于 450°C 用末段斜率外推 */
    if (adc <= g_config.cal_adc_100[h])
    {
        /* 低于 100°C */
        temp = 100 * adc / g_config.cal_adc_100[h];
    }
    else if (adc <= g_config.cal_adc_450[h])
    {
        /* 100°C~450°C: 校准点区间单调三次样条平滑插值, 消除分段线性拐角,
         * 6 温度点 (100/200/300/350/400/450) 细分常用焊接带 */
        int32_t xs[6] = {
            (int32_t)g_config.cal_adc_100[h],
            (int32_t)g_config.cal_adc_200[h],
            (int32_t)g_config.cal_adc_300[h],
            (int32_t)g_config.cal_adc_350[h],
            (int32_t)g_config.cal_adc_400[h],
            (int32_t)g_config.cal_adc_450[h],
        };
        int32_t ys[6] = {100, 200, 300, 350, 400, 450};
        temp = spline_interp(xs, ys, 6, (int32_t)adc);
    }
    else
    {
        /* 高于 450°C: 沿用 400~450 段斜率外推 */
        int32_t span = (int32_t)g_config.cal_adc_450[h] - g_config.cal_adc_400[h];
        if (span <= 0) span = 1;
        temp = 450 + (int32_t)(adc - g_config.cal_adc_450[h]) * 50 / span;
    }

    /* 叠加按温度区间插值的校准偏移 */
    temp += cal_offset_interp((int16_t)temp);
    if (temp < 0) temp = 0;
    if (temp > OVER_TEMP_LIMIT) temp = OVER_TEMP_LIMIT;

    return (int16_t)temp;
}
#endif /* !HW_SIM_MODE */

/* 计算输入电压 (分压比 11:1, 3.3V 参考) */
/* Vin = ADC * 3.3 / 4095 * 11, 输出 *10 (如 240 = 24.0V) */
static uint16_t calc_vin_x10(uint16_t adc)
{
    /* Vin *10 = adc * 33 * 11 / 4095 = adc * 363 / 4095 */
    /* 简化为: adc * 363 / 4095 */
    return (uint16_t)((uint32_t)adc * 363 / 4095);
}

#if !HW_SIM_MODE
/* 计算 NTC 温度 (简化查表) */
static int16_t calc_ntc_temp(uint16_t adc)
{
    /* NTC 分压: Vntc = 3.3 * Rntc / (Rntc + 10k)
     * 10k NTC, 10k 上拉, 25°C 时 Rntc=10k, Vntc=1.65V, ADC=2047
     *
     * 简化: 用 ADC 值的倒数估算温度
     * 温度范围 0~50°C, ADC 约 1500~2500
     * 粗略线性: temp = 50 - (adc - 1500) * 50 / 1000 */
    if (adc < 1200 || adc > 2800)
        return 0;   /* 开路或短路 */

    return (int16_t)(50 - ((int32_t)adc - 1500) * 50 / 1000);
}
#endif /* !HW_SIM_MODE */

/* 读取输入电压 */
static void read_input_voltage(void)
{
#if HW_SIM_MODE
    /* 模拟: 假定电源已接入, 恒为 24.0V */
    work.vin_voltage_x10 = 240;   /* 直接固定 24.0V, 避免二次取整误差 */
#else
    work.vin_voltage_x10 = calc_vin_x10(work.vin_adc);
#endif
}

/* 读取 NTC 并检测手柄连接 */
static void read_ntc_handle(void)
{
#if HW_SIM_MODE
    /* 模拟: 假定已接入手柄, NTC 恒为室温 → 判定"已连接" */
    work.handle_connected  = true;
    work.ntc_temp          = NTC_ROOM_TEMP;
    work.handle_err_count  = 0;
#else
    uint16_t adc = AD_GetNTC();
    work.ntc_temp = calc_ntc_temp(adc);

    /* 手柄连接检测: 直接对 NTC ADC 原始值做区间判定,
     * 短路(≈0)和开路/未插(≈4095)都在区间外 → 判为断开 */
    if (adc >= HANDLE_ADC_MIN && adc <= HANDLE_ADC_MAX)
    {
        work.handle_connected = true;
        work.handle_err_count = 0;
    }
    else
    {
        if (++work.handle_err_count >= 8)
            work.handle_connected = false;
    }
#endif
}

/* 检查运动检测 (PA8 SLEEP 引脚, 震动开关) */
static void read_motion_sensor(void)
{
#if HW_SIM_MODE
    /* 模拟: 默认静止(不触发唤醒/复位休眠计时) */
    work.motion_detected = false;
#else
    /* PA8 配置为输入, 低电平有效 (震动开关闭合时 = 0) */
    work.motion_detected = (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET);
#endif
}

/* 检查休眠/关机计时 */
static void check_sleep_shutdown(uint32_t now)
{
    if (work.state != WORK_STATE_ACTIVE)
        return;

    /* 检测到运动 → 重置计时 */
    if (work.motion_detected)
    {
        work.last_activity = now;
        return;
    }

    uint32_t elapsed_ms = now - work.last_activity;
    uint32_t elapsed_sec = elapsed_ms / 1000;

    if (g_config.sleep_time > 0 && elapsed_sec >= (uint32_t)g_config.sleep_time * 60)
    {
        work.state = WORK_STATE_SLEEP;
        PID_Controller_SetSetpoint((int16_t)g_config.sleep_temp);
        PID_Controller_Reset();
        return;
    }

    if (g_config.shutdown_time > 0 && elapsed_sec >= (uint32_t)g_config.shutdown_time * 60)
    {
        work.state = WORK_STATE_SHUTDOWN;
        Heater_Disable();
        return;
    }
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
        work.diag_temp_last = work.measured_temp;
        return;
    }

    if ((now - work.diag_tick) < DIAG_CHECK_MS)
        return;

    int16_t rise = work.measured_temp - work.diag_temp_last;
    if (rise < DIAG_TEMP_RISE_THRESHOLD)
    {
        work.heater_fault = true;
        Heater_Disable();
        work.power = 0;

        if (g_config.buzzer_enable)
            Buzzer_Beep(BUZZER_EVENT_ALARM);
    }

    work.diag_temp_last = work.measured_temp;
}

/* 清除 Boost 自动加温状态: 设定值变化/型号切换/手柄断开/异常结束后调用,
 * 防止残留抬升量跨状态叠加到有效设定值. */
static void boost_reset(void)
{
    boost_elapsed  = 0;       /* 本次持续计时清零 */
    boost_enabled  = false;   /* 需重新"到达"才允许触发 */
    boost_target   = 0;       /* 抬升目标归零, offset 由斜坡自然回落 */
}

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

        if (work.handle_connected)
        {
            if (sim < setpoint)
                sim = (int16_t)(sim + SIM_HEAT_RATE);
            else if (sim > setpoint)
                sim = (int16_t)(sim - SIM_HEAT_RATE);
        }
        if (sim < 25) sim = 25;
        if (sim > OVER_TEMP_LIMIT) sim = OVER_TEMP_LIMIT;
        meas_temp = sim;
    }
#else
    uint16_t adc_val = AD_GetT12();

    /* 热电偶失效检测: ADC 低于物理下限(短路/断线) → 锁存断热,
     * 防止读数偏低被当低温、满功率加热烧烙铁. 开路(近满量程)与
     * 450°C+ 在 ADC 域重叠, 由加热器故障诊断(功率>0 温升不足)兜底. */
    if (adc_val < THERMO_ADC_MIN)
    {
        work.thermo_fault = true;
        Heater_Disable();
        work.power = 0;
        if (g_config.buzzer_enable)
            Buzzer_Beep(BUZZER_EVENT_ALARM);
        return;
    }

    /* 一阶低通: 替代原 ADC 内部 5 次平均, 抑制单次置乱噪声 (k ≈ 0.75) */
    work.temp_filt = work.temp_filt + ((((int32_t)adc_val - work.temp_filt) * TEMP_FILT_K) >> 8);
    meas_temp = adc_to_temp((uint16_t)work.temp_filt);
#endif
    work.measured_temp = meas_temp;

    /* 3. 过温保护 (锁存: 直到手动清除才恢复, 防读数抖动在阈值附近反复开断) */
    if (meas_temp > OVER_TEMP_LIMIT)
    {
        work.overtemp_fault = true;
        Heater_Disable();
        work.power = 0;
        if (g_config.buzzer_enable)
            Buzzer_Beep(BUZZER_EVENT_ALARM);
        return;
    }

#if !HW_SIM_MODE
    /* 3.5 过流保护: AD_GetCurrent() = 导通电流(注入组 CC4) - 关断电流(规则组, 自由采样)
     * 差分校准后消去运放失调/偏置, 接近真实工作电流, 超过阈值判过流 → 断热。
     * 只在真实硬件模式有效(HW_SIM 下无电流采样, 恒 0, 不触发)。 */
    if (AD_GetCurrent() > OVERCURRENT_LIMIT)
    {
        Heater_Disable();
        work.power = 0;
        return;
    }
#endif

    /* 4. 更新 PID 实测温度 */
    PID_Controller_SetMeasured(meas_temp);

    /* 4.5 大焊点自动加温 (Boost):
     * 仅在"该目标温度下曾真正到达"且处于工作态、加热器正常、供电充足时,
     * 若实测连续大幅低于设定值(压在大面积焊点/铜箔上吸热), 自动抬升有效
     * 设定值扩大加热; 抬升按曲线平滑爬升, 恢复/超时后平滑回落. */
    {
        int16_t setpoint = PID_Controller_GetSetpoint();
        int16_t diff = (int16_t)(setpoint - meas_temp);
        int16_t cur_off = PID_Controller_GetBoostOffset();

        /* 0. 设定值变化(用户调温/休眠/唤醒) → 必须重新"到达"才允许 Boost,
         *    清除残留抬升(需要时由下方斜坡自然回落到 0) */
        if (setpoint != last_setpoint)
        {
            boost_elapsed  = 0;
            boost_enabled  = false;
            boost_target   = 0;
        }

        /* 冷却倒计时递减; 冷却结束后累计 Boost 时长(触发前=持续低温计时, 触发后=加温计时) */
        if (boost_cooldown > 0)
        {
            if (boost_cooldown >= PID_INTERVAL_MS)
                boost_cooldown -= PID_INTERVAL_MS;
            else
                boost_cooldown = 0;
        }
        else
        {
            boost_elapsed += PID_INTERVAL_MS;
        }

        if (boost_target > 0)
        {
            /* 加温中: 用有效设定值(含抬升)判断恢复 —— 温度追到抬升目标附近
             * 才算真正恢复, 避免大焊点尚未融化、温度只回到基础设定值就提前回落.
             * 恢复后目标归零, 下方斜坡平滑回落抬升量. */
            int16_t eff_set = PID_Controller_GetEffectiveSetpoint();
            int16_t diff_eff = (int16_t)(eff_set - meas_temp);
            if (diff_eff <= BOOST_RECOVER_HYS)
            {
                boost_target = 0;
            }
            else
            {
                /* 时长限制: 长时间仍回不了温 → 判定为异常(非焊点), 强制结束进冷却 */
                if (boost_elapsed >= BOOST_MAX_MS)
                {
                    boost_target   = 0;
                    boost_enabled  = false;
                    boost_cooldown = BOOST_COOLDOWN_MS;
                }
            }
        }

        /* 需要升降斜坡 (正在抬升或 offset 未归零): 向目标抬升量逼近,
         * 恢复/超时后目标为 0 → 自然沿斜坡回落, 避免 offset 残留 */
        if (boost_target > 0 || cur_off > 0)
        {
            int16_t rise_target = (int16_t)(boost_target - setpoint);
            int16_t delta;

            if (rise_target < 0) rise_target = 0;
            delta = (int16_t)(rise_target - cur_off);
            if (delta >  BOOST_RAMP_RATE) delta =  BOOST_RAMP_RATE;
            else if (delta < -BOOST_RAMP_RATE) delta = -BOOST_RAMP_RATE;
            PID_Controller_SetBoostOffset((int16_t)(cur_off + delta));
        }

        /* 触发前提(防误判): 曾到达过 && 工作态 && 加热器正常 && 供电充足 && 冷却结束 */
        if (boost_enabled
            && work.vin_voltage_x10 >= BOOST_VIN_MIN_x10
            && !work.heater_fault
            && work.state == WORK_STATE_ACTIVE
            && Heater_IsEnabled()
            && boost_cooldown == 0)
        {
            if (boost_elapsed >= BOOST_SAG_MS)
            {
                boost_elapsed = 0;
                boost_target  = (int16_t)(setpoint + BOOST_RISE_DEG);
                if (boost_target > (int16_t)Config_GetBoostMax())
                    boost_target = (int16_t)Config_GetBoostMax();
            }
        }
        else
        {
            boost_elapsed = 0;
        }
    }

    /* 5. 执行分段 PID 计算 */
    power = PID_Controller_Compute();
    work.power = power;

    /* 6. 设置加热器功率 */
    Heater_SetPower(power);

    /* 7. 检测温度是否到达目标 */
    {
        int16_t setpoint = PID_Controller_GetSetpoint();
        int16_t diff = meas_temp - setpoint;
        if (diff < 0)
            diff = (int16_t)(-diff);

        work.temp_reached = (diff <= TEMP_REACHED_THRESHOLD);

        /* 曾真正到达过 → 允许本目标温度下后续触发 Boost */
        boost_enabled = true;

        if (work.temp_reached && !work.temp_reached_prev)
        {
            if (g_config.buzzer_enable)
                Buzzer_Beep(BUZZER_EVENT_TEMP_REACHED);
        }
        work.temp_reached_prev = work.temp_reached;
    }

    /* 8. 加热器故障检测 */
    check_heater_diagnostic(Timer_GetTick());
}


/* ========== 接口实现 ========== */

void WorkTask_Init(void)
{
    /* 初始化运动检测引脚 (PA8, 震动开关, 下拉输入) */
    GPIO_InitTypeDef gpio;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    gpio.GPIO_Mode = GPIO_Mode_IPU;    /* 上拉输入, 开关闭合=低电平 */
    gpio.GPIO_Pin = GPIO_Pin_8;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    work.diag_temp_last = NTC_ROOM_TEMP;
    work.last_activity = Timer_GetTick();
    work.last_pid_tick = Timer_GetTick();
    work.measured_temp = NTC_ROOM_TEMP;
    work.power = 0;
    work.temp_reached = false;
    work.temp_reached_prev = false;
    work.vin_adc = 0;
    work.vin_voltage_x10 = 0;
    work.ntc_adc = 0;
    work.ntc_temp = NTC_ROOM_TEMP;
    work.temp_filt = (int32_t)AD_GetT12();   /* 首次滤波以当前读数初始化, 避免从 0 爬升 */
    work.handle_connected = true;
    work.handle_err_count = 0;
    work.motion_detected = false;
    work.diag_temp_last = work.measured_temp;
    work.thermo_fault  = false;
    work.overtemp_fault = false;
    work.diag_tick = Timer_GetTick();

    PID_Controller_SetSetpoint((int16_t)g_config.default_temp);
    PID_Controller_Reset();

    /* Boost 状态初值 */
    boost_elapsed  = 0;
    boost_enabled  = false;
    boost_cooldown = 0;
    boost_target   = 0;

    /* 运行时目标温度初值 = 默认温度 (RAM, 掉电丢失) */
    runtime_temp = (int16_t)g_config.default_temp;
}

void WorkTask_Process(void)
{
    uint32_t now = Timer_GetTick();

    /* 1. 读取输入电压 (每 200ms) */
    if ((now - work.last_pid_tick) >= PID_INTERVAL_MS)
        read_input_voltage();

    /* 2. 读取 NTC 手柄检测 (每 200ms) */
    if ((now - work.last_pid_tick) >= PID_INTERVAL_MS)
        read_ntc_handle();

    /* 3. 读取运动检测 */
    read_motion_sensor();

    /* 4. 检查休眠/关机计时器 */
    check_sleep_shutdown(now);

    /* 5. PID 控制周期 */
    if ((now - work.last_pid_tick) >= PID_INTERVAL_MS)
    {
        work.last_pid_tick = now;

        if (work.state == WORK_STATE_SHUTDOWN)
        {
            Heater_Disable();
            work.power = 0;
            return;
        }

        /* 锁存保护故障(过温/热电偶失效): 持续断热, 直到手动清除 */
        if (work.thermo_fault || work.overtemp_fault)
        {
            Heater_Disable();
            work.power = 0;
            return;
        }

        /* 手柄未连接时关加热, 并复位 PID/清除 Boost, 防止重连后从断开前的
         * 积分与陈旧微分续跑, 产生瞬时功率跳变 */
        if (!work.handle_connected)
        {
            Heater_Disable();
            work.power = 0;
            PID_Controller_Reset();
            boost_reset();
            return;
        }

        run_pid_cycle();
    }
}

void WorkTask_MarkActivity(void)
{
    work.last_activity = Timer_GetTick();

    if (work.state != WORK_STATE_ACTIVE)
    {
        work.diag_temp_last = NTC_ROOM_TEMP;
        /* 唤醒恢复运行时目标温度(HOME 临时调解值), 而非默认值 */
        PID_Controller_SetSetpoint(runtime_temp);
        PID_Controller_Reset();
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
 *   1. 重载现役 PID 参数(每型号独立) 并复位(清积分/微分/Boost 抬升);
 *   2. 运行时目标温度若超过新型号上限则钳位(T12=400, 其余=450), 避免切到
 *      低上限型号后仍按旧目标持续加热超标称;
 *   3. 目标温度重新下发并复位 PID, 用新型号控制律重新起控. */
void WorkTask_OnHandleChanged(void)
{
    /* 先重载 PID(内部 Reset), 再用新型号上限钳位目标并复位 */
    PID_Controller_Reload();

    if (runtime_temp > (int16_t)Config_GetTempMax())
        runtime_temp = (int16_t)Config_GetTempMax();
    if (runtime_temp < TEMP_MIN)
        runtime_temp = TEMP_MIN;

    PID_Controller_SetSetpoint(runtime_temp);
    PID_Controller_Reset();

    /* Boost 状态清空, 防止跨型号残留抬升 */
    boost_reset();
    last_setpoint = runtime_temp;

    /* 新型号校准点非法(会退化线性映射, 读温偏差大)时蜂鸣警示, 提示需重新校准 */
    if (!Config_IsHandleCalValid(Config_GetHandleIdx()))
        Buzzer_Beep(BUZZER_EVENT_ALARM);
}

work_state_t WorkTask_GetState(void)
{
    return work.state;
}

bool WorkTask_IsTempReached(void)
{
    return work.temp_reached;
}

bool WorkTask_IsBoostActive(void)
{
    /* 正在抬升有效设定值 或 抬升量尚未回落到 0(回落过程), 均视为 Boost 生效 */
    return (boost_target > 0) || (PID_Controller_GetBoostOffset() > 0);
}

int16_t WorkTask_GetMeasuredTemp(void)
{
    return work.measured_temp;
}

int16_t WorkTask_GetPower(void)
{
    return work.power;
}

uint8_t WorkTask_GetPowerPercent(void)
{
    uint32_t p = (uint32_t)work.power * 100 / HEATER_POWER_MAX;
    return (uint8_t)p;
}

bool WorkTask_IsHeaterFault(void)
{
    return work.heater_fault;
}

void WorkTask_ClearHeaterFault(void)
{
    work.diag_temp_last = work.measured_temp;
    work.diag_tick = Timer_GetTick();
}

/* 任一锁存保护故障(过温/热电偶失效)是否处于锁定态 */
bool WorkTask_HasFault(void)
{
    return work.heater_fault || work.thermo_fault || work.overtemp_fault;
}

/* 清除全部锁存故障, 允许重新加热 */
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

int16_t WorkTask_GetNtcTemp(void)
{
    return work.ntc_temp;
}

bool WorkTask_IsHandleConnected(void)
{
    return work.handle_connected;
}

bool WorkTask_IsMotionDetected(void)
{
    return work.motion_detected;
}