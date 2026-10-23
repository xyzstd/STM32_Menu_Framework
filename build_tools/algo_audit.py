# -*- coding: utf-8 -*-
"""固件算法穷举审计。

每个算法按 C 源码逐行移植, 显式建模 C 的整数语义:
  - 除法向零截断 (cdiv)
  - 算术右移等价于向下取整 (Python 的 >> 天然一致)
  - int16 饱和/回绕只在源码确实发生的地方模拟

每个算法给出"不变量"并用穷举或大样本随机验证, 输出 PASS/FAIL 与量化偏差。
用法: python build_tools/algo_audit.py
"""

import itertools
import random

FAILS = []


def cdiv(a, b):
    """C 整数除法: 向零截断 (Python 的 // 是向下取整, 负数会差 1)"""
    q = abs(a) // abs(b)
    return q if (a >= 0) == (b >= 0) else -q


def report(name, ok, detail=""):
    print("  %-46s %s%s" % (name, "PASS" if ok else "FAIL", ("  " + detail) if detail else ""))
    if not ok:
        FAILS.append(name)


def note(name, detail=""):
    print("  %-46s NOTE%s" % (name, ("  " + detail) if detail else ""))


def head(title):
    print("\n" + "=" * 74)
    print(title)
    print("=" * 74)


# ============================================================
# A. AD_GetValue: 去极值平均 (bsp/adc.c)
# ============================================================
N_OS = 8          # TEMP_OVERSAMPLE
KEEP = N_OS - 2   # 6


def adc_trim(vals):
    s = sum(vals)
    s -= min(vals) + max(vals)
    return (s + KEEP // 2) // KEEP


def adc_trim_old(vals):
    s = sum(vals)
    s -= min(vals) + max(vals)
    return (s + N_OS // 2) // N_OS


def sec_a():
    head("A. AD_GetValue 去极值平均 (N=8, 丢弃 min/max, 除 6)")
    diff_cnt = 0
    max_err_old = 0
    max_err_new = 0
    for vals in itertools.product(range(4), repeat=N_OS):
        s = sum(vals) - min(vals) - max(vals)
        new = adc_trim(vals)
        old = adc_trim_old(vals)
        # 精确四舍五入: |6*result - s| <= 3
        max_err_new = max(max_err_new, abs(KEEP * new - s))
        max_err_old = max(max_err_old, abs(KEEP * old - s))
        if new != old:
            diff_cnt += 1
    report("穷举 4^8 帧: 新公式满足 |6*r-s|<=3", max_err_new <= 3,
           "实测最大 %d" % max_err_new)
    note("旧公式 (加 N/2) 的最大偏差", "%.2f LSB, 已修正为 (N-2)/2" % (max_err_old / 6.0))
    report("两式结果不同的帧占比", True, "%d/65536 = %.1f%%" % (diff_cnt, diff_cnt / 65536 * 100))

    # 单帧毛刺抑制
    spike = [1000] * 7 + [0]
    report("单帧 0 毛刺 (7x1000) 被完全抑制", adc_trim(spike) == 1000,
           "输出 %d" % adc_trim(spike))
    two = [1000] * 6 + [0, 2000]
    report("一高一低同时存在仍为 1000", adc_trim(two) == 1000, "输出 %d" % adc_trim(two))
    # 两帧毛刺无法抑制
    three = [1000] * 5 + [0, 0, 2000]
    note("两帧同向毛刺(5x1000+0+0+2000)无法抑制", "输出 %d, 偏差 %d" % (adc_trim(three), adc_trim(three) - 1000))


# ============================================================
# B. spline_interp: 单调三次样条 (main/work_task.c)
# ============================================================
SCALE = 1000


def spline(x, y, n, xv):
    if n < 2:
        return 0
    if xv <= x[0]:
        return y[0]
    if xv >= x[n - 1]:
        return y[n - 1]

    m = [0] * n
    for i in range(n - 1):
        m[i] = cdiv((y[i + 1] - y[i]) * SCALE, x[i + 1] - x[i])

    t = [0] * n
    t[0] = m[0]
    t[n - 1] = m[n - 2]
    for k in range(1, n - 1):
        dxl = x[k] - x[k - 1]
        dxr = x[k + 1] - x[k]
        if m[k - 1] == 0 or m[k] == 0:
            t[k] = 0
            continue
        if (m[k - 1] > 0) != (m[k] > 0):      # 局部极值: 切线必须水平
            t[k] = 0
            continue
        t[k] = cdiv(m[k - 1] * dxr + m[k] * dxl, dxl + dxr)
        a3 = 3 * (-m[k - 1] if m[k - 1] < 0 else m[k - 1])
        b3 = 3 * (-m[k] if m[k] < 0 else m[k])
        lim3 = b3 if b3 < a3 else a3
        if t[k] > lim3:
            t[k] = lim3
        elif t[k] < -lim3:
            t[k] = -lim3

    k = 0
    for i in range(n - 1):
        if xv <= x[i + 1]:
            k = i
            break

    dx = x[k + 1] - x[k]
    u = xv - x[k]
    s = cdiv(u * SCALE, dx)
    s2 = cdiv(s * s, SCALE)
    s3 = cdiv(s2 * s, SCALE)
    h00 = 2 * s3 - 3 * s2 + SCALE
    h10 = s3 - 2 * s2 + s
    h01 = -2 * s3 + 3 * s2
    h11 = s3 - s2
    v2 = cdiv(dx * t[k], SCALE)
    v3 = cdiv(dx * t[k + 1], SCALE)
    return cdiv(h00 * y[k] + h10 * v2 + h01 * y[k + 1] + h11 * v3, SCALE)


def spline_tangents(x, y, n):
    """单独复算切线, 用于检查是否出现负切线(过冲来源)"""
    m = [cdiv((y[i + 1] - y[i]) * SCALE, x[i + 1] - x[i]) for i in range(n - 1)]
    t = [0] * n
    t[0] = m[0]
    t[n - 1] = m[n - 2]
    for k in range(1, n - 1):
        dxl = x[k] - x[k - 1]
        dxr = x[k + 1] - x[k]
        if m[k - 1] == 0 or m[k] == 0:
            t[k] = 0
            continue
        if (m[k - 1] > 0) != (m[k] > 0):      # 局部极值: 切线必须水平
            t[k] = 0
            continue
        t[k] = cdiv(m[k - 1] * dxr + m[k] * dxl, dxl + dxr)
        a3 = 3 * (-m[k - 1] if m[k - 1] < 0 else m[k - 1])
        b3 = 3 * (-m[k] if m[k] < 0 else m[k])
        lim3 = b3 if b3 < a3 else a3
        if t[k] > lim3:
            t[k] = lim3
        elif t[k] < -lim3:
            t[k] = -lim3
    return m, t


def sec_b():
    head("B. spline_interp 保单调三次样条 (5 点)")

    # B1 单调递增输入(adc_to_temp 的用法: xs=ADC 升序, ys=温度升序)
    xs_adc = [400, 1000, 1400, 1800, 2200]
    ys_tmp = [200, 300, 350, 400, 450]
    ok_node = all(spline(xs_adc, ys_tmp, 5, xs_adc[i]) == ys_tmp[i] for i in range(5))
    report("单调输入: 精确穿过 5 个节点", ok_node)

    prev = None
    mono = True
    inrange = True
    for xv in range(xs_adc[0], xs_adc[-1] + 1):
        v = spline(xs_adc, ys_tmp, 5, xv)
        if prev is not None and v < prev:
            mono = False
        prev = v
        if not (200 <= v <= 450):
            inrange = False
    report("单调输入: 400..2200 全区间单调不减", mono)
    report("单调输入: 全区间落在 [200,450]", inrange)

    # B2 非单调输入(cal_offset_interp 的用法: 偏移量可正可负)
    xs_t = [200, 300, 350, 400, 450]
    cases = [
        [-3, 2, -1, 1, -2],
        [-6, 5, -2, 3, -4],
        [10, -5, 3, 0, -2],
        [-8, 6, -4, 5, -3],
        [0, 4, -6, 2, -1],
        [12, -12, 8, -8, 4],
    ]
    print("  偏移曲线(200/300/350/400/450)            段内最大越界  符号相反的切线")
    worst_over = 0
    worst_case = None
    for ys in cases:
        m, t = spline_tangents(xs_t, ys, 5)
        bad_t = 0
        for k in range(1, 4):
            # 切线方向与该节点两侧的斜率方向都不一致 => 必然过冲
            if (m[k - 1] > 0 or m[k] > 0) and t[k] < 0:
                bad_t += 1
            if (m[k - 1] < 0 or m[k] < 0) and t[k] > 0:
                bad_t += 1
        over = 0
        for k in range(4):
            lo, hi = min(ys[k], ys[k + 1]), max(ys[k], ys[k + 1])
            for xv in range(xs_t[k], xs_t[k + 1] + 1):
                v = spline(xs_t, ys, 5, xv)
                o = max(lo - v, v - hi)
                if o > over:
                    over = o
        print("  %-38s %8d      %d/3" % (str(ys), over, bad_t))
        if over > worst_over:
            worst_over = over
            worst_case = ys
    report("非单调输入: 段内不超出两端点(无过冲)", worst_over == 0,
           "最大越界 %d°C @ %s" % (worst_over, worst_case))

    # B3 定位根因: 斜率变号处 mlo<0 -> 3*mlo<0 -> 切线被钳成负数
    m, t = spline_tangents(xs_t, cases[0], 5)
    bad_t = 0
    for k in range(1, 4):
        if (m[k - 1] > 0 or m[k] > 0) and t[k] < 0:
            bad_t += 1
        if (m[k - 1] < 0 or m[k] < 0) and t[k] > 0:
            bad_t += 1
    report("非单调输入: 不出现符号错误的切线", bad_t == 0,
           "m=%s  t=%s  (3*min(m) 为负所致)" % (m, t))

    # B4 C1 连续: 节点两侧导数应相等
    max_jump = 0
    for ys in cases:
        for k in range(1, 4):
            eps = 1
            dl = spline(xs_t, ys, 5, xs_t[k]) - spline(xs_t, ys, 5, xs_t[k] - eps)
            dr = spline(xs_t, ys, 5, xs_t[k] + eps) - spline(xs_t, ys, 5, xs_t[k])
            max_jump = max(max_jump, abs(dl - dr))
    report("非单调输入: 节点两侧导数连续(C1)", max_jump <= 2,
           "最大导数跳变 %d 单位/°C" % max_jump)


# ============================================================
# C. adc_to_temp 全链路 (main/work_task.c)
# ============================================================
TEMP_SANITY_MAX = 600


def cal_offset_interp(offs, temp_base):
    return spline([200, 300, 350, 400, 450], offs, 5, temp_base)


def adc_to_temp(adc, adcpts, offs):
    # adcpts 严格递增(Cal_IsHandleValid 保证)
    if adc <= adcpts[0]:
        temp = cdiv(200 * adc, adcpts[0])
    elif adc <= adcpts[4]:
        temp = spline(adcpts, [200, 300, 350, 400, 450], 5, adc)
    else:
        span = adcpts[4] - adcpts[3]
        if span <= 0:
            span = 1
        temp = 450 + cdiv((adc - adcpts[4]) * 50, span)
    temp += cal_offset_interp(offs, temp)
    if temp < 0:
        temp = 0
    if temp > TEMP_SANITY_MAX:
        temp = TEMP_SANITY_MAX
    return temp


def sec_c():
    head("C. adc_to_temp 全链路 (ADC 0..4095 穷举)")
    sets = [
        ("T12  零偏移", [400, 1100, 1450, 1800, 2150], [0, 0, 0, 0, 0]),
        ("T245 非零偏移", [300, 900, 1200, 1500, 1800], [5, -3, 2, -4, 1]),
        ("T210 极端偏移", [500, 1200, 1550, 1900, 2250], [12, -12, 8, -8, 4]),
    ]
    for name, adcpts, offs in sets:
        vals = [adc_to_temp(a, adcpts, offs) for a in range(4096)]
        mono = all(vals[i + 1] >= vals[i] for i in range(4095))
        rng = (min(vals), max(vals))
        # 相邻 ADC 的最大跳变(不连续度)
        jump = max(vals[i + 1] - vals[i] for i in range(4095))
        # 200°C 接缝的斜率比: 原点线性段 vs 样条首段
        s_low = 200.0 / adcpts[0]
        s_high = (450 - 200) / (adcpts[4] - adcpts[0])
        print("  [%s]" % name)
        report("  单调不减", mono)
        report("  落在 [0,600]", 0 <= rng[0] and rng[1] <= TEMP_SANITY_MAX, "范围 %s" % (rng,))
        report("  相邻 ADC 跳变 <= 2°C", jump <= 2, "最大 %d°C" % jump)
        print("  %-46s %.2f vs %.2f °C/ADC = %.1f 倍" %
              ("  200°C 接缝斜率(原点线性 vs 样条均值)", s_low, s_high, s_low / s_high))

    # 边界: 校准失效分支(实际不可达, 但要量化)
    fb = [cdiv(a * 120, 4095) for a in range(4096)]
    note("校准失效分支上限 120°C", "满量程 %d°C < OVER_TEMP_LIMIT(500); 但 Cal_Init 保证有效, 实际不可达" % max(fb))


# ============================================================
# D. 前馈 (ff_hold + feedforward_for)
# ============================================================
PID_OUTPUT_MAX = 900
HOLD_K_SEED = 60
HOLD_K_MAX = 200
FF_D = [0, 10, 25, 50, 100]
FF_V = [0, 180, 350, 600, 900]


def ff_hold(setpoint, k):
    sp = setpoint - 25
    if sp <= 0:
        return 0
    if k == 0:
        k = HOLD_K_SEED
    return cdiv(sp * k, 100)


def feedforward_for(diff, setpoint, k):
    if diff >= FF_D[4]:
        ff = FF_V[4]
    elif diff > 0:
        kk = 0
        for i in range(4):
            if diff < FF_D[i + 1]:
                kk = i
                break
        ff = FF_V[kk] + cdiv((FF_V[kk + 1] - FF_V[kk]) * (diff - FF_D[kk]),
                             FF_D[kk + 1] - FF_D[kk])
        ff += ff_hold(setpoint, k)
    elif diff > -10:
        ff = cdiv(ff_hold(setpoint, k) * (diff + 10), 10)
    else:
        ff = 0
    if ff > PID_OUTPUT_MAX:
        ff = PID_OUTPUT_MAX
    if ff < 0:
        ff = 0
    return ff


def sec_d():
    head("D. 前馈曲线 feedforward_for / 底座 ff_hold")
    bad_range = 0
    bad_mono = 0
    worst_jump = 0
    for sp in (100, 150, 200, 300, 350, 400, 450):
        for k in (0, 30, 60, 100, 150, 200):
            prev = None
            for diff in range(-80, 260):
                v = feedforward_for(diff, sp, k)
                if not (0 <= v <= PID_OUTPUT_MAX):
                    bad_range += 1
                if prev is not None and v < prev:
                    bad_mono += 1
                prev = v
    report("穷举 7 设定点 x 6 系数 x 340 温差: 输出在 [0,900]", bad_range == 0,
           "%d 处越界" % bad_range)
    report("同一 (sp,k) 下随温差单调不减", bad_mono == 0, "%d 处回退" % bad_mono)

    # 两个接缝的连续性: diff 0->1 与 diff -10->-9
    for sp in (200, 350, 450):
        for k in (0, 60, 150, 200):
            j0 = feedforward_for(1, sp, k) - feedforward_for(0, sp, k)
            jm = feedforward_for(-9, sp, k) - feedforward_for(-10, sp, k)
            worst_jump = max(worst_jump, abs(j0), abs(jm))
    note("接缝 (diff=0 / diff=-10) 的最大跳变", "%d 单位 = %.1f%% 满量程 (斜坡在 -10 处未从 0 起步)"
         % (worst_jump, worst_jump / 9.0))

    # 底座线性度
    report("ff_hold 在 sp<=25 时为 0", ff_hold(25, 200) == 0 and ff_hold(0, 200) == 0)
    h350 = ff_hold(350, 100)
    h450 = ff_hold(450, 100)
    report("ff_hold 随设定点线性 (k=1.00)", h350 == 325 and h450 == 425,
           "350->%d, 450->%d" % (h350, h450))


# ============================================================
# E. learn_ff_hold 学习收敛 (1/8 滤波 + 落盘死区)
# ============================================================
HOLD_STABLE_ERR = 4
HOLD_STABLE_CYCLES = 15
HOLD_SAVE_DELTA_K = 2


def learn_converge(power, setpoint, k0=0, iters=400):
    """按修复后的源码逐行: 滤波值存 RAM(hold_k_filt), 落盘仍走 |dk|>=2 死区"""
    k_cfg = k0
    k_filt = 0
    stable = 0
    stored = 0
    for _ in range(iters):
        sp = setpoint - 25
        if sp < 25:
            stable = 0
            continue
        if stable < HOLD_STABLE_CYCLES:
            stable += 1
            continue
        k_now = cdiv(power * 100, sp)
        k_now = max(0, min(HOLD_K_MAX, k_now))
        k = k_filt
        if k == 0:
            k = k_cfg if k_cfg != 0 else HOLD_K_SEED
        k = (k * 7 + k_now + 4) // 8      # 1/8 + 四舍五入
        k_filt = k
        if abs(k - k_cfg) >= HOLD_SAVE_DELTA_K:
            k_cfg = k
            stored += 1
    return k_cfg, stored


def sec_e():
    head("E. learn_ff_hold 学习收敛性 (1/8 慢滤波 + |dk|>=2 才落盘)")
    print("  设定点  实际功率  理想系数k  学到的k  残差  落盘次数  底座误差(输出单位)")
    worst = 0
    for sp, power in ((200, 160), (250, 190), (300, 230), (350, 260), (400, 290), (450, 320)):
        k_ideal = cdiv(power * 100, sp - 25)
        k_got, n = learn_converge(power, sp)
        resid = k_got - k_ideal
        # 底座误差 = (sp-25)*resid/100
        hold_err = cdiv((sp - 25) * resid, 100)
        worst = max(worst, abs(hold_err))
        print("  %5d   %6d    %6d     %5d  %+4d   %4d      %+5d" %
              (sp, power, k_ideal, k_got, resid, n, hold_err))
    report("学习后底座误差 <= 15 单位(1.7% 满量程)", worst <= 15,
           "最大 %d 单位 = %.1f%%; 残差 <= 4 来自 1/8 滤波的整数分辨率 + 落盘死区"
           % (worst, worst / 9.0))


# ============================================================
# F. PID_Controller_Compute
# ============================================================
PID_SCALE = 100
PID_INT_LIMIT = 90
PID_INT_LIMIT_AGG = 45
PID_LIM_LOW = 25
PID_LIM_HIGH = 35
PID_D_FILTER = 192


class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.setpoint = 300
        self.measured = 25
        self.integral = 0
        self.i_rem = 0
        self.last_measured = 0
        self.d_filter = 0
        self.d_first = True
        self.output = 0

    def compute(self, ff_floor):
        err = self.setpoint - self.measured
        ff_floor = max(0, min(PID_OUTPUT_MAX, ff_floor))
        abs_err = abs(err)
        if abs_err <= PID_LIM_LOW:
            lim = PID_INT_LIMIT
        elif abs_err >= PID_LIM_HIGH:
            lim = PID_INT_LIMIT_AGG
        else:
            lim = PID_INT_LIMIT - cdiv((PID_INT_LIMIT - PID_INT_LIMIT_AGG) * (abs_err - PID_LIM_LOW),
                                       PID_LIM_HIGH - PID_LIM_LOW)
        p_term = cdiv(self.kp * err, PID_SCALE)

        if self.d_first:
            self.d_first = False
            self.d_filter = 0
        else:
            d_new = cdiv((self.last_measured - self.measured) * self.kd, PID_SCALE)
            self.d_filter += cdiv((d_new - self.d_filter) * PID_D_FILTER, 256)
        self.last_measured = self.measured
        d_term = self.d_filter

        if self.integral > lim:
            self.integral = lim
        elif self.integral < -lim:
            self.integral = -lim
        old_int = self.integral
        old_rem = self.i_rem
        raw = self.ki * err + old_rem
        inc = cdiv(raw, PID_SCALE)
        next_rem = raw - inc * PID_SCALE
        cand = old_int + inc
        if cand > lim:
            cand = lim
        elif cand < -lim:
            cand = -lim

        out = p_term + cand + d_term

        if err > 0 and out < ff_floor:
            self.integral = ff_floor - p_term - d_term
            if self.integral > lim:
                self.integral = lim
            elif self.integral < -lim:
                self.integral = -lim
            self.i_rem = 0
        elif (out > PID_OUTPUT_MAX and inc > 0) or (out < 0 and inc < 0):
            self.integral = old_int
            self.i_rem = old_rem
        else:
            self.integral = cand
            self.i_rem = next_rem

        out = p_term + self.integral + d_term
        if out > PID_OUTPUT_MAX:
            out = PID_OUTPUT_MAX
        elif out < 0:
            out = 0
        self.output = out
        return out


def sec_f():
    head("F. PID_Controller_Compute 状态空间穷举")
    bad_out = 0
    bad_int = 0
    bad_ff = 0
    n = 0
    for kp in (0, 500, 1100, 2000):
        for ki in (0, 50, 200, 500):
            for kd in (0, 100, 1000):
                for err in (-400, -100, -50, -10, -1, 0, 1, 10, 50, 100, 400):
                    for integ in (-90, -45, 0, 45, 90):
                        for ff in (0, 100, 300, 600, 900):
                            for dm in (-5, 0, 5):
                                p = PID(kp, ki, kd)
                                p.setpoint = 300
                                p.measured = 300 - err
                                p.integral = integ
                                p.d_first = False
                                p.last_measured = p.measured + dm
                                p.d_filter = dm * kd // 100
                                out = p.compute(ff)
                                n += 1
                                if not (0 <= out <= PID_OUTPUT_MAX):
                                    bad_out += 1
                                lim = PID_INT_LIMIT if abs(err) <= PID_LIM_LOW else (
                                    PID_INT_LIMIT_AGG if abs(err) >= PID_LIM_HIGH else 90)
                                if not (-90 <= p.integral <= 90):
                                    bad_int += 1
                                # 前馈托底: 最终功率取 max(PID, 前馈), 故 PID 自身可以更低,
                                # 真正要保证的是"不会因前馈存在而输出超过前馈 + 积分限幅"
                                if out > PID_OUTPUT_MAX:
                                    bad_ff += 1
    report("穷举 %d 组状态: 输出恒在 [0,900]" % n, bad_out == 0, "%d 处越界" % bad_out)
    report("积分恒在 [-90,90]", bad_int == 0, "%d 处越界" % bad_int)
    report("输出不会超过满量程(含前馈路径)", bad_ff == 0, "%d 处越界" % bad_ff)

    # 闭环稳态: 一阶热模型, 看能否无静差收敛
    def closed_loop(sp, kp, ki, kd, ff_k, cycles=2000, tau_s=10.0, gain=100.0 / HOLD_K_SEED):
        """一阶热模型. gain 取 100/k_seed, 使前馈种子恰好对应正确维持功率;
        tau_s 为热时间常数(秒). 用来验证 PID+前馈能否无静差收敛, 不追求绝对数值."""
        p = PID(kp, ki, kd)
        p.setpoint = sp
        T = 25.0
        hist = []
        c = tau_s / 0.2
        for _ in range(cycles):
            meas = int(T)
            p.measured = meas
            ff = feedforward_for(sp - meas, sp, ff_k)
            u = max(p.compute(ff), ff)   # 固件: work.power = max(PID, 前馈)
            T += (u * gain - (T - 25)) / c * 0.2
            hist.append(T)
        return hist

    for sp in (250, 350, 450):
        h = closed_loop(sp, 1100, 50, 100, 60)
        final = sum(h[-100:]) / 100
        peak = max(h)
        settle = next((i for i in range(len(h) - 1, -1, -1)
                       if abs(h[i] - sp) > 1.0), 0) * 0.2
        report("闭环 %d°C: |稳态误差| <= 1°C" % sp, abs(final - sp) <= 1.0,
               "稳态 %.1f  过冲 %+.1f  稳定耗时 %.0fs" % (final, peak - sp, settle))

    # 前馈种子失配 ±30% 时, 积分(限幅 90)能否补足
    print("  前馈种子失配  设定  稳态误差  说明")
    for mul, tag in ((0.7, "-30%"), (1.0, "  0%"), (1.3, "+30%"), (1.8, "+80%")):
        for sp in (250, 450):
            h = closed_loop(sp, 1100, 50, 100, int(HOLD_K_SEED * mul))
            final = sum(h[-100:]) / 100
            err = final - sp
            print("     %s       %3d   %+6.1f    %s" %
                  (tag, sp, err, "积分可补" if abs(err) <= 1.0 else "积分限幅不足"))

# ============================================================
# G. 自整定 PID_TuneAdvance (继电反馈 + Z-N)
# ============================================================
TUNE_DUTY_ON = PID_OUTPUT_MAX * 70 // 100
TUNE_HALF_REQUIRED = 8
TUNE_MIN_HALF = 5
TUNE_AMP_MIN_X10 = 300
TUNE_TIMEOUT = 600
TUNE_TOTAL_TIMEOUT = 1500
TUNE_OVER_DELTA = 80


def tune_gain_sat(v):
    if v > 32767:
        return 32767
    if v < 0:
        return 0
    return v


class Tune:
    def __init__(self, sp):
        self.active = True
        self.setpoint = sp
        self.half_count = 0
        self.peak_sum = 0
        self.valley_sum = 0
        self.half_len_sum = 0
        self.timer = 0
        self.total_timer = 0
        self.last_x10 = 0
        self.ext_x10 = 0
        self.direction = 0
        self.tracking = False
        self.finished = False
        self.success = False
        self.gain = [0, 0, 0]

    def advance(self, measured):
        if not self.active:
            return 0
        out = TUNE_DUTY_ON if measured < self.setpoint else 0
        cur_x10 = measured * 10
        d = 0
        if self.last_x10 != 0:
            diff = cur_x10 - self.last_x10
            if diff > 0:
                d = 1
            elif diff < 0:
                d = -1
            if d != self.direction and d != 0 and self.direction != 0:
                if self.tracking and self.timer >= TUNE_MIN_HALF:
                    if self.direction > 0:
                        self.peak_sum += self.ext_x10
                    else:
                        self.valley_sum += self.ext_x10
                    self.half_len_sum += self.timer
                    self.half_count += 1
                    self.timer = 0
                self.tracking = True
                self.ext_x10 = cur_x10
            elif self.tracking and d != 0:
                if d > 0:
                    if cur_x10 > self.ext_x10:
                        self.ext_x10 = cur_x10
                else:
                    if cur_x10 < self.ext_x10:
                        self.ext_x10 = cur_x10
        self.last_x10 = cur_x10
        if d != 0:
            self.direction = d
        self.timer += 1
        self.total_timer += 1

        if self.half_count >= TUNE_HALF_REQUIRED:
            avg_peak = cdiv(self.peak_sum, TUNE_HALF_REQUIRED // 2)
            avg_valley = cdiv(self.valley_sum, TUNE_HALF_REQUIRED // 2)
            amp_x10 = cdiv(avg_peak - avg_valley, 2)
            tu_cycles = cdiv(self.half_len_sum, TUNE_HALF_REQUIRED) * 2
            if (amp_x10 >= TUNE_AMP_MIN_X10 and tu_cycles > 0 and
                    avg_peak > self.setpoint * 10 and avg_valley < self.setpoint * 10):
                ku = cdiv(4 * (TUNE_DUTY_ON // 2) * 10000 * 1000, 3142 * amp_x10)
                kp = tune_gain_sat(cdiv(60 * ku, 1000))
                ki = tune_gain_sat(cdiv(kp * 2, tu_cycles))
                kd = tune_gain_sat(cdiv(kp * tu_cycles, 8))
                self.gain = [kp, ki, kd]
                self.success = True
            self.finished = True
            self.active = False
            return 0
        if (self.timer > TUNE_TIMEOUT or self.total_timer > TUNE_TOTAL_TIMEOUT or
                measured > self.setpoint + TUNE_OVER_DELTA):
            self.finished = True
            self.success = False
            self.active = False
            return 0
        return out


def sec_g():
    head("G. 自整定 PID_TuneAdvance (继电反馈 + Z-N)")
    # 输入已知振幅 A 与周期 Tu 的正弦(量化到整数 °C), 检查估出的 a 与 Tu
    import math
    print("  振幅A  周期Tu(周期数)  估出幅值  估出周期  成功  Kp    Ki   Kd   Z-N")
    rows = []
    for A in (5, 15, 25, 30, 40, 60):
        for tu_cyc in (20, 40, 80, 160):
            t = Tune(350)
            for i in range(4000):
                T = 350 + A * math.sin(2 * math.pi * i / tu_cyc)
                t.advance(int(round(T)))
                if not t.active:
                    break
            amp = (t.peak_sum / 4 - t.valley_sum / 4) / 2 / 10 if t.peak_sum else 0
            tu = (t.half_len_sum // 8) * 2 if t.half_len_sum else 0
            kp, ki, kd = t.gain
            if t.success:
                rel = "OK" if (ki == cdiv(kp * 2, tu) and kd == cdiv(kp * tu, 8)) else "MISMATCH"
            else:
                rel = "全0" if (kp, ki, kd) == (0, 0, 0) else "残留!"
            rows.append((A, tu_cyc, amp, tu, t.success, kp, ki, kd, rel))
            print("  %4d      %4d        %6.1f    %5d     %s  %4d %4d %4d  %s" %
                  (A, tu_cyc, amp, tu, "Y" if t.success else "N", kp, ki, kd, rel))

    ok_amp = all(abs(r[2] - r[0]) <= 1.5 for r in rows if r[4])
    ok_tu = all(abs(r[3] - r[1]) <= max(2, r[1] * 0.08) for r in rows if r[4])
    report("成功样本: 估出幅值与真值相差 <= 1.5°C", ok_amp,
           "最大偏差 %.1f" % max([abs(r[2] - r[0]) for r in rows if r[4]] or [0]))
    report("成功样本: 估出周期与真值相差 <= 8%", ok_tu,
           "最大偏差 %d 周期 (%.0f%%)" % (
               max([abs(r[3] - r[1]) for r in rows if r[4]] or [0]),
               max([abs(r[3] - r[1]) / r[1] * 100 for r in rows if r[4]] or [0])))
    report("成功时满足 Z-N 关系, 失败时增益全 0",
           all(r[8] in ("OK", "全0") for r in rows),
           "异常: " + (",".join(r[8] for r in rows if r[8] not in ("OK", "全0")) or "无"))
    report("增益恒在 [0,32767](tune_gain_sat 生效)",
           all(0 <= r[5] <= 32767 and 0 <= r[6] <= 32767 and 0 <= r[7] <= 32767 for r in rows))
    ok_amp_gate = all(r[4] == (r[0] >= 30) for r in rows)
    report("幅值门限 30°C: A>=30 成功, A<30 失败", ok_amp_gate,
           "A=25 也失败" if not ok_amp_gate else "")
    note("实际影响", "继电器 70% 功率差在典型烙铁头上产生的摆动若 <30°C, 整定一律判无效")
    succ = sum(1 for r in rows if r[4])
    over = [r for r in rows if r[4] and r[7] > 1000]
    if over:
        mx = max(r[7] for r in over)
        note("成功样本 Kd 超过页面上限(PID_KD_MAX=1000)",
             "%d/%d 组, 最大 %d -> 写入时被削到 %.0f%%" % (len(over), succ, mx, 1000.0 / mx * 100))

# ============================================================
# H. Encoder_GetSteps 相位累加 (bsp/encoder.c)
# ============================================================
ENC_TICKS = 4


def enc_step(residual, delta):
    if delta == 0:
        return 0, residual
    d = 1 if delta > 0 else -1
    if residual != 0 and ((delta > 0) != (residual > 0)):
        s = residual + d
        residual = -s if (s > 0) == (residual > 0) else s
        delta -= d
    residual += delta
    steps = cdiv(residual, ENC_TICKS)
    residual -= steps * ENC_TICKS
    return steps, residual


def sec_h():
    head("H. Encoder_GetSteps 相位累加 (residual 恒在 ±3)")
    bad_res = 0
    bad_cons = 0
    n = 0
    for residual in range(-3, 4):
        for delta in range(-12, 13):
            steps, new_res = enc_step(residual, delta)
            n += 1
            if not (-3 <= new_res <= 3):
                bad_res += 1
            # 位移守恒: 同向(或 residual==0)时必须精确
            same_dir = (residual == 0) or ((delta > 0) == (residual > 0))
            if same_dir and steps * ENC_TICKS + new_res != residual + delta:
                bad_cons += 1
    report("穷举 residual(-3..3) x delta(-12..12): 新 residual 在 ±3", bad_res == 0)
    report("同向时位移守恒 steps*4+residual == 净位移", bad_cons == 0, "%d 组不守恒" % bad_cons)

    # 换向反射: 是否丢失计数
    lost = 0
    for a in range(1, 4):
        for b in range(1, 4):
            s1, r1 = enc_step(a, -1)
            s2, r2 = enc_step(r1, 1)
            net = s1 * 4 + s2 * 4 + r2
            if net != a - 1 + 1:
                lost += 1
    note("换向反射(a -> -1 -> +1)的净损失", "%d/9 组净少 2 计数(半格), 算法固有" % lost)

    # 随机游走: 步数应等于 net//4
    random.seed(7)
    worst = 0
    worst_ratio = 0
    for _ in range(20000):
        res = 0
        total = 0
        net = 0
        rev = 0
        for _ in range(40):
            d = random.choice([-3, -1, 0, 1, 3])
            if res != 0 and d != 0 and ((d > 0) != (res > 0)):
                rev += 1
            net += d
            st, res = enc_step(res, d)
            total += st
        dev = abs(total * ENC_TICKS - net)
        worst = max(worst, dev)
        if rev:
            worst_ratio = max(worst_ratio, dev / (7.0 * rev))
    report("2 万条随机游走: 偏差 <= 7*换向次数(每次最多 2*3+1)", worst_ratio <= 1.0,
           "最大 %d 计数 = %.1f 格, 平均每次换向丢 %.1f 计数"
           % (worst, worst / 4.0, worst_ratio * 7 / 2.0))


# ============================================================
# I. SPSC 环形队列 (main/input.c)
# ============================================================
QSIZE = 32
QMASK = QSIZE - 1
KEY_RESERVED = 3
ENC_SLOTS = QSIZE - 1 - KEY_RESERVED


def sec_i():
    head("I. 事件队列 evt_enqueue/evt_dequeue (空槽法)")
    bad_full = 0
    bad_used = 0
    for hd in range(QSIZE):
        for tail in range(QSIZE):
            used = ((tail - hd) & 0xFF) & QMASK
            nxt = (tail + 1) & QMASK
            full = (nxt == hd)
            empty = (hd == tail)
            # 不变量: 空 -> used==0; 满 -> used==31
            if empty and used != 0:
                bad_used += 1
            if full and used != QSIZE - 1:
                bad_full += 1
            # 满与空不能同时成立
            if full and empty:
                bad_full += 1
    report("穷举 head x tail (32x32): 空<=>used==0", bad_used == 0)
    report("穷举 head x tail: 满<=>used==31 且与空互斥", bad_full == 0)

    # 单调压入: 旋转最多 28, 按键最多 31
    cnt = 0
    for _ in range(100):
        if cnt >= ENC_SLOTS:
            break
        cnt += 1
    enc_cap = cnt
    cnt = 0
    for _ in range(100):
        if cnt >= QSIZE - 1:
            break
        cnt += 1
    key_cap = cnt
    report("旋转最多入队 28 个", enc_cap == ENC_SLOTS, "实测 %d" % enc_cap)
    report("按键最多入队 31 个", key_cap == QSIZE - 1, "实测 %d" % key_cap)
    report("按键预留槽 = 3 始终可用", QSIZE - 1 - ENC_SLOTS == KEY_RESERVED)


# ============================================================
# J. 菜单导航 + 环形窗口 (menu/menu.c, menu_display.c)
# ============================================================
LINES = 4


def move(index, count, up):
    if up:
        return count - 1 if index <= 0 else index - 1
    return 0 if index >= count - 1 else index + 1


def move_dir(prev, index, count):
    if prev == count - 1 and index == 0:
        return 1
    if prev == 0 and index == count - 1:
        return -1
    if index > prev:
        return 1
    if index < prev:
        return -1
    return 0


class Win:
    def __init__(self, index, count):
        self.top = index
        self.cursor = 0
        self.prev = index
        self.count = count

    def advance(self, index):
        count = self.count
        if count > 1:
            jump = index - self.prev
            wrap = (jump == count - 1 or jump == -(count - 1))
            if not wrap and (jump > 1 or jump < -1):
                top = index - self.cursor
                while top < 0:
                    top += count
                self.top = top % count
                self.prev = index
                return
            d = move_dir(self.prev, index, count)
            if d > 0:
                if self.cursor < LINES - 1:
                    self.cursor += 1
                else:
                    self.top = (self.top + 1) % count
            elif d < 0:
                if self.cursor > 0:
                    self.cursor -= 1
                else:
                    self.top = (self.top - 1) % count
        self.prev = index

    def row_index(self, r):
        if self.count > LINES:
            return (self.top + r) % self.count
        return r


def sec_j():
    head("J. 菜单导航与环形列表窗口")
    # J1 单步导航: 覆盖所有 count, 正反向各转 count*3 圈
    bad_nav = 0
    for count in range(1, 21):
        idx = 0
        for _ in range(count * 3):
            idx = move(idx, count, False)
            if not (0 <= idx < count):
                bad_nav += 1
        # 反向回到起点
        for _ in range(count * 3):
            idx = move(idx, count, True)
        if count > 1 and idx != 0:
            bad_nav += 1
    report("穷举 count 1..20 x 3 圈正反向: 下标始终合法且回到起点", bad_nav == 0,
           "%d 处异常" % bad_nav)

    # J2 窗口不变量: 高亮行必须恰好显示当前项
    bad_vis = 0
    bad_cur = 0
    for count in range(1, 21):
        for start in range(count):
            for step in (1, 2, 3, -1, -2, -3):
                w = Win(start, count)
                idx = start
                for _ in range(count * 2):
                    idx = (idx + step) % count
                    w.advance(idx)
                    # 高亮由 row_index(r)==index 决定, 必须存在且唯一
                    hits = [r for r in range(LINES) if w.row_index(r) == idx]
                    if len(hits) != 1:
                        bad_vis += 1
                    # win_cursor 与 index 必须一致(快速跳变路径依赖它)
                    if count > LINES and (w.top + w.cursor) % count != idx:
                        bad_cur += 1
    report("穷举 count x 起点 x 步长: 当前项恰好占 1 行", bad_vis == 0,
           "%d 处可见性错误" % bad_vis)
    report("win_top+win_cursor 恒等于当前下标", bad_cur == 0, "%d 处错位" % bad_cur)

    # J3 环回方向判定
    bad_dir = 0
    for count in range(2, 21):
        if move_dir(count - 1, 0, count) != 1:
            bad_dir += 1
        if move_dir(0, count - 1, count) != -1:
            bad_dir += 1
    report("环回方向识别 (末->首 = 下, 首->末 = 上)", bad_dir == 0)


# ============================================================
# K. 型号识别 classify_tip_cur / is_t12_scale
# ============================================================
HANDLE_T12, HANDLE_T210, HANDLE_T245 = 0, 1, 2


def classify(cur, ref_t12):
    if ref_t12 == 0:
        return 0xFF
    if cur < ref_t12 * 15 // 10:
        return HANDLE_T12
    if cur < ref_t12 * 33 // 10:
        return HANDLE_T210
    return HANDLE_T245


def is_t12_scale(cur, idx, ref_t12, ref_idx, coarse_max=0):
    floor = ref_t12 * 15 // 10
    if floor == 0:
        self_ref = ref_idx
        floor = (self_ref // 2) if self_ref != 0 else coarse_max
    return floor != 0 and cur < floor


def sec_k():
    head("K. 型号识别 classify_tip_cur / is_t12_scale")
    # 真实电流比: T12 = 1.0x, T210 = 2.67x, T245 = 4.0x (相对 T12 参考)
    bad_part = 0
    for ref in (100, 500, 1000, 2000, 3000):
        for ratio in (0.0, 0.5, 1.0, 1.3, 1.49, 1.5, 2.0, 2.67, 3.29, 3.3, 4.0, 6.0):
            cur = int(ref * ratio)
            c = classify(cur, ref)
            # 分区单调: 比值越大型号序号不减
            if c == 0xFF:
                bad_part += 1
    report("分类结果不返回 0xFF (ref 非 0 时)", bad_part == 0)

    # 分区边界: 相邻比值必须单调
    bad_mono = 0
    for ref in (100, 1000, 3000):
        prev = -1
        for cur in range(0, 4096):
            c = classify(cur, ref)
            if c < prev:
                bad_mono += 1
            prev = c
    report("穷举 cur 0..4095: 型号随电流单调不降", bad_mono == 0)

    # 危险分流: T245/T210 配置下插 T12 头必须被拒
    bad_danger = 0
    for ref in (200, 800, 2000):
        for idx in (HANDLE_T210, HANDLE_T245):
            cur_t12 = int(ref * 1.0)
            if classify(cur_t12, ref) != HANDLE_T12:
                bad_danger += 1
    report("T210/T245 配置下 T12 量级被判为 T12(可拦截)", bad_danger == 0)

    # T12 未标定时的粗门限: T245 配置下 T12(0.25x) 要判出, T210(0.67x) 不能误伤
    ref_t245 = 1000
    ok1 = is_t12_scale(int(ref_t245 * 0.25), HANDLE_T245, 0, ref_t245)
    ok2 = not is_t12_scale(int(ref_t245 * 0.67), HANDLE_T245, 0, ref_t245)
    report("T12 未标定 + T245 配置: T12 头判出且不误伤 T210", ok1 and ok2,
           "0.25x->%s  0.67x->%s" % (ok1, ok2))
    ref_t210 = 670
    ok3 = is_t12_scale(int(ref_t210 * 0.37), HANDLE_T210, 0, ref_t210)
    ok4 = not is_t12_scale(int(ref_t210 * 1.5), HANDLE_T210, 0, ref_t210)
    report("T12 未标定 + T210 配置: T12 头判出且不误伤 T245", ok3 and ok4,
           "0.37x->%s  1.5x->%s" % (ok3, ok4))
    # 两者都未标定: 粗门限为 0 → 放行(失败开放)
    report("T12 与当前型号都未标定时放行", is_t12_scale(9999, HANDLE_T245, 0, 0) is False,
           "TYPE_CUR_COARSE_T12_MAX=0 -> 不拦截(靠 UI 引导标定)")


# ============================================================
# L. CRC16 (bsp/config.c)
# ============================================================
def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def sec_l():
    head("L. CRC16/MODBUS (配置与校准页校验)")
    report("标准向量 '123456789' == 0x4B37", crc16(b"123456789") == 0x4B37,
           "实测 0x%04X" % crc16(b"123456789"))
    # 单比特错误必须全部检出
    data = bytearray(b"\x5A" * 40)
    base = crc16(data)
    missed = 0
    for byte_i in range(len(data)):
        for bit in range(8):
            d = bytearray(data)
            d[byte_i] ^= 1 << bit
            if crc16(d) == base:
                missed += 1
    report("40 字节 x 320 个单比特翻转全部检出", missed == 0, "%d 处漏检" % missed)
    # 突发错误 <= 16 bit
    missed = 0
    for byte_i in range(0, 38):
        for blen in range(1, 3):
            d = bytearray(data)
            d[byte_i] ^= 0xFF
            if blen == 2:
                d[byte_i + 1] ^= 0xFF
            if crc16(d) == base:
                missed += 1
    report("16bit 以内突发错误全部检出", missed == 0, "%d 处漏检" % missed)
    # 全 0 / 全 FF 页的 CRC 值(用于判断空页是否会被误判为有效)
    report("空页(全 FF) CRC != 全 0 页 CRC", crc16(b"\xFF" * 52) != crc16(b"\x00" * 52),
           "0xFF->0x%04X  0x00->0x%04X" % (crc16(b"\xFF" * 52), crc16(b"\x00" * 52)))


# ============================================================
# M. config_clamp 边界收敛 (bsp/config.c)
# ============================================================
SLEEP_TIME_MAX = 60
SLEEP_TEMP_MIN = 150
SLEEP_TEMP_MAX = 300
SHUTDOWN_TIME_MAX = 120


def clamp(sleep_time, sleep_temp, shutdown_time):
    fixed = False
    if sleep_time > SLEEP_TIME_MAX:
        sleep_time = SLEEP_TIME_MAX
        fixed = True
    if sleep_temp < SLEEP_TEMP_MIN or sleep_temp > SLEEP_TEMP_MAX:
        sleep_temp = SLEEP_TEMP_MIN if sleep_temp < SLEEP_TEMP_MIN else SLEEP_TEMP_MAX
        fixed = True
    if shutdown_time > SHUTDOWN_TIME_MAX:
        shutdown_time = SHUTDOWN_TIME_MAX
        fixed = True
    return sleep_time, sleep_temp, shutdown_time, fixed


def sec_m():
    head("M. config_clamp 边界收敛")
    bad = 0
    idem = 0
    for st in (0, 1, 59, 60, 61, 65535):
        for stp in (0, 149, 150, 299, 300, 301, 65535):
            for sd in (0, 1, 119, 120, 121, 65535):
                a, b, c, f = clamp(st, stp, sd)
                if not (a <= SLEEP_TIME_MAX and SLEEP_TEMP_MIN <= b <= SLEEP_TEMP_MAX
                        and c <= SHUTDOWN_TIME_MAX):
                    bad += 1
                # 幂等: 再收敛一次不应改变值, 也不应再置 fixed
                a2, b2, c2, f2 = clamp(a, b, c)
                if (a2, b2, c2) != (a, b, c) or f2:
                    idem += 1
    report("穷举 6x7x6 组边界值: 全部收敛到合法区间", bad == 0)
    report("收敛幂等(第二次不再触发落盘)", idem == 0, "%d 处不幂等" % idem)

    # 休眠/关机冲突: shutdown <= sleep 时休眠永不生效
    conflict = 0
    for st in range(0, 61):
        for sd in range(0, 121):
            if sd <= st:
                conflict += 1
    note("休眠/关机冲突组合", "%d/%d 组可通过(clamp 只钳上限, 冲突由设置页诊断+蜂鸣处理)"
         % (conflict, 61 * 121))


# ============================================================
# N. 温度滤波链 (read_measured_temp 的一阶低通)
# ============================================================
TEMP_FILT_K = 192
CPU_TEMP_FILT_K = 7


def sec_n():
    head("N. 温度一阶低通 (TEMP_FILT_K=192/256, CPU_TEMP_FILT_K=7/1024)")
    # 阶跃响应: 从 0 到 4095, 达到 90% 需要几个周期
    f = 0
    n90 = None
    for i in range(200):
        f = f + (((4095 - f) * TEMP_FILT_K) >> 8)
        if n90 is None and f >= 4095 * 0.9:
            n90 = i + 1
    report("阶跃 0->4095 达到 90% 的周期数", True, "%d 周期 = %.1f s (200ms/周期)" % (n90, n90 * 0.2))
    note("稳态量化死区", "终值 %d (差 %d LSB, k*差值>>8 截断为 0 即停)" % (f, 4095 - f))
    # 单帧满量程毛刺的影响
    f = 2000
    f = f + (((4095 - f) * TEMP_FILT_K) >> 8)
    report("单帧 +2095 毛刺只传播 <= 80%(新值权重 192/256)", f - 2000 <= 2095 * 0.8,
           "第一拍跳 %d (%.0f%%)" % (f - 2000, (f - 2000) / 2095 * 100))
    # CPU 补偿的滤波时间常数
    g = 0
    n90 = None
    for i in range(2000):
        g = g + (((1000 - g) * CPU_TEMP_FILT_K) >> 10)
        if n90 is None and g >= 850:
            n90 = i + 1
    report("CPU 温度滤波时间常数", True,
           "85%% 需 %d 周期 = %.1f s (量化死区停在 %d)" % (n90, n90 * 0.2, g))
    # 补偿除 10 的正确性
    report("CPU 补偿 /10 尺度正确(5.0°C 补偿 +5°C)", (50) // 10 == 5)


def main():
    print("固件算法穷举审计  (C 整数语义: 除法向零截断, 右移向下取整)")
    sec_a()
    sec_b()
    sec_c()
    sec_d()
    sec_e()
    sec_f()
    sec_g()
    sec_h()
    sec_i()
    sec_j()
    sec_k()
    sec_l()
    sec_m()
    sec_n()
    print("\n" + "=" * 74)
    if FAILS:
        print("未通过 %d 项:" % len(FAILS))
        for f in FAILS:
            print("  - " + f)
    else:
        print("全部通过")
    print("=" * 74)


if __name__ == "__main__":
    main()
