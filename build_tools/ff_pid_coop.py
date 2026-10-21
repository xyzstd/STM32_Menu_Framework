# -*- coding: utf-8 -*-
"""前馈曲线与 PID 的配合方式对比仿真（PC 端）.

把 ctrl/pid_controller.c 的 PID_Controller_Compute 与 main/work_task.c 的
feedforward_for 逐行移植，配双节点热模型，跑同一阶跃，对比五种配合方案：

  A 现状      : power = max(PID, FF)，积分跟踪把 PID 输出钉在 FF 上
  B 前馈+稳态 : FF = hold(T_set) + 曲线(误差)，仍用 max()
  C B + 收限幅: 积分限幅 30%/15% → 10%/8%
  D 现状 + Kd : Kd 放大到 30 倍（当前 Kd=1.00 的 D 项只有 ±2 个单位，形同虚设）
  E B+C+D     : 三者叠加

模型参数是估算值，**绝对数值不能当真**，只看方案之间的相对差异。
真实过冲必须真机验证（热质量、焊盘负载、接触电阻都会改变结果）。

用法: python build_tools/ff_pid_coop.py [--setpoint 350] [--time 180]
"""

import argparse

OUTPUT_MAX = 900          # HEATER_POWER_MAX(1000) * 90%
PID_SCALE = 100
D_FILTER = 192            # 不完全微分低通 (192/256)
LIM_LOW, LIM_HIGH = 25, 35   # 积分限幅调度区间

PLANT = dict(c1=1.0, c2=4.0, g=5.0, k=0.05, p_max=65.0, dead=0.2, amb=25.0)

DT = 0.2                  # 控制周期 200ms


def cdiv(a, b):
    q = abs(a) // abs(b)
    return q if (a >= 0) == (b >= 0) else -q


def ff_table(diff):
    """与 work_task.c feedforward_for 完全一致: 误差 → 功率."""
    d = (0, 10, 25, 50, 100)
    v = (0, 180, 350, 600, 900)
    if diff >= d[4]:
        return v[4]
    if diff > 0:
        k = 0
        while k < 4 and diff >= d[k + 1]:
            k += 1
        return v[k] + (v[k + 1] - v[k]) * (diff - d[k]) // (d[k + 1] - d[k])
    return 0


USE_FORMULA = False
HOLD_K = 70


def hold_units(sp):
    """稳态维持功率(输出单位). USE_FORMULA 时用固件可直接实现的整数式."""
    if USE_FORMULA:
        v = (sp - 25) * HOLD_K // 100     # (T_set - 25°C) * HOLD_K/100, 单位: 输出量
        return max(0, min(300, v))
    w = PLANT["k"] * (sp - PLANT["amb"])
    return int(round(w / PLANT["p_max"] * OUTPUT_MAX))


class Ctrl:
    def __init__(self, kp=1100, ki=50, kd=100, hold=False, lim_scale=1.0,
                 additive=False, d_boost=1, no_track=False, slope=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.hold = hold
        self.lim_scale = lim_scale
        self.additive = additive
        self.d_boost = d_boost
        self.no_track = no_track
        self.slope = slope
        self.integral = 0
        self.i_rem = 0
        self.d_filter = 0
        self.last_meas = 0
        self.d_first = True
        self.ff_hold = 0

    def ff(self, error):
        if self.hold and self.slope is not None:
            # 重塑曲线: 从 hold 起, 斜率 slope(单位/°C), 不再叠原表
            return max(0, min(OUTPUT_MAX, self.ff_hold + self.slope * error))
        base = ff_table(error)
        if self.hold:
            base = min(OUTPUT_MAX, base + self.ff_hold)
        return base

    def compute(self, setpoint, meas):
        error = setpoint - meas
        ff = self.ff(error)

        a = abs(error)
        lim = int(OUTPUT_MAX * 0.30 * self.lim_scale)
        if a <= LIM_LOW:
            pass
        elif a >= LIM_HIGH:
            lim = int(OUTPUT_MAX * 0.15 * self.lim_scale)
        else:
            lo = int(OUTPUT_MAX * 0.30 * self.lim_scale)
            hi = int(OUTPUT_MAX * 0.15 * self.lim_scale)
            lim = lo - (lo - hi) * (a - LIM_LOW) // (LIM_HIGH - LIM_LOW)

        p_term = cdiv(self.kp * error, PID_SCALE)

        kd = self.kd * self.d_boost
        if self.d_first:
            self.d_first = False
            self.d_filter = 0
        else:
            d_new = cdiv((self.last_meas - meas) * kd, PID_SCALE)
            self.d_filter += cdiv((d_new - self.d_filter) * D_FILTER, 256)
        self.last_meas = meas
        d_term = self.d_filter

        if self.integral > lim:
            self.integral = lim
        elif self.integral < -lim:
            self.integral = -lim
        old_int, old_rem = self.integral, self.i_rem

        raw = self.ki * error + old_rem
        inc = cdiv(raw, PID_SCALE)
        next_rem = raw - inc * PID_SCALE
        cand = old_int + inc
        if cand > lim:
            cand = lim
        elif cand < -lim:
            cand = -lim

        out = p_term + cand + d_term

        if self.additive:
            # 相加式: FF 承担模型项, PID 只做修正, 不再需要"积分跟踪前馈"
            self.integral, self.i_rem = cand, next_rem
            out = ff + p_term + self.integral + d_term
        else:
            if error > 0 and out < ff and not self.no_track:
                self.integral = ff - p_term - d_term
                if self.integral > lim:
                    self.integral = lim
                elif self.integral < -lim:
                    self.integral = -lim
                self.i_rem = 0
            elif (out > OUTPUT_MAX and inc > 0) or (out < 0 and inc < 0):
                self.integral, self.i_rem = old_int, old_rem
            else:
                self.integral, self.i_rem = cand, next_rem
            out = p_term + self.integral + d_term
            out = max(out, ff)

        return max(0, min(OUTPUT_MAX, out))


def run(sp, t_total, ctrl):
    ctrl.ff_hold = hold_units(sp)
    ctrl.last_meas = PLANT["amb"]
    T1 = T2 = PLANT["amb"]
    sub = 20
    dt = DT / sub
    dead_steps = int(round(PLANT["dead"] / dt))
    hist = []
    t = 0.0
    power = 0
    t_ctl = 0.0
    while t < t_total:
        if abs(t - t_ctl) < 1e-9:
            power = ctrl.compute(sp, T2)
            t_ctl += DT
        for _ in range(sub):
            p_w = power / OUTPUT_MAX * PLANT["p_max"]
            dT1 = (p_w - PLANT["g"] * (T1 - T2)) / PLANT["c1"]
            dT2 = (PLANT["g"] * (T1 - T2) - PLANT["k"] * (T2 - PLANT["amb"])) / PLANT["c2"]
            T1 += dT1 * dt
            T2 += dT2 * dt
            t += dt
        hist.append((t, T2, power))
    return hist


def metrics(hist, sp):
    t_rise = None
    peak = 0.0
    t_settle = 0.0
    for t, temp, _ in hist:
        if t_rise is None and temp >= sp - 5:
            t_rise = t
        peak = max(peak, temp)
        if abs(temp - sp) > 3:
            t_settle = t
    final = hist[-1]
    return dict(rise=t_rise, peak=peak, over=peak - sp, settle=t_settle,
                final=final[1], power=final[2])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--setpoint", type=int, default=350)
    ap.add_argument("--time", type=float, default=180.0)
    ap.add_argument("--p-max", type=float, default=PLANT["p_max"])
    ap.add_argument("--c2", type=float, default=PLANT["c2"])
    ap.add_argument("--k", type=float, default=PLANT["k"])
    ap.add_argument("--hold-formula", action="store_true")
    ap.add_argument("--hold-k", type=int, default=70)
    ap.add_argument("--lim-scale", type=float, default=1.0)
    args = ap.parse_args()
    PLANT.update(p_max=args.p_max, c2=args.c2, k=args.k)
    global USE_FORMULA, HOLD_K
    USE_FORMULA = args.hold_formula
    HOLD_K = args.hold_k
    LS = args.lim_scale
    sp = args.setpoint

    cases = [
        ("A 现状", dict()),
        ("B 前馈+稳态项", dict(hold=True)),
        ("C B+限幅%d/%d%%" % (int(30*LS), int(15*LS)), dict(hold=True, lim_scale=LS)),
        ("D 现状+Kd×30", dict(d_boost=30)),
        ("E B+C+D", dict(hold=True, lim_scale=LS, d_boost=30)),
        ("F 相加式(B+收限幅)", dict(hold=True, lim_scale=LS, additive=True)),
        ("G 去跟踪+收限幅", dict(hold=True, lim_scale=LS, no_track=True)),
        ("H 去跟踪+放限幅50/25%", dict(lim_scale=1.667, no_track=True)),
        ("I 重塑曲线 hold+10*e", dict(hold=True, lim_scale=1.0/3.0, slope=10)),
        ("J 重塑曲线 hold+15*e", dict(hold=True, lim_scale=1.0/3.0, slope=15)),
        ("K J+去跟踪", dict(hold=True, lim_scale=1.0/3.0, slope=15, no_track=True)),
        ("L 恒hold+限幅10%", dict(hold=True, lim_scale=1.0/3.0, slope=0)),
        ("M hold+5*e", dict(hold=True, lim_scale=1.0/3.0, slope=5)),
    ]

    print("设定 %d°C, 环境 %.0f°C, 满功率 %.0fW, 稳态维持功率约 %d/900 (%.0f%%)"
          % (sp, PLANT["amb"], PLANT["p_max"], hold_units(sp),
             hold_units(sp) * 100.0 / OUTPUT_MAX))
    print("-" * 78)
    print("%-22s %8s %8s %8s %8s %8s" % ("方案", "到温-5℃", "峰值", "过冲", "稳定±3", "末功率"))
    for name, kw in cases:
        c = Ctrl(**kw)
        h = run(sp, args.time, c)
        m = metrics(h, sp)
        print("%-22s %7.1fs %7.1f %+7.1f %7.1fs %8d"
              % (name, m["rise"] or -1, m["peak"], m["over"], m["settle"], m["power"]))
    print("-" * 78)
    print("说明: 到温/稳定时间越短越好; 过冲越接近 0 越好; 末功率应≈稳态维持功率.")
    print("Kd×30 用于验证「当前 D 项几乎不参与」这一判断, 不代表建议的 Kd 取值.")


if __name__ == "__main__":
    main()
