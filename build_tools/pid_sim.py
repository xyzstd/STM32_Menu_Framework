#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PID 控制策略对比仿真（PC 端，双节点热模型）

目的：回答"双 PID 到底买到了什么"。把 ctrl/pid_controller.c 的算法逐行移植到
PC 端，配一个双节点热模型（加热丝 + 烙铁头，中间有热阻），跑同一个阶跃，对比：

    A 双组（现状：25~35°C 混合，过冲用保守组）
    B 单组激进（全程 Kp 11 / Ki 0.50 / Kd 1.00）
    C 单组保守（全程 Kp 0.50 / Ki 0.60 / Kd 6.00）
    D 单组激进 + 过冲 Kd 放大（error<0 时 Kd 用 6.00）

输出的指标：到温时间、超调量、稳定时间、稳态误差、振荡次数。

⚠️ 模型参数是估算值（见 PLANT 默认），绝对数值不能当真；有意义的只是**四种策略
   之间的相对差异**。真实过冲量必须真机验证 —— 烙铁头的热质量、焊盘负载、
   环境散热都会显著改变结果。

用法:
    python build_tools/pid_sim.py
    python build_tools/pid_sim.py --setpoint 350 --time 180
    python build_tools/pid_sim.py --svg docs/pid_sim.svg
    python build_tools/pid_sim.py --p-max 90 --c2 6 --dead 0.4
"""

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from pid_analysis import (OUTPUT_MAX, PID_SCALE, BLEND_LOW, BLEND_HIGH,  # noqa: E402
                          feedforward, blend, parse_source, FALLBACK)

DT = 0.2            # 控制周期 (s), 对应 PID_INTERVAL_MS = 200
LIM_CONS = OUTPUT_MAX * 30 // 100    # 270
LIM_AGG = OUTPUT_MAX * 15 // 100     # 135
D_FILTER = 192      # 不完全微分低通系数 (192/256)

# 双节点热模型参数（估算，仅用于相对比较）
PLANT = dict(
    c1=1.0,      # 加热丝热容 J/K
    c2=4.0,      # 烙铁头热容 J/K
    g=5.0,       # 加热丝→烙铁头 热导 W/K
    k=0.05,      # 烙铁头→环境 散热系数 W/K
    p_max=65.0,  # 满占空比功率 W
    dead=0.2,    # 纯延迟 s
    amb=25.0,    # 环境温度
)


def cdiv(a, b):
    """C 语言整除（向零截断），与固件的 int 除法一致."""
    q = abs(a) // abs(b)
    return q if (a >= 0) == (b >= 0) else -q


class Plant:
    """双节点：加热丝 T1 经热阻连到烙铁头 T2，T2 向环境散热。

    控制周期是 0.2s，但热时间常数可能远小于它（c2/g 小到 0.1s 量级）。
    显式 Euler 在 dt > tau 时会数值发散，故内部按 SUB 个子步积分 ——
    控制器仍每 0.2s 更新一次，物理过程按 10ms 推进。
    """

    SUB = 20

    def __init__(self, dt, **kw):
        self.p = dict(PLANT)
        self.p.update(kw)
        self.dt = dt
        self.t1 = self.p["amb"]
        self.t2 = self.p["amb"]
        n = max(1, int(round(self.p["dead"] / dt)))
        self.buf = [0.0] * n

    def step(self, duty):
        """duty: 0~OUTPUT_MAX 的占空比（900 = 90% 上限）."""
        p = self.p
        self.buf.append(duty)
        d = self.buf.pop(0)
        watt = p["p_max"] * (d / OUTPUT_MAX)

        h = self.dt / self.SUB
        for _ in range(self.SUB):
            q12 = p["g"] * (self.t1 - self.t2)
            self.t1 += (watt - q12) / p["c1"] * h
            self.t2 += (q12 - p["k"] * (self.t2 - p["amb"])) / p["c2"] * h
        return self.t2


class Controller:
    """PID_Controller_Compute() 的逐行移植（含前馈跟踪抗饱和）."""

    def __init__(self, mode, kp_c, ki_c, kd_c, kp_a, ki_a, kd_a,
                 kd_over=None, blend_low=BLEND_LOW, blend_high=BLEND_HIGH,
                 lim_low=None, lim_high=None):
        self.mode = mode
        self.kp_c, self.ki_c, self.kd_c = kp_c, ki_c, kd_c
        self.kp_a, self.ki_a, self.kd_a = kp_a, ki_a, kd_a
        self.kd_over = kd_over
        self.lo = blend_low
        self.hi = blend_high
        # 积分限幅的调度区间可与增益调度分开：固件里两者共用 BLEND_LOW/HIGH，
        # 但这会互相牵制 —— 增益想早点激进，限幅就被迫提前收紧。
        self.lim_lo = blend_low if lim_low is None else lim_low
        self.lim_hi = blend_high if lim_high is None else lim_high
        self.integral = 0
        self.i_rem = 0
        self.d_filter = 0
        self.last_meas = None
        self.d_first = True

    def _blend(self, cons, agg, error):
        span = self.hi - self.lo
        pos = error - self.lo
        return cons + (agg - cons) * pos // span

    def gains(self, error):
        if self.mode == "cons":
            return self.kp_c, self.ki_c, self.kd_c
        if self.mode == "agg":
            return self.kp_a, self.ki_a, self.kd_a
        if self.mode == "agg_kd":
            kd = self.kd_over if error < 0 else self.kd_a
            return self.kp_a, self.ki_a, kd
        # dual: 三段调度, 边界可配
        if error <= self.lo:
            return self.kp_c, self.ki_c, self.kd_c
        if error >= self.hi:
            return self.kp_a, self.ki_a, self.kd_a
        return (self._blend(self.kp_c, self.kp_a, error),
                self._blend(self.ki_c, self.ki_a, error),
                self._blend(self.kd_c, self.kd_a, error))

    def compute(self, setpoint, meas, ff):
        error = setpoint - meas
        kp, ki, kd = self.gains(error)

        a = abs(error)
        if a <= self.lim_lo:
            lim = LIM_CONS
        elif a >= self.lim_hi:
            lim = LIM_AGG
        else:
            lim = LIM_CONS - (LIM_CONS - LIM_AGG) * (a - self.lim_lo) // (self.lim_hi - self.lim_lo)

        p_term = cdiv(kp * error, PID_SCALE)

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

        raw = ki * error + old_rem
        inc = cdiv(raw, PID_SCALE)
        next_rem = raw - inc * PID_SCALE
        cand = old_int + inc
        if cand > lim:
            cand = lim
        elif cand < -lim:
            cand = -lim

        out = p_term + cand + d_term

        if error > 0 and out < ff:
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
        return max(0, min(OUTPUT_MAX, out))


def simulate(mode, sp, t_total, plant_kw, kd_over=None, kp=None,
             blend_low=BLEND_LOW, blend_high=BLEND_HIGH,
             lim_low=None, lim_high=None):
    """跑一次阶跃，返回 (时间序列, 指标)."""
    p = dict(FALLBACK)
    parsed = parse_source()
    if parsed:
        p.update(parsed)

    kp_c = kp if kp is not None and mode == "cons" else p["cons_kp"]
    ctrl = Controller(mode, kp_c, p["cons_ki"], p["cons_kd"],
                      p["agg_kp"], p["agg_ki"], p["agg_kd"], kd_over,
                      blend_low, blend_high, lim_low, lim_high)
    plant = Plant(DT, **plant_kw)

    ts, temps = [], []
    n = int(t_total / DT)
    for i in range(n):
        t = i * DT
        meas = plant.t2
        ff = feedforward(sp - meas, p["ff_d"], p["ff_v"])
        duty = max(ctrl.compute(sp, meas, ff), ff)
        temp = plant.step(duty)
        ts.append(t + DT)
        temps.append(temp)

    return ts, temps, metrics(ts, temps, sp)


def metrics(ts, temps, sp):
    """到温时间 / 超调 / 稳定时间 / 稳态误差 / 振荡次数."""
    t_reach = None
    for t, v in zip(ts, temps):
        if v >= sp:
            t_reach = t
            break

    peak = max(temps)
    overshoot = peak - sp

    band = 5.0
    settle = None
    for i in range(len(ts) - 1, -1, -1):
        if abs(temps[i] - sp) > band:
            settle = ts[i] if i + 1 >= len(ts) else ts[min(i + 1, len(ts) - 1)]
            break
    if settle is None:
        settle = 0.0

    tail = temps[-25:] if len(temps) > 25 else temps
    steady = sum(tail) / len(tail) - sp

    # 振荡次数: 过零点(相对 sp)计数 / 2
    crossings = 0
    prev = temps[0] - sp
    for v in temps[1:]:
        cur = v - sp
        if (prev > 0) != (cur > 0) and abs(cur) > 1.0:
            crossings += 1
        prev = cur
    osc = crossings // 2

    return dict(t_reach=t_reach, overshoot=overshoot, settle=settle,
                steady=steady, osc=osc, peak=peak)


def write_svg(path, curves, sp, t_total):
    """四条温度曲线叠在一张图上."""
    W, H = 680, 380
    x0, x1 = 70, 620
    y0, y1 = 60, 300          # y1 = 温度 0, y0 = 温度上限
    tmax = t_total
    vmax = max(sp * 1.15, max(max(c[1][1]) for c in curves) * 1.05)

    def X(t):
        return x0 + (x1 - x0) * t / tmax

    def Y(v):
        return y1 - (y1 - y0) * v / vmax

    colors = ["#378ADD", "#EF9F27", "#888780", "#1D9E75"]
    parts = ['<?xml version="1.0" encoding="UTF-8"?>',
             '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 %d %d" width="%d" height="%d">'
             % (W, H, W * 2, H * 2),
             '<title>PID 策略对比仿真</title>',
             '<rect width="%d" height="%d" fill="#FFFFFF"/>' % (W, H)]

    for v in range(0, int(vmax) + 1, 100):
        parts.append('<line x1="%d" y1="%.1f" x2="%d" y2="%.1f" stroke="#D3D1C7" stroke-width="0.5"/>'
                     % (x0, Y(v), x1, Y(v)))
        parts.append('<text x="%d" y="%.1f" font-size="11" fill="#5F5E5A" text-anchor="end" '
                     'dominant-baseline="central">%d</text>' % (x0 - 8, Y(v), v))
    for t in range(0, int(tmax) + 1, 30):
        parts.append('<line x1="%.1f" y1="%d" x2="%.1f" y2="%d" stroke="#D3D1C7" stroke-width="0.5"/>'
                     % (X(t), y0, X(t), y1))
        parts.append('<text x="%.1f" y="%d" font-size="11" fill="#5F5E5A" text-anchor="middle" '
                     'dominant-baseline="central">%d</text>' % (X(t), y1 + 16, t))

    parts.append('<line x1="%d" y1="%.1f" x2="%d" y2="%.1f" stroke="#A32D2D" stroke-width="1" '
                 'stroke-dasharray="4 3"/>' % (x0, Y(sp), x1, Y(sp)))
    parts.append('<text x="%d" y="%.1f" font-size="11" fill="#A32D2D" text-anchor="end" '
                 'dominant-baseline="central">目标 %d</text>' % (x1 - 4, Y(sp) - 10, sp))

    for (label, (ts, temps)), c in zip(curves, colors):
        step = max(1, len(ts) // 300)
        pts = " ".join("%.1f,%.1f" % (X(ts[i]), Y(temps[i])) for i in range(0, len(ts), step))
        parts.append('<polyline points="%s" fill="none" stroke="%s" stroke-width="1.6"/>'
                     % (pts, c))

    ly = y0 - 22
    for i, ((label, _), c) in enumerate(zip(curves, colors)):
        lx = x0 + i * 132
        parts.append('<rect x="%d" y="%d" width="10" height="10" rx="2" fill="%s"/>'
                     % (lx, ly - 5, c))
        parts.append('<text x="%d" y="%d" font-size="11" fill="#444441" '
                     'dominant-baseline="central">%s</text>' % (lx + 15, ly, label))

    parts.append('<text x="%d" y="%d" font-size="11" fill="#5F5E5A" text-anchor="middle">时间 s</text>'
                 % ((x0 + x1) // 2, y1 + 34))
    parts.append('</svg>')

    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(parts))


def main():
    ap = argparse.ArgumentParser(description="PID 策略对比仿真")
    ap.add_argument("--setpoint", type=float, default=350.0)
    ap.add_argument("--time", type=float, default=180.0)
    ap.add_argument("--svg", help="输出对比曲线 SVG")
    ap.add_argument("--p-max", type=float, help="满占空比功率 W")
    ap.add_argument("--c2", type=float, help="烙铁头热容 J/K")
    ap.add_argument("--g", type=float, help="加热丝→烙铁头 热导 W/K")
    ap.add_argument("--k", type=float, help="散热系数 W/K")
    ap.add_argument("--dead", type=float, help="纯延迟 s")
    ap.add_argument("--cons-kp", type=float, default=None,
                    help="覆盖保守组 Kp (小数, 如 2.0) 用于验证'提高 Kp 是否有效'")
    ap.add_argument("--blend-low", type=int, default=BLEND_LOW, help="混合带下界 (默认 25)")
    ap.add_argument("--blend-high", type=int, default=BLEND_HIGH, help="混合带上界 (默认 35)")
    ap.add_argument("--sweep", action="store_true", help="扫描多组混合带边界并对比")
    args = ap.parse_args()

    kw = {}
    for name, key in (("p_max", "p_max"), ("c2", "c2"), ("g", "g"), ("k", "k"), ("dead", "dead")):
        v = getattr(args, name)
        if v is not None:
            kw[key] = v

    print("热模型: %s" % ", ".join("%s=%s" % (k, v) for k, v in
                                   (dict(PLANT, **kw)).items()))
    print("阶跃 %d°C, 仿真 %ds, 控制周期 %.1fs\n" % (args.setpoint, args.time, DT))

    cases = [
        ("A 双组(现状)", "dual", None, None),
        ("B 单组激进", "agg", None, None),
        ("C 单组保守", "cons", None, None),
        ("D 激进+过冲Kd", "agg_kd", 600, None),
    ]
    if args.cons_kp is not None:
        cases.append(("E 保守Kp=%.2f" % args.cons_kp, "cons", None, int(args.cons_kp * 100)))

    if args.sweep:
        combos = [
            (25, 35, 25, 35, "现状"),
            (20, 30, 20, 30, "整体下移"),
            (10, 20, 10, 20, "整体下移更多"),
            (5, 15, 5, 15, "激进区最宽"),
            (10, 20, 25, 35, "增益早/限幅不变"),
            (5, 15, 25, 35, "增益最早/限幅不变"),
            (5, 15, 35, 45, "增益最早/限幅最宽"),
            (10, 20, 35, 45, "折中"),
        ]
        print("%-26s%9s%8s%9s" % ("增益带 / 限幅带", "到温 s", "超调", "稳定 s"))
        print("-" * 54)
        _, _, mb = simulate("agg", args.setpoint, args.time, kw)
        print("%-26s%9.1f%+8.1f%9.1f"
              % ("[参考] 全程激进", mb["t_reach"] or 0.0, mb["overshoot"], mb["settle"]))
        for glo, ghi, llo, lhi, tag in combos:
            _, _, m = simulate("dual", args.setpoint, args.time, kw,
                               blend_low=glo, blend_high=ghi,
                               lim_low=llo, lim_high=lhi)
            print("%-26s%9.1f%+8.1f%9.1f"
                  % ("%d~%d / %d~%d %s" % (glo, ghi, llo, lhi, tag),
                     m["t_reach"] or 0.0, m["overshoot"], m["settle"]))
        print("\n[参考] = 全程激进组; 前四行是'增益带=限幅带'(固件现状的做法),")
        print("后四行把两者拆开: 增益早切、限幅保持宽松.")
        return 0

    curves, rows = [], []
    for label, mode, kd_over, kp in cases:
        ts, temps, m = simulate(mode, args.setpoint, args.time, kw, kd_over, kp)
        curves.append((label, (ts, temps)))
        rows.append((label, m))

    hdr = ("策略", 16), ("到温 s", 8), ("峰值", 7), ("超调", 7), ("稳定 s", 8), ("稳态误差", 9), ("振荡", 6)
    print("".join("%-*s" % (w, h) if i == 0 else "%*s" % (w, h) for i, (h, w) in enumerate(hdr)))
    print("-" * sum(w for _h, w in hdr))
    for label, m in rows:
        reach = "%.1f" % m["t_reach"] if m["t_reach"] else "未到"
        print("%-16s%8s%7.1f%+7.1f%8.1f%+9.1f%6d"
              % (label, reach, m["peak"], m["overshoot"], m["settle"], m["steady"], m["osc"]))
    print("\n到温 = 首次触及目标; 稳定 = 此后不再离开 ±5°C 的时刻 (可早于到温);")
    print("振荡 = 相对目标过零次数/2; 稳态误差 = 最后 5s 均值 - 目标.")

    if args.svg:
        write_svg(args.svg, curves, args.setpoint, args.time)
        print("\n已写出 %s" % args.svg)

    print("\n注: 热模型参数为估算值, 只看相对差异; 真实过冲必须真机验证.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
