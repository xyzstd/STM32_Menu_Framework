# -*- coding: utf-8 -*-
"""温度校准表分析：多段校准到底买到了什么

读 bsp/config.c 的默认校准表，回答三个问题：
  1. 每段斜率（ADC counts/°C）与温度分辨率（°C/count）——哪一段最脆
  2. 2 点 / 3 点 / 5 点插值之间的最大差异——多出来的点改变了多少结果
  3. 表外推到哪里会撞上热电偶开路阈值（THERMO_ADC_OPEN）

用法：
    python build_tools/cal_analysis.py
    python build_tools/cal_analysis.py --open 4000 --noise 3
"""
import argparse
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TEMPS = [200, 300, 350, 400, 450]


def load_cal(override=None):
    if override:
        vals = [int(x) for x in override.split(",")]
        if len(vals) != len(TEMPS):
            raise SystemExit("--adc 需要 %d 个值" % len(TEMPS))
        return vals

    src = open(os.path.join(ROOT, "bsp", "config.c"), encoding="utf-8-sig").read()
    vals = []
    for t in TEMPS:
        m = re.search(r"\.cal_adc_%d\s*=\s*\{\s*(\d+)" % t, src)
        if not m:
            raise SystemExit("解析失败: cal_adc_%d" % t)
        vals.append(int(m.group(1)))
    return vals


def load_open_threshold():
    h = open(os.path.join(ROOT, "bsp", "config.h"), encoding="utf-8-sig").read()
    m = re.search(r"#define\s+THERMO_ADC_OPEN\s+(\d+)", h)
    return int(m.group(1)) if m else 4000


def monotone_cubic(xs, ys):
    """Fritsch–Carlson 单调三次插值，返回 f(x)（区间外线性外推）."""
    n = len(xs)
    h = [xs[i + 1] - xs[i] for i in range(n - 1)]
    d = [(ys[i + 1] - ys[i]) / h[i] for i in range(n - 1)]

    m = [0.0] * n
    m[0], m[-1] = d[0], d[-1]
    for i in range(1, n - 1):
        if d[i - 1] * d[i] <= 0:
            m[i] = 0.0
        else:
            w1 = 2 * h[i] + h[i - 1]
            w2 = h[i] + 2 * h[i - 1]
            m[i] = (w1 + w2) / (w1 / d[i - 1] + w2 / d[i])

    def f(x):
        if x <= xs[0]:
            return ys[0] + m[0] * (x - xs[0])
        if x >= xs[-1]:
            return ys[-1] + m[-1] * (x - xs[-1])
        i = 0
        while i < n - 2 and x > xs[i + 1]:
            i += 1
        t = (x - xs[i]) / h[i]
        h00 = 2 * t ** 3 - 3 * t ** 2 + 1
        h10 = t ** 3 - 2 * t ** 2 + t
        h01 = -2 * t ** 3 + 3 * t ** 2
        h11 = t ** 3 - t ** 2
        return h00 * ys[i] + h10 * h[i] * m[i] + h01 * ys[i + 1] + h11 * h[i] * m[i + 1]

    return f


def piecewise_linear(xs, ys):
    def f(x):
        if x <= xs[0]:
            k = (ys[1] - ys[0]) / (xs[1] - xs[0])
            return ys[0] + k * (x - xs[0])
        if x >= xs[-1]:
            k = (ys[-1] - ys[-2]) / (xs[-1] - xs[-2])
            return ys[-1] + k * (x - xs[-1])
        i = 0
        while i < len(xs) - 2 and x > xs[i + 1]:
            i += 1
        k = (ys[i + 1] - ys[i]) / (xs[i + 1] - xs[i])
        return ys[i] + k * (x - xs[i])
    return f


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--noise", type=int, default=3, help="ADC 峰峰噪声 (counts), 默认 3")
    ap.add_argument("--step", type=int, default=2, help="扫描步长 (counts)")
    ap.add_argument("--adc", help="覆盖校准表: 逗号分隔 5 个 ADC 值")
    args = ap.parse_args()

    adc = load_cal(args.adc)
    open_th = load_open_threshold()

    print("校准表 (默认值):")
    for t, a in zip(TEMPS, adc):
        print("  %3d°C → %4d" % (t, a))

    print("\n分段斜率 / 分辨率:")
    print("  %-14s%10s%12s%14s" % ("区间", "counts/°C", "°C/count", "±噪声→±°C"))
    for i in range(len(TEMPS) - 1):
        span_t = TEMPS[i + 1] - TEMPS[i]
        span_a = adc[i + 1] - adc[i]
        per_c = span_a / span_t if span_t else 0
        c_per = span_t / span_a if span_a else float("inf")
        print("  %-14s%10.2f%12.2f%14.2f"
              % ("%d→%d°C" % (TEMPS[i], TEMPS[i + 1]),
                 per_c, c_per, c_per * args.noise))

    f5 = monotone_cubic(adc, TEMPS)
    schemes = [
        ("2 点 (200/450) 线性", piecewise_linear([adc[0], adc[-1]], [TEMPS[0], TEMPS[-1]])),
        ("3 点 (200/350/450) 线性",
         piecewise_linear([adc[0], adc[2], adc[-1]], [TEMPS[0], TEMPS[2], TEMPS[-1]])),
        ("5 点 单调三次样条", f5),
    ]

    print("\n与 5 点样条的偏差 (°C), 扫描 ADC %d~%d:" % (adc[0], adc[-1]))
    print("  %-24s%10s%10s" % ("方案", "最大偏差", "平均偏差"))
    for name, f in schemes:
        errs = []
        x = adc[0]
        while x <= adc[-1]:
            errs.append(abs(f(x) - f5(x)))
            x += args.step
        print("  %-24s%10.2f%10.2f" % (name, max(errs), sum(errs) / len(errs)))

    t_open = f5(open_th)
    print("\n开路阈值 %d 对应的温度: %.1f°C" % (open_th, t_open))
    if t_open < TEMPS[-1]:
        print("  ⚠ 阈值落在标称量程(%d°C)以内 —— 高于 %.0f°C 后读数会先撞开路检测"
              % (TEMPS[-1], t_open))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
