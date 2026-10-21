#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PID + 前馈 静态分析（PC 端，不依赖硬件）

复刻 ctrl/pid_controller.c 的参数调度与 main/work_task.c 的 feedforward_for(),
在"积分 = 0、微分 = 0"的前提下扫描 功率—误差 曲线, 回答两个问题:

  1. 前馈和 PID 谁在哪个误差区间主导? 交叉点在哪?
  2. 调某个增益, 实际能改变多少输出? 哪些参数是"死的"?

常量优先从源码解析(避免脚本与固件漂移), 解析失败则回落到内置默认值并告警.

用法:
    python build_tools/pid_analysis.py
    python build_tools/pid_analysis.py --csv out.csv
    python build_tools/pid_analysis.py --step 2 --max-error 150
    python build_tools/pid_analysis.py --no-verify

注意: 本脚本只做"静态映射"分析. 不含热惯性、不含积分累积、不含微分抑制,
      因此结论用于判断"参数敏感度", 不能代替真机轨迹验证.
"""

import argparse
import os
import re
import sys
import unicodedata

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ---- 与固件保持一致的常量 (verify_constants 会核对) ----
PID_SCALE = 100
BLEND_LOW = 25
BLEND_HIGH = 35
HEATER_POWER_MAX = 1000
OUTPUT_MAX = HEATER_POWER_MAX * 90 // 100   # = HEATER_POWER_LIMIT

FALLBACK = {
    "ff_d": [0, 10, 25, 50, 100],
    "ff_v": [0, 180, 350, 600, 900],
    # 2026-10-19 起固件为单组 PID: agg_* 与 cons_* 取同一组值,
    # 保留双键名是为了让本脚本与 pid_sim.py 的既有结构继续可用.
    "agg_kp": 1100, "agg_ki": 50, "agg_kd": 100,
    "cons_kp": 1100, "cons_ki": 50, "cons_kd": 100,
}


def dwidth(s):
    """显示宽度: 中文/全角算 2 列."""
    return sum(2 if unicodedata.east_asian_width(c) in "WF" else 1 for c in s)


def pad(s, w, align="left"):
    gap = max(0, w - dwidth(s))
    return s + " " * gap if align == "left" else " " * gap + s


def parse_source():
    """从 work_task.c / config.c 解析前馈表与默认增益. 任一失败返回 None."""
    got = {}
    try:
        wt = open(os.path.join(ROOT, "main", "work_task.c"), encoding="utf-8-sig").read()
        m = re.search(r"ff_d\s*\[\s*\d+\s*\]\s*=\s*\{([^}]*)\}", wt)
        if not m:
            return None
        got["ff_d"] = [int(x) for x in m.group(1).split(",")]
        m = re.search(r"ff_v\s*\[\s*\d+\s*\]\s*=\s*\{([^}]*)\}", wt)
        if not m:
            return None
        got["ff_v"] = [int(x) for x in m.group(1).split(",")]

        cfg = open(os.path.join(ROOT, "bsp", "config.c"), encoding="utf-8-sig").read()
        # 固件已单组化: 配置文件里只有 .pid_kp/.pid_ki/.pid_kd, 同时填入两组键
        for key in ("kp", "ki", "kd"):
            m = re.search(r"\.pid_%s\s*=\s*\{\s*(-?\d+)" % key, cfg)
            if not m:
                return None
            got["agg_" + key] = int(m.group(1))
            got["cons_" + key] = int(m.group(1))
    except OSError:
        return None

    if len(got["ff_d"]) != len(got["ff_v"]) or len(got["ff_d"]) < 2:
        return None
    return got


def feedforward(diff, ff_d, ff_v):
    """复刻 feedforward_for(): 断点间线性插值, 上限取末点."""
    if diff >= ff_d[-1]:
        return ff_v[-1]
    if diff <= 0:
        return 0
    k = 0
    while k < len(ff_d) - 1 and diff >= ff_d[k + 1]:
        k += 1
    return ff_v[k] + (ff_v[k + 1] - ff_v[k]) * (diff - ff_d[k]) // (ff_d[k + 1] - ff_d[k])


def blend(cons, agg, error):
    """复刻 pid_blend_gain(): 25~35°C 线性过渡."""
    span = BLEND_HIGH - BLEND_LOW
    pos = error - BLEND_LOW
    return cons + (agg - cons) * pos // span


def gains_at(error, p):
    """复刻 Compute() 的参数选择: 带符号 error 决定组, 负误差一律保守."""
    if error <= BLEND_LOW:
        return p["cons_kp"], p["cons_ki"], p["cons_kd"]
    if error >= BLEND_HIGH:
        return p["agg_kp"], p["agg_ki"], p["agg_kd"]
    return (blend(p["cons_kp"], p["agg_kp"], error),
            blend(p["cons_ki"], p["agg_ki"], error),
            blend(p["cons_kd"], p["agg_kd"], error))


def integral_limit(error):
    """复刻积分限幅: |error| <= 25 → 30%, >= 35 → 15%, 中间线性."""
    a = abs(error)
    lim_cons = OUTPUT_MAX * 30 // 100
    lim_agg = OUTPUT_MAX * 15 // 100
    if a <= BLEND_LOW:
        return lim_cons
    if a >= BLEND_HIGH:
        return lim_agg
    return lim_cons - (lim_cons - lim_agg) * (a - BLEND_LOW) // (BLEND_HIGH - BLEND_LOW)


def sample(error, p):
    """返回 (前馈, P 项, 最终功率, 主导方, 积分限幅)."""
    ff = feedforward(error, p["ff_d"], p["ff_v"])
    kp, _ki, _kd = gains_at(error, p)
    pterm = kp * error // PID_SCALE
    power = min(max(pterm, ff), OUTPUT_MAX)
    if pterm > ff + 2:
        lead = "PID"
    elif ff > pterm + 2:
        lead = "前馈"
    else:
        lead = "持平"
    return ff, pterm, power, lead, integral_limit(error)


def sensitivity(error, p):
    """Kp +10% 引起的最终功率变化 (仅 P 主导且未饱和时非零)."""
    ff, pterm, power, lead, _ = sample(error, p)
    kp, _ki, _kd = gains_at(error, p)
    bumped = (kp * 11 // 10) * error // PID_SCALE
    return min(max(bumped, ff), OUTPUT_MAX) - power


def kp_effect(kp, error, p):
    """给定 Kp, +10% 对最终功率的影响. 0 = 被前馈遮蔽或已饱和."""
    ff = feedforward(error, p["ff_d"], p["ff_v"])
    base = min(max(kp * error // PID_SCALE, ff), OUTPUT_MAX)
    bump = min(max((kp * 11 // 10) * error // PID_SCALE, ff), OUTPUT_MAX)
    return bump - base


def kp_ranges(kp, p, max_error):
    """Kp 有影响的连续误差区间列表 [(起, 止), ...]."""
    out, start, end = [], None, None
    for e in range(1, max_error + 1):
        if kp_effect(kp, e, p):
            if start is None:
                start = e
            end = e
        elif start is not None:
            out.append((start, end))
            start = None
    if start is not None:
        out.append((start, end))
    return out


def kp_threshold(p, upto):
    """Kp 至少要多大, 才能在 [1, upto] 内某点追上前馈 (增益 ×100)."""
    return min(feedforward(e, p["ff_d"], p["ff_v"]) * PID_SCALE // e
               for e in range(1, upto + 1))


def find_cross(p, max_error):
    """P 项首次超过前馈的误差点."""
    for e in range(1, max_error + 1):
        if sample(e, p)[1] >= sample(e, p)[0]:
            return e
    return None


def health_report(p, max_error):
    """参数体检: 报告各组 Kp 的实际有效区间. 只提示, 不判失败."""
    cross = find_cross(p, max_error)
    sat = saturation_point(p, max_error)

    print("前馈/PID 交叉点: %s" % (("误差 %d°C" % cross) if cross else "扫描范围内未交叉"))
    print("输出饱和点:     %s" % (("误差 %d°C" % sat) if sat else "未饱和"))

    bad = False
    kp = p["agg_kp"]
    rs = kp_ranges(kp, p, max_error)
    if not rs:
        print("[WARN] Kp = %.2f 全区间无影响 (被前馈遮蔽或输出已饱和)" % (kp / 100.0))
        bad = True
    else:
        desc = "、".join(("%d~%d°C" % (a, b)) if a != b else ("%d°C" % a) for a, b in rs)
        print("Kp = %.2f 有效区间: %s" % (kp / 100.0, desc))

    thr = kp_threshold(p, BLEND_LOW)
    if kp < thr:
        print("[NOTE] Kp 需 >= %.2f 才能在误差 %d°C 处追上前馈 (当前 %.2f)"
              % (thr / 100.0, BLEND_LOW, kp / 100.0))

    print("注: 单组 PID (2026-10-19 由双组合并); Ki / Kd 不受前馈遮蔽, 全程有效.")
    return 1 if bad else 0


def saturation_point(p, max_error):
    """最终功率首次到达 OUTPUT_MAX 的误差点."""
    for e in range(1, max_error + 1):
        if sample(e, p)[2] >= OUTPUT_MAX:
            return e
    return None


def main():
    ap = argparse.ArgumentParser(description="PID + 前馈静态分析")
    ap.add_argument("--step", type=int, default=5, help="误差扫描步长 (默认 5)")
    ap.add_argument("--max-error", type=int, default=120, help="最大误差 (默认 120)")
    ap.add_argument("--csv", help="把逐点数据写到 CSV")
    ap.add_argument("--no-verify", action="store_true", help="跳过源码常量核对")
    ap.add_argument("--check", action="store_true",
                    help="只做参数体检 (供 prebuild.py 调用), 输出有效区间并返回 0/1")
    args = ap.parse_args()

    p = dict(FALLBACK)
    if not args.no_verify:
        parsed = parse_source()
        if parsed is None:
            print("警告: 未能从源码解析常量, 使用内置默认值 (可能与固件不一致)\n")
        else:
            diff = [k for k in parsed if parsed[k] != FALLBACK[k]]
            p.update(parsed)
            if diff:
                print("提示: 以下常量已按源码更新: %s\n" % ", ".join(diff))
            else:
                print("常量核对: 与源码一致\n")

    print("功率标尺: 0~%d (满功率 %d 的 90%%)   PID_SCALE=%d   混合带 %d~%d°C\n"
          % (OUTPUT_MAX, HEATER_POWER_MAX, PID_SCALE, BLEND_LOW, BLEND_HIGH))

    if args.check:
        return health_report(p, args.max_error)

    hdr = [("误差", 6), ("前馈", 6), ("PID(P)", 7), ("功率", 6), ("主导", 6),
           ("积分限幅", 9), ("Kp+10%", 8)]
    line = "".join(pad(h, w, "right") + "  " for h, w in hdr)
    print(line)
    print("-" * dwidth(line))

    rows = []
    e = 0
    while e <= args.max_error:
        ff, pterm, power, lead, lim = sample(e, p)
        d = sensitivity(e, p)
        rows.append((e, ff, pterm, power, lead, lim, d))
        print(pad(str(e), 6, "right") + "  " + pad(str(ff), 6, "right") + "  "
              + pad(str(pterm), 7, "right") + "  " + pad(str(power), 6, "right") + "  "
              + pad(lead, 6, "right") + "  " + pad(str(lim), 9, "right") + "  "
              + pad(("+%d" % d) if d else "0", 8, "right"))
        e += args.step

    cross = find_cross(p, args.max_error)
    sat = saturation_point(p, args.max_error)
    print()
    if cross:
        print("交叉点: 误差 %d°C 处 P 项首次追上前馈 → 此后由 PID 主导" % cross)
    else:
        print("交叉点: 扫描范围内前馈始终主导 (PID 未追上)")
    if sat:
        print("饱和点: 误差 %d°C 处输出到达 %d (90%%), 再增大增益不再改变功率" % (sat, OUTPUT_MAX))

    # 收集"Kp 无影响"的连续区间: 首尾不能直接相减, 中间可能夹着敏感段
    ranges = []
    start = None
    for r in rows:
        if r[6] == 0:
            if start is None:
                start = r[0]
            end = r[0]
        else:
            if start is not None:
                ranges.append((start, end))
                start = None
    if start is not None:
        ranges.append((start, end))

    if ranges:
        desc = "、".join(("%d~%d°C" % (a, b)) if a != b else ("%d°C" % a) for a, b in ranges)
        print("Kp 无影响区间 (Kp+10%% 不改变输出): %s" % desc)
    if len(ranges) > 1:
        print("  → 敏感段夹在中间, 两端都是\"调了没用\"的死区")

    if args.csv:
        with open(args.csv, "w", encoding="utf-8-sig", newline="") as f:
            f.write("error,feedforward,pid_p,power,lead,integral_limit,delta_kp10\n")
            for r in rows:
                f.write("%d,%d,%d,%d,%s,%d,%d\n" % r)
        print("\n已写出 %s" % args.csv)

    return 0


if __name__ == "__main__":
    sys.exit(main())
