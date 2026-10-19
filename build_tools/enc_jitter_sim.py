# -*- coding: utf-8 -*-
"""量化换向反射的相位丢失, 并仿真"快速旋转 + 接触抖动"下的卡顿.

背景: bsp/encoder.c 的转向反射在 acc=+3 反向 1 计数时映射到 -2 (物理 p=2, 正确),
但再正向 1 计数时映射到 +1 (物理 p=3), 即"来回一次净丢 2 计数" —— 反射不是对合.
本脚本: (1) 穷举来回一次/多次的相位漂移; (2) 用抖动模型仿真步数产出与最长停滞.
"""

TICKS = 4
ENC_POLL_MS = 4


def fw(acc, delta):
    if delta == 0:
        return 0, acc
    r = acc
    d = 1 if delta > 0 else -1
    if r != 0 and ((delta > 0) != (r > 0)):
        s = r + d
        r = -s if (s > 0) == (r > 0) else s
        delta -= d
    r += delta
    st = int(r / TICKS) if r >= 0 else -int(-r / TICKS)
    return st, r - st * TICKS


def plain(acc, delta):
    """无反射: 纯累加, 满 4 计数出一步 (物理最直白, 无漂移)."""
    r = acc + delta
    st = int(r / TICKS) if r >= 0 else -int(-r / TICKS)
    return st, r - st * TICKS


def ph(acc):
    return ((acc % TICKS) + TICKS) % TICKS


print("=" * 76)
print("一、来回一次的相位漂移 (固件反射 vs 物理相位)")
print("=" * 76)
print(" 起点acc | 反向1计数后 | 再正向1计数后 | 物理应为 | 净丢计数")
for a0 in range(-3, 4):
    _, a1 = fw(a0, -1 if a0 > 0 else +1)
    back = +1 if a0 > 0 else -1
    _, a2 = fw(a1, back)
    # 物理: 回到原相位
    loss = (ph(a0) - ph(a2)) % TICKS
    loss = loss if loss <= 2 else loss - 4
    print("   %+d    |    %+d      |     %+d      |    %+d    |   %+d"
          % (a0, a1, a2, a0, -loss if loss else 0))

print()
print("=" * 76)
print("二、快速旋转 + 抖动仿真 (60s, 每 4ms 采样一次)")
print("=" * 76)


def simulate(step_fn, counts_per_ms, jitter, seed):
    """counts_per_ms: 净转速; jitter: 每个采样窗口插入的随机反向计数幅度."""
    import random
    rnd = random.Random(seed)
    acc = 0
    pos = 0.0
    steps_total = 0
    step_times = []
    t = 0
    while t < 60000:
        t += ENC_POLL_MS
        pos += counts_per_ms * ENC_POLL_MS
        whole = int(pos)
        pos -= whole
        delta = whole
        if jitter:
            delta += rnd.randint(-jitter, jitter)
        st, acc = step_fn(acc, delta)
        if st:
            steps_total += st
            step_times.append(t)
    gaps = [step_times[0]] + [b - a for a, b in zip(step_times, step_times[1:])]
    max_gap = max(gaps) if gaps else 60000
    return steps_total, max_gap


print(" 转速(c/ms) 抖动 | 固件: 步数 最长停滞 | 纯累加: 步数 最长停滞")
for cpm in (0.3, 0.6, 1.2):
    for jit in (0, 1, 2):
        s1, g1 = simulate(fw, cpm, jit, 1)
        s2, g2 = simulate(plain, cpm, jit, 1)
        print("   %.1f      ±%d  |  %6d  %6dms   |  %6d  %6dms"
              % (cpm, jit, s1, g1, s2, g2))

print()
print("=" * 76)
print("三、来回抖动 (用户快速小幅往复) 的净步数")
print("=" * 76)
print(" 场景: 正转 3 计数 -> 反 1 -> 正 1, 重复 N 次")
for name, fn in (("固件", fw), ("纯累加", plain)):
    acc = 0
    steps = 0
    for _ in range(50):
        for d in (3, -1, 1):
            st, acc = fn(acc, d)
            steps += st
    print("  %s: 50 轮后累计步数 = %+d (物理净位移 = %+d 计数 -> 应为 %+d 步)"
          % (name, steps, 50 * 3, (50 * 3) // 4))


# ============ 四、卡死看门狗 (ENC_STUCK_COUNTS=12) 验证 ============
STUCK_LIMIT = 16


def fw_wd(acc, delta, stuck_acc):
    """带看门狗的固件算法: 返回 (steps, acc, stuck_acc, fired)."""
    st, acc2 = fw(acc, delta)
    if delta:
        stuck_acc += abs(delta)
    fired = False
    if st != 0:
        stuck_acc = 0
    elif stuck_acc >= STUCK_LIMIT:
        acc2 = 0
        stuck_acc = 0
        fired = True
    return st, acc2, stuck_acc, fired


print()
print("=" * 76)
print("四、看门狗验证: 正常转动是否误触发 / 抖动锁死能否自动解锁")
print("=" * 76)
cases = [
    ("整格快转 delta=+4", [4] * 200),
    ("慢转 delta=+1", [1] * 200),
    ("快转+回弹 +4,-1", [4, -1] * 100),
    ("重抖动 +3,-1,+1", [3, -1, 1] * 100),
    ("抖动锁死 +1,-1", [1, -1] * 200),
    ("抖动后恢复 +1,-1 然后 +4", [1, -1] * 60 + [4] * 50),
]
for name, seq in cases:
    acc = 0
    stuck = 0
    steps = 0
    fires = 0
    for d in seq:
        st, acc, stuck, fired = fw_wd(acc, d, stuck)
        steps += st
        fires += 1 if fired else 0
    print("  %-26s 步数=%+4d 看门狗触发=%3d 次" % (name, steps, fires))
print("  判读: 前三种不应触发; 抖动锁死应触发; 恢复后应立刻继续出步")

