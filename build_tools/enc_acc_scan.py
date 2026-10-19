# -*- coding: utf-8 -*-
"""穷举 Encoder_GetSteps() 的 enc_acc 状态机, 与理想相位模型逐项对账.

固件实现 (bsp/encoder.c):
  residual = enc_acc; dir = sign(delta)
  若 residual 与 delta 反号: 先预支一个单位计数(越锚则反射相位), delta -= dir
  residual += delta; steps = residual/4 (向零截断); residual -= steps*4

理想模型: 相位 p in [0,3] 表示"距上一档的计数"; 位移 delta 后
  steps = floor((p+delta)/4), p = mod(p+delta, 4)

对账口径: 把实现输出的 enc_acc 折算成相位 p = ((acc%4)+4)%4, 比较 steps 与 p.
"""

TICKS = 4


def fw_get_steps(acc, delta):
    """逐行移植 bsp/encoder.c 的 Encoder_GetSteps (delta==0 分支返回 0)."""
    if delta == 0:
        return 0, acc
    residual = acc
    steps = 0
    d = 1 if delta > 0 else -1

    if residual != 0 and ((delta > 0) != (residual > 0)):
        s = residual + d
        if (s > 0) == (residual > 0):
            residual = -s
        else:
            residual = s
        delta -= d

    residual += delta
    # C99 有符号除法向零截断
    steps = int(residual / TICKS) if residual >= 0 else -int(-residual / TICKS)
    residual -= steps * TICKS
    return steps, residual


def ref_get_steps(p, delta):
    total = p + delta
    steps = total // TICKS          # Python // 即 floor
    p2 = total - steps * TICKS
    return steps, p2


def phase_of(acc):
    return ((acc % TICKS) + TICKS) % TICKS


def main():
    print("=" * 78)
    print("一、单批对账: acc in [-3,3] x delta in [-16,16]")
    print("=" * 78)
    bad = []
    for acc in range(-3, 4):
        for delta in range(-16, 17):
            if delta == 0:
                continue
            s1, a1 = fw_get_steps(acc, delta)
            s2, p2 = ref_get_steps(phase_of(acc), delta)
            if s1 != s2 or phase_of(a1) != p2:
                bad.append((acc, delta, s1, a1, phase_of(a1), s2, p2))

    if not bad:
        print("全部一致, 无偏差")
    else:
        print("不一致 %d 例 (acc, delta, 固件steps, 固件acc, 折算相位, 理想steps, 理想相位):" % len(bad))
        for b in bad[:40]:
            print("  acc=%+d delta=%+d -> steps=%+d acc=%+d (p=%d) | 理想 steps=%+d p=%d%s"
                  % (b[0], b[1], b[2], b[3], b[4], b[5], b[6],
                     "   <== 步数多/少一格" if b[2] != b[5] else ""))
        if len(bad) > 40:
            print("  ... 共 %d 例" % len(bad))

    print()
    print("=" * 78)
    print("二、换向专项: 正转积累 acc 后反打一批, 看是否多吐一格")
    print("=" * 78)
    print(" acc  delta | 固件steps 固件acc | 理想steps 理想p | 判定")
    for acc in range(1, 4):
        for delta in (-1, -2, -3, -4, -5, -6, -7, -8):
            s1, a1 = fw_get_steps(acc, delta)
            s2, p2 = ref_get_steps(phase_of(acc), delta)
            mark = "OK" if (s1 == s2 and phase_of(a1) == p2) else "MISMATCH"
            print("  %+d   %+d   |   %+d      %+d    |   %+d      %d   | %s"
                  % (acc, delta, s1, a1, s2, p2, mark))
        print()

    print("=" * 78)
    print("三、锁定检查: 是否存在 acc 使同向连续旋转永远吐 0 步")
    print("=" * 78)
    locked = []
    for acc in range(-3, 4):
        a = acc
        total = 0
        for _ in range(400):
            s, a = fw_get_steps(a, 1)
            total += s
        if total == 0:
            locked.append(acc)
    print("同向 delta=+1 连续 400 次后累计步数=0 的初态: %s"
          % (locked if locked else "无 —— 累加器本身不会永久锁死"))
    print("同向 delta=+4 (整格) 单批必出 1 步: %s"
          % all(fw_get_steps(a, 4)[0] == 1 for a in range(-3, 4)))

    print()
    print("=" * 78)
    print("四、随机序列对账 (20000 组, 含换向与毛刺)")
    print("=" * 78)
    import random
    random.seed(20261020)
    acc = 0
    p = 0
    err = 0
    first_err = None
    for i in range(20000):
        delta = random.choice([-8, -6, -4, -3, -2, -1, 1, 2, 3, 4, 6, 8])
        s1, acc = fw_get_steps(acc, delta)
        s2, p = ref_get_steps(p, delta)
        if s1 != s2 or phase_of(acc) != p:
            err += 1
            if first_err is None:
                first_err = (i, delta, s1, acc, s2, p)
    print("偏差批次: %d / 20000 (%.2f%%)" % (err, err / 200.0))
    if first_err:
        print("首例: 第%d批 delta=%+d -> 固件 steps=%+d acc=%+d | 理想 steps=%+d p=%d"
              % first_err)


if __name__ == "__main__":
    main()
