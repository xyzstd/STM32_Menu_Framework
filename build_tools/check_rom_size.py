# -*- coding: utf-8 -*-
"""App 镜像 ROM 体积自检: 从 Keil .map 读取真实写入 Flash 的字节数, 对照自律线报警.

为什么不能用 Keil 的 "Program Size" 直接判: 它报的 Code+RO **不含 RW 初值**,
而已初始化的全局变量(RW-data)同样写进 Flash. 必须用 map 的
    Total ROM Size (Code + RO Data + RW Data)
才是真正占用的 Flash 字节数. 详见 docs/Flash布局与预留核算_2026-10-14.md.

两条线(第六节"无法核实"前提下, 按固定 32 KB 槽这一更严口径设):
  * 硬性上限 32768 B(32 KB 槽): 超过即 FAIL —— Boot 若按固定 32 KB 槽擦写,
    超出的部分不会被擦/写, OTA 会留下半新半旧代码.
  * 警戒线   32000 B(留 768 B 裕度): 超过仅 WARN, 提示余量不足.

增删功能后重跑; 建议挂到构建后步骤.

用法: python build_tools/check_rom_size.py [map路径]
      默认 Listings/Release-HW/SolderingIron-HW.map(真机 target).
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_MAP = os.path.join(ROOT, 'Listings', 'Release-HW', 'SolderingIron-HW.map')

HARD_LIMIT = 32768   # 32 KB 槽: 硬约束
WARN_LIMIT = 32000   # 32768 - 768 裕度

RE_TOTAL = re.compile(
    r'Total\s+ROM\s+Size\s*\(Code \+ RO Data \+ RW Data\)\s+(\d+)')


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_MAP
    if not os.path.isfile(path):
        sys.exit('FAIL: 找不到 map 文件: %s (先完整编译一次)' % path)

    with open(path, 'r', encoding='utf-8', errors='replace') as f:
        text = f.read()

    m = RE_TOTAL.search(text)
    if not m:
        sys.exit('FAIL: %s 里找不到 "Total ROM Size (Code + RO Data + RW Data)"'
                 % os.path.basename(path))

    rom = int(m.group(1))
    print('Total ROM Size = %d B (%.2f KB)  [%s]'
          % (rom, rom / 1024.0, os.path.basename(path)))
    print('  距 32 KB 槽(%d): %d B' % (HARD_LIMIT, HARD_LIMIT - rom))

    if rom > HARD_LIMIT:
        sys.exit('FAIL: ROM %d B 超出 32 KB 硬约束, 超 %d B' % (rom, rom - HARD_LIMIT))

    if rom > WARN_LIMIT:
        print('WARN: ROM 已超警戒线 %d B(余量不足 768 B, 当前余 %d B), '
              '建议按文档第七节压缩' % (WARN_LIMIT, HARD_LIMIT - rom))
    else:
        print('OK: ROM 在警戒线内, 距警戒线仍留 %d B 裕度' % (WARN_LIMIT - rom))


if __name__ == '__main__':
    main()
