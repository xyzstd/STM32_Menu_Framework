# -*- coding: utf-8 -*-
"""统计 Release-HW(HW_SIM_MODE=0) 下 OLED ASCII 字库实际会被绘制的字符.

来源:
  1) OLED_ShowString / OLED_ShowChar / OLED_ShowLargeChar 调用点的字面量
  2) menu_table.c 的 .name(列表标题)
  3) 配置/型号名字符串表(bsp/config.c 等) —— UI 显示的动态文本
  4) 显示层可能输出的符号(箭头/滚动条/占位符) —— 需人工确认后加入白名单
输出: 95 个可打印 ASCII 中哪些在 8x16 / 6x8 / 16x32 三种字库里根本用不到.
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_DIRS = ['page', 'menu', 'main', 'bsp', 'ctrl', 'sys', 'ota']

PROBE = {
    'OLED_F8x16 (8x16)': None,
    'OLED_F6x8 (6x8)': None,
    'OLED_F16x32 (16x32)': None,
}


def read_bom(path):
    with open(path, 'rb') as f:
        b = f.read()
    if b.startswith(b'\xef\xbb\xbf'):
        b = b[3:]
    return b.decode('utf-8', 'replace')


def strip_comments(t):
    t = re.sub(r'/\*.*?\*/', '', t, flags=re.S)
    t = re.sub(r'//[^\n]*', '', t)
    return t


def active_lines(text, macros):
    stack = []
    out = []
    for raw in text.split('\n'):
        s = raw.strip()
        if re.match(r'#\s*if(n?def)?\b', s):
            parent = all(f[2] for f in stack) if stack else True
            macros2 = macros
            val = parent
            if parent:
                expr = re.sub(r'^#\s*(ifdef|ifndef|if)\b', '', s).strip()
                if expr.startswith('(') or 'defined' in expr:
                    # 以 HW_SIM_MODE=0 为前提求值: 逐项替换
                    e = expr.replace('HW_SIM_MODE', '0')
                    e = re.sub(r'defined\s*\(?\s*(\w+)\s*\)?', lambda m: '1' if (m.group(1) == 'HW_SIM_MODE' and False) or (m.group(1) in macros2 and m.group(1) != 'HW_SIM_MODE') else '0', e)
                    e = re.sub(r'[A-Za-z_]\w*', '0', e)
                    e = e.replace('&&', ' and ').replace('||', ' or ').replace('!', ' not ')
                    try:
                        val = bool(eval(e, {'__builtins__': {}}, {}))
                    except Exception:
                        val = True
                else:
                    val = (expr in macros2) if not expr.startswith('!') else (expr[1:] not in macros2)
            stack.append([parent, parent and val, parent and val])
            continue
        if re.match(r'#\s*elif\b', s):
            if stack:
                f = stack[-1]
                f[2] = False   # 简化: 取第一个分支
            continue
        if re.match(r'#\s*else\b', s):
            if stack:
                f = stack[-1]
                f[2] = f[0] and not f[1]
            continue
        if re.match(r'#\s*endif\b', s):
            if stack:
                stack.pop()
            continue
        if re.match(r'#\s*(include|define|undef|pragma|line|error|warning)\b', s):
            continue
        if all(f[2] for f in stack) if stack else True:
            out.append(raw)
    return out


def collect():
    macros = {'OLED_CHARSET_UTF8', 'STM32F10X_HD', 'USE_STDPERIPH_DRIVER'}
    used_8x16 = set()
    used_6x8 = set()
    used_16x32 = set()
    evidence = []   # (char, font, loc)

    files = []
    for d in SRC_DIRS:
        p = os.path.join(ROOT, d)
        if not os.path.isdir(p):
            continue
        for fn in sorted(os.listdir(p)):
            if fn.endswith(('.c', '.h')) and fn != 'oled_data.c':
                files.append(os.path.join(p, fn))

    for path in files:
        rel = os.path.relpath(path, ROOT).replace('\\', '/')
        text = strip_comments(read_bom(path))
        lines = active_lines(text, macros)
        for i, line in enumerate(lines, 1):
            for m in re.finditer(r'OLED_ShowString\s*\([^;]*?\)', line):
                seg = m.group(0)
                for sm in re.finditer(r'"((?:[^"\\]|\\.)*)"', seg):
                    for ch in sm.group(1):
                        if ' ' <= ch <= '~':
                            # 同一调用里的字体参数
                            if re.search(r'OLED_6X8', seg):
                                used_6x8.add(ch)
                            else:
                                used_8x16.add(ch)
                            evidence.append((ch, 'OLED_8X16' if 'OLED_6X8' not in seg else 'OLED_6X8', '%s:%d' % (rel, i)))
            for m in re.finditer(r'OLED_ShowChar\s*\([^;]*?\)', line):
                seg = m.group(0)
                cm = re.search(r"'((?:[^'\\]|\\.))'", seg)
                if cm:
                    ch = cm.group(1)
                    if ' ' <= ch <= '~':
                        if re.search(r'OLED_6X8', seg):
                            used_6x8.add(ch)
                        else:
                            used_8x16.add(ch)
                        evidence.append((ch, 'OLED_8X16' if 'OLED_6X8' not in seg else 'OLED_6X8', '%s:%d' % (rel, i)))

    # 16x32: 大数字 + '-' + DEG
    used_16x32 |= set('0123456789-')
    return used_8x16, used_6x8, used_16x32, evidence


def main():
    u8, u6, u32, ev = collect()
    printable = [chr(c) for c in range(0x20, 0x7F)]
    miss8 = [c for c in printable if c not in u8]
    miss6 = [c for c in printable if c not in u6]
    # 16x32 表只有 12 项(0-9, DEG, MINUS), 另外统计
    print('8x16: 字面量直接用 %d/95 个; 未直接出现 %d 个: %s' % (len(u8), len(miss8), ''.join(miss8)))
    print('6x8 : 字面量直接用 %d/95 个; 未直接出现 %d 个: %s' % (len(u6), len(miss6), ''.join(miss6)))
    print()
    print('16x32 字库 12 项(0-9, DEG, MINUS): 数字与 "-" 由 OLED_ShowLargeNum 使用;')
    print('  DEG(下标10) 由页 home 直接索引: page/home.c')
    print()
    print('== 8x16 每个字符的出现位置(前 6 处) ==')
    for c in printable:
        locs = [e[2] for e in ev if e[0] == c and e[1] == 'OLED_8X16'][:6]
        print('  %r  %s' % (c, ', '.join(locs) if locs else '— 未出现'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
