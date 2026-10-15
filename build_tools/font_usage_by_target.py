# -*- coding: utf-8 -*-
"""按预处理条件分别统计两套目标(真机 HW_SIM_MODE=0 / 模拟 HW_SIM_MODE=1)
下 UI 实际会显示的汉字, 与 bsp/oled_data.c 字库对比, 找出各自"用不到的字".

用法: python build_tools/font_usage_by_target.py
退出码: 0=两套目标字库均无多余; 1=至少一套存在多余字(不失败构建, 仅报表).
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_DIRS = ['page', 'menu', 'main', 'bsp', 'ctrl', 'sys', 'ota']
FONT_FILE = os.path.join(ROOT, 'bsp', 'oled_data.c')

DEFINED_BASE = [
    'OLED_CHARSET_UTF8',
    'STM32F10X_HD',
    'USE_STDPERIPH_DRIVER',
    '__CC_ARM',
    'ARMCC',
    '__arm__',
    '__cplusplus_DISABLED',
]


def read_bom(path):
    with open(path, 'rb') as f:
        b = f.read()
    if b.startswith(b'\xef\xbb\xbf'):
        b = b[3:]
    return b.decode('utf-8', 'replace')


def strip_comments(t):
    t = re.sub(r'/\*.*?\*/', '', t, flags=re.S)
    t = re.sub(r'//[^\n]*', '', t)
    t = re.sub(r'^[ \t]*#[ \t]*(?:warning|error)\b[^\n]*', '', t, flags=re.M)
    return t


# ---------------------------------------------------------------- 预处理
def eval_cond(expr, macros):
    """极简 #if 表达式求值: 识别整数常量 / defined() / 标识符真假 / ! && || ()."""
    e = expr.strip()
    e = re.sub(r'/\*.*?\*/', '', e)
    # defined(X) / defined X
    e = re.sub(r'defined\s*\(\s*(\w+)\s*\)',
               lambda m: '1' if m.group(1) in macros else '0', e)
    e = re.sub(r'defined\s+(\w+)',
               lambda m: '1' if m.group(1) in macros else '0', e)
    # 未定义标识符一律视为 0 (C 语义: 未定义标识符替换为 0)
    e = re.sub(r'(?<![\w.])(?!\d)([A-Za-z_]\w*)(?![\w(])', '0', e)

    py = re.sub(r'&&', ' and ', e)
    py = re.sub(r'\|\|', ' or ', py)
    py = re.sub(r'!', ' not ', py)
    try:
        return bool(eval(py, {'__builtins__': {}}, {}))
    except Exception:
        return True   # 看不懂就当真, 宁可多算不可漏算


def active_lines(text, macros):
    """返回条件编译后仍存续的行 (不含预处理指令行, 注释已剥离)."""
    stack = []          # [parent_active, branch_taken, this_active]
    out = []
    for raw in text.split('\n'):
        s = raw.strip()
        m = re.match(r'#\s*(ifdef|ifndef)\s+(\w+)', s)
        if m:
            parent = all(f[2] for f in stack) if stack else True
            name = m.group(2)
            val = (name in macros) if m.group(1) == 'ifdef' else (name not in macros)
            stack.append([parent, parent and val, parent and val])
            continue
        m = re.match(r'#\s*if(n?def)?\b', s)
        if re.match(r'#\s*if\b', s):
            parent = all(f[2] for f in stack) if stack else True
            expr = s[s.index('#') + 1:][2:].lstrip()
            val = eval_cond(expr, macros) if parent else False
            stack.append([parent, parent and val, parent and val])
            continue
        if re.match(r'#\s*elif\b', s):
            if stack:
                f = stack[-1]
                parent = f[0]
                val = False if f[1] else (eval_cond(s[4:].lstrip()[4:].lstrip(), macros) if parent else False)
                expr = re.sub(r'^#\s*elif', '', s).strip()
                if parent and not f[1]:
                    val = eval_cond(expr, macros)
                f[2] = parent and val
                f[1] = f[1] or (parent and val)
            continue
        if re.match(r'#\s*else\b', s):
            if stack:
                f = stack[-1]
                parent = f[0]
                f[2] = parent and not f[1]
                f[1] = True
            continue
        if re.match(r'#\s*endif\b', s):
            if stack:
                stack.pop()
            continue
        if re.match(r'#\s*(include|define|undef|pragma|line|error|warning)\b', s):
            continue
        active = all(f[2] for f in stack) if stack else True
        if active:
            out.append(raw)
    return out


def cjk_in_strings(text):
    out = set()
    for m in re.finditer(r'"((?:[^"\\]|\\.)*)"', text):
        out |= set(re.findall(u'[\u4e00-\u9fff]', m.group(1)))
    return out


def font_chars():
    src = read_bom(FONT_FILE)
    i = src.index('const ChineseCell_t OLED_CF16x16[]')
    body = src[i:]
    body = body[:body.index('};')]
    chars = set()
    for line in body.split('\n'):
        m = re.match(r'^\s*"([^"]+)",\s*$', line)
        if m:
            chars.add(m.group(1))
            continue
        m = re.search(r'"([^"]+)",\s*$', line)
        if m and line.strip().startswith('0x'):
            chars.add(m.group(1))
    return chars


def used_for(macros):
    used = set()
    where = {}
    for d in SRC_DIRS:
        p = os.path.join(ROOT, d)
        if not os.path.isdir(p):
            continue
        for fn in sorted(os.listdir(p)):
            if not (fn.endswith('.c') or fn.endswith('.h')) or fn == 'oled_data.c':
                continue
            path = os.path.join(p, fn)
            rel = os.path.relpath(path, ROOT).replace('\\', '/')
            text = strip_comments(read_bom(path))
            rel_lines = active_lines(text, macros)
            for i, line in enumerate(rel_lines, 1):
                for ch in cjk_in_strings(line):
                    used.add(ch)
                    where.setdefault(ch, []).append('%s:%d' % (rel, i))
    return used, where


def main():
    font = font_chars()
    print('字库字数: %d' % len(font))

    results = []
    for name, sim in (('Release-HW (HW_SIM_MODE=0)', 0), ('Debug-Sim (HW_SIM_MODE=1)', 1)):
        macros = set(DEFINED_BASE)
        if sim:
            macros.add('HW_SIM_MODE')
        used, where = used_for(macros)
        extra = sorted(font - used)
        missing = sorted(used - font)
        results.append((name, extra, missing))
        print('\n== %s ==' % name)
        print('  已定义宏: %s' % ' '.join(sorted(macros)))
        print('  UI 用字: %d  字库多余: %d  显示缺字: %d' % (len(used), len(extra), len(missing)))
        if extra:
            for ch in extra:
                locs = where.get(ch, [])
                print('    多余 %s  出现于: %s' % (ch, ', '.join(locs[:4]) if locs else '(仅注释)'))
        if missing:
            print('    缺字: %s' % ''.join(missing))

    hw_extra = set(results[0][1])
    sim_extra = set(results[1][1])
    both = hw_extra & sim_extra
    print('\n两套目标都用不到的字 (%d): %s' % (len(both), ''.join(sorted(both)) if both else '无'))
    hw_only = hw_extra - sim_extra
    sim_only = sim_extra - hw_extra
    if hw_only:
        print('仅真机用不到 (HW extra): %s' % ''.join(sorted(hw_only)))
    if sim_only:
        print('仅模拟用不到 (SIM extra): %s' % ''.join(sorted(sim_only)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
