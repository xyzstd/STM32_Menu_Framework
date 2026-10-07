# -*- coding: utf-8 -*-
"""审查: 页面字符串字面量中的汉字是否全部存在于 oled_data.c 字库.
缺失字符会被 OLED_ShowChinese 静默跳过(x+=16 无绘制) -> 显示 bug.
只统计字符串字面量, 排除注释/头文件噪声.
"""
import os, re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

def read_bom(path):
    with open(path, 'rb') as f:
        b = f.read()
    if b.startswith(b'\xef\xbb\xbf'):
        b = b[3:]
    return b.decode('utf-8')

def cjk_in_strings(src):
    out = []
    for m in re.finditer(r'"((?:[^"\\]|\\.)*)"', src):
        out.extend(re.findall(u'[\u4e00-\u9fff]', m.group(1)))
    return out

font_src = read_bom(os.path.join(ROOT, 'bsp', 'oled_data.c'))
font_chars = set(cjk_in_strings(font_src))

used = set()
for fn in sorted(os.listdir(os.path.join(ROOT, 'page'))):
    if fn.endswith('.c'):
        used |= set(cjk_in_strings(read_bom(os.path.join(ROOT, 'page', fn))))
used |= set(cjk_in_strings(read_bom(os.path.join(ROOT, 'menu', 'menu_table.c'))))

missing = sorted(used - font_chars)
print('font_chars=%d used_chars=%d missing=%d' % (len(font_chars), len(used), len(missing)))
if missing:
    print('MISSING:', ''.join(missing))
else:
    print('OK: all used chars present in font')
