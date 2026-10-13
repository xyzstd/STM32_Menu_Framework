# -*- coding: utf-8 -*-
"""审查: 源码字符串字面量中的汉字是否全部存在于 oled_data.c 字库.

缺失字符会被 OLED_ShowChinese 静默跳过(x+=16 无绘制) -> 显示 bug, 且不报错,
只能在界面上肉眼发现. 故在改动文案后跑一次本脚本.

要点(旧版曾误报 48 个"缺失", 原因在此):
  * 统计前必须剥离注释 —— 注释里常有带引号的示例文本(如 "切换到"+"T245?"),
    其汉字并非显示用字, 混进来会产生大量假缺失, 淹没真正的缺字.
  * 字库字符只从 OLED_CF16x16[] 数组体内提取. 该数组有两种书写格式:
      格式1:  "字",            单独一行
      格式2:  0x..,..,0x..,    "字",   数据行末尾
    只认格式1 会漏掉格式2 的字(实测漏 7 个), 从而误报为缺失.
  * 顺带报告"孤儿"(字库有、源码不用), 供决定是否精简字库.
"""
import os, re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_DIRS = ['page', 'menu', 'main', 'bsp', 'ctrl', 'sys', 'ota']


def read_bom(path):
    with open(path, 'rb') as f:
        b = f.read()
    if b.startswith(b'\xef\xbb\xbf'):
        b = b[3:]
    return b.decode('utf-8')


def strip_comments(t):
    """去掉块注释、行注释与预处理诊断指令, 避免非显示文本污染统计."""
    t = re.sub(r'/\*.*?\*/', '', t, flags=re.S)
    t = re.sub(r'//[^\n]*', '', t)
    # #warning / #error 的字符串是给编译器看的诊断信息, 永远不会上屏,
    # 不应计入"需要字库支持"的汉字. (menu.c 的 MENU_DEBUG_ASSERT 提示即属此类.)
    t = re.sub(r'^[ \t]*#[ \t]*(?:warning|error)\b[^\n]*', '', t, flags=re.M)
    return t


def cjk_in_strings(src):
    """提取字符串字面量里的汉字(不含注释)."""
    out = set()
    for m in re.finditer(r'"((?:[^"\\]|\\.)*)"', src):
        out |= set(re.findall(u'[\u4e00-\u9fff]', m.group(1)))
    return out


def font_chars(path):
    """只从 OLED_CF16x16[] 数组体内提取字库字符(兼容两种书写格式)."""
    src = read_bom(path)
    i = src.index('const ChineseCell_t OLED_CF16x16[]')
    body = src[i:]
    body = body[:body.index('};')]
    chars = set()
    for line in body.split('\n'):
        m = re.match(r'^\s*"([^"]+)",\s*$', line)          # 格式1
        if m:
            chars.add(m.group(1))
            continue
        m = re.search(r'"([^"]+)",\s*$', line)             # 格式2
        if m and line.strip().startswith('0x'):
            chars.add(m.group(1))
    return chars


font = font_chars(os.path.join(ROOT, 'bsp', 'oled_data.c'))

used = set()
for d in SRC_DIRS:
    p = os.path.join(ROOT, d)
    if not os.path.isdir(p):
        continue
    for fn in sorted(os.listdir(p)):
        if fn.endswith('.c') and fn != 'oled_data.c':
            used |= cjk_in_strings(strip_comments(read_bom(os.path.join(p, fn))))

missing = sorted(used - font)
orphan = sorted(font - used)

print('font_chars=%d used_chars=%d missing=%d orphan=%d'
      % (len(font), len(used), len(missing), len(orphan)))
if missing:
    print('MISSING:', ''.join(missing))
    print('!! 这些字会显示为空白, 需补进字库(见 build_tools/gen_fonts.py)')
else:
    print('OK: all used chars present in font')
if orphan:
    print('ORPHAN(字库有但源码不用, 可精简):', ''.join(orphan))
