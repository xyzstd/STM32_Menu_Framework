# -*- coding: utf-8 -*-
"""ASCII 字库裁剪校验: 运行期可达字符必须全部编入固件(flag=1).

背景: bsp/oled_data.c 的 ASCII 字库是紧凑表 + 逐字形 G8 宏开关
(见该文件顶部裁剪说明). flag=0 的字形不写入固件, 显示时经
oled_ascii_glyph() 退化为 '?' —— 失败可见, 但终归是显示缺陷.
本脚本从源码重新推导"运行期可达字符集", 确保没有可达字符被误剔.
增删显示文案/动态字符串后必须重跑.

可达集 = 源码字符串字面量 + 字符字面量(已剥离注释与 #include)
       + 运行期取值域(OLED_ShowNum: '0'+digit 与 '-'; 补位空格;
         OLED_ShowChar 越界兜底 '?'; 校准页 '+'/'-' 号)
       - 仅用于比较的非显示常量(如 OLED_ShowChar 里 ch>'~' 的 '~')

已知局限(按字面量统计, 无法区分上屏与不上屏):
  * 串口/调试 print 的字符串永远不上屏, 但也会被计入字面量.
    若因此报错, 正确处理三选一:
      1) 该字符确实要上屏   -> 把 oled_data.c 对应行 flag 改成 1;
      2) 仅调试用, 不上屏   -> 把该字符加进本脚本 NON_DISPLAY 并注明原因;
      3) 都不需要           -> 改文案.
  * 运行期拼接出的字符串(如 "切换到"+型号+"?")要求各来源都是字面量;
    若将来引入真正的运行期生成(如 itoa 之外的花样), 需手工扩 RUNTIME_ONLY.

同时校验字库表自身结构:
  * 8x16 与 6x8 两表的编入字符集必须一致(并集策略, 防止两种字体缺字不对称);
  * 95 行无重复字符, 第 i 行对应 chr(0x20+i)(与旧 ch-' ' 索引可对照);
  * '?' 必须 flag=1(未命中兜底行的前提);
  * 每行字节数 = 字符 1 + 字形宽度.
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FONT = os.path.join(ROOT, 'bsp', 'oled_data.c')
SRC_DIRS = ['page', 'menu', 'main', 'bsp', 'ctrl', 'sys', 'ota']

# 运行期取值域: 源码里没有(或不只有)字面量, 但界面一定会显示
RUNTIME_ONLY = set('0123456789+- ?')
# 仅用于比较、从不上屏的字符字面量
NON_DISPLAY = {'~'}          # bsp/oled.c: if(ch<' '||ch>'~') 的边界比较

# 不参与"可达字符"统计的源文件(相对 ROOT 的路径):
#   * bsp/oled_data.c          —— 字库本体, 含全部 95 字符, 统计它无意义;
#   * ota/app_side/ota_boot.c  —— Boot 标志页搬运 + UART 调试 printf, 从不上屏,
#     其调试文案里的 '['/']'/'X'/'('/')' 等会把可达集污染成假 MISSING.
SKIP_FILES = {os.path.join('bsp', 'oled_data.c'),
              os.path.join('ota', 'app_side', 'ota_boot.c')}

# 紧凑表行: G8(flag, 'ch', 0x..,...)
ROW_RE = re.compile(r"G8\(\s*([01])\s*,\s*'(\\.|[^'\\])'\s*,([^)]*)\)")
STR_RE = re.compile(r'"((?:[^"\\\n]|\\.)*)"')
CHR_RE = re.compile(r"'(\\.|[^'\\])'")


def read_bom(path):
    with open(path, 'rb') as f:
        b = f.read()
    if b.startswith(b'\xef\xbb\xbf'):
        b = b[3:]
    return b.decode('utf-8')


def strip_noise(t):
    """剥离注释、预处理诊断与 #include 字符串, 避免非显示文本混入统计."""
    out = []
    i, n = 0, len(t)
    while i < n:
        c = t[i]
        if c == '"':                       # 字符串字面量整体保留
            j = i + 1
            while j < n:
                if t[j] == '\\':
                    j += 2
                    continue
                if t[j] == '"':
                    j += 1
                    break
                j += 1
            out.append(t[i:j])
            i = j
        elif c == '/' and i + 1 < n and t[i + 1] == '*':
            j = t.find('*/', i + 2)
            j = n if j < 0 else j + 2
            out.append(' ' * (j - i))
            i = j
        elif c == '/' and i + 1 < n and t[i + 1] == '/':
            j = t.find('\n', i)
            j = n if j < 0 else j
            out.append(' ' * (j - i))
            i = j
        else:
            out.append(c)
            i += 1
    t = ''.join(out)
    # #include "..." 是预处理指令; #error/#warning 是给编译器看的, 均不上屏
    t = re.sub(r'(?m)^[ \t]*#[ \t]*include\s+"[^"\n]*"', '', t)
    t = re.sub(r'(?m)^[ \t]*#[ \t]*(?:error|warning)\b[^\n]*', '', t)
    return t


ESC = {'n': '\n', 't': '\t', 'r': '\r', '0': '\0', '\\': '\\',
       "'": "'", '"': '"', 'a': '\a', 'b': '\b', 'f': '\f', 'v': '\v'}


def decode_chr(s):
    if s.startswith('\\'):
        return ESC.get(s[1], '\x01')
    return s


def decode_str(s):
    """还原 C 字符串字面量里的转义序列, 再统计可达字符.
    不还原时 "\\n" 会被当成 '\\' 与 'n' 两个字符 —— 反斜杠(0x5C)不在字库,
    随即报出假 MISSING; 还原后换行/回车等控制字符落在 0x20 以下被自然排除."""
    out = []
    i, n = 0, len(s)
    while i < n:
        c = s[i]
        if c == '\\' and i + 1 < n:
            nx = s[i + 1]
            if nx == 'x':                       # \xNN 十六进制
                j, hx = i + 2, ''
                while j < n and len(hx) < 2 and s[j] in '0123456789abcdefABCDEF':
                    hx += s[j]
                    j += 1
                out.append(chr(int(hx, 16)) if hx else '\\')
                i = j
            elif nx in '01234567':              # \ooo 八进制
                j, oc = i + 1, ''
                while j < n and len(oc) < 3 and s[j] in '01234567':
                    oc += s[j]
                    j += 1
                out.append(chr(int(oc, 8) & 0xFF))
                i = j
            else:
                out.append(ESC.get(nx, nx))
                i += 2
        else:
            out.append(c)
            i += 1
    return ''.join(out)


def font_tables(src):
    """解析两张紧凑表, 返回 {name: (compiled_set, all_rows, width)}."""
    # 行数常量必须由定义处 sizeof 导出(头文件里数组外层维度未知, sizeof 会报
    # "incomplete type"; 写成字面量又会与 G8 开关脱钩). 用法同 OLED_CF16x16_COUNT.
    for name in ('OLED_F8x16', 'OLED_F6x8'):
        if not re.search(r"const\s+uint8_t\s+%s_COUNT\s*=\s*\(uint8_t\)\s*\(\s*sizeof\(\s*%s\s*\)"
                         r"\s*/\s*sizeof\(\s*%s\s*\[\s*0\s*\]\s*\)\s*\)" % (name, name, name), src):
            sys.exit('FAIL: 找不到 %s_COUNT 的 sizeof 导出(线性查找的上界会缺失)' % name)
    tables = {}
    for name, width in (('OLED_F8x16', 17), ('OLED_F6x8', 7)):
        m = re.search(r"const\s+uint8_t\s+%s\s*\[\s*\]\s*\[\s*%d\s*\]\s*=\s*\{(.*?)\n\};"
                      % (name, width), src, re.S)
        if not m:
            sys.exit('FAIL: %s[][%d] 表没找到 —— oled_data.c 不是紧凑布局?' % (name, width))
        rows = ROW_RE.findall(m.group(1))
        if len(rows) != 95:
            sys.exit('FAIL: %s 解析出 %d 行 G8(...), 应为 95' % (name, len(rows)))
        chars, compiled, seen = [], set(), set()
        for i, (flag, ch, data) in enumerate(rows):
            ch = decode_chr(ch)
            if ch != chr(0x20 + i):
                sys.exit('FAIL: %s 第 %d 行字符是 %r, 应为 %r(行序须与旧 ch-\' \' 索引一致)'
                         % (name, i, ch, chr(0x20 + i)))
            if ch in seen:
                sys.exit('FAIL: %s 字符 %r 重复' % (name, ch))
            seen.add(ch)
            nbytes = len(re.findall(r'0x[0-9A-Fa-f]{2}', data))
            if nbytes != width - 1:
                sys.exit('FAIL: %s 行 %r 有 %d 个字形字节, 应为 %d' % (name, ch, nbytes, width - 1))
            chars.append(ch)
            if flag == '1':
                compiled.add(ch)
        if '?' not in compiled:
            sys.exit("FAIL: %s 的 '?' 兜底字形 flag=0, 未命中时无处可退" % name)
        tables[name] = (compiled, chars, width)
    return tables


def reachable_chars():
    """从项目 .c 源码推导运行期可达字符(字符串 + 字符字面量 + 运行期取值域)."""
    found = {}

    def add(ch, tag):
        if 0x20 <= ord(ch) <= 0x7e and ch not in NON_DISPLAY:
            found.setdefault(ch, set()).add(tag)

    for d in SRC_DIRS:
        p = os.path.join(ROOT, d)
        if not os.path.isdir(p):
            continue
        # 递归扫描: 源码可能放在子目录(如 ota/app_side), 只 listdir 会漏扫而漏报.
        for dirpath, _dirs, files in os.walk(p):
            for fn in sorted(files):
                if not fn.endswith('.c'):
                    continue
                rel = os.path.relpath(os.path.join(dirpath, fn), ROOT)
                if rel in SKIP_FILES:          # 字库本体 / 非上屏的 boot 调试源
                    continue
                src = strip_noise(read_bom(os.path.join(dirpath, fn)))
                for m in STR_RE.finditer(src):
                    for ch in decode_str(m.group(1)):
                        add(ch, rel)
                for m in CHR_RE.finditer(src):
                    add(decode_chr(m.group(1)), rel)
    for ch in RUNTIME_ONLY:
        found.setdefault(ch, set()).add('<runtime>')
    return found


def main():
    src = read_bom(FONT)
    tables = font_tables(src)

    c16 = tables['OLED_F8x16'][0]
    c6 = tables['OLED_F6x8'][0]
    if c16 != c6:
        only16 = ''.join(sorted(c16 - c6))
        only6 = ''.join(sorted(c6 - c16))
        sys.exit('FAIL: 两表编入字符集不一致: 仅 8x16 有 %r, 仅 6x8 有 %r\n'
                 '      (并集策略要求两表一致, 否则文案换字体会缺字)' % (only16, only6))
    compiled = c16

    reach = reachable_chars()
    missing = sorted(set(reach) - compiled)
    over = sorted(compiled - set(reach))

    old_bytes = 95 * 16 + 95 * 6
    new_bytes = len(compiled) * 17 + len(compiled) * 7
    dropped = ''.join(sorted(chr(c) for c in range(0x20, 0x7F) if chr(c) not in compiled))

    print('compiled=%d/95 per table: %s' % (len(compiled), ''.join(sorted(compiled))))
    print('reachable=%d  dropped=%d: %s' % (len(reach), 95 - len(compiled), dropped))
    print('flash: ASCII %d -> %d B (-%d B)'
          % (old_bytes, new_bytes, old_bytes - new_bytes))
    if over:
        print('NOTE: 编入但不可达(可考虑改 0): %s' % ''.join(over))
    if missing:
        for ch in missing:
            locs = ','.join(sorted(reach[ch]))
            print('MISSING %r <- %s' % (ch, locs))
        sys.exit('FAIL: %d 个可达字符被剔除, 会上屏显示为问号\n'
                 '       处理: bsp/oled_data.c 把对应行 flag 0 改 1, 或改文案' % len(missing))
    print('OK: 所有可达字符均已编入, 无剔除字符会上屏')


if __name__ == '__main__':
    main()
