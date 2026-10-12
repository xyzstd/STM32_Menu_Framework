# -*- coding: utf-8 -*-
"""判断 Boost 功能是否已从工程中彻底移除.

"干净"的可执行定义 —— 三条断言全部成立才算通过:

  A. 已删除的符号不得再出现在**代码**里(声明/定义/调用都算):
       BOOST_* 宏、boost_advance/boost_reset/boost_target/boost_enabled/
       boost_cooldown/boost_sag/boost_rt/boost_offset、last_setpoint、
       effective_setpoint、Config_GetBoostMax、
       PID_Controller_SetBoostOffset/GetBoostOffset/GetEffectiveSetpoint、
       WorkTask_IsBoostActive
     (注释里提到这些名字属文档, 不算残留 —— 由 B 判断其是否已注明"已移除".)
  B. 代码或注释里出现 "boost" 字样时, 必须属于**白名单**之一:
       1) bsp/config.h 的 config_v10_t / config_v12_t 结构体内部 —— 旧 Flash 布局
          快照, 必须保留 boost_max[] 才能正确读取并迁移旧数据;
       2) 注释中显式说明"移除/删除"—— 即记录本次移除本身.
     其余一律视为残留.
  C. 显示侧不得出现 Boost 相关中文文案(只查字符串字面量, 不查注释).

判据要点(踩过的坑): C 语言标识符区分大小写, 故符号匹配必须区分大小写 ——
否则 BOOST_ 会误伤迁移结构里的 boost_max; 中文文案只可能出现在字符串字面量里,
注释中的"大焊点"属正常描述(前馈加速回温), 不算显示残留.

用法: python build_tools/check_boost_removed.py   (退出码 0=干净, 1=有残留)
"""
import os, re, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DIRS = ['bsp', 'ctrl', 'main', 'menu', 'page', 'sys', 'ota']
EXTS = ('.c', '.h')

# A. 必须彻底消失的符号(区分大小写)
DEAD_SYMS = [
    'BOOST_',
    'boost_advance', 'boost_reset', 'boost_target', 'boost_enabled',
    'boost_cooldown', 'boost_sag', 'boost_rt', 'boost_offset',
    'last_setpoint', 'effective_setpoint',
    'Config_GetBoostMax',
    'PID_Controller_SetBoostOffset', 'PID_Controller_GetBoostOffset',
    'PID_Controller_GetEffectiveSetpoint',
    'WorkTask_IsBoostActive',
]

# C. 显示侧中文关键字
UI_WORDS = ['大焊点', '抬升']


def read_bom(path):
    with open(path, 'rb') as f:
        b = f.read()
    if b.startswith(b'\xef\xbb\xbf'):
        b = b[3:]
    return b.decode('utf-8', 'replace')


def split_line(line):
    """把一行拆成 (代码部分, 注释部分). 整行注释返回 ('', 整行)."""
    s = line.strip()
    if s.startswith('*') or s.startswith('/*') or s.startswith('//'):
        return '', line
    code, comment = line, ''
    p = code.find('/*')
    if p >= 0:
        comment, code = code[p:], code[:p]
    q = code.find('//')
    if q >= 0:
        comment, code = code[q:] + comment, code[:q]
    return code, comment


def config_struct_ranges(lines):
    """bsp/config.h 中 config_v10_t / config_v12_t 结构体占用的行号集合(0 基)."""
    allowed = set()
    for tag in ('}config_v10_t;', '}config_v12_t;'):
        end = next((i for i, l in enumerate(lines) if l.strip().startswith(tag)), None)
        if end is None:
            continue
        start = max(i for i in range(end) if lines[i].strip().startswith('typedef struct'))
        allowed.update(range(start, end + 1))
    return allowed


def mentions_removal(comment):
    return ('移除' in comment) or ('删除' in comment)


def main():
    bad = []

    for d in DIRS:
        p = os.path.join(ROOT, d)
        if not os.path.isdir(p):
            continue
        for fn in sorted(os.listdir(p)):
            if not fn.endswith(EXTS):
                continue
            path = os.path.join(p, fn)
            rel = os.path.relpath(path, ROOT).replace('\\', '/')
            lines = read_bom(path).split('\n')
            allowed_lines = config_struct_ranges(lines) if rel == 'bsp/config.h' else set()

            for i, line in enumerate(lines):
                code, comment = split_line(line)

                # A. 死符号只能出现在注释里
                for sym in DEAD_SYMS:
                    if sym in code:
                        bad.append('A %s:%d 代码残留符号 %s | %s'
                                   % (rel, i + 1, sym, code.strip()[:70]))

                # B. boost 字样白名单
                if 'boost' in code.lower() and i not in allowed_lines:
                    bad.append('B %s:%d 代码中 boost 引用(非旧布局快照) | %s'
                               % (rel, i + 1, code.strip()[:70]))
                elif 'boost' in comment.lower() and not mentions_removal(comment):
                    bad.append('B %s:%d 注释提及 boost 但未说明已移除 | %s'
                               % (rel, i + 1, comment.strip()[:70]))

                # C. 显示侧文案: 只看字符串字面量
                for lit in re.findall(r'"((?:[^"\\]|\\.)*)"', code):
                    for w in UI_WORDS:
                        if w in lit:
                            bad.append('C %s:%d 显示文案 "%s" | "%s"'
                                       % (rel, i + 1, w, lit[:50]))

    if bad:
        print('FAIL: 发现 %d 处残留' % len(bad))
        for b in bad:
            print('  ' + b)
        return 1

    print('PASS: Boost 已彻底移除')
    print('  A 代码中死符号 0 处 / B 非白名单 boost 引用 0 处 / C 显示文案 0 处')
    return 0


if __name__ == '__main__':
    sys.exit(main())
