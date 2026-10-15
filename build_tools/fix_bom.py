# -*- coding: utf-8 -*-
"""检查/补回 UTF-8 BOM (EF BB BF).

为什么需要: ARM Compiler 5 只有在文件带 BOM 时才把源文件当 UTF-8 处理.
缺 BOM 时, 源文件里的**中文字符串字面量**会被按本地代码页解析, 编译报出的
却是 `#8: missing closing quote` 这类与真实原因毫无关系的错误 —— 排查成本很高.
(仅出现在注释里的中文不受影响, 所以这个问题常常只在一部分文件上暴露.)

本脚本扫描全部自研源码(排除 package/ 第三方库), 列出"含非 ASCII 但缺 BOM"的
文件; 带 --fix 参数则直接补上. 许多编辑器保存时会静默丢掉 BOM, 建议改完中文
文案后跑一次:

注意: 扫描范围含 docs/ 下的 .md (纯文档不需要 BOM 编译, 但为保持仓库统一编码,
同样纳入检查).

    python build_tools/fix_bom.py          # 只检查
    python build_tools/fix_bom.py --fix    # 检查并补回
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOM = b'\xef\xbb\xbf'

# 扫描这些目录下的源码/文档; package/ 是 ST 标准库, 保持原样不碰
SCAN_DIRS = ['bsp', 'ctrl', 'menu', 'page', 'sys', 'main', 'ota', 'docs']
SCAN_EXTS = ('.c', '.h', '.md')
SKIP_DIRS = {'package', 'Objects', 'Listings', 'obj', '_objs', '_b', 'build_tools'}


def iter_sources():
    for base, dirs, files in os.walk(ROOT):
        dirs[:] = [d for d in dirs if d not in SKIP_DIRS]
        rel = os.path.relpath(base, ROOT)
        top = rel.split(os.sep)[0]
        if top not in SCAN_DIRS:
            continue
        for fn in sorted(files):
            if fn.endswith(SCAN_EXTS):
                yield os.path.join(base, fn)


def main():
    fix = '--fix' in sys.argv
    missing, ok, ascii_only, fixed = [], [], [], 0

    for path in iter_sources():
        with open(path, 'rb') as f:
            raw = f.read()
        rel = os.path.relpath(path, ROOT)
        has_bom = raw.startswith(BOM)
        non_ascii = any(b > 0x7F for b in (raw[3:] if has_bom else raw))

        if not non_ascii:
            ascii_only.append(rel)
        elif has_bom:
            ok.append(rel)
        else:
            missing.append(rel)
            if fix:
                with open(path, 'wb') as f:
                    f.write(BOM + raw)
                fixed += 1

    print('含非 ASCII 且 BOM 正常 : %d' % len(ok))
    print('纯 ASCII (无需 BOM)   : %d' % len(ascii_only))
    print('含非 ASCII 但缺 BOM   : %d' % len(missing))
    for p in missing:
        print('   MISSING  ' + p)
    if fix and fixed:
        print('\n已补回 BOM: %d 个文件' % fixed)
    elif missing and not fix:
        print('\n用 --fix 补回.')
    return 1 if (missing and not fix) else 0


if __name__ == '__main__':
    sys.exit(main())
