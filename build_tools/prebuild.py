#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""构建前检查入口 —— 把散落的校验脚本收成一条命令.

为什么需要
----------
菜单内核已不保留运行期表校验(menu.c 里没有 menu_validate_table), 构建期脚本是
"表配错"的**唯一**防线; 但它不在编译流程里 —— 改表后忘记跑, 构建照样 0 Error /
0 Warning, 问题要到上机按键异常才暴露. 同理, 改文案忘裁字库会显示空白, 改源码
忘补 BOM 会让 AC5 按本地代码页解析中文注释.

用法
----
    python build_tools/prebuild.py            # 只检查
    python build_tools/prebuild.py --build    # 检查通过后全量编译两个 target

退出码: 0 = 全部通过; 1 = 有硬性失败(--build 时编译失败同样返回 1).

判据说明
--------
- 菜单表 / ASCII 字库 / BOM: 脚本自身返回码即结论.
- 汉字字库: check_chars.py 不设返回码, 故解析 `missing=N` 判失败; orphan 只提示.
- ROM 余量: **只提示不判失败** —— OTA 关闭时真实上限是链接器 IROM
  (0x08008000 + 0x36800 ≈ 218KB), 32KB 槽不再是阻塞项.

接进 Keil(可选): Project → Options for Target → User → Before Build,
Run #1 填 `python build_tools/prebuild.py`, 路径按本机 Python 调整.
"""
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOOLS = os.path.join(ROOT, "build_tools")
PY = sys.executable
UV4 = os.environ.get("UV4", r"D:/UV4/UV4.exe")
TARGETS = ("Release-HW", "Debug-Sim")


def run_script(name, *args):
    """跑一个检查脚本, 回显输出并返回 (返回码, 输出文本)."""
    path = os.path.join(TOOLS, name)
    if not os.path.isfile(path):
        print("  [SKIP] 找不到 %s" % name)
        return 0, ""
    p = subprocess.run([PY, path] + list(args), cwd=ROOT,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    out = p.stdout.decode("utf-8", "replace")
    print(out.rstrip())
    return p.returncode, out


def main():
    failed = []

    print("== 1/5 菜单表结构 ==")
    rc, _ = run_script("check_menu_table.py")
    if rc != 0:
        failed.append("菜单表")

    print("\n== 2/5 汉字字库 ==")
    rc, out = run_script("check_chars.py")
    m = re.search(r"missing=(\d+)", out)
    if rc != 0 or (m and int(m.group(1)) != 0):
        failed.append("汉字字库")

    print("\n== 3/5 ASCII 字库 ==")
    rc, _ = run_script("check_ascii_font.py")
    if rc != 0:
        failed.append("ASCII 字库")

    print("\n== 4/5 UTF-8 BOM ==")
    rc, _ = run_script("fix_bom.py")
    if rc != 0:
        failed.append("BOM(用 fix_bom.py --fix 补回)")

    print("\n== 5/5 ROM 余量(仅提示) ==")
    run_script("check_rom_size.py")

    if failed:
        print("\n检查失败: %s" % ", ".join(failed))
        return 1

    print("\n全部检查通过.")

    if "--build" in sys.argv:
        if not os.path.isfile(UV4):
            print("找不到 UV4: %s (可用环境变量 UV4 指定)" % UV4)
            return 1
        for t in TARGETS:
            log = os.path.join(ROOT, "build_%s.txt" % t)
            print("\n== 编译 %s ==" % t)
            rc = subprocess.run([UV4, "-r", "Project.uvprojx", "-t", t, "-j0", "-o", log],
                                cwd=ROOT).returncode
            tail = ""
            if os.path.isfile(log):
                with open(log, encoding="utf-8", errors="replace") as f:
                    tail = f.read()
            for line in tail.splitlines():
                if "Error(s)" in line or "Program Size" in line:
                    print("  " + line.strip())
            if rc != 0 or "0 Error(s)" not in tail:
                print("  [FAIL] %s 编译失败, 详见 %s" % (t, log))
                return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
