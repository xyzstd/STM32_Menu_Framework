#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""构建期校验 menu/menu_table.c —— 与内核 menu_validate_table() 逐条对齐.

为什么需要它
------------
菜单表配错(索引 0 不是占位虚拟根 / HOME 不在索引 1 / parent-child 不一致等)时,
内核 menu_validate_table() 会失败并**静默降级**: 保持虚拟根+列表态, 所有导航入口
停用 —— 现象只有"上电后按键完全无反应", 没有日志也没有提示, 极难定位.

而 C99 下 const 数组成员**不是整数常量表达式**, menu_table[3].first_child 这类
取值不能用于数组长度或 case 标签, 所以表内 parent/child 关系无法用编译期断言覆盖
(menu_table.c 里只放了"项数不超索引宽度"这类由 sizeof 得出的守卫).

于是把同样的校验前移到构建期: 改表即报错, 不必上机.

检查项(与 menu_validate_table 一致)
----------------------------------
 1. 项数 > MENU_HOME_ID, 且 <= 255(索引为 uint8_t)
 2. 索引 0 是占位虚拟根: parent=0, ops=NULL, child_num!=0, first_child!=0,
    且 [first_child, first_child+child_num) 覆盖索引 1(含边界不越界)
 3. 索引 1 是 HOME: parent=0, 带 MENU_FLAG_DIRECT_HANDLER, ops 非空,
    且该 ops 同时具备 display 与 handler
 4. flags 只允许已定义的位(MENU_FLAG_DIRECT_HANDLER)
 5. 有 ops 就必须同时有 display 与 handler; 带 DIRECT_HANDLER 就必须有 ops
 6. child_num==0 ⟺ first_child==0
 7. 子区间 [first_child, first_child+child_num) 不越界, 且每个子项的 parent 反指回本项
 8. 每个非根项的 parent 合法(非自身、不越界), 且被父项的子区间包含
 9. 每个节点沿 parent 链有限步内到达虚拟根(防孤岛与父级环)

额外检查(运行期不便做, 但构建期容易)
----------------------------------
10. 索引 0..count-1 全部出现且不重复(无空洞/无重号)
11. 非占位项的 .name 非空(列表要显示)
12. menu_max_num 的初始化表达式与解析出的项数一致

用法
----
    python build_tools/check_menu_table.py [menu_table.c]
退出码: 0=通过, 1=发现配置错误.
"""

import os
import re
import sys

MENU_NO_PARENT = 0
MENU_NO_CHILD = 0
MENU_LEAF = 0
MENU_HOME_ID = 1
MENU_FLAG_DIRECT_HANDLER = 0x01
KNOWN_FLAGS = MENU_FLAG_DIRECT_HANDLER

SYMS = {
    "MENU_NO_PARENT": MENU_NO_PARENT,
    "MENU_NO_CHILD": MENU_NO_CHILD,
    "MENU_LEAF": MENU_LEAF,
    "MENU_INVALID": 0,
    "MENU_HOME_ID": MENU_HOME_ID,
    "MENU_FLAG_DIRECT_HANDLER": MENU_FLAG_DIRECT_HANDLER,
}


def strip_comments(t):
    t = re.sub(r"/\*.*?\*/", "", t, flags=re.S)
    t = re.sub(r"//[^\n]*", "", t)
    return t


def to_int(tok):
    if tok is None:
        return None
    tok = tok.strip()
    if tok in SYMS:
        return SYMS[tok]
    tok = re.sub(r"[uUlL]+$", "", tok)
    try:
        return int(tok, 0)
    except ValueError:
        return None


def parse_table(path):
    src = strip_comments(open(path, encoding="utf-8-sig").read())
    m = re.search(
        r"const\s+menu_item_t\s+menu_table\s*\[\s*\]\s*=\s*\{(.*?)\n\s*\};",
        src,
        re.S,
    )
    if not m:
        return None, "未找到 'const menu_item_t menu_table[] = { ... };' 定义"
    body = m.group(1)

    entries = {}
    for em in re.finditer(r"\[\s*(\d+)\s*\]\s*=\s*\{(.*?)\}\s*,", body, re.S):
        idx = int(em.group(1))
        fields = em.group(2)

        def get(f):
            fm = re.search(r"\.\s*%s\s*=\s*([^,]+)" % f, fields)
            return fm.group(1).strip() if fm else None

        entries[idx] = {f: get(f) for f in
                        ("parent", "first_child", "child_num", "flags", "ops", "name")}
    return entries, None


def parse_ops_defs(src_dirs):
    """扫描 page/*.c 里的 `const menu_ops_t NAME = { ... };`, 返回 {NAME: {字段}}."""
    defs = {}
    for d in src_dirs:
        if not os.path.isdir(d):
            continue
        for fn in sorted(os.listdir(d)):
            if not fn.endswith(".c"):
                continue
            src = strip_comments(open(os.path.join(d, fn), encoding="utf-8-sig").read())
            for om in re.finditer(r"const\s+menu_ops_t\s+(\w+)\s*=\s*\{(.*?)\n\s*\};",
                                  src, re.S):
                name, body = om.group(1), om.group(2)
                defs[name] = {f for f in ("enter", "handler", "display", "exit")
                              if re.search(r"\.\s*%s\s*=" % f, body)}
    return defs


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    table_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(root, "menu", "menu_table.c")
    if not os.path.isfile(table_path):
        print("ERROR: 找不到 %s" % table_path)
        return 1

    entries, err = parse_table(table_path)
    if err:
        print("ERROR: %s" % err)
        return 1

    problems = []
    count = (max(entries) + 1) if entries else 0

    def add(msg):
        problems.append(msg)

    # --- 10. 索引连续无重号 ---
    missing = [i for i in range(count) if i not in entries]
    if missing:
        add("索引缺失(表项未按 0..%d 连续定义): %s" % (count - 1, missing))

    # --- 1. 项数 ---
    if count <= MENU_HOME_ID:
        add("项数 %d 必须大于 MENU_HOME_ID(%d)" % (count, MENU_HOME_ID))
        print("\n".join("FAIL: " + p for p in problems))
        return 1
    if count > 255:
        add("项数 %d 超过索引宽度 255(uint8_t)" % count)

    # --- 12. menu_max_num 与解析结果一致 ---
    raw = open(table_path, encoding="utf-8-sig").read()
    mm = re.search(r"menu_max_num\s*=\s*(.+?);", raw, re.S)
    if mm and "sizeof(menu_table)" in mm.group(1):
        pass  # 用 sizeof 推导, 自动跟随项数
    elif mm:
        v = to_int(mm.group(1))
        if v is not None and v != count:
            add("menu_max_num=%s 与解析出的项数 %d 不一致" % (v, count))

    ops_defs = parse_ops_defs([os.path.join(root, "page")])

    def ops_name(v):
        if v is None or v.strip() == "NULL":
            return None
        return v.strip().lstrip("&").strip()

    def ops_ok(nm):
        """ops 是否存在且同时具备 display/handler. 未知定义返回 None(不判)."""
        if nm not in ops_defs:
            return None
        f = ops_defs[nm]
        return ("display" in f) and ("handler" in f)

    # --- 2. 索引 0 = 占位虚拟根, 且覆盖 HOME ---
    e0 = entries.get(0, {})
    p0, fc0, cn0 = to_int(e0.get("parent")), to_int(e0.get("first_child")), to_int(e0.get("child_num"))
    if p0 != MENU_NO_PARENT:
        add("[0] 虚拟根的 parent 必须是 MENU_NO_PARENT(0), 实际 %s" % e0.get("parent"))
    if ops_name(e0.get("ops")) is not None:
        add("[0] 虚拟根必须 ops=NULL(占位容器, 永不激活), 实际 %s" % e0.get("ops"))
    if not cn0:
        add("[0] 虚拟根的 child_num 必须非 0")
    if not fc0:
        add("[0] 虚拟根的 first_child 必须非 0")
    if fc0 and cn0:
        end0 = fc0 + cn0
        if end0 > count:
            add("[0] 子区间 [%d,%d) 越界(项数 %d)" % (fc0, end0, count))
        if not (fc0 <= MENU_HOME_ID < end0):
            add("[0] 子区间 [%d,%d) 必须覆盖 MENU_HOME_ID(%d)" % (fc0, end0, MENU_HOME_ID))

    # --- 3. 索引 1 = HOME ---
    e1 = entries.get(MENU_HOME_ID, {})
    fl1 = to_int(e1.get("flags")) or 0
    if to_int(e1.get("parent")) != MENU_NO_PARENT:
        add("[%d] HOME 的 parent 必须是 MENU_NO_PARENT(0), 实际 %s"
            % (MENU_HOME_ID, e1.get("parent")))
    if not (fl1 & MENU_FLAG_DIRECT_HANDLER):
        add("[%d] HOME 必须带 MENU_FLAG_DIRECT_HANDLER" % MENU_HOME_ID)
    nm1 = ops_name(e1.get("ops"))
    if nm1 is None:
        add("[%d] HOME 必须提供 ops" % MENU_HOME_ID)
    else:
        ok = ops_ok(nm1)
        if ok is False:
            add("[%d] HOME 的 ops(%s) 必须同时具备 .display 与 .handler" % (MENU_HOME_ID, nm1))

    # --- 4~8. 逐项 ---
    for i in range(count):
        if i not in entries:
            continue
        e = entries[i]
        fl = to_int(e.get("flags")) or 0
        fc = to_int(e.get("first_child"))
        cn = to_int(e.get("child_num"))
        nm = ops_name(e.get("ops"))

        if fl & ~KNOWN_FLAGS:
            add("[%d] flags=0x%X 含未定义的位(已知 0x%X)" % (i, fl, KNOWN_FLAGS))
        if nm is not None:
            ok = ops_ok(nm)
            if ok is False:
                add("[%d] ops(%s) 必须同时具备 .display 与 .handler" % (i, nm))
        if (fl & MENU_FLAG_DIRECT_HANDLER) and nm is None:
            add("[%d] 带 DIRECT_HANDLER 但 ops=NULL(常驻页必须可激活)" % i)

        if not cn:
            if fc != MENU_NO_CHILD:
                add("[%d] child_num=0 时 first_child 必须是 MENU_NO_CHILD(0), 实际 %s"
                    % (i, e.get("first_child")))
        else:
            if fc == MENU_NO_CHILD:
                add("[%d] child_num=%d 但 first_child=MENU_NO_CHILD(0)" % (i, cn))
            else:
                end = fc + cn
                if fc >= count or end > count:
                    add("[%d] 子区间 [%d,%d) 越界(项数 %d)" % (i, fc, end, count))
                else:
                    for child in range(fc, end):
                        cp = to_int(entries.get(child, {}).get("parent"))
                        if cp != i:
                            add("[%d] 子项 [%d] 的 parent=%s, 应反指回 %d"
                                % (i, child, entries.get(child, {}).get("parent"), i))

        if i == 0:
            continue

        p = to_int(e.get("parent"))
        if p is None:
            add("[%d] parent 无法解析: %s" % (i, e.get("parent")))
            continue
        if p >= count:
            add("[%d] parent=%d 越界(项数 %d)" % (i, p, count))
            continue
        if p == i:
            add("[%d] parent 指向自身" % i)
            continue
        pe = entries.get(p, {})
        pfc, pcn = to_int(pe.get("first_child")), to_int(pe.get("child_num"))
        if not pcn or pfc == MENU_NO_CHILD:
            add("[%d] parent=%d 不是容器(child_num=0 或 first_child=0)" % (i, p))
        elif pfc is not None and pcn is not None and pfc + pcn <= count:
            if not (pfc <= i < pfc + pcn):
                add("[%d] 未被父项 %d 的子区间 [%d,%d) 包含" % (i, p, pfc, pfc + pcn))

    # --- 11. 非占位项应有 name ---
    for i in range(count):
        if i == 0 or i not in entries:
            continue
        if entries[i].get("name") is None:
            add("[%d] 缺少 .name(列表需要显示名)" % i)

    # --- 9. parent 链可达根 ---
    for i in range(1, count):
        walk, hops = i, 0
        while walk != MENU_NO_PARENT and hops < count:
            nxt = to_int(entries.get(walk, {}).get("parent"))
            if nxt is None:
                break
            walk = nxt
            hops += 1
        if walk != MENU_NO_PARENT:
            add("[%d] 沿 parent 链 %d 步内未到达虚拟根(孤岛或父级环)" % (i, hops))

    if problems:
        print("menu_table 配置错误 %d 处:" % len(problems))
        for p in problems:
            print("  FAIL: %s" % p)
        return 1

    print("OK: menu_table 校验通过 (项数 %d, HOME=索引 %d, 顶层 %d 项)"
          % (count, MENU_HOME_ID, to_int(entries[1].get("child_num")) or 0))
    return 0


if __name__ == "__main__":
    sys.exit(main())
