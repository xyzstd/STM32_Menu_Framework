# -*- coding: utf-8 -*-
"""小号中文提示 -> 8x16 英文提示, 并移除 12x16 小字库机制 (OLED_ShowChineseSmall / OLED_CF12x16).
字节级替换, 保持各文件 BOM 与既有编码不变. 每个替换都断言唯一匹配. 纯 ASCII 注释.
"""
import os, io

ROOT = r"c:\Users\xyzstd\Desktop\menu\soldering_iron"

def rd(p):
    return open(p, "rb").read()

def wr(p, b):
    open(p, "wb").write(b)

def rep(data, old, new, label):
    o, n = old.encode("utf-8"), new.encode("utf-8")
    c = data.count(o)
    if c == 0:
        assert data.count(n) == 1, "%s: old absent and new not applied" % label
        return data
    assert c == 1, "%s: expect 1 match, got %d" % (label, c)
    return data.replace(o, n, 1)

def ensure_bom(b):
    return b if b.startswith(b"\xef\xbb\xbf") else b"\xef\xbb\xbf" + b

def trim_tail(b):
    return b.rstrip(b"\n") + b"\n"

# ---------------- handle.c ----------------
p = os.path.join(ROOT, "page", "handle.c")
d = rd(p)
d = rep(d,
    '    /* 底部提示: 确认/长按切换 (小号 12x16, 80px 居中 x24..104, y48) */\n    OLED_ShowChineseSmall(24, 48, "确认/长按切换");',
    '    /* 底部提示: OK/HOLD Switch (8x16, 112px 居中 x8..120, y48) */\n    OLED_ShowString(8, 48, "OK/HOLD Switch", OLED_8X16);',
    "handle.c bottom hint")
wr(p, ensure_bom(d))

# ---------------- cur_cal.c ----------------
p = os.path.join(ROOT, "page", "cur_cal.c")
d = rd(p)
d = rep(d,
    'OLED_ShowChineseSmall(8, 48, "长按跳过");   /* 小号提示 */',
    'OLED_ShowString(8, 48, "HOLD Skip", OLED_8X16);   /* 英文提示 */',
    "cur_cal.c cooling hint")
d = rep(d,
    '    /* 第3行: 操作提示 (引导模式 LPRESS=跳过标定)\n     * 保存成功后短暂显示"已保存" (右对齐 x92..128, 3 字 36px) */',
    '    /* 第3行: 操作提示 (引导模式 LPRESS=跳过标定)\n     * 保存成功后短暂显示 "Saved" (右对齐 x88..128) */',
    "cur_cal.c row3 comment")
d = rep(d,
    'OLED_ShowChineseSmall(92, 48, "已保存");',
    'OLED_ShowString(88, 48, "Saved", OLED_8X16);',
    "cur_cal.c saved feedback")
d = rep(d,
    'OLED_ShowChineseSmall(8, 48, "保存");',
    'OLED_ShowString(8, 48, "SAVE", OLED_8X16);',
    "cur_cal.c save hint")
d = rep(d,
    'OLED_ShowChineseSmall(80, 48, cal_boot ? "长按跳过" : "长按退出");',
    'OLED_ShowString(48, 48, cal_boot ? "HOLD Skip" : "HOLD Exit", OLED_8X16);',
    "cur_cal.c skip/exit hint")
wr(p, ensure_bom(d))

# ---------------- eval_page.c ----------------
p = os.path.join(ROOT, "page", "eval_page.c")
d = rd(p)
d = rep(d,
    'OLED_ShowChineseSmall(40, 48, "点击开始");   /* 小号 4 字 48px 居中 */',
    'OLED_ShowString(20, 48, "CLICK Start", OLED_8X16);   /* 8x16 11 字 88px 居中 */',
    "eval_page.c click start")
wr(p, ensure_bom(d))

# ---------------- oled.c: 删除 OLED_ShowChineseSmall 实现段 ----------------
p = os.path.join(ROOT, "bsp", "oled.c")
d = rd(p)
m = d.find(b"/*================ 12x16 Small Chinese =================*/")
if m > 0:
    assert d.count(b"12x16 Small Chinese") == 1, "oled.c marker not unique"
    d = trim_tail(d[:m])
    wr(p, ensure_bom(d))
else:
    assert b"OLED_ShowChineseSmall" not in d, "oled.c marker gone but function still present"
    print("oled.c already cleaned")

# ---------------- oled.h: 删除原型 (保留文件尾 #endif) ----------------
p = os.path.join(ROOT, "bsp", "oled.h")
d = rd(p)
m = d.find(b"/* 12x12 small Chinese (hint text) */")
if m > 0:
    assert d.count(b"OLED_ShowChineseSmall") == 1, "oled.h proto not unique"
    d = trim_tail(d[:m]) + b"\n\n#endif\n"
    wr(p, ensure_bom(d))
else:
    assert b"OLED_ShowChineseSmall" not in d, "oled.h marker gone but proto still present"
    if not d.rstrip(b"\n").endswith(b"#endif"):
        d = trim_tail(d) + b"\n\n#endif\n"
        wr(p, ensure_bom(d))
    print("oled.h already cleaned")

# ---------------- oled_data.h: 删除小字结构体与声明 (保留文件尾 #endif) ----------------
p = os.path.join(ROOT, "bsp", "oled_data.h")
d = rd(p)
m = d.find(b"/* Small Chinese font cell: 12x16 (24 bytes) */")
if m > 0:
    assert d.count(b"ChineseCell12_t") == 2, "oled_data.h cell refs not as expected"
    d = trim_tail(d[:m]) + b"\n\n#endif\n"
    wr(p, ensure_bom(d))
else:
    assert b"ChineseCell12_t" not in d, "oled_data.h marker gone but cell type still present"
    if not d.rstrip(b"\n").endswith(b"#endif"):
        d = trim_tail(d) + b"\n\n#endif\n"
        wr(p, ensure_bom(d))
    print("oled_data.h already cleaned")

# ---------------- oled_data.c: 删除 12x16 字表段 ----------------
p = os.path.join(ROOT, "bsp", "oled_data.c")
d = rd(p)
m = d.find(b"/*================ 12x16 Small Chinese Font ================*/")
if m > 0:
    d = trim_tail(d[:m])
    wr(p, ensure_bom(d))
else:
    assert b"OLED_CF12x16" not in d, "oled_data.c marker gone but table still present"
    print("oled_data.c already cleaned")

print("OK: hints switched to English, small-font machinery removed")
