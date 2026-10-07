# -*- coding: utf-8 -*-
# 批量生成 OLED_CF16x16 缺字点阵 (SimSun 14pt, offy=1, offx=0, 与现库风格一致)
# 输出: 与 oled_data.c 数组格式一致的 C 片段
import io, sys
from PIL import Image, ImageDraw, ImageFont

FONT = r"C:\Windows\Fonts\simsun.ttc"
SIZE = 14
OFFY = 1
OFFX = 0
TH   = 128

# 各页面中文化所需新增汉字 (已核对现库, 以下均为缺字)
CHARS = list("确到存")

def gen(char):
    img = Image.new("L", (64, 64), 0)
    d = ImageDraw.Draw(img)
    f = ImageFont.truetype(FONT, SIZE)
    d.text((8, 8), char, font=f, fill=255)
    bbox = img.getbbox()
    if not bbox:
        return None
    w, h = bbox[2] - bbox[0], bbox[3] - bbox[1]
    top  = bbox[1] - OFFY
    left = bbox[0] + OFFX - (16 - w) // 2
    out = []
    for col in range(16):
        b0 = b1 = 0
        for row in range(8):
            x, y = left + col, top + row
            if 0 <= x < 64 and 0 <= y < 64 and img.getpixel((x, y)) >= TH:
                b0 |= 1 << (7 - row)
        for row in range(8, 16):
            x, y = left + col, top + row
            if 0 <= x < 64 and 0 <= y < 64 and img.getpixel((x, y)) >= TH:
                b1 |= 1 << (7 - (row - 8))
        out.append(b0)
    for col in range(16):
        b0 = 0
        for row in range(8):
            x, y = left + col, top + 8 + row
            if 0 <= x < 64 and 0 <= y < 64 and img.getpixel((x, y)) >= TH:
                b0 |= 1 << (7 - row)
        out.append(b0)
    return out

# 参考"温"字 (现库) 前 32 字节, 验证生成参数一致性
REF_WEN = [0x00,0x04,0x22,0x10,0x00,0x00,0x7F,0x49,0x49,0x49,0x49,0x7F,0x00,0x00,0x00,0x00,
           0x00,0x0C,0x30,0xC0,0x02,0x7E,0x42,0x7E,0x42,0x42,0x7E,0x42,0x7E,0x02,0x00,0x00]
g = gen("\u6e29")  # 温
assert g == REF_WEN, "SimSun 参数与现库不一致!"

lines = []
for ch in CHARS:
    g = gen(ch)
    assert g is not None, "无法渲染: %s" % ch
    hexs = ",".join("0x%02X" % b for b in g)
    lines.append("        \"%s\",\n    %s," % (ch, hexs))

out = "\n".join(lines) + "\n"
with io.open(r"c:\Users\xyzstd\Desktop\menu\soldering_iron\build_tools\new_fonts.inc", "w", encoding="utf-8") as f:
    f.write(out)
sys.stdout.write("OK chars=%d bytes=%d\n" % (len(CHARS), len(out)))
