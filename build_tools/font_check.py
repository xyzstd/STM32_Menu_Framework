# -*- coding: utf-8 -*-
# 验证 SimSun 栅格化参数与现库 16x16 点阵风格是否一致 (对比 "温" 字)
import sys, itertools
from PIL import Image, ImageDraw, ImageFont

# 现库 "温" (oled_data.c L400-401), MSB 在上, 32 字节
REF = [0x00,0x04,0x22,0x10,0x00,0x00,0x7F,0x49,0x49,0x49,0x49,0x7F,0x00,0x00,0x00,0x00,
       0x00,0x0C,0x30,0xC0,0x02,0x7E,0x42,0x7E,0x42,0x42,0x7E,0x42,0x7E,0x02,0x00,0x00]

def gen(char, path, size, offx, offy, thresh=128):
    # 大画布渲染再裁 16x16, 避免抗锯齿裁边
    img = Image.new("L", (64, 64), 0)
    d = ImageDraw.Draw(img)
    f = ImageFont.truetype(path, size)
    d.text((8, 8), char, font=f, fill=255)
    # 找文字包围盒
    bbox = img.getbbox()
    if not bbox:
        return None
    w, h = bbox[2]-bbox[0], bbox[3]-bbox[1]
    # 按 16x16 目标, offy 控制垂直偏移(负=上移), 顶部留 1px
    top = bbox[1] - offy
    left = bbox[0] + offx - (16-w)//2
    out = []
    for col in range(16):
        b0 = b1 = 0
        for row in range(8):
            x = left + col
            y = top + row
            if 0 <= x < 64 and 0 <= y < 64 and img.getpixel((x, y)) >= thresh:
                b0 |= 1 << (7-row)      # bit7 = 最上
        for row in range(8, 16):
            x = left + col
            y = top + row
            if 0 <= x < 64 and 0 <= y < 64 and img.getpixel((x, y)) >= thresh:
                b1 |= 1 << (7-(row-8))
        out.append(b0)
    for col in range(16):
        b0 = 0
        for row in range(8):
            x = left + col
            y = top + 8 + row
            if 0 <= x < 64 and 0 <= y < 64 and img.getpixel((x, y)) >= thresh:
                b0 |= 1 << (7-row)
        out.append(b0)
    return out

def diff(a, b):
    return sum(bin(x ^ y).count("1") for x, y in zip(a, b))

cands = []
for size, offy, offx, th in itertools.product([12, 13, 14, 15, 16], range(-2, 3), [0, 1], [100, 128, 160]):
    g = gen("温", r"C:\Windows\Fonts\simsun.ttc", size, offx, offy, th)
    if g:
        cands.append((diff(REF, g), size, offy, offx, th, g))

cands.sort(key=lambda t: t[0])
for c in cands[:8]:
    print("diff=%d size=%d offy=%d offx=%d th=%d" % (c[0], c[1], c[2], c[3], c[4]))
    print("  gen:", ",".join("0x%02X" % x for x in c[5]))
print("  ref:", ",".join("0x%02X" % x for x in REF))
