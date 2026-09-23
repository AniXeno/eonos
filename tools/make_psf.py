#!/usr/bin/env python3
"""Render a bitmap-style TTF/OTF (e.g. GNU Unifont) into a PSF2 console font.

usage: make_psf.py FONT.(ttf|otf) OUT.psf [height=16] [width=8] [baseline=14]

The first 256 glyph slots map directly to U+0000..U+00FF. Dedicated 8x16
box-drawing glyphs are appended for the console's lsblk tree characters.
Only pixel-exact bitmap fonts look good this way.
"""
import struct, sys
from PIL import Image, ImageDraw, ImageFont

src, out = sys.argv[1], sys.argv[2]
H = int(sys.argv[3]) if len(sys.argv) > 3 else 16
W = int(sys.argv[4]) if len(sys.argv) > 4 else 8
NUM = 256
bpr = (W + 7) // 8            # bytes per row
charsize = bpr * H

S = 8                                   # oversample factor
font = ImageFont.truetype(src, H * S)
BASE = int(sys.argv[5]) if len(sys.argv) > 5 else 14   # baseline row (Unifont: 14 of 16)
glyphs = bytearray()
for cp in range(NUM):
    big = Image.new("L", (W * S, H * S), 0)
    if cp >= 32:
        ImageDraw.Draw(big).text((0, BASE * S), chr(cp), font=font, fill=255, anchor="ls")
    bp = big.load()
    class _P:                                   # sample the centre of each S x S cell
        def __getitem__(self, xy):
            return bp[xy[0] * S + S // 2, xy[1] * S + S // 2] > 127
    px = _P()
    for y in range(H):
        for bx in range(bpr):
            b = 0
            for bit in range(8):
                x = bx * 8 + bit
                if x < W and px[x, y]:
                    b |= 0x80 >> bit          # MSB = leftmost pixel (PSF rule)
            glyphs.append(b)

# Append stable 8x16 line art so box drawing does not depend on the source
# font containing Unicode box characters or on its glyph width metrics.
extra_glyphs = {
    "─": [0x00] * 7 + [0xff] + [0x00] * 8,
    "│": [0x08] * H,
    "├": [0x08] * 7 + [0xff] + [0x08] * 8,
    "└": [0x08] * 7 + [0xff] + [0x00] * 8,
}
if W != 8 or H != 16:
    raise SystemExit("appended box-drawing glyphs currently require an 8x16 font")
for rows in extra_glyphs.values():
    glyphs.extend(rows)

# PSF2 Unicode table: one UTF-8 character per glyph, terminated by 0xFF.
unicode_table = bytearray()
for cp in range(NUM):
    if cp >= 32:
        unicode_table.extend(chr(cp).encode("utf-8"))
    unicode_table.append(0xff)
for char in extra_glyphs:
    unicode_table.extend(char.encode("utf-8"))
    unicode_table.append(0xff)

# PSF2 header: magic, version, headersize, flags, length, charsize, height, width
count = NUM + len(extra_glyphs)
hdr = struct.pack("<IIIIIIII", 0x864AB572, 0, 32, 1, count, charsize, H, W)
open(out, "wb").write(hdr + glyphs + unicode_table)
print(f"wrote {out}: PSF2 {W}x{H}, {count} glyphs + Unicode table, {len(hdr)+len(glyphs)+len(unicode_table)} bytes")
