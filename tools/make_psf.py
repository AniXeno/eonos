#!/usr/bin/env python3
"""Render a bitmap-style TTF/OTF (e.g. GNU Unifont) into a PSF2 console font.

usage: make_psf.py FONT.(ttf|otf) OUT.psf [height=16] [width=8] [baseline=14]

Glyph N of the PSF is the Unicode codepoint N (0..255), which is what the
kernel's console expects. Only pixel-exact bitmap fonts look good this way.
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
    if 32 <= cp < 127 or cp >= 161:
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

# PSF2 header: magic, version, headersize, flags, length, charsize, height, width
hdr = struct.pack("<IIIIIIII", 0x864AB572, 0, 32, 0, NUM, charsize, H, W)
open(out, "wb").write(hdr + glyphs)
print(f"wrote {out}: PSF2 {W}x{H}, {NUM} glyphs, {len(hdr)+len(glyphs)} bytes")
