use core::fmt;
use alloc::vec;
use alloc::vec::Vec;
use crate::sync::IrqMutex;

use crate::framebuffer::FbInfo;

static FONT_DATA: &[u8] = include_bytes!("../font.psf");


const PALETTE: [u32; 16] = [
    0x001B1F27, 0x00E06C75, 0x0098C379, 0x00E5C07B,
    0x0061AFEF, 0x00C678DD, 0x0056B6C2, 0x00ABB2BF, 
    0x004B5263, 0x00FF7B86, 0x00B5E890, 0x00FFD98E, 
    0x0082C4FF, 0x00E09BFF, 0x007FDCE8, 0x00FFFFFF, 
];

const DEFAULT_FG: u32 = PALETTE[7];
const DEFAULT_BG: u32 = 0x00000000; 

pub static CONSOLE: IrqMutex<Option<Console>> = IrqMutex::new(None);

#[derive(Clone, Copy)]
struct Font {
    glyphs: &'static [u8],
    num_glyphs: usize,
    charsize: usize,
    width: usize,
    height: usize,
    bytes_per_row: usize,
}

impl Font {
    fn parse(data: &'static [u8]) -> Option<Font> {
        let (headersize, num_glyphs, charsize, width, height) =
            if data.len() >= 32 && data.starts_with(&[0x72, 0xb5, 0x4a, 0x86]) {
                let rd = |o: usize| {
                    u32::from_le_bytes([data[o], data[o + 1], data[o + 2], data[o + 3]]) as usize
                };
                (rd(8), rd(16), rd(20), rd(28), rd(24))
            } else if data.len() >= 4 && data[0] == 0x36 && data[1] == 0x04 {
                let num = if data[2] & 1 != 0 { 512 } else { 256 };
                (4, num, data[3] as usize, 8, data[3] as usize)
            } else {
                return None;
            };

        let bytes_per_row = (width + 7) / 8;
        if width == 0 || height == 0 || num_glyphs == 0 || charsize < bytes_per_row * height {
            return None;
        }
        let end = headersize.checked_add(num_glyphs.checked_mul(charsize)?)?;
        if end > data.len() {
            return None;
        }

        Some(Font {
            glyphs: &data[headersize..end],
            num_glyphs,
            charsize,
            width,
            height,
            bytes_per_row,
        })
    }

    fn glyph(&self, ch: char) -> &'static [u8] {
        let mut idx = ch as usize;
        if idx >= self.num_glyphs {
            idx = '?' as usize;
        }
        if idx >= self.num_glyphs {
            idx = 0;
        }
        let glyphs: &'static [u8] = self.glyphs;
        &glyphs[idx * self.charsize..(idx + 1) * self.charsize]
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum EscState {
    Normal,
    Escape, 
    Csi,    
}

pub struct Console {
    addr: *mut u8,
    shadow: Vec<u8>,
    width: usize,
    height: usize,
    pitch: usize,
    bytes_per_pixel: usize,
    font: Font,
    cols: usize,
    rows: usize,
    cursor_col: usize,
    cursor_row: usize,

    fg: u32,
    bg: u32,
    bold: bool,

    esc: EscState,
    params: [u32; 8],
    nparams: usize,
    cur: u32,
    have_cur: bool,
}

unsafe impl Send for Console {}

impl Console {
    fn new(fb: &FbInfo, font: Font) -> Self {
        let width = fb.width as usize;
        let height = fb.height as usize;
        let pitch = fb.pitch as usize;
        Self {
            addr: fb.addr,
            shadow: vec![0; pitch.saturating_mul(height)],
            width,
            height,
            pitch,
            bytes_per_pixel: (fb.bpp as usize) / 8,
            font,
            cols: width / font.width,
            rows: height / font.height,
            cursor_col: 0,
            cursor_row: 0,
            fg: DEFAULT_FG,
            bg: DEFAULT_BG,
            bold: false,
            esc: EscState::Normal,
            params: [0; 8],
            nparams: 0,
            cur: 0,
            have_cur: false,
        }
    }

    fn put_pixel(&mut self, x: usize, y: usize, color: u32) {
        if x >= self.width || y >= self.height {
            return;
        }
        let offset = y * self.pitch + x * self.bytes_per_pixel;
        self.shadow[offset] = color as u8;
        self.shadow[offset + 1] = (color >> 8) as u8;
        self.shadow[offset + 2] = (color >> 16) as u8;
        if self.bytes_per_pixel == 4 {
            self.shadow[offset + 3] = (color >> 24) as u8;
        }
    }

    fn fill_rect(&mut self, x: usize, y: usize, w: usize, h: usize, color: u32) {
        let x_end = (x + w).min(self.width);
        let y_end = (y + h).min(self.height);
        let bpp = self.bytes_per_pixel;
        for py in y..y_end {
            let start = py * self.pitch + x * bpp;
            let row = &mut self.shadow[start..start + (x_end - x) * bpp];
            if color == 0 {
                row.fill(0);
            } else {
                for pixel in row.chunks_exact_mut(bpp) {
                    pixel[0] = color as u8;
                    pixel[1] = (color >> 8) as u8;
                    pixel[2] = (color >> 16) as u8;
                    if bpp == 4 { pixel[3] = (color >> 24) as u8; }
                }
            }
        }
    }

    /// Copy a changed rectangle from cached RAM to the write-combining
    /// framebuffer. Reads and scrolling stay in ordinary RAM; the device
    /// mapping only sees forward, contiguous writes.
    fn flush_rect(&self, x: usize, y: usize, w: usize, h: usize) {
        let x_end = x.saturating_add(w).min(self.width);
        let y_end = y.saturating_add(h).min(self.height);
        let row_bytes = (x_end - x) * self.bytes_per_pixel;
        for py in y..y_end {
            let offset = py * self.pitch + x * self.bytes_per_pixel;
            unsafe {
                core::ptr::copy_nonoverlapping(
                    self.shadow.as_ptr().add(offset),
                    self.addr.add(offset),
                    row_bytes,
                );
            }
        }
    }

    pub fn cols(&self) -> usize {
        self.cols
    }

    pub fn clear(&mut self) {
        let bg = self.bg;
        self.fill_rect(0, 0, self.width, self.height, bg);
        self.flush_rect(0, 0, self.width, self.height);
        self.cursor_col = 0;
        self.cursor_row = 0;
    }

    fn draw_glyph(&mut self, ch: char, col: usize, row: usize) {
        let font = self.font;
        let glyph = font.glyph(ch);
        let (fg, bg) = (self.fg, self.bg);
        let ox = col * font.width;
        let oy = row * font.height;

        for y in 0..font.height {
            for x in 0..font.width {
                let byte = glyph[y * font.bytes_per_row + x / 8];
                let on = byte & (0x80 >> (x % 8)) != 0;
                self.put_pixel(ox + x, oy + y, if on { fg } else { bg });
            }
        }
        self.flush_rect(ox, oy, font.width, font.height);
    }

    fn newline(&mut self) {
        self.cursor_col = 0;
        if self.cursor_row + 1 >= self.rows {
            self.scroll();
        } else {
            self.cursor_row += 1;
        }
    }

    fn scroll(&mut self) {
        let row_bytes = self.font.height * self.pitch;
        let text_bytes = self.rows * row_bytes;
        self.shadow.copy_within(row_bytes..text_bytes, 0);
        let y = (self.rows - 1) * self.font.height;
        let h = self.font.height;
        self.fill_rect(0, y, self.width, h, DEFAULT_BG);
        self.flush_rect(0, 0, self.width, text_bytes / self.pitch);
    }

    fn put_printable(&mut self, c: char) {
        if self.cursor_col >= self.cols {
            self.newline();
        }
        self.draw_glyph(c, self.cursor_col, self.cursor_row);
        self.cursor_col += 1;
    }

    fn push_param(&mut self) {
        let v = if self.have_cur { self.cur } else { 0 };
        if self.nparams < self.params.len() {
            self.params[self.nparams] = v;
            self.nparams += 1;
        }
        self.cur = 0;
        self.have_cur = false;
    }

    fn extended_color(&self, i: usize) -> Option<(u32, usize)> {
        let n = self.nparams;
        if i + 4 < n && self.params[i + 1] == 2 {
            let r = self.params[i + 2] & 0xFF;
            let g = self.params[i + 3] & 0xFF;
            let b = self.params[i + 4] & 0xFF;
            return Some(((r << 16) | (g << 8) | b, 4));
        }
        if i + 2 < n && self.params[i + 1] == 5 {
            let idx = self.params[i + 2] as usize;
            if idx < 16 {
                return Some((PALETTE[idx], 2));
            }
            return Some((DEFAULT_FG, 2)); 
        }
        None
    }

    fn apply_sgr(&mut self) {
        let mut i = 0;
        while i < self.nparams {
            let p = self.params[i];
            match p {
                0 => {
                    self.fg = DEFAULT_FG;
                    self.bg = DEFAULT_BG;
                    self.bold = false;
                }
                1 => self.bold = true,
                22 => self.bold = false,
                30..=37 => {
                    let idx = (p - 30) as usize + if self.bold { 8 } else { 0 };
                    self.fg = PALETTE[idx];
                }
                39 => self.fg = DEFAULT_FG,
                40..=47 => self.bg = PALETTE[(p - 40) as usize],
                49 => self.bg = DEFAULT_BG,
                90..=97 => self.fg = PALETTE[(p - 90) as usize + 8],
                100..=107 => self.bg = PALETTE[(p - 100) as usize + 8],
                38 | 48 => {
                    if let Some((color, used)) = self.extended_color(i) {
                        if p == 38 {
                            self.fg = color;
                        } else {
                            self.bg = color;
                        }
                        i += used;
                    }
                }
                _ => {}
            }
            i += 1;
        }
    }

    fn put_char(&mut self, c: char) {
        if c == '\x1b' {
            self.esc = EscState::Escape;
            return;
        }

        match self.esc {
            EscState::Escape => {
                if c == '[' {
                    self.esc = EscState::Csi;
                    self.nparams = 0;
                    self.cur = 0;
                    self.have_cur = false;
                } else {
                    self.esc = EscState::Normal;
                }
                return;
            }
            EscState::Csi => {
                match c {
                    '0'..='9' => {
                        self.cur = (self.cur * 10 + (c as u32 - '0' as u32)).min(65535);
                        self.have_cur = true;
                    }
                    ';' => self.push_param(),
                    'm' => {
                        self.push_param();
                        self.apply_sgr();
                        self.esc = EscState::Normal;
                    }
                    'D' => {
                        // CSI n D -- "cursor back n": standard ANSI,
                        // and the escape `syscall::redraw_tail` uses to
                        // reposition after a redraw without disturbing
                        // the just-drawn cells it passes over (unlike
                        // `\x08`, which intentionally blanks). Default
                        // to 1 with no parameter, same as real terminals.
                        self.push_param();
                        let n = if self.nparams > 0 { self.params[0] as usize } else { 1 };
                        self.cursor_left(n);
                        self.esc = EscState::Normal;
                    }
                    'J' => {
                        // CSI 2 J -- "clear entire screen". Real
                        // terminals distinguish "clear from cursor",
                        // "clear to cursor", and "clear all" by the
                        // parameter; this console only ever needs the
                        // shell's `clear` command, which always wants
                        // the whole screen, so every parameter value
                        // clears everything rather than tracking the
                        // distinction. Home the cursor too, matching
                        // what `clear` on a real terminal looks like
                        // (it's normally paired with CSI H).
                        self.push_param();
                        self.clear();
                        self.esc = EscState::Normal;
                    }
                    _ => self.esc = EscState::Normal,
                }
                return;
            }
            EscState::Normal => {}
        }

        match c {
            '\n' => self.newline(),
            '\r' => self.cursor_col = 0,
            '\x08' => self.backspace(),
            '\t' => {
                let next = (self.cursor_col + 4) & !3;
                while self.cursor_col < next {
                    self.put_printable(' ');
                }
            }
            c => self.put_printable(c),
        }
    }

    /// Move the cursor one cell left and blank whatever glyph is there,
    /// matching what a real terminal does for `0x08` -- unlike '\n'/'\r'
    /// this can't be a bare cursor-position update, because leaving the
    /// old glyph on screen is exactly the bug this exists to fix (a
    /// deleted character staying visible after the line shrinks). Does
    /// nothing at column 0: this console has no concept of un-wrapping
    /// onto the previous row, so a caller backspacing across a line
    /// boundary would need to track that itself.
    fn backspace(&mut self) {
        if self.cursor_col == 0 {
            return;
        }
        self.cursor_col -= 1;
        self.draw_glyph(' ', self.cursor_col, self.cursor_row);
    }

    /// Move the cursor left `n` cells without touching what's drawn
    /// there -- for repositioning after a redraw, where the cells being
    /// passed over already hold the correct final content and must not
    /// be blanked (unlike `backspace`, which exists precisely to blank
    /// the cell it moves over). Exposed to callers via the `\x1b[<n>D`
    /// CSI sequence (see `EscState::Csi` handling above).
    pub fn cursor_left(&mut self, n: usize) {
        self.cursor_col = self.cursor_col.saturating_sub(n);
    }
}

impl fmt::Write for Console {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for c in s.chars() {
            self.put_char(c);
        }
        Ok(())
    }
}

pub fn init() {
    let fb_guard = crate::framebuffer::FRAMEBUFFER.lock();
    let Some(fb) = fb_guard.as_ref() else {
        crate::log_fail!("Console", "Init", "No framebuffer available");
        return;
    };

    if fb.bpp != 32 && fb.bpp != 24 {
        crate::log_fail!(
            "Console",
            "Init",
            "Unsupported bpp {} (only 24bpp/32bpp supported)",
            fb.bpp
        );
        return;
    }

    let Some(font) = Font::parse(FONT_DATA) else {
        crate::log_fail!("Console", "Init", "font.psf is not a valid PSF1/PSF2 font");
        return;
    };

    let mut console = Console::new(fb, font);
    if console.cols == 0 || console.rows == 0 {
        crate::log_fail!("Console", "Init", "Screen too small for the selected font");
        return;
    }

    console.clear();

    let (cols, rows) = (console.cols, console.rows);

    *CONSOLE.lock() = Some(console);
    drop(fb_guard);

    crate::logger::flush_to_console();

    crate::log_ok!(
        "Console",
        "Init",
        "Text console ready ({}x{} chars, {}x{} PSF font)",
        cols,
        rows,
        font.width,
        font.height
    );
}
