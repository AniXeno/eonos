//! 8042 PS/2 controller and keyboard.
//!
//! This is EonOS's first real keyboard: everything before this drove
//! the shell over the serial port instead (see `syscall::read_line`).
//! The controller predates USB by decades but QEMU (and most real
//! hardware, even now, via USB legacy emulation) still exposes it, so
//! it's the simplest path to typing on bare metal.
//!
//! Only scancode set 1 (the controller's power-on default) is handled,
//! translated to ASCII for a US QWERTY layout. No PS/2 mouse support
//! (port 2 is left disabled) -- just enough to type shell commands.
//!
//! IRQ1 fires once per scancode byte; the handler does the minimum
//! possible (read the byte, translate it, push to a ring buffer) and
//! leaves everything else to whoever calls `read_byte`/`try_read_byte`,
//! the same shape `serial::try_read_byte` already has so `syscall.rs`
//! can treat both input sources the same way.

#![allow(dead_code)]

use core::sync::atomic::{AtomicBool, Ordering};

use crate::sync::IrqMutex;
use crate::idt;
use crate::{log_fail, log_ok};

const DATA_PORT: u16 = 0x60;
const STATUS_PORT: u16 = 0x64;
const COMMAND_PORT: u16 = 0x64;

const STATUS_OUTPUT_FULL: u8 = 1 << 0;
const STATUS_INPUT_FULL: u8 = 1 << 1;

const CMD_READ_CONFIG: u8 = 0x20;
const CMD_WRITE_CONFIG: u8 = 0x60;
const CMD_DISABLE_PORT1: u8 = 0xAD;
const CMD_ENABLE_PORT1: u8 = 0xAE;
const CMD_DISABLE_PORT2: u8 = 0xA7;
const CMD_SELF_TEST: u8 = 0xAA;
const CMD_TEST_PORT1: u8 = 0xAB;

const SELF_TEST_OK: u8 = 0x55;
const PORT_TEST_OK: u8 = 0x00;

const CONFIG_PORT1_IRQ: u8 = 1 << 0;
const CONFIG_PORT1_TRANSLATION: u8 = 1 << 6;

const KB_CMD_RESET: u8 = 0xFF;
const KB_RESET_ACK: u8 = 0xFA;
const KB_RESET_PASS: u8 = 0xAA;

const IRQ_LINE: u8 = 1;

/// How many bytes buffered scancodes/ASCII can queue up before the
/// oldest is dropped. Generous for a human typing at a prompt -- this
/// is not meant to survive someone holding a key down for a minute
/// with nobody reading.
const BUFFER_CAP: usize = 256;

struct RingBuffer {
    buf: [u8; BUFFER_CAP],
    head: usize, // next slot to write
    tail: usize, // next slot to read
    len: usize,
}

impl RingBuffer {
    const fn new() -> Self {
        RingBuffer { buf: [0; BUFFER_CAP], head: 0, tail: 0, len: 0 }
    }

    fn push(&mut self, byte: u8) {
        if self.len == BUFFER_CAP {
            // Drop the oldest byte rather than the newest -- an idle
            // reader catching up should see recent keystrokes, not
            // ones from a minute ago.
            self.tail = (self.tail + 1) % BUFFER_CAP;
            self.len -= 1;
        }
        self.buf[self.head] = byte;
        self.head = (self.head + 1) % BUFFER_CAP;
        self.len += 1;
    }

    fn pop(&mut self) -> Option<u8> {
        if self.len == 0 {
            return None;
        }
        let b = self.buf[self.tail];
        self.tail = (self.tail + 1) % BUFFER_CAP;
        self.len -= 1;
        Some(b)
    }
}

static ASCII_QUEUE: IrqMutex<RingBuffer> = IrqMutex::new(RingBuffer::new());
static SHIFT_HELD: AtomicBool = AtomicBool::new(false);
static CAPS_LOCK: AtomicBool = AtomicBool::new(false);
static READY: AtomicBool = AtomicBool::new(false);

unsafe fn outb(port: u16, val: u8) {
    core::arch::asm!("out dx, al", in("dx") port, in("al") val, options(nomem, nostack, preserves_flags));
}

unsafe fn inb(port: u16) -> u8 {
    let val: u8;
    core::arch::asm!("in al, dx", in("dx") port, out("al") val, options(nomem, nostack, preserves_flags));
    val
}

/// Spin until the controller's input buffer is empty (safe to write a
/// command/data byte), or give up. Every wait in this driver is bounded
/// -- a wedged or absent controller must not hang boot.
unsafe fn wait_input_clear() -> bool {
    for _ in 0..100_000 {
        if inb(STATUS_PORT) & STATUS_INPUT_FULL == 0 {
            return true;
        }
    }
    false
}

/// Spin until the controller's output buffer has a byte for us to read.
unsafe fn wait_output_full() -> bool {
    for _ in 0..100_000 {
        if inb(STATUS_PORT) & STATUS_OUTPUT_FULL != 0 {
            return true;
        }
    }
    false
}

unsafe fn write_command(cmd: u8) -> bool {
    if !wait_input_clear() {
        return false;
    }
    outb(COMMAND_PORT, cmd);
    true
}

unsafe fn write_data(byte: u8) -> bool {
    if !wait_input_clear() {
        return false;
    }
    outb(DATA_PORT, byte);
    true
}

unsafe fn read_data() -> Option<u8> {
    if !wait_output_full() {
        return None;
    }
    Some(inb(DATA_PORT))
}

/// Bring up the 8042 controller and keyboard, and register the IRQ1
/// handler. Safe to call even on hardware/emulation with no PS/2
/// controller at all -- every step is failure-checked and `init` just
/// logs and returns rather than hanging or panicking, since USB HID
/// (once it exists) may be the only input path on such a machine.
pub fn init() {
    unsafe {
        // An absent 8042 can return 0xFF from its unmapped status port.
        // In particular, QEMU returns this when started with
        // `-machine ...,i8042=off`; recognize it before trying to drain
        // the output buffer, whose full bit would otherwise look stuck.
        if inb(STATUS_PORT) == 0xFF {
            log_ok!("PS2", "Init", "No PS/2 controller detected; continuing without it");
            return;
        }

        // Disable both ports first: a stray byte arriving mid-init
        // (from a mouse on port 2, or a keyboard that was already
        // sending) would otherwise be misread as a command response.
        write_command(CMD_DISABLE_PORT1);
        write_command(CMD_DISABLE_PORT2);

        // Flush anything left in the output buffer from before we took
        // over (firmware POST, a previous OS, ...).
        for _ in 0..100_000 {
            if inb(STATUS_PORT) & STATUS_OUTPUT_FULL == 0 {
                break;
            }
            inb(DATA_PORT);
        }

        if !write_command(CMD_SELF_TEST) {
            log_fail!("PS2", "Init", "Controller did not accept the self-test command");
            return;
        }
        match read_data() {
            Some(SELF_TEST_OK) => {}
            Some(other) => {
                log_fail!("PS2", "Init", "Controller self-test failed (response {:#04x})", other);
                return;
            }
            None => {
                log_fail!("PS2", "Init", "Controller self-test timed out -- no PS/2 controller?");
                return;
            }
        }

        if !write_command(CMD_TEST_PORT1) {
            log_fail!("PS2", "Init", "Controller did not accept the port-1 test command");
            return;
        }
        match read_data() {
            Some(PORT_TEST_OK) => {}
            Some(other) => {
                log_fail!("PS2", "Init", "Port 1 test failed (response {:#04x})", other);
                return;
            }
            None => {
                log_fail!("PS2", "Init", "Port 1 test timed out");
                return;
            }
        }

        // Read-modify-write the controller configuration: enable port 1's
        // IRQ and its scancode translation (set 2 -> set 1, done by the
        // controller itself; simplest for us since we only speak set 1).
        if !write_command(CMD_READ_CONFIG) {
            log_fail!("PS2", "Init", "Controller did not accept read-config command");
            return;
        }
        let Some(mut config) = read_data() else {
            log_fail!("PS2", "Init", "Timed out reading controller configuration");
            return;
        };
        config |= CONFIG_PORT1_IRQ | CONFIG_PORT1_TRANSLATION;
        if !write_command(CMD_WRITE_CONFIG) || !write_data(config) {
            log_fail!("PS2", "Init", "Failed to write back controller configuration");
            return;
        }

        if !write_command(CMD_ENABLE_PORT1) {
            log_fail!("PS2", "Init", "Failed to enable port 1");
            return;
        }

        // Reset the keyboard itself. Expect ACK (0xFA) then a pass code
        // (0xAA); tolerate either order since real hardware isn't
        // always consistent about it.
        if !write_data(KB_CMD_RESET) {
            log_fail!("PS2", "Init", "Failed to send reset to keyboard");
            return;
        }
        let mut saw_ack = false;
        let mut saw_pass = false;
        for _ in 0..2 {
            match read_data() {
                Some(KB_RESET_ACK) => saw_ack = true,
                Some(KB_RESET_PASS) => saw_pass = true,
                Some(_) => {} // ignore stray bytes (e.g. a break code in flight)
                None => break,
            }
        }
        if !saw_ack && !saw_pass {
            log_fail!(
                "PS2",
                "Init",
                "Keyboard did not respond to reset -- no keyboard attached?"
            );
            return;
        }
    }

    idt::register_irq(IRQ_LINE, on_key_irq);
    crate::interrupts::unmask_isa_irq(IRQ_LINE);
    READY.store(true, Ordering::Release);

    log_ok!("PS2", "Init", "Controller and keyboard ready, IRQ1 registered and unmasked");
}

/// Whether `init()` completed successfully. `syscall::sys_read` checks
/// this to decide whether PS/2 is a usable input source alongside (or
/// instead of) serial.
pub fn is_ready() -> bool {
    READY.load(Ordering::Acquire)
}

fn on_key_irq() {
    // Read exactly one byte per interrupt -- the controller raises IRQ1
    // once per byte, and there's nothing else to drain here.
    let scancode = unsafe { inb(DATA_PORT) };
    handle_scancode(scancode);
}

/// Set-1 scancodes -> ASCII, US QWERTY, unshifted. `0` marks keys with
/// no ASCII meaning (function keys, arrows, modifiers, ...) that this
/// minimal driver doesn't try to represent.
#[rustfmt::skip]
const SET1_UNSHIFTED: [u8; 0x60] = [
    0,    0x1B, b'1', b'2', b'3', b'4', b'5', b'6', // 0x00-0x07
    b'7', b'8', b'9', b'0', b'-', b'=', 0x08, b'\t', // 0x08-0x0F
    b'q', b'w', b'e', b'r', b't', b'y', b'u', b'i', // 0x10-0x17
    b'o', b'p', b'[', b']', b'\n', 0,    b'a', b's', // 0x18-0x1F
    b'd', b'f', b'g', b'h', b'j', b'k', b'l', b';', // 0x20-0x27
    b'\'', b'`', 0,    b'\\', b'z', b'x', b'c', b'v', // 0x28-0x2F
    b'b', b'n', b'm', b',', b'.', b'/', 0,    b'*', // 0x30-0x37
    0,    b' ', 0,    0,    0,    0,    0,    0,    // 0x38-0x3F
    0,    0,    0,    0,    0,    0,    0,    b'7', // 0x40-0x47
    b'8', b'9', b'-', b'4', b'5', b'6', b'+', b'1', // 0x48-0x4F
    b'2', b'3', b'0', b'.', 0,    0,    0,    0,    // 0x50-0x57
    0,    0,    0,    0,    0,    0,    0,    0,    // 0x58-0x5F
];

#[rustfmt::skip]
const SET1_SHIFTED: [u8; 0x60] = [
    0,    0x1B, b'!', b'@', b'#', b'$', b'%', b'^', // 0x00-0x07
    b'&', b'*', b'(', b')', b'_', b'+', 0x08, b'\t', // 0x08-0x0F
    b'Q', b'W', b'E', b'R', b'T', b'Y', b'U', b'I', // 0x10-0x17
    b'O', b'P', b'{', b'}', b'\n', 0,    b'A', b'S', // 0x18-0x1F
    b'D', b'F', b'G', b'H', b'J', b'K', b'L', b':', // 0x20-0x27
    b'"', b'~', 0,    b'|', b'Z', b'X', b'C', b'V', // 0x28-0x2F
    b'B', b'N', b'M', b'<', b'>', b'?', 0,    b'*', // 0x30-0x37
    0,    b' ', 0,    0,    0,    0,    0,    0,    // 0x38-0x3F
    0,    0,    0,    0,    0,    0,    0,    b'7', // 0x40-0x47
    b'8', b'9', b'-', b'4', b'5', b'6', b'+', b'1', // 0x48-0x4F
    b'2', b'3', b'0', b'.', 0,    0,    0,    0,    // 0x50-0x57
    0,    0,    0,    0,    0,    0,    0,    0,    // 0x58-0x5F
];

const SCANCODE_LSHIFT: u8 = 0x2A;
const SCANCODE_RSHIFT: u8 = 0x36;
const SCANCODE_CAPS_LOCK: u8 = 0x3A;
/// High bit set on the make code marks a break (key-release) code in
/// scancode set 1; 0xE0 prefixes an extended-key scancode (arrows,
/// Home/End/Delete, right ctrl/alt, the numpad's non-numpad siblings,
/// ...). The prefix and the byte after it arrive as two separate IRQs,
/// so `EXPECT_EXTENDED` remembers across that gap that the next byte
/// needs the extended table, not the normal one.
const BREAK_BIT: u8 = 0x80;
const EXTENDED_PREFIX: u8 = 0xE0;

static EXPECT_EXTENDED: AtomicBool = AtomicBool::new(false);

// Extended (0xE0-prefixed) scancodes this driver gives meaning to.
const EXT_UP: u8 = 0x48;
const EXT_DOWN: u8 = 0x50;
const EXT_RIGHT: u8 = 0x4D;
const EXT_LEFT: u8 = 0x4B;
const EXT_HOME: u8 = 0x47;
const EXT_END: u8 = 0x4F;
const EXT_DELETE: u8 = 0x53;

/// Push a multi-byte ANSI escape sequence onto the queue as one unit.
/// The line editor (`syscall::read_line`) reads and interprets these
/// the same way a terminal emulator would, so PS/2 and (eventually)
/// USB HID can both feed it through the same plain byte queue instead
/// of a richer "key event" type threading through the syscall layer.
fn push_escape(seq: &[u8]) {
    let mut q = ASCII_QUEUE.lock();
    for &b in seq {
        q.push(b);
    }
}

fn handle_scancode(code: u8) {
    if code == EXTENDED_PREFIX {
        EXPECT_EXTENDED.store(true, Ordering::Relaxed);
        return;
    }

    let is_break = code & BREAK_BIT != 0;
    let make_code = code & !BREAK_BIT;
    let extended = EXPECT_EXTENDED.swap(false, Ordering::Relaxed);

    if extended {
        if is_break {
            return; // only act on the press, same as the normal-key path
        }
        match make_code {
            EXT_UP => push_escape(b"\x1b[A"),
            EXT_DOWN => push_escape(b"\x1b[B"),
            EXT_RIGHT => push_escape(b"\x1b[C"),
            EXT_LEFT => push_escape(b"\x1b[D"),
            EXT_HOME => push_escape(b"\x1b[H"),
            EXT_END => push_escape(b"\x1b[F"),
            EXT_DELETE => push_escape(b"\x1b[3~"),
            _ => {} // right ctrl/alt, numpad enter/slash, ... : not handled
        }
        return;
    }

    match make_code {
        SCANCODE_LSHIFT | SCANCODE_RSHIFT => {
            SHIFT_HELD.store(!is_break, Ordering::Relaxed);
            return;
        }
        SCANCODE_CAPS_LOCK => {
            if !is_break {
                let cur = CAPS_LOCK.load(Ordering::Relaxed);
                CAPS_LOCK.store(!cur, Ordering::Relaxed);
            }
            return;
        }
        _ => {}
    }

    if is_break {
        return; // only make (press) codes produce a character
    }

    if (make_code as usize) >= SET1_UNSHIFTED.len() {
        return;
    }

    let shift = SHIFT_HELD.load(Ordering::Relaxed);
    let caps = CAPS_LOCK.load(Ordering::Relaxed);
    let mut ascii = if shift { SET1_SHIFTED[make_code as usize] } else { SET1_UNSHIFTED[make_code as usize] };

    if ascii == 0 {
        return;
    }

    // Caps Lock only affects letters, and only when shift isn't already
    // doing the same job (the two shouldn't cancel out for a plain 'a'
    // key on a US layout the way some real keyboards behave, but this
    // is a hobby driver, not a full HID layout engine).
    if caps && ascii.is_ascii_lowercase() {
        ascii = ascii.to_ascii_uppercase();
    } else if caps && ascii.is_ascii_uppercase() && shift {
        ascii = ascii.to_ascii_lowercase();
    }

    ASCII_QUEUE.lock().push(ascii);
}

/// Non-blocking: `Some(byte)` if a translated keystroke is waiting,
/// `None` otherwise. Mirrors `serial::try_read_byte` so `syscall.rs`
/// can poll both input sources the same way.
pub fn try_read_byte() -> Option<u8> {
    ASCII_QUEUE.lock().pop()
}
