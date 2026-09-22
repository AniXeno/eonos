//! `SYSCALL`/`SYSRET`: the fast path a user-mode thread uses to trap
//! into the kernel and back, plus `enter_user_mode`, the one-way door
//! from ring 0 into ring 3 that something has to walk through first.
//!
//! `SYSCALL` is deliberately minimal in hardware: it saves RIP in RCX
//! and RFLAGS in R11, loads CS/SS from `STAR` and RIP from `LSTAR`, and
//! masks RFLAGS with `SFMASK` -- and that's it. Unlike a hardware
//! interrupt/exception (which, on a ring 3 -> ring 0 transition, also
//! switches to the stack in `TSS.rsp0` automatically), `SYSCALL` does
//! *not* touch RSP at all: the entry stub is still running on whatever
//! stack the user thread had. The stub's first job is always to get off
//! that stack before doing anything else, using `gdt::TSS_RSP0` -- the
//! same "top of this thread's kernel stack" value the scheduler already
//! maintains for the hardware path -- to find a safe one.
//!
//! Self-test: the test maps a couple of user-accessible pages straight
//! into the kernel's own address space, drops a dedicated thread into
//! ring 3 to run a few hand-assembled instructions there, and checks
//! that the `syscall` those instructions issue makes it back into the
//! kernel with the right argument. (Real processes, with their own
//! address spaces, live in `process.rs`.)
//!
//! System calls follow the Linux x86-64 convention -- number in RAX,
//! arguments in RDI/RSI/RDX/R10/R8/R9, result (or a negative errno) in
//! RAX -- and reuse Linux's numbers for the calls that exist, so
//! ordinary tools and libcs have a chance of working later.

#![allow(dead_code)]

use core::fmt::Write;
use core::sync::atomic::{AtomicBool, AtomicU64, Ordering};

use crate::{gdt, pit, pmm, process, scheduler, sync, vmm};
use crate::{log_debug, log_fail, log_ok};

const MSR_EFER: u32 = 0xC000_0080;
const MSR_STAR: u32 = 0xC000_0081;
const MSR_LSTAR: u32 = 0xC000_0082;
const MSR_SFMASK: u32 = 0xC000_0084;
const EFER_SCE: u64 = 1 << 0;

unsafe fn rdmsr(msr: u32) -> u64 {
    let lo: u32;
    let hi: u32;
    core::arch::asm!(
        "rdmsr",
        in("ecx") msr, out("eax") lo, out("edx") hi,
        options(nomem, nostack, preserves_flags),
    );
    ((hi as u64) << 32) | lo as u64
}

unsafe fn wrmsr(msr: u32, value: u64) {
    let lo = value as u32;
    let hi = (value >> 32) as u32;
    core::arch::asm!(
        "wrmsr",
        in("ecx") msr, in("eax") lo, in("edx") hi,
        options(nomem, nostack, preserves_flags),
    );
}

/// Register frame `syscall_entry` builds on the kernel stack before
/// calling `syscall_dispatch`, lowest address first (i.e. in the order
/// the entry stub pushes them, read back to front).
#[repr(C)]
pub struct SyscallFrame {
    /// Syscall number in, return value out.
    pub rax: u64,
    pub rdi: u64,
    pub rsi: u64,
    pub rdx: u64,
    /// 4th argument. Not RCX -- `syscall` clobbers RCX with the return
    /// RIP, so the calling convention moves that argument to R10.
    pub r10: u64,
    pub r8: u64,
    pub r9: u64,
    /// Saved by the CPU into RCX; restored from here before `sysretq`.
    pub rip: u64,
    /// Saved by the CPU into R11; restored from here before `sysretq`.
    pub rflags: u64,
    pub user_rsp: u64,
}

extern "C" {
    fn syscall_entry();
    fn enter_user_mode_asm(entry: u64, user_rsp: u64, user_cs: u64, user_ds: u64) -> !;
}

core::arch::global_asm!(
    r#"
.section .bss
.balign 8
syscall_scratch_rsp:
    .skip 8

.section .text

.global syscall_entry
syscall_entry:
    # On entry: RIP is in RCX, RFLAGS is in R11, CS/SS are already the
    # kernel selectors (from STAR) -- but RSP is still the user thread's
    # own stack, exactly as `syscall` left it. Get off it before doing
    # anything else.
    mov [rip + syscall_scratch_rsp], rsp
    mov rsp, [rip + TSS_RSP0]
    push qword ptr [rip + syscall_scratch_rsp] # user_rsp
    push r11                               # rflags
    push rcx                               # rip
    push r9
    push r8
    push r10
    push rdx
    push rsi
    push rdi
    push rax
    mov rdi, rsp
    cld
    call syscall_dispatch
    # The dispatcher runs with interrupts enabled. Turn them off again
    # before RSP is pointed back at the *user* stack below: an interrupt
    # taken in ring 0 on a user-controlled stack would be a disaster.
    # SYSRET restores IF from the saved RFLAGS (R11).
    cli
    pop rax
    pop rdi
    pop rsi
    pop rdx
    pop r10
    pop r8
    pop r9
    pop rcx
    pop r11
    pop rsp
    sysretq

.global enter_user_mode_asm
enter_user_mode_asm:
    # rdi = entry, rsi = user_rsp, rdx = user_cs (RPL 3), rcx = user_ds (RPL 3)
    mov ax, cx
    mov ds, ax
    mov es, ax
    mov fs, ax
    mov gs, ax
    push rcx        # SS
    push rsi        # RSP
    pushfq
    pop rax
    or rax, 0x200   # force IF, so the new thread is interruptible
    push rax        # RFLAGS
    push rdx        # CS
    push rdi        # RIP
    # Don't leak kernel register contents into the new program.
    xor eax, eax
    xor ebx, ebx
    xor ecx, ecx
    xor edx, edx
    xor esi, esi
    xor edi, edi
    xor ebp, ebp
    xor r8d, r8d
    xor r9d, r9d
    xor r10d, r10d
    xor r11d, r11d
    xor r12d, r12d
    xor r13d, r13d
    xor r14d, r14d
    xor r15d, r15d
    iretq
"#
);

/// Wire up `SYSCALL`/`SYSRET`. Call once, after `gdt::init` (needs
/// `USER_CS`/`USER_DS` to already be in the GDT) and before any thread
/// might execute a `syscall`.
pub fn init() {
    unsafe {
        let efer = rdmsr(MSR_EFER);
        wrmsr(MSR_EFER, efer | EFER_SCE);

        // STAR[47:32]: SYSCALL loads CS from here (RPL forced 0) and SS
        // from here+8 -- must be the kernel code selector, immediately
        // followed by the kernel data selector.
        //
        // STAR[63:48]: SYSRETQ loads SS from here+8 and CS from here+16
        // (RPL forced 3) -- so this must be 8 less than USER_DS, which
        // is exactly why gdt.rs lays USER_DS/USER_CS out back-to-back.
        let syscall_base = gdt::KERNEL_CS as u64;
        let sysret_base = gdt::USER_DS as u64 - 8;
        wrmsr(MSR_STAR, (sysret_base << 48) | (syscall_base << 32));

        wrmsr(MSR_LSTAR, syscall_entry as usize as u64);

        // Cleared from RFLAGS on entry, before the stub has a valid
        // stack: IF (so the stub can't be preempted mid-switch), TF and
        // DF (so a stray debug/string-op assumption from user mode
        // can't follow it in).
        wrmsr(MSR_SFMASK, (1 << 8) | (1 << 9) | (1 << 10));
    }

    log_ok!(
        "Syscall",
        "Init",
        "SYSCALL/SYSRET enabled (entry={:#018x}, user_cs={:#04x}, user_ds={:#04x})",
        syscall_entry as usize,
        gdt::USER_CS | 3,
        gdt::USER_DS | 3
    );
}

// System call numbers (Linux x86-64 numbering for the ones Linux has).
pub const SYS_READ: u64 = 0;
pub const SYS_WRITE: u64 = 1;
pub const SYS_SCHED_YIELD: u64 = 24;
pub const SYS_GETPID: u64 = 39;
pub const SYS_EXIT: u64 = 60;
pub const SYS_WAIT4: u64 = 61;
pub const SYS_OPEN: u64 = 2;
pub const SYS_CLOSE: u64 = 3;

// EonOS-specific calls, well clear of Linux's own numbers, standing in
// for a real filesystem (open/read/close on the initramfs) until one
// exists. `list_files` writes a newline-separated listing; `read_file`
// looks a path up by name and copies its whole contents in one shot --
// no offsets, no partial reads. Both are stopgaps for the shell's
// `ls`/`cat` and are expected to go away once EonOS has real fds.
pub const SYS_LIST_FILES: u64 = 9001;
pub const SYS_READ_FILE: u64 = 9002;
/// Milliseconds since boot, per `pit::uptime_ms()`. Backs the shell's
/// `uptime` command; Linux has no equivalent single-call primitive
/// (it's normally read out of /proc), so this gets an EonOS-specific
/// number like the two above rather than trying to match a real one.
pub const SYS_UPTIME_MS: u64 = 9003;
pub const SYS_SLEEP_MS: u64 = 9004;
pub const SYS_EXEC: u64 = 9005;

struct OpenFile { pid: u64, fd: u64, path: alloc::string::String, offset: usize }
static OPEN_FILES: sync::IrqMutex<alloc::vec::Vec<OpenFile>> = sync::IrqMutex::new(alloc::vec::Vec::new());

const EBADF: i64 = 9;
const EFAULT: i64 = 14;
const ENOENT: i64 = 2;
const ENOSYS: i64 = 38;
const ENAMETOOLONG: i64 = 36;
const ERANGE: i64 = 34;
const ESRCH: i64 = 3;

/// Encode a positive errno as the negative value a syscall returns.
const fn errno(e: i64) -> u64 {
    (-e) as u64
}

/// Longest run of bytes a single `write` will take; longer requests are
/// short-written (callers loop, exactly as with Linux).
const MAX_WRITE: usize = 4096;

/// Put raw process output on the serial port and the screen, without
/// the log decorations `klog!` adds.
fn console_write(bytes: &[u8]) {
    let text = alloc::string::String::from_utf8_lossy(bytes);
    {
        let mut serial = crate::serial::SERIAL1.lock();
        let _ = serial.write_str(&text);
    }
    if let Some(console) = crate::console::CONSOLE.lock().as_mut() {
        let _ = console.write_str(&text);
    }
}

/// How many previous lines `read_line` remembers for up/down-arrow
/// recall. Bounded so a long session doesn't grow this forever; old
/// entries are dropped, same trade-off as the PS/2 driver's own ring
/// buffer.
const HISTORY_CAP: usize = 32;

static HISTORY: sync::IrqMutex<alloc::collections::VecDeque<alloc::vec::Vec<u8>>> =
    sync::IrqMutex::new(alloc::collections::VecDeque::new());

/// Next raw input byte from whichever source has one, blocking
/// (via yield, not a real wait queue) until one shows up. PS/2 and
/// serial both just fill a byte queue from their own IRQ handlers, so
/// polling either in turn is enough -- no reason to prefer one over
/// the other beyond "whichever has something waiting first".
fn next_byte() -> u8 {
    loop {
        crate::drivers::usb::xhci::poll();
        if let Some(b) = crate::drivers::ps2::try_read_byte() {
            return b;
        }
        if let Some(b) = crate::drivers::usb::xhci::try_read_byte() {
            return b;
        }
        if let Some(b) = crate::serial::SERIAL1.lock().try_read_byte() {
            return b;
        }
        scheduler::yield_now();
    }
}

/// Decode a `\x1b[...` escape sequence (as PS/2 arrow/nav keys are
/// encoded into by `drivers::ps2::push_escape`, and as a real terminal
/// emulator would send them over serial too) into one of a small fixed
/// set of edit actions. Called right after the ESC byte itself is
/// consumed. Reads (and blocks on) further bytes as needed -- safe
/// because a `\x1b` this driver produces is always immediately followed
/// by the rest of its sequence; a lone stray ESC from a real keyboard's
/// Esc key will just stall until another byte arrives, same as it would
/// on a real tty waiting to see if it's the start of a sequence.
enum EditAction {
    Up,
    Down,
    Left,
    Right,
    Home,
    End,
    Delete,
    Unknown,
}

fn decode_escape() -> EditAction {
    if next_byte() != b'[' {
        return EditAction::Unknown;
    }
    match next_byte() {
        b'A' => EditAction::Up,
        b'B' => EditAction::Down,
        b'C' => EditAction::Right,
        b'D' => EditAction::Left,
        b'H' => EditAction::Home,
        b'F' => EditAction::End,
        b'3' => {
            // Delete is "\x1b[3~" -- a three-byte final sequence rather
            // than one letter, so it needs an extra byte consumed.
            let _ = next_byte(); // expected '~'
            EditAction::Delete
        }
        _ => EditAction::Unknown,
    }
}

/// Erase the currently-displayed line on screen and replace both the
/// buffer and the terminal display with `new_line`, cursor at the end.
/// Used for history recall, where the whole line changes at once
/// rather than one character at a time.
fn replace_line(line: &mut alloc::vec::Vec<u8>, cursor: &mut usize, new_line: alloc::vec::Vec<u8>) {
    cursor_left(*cursor);
    for _ in 0..line.len() {
        console_write(b" ");
    }
    cursor_left(line.len());
    console_write(&new_line);
    *line = new_line;
    *cursor = line.len();
}

/// Move the terminal cursor left `n` cells without touching what's
/// drawn there, via the standard `CSI n D` sequence (`console.rs`
/// handles it; a real serial terminal understands it natively too).
/// Distinct from sending literal `0x08` bytes, which both this
/// console and real terminals treat as destructive backspace -- correct
/// for deleting a character, wrong for merely repositioning over
/// content that must stay on screen (redraws, arrow-key movement).
fn cursor_left(n: usize) {
    if n == 0 {
        return;
    }
    let mut seq = alloc::vec::Vec::new();
    seq.extend_from_slice(b"\x1b[");
    seq.extend_from_slice(itoa(n).as_bytes());
    seq.push(b'D');
    console_write(&seq);
}

/// Minimal integer-to-decimal-ASCII, since `core::fmt` formatting isn't
/// worth pulling into this hot little path. `n` is always small here
/// (line lengths), so no need for anything fancier.
fn itoa(mut n: usize) -> alloc::string::String {
    if n == 0 {
        return alloc::string::String::from("0");
    }
    let mut digits = alloc::vec::Vec::new();
    while n > 0 {
        digits.push(b'0' + (n % 10) as u8);
        n /= 10;
    }
    digits.reverse();
    alloc::string::String::from_utf8(digits).unwrap()
}

/// Redraw the visible line from `from` (a byte index into `line`) to
/// its end, then park the cursor back at `cursor`. Used whenever an
/// edit touches anything before the end of the line -- a plain append
/// or trailing backspace doesn't need this and stays on the cheaper
/// path in `read_line` itself.
fn redraw_tail(line: &[u8], from: usize, cursor: usize) {
    console_write(&line[from..]);
    console_write(b" "); // erase whatever character used to trail here
    cursor_left(line.len() - cursor + 1);
}

/// Read one line of input for `sys_read(0, ...)`.
///
/// Input comes from the PS/2 keyboard (`drivers::ps2`), a USB HID boot
/// keyboard (`drivers::usb::xhci`), or the
/// serial port (COM1 -- in QEMU, typically `-serial stdio`, i.e. the
/// host terminal); both are polled so either can drive the shell.
/// This does canonical-mode editing in the kernel -- echoing bytes
/// back, backspace/delete, left/right cursor movement, and up/down
/// history recall via ANSI escape sequences -- the same job a real
/// tty line discipline does, just smaller.
///
/// Polls rather than waiting on an IRQ: both input sources just fill a
/// buffer in their own interrupt handlers, so a bare read-and-retry
/// loop (yielding between attempts so it doesn't starve other threads)
/// is the simplest way to consume either. Returns at most `max` bytes,
/// including the trailing `\n`.
fn read_line(max: usize) -> alloc::vec::Vec<u8> {
    let mut line: alloc::vec::Vec<u8> = alloc::vec::Vec::new();
    if max == 0 {
        return line;
    }
    let mut cursor = 0usize; // byte index into `line`; can be < line.len()
    let mut history_pos: Option<usize> = None; // index into HISTORY while recalling

    loop {
        let byte = next_byte();

        match byte {
            b'\r' | b'\n' => {
                console_write(b"\n");
                if line.len() < max {
                    line.push(b'\n');
                }
                break;
            }
            0x1B => {
                match decode_escape() {
                    EditAction::Left => {
                        if cursor > 0 {
                            cursor -= 1;
                            cursor_left(1);
                        }
                    }
                    EditAction::Right => {
                        if cursor < line.len() {
                            console_write(&line[cursor..cursor + 1]);
                            cursor += 1;
                        }
                    }
                    EditAction::Home => {
                        cursor_left(cursor);
                        cursor = 0;
                    }
                    EditAction::End => {
                        console_write(&line[cursor..]);
                        cursor = line.len();
                    }
                    EditAction::Delete => {
                        if cursor < line.len() {
                            line.remove(cursor);
                            redraw_tail(&line, cursor, cursor);
                        }
                    }
                    EditAction::Up => {
                        let history = HISTORY.lock();
                        if history.is_empty() {
                            continue;
                        }
                        let next_pos = match history_pos {
                            None => history.len() - 1,
                            Some(0) => 0,
                            Some(p) => p - 1,
                        };
                        history_pos = Some(next_pos);
                        let entry = history[next_pos].clone();
                        drop(history);
                        replace_line(&mut line, &mut cursor, entry);
                    }
                    EditAction::Down => {
                        let history = HISTORY.lock();
                        match history_pos {
                            None => {} // already at the blank line, nothing newer
                            Some(p) if p + 1 < history.len() => {
                                let next_pos = p + 1;
                                history_pos = Some(next_pos);
                                let entry = history[next_pos].clone();
                                drop(history);
                                replace_line(&mut line, &mut cursor, entry);
                            }
                            Some(_) => {
                                history_pos = None;
                                drop(history);
                                replace_line(&mut line, &mut cursor, alloc::vec::Vec::new());
                            }
                        }
                    }
                    EditAction::Unknown => {}
                }
            }
            0x08 | 0x7F => {
                // Backspace/DEL: remove the character before the
                // cursor, if any. A trailing backspace (cursor at the
                // end) is the common case and stays cheap; editing
                // mid-line requires reflowing everything after it.
                if cursor > 0 {
                    cursor -= 1;
                    line.remove(cursor);
                    console_write(b"\x08");
                    redraw_tail(&line, cursor, cursor);
                }
            }
            b => {
                if line.len() + 1 < max {
                    if cursor == line.len() {
                        line.push(b);
                        console_write(&[b]);
                        cursor += 1;
                    } else {
                        line.insert(cursor, b);
                        cursor += 1;
                        // Redraw from the inserted character onward (not
                        // from `cursor` alone) so it's drawn exactly
                        // once, then walk the terminal cursor back to
                        // just after it.
                        console_write(&line[cursor - 1..]);
                        for _ in 0..(line.len() - cursor) {
                            console_write(b"\x08");
                        }
                    }
                }
            }
        }
    }

    // Record non-empty, non-history-recalled lines for future recall.
    // (The trailing '\n' is stripped for storage and reattached to
    // whatever's returned to the caller as usual.)
    if line.len() > 1 {
        let mut h = HISTORY.lock();
        let stored = line[..line.len() - 1].to_vec();
        if h.back() != Some(&stored) {
            if h.len() == HISTORY_CAP {
                h.pop_front();
            }
            h.push_back(stored);
        }
    }

    line
}

fn sys_read(fd: u64, buf: u64, len: u64) -> u64 {
    if fd != 0 {
        let pid = scheduler::current_id();
        let mut files = OPEN_FILES.lock();
        let Some(file) = files.iter_mut().find(|f| f.pid == pid && f.fd == fd) else { return errno(EBADF) };
        let n = (len as usize).min(MAX_WRITE);
        let mut data = alloc::vec![0; n];
        let read = match crate::vfs::read_at(&file.path, file.offset, &mut data) {
            Ok(n) => n,
            Err(_) => return errno(ENOENT),
        };
        if !vmm::copy_to_user(buf, &data[..read]) { return errno(EFAULT); }
        file.offset += read;
        return read as u64;
    }
    let n = (len as usize).min(MAX_WRITE);
    if n == 0 {
        return 0;
    }
    let line = read_line(n);
    if !vmm::copy_to_user(buf, &line) {
        return errno(EFAULT);
    }
    line.len() as u64
}

fn sys_open(path_ptr: u64, path_len: u64) -> u64 {
    let n = path_len as usize;
    if n > MAX_NAME { return errno(ENAMETOOLONG); }
    let mut path = alloc::vec![0; n];
    if !vmm::copy_from_user(&mut path, path_ptr) { return errno(EFAULT); }
    let Ok(path) = core::str::from_utf8(&path) else { return errno(ENOENT) };
    if crate::vfs::file_size(path).is_err() { return errno(ENOENT); }
    let pid = scheduler::current_id();
    let mut files = OPEN_FILES.lock();
    let fd = (3..).find(|fd| !files.iter().any(|f| f.pid == pid && f.fd == *fd)).unwrap_or(3);
    files.push(OpenFile { pid, fd, path: alloc::string::String::from(path), offset: 0 });
    fd
}

fn sys_close(fd: u64) -> u64 {
    let pid = scheduler::current_id();
    let mut files = OPEN_FILES.lock();
    if let Some(i) = files.iter().position(|f| f.pid == pid && f.fd == fd) { files.swap_remove(i); 0 } else { errno(EBADF) }
}

/// Drop a process's file descriptors as part of process teardown.
pub fn close_process_files(pid: u64) {
    OPEN_FILES.lock().retain(|f| f.pid != pid);
}

fn sys_kill(pid: u64, signal: u64) -> u64 {
    if pid != scheduler::current_id() { return errno(ESRCH); }
    match signal {
        0 => 0,
        9 | 15 => process::exit_current(128 + signal as i64),
        _ => errno(ENOSYS),
    }
}

fn sys_exec(path_ptr: u64, path_len: u64) -> u64 {
    let n = path_len as usize;
    if n > MAX_NAME { return errno(ENAMETOOLONG); }
    let mut path = alloc::vec![0; n];
    if !vmm::copy_from_user(&mut path, path_ptr) { return errno(EFAULT); }
    let Ok(path) = core::str::from_utf8(&path) else { return errno(ENOENT) };
    match process::spawn(path) { Ok(pid) => pid, Err(_) => errno(ENOENT) }
}

fn sys_waitpid(pid: u64, status_ptr: u64) -> u64 {
    loop {
        if let Some(status) = process::take_exit_status(pid) {
            if status_ptr != 0 && !vmm::copy_to_user(status_ptr, &(status as i32).to_ne_bytes()) { return errno(EFAULT); }
            return pid;
        }
        scheduler::yield_now();
    }
}

/// Longest path/file name the file stopgap syscalls accept.
const MAX_NAME: usize = 255;

fn sys_list_files(buf: u64, len: u64) -> u64 {
    let cap = (len as usize).min(MAX_WRITE);
    let mut out = alloc::vec::Vec::new();
    for e in crate::vfs::list_files() {
        if !out.is_empty() {
            out.push(b'\n');
        }
        out.extend_from_slice(e.as_bytes());
    }
    if out.len() > cap {
        return errno(ERANGE);
    }
    if !vmm::copy_to_user(buf, &out) {
        return errno(EFAULT);
    }
    out.len() as u64
}

fn sys_read_file(name_ptr: u64, name_len: u64, buf: u64, buf_len: u64) -> u64 {
    let name_len = name_len as usize;
    if name_len > MAX_NAME {
        return errno(ENAMETOOLONG);
    }
    let mut name = alloc::vec![0u8; name_len];
    if !vmm::copy_from_user(&mut name, name_ptr) {
        return errno(EFAULT);
    }
    let Ok(name) = core::str::from_utf8(&name) else {
        return errno(ENOENT);
    };
    let cap = (buf_len as usize).min(MAX_WRITE);
    let Ok(file_size) = crate::vfs::file_size(name) else { return errno(ENOENT) };
    if file_size > cap { return errno(ERANGE); }
    let Ok(data) = crate::vfs::read_all(name) else { return errno(ENOENT) };
    if data.len() > cap {
        return errno(ERANGE);
    }
    if !vmm::copy_to_user(buf, &data) {
        return errno(EFAULT);
    }
    data.len() as u64
}

fn sys_write(fd: u64, buf: u64, len: u64) -> u64 {
    if fd != 1 && fd != 2 {
        return errno(EBADF);
    }
    let n = (len as usize).min(MAX_WRITE);
    if n == 0 {
        return 0;
    }
    let mut data = alloc::vec![0u8; n];
    if !vmm::copy_from_user(&mut data, buf) {
        return errno(EFAULT);
    }
    console_write(&data);
    n as u64
}

#[no_mangle]
extern "C" fn syscall_dispatch(frame: &mut SyscallFrame) {
    // `SFMASK` cleared IF on entry because the stub had no stack to
    // trust. The frame now lives on this thread's own kernel stack, so
    // interrupts (and with them preemption) can safely come back on:
    // a syscall may take as long as it likes without stalling the timer.
    crate::idt::enable_interrupts();

    match frame.rax {
        SYS_OPEN => frame.rax = sys_open(frame.rdi, frame.rsi),
        SYS_CLOSE => frame.rax = sys_close(frame.rdi),
        SYS_READ => frame.rax = sys_read(frame.rdi, frame.rsi, frame.rdx),
        SYS_WRITE => frame.rax = sys_write(frame.rdi, frame.rsi, frame.rdx),
        SYS_LIST_FILES => frame.rax = sys_list_files(frame.rdi, frame.rsi),
        SYS_READ_FILE => frame.rax = sys_read_file(frame.rdi, frame.rsi, frame.rdx, frame.r10),
        SYS_UPTIME_MS => frame.rax = pit::uptime_ms(),
        SYS_SLEEP_MS => { scheduler::sleep_ms(frame.rdi); frame.rax = 0; }
        SYS_EXEC => frame.rax = sys_exec(frame.rdi, frame.rsi),
        SYS_WAIT4 => frame.rax = sys_waitpid(frame.rdi, frame.rdx),
        62 => frame.rax = sys_kill(frame.rdi, frame.rsi),
        SYS_SCHED_YIELD => {
            scheduler::yield_now();
            frame.rax = 0;
        }
        SYS_GETPID => frame.rax = scheduler::current_id(),
        // Never returns: records the status, ends the thread, and the
        // scheduler frees the process's address space once it's off the CPU.
        SYS_EXIT => process::exit_current((frame.rdi & 0xff) as i64),
        TEST_SYSCALL_NUM => {
            TEST_ARG.store(frame.rdi, Ordering::SeqCst);
            TEST_DONE.store(true, Ordering::SeqCst);
            // Nothing sensible to sysret to for the self-test's
            // throwaway thread -- tear it down here instead.
            scheduler::exit();
        }
        other => {
            log_debug!(
                "Syscall",
                "Unknown",
                "syscall {:#x} from user mode (rip={:#018x})",
                other,
                frame.rip
            );
            frame.rax = errno(ENOSYS);
        }
    }
}

/// Drop from ring 0 into ring 3 at `entry`, running on `user_rsp`. Never
/// returns to the caller: either the user code eventually `syscall`s
/// into a handler that exits the thread, or (self-test only) it spins.
pub fn enter_user_mode(entry: u64, user_rsp: u64) -> ! {
    unsafe {
        enter_user_mode_asm(
            entry,
            user_rsp,
            (gdt::USER_CS | 3) as u64,
            (gdt::USER_DS | 3) as u64,
        )
    }
}

// ---------------------------------------------------------------------
// Self-test
// ---------------------------------------------------------------------

const TEST_SYSCALL_NUM: u64 = 0x42;
const TEST_MAGIC: u64 = 0xABCD_1234_DEAD_BEEF;

static TEST_DONE: AtomicBool = AtomicBool::new(false);
static TEST_ARG: AtomicU64 = AtomicU64::new(0);

const USER_CODE_VIRT: u64 = 0x0000_0000_0040_0000;
const USER_STACK_TOP_VIRT: u64 = 0x0000_0000_0060_0000;
const USER_STACK_PAGE_VIRT: u64 = USER_STACK_TOP_VIRT - pmm::PAGE_SIZE;

/// Hand-assembled, position-independent (no absolute addresses) machine
/// code for the self-test's "user program":
///
/// ```asm
/// mov rax, 0x42                       ; TEST_SYSCALL_NUM
/// movabs rdi, 0xABCD1234DEADBEEF       ; TEST_MAGIC, passed as arg0
/// syscall
/// 1: jmp 1b                           ; unreachable: the handler above
///                                      ; never sysret's back to us
/// ```
#[rustfmt::skip]
const USER_TEST_PROGRAM: [u8; 24] = [
    0x48, 0xB8, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // mov rax, 0x42
    0x48, 0xBF, 0xEF, 0xBE, 0xAD, 0xDE, 0x34, 0x12, 0xCD, 0xAB, // movabs rdi, 0xABCD1234DEADBEEF
    0x0F, 0x05,                                                 // syscall
    0xEB, 0xFE,                                                 // jmp $
];

fn user_test_thread() {
    let Some(code_phys) = pmm::alloc_frame_zeroed() else {
        log_fail!("Syscall", "SelfTest", "No frame for the user code page");
        scheduler::exit();
    };
    let Some(stack_phys) = pmm::alloc_frame_zeroed() else {
        log_fail!("Syscall", "SelfTest", "No frame for the user stack page");
        scheduler::exit();
    };

    unsafe {
        let dst = pmm::phys_to_virt(code_phys);
        core::ptr::copy_nonoverlapping(USER_TEST_PROGRAM.as_ptr(), dst, USER_TEST_PROGRAM.len());
    }

    vmm::map_page(USER_CODE_VIRT, code_phys, vmm::USER_RX);
    vmm::map_page(USER_STACK_PAGE_VIRT, stack_phys, vmm::USER_RW);

    enter_user_mode(USER_CODE_VIRT, USER_STACK_TOP_VIRT);
}

pub fn self_test() {
    TEST_DONE.store(false, Ordering::SeqCst);
    TEST_ARG.store(0, Ordering::SeqCst);

    if scheduler::spawn("usertest", user_test_thread).is_none() {
        log_fail!("Syscall", "SelfTest", "Could not spawn the user-mode test thread");
        return;
    }

    let deadline = pit::uptime_ms() + 2000;
    while !TEST_DONE.load(Ordering::SeqCst) {
        if pit::uptime_ms() > deadline {
            log_fail!("Syscall", "SelfTest", "Timed out waiting for a syscall from ring 3");
            return;
        }
        scheduler::yield_now();
    }

    // Let the test thread actually get reaped before freeing the pages
    // it was running on top of.
    scheduler::sleep_ms(20);

    let code_phys = vmm::translate(USER_CODE_VIRT);
    let stack_phys = vmm::translate(USER_STACK_PAGE_VIRT);
    vmm::unmap_page(USER_CODE_VIRT);
    vmm::unmap_page(USER_STACK_PAGE_VIRT);
    if let Some(p) = code_phys {
        pmm::free_frame(p);
    }
    if let Some(p) = stack_phys {
        pmm::free_frame(p);
    }

    let arg = TEST_ARG.load(Ordering::SeqCst);
    if arg != TEST_MAGIC {
        log_fail!(
            "Syscall",
            "SelfTest",
            "syscall arrived but rdi was {:#018x}, expected {:#018x}",
            arg,
            TEST_MAGIC
        );
        return;
    }

    log_ok!(
        "Syscall",
        "SelfTest",
        "Dropped to ring 3 (cs={:#04x}), `syscall` reached the kernel with rdi={:#018x}, thread torn down cleanly",
        gdt::USER_CS | 3,
        arg
    );
}
