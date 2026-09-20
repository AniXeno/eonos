//! Intel 8253/8254 Programmable Interval Timer, driven off IRQ0.
//!
//! This is the kernel's first real hardware interrupt (as opposed to a
//! CPU-generated exception like the `int3` self-test): it proves the
//! IDT, the remapped PIC, and `sti` all actually work together, and it
//! gives the future scheduler a tick source.

#![allow(dead_code)]

use core::sync::atomic::{AtomicU64, Ordering};

use crate::pic;
use crate::{idt, log_fail, log_ok};

const CHANNEL0_DATA: u16 = 0x40;
const COMMAND: u16 = 0x43;

/// The PIT's crystal frequency. Every programmable rate is this divided
/// by a 16-bit divisor.
const BASE_FREQUENCY: u32 = 1_193_182;

const IRQ_LINE: u8 = 0;

static TICKS: AtomicU64 = AtomicU64::new(0);
static HZ: AtomicU64 = AtomicU64::new(0);

unsafe fn outb(port: u16, val: u8) {
    core::arch::asm!("out dx, al", in("dx") port, in("al") val, options(nomem, nostack, preserves_flags));
}

/// Program channel 0 for periodic (mode 3, square wave) interrupts at
/// approximately `hz`, and register the IRQ0 handler. Interrupts still
/// need to be globally enabled (`idt::enable_interrupts()`) for ticks to
/// actually start arriving.
pub fn init(hz: u32) {
    // The PIT can't go slower than ~18.2 Hz (divisor 65536 wraps to 0,
    // meaning "65536"), and there's no reason for a kernel tick to run
    // faster than 1 kHz.
    let hz = hz.clamp(19, 1000);
    let divisor = (BASE_FREQUENCY / hz).clamp(1, 65535) as u16;

    unsafe {
        outb(COMMAND, 0x36); // channel 0, lobyte/hibyte access, mode 3, binary
        outb(CHANNEL0_DATA, (divisor & 0xFF) as u8);
        outb(CHANNEL0_DATA, (divisor >> 8) as u8);
    }

    HZ.store(hz as u64, Ordering::Relaxed);
    TICKS.store(0, Ordering::Relaxed);

    idt::register_irq(IRQ_LINE, on_tick);
    pic::unmask(IRQ_LINE);

    log_ok!(
        "PIT",
        "Init",
        "Channel 0 programmed for {} Hz (divisor {}), IRQ0 registered and unmasked",
        hz,
        divisor
    );
}

fn on_tick() {
    TICKS.fetch_add(1, Ordering::Relaxed);
}

/// Ticks observed since `init()`.
pub fn ticks() -> u64 {
    TICKS.load(Ordering::Relaxed)
}

/// The programmed tick rate, or 0 if `init()` hasn't run.
pub fn frequency_hz() -> u64 {
    HZ.load(Ordering::Relaxed)
}

/// Milliseconds elapsed since `init()`, derived from the tick count.
pub fn uptime_ms() -> u64 {
    let hz = frequency_hz();
    if hz == 0 {
        return 0;
    }
    ticks() * 1000 / hz
}

/// Wait (via `hlt`, so the CPU is actually idle) for a handful of ticks
/// to arrive and confirm they did. Call this only after
/// `idt::enable_interrupts()` has run.
pub fn self_test() {
    let hz = frequency_hz();
    if hz == 0 {
        log_fail!("PIT", "SelfTest", "Timer not initialized");
        return;
    }

    let start = ticks();
    let target_ticks = (hz / 20).max(1); // ~50ms worth of ticks
    let deadline = start + target_ticks;

    let mut spins = 0u64;
    while ticks() < deadline {
        unsafe { core::arch::asm!("hlt", options(nomem, nostack)) };
        spins += 1;
        if spins > 50_000_000 {
            log_fail!(
                "PIT",
                "SelfTest",
                "No timer interrupts after {} hlt cycles -- IRQ0 is not reaching the handler",
                spins
            );
            return;
        }
    }

    log_ok!(
        "PIT",
        "SelfTest",
        "{} timer interrupts observed via IRQ0 in {}ms (uptime {}ms)",
        ticks() - start,
        (ticks() - start) * 1000 / hz,
        uptime_ms()
    );
}