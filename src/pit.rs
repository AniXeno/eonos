//! Intel 8253/8254 Programmable Interval Timer, driven off IRQ0.
//!
//! This is the kernel's first real hardware interrupt (as opposed to a
//! CPU-generated exception like the `int3` self-test): it proves the
//! IDT, the interrupt controller, and `sti` all actually work together, and it
//! gives the future scheduler a tick source.

#![allow(dead_code)]

use core::sync::atomic::{AtomicU64, Ordering};

use crate::interrupts;
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
    interrupts::unmask_isa_irq(IRQ_LINE);

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
    // Let the scheduler decide whether to preempt (the actual switch
    // happens after the EOI, in `idt::irq_dispatch`).
    crate::scheduler::tick();
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

/// Wait for a handful of ticks to arrive and confirm they did. Call this
/// only after `idt::enable_interrupts()` has run.
///
/// Spins on `pause`, not `hlt`: `hlt` only returns because *some*
/// interrupt arrived, which is exactly the assumption this test exists
/// to check -- on real hardware where firmware never programmed the
/// legacy 8259 (increasingly common on UEFI-only machines, which
/// expect an IOAPIC-aware OS instead), IRQ0 can simply never arrive,
/// and `hlt` would then block forever with nothing left to wake it,
/// turning a should-fail self-test into a silent hang. The timeout is
/// instead measured with `rdtsc`, which free-runs regardless of
/// interrupt delivery, so "no ticks showed up" is reliably detected
/// and reported rather than hanging boot.
pub fn self_test() {
    let hz = frequency_hz();
    if hz == 0 {
        log_fail!("PIT", "SelfTest", "Timer not initialized");
        return;
    }

    let start = ticks();
    let target_ticks = (hz / 20).max(1); // ~50ms worth of ticks
    let deadline = start + target_ticks;

    let tsc_start = rdtsc();
    // No calibrated TSC frequency exists yet this early in boot, so
    // this timeout is deliberately generous rather than precise: 2^33
    // cycles is on the order of several seconds even on a very slow
    // CPU (at 1 GHz; real CPUs are far faster), comfortably longer than
    // 50ms of real ticks could ever take to arrive if IRQ0 works at
    // all, while still bounded so a genuinely dead PIC reports failure
    // in a few seconds instead of hanging boot indefinitely.
    let tsc_timeout = 1u64 << 33;

    while ticks() < deadline {
        unsafe { core::arch::asm!("pause", options(nomem, nostack)) };
        if rdtsc().wrapping_sub(tsc_start) > tsc_timeout {
            log_fail!(
                "PIT",
                "SelfTest",
                "No timer interrupts after ~{} TSC cycles -- IRQ0 is not reaching the handler \
                 (common on UEFI-only hardware that never programmed the legacy PIC; this kernel \
                 has no IOAPIC support yet, so the timer and any other legacy-IRQ device will be \
                 unusable on this machine)",
                tsc_timeout
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

fn rdtsc() -> u64 {
    let lo: u32;
    let hi: u32;
    unsafe {
        core::arch::asm!("rdtsc", out("eax") lo, out("edx") hi, options(nomem, nostack, preserves_flags));
    }
    ((hi as u64) << 32) | lo as u64
}
