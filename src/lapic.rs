//! Local APIC (xAPIC/MMIO mode only).
//!
//! Every CPU core has its own Local APIC; it's what actually delivers
//! an interrupt to *this* core, whether that interrupt originated from
//! the I/O APIC (external devices), another core (an IPI, not used by
//! this single-core kernel yet), or the CPU itself (a local timer,
//! also not used here since `pit` remains the tick source). Enabling
//! it is a prerequisite for the I/O APIC to have anywhere to deliver
//! interrupts to, and it also becomes the new place to send End-Of-
//! Interrupt to, replacing `pic::end_of_interrupt`.
//!
//! **xAPIC (MMIO) mode only -- no x2APIC (MSR) mode support.** Modern
//! CPUs support both; x2APIC is strictly additive (a CPU capable of it
//! still boots in plain xAPIC mode and stays there until software
//! explicitly switches over), so this is a real simplification, not a
//! guess: it means this driver works unmodified on the large majority
//! of real hardware without needing to detect and branch between two
//! entirely different register-access mechanisms (MMIO loads/stores
//! for xAPIC, `rdmsr`/`wrmsr` for x2APIC) for a first working version.
//! `init()` checks for this case explicitly and fails loudly rather
//! than silently misprogramming an x2APIC-only machine.

#![allow(dead_code)]

use core::sync::atomic::{AtomicBool, AtomicU64, Ordering};

use crate::vmm;
use crate::{log_fail, log_ok};

/// Local APIC register offsets from its MMIO base (Intel SDM Vol 3A,
/// Table 10-1). Only the ones this driver touches.
const REG_ID: u32 = 0x020;
const REG_VERSION: u32 = 0x030;
const REG_EOI: u32 = 0x0B0;
const REG_SPURIOUS: u32 = 0x0F0;
const REG_LVT_TIMER: u32 = 0x320;
const REG_LVT_THERMAL: u32 = 0x330;
const REG_LVT_PERFORMANCE: u32 = 0x340;
const REG_LVT_LINT0: u32 = 0x350;
const REG_LVT_LINT1: u32 = 0x360;
const REG_LVT_ERROR: u32 = 0x370;

const SPURIOUS_APIC_ENABLE: u32 = 1 << 8;
const LVT_MASKED: u32 = 1 << 16;
/// Vector for the spurious-interrupt handler itself. Any unused vector
/// works; picked here to sit just past the legacy IRQ range this
/// kernel's IDT already reserves (`idt::IRQ_BASE` + `idt::IRQ_COUNT`),
/// so it can't collide with a real device vector.
pub const SPURIOUS_VECTOR: u8 = 0xFF;

static LAPIC_BASE: AtomicU64 = AtomicU64::new(0);
static ENABLED: AtomicBool = AtomicBool::new(false);
static APIC_ID: AtomicU64 = AtomicU64::new(0);

fn cpuid(leaf: u32) -> (u32, u32, u32, u32) {
    // `ebx` can't be named directly as an asm operand: LLVM reserves it
    // for its own use (it's the position-independent-code base
    // register) and refuses to compile `lateout("ebx")`/`out("ebx")`.
    // The standard workaround, used by `core::arch::x86_64::__cpuid`
    // itself, is to swap it into a scratch register the compiler picks
    // (`{0:e}`, any general-purpose 32-bit register) around the
    // instruction instead of asking for `ebx` by name.
    let eax_out: u32;
    let ebx_out: u32;
    let ecx_out: u32;
    let edx_out: u32;
    unsafe {
        core::arch::asm!(
            "mov {ebx_scratch:e}, ebx",
            "cpuid",
            "xchg {ebx_scratch:e}, ebx",
            ebx_scratch = out(reg) ebx_out,
            inout("eax") leaf => eax_out,
            out("ecx") ecx_out,
            out("edx") edx_out,
            options(nomem, nostack, preserves_flags),
        );
    }
    (eax_out, ebx_out, ecx_out, edx_out)
}

fn rdmsr(msr: u32) -> u64 {
    let lo: u32;
    let hi: u32;
    unsafe {
        core::arch::asm!("rdmsr", in("ecx") msr, out("eax") lo, out("edx") hi, options(nomem, nostack, preserves_flags));
    }
    ((hi as u64) << 32) | lo as u64
}

fn wrmsr(msr: u32, value: u64) {
    let lo = value as u32;
    let hi = (value >> 32) as u32;
    unsafe {
        core::arch::asm!("wrmsr", in("ecx") msr, in("eax") lo, in("edx") hi, options(nomem, nostack, preserves_flags));
    }
}

const IA32_APIC_BASE: u32 = 0x1B;
const APIC_BASE_EXTD: u64 = 1 << 10; // x2APIC enabled
const APIC_BASE_EN: u64 = 1 << 11; // APIC globally enabled
const APIC_BASE_ADDR_MASK: u64 = 0x0000_000F_FFFF_F000;

fn reg_read(reg: u32) -> u32 {
    let base = LAPIC_BASE.load(Ordering::Relaxed);
    unsafe { core::ptr::read_volatile((base + reg as u64) as *const u32) }
}
fn reg_write(reg: u32, val: u32) {
    let base = LAPIC_BASE.load(Ordering::Relaxed);
    unsafe { core::ptr::write_volatile((base + reg as u64) as *mut u32, val) };
}

/// Bring up this CPU's Local APIC in xAPIC mode. `madt_local_apic_addr`
/// is the address `acpi::find_madt` read out of the MADT, used only as
/// a fallback -- the `IA32_APIC_BASE` MSR is authoritative for where
/// *this* CPU's APIC actually is, and is what real firmware and other
/// kernels trust; the MADT's copy exists mainly for older, non-MSR-
/// aware software and can in principle be stale.
///
/// Returns `false` (having logged why) if the CPU has no APIC at all,
/// or if it's already running in x2APIC mode -- see the module docs
/// for why that case isn't handled rather than silently misprogrammed.
pub fn init(madt_local_apic_addr: u64) -> bool {
    let (_, _, ecx1, edx1) = cpuid(1);
    let has_apic = edx1 & (1 << 9) != 0;
    let has_x2apic = ecx1 & (1 << 21) != 0;

    if !has_apic {
        log_fail!("LAPIC", "Init", "CPUID reports no Local APIC on this CPU");
        return false;
    }

    let apic_base_msr = rdmsr(IA32_APIC_BASE);
    if apic_base_msr & APIC_BASE_EXTD != 0 {
        // Already in x2APIC mode -- per the SDM, this can only be
        // undone with a full system reset, so there's no "downgrade to
        // xAPIC and continue" path available here even though
        // `has_x2apic` being true doesn't necessarily mean this bit is
        // set (most firmware leaves CPUs in xAPIC mode at handoff).
        log_fail!(
            "LAPIC",
            "Init",
            "CPU is already in x2APIC mode, which this driver doesn't support -- falling back to the legacy PIC"
        );
        return false;
    }
    let _ = has_x2apic; // acknowledged, deliberately unused -- see module docs

    let msr_addr = apic_base_msr & APIC_BASE_ADDR_MASK;
    let phys_base = if msr_addr != 0 { msr_addr } else { madt_local_apic_addr };
    if phys_base == 0 {
        log_fail!("LAPIC", "Init", "No usable Local APIC address from either the MSR or the MADT");
        return false;
    }

    // Make sure the APIC global-enable bit is set (it almost always
    // already is at this point in boot -- firmware enables it before
    // handing off -- but there's no cost to confirming rather than
    // assuming).
    if apic_base_msr & APIC_BASE_EN == 0 {
        wrmsr(IA32_APIC_BASE, apic_base_msr | APIC_BASE_EN);
    }

    let virt_base = vmm::map_mmio(phys_base, 4096) as u64;
    LAPIC_BASE.store(virt_base, Ordering::Relaxed);

    // Firmware may leave local timer, thermal, performance, error or
    // legacy virtual-wire interrupts enabled with vectors the kernel
    // has not installed. Mask them; the PIT and external devices arrive
    // through the I/O APIC instead.
    let version = reg_read(REG_VERSION);
    let max_lvt = (version >> 16) & 0xff;
    reg_write(REG_LVT_TIMER, LVT_MASKED);
    if max_lvt >= 1 { reg_write(REG_LVT_THERMAL, LVT_MASKED); }
    if max_lvt >= 2 { reg_write(REG_LVT_PERFORMANCE, LVT_MASKED); }
    if max_lvt >= 3 { reg_write(REG_LVT_LINT0, LVT_MASKED); }
    if max_lvt >= 4 { reg_write(REG_LVT_LINT1, LVT_MASKED); }
    if max_lvt >= 5 { reg_write(REG_LVT_ERROR, LVT_MASKED); }

    // Software-enable the APIC and set the spurious-interrupt vector.
    // Bits 0-7 of the spurious register are the vector; bit 8 is the
    // software enable/disable flag covered in the module docs.
    reg_write(REG_SPURIOUS, (SPURIOUS_VECTOR as u32) | SPURIOUS_APIC_ENABLE);

    ENABLED.store(true, Ordering::Release);

    let id = reg_read(REG_ID) >> 24;
    APIC_ID.store(id as u64, Ordering::Release);
    let version = version & 0xFF;
    log_ok!(
        "LAPIC",
        "Init",
        "Local APIC enabled at {:#x} (ID {}, version {:#x}), spurious vector {:#x}",
        phys_base,
        id,
        version,
        SPURIOUS_VECTOR
    );

    true
}

/// Whether `init()` successfully enabled this CPU's Local APIC.
pub fn is_enabled() -> bool {
    ENABLED.load(Ordering::Acquire)
}

/// Hardware destination ID used in I/O APIC redirection entries.
pub fn id() -> u8 {
    APIC_ID.load(Ordering::Acquire) as u8
}

/// Acknowledge the current interrupt so the Local APIC will accept
/// another. Unlike the 8259 (`pic::end_of_interrupt`), there's no
/// separate master/slave distinction and no IRQ number needed -- one
/// write always acknowledges whatever's currently being serviced.
pub fn end_of_interrupt() {
    reg_write(REG_EOI, 0);
}
