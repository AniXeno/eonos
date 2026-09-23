//! ACPI-configured I/O APICs for routing legacy ISA IRQs to this CPU.

use crate::acpi::{IsaOverride, MadtInfo};
use crate::sync::IrqMutex;
use crate::{idt, lapic, vmm};
use crate::{log_fail, log_ok};

const REG_ID: u32 = 0x00;
const REG_VERSION: u32 = 0x01;
const REG_REDIRECTION_BASE: u32 = 0x10;
const REDIR_MASKED: u32 = 1 << 16;
const REDIR_ACTIVE_LOW: u32 = 1 << 13;
const REDIR_LEVEL_TRIGGERED: u32 = 1 << 15;

#[derive(Clone, Copy)]
struct IoApic {
    base: u64,
    gsi_start: u32,
    gsi_end: u32,
}

#[derive(Clone, Copy)]
struct Config {
    controllers: [IoApic; 4],
    count: usize,
    overrides: [IsaOverride; 16],
    override_count: usize,
}

const EMPTY: IoApic = IoApic {
    base: 0,
    gsi_start: 0,
    gsi_end: 0,
};
static CONFIG: IrqMutex<Option<Config>> = IrqMutex::new(None);

fn read_reg(base: u64, reg: u32) -> u32 {
    unsafe {
        core::ptr::write_volatile(base as *mut u32, reg);
        core::ptr::read_volatile((base + 0x10) as *const u32)
    }
}

fn write_reg(base: u64, reg: u32, value: u32) {
    unsafe {
        core::ptr::write_volatile(base as *mut u32, reg);
        core::ptr::write_volatile((base + 0x10) as *mut u32, value);
    }
}

fn redirection_read(io: IoApic, index: u32) -> (u32, u32) {
    let reg = REG_REDIRECTION_BASE + index * 2;
    (read_reg(io.base, reg), read_reg(io.base, reg + 1))
}

fn redirection_write(io: IoApic, index: u32, low: u32, high: u32) {
    let reg = REG_REDIRECTION_BASE + index * 2;
    // Mask before changing the destination/vector; then commit the new
    // low dword last so a partially updated entry cannot deliver IRQs.
    write_reg(io.base, reg, low | REDIR_MASKED);
    write_reg(io.base, reg + 1, high);
    write_reg(io.base, reg, low);
}

/// Map and mask all I/O APIC inputs. Actual ISA lines are enabled later,
/// after drivers register their handlers.
pub fn init(madt: &MadtInfo) -> bool {
    if madt.ioapic_count == 0 || madt.ioapic_count > 4 {
        log_fail!("IOAPIC", "Init", "MADT has no usable I/O APIC entries");
        return false;
    }

    let mut config = Config {
        controllers: [EMPTY; 4],
        count: 0,
        overrides: madt.overrides,
        override_count: madt.override_count,
    };

    for info in &madt.ioapics[..madt.ioapic_count] {
        if info.mmio_base == 0 {
            log_fail!("IOAPIC", "Init", "I/O APIC {} has a zero MMIO base", info.id);
            return false;
        }
        let base = vmm::map_mmio(info.mmio_base as u64, 0x20) as u64;
        let version = read_reg(base, REG_VERSION);
        let redir_count = ((version >> 16) & 0xff) + 1;
        let Some(gsi_end) = info.gsi_base.checked_add(redir_count) else {
            log_fail!("IOAPIC", "Init", "I/O APIC GSI range overflow");
            return false;
        };
        if config.controllers[..config.count].iter().any(|c| {
            info.gsi_base < c.gsi_end && c.gsi_start < gsi_end
        }) {
            log_fail!("IOAPIC", "Init", "Overlapping I/O APIC GSI ranges");
            return false;
        }
        let io = IoApic { base, gsi_start: info.gsi_base, gsi_end };
        for index in 0..redir_count {
            let (low, high) = redirection_read(io, index);
            redirection_write(io, index, low | REDIR_MASKED, high);
        }
        let id = read_reg(base, REG_ID) >> 24;
        log_ok!("IOAPIC", "Init", "I/O APIC id {} at {:#x}, GSIs {}..{}", id, info.mmio_base, info.gsi_base, gsi_end - 1);
        config.controllers[config.count] = io;
        config.count += 1;
    }

    *CONFIG.lock() = Some(config);
    true
}

fn isa_route(config: &Config, irq: u8) -> Option<(IoApic, u32, u16)> {
    let override_info = config.overrides[..config.override_count]
        .iter()
        .find(|entry| entry.isa_irq == irq)
        .copied();
    let (gsi, flags) = override_info
        .map(|IsaOverride { gsi, flags, .. }| (gsi, flags))
        .unwrap_or((irq as u32, 0));
    let io = config.controllers[..config.count]
        .iter()
        .copied()
        .find(|io| gsi >= io.gsi_start && gsi < io.gsi_end)?;
    Some((io, gsi - io.gsi_start, flags))
}

fn polarity_trigger(flags: u16) -> Option<u32> {
    let polarity = flags & 0b11;
    let trigger = (flags >> 2) & 0b11;
    if polarity == 0b10 || trigger == 0b10 {
        return None;
    }
    let mut bits = 0;
    if polarity == 0b11 { bits |= REDIR_ACTIVE_LOW; }
    if trigger == 0b11 { bits |= REDIR_LEVEL_TRIGGERED; }
    Some(bits)
}

/// Check that an ISA line maps to a present I/O APIC input and has
/// valid ACPI polarity/trigger flags before enabling APIC mode.
pub fn supports_isa_irq(irq: u8) -> bool {
    let guard = CONFIG.lock();
    guard
        .as_ref()
        .and_then(|config| isa_route(config, irq))
        .and_then(|(_, _, flags)| polarity_trigger(flags))
        .is_some()
}

fn set_isa_irq(irq: u8, masked: bool) {
    // Keep the lock through the selector/window register pair so two
    // callers can never interleave I/O APIC register accesses.
    let guard = CONFIG.lock();
    let Some(config) = guard.as_ref() else {
        log_fail!("IOAPIC", "Route", "I/O APIC is not initialized");
        return;
    };
    let Some((io, index, flags)) = isa_route(config, irq) else {
        log_fail!("IOAPIC", "Route", "No I/O APIC covers ISA IRQ{}", irq);
        return;
    };
    let Some(mode_bits) = polarity_trigger(flags) else {
        log_fail!("IOAPIC", "Route", "Reserved polarity/trigger flags for ISA IRQ{}", irq);
        return;
    };
    let (mut low, _) = redirection_read(io, index);
    // Fixed delivery to a physical APIC ID is represented by zeroes in
    // delivery-mode and destination-mode fields. Clear firmware values
    // there so a stale logical/lowest-priority route cannot survive.
    low &= !(0x0fff | REDIR_ACTIVE_LOW | REDIR_LEVEL_TRIGGERED | REDIR_MASKED);
    low |= idt::IRQ_BASE as u32 + irq as u32;
    low |= mode_bits;
    if masked { low |= REDIR_MASKED; }
    else { low &= !REDIR_MASKED; }
    let destination = (lapic::id() as u32) << 24;
    redirection_write(io, index, low, destination);
}

pub fn unmask_isa_irq(irq: u8) { set_isa_irq(irq, false); }
pub fn mask_isa_irq(irq: u8) { set_isa_irq(irq, true); }
