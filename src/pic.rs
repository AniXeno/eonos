//! 8259 Programmable Interrupt Controller (legacy dual-PIC).
//!
//! Firmware leaves the PIC mapped to vectors 0x08-0x0F / 0x70-0x77, which
//! collide head-on with the CPU exception vectors our IDT already owns.
//! `init()` remaps it out of the way (IRQ0-7 -> 32-39, IRQ8-15 -> 40-47)
//! and masks every line; individual drivers (the PIT, keyboard, etc.)
//! unmask only the lines they actually handle.

#![allow(dead_code)]

use crate::log_ok;

const PIC1_COMMAND: u16 = 0x20;
const PIC1_DATA: u16 = 0x21;
const PIC2_COMMAND: u16 = 0xA0;
const PIC2_DATA: u16 = 0xA1;

const ICW1_ICW4: u8 = 0x01; // ICW4 will be sent
const ICW1_INIT: u8 = 0x10; // start the initialization sequence
const ICW4_8086: u8 = 0x01; // 8086/88 mode, not 8080

const PIC_EOI: u8 = 0x20;

/// Vector the first PIC line (IRQ0) is remapped to. IRQ N lands on
/// vector `IRQ_BASE + N`.
pub const IRQ_BASE: u8 = 32;

unsafe fn outb(port: u16, val: u8) {
    core::arch::asm!("out dx, al", in("dx") port, in("al") val, options(nomem, nostack, preserves_flags));
}

unsafe fn inb(port: u16) -> u8 {
    let val: u8;
    core::arch::asm!("in al, dx", in("dx") port, out("al") val, options(nomem, nostack, preserves_flags));
    val
}

/// A write to the unused POST-code port 0x80 takes long enough to act as
/// a bus delay, which the real 8259 silicon needs between init command
/// words on some hardware.
unsafe fn io_wait() {
    outb(0x80, 0);
}

pub fn init() {
    unsafe {
        // ICW1: start initialization, expect an ICW4.
        outb(PIC1_COMMAND, ICW1_INIT | ICW1_ICW4);
        io_wait();
        outb(PIC2_COMMAND, ICW1_INIT | ICW1_ICW4);
        io_wait();

        // ICW2: vector offsets.
        outb(PIC1_DATA, IRQ_BASE);
        io_wait();
        outb(PIC2_DATA, IRQ_BASE + 8);
        io_wait();

        // ICW3: wire the cascade — slave PIC lives on the master's IRQ2.
        outb(PIC1_DATA, 0b0000_0100);
        io_wait();
        outb(PIC2_DATA, 0b0000_0010);
        io_wait();

        // ICW4: 8086 mode.
        outb(PIC1_DATA, ICW4_8086);
        io_wait();
        outb(PIC2_DATA, ICW4_8086);
        io_wait();

        // Mask every line. Drivers unmask what they need via `unmask()`.
        outb(PIC1_DATA, 0xFF);
        outb(PIC2_DATA, 0xFF);
    }

    log_ok!(
        "PIC",
        "Init",
        "8259 remapped: IRQ0-7 -> vectors {}-{}, IRQ8-15 -> {}-{} (all masked)",
        IRQ_BASE,
        IRQ_BASE + 7,
        IRQ_BASE + 8,
        IRQ_BASE + 15
    );
}

/// Allow IRQ `irq` (0-15) to reach the CPU.
pub fn unmask(irq: u8) {
    unsafe {
        if irq < 8 {
            let mask = inb(PIC1_DATA) & !(1 << irq);
            outb(PIC1_DATA, mask);
        } else {
            let mask = inb(PIC2_DATA) & !(1 << (irq - 8));
            outb(PIC2_DATA, mask);
            // The cascade line (master IRQ2) has to stay open or nothing
            // from the slave PIC ever reaches the CPU.
            let master_mask = inb(PIC1_DATA) & !(1 << 2);
            outb(PIC1_DATA, master_mask);
        }
    }
}

/// Stop IRQ `irq` (0-15) from reaching the CPU.
pub fn mask(irq: u8) {
    unsafe {
        if irq < 8 {
            let mask = inb(PIC1_DATA) | (1 << irq);
            outb(PIC1_DATA, mask);
        } else {
            let mask = inb(PIC2_DATA) | (1 << (irq - 8));
            outb(PIC2_DATA, mask);
        }
    }
}

/// Acknowledge IRQ `irq` (0-15) so the PIC will deliver further
/// interrupts. Must be called once per IRQ, after handling it.
pub fn end_of_interrupt(irq: u8) {
    unsafe {
        if irq >= 8 {
            outb(PIC2_COMMAND, PIC_EOI);
        }
        outb(PIC1_COMMAND, PIC_EOI);
    }
}