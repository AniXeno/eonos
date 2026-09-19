//! Global Descriptor Table + Task State Segment.
//!
//! Layout (selectors):
//!   0x00  null
//!   0x08  kernel code (64-bit)
//!   0x10  kernel data
//!   0x18  TSS (16-byte system descriptor, occupies 0x18 and 0x20)
//!
//! The TSS provides IST1: a dedicated stack for the double-fault handler, so a
//! kernel stack overflow produces a readable crash screen instead of a
//! silent triple fault.

use core::mem::size_of;
use core::ptr::{addr_of, addr_of_mut};

use crate::log_ok;

pub const KERNEL_CS: u16 = 0x08;
pub const KERNEL_DS: u16 = 0x10;
const TSS_SELECTOR: u16 = 0x18;

/// IST slot (1-based) used by the double-fault handler.
pub const DOUBLE_FAULT_IST: u8 = 1;

const DOUBLE_FAULT_STACK_SIZE: usize = 4096 * 5;

#[repr(C, packed)]
struct Tss {
    reserved0: u32,
    rsp0: u64,
    rsp1: u64,
    rsp2: u64,
    reserved1: u64,
    ist1: u64,
    ist2: u64,
    ist3: u64,
    ist4: u64,
    ist5: u64,
    ist6: u64,
    ist7: u64,
    reserved2: u64,
    reserved3: u16,
    iomap_base: u16,
}

#[repr(C, packed)]
struct Gdtr {
    limit: u16,
    base: u64,
}

#[repr(C, align(16))]
struct Stack([u8; DOUBLE_FAULT_STACK_SIZE]);

static mut GDT: [u64; 5] = [0; 5];

static mut TSS: Tss = Tss {
    reserved0: 0,
    rsp0: 0,
    rsp1: 0,
    rsp2: 0,
    reserved1: 0,
    ist1: 0,
    ist2: 0,
    ist3: 0,
    ist4: 0,
    ist5: 0,
    ist6: 0,
    ist7: 0,
    reserved2: 0,
    reserved3: 0,
    iomap_base: 0,
};

static mut DOUBLE_FAULT_STACK: Stack = Stack([0; DOUBLE_FAULT_STACK_SIZE]);

/// Build the two 64-bit halves of a 64-bit available-TSS descriptor.
fn tss_descriptor(base: u64, limit: u32) -> (u64, u64) {
    let low = (limit as u64 & 0xFFFF)
        | ((base & 0xFF_FFFF) << 16)
        | (0x89u64 << 40) // present, DPL 0, type 0x9 = available 64-bit TSS
        | (((limit as u64 >> 16) & 0xF) << 48)
        | (((base >> 24) & 0xFF) << 56);
    let high = base >> 32;
    (low, high)
}

pub fn init() {
    unsafe {
        // --- TSS: point IST1 at the top of the double-fault stack ---
        let stack_top = addr_of!(DOUBLE_FAULT_STACK) as u64 + DOUBLE_FAULT_STACK_SIZE as u64;
        let tss = addr_of_mut!(TSS);
        (*tss).ist1 = stack_top;
        (*tss).iomap_base = size_of::<Tss>() as u16; // no I/O permission bitmap

        // --- GDT entries ---
        let (tss_low, tss_high) = tss_descriptor(tss as u64, (size_of::<Tss>() - 1) as u32);
        let gdt = addr_of_mut!(GDT) as *mut u64;
        gdt.add(0).write(0); // null
        gdt.add(1).write(0x00AF_9A00_0000_FFFF); // kernel code, 64-bit
        gdt.add(2).write(0x00CF_9200_0000_FFFF); // kernel data
        gdt.add(3).write(tss_low);
        gdt.add(4).write(tss_high);

        let gdtr = Gdtr {
            limit: (size_of::<[u64; 5]>() - 1) as u16,
            base: gdt as u64,
        };

        // --- load GDT, reload CS via far return, reload data segments ---
        core::arch::asm!(
            "lgdt [{gdtr}]",
            "push {cs}",
            "lea {tmp}, [rip + 2f]",
            "push {tmp}",
            "retfq",
            "2:",
            "mov ds, {sel:x}",
            "mov es, {sel:x}",
            "mov fs, {sel:x}",
            "mov gs, {sel:x}",
            "mov ss, {sel:x}",
            gdtr = in(reg) addr_of!(gdtr),
            cs = const KERNEL_CS,
            sel = in(reg) KERNEL_DS,
            tmp = out(reg) _,
        );

        // --- load the task register ---
        core::arch::asm!(
            "ltr {sel:x}",
            sel = in(reg) TSS_SELECTOR,
            options(nostack, preserves_flags),
        );
    }

    log_ok!(
        "GDT",
        "Init",
        "GDT + TSS loaded (cs={:#04x}, ds={:#04x}, tss={:#04x}, IST{} = double fault)",
        KERNEL_CS,
        KERNEL_DS,
        TSS_SELECTOR,
        DOUBLE_FAULT_IST
    );
}