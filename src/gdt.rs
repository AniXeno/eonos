use core::mem::size_of;
use core::ptr::{addr_of, addr_of_mut};

use crate::log_ok;

pub const KERNEL_CS: u16 = 0x08;
pub const KERNEL_DS: u16 = 0x10;
/// Ring-3 data segment (base value; OR in RPL 3 to actually use it).
/// Must sit immediately before `USER_CS` -- `syscall::init` relies on
/// `USER_DS - 8` being a valid base for the `SYSRET` half of `STAR`.
pub const USER_DS: u16 = 0x18;
/// Ring-3 64-bit code segment (base value; OR in RPL 3 to actually use it).
pub const USER_CS: u16 = 0x20;
const TSS_SELECTOR: u16 = 0x28;

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

static mut GDT: [u64; 7] = [0; 7];

/// Mirror of `TSS.rsp0`, readable by name from the raw asm `syscall`
/// entry stub in `syscall.rs`. `SYSCALL` does not consult the TSS at
/// all (unlike a hardware interrupt/exception, which loads RSP from
/// `TSS.rsp0` automatically on a ring 3 -> ring 0 transition), so the
/// entry stub has to fish the current thread's kernel stack out of
/// somewhere itself; this static is that somewhere. Kept in sync with
/// `TSS.rsp0` by `set_kernel_stack`, which the scheduler calls on every
/// context switch, so both mechanisms always agree on "the current
/// thread's kernel stack".
#[no_mangle]
#[allow(dead_code)] // written here, read only from the raw asm in syscall.rs
static mut TSS_RSP0: u64 = 0;

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

fn tss_descriptor(base: u64, limit: u32) -> (u64, u64) {
    let low = (limit as u64 & 0xFFFF)
        | ((base & 0xFF_FFFF) << 16)
        | (0x89u64 << 40) 
        | (((limit as u64 >> 16) & 0xF) << 48)
        | (((base >> 24) & 0xFF) << 56);
    let high = base >> 32;
    (low, high)
}

pub fn init() {
    unsafe {
        let stack_top = addr_of!(DOUBLE_FAULT_STACK) as u64 + DOUBLE_FAULT_STACK_SIZE as u64;
        let tss = addr_of_mut!(TSS);
        (*tss).ist1 = stack_top;
        (*tss).iomap_base = size_of::<Tss>() as u16; 

        let (tss_low, tss_high) = tss_descriptor(tss as u64, (size_of::<Tss>() - 1) as u32);
        let gdt = addr_of_mut!(GDT) as *mut u64;
        gdt.add(0).write(0);
        gdt.add(1).write(0x00AF_9A00_0000_FFFF); // kernel code (0x08)
        gdt.add(2).write(0x00CF_9200_0000_FFFF); // kernel data (0x10)
        gdt.add(3).write(0x00CF_F200_0000_FFFF); // user data, DPL 3 (0x18)
        gdt.add(4).write(0x00AF_FA00_0000_FFFF); // user code, DPL 3, long mode (0x20)
        gdt.add(5).write(tss_low); // TSS (0x28)
        gdt.add(6).write(tss_high);

        let gdtr = Gdtr {
            limit: (size_of::<[u64; 7]>() - 1) as u16,
            base: gdt as u64,
        };

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

        core::arch::asm!(
            "ltr {sel:x}",
            sel = in(reg) TSS_SELECTOR,
            options(nostack, preserves_flags),
        );
    }

    log_ok!(
        "GDT",
        "Init",
        "GDT + TSS loaded (cs={:#04x}, ds={:#04x}, user_cs={:#04x}, user_ds={:#04x}, tss={:#04x}, IST{} = double fault)",
        KERNEL_CS,
        KERNEL_DS,
        USER_CS,
        USER_DS,
        TSS_SELECTOR,
        DOUBLE_FAULT_IST
    );
}

/// Point both `TSS.rsp0` and its `syscall`-entry mirror at `rsp0`, the
/// top of the kernel stack the CPU should switch to the next time this
/// thread traps into ring 0 (via interrupt/exception *or* `syscall`).
/// The scheduler calls this on every context switch so it's always the
/// stack of whichever thread is about to run.
pub fn set_kernel_stack(rsp0: u64) {
    unsafe {
        (*addr_of_mut!(TSS)).rsp0 = rsp0;
        TSS_RSP0 = rsp0;
    }
}