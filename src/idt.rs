//! Interrupt Descriptor Table + CPU exception handling.
//!
//! Every exception vector (0-31) gets a small assembly stub that normalises
//! the stack (pushes a dummy error code where the CPU doesn't), pushes the
//! vector number and all general-purpose registers, then calls
//! `exception_handler` with a pointer to an `InterruptFrame`.
//!
//! Fatal exceptions print a full register dump to the screen and serial port
//! and halt. #BP (int3) is handled and returns, which makes a handy self-test.

use core::ptr::{addr_of, addr_of_mut};

use crate::gdt::{DOUBLE_FAULT_IST, KERNEL_CS};
use crate::{log_critical, log_debug, log_ok};

const IDT_ENTRIES: usize = 256;
const GATE_INTERRUPT: u8 = 0x8E; // present, DPL 0, 64-bit interrupt gate

#[derive(Clone, Copy)]
#[repr(C)]
struct Entry {
    offset_low: u16,
    selector: u16,
    ist: u8,
    type_attr: u8,
    offset_mid: u16,
    offset_high: u32,
    reserved: u32,
}

impl Entry {
    const MISSING: Entry = Entry {
        offset_low: 0,
        selector: 0,
        ist: 0,
        type_attr: 0,
        offset_mid: 0,
        offset_high: 0,
        reserved: 0,
    };

    fn new(handler: u64, selector: u16, ist: u8, type_attr: u8) -> Entry {
        Entry {
            offset_low: handler as u16,
            selector,
            ist,
            type_attr,
            offset_mid: (handler >> 16) as u16,
            offset_high: (handler >> 32) as u32,
            reserved: 0,
        }
    }
}

#[repr(C, packed)]
struct Idtr {
    limit: u16,
    base: u64,
}

static mut IDT: [Entry; IDT_ENTRIES] = [Entry::MISSING; IDT_ENTRIES];

/// Registers as laid out on the stack by the assembly stubs (lowest address first).
#[repr(C)]
pub struct InterruptFrame {
    pub r15: u64,
    pub r14: u64,
    pub r13: u64,
    pub r12: u64,
    pub r11: u64,
    pub r10: u64,
    pub r9: u64,
    pub r8: u64,
    pub rbp: u64,
    pub rdi: u64,
    pub rsi: u64,
    pub rdx: u64,
    pub rcx: u64,
    pub rbx: u64,
    pub rax: u64,
    pub vector: u64,
    pub error_code: u64,
    // pushed by the CPU:
    pub rip: u64,
    pub cs: u64,
    pub rflags: u64,
    pub rsp: u64,
    pub ss: u64,
}

const EXCEPTION_NAMES: [&str; 32] = [
    "Divide Error (#DE)",
    "Debug (#DB)",
    "Non-Maskable Interrupt (NMI)",
    "Breakpoint (#BP)",
    "Overflow (#OF)",
    "Bound Range Exceeded (#BR)",
    "Invalid Opcode (#UD)",
    "Device Not Available (#NM)",
    "Double Fault (#DF)",
    "Coprocessor Segment Overrun",
    "Invalid TSS (#TS)",
    "Segment Not Present (#NP)",
    "Stack-Segment Fault (#SS)",
    "General Protection Fault (#GP)",
    "Page Fault (#PF)",
    "Reserved",
    "x87 Floating-Point (#MF)",
    "Alignment Check (#AC)",
    "Machine Check (#MC)",
    "SIMD Floating-Point (#XM)",
    "Virtualization (#VE)",
    "Control Protection (#CP)",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
    "Hypervisor Injection (#HV)",
    "VMM Communication (#VC)",
    "Security Exception (#SX)",
    "Reserved",
];

// ---------------------------------------------------------------------------
// Assembly stubs
// ---------------------------------------------------------------------------

core::arch::global_asm!(r#"
.macro isr_noerr num
isr_stub_\num:
    push 0
    push \num
    jmp isr_common
.endm

.macro isr_err num
isr_stub_\num:
    push \num
    jmp isr_common
.endm

    isr_noerr 0
    isr_noerr 1
    isr_noerr 2
    isr_noerr 3
    isr_noerr 4
    isr_noerr 5
    isr_noerr 6
    isr_noerr 7
    isr_err 8
    isr_noerr 9
    isr_err 10
    isr_err 11
    isr_err 12
    isr_err 13
    isr_err 14
    isr_noerr 15
    isr_noerr 16
    isr_err 17
    isr_noerr 18
    isr_noerr 19
    isr_noerr 20
    isr_err 21
    isr_noerr 22
    isr_noerr 23
    isr_noerr 24
    isr_noerr 25
    isr_noerr 26
    isr_noerr 27
    isr_noerr 28
    isr_err 29
    isr_err 30
    isr_noerr 31

isr_common:
    push rax
    push rbx
    push rcx
    push rdx
    push rsi
    push rdi
    push rbp
    push r8
    push r9
    push r10
    push r11
    push r12
    push r13
    push r14
    push r15
    mov rdi, rsp
    cld
    call exception_handler
    pop r15
    pop r14
    pop r13
    pop r12
    pop r11
    pop r10
    pop r9
    pop r8
    pop rbp
    pop rdi
    pop rsi
    pop rdx
    pop rcx
    pop rbx
    pop rax
    add rsp, 16
    iretq

.section .rodata
.balign 8
.global isr_stub_table
isr_stub_table:
    .quad isr_stub_0
    .quad isr_stub_1
    .quad isr_stub_2
    .quad isr_stub_3
    .quad isr_stub_4
    .quad isr_stub_5
    .quad isr_stub_6
    .quad isr_stub_7
    .quad isr_stub_8
    .quad isr_stub_9
    .quad isr_stub_10
    .quad isr_stub_11
    .quad isr_stub_12
    .quad isr_stub_13
    .quad isr_stub_14
    .quad isr_stub_15
    .quad isr_stub_16
    .quad isr_stub_17
    .quad isr_stub_18
    .quad isr_stub_19
    .quad isr_stub_20
    .quad isr_stub_21
    .quad isr_stub_22
    .quad isr_stub_23
    .quad isr_stub_24
    .quad isr_stub_25
    .quad isr_stub_26
    .quad isr_stub_27
    .quad isr_stub_28
    .quad isr_stub_29
    .quad isr_stub_30
    .quad isr_stub_31
.section .text
"#);

#[allow(non_upper_case_globals)]
extern "C" {
    static isr_stub_table: [u64; 32];
}

// ---------------------------------------------------------------------------
// Rust side
// ---------------------------------------------------------------------------

pub fn init() {
    unsafe {
        let idt = addr_of_mut!(IDT) as *mut Entry;

        for vector in 0..32usize {
            let handler = isr_stub_table[vector];
            let ist = if vector == 8 { DOUBLE_FAULT_IST } else { 0 };
            idt.add(vector)
                .write(Entry::new(handler, KERNEL_CS, ist, GATE_INTERRUPT));
        }

        let idtr = Idtr {
            limit: (core::mem::size_of::<[Entry; IDT_ENTRIES]>() - 1) as u16,
            base: idt as u64,
        };
        core::arch::asm!(
            "lidt [{}]",
            in(reg) addr_of!(idtr),
            options(readonly, nostack, preserves_flags),
        );
    }

    log_ok!("IDT", "Init", "32 CPU exception handlers installed (double fault on its own stack)");
}

#[no_mangle]
extern "C" fn exception_handler(frame: &mut InterruptFrame) {
    match frame.vector {
        3 => {
            log_debug!("CPU", "Exception", "Breakpoint (#BP) at rip={:#018x}", frame.rip);
        }
        _ => fatal(frame),
    }
}

fn read_cr2() -> u64 {
    let v: u64;
    unsafe { core::arch::asm!("mov {}, cr2", out(reg) v, options(nomem, nostack, preserves_flags)) };
    v
}

fn read_cr3() -> u64 {
    let v: u64;
    unsafe { core::arch::asm!("mov {}, cr3", out(reg) v, options(nomem, nostack, preserves_flags)) };
    v
}

fn fatal(frame: &InterruptFrame) -> ! {
    // The fault may have happened while one of these locks was held (e.g. in
    // the middle of a log call). We're never returning, so break them open.
    unsafe {
        crate::logger::LOGGER.force_unlock();
        crate::serial::SERIAL1.force_unlock();
        crate::console::CONSOLE.force_unlock();
    }

    let name = EXCEPTION_NAMES
        .get(frame.vector as usize)
        .copied()
        .unwrap_or("Unknown");

    log_critical!(
        "CPU",
        "Exception",
        "{} - vector {}, error code {:#x}",
        name,
        frame.vector,
        frame.error_code
    );

    if frame.vector == 14 {
        let e = frame.error_code;
        log_critical!(
            "CPU",
            "PageFault",
            "address {:#018x}: {} on {}{}",
            read_cr2(),
            if e & 1 != 0 { "protection violation" } else { "page not present" },
            if e & 2 != 0 { "write" } else { "read" },
            if e & 16 != 0 { " (instruction fetch)" } else { "" }
        );
    }

    log_critical!(
        "CPU",
        "Regs",
        "RIP={:#018x} RSP={:#018x} RFLAGS={:#010x}",
        frame.rip,
        frame.rsp,
        frame.rflags
    );
    log_critical!(
        "CPU",
        "Regs",
        "CS={:#06x} SS={:#06x} CR2={:#018x} CR3={:#018x}",
        frame.cs,
        frame.ss,
        read_cr2(),
        read_cr3()
    );
    log_critical!(
        "CPU",
        "Regs",
        "RAX={:016x} RBX={:016x} RCX={:016x} RDX={:016x}",
        frame.rax,
        frame.rbx,
        frame.rcx,
        frame.rdx
    );
    log_critical!(
        "CPU",
        "Regs",
        "RSI={:016x} RDI={:016x} RBP={:016x} R8 ={:016x}",
        frame.rsi,
        frame.rdi,
        frame.rbp,
        frame.r8
    );
    log_critical!(
        "CPU",
        "Regs",
        "R9 ={:016x} R10={:016x} R11={:016x} R12={:016x}",
        frame.r9,
        frame.r10,
        frame.r11,
        frame.r12
    );
    log_critical!(
        "CPU",
        "Regs",
        "R13={:016x} R14={:016x} R15={:016x}",
        frame.r13,
        frame.r14,
        frame.r15
    );

    crate::hcf()
}