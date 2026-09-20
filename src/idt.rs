use core::fmt::Write;
use core::ptr::{addr_of, addr_of_mut};

use crate::gdt::{DOUBLE_FAULT_IST, KERNEL_CS};
use crate::sync::IrqMutex;
use crate::{log_critical, log_debug, log_fail, log_ok};

const IDT_ENTRIES: usize = 256;
const GATE_INTERRUPT: u8 = 0x8E;

/// Vector the remapped PIC's IRQ0 lands on (see `pic::IRQ_BASE`); IRQs
/// occupy vectors [IRQ_BASE, IRQ_BASE + IRQ_COUNT).
const IRQ_BASE: usize = 32;
const IRQ_COUNT: usize = 16;
const TOTAL_VECTORS: usize = IRQ_BASE + IRQ_COUNT;

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

/// A registered IRQ handler takes no arguments — drivers that need the
/// interrupt frame (none do yet) can register through a richer mechanism
/// later; a tick counter or "data ready" flag doesn't need one.
type IrqHandler = fn();

static IRQ_HANDLERS: IrqMutex<[Option<IrqHandler>; IRQ_COUNT]> = IrqMutex::new([None; IRQ_COUNT]);

/// Wire `handler` up to fire whenever IRQ `irq` (0-15, as delivered by
/// the PIC) arrives. Does not unmask the line — call `pic::unmask` too.
pub fn register_irq(irq: u8, handler: IrqHandler) {
    IRQ_HANDLERS.lock()[irq as usize] = Some(handler);
}

pub fn enable_interrupts() {
    unsafe { core::arch::asm!("sti", options(nomem, nostack)) };
}

pub fn disable_interrupts() {
    unsafe { core::arch::asm!("cli", options(nomem, nostack)) };
}

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

    isr_noerr 32
    isr_noerr 33
    isr_noerr 34
    isr_noerr 35
    isr_noerr 36
    isr_noerr 37
    isr_noerr 38
    isr_noerr 39
    isr_noerr 40
    isr_noerr 41
    isr_noerr 42
    isr_noerr 43
    isr_noerr 44
    isr_noerr 45
    isr_noerr 46
    isr_noerr 47

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
    .quad isr_stub_32
    .quad isr_stub_33
    .quad isr_stub_34
    .quad isr_stub_35
    .quad isr_stub_36
    .quad isr_stub_37
    .quad isr_stub_38
    .quad isr_stub_39
    .quad isr_stub_40
    .quad isr_stub_41
    .quad isr_stub_42
    .quad isr_stub_43
    .quad isr_stub_44
    .quad isr_stub_45
    .quad isr_stub_46
    .quad isr_stub_47
.section .text
"#);

#[allow(non_upper_case_globals)]
extern "C" {
    static isr_stub_table: [u64; TOTAL_VECTORS];
}

pub fn init() {
    unsafe {
        let idt = addr_of_mut!(IDT) as *mut Entry;

        for vector in 0..TOTAL_VECTORS {
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

    log_ok!(
        "IDT",
        "Init",
        "32 CPU exception handlers + 16 IRQ vectors ({}-{}) installed (double fault on its own stack)",
        IRQ_BASE,
        IRQ_BASE + IRQ_COUNT - 1
    );
}

#[no_mangle]
extern "C" fn exception_handler(frame: &mut InterruptFrame) {
    if frame.vector as usize >= IRQ_BASE {
        irq_dispatch(frame);
        return;
    }

    // A CPU exception taken while running user code is that process's
    // problem, not the kernel's: kill the process and carry on. NMI,
    // double fault and machine check are the exceptions to that -- they
    // say something is wrong with the machine (or with the kernel's own
    // handling), not merely with the program, and the double fault
    // handler is running on its dedicated IST stack anyway.
    if frame.cs & 3 == 3 && !matches!(frame.vector, 2 | 8 | 18) {
        kill_user_process(frame);
    }

    match frame.vector {
        3 => {
            log_debug!("CPU", "Exception", "Breakpoint (#BP) at rip={:#018x}", frame.rip);
        }
        _ => fatal(frame),
    }
}

/// Terminate the current (user) process after it caused a CPU
/// exception. Runs on the process's kernel stack, so ending the thread
/// from here is no different from ending it in a syscall. The exit
/// status follows the shell convention for death by signal, 128 + the
/// exception vector.
fn kill_user_process(frame: &InterruptFrame) -> ! {
    let name = EXCEPTION_NAMES
        .get(frame.vector as usize)
        .copied()
        .unwrap_or("Unknown");

    log_fail!(
        "CPU",
        "UserFault",
        "{} in user mode (pid {}) - rip={:#018x} rsp={:#018x} error code {:#x}",
        name,
        crate::scheduler::current_id(),
        frame.rip,
        frame.rsp,
        frame.error_code
    );
    if frame.vector == 14 {
        let e = frame.error_code;
        log_fail!(
            "CPU",
            "UserFault",
            "page fault at {:#018x}: {} on {}{}",
            read_cr2(),
            if e & 1 != 0 { "protection violation" } else { "page not present" },
            if e & 2 != 0 { "write" } else { "read" },
            if e & 16 != 0 { " (instruction fetch)" } else { "" }
        );
    }

    crate::process::exit_current(128 + frame.vector as i64)
}

fn irq_dispatch(frame: &InterruptFrame) {
    let irq = (frame.vector as usize - IRQ_BASE) as u8;

    let handler = IRQ_HANDLERS.lock()[irq as usize];
    match handler {
        Some(handler) => handler(),
        None => log_debug!("IRQ", "Unhandled", "IRQ{} fired with no registered handler", irq),
    }

    crate::pic::end_of_interrupt(irq);

    // Now that the PIC is free to deliver more interrupts, the scheduler
    // may switch to another thread from inside this handler.
    crate::scheduler::preempt_if_needed();
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
    crate::panic_screen::force_unlock_all();

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

    let vector = frame.vector;
    let error_code = frame.error_code;
    let rip = frame.rip;
    let rsp = frame.rsp;
    crate::panic_screen::show("KERNEL EXCEPTION", (130, 0, 0), |w| {
        let _ = writeln!(w, "{}", name);
        let _ = writeln!(w, "vector {}   error code {:#x}", vector, error_code);
        if vector == 14 {
            let cr2 = read_cr2();
            let _ = writeln!(w, "faulting address {:#018x}", cr2);
            if let Some(phys) = crate::vmm::try_translate(cr2) {
                let _ = writeln!(
                    w,
                    "(that address is mapped to phys {:#018x} -- likely a permission violation)",
                    phys
                );
            }
        }
        let _ = writeln!(w);
        let _ = writeln!(w, "RIP {:#018x}", rip);
        let _ = writeln!(w, "RSP {:#018x}", rsp);
        let _ = writeln!(w);
        let _ = writeln!(w, "Full register dump is on the serial log.");
    });
}