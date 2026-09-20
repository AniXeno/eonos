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
//! Self-test: since a per-process address space doesn't exist yet
//! (that's the next step), the test maps a couple of user-accessible
//! pages straight into the kernel's own address space, drops a
//! dedicated thread into ring 3 to run a few hand-assembled
//! instructions there, and checks that the `syscall` those instructions
//! issue makes it back into the kernel with the right argument.

#![allow(dead_code)]

use core::sync::atomic::{AtomicBool, AtomicU64, Ordering};

use crate::{gdt, pit, pmm, scheduler, vmm};
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

#[no_mangle]
extern "C" fn syscall_dispatch(frame: &mut SyscallFrame) {
    match frame.rax {
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
            frame.rax = u64::MAX;
        }
    }
}

/// Drop from ring 0 into ring 3 at `entry`, running on `user_rsp`. Never
/// returns to the caller: either the user code eventually `syscall`s
/// into a handler that exits the thread, or (self-test only) it spins.
fn enter_user_mode(entry: u64, user_rsp: u64) -> ! {
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