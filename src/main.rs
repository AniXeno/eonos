#![no_std]
#![no_main]

pub mod console;
pub mod framebuffer;
pub mod gdt;
pub mod idt;
pub mod logger;
pub mod pmm;
pub mod serial;

use core::panic::PanicInfo;
use limine::BaseRevision;

#[used]
#[link_section = ".requests"]
static BASE_REVISION: BaseRevision = BaseRevision::new();

#[used]
#[link_section = ".requests_start_marker"]
static _START_MARKER: limine::request::RequestsStartMarker = limine::request::RequestsStartMarker::new();

#[used]
#[link_section = ".requests_end_marker"]
static _END_MARKER: limine::request::RequestsEndMarker = limine::request::RequestsEndMarker::new();

#[no_mangle]
unsafe extern "C" fn _start() -> ! {
    serial::SERIAL1.lock().init();

    assert!(BASE_REVISION.is_supported());

    log_ok!("Kernel", "Init", "EonOS booting via Limine");

    framebuffer::init();
    framebuffer::self_test();
    console::init();

    gdt::init();
    idt::init();

    // Self-test: int3 -> #BP handler -> iretq back to here.
    core::arch::asm!("int3");
    log_ok!("IDT", "SelfTest", "Breakpoint exception handled and returned");

    // To see the crash screen, temporarily uncomment ONE of these:
    // core::arch::asm!("ud2");                                  // #UD invalid opcode
    // core::ptr::read_volatile(0xffff_8000_dead_0000 as *const u8); // #PF page fault

    pmm::init();
    pmm::self_test();

    // TODO: VMM, MEMORY subsystems go here.

    log_ok!("Kernel", "Init", "Initialization complete, halting");

    hcf();
}

fn hcf() -> ! {
    loop {
        unsafe {
            core::arch::asm!("cli; hlt", options(nomem, nostack));
        }
    }
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    log_critical!("Kernel", "Panic", "{}", info);
    hcf();
}