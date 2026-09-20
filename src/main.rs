#![no_std]
#![no_main]

extern crate alloc;

pub mod console;
pub mod framebuffer;
pub mod gdt;
pub mod heap;
pub mod idt;
pub mod logger;
pub mod panic_screen;
pub mod pic;
pub mod pit;
pub mod pmm;
pub mod serial;
pub mod vmm;

use core::fmt::Write;
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

    core::arch::asm!("int3");
    log_ok!("IDT", "SelfTest", "Breakpoint exception handled and returned");

    // Kernel Exceptions!
    // core::arch::asm!("ud2");                                      // #UD invalid opcode
    // core::ptr::read_volatile(0xffff_8000_dead_0000 as *const u8); // #PF page fault

    pmm::init();
    pmm::self_test();

    vmm::init();
    vmm::self_test();

    heap::init();
    heap::self_test();

    pic::init();
    pit::init(1000); // 1kHz tick, i.e. 1ms resolution
    idt::enable_interrupts();
    pit::self_test();

    // TODO: scheduler/processes go here.

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
    panic_screen::show("KERNEL PANIC", (90, 0, 110), |w| {
        let _ = writeln!(w, "{}", info);
        let _ = writeln!(w);
        let _ = writeln!(w, "Full details are on the serial log.");
    });
}