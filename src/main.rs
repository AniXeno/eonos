#![no_std]
#![no_main]

extern crate alloc;

pub mod blocking;
pub mod block;
pub mod console;
pub mod drivers;
pub mod elf;
pub mod fat32;
pub mod framebuffer;
pub mod gdt;
pub mod heap;
pub mod idt;
pub mod initramfs;
pub mod logger;
pub mod panic_screen;
pub mod pic;
pub mod pit;
pub mod pmm;
pub mod process;
pub mod scheduler;
pub mod serial;
pub mod sync;
pub mod syscall;
pub mod vmm;
pub mod vfs;

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
    framebuffer::bench("pre-VMM, bootloader mapping");
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
    framebuffer::bench("post-VMM, write-combining");

    heap::init();
    heap::self_test();

    pic::init();
    pit::init(1000); // 1kHz tick, i.e. 1ms resolution
    idt::enable_interrupts();
    pit::self_test();

    // Keyboard drivers need the PIC/IDT (for IRQ1) already up, which
    // they are by this point, but come before the scheduler so a typed
    // command doesn't have to wait on anything else finishing init.
    drivers::ps2::init();
    drivers::usb::init();

    scheduler::init();
    scheduler::self_test();

    blocking::self_test();

    syscall::init();
    syscall::self_test();

    vmm::address_space_self_test();

    // The initramfs is a Limine module reached through the direct map, so
    // this has to come after the VMM is up.
    initramfs::init();
    initramfs::self_test();
    vfs::init();
    fat32::mount_boot_volume();

    process::self_test();
    process::start_init();

    log_ok!("Kernel", "Init", "Initialization complete, handing over to the scheduler");

    // The boot thread is done; the idle thread takes over from here.
    scheduler::exit();
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
