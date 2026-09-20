use limine::request::FramebufferRequest;
use spin::Mutex;

use crate::{log_fail, log_ok};

#[used]
#[link_section = ".requests"]
static FRAMEBUFFER_REQUEST: FramebufferRequest = FramebufferRequest::new();

pub struct FbInfo {
    pub addr: *mut u8,
    pub width: u64,
    pub height: u64,
    pub pitch: u64,
    pub bpp: u16,
}

unsafe impl Send for FbInfo {}

pub static FRAMEBUFFER: Mutex<Option<FbInfo>> = Mutex::new(None);

pub fn init() {
    let Some(response) = FRAMEBUFFER_REQUEST.get_response() else {
        log_fail!("Framebuffer", "Init", "No response from bootloader");
        return;
    };

    let Some(fb) = response.framebuffers().next() else {
        log_fail!("Framebuffer", "Init", "No framebuffers reported");
        return;
    };

    let info = FbInfo {
        addr: fb.addr(),
        width: fb.width(),
        height: fb.height(),
        pitch: fb.pitch(),
        bpp: fb.bpp(),
    };

    log_ok!(
        "Framebuffer",
        "Init",
        "Framebuffer Initialized ({}x{} @ {}bpp)",
        info.width,
        info.height,
        info.bpp
    );

    *FRAMEBUFFER.lock() = Some(info);
}

pub fn self_test() {
    let guard = FRAMEBUFFER.lock();
    let Some(fb) = guard.as_ref() else {
        log_fail!("Framebuffer", "SelfTest", "Framebuffer not initialized");
        return;
    };

    let bytes_per_pixel = (fb.bpp as usize) / 8;
    
    unsafe {
        for y in 0..8 {
            for x in 0..8 {
                let offset = y * (fb.pitch as usize) + x * bytes_per_pixel;
                let ptr = fb.addr.add(offset);
                ptr.write_volatile(0xFF);
                ptr.add(1).write_volatile(0xFF);
                ptr.add(2).write_volatile(0xFF);
            }
        }
    }

    crate::log_ok!("Framebuffer", "SelfTest", "Self-test block drawn successfully");
}

unsafe fn rdtsc() -> u64 {
    let lo: u32;
    let hi: u32;
    core::arch::asm!("rdtsc", out("eax") lo, out("edx") hi, options(nomem, nostack));
    ((hi as u64) << 32) | lo as u64
}

/// Time-stamp-counter cycles spent on a burst of pixel writes, purely as
/// a relative before/after signal for the framebuffer's memory type
/// (see `vmm::configure_pat`/`vmm::KERNEL_WC`). Not a calibrated
/// wall-clock measurement — just compare the two numbers this logs
/// during boot (pre-VMM vs. post-VMM) against each other.
pub fn bench(label: &str) {
    let guard = FRAMEBUFFER.lock();
    let Some(fb) = guard.as_ref() else {
        log_fail!("Framebuffer", "Bench", "Framebuffer not initialized");
        return;
    };

    let bytes_per_pixel = (fb.bpp as usize) / 8;
    let width = fb.width as usize;
    let height = fb.height as usize;
    const N: usize = 4096;

    // A row well below the self-test block and comfortably inside the
    // screen even on tiny resolutions, so this never touches memory
    // outside the framebuffer.
    let y0 = 16.min(height.saturating_sub(1));

    unsafe {
        let start = rdtsc();
        for i in 0..N {
            let x = i % width;
            let y = y0 + i / width;
            if y >= height {
                break;
            }
            let offset = y * (fb.pitch as usize) + x * bytes_per_pixel;
            let ptr = fb.addr.add(offset);
            ptr.write_volatile(0x55);
            ptr.add(1).write_volatile(0x55);
            ptr.add(2).write_volatile(0x55);
        }
        let end = rdtsc();
        let cycles = end - start;

        crate::log_ok!(
            "Framebuffer",
            "Bench",
            "{}: {} pixel writes in {} cycles ({} cycles/pixel)",
            label,
            N,
            cycles,
            cycles / N as u64
        );
    }
}