//! Basic framebuffer bring-up via the Limine boot protocol.

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

/// Query Limine for the primary framebuffer and stash it for later use.
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

/// Sanity-check the framebuffer by drawing a 32x32 white box in the top-left corner.
pub fn self_test() {
    let guard = FRAMEBUFFER.lock();
    let Some(fb) = guard.as_ref() else {
        log_fail!("Framebuffer", "SelfTest", "Framebuffer not initialized");
        return;
    };

    let bytes_per_pixel = (fb.bpp as usize) / 8;
    
    unsafe {
        for y in 0..32 {
            for x in 0..32 {
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