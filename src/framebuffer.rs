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

/// Replace the bootloader framebuffer with the backing memory managed by a
/// native display driver. Existing renderer users keep the same framebuffer
/// interface and begin presenting through the driver's flush path.
pub fn use_gpu_backing(addr: *mut u8, width: u64, height: u64, pitch: u64) {
    *FRAMEBUFFER.lock() = Some(FbInfo { addr, width, height, pitch, bpp: 32 });
}

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

    if fb.width < 8 || fb.height < 8 || (fb.bpp != 24 && fb.bpp != 32) {
        log_fail!("Framebuffer", "SelfTest", "Framebuffer is too small or has unsupported pixel format");
        return;
    }

    let bytes_per_pixel = (fb.bpp as usize) / 8;
    let mut original = [0u32; 64];

    unsafe {
        let mut row = fb.addr;
        for y in 0..8 {
            let mut ptr = row;
            for x in 0..8 {
                let i = y * 8 + x;
                original[i] = ptr.read_volatile() as u32
                    | ((ptr.add(1).read_volatile() as u32) << 8)
                    | ((ptr.add(2).read_volatile() as u32) << 16);
                ptr.write_volatile(0xFF);
                ptr.add(1).write_volatile(0xFF);
                ptr.add(2).write_volatile(0xFF);
                ptr = ptr.add(bytes_per_pixel);
            }
            row = row.add(fb.pitch as usize);
        }

        // A diagnostic must not leave its test pattern on the user's
        // framebuffer. Restore the original RGB bytes immediately.
        row = fb.addr;
        for y in 0..8 {
            let mut ptr = row;
            for x in 0..8 {
                let rgb = original[y * 8 + x];
                ptr.write_volatile(rgb as u8);
                ptr.add(1).write_volatile((rgb >> 8) as u8);
                ptr.add(2).write_volatile((rgb >> 16) as u8);
                ptr = ptr.add(bytes_per_pixel);
            }
            row = row.add(fb.pitch as usize);
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
    if (fb.bpp != 24 && fb.bpp != 32) || width == 0 || height == 0 {
        log_fail!("Framebuffer", "Bench", "Unsupported framebuffer format or dimensions");
        return;
    }
    let row_bytes = width.saturating_mul(bytes_per_pixel);
    if (fb.pitch as usize) < row_bytes {
        log_fail!("Framebuffer", "Bench", "Framebuffer pitch is smaller than a pixel row");
        return;
    }
    const N: usize = 4096;

    // A row well below the self-test block and comfortably inside the
    // screen even on tiny resolutions, so this never touches memory
    // outside the framebuffer.
    let y0 = 16.min(height.saturating_sub(1));
    let writes = N.min(height.saturating_sub(y0).saturating_mul(width));
    let mut original = [0u32; N];

    unsafe {
        // Save the pixels before timing so both the benchmark and its
        // cleanup leave the displayed image unchanged.
        let mut offset = y0 * fb.pitch as usize;
        let mut x = 0usize;
        for i in 0..writes {
            let ptr = fb.addr.add(offset);
            original[i] = ptr.read_volatile() as u32
                | ((ptr.add(1).read_volatile() as u32) << 8)
                | ((ptr.add(2).read_volatile() as u32) << 16);
            offset += bytes_per_pixel;
            x += 1;
            if x == width { x = 0; offset += fb.pitch as usize - row_bytes; }
        }

        let start = rdtsc();
        offset = y0 * fb.pitch as usize;
        x = 0;
        for _ in 0..writes {
            let ptr = fb.addr.add(offset);
            ptr.write_volatile(0x55);
            ptr.add(1).write_volatile(0x55);
            ptr.add(2).write_volatile(0x55);
            offset += bytes_per_pixel;
            x += 1;
            if x == width { x = 0; offset += fb.pitch as usize - row_bytes; }
        }
        let end = rdtsc();
        let cycles = end - start;

        offset = y0 * fb.pitch as usize;
        x = 0;
        for i in 0..writes {
            let ptr = fb.addr.add(offset);
            let rgb = original[i];
            ptr.write_volatile(rgb as u8);
            ptr.add(1).write_volatile((rgb >> 8) as u8);
            ptr.add(2).write_volatile((rgb >> 16) as u8);
            offset += bytes_per_pixel;
            x += 1;
            if x == width { x = 0; offset += fb.pitch as usize - row_bytes; }
        }

        crate::log_ok!(
            "Framebuffer",
            "Bench",
            "{}: {} pixel writes in {} cycles ({} cycles/pixel)",
            label,
            writes,
            cycles,
            cycles / writes.max(1) as u64
        );
    }
}
