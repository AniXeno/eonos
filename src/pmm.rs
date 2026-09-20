#![allow(dead_code)]

use core::fmt;
use core::ptr;
use core::sync::atomic::{AtomicU64, Ordering};
use limine::memory_map::EntryType;
use limine::request::{HhdmRequest, MemoryMapRequest};
use crate::sync::IrqMutex;

use crate::{log_debug, log_fail, log_ok};

pub const PAGE_SIZE: u64 = 4096;

const MIN_ALLOC_ADDR: u64 = 0x10_0000;
const MAX_REGIONS: usize = 128;

#[used]
#[link_section = ".requests"]
static MEMORY_MAP_REQUEST: MemoryMapRequest = MemoryMapRequest::new();

#[used]
#[link_section = ".requests"]
static HHDM_REQUEST: HhdmRequest = HhdmRequest::new();

static HHDM_OFFSET: AtomicU64 = AtomicU64::new(0);

#[derive(Clone, Copy, PartialEq, Eq)]
enum Kind {
    Usable,
    Reserved,
    AcpiReclaimable,
    AcpiNvs,
    BadMemory,
    BootloaderReclaimable,
    KernelAndModules,
    Framebuffer,
    Unknown,
}

impl Kind {
    fn name(self) -> &'static str {
        match self {
            Kind::Usable => "Usable",
            Kind::Reserved => "Reserved",
            Kind::AcpiReclaimable => "ACPI reclaimable",
            Kind::AcpiNvs => "ACPI NVS",
            Kind::BadMemory => "Bad memory",
            Kind::BootloaderReclaimable => "Bootloader reclaimable",
            Kind::KernelAndModules => "Kernel and modules",
            Kind::Framebuffer => "Framebuffer",
            Kind::Unknown => "Unknown",
        }
    }
}

fn kind_of(t: EntryType) -> Kind {
    if t == EntryType::USABLE {
        Kind::Usable
    } else if t == EntryType::RESERVED {
        Kind::Reserved
    } else if t == EntryType::ACPI_RECLAIMABLE {
        Kind::AcpiReclaimable
    } else if t == EntryType::ACPI_NVS {
        Kind::AcpiNvs
    } else if t == EntryType::BAD_MEMORY {
        Kind::BadMemory
    } else if t == EntryType::BOOTLOADER_RECLAIMABLE {
        Kind::BootloaderReclaimable
    } else if t == EntryType::KERNEL_AND_MODULES {
        Kind::KernelAndModules
    } else if t == EntryType::FRAMEBUFFER {
        Kind::Framebuffer
    } else {
        Kind::Unknown
    }
}

#[derive(Clone, Copy)]
struct Region {
    base: u64,
    len: u64,
    kind: Kind,
}

impl Region {
    const EMPTY: Region = Region {
        base: 0,
        len: 0,
        kind: Kind::Unknown,
    };
}

struct Size(u64);

impl fmt::Display for Size {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        if self.0 >= (1 << 20) {
            write!(f, "{} MiB", self.0 >> 20)
        } else {
            write!(f, "{} KiB", self.0 >> 10)
        }
    }
}

const fn align_up(v: u64, a: u64) -> u64 {
    (v + a - 1) & !(a - 1)
}

const fn align_down(v: u64, a: u64) -> u64 {
    v & !(a - 1)
}

unsafe fn test_bit(bm: *const u64, i: usize) -> bool {
    *bm.add(i / 64) & (1u64 << (i % 64)) != 0
}

unsafe fn set_bit(bm: *mut u64, i: usize) {
    *bm.add(i / 64) |= 1u64 << (i % 64);
}

unsafe fn clear_bit(bm: *mut u64, i: usize) {
    *bm.add(i / 64) &= !(1u64 << (i % 64));
}

unsafe fn fill_range(bm: *mut u64, mut first: usize, mut count: usize, used: bool) {
    while count > 0 && first % 64 != 0 {
        if used {
            set_bit(bm, first);
        } else {
            clear_bit(bm, first);
        }
        first += 1;
        count -= 1;
    }
    while count >= 64 {
        *bm.add(first / 64) = if used { !0u64 } else { 0u64 };
        first += 64;
        count -= 64;
    }
    while count > 0 {
        if used {
            set_bit(bm, first);
        } else {
            clear_bit(bm, first);
        }
        first += 1;
        count -= 1;
    }
}

enum FreeError {
    NotInitialized,
    Unaligned,
    OutOfRange,
    NotAllocated,
}

impl FreeError {
    fn as_str(&self) -> &'static str {
        match self {
            FreeError::NotInitialized => "PMM not initialized",
            FreeError::Unaligned => "address not page aligned",
            FreeError::OutOfRange => "address outside managed memory",
            FreeError::NotAllocated => "frame is not allocated (double free?)",
        }
    }
}

struct Pmm {
    bitmap: *mut u64,
    bitmap_words: usize,
    total_frames: usize,
    free_frames: usize,
    usable_frames: usize,
    next_hint: usize,
    regions: [Region; MAX_REGIONS],
    region_count: usize,
}

unsafe impl Send for Pmm {}

impl Pmm {
    fn alloc_one(&mut self) -> Option<u64> {
        if self.free_frames == 0 {
            return None;
        }
        let words = self.bitmap_words;
        for n in 0..words {
            let w = (self.next_hint + n) % words;
            let word = unsafe { *self.bitmap.add(w) };
            if word != !0u64 {
                let bit = (!word).trailing_zeros() as usize;
                let frame = w * 64 + bit;
                unsafe { *self.bitmap.add(w) = word | (1u64 << bit) };
                self.free_frames -= 1;
                self.next_hint = w;
                return Some(frame as u64 * PAGE_SIZE);
            }
        }
        None
    }

    fn alloc_contiguous(&mut self, count: usize) -> Option<u64> {
        if count == 0 || count > self.free_frames {
            return None;
        }
        let mut run = 0usize;
        let mut start = 0usize;
        let mut frame = 0usize;
        while frame < self.total_frames {
            if frame % 64 == 0 && unsafe { *self.bitmap.add(frame / 64) } == !0u64 {
                run = 0;
                frame += 64;
                continue;
            }
            if unsafe { test_bit(self.bitmap, frame) } {
                run = 0;
            } else {
                if run == 0 {
                    start = frame;
                }
                run += 1;
                if run == count {
                    unsafe { fill_range(self.bitmap, start, count, true) };
                    self.free_frames -= count;
                    return Some(start as u64 * PAGE_SIZE);
                }
            }
            frame += 1;
        }
        None
    }

    fn free(&mut self, phys: u64, count: usize) -> Result<(), FreeError> {
        if count == 0 {
            return Ok(());
        }
        if phys % PAGE_SIZE != 0 {
            return Err(FreeError::Unaligned);
        }
        let first = (phys / PAGE_SIZE) as usize;
        let end = first.checked_add(count).ok_or(FreeError::OutOfRange)?;
        if end > self.total_frames {
            return Err(FreeError::OutOfRange);
        }
        for f in first..end {            if !unsafe { test_bit(self.bitmap, f) } {
                return Err(FreeError::NotAllocated);
            }
        }
        unsafe { fill_range(self.bitmap, first, count, false) };
        self.free_frames += count;
        self.next_hint = self.next_hint.min(first / 64);
        Ok(())
    }
}

static PMM: IrqMutex<Option<Pmm>> = IrqMutex::new(None);

pub fn hhdm_offset() -> u64 {
    HHDM_OFFSET.load(Ordering::Relaxed)
}

pub fn phys_to_virt(phys: u64) -> *mut u8 {
    (phys + hhdm_offset()) as *mut u8
}

pub fn alloc_frame() -> Option<u64> {
    PMM.lock().as_mut()?.alloc_one()
}

pub fn alloc_frame_zeroed() -> Option<u64> {
    let phys = alloc_frame()?;
    unsafe { ptr::write_bytes(phys_to_virt(phys), 0, PAGE_SIZE as usize) };
    Some(phys)
}

pub fn alloc_contiguous(count: usize) -> Option<u64> {
    let mut guard = PMM.lock();
    let pmm = guard.as_mut()?;
    if count == 1 {
        pmm.alloc_one()
    } else {
        pmm.alloc_contiguous(count)
    }
}

pub fn free_frame(phys: u64) {
    free_frames(phys, 1);
}

pub fn free_frames(phys: u64, count: usize) {
    let result = {
        let mut guard = PMM.lock();
        match guard.as_mut() {
            Some(pmm) => pmm.free(phys, count),
            None => Err(FreeError::NotInitialized),
        }
    };
    if let Err(e) = result {
        log_fail!(
            "PMM",
            "Free",
            "Rejected free of {} frame(s) at {:#x}: {}",
            count,
            phys,
            e.as_str()
        );
    }
}

/// Physical ranges (page-aligned, `[start, end)`) that belong in the
/// higher-half direct map: RAM, the kernel image/modules, and ACPI
/// tables. Holes, MMIO, reserved and bad memory are deliberately left
/// out, and so is the framebuffer (the VMM maps it separately as
/// write-combining). Returned by value so the caller can map pages
/// (which takes the PMM lock) without holding it.
pub fn hhdm_ranges() -> ([(u64, u64); MAX_REGIONS], usize) {
    let mut out = [(0u64, 0u64); MAX_REGIONS];
    let mut n = 0usize;
    let guard = PMM.lock();
    let Some(pmm) = guard.as_ref() else {
        return (out, 0);
    };
    for r in &pmm.regions[..pmm.region_count] {
        match r.kind {
            Kind::Usable
            | Kind::BootloaderReclaimable
            | Kind::KernelAndModules
            | Kind::AcpiReclaimable
            | Kind::AcpiNvs => {
                let start = align_down(r.base, PAGE_SIZE);
                let end = align_up(r.base + r.len, PAGE_SIZE);
                if end > start {
                    out[n] = (start, end);
                    n += 1;
                }
            }
            _ => {}
        }
    }
    (out, n)
}

pub fn free_frame_count() -> usize {
    PMM.lock().as_ref().map(|p| p.free_frames).unwrap_or(0)
}

pub fn usable_frame_count() -> usize {
    PMM.lock().as_ref().map(|p| p.usable_frames).unwrap_or(0)
}

pub fn phys_top() -> u64 {
    PMM.lock()
        .as_ref()
        .map(|p| p.total_frames as u64 * PAGE_SIZE)
        .unwrap_or(0)
}

pub unsafe fn reclaim_bootloader_memory() -> usize {
    let reclaimed = {        let mut guard = PMM.lock();
        let Some(pmm) = guard.as_mut() else { return 0 };
        let mut frames = 0usize;
        for i in 0..pmm.region_count {            let r = pmm.regions[i];
            if r.kind != Kind::BootloaderReclaimable {
                continue;
            }
            let start = align_up(r.base.max(MIN_ALLOC_ADDR), PAGE_SIZE);
            let end = align_down(r.base + r.len, PAGE_SIZE);
            if end > start {
                let count = ((end - start) / PAGE_SIZE) as usize;
                fill_range(pmm.bitmap, (start / PAGE_SIZE) as usize, count, false);
                frames += count;
            }
        }
        pmm.free_frames += frames;
        pmm.usable_frames += frames;
        pmm.next_hint = 0;
        frames
    };
    log_ok!(
        "PMM",
        "Reclaim",
        "{} of bootloader memory returned to the allocator",
        Size(reclaimed as u64 * PAGE_SIZE)
    );
    reclaimed
}

pub fn init() {
    let Some(mm_response) = MEMORY_MAP_REQUEST.get_response() else {
        log_fail!("PMM", "Init", "Limine gave us no memory map");
        return;
    };
    let Some(hhdm_response) = HHDM_REQUEST.get_response() else {
        log_fail!("PMM", "Init", "Limine gave us no HHDM offset");
        return;
    };

    let hhdm = hhdm_response.offset();
    HHDM_OFFSET.store(hhdm, Ordering::Relaxed);
    log_debug!("PMM", "Init", "HHDM offset {:#018x}", hhdm);

    let mut regions = [Region::EMPTY; MAX_REGIONS];
    let mut count = 0usize;
    for entry in mm_response.entries() {        if count == MAX_REGIONS {
            log_fail!(
                "PMM",
                "Init",
                "Memory map has more than {} entries, ignoring the rest",
                MAX_REGIONS
            );
            break;
        }
        regions[count] = Region {
            base: entry.base,
            len: entry.length,
            kind: kind_of(entry.entry_type),
        };
        count += 1;
    }

    let mut highest = 0u64;
    let mut reclaim_bytes = 0u64;
    for r in &regions[..count] {        log_debug!(
            "Memory",
            "Map",
            "{:#014x} - {:#014x}  {:<22} {}",
            r.base,
            r.base + r.len,
            r.kind.name(),
            Size(r.len)
        );
        match r.kind {
            Kind::Usable => highest = highest.max(r.base + r.len),
            Kind::BootloaderReclaimable => {
                highest = highest.max(r.base + r.len);
                reclaim_bytes += r.len;
            }
            _ => {}
        }
    }

    if highest == 0 {
        log_fail!("PMM", "Init", "No usable memory found");
        return;
    }

    let total_frames = (highest / PAGE_SIZE) as usize;
    let bitmap_words = (total_frames + 63) / 64;
    let bitmap_bytes = bitmap_words as u64 * 8;
    let bitmap_frames = align_up(bitmap_bytes, PAGE_SIZE) / PAGE_SIZE;

    let mut bitmap_phys = 0u64;
    for r in &regions[..count] {        if r.kind != Kind::Usable {
            continue;
        }
        let start = align_up(r.base.max(MIN_ALLOC_ADDR), PAGE_SIZE);
        let end = align_down(r.base + r.len, PAGE_SIZE);
        if end > start && end - start >= bitmap_frames * PAGE_SIZE {
            bitmap_phys = start;
            break;
        }
    }

    if bitmap_phys == 0 {
        log_fail!(
            "PMM",
            "Init",
            "No usable region large enough for the {} bitmap",
            Size(bitmap_bytes)
        );
        return;
    }

    let bitmap = (bitmap_phys + hhdm) as *mut u64;

    let mut free_frames = 0usize;
    unsafe {
        ptr::write_bytes(bitmap as *mut u8, 0xFF, bitmap_words * 8);
        for r in &regions[..count] {            if r.kind != Kind::Usable {
                continue;
            }
            let start = align_up(r.base.max(MIN_ALLOC_ADDR), PAGE_SIZE);
            let end = align_down(r.base + r.len, PAGE_SIZE);
            if end > start {
                let n = ((end - start) / PAGE_SIZE) as usize;
                fill_range(bitmap, (start / PAGE_SIZE) as usize, n, false);
                free_frames += n;
            }
        }
        fill_range(
            bitmap,
            (bitmap_phys / PAGE_SIZE) as usize,
            bitmap_frames as usize,
            true,
        );
    }

    free_frames -= bitmap_frames as usize;

    *PMM.lock() = Some(Pmm {
        bitmap,
        bitmap_words,
        total_frames,
        free_frames,
        usable_frames: free_frames,
        next_hint: 0,
        regions,
        region_count: count,
    });

    log_ok!(
        "PMM",
        "Init",
        "{} free ({} frames), bitmap {} at {:#x}",
        Size(free_frames as u64 * PAGE_SIZE),
        free_frames,
        Size(bitmap_bytes),
        bitmap_phys
    );
    log_debug!(
        "PMM",
        "Init",
        "{} of bootloader-reclaimable memory left untouched (Limine's page tables and stack live there)",
        Size(reclaim_bytes)
    );
}

pub fn self_test() {
    let before = free_frame_count();
    if before == 0 {
        log_fail!("PMM", "SelfTest", "No free frames to test with");
        return;
    }

    let (Some(a), Some(b)) = (alloc_frame(), alloc_frame()) else {
        log_fail!("PMM", "SelfTest", "Frame allocation failed");
        return;
    };

    if a == b
        || a % PAGE_SIZE != 0
        || b % PAGE_SIZE != 0
        || a < MIN_ALLOC_ADDR
        || b < MIN_ALLOC_ADDR
    {
        log_fail!(
            "PMM",
            "SelfTest",
            "Bad frames returned: {:#x} and {:#x}",
            a,
            b
        );
        return;
    }

    unsafe {
        let pa = phys_to_virt(a) as *mut u64;
        let pb = phys_to_virt(b) as *mut u64;
        pa.write_volatile(0xDEAD_BEEF_CAFE_F00D);
        pb.write_volatile(0x0123_4567_89AB_CDEF);
        if pa.read_volatile() != 0xDEAD_BEEF_CAFE_F00D
            || pb.read_volatile() != 0x0123_4567_89AB_CDEF
        {
            log_fail!(
                "PMM",
                "SelfTest",
                "Frames {:#x}/{:#x} are not independent memory",
                a,
                b
            );
            return;
        }

        ptr::write_bytes(phys_to_virt(a), 0xAA, PAGE_SIZE as usize);
    }

    free_frame(a);

    let Some(z) = alloc_frame_zeroed() else {
        log_fail!("PMM", "SelfTest", "Zeroed allocation failed");
        return;
    };

    let zeroed = unsafe {
        let p = phys_to_virt(z) as *const u64;
        (0..(PAGE_SIZE as usize / 8)).all(|i| p.add(i).read_volatile() == 0)
    };

    if !zeroed {
        log_fail!("PMM", "SelfTest", "Frame {:#x} was not zeroed", z);
        return;
    }

    let Some(run) = alloc_contiguous(4) else {
        log_fail!(
            "PMM",
            "SelfTest",
            "Contiguous allocation of 4 frames failed"
        );
        return;
    };

    free_frame(z);
    free_frame(b);
    free_frames(run, 4);

    let after = free_frame_count();
    if after != before {
        log_fail!(
            "PMM",
            "SelfTest",
            "Free frame count changed: {} before, {} after",
            before,
            after
        );
        return;
    }

    log_ok!(
        "PMM",
        "SelfTest",
        "alloc/free/zero/contiguous verified (frames {:#x}, {:#x}, run at {:#x}), {} frames free",
        a,
        b,
        run,
        after
    );
}