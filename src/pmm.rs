//! Physical memory manager: a bitmap allocator that hands out 4 KiB frames.
//!
//! ## What Limine still owns (and how this file respects it)
//!
//! Limine's memory map tells us what every physical range is. We only ever
//! allocate from `USABLE` ranges. Everything else is left alone:
//!
//! * `BOOTLOADER_RECLAIMABLE` holds Limine's page tables (CR3 still points
//!   there), the stack we are running on, and the memory-map response itself.
//!   It must stay untouched until the VMM has switched to our own page tables
//!   and we are on our own stack. Only then may
//!   [`reclaim_bootloader_memory`] hand it to the allocator. To make that safe
//!   we copy the memory map into our own storage during `init`, so we never
//!   need to read Limine's structures again afterwards.
//! * `KERNEL_AND_MODULES`, `FRAMEBUFFER`, `ACPI_*`, `RESERVED`, `BAD_MEMORY`
//!   are never allocated.
//! * The first 1 MiB is never handed out (legacy/BIOS area, future SMP
//!   trampoline, DMA-below-1M users).
//!
//! Limine also provides the higher-half direct map (HHDM): all usable RAM is
//! mapped at `phys + hhdm_offset`, which is how we touch physical frames
//! (the bitmap itself, zeroing frames) before we have a VMM.

#![allow(dead_code)]

use core::fmt;
use core::ptr;
use core::sync::atomic::{AtomicU64, Ordering};
use limine::memory_map::EntryType;
use limine::request::{HhdmRequest, MemoryMapRequest};
use spin::Mutex;

use crate::{log_debug, log_fail, log_ok};

pub const PAGE_SIZE: u64 = 4096;

/// Never allocate below this physical address.
const MIN_ALLOC_ADDR: u64 = 0x10_0000;

/// Max memory-map entries we keep a copy of.
const MAX_REGIONS: usize = 128;

#[used]
#[link_section = ".requests"]
static MEMORY_MAP_REQUEST: MemoryMapRequest = MemoryMapRequest::new();

#[used]
#[link_section = ".requests"]
static HHDM_REQUEST: HhdmRequest = HhdmRequest::new();

static HHDM_OFFSET: AtomicU64 = AtomicU64::new(0);

// ---------------------------------------------------------------------------
// Our own copy of the memory map (the only place Limine's map types are used)
// ---------------------------------------------------------------------------

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

/// Human-readable size for log lines.
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

// ---------------------------------------------------------------------------
// Bitmap helpers. Bit set = frame in use / unavailable, bit clear = free.
// ---------------------------------------------------------------------------

unsafe fn test_bit(bm: *const u64, i: usize) -> bool {
    *bm.add(i / 64) & (1u64 << (i % 64)) != 0
}

unsafe fn set_bit(bm: *mut u64, i: usize) {
    *bm.add(i / 64) |= 1u64 << (i % 64);
}

unsafe fn clear_bit(bm: *mut u64, i: usize) {
    *bm.add(i / 64) &= !(1u64 << (i % 64));
}

/// Mark `count` frames starting at `first` as used (`true`) or free (`false`).
unsafe fn fill_range(bm: *mut u64, mut first: usize, mut count: usize, used: bool) {
    // Head: bit by bit until word aligned
    while count > 0 && first % 64 != 0 {
        if used {
            set_bit(bm, first);
        } else {
            clear_bit(bm, first);
        }
        first += 1;
        count -= 1;
    }
    // Body: whole words
    while count >= 64 {
        *bm.add(first / 64) = if used { !0u64 } else { 0u64 };
        first += 64;
        count -= 64;
    }
    // Tail
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

// ---------------------------------------------------------------------------
// Allocator state
// ---------------------------------------------------------------------------

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
    /// Frames covered by the bitmap (frame 0 .. total_frames).
    total_frames: usize,
    free_frames: usize,
    /// Frames that were free right after init (excludes the bitmap itself).
    usable_frames: usize,
    /// Word index where the next allocation search starts.
    next_hint: usize,
    /// Our copy of Limine's memory map.
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
                let bit = (!word).trailing_zeros() as usize; // First clear bit
                let frame = w * 64 + bit;
                unsafe { *self.bitmap.add(w) = word | (1u64 << bit) };
                self.free_frames -= 1;
                self.next_hint = w;
                return Some(frame as u64 * PAGE_SIZE);
            }
        }
        None
    }

    /// First-fit search for `count` physically contiguous free frames.
    fn alloc_contiguous(&mut self, count: usize) -> Option<u64> {
        if count == 0 || count > self.free_frames {
            return None;
        }
        let mut run = 0usize;
        let mut start = 0usize;
        let mut frame = 0usize;
        while frame < self.total_frames {
            // Fast path: skip a whole fully-used word.
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
        // Check everything first so a bad call changes nothing.
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

static PMM: Mutex<Option<Pmm>> = Mutex::new(None);

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

pub fn hhdm_offset() -> u64 {
    HHDM_OFFSET.load(Ordering::Relaxed)
}

/// Virtual address (in the Limine HHDM) of a physical address.
pub fn phys_to_virt(phys: u64) -> *mut u8 {
    (phys + hhdm_offset()) as *mut u8
}

/// Allocate one 4 KiB frame. Returns its physical address.
pub fn alloc_frame() -> Option<u64> {
    PMM.lock().as_mut()?.alloc_one()
}

/// Allocate one frame and zero it.
pub fn alloc_frame_zeroed() -> Option<u64> {
    let phys = alloc_frame()?;
    unsafe { ptr::write_bytes(phys_to_virt(phys), 0, PAGE_SIZE as usize) };
    Some(phys)
}

/// Allocate `count` physically contiguous frames.
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

/// Free `count` frames starting at `phys`. Invalid or double frees are
/// logged and ignored. Only free frames that came from this allocator.
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

pub fn free_frame_count() -> usize {
    PMM.lock().as_ref().map(|p| p.free_frames).unwrap_or(0)
}

pub fn usable_frame_count() -> usize {
    PMM.lock().as_ref().map(|p| p.usable_frames).unwrap_or(0)
}

/// Highest physical address the PMM knows about (i.e. the exclusive end of
/// the frame bitmap). The VMM direct-maps physical memory up to this
/// address, so it covers usable RAM *and* still-reclaimable bootloader
/// memory, but not MMIO regions like the framebuffer, which live elsewhere
/// and are mapped separately.
pub fn phys_top() -> u64 {
    PMM.lock()
        .as_ref()
        .map(|p| p.total_frames as u64 * PAGE_SIZE)
        .unwrap_or(0)
}

/// Hand Limine's bootloader-reclaimable memory to the allocator.
///
/// # Safety
/// Only call this once **all** of these are true:
/// * the kernel runs on its own page tables (CR3 no longer points into
///   bootloader-reclaimable memory),
/// * the kernel runs on its own stack,
/// * nothing will read Limine's response structures any more.
///
/// Uses the copy of the memory map saved by `init`, not Limine's.
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

// ---------------------------------------------------------------------------
// Initialisation
// ---------------------------------------------------------------------------

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

    // 1. Copy Limine's memory map into our own storage.
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

    // 2. Print the map and find the highest address we may ever manage.
    //    Usable AND bootloader-reclaimable memory must fit in the bitmap,
    //    because the latter becomes allocatable after reclaim.
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

    // 3. Size the bitmap: one bit per 4 KiB frame from address 0 to `highest`.
    let total_frames = (highest / PAGE_SIZE) as usize;
    let bitmap_words = (total_frames + 63) / 64;
    let bitmap_bytes = bitmap_words as u64 * 8;
    let bitmap_frames = align_up(bitmap_bytes, PAGE_SIZE) / PAGE_SIZE;

    // 4. Put the bitmap in the first usable region (above 1 MiB) that fits it.
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

    // 5. Everything starts "used"; free only the usable ranges.
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
        // The bitmap lives inside usable memory: mark its own frames used.
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

/// Allocate, use, free and re-check a few frames.
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

    // The two frames must be independent memory, reachable through the HHDM.
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

        // Dirty a whole frame so the zeroing test below is meaningful.
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