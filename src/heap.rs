//! Kernel heap.
//!
//! A `#[global_allocator]` backed by an address-ordered, coalescing
//! free-list (first-fit). The heap lives in a fixed slice of virtual
//! address space reserved below the kernel image; nothing is mapped up
//! front. Physical frames are pulled from the PMM and wired in through
//! `vmm::map_page` only as the heap actually needs to grow, the same
//! lazy-mapping discipline the VMM itself uses for everything else.

#![allow(dead_code)]

use core::alloc::{GlobalAlloc, Layout};
use core::fmt;
use core::mem::{align_of, size_of};
use core::ptr;

use crate::pmm;
use crate::sync::IrqMutex;
use crate::vmm;
use crate::{log_debug, log_fail, log_ok};

/// Virtual base of the kernel heap. Sits well below the kernel image
/// (`0xffffffff80000000` and up) and below the VMM's self-test page
/// (`0xffffffffc0000000`), so nothing else living in the top-2GiB slice
/// of the address space can collide with it.
const HEAP_START: usize = 0xffff_ffff_9000_0000;

/// Upper bound the heap may grow to. This only reserves *virtual*
/// address space — physical frames are mapped in on demand — so it
/// costs nothing until the heap actually grows this large.
const HEAP_MAX_SIZE: usize = 64 * 1024 * 1024; // 64 MiB

/// How much to map in per growth step, rounded up to whole pages. Big
/// requests still get exactly the space they need (see `grow`); this is
/// just the default increment for ordinary small allocations.
const HEAP_GROW_STEP: usize = 64 * 1024; // 64 KiB

const PAGE_SIZE: usize = pmm::PAGE_SIZE as usize;

const fn align_up(addr: usize, align: usize) -> usize {
    (addr + align - 1) & !(align - 1)
}

struct Size(usize);

impl fmt::Display for Size {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        if self.0 >= (1 << 20) {
            write!(f, "{} MiB", self.0 >> 20)
        } else if self.0 >= (1 << 10) {
            write!(f, "{} KiB", self.0 >> 10)
        } else {
            write!(f, "{} B", self.0)
        }
    }
}

/// A free block of heap memory. Lives inline at the start of the free
/// region it describes — freeing memory doesn't cost any bookkeeping
/// storage of its own.
struct FreeBlock {
    size: usize,
    next: *mut FreeBlock,
}

impl FreeBlock {
    fn addr(&self) -> usize {
        self as *const _ as usize
    }

    fn end(&self) -> usize {
        self.addr() + self.size
    }
}

struct FreeList {
    /// First free block in address order, or null if the free list
    /// (and possibly the whole heap) is empty.
    head: *mut FreeBlock,
    /// One past the highest heap address currently backed by a mapped
    /// physical frame. `0` means the heap hasn't grown at all yet.
    mapped_end: usize,
    /// Bytes currently on loan to callers (i.e. not on the free list).
    allocated: usize,
    /// Total bytes ever added to the free list via `grow`.
    total: usize,
}

// `FreeList` is only ever touched from behind a `Mutex`, so handing it
// across cores is fine even though it's built out of raw pointers.
unsafe impl Send for FreeList {}

impl FreeList {
    const fn new() -> Self {
        FreeList {
            head: ptr::null_mut(),
            mapped_end: 0,
            allocated: 0,
            total: 0,
        }
    }

    /// Insert `[addr, addr + size)` into the free list in address order,
    /// merging it with whichever neighbour(s) it turns out to be
    /// adjacent to. This is what keeps long alloc/free cycles from
    /// grinding the heap down into unusable dust.
    unsafe fn insert(&mut self, addr: usize, size: usize) {
        debug_assert_eq!(addr % align_of::<FreeBlock>(), 0);
        debug_assert!(size >= size_of::<FreeBlock>());

        let mut prev: *mut FreeBlock = ptr::null_mut();
        let mut cur = self.head;
        while !cur.is_null() && (*cur).addr() < addr {
            prev = cur;
            cur = (*cur).next;
        }

        // Merge with the following block first, if adjacent.
        let (size, next) = if !cur.is_null() && addr + size == (*cur).addr() {
            (size + (*cur).size, (*cur).next)
        } else {
            (size, cur)
        };

        // Then merge the (possibly already-grown) new block into its
        // predecessor, if adjacent, instead of writing a new node at all.
        if !prev.is_null() && (*prev).end() == addr {
            (*prev).size += size;
            (*prev).next = next;
            return;
        }

        let block = addr as *mut FreeBlock;
        block.write(FreeBlock { size, next });

        if prev.is_null() {
            self.head = block;
        } else {
            (*prev).next = block;
        }
    }

    /// First-fit search: finds a free block with room for `size` bytes
    /// aligned to `align`, removes it from the list, and returns any
    /// leftover space (before and/or after the allocation) to the list.
    unsafe fn find_and_remove(&mut self, size: usize, align: usize) -> Option<usize> {
        let mut prev: *mut FreeBlock = ptr::null_mut();
        let mut cur = self.head;

        while !cur.is_null() {
            let block_addr = (*cur).addr();
            let block_size = (*cur).size;
            let alloc_start = align_up(block_addr, align);
            let padding = alloc_start - block_addr;

            if padding + size <= block_size {
                let next = (*cur).next;
                if prev.is_null() {
                    self.head = next;
                } else {
                    (*prev).next = next;
                }

                if padding >= size_of::<FreeBlock>() {
                    self.insert(block_addr, padding);
                }
                let after_size = block_size - padding - size;
                if after_size >= size_of::<FreeBlock>() {
                    self.insert(alloc_start + size, after_size);
                }
                // Slivers smaller than a `FreeBlock` (on either side) are
                // too small to ever track and are simply handed out as
                // internal fragmentation — a few wasted bytes beats a
                // leaked, unrecoverable region.

                return Some(alloc_start);
            }

            prev = cur;
            cur = (*cur).next;
        }

        None
    }

    /// Map in more physical memory and add it to the free list. Returns
    /// `false` if the heap is already at `HEAP_MAX_SIZE` or the PMM is
    /// out of frames.
    unsafe fn grow(&mut self, min_extra: usize) -> bool {
        if self.mapped_end == 0 {
            self.mapped_end = HEAP_START;
        }

        let remaining_va = HEAP_START + HEAP_MAX_SIZE - self.mapped_end;
        if remaining_va == 0 {
            log_fail!("Heap", "Grow", "Heap has hit its {} ceiling", Size(HEAP_MAX_SIZE));
            return false;
        }

        let mut grow_by = HEAP_GROW_STEP.max(align_up(min_extra, PAGE_SIZE));
        grow_by = grow_by.min(remaining_va);
        grow_by = align_up(grow_by, PAGE_SIZE).min(align_down(remaining_va, PAGE_SIZE));
        if grow_by == 0 {
            return false;
        }

        let start = self.mapped_end;
        let mut mapped = 0usize;
        while mapped < grow_by {
            let Some(phys) = pmm::alloc_frame() else {
                log_fail!("Heap", "Grow", "PMM out of frames while growing the heap");
                break;
            };
            vmm::map_page((start + mapped) as u64, phys, vmm::KERNEL_RW);
            mapped += PAGE_SIZE;
        }

        if mapped == 0 {
            return false;
        }

        self.mapped_end = start + mapped;
        self.total += mapped;
        self.insert(start, mapped);

        log_debug!(
            "Heap",
            "Grow",
            "+{} mapped at {:#x} (heap now {})",
            Size(mapped),
            start,
            Size(self.total)
        );

        true
    }
}

const fn align_down(v: usize, a: usize) -> usize {
    v & !(a - 1)
}

/// Bump a `Layout` up to what `FreeBlock` bookkeeping actually needs:
/// every free (and therefore every allocated-then-freed) region has to
/// be big enough, and aligned enough, to hold a `FreeBlock` header.
fn adjust(layout: Layout) -> (usize, usize) {
    let align = layout.align().max(align_of::<FreeBlock>());
    let size = layout.size().max(size_of::<FreeBlock>());
    let size = align_up(size, align_of::<FreeBlock>());
    (size, align)
}

pub struct KernelHeap {
    inner: IrqMutex<FreeList>,
}

impl KernelHeap {
    const fn new() -> Self {
        KernelHeap {
            inner: IrqMutex::new(FreeList::new()),
        }
    }
}

unsafe impl GlobalAlloc for KernelHeap {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let (size, align) = adjust(layout);
        let mut list = self.inner.lock();

        if let Some(addr) = list.find_and_remove(size, align) {
            list.allocated += size;
            return addr as *mut u8;
        }

        // No single free block was big enough — grow the heap (mapping
        // at least enough for this request plus alignment slop) and
        // try exactly once more.
        if !list.grow(size + align) {
            return ptr::null_mut();
        }

        match list.find_and_remove(size, align) {
            Some(addr) => {
                list.allocated += size;
                addr as *mut u8
            }
            None => ptr::null_mut(),
        }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        let (size, _align) = adjust(layout);
        let mut list = self.inner.lock();
        list.allocated = list.allocated.saturating_sub(size);
        list.insert(ptr as usize, size);
    }
}

#[global_allocator]
static ALLOCATOR: KernelHeap = KernelHeap::new();

/// Bytes currently on loan to callers.
pub fn allocated_bytes() -> usize {
    ALLOCATOR.inner.lock().allocated
}

/// Total bytes the heap has grown to (allocated + free).
pub fn heap_size() -> usize {
    ALLOCATOR.inner.lock().total
}

pub fn init() {
    let mut list = ALLOCATOR.inner.lock();
    if !unsafe { list.grow(HEAP_GROW_STEP) } {
        drop(list);
        log_fail!("Heap", "Init", "Could not map the initial heap region");
        return;
    }
    let total = list.total;
    drop(list);

    log_ok!(
        "Heap",
        "Init",
        "Free-list allocator online, {} at {:#x} (grows lazily up to {})",
        Size(total),
        HEAP_START,
        Size(HEAP_MAX_SIZE)
    );
}

pub fn self_test() {
    use alloc::boxed::Box;
    use alloc::vec::Vec;

    let before = allocated_bytes();

    // Basic box round-trip.
    let boxed = Box::new(0x1234_5678_9abc_def0u64);
    if *boxed != 0x1234_5678_9abc_def0 {
        log_fail!("Heap", "SelfTest", "Box value mismatch");
        return;
    }
    drop(boxed);

    // A growing Vec exercises repeated alloc/realloc/dealloc and forces
    // several internal reallocations.
    let mut v: Vec<u32> = Vec::new();
    for i in 0..2048u32 {
        v.push(i);
    }
    if v.len() != 2048 || v[0] != 0 || v[2047] != 2047 {
        log_fail!("Heap", "SelfTest", "Vec contents corrupted after growth");
        return;
    }
    let sum: u64 = v.iter().map(|&x| x as u64).sum();
    if sum != (0..2048u64).sum::<u64>() {
        log_fail!("Heap", "SelfTest", "Vec checksum mismatch");
        return;
    }
    drop(v);

    // Enough small, individually-freed allocations to force at least
    // one heap growth beyond the initial region, then free them all
    // again to exercise coalescing.
    let heap_before_stress = heap_size();
    {
        let mut chunks: Vec<Box<[u8; 512]>> = Vec::new();
        for _ in 0..300 {
            chunks.push(Box::new([0xAA; 512]));
        }
        for c in &chunks {
            if c[0] != 0xAA || c[511] != 0xAA {
                log_fail!("Heap", "SelfTest", "Corrupted allocation under stress");
                return;
            }
        }
    } // all 300 boxes dropped here

    let grew = heap_size() > heap_before_stress;

    let after = allocated_bytes();
    if after != before {
        log_fail!(
            "Heap",
            "SelfTest",
            "Leak detected: {} bytes allocated before, {} after (everything should have been freed)",
            before,
            after
        );
        return;
    }

    log_ok!(
        "Heap",
        "SelfTest",
        "Box/Vec alloc-free verified, coalesced back to {} allocated ({}grew during stress test)",
        after,
        if grew { "" } else { "did not " }
    );
}