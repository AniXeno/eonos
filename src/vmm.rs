#![allow(dead_code)]

use core::arch::asm;
use core::fmt;
use core::ptr::addr_of;
use core::sync::atomic::{AtomicU64, Ordering};

use limine::request::KernelAddressRequest;
use crate::pmm::{self, PAGE_SIZE};
use crate::sync::IrqMutex;
use crate::{log_debug, log_fail, log_ok};

const PRESENT: u64 = 1 << 0;
const WRITABLE: u64 = 1 << 1;
const USER: u64 = 1 << 2;
const PWT: u64 = 1 << 3;
const PCD: u64 = 1 << 4;
const HUGE: u64 = 1 << 7;
const NO_EXECUTE: u64 = 1 << 63;

pub const KERNEL_RX: u64 = PRESENT;
pub const KERNEL_RO: u64 = PRESENT | NO_EXECUTE;
pub const KERNEL_RW: u64 = PRESENT | WRITABLE | NO_EXECUTE;

/// Ring-3-accessible leaf flags. The CPU also requires the `USER` bit
/// set on every page-table level above the leaf, not just the leaf
/// itself; `child_table`/`split_huge` always set it on the intermediate
/// tables they create, so any leaf can be made user-accessible just by
/// picking one of these for its own flags -- a kernel-only leaf
/// underneath a `USER` intermediate table is still inaccessible from
/// ring 3, since that check is a logical AND across every level.
pub const USER_RX: u64 = PRESENT | USER;
pub const USER_RO: u64 = PRESENT | USER | NO_EXECUTE;
pub const USER_RW: u64 = PRESENT | WRITABLE | USER | NO_EXECUTE;
/// Write-combining: for framebuffers and other linear MMIO the CPU may
/// buffer and merge writes to. See `configure_pat` for how the PWT bit
/// ends up meaning "write-combining" instead of its default "write-
/// through". Never use this for memory that's read back right after
/// being written (WC writes can sit in a fill buffer for a while) or for
/// device registers with side effects on individual writes.
pub const KERNEL_WC: u64 = PRESENT | WRITABLE | NO_EXECUTE | PWT;
/// Fully uncacheable: every load/store goes straight to the device,
/// nothing buffered or reordered by the cache. Required for real
/// device registers with side effects on read/write -- xHCI's
/// capability/operational/runtime/doorbell registers, for instance --
/// where `KERNEL_WC`'s write-buffering or an ordinary cacheable mapping
/// would both be actively wrong (a doorbell write sitting in a fill
/// buffer instead of reaching the controller, or a stale cached read of
/// a status register that the controller has since changed).
pub const KERNEL_UC: u64 = PRESENT | WRITABLE | NO_EXECUTE | PCD;

const ADDR_MASK: u64 = 0x000f_ffff_ffff_f000;
const ENTRIES: usize = 512;
const HUGE_2M: u64 = 0x20_0000;
const HUGE_1G: u64 = 0x4000_0000;

/// Physical-address masks for huge entries. Bit 12 is the PAT bit in a
/// 2M/1G entry, so it must not be treated as part of the address.
const HUGE_2M_MASK: u64 = 0x000f_ffff_ffe0_0000;
const HUGE_1G_MASK: u64 = 0x000f_ffff_c000_0000;
const PAT_HUGE: u64 = 1 << 12;
const PAT_4K: u64 = 1 << 7;

const SELF_TEST_VIRT: u64 = 0xffff_ffff_c000_0000;
/// An arbitrary, page-aligned user-space address, used only to test
/// mapping into an `AddressSpace`'s private half. Nothing else uses the
/// user half of any address space yet, so any canonical low address
/// works.
const SELF_TEST_USER_VIRT: u64 = 0x0000_0000_0010_0000;

/// PML4 index at which canonical higher-half addresses (0xffff_8000_0000_0000
/// and up) begin. Every kernel mapping lives at or above this index;
/// every user mapping lives below it.
const KERNEL_PML4_START: usize = 256;

extern "C" {
    static __kernel_text_start: u8;
    static __kernel_text_end: u8;
    static __kernel_rodata_start: u8;
    static __kernel_rodata_end: u8;
    static __kernel_data_start: u8;
    static __kernel_data_end: u8;
}

#[used]
#[link_section = ".requests"]
static KERNEL_ADDRESS_REQUEST: KernelAddressRequest = KernelAddressRequest::new();

#[repr(C, align(4096))]
struct Table([u64; ENTRIES]);

impl Table {
    const ZERO: Table = Table([0; ENTRIES]);
}

struct Vmm {
    pml4_phys: u64,
}

unsafe impl Send for Vmm {}

static VMM: IrqMutex<Option<Vmm>> = IrqMutex::new(None);

/// Physical address of the kernel's own PML4 (the one `init` loads).
/// Kept outside the `VMM` lock so the scheduler can read it on every
/// context switch without touching that lock. Zero until `init` ran.
static KERNEL_CR3: AtomicU64 = AtomicU64::new(0);

/// First address past the user half of the address space. Every
/// canonical address below this is user space, everything at or above
/// `0xffff_8000_0000_0000` is kernel space.
pub const USER_SPACE_END: u64 = 0x0000_8000_0000_0000;

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

fn table_ptr(phys: u64) -> *mut Table {
    pmm::phys_to_virt(phys) as *mut Table
}

fn index(virt: u64, level: usize) -> usize {
    ((virt >> (39 - level * 9)) & 0x1FF) as usize
}


/// Descend one level, allocating a zeroed table if the entry is empty.
/// Only valid for levels whose entries can't be leaf mappings we care
/// about (PML4 -> PDPT, PDPT -> PD); the PD -> PT step goes through
/// `pt_for`, which knows how to deal with 2 MiB pages.
unsafe fn child_table(parent: *mut Table, idx: usize) -> *mut Table {
    let entry = (*parent).0[idx];
    if entry & PRESENT != 0 {
        assert!(
            entry & HUGE == 0,
            "VMM: unexpected 1 GiB page while walking the page tables"
        );
        return table_ptr(entry & ADDR_MASK);
    }
    let phys = pmm::alloc_frame_zeroed().expect("VMM: out of memory for page tables");
    (*parent).0[idx] = phys | PRESENT | WRITABLE | USER;
    table_ptr(phys)
}

/// Replace a 2 MiB mapping with an equivalent page table of 512 4K
/// mappings (same physical range, same permissions and memory type), so
/// individual pages inside it can then be remapped or unmapped.
unsafe fn split_huge(pd: *mut Table, idx: usize) {
    let entry = (*pd).0[idx];
    let base = entry & HUGE_2M_MASK;

    // Keep permission/caching flags, drop the address and the huge bit;
    // the PAT bit lives at a different position in 4K entries.
    let mut flags = entry & !ADDR_MASK & !HUGE;
    if entry & PAT_HUGE != 0 {
        flags |= PAT_4K;
    }

    let phys = pmm::alloc_frame_zeroed().expect("VMM: out of memory splitting a 2 MiB page");
    let pt = table_ptr(phys);
    for i in 0..ENTRIES {
        (*pt).0[i] = (base + i as u64 * PAGE_SIZE) | flags;
    }
    (*pd).0[idx] = phys | PRESENT | WRITABLE | USER;
}

/// The page table covering `virt`, splitting a 2 MiB page first if that's
/// what currently maps it. Without this, a 4K mapping inside a huge page
/// would treat the huge page's memory as a page table and scribble on it.
unsafe fn pt_for(pd: *mut Table, virt: u64) -> *mut Table {
    let idx = index(virt, 2);
    let entry = (*pd).0[idx];
    if entry & PRESENT != 0 && entry & HUGE != 0 {
        split_huge(pd, idx);
        invlpg(virt);
    }
    child_table(pd, idx)
}

unsafe fn map4k(pml4: *mut Table, virt: u64, phys: u64, flags: u64) {
    let pdpt = child_table(pml4, index(virt, 0));
    let pd = child_table(pdpt, index(virt, 1));
    let pt = pt_for(pd, virt);
    (*pt).0[index(virt, 3)] = (phys & ADDR_MASK) | flags | PRESENT;
}

unsafe fn map2m(pml4: *mut Table, virt: u64, phys: u64, flags: u64) {
    let pdpt = child_table(pml4, index(virt, 0));
    let pd = child_table(pdpt, index(virt, 1));
    let idx = index(virt, 2);
    let existing = (*pd).0[idx];
    assert!(
        existing & PRESENT == 0 || existing & HUGE != 0,
        "VMM: 2 MiB mapping would overwrite an existing page table"
    );
    (*pd).0[idx] = (phys & HUGE_2M_MASK) | flags | PRESENT | HUGE;
}

pub fn map_page(virt: u64, phys: u64, flags: u64) {
    let mut guard = VMM.lock();
    let Some(vmm) = guard.as_mut() else { return };
    unsafe {
        map4k(table_ptr(vmm.pml4_phys), virt, phys, flags);
        invlpg(virt);
    }
}

/// Map a device's physical MMIO range so it's actually safe to read
/// through `pmm::phys_to_virt` -- which just does `hhdm_offset() +
/// phys` and assumes the page is already mapped, true for RAM (which
/// `map_hhdm_range` covers at boot) but not for PCI BAR space, which
/// sits outside the bootloader's memory map entirely and is therefore
/// unmapped until something maps it. Reuses the HHDM's own virtual
/// address convention (`hhdm_offset() + phys`) rather than inventing a
/// separate MMIO virtual range, so `phys_to_virt` keeps working
/// unchanged for both RAM and, once mapped here, device registers.
///
/// Always maps `KERNEL_UC`: every current caller (xHCI's capability/
/// operational/runtime/doorbell registers) is registers with side
/// effects, where a cacheable or write-combined mapping would be
/// actively wrong (see `KERNEL_UC`'s own doc comment). A future caller
/// wanting a linear framebuffer-like BAR would need its own entry
/// point using `KERNEL_WC` instead -- deliberately not this one, so
/// picking the wrong attribute for register access isn't the easy
/// default.
///
/// `phys` and `len` need not be page-aligned; every page the range
/// touches is mapped. Safe to call more than once on overlapping
/// ranges as long as pages already mapped by an earlier call aren't
/// mapped again with different flags -- `map4k` asserts on that rather
/// than silently changing an existing mapping's attributes.
pub fn map_mmio(phys: u64, len: u64) -> *mut u8 {
    let hhdm = pmm::hhdm_offset();
    let start = phys & !(PAGE_SIZE - 1);
    let end = (phys + len + PAGE_SIZE - 1) & !(PAGE_SIZE - 1);
    let mut p = start;
    while p < end {
        let virt = hhdm + p;
        if translate(virt).is_none() {
            map_page(virt, p, KERNEL_UC);
        }
        p += PAGE_SIZE;
    }
    (hhdm + phys) as *mut u8
}

pub fn unmap_page(virt: u64) {
    let guard = VMM.lock();
    let Some(vmm) = guard.as_ref() else { return };
    unsafe {
        let pml4 = table_ptr(vmm.pml4_phys);
        let e = (*pml4).0[index(virt, 0)];
        if e & PRESENT == 0 {
            return;
        }
        let pdpt = table_ptr(e & ADDR_MASK);
        let e = (*pdpt).0[index(virt, 1)];
        if e & PRESENT == 0 || e & HUGE != 0 {
            return;
        }
        let pd = table_ptr(e & ADDR_MASK);
        let idx = index(virt, 2);
        let e = (*pd).0[idx];
        if e & PRESENT == 0 {
            return;
        }
        if e & HUGE != 0 {
            split_huge(pd, idx);
        }
        let e = (*pd).0[idx];
        let pt = table_ptr(e & ADDR_MASK);
        (*pt).0[index(virt, 3)] = 0;
        invlpg(virt);
    }
}

/// Software page-table walk. Handles 1 GiB, 2 MiB and 4 KiB mappings.
unsafe fn walk(pml4_phys: u64, virt: u64) -> Option<u64> {
    let pml4 = table_ptr(pml4_phys);
    let e = (*pml4).0[index(virt, 0)];
    if e & PRESENT == 0 {
        return None;
    }
    let pdpt = table_ptr(e & ADDR_MASK);
    let e = (*pdpt).0[index(virt, 1)];
    if e & PRESENT == 0 {
        return None;
    }
    if e & HUGE != 0 {
        return Some((e & HUGE_1G_MASK) + (virt & (HUGE_1G - 1)));
    }
    let pd = table_ptr(e & ADDR_MASK);
    let e = (*pd).0[index(virt, 2)];
    if e & PRESENT == 0 {
        return None;
    }
    if e & HUGE != 0 {
        return Some((e & HUGE_2M_MASK) + (virt & (HUGE_2M - 1)));
    }
    let pt = table_ptr(e & ADDR_MASK);
    let leaf = (*pt).0[index(virt, 3)];
    if leaf & PRESENT == 0 {
        return None;
    }
    Some((leaf & ADDR_MASK) + (virt & (PAGE_SIZE - 1)))
}

pub fn translate(virt: u64) -> Option<u64> {
    let guard = VMM.lock();
    let vmm = guard.as_ref()?;
    unsafe { walk(vmm.pml4_phys, virt) }
}

/// Like `translate`, but gives up instead of waiting if the VMM lock is
/// taken. For crash handlers: the fault may have happened while the
/// faulting code held that very lock, and spinning would hang the
/// machine instead of showing the crash screen.
pub fn try_translate(virt: u64) -> Option<u64> {
    let guard = VMM.try_lock()?;
    let vmm = guard.as_ref()?;
    unsafe { walk(vmm.pml4_phys, virt) }
}

pub fn is_active() -> bool {
    VMM.lock().is_some()
}

/// Physical address of the kernel's own PML4, or 0 before `init`.
pub fn kernel_cr3() -> u64 {
    KERNEL_CR3.load(Ordering::Acquire)
}

/// Physical address of the PML4 the CPU is translating through right
/// now (CR3 with the PWT/PCD flag bits masked off).
pub fn current_cr3() -> u64 {
    unsafe { read_cr3() & ADDR_MASK }
}

pub unsafe fn read_cr3() -> u64 {
    let v: u64;
    asm!("mov {}, cr3", out(reg) v, options(nomem, nostack, preserves_flags));
    v
}

pub unsafe fn write_cr3(phys: u64) {
    asm!("mov cr3, {}", in(reg) phys, options(nostack, preserves_flags));
}

unsafe fn invlpg(virt: u64) {
    asm!("invlpg [{}]", in(reg) virt, options(nostack, preserves_flags));
}

unsafe fn enable_nxe() {
    const EFER: u32 = 0xC000_0080;
    let lo: u32;
    let hi: u32;
    asm!("rdmsr", in("ecx") EFER, out("eax") lo, out("edx") hi, options(nomem, nostack, preserves_flags));
    let value = (((hi as u64) << 32) | lo as u64) | (1 << 11);
    let lo = value as u32;
    let hi = (value >> 32) as u32;
    asm!("wrmsr", in("ecx") EFER, in("eax") lo, in("edx") hi, options(nomem, nostack, preserves_flags));
}

/// Repurpose PAT slot 1 as Write-Combining.
///
/// The IA32_PAT MSR holds eight 8-bit memory-type slots; which slot a
/// page uses is selected by the PAT/PCD/PWT bits in its page table
/// entry. Firmware's default PAT leaves slot 1 (selected by PWT=1,
/// PCD=0, PAT=0) as Write-Through, a leftover from before PAT existed.
/// We overwrite just that slot with Write-Combining (memory type 0x01),
/// which means any page mapped with the PWT bit set (and PCD/PAT clear)
/// — see `KERNEL_WC` — becomes WC without ever needing to touch the PAT
/// bit itself, which conveniently sits in a different position on 4K
/// PTEs (bit 7) than on 2M/1G huge-page entries (bit 12).
///
/// This matters because on real hardware, MMIO regions like a linear
/// framebuffer default to Uncacheable: every write is a small,
/// individually-serialized bus transaction. WC lets the CPU buffer and
/// merge writes before flushing them out, which is the difference
/// between fast and unusably slow pixel plotting. QEMU's emulated
/// framebuffer doesn't model this cost, which is why the slowdown only
/// shows up on real hardware.
unsafe fn configure_pat() {
    const PAT_MSR: u32 = 0x277;
    const WRITE_COMBINING: u64 = 0x01;

    // Flush and invalidate caches before changing a memory type that
    // may already be in use (Intel SDM Vol. 3A 11.11.8), and again
    // after, so nothing straddles the change with stale attributes.
    asm!("wbinvd", options(nomem, nostack));

    let lo: u32;
    let hi: u32;
    asm!("rdmsr", in("ecx") PAT_MSR, out("eax") lo, out("edx") hi, options(nomem, nostack, preserves_flags));
    let mut value = ((hi as u64) << 32) | lo as u64;

    value &= !(0xFFu64 << 8);
    value |= WRITE_COMBINING << 8;

    let lo = value as u32;
    let hi = (value >> 32) as u32;
    asm!("wrmsr", in("ecx") PAT_MSR, in("eax") lo, in("edx") hi, options(nomem, nostack, preserves_flags));

    asm!("wbinvd", options(nomem, nostack));
}

/// Sort `[start, end)` ranges and merge any that touch or overlap.
/// Returns how many ranges remain at the front of the slice.
fn merge_ranges(r: &mut [(u64, u64)]) -> usize {
    for i in 1..r.len() {
        let mut j = i;
        while j > 0 && r[j - 1].0 > r[j].0 {
            r.swap(j - 1, j);
            j -= 1;
        }
    }
    let mut out = 0usize;
    for i in 0..r.len() {
        if out > 0 && r[i].0 <= r[out - 1].1 {
            if r[i].1 > r[out - 1].1 {
                r[out - 1].1 = r[i].1;
            }
        } else {
            r[out] = r[i];
            out += 1;
        }
    }
    out
}

/// Map physical `[start, end)` into the HHDM, using 2 MiB pages wherever
/// a whole aligned 2 MiB fits inside the range and 4K pages elsewhere.
unsafe fn map_hhdm_range(pml4: *mut Table, hhdm: u64, start: u64, end: u64) {
    let mut p = start;
    while p < end {
        if p % HUGE_2M == 0 && end - p >= HUGE_2M {
            map2m(pml4, hhdm + p, p, KERNEL_RW);
            p += HUGE_2M;
        } else {
            map4k(pml4, hhdm + p, p, KERNEL_RW);
            p += PAGE_SIZE;
        }
    }
}

unsafe fn map_kernel_sections(pml4: *mut Table, kernel_phys: u64, kernel_virt: u64) {
    let to_phys = |v: u64| kernel_phys + (v - kernel_virt);

    let mut map_section = |start: u64, end: u64, flags: u64| {
        let start = align_down(start, PAGE_SIZE);
        let end = align_up(end, PAGE_SIZE);
        let mut v = start;
        while v < end {
            map4k(pml4, v, to_phys(v), flags);
            v += PAGE_SIZE;
        }
    };

    map_section(
        addr_of!(__kernel_text_start) as u64,
        addr_of!(__kernel_text_end) as u64,
        KERNEL_RX,
    );
    map_section(
        addr_of!(__kernel_rodata_start) as u64,
        addr_of!(__kernel_rodata_end) as u64,
        KERNEL_RO,
    );
    map_section(
        addr_of!(__kernel_data_start) as u64,
        addr_of!(__kernel_data_end) as u64,
        KERNEL_RW,
    );
}

pub fn init() {
    unsafe { enable_nxe() };
    unsafe { configure_pat() };

    let Some(ka) = KERNEL_ADDRESS_REQUEST.get_response() else {
        log_fail!("VMM", "Init", "Limine gave us no kernel address info");
        return;
    };
    let kernel_phys = ka.physical_base();
    let kernel_virt = ka.virtual_base();

    let hhdm = pmm::hhdm_offset();
    let phys_top = pmm::phys_top();
    if phys_top == 0 {
        log_fail!("VMM", "Init", "PMM has no memory range to map (run pmm::init first)");
        return;
    }

    let Some(pml4_phys) = pmm::alloc_frame_zeroed() else {
        log_fail!("VMM", "Init", "Out of memory allocating the PML4");
        return;
    };
    let pml4 = table_ptr(pml4_phys);

    let mut mapped_bytes = 0u64;
    unsafe {
        // Map only the ranges the memory map says are real (RAM, kernel,
        // ACPI), not everything from 0 to the top of RAM: the holes in
        // between hold MMIO such as the framebuffer, which must not end
        // up cacheable, and which the WC mapping below must not collide
        // with.
        let (mut ranges, n) = pmm::hhdm_ranges();
        let n = merge_ranges(&mut ranges[..n]);
        for &(start, end) in &ranges[..n] {
            map_hhdm_range(pml4, hhdm, start, end);
            mapped_bytes += end - start;
        }

        if let Some(fb) = crate::framebuffer::FRAMEBUFFER.lock().as_ref() {
            if let Some(fb_phys) = (fb.addr as u64).checked_sub(hhdm) {
                let start = align_down(fb_phys, PAGE_SIZE);
                let end = align_up(fb_phys + fb.pitch * fb.height, PAGE_SIZE);
                let mut p = start;
                while p < end {
                    map4k(pml4, hhdm + p, p, KERNEL_WC);
                    p += PAGE_SIZE;
                }
            }
        }

        map_kernel_sections(pml4, kernel_phys, kernel_virt);
    }

    let old_cr3 = unsafe { read_cr3() };
    unsafe { write_cr3(pml4_phys) };

    *VMM.lock() = Some(Vmm { pml4_phys });
    KERNEL_CR3.store(pml4_phys, Ordering::Release);

    log_ok!(
        "VMM",
        "Init",
        "Own page tables active ({} physical mapped, cr3 {:#x} -> {:#x}, PAT slot 1 = write-combining)",
        Size(mapped_bytes),
        old_cr3,
        pml4_phys
    );
    log_debug!(
        "VMM",
        "Init",
        "Kernel image {:#018x} (phys {:#x}) mapped RX/RO/RW by section, NXE enabled",
        kernel_virt,
        kernel_phys
    );
}

// ---------------------------------------------------------------------
// Per-process address spaces
// ---------------------------------------------------------------------

/// A process's page tables.
///
/// The lower 256 PML4 entries (user space, `0x0000_..` addresses) are
/// private to this address space. The upper 256 (kernel space) are
/// copied verbatim from the kernel's own PML4 at creation time: each
/// copied entry is a pointer to the very same PDPT the kernel itself
/// uses, not a fresh copy of one. So every address space shares its
/// kernel half with the kernel and with every other address space --
/// there is no synchronization to do when the kernel maps or unmaps
/// something, because there is nothing to keep in sync.
///
/// Caveat: this sharing happens at the granularity of whole 512 GiB
/// PML4 entries, once, at creation time. If the kernel later starts
/// using a PML4 slot it had never touched before (unlikely -- image,
/// heap, stacks and HHDM already occupy theirs from `vmm::init`
/// onward), an address space created before that point would not see
/// it. Everything below the PML4 (PDPT/PD/PT) is always shared live,
/// since only the PML4 itself is copied.
pub struct AddressSpace {
    pml4_phys: u64,
}

unsafe impl Send for AddressSpace {}

impl AddressSpace {
    /// Physical address of this address space's PML4, i.e. the value to
    /// load into CR3 to make it current.
    pub fn phys(&self) -> u64 {
        self.pml4_phys
    }

    /// Load this address space into CR3, making it the one the CPU
    /// translates through. The caller is responsible for switching back
    /// (or into another address space) before this one is dropped, and
    /// for keeping the `AddressSpace` alive for as long as it's active.
    pub unsafe fn activate(&self) {
        write_cr3(self.pml4_phys);
    }

    /// Map a single 4 KiB page in this address space's private half.
    /// Only valid for `virt < 0x0000_8000_0000_0000` (user space); use
    /// the free-standing `vmm::map_page` for kernel addresses instead.
    pub fn map(&self, virt: u64, phys: u64, flags: u64) {
        debug_assert!(
            index_of_top_level(virt) < KERNEL_PML4_START,
            "AddressSpace::map called with a kernel-half address; use vmm::map_page"
        );
        unsafe {
            map4k(table_ptr(self.pml4_phys), virt, phys, flags);
            // Only meaningful while this space is the active one, but
            // harmless (a no-op) otherwise: INVLPG only ever discards a
            // TLB entry for the current CR3.
            invlpg(virt);
        }
    }

    /// Software page-table walk in this address space, independent of
    /// which address space is currently active in CR3.
    pub fn translate(&self, virt: u64) -> Option<u64> {
        unsafe { walk(self.pml4_phys, virt) }
    }
}

impl Drop for AddressSpace {
    /// Tear down the private (user) half: every page table it owns, and
    /// every leaf frame still mapped underneath them, then the PML4
    /// itself. The shared kernel half is never touched -- those PDPTs
    /// belong to the kernel's own address space and outlive any process.
    fn drop(&mut self) {
        unsafe { free_user_half(self.pml4_phys) };
    }
}

fn index_of_top_level(virt: u64) -> usize {
    index(virt, 0)
}

unsafe fn free_user_half(pml4_phys: u64) {
    let pml4 = table_ptr(pml4_phys);
    for i in 0..KERNEL_PML4_START {
        let e = (*pml4).0[i];
        if e & PRESENT != 0 {
            free_pdpt(e & ADDR_MASK);
        }
    }
    pmm::free_frame(pml4_phys);
}

unsafe fn free_pdpt(phys: u64) {
    let t = table_ptr(phys);
    for i in 0..ENTRIES {
        let e = (*t).0[i];
        if e & PRESENT == 0 {
            continue;
        }
        if e & HUGE != 0 {
            pmm::free_frame(e & HUGE_1G_MASK);
        } else {
            free_pd(e & ADDR_MASK);
        }
    }
    pmm::free_frame(phys);
}

unsafe fn free_pd(phys: u64) {
    let t = table_ptr(phys);
    for i in 0..ENTRIES {
        let e = (*t).0[i];
        if e & PRESENT == 0 {
            continue;
        }
        if e & HUGE != 0 {
            pmm::free_frame(e & HUGE_2M_MASK);
        } else {
            free_pt(e & ADDR_MASK);
        }
    }
    pmm::free_frame(phys);
}

unsafe fn free_pt(phys: u64) {
    let t = table_ptr(phys);
    for i in 0..ENTRIES {
        let e = (*t).0[i];
        if e & PRESENT != 0 {
            pmm::free_frame(e & ADDR_MASK);
        }
    }
    pmm::free_frame(phys);
}

/// Software walk for a ring-3 read of `virt` through `pml4_phys`.
///
/// Succeeds only for a 4 KiB mapping that is user-accessible at *every*
/// level (the CPU ANDs the U/S bit across the whole walk). Huge pages
/// are refused: nothing in the user half is ever mapped that way, so
/// seeing one there means something is wrong. Returns the physical
/// address of the exact byte, like `walk`.
unsafe fn walk_user(pml4_phys: u64, virt: u64) -> Option<u64> {
    let pml4 = table_ptr(pml4_phys);
    let e = (*pml4).0[index(virt, 0)];
    if e & (PRESENT | USER) != (PRESENT | USER) {
        return None;
    }
    let pdpt = table_ptr(e & ADDR_MASK);
    let e = (*pdpt).0[index(virt, 1)];
    if e & (PRESENT | USER) != (PRESENT | USER) || e & HUGE != 0 {
        return None;
    }
    let pd = table_ptr(e & ADDR_MASK);
    let e = (*pd).0[index(virt, 2)];
    if e & (PRESENT | USER) != (PRESENT | USER) || e & HUGE != 0 {
        return None;
    }
    let pt = table_ptr(e & ADDR_MASK);
    let leaf = (*pt).0[index(virt, 3)];
    if leaf & (PRESENT | USER) != (PRESENT | USER) {
        return None;
    }
    Some((leaf & ADDR_MASK) + (virt & (PAGE_SIZE - 1)))
}

/// Copy `dst.len()` bytes from the *current* address space's user
/// memory at `src` into `dst`. Returns `false` (leaving `dst` partly
/// written) if any part of the range is outside user space or not
/// mapped user-accessible.
///
/// This never dereferences the user pointer: it walks the page tables in
/// software and reads each page through the kernel's direct map, so a
/// bad pointer from a process is a clean `false` here instead of a
/// kernel-mode page fault (which would panic the whole kernel).
pub fn copy_from_user(dst: &mut [u8], src: u64) -> bool {
    if dst.is_empty() {
        return true;
    }
    let Some(end) = src.checked_add(dst.len() as u64) else {
        return false;
    };
    if end > USER_SPACE_END {
        return false;
    }

    let pml4 = current_cr3();
    let mut done = 0usize;
    while done < dst.len() {
        let va = src + done as u64;
        let page_off = (va & (PAGE_SIZE - 1)) as usize;
        let n = (dst.len() - done).min(PAGE_SIZE as usize - page_off);
        let Some(phys) = (unsafe { walk_user(pml4, va) }) else {
            return false;
        };
        unsafe {
            core::ptr::copy_nonoverlapping(
                pmm::phys_to_virt(phys) as *const u8,
                dst.as_mut_ptr().add(done),
                n,
            );
        }
        done += n;
    }
    true
}

/// Copy `src` into the *current* address space's user memory at `dst`.
/// Returns `false` (leaving the destination partly written) if any part
/// of the range is outside user space or not mapped user-writable.
///
/// Mirror image of `copy_from_user`: same software page-table walk
/// through the direct map, so a bad or read-only pointer from a process
/// is a clean `false` here instead of a kernel-mode page fault.
pub fn copy_to_user(dst: u64, src: &[u8]) -> bool {
    if src.is_empty() {
        return true;
    }
    let Some(end) = dst.checked_add(src.len() as u64) else {
        return false;
    };
    if end > USER_SPACE_END {
        return false;
    }

    let pml4 = current_cr3();
    let mut done = 0usize;
    while done < src.len() {
        let va = dst + done as u64;
        let page_off = (va & (PAGE_SIZE - 1)) as usize;
        let n = (src.len() - done).min(PAGE_SIZE as usize - page_off);
        let Some(phys) = (unsafe { walk_user_write(pml4, va) }) else {
            return false;
        };
        unsafe {
            core::ptr::copy_nonoverlapping(
                src.as_ptr().add(done),
                pmm::phys_to_virt(phys) as *mut u8,
                n,
            );
        }
        done += n;
    }
    true
}

/// Software walk for a ring-3 write of `virt` through `pml4_phys`.
/// Same as `walk_user`, but also demands the `WRITABLE` bit at the leaf
/// -- writing through a read-only user mapping should fail cleanly, not
/// silently succeed.
unsafe fn walk_user_write(pml4_phys: u64, virt: u64) -> Option<u64> {
    let pml4 = table_ptr(pml4_phys);
    let e = (*pml4).0[index(virt, 0)];
    if e & (PRESENT | USER) != (PRESENT | USER) {
        return None;
    }
    let pdpt = table_ptr(e & ADDR_MASK);
    let e = (*pdpt).0[index(virt, 1)];
    if e & (PRESENT | USER) != (PRESENT | USER) || e & HUGE != 0 {
        return None;
    }
    let pd = table_ptr(e & ADDR_MASK);
    let e = (*pd).0[index(virt, 2)];
    if e & (PRESENT | USER) != (PRESENT | USER) || e & HUGE != 0 {
        return None;
    }
    let pt = table_ptr(e & ADDR_MASK);
    let leaf = (*pt).0[index(virt, 3)];
    if leaf & (PRESENT | USER | WRITABLE) != (PRESENT | USER | WRITABLE) {
        return None;
    }
    Some((leaf & ADDR_MASK) + (virt & (PAGE_SIZE - 1)))
}

/// Create a new address space for a process: a private, empty lower
/// half plus the kernel's upper half, shared as described on
/// `AddressSpace`. Returns `None` if the kernel VMM isn't initialized
/// yet or if a physical frame for the new PML4 can't be allocated.
pub fn create_address_space() -> Option<AddressSpace> {
    let guard = VMM.lock();
    let kernel = guard.as_ref()?;
    let kernel_pml4 = table_ptr(kernel.pml4_phys);

    let pml4_phys = pmm::alloc_frame_zeroed()?;
    let pml4 = table_ptr(pml4_phys);
    unsafe {
        for i in KERNEL_PML4_START..ENTRIES {
            (*pml4).0[i] = (*kernel_pml4).0[i];
        }
    }

    Some(AddressSpace { pml4_phys })
}

pub fn self_test() {
    if !is_active() {
        log_fail!("VMM", "SelfTest", "VMM not initialized");
        return;
    }

    let Some(phys) = pmm::alloc_frame() else {
        log_fail!("VMM", "SelfTest", "No free frame to test with");
        return;
    };

    map_page(SELF_TEST_VIRT, phys, KERNEL_RW);

    let ok = unsafe {
        let ptr = SELF_TEST_VIRT as *mut u64;
        ptr.write_volatile(0xCAFE_F00D_1234_5678);
        let read_back = ptr.read_volatile();
        read_back == 0xCAFE_F00D_1234_5678 && translate(SELF_TEST_VIRT) == Some(phys)
    };

    unmap_page(SELF_TEST_VIRT);
    pmm::free_frame(phys);

    if translate(SELF_TEST_VIRT).is_some() {
        log_fail!("VMM", "SelfTest", "Page survived unmap at {:#x}", SELF_TEST_VIRT);
        return;
    }

    if ok {
        log_ok!(
            "VMM",
            "SelfTest",
            "Mapped, wrote, translated and unmapped a test page ({:#x} -> {:#x})",
            SELF_TEST_VIRT,
            phys
        );
    } else {
        log_fail!(
            "VMM",
            "SelfTest",
            "Mapping/translation mismatch for {:#x}",
            SELF_TEST_VIRT
        );
    }
}

pub fn address_space_self_test() {
    if !is_active() {
        log_fail!("VMM", "AddressSpaceSelfTest", "VMM not initialized");
        return;
    }

    let frames_before = pmm::free_frame_count();

    let Some(aspace) = create_address_space() else {
        log_fail!("VMM", "AddressSpaceSelfTest", "create_address_space failed");
        return;
    };

    // Lower half must start out completely empty.
    unsafe {
        let pml4 = table_ptr(aspace.pml4_phys);
        for i in 0..KERNEL_PML4_START {
            if (*pml4).0[i] != 0 {
                log_fail!(
                    "VMM",
                    "AddressSpaceSelfTest",
                    "User half not empty at PML4[{}]",
                    i
                );
                return;
            }
        }
    }

    // Upper half must be byte-for-byte the kernel's own entries, i.e.
    // pointers to the exact same PDPTs, not fresh copies.
    let kernel_matches = {
        let guard = VMM.lock();
        let kernel_pml4 = table_ptr(guard.as_ref().unwrap().pml4_phys);
        let new_pml4 = table_ptr(aspace.pml4_phys);
        unsafe {
            (KERNEL_PML4_START..ENTRIES)
                .all(|i| (*kernel_pml4).0[i] == (*new_pml4).0[i])
        }
    };
    if !kernel_matches {
        log_fail!(
            "VMM",
            "AddressSpaceSelfTest",
            "Kernel half diverges from the kernel's own PML4"
        );
        return;
    }

    // A software walk through the new address space must already agree
    // with the kernel's own translation for a kernel address, with no
    // activation needed -- confirming the sharing is real, not just a
    // one-time value copy that happened to match.
    let probe = addr_of!(__kernel_text_start) as u64;
    if aspace.translate(probe) != translate(probe) {
        log_fail!(
            "VMM",
            "AddressSpaceSelfTest",
            "New address space disagrees with the kernel on kernel text at {:#x}",
            probe
        );
        return;
    }

    // Map a page into the new address space's *private* half, through
    // `AddressSpace::map` (not the free-standing `map_page`, which only
    // ever touches the kernel VMM's own PML4).
    let Some(user_phys) = pmm::alloc_frame() else {
        log_fail!("VMM", "AddressSpaceSelfTest", "No free frame to test with");
        return;
    };
    aspace.map(SELF_TEST_USER_VIRT, user_phys, USER_RW);

    // Actually switch into it: if the kernel half weren't really shared,
    // the very next instruction fetch after loading CR3 would triple
    // fault the machine instead of returning here.
    let old_cr3 = unsafe { read_cr3() };
    unsafe { aspace.activate() };

    let ok = unsafe {
        let ptr = SELF_TEST_USER_VIRT as *mut u64;
        ptr.write_volatile(0xA11C_0A11_C0DE_5A5A);
        ptr.read_volatile() == 0xA11C_0A11_C0DE_5A5A
    };

    // Switch back before touching anything else: kernel code and data
    // must be exactly as usable as before, since it never actually
    // moved -- only the private half changed underneath it.
    unsafe { write_cr3(old_cr3) };
    log_debug!(
        "VMM",
        "AddressSpaceSelfTest",
        "Restored kernel CR3 {:#x} after running with {:#x} active",
        old_cr3,
        aspace.pml4_phys
    );

    if !ok {
        log_fail!(
            "VMM",
            "AddressSpaceSelfTest",
            "Private-half mapping did not read back correctly while active"
        );
        return;
    }

    // Dropping tears down the private half -- including the page tables
    // `map` just allocated and the data frame itself -- without going
    // anywhere near the shared kernel half.
    drop(aspace);

    let frames_after = pmm::free_frame_count();
    if frames_after != frames_before {
        log_fail!(
            "VMM",
            "AddressSpaceSelfTest",
            "Frames leaked: {} free before, {} after",
            frames_before,
            frames_after
        );
        return;
    }

    log_ok!(
        "VMM",
        "AddressSpaceSelfTest",
        "New address space shares the kernel half (verified by walk and by live CR3 switch), private half torn down cleanly ({} frames free)",
        frames_after
    );
}