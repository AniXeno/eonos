//! Virtual memory manager: kernel-owned 4-level x86_64 paging.
//!
//! Limine hands the kernel a working set of page tables to boot with, but
//! they're Limine's, living in `BOOTLOADER_RECLAIMABLE` memory. This module
//! builds a fresh PML4 and switches `CR3` to it, so the kernel is no longer
//! borrowing the bootloader's mappings.
//!
//! Three things get mapped into the new tables before the switch:
//!
//! * The kernel image itself, at the physical/virtual base Limine reports,
//!   with per-section permissions: `.text` is read+execute, `.rodata` is
//!   read-only, `.data`/`.bss` are read+write, and everything except
//!   `.text` is marked no-execute (NX).
//! * All physical memory the PMM manages (usable RAM plus still-reclaimable
//!   bootloader memory), direct-mapped 1:1 at `phys + hhdm_offset` using
//!   2 MiB pages. This matches Limine's own HHDM, so nothing that already
//!   holds an HHDM pointer (e.g. `pmm::phys_to_virt`) needs to change.
//! * The framebuffer's physical range, at the same HHDM-relative address
//!   Limine already gave `framebuffer::FbInfo::addr`, so the console and
//!   framebuffer code keep working unmodified across the switch.
//!
//! `BOOTLOADER_RECLAIMABLE` memory (Limine's own page tables, GDT, and the
//! stack we boot on) is covered by the direct-map range above, so it stays
//! readable/writable until `pmm::reclaim_bootloader_memory` hands it to the
//! frame allocator. Nothing here reads Limine's page tables directly once
//! `CR3` has been switched.

#![allow(dead_code)]

use core::arch::asm;
use core::fmt;
use core::ptr::addr_of;

use limine::request::KernelAddressRequest;
use spin::Mutex;

use crate::pmm::{self, PAGE_SIZE};
use crate::{log_debug, log_fail, log_ok};

// ---------------------------------------------------------------------------
// Page table entry flags
// ---------------------------------------------------------------------------

const PRESENT: u64 = 1 << 0;
const WRITABLE: u64 = 1 << 1;
const HUGE: u64 = 1 << 7; // PS bit: valid on PDPT/PD entries only
const NO_EXECUTE: u64 = 1 << 63;

/// Read+execute, not writable. For `.text`.
pub const KERNEL_RX: u64 = PRESENT;
/// Read-only, not executable. For `.rodata`.
pub const KERNEL_RO: u64 = PRESENT | NO_EXECUTE;
/// Read+write, not executable. For `.data`/`.bss`, the direct map, MMIO.
pub const KERNEL_RW: u64 = PRESENT | WRITABLE | NO_EXECUTE;

const ADDR_MASK: u64 = 0x000f_ffff_ffff_f000;
const ENTRIES: usize = 512;
const HUGE_2M: u64 = 0x20_0000;

/// A scratch virtual address used only by [`self_test`]. Chosen well past
/// the kernel image (which is a few MiB at most) but still inside the
/// canonical top-2GiB region the kernel lives in, so it needs no mapping
/// beyond what `map_page` creates on demand.
const SELF_TEST_VIRT: u64 = 0xffff_ffff_c000_0000;

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

static VMM: Mutex<Option<Vmm>> = Mutex::new(None);

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
// Table walking
// ---------------------------------------------------------------------------

fn table_ptr(phys: u64) -> *mut Table {
    pmm::phys_to_virt(phys) as *mut Table
}

/// Index into the table at `level` (0 = PML4 ... 3 = PT) for `virt`.
fn index(virt: u64, level: usize) -> usize {
    ((virt >> (39 - level * 9)) & 0x1FF) as usize
}

/// Return the child table at `index`, allocating and zeroing a fresh frame
/// for it if the slot is empty. Intermediate entries are always
/// present+writable; the *leaf* entry's own flags are what actually
/// restrict a mapping, so this never widens permissions.
unsafe fn child_table(parent: *mut Table, idx: usize) -> *mut Table {
    let entry = (*parent).0[idx];
    if entry & PRESENT != 0 {
        return table_ptr(entry & ADDR_MASK);
    }
    let phys = pmm::alloc_frame_zeroed().expect("VMM: out of memory for page tables");
    (*parent).0[idx] = phys | PRESENT | WRITABLE;
    table_ptr(phys)
}

/// Map one 4 KiB page, creating page tables as needed.
unsafe fn map4k(pml4: *mut Table, virt: u64, phys: u64, flags: u64) {
    let pdpt = child_table(pml4, index(virt, 0));
    let pd = child_table(pdpt, index(virt, 1));
    let pt = child_table(pd, index(virt, 2));
    (*pt).0[index(virt, 3)] = (phys & ADDR_MASK) | flags | PRESENT;
}

/// Map one 2 MiB huge page (at the PD level), creating tables as needed.
unsafe fn map2m(pml4: *mut Table, virt: u64, phys: u64, flags: u64) {
    let pdpt = child_table(pml4, index(virt, 0));
    let pd = child_table(pdpt, index(virt, 1));
    (*pd).0[index(virt, 2)] = (phys & ADDR_MASK) | flags | PRESENT | HUGE;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Map one 4 KiB page in the kernel's page tables. No-op if the VMM hasn't
/// been initialised yet.
pub fn map_page(virt: u64, phys: u64, flags: u64) {
    let mut guard = VMM.lock();
    let Some(vmm) = guard.as_mut() else { return };
    unsafe { map4k(table_ptr(vmm.pml4_phys), virt, phys, flags) };
}

/// Remove a single 4 KiB mapping (leaves any huge-page mapping covering
/// `virt` untouched -- this only tears down 4 KiB leaves).
pub fn unmap_page(virt: u64) {
    let guard = VMM.lock();
    let Some(vmm) = guard.as_ref() else { return };
    unsafe {
        let pml4 = table_ptr(vmm.pml4_phys);
        let pdpte = (*pml4).0[index(virt, 0)];
        if pdpte & PRESENT == 0 {
            return;
        }
        let pdpt = table_ptr(pdpte & ADDR_MASK);
        let pde = (*pdpt).0[index(virt, 1)];
        if pde & PRESENT == 0 || pde & HUGE != 0 {
            return;
        }
        let pd = table_ptr(pde & ADDR_MASK);
        let pte_entry = (*pd).0[index(virt, 2)];
        if pte_entry & PRESENT == 0 {
            return;
        }
        let pt = table_ptr(pte_entry & ADDR_MASK);
        (*pt).0[index(virt, 3)] = 0;
        invlpg(virt);
    }
}

/// Translate a virtual address to its mapped physical address, if any.
pub fn translate(virt: u64) -> Option<u64> {
    let guard = VMM.lock();
    let vmm = guard.as_ref()?;
    unsafe {
        let pml4 = table_ptr(vmm.pml4_phys);
        let pdpte = (*pml4).0[index(virt, 0)];
        if pdpte & PRESENT == 0 {
            return None;
        }
        let pdpt = table_ptr(pdpte & ADDR_MASK);
        let pde = (*pdpt).0[index(virt, 1)];
        if pde & PRESENT == 0 {
            return None;
        }
        if pde & HUGE != 0 {
            let base = pde & ADDR_MASK;
            return Some(base + (virt & (HUGE_2M - 1)));
        }
        let pd = table_ptr(pde & ADDR_MASK);
        let pte_entry = (*pd).0[index(virt, 2)];
        if pte_entry & PRESENT == 0 {
            return None;
        }
        let pt = table_ptr(pte_entry & ADDR_MASK);
        let leaf = (*pt).0[index(virt, 3)];
        if leaf & PRESENT == 0 {
            return None;
        }
        Some((leaf & ADDR_MASK) + (virt & (PAGE_SIZE - 1)))
    }
}

/// Whether the VMM has switched to its own page tables yet.
pub fn is_active() -> bool {
    VMM.lock().is_some()
}

// ---------------------------------------------------------------------------
// CPU helpers
// ---------------------------------------------------------------------------

unsafe fn read_cr3() -> u64 {
    let v: u64;
    asm!("mov {}, cr3", out(reg) v, options(nomem, nostack, preserves_flags));
    v
}

unsafe fn write_cr3(phys: u64) {
    asm!("mov cr3, {}", in(reg) phys, options(nostack, preserves_flags));
}

unsafe fn invlpg(virt: u64) {
    asm!("invlpg [{}]", in(reg) virt, options(nostack, preserves_flags));
}

/// Set `EFER.NXE`. Must happen before any page table entry has `NO_EXECUTE`
/// set, or using that entry raises a fault: the bit is simply reserved
/// (and therefore illegal to set) until the CPU is told to honour it.
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

// ---------------------------------------------------------------------------
// Initialisation
// ---------------------------------------------------------------------------

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

    unsafe {
        // 1. Direct-map all physical memory the PMM manages at
        //    `phys + hhdm`, using 2 MiB pages to keep table count small.
        let mut phys = 0u64;
        while phys < phys_top {
            map2m(pml4, hhdm + phys, phys, KERNEL_RW);
            phys += HUGE_2M;
        }

        // 2. Framebuffer MMIO: same HHDM-relative address Limine already
        //    used, so `FbInfo::addr` stays valid across the CR3 switch.
        if let Some(fb) = crate::framebuffer::FRAMEBUFFER.lock().as_ref() {
            if let Some(fb_phys) = (fb.addr as u64).checked_sub(hhdm) {
                let start = align_down(fb_phys, PAGE_SIZE);
                let end = align_up(fb_phys + fb.pitch * fb.height, PAGE_SIZE);
                let mut p = start;
                while p < end {
                    map4k(pml4, hhdm + p, p, KERNEL_RW);
                    p += PAGE_SIZE;
                }
            }
        }

        // 3. Kernel image, per-section permissions.
        map_kernel_sections(pml4, kernel_phys, kernel_virt);
    }

    let old_cr3 = unsafe { read_cr3() };
    unsafe { write_cr3(pml4_phys) };

    *VMM.lock() = Some(Vmm { pml4_phys });

    log_ok!(
        "VMM",
        "Init",
        "Own page tables active ({} physical mapped, cr3 {:#x} -> {:#x})",
        Size(phys_top),
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

/// Map a scratch page, write through it, translate it, then unmap it.
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