#![allow(dead_code)]

use core::arch::asm;
use core::fmt;
use core::ptr::addr_of;

use limine::request::KernelAddressRequest;
use spin::Mutex;

use crate::pmm::{self, PAGE_SIZE};
use crate::{log_debug, log_fail, log_ok};

const PRESENT: u64 = 1 << 0;
const WRITABLE: u64 = 1 << 1;
const PWT: u64 = 1 << 3;
const HUGE: u64 = 1 << 7;
const NO_EXECUTE: u64 = 1 << 63;

pub const KERNEL_RX: u64 = PRESENT;
pub const KERNEL_RO: u64 = PRESENT | NO_EXECUTE;
pub const KERNEL_RW: u64 = PRESENT | WRITABLE | NO_EXECUTE;
/// Write-combining: for framebuffers and other linear MMIO the CPU may
/// buffer and merge writes to. See `configure_pat` for how the PWT bit
/// ends up meaning "write-combining" instead of its default "write-
/// through". Never use this for memory that's read back right after
/// being written (WC writes can sit in a fill buffer for a while) or for
/// device registers with side effects on individual writes.
pub const KERNEL_WC: u64 = PRESENT | WRITABLE | NO_EXECUTE | PWT;

const ADDR_MASK: u64 = 0x000f_ffff_ffff_f000;
const ENTRIES: usize = 512;
const HUGE_2M: u64 = 0x20_0000;

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


unsafe fn child_table(parent: *mut Table, idx: usize) -> *mut Table {
    let entry = (*parent).0[idx];
    if entry & PRESENT != 0 {
        return table_ptr(entry & ADDR_MASK);
    }
    let phys = pmm::alloc_frame_zeroed().expect("VMM: out of memory for page tables");
    (*parent).0[idx] = phys | PRESENT | WRITABLE;
    table_ptr(phys)
}

unsafe fn map4k(pml4: *mut Table, virt: u64, phys: u64, flags: u64) {
    let pdpt = child_table(pml4, index(virt, 0));
    let pd = child_table(pdpt, index(virt, 1));
    let pt = child_table(pd, index(virt, 2));
    (*pt).0[index(virt, 3)] = (phys & ADDR_MASK) | flags | PRESENT;
}

unsafe fn map2m(pml4: *mut Table, virt: u64, phys: u64, flags: u64) {
    let pdpt = child_table(pml4, index(virt, 0));
    let pd = child_table(pdpt, index(virt, 1));
    (*pd).0[index(virt, 2)] = (phys & ADDR_MASK) | flags | PRESENT | HUGE;
}

pub fn map_page(virt: u64, phys: u64, flags: u64) {
    let mut guard = VMM.lock();
    let Some(vmm) = guard.as_mut() else { return };
    unsafe { map4k(table_ptr(vmm.pml4_phys), virt, phys, flags) };
}

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

pub fn is_active() -> bool {
    VMM.lock().is_some()
}

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

    unsafe {
        let mut phys = 0u64;
        while phys < phys_top {
            map2m(pml4, hhdm + phys, phys, KERNEL_RW);
            phys += HUGE_2M;
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

    log_ok!(
        "VMM",
        "Init",
        "Own page tables active ({} physical mapped, cr3 {:#x} -> {:#x}, PAT slot 1 = write-combining)",
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