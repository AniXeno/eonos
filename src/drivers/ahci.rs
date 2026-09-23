//! Read-only synchronous AHCI/SATA block driver.

use alloc::boxed::Box;
use alloc::vec::Vec;
use core::sync::atomic::{fence, Ordering};
use crate::block::{BlockDevice, BlockError};
use crate::fat32::SECTOR_SIZE;
use spin::Mutex;

const CF8: u16 = 0xcf8;
const CFC: u16 = 0xcfc;
const PORT_BASE: usize = 0x100;
const PORT_STRIDE: usize = 0x80;
const TIMEOUT_MS: u64 = 3000;

#[derive(Clone, Copy)] struct PciLoc { bus: u8, dev: u8, fun: u8 }
unsafe fn outl(port: u16, val: u32) { core::arch::asm!("out dx, eax", in("dx") port, in("eax") val, options(nomem, nostack, preserves_flags)); }
unsafe fn inl(port: u16) -> u32 { let v: u32; core::arch::asm!("in eax, dx", in("dx") port, out("eax") v, options(nomem, nostack, preserves_flags)); v }
unsafe fn outw(port: u16, val: u16) { core::arch::asm!("out dx, ax", in("dx") port, in("ax") val, options(nomem, nostack, preserves_flags)); }
fn pci_addr(l: PciLoc, off: u8) -> u32 { (1 << 31) | ((l.bus as u32) << 16) | ((l.dev as u32) << 11) | ((l.fun as u32) << 8) | (off as u32 & 0xfc) }
fn pci_read(l: PciLoc, off: u8) -> u32 { unsafe { outl(CF8, pci_addr(l, off)); inl(CFC) } }

fn pci_ahci_controllers() -> Vec<(PciLoc, u64)> {
    let mut found = Vec::new();
    for bus in 0..=255u16 { for dev in 0..32u8 {
        let l = PciLoc { bus: bus as u8, dev, fun: 0 };
        if pci_read(l, 0) as u16 == 0xffff { continue; }
        let multi = (pci_read(l, 0x0c) >> 23) & 1 != 0;
        for fun in 0..if multi { 8 } else { 1 } {
            let l = PciLoc { fun, ..l };
            if pci_read(l, 0) as u16 == 0xffff || pci_read(l, 8) >> 8 & 0x00ff_ffff != 0x0106_01 { continue; }
            let bar = pci_read(l, 0x24);
            if bar & 1 != 0 { continue; }
            let mut base = (bar & !0xf) as u64;
            if (bar >> 1) & 3 == 2 { base |= (pci_read(l, 0x28) as u64) << 32; }
            if base == 0 { continue; }
            let command = (pci_read(l, 4) as u16) | 0x6;
            unsafe { outl(CF8, pci_addr(l, 4)); outw(CFC, command); }
            found.push((l, base));
        }
    }}
    found
}

#[repr(C, align(1024))] struct CommandList([u8; 1024]);
#[repr(C, align(256))] struct FisArea([u8; 256]);
#[repr(C, align(128))] struct CommandTable([u8; 512]);
#[repr(C, align(4096))] struct DataPage([u8; 4096]);

struct DmaMem<T> { phys: u64, ptr: *mut T }
unsafe impl<T> Send for DmaMem<T> {}
fn dma_mem<T>(zero: bool) -> Option<DmaMem<T>> {
    let phys = crate::pmm::alloc_contiguous(core::mem::size_of::<T>().div_ceil(4096))?;
    let ptr = crate::pmm::phys_to_virt(phys) as *mut T;
    if zero { unsafe { core::ptr::write_bytes(ptr.cast::<u8>(), 0, core::mem::size_of::<T>()); } }
    Some(DmaMem { phys, ptr })
}

struct AhciDisk {
    port: *mut u32,
    lock: Mutex<()>,
    _cl: DmaMem<CommandList>,
    _fis: DmaMem<FisArea>,
    table: DmaMem<CommandTable>,
    data: DmaMem<DataPage>,
    sectors: u64,
}
unsafe impl Send for AhciDisk {}
unsafe impl Sync for AhciDisk {}

fn rd(p: *mut u32, n: usize) -> u32 { unsafe { core::ptr::read_volatile(p.add(n)) } }
fn wr(p: *mut u32, n: usize, v: u32) { unsafe { core::ptr::write_volatile(p.add(n), v); } }
fn spin_until(start: u64, timeout: u64, mut predicate: impl FnMut() -> bool) -> bool {
    while crate::pit::uptime_ms().saturating_sub(start) < timeout { if predicate() { return true; } core::hint::spin_loop(); }
    predicate()
}

impl AhciDisk {
    fn new(port: *mut u32, supports_64bit_dma: bool) -> Option<Self> {
        let cl = dma_mem::<CommandList>(true)?;
        let fis = dma_mem::<FisArea>(true)?;
        let table = dma_mem::<CommandTable>(true)?;
        let data = dma_mem::<DataPage>(true)?;
        if !supports_64bit_dma && [cl.phys, fis.phys, table.phys, data.phys].iter().any(|&p| p >= (1u64 << 32)) {
            return None;
        }
        // Stop the command engine before replacing its DMA pointers.
        let cmd = rd(port, 0x18 / 4);
        wr(port, 0x18 / 4, cmd & !1);
        let now = crate::pit::uptime_ms();
        if !spin_until(now, TIMEOUT_MS, || rd(port, 0x18 / 4) & (1 << 15) == 0) { return None; }
        wr(port, 0x18 / 4, rd(port, 0x18 / 4) & !(1 << 4));
        if !spin_until(crate::pit::uptime_ms(), TIMEOUT_MS, || rd(port, 0x18 / 4) & (1 << 14) == 0) { return None; }
        wr(port, 0x00 / 4, cl.phys as u32);
        wr(port, 0x04 / 4, (cl.phys >> 32) as u32);
        wr(port, 0x08 / 4, fis.phys as u32);
        wr(port, 0x0c / 4, (fis.phys >> 32) as u32);
        // One PRDT entry, command FIS length 5 dwords, read direction.
        let h = unsafe { &mut (&mut (*cl.ptr).0)[..32] };
        h[0..2].copy_from_slice(&5u16.to_le_bytes());
        h[2..4].copy_from_slice(&1u16.to_le_bytes());
        let table_addr = table.phys.to_le_bytes();
        h[8..16].copy_from_slice(&table_addr);
        let t = unsafe { &mut (*table.ptr).0 };
        t[0x80..0x88].copy_from_slice(&data.phys.to_le_bytes());
        t[0x8c..0x90].copy_from_slice(&((SECTOR_SIZE as u32 - 1) | (1 << 31)).to_le_bytes());
        wr(port, 0x10 / 4, 0xffff_ffff);
        wr(port, 0x14 / 4, 0xffff_ffff);
        wr(port, 0x30 / 4, 0xffff_ffff);
        let cmd = rd(port, 0x18 / 4);
        wr(port, 0x18 / 4, cmd | (1 << 4));
        if !spin_until(crate::pit::uptime_ms(), TIMEOUT_MS, || rd(port, 0x18 / 4) & (1 << 14) != 0) { return None; }
        wr(port, 0x18 / 4, cmd | (1 << 4) | 1);
        let mut disk = Self { port, lock: Mutex::new(()), _cl: cl, _fis: fis, table, data, sectors: 0 };
        let mut identify = [0u8; 512];
        if !disk.transfer(0xec, 0, 1, &mut identify) { return None; }
        let sectors = u64::from_le_bytes(identify[200..208].try_into().ok()?);
        if sectors == 0 { return None; }
        disk.sectors = sectors;
        Some(disk)
    }

    fn transfer(&self, command: u8, lba: u64, count: u16, out: &mut [u8; 512]) -> bool {
        let _guard = self.lock.lock();
        let p = self.port;
        if rd(p, 0x20 / 4) & 0x88 != 0 { return false; }
        unsafe { core::ptr::write_bytes(self.data.ptr.cast::<u8>(), 0, 4096); }
        let fis = unsafe { &mut (&mut (*self.table.ptr).0)[0..20] };
        fis.fill(0);
        fis[0] = 0x27; fis[1] = 0x80; fis[2] = command;
        if command == 0x25 {
            fis[4] = lba as u8; fis[5] = (lba >> 8) as u8; fis[6] = (lba >> 16) as u8;
            fis[7] = 1 << 6; fis[8] = (lba >> 24) as u8; fis[9] = (lba >> 32) as u8; fis[10] = (lba >> 40) as u8;
            fis[12..14].copy_from_slice(&count.to_le_bytes());
        }
        fence(Ordering::SeqCst);
        wr(p, 0x30 / 4, 0xffff_ffff);
        wr(p, 0x38 / 4, 1);
        let start = crate::pit::uptime_ms();
        let completed = spin_until(start, TIMEOUT_MS, || rd(p, 0x38 / 4) & 1 == 0 || rd(p, 0x10 / 4) & 1 != 0);
        let status = rd(p, 0x10 / 4);
        if !completed || status & 1 != 0 { return false; }
        fence(Ordering::SeqCst);
        out.copy_from_slice(unsafe { &(&(*self.data.ptr).0)[..512] });
        true
    }
}

impl BlockDevice for AhciDisk {
    fn sector_count(&self) -> u64 { self.sectors }
    fn read_sector(&self, lba: u64, out: &mut [u8; SECTOR_SIZE]) -> Result<(), BlockError> {
        if lba >= self.sectors { return Err(BlockError::OutOfRange); }
        if self.transfer(0x25, lba, 1, out) { Ok(()) } else { Err(BlockError::BadBuffer) }
    }
}

pub fn init() {
    let mut index = 0usize;
    let controllers = pci_ahci_controllers();
    if controllers.is_empty() {
        crate::log_ok!("AHCI", "Init", "No AHCI PCI controller found");
        return;
    }
    for (loc, base) in controllers {
        crate::log_ok!("AHCI", "Init", "Controller at {:02x}:{:02x}.{} BAR5 {:#x}", loc.bus, loc.dev, loc.fun, base);
        let hba = crate::vmm::map_mmio(base, 0x1100) as *mut u8;
        let caps = unsafe { core::ptr::read_volatile(hba.cast::<u32>()) };
        let supports_64bit_dma = caps & (1 << 31) != 0;
        let ghc = unsafe { hba.add(4).cast::<u32>() };
        unsafe { core::ptr::write_volatile(ghc, core::ptr::read_volatile(ghc) | (1 << 31)); }
        let pi = unsafe { core::ptr::read_volatile(hba.add(0x0c).cast::<u32>()) };
        for port_index in 0..32usize {
            if pi & (1 << port_index) == 0 { continue; }
            let port = unsafe { hba.add(PORT_BASE + PORT_STRIDE * port_index).cast::<u32>() };
            let ssts = unsafe { core::ptr::read_volatile(port.add(0x28 / 4)) };
            let sig = unsafe { core::ptr::read_volatile(port.add(0x24 / 4)) };
            if ssts & 0xf != 3 || (ssts >> 8) & 0xf != 1 || sig != 0x0000_0101 { continue; }
            let Some(disk) = AhciDisk::new(port, supports_64bit_dma) else { continue; };
            let disk: &'static dyn BlockDevice = Box::leak(Box::new(disk));
            let name = alloc::format!("sd{}", (b'a' + index as u8) as char);
            let name: &'static str = Box::leak(name.into_boxed_str());
            crate::block::register_device(name, disk);
            crate::log_ok!("AHCI", "Disk", "{}: SATA disk at {} sectors ({})", name, disk.sector_count(), base);
            index += 1;
        }
    }
    if index == 0 { crate::log_ok!("AHCI", "Init", "No usable SATA disks found"); }
}
