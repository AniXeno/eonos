//! Just enough ACPI to find the MADT (Multiple APIC Description Table)
//! and pull out of it what `lapic::init` and `ioapic::init` need: the Local APIC's MMIO
//! base, every I/O APIC's MMIO base and GSI (Global System Interrupt)
//! base, and every legacy ISA IRQ that's been remapped to a different
//! GSI than its own number.
//!
//! That last part matters more than it sounds like it should: on real
//! hardware, IRQ0 (the PIT) is very often wired to GSI 2, not GSI 0 --
//! programming the I/O APIC's redirection entry for GSI 0 when the
//! timer is actually sitting on GSI 2 looks identical to "the timer
//! doesn't work" (no crash, no error, just silence), which is exactly
//! the failure this module exists to avoid repeating in a new form.
//!
//! No AML interpretation, no other ACPI tables, no shutdown/sleep
//! support -- this reads three or four fixed-layout tables and stops.

#![allow(dead_code)]

use crate::vmm;
use crate::{log_debug, log_fail, log_ok};
use limine::request::RsdpRequest;

/// Every ACPI structure this module reads is accessed through this,
/// not `pmm::phys_to_virt` directly: firmware commonly reports the
/// RSDP's own memory (the EBDA, or the top of the BIOS ROM area below
/// 1 MiB) as plain `Reserved`, which `pmm::hhdm_ranges` deliberately
/// excludes from the direct map (see its doc comment) -- the same gap
/// that made the xHCI driver's first draft read unmapped memory for
/// its BAR. `map_mmio` is technically for device registers, but its
/// actual job here -- "make sure this physical range has a valid,
/// present mapping before touching it" -- is exactly what ACPI tables
/// in reserved memory also need; the cacheability difference doesn't
/// matter for data that's only ever read once at boot and never
/// written by hardware afterward.
fn map_table(phys: u64, len: u64) -> *const u8 {
    vmm::map_mmio(phys, len) as *const u8
}

static RSDP_REQUEST: RsdpRequest = RsdpRequest::new();

/// Root System Description Pointer (ACPI 5.2.5). The v1 (ACPI 1.0)
/// layout is the first 20 bytes; v2+ extends it with the fields below
/// `length`. Only read as v2 when `revision >= 2` says the extra fields
/// are actually present -- reading past a v1 RSDP would just be
/// whatever memory happens to follow it.
#[repr(C, packed)]
struct RsdpV1 {
    signature: [u8; 8], // "RSD PTR "
    checksum: u8,
    oem_id: [u8; 6],
    revision: u8,
    rsdt_address: u32,
}

#[repr(C, packed)]
struct RsdpV2 {
    v1: RsdpV1,
    length: u32,
    xsdt_address: u64,
    extended_checksum: u8,
    _reserved: [u8; 3],
}

/// Common header every ACPI table (RSDT, XSDT, MADT, ...) starts with
/// (ACPI 5.2.6).
#[repr(C, packed)]
struct SdtHeader {
    signature: [u8; 4],
    length: u32,
    revision: u8,
    checksum: u8,
    oem_id: [u8; 6],
    oem_table_id: [u8; 8],
    oem_revision: u32,
    creator_id: u32,
    creator_revision: u32,
}

fn checksum_ok(bytes: &[u8]) -> bool {
    bytes.iter().fold(0u8, |acc, &b| acc.wrapping_add(b)) == 0
}

/// What `apic::init` needs out of the MADT. `ioapics` and `overrides`
/// are fixed-size arrays rather than a heap `Vec` -- this runs before
/// the heap is guaranteed to be a good idea to lean on for something
/// this early and this small (real machines have one, occasionally
/// two, I/O APICs, and a handful of overrides at most).
pub struct MadtInfo {
    pub local_apic_addr: u64,
    pub ioapics: [IoApicInfo; MAX_IOAPICS],
    pub ioapic_count: usize,
    pub overrides: [IsaOverride; MAX_OVERRIDES],
    pub override_count: usize,
}

const MAX_IOAPICS: usize = 4;
const MAX_OVERRIDES: usize = 16;

#[derive(Clone, Copy)]
pub struct IoApicInfo {
    pub id: u8,
    pub mmio_base: u32,
    pub gsi_base: u32,
}

#[derive(Clone, Copy)]
pub struct IsaOverride {
    pub isa_irq: u8,
    pub gsi: u32,
    /// Raw MPS INTI flags (ACPI 5.2.12.5, bits 0:1 polarity, bits 2:3
    /// trigger mode). `ioapic::redirect` needs these to program the
    /// I/O APIC entry correctly -- an override that changes the GSI
    /// number but not the polarity/trigger mode is common, but the
    /// flags can legitimately differ from the ISA default too (0 means
    /// "use the bus default", which for ISA is active-high edge-
    /// triggered).
    pub flags: u16,
}

impl MadtInfo {
    /// Look up whether ISA IRQ `irq` has been remapped to a different
    /// GSI, returning the GSI and override flags if so.
    pub fn override_for(&self, irq: u8) -> Option<(u32, u16)> {
        self.overrides[..self.override_count]
            .iter()
            .find(|o| o.isa_irq == irq)
            .map(|o| (o.gsi, o.flags))
    }
}

/// Find and parse the MADT. Returns `None` (logging why) if Limine
/// didn't hand us an RSDP, any checksum fails, or the MADT is simply
/// absent -- all of which mean the caller should fall back to the
/// legacy PIC rather than proceeding with IOAPIC/LAPIC setup on data
/// that can't be trusted.
pub fn find_madt() -> Option<MadtInfo> {
    let response = RSDP_REQUEST.get_response()?;
    // `address()` matches this crate's other responses' pattern
    // (`HhdmResponse::offset()`, etc). If this doesn't compile as-is,
    // the field/method has a different name or return type in the
    // exact `limine` version pinned in Cargo.toml than expected here --
    // check `response`'s actual type (`cargo doc --open` or the error
    // message will show it) and adjust just this one line; everything
    // below only needs `rsdp_addr: u64`.
    let rsdp_addr = response.address() as u64;
    if rsdp_addr == 0 {
        log_fail!("ACPI", "Init", "Limine gave no RSDP address");
        return None;
    }

    // RSDP is at most 36 bytes (v2); 4 KiB is a trivially generous
    // single-page mapping for it.
    let rsdp_base = map_table(rsdp_addr, 4096);
    let v1 = unsafe { &*(rsdp_base as *const RsdpV1) };
    if &v1.signature != b"RSD PTR " {
        log_fail!("ACPI", "Init", "RSDP signature mismatch -- refusing to trust it");
        return None;
    }

    // Table address: XSDT (64-bit pointers) on ACPI 2.0+, RSDT
    // (32-bit) otherwise. Prefer XSDT when the revision says it's
    // present and its own checksum is valid; fall back to RSDT
    // otherwise rather than failing outright, since plenty of real
    // firmware is ACPI 2.0+ but still ships a valid RSDT alongside.
    let mut sdt_addr = v1.rsdt_address as u64;
    let mut sdt_is_xsdt = false;
    if v1.revision >= 2 {
        let v2 = unsafe { &*(rsdp_base as *const RsdpV2) };
        let len = v2.length as usize;
        let v2_ok = (core::mem::size_of::<RsdpV2>()..=4096).contains(&len)
            && checksum_ok(unsafe { core::slice::from_raw_parts(rsdp_base, len) });
        if v2_ok && v2.xsdt_address != 0 {
            sdt_addr = v2.xsdt_address;
            sdt_is_xsdt = true;
        } else {
            let v1_bytes = unsafe { core::slice::from_raw_parts(rsdp_base, core::mem::size_of::<RsdpV1>()) };
            if !checksum_ok(v1_bytes) {
                log_fail!("ACPI", "Init", "RSDP v1 checksum failed -- refusing to trust it");
                return None;
            }
            log_debug!("ACPI", "Init", "RSDP v2 checksum failed or no XSDT -- falling back to RSDT");
        }
    } else {
        let v1_bytes = unsafe { core::slice::from_raw_parts(rsdp_base, core::mem::size_of::<RsdpV1>()) };
        if !checksum_ok(v1_bytes) {
            log_fail!("ACPI", "Init", "RSDP v1 checksum failed -- refusing to trust it");
            return None;
        }
    }

    if sdt_addr == 0 {
        log_fail!("ACPI", "Init", "No usable RSDT/XSDT address");
        return None;
    }

    // 64 KiB is comfortably larger than any real RSDT/XSDT (a table of
    // pure 8-byte pointers to other tables -- even a machine with a
    // hundred ACPI tables fits in a small fraction of this).
    let sdt_base = map_table(sdt_addr, 0x10000);
    let sdt_header = unsafe { &*(sdt_base as *const SdtHeader) };
    let sdt_len = sdt_header.length as usize;
    let expected_sig = if sdt_is_xsdt { b"XSDT" } else { b"RSDT" };
    if &sdt_header.signature != expected_sig
        || sdt_len < core::mem::size_of::<SdtHeader>()
        || sdt_len > 0x10000
        || (sdt_len - core::mem::size_of::<SdtHeader>()) % if sdt_is_xsdt { 8 } else { 4 } != 0
    {
        log_fail!("ACPI", "Init", "Root table has an invalid signature or length");
        return None;
    }
    let sdt_bytes = unsafe { core::slice::from_raw_parts(sdt_base, sdt_len) };
    if !checksum_ok(sdt_bytes) {
        log_fail!("ACPI", "Init", "{} checksum failed", if sdt_is_xsdt { "XSDT" } else { "RSDT" });
        return None;
    }

    let entries_start = core::mem::size_of::<SdtHeader>();
    let entry_size = if sdt_is_xsdt { 8 } else { 4 };
    let mut offset = entries_start;
    let mut madt_addr = None;
    while offset < sdt_len {
        // The XSDT payload begins immediately after its 36-byte header,
        // so its u64 entries are not necessarily naturally aligned.
        let addr = unsafe {
            if sdt_is_xsdt {
                core::ptr::read_unaligned(sdt_base.add(offset) as *const u64)
            } else {
                core::ptr::read_unaligned(sdt_base.add(offset) as *const u32) as u64
            }
        };
        if table_signature_is(addr, b"APIC") {
            madt_addr = Some(addr);
            break;
        }
        offset += entry_size;
    }

    let Some(madt_addr) = madt_addr else {
        log_fail!("ACPI", "Init", "No MADT (APIC table) found in the {}", if sdt_is_xsdt { "XSDT" } else { "RSDT" });
        return None;
    };

    parse_madt(madt_addr)
}

fn table_signature_is(phys_addr: u64, sig: &[u8; 4]) -> bool {
    if phys_addr == 0 {
        return false;
    }
    // Only the header (36 bytes) is needed to check the signature; a
    // full page is mapped anyway since map_mmio works in page
    // granularity, so there's no reason to ask for less.
    let base = map_table(phys_addr, 4096);
    let header = unsafe { &*(base as *const SdtHeader) };
    &header.signature == sig
}

/// MADT-specific header fields that follow the common `SdtHeader`
/// (ACPI 5.2.12): the Local APIC's own MMIO address, plus flags this
/// driver doesn't need.
#[repr(C, packed)]
struct MadtHeader {
    sdt: SdtHeader,
    local_apic_addr: u32,
    flags: u32,
    // Variable-length list of entries follows, each starting with a
    // (entry_type: u8, entry_length: u8) pair.
}

const MADT_ENTRY_IOAPIC: u8 = 1;
const MADT_ENTRY_ISA_OVERRIDE: u8 = 2;
const MADT_ENTRY_LOCAL_APIC_OVERRIDE: u8 = 5;

fn parse_madt(madt_addr: u64) -> Option<MadtInfo> {
    // 64 KiB is comfortably larger than any real MADT (a few hundred
    // bytes per CPU/IOAPIC/override entry; even a large multi-socket
    // server's MADT is a few KiB at most).
    let base = map_table(madt_addr, 0x10000);
    let header = unsafe { &*(base as *const MadtHeader) };
    let total_len = header.sdt.length as usize;
    if &header.sdt.signature != b"APIC"
        || total_len < core::mem::size_of::<MadtHeader>()
        || total_len > 0x10000
    {
        log_fail!("ACPI", "Init", "MADT has an invalid signature or length");
        return None;
    }
    let sdt_bytes = unsafe { core::slice::from_raw_parts(base, total_len) };
    if !checksum_ok(sdt_bytes) {
        log_fail!("ACPI", "Init", "MADT checksum failed -- refusing to trust it");
        return None;
    }

    let mut info = MadtInfo {
        local_apic_addr: header.local_apic_addr as u64,
        ioapics: [IoApicInfo { id: 0, mmio_base: 0, gsi_base: 0 }; MAX_IOAPICS],
        ioapic_count: 0,
        overrides: [IsaOverride { isa_irq: 0, gsi: 0, flags: 0 }; MAX_OVERRIDES],
        override_count: 0,
    };

    let entries_start = core::mem::size_of::<MadtHeader>();
    let mut offset = entries_start;
    while offset + 2 <= total_len {
        let entry_type = unsafe { core::ptr::read_volatile(base.add(offset)) };
        let entry_len = unsafe { core::ptr::read_volatile(base.add(offset + 1)) } as usize;
        if entry_len < 2 || offset + entry_len > total_len {
            log_fail!("ACPI", "Init", "MADT contains a malformed entry");
            return None;
        }

        match entry_type {
            MADT_ENTRY_IOAPIC if entry_len >= 12 => {
                if info.ioapic_count < MAX_IOAPICS {
                    let id = unsafe { core::ptr::read_volatile(base.add(offset + 2)) };
                    let mmio_base = unsafe { core::ptr::read_unaligned(base.add(offset + 4) as *const u32) };
                    let gsi_base = unsafe { core::ptr::read_unaligned(base.add(offset + 8) as *const u32) };
                    info.ioapics[info.ioapic_count] = IoApicInfo { id, mmio_base, gsi_base };
                    info.ioapic_count += 1;
                }
            }
            MADT_ENTRY_ISA_OVERRIDE if entry_len >= 10 => {
                if info.override_count < MAX_OVERRIDES {
                    let isa_irq = unsafe { core::ptr::read_volatile(base.add(offset + 3)) };
                    let gsi = unsafe { core::ptr::read_unaligned(base.add(offset + 4) as *const u32) };
                    let flags = unsafe { core::ptr::read_unaligned(base.add(offset + 8) as *const u16) };
                    info.overrides[info.override_count] = IsaOverride { isa_irq, gsi, flags };
                    info.override_count += 1;
                }
            }
            MADT_ENTRY_LOCAL_APIC_OVERRIDE if entry_len >= 12 => {
                // 64-bit Local APIC address override -- some machines
                // need this instead of (or in addition to) the 32-bit
                // one in the MADT header, though it's rare outside
                // exotic/large-memory configurations.
                let addr64 = unsafe { core::ptr::read_unaligned(base.add(offset + 4) as *const u64) };
                if addr64 != 0 {
                    info.local_apic_addr = addr64;
                }
            }
            _ => {}
        }

        offset += entry_len;
    }

    if info.ioapic_count == 0 {
        log_fail!("ACPI", "Init", "MADT parsed but no I/O APIC entries found");
        return None;
    }

    log_ok!(
        "ACPI",
        "Init",
        "MADT parsed: Local APIC at {:#x}, {} I/O APIC(s), {} ISA override(s)",
        info.local_apic_addr,
        info.ioapic_count,
        info.override_count
    );

    Some(info)
}
