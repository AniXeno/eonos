//! MBR and GPT partition table discovery over a block device.

use alloc::vec::Vec;

use crate::block::BlockDevice;
use crate::fat32::SECTOR_SIZE;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TableKind { Mbr, Gpt }

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Partition {
    pub index: u32,
    pub start_lba: u64,
    pub sectors: u64,
    /// MBR type byte, or the low 32 bits of a GPT type GUID.
    pub type_tag: u32,
    pub table: TableKind,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PartitionError { Io, InvalidTable, Unsupported, Corrupt }

fn u32le(b: &[u8], at: usize) -> u32 {
    u32::from_le_bytes([b[at], b[at + 1], b[at + 2], b[at + 3]])
}
fn u64le(b: &[u8], at: usize) -> u64 {
    u64::from_le_bytes([b[at], b[at + 1], b[at + 2], b[at + 3], b[at + 4], b[at + 5], b[at + 6], b[at + 7]])
}

/// Scan the standard MBR entries or a protective MBR followed by GPT.
/// Extended MBR partitions are currently skipped.
pub fn scan(device: &dyn BlockDevice) -> Result<Vec<Partition>, PartitionError> {
    if device.sector_count() < 2 { return Err(PartitionError::InvalidTable); }
    let mut mbr = [0u8; SECTOR_SIZE];
    device.read_sector(0, &mut mbr).map_err(|_| PartitionError::Io)?;
    if mbr[510..512] != [0x55, 0xaa] { return Err(PartitionError::InvalidTable); }

    let mut protective = false;
    let mut parts = Vec::new();
    for index in 0..4usize {
        let at = 446 + index * 16;
        let kind = mbr[at + 4];
        let start = u32le(&mbr, at + 8) as u64;
        let sectors = u32le(&mbr, at + 12) as u64;
        if kind == 0 || sectors == 0 { continue; }
        if kind == 0xee { protective = true; continue; }
        if kind == 0x05 || kind == 0x0f || kind == 0x85 { continue; }
        let end = start.checked_add(sectors).ok_or(PartitionError::Corrupt)?;
        if start == 0 || end > device.sector_count() { return Err(PartitionError::Corrupt); }
        parts.push(Partition { index: index as u32 + 1, start_lba: start, sectors, type_tag: kind as u32, table: TableKind::Mbr });
    }
    if !protective { return Ok(parts); }
    scan_gpt(device)
}

fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = !0u32;
    for &byte in bytes {
        crc ^= byte as u32;
        for _ in 0..8 { crc = (crc >> 1) ^ (0xedb8_8320 & (0u32.wrapping_sub(crc & 1))); }
    }
    !crc
}

fn scan_gpt(device: &dyn BlockDevice) -> Result<Vec<Partition>, PartitionError> {
    let mut hdr = [0u8; SECTOR_SIZE];
    device.read_sector(1, &mut hdr).map_err(|_| PartitionError::Io)?;
    if &hdr[..8] != b"EFI PART" { return Err(PartitionError::InvalidTable); }
    let header_size = u32le(&hdr, 12) as usize;
    if !(92..=SECTOR_SIZE).contains(&header_size) { return Err(PartitionError::Corrupt); }
    let expected = u32le(&hdr, 16);
    let mut crc_hdr = hdr;
    crc_hdr[16..20].fill(0);
    if crc32(&crc_hdr[..header_size]) != expected { return Err(PartitionError::Corrupt); }
    let table_lba = u64le(&hdr, 72);
    let entries = u32le(&hdr, 80).min(4096) as usize;
    let entry_size = u32le(&hdr, 84) as usize;
    if entry_size < 128 || entry_size > 4096 || !entry_size.is_multiple_of(8) || entries == 0 {
        return Err(PartitionError::Unsupported);
    }
    let total_bytes = entries.checked_mul(entry_size).ok_or(PartitionError::Corrupt)?;
    let sectors = total_bytes.div_ceil(SECTOR_SIZE);
    if table_lba.checked_add(sectors as u64).is_none_or(|end| end > device.sector_count()) {
        return Err(PartitionError::Corrupt);
    }
    let mut raw = alloc::vec![0u8; sectors * SECTOR_SIZE];
    for i in 0..sectors {
        let mut sector = [0u8; SECTOR_SIZE];
        device.read_sector(table_lba + i as u64, &mut sector).map_err(|_| PartitionError::Io)?;
        raw[i * SECTOR_SIZE..(i + 1) * SECTOR_SIZE].copy_from_slice(&sector);
    }
    if crc32(&raw[..total_bytes]) != u32le(&hdr, 88) { return Err(PartitionError::Corrupt); }

    let mut out = Vec::new();
    for i in 0..entries {
        let at = i * entry_size;
        if raw[at..at + 16].iter().all(|&b| b == 0) { continue; }
        let first = u64le(&raw, at + 32);
        let last = u64le(&raw, at + 40);
        if last < first || first == 0 || last >= device.sector_count() { return Err(PartitionError::Corrupt); }
        out.push(Partition { index: i as u32 + 1, start_lba: first, sectors: last - first + 1, type_tag: u32le(&raw, at), table: TableKind::Gpt });
    }
    Ok(out)
}
