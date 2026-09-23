//! On-disk filesystem identification helpers.
//!
//! Identification is deliberately separate from mounting: a recognized
//! signature does not imply that EonOS can safely interpret that format.

use crate::block::BlockDevice;
use crate::fat32::SECTOR_SIZE;
use alloc::string::{String, ToString};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FsType { Fat32, Vfat, ExFat, Ntfs, Ext4, LinuxSwap, Unknown }

#[derive(Clone, Debug)]
pub struct FsInfo {
    pub kind: FsType,
    pub version: String,
    pub label: String,
    pub uuid: String,
    pub available: Option<u64>,
    pub used_percent: Option<u8>,
}

impl FsType {
    pub const fn name(self) -> &'static str {
        match self {
            Self::Fat32 => "fat32",
            Self::Vfat => "vfat",
            Self::ExFat => "exfat",
            Self::Ntfs => "ntfs",
            Self::Ext4 => "ext4",
            Self::LinuxSwap => "swap",
            Self::Unknown => "unknown",
        }
    }
}

/// Probe well-known boot-sector/superblock signatures. This does not
/// validate all metadata or determine whether the volume is clean.
pub fn detect(device: &dyn BlockDevice) -> FsType {
    let mut boot = [0u8; SECTOR_SIZE];
    if device.read_sector(0, &mut boot).is_err() { return FsType::Unknown; }
    if &boot[3..11] == b"EXFAT   " { return FsType::ExFat; }
    if &boot[3..11] == b"NTFS    " { return FsType::Ntfs; }
    if boot[510..512] == [0x55, 0xaa]
        && u16::from_le_bytes([boot[11], boot[12]]) == SECTOR_SIZE as u16
        && boot[13].is_power_of_two()
    {
        // This BPB check alone also matches FAT12/16. FAT32's root-entry
        // count and FAT16-size fields must both be zero.
        if boot[17] == 0 && boot[18] == 0 && boot[22] == 0 && boot[23] == 0 {
            return FsType::Fat32;
        }
        return FsType::Vfat;
    }

    // ext superblocks start 1024 bytes from the beginning; s_magic is
    // at offset 0x38 within the superblock.
    if device.sector_count() > 2 {
        let mut sb = [0u8; SECTOR_SIZE];
        if device.read_sector(2, &mut sb).is_ok() && sb[56..58] == [0x53, 0xef] {
            return FsType::Ext4;
        }
    }

    // Linux swap stores its signature at the end of the first page.
    // Accept both signatures used by contemporary and older Linux.
    if device.sector_count() >= 8 {
        let mut page = [0u8; 4096];
        let mut ok = true;
        for i in 0..8 {
            let mut sector = [0u8; SECTOR_SIZE];
            if device.read_sector(i, &mut sector).is_err() { ok = false; break; }
            page[i as usize * SECTOR_SIZE..(i as usize + 1) * SECTOR_SIZE].copy_from_slice(&sector);
        }
        if ok && (&page[4086..4096] == b"SWAPSPACE2" || &page[4086..4096] == b"SWAP-SPACE") {
            return FsType::LinuxSwap;
        }
    }
    FsType::Unknown
}

fn le32(b: &[u8], at: usize) -> u32 { u32::from_le_bytes(b[at..at+4].try_into().unwrap()) }
fn hex_uuid(bytes: &[u8]) -> String {
    let mut s = String::new();
    for (i, b) in bytes.iter().enumerate() {
        if i == 4 || i == 6 || i == 8 || i == 10 { s.push('-'); }
        s.push(char::from_digit((b >> 4) as u32, 16).unwrap());
        s.push(char::from_digit((b & 15) as u32, 16).unwrap());
    }
    s
}
fn text(bytes: &[u8]) -> String {
    let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
    String::from_utf8_lossy(&bytes[..end]).trim().into()
}

/// Read common filesystem label, version, and UUID fields where the
/// format stores them in its boot/superblock metadata.
pub fn inspect(device: &dyn BlockDevice) -> FsInfo {
    let kind = detect(device);
    let mut info = FsInfo { kind, version: String::new(), label: String::new(), uuid: String::new(), available: None, used_percent: None };
    let mut boot = [0u8; SECTOR_SIZE];
    if device.read_sector(0, &mut boot).is_err() { return info; }
    match kind {
        FsType::Fat32 => {
            info.version = "FAT32".into();
            info.label = text(&boot[71..82]);
            let serial = le32(&boot, 67);
            if serial != 0 { info.uuid = alloc::format!("{:04X}-{:04X}", serial >> 16, serial & 0xffff); }
            let spc = boot[13] as u64;
            let reserved = u16::from_le_bytes([boot[14], boot[15]]) as u64;
            let fats = boot[16] as u64;
            let spf = le32(&boot, 36) as u64;
            let total = le32(&boot, 32) as u64;
            let data_start = reserved.saturating_add(fats.saturating_mul(spf));
            let clusters = total.saturating_sub(data_start) / spc.max(1);
            let fsinfo = u16::from_le_bytes([boot[48], boot[49]]) as u64;
            let mut sector = [0u8; SECTOR_SIZE];
            if fsinfo < device.sector_count() && device.read_sector(fsinfo, &mut sector).is_ok()
                && le32(&sector, 0) == 0x4161_5252 && le32(&sector, 484) == 0x6141_7272 {
                let free = le32(&sector, 488) as u64;
                if free != u32::MAX as u64 {
                    let free = free.min(clusters);
                    info.available = Some(free.saturating_mul(spc).saturating_mul(SECTOR_SIZE as u64));
                    info.used_percent = if clusters == 0 { None } else { Some((((clusters-free)*100)/clusters) as u8) };
                }
            }
        }
        FsType::Vfat => {
            info.version = if boot[22] == 0 && boot[23] == 0 { "FAT32".into() } else { "FAT".into() };
            info.label = text(&boot[43..54]);
            let serial = le32(&boot, 39);
            if serial != 0 { info.uuid = alloc::format!("{:04X}-{:04X}", serial >> 16, serial & 0xffff); }
        }
        FsType::ExFat => {
            info.version = alloc::format!("{}.{}", boot[105], boot[107]);
            let serial = le32(&boot, 100);
            if serial != 0 { info.uuid = alloc::format!("{:08X}", serial); }
            if boot[108] == 9 && boot[109] < 16 {
                let sectors_per_cluster = 1u64 << boot[109];
                let fat_start = u64::from_le_bytes(boot[80..88].try_into().unwrap());
                let heap_start = u64::from_le_bytes(boot[88..96].try_into().unwrap());
                let mut cluster = le32(&boot, 96);
                for _ in 0..16 {
                    if cluster < 2 { break; }
                    let first_lba = heap_start.saturating_add((cluster as u64 - 2).saturating_mul(sectors_per_cluster));
                    let mut end_chain = false;
                    for s in 0..sectors_per_cluster.min(8) {
                        let mut dir = [0u8; SECTOR_SIZE];
                        if device.read_sector(first_lba.saturating_add(s), &mut dir).is_err() { end_chain = true; break; }
                        for entry in dir.chunks_exact(32) {
                            if entry[0] == 0 { end_chain = true; break; }
                            if entry[0] == 0x83 {
                                let count = (entry[1] as usize).min(15);
                                let mut units = alloc::vec::Vec::with_capacity(count);
                                for i in 0..count { units.push(u16::from_le_bytes([entry[2+i*2], entry[3+i*2]])); }
                                info.label = String::from_utf16_lossy(&units);
                                end_chain = true;
                                break;
                            }
                        }
                        if end_chain { break; }
                    }
                    if end_chain { break; }
                    let fat_sector = fat_start.saturating_add((cluster as u64 * 4) / SECTOR_SIZE as u64);
                    let fat_offset = ((cluster as u64 * 4) % SECTOR_SIZE as u64) as usize;
                    let mut fat = [0u8; SECTOR_SIZE];
                    if device.read_sector(fat_sector, &mut fat).is_err() { break; }
                    cluster = le32(&fat, fat_offset);
                    if cluster >= 0xffff_fff8 { break; }
                }
            }
            let percent = boot[112];
            if percent <= 100 {
                info.used_percent = Some(percent);
                info.available = Some(device.sector_count().saturating_mul(SECTOR_SIZE as u64).saturating_mul((100-percent) as u64)/100);
            }
        }
        FsType::Ntfs => {
            let serial = u64::from_le_bytes(boot[72..80].try_into().unwrap());
            if serial != 0 { info.uuid = alloc::format!("{:016X}", serial); }
        }
        FsType::Ext4 => {
            let mut sb = [0u8; SECTOR_SIZE];
            if device.read_sector(2, &mut sb).is_ok() {
                info.version = "1.0".into();
                info.label = text(&sb[120..136]);
                info.uuid = hex_uuid(&sb[104..120]);
                let block_log = le32(&sb, 24).min(6);
                let block_size = 1024u64 << block_log;
                let mut blocks = le32(&sb, 4) as u64;
                let mut free = le32(&sb, 12) as u64;
                if le32(&sb, 96) & 0x80 != 0 {
                    blocks |= (le32(&sb, 336) as u64) << 32;
                    free |= (le32(&sb, 344) as u64) << 32;
                }
                if blocks != 0 {
                    free = free.min(blocks);
                    info.available = Some(free.saturating_mul(block_size));
                    info.used_percent = Some((((blocks-free)*100)/blocks) as u8);
                }
            }
        }
        FsType::LinuxSwap => {
            let mut page = [0u8; 4096];
            let mut ok = true;
            for i in 0..8 {
                let mut sector = [0u8; SECTOR_SIZE];
                if device.read_sector(i, &mut sector).is_err() { ok = false; break; }
                page[i as usize * SECTOR_SIZE..(i as usize + 1) * SECTOR_SIZE].copy_from_slice(&sector);
            }
            if ok {
                info.version = le32(&page, 1024).to_string();
                info.uuid = hex_uuid(&page[1036..1052]);
                info.label = text(&page[1052..1068]);
            }
        }
        FsType::Unknown => {}
    }
    info
}
