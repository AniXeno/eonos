//! Read-only FAT32 filesystem over a 512-byte block device.

#![allow(dead_code)]

use alloc::string::{String, ToString};
use alloc::vec;
use alloc::vec::Vec;

use crate::block::{BlockDevice, MemoryBlockDevice};
use crate::{log_fail, log_ok};

pub const SECTOR_SIZE: usize = 512;
const MAX_CHAIN_STEPS: u32 = 1_000_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FatError {
    Io,
    InvalidBootSector,
    Unsupported,
    Corrupt,
    NotFound,
    NotDirectory,
    IsDirectory,
    TooLarge,
}

#[derive(Clone, Debug)]
pub struct DirEntry {
    pub name: String,
    pub is_dir: bool,
    pub size: u32,
}

pub struct Fat32 {
    device: &'static dyn BlockDevice,
    sectors_per_cluster: u8,
    reserved_sectors: u16,
    fat_count: u8,
    sectors_per_fat: u32,
    root_cluster: u32,
    data_start: u64,
    cluster_count: u32,
}

fn u16le(b: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([b[at], b[at + 1]])
}
fn u32le(b: &[u8], at: usize) -> u32 {
    u32::from_le_bytes([b[at], b[at + 1], b[at + 2], b[at + 3]])
}

impl Fat32 {
    pub fn mount(device: &'static dyn BlockDevice) -> Result<Self, FatError> {
        let mut b = [0u8; SECTOR_SIZE];
        device.read_sector(0, &mut b).map_err(|_| FatError::Io)?;
        if b[510] != 0x55 || b[511] != 0xAA {
            return Err(FatError::InvalidBootSector);
        }

        let bytes_per_sector = u16le(&b, 11);
        let sectors_per_cluster = b[13];
        let reserved_sectors = u16le(&b, 14);
        let fat_count = b[16];
        let root_entries = u16le(&b, 17);
        let total_sectors = if u16le(&b, 19) != 0 {
            u16le(&b, 19) as u32
        } else {
            u32le(&b, 32)
        };
        let fat16_size = u16le(&b, 22);
        let sectors_per_fat = u32le(&b, 36);
        let root_cluster = u32le(&b, 44) & 0x0fff_ffff;

        if bytes_per_sector != SECTOR_SIZE as u16
            || sectors_per_cluster == 0
            || !sectors_per_cluster.is_power_of_two()
            || reserved_sectors == 0
            || fat_count == 0
            || sectors_per_fat == 0
            || root_entries != 0
            || fat16_size != 0
            || total_sectors as u64 > device.sector_count()
        {
            return Err(FatError::Unsupported);
        }

        let fats_sectors = (fat_count as u64)
            .checked_mul(sectors_per_fat as u64)
            .ok_or(FatError::Corrupt)?;
        let data_start = (reserved_sectors as u64)
            .checked_add(fats_sectors)
            .ok_or(FatError::Corrupt)?;
        if data_start >= total_sectors as u64 {
            return Err(FatError::InvalidBootSector);
        }
        let cluster_count =
            ((total_sectors as u64 - data_start) / sectors_per_cluster as u64) as u32;
        let fat_entries = sectors_per_fat as u64 * SECTOR_SIZE as u64 / 4;
        // FAT32 reserves cluster IDs 0 and 1, and the root must be in the data range.
        if cluster_count < 1
            || fat_entries < cluster_count as u64 + 2
            || root_cluster < 2
            || root_cluster >= cluster_count + 2
        {
            return Err(FatError::InvalidBootSector);
        }
        Ok(Self {
            device,
            sectors_per_cluster,
            reserved_sectors,
            fat_count,
            sectors_per_fat,
            root_cluster,
            data_start,
            cluster_count,
        })
    }

    fn read_sector(&self, lba: u64, out: &mut [u8; SECTOR_SIZE]) -> Result<(), FatError> {
        self.device.read_sector(lba, out).map_err(|_| FatError::Io)
    }

    fn next_cluster(&self, cluster: u32) -> Result<Option<u32>, FatError> {
        if cluster < 2 || cluster >= self.cluster_count + 2 {
            return Err(FatError::Corrupt);
        }
        let byte = cluster as u64 * 4;
        let fat_sector = self.reserved_sectors as u64 + byte / SECTOR_SIZE as u64;
        if byte / SECTOR_SIZE as u64 >= self.sectors_per_fat as u64 {
            return Err(FatError::Corrupt);
        }
        let mut b = [0u8; SECTOR_SIZE];
        self.read_sector(fat_sector, &mut b)?;
        let at = (byte % SECTOR_SIZE as u64) as usize;
        let next = u32le(&b, at) & 0x0fff_ffff;
        if next >= 0x0fff_fff8 {
            Ok(None)
        } else if next == 0 || next == 1 || (0x0fff_fff0..=0x0fff_fff7).contains(&next) {
            Err(FatError::Corrupt)
        } else if next < 2 || next >= self.cluster_count + 2 {
            Err(FatError::Corrupt)
        } else {
            Ok(Some(next))
        }
    }

    fn cluster_lba(&self, cluster: u32) -> Result<u64, FatError> {
        if cluster < 2 || cluster >= self.cluster_count + 2 {
            return Err(FatError::Corrupt);
        }
        self.data_start
            .checked_add((cluster as u64 - 2) * self.sectors_per_cluster as u64)
            .ok_or(FatError::Corrupt)
    }

    fn cluster_bytes(&self) -> usize {
        self.sectors_per_cluster as usize * SECTOR_SIZE
    }

    fn read_cluster(&self, cluster: u32) -> Result<Vec<u8>, FatError> {
        let first = self.cluster_lba(cluster)?;
        let mut data = vec![0u8; self.cluster_bytes()];
        for s in 0..self.sectors_per_cluster as usize {
            let mut sector = [0u8; SECTOR_SIZE];
            self.read_sector(first + s as u64, &mut sector)?;
            data[s * SECTOR_SIZE..(s + 1) * SECTOR_SIZE].copy_from_slice(&sector);
        }
        Ok(data)
    }

    fn directory_entries(&self, cluster: u32) -> Result<Vec<(DirEntry, u32)>, FatError> {
        let mut out = Vec::new();
        let mut current = Some(cluster);
        let mut steps = 0;
        while let Some(c) = current {
            if steps >= MAX_CHAIN_STEPS {
                return Err(FatError::Corrupt);
            }
            steps += 1;
            let data = self.read_cluster(c)?;
            let mut lfn: [u16; 260] = [0xffff; 260];
            let mut have_lfn = false;
            for e in data.chunks_exact(32) {
                if e[0] == 0 {
                    return Ok(out);
                }
                if e[0] == 0xe5 {
                    have_lfn = false;
                    continue;
                }
                let attr = e[11];
                if attr == 0x0f {
                    let ord = (e[0] & 0x1f) as usize;
                    if ord == 0 || ord > 20 {
                        have_lfn = false;
                        continue;
                    }
                    if e[0] & 0x40 != 0 {
                        lfn = [0xffff; 260];
                        have_lfn = true;
                    }
                    if !have_lfn {
                        continue;
                    }
                    let positions = [1usize, 3, 5, 7, 9, 14, 16, 18, 20, 22, 24, 28, 30];
                    let base = (ord - 1) * 13;
                    for (i, p) in positions.iter().enumerate() {
                        lfn[base + i] = u16le(e, *p);
                    }
                    continue;
                }
                if attr & 0x08 != 0 {
                    have_lfn = false;
                    continue;
                } // volume label
                let hi = u16le(e, 20) as u32;
                let lo = u16le(e, 26) as u32;
                let first_cluster = (hi << 16) | lo;
                let name = if have_lfn {
                    let n = lfn
                        .iter()
                        .position(|&x| x == 0 || x == 0xffff)
                        .unwrap_or(lfn.len());
                    let mut s = String::new();
                    for &u in &lfn[..n] {
                        if let Some(ch) = char::from_u32(u as u32) {
                            s.push(ch);
                        }
                    }
                    if s.is_empty() {
                        short_name(&e[0..11])
                    } else {
                        s
                    }
                } else {
                    short_name(&e[0..11])
                };
                have_lfn = false;
                if name == "." || name == ".." {
                    continue;
                }
                out.push((
                    DirEntry {
                        name,
                        is_dir: attr & 0x10 != 0,
                        size: u32le(e, 28),
                    },
                    first_cluster,
                ));
            }
            current = self.next_cluster(c)?;
        }
        Ok(out)
    }

    fn resolve(&self, path: &str) -> Result<(DirEntry, u32), FatError> {
        let mut cluster = self.root_cluster;
        let mut last: Option<(DirEntry, u32)> = None;
        let clean = path.trim_matches('/');
        if clean.is_empty() {
            return Ok((
                DirEntry {
                    name: "/".to_string(),
                    is_dir: true,
                    size: 0,
                },
                cluster,
            ));
        }
        for component in clean.split('/') {
            if component.is_empty() || component == "." {
                continue;
            }
            if component == ".." {
                return Err(FatError::NotFound);
            }
            if last.as_ref().map(|(e, _)| !e.is_dir).unwrap_or(false) {
                return Err(FatError::NotDirectory);
            }
            let entries = self.directory_entries(cluster)?;
            let found = entries
                .into_iter()
                .find(|(e, _)| e.name.eq_ignore_ascii_case(component))
                .ok_or(FatError::NotFound)?;
            cluster = found.1;
            last = Some(found);
        }
        Ok(last.unwrap_or((
            DirEntry {
                name: "/".to_string(),
                is_dir: true,
                size: 0,
            },
            cluster,
        )))
    }

    pub fn stat(&self, path: &str) -> Result<DirEntry, FatError> {
        self.resolve(path).map(|x| x.0)
    }

    pub fn list(&self, path: &str) -> Result<Vec<DirEntry>, FatError> {
        let (entry, cluster) = self.resolve(path)?;
        if !entry.is_dir {
            return Err(FatError::NotDirectory);
        }
        Ok(self
            .directory_entries(cluster)?
            .into_iter()
            .map(|x| x.0)
            .collect())
    }

    pub fn read_file(&self, path: &str) -> Result<Vec<u8>, FatError> {
        let entry = self.stat(path)?;
        if entry.is_dir {
            return Err(FatError::IsDirectory);
        }
        if entry.size > 64 * 1024 * 1024 {
            return Err(FatError::TooLarge);
        }
        let mut result = vec![0; entry.size as usize];
        let read = self.read_at(path, 0, &mut result)?;
        if read != result.len() {
            return Err(FatError::Corrupt);
        }
        Ok(result)
    }

    pub fn read_at(&self, path: &str, offset: usize, out: &mut [u8]) -> Result<usize, FatError> {
        let (entry, mut cluster) = self.resolve(path)?;
        if entry.is_dir {
            return Err(FatError::IsDirectory);
        }
        let size = entry.size as usize;
        if offset >= size || out.is_empty() {
            return Ok(0);
        }
        let wanted = out.len().min(size - offset);
        let cluster_bytes = self.cluster_bytes();
        let mut skip = offset / cluster_bytes;
        let mut steps = 0usize;
        while skip > 0 {
            cluster = self.next_cluster(cluster)?.ok_or(FatError::Corrupt)?;
            skip -= 1;
            steps += 1;
            if steps > MAX_CHAIN_STEPS as usize {
                return Err(FatError::Corrupt);
            }
        }
        let mut in_cluster = offset % cluster_bytes;
        let mut copied = 0;
        while copied < wanted {
            if steps >= MAX_CHAIN_STEPS as usize {
                return Err(FatError::Corrupt);
            }
            let data = self.read_cluster(cluster)?;
            let n = (wanted - copied).min(cluster_bytes - in_cluster);
            out[copied..copied + n].copy_from_slice(&data[in_cluster..in_cluster + n]);
            copied += n;
            in_cluster = 0;
            if copied < wanted {
                cluster = self.next_cluster(cluster)?.ok_or(FatError::Corrupt)?;
            }
            steps += 1;
        }
        Ok(copied)
    }
}

/// Mount the FAT32 boot image at `/disk`, if Limine loaded it.
pub fn mount_boot_volume() {
    let Some(bytes) = crate::initramfs::module_bytes("fat32.img") else {
        log_fail!(
            "FAT32",
            "Mount",
            "fat32.img boot module is missing or unmapped"
        );
        return;
    };
    let Some(device) = MemoryBlockDevice::new(bytes) else {
        log_fail!(
            "FAT32",
            "Mount",
            "fat32.img size is not a multiple of 512 bytes"
        );
        return;
    };
    let device: &'static dyn BlockDevice = alloc::boxed::Box::leak(alloc::boxed::Box::new(device));
    match crate::vfs::mount_fat32("/disk", device) {
        Ok(()) => log_ok!(
            "FAT32",
            "Mount",
            "Read-only FAT32 volume mounted at /disk ({} sectors)",
            device.sector_count()
        ),
        Err(e) => log_fail!("FAT32", "Mount", "Could not mount boot volume: {:?}", e),
    }
}

fn short_name(raw: &[u8]) -> String {
    let base = raw[..8]
        .iter()
        .copied()
        .take_while(|&b| b != b' ')
        .map(|b| b as char)
        .collect::<String>();
    let ext = raw[8..11]
        .iter()
        .copied()
        .take_while(|&b| b != b' ')
        .map(|b| b as char)
        .collect::<String>();
    if ext.is_empty() {
        base
    } else {
        alloc::format!("{}.{}", base, ext)
    }
}
