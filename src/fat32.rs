//! FAT32 filesystem over a 512-byte block device.

#![allow(dead_code)]

use alloc::string::{String, ToString};
use alloc::vec;
use alloc::vec::Vec;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

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
    NoSpace,
    InvalidName,
    AlreadyExists,
    DirectoryNotEmpty,
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
    next_free_hint: AtomicU32,
    fsinfo_sector: Option<u16>,
    fsinfo_invalidated: AtomicBool,
}

#[derive(Clone, Copy)]
struct EntryLocation {
    cluster: u32,
    offset: usize,
    first_cluster: u32,
    size: u32,
    is_dir: bool,
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
        let fsinfo_sector = u16le(&b, 48);

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
            || cluster_count > 0x0fff_fff5
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
            next_free_hint: AtomicU32::new(2),
            fsinfo_sector: if fsinfo_sector != 0 && fsinfo_sector < reserved_sectors {
                Some(fsinfo_sector)
            } else {
                None
            },
            fsinfo_invalidated: AtomicBool::new(false),
        })
    }

    fn read_sector(&self, lba: u64, out: &mut [u8; SECTOR_SIZE]) -> Result<(), FatError> {
        self.device.read_sector(lba, out).map_err(|_| FatError::Io)
    }

    fn write_sector(&self, lba: u64, data: &[u8; SECTOR_SIZE]) -> Result<(), FatError> {
        self.device.write_sector(lba, data).map_err(|_| FatError::Io)
    }

    pub fn flush(&self) -> Result<(), crate::block::BlockError> { self.device.flush() }

    fn invalidate_fsinfo_hint(&self) -> Result<(), FatError> {
        if self.fsinfo_invalidated.load(Ordering::Acquire) {
            return Ok(());
        }
        let Some(sector_no) = self.fsinfo_sector else {
            self.fsinfo_invalidated.store(true, Ordering::Release);
            return Ok(());
        };
        let mut sector = [0u8; SECTOR_SIZE];
        self.read_sector(sector_no as u64, &mut sector)?;
        if u32le(&sector, 0) == 0x4161_5252
            && u32le(&sector, 484) == 0x6141_7272
            && u32le(&sector, 508) == 0xaa55_0000
        {
            sector[488..492].copy_from_slice(&u32::MAX.to_le_bytes());
            sector[492..496].copy_from_slice(&u32::MAX.to_le_bytes());
            self.write_sector(sector_no as u64, &sector)?;
        }
        self.fsinfo_invalidated.store(true, Ordering::Release);
        Ok(())
    }

    fn fat_entry(&self, cluster: u32) -> Result<u32, FatError> {
        if cluster >= self.cluster_count + 2 {
            return Err(FatError::Corrupt);
        }
        let byte = cluster as u64 * 4;
        let sector_in_fat = byte / SECTOR_SIZE as u64;
        if sector_in_fat >= self.sectors_per_fat as u64 {
            return Err(FatError::Corrupt);
        }
        let lba = self.reserved_sectors as u64 + sector_in_fat;
        let mut sector = [0u8; SECTOR_SIZE];
        self.read_sector(lba, &mut sector)?;
        Ok(u32le(&sector, (byte % SECTOR_SIZE as u64) as usize) & 0x0fff_ffff)
    }

    fn set_fat_entry(&self, cluster: u32, value: u32) -> Result<(), FatError> {
        if cluster >= self.cluster_count + 2 {
            return Err(FatError::Corrupt);
        }
        let byte = cluster as u64 * 4;
        let sector_in_fat = byte / SECTOR_SIZE as u64;
        if sector_in_fat >= self.sectors_per_fat as u64 {
            return Err(FatError::Corrupt);
        }
        self.invalidate_fsinfo_hint()?;
        let at = (byte % SECTOR_SIZE as u64) as usize;
        for fat in 0..self.fat_count as u64 {
            let lba = self.reserved_sectors as u64
                + fat * self.sectors_per_fat as u64
                + sector_in_fat;
            let mut sector = [0u8; SECTOR_SIZE];
            self.read_sector(lba, &mut sector)?;
            let old = u32le(&sector, at);
            sector[at..at + 4].copy_from_slice(&((old & 0xf000_0000) | (value & 0x0fff_ffff)).to_le_bytes());
            self.write_sector(lba, &sector)?;
        }
        Ok(())
    }

    fn allocate_cluster(&self) -> Result<u32, FatError> {
        let count = self.cluster_count;
        let start = self.next_free_hint.load(Ordering::Relaxed).clamp(2, count + 1);
        for step in 0..count {
            let cluster = 2 + ((start - 2 + step) % count);
            if self.fat_entry(cluster)? == 0 {
                self.set_fat_entry(cluster, 0x0fff_ffff)?;
                if let Err(error) = self.zero_cluster(cluster) {
                    let _ = self.set_fat_entry(cluster, 0);
                    return Err(error);
                }
                self.next_free_hint.store(if cluster == count + 1 { 2 } else { cluster + 1 }, Ordering::Relaxed);
                return Ok(cluster);
            }
        }
        Err(FatError::NoSpace)
    }

    fn zero_cluster(&self, cluster: u32) -> Result<(), FatError> {
        let first = self.cluster_lba(cluster)?;
        let zero = [0u8; SECTOR_SIZE];
        for sector in 0..self.sectors_per_cluster as u64 {
            self.write_sector(first + sector, &zero)?;
        }
        Ok(())
    }

    fn next_cluster(&self, cluster: u32) -> Result<Option<u32>, FatError> {
        if cluster < 2 || cluster >= self.cluster_count + 2 {
            return Err(FatError::Corrupt);
        }
        let next = self.fat_entry(cluster)?;
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
        let mut lfn: [u16; 260] = [0xffff; 260];
        let mut have_lfn = false;
        while let Some(c) = current {
            if steps >= MAX_CHAIN_STEPS {
                return Err(FatError::Corrupt);
            }
            steps += 1;
            let data = self.read_cluster(c)?;
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
                    let s = String::from_utf16_lossy(&lfn[..n]);
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

    fn parent_and_name(&self, path: &str) -> Result<(u32, String), FatError> {
        let clean = path.trim_matches('/');
        let (parent_path, name) = clean.rsplit_once('/').unwrap_or(("", clean));
        if name.is_empty() || name == "." || name == ".." {
            return Err(FatError::InvalidName);
        }
        let parent = if parent_path.is_empty() {
            self.root_cluster
        } else {
            let (entry, cluster) = self.resolve(parent_path)?;
            if !entry.is_dir { return Err(FatError::NotDirectory); }
            cluster
        };
        if name.len() > 255 || name.ends_with('.') || name.ends_with(' ') || name.contains(['\\', ':', '*', '?', '"', '<', '>', '|']) {
            return Err(FatError::InvalidName);
        }
        Ok((parent, String::from(name)))
    }

    fn find_short_entry(&self, directory: u32, short: &[u8; 11]) -> Result<Option<EntryLocation>, FatError> {
        let mut current = Some(directory);
        let mut steps = 0;
        while let Some(cluster) = current {
            if steps >= self.cluster_count { return Err(FatError::Corrupt); }
            steps += 1;
            let data = self.read_cluster(cluster)?;
            for (index, entry) in data.chunks_exact(32).enumerate() {
                if entry[0] == 0 { return Ok(None); }
                if entry[0] == 0xe5 || entry[11] == 0x0f || entry[11] & 0x08 != 0 { continue; }
                if &entry[..11] == short {
                    return Ok(Some(EntryLocation {
                        cluster,
                        offset: index * 32,
                        first_cluster: ((u16le(entry, 20) as u32) << 16) | u16le(entry, 26) as u32,
                        size: u32le(entry, 28),
                        is_dir: entry[11] & 0x10 != 0,
                    }));
                }
            }
            current = self.next_cluster(cluster)?;
        }
        Ok(None)
    }

    fn find_named_entry(&self, directory: u32, name: &str) -> Result<Option<EntryLocation>, FatError> {
        let mut current = Some(directory);
        let mut steps = 0;
        let mut lfn = [0xffffu16; 260];
        let mut have_lfn = false;
        while let Some(cluster) = current {
            if steps >= self.cluster_count { return Err(FatError::Corrupt); }
            steps += 1;
            let data = self.read_cluster(cluster)?;
            for (index, entry) in data.chunks_exact(32).enumerate() {
                if entry[0] == 0 { return Ok(None); }
                if entry[0] == 0xe5 { have_lfn = false; continue; }
                if entry[11] == 0x0f {
                    let order = (entry[0] & 0x1f) as usize;
                    if order == 0 || order > 20 { have_lfn = false; continue; }
                    if entry[0] & 0x40 != 0 { lfn = [0xffff; 260]; have_lfn = true; }
                    if !have_lfn { continue; }
                    let positions = [1usize, 3, 5, 7, 9, 14, 16, 18, 20, 22, 24, 28, 30];
                    for (i, pos) in positions.iter().enumerate() {
                        lfn[(order - 1) * 13 + i] = u16le(entry, *pos);
                    }
                    continue;
                }
                if entry[11] & 0x08 != 0 { have_lfn = false; continue; }
                let entry_name = if have_lfn {
                    let n = lfn.iter().position(|&u| u == 0 || u == 0xffff).unwrap_or(lfn.len());
                    let text = String::from_utf16_lossy(&lfn[..n]);
                    if text.is_empty() { short_name(&entry[..11]) } else { text }
                } else {
                    short_name(&entry[..11])
                };
                have_lfn = false;
                if entry_name.eq_ignore_ascii_case(name) {
                    let first = ((u16le(entry, 20) as u32) << 16) | u16le(entry, 26) as u32;
                    return Ok(Some(EntryLocation {
                        cluster,
                        offset: index * 32,
                        first_cluster: first,
                        size: u32le(entry, 28),
                        is_dir: entry[11] & 0x10 != 0,
                    }));
                }
            }
            current = self.next_cluster(cluster)?;
        }
        Ok(None)
    }

    fn choose_short_alias(&self, directory: u32, name: &str) -> Result<[u8; 11], FatError> {
        if let Ok(short) = to_short_name(name) {
            if self.find_short_entry(directory, &short)?.is_none() { return Ok(short); }
        }
        let (stem, ext) = name.rsplit_once('.').unwrap_or((name, ""));
        let sanitize = |part: &str| -> String {
            part.chars()
                .filter(|&c| c != '.')
                .map(|c| {
                    if !c.is_ascii() { return '_'; }
                    let upper = c.to_ascii_uppercase();
                    if upper.is_ascii_alphanumeric() || b"$%'-_@~`!(){}^#&".contains(&(upper as u8)) {
                        upper
                    } else { '_' }
                })
                .collect()
        };
        let stem = sanitize(stem);
        let ext = sanitize(ext);
        let ext = &ext[..ext.len().min(3)];
        let stem = if stem.is_empty() { "FILE" } else { stem.as_str() };
        for serial in 1..100_000u32 {
            let suffix = alloc::format!("~{}", serial);
            let prefix_len = 8usize.saturating_sub(suffix.len());
            let mut raw = [b' '; 11];
            let prefix = &stem[..stem.len().min(prefix_len)];
            raw[..prefix.len()].copy_from_slice(prefix.as_bytes());
            raw[prefix.len()..prefix.len() + suffix.len()].copy_from_slice(suffix.as_bytes());
            raw[8..8 + ext.len()].copy_from_slice(ext.as_bytes());
            if self.find_short_entry(directory, &raw)?.is_none() { return Ok(raw); }
        }
        Err(FatError::NoSpace)
    }

    fn append_directory_records(&self, directory: u32, records: &[[u8; 32]]) -> Result<(), FatError> {
        let chain = self.collect_chain(directory)?;
        let tail = *chain.last().ok_or(FatError::Corrupt)?;

        // The old end marker makes every later slot in the chain unused.
        // Convert that ignored tail to deleted entries before extending,
        // so old garbage cannot become visible when the chain grows.
        let mut after_end = false;
        for &cluster in &chain {
            let data = self.read_cluster(cluster)?;
            for (index, entry) in data.chunks_exact(32).enumerate() {
                if !after_end && entry[0] == 0 { after_end = true; }
                if after_end {
                    self.write_directory_bytes(cluster, index * 32, &[0xe5])?;
                }
            }
        }

        let slots_needed = records.len() + 1; // leave a zero end marker
        let clusters_needed = (slots_needed * 32).div_ceil(self.cluster_bytes());
        let mut added = Vec::with_capacity(clusters_needed);
        for _ in 0..clusters_needed {
            match self.allocate_cluster() {
                Ok(cluster) => added.push(cluster),
                Err(error) => {
                    let _ = self.release_chain(&added);
                    return Err(error);
                }
            }
        }
        for pair in added.windows(2) {
            if let Err(error) = self.set_fat_entry(pair[0], pair[1]) {
                let _ = self.release_chain(&added);
                return Err(error);
            }
        }
        if let Err(error) = self.set_fat_entry(tail, added[0]) {
            let _ = self.release_chain(&added);
            return Err(error);
        }
        for (i, record) in records.iter().enumerate() {
            let byte_offset = i * 32;
            let cluster = added[byte_offset / self.cluster_bytes()];
            let offset = byte_offset % self.cluster_bytes();
            self.write_directory_bytes(cluster, offset, record)?;
        }
        Ok(())
    }

    fn write_directory_bytes(&self, cluster: u32, offset: usize, bytes: &[u8]) -> Result<(), FatError> {
        let lba = self.cluster_lba(cluster)? + (offset / SECTOR_SIZE) as u64;
        let in_sector = offset % SECTOR_SIZE;
        if in_sector + bytes.len() > SECTOR_SIZE { return Err(FatError::Corrupt); }
        let mut sector = [0u8; SECTOR_SIZE];
        self.read_sector(lba, &mut sector)?;
        sector[in_sector..in_sector + bytes.len()].copy_from_slice(bytes);
        self.write_sector(lba, &sector)
    }

    fn read_entry(&self, location: EntryLocation) -> Result<[u8; 32], FatError> {
        let lba = self.cluster_lba(location.cluster)? + (location.offset / SECTOR_SIZE) as u64;
        let mut sector = [0u8; SECTOR_SIZE];
        self.read_sector(lba, &mut sector)?;
        let at = location.offset % SECTOR_SIZE;
        let mut entry = [0u8; 32];
        entry.copy_from_slice(&sector[at..at + 32]);
        Ok(entry)
    }

    fn write_entry(&self, location: EntryLocation, entry: &[u8; 32]) -> Result<(), FatError> {
        self.write_directory_bytes(location.cluster, location.offset, entry)
    }

    fn write_entry_metadata(&self, location: EntryLocation, first: u32, size: u32) -> Result<(), FatError> {
        let mut entry = self.read_entry(location)?;
        entry[20..22].copy_from_slice(&((first >> 16) as u16).to_le_bytes());
        entry[26..28].copy_from_slice(&(first as u16).to_le_bytes());
        entry[28..32].copy_from_slice(&size.to_le_bytes());
        self.write_entry(location, &entry)
    }

    fn collect_chain(&self, first: u32) -> Result<Vec<u32>, FatError> {
        let mut chain = Vec::new();
        let mut current = if first == 0 { None } else { Some(first) };
        while let Some(cluster) = current {
            if chain.len() >= self.cluster_count as usize { return Err(FatError::Corrupt); }
            chain.push(cluster);
            current = self.next_cluster(cluster)?;
        }
        Ok(chain)
    }

    fn release_chain(&self, chain: &[u32]) -> Result<(), FatError> {
        for &cluster in chain {
            self.set_fat_entry(cluster, 0)?;
            self.next_free_hint.fetch_min(cluster, Ordering::Relaxed);
        }
        Ok(())
    }

    fn rollback_chain_growth(&self, chain: &mut Vec<u32>, original_len: usize) {
        if original_len > 0 {
            let _ = self.set_fat_entry(chain[original_len - 1], 0x0fff_ffff);
        }
        let added = chain.split_off(original_len);
        let _ = self.release_chain(&added);
    }

    fn ensure_chain(&self, chain: &mut Vec<u32>, needed: usize) -> Result<Vec<u32>, FatError> {
        let original_len = chain.len();
        while chain.len() < needed {
            let new_cluster = match self.allocate_cluster() {
                Ok(c) => c,
                Err(error) => {
                    self.rollback_chain_growth(chain, original_len);
                    return Err(error);
                }
            };
            if let Some(&previous) = chain.last() {
                if let Err(error) = self.set_fat_entry(previous, new_cluster) {
                    let _ = self.set_fat_entry(new_cluster, 0);
                    self.rollback_chain_growth(chain, original_len);
                    return Err(error);
                }
            }
            chain.push(new_cluster);
        }
        Ok(chain[original_len..].to_vec())
    }

    fn write_chain_range(&self, chain: &[u32], mut offset: usize, mut data: &[u8]) -> Result<(), FatError> {
        let cluster_bytes = self.cluster_bytes();
        while !data.is_empty() {
            let ci = offset / cluster_bytes;
            if ci >= chain.len() { return Err(FatError::Corrupt); }
            let in_cluster = offset % cluster_bytes;
            let sector_index = in_cluster / SECTOR_SIZE;
            let in_sector = in_cluster % SECTOR_SIZE;
            let n = data.len().min(SECTOR_SIZE - in_sector);
            let lba = self.cluster_lba(chain[ci])? + sector_index as u64;
            let mut sector = [0u8; SECTOR_SIZE];
            self.read_sector(lba, &mut sector)?;
            sector[in_sector..in_sector + n].copy_from_slice(&data[..n]);
            self.write_sector(lba, &sector)?;
            offset += n;
            data = &data[n..];
        }
        Ok(())
    }

    fn write_zero_range(&self, chain: &[u32], mut offset: usize, mut len: usize) -> Result<(), FatError> {
        let zeros = [0u8; SECTOR_SIZE];
        while len != 0 {
            let in_sector = offset % SECTOR_SIZE;
            let n = len.min(SECTOR_SIZE - in_sector);
            self.write_chain_range(chain, offset, &zeros[..n])?;
            offset += n;
            len -= n;
        }
        Ok(())
    }

    /// Create an empty 8.3 file in an existing directory. Existing files
    /// are left intact; directories and unsupported names are rejected.
    pub fn create_file(&self, path: &str) -> Result<(), FatError> {
        let (parent, name) = self.parent_and_name(path)?;
        if let Some(existing) = self.find_named_entry(parent, &name)? {
            return if existing.is_dir { Err(FatError::IsDirectory) } else { Ok(()) };
        }
        let short = self.choose_short_alias(parent, &name)?;
        let mut records = lfn_records(&name, &short)?;
        let mut entry = [0u8; 32];
        entry[..11].copy_from_slice(&short);
        entry[11] = 0x20;
        records.push(entry);
        self.append_directory_records(parent, &records)
    }

    /// Create a directory with the FAT `.` and `..` entries.
    pub fn create_dir(&self, path: &str) -> Result<(), FatError> {
        let (parent, name) = self.parent_and_name(path)?;
        if self.find_named_entry(parent, &name)?.is_some() {
            return Err(FatError::AlreadyExists);
        }
        let cluster = self.allocate_cluster()?;
        let result = (|| {
            let mut dot = [0u8; 32];
            dot[..11].copy_from_slice(b".          ");
            dot[11] = 0x10;
            dot[20..22].copy_from_slice(&((cluster >> 16) as u16).to_le_bytes());
            dot[26..28].copy_from_slice(&(cluster as u16).to_le_bytes());
            let mut dotdot = [0u8; 32];
            dotdot[..11].copy_from_slice(b"..         ");
            dotdot[11] = 0x10;
            dotdot[20..22].copy_from_slice(&((parent >> 16) as u16).to_le_bytes());
            dotdot[26..28].copy_from_slice(&(parent as u16).to_le_bytes());
            self.write_directory_bytes(cluster, 0, &dot)?;
            self.write_directory_bytes(cluster, 32, &dotdot)?;
            let short = self.choose_short_alias(parent, &name)?;
            let mut records = lfn_records(&name, &short)?;
            let mut entry = [0u8; 32];
            entry[..11].copy_from_slice(&short);
            entry[11] = 0x10;
            entry[20..22].copy_from_slice(&((cluster >> 16) as u16).to_le_bytes());
            entry[26..28].copy_from_slice(&(cluster as u16).to_le_bytes());
            records.push(entry);
            self.append_directory_records(parent, &records)
        })();
        if result.is_err() { let _ = self.release_chain(&[cluster]); }
        result
    }

    /// Remove a file or an empty directory. Non-empty directories are kept.
    pub fn remove(&self, path: &str) -> Result<(), FatError> {
        let (parent, name) = self.parent_and_name(path)?;
        let location = self.find_named_entry(parent, &name)?.ok_or(FatError::NotFound)?;
        if location.is_dir && !self.directory_entries(location.first_cluster)?.is_empty() {
            return Err(FatError::DirectoryNotEmpty);
        }
        let chain = self.collect_chain(location.first_cluster)?;
        let dir_chain = self.collect_chain(parent)?;
        let mut pending = Vec::new();
        let mut found = Vec::new();
        for cluster in dir_chain {
            let data = self.read_cluster(cluster)?;
            for (index, entry) in data.chunks_exact(32).enumerate() {
                if entry[0] == 0 { break; }
                if entry[0] == 0xe5 { pending.clear(); continue; }
                let loc = EntryLocation { cluster, offset: index * 32, first_cluster: 0, size: 0, is_dir: false };
                if entry[11] == 0x0f { pending.push(loc); continue; }
                if cluster == location.cluster && index * 32 == location.offset {
                    found.append(&mut pending);
                    found.push(loc);
                    break;
                }
                pending.clear();
            }
            if !found.is_empty() { break; }
        }
        if found.is_empty() { return Err(FatError::Corrupt); }
        for loc in found {
            let mut entry = self.read_entry(loc)?;
            entry[0] = 0xe5;
            self.write_entry(loc, &entry)?;
        }
        self.release_chain(&chain)
    }

    /// Truncate an existing regular file and release its cluster chain.
    pub fn truncate(&self, path: &str) -> Result<(), FatError> {
        let (parent, name) = self.parent_and_name(path)?;
        let location = self.find_named_entry(parent, &name)?.ok_or(FatError::NotFound)?;
        if location.is_dir { return Err(FatError::IsDirectory); }
        let chain = self.collect_chain(location.first_cluster)?;
        self.write_entry_metadata(location, 0, 0)?;
        self.release_chain(&chain)
    }

    /// Write bytes at an offset, extending the file and zero-filling any
    /// gap. Files must already exist and use a FAT 8.3 name.
    pub fn write_at(&self, path: &str, offset: usize, data: &[u8]) -> Result<usize, FatError> {
        if data.is_empty() { return Ok(0); }
        let end = offset.checked_add(data.len()).ok_or(FatError::TooLarge)?;
        if end > u32::MAX as usize { return Err(FatError::TooLarge); }
        let (parent, name) = self.parent_and_name(path)?;
        let location = self.find_named_entry(parent, &name)?.ok_or(FatError::NotFound)?;
        if location.is_dir { return Err(FatError::IsDirectory); }

        let mut chain = self.collect_chain(location.first_cluster)?;
        let needed_bytes = (location.size as usize).max(end);
        let needed_clusters = needed_bytes.div_ceil(self.cluster_bytes());
        let original_len = chain.len();
        let added = self.ensure_chain(&mut chain, needed_clusters)?;
        let first = chain.first().copied().unwrap_or(0);
        let result = (|| {
            if offset > location.size as usize {
                self.write_zero_range(&chain, location.size as usize, offset - location.size as usize)?;
            }
            self.write_chain_range(&chain, offset, data)?;
            self.write_entry_metadata(location, first, needed_bytes as u32)
        })();
        if let Err(error) = result {
            if !added.is_empty() { self.rollback_chain_growth(&mut chain, original_len); }
            return Err(error);
        }
        Ok(data.len())
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
    crate::block::register_device("ram0", device);
    match crate::vfs::mount_fat32("/disk", device) {
        Ok(()) => log_ok!(
            "FAT32",
            "Mount",
            "Writable RAM-overlay FAT32 volume mounted at /disk ({} sectors)",
            device.sector_count()
        ),
        Err(e) => log_fail!("FAT32", "Mount", "Could not mount boot volume: {:?}", e),
    }
}

fn to_short_name(name: &str) -> Result<[u8; 11], FatError> {
    let mut parts = name.split('.');
    let base = parts.next().unwrap_or("");
    let ext = parts.next().unwrap_or("");
    if parts.next().is_some() || base.is_empty() || base.len() > 8 || ext.len() > 3 {
        return Err(FatError::InvalidName);
    }
    let valid = |part: &str| {
        part.bytes().all(|b| b.is_ascii_alphanumeric() || b"$%'-_@~`!(){}^#&".contains(&b))
    };
    if !valid(base) || !valid(ext) { return Err(FatError::InvalidName); }
    let mut raw = [b' '; 11];
    for (i, b) in base.bytes().enumerate() { raw[i] = b.to_ascii_uppercase(); }
    for (i, b) in ext.bytes().enumerate() { raw[8 + i] = b.to_ascii_uppercase(); }
    Ok(raw)
}

fn lfn_records(name: &str, short: &[u8; 11]) -> Result<Vec<[u8; 32]>, FatError> {
    let units: Vec<u16> = name.encode_utf16().collect();
    if units.is_empty() || units.len() > 255 {
        return Err(FatError::InvalidName);
    }
    if units.iter().any(|&u| u < 0x20 || u == b'"' as u16 || u == b'*' as u16 || u == b'/' as u16 || u == b':' as u16 || u == b'<' as u16 || u == b'>' as u16 || u == b'?' as u16 || u == b'\\' as u16 || u == b'|' as u16) {
        return Err(FatError::InvalidName);
    }
    let count = (units.len() + 1).div_ceil(13);
    let mut checksum = 0u8;
    for byte in short {
        checksum = ((checksum & 1) << 7).wrapping_add(checksum >> 1).wrapping_add(*byte);
    }
    let positions = [1usize, 3, 5, 7, 9, 14, 16, 18, 20, 22, 24, 28, 30];
    let mut records = Vec::with_capacity(count);
    for order in (1..=count).rev() {
        let mut entry = [0u8; 32];
        entry[0] = order as u8 | if order == count { 0x40 } else { 0 };
        entry[11] = 0x0f;
        entry[12] = 0;
        entry[13] = checksum;
        entry[26] = 0;
        entry[27] = 0;
        for (i, position) in positions.iter().enumerate() {
            let name_index = (order - 1) * 13 + i;
            let unit = if name_index < units.len() {
                units[name_index]
            } else if name_index == units.len() {
                0
            } else {
                0xffff
            };
            entry[*position..*position + 2].copy_from_slice(&unit.to_le_bytes());
        }
        records.push(entry);
    }
    Ok(records)
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
