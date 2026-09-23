//! Small synchronous block-device interface used by filesystems.

#![allow(dead_code)]

use crate::fat32::SECTOR_SIZE;
use crate::sync::IrqMutex;
use alloc::collections::BTreeMap;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BlockError {
    OutOfRange,
    BadBuffer,
    ReadOnly,
}

pub trait BlockDevice: Send + Sync {
    fn sector_count(&self) -> u64;
    fn read_sector(&self, lba: u64, out: &mut [u8; SECTOR_SIZE]) -> Result<(), BlockError>;
    fn write_sector(&self, _lba: u64, _data: &[u8; SECTOR_SIZE]) -> Result<(), BlockError> {
        Err(BlockError::ReadOnly)
    }
}

/// A block device backed by a Limine-loaded image. Writes are kept in a
/// sparse RAM overlay, leaving the bootloader's original module intact.
pub struct MemoryBlockDevice {
    bytes: &'static [u8],
    overlay: IrqMutex<BTreeMap<u64, [u8; SECTOR_SIZE]>>,
}

impl MemoryBlockDevice {
    pub fn new(bytes: &'static [u8]) -> Option<Self> {
        if bytes.len() < SECTOR_SIZE || bytes.len() % SECTOR_SIZE != 0 {
            return None;
        }
        Some(Self { bytes, overlay: IrqMutex::new(BTreeMap::new()) })
    }
}

unsafe impl Send for MemoryBlockDevice {}
unsafe impl Sync for MemoryBlockDevice {}

impl BlockDevice for MemoryBlockDevice {
    fn sector_count(&self) -> u64 {
        (self.bytes.len() / SECTOR_SIZE) as u64
    }

    fn read_sector(&self, lba: u64, out: &mut [u8; SECTOR_SIZE]) -> Result<(), BlockError> {
        if let Some(sector) = self.overlay.lock().get(&lba) {
            out.copy_from_slice(sector);
            return Ok(());
        }
        let start = usize::try_from(lba)
            .ok()
            .and_then(|n| n.checked_mul(SECTOR_SIZE))
            .ok_or(BlockError::OutOfRange)?;
        let end = start
            .checked_add(SECTOR_SIZE)
            .ok_or(BlockError::OutOfRange)?;
        let src = self.bytes.get(start..end).ok_or(BlockError::OutOfRange)?;
        out.copy_from_slice(src);
        Ok(())
    }

    fn write_sector(&self, lba: u64, data: &[u8; SECTOR_SIZE]) -> Result<(), BlockError> {
        if lba >= self.sector_count() {
            return Err(BlockError::OutOfRange);
        }
        self.overlay.lock().insert(lba, *data);
        Ok(())
    }
}
