//! Small synchronous block-device interface used by filesystems.

#![allow(dead_code)]

use crate::fat32::SECTOR_SIZE;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BlockError {
    OutOfRange,
    BadBuffer,
}

pub trait BlockDevice: Send + Sync {
    fn sector_count(&self) -> u64;
    fn read_sector(&self, lba: u64, out: &mut [u8; SECTOR_SIZE]) -> Result<(), BlockError>;
}

/// A read-only block device backed by a Limine-loaded disk image.
pub struct MemoryBlockDevice {
    bytes: &'static [u8],
}

impl MemoryBlockDevice {
    pub fn new(bytes: &'static [u8]) -> Option<Self> {
        if bytes.len() < SECTOR_SIZE || bytes.len() % SECTOR_SIZE != 0 {
            return None;
        }
        Some(Self { bytes })
    }
}

unsafe impl Send for MemoryBlockDevice {}
unsafe impl Sync for MemoryBlockDevice {}

impl BlockDevice for MemoryBlockDevice {
    fn sector_count(&self) -> u64 {
        (self.bytes.len() / SECTOR_SIZE) as u64
    }

    fn read_sector(&self, lba: u64, out: &mut [u8; SECTOR_SIZE]) -> Result<(), BlockError> {
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
}
