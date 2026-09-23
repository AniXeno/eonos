//! Small path-based VFS. Filesystems are mounted at path prefixes and
//! implement lookup/listing, with writes currently supported by FAT32 mounts.

#![allow(dead_code)]

use alloc::string::{String, ToString};
use alloc::vec::Vec;

use crate::block::BlockDevice;
use crate::fat32::{DirEntry, Fat32};
use crate::sync::IrqMutex;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FsError {
    NotFound,
    NotDirectory,
    IsDirectory,
    Io,
    InvalidPath,
    TooLarge,
    ReadOnly,
    NoSpace,
    InvalidName,
    AlreadyExists,
    DirectoryNotEmpty,
}

enum Filesystem {
    Initramfs,
    Fat32(Fat32),
}
struct Mount {
    prefix: String,
    fs: Filesystem,
}
static MOUNTS: IrqMutex<Vec<Mount>> = IrqMutex::new(Vec::new());
const MAX_READ_ALL: usize = 16 * 1024 * 1024;

fn normalize(path: &str) -> Result<String, FsError> {
    let mut parts: Vec<&str> = Vec::new();
    for p in path.split('/') {
        match p {
            "" | "." => {}
            ".." => {
                if parts.pop().is_none() {
                    return Err(FsError::InvalidPath);
                }
            }
            _ => parts.push(p),
        }
    }
    let mut out = String::from("/");
    out.push_str(&parts.join("/"));
    Ok(out)
}

fn route<'m, 'p>(mounts: &'m [Mount], path: &'p str) -> Option<(&'m Mount, &'p str)> {
    mounts
        .iter()
        .filter_map(|m| {
            if m.prefix == "/" {
                Some((m, path.trim_start_matches('/')))
            } else if path == m.prefix {
                Some((m, ""))
            } else if path
                .strip_prefix(&m.prefix)
                .is_some_and(|tail| tail.starts_with('/'))
            {
                Some((
                    m,
                    path.strip_prefix(&m.prefix)
                        .unwrap()
                        .trim_start_matches('/'),
                ))
            } else {
                None
            }
        })
        .max_by_key(|(m, _)| m.prefix.len())
}

/// Mount the boot archive as the root filesystem.
pub fn init() {
    let mut mounts = MOUNTS.lock();
    mounts.clear();
    mounts.push(Mount {
        prefix: String::from("/"),
        fs: Filesystem::Initramfs,
    });
}

/// Mount a FAT32 volume at an absolute path, e.g. `/disk`.
pub fn mount_fat32(prefix: &str, device: &'static dyn BlockDevice) -> Result<(), FsError> {
    let prefix = normalize(prefix)?;
    if prefix == "/" {
        return Err(FsError::InvalidPath);
    }
    let fs = Fat32::mount(device).map_err(|_| FsError::Io)?;
    let mut mounts = MOUNTS.lock();
    if mounts.iter().any(|m| m.prefix == prefix) {
        return Err(FsError::InvalidPath);
    }
    mounts.push(Mount {
        prefix,
        fs: Filesystem::Fat32(fs),
    });
    Ok(())
}

pub fn read_all(path: &str) -> Result<Vec<u8>, FsError> {
    let size = file_size(path)?;
    if size > MAX_READ_ALL {
        return Err(FsError::TooLarge);
    }
    let mut data = alloc::vec![0; size];
    let n = read_at(path, 0, &mut data)?;
    if n != size {
        return Err(FsError::Io);
    }
    Ok(data)
}

pub fn read_at(path: &str, offset: usize, out: &mut [u8]) -> Result<usize, FsError> {
    let path = normalize(path)?;
    let mounts = MOUNTS.lock();
    let (mount, inner) = route(&mounts, &path).ok_or(FsError::NotFound)?;
    match &mount.fs {
        Filesystem::Initramfs => {
            let data = crate::initramfs::find(inner).ok_or(FsError::NotFound)?;
            if offset >= data.len() {
                return Ok(0);
            }
            let n = out.len().min(data.len() - offset);
            out[..n].copy_from_slice(&data[offset..offset + n]);
            Ok(n)
        }
        Filesystem::Fat32(fs) => fs.read_at(inner, offset, out).map_err(map_fat_error),
    }
}

pub fn create_file(path: &str) -> Result<(), FsError> {
    let path = normalize(path)?;
    let mounts = MOUNTS.lock();
    let (mount, inner) = route(&mounts, &path).ok_or(FsError::NotFound)?;
    match &mount.fs {
        Filesystem::Initramfs => Err(FsError::ReadOnly),
        Filesystem::Fat32(fs) => fs.create_file(inner).map_err(map_fat_error),
    }
}

pub fn create_dir(path: &str) -> Result<(), FsError> {
    let path = normalize(path)?;
    let mounts = MOUNTS.lock();
    let (mount, inner) = route(&mounts, &path).ok_or(FsError::NotFound)?;
    match &mount.fs {
        Filesystem::Initramfs => Err(FsError::ReadOnly),
        Filesystem::Fat32(fs) => fs.create_dir(inner).map_err(map_fat_error),
    }
}

pub fn remove(path: &str) -> Result<(), FsError> {
    let path = normalize(path)?;
    let mounts = MOUNTS.lock();
    let (mount, inner) = route(&mounts, &path).ok_or(FsError::NotFound)?;
    match &mount.fs {
        Filesystem::Initramfs => Err(FsError::ReadOnly),
        Filesystem::Fat32(fs) => fs.remove(inner).map_err(map_fat_error),
    }
}

pub fn truncate(path: &str) -> Result<(), FsError> {
    let path = normalize(path)?;
    let mounts = MOUNTS.lock();
    let (mount, inner) = route(&mounts, &path).ok_or(FsError::NotFound)?;
    match &mount.fs {
        Filesystem::Initramfs => Err(FsError::ReadOnly),
        Filesystem::Fat32(fs) => fs.truncate(inner).map_err(map_fat_error),
    }
}

pub fn write_at(path: &str, offset: usize, data: &[u8]) -> Result<usize, FsError> {
    let path = normalize(path)?;
    let mounts = MOUNTS.lock();
    let (mount, inner) = route(&mounts, &path).ok_or(FsError::NotFound)?;
    match &mount.fs {
        Filesystem::Initramfs => Err(FsError::ReadOnly),
        Filesystem::Fat32(fs) => fs.write_at(inner, offset, data).map_err(map_fat_error),
    }
}

pub fn file_size(path: &str) -> Result<usize, FsError> {
    let path = normalize(path)?;
    let mounts = MOUNTS.lock();
    let (mount, inner) = route(&mounts, &path).ok_or(FsError::NotFound)?;
    match &mount.fs {
        Filesystem::Initramfs => crate::initramfs::find(inner)
            .map(|b| b.len())
            .ok_or(FsError::NotFound),
        Filesystem::Fat32(fs) => {
            let e = fs.stat(inner).map_err(map_fat_error)?;
            if e.is_dir {
                Err(FsError::IsDirectory)
            } else {
                Ok(e.size as usize)
            }
        }
    }
}

pub fn list(path: &str) -> Result<Vec<DirEntry>, FsError> {
    let path = normalize(path)?;
    let mounts = MOUNTS.lock();
    let (mount, inner) = route(&mounts, &path).ok_or(FsError::NotFound)?;
    let mut out = match &mount.fs {
        Filesystem::Fat32(fs) => fs.list(inner).map_err(map_fat_error),
        Filesystem::Initramfs => {
            let prefix = if inner.is_empty() {
                String::new()
            } else {
                alloc::format!("{}/", inner.trim_end_matches('/'))
            };
            let mut out = Vec::new();
            for e in crate::initramfs::entries() {
                let Some(rest) = e.name.strip_prefix(&prefix) else {
                    continue;
                };
                if rest.is_empty() {
                    continue;
                }
                let name = rest.split('/').next().unwrap_or(rest);
                if !out.iter().any(|entry: &DirEntry| entry.name == name) {
                    let is_dir = rest.contains('/');
                    out.push(DirEntry {
                        name: name.to_string(),
                        is_dir,
                        size: if is_dir { 0 } else { e.data.len() as u32 },
                    });
                }
            }
            if !inner.is_empty() && crate::initramfs::find(inner).is_some() {
                return Err(FsError::NotDirectory);
            }
            if out.is_empty() && !inner.is_empty() {
                return Err(FsError::NotFound);
            }
            Ok(out)
        }
    }?;
    // Mounted filesystems appear as directories in their parent mount.
    for child_mount in mounts.iter() {
        if child_mount.prefix == path {
            continue;
        }
        let rest = if path == "/" {
            child_mount.prefix.trim_start_matches('/')
        } else if let Some(tail) = child_mount.prefix.strip_prefix(path.trim_end_matches('/')) {
            tail.trim_start_matches('/')
        } else {
            continue;
        };
        let Some(name) = rest.split('/').next().filter(|s| !s.is_empty()) else {
            continue;
        };
        if !out.iter().any(|e| e.name == name) {
            out.push(DirEntry {
                name: name.to_string(),
                is_dir: true,
                size: 0,
            });
        }
    }
    Ok(out)
}

/// List all files recursively, returning paths relative to the VFS root.
pub fn list_files() -> Vec<String> {
    fn walk(path: &str, out: &mut Vec<String>, depth: usize) {
        if depth >= 32 || out.len() >= 4096 {
            return;
        }
        let Ok(entries) = list(path) else { return };
        for e in entries {
            let child = if path == "/" {
                alloc::format!("/{}", e.name)
            } else {
                alloc::format!("{}/{}", path.trim_end_matches('/'), e.name)
            };
            if e.is_dir {
                walk(&child, out, depth + 1);
            } else {
                out.push(child);
                if out.len() >= 4096 {
                    break;
                }
            }
        }
    }
    let mut out = Vec::new();
    walk("/", &mut out, 0);
    out
}

fn map_fat_error(e: crate::fat32::FatError) -> FsError {
    use crate::fat32::FatError as F;
    match e {
        F::NotFound => FsError::NotFound,
        F::NotDirectory => FsError::NotDirectory,
        F::IsDirectory => FsError::IsDirectory,
        F::Io => FsError::Io,
        F::TooLarge => FsError::TooLarge,
        F::NoSpace => FsError::NoSpace,
        F::InvalidName => FsError::InvalidName,
        F::AlreadyExists => FsError::AlreadyExists,
        F::DirectoryNotEmpty => FsError::DirectoryNotEmpty,
        _ => FsError::Io,
    }
}
