//! The initial RAM filesystem: a plain `ustar` tar archive that Limine
//! loads next to the kernel as a boot module (`module_path` in
//! `limine.conf`).
//!
//! The archive is never copied or unpacked. Limine puts it in memory
//! marked "kernel and modules", which the PMM never hands out and the VMM
//! maps into the higher-half direct map, so the bytes stay valid (and
//! immutable, as far as the kernel is concerned) for the lifetime of the
//! kernel. `find` just walks the 512-byte tar headers and returns a slice
//! straight into that memory.
//!
//! Supported: regular files, in ustar format, names up to 100 bytes plus
//! an optional ustar prefix. Directories, links and extension headers
//! (pax `x`/`g`, GNU `L`/`K`) are skipped. Paths are compared without
//! leading `./` or `/`, so `"/init"`, `"init"` and tar's `"./init"` are
//! all the same file.

#![allow(dead_code)]

use alloc::string::String;
use core::sync::atomic::{AtomicUsize, Ordering};

use limine::request::ModuleRequest;

use crate::sync::IrqMutex;
use crate::{log_debug, log_fail, log_ok};

#[used]
#[link_section = ".requests"]
static MODULE_REQUEST: ModuleRequest = ModuleRequest::new();

/// The archive, once `init` has found and sanity-checked it.
static ARCHIVE: IrqMutex<Option<&'static [u8]>> = IrqMutex::new(None);

/// Number of regular files seen by `init`, for the log line.
static FILE_COUNT: AtomicUsize = AtomicUsize::new(0);

const BLOCK: usize = 512;

/// One regular file in the archive.
pub struct Entry {
    /// Normalized path: no leading `./` or `/`.
    pub name: String,
    pub data: &'static [u8],
}

pub struct Entries {
    data: &'static [u8],
    off: usize,
}

fn cstr(field: &[u8]) -> &[u8] {
    let end = field.iter().position(|&b| b == 0).unwrap_or(field.len());
    &field[..end]
}

/// Parse a tar numeric field: octal digits, optionally padded with
/// leading spaces and terminated by a NUL or space.
fn parse_octal(field: &[u8]) -> Option<u64> {
    let mut value = 0u64;
    let mut seen = false;
    for &b in field {
        match b {
            b'0'..=b'7' => {
                value = value.checked_mul(8)?.checked_add((b - b'0') as u64)?;
                seen = true;
            }
            b' ' | 0 => {
                if seen {
                    break;
                }
            }
            _ => return None,
        }
    }
    if seen {
        Some(value)
    } else {
        None
    }
}

fn checksum_ok(hdr: &[u8]) -> bool {
    // The checksum is the sum of all header bytes with the checksum
    // field itself counted as eight spaces.
    let mut sum = 0u64;
    for (i, &b) in hdr.iter().enumerate() {
        sum += if (148..156).contains(&i) { b' ' as u64 } else { b as u64 };
    }
    parse_octal(&hdr[148..156]) == Some(sum)
}

/// Strip any leading `./` and `/` components.
fn normalize(mut s: &str) -> &str {
    loop {
        if let Some(rest) = s.strip_prefix("./") {
            s = rest;
        } else if let Some(rest) = s.strip_prefix('/') {
            s = rest;
        } else {
            return s;
        }
    }
}

fn entry_name(hdr: &[u8]) -> String {
    let name = cstr(&hdr[0..100]);
    let is_ustar = &hdr[257..262] == b"ustar";
    let prefix = if is_ustar { cstr(&hdr[345..500]) } else { &[][..] };

    let mut full = String::new();
    if !prefix.is_empty() {
        full.push_str(&String::from_utf8_lossy(prefix));
        full.push('/');
    }
    full.push_str(&String::from_utf8_lossy(name));
    String::from(normalize(&full))
}

impl Iterator for Entries {
    type Item = Entry;

    fn next(&mut self) -> Option<Entry> {
        // A copy of the `&'static` slice, so everything borrowed from it
        // below is independent of the borrow of `self`.
        let data: &'static [u8] = self.data;
        loop {
            let hdr_end = self.off.checked_add(BLOCK)?;
            if hdr_end > data.len() {
                return None;
            }
            let hdr = &data[self.off..hdr_end];

            // An all-zero block marks the end of the archive.
            if hdr.iter().all(|&b| b == 0) {
                return None;
            }
            // Anything that isn't a valid header means we've lost sync
            // (or this isn't a tar at all): stop rather than guess.
            if !checksum_ok(hdr) {
                return None;
            }

            let size = parse_octal(&hdr[124..136])?;
            let size = usize::try_from(size).ok()?;
            let data_start = hdr_end;
            let data_end = data_start.checked_add(size)?;
            if data_end > data.len() {
                return None;
            }
            // File data is padded up to a whole number of blocks.
            let padded = size.checked_add(BLOCK - 1)? / BLOCK * BLOCK;
            self.off = data_start.checked_add(padded)?;

            let typeflag = hdr[156];
            if typeflag == b'0' || typeflag == 0 {
                return Some(Entry {
                    name: entry_name(hdr),
                    data: &data[data_start..data_end],
                });
            }
            // Directory, symlink, extension header, ...: skip it.
        }
    }
}

/// Every regular file in the archive, in archive order. Empty if there
/// is no initramfs.
pub fn entries() -> Entries {
    Entries {
        data: (*ARCHIVE.lock()).unwrap_or(&[]),
        off: 0,
    }
}

/// The contents of the file at `path`, or `None`.
pub fn find(path: &str) -> Option<&'static [u8]> {
    let want = normalize(path);
    entries().find(|e| e.name == want).map(|e| e.data)
}

/// Return bytes for a boot module whose configured path ends in
/// `suffix`, such as `initramfs.tar` or `fat32.img`.
pub fn module_bytes(suffix: &str) -> Option<&'static [u8]> {
    let response = MODULE_REQUEST.get_response()?;
    let file = response.modules().iter().find(|f| {
        let path = core::str::from_utf8(f.path().to_bytes()).unwrap_or("");
        path.ends_with(suffix)
    })?;
    let base = file.addr() as *const u8;
    let size = usize::try_from(file.size()).ok()?;
    if size == 0 { return None; }
    let first = base as u64;
    let last = first.checked_add(size as u64 - 1)?;
    if crate::vmm::translate(first).is_none() || crate::vmm::translate(last).is_none() { return None; }
    Some(unsafe { core::slice::from_raw_parts(base, size) })
}

/// Locate the initramfs module Limine loaded. Call after `vmm::init`
/// (the module is reached through the direct map).
pub fn init() {
    let Some(response) = MODULE_REQUEST.get_response() else {
        log_fail!(
            "Initramfs",
            "Init",
            "No response to the module request (is `module_path` set in limine.conf?)"
        );
        return;
    };
    let Some(file) = response.modules().iter().find(|f| {
        core::str::from_utf8(f.path().to_bytes()).unwrap_or("").ends_with("initramfs.tar")
    }) else {
        log_fail!(
            "Initramfs",
            "Init",
            "Limine did not load initramfs.tar (check module_path in limine.conf)"
        );
        return;
    };

    let base = file.addr() as *const u8;
    let size = file.size() as usize;
    if size < BLOCK {
        log_fail!("Initramfs", "Init", "Module is only {} bytes, too small for a tar archive", size);
        return;
    }

    // The direct map is built from the memory map, so a module Limine
    // put somewhere unexpected would show up here as an unmapped page
    // instead of as a page fault later.
    let first = base as u64;
    let last = first + (size as u64 - 1);
    if crate::vmm::translate(first).is_none() || crate::vmm::translate(last).is_none() {
        log_fail!(
            "Initramfs",
            "Init",
            "Module at {:#018x} ({} bytes) is not mapped",
            first,
            size
        );
        return;
    }

    let data: &'static [u8] = unsafe { core::slice::from_raw_parts(base, size) };
    *ARCHIVE.lock() = Some(data);

    let mut count = 0usize;
    for e in entries() {
        log_debug!("Initramfs", "Entry", "/{} ({} bytes)", e.name, e.data.len());
        count += 1;
    }
    FILE_COUNT.store(count, Ordering::Relaxed);

    if count == 0 {
        log_fail!(
            "Initramfs",
            "Init",
            "Module at {:#018x} ({} bytes) holds no files -- is it a ustar tar archive?",
            first,
            size
        );
        return;
    }

    log_ok!(
        "Initramfs",
        "Init",
        "{} files in a {} byte tar archive at {:#018x}",
        count,
        size,
        first
    );
}

pub fn self_test() {
    if ARCHIVE.lock().is_none() {
        log_fail!("Initramfs", "SelfTest", "No initramfs loaded");
        return;
    }

    let Some(init) = find("/init") else {
        log_fail!("Initramfs", "SelfTest", "/init is missing from the initramfs");
        return;
    };
    if !init.starts_with(b"\x7fELF") {
        log_fail!("Initramfs", "SelfTest", "/init is not an ELF file");
        return;
    }
    if find("init").map(|d| d.len()) != Some(init.len())
        || find("./init").map(|d| d.len()) != Some(init.len())
    {
        log_fail!("Initramfs", "SelfTest", "Path normalization disagrees about /init");
        return;
    }
    if find("/definitely/not/here").is_some() {
        log_fail!("Initramfs", "SelfTest", "find() returned a file that does not exist");
        return;
    }

    log_ok!(
        "Initramfs",
        "SelfTest",
        "/init found ({} bytes, ELF), lookups behave ({} files total)",
        init.len(),
        FILE_COUNT.load(Ordering::Relaxed)
    );
}
