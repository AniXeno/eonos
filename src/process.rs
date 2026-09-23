//! User processes.
//!
//! A process is one user address space plus one thread running in it
//! (there are no threads-within-a-process yet). `spawn` reads an ELF
//! executable out of the initramfs, loads it into a fresh address space
//! (`elf::load`), builds the initial user stack, and hands both to the
//! scheduler (`scheduler::spawn_user`). The thread owns the address
//! space from then on: when the process exits, or is killed by a CPU
//! exception (`idt::kill_user_process`), the scheduler frees every page
//! it had along with the thread.
//!
//! A process's pid is simply the id of its thread. Exit statuses are
//! kept in a small table until somebody collects them with
//! `take_exit_status`.

#![allow(dead_code)]

use alloc::string::String;
use alloc::vec::Vec;
use core::fmt;

use crate::elf::{self, ElfError};
use crate::pmm::{self, PAGE_SIZE};
use crate::sync::IrqMutex;
use crate::{heap, pit, scheduler, vfs, vmm};
use crate::{log_debug, log_fail, log_ok};

/// One past the highest byte of the user stack. The stack grows down
/// from here; the page below it is the first stack page.
pub const USER_STACK_TOP: u64 = 0x0000_7fff_ffff_f000;
/// Size of the initial user stack. The page below it is left unmapped,
/// so overflowing the stack is a page fault (which kills the process)
/// rather than silent corruption.
const USER_STACK_PAGES: u64 = 16; // 64 KiB

/// The first program the kernel starts.
const INIT_PATH: &str = "/init";
/// If `true`, boot waits (up to `INIT_TIMEOUT_MS`) for `/init` to exit
/// and reports how it went -- right for a demo `/init` that prints
/// something and quits. Set to `false` once `/init` is a long-running
/// program (a shell, a service manager), so boot doesn't sit waiting for
/// something that never ends.
const WAIT_FOR_INIT: bool = false;
const INIT_TIMEOUT_MS: u64 = 5000;

struct Record {
    pid: u64,
    name: String,
    /// `None` while running, `Some(status)` once it has exited.
    status: Option<i64>,
}

/// Stable copy of one process record for user-facing inspection.
pub struct ProcessInfo {
    pub pid: u64,
    pub name: String,
    pub exit_status: Option<i64>,
}

static PROCS: IrqMutex<Vec<Record>> = IrqMutex::new(Vec::new());

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SpawnError {
    /// No such file in the initramfs.
    NotFound,
    /// The file isn't a loadable executable.
    Elf(ElfError),
    NoMemory,
    /// The scheduler had no thread (or kernel stack) to give.
    NoThread,
}

impl fmt::Display for SpawnError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            SpawnError::NotFound => f.write_str("file not found in the initramfs"),
            SpawnError::Elf(e) => write!(f, "cannot load executable: {}", e),
            SpawnError::NoMemory => f.write_str("out of memory"),
            SpawnError::NoThread => f.write_str("no free thread slot"),
        }
    }
}

/// Note that `pid` is running `name`. If the process has already run and
/// exited by the time we get here (the scheduler may preempt us between
/// creating the thread and this call), its record exists already and
/// only needs its name filled in.
fn register(pid: u64, name: &str) {
    let mut procs = PROCS.lock();
    match procs.iter().position(|r| r.pid == pid) {
        Some(i) => procs[i].name = String::from(name),
        None => procs.push(Record {
            pid,
            name: String::from(name),
            status: None,
        }),
    }
}

/// End the calling process with `code`. Called by `sys_exit` and by the
/// user-mode fault handler; never returns. The address space is freed by
/// the scheduler once this thread is off the CPU for good.
pub fn exit_current(code: i64) -> ! {
    let pid = scheduler::current_id();
    crate::syscall::close_process_files(pid);
    let name = {
        let mut procs = PROCS.lock();
        match procs.iter().position(|r| r.pid == pid) {
            Some(i) => {
                procs[i].status = Some(code);
                procs[i].name.clone()
            }
            None => {
                procs.push(Record {
                    pid,
                    name: String::new(),
                    status: Some(code),
                });
                String::new()
            }
        }
    };
    log_debug!("Process", "Exit", "pid {} ({}) exited with status {}", pid, name, code);
    scheduler::exit()
}

/// If `pid` has exited, forget it and return its exit status.
pub fn take_exit_status(pid: u64) -> Option<i64> {
    let mut procs = PROCS.lock();
    let i = procs
        .iter()
        .position(|r| r.pid == pid && r.status.is_some())?;
    procs.swap_remove(i).status
}

/// Return a snapshot without keeping the process-table lock held while
/// callers format or copy the result.
pub fn list() -> Vec<ProcessInfo> {
    PROCS
        .lock()
        .iter()
        .map(|record| ProcessInfo {
            pid: record.pid,
            name: record.name.clone(),
            exit_status: record.status,
        })
        .collect()
}

/// Map the user stack and lay out its initial contents the way the
/// System V x86-64 ABI expects at process entry:
///
/// ```text
///   rsp -> argc
///          argv[0] ... argv[argc-1], NULL
///          envp[0] ... NULL            (none)
///          auxv: AT_NULL               (none)
///          ... argument strings ...
/// ```
///
/// with `rsp` 16-byte aligned. Returns the initial stack pointer, or
/// `None` if out of memory. Every frame is mapped the moment it is
/// allocated, so on failure the address space frees whatever was done.
fn build_stack(aspace: &vmm::AddressSpace, arg0: &str) -> Option<u64> {
    let mut top_frame = 0u64;
    for i in 0..USER_STACK_PAGES {
        let phys = pmm::alloc_frame_zeroed()?;
        aspace.map(USER_STACK_TOP - (i + 1) * PAGE_SIZE, phys, vmm::USER_RW);
        if i == 0 {
            top_frame = phys;
        }
    }

    // Everything below is written into the topmost stack page through
    // the direct map; it all fits comfortably inside that one page.
    let page_base = USER_STACK_TOP - PAGE_SIZE;
    let page = pmm::phys_to_virt(top_frame);
    let write = |virt: u64, bytes: &[u8]| {
        let off = (virt - page_base) as usize;
        debug_assert!(off + bytes.len() <= PAGE_SIZE as usize);
        unsafe { core::ptr::copy_nonoverlapping(bytes.as_ptr(), page.add(off), bytes.len()) };
    };

    let name = arg0.as_bytes();
    let name = &name[..name.len().min(255)];
    // The page is zeroed, so the string's NUL terminator is already there.
    let str_addr = USER_STACK_TOP - (name.len() as u64 + 1);
    write(str_addr, name);

    // argc, argv[0], argv terminator, envp terminator, auxv AT_NULL (key, value)
    let words: [u64; 6] = [1, str_addr, 0, 0, 0, 0];
    let sp = (str_addr & !0xF) - (words.len() as u64) * 8;
    for (i, w) in words.iter().enumerate() {
        write(sp + i as u64 * 8, &w.to_le_bytes());
    }
    Some(sp)
}

/// Start the executable at `path` in the initramfs as a new process and
/// return its pid.
pub fn spawn(path: &str) -> Result<u64, SpawnError> {
    let image = vfs::read_all(path).map_err(|_| SpawnError::NotFound)?;

    let aspace = vmm::create_address_space().ok_or(SpawnError::NoMemory)?;
    let entry = elf::load(&image, &aspace).map_err(SpawnError::Elf)?;
    let rsp = build_stack(&aspace, path).ok_or(SpawnError::NoMemory)?;

    // From here on the thread owns the address space (and if creating
    // the thread fails, `spawn_user` drops it, freeing everything).
    let pid = scheduler::spawn_user("user", aspace, entry, rsp).ok_or(SpawnError::NoThread)?;
    register(pid, path);
    Ok(pid)
}

/// Boot the first user program, `/init`.
pub fn start_init() {
    let frames_before = pmm::free_frame_count();
    let heap_before = heap::heap_size();

    let pid = match spawn(INIT_PATH) {
        Ok(pid) => pid,
        Err(e) => {
            log_fail!("Process", "Init", "Could not start {}: {}", INIT_PATH, e);
            return;
        }
    };
    log_ok!("Process", "Init", "Started {} as pid {}", INIT_PATH, pid);

    if !WAIT_FOR_INIT {
        return;
    }

    let deadline = pit::uptime_ms() + INIT_TIMEOUT_MS;
    let status = loop {
        if let Some(status) = take_exit_status(pid) {
            break status;
        }
        if pit::uptime_ms() > deadline {
            log_fail!(
                "Process",
                "Init",
                "{} (pid {}) still running after {} ms",
                INIT_PATH,
                pid,
                INIT_TIMEOUT_MS
            );
            return;
        }
        scheduler::yield_now();
    };

    // The status is recorded just before the thread actually ends; give
    // the scheduler time to reap it (and free its address space) before
    // counting frames.
    scheduler::sleep_ms(20);

    let heap_growth = (heap::heap_size().saturating_sub(heap_before) as u64 / PAGE_SIZE) as i64;
    let leaked = frames_before as i64 - pmm::free_frame_count() as i64 - heap_growth;

    if status != 0 {
        log_fail!(
            "Process",
            "Init",
            "{} (pid {}) exited with status {}",
            INIT_PATH,
            pid,
            status
        );
    } else if leaked != 0 {
        log_fail!(
            "Process",
            "Init",
            "{} exited cleanly but {} physical frames were not returned",
            INIT_PATH,
            leaked
        );
    } else {
        log_ok!(
            "Process",
            "Init",
            "{} (pid {}) ran to completion, exit status 0, all its memory returned",
            INIT_PATH,
            pid
        );
    }
}

// ---------------------------------------------------------------------
// Self-test
// ---------------------------------------------------------------------

const TEST_CODE_VIRT: u64 = 0x0040_0000;
/// 4 bytes of file data followed by zeros up to `TEST_MEMSZ` (.bss-like).
const TEST_DATA: [u8; 4] = [0xDE, 0xAD, 0xBE, 0xEF];
const TEST_MEMSZ: u64 = 2 * PAGE_SIZE;

/// A minimal, hand-built ELF64 image (header, one program header, four
/// bytes of segment data) with knobs for the fields the loader must
/// validate.
fn make_test_elf(entry: u64, vaddr: u64, filesz: u64, memsz: u64, flags: u32) -> [u8; 124] {
    let mut b = [0u8; 124];
    b[0..4].copy_from_slice(b"\x7fELF");
    b[4] = 2; // ELFCLASS64
    b[5] = 1; // ELFDATA2LSB
    b[6] = 1; // EV_CURRENT
    b[16..18].copy_from_slice(&2u16.to_le_bytes()); // ET_EXEC
    b[18..20].copy_from_slice(&62u16.to_le_bytes()); // EM_X86_64
    b[20..24].copy_from_slice(&1u32.to_le_bytes()); // e_version
    b[24..32].copy_from_slice(&entry.to_le_bytes());
    b[32..40].copy_from_slice(&64u64.to_le_bytes()); // e_phoff
    b[52..54].copy_from_slice(&64u16.to_le_bytes()); // e_ehsize
    b[54..56].copy_from_slice(&56u16.to_le_bytes()); // e_phentsize
    b[56..58].copy_from_slice(&1u16.to_le_bytes()); // e_phnum
    // Program header at offset 64
    b[64..68].copy_from_slice(&1u32.to_le_bytes()); // PT_LOAD
    b[68..72].copy_from_slice(&flags.to_le_bytes());
    b[72..80].copy_from_slice(&120u64.to_le_bytes()); // p_offset: the 4 data bytes
    b[80..88].copy_from_slice(&vaddr.to_le_bytes());
    b[88..96].copy_from_slice(&vaddr.to_le_bytes()); // p_paddr
    b[96..104].copy_from_slice(&filesz.to_le_bytes());
    b[104..112].copy_from_slice(&memsz.to_le_bytes());
    b[112..120].copy_from_slice(&PAGE_SIZE.to_le_bytes()); // p_align
    b[120..124].copy_from_slice(&TEST_DATA);
    b
}

/// Run `elf::load` on `image` into a scratch address space and return
/// the result, dropping the address space (and whatever got mapped).
fn try_load(image: &[u8]) -> Option<Result<u64, ElfError>> {
    let aspace = vmm::create_address_space()?;
    Some(elf::load(image, &aspace))
}

pub fn self_test() {
    let frames_before = pmm::free_frame_count();
    let heap_before = heap::heap_size();

    // --- Part 1: malformed executables are rejected --------------------
    let good = make_test_elf(TEST_CODE_VIRT, TEST_CODE_VIRT, 4, TEST_MEMSZ, 5); // R+X

    let mut bad_magic = good;
    bad_magic[0] = 0;
    let mut not_elf64 = good;
    not_elf64[4] = 1;
    let mut wrong_arch = good;
    wrong_arch[18] = 3; // EM_386
    let kernel_addr = make_test_elf(
        0xffff_ffff_8000_0000,
        0xffff_ffff_8000_0000,
        4,
        TEST_MEMSZ,
        5,
    );
    let null_page = make_test_elf(0x10, 0x10, 4, TEST_MEMSZ, 5);
    let wx = make_test_elf(TEST_CODE_VIRT, TEST_CODE_VIRT, 4, TEST_MEMSZ, 7); // R+W+X
    let entry_outside = make_test_elf(TEST_CODE_VIRT + 0x10_0000, TEST_CODE_VIRT, 4, TEST_MEMSZ, 5);
    let too_long = make_test_elf(TEST_CODE_VIRT, TEST_CODE_VIRT, 4096, TEST_MEMSZ, 5);
    let filesz_over_memsz = make_test_elf(TEST_CODE_VIRT, TEST_CODE_VIRT, 8, 4, 5);

    let cases: [(&str, &[u8], ElfError); 9] = [
        ("truncated header", &good[..32], ElfError::TooShort),
        ("bad magic", &bad_magic, ElfError::BadMagic),
        ("32-bit ELF", &not_elf64, ElfError::NotElf64Le),
        ("wrong architecture", &wrong_arch, ElfError::WrongArch),
        ("kernel-space segment", &kernel_addr, ElfError::BadSegment),
        ("segment at the null page", &null_page, ElfError::BadSegment),
        ("filesz > memsz", &filesz_over_memsz, ElfError::BadSegment),
        ("segment past end of file", &too_long, ElfError::Truncated),
        ("writable + executable", &wx, ElfError::WritableExecutable),
    ];
    for (what, image, expected) in cases {
        match try_load(image) {
            Some(Err(e)) if e == expected => {}
            Some(other) => {
                log_fail!(
                    "Process",
                    "SelfTest",
                    "Loader on '{}': expected {:?}, got {:?}",
                    what,
                    expected,
                    other
                );
                return;
            }
            None => {
                log_fail!("Process", "SelfTest", "No memory for a scratch address space");
                return;
            }
        }
    }
    match try_load(&entry_outside) {
        Some(Err(ElfError::BadEntry)) => {}
        _ => {
            log_fail!("Process", "SelfTest", "Loader accepted an entry point outside every segment");
            return;
        }
    }

    // --- Part 2: a valid image is mapped correctly ---------------------
    let Some(aspace) = vmm::create_address_space() else {
        log_fail!("Process", "SelfTest", "No memory for a scratch address space");
        return;
    };
    match elf::load(&good, &aspace) {
        Ok(entry) if entry == TEST_CODE_VIRT => {}
        other => {
            log_fail!("Process", "SelfTest", "Loader failed on a valid image: {:?}", other);
            return;
        }
    }
    let (Some(p0), Some(p1)) = (
        aspace.translate(TEST_CODE_VIRT),
        aspace.translate(TEST_CODE_VIRT + PAGE_SIZE),
    ) else {
        log_fail!("Process", "SelfTest", "Loaded segment is not fully mapped");
        return;
    };
    if aspace.translate(TEST_CODE_VIRT + 2 * PAGE_SIZE).is_some()
        || aspace.translate(TEST_CODE_VIRT - PAGE_SIZE).is_some()
    {
        log_fail!("Process", "SelfTest", "Loader mapped pages outside the segment");
        return;
    }
    let contents_ok = unsafe {
        let first = pmm::phys_to_virt(p0);
        let second = pmm::phys_to_virt(p1);
        let mut ok = true;
        for (i, &expected) in TEST_DATA.iter().enumerate() {
            ok &= first.add(i).read() == expected;
        }
        // Everything after the file data (rest of page 0, all of page 1)
        // must be zero: that is what .bss relies on.
        for i in TEST_DATA.len()..PAGE_SIZE as usize {
            ok &= first.add(i).read() == 0 && second.add(i).read() == 0;
        }
        ok
    };
    if !contents_ok {
        log_fail!("Process", "SelfTest", "Loaded segment contents are wrong (data or zero fill)");
        return;
    }
    drop(aspace);

    // --- Part 3: nothing leaked -----------------------------------------
    let heap_growth = (heap::heap_size().saturating_sub(heap_before) as u64 / PAGE_SIZE) as i64;
    let leaked = frames_before as i64 - pmm::free_frame_count() as i64 - heap_growth;
    if leaked != 0 {
        log_fail!(
            "Process",
            "SelfTest",
            "Frames leaked by the loader tests: {} free before, {} after (heap grew {} pages)",
            frames_before,
            pmm::free_frame_count(),
            heap_growth
        );
        return;
    }

    log_ok!(
        "Process",
        "SelfTest",
        "ELF loader rejects {} kinds of bad images, maps a valid one with correct data and zero fill, no frames leaked",
        cases.len() + 1
    );
}
