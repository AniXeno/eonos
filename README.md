<div align="center">

<img src="https://github.com/AniXeno/eonos/blob/main/branding/title_white.png?raw=true" alt="EonOS" width="500">

</div>

# EonOS

[![Language: Rust](https://img.shields.io/badge/Language-Rust-orange.svg)](https://www.rust-lang.org/)
[![Bootloader: Limine](https://img.shields.io/badge/Bootloader-Limine-blue.svg)](https://limine-bootloader.org/)
[![Architecture: x86_64](https://img.shields.io/badge/Architecture-x86__64-lightgrey.svg)](https://en.wikipedia.org/wiki/X86-64)

A custom x86_64 hobby operating system kernel written in Rust and booted using the Limine boot protocol.

EonOS is built from the ground up as an experimental modern operating system project. The main goal is to explore low level systems programming, kernel architecture, memory management, hardware interaction, and the fundamentals required to build an operating system.

The project is still in early development and is primarily intended for learning, experimentation, and eventually building a more complete standalone operating system.

The userspace shell can launch the bundled `/hello` ELF with `exec /hello`.

Use `./runb.sh` for a normal build, which suppresses `DEBUG` log lines and
creates `build/iso_output/eonos.iso`. Use `./runb.sh --debug` to include
`DEBUG` logs and create `build/iso_output/eonos_debugbuild.iso`.

Run `python3 tests/qemu_smoke.py` for a headless QEMU smoke test. It builds
into a temporary directory, checks boot and shell behavior, then rebuilds and
reboots with the same disk image to verify file persistence. It triggers the
`panic` command last to verify the kernel panic path. Prerequisites are QEMU,
the Rust nightly toolchain, GNU binutils, `dosfstools`,
`mtools`, and `xorriso`.

The initramfs is mounted as `/`. The build creates a persistent, sparse 4 GiB
FAT32 image at `build/eonos_disk.img` and attaches it to QEMU as an NVMe disk.
The image is preserved across builds; files written to it survive guest
reboots. The disk starts unmounted so writes require an explicit mount, for
example `mount /device/nvme0n1 /disk`. `/device/...` is the device namespace
used by EonOS for devices such as `/device/console`. `lsblk` lists detected
disks and filesystems, including their mountpoints. FAT32 supports short and
long names, file writes, directory creation, and removal. Shell commands include
`mkdir`, `rm`, `touch`, `cp`, and `write`; for example, `mkdir /disk/notes` followed by
`write /disk/notes/todo.txt remember`. `edit /disk/notes/todo.txt` opens a
small line-oriented editor: enter lines to append, `.save` to write, or
`.quit` to leave. `cp source destination` copies files in 4 KiB chunks.
User processes receive stdin, stdout, and stderr as descriptors 0, 1, and 2.
`/device/console` opens canonical console input and output through the
file descriptor API. `/device/null`, `/device/zero`, and `/device/urandom`
provide EOF/discard, zero-filled, and random-byte streams. `urandom` uses
RDRAND when available and a timing-seeded fallback otherwise; it is not
intended for cryptographic secrets. `exec`
launches a child ELF with `argc`, `argv`, and
`envp` on its initial stack; `wait4` only reaps children of the caller.
Opened files use per-process descriptors with sequential reads and writes;
`open` supports read-only, write-only, read/write, create, truncate, and append
modes.

The shell's `sync` command flushes mounted storage to the backing disk. FAT32
mutations also issue a device flush before reporting success. The root
initramfs remains read-only. After mounting the disk, `ls` shows `/disk` in the
root listing; use `cat /disk/README.TXT` or `exec /disk/HELLO.ELF` to access
the bundled files.

The kernel probes AHCI/SATA and NVMe controllers for block access. NVMe
devices support writes; AHCI/SATA remains read-only. To use a different raw
NVMe disk image in QEMU, set `EONOS_NVME_DISK=/path/to/disk.img ./run.sh`.
