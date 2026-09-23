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

The initramfs is mounted as `/`. The build also includes a FAT32 image
mounted at `/disk`. FAT32 supports short and long names, file writes, directory
creation, and removal. `lsblk -f` lists detected disks and partitions with
filesystem labels and UUIDs where available. Shell commands include `mkdir`, `rm`, `touch`, `cp`,
and `write`; for example, `mkdir /disk/notes` followed by
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

Changes live in a sparse RAM overlay and are lost when the machine reboots.
The shell's `sync` command asks mounted storage to commit pending writes. The
boot image has no writable backing device, so it reports that changes remain
volatile; a device with durable flush support can implement the same interface.
The initramfs remains read-only. Use `ls`, `cat /disk/README.TXT`, or
`exec /disk/HELLO.ELF` to access the bundled files.

The kernel probes AHCI/SATA and NVMe controllers for read-only block access.
Under QEMU, attach an optional raw disk as a virtual NVMe device with
`EONOS_NVME_DISK=/path/to/disk.img ./run.sh`. QEMU opens it read-only, and
EonOS's current physical-disk drivers do not issue writes.
