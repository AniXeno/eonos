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

The initramfs is mounted as `/`. The build also includes a read-only FAT32
image mounted at `/disk`; use `ls`, `cat /disk/README.TXT`, or
`exec /disk/HELLO.ELF` from the shell to access it.
