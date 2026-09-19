# EonOS

[![Language: Rust](https://img.shields.io/badge/Language-Rust-orange.svg)](https://www.rust-lang.org/)
[![Bootloader: Limine](https://img.shields.io/badge/Bootloader-Limine-blue.svg)](https://limine-bootloader.org/)
[![Architecture: x86_64](https://img.shields.io/badge/Architecture-x86__64-lightgrey.svg)]()

A custom x86_64 hobby operating system kernel written in Rust and booted using the Limine boot protocol.

EonOS is built from the ground up as an experimental modern operating system project. The main goal is to explore low level systems programming, kernel architecture, memory management, hardware interaction, and the fundamentals required to build an operating system.

The project is still in early development and is primarily intended for learning, experimentation, and eventually building a more complete standalone operating system.

---

## Features

Currently implemented:

- x86_64 kernel
- Limine boot protocol support
- Rust based kernel
- Framebuffer initialization
- Basic framebuffer pixel read/write testing
- Kernel initialization and logging
- Bootable ISO generation
- QEMU support for testing

More kernel functionality will be added as development continues.

---

## Project Goals

Some of the longer term goals for EonOS include:

- Physical and virtual memory management
- Interrupt handling
- CPU and architecture abstractions
- Process and thread management
- A proper scheduler
- User mode applications
- System calls
- Filesystem support
- Device drivers
- Networking
- A shell and basic userspace utilities
- Better hardware support
- A more complete userspace environment

The architecture may change significantly as the kernel grows.

---

## Prerequisites

Ensure you have the following tools installed before building:

- **Rust toolchain** with the `nightly` toolchain for bare metal development
- **QEMU** with `qemu-system-x86_64`
- **Xorriso** for ISO creation
- **mtools** for filesystem image manipulation

A Linux development environment is currently recommended.

---

## Building

Run the automated build script to compile the kernel and produce a bootable ISO.

### Debug build

```sh
./build.sh
```

This produces:

```text
eonos.iso
```

### Release build

```sh
./build.sh release
```

The resulting ISO can then be booted using QEMU or written to suitable boot media for testing.

---

## Running

The included run script launches EonOS using QEMU:

```sh
./runb.sh
```

You should see output similar to:

```text
Kernel:Init > EonOS booting via Limine | OK
Framebuffer:Init > Framebuffer Initialized (1024x768 @ 32bpp) | OK
Framebuffer:SelfTest > Pixel read/write verified | OK
Kernel:Init > Initialization complete, halting | OK
```

The current kernel performs its initialization sequence and then halts. As more kernel subsystems are implemented, this behavior will gradually be replaced by an actual kernel runtime environment.

---

## Architecture

EonOS currently targets **x86_64** systems.

The kernel is designed around a small and modular architecture so that individual subsystems can be developed and tested independently. Rust provides memory safety and strong type guarantees while still allowing the low level control required for kernel development.

The current boot process is handled by **Limine**, which provides the kernel with information such as the framebuffer and boot environment before transferring control to EonOS.

---

## Development

EonOS is currently a hobby and educational project. Expect breaking changes, unfinished subsystems, experimental implementations, and occasional completely broken builds.

Development is currently focused on establishing the fundamental kernel infrastructure before moving into more advanced functionality.

Some planned areas include:

```text
Boot
 ├── Limine
 └── Kernel entry

CPU
 ├── GDT
 ├── IDT
 └── Interrupts

Memory
 ├── Physical memory
 ├── Paging
 └── Heap allocation

Kernel
 ├── Scheduler
 ├── Processes
 ├── Threads
 └── Syscalls

Userspace
 ├── Shell
 ├── Utilities
 └── Applications

Hardware
 ├── Storage
 ├── Input
 ├── Display
 └── Networking
```

---

## AI Disclosure

This project is developed with AI assistance for **partial code generation, debugging, documentation, research, and architectural advice**.

AI generated code is reviewed, modified, tested, and integrated manually as part of the development process. AI assistance does not replace the project's own design decisions or testing.

---

## Status

**EonOS is currently experimental and under active development.**

The project is not intended to replace an existing operating system yet. At its current stage, it is primarily a learning project focused on understanding how modern operating systems work internally.

More functionality will be added as the kernel develops.