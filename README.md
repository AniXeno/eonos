# EonOS

A hobby x86_64 operating system kernel written in Rust, booted via
[Limine](https://limine-bootloader.org/).

## Prerequisites

You already have these (confirmed via your terminal output):

- `rustc`/`cargo` (nightly toolchain — pinned in `rust-toolchain.toml`)
- `clang` / `ld.lld`
- `nasm`
- `qemu-system-x86_64`
- `gdb`
- `xorriso`, `mtools`, `edk2-ovmf` (installed via pacman)

Make sure the nightly `rust-src` and `llvm-tools-preview` components are
present (rustup will pull these automatically the first time you build,
thanks to `rust-toolchain.toml`):

```sh
rustup component add rust-src llvm-tools-preview --toolchain nightly
```

## Building

```sh
./build.sh          # debug build -> eonos.iso
./build.sh release  # release build -> eonos.iso
```

The first run clones and builds Limine's binary branch into `limine/`
(cached afterwards — delete the folder to force a re-fetch/update).

## Running

```sh
./run.sh            # BIOS boot via QEMU, serial log printed to your terminal
UEFI=1 ./run.sh      # UEFI boot via OVMF instead
```

You should see something like:

```
Kernel:Init > EonOS booting via Limine | OK
Framebuffer:Init > Framebuffer Initialized (1024x768 @ 32bpp) | OK
Framebuffer:SelfTest > Pixel read/write verified | OK
Kernel:Init > Initialization complete, halting | OK
```

## Logging system

Every subsystem logs through the macros in `src/logger.rs`, using the format:

```
Module:Function > Output | STATUS
```

- **Module** — subsystem name (`PMM`, `VMM`, `Memory`, `Framebuffer`, `GDT`, `IDT`, ...)
- **Function** — what it was doing (`Init`, `SelfTest`, ...)
- **Output** — free-form message, `format_args!`-style
- **Status** — `OK`, `BUSY`, `FAIL`, `CRITICAL`, `DEBUG`

Macros available: `log_ok!`, `log_busy!`, `log_fail!`, `log_critical!`, `log_debug!`.

```rust
log_ok!("PMM", "Init", "Mapped {} pages", page_count);
// -> PMM:Init > Mapped 4096 pages | OK
```

Output currently goes to the COM1 serial port (`-serial stdio` in QEMU), with
ANSI colors per status level. A framebuffer text console can be layered on
top later using the same `Logger` sink.

## Console font & colours

The screen console renders glyphs from a **PSF1/PSF2 bitmap font** embedded at
build time from `font.psf` (project root). Any size works (8x16, 10x18, 16x32...).
To change the look, replace `font.psf` with an **uncompressed** PSF and rebuild:

```sh
ls /usr/share/kbd/consolefonts/                                  # fonts on your system
gunzip -c /usr/share/kbd/consolefonts/ter-v16n.psf.gz > font.psf  # e.g. Terminus 8x16
./build.sh && ./run.sh
```

The bundled `font.psf` was generated from GNU Unifont with `tools/make_psf.py`
(`make_psf.py FONT.otf OUT.psf [height] [width] [baseline]`).
Glyph N is expected to be codepoint N (true for ASCII in all common PSFs).

Colour is done with ANSI SGR escapes (`ESC[31m`, `ESC[92m`, `ESC[38;2;R;G;Bm`,
`ESC[0m`...), understood by both the serial port and the framebuffer console.
The 16-colour palette lives in `PALETTE` at the top of `src/console.rs`.

## Project layout

```
src/
  main.rs         kernel entry point (_start), Limine requests, panic handler
  logger.rs       Module:Function > Output | Status logging system
  serial.rs       16550 UART driver (log sink)
  framebuffer.rs  Limine framebuffer request + init/self-test
  console.rs      framebuffer text console (PSF font, ANSI colours)
font.psf          console font (PSF2)
tools/make_psf.py TTF/OTF -> PSF2 converter
linker.ld         higher-half kernel linker script
x86_64-eonos.json custom Rust target spec (freestanding, no SSE, kernel code model)
limine.conf       Limine boot menu config
build.sh          builds kernel + fetches Limine + makes eonos.iso
run.sh            boots eonos.iso in QEMU
```

## Roadmap

Next subsystems to bring up, in the order that tends to work best:

1. **GDT** — flat 64-bit segments + TSS (needed before IDT for the double-fault IST)
2. **IDT** — exception handlers, then hardware interrupts (PIC/APIC)
3. **PMM** — physical memory manager, using Limine's memory map request
4. **VMM** — page tables / virtual memory manager, building on Limine's HHDM
5. **Memory** — kernel heap allocator (`#[global_allocator]`) on top of VMM

Each should log through `log_ok!`/`log_fail!`/etc. exactly like `Framebuffer`
does now, e.g. `PMM:Init > Mapped 4096 pages | OK`.
