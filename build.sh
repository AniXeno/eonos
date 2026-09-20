#!/usr/bin/env bash
# EonOS build script.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

PROFILE="${1:-dev}"
TARGET_DIR="target/x86_64-eonos"
LIMINE_DIR="limine"
ISO_ROOT="iso_root"
ISO_NAME="eonos.iso"

echo "==> Building kernel (profile: $PROFILE)"
BUILD_STD_FLAGS=(
    -Zjson-target-spec
    -Zbuild-std=core,alloc,compiler_builtins
    -Zbuild-std-features=compiler-builtins-mem
)

if [ "$PROFILE" = "release" ]; then
    cargo build --release "${BUILD_STD_FLAGS[@]}" --target x86_64-eonos.json
    KERNEL_BIN="$TARGET_DIR/release/eonos"
else
    cargo build "${BUILD_STD_FLAGS[@]}" --target x86_64-eonos.json
    KERNEL_BIN="$TARGET_DIR/debug/eonos"
fi

if [ ! -d "$LIMINE_DIR" ]; then
    echo "==> Fetching Limine (binary branch)"
    git clone https://github.com/limine-bootloader/limine.git --branch v8.x-binary --depth=1 "$LIMINE_DIR"
    make -C "$LIMINE_DIR"
fi

echo "==> Assembling ISO tree"
rm -rf "$ISO_ROOT"
mkdir -p "$ISO_ROOT/boot" "$ISO_ROOT/boot/limine" "$ISO_ROOT/EFI/BOOT"

cp "$KERNEL_BIN" "$ISO_ROOT/boot/eonos.elf"
cp limine.conf "$ISO_ROOT/boot/limine/limine.conf"
cp limine.conf "$ISO_ROOT/limine.conf"

cp "$LIMINE_DIR/limine-bios.sys" "$LIMINE_DIR/limine-bios-cd.bin" "$LIMINE_DIR/limine-uefi-cd.bin" "$ISO_ROOT/boot/limine/"
cp "$LIMINE_DIR/BOOTX64.EFI" "$ISO_ROOT/EFI/BOOT/"
cp "$LIMINE_DIR/BOOTIA32.EFI" "$ISO_ROOT/EFI/BOOT/" 2>/dev/null || true

echo "==> Building ISO"
xorriso -as mkisofs -R -r -J -b boot/limine/limine-bios-cd.bin \
    -no-emul-boot -boot-load-size 4 -boot-info-table \
    -hfsplus -apm-block-size 2048 \
    --efi-boot boot/limine/limine-uefi-cd.bin \
    -efi-boot-part --efi-boot-image --protective-msdos-label \
    "$ISO_ROOT" -o "$ISO_NAME"

echo "==> Installing BIOS stage 1 into ISO"
"$LIMINE_DIR/limine" bios-install "$ISO_NAME"

echo "==> Done: $ISO_NAME"