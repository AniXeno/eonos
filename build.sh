#!/usr/bin/env bash
# EonOS build script.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

PROFILE="${1:-dev}"

BUILD_DIR="$ROOT/build"
ISO_ROOT="$BUILD_DIR/iso_root"
ISO_OUTPUT="$BUILD_DIR/iso_output"

LIMINE_DIR="$ROOT/limine"
USERLAND_DIR="$ROOT/userland"
INITRAMFS_SRC="$ROOT/initramfs"

INITRAMFS_ROOT="$BUILD_DIR/initramfs_root"
INITRAMFS_NAME="initramfs.tar"
ISO_NAME="eonos.iso"
ISO_PATH="$ISO_OUTPUT/$ISO_NAME"

TARGET_DIR="$ROOT/target/x86_64-eonos"

echo "==> Cleaning build directory"
rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR" "$ISO_OUTPUT"

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

echo "==> Building userland"

as --64 "$USERLAND_DIR/init.S" -o "$BUILD_DIR/init.o"

ld -static -nostdlib \
    -z max-page-size=4096 \
    -z noexecstack \
    -T "$USERLAND_DIR/linker.ld" \
    -o "$BUILD_DIR/init.elf" \
    "$BUILD_DIR/init.o"

echo "==> Packing initramfs"

mkdir -p "$INITRAMFS_ROOT"

cp -r "$INITRAMFS_SRC"/. "$INITRAMFS_ROOT"/
cp "$BUILD_DIR/init.elf" "$INITRAMFS_ROOT/init"

tar --format=ustar \
    -cf "$BUILD_DIR/$INITRAMFS_NAME" \
    -C "$INITRAMFS_ROOT" .

if [ ! -d "$LIMINE_DIR" ]; then
    echo "==> Fetching Limine (binary branch)"

    git clone \
        https://github.com/limine-bootloader/limine.git \
        --branch v8.x-binary \
        --depth=1 \
        "$LIMINE_DIR"

    make -C "$LIMINE_DIR"
fi

echo "==> Assembling ISO tree"

mkdir -p \
    "$ISO_ROOT/boot/limine" \
    "$ISO_ROOT/EFI/BOOT"

cp "$KERNEL_BIN" \
    "$ISO_ROOT/boot/eonos.elf"

cp "$ROOT/limine.conf" \
    "$ISO_ROOT/boot/limine/limine.conf"

cp "$ROOT/limine.conf" \
    "$ISO_ROOT/limine.conf"

cp "$BUILD_DIR/$INITRAMFS_NAME" \
    "$ISO_ROOT/boot/$INITRAMFS_NAME"

cp \
    "$LIMINE_DIR/limine-bios.sys" \
    "$LIMINE_DIR/limine-bios-cd.bin" \
    "$LIMINE_DIR/limine-uefi-cd.bin" \
    "$ISO_ROOT/boot/limine/"

cp \
    "$LIMINE_DIR/BOOTX64.EFI" \
    "$ISO_ROOT/EFI/BOOT/"

cp \
    "$LIMINE_DIR/BOOTIA32.EFI" \
    "$ISO_ROOT/EFI/BOOT/" \
    2>/dev/null || true

echo "==> Building ISO"

xorriso -as mkisofs \
    -R -r -J \
    -b boot/limine/limine-bios-cd.bin \
    -no-emul-boot \
    -boot-load-size 4 \
    -boot-info-table \
    -hfsplus \
    -apm-block-size 2048 \
    --efi-boot boot/limine/limine-uefi-cd.bin \
    -efi-boot-part \
    --efi-boot-image \
    --protective-msdos-label \
    "$ISO_ROOT" \
    -o "$ISO_PATH"

echo "==> Installing BIOS stage 1 into ISO"

"$LIMINE_DIR/limine" bios-install "$ISO_PATH"

echo "==> Done: $ISO_PATH"