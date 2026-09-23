#!/usr/bin/env bash
# EonOS build script.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

PROFILE="dev"
DEBUG_BUILD=0
for arg in "$@"; do
    case "$arg" in
        --debug) DEBUG_BUILD=1 ;;
        dev|release) PROFILE="$arg" ;;
        *) echo "Unknown build option: $arg" >&2; exit 2 ;;
    esac
done

BUILD_DIR="$ROOT/build"
ISO_ROOT="$BUILD_DIR/iso_root"
ISO_OUTPUT="$BUILD_DIR/iso_output"

LIMINE_DIR="$ROOT/limine"
USERLAND_DIR="$ROOT/userland"
INITRAMFS_SRC="$ROOT/initramfs"

INITRAMFS_ROOT="$BUILD_DIR/initramfs_root"
INITRAMFS_NAME="initramfs.tar"
if [ "$DEBUG_BUILD" = "1" ]; then
    ISO_NAME="eonos_debugbuild.iso"
else
    ISO_NAME="eonos.iso"
fi
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
    if [ "$DEBUG_BUILD" = "1" ]; then
        cargo build --release --features debug-logs "${BUILD_STD_FLAGS[@]}" --target x86_64-eonos.json
    else
        cargo build --release "${BUILD_STD_FLAGS[@]}" --target x86_64-eonos.json
    fi
    KERNEL_BIN="$TARGET_DIR/release/eonos"
else
    if [ "$DEBUG_BUILD" = "1" ]; then
        cargo build --features debug-logs "${BUILD_STD_FLAGS[@]}" --target x86_64-eonos.json
    else
        cargo build "${BUILD_STD_FLAGS[@]}" --target x86_64-eonos.json
    fi
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

as --64 "$USERLAND_DIR/hello.S" -o "$BUILD_DIR/hello.o"

ld -static -nostdlib \
    -z max-page-size=4096 \
    -z noexecstack \
    -T "$USERLAND_DIR/linker.ld" \
    -o "$BUILD_DIR/hello.elf" \
    "$BUILD_DIR/hello.o"

echo "==> Creating FAT32 data volume"
for tool in mkfs.fat mcopy; do
    if ! command -v "$tool" >/dev/null 2>&1; then
        echo "Required tool '$tool' is missing (install dosfstools and mtools)" >&2
        exit 1
    fi
done
FAT32_IMAGE="$BUILD_DIR/fat32.img"
truncate -s 64M "$FAT32_IMAGE"
mkfs.fat -F 32 -n EONOS "$FAT32_IMAGE" >/dev/null
mcopy -i "$FAT32_IMAGE" "$BUILD_DIR/hello.elf" ::/HELLO.ELF
mcopy -i "$FAT32_IMAGE" "$ROOT/README.md" ::/README.TXT

echo "==> Packing initramfs"

mkdir -p "$INITRAMFS_ROOT"

cp -r "$INITRAMFS_SRC"/. "$INITRAMFS_ROOT"/
cp "$BUILD_DIR/init.elf" "$INITRAMFS_ROOT/init"
cp "$BUILD_DIR/hello.elf" "$INITRAMFS_ROOT/hello"

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

cp "$FAT32_IMAGE" "$ISO_ROOT/boot/fat32.img"

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
echo "==> QEMU launch scripts attach a virtual USB keyboard; run ./run.sh or ./runb.sh to test it"
