#!/usr/bin/env bash
# Boot EonOS in QEMU. Serial output goes to your terminal.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

BUILD_DIR="$ROOT/build"
ISO_PATH="$BUILD_DIR/iso_output/eonos.iso"

UEFI="${UEFI:-1}"

if [ ! -f "$ISO_PATH" ]; then
    echo "No built ISO found, building it first..."
    ./build.sh
fi

ARGS=(
    -M q35
    -vga std
    -m 2G
    -serial stdio
    -cdrom "$ISO_PATH"
    -boot d
)

if [ "$UEFI" = "1" ]; then
    OVMF_CODE="/usr/share/edk2/x64/OVMF_CODE.4m.fd"
    OVMF_VARS="/usr/share/edk2/x64/OVMF_VARS.4m.fd"

    if [ ! -f "$OVMF_CODE" ]; then
        OVMF_CODE="/usr/share/edk2-ovmf/x64/OVMF_CODE.4m.fd"
        OVMF_VARS="/usr/share/edk2-ovmf/x64/OVMF_VARS.4m.fd"
    fi

    if [ ! -f "$OVMF_CODE" ]; then
        echo "OVMF firmware not found. Install edk2-ovmf:"
        echo "  sudo pacman -S edk2-ovmf"
        exit 1
    fi

    VARS_FILE="$BUILD_DIR/OVMF_VARS.fd"

    if [ ! -f "$VARS_FILE" ]; then
        cp "$OVMF_VARS" "$VARS_FILE"
    fi

    ARGS+=(
        -drive if=pflash,format=raw,readonly=on,file="$OVMF_CODE"
        -drive if=pflash,format=raw,file="$VARS_FILE"
    )
fi

exec qemu-system-x86_64 "${ARGS[@]}"