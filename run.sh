#!/usr/bin/env bash
# Boot EonOS in QEMU. Serial output goes to your terminal.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

ISO_NAME="eonos.iso"
UEFI="${UEFI:-1}"

if [ ! -f "$ISO_NAME" ]; then
    echo "No $ISO_NAME found, building it first..."
    ./build.sh
fi

ARGS=(-M q35 -vga std -m 2G -serial stdio -cdrom "$ISO_NAME" -boot d)

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

    if [ ! -f "vars.fd" ]; then
        cp "$OVMF_VARS" vars.fd
    fi

    ARGS+=(
        -drive if=pflash,format=raw,readonly=on,file="$OVMF_CODE"
        -drive if=pflash,format=raw,file=vars.fd
    )
fi

exec qemu-system-x86_64 "${ARGS[@]}"