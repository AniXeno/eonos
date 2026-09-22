#!/usr/bin/env bash
# Build and Run EonOS in QEMU. Serial output goes to your terminal.
set -euo pipefail

if [ "$#" -eq 1 ] && [ "$1" = "--debug" ]; then
    ./build.sh --debug
    exec ./run.sh --debug
fi

if [ "$#" -ne 0 ]; then
    echo "Usage: $0 [--debug]" >&2
    exit 2
fi

./build.sh
exec ./run.sh
