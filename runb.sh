#!/usr/bin/env bash
# Build and Run EonOS in QEMU. Serial output goes to your terminal.
set -euo pipefail

./build.sh
exec ./run.sh