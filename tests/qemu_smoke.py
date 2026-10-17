#!/usr/bin/env python3
"""Build EonOS in a temporary directory and smoke-test it in headless QEMU."""

from __future__ import annotations

import os
import selectors
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PROMPT = b"eonsh> "


class SmokeTest:
    def __init__(self, process: subprocess.Popen[bytes], timeout: float = 30.0):
        self.process = process
        self.timeout = timeout
        self.output = bytearray()
        self.cursor = 0
        self.selector = selectors.DefaultSelector()
        assert process.stdout is not None
        self.selector.register(process.stdout, selectors.EVENT_READ)

    def read_until(self, needle: bytes, start: int, description: str) -> bytes:
        deadline = time.monotonic() + self.timeout
        while True:
            found = self.output.find(needle, start)
            if found >= 0:
                return bytes(self.output[start : found + len(needle)])
            if self.process.poll() is not None:
                self.fail(f"QEMU exited while waiting for {description}")
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self.fail(f"Timed out waiting for {description}")
            events = self.selector.select(min(remaining, 0.25))
            for key, _ in events:
                chunk = os.read(key.fd, 4096)
                if chunk:
                    self.output.extend(chunk)

    def expect(self, needle: bytes, description: str, start: int = 0) -> None:
        self.read_until(needle, start, description)

    def command(self, line: str, expected: tuple[bytes, ...] = ()) -> bytes:
        assert self.process.stdin is not None
        start = len(self.output)
        self.process.stdin.write(line.encode("utf-8") + b"\n")
        self.process.stdin.flush()
        response = self.read_until(PROMPT, start, f"prompt after {line!r}")
        for marker in expected:
            if marker not in response:
                self.fail(f"{line!r} did not produce {marker!r}")
        print(f"PASS shell: {line or '<blank line>'}")
        return response

    def fail(self, message: str) -> None:
        output = bytes(self.output).decode("utf-8", errors="replace")
        raise RuntimeError(f"{message}\n--- QEMU serial output ---\n{output[-12000:]}")


def run() -> None:
    qemu = shutil.which("qemu-system-x86_64")
    if qemu is None:
        raise RuntimeError("qemu-system-x86_64 is required")

    with tempfile.TemporaryDirectory(prefix="eonos-qemu-smoke-") as tmp:
        build_dir = Path(tmp) / "build"
        env = os.environ.copy()
        env["EONOS_BUILD_DIR"] = str(build_dir)

        print("Building EonOS in a temporary directory...")
        subprocess.run([str(ROOT / "build.sh")], cwd=ROOT, env=env, check=True)
        iso = build_dir / "iso_output" / "eonos.iso"
        disk = build_dir / "eonos_disk.img"
        if not iso.is_file():
            raise RuntimeError(f"build did not produce {iso}")
        if not disk.is_file() or disk.stat().st_size != 4 * 1024**3:
            raise RuntimeError("build did not produce the expected 4 GiB disk image")

        args = [
            qemu,
            "-M", "q35,i8042=off",
            "-m", "2G",
            "-vga", "std",
            "-display", "none",
            "-monitor", "none",
            "-no-reboot",
            "-no-shutdown",
            "-serial", "stdio",
            "-cdrom", str(iso),
            "-boot", "d",
            "-device", "qemu-xhci,id=xhci",
            "-device", "usb-kbd,bus=xhci.0",
            "-drive", f"file={disk},format=raw,if=none,id=eonos_nvme",
            "-device", "nvme,drive=eonos_nvme,serial=EONOSNVME0001",
        ]

        def boot() -> SmokeTest:
            print("Booting headless QEMU...")
            process = subprocess.Popen(
                args,
                cwd=ROOT,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                bufsize=0,
            )
            smoke = SmokeTest(process)
            smoke.expect(b"Mapped, wrote, translated and unmapped", "VMM self-test")
            smoke.expect(b"/init found", "initramfs self-test")
            smoke.expect(b"Dropped to ring 3", "syscall self-test")
            smoke.expect(PROMPT, "initial shell prompt")
            print("PASS boot: VMM, initramfs, syscall self-tests and shell prompt")
            return smoke

        def stop(smoke: SmokeTest) -> None:
            process = smoke.process
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            smoke.selector.close()
            if process.stdin is not None:
                process.stdin.close()
            if process.stdout is not None:
                process.stdout.close()

        print("Booting headless QEMU with the persistent NVMe image...")
        first = boot()
        try:
            first.command("", ())
            first.command("help", (b"commands:", b"lsblk", b"mount", b"panic", b"exit"))
            initial_lsblk = first.command("lsblk", (b"nvme0n1", b"vfat"))
            if b"/disk" in initial_lsblk:
                first.fail("the persistent disk was mounted automatically")
            first.command("lsblk -f", (b"unknown command: lsblk -f",))
            first.command("ls", (b"/init", b"/hello"))
            first.command("cat /etc/motd", (b"Welcome to EonOS.",))
            first.command("exec /hello", (b"hello from /hello",))
            first.command("write /disk/early.txt should-fail", (b"write: failed",))
            first.command("mount /device/nvme0n1 /disk", (b"mount: mounted",))
            first.command("lsblk", (b"nvme0n1", b"/disk"))
            first.command("exec /disk/HELLO.ELF", (b"hello from /hello",))
            first.command("mkdir /disk/smoke")
            first.command("write /disk/smoke/persist.txt survives-reboot", (b"write: saved",))
            first.command("cat /disk/smoke/persist.txt", (b"survives-reboot",))
            first.command("sync", (b"sync: storage reports durable writes",))
        finally:
            stop(first)

        print("Rebuilding while preserving the data image...")
        subprocess.run([str(ROOT / "build.sh")], cwd=ROOT, env=env, check=True)
        print("Rebooting QEMU with the same disk image...")
        second = boot()
        try:
            second.command("mount /device/nvme0n1 /disk", (b"mount: mounted",))
            second.command("lsblk", (b"nvme0n1", b"/disk"))
            second.command("cat /disk/smoke/persist.txt", (b"survives-reboot",))
            second.command("rm /disk/smoke/persist.txt")
            second.command("rm /disk/smoke")
            second.command("sync", (b"sync: storage reports durable writes",))
            second.command("smoke-unknown", (b"unknown command: smoke-unknown",))

            # Panic is terminal: exercise it last and expect QEMU to remain halted.
            assert second.process.stdin is not None
            panic_start = len(second.output)
            second.process.stdin.write(b"panic\n")
            second.process.stdin.flush()
            second.expect(b"kernel panic requested by process", "panic command kernel log", panic_start)
            second.expect(b"KERNEL PANIC -- system halted", "panic halt log", panic_start)
            print("PASS shell: panic enters the kernel panic handler")
        finally:
            stop(second)

    print("QEMU smoke test passed")


if __name__ == "__main__":
    try:
        run()
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f"QEMU smoke test failed: {error}", file=sys.stderr)
        sys.exit(1)
