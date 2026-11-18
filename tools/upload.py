"""Compile and upload the firmware to an Arduino UNO R4 Minima.

    python tools/upload.py [--port COM5] [--cli path/to/arduino-cli]

On Windows the R4's automatic jump to the bootloader often fails ("Lost device after
RESET?", exit status 74). This script compiles, tries a 1200-baud "touch" to enter the
bootloader, and if that doesn't work asks you to double-tap RESET, then waits and uploads.
"""
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import serial
from serial.tools import list_ports

FQBN = "arduino:renesas_uno:minima"
SKETCH = Path(__file__).resolve().parent.parent / "dyneema_rig"
ARDUINO_VID = 0x2341
BOOTLOADER_PID = 0x0369


def find_cli(explicit: str | None) -> str:
    if explicit:
        return explicit
    on_path = shutil.which("arduino-cli")
    if on_path:
        return on_path
    ide = Path(os.environ.get("LOCALAPPDATA", "")) / "Programs/arduino-ide/resources/app/lib/backend/resources/arduino-cli.exe"
    if ide.exists():
        return str(ide)
    sys.exit("arduino-cli not found: install it, or pass --cli")


def find_port(explicit: str | None) -> str | None:
    if explicit:
        return explicit
    for p in list_ports.comports():
        if p.vid == ARDUINO_VID and p.pid != BOOTLOADER_PID:
            return p.device
    return None


def in_bootloader() -> bool:
    """The R4 bootloader is a DFU device (no COM port), so ask Windows / pyserial."""
    if any(p.vid == ARDUINO_VID and p.pid == BOOTLOADER_PID for p in list_ports.comports()):
        return True
    if os.name == "nt":
        ps = ("if (Get-PnpDevice -PresentOnly -EA SilentlyContinue | "
              "? { $_.InstanceId -like '*VID_2341&PID_0369*' }) {exit 0} else {exit 1}")
        return subprocess.run(["powershell", "-NoProfile", "-Command", ps]).returncode == 0
    return False


def touch(port: str) -> None:
    try:
        s = serial.Serial(port, 1200)
        s.dtr = False
        time.sleep(0.05)
        s.close()
    except (serial.SerialException, OSError):
        pass


def wait_for(cond, seconds: float) -> bool:
    end = time.time() + seconds
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.5)
    return False


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="board COM port (default: auto-detect)")
    ap.add_argument("--cli", help="path to arduino-cli (default: PATH or Arduino IDE 2 install)")
    args = ap.parse_args()
    cli = find_cli(args.cli)

    print("compiling…", flush=True)
    r = subprocess.run([cli, "compile", "--fqbn", FQBN, str(SKETCH)], capture_output=True, text=True)
    if r.returncode != 0:
        sys.exit(r.stdout + r.stderr)
    print([l for l in r.stdout.splitlines() if l.startswith("Sketch uses")][0])

    port = find_port(args.port)
    if not in_bootloader():
        if port:
            print(f"requesting bootloader on {port}…", flush=True)
            for _ in range(3):
                touch(port)
                if wait_for(in_bootloader, 3):
                    break
        if not in_bootloader():
            print(">>> Double-tap the RESET button on the board now (LED starts pulsing)…", flush=True)
            if not wait_for(in_bootloader, 120):
                sys.exit("bootloader not detected; check the USB cable and try again")

    print("bootloader detected, uploading…", flush=True)
    for attempt in range(3):
        cmd = [cli, "upload", "--fqbn", FQBN, str(SKETCH)] + (["-p", port] if port else [])
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode == 0:
            print("upload complete")
            return
        msg = [l for l in (r.stdout + r.stderr).replace("\r", "\n").splitlines()
               if l.strip() and not l.startswith("Download")]
        print("upload attempt failed:", msg[-1] if msg else r.returncode, flush=True)
        if not in_bootloader():
            print(">>> Double-tap RESET again…", flush=True)
            wait_for(in_bootloader, 120)
    sys.exit("upload failed")


if __name__ == "__main__":
    main()
