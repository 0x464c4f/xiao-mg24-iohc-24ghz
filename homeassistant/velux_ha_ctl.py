#!/usr/bin/env python3
"""Zero-dependency CLI & Home Assistant helper for Seeed Studio XIAO MG24 2.4 GHz Controller.

SPDX-License-Identifier: Apache-2.0

Talks to the on-chip `velux` firmware command over USB Serial (115200 8N1).
All AES-128 crypto, 1-Way MAC computation, key pairing, and Flash NVM sequence
counters run directly on the XIAO MG24 chip!

Usage:
  python3 velux_ha_ctl.py status
  python3 velux_ha_ctl.py set_transfer_key <32-hex-chars>
  python3 velux_ha_ctl.py pair blind
  python3 velux_ha_ctl.py pair skylight
  python3 velux_ha_ctl.py blind open
  python3 velux_ha_ctl.py blind close
  python3 velux_ha_ctl.py blind stop
  python3 velux_ha_ctl.py blind pos 50
  python3 velux_ha_ctl.py skylight open
  python3 velux_ha_ctl.py skylight close
  python3 velux_ha_ctl.py skylight stop
"""

import os
import sys
import time
import glob
import fcntl
import select
import termios


def find_serial_port():
    # Linux / Home Assistant OS persistent USB ID paths first
    by_id = (
        sorted(glob.glob("/dev/serial/by-id/*Seeed*"))
        + sorted(glob.glob("/dev/serial/by-id/*XIAO*"))
        + sorted(glob.glob("/dev/serial/by-id/*CMSIS-DAP*"))
        + sorted(glob.glob("/dev/serial/by-id/*"))
    )
    for p in by_id:
        if os.path.exists(p):
            return p
    # Fallback to macOS (/dev/cu.usbmodem*) and Linux (/dev/ttyACM*)
    candidates = sorted(glob.glob("/dev/cu.usbmodem*")) + sorted(glob.glob("/dev/ttyACM*"))
    for p in candidates:
        if os.path.exists(p):
            return p
    raise RuntimeError("No XIAO MG24 serial port found (/dev/serial/by-id/*, /dev/ttyACM*, or /dev/cu.usbmodem*)")


def open_serial(port):
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    # Serialize access: HA may poll status while a command is running
    fcntl.flock(fd, fcntl.LOCK_EX)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0
    attrs[4] = termios.B115200
    attrs[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def send_velux_cmd(fd, cli_line, timeout=6.0):
    os.write(fd, b"\r\n")
    time.sleep(0.03)
    while True:
        r, _, _ = select.select([fd], [], [], 0.02)
        if not r:
            break
        try:
            if not os.read(fd, 4096):
                break
        except BlockingIOError:
            break

    os.write(fd, (cli_line.strip() + "\r\n").encode())
    out = b""
    deadline = time.time() + timeout
    while time.time() < deadline:
        r, _, _ = select.select([fd], [], [], 0.05)
        if r:
            try:
                chunk = os.read(fd, 4096)
                out += chunk
                if b'"event":"velux_' in out and out.rstrip().endswith(b">"):
                    break
            except BlockingIOError:
                pass
    return out.decode(errors="replace")


def main():
    port = os.environ.get("VELUX_PORT") or find_serial_port()
    args = [a for a in sys.argv[1:] if not a.startswith("--port=")]
    for a in sys.argv[1:]:
        if a.startswith("--port="):
            port = a.split("=", 1)[1]

    if not args:
        args = ["status"]

    cli_cmd = "velux " + " ".join(args)
    fd = open_serial(port)
    try:
        if args[0] == "pair":
            target = args[1] if len(args) > 1 else "skylight"
            out = send_velux_cmd(fd, cli_cmd, timeout=1.0)
            for line in out.splitlines():
                if line.strip().startswith("{"):
                    print(line.strip(), flush=True)
                    if '"velux_error"' in line:
                        return 1
            print(f"Waiting for rear PAIR/RESET button press on {target} remote...", flush=True)
            deadline = time.time() + 120.0
            buf = ""
            while time.time() < deadline:
                r, _, _ = select.select([fd], [], [], 0.1)
                if r:
                    try:
                        chunk = os.read(fd, 4096).decode(errors="replace")
                        buf += chunk
                        while "\n" in buf:
                            line, buf = buf.split("\n", 1)
                            line_s = line.strip()
                            if ("[CRC-OK] PHR=22" in line_s or "EXHAUSTIVE" in line_s or
                                "NEW COMMAND" in line_s or line_s.startswith("{")):
                                print(line_s, flush=True)
                            if line_s.startswith("{") and '"velux_paired"' in line_s:
                                return 0
                    except BlockingIOError:
                        pass
            send_velux_cmd(fd, "velux stop_pair", timeout=1.0)
            print('{"event":"velux_error","error":"Pairing timed out after 120s"}')
            return 1
        else:
            out = send_velux_cmd(fd, cli_cmd, timeout=6.0)
            for line in out.splitlines():
                line_s = line.strip()
                if line_s.startswith("{"):
                    print(line_s)
            return 0
    finally:
        os.close(fd)


if __name__ == "__main__":
    sys.exit(main())
