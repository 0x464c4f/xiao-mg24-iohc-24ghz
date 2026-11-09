"""Simulated XIAO MG24 on a pseudo-terminal, for testing velux_ha_ctl.py.

SPDX-License-Identifier: Apache-2.0
"""
import json
import os
import pty
import select
import threading
import time
import tty


def device_json(event, device, action, position=0):
    return json.dumps({
        "event": event, "device": device, "addr": "aabbcc", "action": action,
        "position": position, "seq": 7, "seq_hex": "0x0007", "paired": True,
        "transfer_key_set": True, "key": "00",
    }, separators=(",", ":"))


class FakeDevice(threading.Thread):
    """Plays the firmware side of the serial link.

    Echoes each command line (like the RAILtest CLI), prints JSON replies and
    the '> ' prompt. Records every command it receives in `self.commands`.
    """

    def __init__(self, pair_succeeds=True, reply_delay=0.0, pair_delay=0.3):
        super().__init__(daemon=True)
        self.master, self._slave = pty.openpty()
        tty.setraw(self._slave)
        self.port = os.ttyname(self._slave)
        self.commands = []
        self.pair_succeeds = pair_succeeds
        self.reply_delay = reply_delay
        self.pair_delay = pair_delay
        self.overlap = False   # set if a client writes while a reply is pending
        self._stop = threading.Event()

    def close(self):
        self._stop.set()
        self.join(2)
        for fd in (self.master, self._slave):
            try:
                os.close(fd)
            except OSError:
                pass

    def send(self, text):
        os.write(self.master, text.replace("\n", "\r\n").encode())

    def reply_for(self, line):
        args = line.split()[1:]
        if not args or args[0] == "status":
            return (device_json("velux_status", "blind", "status", 0) + "\n"
                    + device_json("velux_status", "skylight", "status", 100) + "\n")
        if args[0] == "pair":
            dev = args[1] if len(args) > 1 else "skylight"
            if self.pair_succeeds:
                threading.Timer(self.pair_delay, self.send, args=(
                    "[HOP] +  10.0 ms | [CRC-OK] PHR=22 PSDU: 01 69 6f fc\n"
                    + device_json("velux_paired", dev, "paired") + "\n",)).start()
            return json.dumps({"event": "velux_pairing_armed", "device": dev}, separators=(",", ":")) + "\n"
        if args[0] == "stop_pair":
            return '{"event":"velux_pairing_stopped"}\n'
        if args[0] == "set_transfer_key":
            return '{"event":"velux_transfer_key","transfer_key_set":true}\n'
        dev = args[0]
        action = args[1] if len(args) > 1 else "status"
        if action == "status":
            return device_json("velux_status", dev, "status", 42) + "\n"
        return ("[VELUX-TX] LPM x3 PSDU (28B): 01 69 6f f6\n"
                + device_json("velux_state", dev, action) + "\n")

    def run(self):
        buf = b""
        while not self._stop.is_set():
            ready, _, _ = select.select([self.master], [], [], 0.05)
            if not ready:
                continue
            try:
                chunk = os.read(self.master, 1024)
            except OSError:
                return
            if not chunk:
                return
            buf += chunk
            while b"\n" in buf or b"\r" in buf:
                idx = min(i for i in (buf.find(b"\r"), buf.find(b"\n")) if i >= 0)
                line, buf = buf[:idx].decode(errors="replace").strip(), buf[idx + 1:]
                if not line:
                    self.send("\n> ")
                    continue
                self.commands.append(line)
                if self.reply_delay:
                    end = time.monotonic() + self.reply_delay
                    while not self.overlap and time.monotonic() < end:
                        ready, _, _ = select.select([self.master], [], [], end - time.monotonic())
                        if ready:
                            self.overlap = True
                    time.sleep(max(0.0, end - time.monotonic()))
                self.send(line + "\n" + self.reply_for(line) + "> ")
