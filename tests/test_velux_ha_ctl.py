#!/usr/bin/env python3
"""Tests for homeassistant/velux_ha_ctl.py (standard library only).

Run from the repo root:  python3 -m unittest discover -s tests -v

SPDX-License-Identifier: Apache-2.0
"""
import contextlib
import io
import json
import os
import subprocess
import sys
import threading
import time
import unittest
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "homeassistant"))

import velux_ha_ctl  # noqa: E402
from fake_serial_device import FakeDevice  # noqa: E402


def run_ctl(port, *args):
    """Run velux_ha_ctl.main() with the given CLI args; return (rc, stdout lines)."""
    out = io.StringIO()
    with mock.patch.object(sys, "argv", ["velux_ha_ctl.py", *args]), \
         mock.patch.dict(os.environ, {"VELUX_PORT": port}), \
         contextlib.redirect_stdout(out):
        rc = velux_ha_ctl.main()
    return rc, [l for l in out.getvalue().splitlines() if l.strip()]


class CtlTestBase(unittest.TestCase):
    pair_succeeds = True

    def setUp(self):
        self.dev = FakeDevice(pair_succeeds=self.pair_succeeds)
        self.dev.start()

    def tearDown(self):
        self.dev.close()


class TestCommands(CtlTestBase):
    def test_default_is_status_for_both_devices(self):
        rc, lines = run_ctl(self.dev.port)
        self.assertEqual(rc, 0)
        self.assertEqual(self.dev.commands, ["velux status"])
        events = [json.loads(l) for l in lines]
        self.assertEqual([e["device"] for e in events], ["blind", "skylight"])

    def test_single_device_status_is_one_json_line(self):
        # Home Assistant's command_line sensor parses this with value_json
        rc, lines = run_ctl(self.dev.port, "blind", "status")
        self.assertEqual(rc, 0)
        self.assertEqual(len(lines), 1)
        self.assertEqual(json.loads(lines[0])["position"], 42)

    def test_commands_are_forwarded_verbatim(self):
        cases = [
            (["blind", "open"], "velux blind open"),
            (["skylight", "close"], "velux skylight close"),
            (["blind", "stop"], "velux blind stop"),
            (["skylight", "pos", "35"], "velux skylight pos 35"),
            (["set_transfer_key", "00" * 16], "velux set_transfer_key " + "00" * 16),
        ]
        for args, expected in cases:
            with self.subTest(args=args):
                self.dev.commands.clear()
                rc, _ = run_ctl(self.dev.port, *args)
                self.assertEqual(rc, 0)
                self.assertEqual(self.dev.commands, [expected])

    def test_only_json_lines_are_printed(self):
        # Firmware debug lines and the echoed command must not reach Home Assistant
        rc, lines = run_ctl(self.dev.port, "blind", "close")
        self.assertEqual(rc, 0)
        self.assertEqual(len(lines), 1)
        event = json.loads(lines[0])
        self.assertEqual((event["event"], event["action"]), ("velux_state", "close"))

    def test_port_argument_overrides_env(self):
        out = io.StringIO()
        with mock.patch.object(sys, "argv", ["x", "--port=" + self.dev.port, "status"]), \
             mock.patch.dict(os.environ, {"VELUX_PORT": "/dev/does-not-exist"}), \
             contextlib.redirect_stdout(out):
            rc = velux_ha_ctl.main()
        self.assertEqual(rc, 0)
        self.assertEqual(self.dev.commands, ["velux status"])


class TestPairing(CtlTestBase):
    def test_pair_waits_for_paired_event(self):
        rc, lines = run_ctl(self.dev.port, "pair", "blind")
        self.assertEqual(rc, 0)
        self.assertEqual(self.dev.commands, ["velux pair blind"])
        self.assertTrue(any('"velux_paired"' in l for l in lines))
        self.assertTrue(any("blind remote" in l for l in lines))


class TestPairingFastPress(unittest.TestCase):
    def test_paired_event_inside_first_reply_window(self):
        # Button pressed almost immediately: the paired event arrives while the
        # script is still reading the reply to the pair command itself
        dev = FakeDevice(pair_delay=0.0)
        dev.start()
        try:
            real_time = time.time
            start = real_time()
            # Speed up the 120 s window so a missed event fails fast, not hangs
            with mock.patch.object(velux_ha_ctl.time, "time",
                                   side_effect=lambda: start + (real_time() - start) * 150):
                rc, lines = run_ctl(dev.port, "pair", "skylight")
            self.assertEqual(rc, 0)
            self.assertTrue(any('"velux_paired"' in l for l in lines))
        finally:
            dev.close()


class TestPairingTimeout(CtlTestBase):
    pair_succeeds = False

    def test_pair_timeout_sends_stop_pair(self):
        real_time = time.time
        start = real_time()
        # Make the 120 s pairing window elapse in ~1 s
        with mock.patch.object(velux_ha_ctl.time, "time",
                               side_effect=lambda: start + (real_time() - start) * 150):
            rc, lines = run_ctl(self.dev.port, "pair", "skylight")
        self.assertEqual(rc, 1)
        self.assertIn("velux stop_pair", self.dev.commands)
        self.assertIn("timed out", lines[-1])


class TestLocking(unittest.TestCase):
    def test_concurrent_processes_do_not_interleave(self):
        # Home Assistant runs each command as its own process. With a slow
        # device, both would write before the first reply arrives unless the
        # script holds the port lock for the whole exchange.
        dev = FakeDevice(reply_delay=0.4)
        dev.start()
        script = os.path.join(HERE, "..", "homeassistant", "velux_ha_ctl.py")
        env = dict(os.environ, VELUX_PORT=dev.port)
        try:
            p1 = subprocess.Popen([sys.executable, script, "blind", "open"],
                                  stdout=subprocess.PIPE, text=True, env=env)
            time.sleep(0.15)
            p2 = subprocess.Popen([sys.executable, script, "skylight", "close"],
                                  stdout=subprocess.PIPE, text=True, env=env)
            out1, _ = p1.communicate(timeout=15)
            out2, _ = p2.communicate(timeout=15)
            self.assertEqual((p1.returncode, p2.returncode), (0, 0))
            self.assertFalse(dev.overlap, "second process wrote while the first was waiting for its reply")
            self.assertEqual(dev.commands, ["velux blind open", "velux skylight close"])
            self.assertEqual(json.loads(out1.strip().splitlines()[-1])["device"], "blind")
            self.assertEqual(json.loads(out2.strip().splitlines()[-1])["device"], "skylight")
        finally:
            dev.close()


class TestPortDiscovery(unittest.TestCase):
    def test_prefers_serial_by_id(self):
        paths = {
            "/dev/serial/by-id/*Seeed*": ["/dev/serial/by-id/usb-Seeed_XIAO_MG24-if00"],
            "/dev/ttyACM*": ["/dev/ttyACM0"],
        }
        with mock.patch.object(velux_ha_ctl.glob, "glob", side_effect=lambda p: paths.get(p, [])), \
             mock.patch.object(velux_ha_ctl.os.path, "exists", return_value=True):
            self.assertEqual(velux_ha_ctl.find_serial_port(),
                             "/dev/serial/by-id/usb-Seeed_XIAO_MG24-if00")

    def test_falls_back_to_ttyacm(self):
        paths = {"/dev/ttyACM*": ["/dev/ttyACM1"]}
        with mock.patch.object(velux_ha_ctl.glob, "glob", side_effect=lambda p: paths.get(p, [])), \
             mock.patch.object(velux_ha_ctl.os.path, "exists", return_value=True):
            self.assertEqual(velux_ha_ctl.find_serial_port(), "/dev/ttyACM1")

    def test_no_port_raises(self):
        with mock.patch.object(velux_ha_ctl.glob, "glob", return_value=[]):
            with self.assertRaises(RuntimeError):
                velux_ha_ctl.find_serial_port()


if __name__ == "__main__":
    unittest.main()
