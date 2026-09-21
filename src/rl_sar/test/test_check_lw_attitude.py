#!/usr/bin/env python3
"""Offline protocol, decision and Linux pseudo-terminal tests; no robot I/O."""

import binascii
import importlib.util
import math
import os
from pathlib import Path
import pty
import select
import signal
import struct
import subprocess
import sys
import tempfile
import termios
import time
import unittest
from unittest import mock


SCRIPT = Path(sys.argv.pop(1)).resolve()
SPEC = importlib.util.spec_from_file_location("check_lw_attitude", SCRIPT)
CHECK = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CHECK)


def payload(stamp, roll=0, pitch=0, yaw=0, scale=1):
    r, p, y = [math.radians(value) / 2 for value in (roll, pitch, yaw)]
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    # Invert the device_type=1 transform to synthesize desired body angles.
    q = (cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
         -(cr*sp*cy + sr*cp*sy), -(cr*cp*sy - sr*sp*cy))
    return struct.pack("<10fq", *([0.0]*6), *(value*scale for value in q), stamp)


def frame(data, kind=0x41):
    header = bytes((0xFC, kind, len(data), 7))
    return (header + bytes((CHECK.crc8(header),))
            + binascii.crc_hqx(data, 0).to_bytes(2, "big") + data + b"\xfd")


class LogicTests(unittest.TestCase):
    def test_independent_crc_vectors(self):
        self.assertEqual(CHECK.crc8(b"123456789"), 0xA1)
        self.assertEqual(binascii.crc_hqx(b"123456789", 0), 0x31C3)

    def test_fragmented_interleaved_frames(self):
        parser = CHECK.FrameParser()
        raw = frame(bytes(56), 0x40) + frame(payload(1)) + frame(b"ignore", 0xF0)
        events = []
        for byte in raw:
            events.extend(parser.feed(bytes((byte,))))
        self.assertEqual([kind for kind, _ in events], [0x40, 0x41, 0xF0])

    def test_corruption_and_resynchronization(self):
        valid = frame(payload(3))
        for position in (1, 2, 4, 5, 15, len(valid)-1):
            with self.subTest(position=position):
                damaged = bytearray(valid)
                damaged[position] ^= 1
                events = list(CHECK.FrameParser().feed(b"noise" + damaged + valid))
                self.assertTrue(any(kind is None for kind, _ in events))
                self.assertEqual(events[-1], (0x41, payload(3)))

    def test_driver_coordinates_and_normalization(self):
        for angles in ((0, 0, 0), (5, -7, 120), (-6, 4, -80)):
            actual, stamp = CHECK.decode_attitude(payload(123, *angles, scale=1.05))
            self.assertEqual(stamp, 123)
            for observed, expected in zip(actual, angles):
                self.assertAlmostEqual(observed, expected, places=4)

    def test_invalid_quaternion_and_nonfinite(self):
        for scale in (0, 0.89, 1.11, math.nan, math.inf):
            with self.subTest(scale=scale), self.assertRaises(ValueError):
                CHECK.decode_attitude(payload(1, scale=scale))

    def stream(self, check, start=0, count=101, **angles):
        for index in range(count):
            now = start + index * .005
            check.accept(payload(round(now*1e6)+1, **angles), now)

    def test_pass_ignores_yaw_and_wait_does_not_advance_stability(self):
        check = CHECK.AttitudeCheck(8, .5, .02)
        self.stream(check, count=100, yaw=160)
        self.assertEqual(check.result(.5)[0], 1)
        check.accept(payload(500001, yaw=160), .5)
        self.assertEqual(check.result(.501)[0], 0)

    def test_final_tilt_or_stale_cannot_use_earlier_pass(self):
        for axis in ("roll", "pitch"):
            check = CHECK.AttitudeCheck(8, .5, .02)
            self.stream(check)
            check.accept(payload(505001, **{axis: 9}), .505)
            self.assertEqual(check.result(.505)[0], 1)
        check = CHECK.AttitudeCheck(8, .5, .02)
        self.stream(check)
        self.assertEqual(check.result(.6)[0], 2)

    def test_fault_gap_and_timestamp_reset_stability(self):
        for fault in ("crc", "gap", "duplicate", "backwards", "invalid"):
            with self.subTest(fault=fault):
                check = CHECK.AttitudeCheck(8, .5, .02)
                self.stream(check)
                if fault == "crc":
                    check.reject("CRC16")
                elif fault == "gap":
                    check.accept(payload(600001), .6)
                else:
                    check.accept(payload(500001 if fault == "duplicate" else 1,
                                         scale=0 if fault == "invalid" else 1), .505)
                self.assertNotEqual(check.result(.6 if fault == "gap" else .505)[0], 0)
                self.stream(check, start=.7)
                self.assertEqual(check.result(1.2)[0], 0)

    def test_batch_and_repeated_timestamp_cannot_fake_duration(self):
        check = CHECK.AttitudeCheck(8, .5, .02)
        for stamp in range(1, 201):
            check.accept(payload(stamp), 1.0)
        self.assertEqual(check.result(1.0)[0], 1)
        check.accept(payload(200), 1.005)
        self.assertEqual(check.result(1.005)[0], 2)

    def test_config_validation(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "base.yaml"
            path.write_text("LW:\n  policy_entry_angle_deg: 8\n  policy_entry_stable_time: 0.5\n  trusted_imu_timeout: 0.02\n")
            self.assertEqual(CHECK.load_limits(path), (8, .5, .02))
            original = path.read_text()
            for bad in ("true", ".nan", "0", "-1", "90", "'8'"):
                path.write_text(original.replace("angle_deg: 8", f"angle_deg: {bad}"))
                with self.assertRaises(ValueError):
                    CHECK.load_limits(path)
            path.write_text("LW: {}")
            with self.assertRaises(ValueError):
                CHECK.load_limits(path)


class SerialTests(unittest.TestCase):
    def setUp(self):
        self.master, slave = pty.openpty()
        self.port = os.ttyname(slave)
        self.original = termios.tcgetattr(slave)
        os.close(slave)
        self.temporary = tempfile.TemporaryDirectory()
        self.config = Path(self.temporary.name) / "base.yaml"
        self.config.write_text("LW:\n  policy_entry_angle_deg: 8\n  policy_entry_stable_time: 0.08\n  trusted_imu_timeout: 0.1\n")
        self.process = None

    def tearDown(self):
        if self.process is not None and self.process.poll() is None:
            self.process.kill()
            self.process.communicate()
        os.close(self.master)
        self.temporary.cleanup()

    def launch(self):
        self.process = subprocess.Popen(
            [sys.executable, str(SCRIPT), "--config", str(self.config), "--port", self.port, "--duration", ".35"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
        )
        deadline = time.monotonic() + 5
        while termios.tcgetattr(self.master) == self.original:
            if self.process.poll() is not None:
                self.fail(str(self.process.communicate()))
            if time.monotonic() > deadline:
                self.fail("serial setup timed out")
            time.sleep(.005)

    def assert_restored(self):
        self.assertEqual(termios.tcgetattr(self.master), self.original)
        fd = os.open(self.port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
        os.close(fd)  # Also verifies exclusive mode has been released.

    def run_stream(self, roll=0):
        self.launch()
        start = time.monotonic()
        stamp = 0
        wrote_back = False
        while self.process.poll() is None and time.monotonic() - start < 4:
            # Do not inject bytes after restored canonical/echo settings.
            if termios.tcgetattr(self.master) != self.original:
                stamp += 5000
                os.write(self.master, frame(payload(stamp, roll=roll)))
                if select.select([self.master], [], [], 0)[0]:
                    try:
                        wrote_back |= bool(os.read(self.master, 4096))
                    except OSError:
                        pass
            time.sleep(.005)
        output, error = self.process.communicate(timeout=5)
        self.assertFalse(wrote_back, "checker wrote data to the serial peer")
        self.assert_restored()
        return self.process.returncode, output, error

    def test_pass_and_no_device_writes(self):
        code, output, error = self.run_stream()
        self.assertEqual(code, 0, output + error)
        self.assertIn("PASS:", output)

    def test_tilt_is_exit_one(self):
        code, output, error = self.run_stream(12)
        self.assertEqual(code, 1, output + error)

    def test_silent_device_is_exit_two(self):
        self.launch()
        output, error = self.process.communicate(timeout=5)
        self.assertEqual(self.process.returncode, 2, output + error)
        self.assert_restored()

    def test_busy_port_rejected_without_setting_changes(self):
        fd = os.open(self.port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            with self.assertRaisesRegex(RuntimeError, "占用"):
                with CHECK.passive_serial(self.port):
                    self.fail("busy port accepted")
            self.assertEqual(termios.tcgetattr(fd), self.original)
        finally:
            os.close(fd)

    def test_exception_restores_settings(self):
        with self.assertRaisesRegex(RuntimeError, "injected"):
            with CHECK.passive_serial(self.port):
                raise RuntimeError("injected")
        self.assert_restored()

    def test_restore_failure_is_error_and_fd_is_closed(self):
        real_setattr = termios.tcsetattr
        def fail_restore(fd, mode, settings):
            if settings == self.original:
                raise OSError("restore failure")
            real_setattr(fd, mode, settings)
        with mock.patch.object(CHECK.termios, "tcsetattr", side_effect=fail_restore):
            with self.assertRaisesRegex(OSError, "restore failure"):
                with CHECK.passive_serial(self.port) as fd:
                    owned_fd = fd
        with self.assertRaises(OSError):
            os.fstat(owned_fd)
        real_setattr(self.master, termios.TCSANOW, self.original)
        self.assert_restored()

    def test_sigint_and_sigterm_restore_settings(self):
        for sig in (signal.SIGINT, signal.SIGTERM):
            with self.subTest(signal=sig):
                self.launch()
                self.process.send_signal(sig)
                output, error = self.process.communicate(timeout=5)
                self.assertEqual(self.process.returncode, 130, output + error)
                self.assert_restored()


if __name__ == "__main__":
    unittest.main()
