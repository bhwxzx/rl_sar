#!/usr/bin/env python3

import os
from pathlib import Path
import pty
import signal
import subprocess
import sys
import time
import unittest


EXECUTABLE = Path(sys.argv.pop(1)).resolve()


class FDILinkProcessLifecycleTests(unittest.TestCase):
    def run_driver(self, port: str) -> subprocess.Popen[str]:
        return subprocess.Popen(
            [
                str(EXECUTABLE),
                "--ros-args",
                "-p",
                f"serial_port_:={port}",
            ],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

    def test_missing_serial_port_returns_failure(self) -> None:
        started = time.monotonic()
        completed = subprocess.run(
            [
                str(EXECUTABLE),
                "--ros-args",
                "-p",
                "serial_port_:=/definitely/missing/fdilink-device",
            ],
            check=False,
            text=True,
            capture_output=True,
            timeout=3,
        )
        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("FDILink AHRS driver failed", completed.stderr)
        self.assertLess(time.monotonic() - started, 2.0)

    def test_sigterm_during_empty_serial_read_exits_successfully(self) -> None:
        master_fd, slave_fd = pty.openpty()
        process: subprocess.Popen[str] | None = None
        try:
            slave_path = os.ttyname(slave_fd)
            process = self.run_driver(slave_path)
            time.sleep(0.3)
            self.assertIsNone(process.poll())

            started = time.monotonic()
            process.send_signal(signal.SIGTERM)
            stdout, stderr = process.communicate(timeout=2)
            self.assertEqual(process.returncode, 0, stdout + stderr)
            self.assertLess(time.monotonic() - started, 1.0)
        finally:
            if process is not None and process.poll() is None:
                process.kill()
                process.wait(timeout=2)
            os.close(master_fd)
            os.close(slave_fd)


if __name__ == "__main__":
    unittest.main()
