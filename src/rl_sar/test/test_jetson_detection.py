#!/usr/bin/env python3

import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


DETECT_SCRIPT = Path(sys.argv.pop(1)).resolve()


class JetsonDetectionTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory(
            prefix="lw-jetson-detection-test-"
        )
        self.platform_root = Path(self.temporary.name)

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def detect(
        self,
        *,
        os_name: str = "Linux",
        architecture: str = "aarch64",
        override: str | None = None,
    ) -> subprocess.CompletedProcess[str]:
        environment = os.environ.copy()
        for name in ("IS_JETSON", "JETSON_DETECTION_SOURCE"):
            environment.pop(name, None)
        environment.update(
            {
                "RL_SAR_PLATFORM_OS": os_name,
                "RL_SAR_PLATFORM_ARCH": architecture,
                "RL_SAR_PLATFORM_ROOT": str(self.platform_root),
            }
        )
        if override is not None:
            environment["IS_JETSON"] = override
        return subprocess.run(
            ["bash", str(DETECT_SCRIPT)],
            check=False,
            text=True,
            capture_output=True,
            env=environment,
        )

    def test_x86_is_not_jetson(self) -> None:
        result = self.detect(architecture="x86_64")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(result.stdout.strip(), "false;non-aarch64-platform")

    def test_unidentified_aarch64_is_not_assumed_jetson(self) -> None:
        result = self.detect()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(result.stdout.strip(), "false;unidentified-aarch64")

    def test_nv_tegra_release_detects_jetson(self) -> None:
        marker = self.platform_root / "etc" / "nv_tegra_release"
        marker.parent.mkdir(parents=True)
        marker.write_text("# R36\n", encoding="utf-8")
        result = self.detect()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(result.stdout.strip(), "true;nv-tegra-release")

    def test_tegra_library_detects_jetson(self) -> None:
        (self.platform_root / "usr/lib/aarch64-linux-gnu/tegra").mkdir(
            parents=True
        )
        result = self.detect()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(result.stdout.strip(), "true;tegra-library")

    def test_jetson_cuda_target_detects_jetson(self) -> None:
        (self.platform_root / "usr/local/cuda-12.6/targets/aarch64-linux").mkdir(
            parents=True
        )
        result = self.detect()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(result.stdout.strip(), "true;jetson-cuda-target")

    def test_false_override_wins_over_hardware_marker(self) -> None:
        marker = self.platform_root / "etc" / "nv_tegra_release"
        marker.parent.mkdir(parents=True)
        marker.write_text("# R36\n", encoding="utf-8")
        result = self.detect(override="false")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(result.stdout.strip(), "false;explicit-override")

    def test_true_override_requires_native_aarch64(self) -> None:
        result = self.detect(architecture="x86_64", override="true")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("requires a native Linux aarch64 host", result.stdout)

    def test_invalid_override_fails(self) -> None:
        result = self.detect(override="maybe")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("must be true, false", result.stdout)


if __name__ == "__main__":
    unittest.main()
