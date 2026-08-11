#!/usr/bin/env python3

from pathlib import Path
import os
import struct
import subprocess
import sys
import tempfile
import unittest


VALIDATOR = Path(sys.argv.pop(1)).resolve()


def elf_header(machine: int) -> bytes:
    identification = bytearray(16)
    identification[0:4] = b"\x7fELF"
    identification[4] = 2  # ELFCLASS64
    identification[5] = 1  # ELFDATA2LSB
    identification[6] = 1  # EV_CURRENT
    return struct.pack(
        "<16sHHIQQQIHHHHHH",
        bytes(identification),
        3,  # ET_DYN
        machine,
        1,
        0,
        0,
        0,
        0,
        64,
        0,
        0,
        0,
        0,
        0,
    )


class InferenceRuntimeArchitectureTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory(
            prefix="lw-inference-architecture-test-"
        )
        self.root = Path(self.temporary.name)

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def make_onnx_runtime(self, machine: int = 62) -> Path:
        runtime = self.root / "onnxruntime"
        (runtime / "include").mkdir(parents=True)
        (runtime / "lib").mkdir()
        (runtime / "include/onnxruntime_cxx_api.h").write_text(
            "// fixture\n", encoding="utf-8"
        )
        (runtime / "lib/libonnxruntime.so.1").write_bytes(elf_header(machine))
        (runtime / "lib/libonnxruntime.so").symlink_to("libonnxruntime.so.1")
        return runtime

    def make_libtorch_runtime(self, machine: int = 62) -> Path:
        runtime = self.root / "libtorch"
        (runtime / "include/torch/csrc/api/include/torch").mkdir(parents=True)
        (runtime / "lib").mkdir()
        (runtime / "include/torch/csrc/api/include/torch/torch.h").write_text(
            "// fixture\n", encoding="utf-8"
        )
        (runtime / "lib/libtorch_cpu.so").write_bytes(elf_header(machine))
        return runtime

    def validate(
        self,
        kind: str,
        runtime: Path,
        architecture: str,
        os_name: str = "Linux",
    ) -> subprocess.CompletedProcess[str]:
        environment = os.environ.copy()
        environment["RL_SAR_RUNTIME_VALIDATION_OS"] = os_name
        return subprocess.run(
            ["bash", str(VALIDATOR), kind, str(runtime), architecture],
            check=False,
            text=True,
            capture_output=True,
            env=environment,
        )

    def test_matching_x86_64_onnx_is_accepted(self) -> None:
        result = self.validate("onnx", self.make_onnx_runtime(62), "x86_64")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("valid for x86_64", result.stdout)

    def test_matching_aarch64_onnx_is_accepted(self) -> None:
        result = self.validate("onnx", self.make_onnx_runtime(183), "aarch64")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("valid for aarch64", result.stdout)

    def test_x86_64_onnx_is_rejected_on_aarch64(self) -> None:
        result = self.validate("onnx", self.make_onnx_runtime(62), "aarch64")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("architecture mismatch", result.stderr)

    def test_aarch64_onnx_is_rejected_on_x86_64(self) -> None:
        result = self.validate("onnx", self.make_onnx_runtime(183), "x86_64")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("architecture mismatch", result.stderr)

    def test_matching_libtorch_is_accepted_for_development(self) -> None:
        result = self.validate(
            "libtorch", self.make_libtorch_runtime(62), "x86_64"
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_corrupt_elf_is_rejected(self) -> None:
        runtime = self.make_onnx_runtime()
        (runtime / "lib/libonnxruntime.so.1").write_bytes(b"not an elf")
        result = self.validate("onnx", runtime, "x86_64")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Cannot read ELF machine type", result.stderr)

    def test_missing_header_is_rejected(self) -> None:
        runtime = self.make_onnx_runtime()
        (runtime / "include/onnxruntime_cxx_api.h").unlink()
        result = self.validate("onnx", runtime, "x86_64")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("header is missing", result.stderr)

    def test_broken_library_symlink_is_rejected(self) -> None:
        runtime = self.make_onnx_runtime()
        (runtime / "lib/libonnxruntime.so").unlink()
        (runtime / "lib/libonnxruntime.so.1").unlink()
        (runtime / "lib/libonnxruntime.so").symlink_to("missing.so")
        result = self.validate("onnx", runtime, "x86_64")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("shared library is missing", result.stderr)

    def test_non_linux_host_retains_structural_validation(self) -> None:
        runtime = self.root / "onnxruntime-macos"
        (runtime / "include").mkdir(parents=True)
        (runtime / "lib").mkdir()
        (runtime / "include/onnxruntime_cxx_api.h").write_text(
            "// fixture\n", encoding="utf-8"
        )
        result = self.validate("onnx", runtime, "arm64", "Darwin")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("valid for Darwin", result.stdout)


if __name__ == "__main__":
    unittest.main()
