#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest


GENERATOR_PATH = Path(sys.argv.pop(1)).resolve()
SPEC = importlib.util.spec_from_file_location(
    "generate_lw_deployment_manifest", GENERATOR_PATH
)
GENERATOR = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(GENERATOR)


def elf64_bytes(machine: int) -> bytes:
    header = bytearray(20)
    header[:4] = b"\x7fELF"
    header[4] = 2
    header[5] = 1
    header[6] = 1
    header[18:20] = machine.to_bytes(2, "little")
    return bytes(header) + b"test ELF payload\n"


class ManifestGeneratorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory(
            prefix="lw-manifest-generator-test-"
        )
        self.prefix = Path(self.temporary.name) / "install"
        self.bundle = (
            self.prefix / "share" / "rl_sar" / "deployment" / "LW"
        )
        self.executable = self.prefix / "lib" / "rl_sar" / "rl_real_LW"
        self.executable.parent.mkdir(parents=True)
        self.executable.write_bytes(b"test executable\n")
        for relative in GENERATOR.POLICY_FILES:
            asset = self.bundle / relative
            asset.parent.mkdir(parents=True, exist_ok=True)
            asset.write_bytes(f"asset:{relative}\n".encode())
        for relative in GENERATOR.RUNTIME_FILES:
            asset = self.prefix / relative
            asset.parent.mkdir(parents=True, exist_ok=True)
            asset.write_bytes(f"runtime:{relative}\n".encode())
        for relative in GENERATOR.ONNX_RUNTIME_FILES:
            library = self.prefix / relative
            library.parent.mkdir(parents=True, exist_ok=True)
            library.write_bytes(elf64_bytes(62))

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def generate(
        self,
        commit: str = "a" * 40,
        build_type: str = "Release",
        version: str = "1.22.0",
        architecture: str = "x86_64",
    ) -> Path:
        return GENERATOR.generate_manifest(
            self.prefix,
            commit,
            build_type,
            version,
            architecture,
        )

    def test_generates_complete_manifest(self) -> None:
        commit = "a" * 40
        manifest = self.generate(commit=commit)
        content = manifest.read_text(encoding="utf-8")
        self.assertIn("schema_version: 3", content)
        self.assertIn(f'source_commit: "{commit}"', content)
        self.assertIn('name: "rl_real_LW"', content)
        self.assertIn("onnx_runtime:", content)
        self.assertIn('  version: "1.22.0"', content)
        self.assertIn('  architecture: "x86_64"', content)
        self.assertIn("runtime_files:", content)
        self.assertEqual(
            sum(line.startswith("  - path:") for line in content.splitlines()),
            len(GENERATOR.POLICY_FILES) + len(GENERATOR.RUNTIME_FILES),
        )
        self.assertEqual(
            sum(line.startswith("    - path:") for line in content.splitlines()),
            len(GENERATOR.ONNX_RUNTIME_FILES),
        )

    def test_rejects_untracked_style_symlink_asset(self) -> None:
        relative = Path(GENERATOR.POLICY_FILES[0])
        asset = self.bundle / relative
        external = Path(self.temporary.name) / "external.yaml"
        external.write_text("external\n", encoding="utf-8")
        asset.unlink()
        asset.symlink_to(external)
        with self.assertRaisesRegex(RuntimeError, "symbolic link"):
            self.generate()

    def test_rejects_symlink_installed_executable(self) -> None:
        external = Path(self.temporary.name) / "external-rl_real_LW"
        self.executable.rename(external)
        self.executable.symlink_to(external)
        with self.assertRaisesRegex(RuntimeError, "installed executable"):
            self.generate()

    def test_rejects_missing_runtime_dependency(self) -> None:
        runtime = self.prefix / GENERATOR.RUNTIME_FILES[0]
        runtime.unlink()
        with self.assertRaisesRegex(RuntimeError, "runtime dependency"):
            self.generate()

    def test_rejects_symlink_runtime_dependency(self) -> None:
        runtime = self.prefix / GENERATOR.RUNTIME_FILES[0]
        external = Path(self.temporary.name) / "external-runtime"
        external.write_text("external\n", encoding="utf-8")
        runtime.unlink()
        runtime.symlink_to(external)
        with self.assertRaisesRegex(RuntimeError, "symbolic link"):
            self.generate()

    def test_rejects_missing_onnx_runtime_library(self) -> None:
        library = self.prefix / GENERATOR.ONNX_RUNTIME_FILES[0]
        library.unlink()
        with self.assertRaisesRegex(RuntimeError, "exact approved library set"):
            self.generate()

    def test_rejects_symlink_onnx_runtime_library(self) -> None:
        library = self.prefix / GENERATOR.ONNX_RUNTIME_FILES[0]
        external = Path(self.temporary.name) / "external-onnx"
        external.write_bytes(elf64_bytes(62))
        library.unlink()
        library.symlink_to(external)
        with self.assertRaisesRegex(RuntimeError, "symbolic link"):
            self.generate()

    def test_rejects_wrong_onnx_runtime_architecture(self) -> None:
        library = self.prefix / GENERATOR.ONNX_RUNTIME_FILES[-1]
        library.write_bytes(elf64_bytes(183))
        with self.assertRaisesRegex(RuntimeError, "architecture mismatch"):
            self.generate()

    def test_rejects_extra_onnx_runtime_library(self) -> None:
        extra = self.prefix / "lib/rl_sar/onnxruntime/libunexpected.so"
        extra.write_bytes(elf64_bytes(62))
        with self.assertRaisesRegex(RuntimeError, "exact approved library set"):
            self.generate()

    def test_normalizes_architecture_alias(self) -> None:
        content = self.generate(architecture="amd64").read_text(encoding="utf-8")
        self.assertIn('  architecture: "x86_64"', content)

    def test_rejects_invalid_onnx_runtime_version(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "major.minor.patch"):
            self.generate(version="latest")

    def test_rejects_non_release_build(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "build type Release"):
            self.generate(build_type="Debug")

    def test_rejects_short_commit(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "full lowercase Git SHA-1"):
            self.generate(commit="abc123")


if __name__ == "__main__":
    unittest.main()
