#!/usr/bin/env python3

import importlib.util
import hashlib
import json
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
        self.catalog = Path(self.temporary.name) / "runtime-catalog.json"
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
        self.origin_entry = {
            "kind": "onnx",
            "version": "1.22.0",
            "os": "Linux",
            "architecture": "x86_64",
            "archive_name": "onnxruntime-linux-x64-1.22.0.tgz",
            "archive_format": "tgz",
            "root_directory": "onnxruntime-linux-x64-1.22.0",
            "url": "https://example.invalid/onnxruntime-linux-x64-1.22.0.tgz",
            "sha256": "b" * 64,
        }
        self.library_files = [
            {
                "path": "lib/libonnxruntime.so.1.22.0",
                "deployment_path": GENERATOR.ONNX_RUNTIME_FILES[0],
                "sha256": hashlib.sha256(
                    (self.prefix / GENERATOR.ONNX_RUNTIME_FILES[0]).read_bytes()
                ).hexdigest(),
            },
            {
                "path": "lib/libonnxruntime_providers_shared.so",
                "deployment_path": GENERATOR.ONNX_RUNTIME_FILES[1],
                "sha256": hashlib.sha256(
                    (self.prefix / GENERATOR.ONNX_RUNTIME_FILES[1]).read_bytes()
                ).hexdigest(),
            },
        ]
        self.catalog.write_text(
            json.dumps(
                {
                    "schema_version": 2,
                    "archives": [
                        {**self.origin_entry, "library_files": self.library_files}
                    ],
                }
            ),
            encoding="utf-8",
        )
        self.origin = (
            self.prefix
            / "lib/rl_sar/onnxruntime"
            / GENERATOR.ORIGIN_FILENAME
        )
        self.origin.write_text(
            json.dumps({"schema_version": 1, **self.origin_entry}),
            encoding="utf-8",
        )

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
            self.catalog,
        )

    def test_generates_complete_manifest(self) -> None:
        commit = "a" * 40
        manifest = self.generate(commit=commit)
        content = manifest.read_text(encoding="utf-8")
        self.assertIn("schema_version: 4", content)
        self.assertIn(f'source_commit: "{commit}"', content)
        self.assertIn('name: "rl_real_LW"', content)
        self.assertIn("onnx_runtime:", content)
        self.assertIn('  version: "1.22.0"', content)
        self.assertIn('  architecture: "x86_64"', content)
        self.assertIn('    name: "onnxruntime-linux-x64-1.22.0.tgz"', content)
        self.assertIn('    sha256: "' + "b" * 64 + '"', content)
        self.assertIn('    path: "lib/rl_sar/onnxruntime/origin.json"', content)
        self.assertIn("runtime_files:", content)
        self.assertIn(
            '  - path: "share/rl_sar/launch/rl_real_LW.launch.py"',
            content,
        )
        self.assertIn(
            '  - path: "share/ament_index/resource_index/packages/rl_sar"',
            content,
        )
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

    def test_rejects_missing_production_launch(self) -> None:
        launch = self.prefix / "share/rl_sar/launch/rl_real_LW.launch.py"
        launch.unlink()
        with self.assertRaisesRegex(RuntimeError, "exact approved file set"):
            self.generate()

    def test_rejects_extra_production_launch(self) -> None:
        extra = self.prefix / "share/rl_sar/launch/gazebo.launch.py"
        extra.write_text("development launch\n", encoding="utf-8")
        with self.assertRaisesRegex(RuntimeError, "exact approved file set"):
            self.generate()

    def test_rejects_production_launch_bytecode_cache(self) -> None:
        cache = self.prefix / "share/rl_sar/launch/__pycache__"
        cache.mkdir()
        (cache / "rl_real_LW.launch.pyc").write_bytes(b"unapproved bytecode")
        with self.assertRaisesRegex(RuntimeError, "exact approved file set"):
            self.generate()

    def test_rejects_symlink_production_launch(self) -> None:
        launch = self.prefix / "share/rl_sar/launch/rl_real_LW.launch.py"
        external = Path(self.temporary.name) / "external-launch.py"
        external.write_text("external launch\n", encoding="utf-8")
        launch.unlink()
        launch.symlink_to(external)
        with self.assertRaisesRegex(RuntimeError, "symbolic link"):
            self.generate()

    def test_rejects_symlink_production_launch_directory(self) -> None:
        launch_directory = self.prefix / "share/rl_sar/launch"
        external = Path(self.temporary.name) / "external-launch-directory"
        launch_directory.rename(external)
        launch_directory.symlink_to(external, target_is_directory=True)
        with self.assertRaisesRegex(RuntimeError, "real directory"):
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

    def test_rejects_onnx_library_bytes_not_from_approved_archive(self) -> None:
        library = self.prefix / GENERATOR.ONNX_RUNTIME_FILES[0]
        library.write_bytes(library.read_bytes() + b"tampered")
        with self.assertRaisesRegex(RuntimeError, "approved archive"):
            self.generate()

    def test_rejects_extra_onnx_runtime_library(self) -> None:
        extra = self.prefix / "lib/rl_sar/onnxruntime/libunexpected.so"
        extra.write_bytes(elf64_bytes(62))
        with self.assertRaisesRegex(RuntimeError, "exact approved library set"):
            self.generate()

    def test_rejects_missing_or_mismatched_runtime_provenance(self) -> None:
        self.origin.unlink()
        with self.assertRaisesRegex(RuntimeError, "provenance"):
            self.generate()

        self.origin.write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    **self.origin_entry,
                    "sha256": "c" * 64,
                }
            ),
            encoding="utf-8",
        )
        with self.assertRaisesRegex(RuntimeError, "differs"):
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
