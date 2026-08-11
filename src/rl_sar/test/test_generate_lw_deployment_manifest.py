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

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def test_generates_complete_manifest(self) -> None:
        commit = "a" * 40
        manifest = GENERATOR.generate_manifest(
            self.prefix, commit, "Release"
        )
        content = manifest.read_text(encoding="utf-8")
        self.assertIn("schema_version: 2", content)
        self.assertIn(f'source_commit: "{commit}"', content)
        self.assertIn('name: "rl_real_LW"', content)
        self.assertIn("runtime_files:", content)
        self.assertEqual(
            content.count("  - path:"),
            len(GENERATOR.POLICY_FILES) + len(GENERATOR.RUNTIME_FILES),
        )

    def test_rejects_untracked_style_symlink_asset(self) -> None:
        relative = Path(GENERATOR.POLICY_FILES[0])
        asset = self.bundle / relative
        external = Path(self.temporary.name) / "external.yaml"
        external.write_text("external\n", encoding="utf-8")
        asset.unlink()
        asset.symlink_to(external)
        with self.assertRaisesRegex(RuntimeError, "symbolic link"):
            GENERATOR.generate_manifest(self.prefix, "a" * 40, "Release")

    def test_rejects_symlink_installed_executable(self) -> None:
        external = Path(self.temporary.name) / "external-rl_real_LW"
        self.executable.rename(external)
        self.executable.symlink_to(external)
        with self.assertRaisesRegex(RuntimeError, "installed executable"):
            GENERATOR.generate_manifest(self.prefix, "a" * 40, "Release")

    def test_rejects_missing_runtime_dependency(self) -> None:
        runtime = self.prefix / GENERATOR.RUNTIME_FILES[0]
        runtime.unlink()
        with self.assertRaisesRegex(RuntimeError, "runtime dependency"):
            GENERATOR.generate_manifest(self.prefix, "a" * 40, "Release")

    def test_rejects_symlink_runtime_dependency(self) -> None:
        runtime = self.prefix / GENERATOR.RUNTIME_FILES[0]
        external = Path(self.temporary.name) / "external-runtime"
        external.write_text("external\n", encoding="utf-8")
        runtime.unlink()
        runtime.symlink_to(external)
        with self.assertRaisesRegex(RuntimeError, "symbolic link"):
            GENERATOR.generate_manifest(self.prefix, "a" * 40, "Release")

    def test_rejects_non_release_build(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "build type Release"):
            GENERATOR.generate_manifest(self.prefix, "a" * 40, "Debug")

    def test_rejects_short_commit(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "full lowercase Git SHA-1"):
            GENERATOR.generate_manifest(self.prefix, "abc123", "Release")


if __name__ == "__main__":
    unittest.main()
