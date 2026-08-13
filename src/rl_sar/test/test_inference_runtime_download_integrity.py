#!/usr/bin/env python3

import hashlib
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tarfile
import tempfile
import unittest


MANAGER = Path(sys.argv.pop(1)).resolve()
VALIDATOR = Path(sys.argv.pop(1)).resolve()
PRODUCTION_CATALOG = Path(sys.argv.pop(1)).resolve()


def elf_header(machine: int = 62) -> bytes:
    identification = bytearray(16)
    identification[0:4] = b"\x7fELF"
    identification[4] = 2
    identification[5] = 1
    identification[6] = 1
    return struct.pack(
        "<16sHHIQQQIHHHHHH",
        bytes(identification),
        3,
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


class InferenceRuntimeDownloadIntegrityTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory(prefix="lw-runtime-integrity-")
        self.root = Path(self.temporary.name)
        self.runtime_root = self.root / "runtime"
        self.runtime_root.mkdir()
        self.archive = self.root / "onnx.tgz"
        self.catalog = self.root / "catalog.json"

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def make_archive(self, *, valid: bool = True) -> None:
        source = self.root / "source" / "onnx-fixture"
        (source / "include").mkdir(parents=True)
        (source / "lib").mkdir()
        (source / "include/onnxruntime_cxx_api.h").write_text(
            "// fixture\n", encoding="utf-8"
        )
        if valid:
            (source / "lib/libonnxruntime.so").write_bytes(elf_header())
        with tarfile.open(self.archive, "w:gz") as output:
            output.add(source, arcname="onnx-fixture")

    def write_catalog(self, *, duplicate: bool = False) -> dict[str, str]:
        entry = {
            "kind": "onnx",
            "version": "1.22.0",
            "os": "Linux",
            "architecture": "x86_64",
            "archive_name": "onnx.tgz",
            "archive_format": "tgz",
            "root_directory": "onnx-fixture",
            "url": "https://example.invalid/onnx.tgz",
            "sha256": hashlib.sha256(self.archive.read_bytes()).hexdigest(),
        }
        archives = [entry, dict(entry)] if duplicate else [entry]
        self.catalog.write_text(
            json.dumps({"schema_version": 1, "archives": archives}),
            encoding="utf-8",
        )
        return entry

    def run_manager(self, action: str, *extra: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [
                sys.executable,
                str(MANAGER),
                action,
                "--catalog",
                str(self.catalog),
                "--kind",
                "onnx",
                "--version",
                "1.22.0",
                "--os",
                "Linux",
                "--architecture",
                "x86_64",
                *extra,
            ],
            check=False,
            text=True,
            capture_output=True,
        )

    def install(self) -> subprocess.CompletedProcess[str]:
        return self.run_manager(
            "install",
            "--runtime-root",
            str(self.runtime_root),
            "--validator",
            str(VALIDATOR),
            "--archive",
            str(self.archive),
        )

    def check(self) -> subprocess.CompletedProcess[str]:
        return self.run_manager(
            "check",
            "--runtime-root",
            str(self.runtime_root),
            "--validator",
            str(VALIDATOR),
        )

    def test_valid_archive_installs_and_records_exact_provenance(self) -> None:
        self.make_archive()
        entry = self.write_catalog()
        result = self.install()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        origin = json.loads(
            (self.runtime_root / "onnxruntime/.rl_sar_runtime_origin.json")
            .read_text(encoding="utf-8")
        )
        self.assertEqual(origin, {"schema_version": 1, **entry})
        checked = self.check()
        self.assertEqual(checked.returncode, 0, checked.stdout + checked.stderr)

    def test_one_byte_change_is_rejected_before_old_runtime_replacement(self) -> None:
        self.make_archive()
        self.write_catalog()
        self.archive.write_bytes(self.archive.read_bytes() + b"x")
        old = self.runtime_root / "onnxruntime"
        old.mkdir()
        (old / "preserve.txt").write_text("old\n", encoding="utf-8")
        result = self.install()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("SHA-256 mismatch", result.stderr)
        self.assertEqual((old / "preserve.txt").read_text(encoding="utf-8"), "old\n")
        self.assertFalse(any(self.runtime_root.glob(".runtime-*")))

    def test_invalid_candidate_preserves_existing_directory(self) -> None:
        self.make_archive(valid=False)
        self.write_catalog()
        old = self.runtime_root / "onnxruntime"
        old.mkdir()
        (old / "preserve.txt").write_text("old\n", encoding="utf-8")
        result = self.install()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("candidate runtime failed", result.stderr)
        self.assertTrue((old / "preserve.txt").is_file())

    def test_valid_unproven_runtime_is_not_automatically_replaced(self) -> None:
        self.make_archive()
        self.write_catalog()
        runtime = self.runtime_root / "onnxruntime"
        (runtime / "include").mkdir(parents=True)
        (runtime / "lib").mkdir()
        (runtime / "include/onnxruntime_cxx_api.h").write_text("// old\n")
        (runtime / "lib/libonnxruntime.so").write_bytes(elf_header())
        result = self.check()
        self.assertEqual(result.returncode, 3)
        self.assertIn("no approved archive provenance", result.stderr)
        installed = self.install()
        self.assertNotEqual(installed.returncode, 0)
        self.assertIn("refusing automatic replacement", installed.stderr)

    def test_invalid_old_directory_is_replaced_only_after_candidate_validation(self) -> None:
        self.make_archive()
        self.write_catalog()
        old = self.runtime_root / "onnxruntime"
        old.mkdir()
        (old / "obsolete.txt").write_text("old\n", encoding="utf-8")
        result = self.install()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertFalse((old / "obsolete.txt").exists())
        self.assertEqual(self.check().returncode, 0)

    def test_unsupported_and_duplicate_catalog_entries_fail_closed(self) -> None:
        self.make_archive()
        self.write_catalog()
        unsupported = subprocess.run(
            [
                sys.executable,
                str(MANAGER),
                "select",
                "--catalog",
                str(self.catalog),
                "--kind",
                "onnx",
                "--version",
                "9.9.9",
                "--os",
                "Linux",
                "--architecture",
                "x86_64",
            ],
            check=False,
            text=True,
            capture_output=True,
        )
        self.assertNotEqual(unsupported.returncode, 0)
        self.assertIn("unsupported or ambiguous", unsupported.stderr)

        self.write_catalog(duplicate=True)
        duplicate = self.run_manager("select")
        self.assertNotEqual(duplicate.returncode, 0)
        self.assertIn("duplicate", duplicate.stderr)

    def test_production_catalog_has_only_the_reviewed_linux_matrix(self) -> None:
        catalog = json.loads(PRODUCTION_CATALOG.read_text(encoding="utf-8"))
        actual = {
            (
                entry["kind"],
                entry["version"],
                entry["os"],
                entry["architecture"],
                entry["sha256"],
            )
            for entry in catalog["archives"]
        }
        self.assertEqual(
            actual,
            {
                (
                    "libtorch",
                    "2.3.0",
                    "Linux",
                    "x86_64",
                    "f60009d2a74b6c8bdb174e398c70d217b7d12a4d3d358cd1db0690b32f6e193b",
                ),
                (
                    "onnx",
                    "1.22.0",
                    "Linux",
                    "x86_64",
                    "8344d55f93d5bc5021ce342db50f62079daf39aaafb5d311a451846228be49b3",
                ),
                (
                    "onnx",
                    "1.22.0",
                    "Linux",
                    "aarch64",
                    "bb76395092d150b52c7092dc6b8f2fe4d80f0f3bf0416d2f269193e347e24702",
                ),
            },
        )


if __name__ == "__main__":
    unittest.main()
