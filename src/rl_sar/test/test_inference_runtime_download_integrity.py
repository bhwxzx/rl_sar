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
            main_library = source / "lib/libonnxruntime.so.1.22.0"
            main_library.write_bytes(elf_header())
            (source / "lib/libonnxruntime_providers_shared.so").write_bytes(
                elf_header()
            )
            (source / "lib/libonnxruntime.so.1").symlink_to(
                "libonnxruntime.so.1.22.0"
            )
            (source / "lib/libonnxruntime.so").symlink_to(
                "libonnxruntime.so.1"
            )
        with tarfile.open(self.archive, "w:gz") as output:
            output.add(source, arcname="onnx-fixture")

    def write_catalog(self, *, duplicate: bool = False) -> dict:
        approved_library_digest = hashlib.sha256(elf_header()).hexdigest()
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
            "library_files": [
                {
                    "path": "lib/libonnxruntime.so.1.22.0",
                    "deployment_path": (
                        "lib/rl_sar/onnxruntime/libonnxruntime.so.1"
                    ),
                    "sha256": approved_library_digest,
                },
                {
                    "path": "lib/libonnxruntime_providers_shared.so",
                    "deployment_path": (
                        "lib/rl_sar/onnxruntime/"
                        "libonnxruntime_providers_shared.so"
                    ),
                    "sha256": approved_library_digest,
                },
            ],
        }
        archives = [entry, dict(entry)] if duplicate else [entry]
        self.catalog.write_text(
            json.dumps({"schema_version": 2, "archives": archives}),
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
        expected_origin = {
            key: value for key, value in entry.items() if key != "library_files"
        }
        self.assertEqual(origin, {"schema_version": 1, **expected_origin})
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

    def test_installed_library_tamper_is_rejected_with_origin_unchanged(self) -> None:
        self.make_archive()
        self.write_catalog()
        self.assertEqual(self.install().returncode, 0)
        provider = (
            self.runtime_root
            / "onnxruntime/lib/libonnxruntime_providers_shared.so"
        )
        provider.write_bytes(provider.read_bytes() + b"tampered")

        result = self.check()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("library SHA-256 mismatch", result.stderr)

    def test_unapproved_main_library_alias_is_rejected(self) -> None:
        self.make_archive()
        self.write_catalog()
        self.assertEqual(self.install().returncode, 0)
        alias = self.runtime_root / "onnxruntime/lib/libonnxruntime.so"
        alias.unlink()
        alias.symlink_to("libonnxruntime_providers_shared.so")

        result = self.check()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("alias resolves to an unapproved file", result.stderr)

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

    def test_malformed_library_catalog_entries_fail_closed(self) -> None:
        self.make_archive()
        self.write_catalog()
        baseline = json.loads(self.catalog.read_text(encoding="utf-8"))

        cases = []
        missing = json.loads(json.dumps(baseline))
        del missing["archives"][0]["library_files"]
        cases.append((missing, "unexpected field set"))

        bad_digest = json.loads(json.dumps(baseline))
        bad_digest["archives"][0]["library_files"][0]["sha256"] = "bad"
        cases.append((bad_digest, "library SHA-256"))

        duplicate_path = json.loads(json.dumps(baseline))
        duplicate_path["archives"][0]["library_files"][1][
            "deployment_path"
        ] = duplicate_path["archives"][0]["library_files"][0][
            "deployment_path"
        ]
        cases.append((duplicate_path, "duplicate library path"))

        escaping_path = json.loads(json.dumps(baseline))
        escaping_path["archives"][0]["library_files"][0]["path"] = (
            "../libonnxruntime.so"
        )
        cases.append((escaping_path, "normalized relative path"))

        for catalog, expected_error in cases:
            with self.subTest(expected_error=expected_error):
                self.catalog.write_text(json.dumps(catalog), encoding="utf-8")
                result = self.run_manager("select")
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(expected_error, result.stderr)

    def test_production_catalog_has_only_the_reviewed_linux_matrix(self) -> None:
        catalog = json.loads(PRODUCTION_CATALOG.read_text(encoding="utf-8"))
        self.assertEqual(catalog["schema_version"], 2)
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
        library_matrix = {
            entry["architecture"]: {
                record["deployment_path"]: record["sha256"]
                for record in entry["library_files"]
            }
            for entry in catalog["archives"]
        }
        self.assertEqual(
            library_matrix,
            {
                "x86_64": {
                    "lib/rl_sar/onnxruntime/libonnxruntime.so.1": (
                        "3da6146e14e7b8aaec625dde11d6114c7457c87a5f93d744897da8781e35c673"
                    ),
                    "lib/rl_sar/onnxruntime/libonnxruntime_providers_shared.so": (
                        "e4706ea02be3999167f000bb1ff7391c86e95d2d96f78eb33225c14b172a8451"
                    ),
                },
                "aarch64": {
                    "lib/rl_sar/onnxruntime/libonnxruntime.so.1": (
                        "0afd69a0ae38c5099fd0e8604dda398ac43dee67cd9c6394b5142b19e82528de"
                    ),
                    "lib/rl_sar/onnxruntime/libonnxruntime_providers_shared.so": (
                        "3b53a2c1f3ecb87dedaee20e2a6d4d9aaec133e3ec280e2b9cc34efbed7de423"
                    ),
                },
            },
        )


if __name__ == "__main__":
    unittest.main()
