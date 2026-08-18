#!/usr/bin/env python3

import argparse
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile
import unittest
from unittest import mock
import zipfile


MANAGER = Path(sys.argv.pop(1)).resolve()
PRODUCTION_CATALOG = Path(sys.argv.pop(1)).resolve()
DOWNLOADER = Path(sys.argv.pop(1)).resolve()

SPEC = importlib.util.spec_from_file_location("manage_mujoco", MANAGER)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError("cannot load MuJoCo manager")
MANAGER_MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MANAGER_MODULE)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class MujocoDownloadIntegrityTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory(prefix="lw-mujoco-integrity-")
        self.root = Path(self.temporary.name)
        self.runtime_root = self.root / "runtime"
        self.runtime_root.mkdir()
        self.catalog = self.root / "catalog.json"

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def linux_archive(
        self,
        architecture: str = "x86_64",
        *,
        complete: bool = True,
        marker: bytes = b"x86",
    ) -> Path:
        archive = self.root / f"mujoco-3.2.7-linux-{architecture}.tar.gz"
        source = self.root / f"source-{architecture}" / "mujoco-3.2.7"
        (source / "include/mujoco").mkdir(parents=True)
        (source / "lib").mkdir()
        (source / "bin").mkdir()
        (source / "include/mujoco/mujoco.h").write_bytes(b"// fixture\n")
        (source / "bin/simulate").write_bytes(b"#!/bin/sh\n" + marker)
        (source / "bin/simulate").chmod(0o755)
        if complete:
            (source / "lib/libmujoco.so.3.2.7").write_bytes(
                b"mujoco\n" + marker
            )
            (source / "lib/libmujoco.so").symlink_to(
                "libmujoco.so.3.2.7"
            )
        with tarfile.open(archive, "w:gz") as output:
            output.add(source, arcname="mujoco-3.2.7")
        return archive

    def windows_archive(self) -> Path:
        archive = self.root / "mujoco-3.2.7-windows-x86_64.zip"
        with zipfile.ZipFile(archive, "w") as output:
            output.writestr("include\\mujoco\\mujoco.h", b"// fixture\n")
            output.writestr("bin\\mujoco.dll", b"dll\n")
            output.writestr("lib\\mujoco.lib", b"library\n")
            output.writestr("bin\\simulate.exe", b"executable\n")
        return archive

    def entry(
        self,
        archive: Path,
        *,
        os_name: str = "Linux",
        architecture: str = "x86_64",
    ) -> dict:
        if os_name == "Linux":
            archive_format = "tgz"
            root_directory = "mujoco-3.2.7"
            required_files = [
                "include/mujoco/mujoco.h",
                "lib/libmujoco.so.3.2.7",
                "bin/simulate",
            ]
            archive_name = f"mujoco-3.2.7-linux-{architecture}.tar.gz"
        elif os_name == "Windows":
            archive_format = "zip"
            root_directory = "."
            required_files = [
                "include/mujoco/mujoco.h",
                "bin/mujoco.dll",
                "lib/mujoco.lib",
                "bin/simulate.exe",
            ]
            archive_name = "mujoco-3.2.7-windows-x86_64.zip"
        else:
            archive_format = "dmg"
            root_directory = "MuJoCo.app/Contents/Frameworks/MuJoCo.framework"
            required_files = [
                "Headers/mujoco.h",
                "Versions/Current/libmujoco.3.2.7.dylib",
            ]
            archive_name = "mujoco-3.2.7-macos-universal2.dmg"
        return {
            "version": "3.2.7",
            "os": os_name,
            "architecture": architecture,
            "archive_name": archive_name,
            "archive_format": archive_format,
            "root_directory": root_directory,
            "url": (
                "https://github.com/google-deepmind/mujoco/releases/download/"
                f"3.2.7/{archive_name}"
            ),
            "sha256": sha256(archive),
            "required_files": required_files,
        }

    def write_catalog(self, entries: list[dict]) -> None:
        self.catalog.write_text(
            json.dumps({"schema_version": 1, "archives": entries}),
            encoding="utf-8",
        )

    def manager_arguments(
        self,
        action: str,
        *,
        os_name: str = "Linux",
        architecture: str = "x86_64",
    ) -> list[str]:
        return [
            sys.executable,
            str(MANAGER),
            action,
            "--catalog",
            str(self.catalog),
            "--version",
            "3.2.7",
            "--os",
            os_name,
            "--architecture",
            architecture,
        ]

    def install(
        self,
        archive: Path,
        *,
        os_name: str = "Linux",
        architecture: str = "x86_64",
    ) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [
                *self.manager_arguments(
                    "install", os_name=os_name, architecture=architecture
                ),
                "--runtime-root",
                str(self.runtime_root),
                "--archive",
                str(archive),
            ],
            check=False,
            text=True,
            capture_output=True,
        )

    def check(
        self,
        *,
        os_name: str = "Linux",
        architecture: str = "x86_64",
    ) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [
                *self.manager_arguments(
                    "check", os_name=os_name, architecture=architecture
                ),
                "--runtime-root",
                str(self.runtime_root),
            ],
            check=False,
            text=True,
            capture_output=True,
        )

    def assert_no_private_candidates(self) -> None:
        self.assertFalse(any(self.runtime_root.glob(".mujoco-candidate.*")))
        self.assertFalse(any(self.runtime_root.glob(".mujoco-backup.*")))

    def old_installation(self) -> Path:
        destination = self.runtime_root / "mujoco"
        destination.mkdir()
        (destination / "preserve.txt").write_text("old\n", encoding="utf-8")
        return destination

    def add_tar_bytes(
        self, output: tarfile.TarFile, name: str, content: bytes = b"fixture"
    ) -> None:
        member = tarfile.TarInfo(name)
        member.size = len(content)
        output.addfile(member, io.BytesIO(content))

    def test_production_catalog_pins_every_supported_asset(self) -> None:
        entries = MANAGER_MODULE.load_catalog(PRODUCTION_CATALOG)
        identities = {
            (entry["os"], entry["architecture"], entry["sha256"])
            for entry in entries
        }
        self.assertEqual(len(entries), 5)
        self.assertIn(
            (
                "Linux",
                "x86_64",
                "520a65fbb0b75cac5245cc07526840545c01faa354f18c623a5e62b36571ed61",
            ),
            identities,
        )
        self.assertIn(
            (
                "Linux",
                "aarch64",
                "3ef2e82658aa6b81ae7ceeb01ed6e8ef4c1ac6d2a45fd5c93aaaf2ec49025998",
            ),
            identities,
        )
        self.assertIn(
            (
                "Windows",
                "x86_64",
                "dba33636fce9f7fe91be3d7b0c462ecfad605533b19f7538fc7ad820dde29e38",
            ),
            identities,
        )
        mac_digests = {
            entry["sha256"] for entry in entries if entry["os"] == "Darwin"
        }
        self.assertEqual(
            mac_digests,
            {"57458727426eadd1ab54476566b0335bf55c07dc48069c8ca5ad09b86151089e"},
        )

    def test_downloader_has_valid_bash_syntax(self) -> None:
        result = subprocess.run(
            ["bash", "-n", str(DOWNLOADER)],
            check=False,
            text=True,
            capture_output=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_download_failure_preserves_existing_installation(self) -> None:
        old = self.old_installation()
        fake_bin = self.root / "fake-bin"
        fake_bin.mkdir()
        fake_curl = fake_bin / "curl"
        fake_curl.write_text("#!/bin/bash\nexit 7\n", encoding="utf-8")
        fake_curl.chmod(0o755)
        environment = os.environ.copy()
        environment.update(
            {
                "PATH": f"{fake_bin}:{environment['PATH']}",
                "RL_SAR_PLATFORM_OS": "Linux",
                "RL_SAR_PLATFORM_ARCH": "x86_64",
            }
        )

        result = subprocess.run(
            ["bash", str(DOWNLOADER), str(self.runtime_root)],
            check=False,
            text=True,
            capture_output=True,
            env=environment,
        )

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Download failed", result.stdout)
        self.assertEqual((old / "preserve.txt").read_text(), "old\n")
        self.assertFalse(any(self.runtime_root.glob(".mujoco-3.2.7-*")))

    def test_valid_linux_archive_installs_checks_and_is_idempotent(self) -> None:
        archive = self.linux_archive()
        entry = self.entry(archive)
        self.write_catalog([entry])

        result = self.install(archive)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(self.check().returncode, 0)
        destination = self.runtime_root / "mujoco"
        self.assertEqual(
            (destination / "VERSION_NUMBER").read_text(encoding="utf-8"),
            "3.2.7\n",
        )
        origin = json.loads(
            (destination / ".rl_sar_mujoco_origin.json").read_text(
                encoding="utf-8"
            )
        )
        self.assertEqual(origin, {"schema_version": 1, **entry})
        repeated = self.install(archive)
        self.assertEqual(repeated.returncode, 0, repeated.stdout + repeated.stderr)
        self.assertIn("already installed", repeated.stdout)
        self.assert_no_private_candidates()

    def test_windows_backslash_archive_installs_with_normalized_layout(self) -> None:
        archive = self.windows_archive()
        self.write_catalog([self.entry(archive, os_name="Windows")])

        result = self.install(archive, os_name="Windows")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertTrue((self.runtime_root / "mujoco/bin/mujoco.dll").is_file())
        self.assertTrue(
            (self.runtime_root / "mujoco/include/mujoco/mujoco.h").is_file()
        )
        self.assertEqual(self.check(os_name="Windows").returncode, 0)

    def test_archive_tamper_is_rejected_before_existing_replacement(self) -> None:
        archive = self.linux_archive()
        self.write_catalog([self.entry(archive)])
        old = self.old_installation()
        archive.write_bytes(archive.read_bytes() + b"tampered")

        result = self.install(archive)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("SHA-256 mismatch", result.stderr)
        self.assertEqual((old / "preserve.txt").read_text(), "old\n")
        self.assert_no_private_candidates()

    def test_platform_archive_mismatch_is_rejected_by_digest(self) -> None:
        x86_archive = self.linux_archive(marker=b"x86")
        arm_archive = self.linux_archive("aarch64", marker=b"arm")
        self.write_catalog(
            [
                self.entry(x86_archive),
                self.entry(arm_archive, architecture="aarch64"),
            ]
        )

        result = self.install(x86_archive, architecture="aarch64")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("SHA-256 mismatch", result.stderr)
        self.assertFalse((self.runtime_root / "mujoco").exists())

    def test_incomplete_candidate_preserves_existing_installation(self) -> None:
        archive = self.linux_archive(complete=False)
        self.write_catalog([self.entry(archive)])
        old = self.old_installation()

        result = self.install(archive)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("required file is missing", result.stderr)
        self.assertEqual((old / "preserve.txt").read_text(), "old\n")
        self.assert_no_private_candidates()

    def test_path_traversal_is_rejected_without_writing_outside_stage(self) -> None:
        archive = self.root / "mujoco-3.2.7-linux-x86_64.tar.gz"
        with tarfile.open(archive, "w:gz") as output:
            self.add_tar_bytes(output, "../escape")
        self.write_catalog([self.entry(archive)])

        result = self.install(archive)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("unsafe path", result.stderr)
        self.assertFalse((self.runtime_root / "escape").exists())
        self.assertFalse((self.root / "escape").exists())
        self.assert_no_private_candidates()

    def test_absolute_archive_path_is_rejected_before_extraction(self) -> None:
        archive = self.root / "mujoco-3.2.7-linux-x86_64.tar.gz"
        with tarfile.open(archive, "w:gz") as output:
            self.add_tar_bytes(output, "/absolute-escape")
        self.write_catalog([self.entry(archive)])

        result = self.install(archive)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("unsafe path", result.stderr)
        self.assert_no_private_candidates()

    def test_link_escape_is_rejected_before_extraction(self) -> None:
        archive = self.root / "mujoco-3.2.7-linux-x86_64.tar.gz"
        with tarfile.open(archive, "w:gz") as output:
            link = tarfile.TarInfo("mujoco-3.2.7/lib/escape")
            link.type = tarfile.SYMTYPE
            link.linkname = "../../../outside"
            output.addfile(link)
        self.write_catalog([self.entry(archive)])

        result = self.install(archive)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("link escapes", result.stderr)
        self.assertFalse((self.runtime_root / "outside").exists())
        self.assert_no_private_candidates()

    def test_ambiguous_top_level_root_is_rejected(self) -> None:
        archive = self.linux_archive()
        source = self.root / "extra"
        source.write_text("extra\n", encoding="utf-8")
        replacement = self.root / "replacement.tar.gz"
        fixture_root = self.root / "source-x86_64/mujoco-3.2.7"
        with tarfile.open(replacement, "w:gz") as output:
            output.add(fixture_root, arcname="mujoco-3.2.7")
            output.add(source, arcname="unexpected-root")
        os.replace(replacement, archive)
        self.write_catalog([self.entry(archive)])

        result = self.install(archive)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("exact approved root directory", result.stderr)
        self.assert_no_private_candidates()

    def test_failed_final_rename_rolls_back_existing_installation(self) -> None:
        archive = self.linux_archive()
        self.write_catalog([self.entry(archive)])
        old = self.old_installation()
        entry = MANAGER_MODULE.load_catalog(self.catalog)[0]
        args = argparse.Namespace(
            archive=archive,
            runtime_root=self.runtime_root,
        )
        original_replace = os.replace

        def fail_candidate_replace(source, destination):
            if Path(source).name == "candidate" and Path(destination).name == "mujoco":
                raise OSError("injected final rename failure")
            return original_replace(source, destination)

        with mock.patch.object(
            MANAGER_MODULE.os, "replace", side_effect=fail_candidate_replace
        ):
            with self.assertRaisesRegex(OSError, "injected final rename failure"):
                MANAGER_MODULE.install_runtime(args, entry)

        self.assertEqual((old / "preserve.txt").read_text(), "old\n")
        self.assert_no_private_candidates()

    @unittest.skipIf(os.name == "nt", "POSIX lock fixture")
    def test_concurrent_install_fails_without_touching_existing_directory(self) -> None:
        archive = self.linux_archive()
        self.write_catalog([self.entry(archive)])
        old = self.old_installation()

        with MANAGER_MODULE.installation_lock(self.runtime_root):
            result = self.install(archive)

        self.assertEqual(result.returncode, 5, result.stdout + result.stderr)
        self.assertIn("another MuJoCo installation is in progress", result.stderr)
        self.assertEqual((old / "preserve.txt").read_text(), "old\n")
        self.assert_no_private_candidates()

    def test_unsafe_destination_symlink_is_never_replaced(self) -> None:
        archive = self.linux_archive()
        self.write_catalog([self.entry(archive)])
        external = self.root / "external"
        external.mkdir()
        (external / "preserve.txt").write_text("external\n", encoding="utf-8")
        (self.runtime_root / "mujoco").symlink_to(external, target_is_directory=True)

        result = self.install(archive)
        self.assertEqual(result.returncode, 3, result.stdout + result.stderr)
        self.assertEqual((external / "preserve.txt").read_text(), "external\n")
        self.assertTrue((self.runtime_root / "mujoco").is_symlink())
        self.assert_no_private_candidates()


if __name__ == "__main__":
    unittest.main()
