#!/usr/bin/env python3

from __future__ import annotations

import hashlib
import importlib.util
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "archive_advised_policy.py"
SPEC = importlib.util.spec_from_file_location("archive_advised_policy", SCRIPT)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class AdvisorArchiveTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        root = Path(self.temporary.name).resolve()
        self.storage = root / "storage"
        self.artifacts = root / "artifacts"
        self.storage.mkdir()
        self.artifacts.mkdir()
        (self.storage / "LW" / "leg_loco").mkdir(parents=True)
        subprocess.run(["git", "init", "-q", str(self.storage)], check=True)
        subprocess.run(["git", "-C", str(self.storage), "config", "user.email", "test@example.invalid"], check=True)
        subprocess.run(["git", "-C", str(self.storage), "config", "user.name", "Test"], check=True)
        (self.storage / ".gitkeep").write_text("\n", encoding="utf-8")
        (self.storage / "LW" / "leg_loco" / ".gitkeep").write_text("\n", encoding="utf-8")
        subprocess.run(["git", "-C", str(self.storage), "add", "."], check=True)
        subprocess.run(["git", "-C", str(self.storage), "commit", "-qm", "init"], check=True)
        self.checkpoint = self.artifacts / "model_10.pt"
        self.jit = self.artifacts / "policy.pt"
        self.onnx = self.artifacts / "policy.onnx"
        self.checkpoint.write_bytes(b"checkpoint")
        self.jit.write_bytes(b"jit")
        self.onnx.write_bytes(b"onnx")
        self.export_receipt = self.artifacts / "receipt.json"
        self.export_receipt.write_text("{}\n", encoding="utf-8")

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def manifest(self) -> dict:
        return {
            "version": 2,
            "archive_authorized": True,
            "storage_root": str(self.storage),
            "collection": "LW/leg_loco",
            "task": "lw-leg-rough",
            "algorithm": "AMP-ROA",
            "runner": "OnPolicyRunnerAmpROA",
            "selected_checkpoint": {
                "path": str(self.checkpoint),
                "sha256": sha(self.checkpoint),
                "iteration": 10,
            },
            "artifacts": {
                "jit": {"path": str(self.jit), "sha256": sha(self.jit)},
                "onnx": {"path": str(self.onnx), "sha256": sha(self.onnx)},
            },
            "source": {"commit": "1" * 40, "dirty": False},
            "parameters": {"action_rate_l2": -0.15},
            "evaluation": {
                "results": [{"path": "/tmp/result.json", "sha256": "3" * 64}]
            },
            "export_receipt": {
                "path": str(self.export_receipt),
                "sha256": sha(self.export_receipt),
            },
            "description_notes": "supervised test candidate",
        }

    def export_validation(self) -> dict:
        manifest = self.manifest()
        selection = {
            "task": manifest["task"],
            "algorithm": manifest["algorithm"],
            "runner": manifest["runner"],
            "checkpoint": manifest["selected_checkpoint"],
            "evaluation_results": [
                {**manifest["evaluation"]["results"][0], "evaluation_id": "eval-1"}
            ],
            "run_identity": {
                "document": {
                    "source": {"head": "1" * 40, "dirty": False}
                }
            },
        }
        document = {
            "inputs": {"checkpoint_selection": {"path": "/tmp/selection.json"}},
            "outputs": {
                "jit": {"path": str(self.jit), "sha256": sha(self.jit)},
                "onnx": {"path": str(self.onnx), "sha256": sha(self.onnx)},
            },
        }
        return {
            "receipt": {"sha256": sha(self.export_receipt)},
            "selection_document": selection,
            "document": document,
        }

    def archive(self, manifest: dict, timestamp: str) -> dict:
        with patch.object(
            MODULE,
            "validate_export_bundle",
            return_value=self.export_validation(),
        ):
            return MODULE.archive_policy(manifest, timestamp=timestamp)

    def replacement_contract(self, destination: Path) -> dict:
        return {
            "authorized": True,
            "path": str(destination),
            "files": {
                name: sha(destination / name)
                for name in (
                    "policy.pt",
                    "policy.onnx",
                    "策略说明.txt",
                    "archive_manifest.json",
                )
            },
        }

    def test_archives_four_files_without_git_action(self) -> None:
        receipt = self.archive(self.manifest(), "2026-07-31-18-00-00")
        destination = Path(receipt["archive_path"])
        self.assertEqual(
            {path.name for path in destination.iterdir()},
            {"policy.pt", "policy.onnx", "策略说明.txt", "archive_manifest.json"},
        )
        self.assertEqual(receipt["git_action"], "none")
        self.assertFalse(receipt["replaced_existing"])
        self.assertEqual(receipt["version"], 2)
        self.assertFalse(receipt["hardware_ready"])
        archived_manifest = json.loads((destination / "archive_manifest.json").read_text(encoding="utf-8"))
        self.assertFalse(archived_manifest["hardware_ready"])

    def test_refuses_dirty_storage(self) -> None:
        (self.storage / "dirty.txt").write_text("dirty", encoding="utf-8")
        with self.assertRaisesRegex(MODULE.ArchiveError, "must be clean"):
            self.archive(self.manifest(), "2026-07-31-18-00-00")

    def test_refuses_duplicate_pair(self) -> None:
        self.archive(self.manifest(), "2026-07-31-18-00-00")
        subprocess.run(["git", "-C", str(self.storage), "add", "."], check=True)
        subprocess.run(["git", "-C", str(self.storage), "commit", "-qm", "archive"], check=True)
        with self.assertRaisesRegex(MODULE.ArchiveError, "already exists"):
            self.archive(self.manifest(), "2026-07-31-18-00-01")

    def test_refuses_destination_collision_without_replacement_contract(self) -> None:
        receipt = self.archive(self.manifest(), "2026-07-31-18-00-00")
        subprocess.run(["git", "-C", str(self.storage), "add", "."], check=True)
        subprocess.run(
            ["git", "-C", str(self.storage), "commit", "-qm", "archive"],
            check=True,
        )
        self.jit.write_bytes(b"new-jit")
        self.onnx.write_bytes(b"new-onnx")
        with self.assertRaisesRegex(MODULE.ArchiveError, "already exists"):
            self.archive(self.manifest(), Path(receipt["archive_path"]).name)

    def test_atomically_replaces_exact_authorized_archive(self) -> None:
        receipt = self.archive(self.manifest(), "2026-07-31-18-00-00")
        destination = Path(receipt["archive_path"])
        replacement = self.replacement_contract(destination)
        subprocess.run(["git", "-C", str(self.storage), "add", "."], check=True)
        subprocess.run(
            ["git", "-C", str(self.storage), "commit", "-qm", "archive"],
            check=True,
        )
        self.jit.write_bytes(b"new-jit")
        self.onnx.write_bytes(b"new-onnx")
        manifest = self.manifest()
        manifest["replace_existing"] = replacement

        replaced = self.archive(manifest, destination.name)

        self.assertTrue(replaced["replaced_existing"])
        self.assertEqual((destination / "policy.pt").read_bytes(), b"new-jit")
        self.assertEqual((destination / "policy.onnx").read_bytes(), b"new-onnx")
        self.assertFalse(any(destination.parent.glob(".advisor-archive-*")))
        archived_manifest = json.loads(
            (destination / "archive_manifest.json").read_text(encoding="utf-8")
        )
        self.assertEqual(archived_manifest["replace_existing"], replacement)

    def test_refuses_replacement_when_old_hash_contract_differs(self) -> None:
        receipt = self.archive(self.manifest(), "2026-07-31-18-00-00")
        destination = Path(receipt["archive_path"])
        replacement = self.replacement_contract(destination)
        replacement["files"]["policy.onnx"] = "0" * 64
        subprocess.run(["git", "-C", str(self.storage), "add", "."], check=True)
        subprocess.run(
            ["git", "-C", str(self.storage), "commit", "-qm", "archive"],
            check=True,
        )
        self.jit.write_bytes(b"new-jit")
        self.onnx.write_bytes(b"new-onnx")
        manifest = self.manifest()
        manifest["replace_existing"] = replacement

        with self.assertRaisesRegex(MODULE.ArchiveError, "hash/type mismatch"):
            self.archive(manifest, destination.name)
        self.assertEqual((destination / "policy.onnx").read_bytes(), b"onnx")

    def test_rejects_manifest_artifact_that_differs_from_export(self) -> None:
        manifest = self.manifest()
        manifest["artifacts"]["jit"]["sha256"] = "0" * 64
        with self.assertRaisesRegex(MODULE.ArchiveError, "differs from export"):
            self.archive(manifest, "2026-07-31-18-00-00")


if __name__ == "__main__":
    unittest.main()
