#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))
SCRIPT = SCRIPT_DIR / "capture_run_identity.py"
SPEC = importlib.util.spec_from_file_location("capture_run_identity", SCRIPT)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


HEAD = "1" * 40


def scenario_contract() -> dict:
    return {
        "scenario_id": "quick-native",
        "scenario_overrides": {"terrain.level": 2, "commands.heading": False},
        "command_schedule": [
            {"start_step": 0, "end_step": 499, "command": [0.5, 0.0, 0.0]}
        ],
        "duration_steps": 500,
        "num_envs": 1,
        "seed": 42,
    }


class RunIdentityTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary_directory.cleanup)
        self.repo = Path(self.temporary_directory.name).resolve() / "repo"
        self.repo.mkdir()
        (self.repo / "configs").mkdir()
        (self.repo / "configs" / "agent.yaml").write_text(
            "seed: 42\n",
            encoding="utf-8",
        )
        (self.repo / "configs" / "env.yaml").write_text(
            "num_envs: 4096\n",
            encoding="utf-8",
        )
        self.status: dict[str, str] = {}
        self.untracked: set[str] = set()
        self.head_contents: dict[str, bytes] = {}
        self.change_head_during_capture = False
        self.change_dirty_during_capture = False
        self.change_diff_during_capture = False
        self.head_reads = 0
        self.status_reads: dict[str, int] = {}
        self.diff_reads = 0
        self.git_calls: list[tuple[str, ...]] = []

    def git_text(
        self,
        repo_root: Path,
        *args: str,
        allow_failure: bool = False,
    ) -> str:
        self.assertEqual(repo_root, self.repo)
        self.git_calls.append(args)
        if args == ("rev-parse", "--show-toplevel"):
            return str(self.repo) + "\n"
        if args == ("symbolic-ref", "--quiet", "--short", "HEAD"):
            return "main\n"
        if args == ("rev-parse", "HEAD"):
            self.head_reads += 1
            if self.change_head_during_capture and self.head_reads > 1:
                return "2" * 40 + "\n"
            return HEAD + "\n"
        if args[:4] == (
            "status",
            "--porcelain=v1",
            "--untracked-files=all",
            "--",
        ):
            relative = args[4]
            self.status_reads[relative] = self.status_reads.get(relative, 0) + 1
            if (
                self.change_dirty_during_capture
                and relative == "configs/env.yaml"
                and self.status_reads[relative] > 1
            ):
                return " M configs/env.yaml\n"
            return self.status.get(args[4], "")
        if args[:3] == ("ls-files", "--error-unmatch", "--"):
            return "" if args[3] in self.untracked else args[3] + "\n"
        self.fail(f"unexpected local Git query: {args}")

    def git_bytes(
        self,
        repo_root: Path,
        *args: str,
        allow_failure: bool = False,
    ) -> bytes | None:
        self.assertEqual(repo_root, self.repo)
        self.git_calls.append(args)
        if args[0] == "show":
            relative = args[1].split(":", 1)[1]
            return self.head_contents.get(
                relative,
                (self.repo / relative).read_bytes(),
            )
        if args[0] == "diff":
            self.diff_reads += 1
            if self.change_diff_during_capture and self.diff_reads > 1:
                return b"changed binary diff\n"
            return b"binary diff\n"
        self.fail(f"unexpected local Git query: {args}")

    def capture(
        self,
        *,
        host_id: str = "younghit",
        patch_evidence_path: Path | None = None,
        scenario: dict | None = None,
    ) -> dict:
        with (
            patch.object(MODULE, "_git_text", side_effect=self.git_text),
            patch.object(MODULE, "_git_bytes", side_effect=self.git_bytes),
        ):
            return MODULE.capture_run_identity(
                self.repo,
                task="lw-leg-rough",
                run_id="run-001",
                host_id=host_id,
                backend="isaaclab",
                algorithm="amp-roa",
                runner="OnPolicyRunnerAmpROA",
                seed=42,
                training_command=[
                    "python",
                    "train.py",
                    "--task=lw-leg-rough",
                    "env.scene.num_envs=4096",
                ],
                hydra_overrides=["env.scene.num_envs=4096"],
                config_paths=["configs/env.yaml", "configs/agent.yaml"],
                scenario_contract=scenario or scenario_contract(),
                patch_evidence_path=patch_evidence_path,
            )

    def test_clean_identity_is_deterministic_and_sorts_configs(self) -> None:
        first = self.capture()
        second = self.capture()
        self.assertEqual(first, second)
        self.assertFalse(first["source"]["dirty"])
        self.assertEqual(
            [item["path"] for item in first["config_files"]],
            ["configs/agent.yaml", "configs/env.yaml"],
        )
        MODULE.validate_run_identity(first)

    def test_host_id_changes_host_identity_without_shared_state(self) -> None:
        first = self.capture(host_id="younghit")
        second = self.capture(host_id="server5090")
        self.assertNotEqual(first["host_id"], second["host_id"])
        self.assertNotEqual(first["identity_sha256"], second["identity_sha256"])
        self.assertEqual(first["source"]["head"], second["source"]["head"])

    def test_scenario_hash_is_canonical(self) -> None:
        scenario = scenario_contract()
        reversed_scenario = dict(reversed(list(scenario.items())))
        first = self.capture(scenario=scenario)
        second = self.capture(scenario=reversed_scenario)
        self.assertEqual(
            first["evaluation_scenario"]["sha256"],
            second["evaluation_scenario"]["sha256"],
        )

    def test_tracked_dirty_source_records_diff_sha256(self) -> None:
        self.status["configs/env.yaml"] = " M configs/env.yaml\n"
        identity = self.capture()
        self.assertTrue(identity["source"]["dirty"])
        self.assertEqual(
            identity["source"]["diff_sha256"],
            MODULE._sha256_bytes(b"binary diff\n"),
        )
        self.assertIsNone(identity["source"]["patch_evidence"])

    def test_untracked_dirty_source_requires_patch_evidence(self) -> None:
        self.status["configs/env.yaml"] = "?? configs/env.yaml\n"
        self.untracked.add("configs/env.yaml")
        with self.assertRaisesRegex(MODULE.RunIdentityError, "requires --patch-evidence"):
            self.capture()
        patch_path = (
            self.repo
            / "learnings"
            / "policy_tuning"
            / "lw-leg-rough"
            / "run-001"
            / "evidence"
            / "source"
            / "source-snapshot-001.patch"
        )
        patch_path.parent.mkdir(parents=True)
        patch_path.write_text(
            "diff --git a/configs/env.yaml b/configs/env.yaml\n",
            encoding="utf-8",
        )
        identity = self.capture(patch_evidence_path=patch_path)
        self.assertIsNone(identity["source"]["diff_sha256"])
        self.assertEqual(
            identity["source"]["patch_evidence"]["sha256"],
            MODULE._sha256_file(patch_path),
        )

    def test_patch_evidence_rejects_wrong_directory_and_missing_dirty_path(self) -> None:
        self.status.update(
            {
                "configs/agent.yaml": "?? configs/agent.yaml\n",
                "configs/env.yaml": "?? configs/env.yaml\n",
            }
        )
        self.untracked.update({"configs/agent.yaml", "configs/env.yaml"})
        wrong_path = self.repo / "source-snapshot-001.patch"
        wrong_path.write_text(
            "diff --git a/configs/agent.yaml b/configs/agent.yaml\n"
            "diff --git a/configs/env.yaml b/configs/env.yaml\n",
            encoding="utf-8",
        )
        with self.assertRaisesRegex(MODULE.RunIdentityError, "this run's evidence/source"):
            self.capture(patch_evidence_path=wrong_path)

        patch_path = (
            self.repo
            / "learnings"
            / "policy_tuning"
            / "lw-leg-rough"
            / "run-001"
            / "evidence"
            / "source"
            / "source-snapshot-001.patch"
        )
        patch_path.parent.mkdir(parents=True)
        patch_path.write_text(
            "diff --git a/configs/env.yaml b/configs/env.yaml\n",
            encoding="utf-8",
        )
        with self.assertRaisesRegex(MODULE.RunIdentityError, "configs/agent.yaml"):
            self.capture(patch_evidence_path=patch_path)

    def test_rejects_config_traversal_and_symlink(self) -> None:
        with self.assertRaisesRegex(MODULE.RunIdentityError, "traversal"):
            MODULE._repository_file(self.repo, "configs/../configs/env.yaml")
        link = self.repo / "configs" / "linked.yaml"
        link.symlink_to(self.repo / "configs" / "env.yaml")
        with self.assertRaisesRegex(MODULE.RunIdentityError, "symlinked"):
            MODULE._repository_file(self.repo, "configs/linked.yaml")

    def test_identity_hash_mutation_is_rejected(self) -> None:
        identity = self.capture()
        identity["training"]["command"].append("--changed")
        with self.assertRaisesRegex(MODULE.RunIdentityError, "identity_sha256 mismatch"):
            MODULE.validate_run_identity(identity)

    def test_output_requires_new_absolute_non_symlinked_path(self) -> None:
        with self.assertRaisesRegex(MODULE.RunIdentityError, "new absolute"):
            MODULE.write_new_absolute_output(Path("identity.json"), "{}\n")
        output = Path(self.temporary_directory.name).resolve() / "identity.json"
        MODULE.write_new_absolute_output(output, "{}\n")
        with self.assertRaisesRegex(MODULE.RunIdentityError, "already exists"):
            MODULE.write_new_absolute_output(output, "changed\n")
        self.assertEqual(output.read_text(encoding="utf-8"), "{}\n")
        real_parent = Path(self.temporary_directory.name).resolve() / "real-parent"
        real_parent.mkdir()
        linked_parent = Path(self.temporary_directory.name).resolve() / "linked-parent"
        linked_parent.symlink_to(real_parent, target_is_directory=True)
        with self.assertRaisesRegex(MODULE.RunIdentityError, "symlinked"):
            MODULE.write_new_absolute_output(linked_parent / "identity.json", "{}\n")

    def test_hydra_overrides_must_appear_in_command_order(self) -> None:
        identity = self.capture()
        identity["training"]["command"].remove("env.scene.num_envs=4096")
        identity["identity_sha256"] = MODULE._sha256_bytes(
            MODULE._canonical_json(
                {key: value for key, value in identity.items() if key != "identity_sha256"}
            ).encode("utf-8")
        )
        with self.assertRaisesRegex(MODULE.RunIdentityError, "command order"):
            MODULE.validate_run_identity(identity)

    def test_git_queries_are_local_and_read_only(self) -> None:
        self.capture()
        allowed = {"rev-parse", "symbolic-ref", "status", "ls-files", "show", "diff"}
        self.assertTrue(self.git_calls)
        self.assertTrue(all(call[0] in allowed for call in self.git_calls))

    def test_git_subprocess_disables_locks_fsmonitor_and_external_diff(self) -> None:
        calls = []

        def fake_run(command, **kwargs):
            calls.append((command, kwargs))
            binary = not kwargs.get("text", False)
            return MODULE.subprocess.CompletedProcess(
                command,
                0,
                stdout=b"" if binary else "",
                stderr=b"" if binary else "",
            )

        with patch.object(MODULE.subprocess, "run", side_effect=fake_run):
            MODULE._git_text(self.repo, "status", "--porcelain=v1")
            MODULE._git_bytes(
                self.repo,
                "diff",
                "--no-ext-diff",
                "--no-textconv",
                "--binary",
                "--full-index",
                "HEAD",
                "--",
                "configs/env.yaml",
            )

        self.assertEqual(len(calls), 2)
        for command, kwargs in calls:
            self.assertEqual(command[:3], ["git", "-c", "core.fsmonitor=false"])
            self.assertEqual(kwargs["env"]["GIT_OPTIONAL_LOCKS"], "0")
            self.assertEqual(kwargs["env"]["GIT_TERMINAL_PROMPT"], "0")
        diff_command = calls[1][0]
        self.assertIn("--no-ext-diff", diff_command)
        self.assertIn("--no-textconv", diff_command)
        self.assertIn("--binary", diff_command)
        self.assertIn("--full-index", diff_command)

    def test_ignored_untracked_config_is_not_treated_as_clean(self) -> None:
        self.untracked.add("configs/env.yaml")
        with self.assertRaisesRegex(MODULE.RunIdentityError, "requires --patch-evidence"):
            self.capture()

    def test_hidden_tracked_content_change_is_not_treated_as_clean(self) -> None:
        self.head_contents["configs/env.yaml"] = b"num_envs: 2048\n"
        identity = self.capture()
        self.assertTrue(identity["source"]["dirty"])
        self.assertIn("configs/env.yaml", identity["source"]["dirty_paths"])

    def test_source_change_during_capture_is_rejected(self) -> None:
        self.change_head_during_capture = True
        with self.assertRaisesRegex(MODULE.RunIdentityError, "changed during"):
            self.capture()

    def test_dirty_set_change_during_capture_is_rejected(self) -> None:
        self.change_dirty_during_capture = True
        with self.assertRaisesRegex(MODULE.RunIdentityError, "changed during"):
            self.capture()

    def test_config_content_change_during_capture_is_rejected(self) -> None:
        original_sha256_file = MODULE._sha256_file
        env_reads = 0

        def changing_sha256(path: Path) -> str:
            nonlocal env_reads
            if path.name == "env.yaml":
                env_reads += 1
                if env_reads > 1:
                    return "f" * 64
            return original_sha256_file(path)

        with patch.object(MODULE, "_sha256_file", side_effect=changing_sha256):
            with self.assertRaisesRegex(MODULE.RunIdentityError, "changed during"):
                self.capture()

    def test_diff_change_during_capture_is_rejected(self) -> None:
        self.status["configs/env.yaml"] = " M configs/env.yaml\n"
        self.change_diff_during_capture = True
        with self.assertRaisesRegex(MODULE.RunIdentityError, "changed during"):
            self.capture()

    def test_patch_change_during_capture_is_rejected(self) -> None:
        self.status["configs/env.yaml"] = "?? configs/env.yaml\n"
        self.untracked.add("configs/env.yaml")
        patch_path = (
            self.repo
            / "learnings"
            / "policy_tuning"
            / "lw-leg-rough"
            / "run-001"
            / "evidence"
            / "source"
            / "source-snapshot-001.patch"
        )
        patch_path.parent.mkdir(parents=True)
        patch_path.write_text(
            "diff --git a/configs/env.yaml b/configs/env.yaml\n",
            encoding="utf-8",
        )
        original_sha256_file = MODULE._sha256_file

        def changing_patch_sha256(path: Path) -> str:
            if path == patch_path:
                return "f" * 64
            return original_sha256_file(path)

        with patch.object(MODULE, "_sha256_file", side_effect=changing_patch_sha256):
            with self.assertRaisesRegex(MODULE.RunIdentityError, "changed during"):
                self.capture(patch_evidence_path=patch_path)

    def test_cli_writes_valid_identity(self) -> None:
        output = Path(self.temporary_directory.name).resolve() / "identity-cli.json"
        command = [
            "python",
            "train.py",
            "--task=lw-leg-rough",
            "env.scene.num_envs=4096",
        ]
        argv = [
            str(SCRIPT),
            "--task",
            "lw-leg-rough",
            "--run-id",
            "run-001",
            "--host-id",
            "younghit",
            "--backend",
            "isaaclab",
            "--algorithm",
            "amp-roa",
            "--runner",
            "OnPolicyRunnerAmpROA",
            "--seed",
            "42",
            "--training-command-json",
            MODULE._canonical_json(command),
            "--hydra-overrides-json",
            '["env.scene.num_envs=4096"]',
            "--config",
            "configs/env.yaml",
            "--scenario-contract-json",
            MODULE._canonical_json(scenario_contract()),
            "--repo-root",
            str(self.repo),
            "--output",
            str(output),
        ]
        with (
            patch.object(sys, "argv", argv),
            patch.object(MODULE, "_git_text", side_effect=self.git_text),
            patch.object(MODULE, "_git_bytes", side_effect=self.git_bytes),
        ):
            self.assertEqual(MODULE.main(), 0)
        identity = MODULE.json.loads(output.read_text(encoding="utf-8"))
        MODULE.validate_run_identity(identity)


if __name__ == "__main__":
    unittest.main()
