#!/usr/bin/env python3

from __future__ import annotations

import contextlib
import hashlib
import importlib.util
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))
SCRIPT = SCRIPT_DIR / "discover_algorithm_profile.py"
SPEC = importlib.util.spec_from_file_location("discover_algorithm_profile", SCRIPT)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)

import capture_run_identity as IDENTITY  # noqa: E402


def run_identity(
    *,
    backend: str = "isaaclab",
    algorithm: str = "AMPROAPPO",
    runner: str = "OnPolicyRunnerAmpROA",
) -> dict:
    scenario = {
        "scenario_id": "quick-native",
        "scenario_overrides": {},
        "command_schedule": [],
        "duration_steps": 500,
        "num_envs": 1,
        "seed": 42,
    }
    value = {
        "version": 1,
        "task": "lw-leg-rough",
        "run_id": "run-001",
        "host_id": "younghit",
        "backend": backend,
        "algorithm": algorithm,
        "runner": runner,
        "seed": 42,
        "source": {
            "repository_root": "/absolute/robot_lab",
            "branch": "main",
            "head": "1" * 40,
            "dirty": False,
            "dirty_paths": [],
            "diff_sha256": None,
            "patch_evidence": None,
        },
        "training": {
            "command": [
                "python",
                "scripts/reinforcement_learning/rsl_rl/train.py",
                "--task=lw-leg-rough",
            ],
            "hydra_overrides": [],
        },
        "config_files": [{"path": "config.yaml", "sha256": "2" * 64}],
        "evaluation_scenario": {
            "contract": scenario,
            "sha256": IDENTITY._sha256_bytes(
                IDENTITY._canonical_json(scenario).encode("utf-8")
            ),
        },
    }
    value["identity_sha256"] = IDENTITY._sha256_bytes(
        IDENTITY._canonical_json(value).encode("utf-8")
    )
    return value


def bind_config(value: dict, repository_root: Path, config: Path) -> dict:
    value["source"]["repository_root"] = str(repository_root)
    relative_path = config.resolve().relative_to(repository_root).as_posix()
    value["config_files"] = [
        {
            "path": relative_path,
            "sha256": IDENTITY._sha256_file(config),
        }
    ]
    value.pop("identity_sha256", None)
    value["identity_sha256"] = IDENTITY._sha256_bytes(
        IDENTITY._canonical_json(value).encode("utf-8")
    )
    return value


class AlgorithmProfileDiscoveryTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary_directory.cleanup)
        self.directory = Path(self.temporary_directory.name).resolve()
        self.registry = MODULE.load_registry()

    def test_exact_run_identity_matches_specific_profile(self) -> None:
        result = MODULE.discover(run_identity(), self.registry, None, None)
        self.assertEqual(result["schema_version"], 2)
        self.assertEqual(result["status"], "matched")
        self.assertEqual(result["matched_profile"]["id"], "rsl-rl-amp-roa")
        self.assertEqual(result["identity"]["backend"], "rsl_rl")
        self.assertEqual(
            result["identity_resolution"]["backend_source"],
            "training_command",
        )
        self.assertEqual(
            result["run_identity"]["identity_sha256"],
            run_identity()["identity_sha256"],
        )

    def test_dumped_config_resolves_algorithm_label_without_overriding_runner(self) -> None:
        config = self.directory / "trainer.yaml"
        config.write_text(
            "algorithm:\n  class_name: AMPROAPPO\n"
            "runner:\n  class_name: OnPolicyRunnerAmpROA\n",
            encoding="utf-8",
        )
        result = MODULE.discover(
            bind_config(
                run_identity(algorithm="amp-roa"),
                self.directory,
                config,
            ),
            self.registry,
            None,
            config,
        )
        self.assertEqual(result["matched_profile"]["id"], "rsl-rl-amp-roa")
        self.assertEqual(result["identity_resolution"]["algorithm_source"], "config")
        self.assertEqual(
            result["identity_resolution"]["runner_source"],
            "run_identity_and_config",
        )
        self.assertEqual(
            result["identity_resolution"]["config_file"],
            {
                "path": "trainer.yaml",
                "sha256": IDENTITY._sha256_file(config),
            },
        )

    def test_dumped_config_must_be_bound_to_run_identity_bytes(self) -> None:
        config = self.directory / "trainer.yaml"
        config.write_text("algorithm:\n  class_name: AMPROAPPO\n", encoding="utf-8")
        listed = self.directory / "listed.yaml"
        listed.write_text("algorithm:\n  class_name: AMPROAPPO\n", encoding="utf-8")
        with self.assertRaisesRegex(MODULE.ProfileError, "not listed"):
            MODULE.discover(
                bind_config(run_identity(), self.directory, listed),
                self.registry,
                None,
                config,
            )

        identity = bind_config(run_identity(), self.directory, config)
        config.write_text("algorithm:\n  class_name: PPO\n", encoding="utf-8")
        with self.assertRaisesRegex(MODULE.ProfileError, "sha256 does not match"):
            MODULE.discover(identity, self.registry, None, config)

    def test_old_draft_and_mutated_identity_are_rejected(self) -> None:
        draft_path = self.directory / "draft.json"
        draft_path.write_text('{"training": {"command": ["python"]}}', encoding="utf-8")
        with self.assertRaisesRegex(MODULE.ProfileError, "run_identity.version"):
            MODULE._load_run_identity(draft_path)

        mutated = run_identity()
        mutated["runner"] = "OnPolicyRunner"
        mutated_path = self.directory / "mutated.json"
        mutated_path.write_text(json.dumps(mutated), encoding="utf-8")
        with self.assertRaisesRegex(MODULE.ProfileError, "identity_sha256 mismatch"):
            MODULE._load_run_identity(mutated_path)

    def test_legacy_auto_identity_is_rejected(self) -> None:
        value = run_identity(algorithm="auto")
        with self.assertRaisesRegex(MODULE.ProfileError, "legacy auto"):
            MODULE.discover(value, self.registry, None, None)

    def test_ambiguous_config_and_runner_conflict_are_rejected(self) -> None:
        ambiguous = self.directory / "ambiguous.yaml"
        ambiguous.write_text(
            "first:\n  class_name: AMPROAPPO\n"
            "second:\n  class_name: PPO\n",
            encoding="utf-8",
        )
        with self.assertRaisesRegex(MODULE.ProfileError, "ambiguous algorithm"):
            MODULE.discover(
                bind_config(run_identity(), self.directory, ambiguous),
                self.registry,
                None,
                ambiguous,
            )

        conflicting = self.directory / "conflicting.yaml"
        conflicting.write_text(
            "runner:\n  class_name: OnPolicyRunner\n",
            encoding="utf-8",
        )
        with self.assertRaisesRegex(MODULE.ProfileError, "runner conflicts"):
            MODULE.discover(
                bind_config(run_identity(), self.directory, conflicting),
                self.registry,
                None,
                conflicting,
            )

    def test_declared_profile_backend_conflict_is_rejected(self) -> None:
        normalized = MODULE.discover(
            run_identity(backend="RSL_RL"),
            self.registry,
            None,
            None,
        )
        self.assertEqual(normalized["identity"]["backend"], "rsl_rl")
        with self.assertRaisesRegex(MODULE.ProfileError, "backend conflicts"):
            MODULE.discover(
                run_identity(backend="skrl"),
                self.registry,
                None,
                None,
            )

    def test_generic_candidate_and_metric_aliases_are_deterministic(self) -> None:
        log = self.directory / "train.log"
        log.write_text("Mean custom score: 1.25\n", encoding="utf-8")
        registry_before = json.dumps(self.registry, sort_keys=True)
        result = MODULE.discover(
            run_identity(algorithm="NovelPPO", runner="NovelRunner"),
            self.registry,
            log,
            None,
        )
        self.assertEqual(result["status"], "candidate")
        self.assertEqual(
            result["candidate_profile"]["metric_aliases"],
            {"Mean custom score": "mean_custom_score"},
        )
        self.assertEqual(
            result["candidate_profile"]["metric_source"],
            {
                "path": str(log),
                "sha256": hashlib.sha256(log.read_bytes()).hexdigest(),
                "size_bytes": log.stat().st_size,
            },
        )
        self.assertEqual(json.dumps(self.registry, sort_keys=True), registry_before)

    def test_log_change_during_metric_discovery_is_rejected(self) -> None:
        log = self.directory / "train.log"
        content = b"Mean custom score: 1.25\n"
        log.write_bytes(content)
        before = SimpleNamespace(
            st_dev=1,
            st_ino=2,
            st_size=len(content),
            st_mtime_ns=3,
        )
        after = SimpleNamespace(
            st_dev=1,
            st_ino=2,
            st_size=len(content),
            st_mtime_ns=4,
        )
        with patch.object(MODULE.os, "fstat", side_effect=[before, after]):
            with self.assertRaisesRegex(MODULE.ProfileError, "log file changed"):
                MODULE.discover(
                    run_identity(algorithm="NovelPPO", runner="NovelRunner"),
                    self.registry,
                    log,
                    None,
                )

    def test_cli_accepts_run_identity_and_emits_bound_result(self) -> None:
        identity_path = self.directory / "identity.json"
        identity_path.write_text(json.dumps(run_identity()), encoding="utf-8")
        stdout = io.StringIO()
        argv = [str(SCRIPT), str(identity_path)]
        with patch.object(sys, "argv", argv), contextlib.redirect_stdout(stdout):
            self.assertEqual(MODULE.main(), 0)
        result = json.loads(stdout.getvalue())
        self.assertEqual(result["run_identity"]["run_id"], "run-001")
        self.assertEqual(result["matched_profile"]["id"], "rsl-rl-amp-roa")


if __name__ == "__main__":
    unittest.main()
