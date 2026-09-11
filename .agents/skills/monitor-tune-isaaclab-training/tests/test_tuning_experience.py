#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import copy
import sys
import tempfile
import unittest
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from threading import Barrier
from unittest.mock import patch


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))
SCRIPT = SCRIPT_DIR / "record_tuning_experience.py"
SPEC = importlib.util.spec_from_file_location("record_tuning_experience", SCRIPT)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)

import capture_run_identity as IDENTITY  # noqa: E402
import capture_effective_training_config as CONFIG  # noqa: E402


ENV_YAML = """\
seed: 42
scene:
  num_envs: 4096
rewards:
  action_rate:
    func: example.rewards:action_rate
    weight: -0.3
  disabled_term: null
"""

AGENT_YAML = """\
seed: 42
device: cuda:0
num_steps_per_env: 24
max_iterations: 100000
experiment_name: effective-config-test
run_name: ''
logger: wandb
resume: false
class_name: OnPolicyRunnerAmpROA
algorithm:
  class_name: AMPROAPPO
"""


def event(event_type: str = "assessment") -> dict:
    value = {
        "version": 1,
        "event_id": "assessment-001",
        "event_type": event_type,
        "recorded_at": "2026-07-31T18:00:00+08:00",
        "task": "lw-leg-rough",
        "run_id": "run-001",
        "algorithm": "amp-roa",
        "context": {
            "observation_fingerprint": "obs-hash",
            "reward_fingerprint": "reward-hash",
            "deployment_fingerprint": "deploy-hash",
        },
        "parameters": {"action_rate_l2": -0.15},
        "evidence": {},
        "analysis": {"summary": "healthy", "confidence": "medium"},
        "next_suggestion": "recheck at iteration 40000",
    }
    if event_type == "feedback":
        value["evidence"] = {"source": "sim2real"}
    return value


def version_two_event(repository_root: Path | None = None) -> dict:
    value = event()
    value["version"] = 2
    scenario = {
        "scenario_id": "quick-native",
        "scenario_overrides": {},
        "command_schedule": [],
        "duration_steps": 500,
        "num_envs": 1,
        "seed": 42,
    }
    identity = {
        "version": 1,
        "task": value["task"],
        "run_id": value["run_id"],
        "host_id": "younghit",
        "backend": "isaaclab",
        "algorithm": value["algorithm"],
        "runner": "OnPolicyRunnerAmpROA",
        "seed": 42,
        "source": {
            "repository_root": str(repository_root or Path("/absolute/robot_lab")),
            "branch": "main",
            "head": "1" * 40,
            "dirty": False,
            "dirty_paths": [],
            "diff_sha256": None,
            "patch_evidence": None,
        },
        "training": {
            "command": ["python", "train.py", "--task=lw-leg-rough"],
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
    identity["identity_sha256"] = IDENTITY._sha256_bytes(
        IDENTITY._canonical_json(identity).encode("utf-8")
    )
    value["run_identity"] = identity
    return value


def history_root(directory: str) -> Path:
    root = Path(directory).resolve() / "robot_lab" / "learnings" / "policy_tuning"
    root.mkdir(parents=True)
    return root


def version_three_event(root: Path, event_type: str = "assessment") -> dict:
    repository_root = root.parents[1]
    value = version_two_event(repository_root)
    value["version"] = 3
    value["event_type"] = event_type
    if event_type == "feedback":
        value["evidence"] = {"source": "sim2real"}
    log_directory = (
        repository_root
        / "logs"
        / "rsl_rl"
        / "effective-config-test"
        / value["run_id"]
    )
    params = log_directory / "params"
    params.mkdir(parents=True, exist_ok=True)
    (params / "env.yaml").write_text(ENV_YAML, encoding="utf-8")
    (params / "agent.yaml").write_text(AGENT_YAML, encoding="utf-8")
    config = CONFIG.capture_effective_config(value["run_identity"], log_directory)
    config_path = (
        root
        / value["task"]
        / value["run_id"]
        / "evidence"
        / "source"
        / "effective-config-snapshot-001.json"
    )
    config_path.parent.mkdir(parents=True, exist_ok=True)
    receipt = (
        {
            "sha256": MODULE.hashlib.sha256(config_path.read_bytes()).hexdigest(),
        }
        if config_path.exists()
        else CONFIG.write_new_evidence(config_path, config)
    )
    value["context"]["reward_fingerprint"] = config["fingerprints"]["reward"]
    value["evidence"]["effective_config"] = {
        "path": str(config_path),
        "sha256": receipt["sha256"],
        "effective_config_fingerprint": config["fingerprints"]["effective_config"],
        "reward_fingerprint": config["fingerprints"]["reward"],
    }
    return value


def version_four_event(root: Path, event_type: str = "assessment") -> dict:
    value = version_three_event(root, event_type)
    value["version"] = 4
    if event_type in MODULE.EVIDENCE_EVENT_TYPES:
        value["evidence"]["event"] = {
            "status": "unavailable",
            "reason": "raw event evidence not captured",
        }
    if event_type == "feedback":
        value["evidence"]["policy_binding"] = {
            "status": "unavailable",
            "reason": "policy identity not available",
        }
    value["evidence"]["outcome"] = {
        "status": "unavailable",
        "reason": "no baseline comparison was recorded",
    }
    return value


class TuningExperienceTests(unittest.TestCase):
    def test_writes_immutable_event(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = history_root(directory)
            value = version_four_event(root)
            receipt = MODULE.write_event(root, value)
            self.assertTrue(Path(receipt["event_path"]).is_file())
            self.assertTrue(receipt["immutable"])
            with self.assertRaises(MODULE.ExperienceError):
                MODULE.write_event(root, value)

    def test_concurrent_writers_publish_exactly_one_event(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = history_root(directory)
            value = version_four_event(root)
            barrier = Barrier(2)

            def write_once() -> str:
                barrier.wait()
                try:
                    MODULE.write_event(root, value)
                except MODULE.ExperienceError:
                    return "rejected"
                return "published"

            with ThreadPoolExecutor(max_workers=2) as executor:
                outcomes = sorted(executor.map(lambda _: write_once(), range(2)))
            self.assertEqual(outcomes, ["published", "rejected"])
            run_root = root / value["task"] / value["run_id"]
            self.assertEqual(len(list(run_root.glob("*.json"))), 1)
            self.assertEqual(len(list(root.rglob("*.tmp-*"))), 0)

    def test_feedback_requires_source(self) -> None:
        value = event("feedback")
        value["evidence"] = {}
        with self.assertRaises(MODULE.ExperienceError):
            MODULE.validate_event(value)

    def test_feedback_accepts_sim2sim_and_sim2real(self) -> None:
        MODULE.validate_event(event("feedback"))
        value = event("feedback")
        value["evidence"]["source"] = "sim2sim"
        MODULE.validate_event(value)

    def test_version_two_requires_valid_run_identity(self) -> None:
        value = version_two_event()
        MODULE.validate_event(value)
        del value["run_identity"]
        with self.assertRaisesRegex(MODULE.ExperienceError, "run_identity.version"):
            MODULE.validate_event(value)

    def test_version_two_scope_must_match_run_identity(self) -> None:
        value = version_two_event()
        value["run_id"] = "different-run"
        with self.assertRaisesRegex(MODULE.ExperienceError, "must match"):
            MODULE.validate_event(value)

    def test_version_one_remains_compatible(self) -> None:
        value = event()
        MODULE.validate_event(value)

    def test_new_writes_require_version_four_and_verified_config(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = history_root(directory)
            with self.assertRaisesRegex(MODULE.ExperienceError, "version 4"):
                MODULE.write_event(root, version_three_event(root))

            value = version_four_event(root)
            MODULE.validate_event(value)
            MODULE.validate_effective_config_binding(root, value)
            value["evidence"]["effective_config"]["sha256"] = "0" * 64
            with self.assertRaisesRegex(MODULE.ExperienceError, "SHA-256 mismatch"):
                MODULE.write_event(root, value)

    def test_version_four_reward_and_scope_binding_are_enforced(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = history_root(directory)
            value = version_four_event(root)
            reward_mismatch = copy.deepcopy(value)
            reward_mismatch["context"]["reward_fingerprint"] = "0" * 64
            with self.assertRaisesRegex(MODULE.ExperienceError, "reward_fingerprint"):
                MODULE.validate_event(reward_mismatch)

            reference = value["evidence"]["effective_config"]
            reference["path"] = str(
                root / value["task"] / "another-run" / "evidence" / "source" / "effective-config-x.json"
            )
            with self.assertRaisesRegex(MODULE.ExperienceError, "direct source artifact"):
                MODULE.write_event(root, value)

    def test_version_four_requires_explicit_event_and_outcome_availability(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = history_root(directory)
            value = version_four_event(root)
            del value["evidence"]["event"]
            with self.assertRaisesRegex(MODULE.ExperienceError, "declare"):
                MODULE.validate_event(value)

            value = version_four_event(root, "recommendation")
            value["evidence"]["outcome"] = {
                "status": "available",
                "baseline": {},
                "parameter_changes": {},
                "result_window": {},
                "observed_effect": {},
            }
            with self.assertRaisesRegex(MODULE.ExperienceError, "cannot declare"):
                MODULE.validate_event(value)

    def test_unavailable_version_four_evidence_is_recordable_but_incomplete(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = history_root(directory)
            value = version_four_event(root)
            current_config = MODULE.validate_effective_config_binding(root, value)
            status = MODULE.validate_event_evidence(
                root,
                value,
                current_config=current_config,
            )
            self.assertFalse(status["event_evidence_complete"])
            self.assertFalse(status["outcome_evidence_complete"])
            self.assertIn("event_evidence_unavailable", status["reasons"])
            self.assertTrue(Path(MODULE.write_event(root, value)["event_path"]).is_file())

    def test_type_specific_evidence_is_separate_from_outcome(self) -> None:
        validators = {
            "checkpoint_evaluation": "_validate_evaluation_reference",
            "checkpoint_selection": "_validate_selection_reference",
            "export": "_validate_export_reference",
            "archive": "_validate_archive_reference",
        }
        for event_type, validator in validators.items():
            with self.subTest(event_type=event_type), tempfile.TemporaryDirectory() as directory:
                root = history_root(directory)
                value = version_four_event(root, event_type)
                value["evidence"]["event"] = {
                    "status": "available",
                    "path": str(root / "placeholder.json"),
                    "sha256": "a" * 64,
                }
                current_config = MODULE.validate_effective_config_binding(root, value)
                with patch.object(MODULE, validator) as validate_reference:
                    status = MODULE.validate_event_evidence(
                        root,
                        value,
                        current_config=current_config,
                    )
                validate_reference.assert_called_once()
                self.assertTrue(status["event_evidence_complete"])
                self.assertFalse(status["outcome_evidence_complete"])
                self.assertIn(
                    (
                        "outcome_evidence_unavailable"
                        if event_type == "checkpoint_evaluation"
                        else "event_type_is_not_outcome_bearing"
                    ),
                    status["reasons"],
                )

    def test_rejects_symlinked_experience_root(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            parent = Path(directory).resolve()
            real_root = history_root(directory)
            value = version_four_event(real_root)
            linked_root = parent / "linked-root"
            linked_root.symlink_to(real_root, target_is_directory=True)
            with self.assertRaisesRegex(MODULE.ExperienceError, "symlinked"):
                MODULE.write_event(linked_root, value)

    def test_rejects_symlinked_run_component_before_creation(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory).resolve() / "events"
            root.mkdir()
            safe_root = Path(directory).resolve() / "safe" / "learnings" / "policy_tuning"
            safe_root.mkdir(parents=True)
            value = version_four_event(safe_root)
            external = Path(directory).resolve() / "external"
            external.mkdir()
            (root / "lw-leg-rough").symlink_to(external, target_is_directory=True)
            value["evidence"]["effective_config"]["path"] = str(
                root
                / value["task"]
                / value["run_id"]
                / "evidence"
                / "source"
                / "effective-config-snapshot-001.json"
            )
            with self.assertRaisesRegex(MODULE.ExperienceError, "symlinked"):
                MODULE.write_event(root, value)
            self.assertFalse((external / "run-001").exists())

    def test_receipt_output_is_exclusive(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory).resolve() / "receipt.json"
            MODULE._write_new_output(output, "{}\n")
            with self.assertRaisesRegex(MODULE.ExperienceError, "new absolute"):
                MODULE._write_new_output(output, "changed\n")
            self.assertEqual(output.read_text(encoding="utf-8"), "{}\n")


if __name__ == "__main__":
    unittest.main()
