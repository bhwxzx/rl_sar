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
from unittest.mock import patch


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))
SCRIPT = SCRIPT_DIR / "capture_effective_training_config.py"
SPEC = importlib.util.spec_from_file_location("capture_effective_training_config", SCRIPT)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)

import capture_run_identity as IDENTITY  # noqa: E402


ENV_YAML = """\
seed: 42
scene:
  num_envs: 4096
rewards:
  track_velocity:
    func: example.rewards:track_velocity
    params:
      std: 0.5
      dimensions: !!python/tuple
      - 0
      - 2
    weight: 5.0
  action_rate:
    func: example.rewards:action_rate
    params: {}
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
  learning_rate: 0.001
"""


def run_identity(
    repository_root: Path,
    *,
    seed: int = 42,
    runner: str = "OnPolicyRunnerAmpROA",
    run_id: str = "2026-08-03_12-00-00",
) -> dict:
    scenario = {
        "scenario_id": "quick-native",
        "scenario_overrides": {},
        "command_schedule": [],
        "duration_steps": 500,
        "num_envs": 1,
        "seed": seed,
    }
    value = {
        "version": 1,
        "task": "lw-leg-rough",
        "run_id": run_id,
        "host_id": "younghit",
        "backend": "isaaclab",
        "algorithm": "amp-roa",
        "runner": runner,
        "seed": seed,
        "source": {
            "repository_root": str(repository_root),
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
        "config_files": [{"path": "config.py", "sha256": "2" * 64}],
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


class EffectiveTrainingConfigTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary_directory.cleanup)
        self.repository_root = Path(self.temporary_directory.name).resolve() / "robot_lab"
        self.log_directory = (
            self.repository_root
            / "logs"
            / "rsl_rl"
            / "effective-config-test"
            / "2026-08-03_12-00-00"
        )
        self.params_directory = self.log_directory / "params"
        self.params_directory.mkdir(parents=True)
        self.env_path = self.params_directory / "env.yaml"
        self.agent_path = self.params_directory / "agent.yaml"
        self.env_path.write_text(ENV_YAML, encoding="utf-8")
        self.agent_path.write_text(AGENT_YAML, encoding="utf-8")
        self.identity = run_identity(self.repository_root)

    def test_captures_full_config_reward_weights_and_deterministic_fingerprints(self) -> None:
        first = MODULE.capture_effective_config(self.identity, self.log_directory)
        second = MODULE.capture_effective_config(self.identity, self.log_directory)
        self.assertEqual(first, second)
        self.assertEqual(
            first["reward_weights"],
            {"action_rate": -0.3, "disabled_term": None, "track_velocity": 5.0},
        )
        self.assertFalse(first["reward_terms"]["disabled_term"]["enabled"])
        self.assertEqual(first["training_parameters"]["max_iterations"], 100000)
        self.assertEqual(first["training_parameters"]["num_envs"], 4096)
        self.assertEqual(first["resolved_identity"]["algorithm_class"], "AMPROAPPO")
        self.assertEqual(
            first["source_files"]["environment"]["content_utf8"],
            ENV_YAML,
        )
        self.assertEqual(
            first["source_files"]["environment"]["sha256"],
            hashlib.sha256(ENV_YAML.encode("utf-8")).hexdigest(),
        )
        for fingerprint in first["fingerprints"].values():
            self.assertRegex(fingerprint, r"^[0-9a-f]{64}$")

        self.env_path.write_text(
            ENV_YAML.replace("weight: -0.3", "weight: -0.4"),
            encoding="utf-8",
        )
        changed = MODULE.capture_effective_config(self.identity, self.log_directory)
        self.assertEqual(
            changed["fingerprints"]["agent"],
            first["fingerprints"]["agent"],
        )
        for name in ("environment", "reward", "effective_config"):
            self.assertNotEqual(
                changed["fingerprints"][name],
                first["fingerprints"][name],
            )

    def test_publishes_one_new_immutable_evidence_file(self) -> None:
        output = self.repository_root / "evidence" / "effective-config.json"
        output.parent.mkdir()
        evidence = MODULE.capture_effective_config(self.identity, self.log_directory)
        receipt = MODULE.write_new_evidence(output, evidence)
        self.assertEqual(json.loads(output.read_text(encoding="utf-8")), evidence)
        self.assertEqual(receipt["evidence_path"], str(output))
        self.assertRegex(receipt["sha256"], r"^[0-9a-f]{64}$")
        with self.assertRaisesRegex(MODULE.EffectiveConfigError, "new absolute"):
            MODULE.write_new_evidence(output, evidence)
        self.assertEqual(list(output.parent.glob(".*.tmp-*")), [])

    def test_reloads_and_recomputes_effective_config_evidence(self) -> None:
        output = self.repository_root / "evidence" / "effective-config.json"
        output.parent.mkdir()
        evidence = MODULE.capture_effective_config(self.identity, self.log_directory)
        receipt = MODULE.write_new_evidence(output, evidence)
        loaded, metadata = MODULE.load_and_validate_effective_config(
            output,
            expected_sha256=receipt["sha256"],
            run_identity=self.identity,
        )
        self.assertEqual(loaded, evidence)
        self.assertEqual(metadata["sha256"], receipt["sha256"])

        tampered = json.loads(output.read_text(encoding="utf-8"))
        tampered["training_parameters"]["max_iterations"] = 7
        tampered_path = output.parent / "tampered.json"
        encoded = json.dumps(tampered, sort_keys=True).encode("utf-8")
        tampered_path.write_bytes(encoded)
        with self.assertRaisesRegex(MODULE.EffectiveConfigError, "training_parameters"):
            MODULE.load_and_validate_effective_config(
                tampered_path,
                expected_sha256=hashlib.sha256(encoded).hexdigest(),
                run_identity=self.identity,
            )

    def test_compares_complete_semantic_reward_and_training_changes(self) -> None:
        baseline = MODULE.capture_effective_config(self.identity, self.log_directory)
        self.env_path.write_text(
            ENV_YAML.replace("weight: -0.3", "weight: -0.4"),
            encoding="utf-8",
        )
        self.agent_path.write_text(
            AGENT_YAML.replace("max_iterations: 100000", "max_iterations: 90000"),
            encoding="utf-8",
        )
        current = MODULE.capture_effective_config(self.identity, self.log_directory)
        first = MODULE.compare_effective_configs(baseline, current)
        second = MODULE.compare_effective_configs(baseline, current)
        self.assertEqual(first, second)
        self.assertTrue(first["complete"])
        self.assertEqual(
            first["reward_weight_changes"],
            [
                {
                    "path": "action_rate",
                    "change": "changed",
                    "before": -0.3,
                    "after": -0.4,
                }
            ],
        )
        self.assertEqual(
            first["training_parameter_changes"],
            [
                {
                    "path": "max_iterations",
                    "change": "changed",
                    "before": 100000,
                    "after": 90000,
                }
            ],
        )
        self.assertEqual(
            {item["path"] for item in first["semantic_changes"]},
            {
                "/agent/max_iterations",
                "/environment/rewards/action_rate/weight",
            },
        )
        with self.assertRaisesRegex(MODULE.EffectiveConfigError, "max-diff-entries=1"):
            MODULE.compare_effective_configs(
                baseline,
                current,
                max_diff_entries=1,
            )

    def test_rejects_seed_and_runner_identity_mismatch(self) -> None:
        with self.assertRaisesRegex(MODULE.EffectiveConfigError, "seed"):
            MODULE.capture_effective_config(
                run_identity(self.repository_root, seed=7),
                self.log_directory,
            )
        with self.assertRaisesRegex(MODULE.EffectiveConfigError, "runner"):
            MODULE.capture_effective_config(
                run_identity(self.repository_root, runner="OnPolicyRunner"),
                self.log_directory,
            )
        with self.assertRaisesRegex(MODULE.EffectiveConfigError, "run_id"):
            MODULE.capture_effective_config(
                run_identity(self.repository_root, run_id="another-run"),
                self.log_directory,
            )
        missing_task = run_identity(self.repository_root)
        missing_task["training"]["command"] = ["python", "train.py"]
        missing_task.pop("identity_sha256")
        missing_task["identity_sha256"] = IDENTITY._sha256_bytes(
            IDENTITY._canonical_json(missing_task).encode("utf-8")
        )
        with self.assertRaisesRegex(MODULE.EffectiveConfigError, "exact task"):
            MODULE.capture_effective_config(missing_task, self.log_directory)

        self.agent_path.write_text(
            AGENT_YAML.replace(
                "experiment_name: effective-config-test",
                "experiment_name: another-experiment",
            ),
            encoding="utf-8",
        )
        with self.assertRaisesRegex(MODULE.EffectiveConfigError, "experiment_name"):
            MODULE.capture_effective_config(self.identity, self.log_directory)

    def test_rejects_duplicate_keys_and_non_finite_weights(self) -> None:
        self.env_path.write_text(
            ENV_YAML.replace("seed: 42\n", "seed: 42\nseed: 42\n", 1),
            encoding="utf-8",
        )
        with self.assertRaisesRegex(MODULE.EffectiveConfigError, "duplicate key"):
            MODULE.capture_effective_config(self.identity, self.log_directory)

        self.env_path.write_text(
            ENV_YAML.replace("weight: 5.0", "weight: .nan"),
            encoding="utf-8",
        )
        with self.assertRaisesRegex(MODULE.EffectiveConfigError, "non-finite"):
            MODULE.capture_effective_config(self.identity, self.log_directory)

    def test_rejects_symlinked_source_and_out_of_scope_log_directory(self) -> None:
        linked_env = self.params_directory / "linked-env.yaml"
        linked_env.symlink_to(self.env_path)
        self.env_path.rename(self.params_directory / "real-env.yaml")
        (self.params_directory / "env.yaml").symlink_to(linked_env)
        with self.assertRaisesRegex(MODULE.EffectiveConfigError, "symlinked"):
            MODULE.capture_effective_config(self.identity, self.log_directory)

        outside = self.repository_root / "outside-run"
        (outside / "params").mkdir(parents=True)
        with self.assertRaisesRegex(MODULE.EffectiveConfigError, "logs/rsl_rl"):
            MODULE.capture_effective_config(self.identity, outside)
        noncanonical = self.log_directory / ".." / self.log_directory.name
        with self.assertRaisesRegex(MODULE.EffectiveConfigError, "canonical"):
            MODULE.capture_effective_config(self.identity, noncanonical)

    def test_rejects_source_change_during_paired_capture(self) -> None:
        original_read = MODULE._read_stable_utf8

        def mutate_between_reads(path: Path, *, label: str):
            result = original_read(path, label=label)
            if label == "agent config":
                self.env_path.write_text(ENV_YAML + "changed: true\n", encoding="utf-8")
            return result

        with patch.object(MODULE, "_read_stable_utf8", side_effect=mutate_between_reads):
            with self.assertRaisesRegex(MODULE.EffectiveConfigError, "paired capture"):
                MODULE.capture_effective_config(self.identity, self.log_directory)

    def test_cli_binds_identity_and_writes_receipt(self) -> None:
        identity_path = self.repository_root / "identity.json"
        identity_path.write_text(json.dumps(self.identity), encoding="utf-8")
        output = self.repository_root / "effective-config.json"
        stdout = io.StringIO()
        argv = [
            str(SCRIPT),
            str(identity_path),
            "--log-dir",
            str(self.log_directory),
            "--output",
            str(output),
        ]
        with patch.object(sys, "argv", argv), contextlib.redirect_stdout(stdout):
            self.assertEqual(MODULE.main(), 0)
        receipt = json.loads(stdout.getvalue())
        self.assertEqual(receipt["evidence_path"], str(output))
        self.assertEqual(
            json.loads(output.read_text(encoding="utf-8"))["run_identity_sha256"],
            self.identity["identity_sha256"],
        )


if __name__ == "__main__":
    unittest.main()
