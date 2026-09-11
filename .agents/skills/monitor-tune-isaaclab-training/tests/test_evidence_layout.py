#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))


def load_script(name: str):
    path = SCRIPT_DIR / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


LAYOUT = load_script("prepare_evidence_layout")
IDENTITY = load_script("capture_run_identity")
CONFIG = load_script("capture_effective_training_config")
EXPERIENCE = load_script("record_tuning_experience")


ENV_YAML = """\
seed: 42
scene:
  num_envs: 1
rewards:
  progress:
    func: example.rewards:progress
    weight: 1.0
"""

AGENT_YAML = """\
seed: 42
experiment_name: effective-config-test
class_name: OnPolicyRunnerAmpROA
algorithm:
  class_name: AMPROAPPO
"""


def experience_event(repository_root: Path, effective_path: Path) -> dict:
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
        "task": "task-a",
        "run_id": "run-001",
        "host_id": "younghit",
        "backend": "isaaclab",
        "algorithm": "AMP-ROA",
        "runner": "OnPolicyRunnerAmpROA",
        "seed": 42,
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
            "command": ["python", "train.py", "--task=task-a"],
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
    identity["identity_sha256"] = IDENTITY._sha256_bytes(
        IDENTITY._canonical_json(identity).encode("utf-8")
    )
    log_directory = (
        repository_root
        / "logs"
        / "rsl_rl"
        / "effective-config-test"
        / identity["run_id"]
    )
    params = log_directory / "params"
    params.mkdir(parents=True)
    (params / "env.yaml").write_text(ENV_YAML, encoding="utf-8")
    (params / "agent.yaml").write_text(AGENT_YAML, encoding="utf-8")
    config = CONFIG.capture_effective_config(identity, log_directory)
    receipt = CONFIG.write_new_evidence(effective_path, config)
    return {
        "version": 4,
        "event_id": "snapshot-001",
        "event_type": "run_snapshot",
        "recorded_at": "2026-08-01T18:00:00+08:00",
        "task": "task-a",
        "run_id": "run-001",
        "algorithm": "AMP-ROA",
        "context": {
            "observation_fingerprint": "unknown",
            "reward_fingerprint": config["fingerprints"]["reward"],
            "deployment_fingerprint": "unknown",
        },
        "parameters": {},
        "evidence": {
            "effective_config": {
                "path": str(effective_path),
                "sha256": receipt["sha256"],
                "effective_config_fingerprint": config["fingerprints"][
                    "effective_config"
                ],
                "reward_fingerprint": config["fingerprints"]["reward"],
            },
            "outcome": {
                "status": "unavailable",
                "reason": "layout separation fixture has no outcome comparison",
            },
        },
        "analysis": {"summary": "", "confidence": "low"},
        "next_suggestion": "",
        "run_identity": identity,
    }


class EvidenceLayoutTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary_directory.cleanup)
        self.repository_root = (
            Path(self.temporary_directory.name).resolve() / "robot_lab"
        )
        self.root = self.repository_root / "learnings" / "policy_tuning"

    def prepare(
        self,
        snapshot: str = "snapshot-001",
        evaluation: str | None = "eval-001",
        selection: str | None = "selection-001",
        export: str | None = "export-001",
    ) -> dict:
        return LAYOUT.prepare_evidence_layout(
            self.root,
            task="task-a",
            run_id="run-001",
            snapshot_id=snapshot,
            evaluation_id=evaluation,
            selection_id=selection,
            export_id=export,
        )

    def test_returns_deterministic_absolute_paths_and_creates_only_directories(self) -> None:
        layout = self.prepare()
        self.assertEqual(layout, self.prepare())
        for path in layout["paths"].values():
            self.assertTrue(Path(path).is_absolute())
            self.assertFalse(Path(path).exists())
        self.assertTrue(Path(layout["evidence_root"]).is_dir())
        self.assertEqual(
            Path(layout["paths"]["health"]),
            self.root / "task-a" / "run-001" / "evidence" / "health" / "health-snapshot-001.json",
        )

    def test_snapshots_and_evaluations_do_not_conflict(self) -> None:
        first = self.prepare("snapshot-001", "eval-001")
        second = self.prepare(
            "snapshot-002",
            "eval-002",
            "selection-002",
            "export-002",
        )
        for name in (
            "criteria",
            "health",
            "source_identity",
            "effective_config",
            "source_patch",
            "summary",
            "assessment",
            "play_result",
            "telemetry",
            "video",
            "checkpoint_selection",
            "export_jit",
            "export_onnx",
            "export_receipt",
        ):
            self.assertNotEqual(first["paths"][name], second["paths"][name])

    def test_optional_evaluation_omits_play_targets(self) -> None:
        layout = self.prepare(evaluation=None)
        for name in ("play_result", "telemetry", "video"):
            self.assertIsNone(layout["paths"][name])
        self.assertIsNone(layout["directories"]["evaluation"])
        self.assertIn(
            "unset PLAY_RESULT_PATH TELEMETRY_PATH VIDEO_PATH",
            LAYOUT._shell_assignments(layout),
        )

    def test_optional_selection_and_export_unset_all_outputs(self) -> None:
        layout = self.prepare(selection=None, export=None)
        for name in (
            "checkpoint_selection",
            "export_jit",
            "export_onnx",
            "export_receipt",
        ):
            self.assertIsNone(layout["paths"][name])
        self.assertIn(
            "unset CHECKPOINT_SELECTION_PATH EXPORT_JIT_PATH EXPORT_ONNX_PATH "
            "EXPORT_RECEIPT_PATH",
            LAYOUT._shell_assignments(layout),
        )

    def test_selection_and_export_paths_are_canonical(self) -> None:
        layout = self.prepare()
        evidence = Path(layout["evidence_root"])
        self.assertEqual(
            Path(layout["paths"]["checkpoint_selection"]),
            evidence / "checkpoint_selection" / "selection-selection-001.json",
        )
        self.assertEqual(
            Path(layout["paths"]["export_receipt"]),
            evidence / "export" / "export-001" / "receipt.json",
        )

    def test_rejects_unsafe_identifiers(self) -> None:
        invalid_values = ("../escape", "nested/name", ".", "..", " hidden", "非ascii")
        for value in invalid_values:
            with self.subTest(value=value), self.assertRaisesRegex(
                LAYOUT.EvidenceLayoutError,
                "safe ASCII identifier",
            ):
                LAYOUT.prepare_evidence_layout(
                    self.root,
                    task=value,
                    run_id="run-001",
                    snapshot_id="snapshot-001",
                )

    def test_rejects_symlinked_path_component(self) -> None:
        self.root.mkdir(parents=True)
        real_task = self.root / "real-task"
        real_task.mkdir()
        (self.root / "task-a").symlink_to(real_task, target_is_directory=True)
        with self.assertRaisesRegex(LAYOUT.EvidenceLayoutError, "symlinked path component"):
            self.prepare()

    def test_rejects_existing_evidence_target(self) -> None:
        layout = self.prepare()
        Path(layout["paths"]["health"]).write_text("{}\n", encoding="utf-8")
        with self.assertRaisesRegex(LAYOUT.EvidenceLayoutError, "already exists"):
            self.prepare()

    def test_raw_evidence_and_immutable_events_have_separate_roots(self) -> None:
        layout = self.prepare()
        receipt = EXPERIENCE.write_event(
            self.root,
            experience_event(
                self.repository_root,
                Path(layout["paths"]["effective_config"]),
            ),
        )
        event_path = Path(receipt["event_path"])
        run_root = Path(layout["run_root"])
        evidence_root = Path(layout["evidence_root"])
        self.assertEqual(event_path.parent, run_root)
        self.assertEqual(evidence_root.parent, run_root)
        self.assertNotIn(evidence_root, event_path.parents)

    def test_effective_config_path_is_source_evidence(self) -> None:
        layout = self.prepare()
        self.assertEqual(
            Path(layout["paths"]["effective_config"]),
            self.root
            / "task-a"
            / "run-001"
            / "evidence"
            / "source"
            / "effective-config-snapshot-001.json",
        )
        self.assertIn(
            f"EFFECTIVE_CONFIG_PATH={layout['paths']['effective_config']}",
            LAYOUT._shell_assignments(layout),
        )


if __name__ == "__main__":
    unittest.main()
