#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


SKILL_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_DIR = SKILL_ROOT / "scripts"
REPO_ROOT = Path(__file__).resolve().parents[4]
RSL_RL_DIR = REPO_ROOT / "scripts" / "reinforcement_learning" / "rsl_rl"
sys.path.insert(0, str(SCRIPT_DIR))
sys.path.insert(0, str(RSL_RL_DIR))


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


EXPORT = load_module("policy_export_evidence", SCRIPT_DIR / "policy_export_evidence.py")
LAYOUT = load_module("policy_export_layout", SCRIPT_DIR / "prepare_evidence_layout.py")
IDENTITY = load_module("policy_export_identity", SCRIPT_DIR / "capture_run_identity.py")
CONFIG = load_module(
    "policy_export_effective_config",
    SCRIPT_DIR / "capture_effective_training_config.py",
)
EVALUATION = load_module(
    "policy_export_evaluation",
    RSL_RL_DIR / "policy_evaluation_evidence.py",
)
ARCHIVE = load_module(
    "policy_export_archive",
    SCRIPT_DIR / "archive_advised_policy.py",
)


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
experiment_name: export-test
class_name: OnPolicyRunnerAmpROA
algorithm:
  class_name: AMPROAPPO
"""


def write_json(path: Path, value: dict) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return EXPORT.sha256_file(path)


class PolicyExportEvidenceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.repo_root = Path(self.temporary.name).resolve() / "robot_lab"
        self.task = "task-a"
        self.run_id = "run-001"
        self.selection_id = "selection-001"
        self.export_id = "export-001"
        self.layout = LAYOUT.prepare_evidence_layout(
            self.repo_root / "learnings" / "policy_tuning",
            task=self.task,
            run_id=self.run_id,
            snapshot_id="snapshot-001",
            evaluation_id="eval-001",
            selection_id=self.selection_id,
            export_id=self.export_id,
        )
        self.scenario = {
            "scenario_id": "quick-native",
            "scenario_overrides": {},
            "command_schedule": [],
            "duration_steps": 8,
            "num_envs": 1,
            "seed": 42,
        }
        self.identity_path = Path(self.layout["paths"]["source_identity"])
        self.identity = {
            "version": 1,
            "task": self.task,
            "run_id": self.run_id,
            "host_id": "host-a",
            "backend": "isaaclab",
            "algorithm": "AMP-ROA",
            "runner": "OnPolicyRunnerAmpROA",
            "seed": 42,
            "source": {
                "repository_root": str(self.repo_root),
                "branch": "main",
                "head": "1" * 40,
                "dirty": False,
                "dirty_paths": [],
                "diff_sha256": None,
                "patch_evidence": None,
            },
            "training": {
                "command": ["python", "train.py", f"--task={self.task}"],
                "hydra_overrides": [],
            },
            "config_files": [{"path": "config.py", "sha256": "2" * 64}],
            "evaluation_scenario": {
                "contract": self.scenario,
                "sha256": IDENTITY._sha256_bytes(
                    IDENTITY._canonical_json(self.scenario).encode("utf-8")
                ),
            },
        }
        self.identity["identity_sha256"] = IDENTITY._sha256_bytes(
            IDENTITY._canonical_json(self.identity).encode("utf-8")
        )
        self.identity_file_sha256 = write_json(self.identity_path, self.identity)

        self.log_dir = (
            self.repo_root / "logs" / "rsl_rl" / "export-test" / self.run_id
        )
        params = self.log_dir / "params"
        params.mkdir(parents=True)
        (params / "env.yaml").write_text(ENV_YAML, encoding="utf-8")
        (params / "agent.yaml").write_text(AGENT_YAML, encoding="utf-8")
        self.checkpoint = self.log_dir / "model_10.pt"
        self.checkpoint.write_bytes(b"checkpoint")
        self.checkpoint_sha256 = EXPORT.sha256_file(self.checkpoint)
        self.effective_path = Path(self.layout["paths"]["effective_config"])
        config = CONFIG.capture_effective_config(self.identity, self.log_dir)
        self.effective_receipt = CONFIG.write_new_evidence(self.effective_path, config)

        self.evaluation_path = Path(self.layout["paths"]["play_result"])
        evaluation_plan = EVALUATION.preflight_evaluation(
            repo_root=self.repo_root,
            task=self.task,
            run_id=self.run_id,
            evaluation_id="eval-001",
            result_path=self.evaluation_path,
            telemetry_path=Path(self.layout["paths"]["telemetry"]),
            video_path=None,
            checkpoint_path=self.checkpoint,
            checkpoint_sha256=self.checkpoint_sha256,
            artifact_kind="native",
            artifact_path=self.checkpoint,
            artifact_sha256=self.checkpoint_sha256,
            run_identity_path=self.identity_path,
            run_identity_file_sha256=self.identity_file_sha256,
            scenario_contract=self.scenario,
        )
        evaluation_binding = {
            "task": self.task,
            "run_id": self.run_id,
            "evaluation_id": "eval-001",
            "candidate_id": "model_10",
            "runner": self.identity["runner"],
        }
        evaluation_inputs = {
            "checkpoint": evaluation_plan.checkpoint,
            "artifact": {"kind": "native", **evaluation_plan.artifact},
            "run_identity": evaluation_plan.run_identity,
            "scenario": {
                "contract": self.scenario,
                "sha256": evaluation_plan.scenario_sha256,
            },
            "resource_mode": {
                "training_overlap": False,
                "idle_gpu_required": True,
                "video_requested": False,
                "telemetry_requested": True,
            },
        }
        evaluation_result = {
            "version": 2,
            "status": "completed",
            "evaluation": evaluation_binding,
            "inputs": evaluation_inputs,
            "telemetry_status": "complete",
        }
        telemetry = {
            "version": 3,
            "evaluation": evaluation_binding,
            "inputs": evaluation_inputs,
            "telemetry_status": "complete",
            "samples": [],
        }
        with EVALUATION.EvaluationPublisher(evaluation_plan) as publisher:
            publisher.publish(evaluation_result, telemetry=telemetry, video_source=None)

        self.report_path = self.repo_root / "selection-report.json"
        self.report_sha256 = write_json(
            self.report_path,
            {
                "version": 1,
                "advisory_only": True,
                "inventory": [
                    {
                        "path": str(self.checkpoint),
                        "step": 10,
                        "stable": True,
                        "sha256": self.checkpoint_sha256,
                    }
                ],
                "comparison": {"pending_user_selection": True},
            },
        )
        self.selection_path = Path(self.layout["paths"]["checkpoint_selection"])
        self.tensor_contract = EXPORT.expected_tensor_contract(self.identity["runner"])
        self.selection_metadata = EXPORT.record_checkpoint_selection(
            selection_id=self.selection_id,
            approved_at="2026-08-03T18:00:00+08:00",
            checkpoint_id="model_10",
            checkpoint_path=self.checkpoint,
            checkpoint_sha256=self.checkpoint_sha256,
            selection_report_path=self.report_path,
            selection_report_sha256=self.report_sha256,
            run_identity_path=self.identity_path,
            run_identity_file_sha256=self.identity_file_sha256,
            effective_config_path=self.effective_path,
            effective_config_sha256=self.effective_receipt["sha256"],
            evaluation_result_paths=[self.evaluation_path],
            tensor_contract=self.tensor_contract,
            output_path=self.selection_path,
        )

    def plan(self, **overrides):
        values = {
            "task": self.task,
            "run_id": self.run_id,
            "checkpoint_id": "model_10",
            "checkpoint_path": self.checkpoint,
            "checkpoint_sha256": self.checkpoint_sha256,
            "export_id": self.export_id,
            "selection_receipt_path": self.selection_path,
            "selection_receipt_sha256": self.selection_metadata[
                "selection_receipt"
            ]["sha256"],
            "jit_path": Path(self.layout["paths"]["export_jit"]),
            "onnx_path": Path(self.layout["paths"]["export_onnx"]),
            "receipt_path": Path(self.layout["paths"]["export_receipt"]),
            "history_contract": self.tensor_contract["history_contract"],
            "normalization_contract": self.tensor_contract[
                "normalization_contract"
            ],
            "reset_contract": self.tensor_contract["reset_contract"],
            "onnx_export_profile": "static_batch_1_simplified",
            "parity_steps": 4,
            "reset_step": 2,
            "minimum_parity_samples": 4,
            "max_abs_action_error": 1.0e-5,
            "num_envs": 1,
            "seed": 42,
        }
        values.update(overrides)
        return EXPORT.preflight_export(**values)

    def receipt(self, plan) -> dict:
        static_batch = plan.onnx_export_contract["batch_contract"] == "static_batch_1"
        batch_dimension = 1 if static_batch else "batch"
        simplified = plan.onnx_export_contract["simplified"]
        return {
            "version": 4,
            "status": "completed",
            "export": {
                "task": plan.task,
                "run_id": plan.run_id,
                "export_id": plan.export_id,
                "checkpoint_id": plan.checkpoint["checkpoint_id"],
                "runner": plan.selection_receipt["runner"],
            },
            "inputs": {
                "checkpoint": plan.checkpoint,
                "checkpoint_selection": {
                    "path": str(self.selection_path),
                    "sha256": self.selection_metadata["selection_receipt"]["sha256"],
                    "selection_id": self.selection_id,
                },
                "run_identity": plan.selection_receipt["run_identity"],
                "effective_config": plan.selection_receipt["effective_config"],
                "tensor_contract": plan.tensor_contract,
                "onnx_export_contract": plan.onnx_export_contract,
                "parity_contract": plan.parity_contract,
            },
            "onnx_export": {
                "contract": plan.onnx_export_contract,
                "input_shape": [batch_dimension, 48],
                "output_shape": [batch_dimension, 12],
                "pre_simplify_node_count": 40,
                "post_simplify_node_count": 22 if simplified else 40,
                "simplifier_check": True if simplified else None,
            },
            "parity": {
                "sample_count": 4,
                "boundaries": [
                    {
                        "label": label,
                        "step": step,
                        "observation_sha256": "4" * 64,
                        "native_output_sha256": "5" * 64,
                        "input_shape": [1, 48],
                        "output_shape": [1, 12],
                    }
                    for label, step in (
                        ("initial", 0),
                        ("pre_reset", 1),
                        ("post_reset", 2),
                        ("final", 3),
                    )
                ],
                "observation_batch_sha256": "6" * 64,
                "native_output_sha256": "7" * 64,
                "native_device_to_cpu_max_abs_action_error": 0.0,
                "jit": {
                    "finite": True,
                    "max_abs_action_error": 1.0e-7,
                    "input_shape": [4, 48],
                    "output_shape": [4, 12],
                },
                "onnx": {
                    "finite": True,
                    "max_abs_action_error": 1.0e-7,
                    "input_shape": [4, 48],
                    "output_shape": [4, 12],
                },
            },
        }

    def test_selection_export_publish_and_revalidate(self) -> None:
        plan = self.plan()
        with EXPORT.ExportPublisher(plan) as publisher:
            publisher.jit_work_path.write_bytes(b"jit")
            publisher.onnx_work_path.write_bytes(b"onnx")
            published = publisher.publish(self.receipt(plan))
        self.assertEqual(published["version"], 4)
        validation = EXPORT.validate_export_bundle(plan.paths["receipt"])
        self.assertEqual(validation["status"], "valid")
        self.assertFalse((plan.paths["export_dir"] / ".attempt").exists())

    def test_legacy_version_3_receipt_remains_valid(self) -> None:
        plan = self.plan()
        receipt = self.receipt(plan)
        receipt["version"] = 3
        del receipt["inputs"]["onnx_export_contract"]
        del receipt["onnx_export"]
        with EXPORT.ExportPublisher(plan) as publisher:
            publisher.jit_work_path.write_bytes(b"jit")
            publisher.onnx_work_path.write_bytes(b"onnx")
            publisher.publish(receipt)
        validation = EXPORT.validate_export_bundle(plan.paths["receipt"])
        self.assertEqual(validation["document"]["version"], 3)

    def test_dynamic_batch_version_4_receipt_is_valid(self) -> None:
        plan = self.plan(onnx_export_profile="dynamic_batch")
        with EXPORT.ExportPublisher(plan) as publisher:
            publisher.jit_work_path.write_bytes(b"jit")
            publisher.onnx_work_path.write_bytes(b"onnx")
            publisher.publish(self.receipt(plan))
        validation = EXPORT.validate_export_bundle(plan.paths["receipt"])
        self.assertEqual(
            validation["document"]["onnx_export"]["input_shape"],
            ["batch", 48],
        )

    def test_version_4_missing_onnx_contract_rolls_back_outputs(self) -> None:
        plan = self.plan()
        receipt = self.receipt(plan)
        del receipt["inputs"]["onnx_export_contract"]
        with EXPORT.ExportPublisher(plan) as publisher:
            publisher.jit_work_path.write_bytes(b"jit")
            publisher.onnx_work_path.write_bytes(b"onnx")
            with self.assertRaisesRegex(
                EXPORT.PolicyExportEvidenceError, "publication failed"
            ):
                publisher.publish(receipt)
        for label in ("jit", "onnx", "receipt"):
            self.assertFalse(plan.paths[label].exists())

    def test_version_4_tampered_onnx_evidence_rolls_back_outputs(self) -> None:
        plan = self.plan()
        receipt = self.receipt(plan)
        receipt["onnx_export"]["input_shape"] = ["batch", 48]
        with EXPORT.ExportPublisher(plan) as publisher:
            publisher.jit_work_path.write_bytes(b"jit")
            publisher.onnx_work_path.write_bytes(b"onnx")
            with self.assertRaisesRegex(
                EXPORT.PolicyExportEvidenceError, "publication failed"
            ):
                publisher.publish(receipt)
        for label in ("jit", "onnx", "receipt"):
            self.assertFalse(plan.paths[label].exists())

    def test_checkpoint_id_and_tensor_contract_conflicts_are_rejected(self) -> None:
        with self.assertRaisesRegex(EXPORT.PolicyExportEvidenceError, "checkpoint_id"):
            self.plan(checkpoint_id="model_11")
        with self.assertRaisesRegex(EXPORT.PolicyExportEvidenceError, "tensor contract"):
            self.plan(history_contract="current_observation")
        with self.assertRaisesRegex(EXPORT.PolicyExportEvidenceError, "unsupported ONNX"):
            self.plan(onnx_export_profile="implicit-default")

    def test_wrong_output_and_existing_target_are_rejected(self) -> None:
        with self.assertRaisesRegex(EXPORT.PolicyExportEvidenceError, "outside"):
            self.plan(jit_path=self.repo_root / "policy.pt")
        Path(self.layout["paths"]["export_jit"]).write_bytes(b"existing")
        with self.assertRaisesRegex(EXPORT.PolicyExportEvidenceError, "already exists"):
            self.plan()

    def test_selection_source_drift_is_rejected(self) -> None:
        self.report_path.write_text("{}\n", encoding="utf-8")
        with self.assertRaisesRegex(EXPORT.PolicyExportEvidenceError, "SHA-256"):
            self.plan()

    def test_symlinked_export_directory_is_rejected(self) -> None:
        export_dir = Path(self.layout["directories"]["export_attempt"])
        external = self.repo_root / "external-export"
        external.mkdir()
        export_dir.rmdir()
        export_dir.symlink_to(external, target_is_directory=True)
        with self.assertRaisesRegex(EXPORT.PolicyExportEvidenceError, "symlinked"):
            self.plan()

    def test_mid_attempt_failure_has_no_completed_outputs(self) -> None:
        plan = self.plan()
        with self.assertRaisesRegex(RuntimeError, "export failed"):
            with EXPORT.ExportPublisher(plan) as publisher:
                publisher.jit_work_path.write_bytes(b"partial")
                raise RuntimeError("export failed")
        for label in ("jit", "onnx", "receipt"):
            self.assertFalse(plan.paths[label].exists())

    def test_concurrent_claim_is_rejected(self) -> None:
        plan = self.plan()
        with EXPORT.ExportPublisher(plan):
            with self.assertRaisesRegex(
                EXPORT.PolicyExportEvidenceError, "already claimed"
            ):
                with EXPORT.ExportPublisher(plan):
                    self.fail("second publisher acquired the claim")

    def test_publication_collision_rolls_back_only_owned_link(self) -> None:
        plan = self.plan()
        with EXPORT.ExportPublisher(plan) as publisher:
            publisher.jit_work_path.write_bytes(b"jit")
            publisher.onnx_work_path.write_bytes(b"onnx")
            real_link = os.link

            def racing_link(source, target, *, follow_symlinks=False):
                real_link(source, target, follow_symlinks=follow_symlinks)
                if target == plan.paths["jit"]:
                    plan.paths["onnx"].write_bytes(b"external")

            with self.assertRaisesRegex(
                EXPORT.PolicyExportEvidenceError, "publication failed"
            ):
                with patch.object(EXPORT.os, "link", side_effect=racing_link):
                    publisher.publish(self.receipt(plan))
        self.assertFalse(plan.paths["jit"].exists())
        self.assertFalse(plan.paths["receipt"].exists())
        self.assertEqual(plan.paths["onnx"].read_bytes(), b"external")

    def test_receipt_is_last_published_target(self) -> None:
        plan = self.plan()
        targets: list[Path] = []
        real_link = os.link

        def recording_link(source, target, *, follow_symlinks=False):
            targets.append(Path(target))
            return real_link(source, target, follow_symlinks=follow_symlinks)

        with EXPORT.ExportPublisher(plan) as publisher:
            publisher.jit_work_path.write_bytes(b"jit")
            publisher.onnx_work_path.write_bytes(b"onnx")
            with patch.object(EXPORT.os, "link", side_effect=recording_link):
                publisher.publish(self.receipt(plan))
        self.assertEqual(targets[-1], plan.paths["receipt"])

    def test_checkpoint_drift_before_completion_rolls_back_outputs(self) -> None:
        plan = self.plan()
        with EXPORT.ExportPublisher(plan) as publisher:
            publisher.jit_work_path.write_bytes(b"jit")
            publisher.onnx_work_path.write_bytes(b"onnx")
            self.checkpoint.write_bytes(b"changed-after-preflight")
            with self.assertRaisesRegex(
                EXPORT.PolicyExportEvidenceError, "publication failed"
            ):
                publisher.publish(self.receipt(plan))
        for label in ("jit", "onnx", "receipt"):
            self.assertFalse(plan.paths[label].exists())

    def test_missing_reset_boundary_rolls_back_all_outputs(self) -> None:
        plan = self.plan()
        receipt = self.receipt(plan)
        receipt["parity"]["boundaries"] = [{"label": "initial", "step": 0}]
        with EXPORT.ExportPublisher(plan) as publisher:
            publisher.jit_work_path.write_bytes(b"jit")
            publisher.onnx_work_path.write_bytes(b"onnx")
            with self.assertRaisesRegex(
                EXPORT.PolicyExportEvidenceError, "publication failed"
            ):
                publisher.publish(receipt)
        for label in ("jit", "onnx", "receipt"):
            self.assertFalse(plan.paths[label].exists())

    def test_validated_export_archives_in_a_temporary_git_repository(self) -> None:
        plan = self.plan()
        with EXPORT.ExportPublisher(plan) as publisher:
            publisher.jit_work_path.write_bytes(b"jit")
            publisher.onnx_work_path.write_bytes(b"onnx")
            published = publisher.publish(self.receipt(plan))
        storage = self.repo_root / "policy_storage"
        collection = storage / "LW" / "leg_loco"
        collection.mkdir(parents=True)
        subprocess.run(["git", "init", "-q", str(storage)], check=True)
        subprocess.run(
            ["git", "-C", str(storage), "config", "user.email", "test@example.invalid"],
            check=True,
        )
        subprocess.run(
            ["git", "-C", str(storage), "config", "user.name", "Test"],
            check=True,
        )
        (collection / ".gitkeep").write_text("\n", encoding="utf-8")
        subprocess.run(["git", "-C", str(storage), "add", "."], check=True)
        subprocess.run(
            ["git", "-C", str(storage), "commit", "-qm", "init"], check=True
        )
        manifest = {
            "version": 2,
            "archive_authorized": True,
            "storage_root": str(storage),
            "collection": "LW/leg_loco",
            "task": self.task,
            "algorithm": self.identity["algorithm"],
            "runner": self.identity["runner"],
            "selected_checkpoint": {
                "path": str(self.checkpoint),
                "sha256": self.checkpoint_sha256,
                "iteration": 10,
            },
            "artifacts": {
                kind: {
                    "path": published["outputs"][kind]["path"],
                    "sha256": published["outputs"][kind]["sha256"],
                }
                for kind in ("jit", "onnx")
            },
            "source": {"commit": "1" * 40, "dirty": False},
            "parameters": {},
            "evaluation": {
                "results": [
                    {"path": item["path"], "sha256": item["sha256"]}
                    for item in plan.selection_receipt["evaluation_results"]
                ]
            },
            "export_receipt": {
                "path": str(plan.paths["receipt"]),
                "sha256": EXPORT.sha256_file(plan.paths["receipt"]),
            },
            "description_notes": "supervised test candidate",
        }
        receipt = ARCHIVE.archive_policy(
            manifest,
            timestamp="2026-08-03-18-00-00",
        )
        self.assertEqual(receipt["version"], 2)
        self.assertTrue(Path(receipt["archive_path"]).is_dir())

    def test_export_resources_close_publisher_before_simulation(self) -> None:
        close_order: list[str] = []

        class Publisher:
            def close(self) -> None:
                close_order.append("publisher")

        class SimulationApp:
            def close(self) -> None:
                close_order.append("simulation")

        EXPORT.close_export_resources(Publisher(), SimulationApp())

        self.assertEqual(close_order, ["publisher", "simulation"])

    def test_simulation_closes_when_export_publisher_close_fails(self) -> None:
        close_order: list[str] = []

        class Publisher:
            def close(self) -> None:
                close_order.append("publisher")
                raise RuntimeError("export publisher cleanup failed")

        class SimulationApp:
            def close(self) -> None:
                close_order.append("simulation")

        with self.assertRaisesRegex(RuntimeError, "export publisher cleanup failed"):
            EXPORT.close_export_resources(Publisher(), SimulationApp())

        self.assertEqual(close_order, ["publisher", "simulation"])


if __name__ == "__main__":
    unittest.main()
