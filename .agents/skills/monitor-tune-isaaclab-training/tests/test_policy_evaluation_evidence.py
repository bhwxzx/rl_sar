#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


REPO_ROOT = Path(__file__).resolve().parents[4]
RSL_RL_DIR = REPO_ROOT / "scripts" / "reinforcement_learning" / "rsl_rl"
SKILL_SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(RSL_RL_DIR))
sys.path.insert(0, str(SKILL_SCRIPTS))


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


MODULE = load_module(
    "policy_evaluation_evidence",
    RSL_RL_DIR / "policy_evaluation_evidence.py",
)
LAYOUT = load_module(
    "policy_evaluation_evidence_layout",
    SKILL_SCRIPTS / "prepare_evidence_layout.py",
)


def write_json(path: Path, value: dict) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return MODULE.sha256_file(path)


class PolicyEvaluationEvidenceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary_directory.cleanup)
        self.repo_root = Path(self.temporary_directory.name).resolve() / "robot_lab"
        self.task = "task-a"
        self.run_id = "run-001"
        self.evaluation_id = "eval-001"
        self.scenario = MODULE.build_scenario_contract(
            scenario_id="quick-native",
            scenario_overrides_json="{}",
            command_schedule_json="[]",
            duration_steps=500,
            num_envs=1,
            seed=42,
        )
        self.layout = LAYOUT.prepare_evidence_layout(
            self.repo_root / "learnings" / "policy_tuning",
            task=self.task,
            run_id=self.run_id,
            snapshot_id="snapshot-001",
            evaluation_id=self.evaluation_id,
        )
        self.checkpoint = self.repo_root / "inputs" / "model_100.pt"
        self.artifact = self.repo_root / "inputs" / "policy.pt"
        self.checkpoint.parent.mkdir(parents=True)
        self.checkpoint.write_bytes(b"checkpoint")
        self.artifact.write_bytes(b"artifact")
        self.identity_path = Path(self.layout["paths"]["source_identity"])
        identity = {
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
            "training": {"command": ["python", "train.py"], "hydra_overrides": []},
            "config_files": [{"path": "config.py", "sha256": "2" * 64}],
            "evaluation_scenario": {
                "contract": self.scenario,
                "sha256": MODULE.scenario_sha256(self.scenario),
            },
        }
        identity["identity_sha256"] = MODULE.sha256_bytes(
            MODULE.canonical_json(identity).encode("utf-8")
        )
        self.identity_file_sha256 = write_json(self.identity_path, identity)

    def plan(
        self,
        *,
        result_path: Path | None = None,
        telemetry: bool = True,
        video: bool = True,
        identity_sha256: str | None = None,
        scenario: dict | None = None,
    ) -> MODULE.EvaluationPlan:
        return MODULE.preflight_evaluation(
            repo_root=self.repo_root,
            task=self.task,
            run_id=self.run_id,
            evaluation_id=self.evaluation_id,
            result_path=result_path or Path(self.layout["paths"]["play_result"]),
            telemetry_path=(
                Path(self.layout["paths"]["telemetry"]) if telemetry else None
            ),
            video_path=Path(self.layout["paths"]["video"]) if video else None,
            checkpoint_path=self.checkpoint,
            checkpoint_sha256=MODULE.sha256_file(self.checkpoint),
            artifact_kind="native",
            artifact_path=self.artifact,
            artifact_sha256=MODULE.sha256_file(self.artifact),
            run_identity_path=self.identity_path,
            run_identity_file_sha256=(
                identity_sha256 or self.identity_file_sha256
            ),
            scenario_contract=scenario or self.scenario,
        )

    def result(self, plan: MODULE.EvaluationPlan) -> dict:
        evaluation = {
            "task": self.task,
            "run_id": self.run_id,
            "evaluation_id": self.evaluation_id,
            "candidate_id": "model-100",
            "runner": "OnPolicyRunnerAmpROA",
        }
        inputs = {
            "checkpoint": plan.checkpoint,
            "artifact": {"kind": plan.artifact_kind, **plan.artifact},
            "run_identity": plan.run_identity,
            "scenario": {
                "contract": plan.scenario_contract,
                "sha256": plan.scenario_sha256,
            },
            "resource_mode": {
                "training_overlap": False,
                "idle_gpu_required": True,
                "video_requested": plan.video_path is not None,
                "telemetry_requested": plan.telemetry_path is not None,
            },
        }
        return {
            "version": 2,
            "status": "completed",
            "evaluation": evaluation,
            "inputs": inputs,
        }

    def telemetry(self, result: dict) -> dict:
        return {
            "version": 3,
            "evaluation": result["evaluation"],
            "inputs": result["inputs"],
            "telemetry_status": "complete",
            "samples": [],
        }

    def test_layout_paths_publish_exclusively_and_revalidate(self) -> None:
        plan = self.plan()
        result = self.result(plan)
        with MODULE.EvaluationPublisher(plan) as publisher:
            publisher.video_work_path.write_bytes(b"video")
            published = publisher.publish(
                result,
                telemetry=self.telemetry(result),
                video_source=publisher.video_work_path,
            )
        self.assertTrue(plan.result_path.is_file())
        self.assertEqual(
            published["outputs"]["video"]["sha256"],
            MODULE.sha256_file(plan.video_path),
        )
        validation = MODULE.validate_evaluation_bundle(plan.result_path)
        self.assertEqual(validation["status"], "valid")
        self.assertFalse((plan.result_path.parent / ".attempt").exists())
        self.assertFalse((plan.result_path.parent / ".publish-claim").exists())

    def test_wrong_layout_and_traversal_are_rejected(self) -> None:
        wrong = self.repo_root / "result.json"
        with self.assertRaisesRegex(
            MODULE.EvaluationEvidenceError, "outside the current run/evaluation"
        ):
            self.plan(result_path=wrong)
        with self.assertRaisesRegex(
            MODULE.EvaluationEvidenceError, "safe ASCII identifier"
        ):
            MODULE.expected_evaluation_paths(
                self.repo_root,
                task="../escape",
                run_id=self.run_id,
                evaluation_id=self.evaluation_id,
            )

    def test_symlinked_output_component_is_rejected(self) -> None:
        evaluation_dir = Path(self.layout["directories"]["evaluation"])
        external = self.repo_root / "external"
        external.mkdir()
        evaluation_dir.rmdir()
        evaluation_dir.symlink_to(external, target_is_directory=True)
        with self.assertRaisesRegex(MODULE.EvaluationEvidenceError, "symlinked"):
            self.plan()

    def test_existing_target_is_never_overwritten(self) -> None:
        result_path = Path(self.layout["paths"]["play_result"])
        result_path.write_text("keep\n", encoding="utf-8")
        with self.assertRaisesRegex(MODULE.EvaluationEvidenceError, "already exists"):
            self.plan()
        self.assertEqual(result_path.read_text(encoding="utf-8"), "keep\n")

    def test_input_and_identity_hash_mismatches_are_rejected(self) -> None:
        with self.assertRaisesRegex(MODULE.EvaluationEvidenceError, "run identity SHA-256"):
            self.plan(identity_sha256="0" * 64)
        checkpoint_before = self.checkpoint.read_bytes()
        self.checkpoint.write_bytes(b"changed")
        with self.assertRaisesRegex(MODULE.EvaluationEvidenceError, "checkpoint SHA-256"):
            MODULE.preflight_evaluation(
                repo_root=self.repo_root,
                task=self.task,
                run_id=self.run_id,
                evaluation_id=self.evaluation_id,
                result_path=Path(self.layout["paths"]["play_result"]),
                telemetry_path=None,
                video_path=None,
                checkpoint_path=self.checkpoint,
                checkpoint_sha256=MODULE.sha256_bytes(checkpoint_before),
                artifact_kind="native",
                artifact_path=self.artifact,
                artifact_sha256=MODULE.sha256_file(self.artifact),
                run_identity_path=self.identity_path,
                run_identity_file_sha256=self.identity_file_sha256,
                scenario_contract=self.scenario,
            )

    def test_scenario_conflict_is_rejected(self) -> None:
        conflicting = dict(self.scenario)
        conflicting["duration_steps"] = 501
        with self.assertRaisesRegex(MODULE.EvaluationEvidenceError, "scenario contract mismatch"):
            self.plan(scenario=conflicting)

    def test_mid_attempt_failure_leaves_no_complete_evidence(self) -> None:
        plan = self.plan()
        with self.assertRaisesRegex(RuntimeError, "simulation failed"):
            with MODULE.EvaluationPublisher(plan) as publisher:
                publisher.video_work_path.write_bytes(b"partial")
                raise RuntimeError("simulation failed")
        for path in (plan.result_path, plan.telemetry_path, plan.video_path):
            self.assertFalse(path.exists())

    def test_concurrent_claim_is_rejected(self) -> None:
        plan = self.plan()
        with MODULE.EvaluationPublisher(plan):
            with self.assertRaisesRegex(
                MODULE.EvaluationEvidenceError, "already claimed"
            ):
                with MODULE.EvaluationPublisher(plan):
                    self.fail("second publisher unexpectedly acquired the claim")

    def test_publish_collision_rolls_back_only_owned_targets(self) -> None:
        plan = self.plan()
        result = self.result(plan)
        assert plan.telemetry_path is not None
        with MODULE.EvaluationPublisher(plan) as publisher:
            publisher.video_work_path.write_bytes(b"video")
            real_link = MODULE.os.link

            def racing_link(source, target, *, follow_symlinks=False):
                real_link(source, target, follow_symlinks=follow_symlinks)
                if target == plan.video_path:
                    plan.telemetry_path.write_text("external\n", encoding="utf-8")

            with self.assertRaisesRegex(
                MODULE.EvaluationEvidenceError, "publication failed"
            ):
                with patch.object(MODULE.os, "link", side_effect=racing_link):
                    publisher.publish(
                        result,
                        telemetry=self.telemetry(result),
                        video_source=publisher.video_work_path,
                    )
        self.assertFalse(plan.video_path.exists())
        self.assertFalse(plan.result_path.exists())
        self.assertEqual(plan.telemetry_path.read_text(encoding="utf-8"), "external\n")

    def test_optional_outputs_remain_absent(self) -> None:
        plan = self.plan(telemetry=False, video=False)
        result = self.result(plan)
        with MODULE.EvaluationPublisher(plan) as publisher:
            published = publisher.publish(result, telemetry=None, video_source=None)
        self.assertIsNone(published["outputs"]["telemetry"])
        self.assertIsNone(published["outputs"]["video"])
        self.assertFalse(plan.canonical_paths["telemetry"].exists())
        self.assertFalse(plan.canonical_paths["video"].exists())
        self.assertEqual(
            MODULE.validate_evaluation_bundle(plan.result_path)["status"],
            "valid",
        )

    def test_input_drift_before_completion_rolls_back_result(self) -> None:
        plan = self.plan(telemetry=False, video=False)
        result = self.result(plan)
        with MODULE.EvaluationPublisher(plan) as publisher:
            self.checkpoint.write_bytes(b"mutated-after-preflight")
            with self.assertRaisesRegex(
                MODULE.EvaluationEvidenceError, "publication failed"
            ):
                publisher.publish(result, telemetry=None, video_source=None)
        self.assertFalse(plan.result_path.exists())

    def test_result_is_the_last_published_target(self) -> None:
        plan = self.plan()
        result = self.result(plan)
        linked_targets: list[Path] = []
        real_link = MODULE.os.link

        def recording_link(source, target, *, follow_symlinks=False):
            linked_targets.append(Path(target))
            return real_link(source, target, follow_symlinks=follow_symlinks)

        with MODULE.EvaluationPublisher(plan) as publisher:
            publisher.video_work_path.write_bytes(b"video")
            with patch.object(MODULE.os, "link", side_effect=recording_link):
                publisher.publish(
                    result,
                    telemetry=self.telemetry(result),
                    video_source=publisher.video_work_path,
                )
        self.assertEqual(linked_targets[-1], plan.result_path)

    def test_evaluation_resources_close_publisher_before_simulation(self) -> None:
        close_order: list[str] = []

        class Publisher:
            def close(self) -> None:
                close_order.append("publisher")

        class SimulationApp:
            def close(self) -> None:
                close_order.append("simulation")

        MODULE.close_evaluation_resources(Publisher(), SimulationApp())

        self.assertEqual(close_order, ["publisher", "simulation"])

    def test_simulation_closes_when_publisher_close_fails(self) -> None:
        close_order: list[str] = []

        class Publisher:
            def close(self) -> None:
                close_order.append("publisher")
                raise RuntimeError("publisher cleanup failed")

        class SimulationApp:
            def close(self) -> None:
                close_order.append("simulation")

        with self.assertRaisesRegex(RuntimeError, "publisher cleanup failed"):
            MODULE.close_evaluation_resources(Publisher(), SimulationApp())

        self.assertEqual(close_order, ["publisher", "simulation"])


if __name__ == "__main__":
    unittest.main()
