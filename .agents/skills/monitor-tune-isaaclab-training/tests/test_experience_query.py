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
SCRIPT = SCRIPT_DIR / "query_tuning_experience.py"
SPEC = importlib.util.spec_from_file_location("query_tuning_experience", SCRIPT)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)

import capture_run_identity as IDENTITY  # noqa: E402
import capture_effective_training_config as CONFIG  # noqa: E402
import record_tuning_experience as RECORD  # noqa: E402


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


def env_yaml(weight: float) -> str:
    return f"""\
seed: 42
scene:
  num_envs: 4096
rewards:
  action_rate:
    func: example.rewards:action_rate
    weight: {weight}
  disabled_term: null
"""


def make_identity(
    repository_root: Path,
    *,
    run_id: str,
    algorithm: str = "amp-roa",
    host_id: str = "younghit",
) -> dict:
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
        "task": "lw-leg-rough",
        "run_id": run_id,
        "host_id": host_id,
        "backend": "isaaclab",
        "algorithm": algorithm,
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
    return identity


def capture_config(
    root: Path,
    identity: dict,
    *,
    reward_weight: float,
    snapshot_id: str,
) -> tuple[dict, Path, dict]:
    repository_root = Path(identity["source"]["repository_root"])
    log_directory = (
        repository_root
        / "logs"
        / "rsl_rl"
        / "effective-config-test"
        / identity["run_id"]
    )
    params = log_directory / "params"
    params.mkdir(parents=True)
    (params / "env.yaml").write_text(env_yaml(reward_weight), encoding="utf-8")
    (params / "agent.yaml").write_text(AGENT_YAML, encoding="utf-8")
    config = CONFIG.capture_effective_config(identity, log_directory)
    path = (
        root
        / identity["task"]
        / identity["run_id"]
        / "evidence"
        / "source"
        / f"effective-config-{snapshot_id}.json"
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    receipt = CONFIG.write_new_evidence(path, config)
    return config, path, receipt


def make_event(
    *,
    event_id: str = "assessment-001",
    event_type: str = "assessment",
    recorded_at: str = "2026-08-01T18:00:00+08:00",
    run_id: str = "run-001",
    algorithm: str = "amp-roa",
    host_id: str = "younghit",
    observation: str = "obs-hash",
    deployment: str = "deploy-hash",
    version: int = 4,
    evidence: dict | None = None,
    root: Path | None = None,
    reward_weight: float = -0.4,
) -> dict:
    event = {
        "version": version,
        "event_id": event_id,
        "event_type": event_type,
        "recorded_at": recorded_at,
        "task": "lw-leg-rough",
        "run_id": run_id,
        "algorithm": algorithm,
        "context": {
            "observation_fingerprint": observation,
            "reward_fingerprint": "unknown",
            "deployment_fingerprint": deployment,
        },
        "parameters": {"action_rate_l2": -0.15},
        "evidence": evidence or {},
        "analysis": {"summary": "healthy", "confidence": "medium"},
        "next_suggestion": "recheck before changing parameters",
    }
    if version == 1:
        event["context"]["reward_fingerprint"] = "legacy-reward-hash"
        return event
    if root is None:
        raise ValueError("root is required for version 2 or 3 events")
    identity = make_identity(
        root.parents[1],
        run_id=run_id,
        algorithm=algorithm,
        host_id=host_id,
    )
    event["run_identity"] = identity
    if version == 2:
        event["context"]["reward_fingerprint"] = "legacy-reward-hash"
        return event
    config, path, receipt = capture_config(
        root,
        identity,
        reward_weight=reward_weight,
        snapshot_id=event_id,
    )
    event["context"]["reward_fingerprint"] = config["fingerprints"]["reward"]
    event["evidence"]["effective_config"] = {
        "path": str(path),
        "sha256": receipt["sha256"],
        "effective_config_fingerprint": config["fingerprints"]["effective_config"],
        "reward_fingerprint": config["fingerprints"]["reward"],
    }
    if version == 4:
        if event_type in RECORD.EVIDENCE_EVENT_TYPES:
            event["evidence"]["event"] = {
                "status": "unavailable",
                "reason": "event artifact not captured",
            }
        if event_type == "feedback":
            event["evidence"]["source"] = "sim2sim"
            event["evidence"]["policy_binding"] = {
                "status": "unavailable",
                "reason": "policy binding not captured",
            }
        event["evidence"]["outcome"] = {
            "status": "unavailable",
            "reason": "baseline comparison not captured",
        }
    return event


def add_complete_assessment_outcome(root: Path, event: dict) -> None:
    identity = event["run_identity"]
    assessment = {
        "version": 2,
        "advisory_only": True,
        "criteria": {
            "expected_scope": {
                field: identity[field]
                for field in ("task", "run_id", "backend", "algorithm", "runner")
            }
        },
    }
    assessment_path = (
        root
        / event["task"]
        / event["run_id"]
        / "evidence"
        / "assessment"
        / f"assessment-{event['event_id']}.json"
    )
    assessment_path.parent.mkdir(parents=True)
    assessment_path.write_text(
        json.dumps(assessment, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    assessment_reference = {
        "path": str(assessment_path),
        "sha256": hashlib.sha256(assessment_path.read_bytes()).hexdigest(),
    }
    event["evidence"]["event"] = {
        "status": "available",
        **assessment_reference,
    }

    baseline_identity = make_identity(
        root.parents[1],
        run_id=f"baseline-{event['event_id']}",
        algorithm=event["algorithm"],
        host_id=identity["host_id"],
    )
    baseline_source = (
        root
        / baseline_identity["task"]
        / baseline_identity["run_id"]
        / "evidence"
        / "source"
    )
    baseline_source.mkdir(parents=True)
    identity_path = baseline_source / "identity-baseline.json"
    identity_path.write_text(
        json.dumps(baseline_identity, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    baseline_config, baseline_config_path, baseline_receipt = capture_config(
        root,
        baseline_identity,
        reward_weight=-0.5,
        snapshot_id="baseline",
    )
    current_reference = event["evidence"]["effective_config"]
    current_config, _ = CONFIG.load_and_validate_effective_config(
        Path(current_reference["path"]),
        expected_sha256=current_reference["sha256"],
        run_identity=identity,
    )
    event["evidence"]["outcome"] = {
        "status": "available",
        "baseline": {
            "run_identity": {
                "path": str(identity_path),
                "sha256": hashlib.sha256(identity_path.read_bytes()).hexdigest(),
            },
            "effective_config": {
                "path": str(baseline_config_path),
                "sha256": baseline_receipt["sha256"],
                "effective_config_fingerprint": baseline_config["fingerprints"][
                    "effective_config"
                ],
                "reward_fingerprint": baseline_config["fingerprints"]["reward"],
            },
        },
        "parameter_changes": CONFIG.compare_effective_configs(
            baseline_config,
            current_config,
        ),
        "result_window": {
            **assessment_reference,
            "start_step": 100,
            "end_step": 200,
        },
        "observed_effect": {
            "summary": "tracking improved over the bounded result window",
            "observations": ["mean tracking error decreased"],
        },
    }


class ExperienceQueryTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary_directory.cleanup)
        self.repository_root = (
            Path(self.temporary_directory.name).resolve() / "robot_lab"
        )
        self.root = self.repository_root / "learnings" / "policy_tuning"
        self.root.mkdir(parents=True)
        self.current_identity = make_identity(
            self.repository_root,
            run_id="run-current",
        )
        (
            self.current_config,
            self.current_config_path,
            self.current_config_receipt,
        ) = capture_config(
            self.root,
            self.current_identity,
            reward_weight=-0.4,
            snapshot_id="current",
        )

    def query(self, **overrides) -> dict:
        arguments = {
            "run_identity": self.current_identity,
            "effective_config_path": self.current_config_path,
            "effective_config_sha256": self.current_config_receipt["sha256"],
            "observation_fingerprint": "obs-hash",
            "deployment_fingerprint": "deploy-hash",
        }
        arguments.update(overrides)
        return MODULE.query_tuning_experience(self.root, **arguments)

    def write(self, event: dict) -> Path:
        if event["version"] == 4:
            receipt = RECORD.write_event(self.root, event)
            return Path(receipt["event_path"])
        run_dir = self.root / event["task"] / event["run_id"]
        run_dir.mkdir(parents=True, exist_ok=True)
        timestamp_slug = MODULE._timestamp_slug(event["recorded_at"])
        path = run_dir / f"{timestamp_slug}__{event['event_id']}.json"
        path.write_text(
            json.dumps(event, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        return path

    def test_exact_version_four_outcome_is_candidate_evidence(self) -> None:
        health_path = "/archive/run-001/evidence/health/health.json"
        value = make_event(
            root=self.root,
            evidence={
                "health": {"path": health_path, "sha256": "a" * 64},
                "video_path": "/archive/run-001/evidence/play/video.mp4",
                "video_sha256": "b" * 64,
            },
        )
        add_complete_assessment_outcome(self.root, value)
        config_path = value["evidence"]["effective_config"]["path"]
        event_path = self.write(value)
        result = self.query()
        self.assertEqual(result["summary"]["compatible"], 1)
        self.assertEqual(
            result["historical_support"]["status"],
            "candidate_outcome_history_available",
        )
        self.assertFalse(result["historical_support"]["direct_parameter_change_supported"])
        item = result["compatible_events"][0]
        self.assertEqual(item["event_path"], str(event_path))
        self.assertEqual(
            item["event_sha256"], hashlib.sha256(event_path.read_bytes()).hexdigest()
        )
        self.assertTrue(
            {
                health_path,
                "/archive/run-001/evidence/play/video.mp4",
                config_path,
            }.issubset(
                {reference["path"] for reference in item["evidence_refs"]}
            )
        )
        self.assertEqual(
            item["effective_config_verification"]["status"],
            "verified",
        )
        self.assertEqual(item["parameter_diff"]["summary"]["semantic_changes"], 0)
        self.assertTrue(item["context_compatible"])
        self.assertTrue(item["event_evidence_complete"])
        self.assertTrue(item["outcome_evidence_complete"])
        self.assertTrue(item["tuning_candidate_evidence"])
        self.assertEqual(result["candidate_events"], [item])

    def test_known_mismatches_are_conflicting_with_explicit_reasons(self) -> None:
        cases = [
            ("algorithm", {"algorithm": "ppo"}, "algorithm_mismatch"),
            ("host", {"host_id": "server5090"}, "host_id_mismatch"),
            (
                "observation",
                {"observation": "different-observation"},
                "observation_fingerprint_mismatch",
            ),
            (
                "reward",
                {"reward_weight": -0.3},
                "reward_fingerprint_mismatch",
            ),
            (
                "deployment",
                {"deployment": "different-deployment"},
                "deployment_fingerprint_mismatch",
            ),
        ]
        expected_reasons = {}
        for index, (name, overrides, reason) in enumerate(cases, start=1):
            event_id = f"assessment-{name}"
            self.write(
                make_event(
                    root=self.root,
                    event_id=event_id,
                    run_id=f"run-{index:03d}",
                    **overrides,
                )
            )
            expected_reasons[event_id] = [reason]
        result = self.query()
        observed_reasons = {
            item["event_id"]: item["classification_reasons"]
            for item in result["conflicting_events"]
        }
        self.assertEqual(
            observed_reasons,
            expected_reasons,
        )
        reward_item = next(
            item
            for item in result["conflicting_events"]
            if item["event_id"] == "assessment-reward"
        )
        self.assertEqual(
            reward_item["effective_config_verification"]["status"],
            "verified",
        )
        self.assertEqual(
            reward_item["parameter_diff"]["reward_weight_changes"],
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
            result["historical_support"]["status"],
            "no_context_compatible_history",
        )

    def test_legacy_and_explicit_unknown_events_never_become_compatible(self) -> None:
        self.write(make_event(version=1))
        self.write(
            make_event(
                root=self.root,
                version=2,
                event_id="assessment-v2",
                run_id="run-v2",
            )
        )
        self.write(
            make_event(
                root=self.root,
                event_id="assessment-002",
                observation="unknown",
            )
        )
        result = self.query()
        self.assertEqual(result["summary"]["unknown"], 3)
        reasons = [item["classification_reasons"] for item in result["unknown_events"]]
        self.assertTrue(
            any(
                "event_host_id_unknown" in item
                and "event_effective_config_unknown" in item
                for item in reasons
            )
        )
        self.assertTrue(
            any(
                "event_effective_config_unknown" in item
                and "event_host_id_unknown" not in item
                for item in reasons
            )
        )
        self.assertIn(["event_observation_fingerprint_unknown"], reasons)

    def test_version_three_can_match_context_but_not_prove_outcome(self) -> None:
        self.write(make_event(root=self.root, version=3))
        result = self.query()
        self.assertEqual(result["summary"]["context_compatible"], 1)
        item = result["compatible_events"][0]
        self.assertTrue(item["context_compatible"])
        self.assertFalse(item["event_evidence_complete"])
        self.assertFalse(item["outcome_evidence_complete"])
        self.assertFalse(item["tuning_candidate_evidence"])
        self.assertEqual(
            result["historical_support"]["status"],
            "context_compatible_outcome_incomplete",
        )

    def test_recommendation_never_becomes_outcome_evidence(self) -> None:
        self.write(
            make_event(
                root=self.root,
                event_id="recommendation-001",
                event_type="recommendation",
            )
        )
        result = self.query()
        item = result["compatible_events"][0]
        self.assertTrue(item["context_compatible"])
        self.assertFalse(item["outcome_evidence_complete"])
        self.assertFalse(item["tuning_candidate_evidence"])
        self.assertIn(
            "recommendation_is_advice_not_outcome",
            item["evidence_completeness_reasons"],
        )
        self.assertFalse(result["historical_support"]["direct_parameter_change_supported"])

    def test_unknown_query_context_suppresses_historical_support(self) -> None:
        self.write(make_event(root=self.root))
        result = self.query(observation_fingerprint="unknown")
        self.assertEqual(result["summary"]["unknown"], 1)
        self.assertEqual(
            result["historical_support"]["status"],
            "query_context_incomplete",
        )

    def test_invalid_json_is_reported_and_suppresses_support(self) -> None:
        self.write(make_event(root=self.root))
        run_dir = self.root / "lw-leg-rough" / "run-002"
        run_dir.mkdir(parents=True)
        invalid = run_dir / "invalid.json"
        invalid.write_text("{invalid", encoding="utf-8")
        result = self.query()
        self.assertEqual(result["summary"]["compatible"], 1)
        self.assertEqual(result["summary"]["invalid"], 1)
        self.assertEqual(result["historical_support"]["status"], "history_invalid")
        self.assertFalse(result["scan"]["complete"])

    def test_matching_context_with_changed_config_artifact_is_invalid(self) -> None:
        value = make_event(root=self.root)
        self.write(value)
        config_path = Path(value["evidence"]["effective_config"]["path"])
        config_path.write_text(
            config_path.read_text(encoding="utf-8") + " ",
            encoding="utf-8",
        )
        result = self.query()
        self.assertEqual(result["summary"]["invalid"], 1)
        self.assertEqual(result["historical_support"]["status"], "history_invalid")
        self.assertIn("SHA-256 mismatch", result["invalid_events"][0]["error"])

    def test_matching_outcome_artifact_drift_is_invalid(self) -> None:
        value = make_event(root=self.root)
        add_complete_assessment_outcome(self.root, value)
        self.write(value)
        assessment_path = Path(value["evidence"]["event"]["path"])
        assessment_path.write_text("{}\n", encoding="utf-8")
        result = self.query()
        self.assertEqual(result["summary"]["invalid"], 1)
        self.assertEqual(result["historical_support"]["status"], "history_invalid")
        self.assertIn("SHA-256 mismatch", result["invalid_events"][0]["error"])

    def test_other_host_config_is_not_dereferenced(self) -> None:
        value = make_event(root=self.root, host_id="server5090")
        self.write(value)
        config_path = Path(value["evidence"]["effective_config"]["path"])
        config_path.write_text("unavailable on this host", encoding="utf-8")
        result = self.query()
        self.assertTrue(result["scan"]["complete"])
        self.assertEqual(result["summary"]["conflicting"], 1)
        item = result["conflicting_events"][0]
        self.assertEqual(item["classification_reasons"], ["host_id_mismatch"])
        self.assertEqual(
            item["effective_config_verification"]["status"],
            "not_checked_context_mismatch",
        )

    def test_storage_scope_and_filename_mismatch_are_invalid(self) -> None:
        event = make_event(root=self.root)
        run_dir = self.root / "lw-leg-rough" / "wrong-run"
        run_dir.mkdir(parents=True)
        (run_dir / "wrong.json").write_text(
            json.dumps(event),
            encoding="utf-8",
        )
        filename_event = make_event(
            root=self.root,
            event_id="assessment-filename",
            run_id="run-002",
        )
        filename_run_dir = self.root / "lw-leg-rough" / "run-002"
        filename_run_dir.mkdir(exist_ok=True)
        (filename_run_dir / "wrong.json").write_text(
            json.dumps(filename_event),
            encoding="utf-8",
        )
        result = self.query()
        self.assertEqual(result["summary"]["invalid"], 2)
        errors = [item["error"] for item in result["invalid_events"]]
        self.assertTrue(any("scope" in error for error in errors))
        self.assertTrue(any("filename" in error for error in errors))

    def test_results_are_deterministic_and_chronologically_sorted(self) -> None:
        self.write(
            make_event(
                root=self.root,
                event_id="assessment-later",
                recorded_at="2026-08-01T18:00:00+08:00",
                run_id="run-z",
            )
        )
        self.write(
            make_event(
                root=self.root,
                event_id="assessment-earlier",
                recorded_at="2026-08-01T09:30:00Z",
                run_id="run-a",
            )
        )
        first = self.query()
        second = self.query()
        self.assertEqual(first, second)
        self.assertEqual(
            [item["event_id"] for item in first["compatible_events"]],
            ["assessment-earlier", "assessment-later"],
        )

    def test_nested_evidence_json_is_not_scanned(self) -> None:
        self.write(make_event(root=self.root))
        nested = (
            self.root
            / "lw-leg-rough"
            / "run-001"
            / "evidence"
            / "health"
            / "health.json"
        )
        nested.parent.mkdir(parents=True)
        nested.write_text("{invalid", encoding="utf-8")
        result = self.query()
        self.assertEqual(result["scan"]["event_files"], 1)
        self.assertEqual(result["summary"]["invalid"], 0)

    def test_rejects_symlinked_root_run_and_event_paths(self) -> None:
        real_root = Path(self.temporary_directory.name).resolve() / "real-root"
        real_root.mkdir()
        linked_root = Path(self.temporary_directory.name).resolve() / "linked-root"
        linked_root.symlink_to(real_root, target_is_directory=True)
        with self.assertRaisesRegex(MODULE.ExperienceQueryError, "symlinked"):
            MODULE.query_tuning_experience(
                linked_root,
                run_identity=self.current_identity,
                effective_config_path=self.current_config_path,
                effective_config_sha256=self.current_config_receipt["sha256"],
                observation_fingerprint="obs-hash",
                deployment_fingerprint="deploy-hash",
            )

        task_dir = self.root / "lw-leg-rough"
        external_run = Path(self.temporary_directory.name).resolve() / "external-run"
        external_run.mkdir()
        (task_dir / "run-linked").symlink_to(external_run, target_is_directory=True)
        with self.assertRaisesRegex(MODULE.ExperienceQueryError, "symlinked run"):
            self.query()

        (task_dir / "run-linked").unlink()
        run_dir = task_dir / "run-001"
        run_dir.mkdir()
        external_event = Path(self.temporary_directory.name).resolve() / "external.json"
        external_event.write_text("{}", encoding="utf-8")
        (run_dir / "event.json").symlink_to(external_event)
        with self.assertRaisesRegex(MODULE.ExperienceQueryError, "symlinked event"):
            self.query()

    def test_scan_and_file_size_bounds_are_enforced(self) -> None:
        self.write(make_event(root=self.root))
        self.write(
            make_event(
                root=self.root,
                event_id="assessment-002",
                run_id="run-002",
            )
        )
        with self.assertRaisesRegex(MODULE.ExperienceQueryError, "max-events=1"):
            self.query(max_events=1)
        result = self.query(max_event_bytes=10)
        self.assertEqual(result["summary"]["invalid"], 2)
        self.assertEqual(result["historical_support"]["status"], "history_invalid")

    def test_cli_prints_json_without_changing_history(self) -> None:
        self.write(make_event(root=self.root))
        identity_path = (
            self.root
            / self.current_identity["task"]
            / self.current_identity["run_id"]
            / "evidence"
            / "source"
            / "identity-current.json"
        )
        identity_path.write_text(json.dumps(self.current_identity), encoding="utf-8")

        def snapshot() -> dict[str, bytes | None]:
            return {
                str(path.relative_to(self.root)): path.read_bytes() if path.is_file() else None
                for path in sorted(self.root.rglob("*"))
            }

        before = snapshot()
        stdout = io.StringIO()
        argv = [
            str(SCRIPT),
            "--root",
            str(self.root),
            "--run-identity",
            str(identity_path),
            "--effective-config",
            str(self.current_config_path),
            "--effective-config-sha256",
            self.current_config_receipt["sha256"],
            "--observation-fingerprint",
            "obs-hash",
            "--deployment-fingerprint",
            "deploy-hash",
        ]
        with patch.object(sys, "argv", argv), contextlib.redirect_stdout(stdout):
            self.assertEqual(MODULE.main(), 0)
        result = json.loads(stdout.getvalue())
        self.assertTrue(result["read_only"])
        self.assertEqual(snapshot(), before)


if __name__ == "__main__":
    unittest.main()
