#!/usr/bin/env python3

from __future__ import annotations

import copy
import importlib.util
import sys
import unittest
from pathlib import Path


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
SCRIPT = SCRIPTS / "assess_training_run.py"
SPEC = importlib.util.spec_from_file_location("assess_training_run", SCRIPT)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)

from assessment_criteria import canonical_contract_sha256  # noqa: E402


SCOPE = {
    "task": "test-task",
    "run_id": "run-001",
    "backend": "isaaclab",
    "profile_id": "rsl-rl-amp-roa",
    "algorithm": "AMP-ROA",
    "runner": "OnPolicyRunnerAmpROA",
}


def criteria(*, status: str = "approved") -> dict:
    contract = {
        "scope": dict(SCOPE),
        "windows": {
            "window_size": 2,
            "minimum_records": 4,
            "plateau_required_metrics": 2,
        },
        "required_metrics": {
            "mean_reward": {
                "direction": "maximize",
                "plateau_relative_tolerance": 0.01,
            },
            "error_vel_xy": {
                "direction": "minimize",
                "plateau_relative_tolerance": 0.01,
            },
        },
        "observed_metrics": {
            "action_noise_std": {
                "direction": "observe",
                "description": "reported only",
            }
        },
        "hard_failures": {
            "non_finite_metrics": True,
            "health_states": ["stalled"],
            "metric_limits": {"error_vel_xy": {"op": "<=", "value": 1.0}},
        },
        "play_gates": {
            "required_for_convergence": True,
            "metrics": {"termination_rate": {"op": "<=", "value": 0.1}},
        },
    }
    if status == "approved":
        approval = {
            "status": "approved",
            "approved_at": "2026-08-01T17:00:00+08:00",
            "approved_contract_sha256": canonical_contract_sha256(contract),
        }
    else:
        approval = {
            "status": "draft",
            "approved_at": None,
            "approved_contract_sha256": None,
        }
    return {"version": 2, "contract": contract, "approval": approval}


def summary(
    rows: list[tuple[float, float]],
    non_finite: list | None = None,
    noise: list[float] | None = None,
) -> dict:
    noise = noise or [0.5] * len(rows)
    return {
        "profile_id": "rsl-rl-amp-roa",
        "records": [
            {
                "progress": index,
                "mean_reward": reward,
                "error_vel_xy": error,
                "action_noise_std": noise[index],
            }
            for index, (reward, error) in enumerate(rows)
        ],
        "non_finite_metrics": non_finite or [],
    }


def assess(rows: list[tuple[float, float]], **kwargs) -> dict:
    return MODULE.assess_training(
        summary(rows, kwargs.pop("non_finite", None), kwargs.pop("noise", None)),
        kwargs.pop("criteria_document", criteria()),
        expected_scope=kwargs.pop("expected_scope", SCOPE),
        **kwargs,
    )


class TrainingAdvisorTests(unittest.TestCase):
    def test_continue_when_required_metrics_improve(self) -> None:
        report = assess(
            [(10.0, 0.8), (11.0, 0.7), (14.0, 0.5), (15.0, 0.4)],
            health={"state": "healthy"},
        )
        self.assertEqual(report["recommendation"], "continue")
        self.assertTrue(report["criteria"]["eligible"])
        self.assertTrue(report["advisory_only"])

    def test_observed_metric_change_does_not_affect_continue(self) -> None:
        report = assess(
            [(10.0, 0.8), (11.0, 0.7), (14.0, 0.5), (15.0, 0.4)],
            noise=[0.1, 0.2, 100.0, 200.0],
            health={"state": "healthy"},
        )
        self.assertEqual(report["recommendation"], "continue")
        self.assertFalse(report["observed_trends"][0]["decision_bearing"])

    def test_suspect_health_requires_recheck_even_when_metrics_improve(self) -> None:
        report = assess(
            [(10.0, 0.8), (11.0, 0.7), (14.0, 0.5), (15.0, 0.4)],
            health={"state": "suspect"},
        )
        self.assertEqual(report["recommendation"], "continue_and_recheck")

    def test_unknown_health_is_insufficient_even_when_metrics_improve(self) -> None:
        report = assess(
            [(10.0, 0.8), (11.0, 0.7), (14.0, 0.5), (15.0, 0.4)],
            health={"state": "unknown"},
        )
        self.assertEqual(report["recommendation"], "insufficient_evidence")

    def test_observing_health_requires_recheck_even_when_metrics_plateau(self) -> None:
        report = assess(
            [(10.0, 0.5), (10.0, 0.5), (10.0, 0.5), (10.0, 0.5)],
            health={"state": "observing"},
        )
        self.assertEqual(report["recommendation"], "continue_and_recheck")

    def test_plateau_is_advisory(self) -> None:
        report = assess(
            [(10.0, 0.5), (10.0, 0.5), (10.0, 0.5), (10.0, 0.5)],
            health={"state": "healthy"},
        )
        self.assertEqual(report["recommendation"], "consider_stop_plateau")
        self.assertTrue(report["pending_user_decision"])

    def test_approved_nonfinite_recommends_stop_without_action(self) -> None:
        report = assess(
            [(10.0, 0.5), (10.0, 0.5), (10.0, 0.5), (10.0, 0.5)],
            non_finite=[{"progress": 3, "metric": "mean_reward"}],
            health={"state": "healthy"},
        )
        self.assertEqual(report["recommendation"], "recommend_stop_invalid")
        self.assertNotIn("signal", report)

    def test_approved_busy_wait_stall_recommends_stop_without_action(self) -> None:
        report = assess(
            [(10.0, 0.8), (11.0, 0.7), (14.0, 0.5), (15.0, 0.4)],
            health={"state": "stalled", "activity_without_progress": True},
        )
        self.assertEqual(report["recommendation"], "recommend_stop_invalid")
        self.assertTrue(report["advisory_only"])
        self.assertTrue(report["pending_user_decision"])
        self.assertNotIn("signal", report)
        self.assertNotIn("action", report)

    def test_hard_metric_limit_is_separate_from_trend(self) -> None:
        report = assess(
            [(10.0, 0.8), (11.0, 0.7), (14.0, 1.2), (15.0, 1.1)],
            health={"state": "healthy"},
        )
        self.assertEqual(report["recommendation"], "recommend_stop_invalid")
        self.assertTrue(report["hard_failures"]["failed"])

    def test_completed_plateau_with_passing_play_is_converged(self) -> None:
        report = assess(
            [(10.0, 0.5), (10.0, 0.5), (10.0, 0.5), (10.0, 0.5)],
            health={"state": "completed"},
            play_results=[
                {
                    "status": "completed",
                    "telemetry_status": "complete",
                    "missing_required_signals": [],
                    "metrics": {"termination_rate": 0.0},
                }
            ],
            training_finished=True,
        )
        self.assertEqual(report["convergence"], "converged")

    def test_completed_without_play_is_indeterminate(self) -> None:
        report = assess(
            [(10.0, 0.5), (10.0, 0.5), (10.0, 0.5), (10.0, 0.5)],
            health={"state": "completed"},
            training_finished=True,
        )
        self.assertEqual(report["convergence"], "indeterminate")

    def test_failed_play_prevents_convergence_but_is_not_hard_stop(self) -> None:
        report = assess(
            [(10.0, 0.5), (10.0, 0.5), (10.0, 0.5), (10.0, 0.5)],
            health={"state": "completed"},
            play_results=[{"status": "completed", "metrics": {"termination_rate": 0.2}}],
            training_finished=True,
        )
        self.assertEqual(report["recommendation"], "consider_stop_plateau")
        self.assertEqual(report["convergence"], "plateaued_with_defects")

    def test_amp_roa_passing_play_with_partial_telemetry_is_indeterminate(self) -> None:
        report = assess(
            [(10.0, 0.5), (10.0, 0.5), (10.0, 0.5), (10.0, 0.5)],
            health={"state": "completed"},
            play_results=[
                {
                    "status": "completed",
                    "telemetry_status": "partial",
                    "missing_required_signals": ["joint_velocity"],
                    "metrics": {"termination_rate": 0.0},
                }
            ],
            training_finished=True,
        )
        self.assertEqual(report["convergence"], "indeterminate")
        self.assertTrue(report["play"]["passed"])
        self.assertFalse(report["play"]["eligible_for_convergence"])
        self.assertEqual(
            report["play"]["telemetry_checks"][0]["missing_required_signals"],
            ["joint_velocity"],
        )

    def test_non_amp_runner_does_not_require_amp_telemetry(self) -> None:
        document = criteria()
        document["contract"]["scope"]["runner"] = "OnPolicyRunner"
        document["approval"]["approved_contract_sha256"] = canonical_contract_sha256(
            document["contract"]
        )
        expected_scope = dict(SCOPE)
        expected_scope["runner"] = "OnPolicyRunner"
        report = assess(
            [(10.0, 0.5), (10.0, 0.5), (10.0, 0.5), (10.0, 0.5)],
            criteria_document=document,
            expected_scope=expected_scope,
            health={"state": "completed"},
            play_results=[
                {
                    "status": "completed",
                    "telemetry_status": "not_requested",
                    "metrics": {"termination_rate": 0.0},
                }
            ],
            training_finished=True,
        )
        self.assertEqual(report["convergence"], "converged")
        self.assertFalse(report["play"]["telemetry_required"])

    def test_missing_criteria_blocks_strong_advice_but_keeps_alert(self) -> None:
        report = MODULE.assess_training(
            summary(
                [(10.0, 0.5), (10.0, 0.5), (10.0, 0.5), (10.0, 0.5)],
                [{"progress": 3, "metric": "mean_reward"}],
            ),
            None,
            expected_scope=SCOPE,
            health={"state": "stalled"},
            training_finished=True,
        )
        self.assertEqual(report["recommendation"], "insufficient_evidence")
        self.assertEqual(report["convergence"], "indeterminate")
        self.assertTrue(report["operator_attention_required"])
        self.assertEqual(report["criteria"]["status"], "missing")

    def test_draft_criteria_blocks_strong_advice(self) -> None:
        report = assess(
            [(10.0, 0.8), (11.0, 0.7), (14.0, 0.5), (15.0, 0.4)],
            criteria_document=criteria(status="draft"),
            health={"state": "healthy"},
        )
        self.assertEqual(report["recommendation"], "insufficient_evidence")
        self.assertEqual(report["criteria"]["status"], "draft")

    def test_scope_mismatch_blocks_strong_advice(self) -> None:
        expected = dict(SCOPE)
        expected["run_id"] = "different-run"
        report = assess(
            [(10.0, 0.8), (11.0, 0.7), (14.0, 0.5), (15.0, 0.4)],
            expected_scope=expected,
            health={"state": "healthy"},
        )
        self.assertEqual(report["recommendation"], "insufficient_evidence")
        self.assertEqual(report["criteria"]["status"], "scope_mismatch")

    def test_contract_mutation_invalidates_approval(self) -> None:
        document = criteria()
        document["contract"]["windows"]["window_size"] = 3
        report = MODULE.assess_training(
            summary([(10.0, 0.8), (11.0, 0.7), (14.0, 0.5), (15.0, 0.4)]),
            document,
            expected_scope=SCOPE,
            health={"state": "healthy"},
        )
        self.assertEqual(report["recommendation"], "insufficient_evidence")
        self.assertEqual(report["criteria"]["status"], "approval_hash_mismatch")

    def test_cached_evidence_cannot_bypass_document_validation(self) -> None:
        document = criteria(status="draft")
        report = MODULE.assess_training(
            summary([(10.0, 0.8), (11.0, 0.7), (14.0, 0.5), (15.0, 0.4)]),
            document,
            expected_scope=SCOPE,
            criteria_evidence={
                "eligible": True,
                "status": "approved",
                "criteria_path": "/tmp/criteria.json",
                "criteria_file_sha256": "0" * 64,
            },
            health={"state": "healthy"},
        )
        self.assertEqual(report["recommendation"], "insufficient_evidence")
        self.assertEqual(report["criteria"]["status"], "draft")


if __name__ == "__main__":
    unittest.main()
