#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


SCRIPT = (
    Path(__file__).resolve().parents[4]
    / "scripts"
    / "reinforcement_learning"
    / "rsl_rl"
    / "policy_evaluation_telemetry.py"
)
SPEC = importlib.util.spec_from_file_location("policy_evaluation_telemetry", SCRIPT)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class SignalLedgerTests(unittest.TestCase):
    def test_one_signal_failure_does_not_hide_another_signal(self) -> None:
        ledger = MODULE.SignalLedger(
            {"joint_velocity": 1, "applied_torque": 1},
            required_signals={"joint_velocity", "applied_torque"},
        )
        missing = ledger.capture(
            "joint_velocity",
            lambda: (_ for _ in ()).throw(AttributeError("joint_vel missing")),
        )
        torque = ledger.capture("applied_torque", lambda: [1.0, 2.0])
        report = ledger.report()
        self.assertIsNone(missing)
        self.assertEqual(torque, [1.0, 2.0])
        self.assertEqual(report["status"], "partial")
        self.assertFalse(report["signals"]["joint_velocity"]["available"])
        self.assertTrue(report["signals"]["applied_torque"]["complete"])
        self.assertIn("AttributeError", report["signals"]["joint_velocity"]["error"])

    def test_intermittent_signal_is_available_but_incomplete(self) -> None:
        ledger = MODULE.SignalLedger(
            {"action": 2},
            required_signals={"action"},
        )
        ledger.capture("action", lambda: [0.0])
        ledger.capture(
            "action",
            lambda: (_ for _ in ()).throw(RuntimeError("sample failed")),
        )
        status = ledger.report()["signals"]["action"]
        self.assertTrue(status["available"])
        self.assertFalse(status["complete"])
        self.assertEqual(status["sample_count"], 1)
        self.assertEqual(status["error_count"], 1)

    def test_all_required_failures_are_unavailable(self) -> None:
        ledger = MODULE.SignalLedger(
            {"command": 1},
            required_signals={"command"},
        )
        ledger.capture(
            "command",
            lambda: (_ for _ in ()).throw(KeyError("base_velocity")),
        )
        self.assertEqual(ledger.report()["status"], "unavailable")

    def test_not_requested_lists_amp_roa_required_signals(self) -> None:
        report = MODULE.telemetry_report(
            requested=False,
            runner=MODULE.AMP_ROA_RUNNER,
            ledger=None,
        )
        self.assertEqual(report["telemetry_status"], "not_requested")
        self.assertTrue(report["telemetry_required_for_complete_assessment"])
        self.assertIn("joint_velocity", report["missing_required_signals"])
        self.assertIn("applied_torque", report["missing_required_signals"])
        self.assertIn("joint_effort_limits", report["missing_required_signals"])
        self.assertIn("joint_velocity_limits", report["missing_required_signals"])

    def test_non_amp_runner_does_not_get_amp_specific_requirements(self) -> None:
        required = MODULE.required_signals_for_runner("OnPolicyRunner")
        self.assertEqual(required, MODULE.BASE_REQUIRED_SIGNALS)
        self.assertNotIn("joint_velocity", required)
        self.assertFalse(
            MODULE.runner_requires_complete_telemetry("OnPolicyRunner")
        )

    def test_incomplete_source_does_not_emit_fake_zero_metric(self) -> None:
        ledger = MODULE.SignalLedger(
            {"joint_velocity": 1},
            required_signals={"joint_velocity"},
        )
        ledger.capture(
            "joint_velocity",
            lambda: (_ for _ in ()).throw(AttributeError("missing")),
        )
        availability = MODULE.metric_availability_report(
            ledger.report()["signals"],
            {"max_abs_joint_velocity": ("joint_velocity",)},
        )
        metrics = {}
        MODULE.record_complete_metric(
            metrics,
            availability,
            "max_abs_joint_velocity",
            lambda: 0.0,
        )
        self.assertNotIn("max_abs_joint_velocity", metrics)
        self.assertFalse(availability["max_abs_joint_velocity"]["available"])


class JointLimitTrackerTests(unittest.TestCase):
    def test_per_joint_limits_expose_violation_hidden_by_global_peak(self) -> None:
        tracker = MODULE.JointLimitTracker(
            ["leg_joint", "foot_joint"],
            [[120.0, 27.0]],
            [[20.0, 10.0]],
        )
        tracker.observe(
            [[104.0, 27.0]],
            [[13.2, 12.2]],
            step=7,
        )
        report = tracker.report()
        self.assertEqual(report["metrics"]["max_joint_effort_utilization"], 1.0)
        self.assertAlmostEqual(
            report["metrics"]["max_joint_velocity_utilization"], 1.22
        )
        self.assertEqual(
            report["metrics"]["joint_velocity_limit_violation_rate"], 0.5
        )
        self.assertEqual(
            report["peak_joints"]["max_joint_velocity_utilization"],
            "foot_joint",
        )
        foot = report["joint_metrics"][1]
        self.assertEqual(foot["effort_violation_count"], 0)
        self.assertEqual(foot["velocity_violation_count"], 1)
        self.assertEqual(foot["velocity_peak_step"], 7)

    def test_tracks_peak_steps_and_rates_across_multiple_steps(self) -> None:
        tracker = MODULE.JointLimitTracker(
            ["joint"],
            [[10.0]],
            [[5.0]],
        )
        tracker.observe([[2.0]], [[1.0]], step=1)
        tracker.observe([[11.0]], [[6.0]], step=4)
        report = tracker.report()
        self.assertEqual(report["sample_count"], 2)
        self.assertEqual(
            report["metrics"]["joint_effort_limit_violation_rate"], 0.5
        )
        self.assertEqual(
            report["metrics"]["joint_velocity_limit_violation_rate"], 0.5
        )
        self.assertEqual(report["peak_steps"]["max_joint_effort_utilization"], 4)
        self.assertEqual(report["peak_steps"]["max_joint_velocity_utilization"], 4)

    def test_rejects_invalid_limits_and_joint_order_shapes(self) -> None:
        with self.assertRaisesRegex(ValueError, "positive"):
            MODULE.JointLimitTracker(["joint"], [[0.0]], [[1.0]])
        with self.assertRaisesRegex(ValueError, "non-finite"):
            MODULE.JointLimitTracker(["joint"], [[1.0]], [[float("inf")]])
        with self.assertRaisesRegex(ValueError, "row width"):
            MODULE.JointLimitTracker(
                ["left", "right"],
                [[1.0]],
                [[1.0, 1.0]],
            )

    def test_rejects_sample_environment_mismatch(self) -> None:
        tracker = MODULE.JointLimitTracker(
            ["joint"],
            [[10.0], [10.0]],
            [[5.0], [5.0]],
        )
        with self.assertRaisesRegex(ValueError, "environment count"):
            tracker.observe([[1.0]], [[1.0]], step=0)


class BodyJitterTrackerTests(unittest.TestCase):
    def make_tracker(self) -> object:
        return MODULE.BodyJitterTracker(
            step_dt_seconds=0.5,
            command_segments=[
                {"start_step": 0, "end_step": 1, "command": [0.5, 0.0, 0.0]},
                {"start_step": 2, "end_step": 3, "command": [0.0, 0.3, 0.0]},
            ],
        )

    def test_records_whole_run_and_per_segment_jitter(self) -> None:
        tracker = self.make_tracker()
        tracker.observe([[0.0, 0.0, 0.0]], [[0.0, 0.0, 0.0]], [False], step=0)
        tracker.observe([[0.0, 0.0, 1.0]], [[3.0, 4.0, 0.0]], [False], step=1)
        tracker.observe([[0.0, 0.0, 5.0]], [[6.0, 8.0, 0.0]], [False], step=2)
        tracker.observe([[0.0, 0.0, 6.0]], [[9.0, 12.0, 0.0]], [False], step=3)

        report = tracker.report()
        angular_acceleration = report["whole_run"][
            "roll_pitch_angular_acceleration"
        ]
        vertical_acceleration = report["whole_run"]["vertical_acceleration"]
        self.assertEqual(angular_acceleration["sample_count"], 2)
        self.assertEqual(angular_acceleration["rms"], 10.0)
        self.assertEqual(angular_acceleration["p95"], 10.0)
        self.assertEqual(angular_acceleration["max"], 10.0)
        self.assertEqual(angular_acceleration["peak_step"], 1)
        self.assertEqual(vertical_acceleration["rms"], 2.0)
        self.assertEqual(report["excluded_transitions"]["command_segment"], 1)
        self.assertEqual(
            report["command_segments"][0]["statistics"][
                "roll_pitch_angular_acceleration"
            ]["sample_count"],
            1,
        )
        self.assertEqual(
            report["metrics"]["body_roll_pitch_angular_acceleration_rms"],
            10.0,
        )

    def test_excludes_reset_transition_but_keeps_following_sample(self) -> None:
        tracker = MODULE.BodyJitterTracker(
            step_dt_seconds=0.5,
            command_segments=[
                {"start_step": 0, "end_step": 2, "command": None},
            ],
        )
        tracker.observe([[0.0, 0.0, 0.0]], [[0.0, 0.0, 0.0]], [False], step=0)
        tracker.observe([[0.0, 0.0, 100.0]], [[100.0, 0.0, 0.0]], [True], step=1)
        tracker.observe([[0.0, 0.0, 101.0]], [[103.0, 4.0, 0.0]], [False], step=2)

        report = tracker.report()
        self.assertEqual(report["excluded_transitions"]["reset"], 1)
        self.assertEqual(
            report["whole_run"]["roll_pitch_angular_acceleration"]["rms"],
            10.0,
        )
        self.assertEqual(
            report["whole_run"]["vertical_acceleration"]["rms"],
            2.0,
        )

    def test_rejects_invalid_contract_and_samples(self) -> None:
        with self.assertRaisesRegex(ValueError, "finite and positive"):
            MODULE.BodyJitterTracker(
                step_dt_seconds=0.0,
                command_segments=[
                    {"start_step": 0, "end_step": 0, "command": None},
                ],
            )
        with self.assertRaisesRegex(ValueError, "contiguous"):
            MODULE.BodyJitterTracker(
                step_dt_seconds=0.5,
                command_segments=[
                    {"start_step": 1, "end_step": 1, "command": None},
                ],
            )
        tracker = MODULE.BodyJitterTracker(
            step_dt_seconds=0.5,
            command_segments=[
                {"start_step": 0, "end_step": 0, "command": None},
            ],
        )
        with self.assertRaisesRegex(ValueError, "non-finite"):
            tracker.observe(
                [[0.0, 0.0, float("nan")]],
                [[0.0, 0.0, 0.0]],
                [False],
                step=0,
            )


if __name__ == "__main__":
    unittest.main()
