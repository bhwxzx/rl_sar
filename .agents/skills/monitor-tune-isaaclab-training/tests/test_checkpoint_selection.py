#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import os
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "select_checkpoint_candidates.py"
SPEC = importlib.util.spec_from_file_location("select_checkpoint_candidates", SCRIPT)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class CheckpointSelectionTests(unittest.TestCase):
    def test_inventory_marks_recent_checkpoint_unstable(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory).resolve()
            stable = root / "model_10.pt"
            recent = root / "model_20.pt"
            stable.write_bytes(b"stable")
            recent.write_bytes(b"recent")
            os.utime(stable, (100.0, 100.0))
            os.utime(recent, (195.0, 195.0))
            inventory = MODULE.inventory_checkpoints(
                root, stable_age_seconds=10.0, now=200.0
            )
            self.assertTrue(inventory[0]["stable"])
            self.assertFalse(inventory[1]["stable"])
            self.assertIsNone(inventory[1]["sha256"])

    def test_shortlist_keeps_latest_and_best_metric_neighbour(self) -> None:
        inventory = [
            {"path": f"/tmp/model_{step}.pt", "step": step, "stable": True}
            for step in (10, 20, 30, 40)
        ]
        summary = {
            "records": [
                {"progress": 10, "mean_reward": 1.0},
                {"progress": 20, "mean_reward": 9.0},
                {"progress": 30, "mean_reward": 4.0},
            ]
        }
        shortlist = MODULE.shortlist_checkpoints(
            inventory,
            maximum=3,
            summary=summary,
            training_metrics={"mean_reward": "maximize"},
        )
        paths = {item["path"] for item in shortlist}
        self.assertIn("/tmp/model_20.pt", paths)
        self.assertIn("/tmp/model_40.pt", paths)

    def test_pareto_comparison_does_not_force_tradeoff(self) -> None:
        shortlist = [
            {"path": "/tmp/model_10.pt"},
            {"path": "/tmp/model_20.pt"},
        ]
        results = [
            {
                "status": "completed",
                "checkpoint_path": "/tmp/model_10.pt",
                "metrics": {"reward": 10.0, "tilt": 0.4},
            },
            {
                "status": "completed",
                "checkpoint_path": "/tmp/model_20.pt",
                "metrics": {"reward": 9.0, "tilt": 0.2},
            },
        ]
        report = MODULE.compare_evaluations(
            shortlist, results, {"reward": "maximize", "tilt": "minimize"}
        )
        self.assertEqual(report["status"], "user_tradeoff_required")
        self.assertIsNone(report["recommended_checkpoint"])
        self.assertEqual(len(report["pareto_front"]), 2)

    def test_single_dominant_checkpoint_is_only_recommended(self) -> None:
        shortlist = [
            {"path": "/tmp/model_10.pt"},
            {"path": "/tmp/model_20.pt"},
        ]
        results = [
            {
                "status": "completed",
                "checkpoint_path": "/tmp/model_10.pt",
                "metrics": {"reward": 8.0, "tilt": 0.4},
            },
            {
                "status": "completed",
                "checkpoint_path": "/tmp/model_20.pt",
                "metrics": {"reward": 9.0, "tilt": 0.2},
            },
        ]
        report = MODULE.compare_evaluations(
            shortlist, results, {"reward": "maximize", "tilt": "minimize"}
        )
        self.assertEqual(report["recommended_checkpoint"], "/tmp/model_20.pt")
        self.assertTrue(report["pending_user_selection"])

    def test_amp_roa_incomplete_telemetry_blocks_checkpoint_recommendation(self) -> None:
        shortlist = [{"path": "/tmp/model_10.pt"}]
        results = [
            {
                "status": "completed",
                "checkpoint_path": "/tmp/model_10.pt",
                "runner": "OnPolicyRunnerAmpROA",
                "telemetry_required_for_complete_assessment": True,
                "telemetry_status": "partial",
                "missing_required_signals": ["applied_torque"],
                "metrics": {"reward": 10.0},
            }
        ]
        report = MODULE.compare_evaluations(
            shortlist, results, {"reward": "maximize"}
        )
        self.assertEqual(report["status"], "evaluation_required")
        self.assertIsNone(report["recommended_checkpoint"])
        self.assertEqual(
            report["incomplete_telemetry"]["/tmp/model_10.pt"][
                "missing_required_signals"
            ],
            ["applied_torque"],
        )


if __name__ == "__main__":
    unittest.main()
