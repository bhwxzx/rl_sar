#!/usr/bin/env python3

from __future__ import annotations

import copy
import json
import sys
import tempfile
import unittest
from pathlib import Path


SKILL = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SKILL / "scripts"))

from assessment_criteria import (  # noqa: E402
    SCOPE_FIELDS,
    canonical_contract_sha256,
    inspect_criteria_document,
    inspect_criteria_file,
)


SCOPE = {
    "task": "task-a",
    "run_id": "run-a",
    "backend": "isaaclab",
    "profile_id": "rsl-rl-amp-roa",
    "algorithm": "AMP-ROA",
    "runner": "OnPolicyRunnerAmpROA",
}


def approved_document() -> dict:
    contract = {
        "scope": dict(SCOPE),
        "windows": {
            "window_size": 10,
            "minimum_records": 20,
            "plateau_required_metrics": 1,
        },
        "required_metrics": {
            "mean_reward": {
                "direction": "maximize",
                "plateau_relative_tolerance": 0.01,
            }
        },
        "observed_metrics": {
            "action_noise_std": {"direction": "observe", "description": "report only"}
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
    return {
        "version": 2,
        "contract": contract,
        "approval": {
            "status": "approved",
            "approved_at": "2026-08-01T17:00:00+08:00",
            "approved_contract_sha256": canonical_contract_sha256(contract),
        },
    }


class AssessmentCriteriaTests(unittest.TestCase):
    def test_repository_template_is_an_unapproved_numberless_draft(self) -> None:
        template = json.loads(
            (SKILL / "assets" / "assessment-criteria-template.json").read_text(
                encoding="utf-8"
            )
        )
        report = inspect_criteria_document(template)
        self.assertEqual(report["status"], "draft")
        self.assertFalse(report["eligible"])
        self.assertEqual(report["errors"], [])

        def contract_numbers(value):
            if isinstance(value, dict):
                return [number for item in value.values() for number in contract_numbers(item)]
            if isinstance(value, list):
                return [number for item in value for number in contract_numbers(item)]
            return [value] if isinstance(value, (int, float)) and not isinstance(value, bool) else []

        self.assertEqual(contract_numbers(template["contract"]), [])

    def test_approved_exact_scope_is_eligible(self) -> None:
        report = inspect_criteria_document(approved_document(), expected_scope=SCOPE)
        self.assertEqual(report["status"], "approved")
        self.assertTrue(report["eligible"])
        self.assertTrue(report["approval"]["hash_matches"])

    def test_every_scope_field_is_exact(self) -> None:
        for field in SCOPE_FIELDS:
            with self.subTest(field=field):
                expected = dict(SCOPE)
                expected[field] += "-different"
                report = inspect_criteria_document(
                    approved_document(), expected_scope=expected
                )
                self.assertEqual(report["status"], "scope_mismatch")
                self.assertFalse(report["eligible"])
                self.assertEqual(report["scope_mismatches"][0]["field"], field)

    def test_contract_mutation_invalidates_approval_hash(self) -> None:
        document = approved_document()
        document["contract"]["required_metrics"]["mean_reward"][
            "plateau_relative_tolerance"
        ] = 0.02
        report = inspect_criteria_document(document, expected_scope=SCOPE)
        self.assertEqual(report["status"], "approval_hash_mismatch")
        self.assertFalse(report["approval"]["hash_matches"])

    def test_observed_metric_cannot_contain_decision_fields(self) -> None:
        document = approved_document()
        document["contract"]["observed_metrics"]["action_noise_std"][
            "plateau_relative_tolerance"
        ] = 0.1
        document["approval"]["approved_contract_sha256"] = canonical_contract_sha256(
            document["contract"]
        )
        report = inspect_criteria_document(document, expected_scope=SCOPE)
        self.assertEqual(report["status"], "invalid")
        self.assertTrue(any("decision fields" in error for error in report["errors"]))

    def test_approval_timestamp_requires_timezone(self) -> None:
        document = approved_document()
        document["approval"]["approved_at"] = "2026-08-01T17:00:00"
        report = inspect_criteria_document(document, expected_scope=SCOPE)
        self.assertEqual(report["status"], "invalid")

    def test_file_report_records_resolved_path_and_both_hashes(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "criteria.json"
            path.write_text(json.dumps(approved_document()), encoding="utf-8")
            _, report = inspect_criteria_file(path, expected_scope=SCOPE)
        self.assertEqual(report["criteria_path"], str(path.resolve()))
        self.assertEqual(len(report["criteria_file_sha256"]), 64)
        self.assertEqual(len(report["contract_sha256"]), 64)


if __name__ == "__main__":
    unittest.main()
