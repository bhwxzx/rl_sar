#!/usr/bin/env python3
"""Produce non-executable continue/stop and convergence advice for one run."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from statistics import fmean
from typing import Any, Callable

from assessment_criteria import (
    CriteriaError,
    SCOPE_FIELDS,
    inspect_criteria_document,
    inspect_criteria_file,
)


class AssessmentError(ValueError):
    """Raised when advisor evidence is malformed."""


OPERATORS: dict[str, Callable[[float, float], bool]] = {
    "<=": lambda actual, limit: actual <= limit,
    ">=": lambda actual, limit: actual >= limit,
    "<": lambda actual, limit: actual < limit,
    ">": lambda actual, limit: actual > limit,
}
RUNNERS_REQUIRING_COMPLETE_TELEMETRY = {"OnPolicyRunnerAmpROA"}


def load_object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise AssessmentError(f"{label} does not exist: {path}") from exc
    except json.JSONDecodeError as exc:
        raise AssessmentError(
            f"invalid {label} JSON at line {exc.lineno}: {exc.msg}"
        ) from exc
    if not isinstance(value, dict):
        raise AssessmentError(f"{label} must be a JSON object")
    if label == "Play result" and value.get("version") in {2, 3}:
        sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "scripts/reinforcement_learning/rsl_rl"))
        from policy_evaluation_evidence import validate_evaluation_bundle
        try:
            validate_evaluation_bundle(path)
        except (ValueError, OSError) as exc:
            raise AssessmentError(str(exc)) from exc
    return value


def _finite_values(records: list[dict[str, Any]], metric: str) -> list[float]:
    values: list[float] = []
    for record in records:
        value = record.get(metric)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            continue
        numeric = float(value)
        if math.isfinite(numeric):
            values.append(numeric)
    return values


def _metric_trend(
    name: str,
    rule: dict[str, Any],
    prior: list[dict[str, Any]],
    recent: list[dict[str, Any]],
) -> dict[str, Any]:
    prior_values = _finite_values(prior, name)
    recent_values = _finite_values(recent, name)
    if len(prior_values) != len(prior) or len(recent_values) != len(recent):
        return {
            "metric": name,
            "status": "insufficient",
            "prior_samples": len(prior_values),
            "recent_samples": len(recent_values),
        }
    prior_mean = fmean(prior_values)
    recent_mean = fmean(recent_values)
    scale = max(abs(prior_mean), 1.0e-12)
    raw_change = (recent_mean - prior_mean) / scale
    relative_improvement = (
        raw_change if rule["direction"] == "maximize" else -raw_change
    )
    tolerance = float(rule["plateau_relative_tolerance"])
    return {
        "metric": name,
        "status": "available",
        "direction": rule["direction"],
        "prior_mean": prior_mean,
        "recent_mean": recent_mean,
        "relative_improvement": relative_improvement,
        "plateau_tolerance": tolerance,
        "plateaued": abs(relative_improvement) <= tolerance,
        "meaningfully_improving": relative_improvement > tolerance,
        "meaningfully_degrading": relative_improvement < -tolerance,
    }


def _observed_metric_trend(
    name: str,
    rule: dict[str, Any],
    prior: list[dict[str, Any]],
    recent: list[dict[str, Any]],
) -> dict[str, Any]:
    prior_values = _finite_values(prior, name)
    recent_values = _finite_values(recent, name)
    result: dict[str, Any] = {
        "metric": name,
        "decision_bearing": False,
        "direction": rule.get("direction", "observe"),
        "description": rule.get("description"),
    }
    if len(prior_values) != len(prior) or len(recent_values) != len(recent):
        result.update(
            status="insufficient",
            prior_samples=len(prior_values),
            recent_samples=len(recent_values),
        )
        return result
    prior_mean = fmean(prior_values)
    recent_mean = fmean(recent_values)
    result.update(
        status="available",
        prior_mean=prior_mean,
        recent_mean=recent_mean,
        relative_change=(recent_mean - prior_mean) / max(abs(prior_mean), 1.0e-12),
    )
    return result


def _gate_results(
    values: dict[str, Any],
    gates: dict[str, Any],
    *,
    source: str,
) -> list[dict[str, Any]]:
    checks: list[dict[str, Any]] = []
    for metric, gate in gates.items():
        actual = values.get(metric)
        limit = gate["value"]
        if (
            isinstance(actual, bool)
            or not isinstance(actual, (int, float))
            or not math.isfinite(float(actual))
        ):
            checks.append(
                {
                    "source": source,
                    "metric": metric,
                    "status": "missing_or_nonfinite",
                    "passed": None,
                }
            )
            continue
        passed = OPERATORS[gate["op"]](float(actual), float(limit))
        checks.append(
            {
                "source": source,
                "metric": metric,
                "actual": float(actual),
                "op": gate["op"],
                "limit": float(limit),
                "status": "passed" if passed else "failed",
                "passed": passed,
            }
        )
    return checks


def _hard_metric_results(
    records: list[dict[str, Any]],
    window_size: int,
    gates: dict[str, Any],
) -> dict[str, Any]:
    if not gates:
        return {"complete": True, "failed": False, "checks": []}
    recent = records[-window_size:] if len(records) >= window_size else records
    means: dict[str, float] = {}
    for metric in gates:
        values = _finite_values(recent, metric)
        if len(values) == len(recent) and values:
            means[metric] = fmean(values)
    checks = _gate_results(means, gates, source="latest_training_window")
    return {
        "complete": bool(checks) and all(item["passed"] is not None for item in checks),
        "failed": any(item["passed"] is False for item in checks),
        "checks": checks,
    }


def _play_gate_results(
    play_results: list[dict[str, Any]],
    gates: dict[str, Any],
    *,
    runner: str | None,
) -> dict[str, Any]:
    if not play_results:
        return {
            "available": False,
            "passed": None,
            "checks": [],
            "telemetry_required": runner in RUNNERS_REQUIRING_COMPLETE_TELEMETRY,
            "telemetry_complete": None,
            "telemetry_checks": [],
            "eligible_for_convergence": False,
        }
    checks: list[dict[str, Any]] = []
    telemetry_checks: list[dict[str, Any]] = []
    telemetry_required = runner in RUNNERS_REQUIRING_COMPLETE_TELEMETRY
    for result_index, result in enumerate(play_results):
        metrics = result.get("metrics")
        if result.get("status") != "completed" or not isinstance(metrics, dict):
            checks.append(
                {
                    "result_index": result_index,
                    "status": "invalid_result",
                    "passed": False,
                }
            )
            telemetry_checks.append(
                {
                    "result_index": result_index,
                    "status": "invalid_result",
                    "complete": False,
                    "missing_required_signals": [],
                }
            )
            continue
        for check in _gate_results(metrics, gates, source="play"):
            check["result_index"] = result_index
            checks.append(check)
        telemetry_status = result.get("telemetry_status")
        missing_signals = result.get("missing_required_signals", [])
        if not isinstance(missing_signals, list):
            missing_signals = []
        telemetry_complete = not telemetry_required or telemetry_status == "complete"
        telemetry_checks.append(
            {
                "result_index": result_index,
                "runner": runner,
                "status": telemetry_status,
                "complete": telemetry_complete,
                "missing_required_signals": missing_signals,
            }
        )
    metrics_passed = bool(checks) and all(
        item.get("passed") is True for item in checks
    )
    telemetry_complete = bool(telemetry_checks) and all(
        item["complete"] is True for item in telemetry_checks
    )
    return {
        "available": True,
        "passed": metrics_passed,
        "checks": checks,
        "telemetry_required": telemetry_required,
        "telemetry_complete": telemetry_complete,
        "telemetry_checks": telemetry_checks,
        "eligible_for_convergence": metrics_passed and telemetry_complete,
    }


def _missing_criteria_report(expected_scope: dict[str, str] | None) -> dict[str, Any]:
    return {
        "version": 1,
        "status": "missing",
        "eligible": False,
        "errors": ["no criteria file was supplied"],
        "scope": {},
        "scope_mismatches": [],
        "contract_sha256": None,
        "approval": {
            "status": None,
            "approved_at": None,
            "approved_contract_sha256": None,
            "hash_matches": None,
        },
        "criteria_path": None,
        "criteria_file_sha256": None,
        "expected_scope": expected_scope,
    }


def assess_training(
    summary: dict[str, Any],
    criteria: dict[str, Any] | None,
    *,
    expected_scope: dict[str, str] | None = None,
    criteria_evidence: dict[str, Any] | None = None,
    health: dict[str, Any] | None = None,
    play_results: list[dict[str, Any]] | None = None,
    training_finished: bool = False,
) -> dict[str, Any]:
    """Assess evidence without mutating or signaling the training process."""
    records = summary.get("records")
    if not isinstance(records, list) or any(not isinstance(item, dict) for item in records):
        raise AssessmentError("summary.records must be an array of objects")
    non_finite = summary.get("non_finite_metrics", [])
    if not isinstance(non_finite, list):
        raise AssessmentError("summary.non_finite_metrics must be an array")

    if criteria is None:
        criteria_report = _missing_criteria_report(expected_scope)
    else:
        evidence_path = (
            criteria_evidence.get("criteria_path")
            if isinstance(criteria_evidence, dict)
            else None
        )
        evidence_file_hash = (
            criteria_evidence.get("criteria_file_sha256")
            if isinstance(criteria_evidence, dict)
            else None
        )
        criteria_report = inspect_criteria_document(
            criteria,
            expected_scope=expected_scope,
            criteria_path=Path(evidence_path) if isinstance(evidence_path, str) else None,
            criteria_file_sha256=(
                evidence_file_hash if isinstance(evidence_file_hash, str) else None
            ),
        )
    criteria_report = dict(criteria_report)
    criteria_report["expected_scope"] = expected_scope
    if expected_scope is None or any(
        not isinstance(expected_scope.get(field), str) or not expected_scope[field]
        for field in SCOPE_FIELDS
    ):
        criteria_report["eligible"] = False
        criteria_report["status"] = "scope_unresolved"
        criteria_report.setdefault("errors", []).append("current run scope is incomplete")
    elif summary.get("profile_id") != expected_scope["profile_id"]:
        criteria_report["eligible"] = False
        criteria_report["status"] = "scope_mismatch"
        criteria_report.setdefault("scope_mismatches", []).append(
            {
                "field": "summary.profile_id",
                "expected": expected_scope["profile_id"],
                "actual": summary.get("profile_id"),
            }
        )

    health_state = health.get("state") if isinstance(health, dict) else None
    safety_alerts: list[dict[str, Any]] = []
    if non_finite:
        safety_alerts.append({"kind": "non_finite_metrics", "details": non_finite})
    if health_state == "stalled":
        safety_alerts.append({"kind": "confirmed_stall", "health_state": health_state})

    if not criteria_report.get("eligible"):
        return {
            "version": 2,
            "advisory_only": True,
            "recommendation": "insufficient_evidence",
            "reason": "criteria_not_approved_or_scope_mismatch",
            "convergence": "indeterminate",
            "training_finished": training_finished,
            "profile_id": summary.get("profile_id"),
            "record_count": len(records),
            "window_size": None,
            "health_state": health_state,
            "non_finite_metrics": non_finite,
            "trends": [],
            "observed_trends": [],
            "counts": {
                "required_available": 0,
                "plateaued": 0,
                "improving": 0,
                "degrading": 0,
                "hard_failures": 0,
            },
            "hard_failures": {"complete": False, "failed": False, "checks": []},
            "play": {
                "available": False,
                "passed": None,
                "checks": [],
                "telemetry_required": False,
                "telemetry_complete": None,
                "telemetry_checks": [],
                "eligible_for_convergence": False,
            },
            "criteria": criteria_report,
            "safety_alerts": safety_alerts,
            "operator_attention_required": bool(safety_alerts),
            "pending_user_decision": True,
        }

    contract = criteria["contract"]
    windows = contract["windows"]
    window_size = windows["window_size"]
    minimum_records = windows["minimum_records"]
    plateau_required = windows["plateau_required_metrics"]
    enough_records = len(records) >= minimum_records
    trends: list[dict[str, Any]] = []
    observed_trends: list[dict[str, Any]] = []
    if len(records) >= window_size * 2:
        prior = records[-2 * window_size : -window_size]
        recent = records[-window_size:]
        trends = [
            _metric_trend(name, rule, prior, recent)
            for name, rule in contract["required_metrics"].items()
        ]
        observed_trends = [
            _observed_metric_trend(name, rule, prior, recent)
            for name, rule in contract["observed_metrics"].items()
        ]
    required_available = [item for item in trends if item["status"] == "available"]
    plateau_count = sum(item.get("plateaued") is True for item in required_available)
    improving_count = sum(
        item.get("meaningfully_improving") is True for item in required_available
    )
    degrading_count = sum(
        item.get("meaningfully_degrading") is True for item in required_available
    )

    hard_config = contract["hard_failures"]
    hard_metrics = _hard_metric_results(
        records,
        window_size,
        hard_config["metric_limits"],
    )
    configured_non_finite = bool(non_finite) and hard_config["non_finite_metrics"]
    configured_health_failure = health_state in hard_config["health_states"]
    invalid = configured_non_finite or configured_health_failure or hard_metrics["failed"]
    if hard_metrics["failed"]:
        safety_alerts.append(
            {"kind": "approved_hard_metric_failure", "checks": hard_metrics["checks"]}
        )

    play = _play_gate_results(
        play_results or [],
        contract["play_gates"]["metrics"],
        runner=expected_scope["runner"],
    )
    trends_complete = enough_records and len(required_available) >= plateau_required
    hard_evidence_complete = hard_metrics["complete"]

    if invalid:
        recommendation = "recommend_stop_invalid"
        reason = "approved_hard_failure"
    elif not training_finished:
        if health_state in {"observing", "suspect"}:
            recommendation = "continue_and_recheck"
            reason = "training_progress_requires_another_snapshot"
        elif health_state != "healthy":
            recommendation = "insufficient_evidence"
            reason = "running_process_health_not_confirmed"
        elif not trends_complete or not hard_evidence_complete:
            recommendation = "continue_and_recheck"
            reason = "incomplete_approved_metric_evidence"
        elif plateau_count >= plateau_required and improving_count == 0:
            recommendation = "consider_stop_plateau"
            reason = "required_metrics_plateaued"
        elif improving_count > 0 and degrading_count == 0:
            recommendation = "continue"
            reason = "meaningful_improvement_without_hard_failure"
        else:
            recommendation = "continue_and_recheck"
            reason = "mixed_metric_trends"
    elif not trends_complete or not hard_evidence_complete:
        recommendation = "insufficient_evidence"
        reason = "incomplete_approved_metric_evidence"
    elif plateau_count >= plateau_required and improving_count == 0:
        recommendation = "consider_stop_plateau"
        reason = "required_metrics_plateaued"
    else:
        recommendation = "continue_and_recheck"
        reason = "training_finished_no_continue_advice"

    if not training_finished:
        convergence = "not_assessed_while_running"
    elif invalid:
        convergence = "not_converged"
    elif not trends_complete or not hard_evidence_complete or not play["available"]:
        convergence = "indeterminate"
    elif plateau_count >= plateau_required and play["eligible_for_convergence"]:
        convergence = "converged"
    elif (
        plateau_count >= plateau_required
        and play["passed"] is True
        and play["telemetry_complete"] is False
    ):
        convergence = "indeterminate"
    elif plateau_count >= plateau_required:
        convergence = "plateaued_with_defects"
    else:
        convergence = "not_converged"

    return {
        "version": 2,
        "advisory_only": True,
        "recommendation": recommendation,
        "reason": reason,
        "convergence": convergence,
        "training_finished": training_finished,
        "profile_id": summary.get("profile_id"),
        "record_count": len(records),
        "window_size": window_size,
        "health_state": health_state,
        "non_finite_metrics": non_finite,
        "trends": trends,
        "observed_trends": observed_trends,
        "counts": {
            "required_available": len(required_available),
            "plateaued": plateau_count,
            "improving": improving_count,
            "degrading": degrading_count,
            "hard_failures": int(invalid),
        },
        "hard_failures": hard_metrics,
        "play": play,
        "criteria": criteria_report,
        "safety_alerts": safety_alerts,
        "operator_attention_required": bool(safety_alerts),
        "pending_user_decision": True,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("summary")
    parser.add_argument("--criteria")
    parser.add_argument("--task", required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--backend", required=True)
    parser.add_argument("--profile-id", required=True)
    parser.add_argument("--algorithm", required=True)
    parser.add_argument("--runner", required=True)
    parser.add_argument("--health")
    parser.add_argument("--play-result", action="append", default=[])
    parser.add_argument("--training-finished", action="store_true")
    parser.add_argument("--output")
    args = parser.parse_args()
    expected_scope = {
        "task": args.task,
        "run_id": args.run_id,
        "backend": args.backend,
        "profile_id": args.profile_id,
        "algorithm": args.algorithm,
        "runner": args.runner,
    }
    try:
        criteria = None
        evidence = None
        if args.criteria:
            criteria_path = Path(args.criteria)
            if not criteria_path.is_absolute():
                parser.error("--criteria must be absolute")
            criteria, evidence = inspect_criteria_file(
                criteria_path,
                expected_scope=expected_scope,
            )
        result = assess_training(
            load_object(Path(args.summary), "summary"),
            criteria,
            expected_scope=expected_scope,
            criteria_evidence=evidence,
            health=load_object(Path(args.health), "health") if args.health else None,
            play_results=[
                load_object(Path(path), "Play result") for path in args.play_result
            ],
            training_finished=args.training_finished,
        )
    except (AssessmentError, CriteriaError) as exc:
        parser.error(str(exc))
    encoded = json.dumps(result, indent=2, sort_keys=True, ensure_ascii=False, allow_nan=False)
    if args.output:
        output = Path(args.output)
        if not output.is_absolute():
            parser.error("--output must be absolute")
        output.write_text(encoded + "\n", encoding="utf-8")
    else:
        print(encoded)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
