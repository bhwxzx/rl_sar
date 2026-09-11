#!/usr/bin/env python3
"""Validate and fingerprint user-approved training assessment criteria."""

from __future__ import annotations

import hashlib
import json
import math
from datetime import datetime
from pathlib import Path
from typing import Any


SCOPE_FIELDS = ("task", "run_id", "backend", "profile_id", "algorithm", "runner")
OPERATORS = {"<=", ">=", "<", ">"}
APPROVAL_STATUSES = {"draft", "approved"}


class CriteriaError(ValueError):
    """Raised when a criteria file cannot be loaded as a JSON object."""


def canonical_contract_sha256(contract: dict[str, Any]) -> str:
    """Return a deterministic SHA-256 for the decision-bearing contract only."""
    try:
        encoded = json.dumps(
            contract,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise CriteriaError(f"criteria.contract is not canonical JSON: {exc}") from exc
    return hashlib.sha256(encoded).hexdigest()


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_criteria(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise CriteriaError(f"criteria does not exist: {path}") from exc
    except json.JSONDecodeError as exc:
        raise CriteriaError(
            f"invalid criteria JSON at line {exc.lineno}: {exc.msg}"
        ) from exc
    if not isinstance(value, dict):
        raise CriteriaError("criteria must be a JSON object")
    return value


def _is_finite_number(value: Any) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def _validate_gate_map(value: Any, label: str, errors: list[str]) -> None:
    if not isinstance(value, dict):
        errors.append(f"{label} must be an object")
        return
    for name, rule in value.items():
        if not isinstance(name, str) or not name or not isinstance(rule, dict):
            errors.append(f"each {label} entry must be a named object")
            continue
        if rule.get("op") not in OPERATORS:
            errors.append(f"{label}.{name}.op is invalid")
        if not _is_finite_number(rule.get("value")):
            errors.append(f"{label}.{name}.value must be finite")


def _validate_contract(contract: Any, *, draft: bool, errors: list[str]) -> None:
    if not isinstance(contract, dict):
        errors.append("criteria.contract must be an object")
        return

    scope = contract.get("scope")
    if not isinstance(scope, dict):
        errors.append("criteria.contract.scope must be an object")
    else:
        missing = [field for field in SCOPE_FIELDS if field not in scope]
        if missing:
            errors.append("criteria.contract.scope is missing: " + ", ".join(missing))
        extra = sorted(set(scope) - set(SCOPE_FIELDS))
        if extra:
            errors.append("criteria.contract.scope has unknown fields: " + ", ".join(extra))
        for field in SCOPE_FIELDS:
            value = scope.get(field)
            if draft and value is None:
                continue
            if not isinstance(value, str) or not value.strip():
                errors.append(f"criteria.contract.scope.{field} must be a non-empty string")

    windows = contract.get("windows")
    window_fields = ("window_size", "minimum_records", "plateau_required_metrics")
    if not isinstance(windows, dict):
        errors.append("criteria.contract.windows must be an object")
    else:
        for field in window_fields:
            if field not in windows:
                errors.append(f"criteria.contract.windows.{field} is required")
        if not draft or any(windows.get(field) is not None for field in window_fields):
            window_size = windows.get("window_size")
            minimum_records = windows.get("minimum_records")
            plateau_required = windows.get("plateau_required_metrics")
            if isinstance(window_size, bool) or not isinstance(window_size, int) or window_size < 2:
                errors.append("criteria.contract.windows.window_size must be an integer of at least 2")
            if (
                isinstance(minimum_records, bool)
                or not isinstance(minimum_records, int)
                or not isinstance(window_size, int)
                or isinstance(window_size, bool)
                or minimum_records < window_size * 2
            ):
                errors.append("criteria.contract.windows.minimum_records must cover two windows")
            if (
                isinstance(plateau_required, bool)
                or not isinstance(plateau_required, int)
                or plateau_required < 1
            ):
                errors.append("criteria.contract.windows.plateau_required_metrics must be positive")

    required_metrics = contract.get("required_metrics")
    if not isinstance(required_metrics, dict):
        errors.append("criteria.contract.required_metrics must be an object")
        required_metrics = {}
    elif not draft and not required_metrics:
        errors.append("criteria.contract.required_metrics must not be empty when approved")
    for name, rule in required_metrics.items():
        if not isinstance(name, str) or not name or not isinstance(rule, dict):
            errors.append("each required metric must be a named object")
            continue
        if rule.get("direction") not in {"maximize", "minimize"}:
            errors.append(f"required metric {name} has invalid direction")
        tolerance = rule.get("plateau_relative_tolerance")
        if not _is_finite_number(tolerance) or float(tolerance) < 0:
            errors.append(f"required metric {name} has invalid plateau tolerance")

    if isinstance(windows, dict) and isinstance(required_metrics, dict):
        plateau_required = windows.get("plateau_required_metrics")
        if (
            isinstance(plateau_required, int)
            and not isinstance(plateau_required, bool)
            and plateau_required > len(required_metrics)
        ):
            errors.append("plateau_required_metrics exceeds required metric count")

    observed_metrics = contract.get("observed_metrics")
    if not isinstance(observed_metrics, dict):
        errors.append("criteria.contract.observed_metrics must be an object")
    else:
        forbidden = {
            "plateau_relative_tolerance",
            "hard_min",
            "hard_max",
            "op",
            "value",
            "required",
        }
        for name, rule in observed_metrics.items():
            if not isinstance(name, str) or not name or not isinstance(rule, dict):
                errors.append("each observed metric must be a named object")
                continue
            unknown_decision_fields = sorted(forbidden.intersection(rule))
            if unknown_decision_fields:
                errors.append(
                    f"observed metric {name} contains decision fields: "
                    + ", ".join(unknown_decision_fields)
                )
            if "direction" in rule and rule["direction"] not in {
                "maximize",
                "minimize",
                "observe",
            }:
                errors.append(f"observed metric {name} has invalid direction")
            if "description" in rule and not isinstance(rule["description"], str):
                errors.append(f"observed metric {name}.description must be a string")

    hard_failures = contract.get("hard_failures")
    if not isinstance(hard_failures, dict):
        errors.append("criteria.contract.hard_failures must be an object")
    else:
        non_finite = hard_failures.get("non_finite_metrics")
        if not (draft and non_finite is None) and not isinstance(non_finite, bool):
            errors.append("criteria.contract.hard_failures.non_finite_metrics must be boolean")
        health_states = hard_failures.get("health_states")
        if not isinstance(health_states, list) or any(
            not isinstance(item, str) or not item for item in health_states
        ):
            errors.append("criteria.contract.hard_failures.health_states must be strings")
        _validate_gate_map(
            hard_failures.get("metric_limits"),
            "criteria.contract.hard_failures.metric_limits",
            errors,
        )

    play_gates = contract.get("play_gates")
    if not isinstance(play_gates, dict):
        errors.append("criteria.contract.play_gates must be an object")
    else:
        required_for_convergence = play_gates.get("required_for_convergence")
        if not (draft and required_for_convergence is None) and required_for_convergence is not True:
            errors.append(
                "criteria.contract.play_gates.required_for_convergence must be true"
            )
        metrics = play_gates.get("metrics")
        _validate_gate_map(metrics, "criteria.contract.play_gates.metrics", errors)
        if not draft and isinstance(metrics, dict) and not metrics:
            errors.append("criteria.contract.play_gates.metrics must not be empty when approved")


def inspect_criteria_document(
    document: dict[str, Any],
    *,
    expected_scope: dict[str, str] | None = None,
    criteria_path: Path | None = None,
    criteria_file_sha256: str | None = None,
) -> dict[str, Any]:
    """Return eligibility, provenance, and validation details without mutation."""
    errors: list[str] = []
    version = document.get("version")
    if version != 2:
        errors.append("criteria.version must be 2")
    approval = document.get("approval")
    approval_status = approval.get("status") if isinstance(approval, dict) else None
    if not isinstance(approval, dict):
        errors.append("criteria.approval must be an object")
    elif approval_status not in APPROVAL_STATUSES:
        errors.append("criteria.approval.status must be draft or approved")

    draft = approval_status == "draft"
    contract = document.get("contract")
    _validate_contract(contract, draft=draft, errors=errors)
    try:
        contract_sha256 = (
            canonical_contract_sha256(contract) if isinstance(contract, dict) else None
        )
    except CriteriaError as exc:
        errors.append(str(exc))
        contract_sha256 = None

    approved_at = approval.get("approved_at") if isinstance(approval, dict) else None
    approved_hash = (
        approval.get("approved_contract_sha256") if isinstance(approval, dict) else None
    )
    hash_matches: bool | None = None
    if draft:
        if approved_at is not None or approved_hash is not None:
            errors.append("draft approval fields approved_at and hash must be null")
    elif approval_status == "approved":
        if not isinstance(approved_at, str):
            errors.append("criteria.approval.approved_at must be an ISO-8601 string")
        else:
            try:
                parsed = datetime.fromisoformat(approved_at.replace("Z", "+00:00"))
                if parsed.tzinfo is None:
                    raise ValueError("timezone missing")
            except ValueError:
                errors.append("criteria.approval.approved_at must include a timezone")
        if (
            not isinstance(approved_hash, str)
            or len(approved_hash) != 64
            or any(char not in "0123456789abcdef" for char in approved_hash)
        ):
            errors.append("criteria.approval.approved_contract_sha256 must be lowercase SHA-256")
        else:
            hash_matches = approved_hash == contract_sha256
            if not hash_matches:
                errors.append("approved contract SHA-256 does not match current contract")

    scope = contract.get("scope", {}) if isinstance(contract, dict) else {}
    scope_mismatches: list[dict[str, Any]] = []
    if expected_scope is not None:
        for field in SCOPE_FIELDS:
            actual = scope.get(field) if isinstance(scope, dict) else None
            expected = expected_scope.get(field)
            if actual != expected:
                scope_mismatches.append(
                    {"field": field, "expected": expected, "actual": actual}
                )

    eligible = approval_status == "approved" and not errors and not scope_mismatches
    if eligible:
        status = "approved"
    elif approval_status == "draft" and not errors:
        status = "draft"
    elif hash_matches is False:
        status = "approval_hash_mismatch"
    elif scope_mismatches and not errors:
        status = "scope_mismatch"
    else:
        status = "invalid"

    return {
        "version": 1,
        "status": status,
        "eligible": eligible,
        "errors": errors,
        "scope": scope if isinstance(scope, dict) else {},
        "scope_mismatches": scope_mismatches,
        "contract_sha256": contract_sha256,
        "approval": {
            "status": approval_status,
            "approved_at": approved_at,
            "approved_contract_sha256": approved_hash,
            "hash_matches": hash_matches,
        },
        "criteria_path": str(criteria_path.resolve()) if criteria_path else None,
        "criteria_file_sha256": criteria_file_sha256,
    }


def inspect_criteria_file(
    path: Path,
    *,
    expected_scope: dict[str, str] | None = None,
) -> tuple[dict[str, Any], dict[str, Any]]:
    resolved = path.resolve()
    document = load_criteria(resolved)
    report = inspect_criteria_document(
        document,
        expected_scope=expected_scope,
        criteria_path=resolved,
        criteria_file_sha256=file_sha256(resolved),
    )
    return document, report
