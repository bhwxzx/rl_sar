#!/usr/bin/env python3
"""Write one immutable, structured policy-tuning experience event."""

from __future__ import annotations

from evidence_provenance import require_source_path

import argparse
import hashlib
import json
import os
import re
import sys
import time
from pathlib import Path
from typing import Any

from capture_effective_training_config import (
    DEFAULT_MAX_DIFF_ENTRIES,
    EffectiveConfigError,
    SHA256_RE,
    compare_effective_configs,
    load_and_validate_effective_config,
)
from capture_run_identity import (
    RunIdentityError,
    validate_run_identity,
)


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = Path(__file__).resolve().parents[4]
RSL_RL_DIR = REPO_ROOT / "scripts" / "reinforcement_learning" / "rsl_rl"
sys.path.insert(0, str(RSL_RL_DIR))

from policy_evaluation_evidence import (  # noqa: E402
    EvaluationEvidenceError,
    validate_evaluation_bundle,
)
from policy_export_evidence import (  # noqa: E402
    PolicyExportEvidenceError,
    validate_checkpoint_selection,
    validate_export_bundle,
)


EVENT_TYPES = {
    "run_snapshot",
    "assessment",
    "decision",
    "checkpoint_evaluation",
    "evaluation_batch",
    "checkpoint_selection",
    "export",
    "archive",
    "feedback",
    "recommendation",
}
EVIDENCE_EVENT_TYPES = {
    "assessment",
    "checkpoint_evaluation",
    "evaluation_batch",
    "checkpoint_selection",
    "export",
    "archive",
    "feedback",
}
OUTCOME_EVENT_TYPES = {"assessment", "checkpoint_evaluation", "feedback"}
SLUG_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
TIMESTAMP_RE = re.compile(r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(?:\.\d+)?(?:Z|[+-]\d{2}:\d{2})$")


class ExperienceError(ValueError):
    """Raised when an experience event is unsafe or malformed."""


def _reject_symlink_components(path: Path, *, label: str) -> None:
    if not path.is_absolute():
        raise ExperienceError(f"{label} must be absolute")
    current = Path(path.anchor)
    for component in path.parts[1:]:
        current /= component
        if current.is_symlink():
            raise ExperienceError(
                f"{label} contains a symlinked path component: {current}"
            )


def _write_new_output(path: Path, encoded: str) -> None:
    if not path.is_absolute():
        raise ExperienceError("--output must be a new absolute path")
    _reject_symlink_components(path.parent, label="--output")
    if not path.parent.is_dir():
        raise ExperienceError("--output parent directory does not exist")
    if path.exists() or path.is_symlink():
        raise ExperienceError("--output must be a new absolute path")
    try:
        with path.open("x", encoding="utf-8") as stream:
            stream.write(encoded)
    except FileExistsError as exc:
        raise ExperienceError("--output must be a new absolute path") from exc


def _load(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ExperienceError(f"event file does not exist: {path}") from exc
    except json.JSONDecodeError as exc:
        raise ExperienceError(f"invalid event JSON at line {exc.lineno}: {exc.msg}") from exc
    if not isinstance(value, dict):
        raise ExperienceError("event must be a JSON object")
    return value


def _validate_reference_schema(value: Any, *, label: str) -> None:
    if not isinstance(value, dict) or value.get("status") not in {
        "available",
        "unavailable",
    }:
        raise ExperienceError(f"{label} must declare available or unavailable")
    if value["status"] == "unavailable":
        if set(value) != {"status", "reason"} or not isinstance(
            value.get("reason"), str
        ) or not value["reason"].strip():
            raise ExperienceError(f"unavailable {label} requires one reason")
        return
    if set(value) != {"status", "path", "sha256"}:
        raise ExperienceError(
            f"available {label} requires only status, path, and sha256"
        )
    path = value.get("path")
    if (
        not isinstance(path, str)
        or not Path(path).is_absolute()
        or ".." in Path(path).parts
    ):
        raise ExperienceError(f"{label}.path must be absolute without traversal")
    digest = value.get("sha256")
    if not isinstance(digest, str) or not SHA256_RE.fullmatch(digest):
        raise ExperienceError(f"{label}.sha256 must be SHA-256")


def _validate_available_reference_schema(value: Any, *, label: str) -> None:
    if not isinstance(value, dict) or set(value) != {"path", "sha256"}:
        raise ExperienceError(f"{label} requires path and sha256")
    _validate_reference_schema(
        {"status": "available", **value},
        label=label,
    )


def _validate_policy_binding_schema(value: Any) -> None:
    if not isinstance(value, dict) or value.get("status") not in {
        "available",
        "unavailable",
    }:
        raise ExperienceError(
            "feedback evidence.policy_binding must declare availability"
        )
    if value["status"] == "unavailable":
        if set(value) != {"status", "reason"} or not isinstance(
            value.get("reason"), str
        ) or not value["reason"].strip():
            raise ExperienceError(
                "unavailable feedback policy binding requires one reason"
            )
        return
    if set(value) != {"status", "kind", "path", "sha256"}:
        raise ExperienceError("available feedback policy binding is malformed")
    if value.get("kind") not in {
        "checkpoint_selection",
        "export",
        "archive",
    }:
        raise ExperienceError("feedback policy binding kind is unsupported")
    _validate_reference_schema(
        {
            "status": "available",
            "path": value.get("path"),
            "sha256": value.get("sha256"),
        },
        label="feedback evidence.policy_binding",
    )


def _validate_outcome_schema(value: Any, *, event_type: str) -> None:
    if not isinstance(value, dict) or value.get("status") not in {
        "available",
        "unavailable",
    }:
        raise ExperienceError("evidence.outcome must declare availability")
    if value["status"] == "unavailable":
        if set(value) != {"status", "reason"} or not isinstance(
            value.get("reason"), str
        ) or not value["reason"].strip():
            raise ExperienceError("unavailable outcome requires one reason")
        return
    if event_type not in OUTCOME_EVENT_TYPES:
        raise ExperienceError(f"{event_type} cannot declare an available outcome")
    expected_keys = {
        "status",
        "baseline",
        "parameter_changes",
        "result_window",
        "observed_effect",
    }
    if set(value) != expected_keys:
        raise ExperienceError("available outcome contract has unexpected fields")
    baseline = value.get("baseline")
    if not isinstance(baseline, dict) or set(baseline) != {
        "run_identity",
        "effective_config",
    }:
        raise ExperienceError(
            "outcome baseline requires run_identity and effective_config"
        )
    _validate_available_reference_schema(
        baseline.get("run_identity"),
        label="outcome baseline run identity",
    )
    config_reference = baseline.get("effective_config")
    if not isinstance(config_reference, dict) or set(config_reference) != {
        "path",
        "sha256",
        "effective_config_fingerprint",
        "reward_fingerprint",
    }:
        raise ExperienceError("outcome baseline effective config is malformed")
    _validate_reference_schema(
        {
            "status": "available",
            "path": config_reference.get("path"),
            "sha256": config_reference.get("sha256"),
        },
        label="outcome baseline effective config",
    )
    for field in ("effective_config_fingerprint", "reward_fingerprint"):
        digest = config_reference.get(field)
        if not isinstance(digest, str) or not SHA256_RE.fullmatch(digest):
            raise ExperienceError(
                f"outcome baseline effective config {field} must be SHA-256"
            )
    if not isinstance(value.get("parameter_changes"), dict):
        raise ExperienceError("outcome parameter_changes must be a complete diff object")
    result_window = value.get("result_window")
    if not isinstance(result_window, dict) or set(result_window) != {
        "path",
        "sha256",
        "start_step",
        "end_step",
    }:
        raise ExperienceError("outcome result_window is malformed")
    _validate_reference_schema(
        {
            "status": "available",
            "path": result_window.get("path"),
            "sha256": result_window.get("sha256"),
        },
        label="outcome result window",
    )
    start = result_window.get("start_step")
    end = result_window.get("end_step")
    if (
        isinstance(start, bool)
        or not isinstance(start, int)
        or isinstance(end, bool)
        or not isinstance(end, int)
        or start < 0
        or end < start
    ):
        raise ExperienceError("outcome result window steps are invalid")
    observed = value.get("observed_effect")
    if not isinstance(observed, dict) or set(observed) != {
        "summary",
        "observations",
    }:
        raise ExperienceError("outcome observed_effect is malformed")
    if not isinstance(observed.get("summary"), str) or not observed["summary"].strip():
        raise ExperienceError("outcome observed_effect summary is required")
    observations = observed.get("observations")
    if (
        not isinstance(observations, list)
        or not observations
        or any(not isinstance(item, str) or not item.strip() for item in observations)
    ):
        raise ExperienceError("outcome observed_effect observations are required")


def validate_event(event: dict[str, Any]) -> None:
    version = event.get("version")
    if version not in {1, 2, 3, 4, 5}:
        raise ExperienceError("event.version must be 1, 2, 3, or 4")
    for field in ("event_id", "task", "run_id", "algorithm"):
        value = event.get(field)
        if not isinstance(value, str) or not SLUG_RE.fullmatch(value):
            raise ExperienceError(f"{field} must be a safe ASCII identifier")
    if event.get("event_type") not in EVENT_TYPES:
        raise ExperienceError("event_type is unsupported")
    if event["event_type"] == "evaluation_batch" and version != 5:
        raise ExperienceError("evaluation_batch requires event version 5")
    timestamp = event.get("recorded_at")
    if not isinstance(timestamp, str) or not TIMESTAMP_RE.fullmatch(timestamp):
        raise ExperienceError("recorded_at must be an ISO-8601 timestamp with timezone")
    for field in ("context", "parameters", "evidence", "analysis"):
        if not isinstance(event.get(field), dict):
            raise ExperienceError(f"{field} must be an object")
    context = event["context"]
    for field in (
        "observation_fingerprint",
        "reward_fingerprint",
        "deployment_fingerprint",
    ):
        value = context.get(field)
        if not isinstance(value, str) or not value:
            raise ExperienceError(f"context.{field} must be a non-empty string")
    confidence = event["analysis"].get("confidence")
    if confidence not in {"low", "medium", "high"}:
        raise ExperienceError("analysis.confidence must be low, medium, or high")
    if not isinstance(event.get("next_suggestion"), str):
        raise ExperienceError("next_suggestion must be a string")
    if event["event_type"] == "feedback" and event["evidence"].get(
        "source"
    ) not in {"sim2sim", "sim2real"}:
        raise ExperienceError("feedback evidence.source must be sim2sim or sim2real")
    if version in {2, 3, 4, 5}:
        try:
            validate_run_identity(event.get("run_identity"))
        except RunIdentityError as exc:
            raise ExperienceError(str(exc)) from exc
        run_identity = event["run_identity"]
        for field in ("task", "run_id", "algorithm"):
            if event[field] != run_identity[field]:
                raise ExperienceError(
                    f"event.{field} must match run_identity.{field}"
                )
    if version in {3, 4, 5}:
        reference = event["evidence"].get("effective_config")
        expected_keys = {
            "effective_config_fingerprint",
            "path",
            "reward_fingerprint",
            "sha256",
        }
        if not isinstance(reference, dict) or set(reference) != expected_keys:
            raise ExperienceError(
                "version-3/4 evidence.effective_config must contain path, sha256, "
                "effective_config_fingerprint, and reward_fingerprint"
            )
        path = reference["path"]
        if (
            not isinstance(path, str)
            or not Path(path).is_absolute()
            or ".." in Path(path).parts
        ):
            raise ExperienceError("effective config evidence path must be absolute")
        for field in (
            "sha256",
            "effective_config_fingerprint",
            "reward_fingerprint",
        ):
            value = reference[field]
            if not isinstance(value, str) or not SHA256_RE.fullmatch(value):
                raise ExperienceError(f"effective config {field} must be SHA-256")
        if event["context"]["reward_fingerprint"] != reference["reward_fingerprint"]:
            raise ExperienceError(
                "context.reward_fingerprint must match effective config reference"
            )
    if version >= 4:
        evidence = event["evidence"]
        if event["event_type"] in EVIDENCE_EVENT_TYPES:
            _validate_reference_schema(
                evidence.get("event"),
                label=f"{event['event_type']} evidence.event",
            )
        elif "event" in evidence:
            raise ExperienceError(
                f"{event['event_type']} must not declare evidence.event"
            )
        if event["event_type"] == "feedback":
            _validate_policy_binding_schema(evidence.get("policy_binding"))
        elif "policy_binding" in evidence:
            raise ExperienceError(
                f"{event['event_type']} must not declare policy_binding"
            )
        _validate_outcome_schema(
            evidence.get("outcome"),
            event_type=event["event_type"],
        )


def validate_effective_config_binding(
    root: Path,
    event: dict[str, Any],
) -> dict[str, Any]:
    """Validate a version-3/4 event's host-local immutable config reference."""
    if event.get("version") not in {3, 4, 5}:
        raise ExperienceError("verifiable experience events must use version 3 or 4")
    reference = event["evidence"]["effective_config"]
    path = Path(reference["path"])
    try:
        require_source_path(path, root / event["task"] / event["run_id"], "config")
    except ValueError as exc:
        raise ExperienceError(f"effective config must be a direct source artifact for this run: {exc}") from exc
    try:
        config, _ = load_and_validate_effective_config(
            path,
            expected_sha256=reference["sha256"],
            run_identity=event["run_identity"],
        )
    except EffectiveConfigError as exc:
        raise ExperienceError(str(exc)) from exc
    if (
        config["fingerprints"]["effective_config"]
        != reference["effective_config_fingerprint"]
    ):
        raise ExperienceError("effective config fingerprint reference mismatch")
    if config["fingerprints"]["reward"] != reference["reward_fingerprint"]:
        raise ExperienceError("effective config reward fingerprint reference mismatch")
    return config


def _stable_file_reference(
    value: dict[str, Any],
    *,
    label: str,
    load_content: bool = False,
) -> tuple[Path, bytes | None]:
    path = Path(value["path"])
    _reject_symlink_components(path, label=label)
    if not path.is_file():
        raise ExperienceError(f"{label} must be an existing regular file")
    before = path.stat()
    digest = hashlib.sha256()
    chunks: list[bytes] | None = [] if load_content else None
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
            if chunks is not None:
                chunks.append(chunk)
    after = path.stat()
    stable_fields = ("st_dev", "st_ino", "st_size", "st_mtime_ns")
    if any(getattr(before, field) != getattr(after, field) for field in stable_fields):
        raise ExperienceError(f"{label} changed while it was read")
    actual = digest.hexdigest()
    if actual != value["sha256"]:
        raise ExperienceError(f"{label} SHA-256 mismatch")
    return path, b"".join(chunks) if chunks is not None else None


def _stable_reference(value: dict[str, Any], *, label: str) -> tuple[Path, dict[str, Any]]:
    path, encoded = _stable_file_reference(value, label=label, load_content=True)
    assert encoded is not None
    try:
        document = json.loads(encoded.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ExperienceError(f"{label} must be valid JSON: {exc}") from exc
    if not isinstance(document, dict):
        raise ExperienceError(f"{label} must be a JSON object")
    return path, document


def _require_run_evidence_path(
    root: Path,
    event: dict[str, Any],
    path: Path,
    *,
    directory: str | None = None,
) -> None:
    evidence_root = root / event["task"] / event["run_id"] / "evidence"
    if evidence_root not in path.parents:
        raise ExperienceError("event evidence is outside this run's evidence directory")
    if directory is not None and path.parent.name != directory:
        raise ExperienceError(
            f"{event['event_type']} evidence must be in evidence/{directory}"
        )


def _validate_assessment_reference(
    root: Path,
    event: dict[str, Any],
    reference: dict[str, Any],
) -> None:
    path, document = _stable_reference(reference, label="assessment evidence")
    _require_run_evidence_path(root, event, path, directory="assessment")
    if document.get("version") != 2 or document.get("advisory_only") is not True:
        raise ExperienceError("assessment evidence must be advisory version 2")
    expected_scope = document.get("criteria", {}).get("expected_scope")
    identity = event["run_identity"]
    if not isinstance(expected_scope, dict) or any(
        expected_scope.get(field) != identity[field]
        for field in ("task", "run_id", "backend", "algorithm", "runner")
    ):
        raise ExperienceError("assessment evidence scope mismatch")


def _validate_evaluation_reference(
    event: dict[str, Any],
    reference: dict[str, Any],
) -> None:
    path = Path(reference["path"])
    _, document = _stable_reference(
        reference,
        label="checkpoint evaluation evidence",
    )
    try:
        validation = validate_evaluation_bundle(path)
    except EvaluationEvidenceError as exc:
        raise ExperienceError(str(exc)) from exc
    if validation["result"]["sha256"] != reference["sha256"]:
        raise ExperienceError("checkpoint evaluation evidence SHA-256 mismatch")
    evaluation = document["evaluation"]
    identity = event["run_identity"]
    if any(
        evaluation.get(field) != identity[field]
        for field in ("task", "run_id", "runner")
    ):
        raise ExperienceError("checkpoint evaluation evidence scope mismatch")


def _validate_selection_reference(
    event: dict[str, Any],
    reference: dict[str, Any],
) -> None:
    try:
        document, _ = validate_checkpoint_selection(
            Path(reference["path"]),
            expected_sha256=reference["sha256"],
        )
    except PolicyExportEvidenceError as exc:
        raise ExperienceError(str(exc)) from exc
    identity = event["run_identity"]
    if any(
        document.get(field) != identity[field]
        for field in ("task", "run_id", "algorithm", "runner")
    ):
        raise ExperienceError("checkpoint selection evidence scope mismatch")


def _validate_export_reference(
    event: dict[str, Any],
    reference: dict[str, Any],
) -> None:
    try:
        validation = validate_export_bundle(Path(reference["path"]))
    except PolicyExportEvidenceError as exc:
        raise ExperienceError(str(exc)) from exc
    if validation["receipt"]["sha256"] != reference["sha256"]:
        raise ExperienceError("export evidence SHA-256 mismatch")
    export = validation["document"]["export"]
    identity = event["run_identity"]
    if any(
        export.get(field) != identity[field]
        for field in ("task", "run_id", "runner")
    ):
        raise ExperienceError("export evidence scope mismatch")


def _validate_archive_reference(
    event: dict[str, Any],
    reference: dict[str, Any],
) -> None:
    path, manifest = _stable_reference(reference, label="archive manifest evidence")
    identity = event["run_identity"]
    if (
        path.name != "archive_manifest.json"
        or manifest.get("version") != 2
        or manifest.get("hardware_ready") is not False
        or manifest.get("archive_path") != str(path.parent)
        or any(
            manifest.get(field) != identity[field]
            for field in ("task", "algorithm", "runner")
        )
    ):
        raise ExperienceError("archive manifest evidence scope mismatch")
    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, dict):
        raise ExperienceError("archive manifest artifact binding is missing")
    for kind, filename in (("jit", "policy.pt"), ("onnx", "policy.onnx")):
        artifact = artifacts.get(kind)
        if not isinstance(artifact, dict) or not isinstance(
            artifact.get("sha256"), str
        ):
            raise ExperienceError(f"archive {kind} artifact binding is missing")
        archived_path = path.parent / filename
        _stable_file_reference(
            {"path": str(archived_path), "sha256": artifact["sha256"]},
            label=f"archive {kind}",
        )
    export_reference = manifest.get("export_receipt")
    if not isinstance(export_reference, dict) or set(export_reference) != {
        "path",
        "sha256",
    }:
        raise ExperienceError("archive export receipt binding is missing")
    _validate_export_reference(event, export_reference)


def _validate_available_event_reference(
    root: Path,
    event: dict[str, Any],
    reference: dict[str, Any],
) -> None:
    event_type = event["event_type"]
    if event_type == "assessment":
        _validate_assessment_reference(root, event, reference)
    elif event_type == "checkpoint_evaluation":
        _validate_evaluation_reference(event, reference)
    elif event_type == "evaluation_batch":
        from summarize_evaluation_batch import validate_batch
        try:
            batch = validate_batch(Path(reference["path"]), expected_sha256=reference["sha256"])
        except (ValueError, OSError) as exc:
            raise ExperienceError(str(exc)) from exc
        if any(batch[field] != event[field] for field in ("task", "run_id")):
            raise ExperienceError("batch event scope mismatch")
        for case in batch["cases"]:
            if case["status"] == "completed":
                _validate_evaluation_reference(event, case["result"])
    elif event_type == "checkpoint_selection":
        _validate_selection_reference(event, reference)
    elif event_type == "export":
        _validate_export_reference(event, reference)
    elif event_type == "archive":
        _validate_archive_reference(event, reference)
    elif event_type == "feedback":
        path, _ = _stable_file_reference(
            reference,
            label="feedback observation evidence",
        )
        _require_run_evidence_path(root, event, path)
    else:
        raise ExperienceError(f"unsupported event evidence type: {event_type}")


def _validate_policy_binding(
    event: dict[str, Any],
    binding: dict[str, Any],
) -> None:
    reference = {"path": binding["path"], "sha256": binding["sha256"]}
    if binding["kind"] == "checkpoint_selection":
        _validate_selection_reference(event, reference)
    elif binding["kind"] == "export":
        _validate_export_reference(event, reference)
    else:
        _validate_archive_reference(event, reference)


def _validate_outcome_evidence(
    root: Path,
    event: dict[str, Any],
    current_config: dict[str, Any],
    event_reference: dict[str, Any],
    *,
    max_diff_entries: int,
) -> None:
    outcome = event["evidence"]["outcome"]
    baseline = outcome["baseline"]
    identity_path, baseline_identity = _stable_reference(
        baseline["run_identity"],
        label="outcome baseline run identity",
    )
    try:
        validate_run_identity(baseline_identity)
    except RunIdentityError as exc:
        raise ExperienceError(str(exc)) from exc
    current_identity = event["run_identity"]
    try:
        require_source_path(identity_path, root / baseline_identity["task"] / baseline_identity["run_id"], "context")
    except ValueError as exc:
        raise ExperienceError(str(exc)) from exc
    for field in ("task", "algorithm", "host_id", "backend", "runner"):
        if baseline_identity[field] != current_identity[field]:
            raise ExperienceError(f"outcome baseline {field} mismatch")
    config_reference = baseline["effective_config"]
    config_path = Path(config_reference["path"])
    try:
        require_source_path(config_path, root / baseline_identity["task"] / baseline_identity["run_id"], "config")
    except ValueError as exc:
        raise ExperienceError(str(exc)) from exc
    try:
        baseline_config, _ = load_and_validate_effective_config(
            config_path,
            expected_sha256=config_reference["sha256"],
            run_identity=baseline_identity,
        )
    except EffectiveConfigError as exc:
        raise ExperienceError(str(exc)) from exc
    if (
        baseline_config["fingerprints"]["effective_config"]
        != config_reference["effective_config_fingerprint"]
        or baseline_config["fingerprints"]["reward"]
        != config_reference["reward_fingerprint"]
    ):
        raise ExperienceError("outcome baseline config fingerprint mismatch")
    expected_diff = compare_effective_configs(
        baseline_config,
        current_config,
        max_diff_entries=max_diff_entries,
    )
    if outcome["parameter_changes"] != expected_diff:
        raise ExperienceError("outcome parameter_changes differs from effective configs")
    result_window = outcome["result_window"]
    if (
        result_window["path"] != event_reference["path"]
        or result_window["sha256"] != event_reference["sha256"]
    ):
        raise ExperienceError("outcome result_window must bind the event evidence")


def validate_event_evidence(
    root: Path,
    event: dict[str, Any],
    *,
    current_config: dict[str, Any],
    max_diff_entries: int = DEFAULT_MAX_DIFF_ENTRIES,
) -> dict[str, Any]:
    """Revalidate type-specific evidence and separate event/outcome completeness."""
    if event["version"] < 4:
        return {
            "event_evidence_complete": False,
            "outcome_evidence_complete": False,
            "reasons": ["legacy_event_contract"],
        }
    evidence = event["evidence"]
    reasons: list[str] = []
    event_complete = event["event_type"] not in EVIDENCE_EVENT_TYPES
    event_reference: dict[str, Any] | None = None
    if event["event_type"] in EVIDENCE_EVENT_TYPES:
        event_value = evidence["event"]
        if event_value["status"] == "available":
            event_reference = {
                "path": event_value["path"],
                "sha256": event_value["sha256"],
            }
            _validate_available_event_reference(root, event, event_reference)
            event_complete = True
        else:
            reasons.append("event_evidence_unavailable")
    if event["event_type"] == "feedback":
        binding = evidence["policy_binding"]
        if binding["status"] == "available":
            _validate_policy_binding(event, binding)
        else:
            event_complete = False
            reasons.append("feedback_policy_binding_unavailable")
    outcome = evidence["outcome"]
    outcome_complete = False
    if event["event_type"] == "recommendation":
        reasons.append("recommendation_is_advice_not_outcome")
    elif event["event_type"] not in OUTCOME_EVENT_TYPES:
        reasons.append("event_type_is_not_outcome_bearing")
    elif outcome["status"] == "unavailable":
        reasons.append("outcome_evidence_unavailable")
    elif not event_complete or event_reference is None:
        reasons.append("event_evidence_incomplete")
    else:
        _validate_outcome_evidence(
            root,
            event,
            current_config,
            event_reference,
            max_diff_entries=max_diff_entries,
        )
        outcome_complete = True
    return {
        "event_evidence_complete": event_complete,
        "outcome_evidence_complete": outcome_complete,
        "reasons": reasons,
    }


def write_event(root: Path, event: dict[str, Any]) -> dict[str, Any]:
    validate_event(event)
    if event.get("version") not in {4, 5}:
        raise ExperienceError("new experience events must use version 4 or 5")
    if not root.is_absolute():
        raise ExperienceError("experience root must be absolute")
    _reject_symlink_components(root, label="experience root")
    if not root.is_dir():
        raise ExperienceError("experience root must be an existing directory")
    run_dir = root / event["task"] / event["run_id"]
    _reject_symlink_components(run_dir, label="experience run directory")
    current_config = validate_effective_config_binding(root, event)
    validate_event_evidence(
        root,
        event,
        current_config=current_config,
    )
    run_dir.mkdir(parents=True, exist_ok=True)
    _reject_symlink_components(run_dir, label="experience run directory")
    if event["version"] == 5:
        run_dir = run_dir / "events"
        _reject_symlink_components(run_dir, label="events directory")
        run_dir.mkdir(exist_ok=True)
    timestamp_slug = re.sub(r"[^0-9A-Za-z]+", "-", event["recorded_at"]).strip("-")
    destination = run_dir / f"{timestamp_slug}__{event['event_id']}.json"
    if destination.exists() or destination.is_symlink():
        raise ExperienceError(f"event already exists: {destination}")
    encoded = json.dumps(
        event,
        indent=2,
        sort_keys=True,
        ensure_ascii=False,
        allow_nan=False,
    ) + "\n"
    temporary = run_dir / f".{destination.name}.tmp-{os.getpid()}-{time.time_ns()}"
    try:
        with temporary.open("x", encoding="utf-8") as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        try:
            os.link(temporary, destination, follow_symlinks=False)
        except FileExistsError as exc:
            raise ExperienceError(f"event already exists: {destination}") from exc
    finally:
        if temporary.exists():
            temporary.unlink()
    return {
        "version": 1,
        "event_path": str(destination),
        "sha256": hashlib.sha256(encoded.encode("utf-8")).hexdigest(),
        "immutable": True,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("event")
    parser.add_argument(
        "--root",
        default=str(Path(__file__).resolve().parents[4] / "learnings" / "policy_tuning"),
    )
    parser.add_argument("--output")
    args = parser.parse_args()
    try:
        receipt = write_event(Path(args.root).resolve(), _load(Path(args.event)))
    except (OSError, ExperienceError) as exc:
        parser.error(str(exc))
    encoded = json.dumps(receipt, indent=2, sort_keys=True, ensure_ascii=False) + "\n"
    if args.output:
        try:
            _write_new_output(Path(args.output), encoded)
        except ExperienceError as exc:
            parser.error(str(exc))
    else:
        print(encoded, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
