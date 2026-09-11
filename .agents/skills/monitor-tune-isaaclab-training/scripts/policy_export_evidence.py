#!/usr/bin/env python3
"""Record checkpoint selection and validate transactional policy exports."""

from __future__ import annotations

from evidence_provenance import require_source_path

import argparse
import hashlib
import json
import math
import os
import re
import shutil
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = Path(__file__).resolve().parents[4]
RSL_RL_DIR = REPO_ROOT / "scripts" / "reinforcement_learning" / "rsl_rl"
sys.path.insert(0, str(SCRIPT_DIR))
sys.path.insert(0, str(RSL_RL_DIR))

from capture_effective_training_config import (  # noqa: E402
    EffectiveConfigError,
    load_and_validate_effective_config,
)
from capture_run_identity import RunIdentityError, validate_run_identity  # noqa: E402
from onnx_export_contract import (  # noqa: E402
    OnnxExportContractError,
    get_onnx_export_contract,
    validate_onnx_export_contract,
)
from policy_evaluation_evidence import (  # noqa: E402
    EvaluationEvidenceError,
    validate_evaluation_bundle,
)


SAFE_IDENTIFIER_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
CHECKPOINT_RE = re.compile(r"^model_(?P<iteration>\d+)\.pt$")


class PolicyExportEvidenceError(ValueError):
    """Raised when selection or export evidence is unsafe or inconsistent."""


def canonical_json(value: Any) -> str:
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    )


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _identifier(name: str, value: Any) -> str:
    if not isinstance(value, str) or not SAFE_IDENTIFIER_RE.fullmatch(value):
        raise PolicyExportEvidenceError(f"{name} must be a safe ASCII identifier")
    return value


def _sha256(name: str, value: Any) -> str:
    if not isinstance(value, str) or not SHA256_RE.fullmatch(value):
        raise PolicyExportEvidenceError(f"{name} must be a lowercase SHA-256")
    return value


def _reject_symlink_components(path: Path, *, label: str) -> None:
    if not path.is_absolute() or ".." in path.parts:
        raise PolicyExportEvidenceError(
            f"{label} must be absolute and contain no traversal"
        )
    current = Path(path.anchor)
    for component in path.parts[1:]:
        current /= component
        if current.is_symlink():
            raise PolicyExportEvidenceError(
                f"{label} contains a symlinked component: {current}"
            )


def _regular_file(
    path: Path,
    *,
    label: str,
    expected_sha256: str | None = None,
) -> dict[str, Any]:
    _reject_symlink_components(path, label=label)
    if not path.is_file():
        raise PolicyExportEvidenceError(f"{label} must be an existing regular file")
    stat_before = path.stat()
    actual_sha256 = sha256_file(path)
    stat_after = path.stat()
    if (
        stat_before.st_dev,
        stat_before.st_ino,
        stat_before.st_size,
        stat_before.st_mtime_ns,
    ) != (
        stat_after.st_dev,
        stat_after.st_ino,
        stat_after.st_size,
        stat_after.st_mtime_ns,
    ):
        raise PolicyExportEvidenceError(f"{label} changed while it was read")
    if expected_sha256 is not None:
        _sha256(f"{label} SHA-256", expected_sha256)
        if actual_sha256 != expected_sha256:
            raise PolicyExportEvidenceError(f"{label} SHA-256 mismatch")
    return {
        "path": str(path),
        "sha256": actual_sha256,
        "size_bytes": stat_after.st_size,
    }


def _load_json(path: Path, *, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise PolicyExportEvidenceError(f"cannot read {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise PolicyExportEvidenceError(f"{label} must be a JSON object")
    return value


def _write_new_json(path: Path, value: dict[str, Any]) -> dict[str, Any]:
    _reject_symlink_components(path, label="output")
    if not path.parent.is_dir() or path.exists() or path.is_symlink():
        raise PolicyExportEvidenceError("output must be a new file in an existing directory")
    encoded = json.dumps(
        value,
        indent=2,
        sort_keys=True,
        ensure_ascii=False,
        allow_nan=False,
    ) + "\n"
    temporary = path.parent / f".{path.name}.tmp-{os.getpid()}-{os.urandom(8).hex()}"
    try:
        with temporary.open("x", encoding="utf-8") as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        try:
            os.link(temporary, path, follow_symlinks=False)
        except FileExistsError as exc:
            raise PolicyExportEvidenceError("output target already exists") from exc
    finally:
        temporary.unlink(missing_ok=True)
    return {"path": str(path), "sha256": sha256_bytes(encoded.encode("utf-8"))}


def _approved_at(value: Any) -> str:
    if not isinstance(value, str):
        raise PolicyExportEvidenceError("approved_at must be a timezone-aware timestamp")
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError as exc:
        raise PolicyExportEvidenceError(
            "approved_at must be a timezone-aware ISO-8601 timestamp"
        ) from exc
    if parsed.tzinfo is None or parsed.utcoffset() is None:
        raise PolicyExportEvidenceError(
            "approved_at must be a timezone-aware ISO-8601 timestamp"
        )
    return value


def expected_selection_path(
    repo_root: Path,
    *,
    task: str,
    run_id: str,
    selection_id: str,
) -> Path:
    _reject_symlink_components(repo_root, label="repository root")
    for name, value in (
        ("task", task),
        ("run_id", run_id),
        ("selection_id", selection_id),
    ):
        _identifier(name, value)
    return (
        repo_root
        / "learnings"
        / "policy_tuning"
        / task
        / run_id
        / "evidence"
        / "checkpoint_selection"
        / f"selection-{selection_id}.json"
    )


def expected_export_paths(
    repo_root: Path,
    *,
    task: str,
    run_id: str,
    export_id: str,
) -> dict[str, Path]:
    _reject_symlink_components(repo_root, label="repository root")
    for name, value in (
        ("task", task),
        ("run_id", run_id),
        ("export_id", export_id),
    ):
        _identifier(name, value)
    export_dir = (
        repo_root
        / "learnings"
        / "policy_tuning"
        / task
        / run_id
        / "evidence"
        / "export"
        / export_id
    )
    return {
        "export_dir": export_dir,
        "jit": export_dir / "policy.pt",
        "onnx": export_dir / "policy.onnx",
        "receipt": export_dir / "receipt.json",
    }


def expected_tensor_contract(runner: str) -> dict[str, Any]:
    _identifier("runner", runner)
    if "ROA" in runner:
        return {
            "history_contract": "flat_time_major_history",
            "normalization_contract": "current_frame_only",
            "reset_contract": "environment_history_reset",
            "actor_input_order": ["current_obs", "code_vel", "hist_latent"],
        }
    if "Dwaq" in runner:
        return {
            "history_contract": "flat_time_major_history",
            "normalization_contract": "combined_actor_input",
            "reset_contract": "environment_history_reset",
            "actor_input_order": ["encoded_velocity", "encoded_latent", "current_obs"],
        }
    return {
        "history_contract": "current_observation",
        "normalization_contract": "backend_export_helper",
        "reset_contract": "stateless_environment_reset",
        "actor_input_order": ["current_obs"],
    }


def _load_run_identity_reference(
    path: Path,
    *,
    expected_sha256: str,
) -> tuple[dict[str, Any], dict[str, Any]]:
    reference = _regular_file(
        path,
        label="run identity",
        expected_sha256=expected_sha256,
    )
    identity = _load_json(path, label="run identity")
    try:
        validate_run_identity(identity)
    except RunIdentityError as exc:
        raise PolicyExportEvidenceError(str(exc)) from exc
    repo_root = Path(identity["source"]["repository_root"])
    try:
        require_source_path(path, repo_root / "learnings" / "policy_tuning" / identity["task"] / identity["run_id"], "context")
    except ValueError as exc:
        raise PolicyExportEvidenceError(str(exc)) from exc
    reference["identity_sha256"] = identity["identity_sha256"]
    return identity, reference


def _load_effective_config_reference(
    path: Path,
    *,
    expected_sha256: str,
    run_identity: dict[str, Any],
) -> tuple[dict[str, Any], dict[str, Any]]:
    repo_root = Path(run_identity["source"]["repository_root"])
    try:
        require_source_path(path, repo_root / "learnings" / "policy_tuning" / run_identity["task"] / run_identity["run_id"], "config")
    except ValueError as exc:
        raise PolicyExportEvidenceError(str(exc)) from exc
    try:
        document, metadata = load_and_validate_effective_config(
            path,
            expected_sha256=expected_sha256,
            run_identity=run_identity,
        )
    except EffectiveConfigError as exc:
        raise PolicyExportEvidenceError(str(exc)) from exc
    return document, {
        "path": str(path),
        "sha256": metadata["sha256"],
        "effective_config_fingerprint": document["fingerprints"][
            "effective_config"
        ],
        "reward_fingerprint": document["fingerprints"]["reward"],
    }


def _checkpoint_reference(
    path: Path,
    *,
    expected_sha256: str,
    checkpoint_id: str,
) -> dict[str, Any]:
    reference = _regular_file(
        path,
        label="checkpoint",
        expected_sha256=expected_sha256,
    )
    match = CHECKPOINT_RE.fullmatch(path.name)
    if match is None:
        raise PolicyExportEvidenceError("checkpoint filename must be model_<iteration>.pt")
    if checkpoint_id != path.stem:
        raise PolicyExportEvidenceError("checkpoint_id must equal the checkpoint filename stem")
    reference.update(
        {
            "checkpoint_id": checkpoint_id,
            "iteration": int(match.group("iteration")),
        }
    )
    return reference


def _validate_selection_report(
    path: Path,
    *,
    expected_sha256: str,
    checkpoint: dict[str, Any],
) -> dict[str, Any]:
    reference = _regular_file(
        path,
        label="checkpoint selection report",
        expected_sha256=expected_sha256,
    )
    report = _load_json(path, label="checkpoint selection report")
    if report.get("version") != 1 or report.get("advisory_only") is not True:
        raise PolicyExportEvidenceError("selection report must be advisory version 1")
    inventory = report.get("inventory")
    if not isinstance(inventory, list):
        raise PolicyExportEvidenceError("selection report inventory is missing")
    matches = [
        item
        for item in inventory
        if isinstance(item, dict)
        and item.get("path") == checkpoint["path"]
        and item.get("sha256") == checkpoint["sha256"]
        and item.get("step") == checkpoint["iteration"]
        and item.get("stable") is True
    ]
    if len(matches) != 1:
        raise PolicyExportEvidenceError(
            "selected checkpoint is not one stable exact inventory entry"
        )
    reference["report_version"] = 1
    return reference


def _evaluation_references(
    paths: list[Path],
    *,
    task: str,
    run_id: str,
    runner: str,
    checkpoint: dict[str, Any],
) -> list[dict[str, Any]]:
    if not paths:
        raise PolicyExportEvidenceError(
            "at least one closed-loop evaluation result is required"
        )
    references: list[dict[str, Any]] = []
    seen: set[Path] = set()
    for path in paths:
        if path in seen:
            raise PolicyExportEvidenceError("duplicate evaluation result path")
        seen.add(path)
        try:
            validate_evaluation_bundle(path)
        except EvaluationEvidenceError as exc:
            raise PolicyExportEvidenceError(str(exc)) from exc
        result = _load_json(path, label="evaluation result")
        evaluation = result.get("evaluation", {})
        inputs = result.get("inputs", {})
        if (
            evaluation.get("task") != task
            or evaluation.get("run_id") != run_id
            or evaluation.get("runner") != runner
            or inputs.get("checkpoint") != {
                "path": checkpoint["path"],
                "sha256": checkpoint["sha256"],
            }
        ):
            raise PolicyExportEvidenceError("evaluation result scope/checkpoint mismatch")
        if (
            runner == "OnPolicyRunnerAmpROA"
            and result.get("telemetry_status") != "complete"
        ):
            raise PolicyExportEvidenceError(
                "AMP-ROA selection requires complete evaluation telemetry"
            )
        references.append(
            {
                "path": str(path),
                "sha256": sha256_file(path),
                "evaluation_id": evaluation.get("evaluation_id"),
                "scenario_sha256": inputs.get("scenario", {}).get("sha256"),
            }
        )
    return references


def record_checkpoint_selection(
    *,
    selection_id: str,
    approved_at: str,
    checkpoint_id: str,
    checkpoint_path: Path,
    checkpoint_sha256: str,
    selection_report_path: Path,
    selection_report_sha256: str,
    run_identity_path: Path,
    run_identity_file_sha256: str,
    effective_config_path: Path,
    effective_config_sha256: str,
    evaluation_result_paths: list[Path],
    tensor_contract: dict[str, Any],
    output_path: Path,
) -> dict[str, Any]:
    """Write one immutable receipt after an explicit user checkpoint choice."""
    selection_id = _identifier("selection_id", selection_id)
    checkpoint_id = _identifier("checkpoint_id", checkpoint_id)
    identity, identity_reference = _load_run_identity_reference(
        run_identity_path,
        expected_sha256=run_identity_file_sha256,
    )
    repo_root = Path(identity["source"]["repository_root"])
    expected_output = expected_selection_path(
        repo_root,
        task=identity["task"],
        run_id=identity["run_id"],
        selection_id=selection_id,
    )
    _reject_symlink_components(output_path, label="selection output")
    if output_path != expected_output:
        raise PolicyExportEvidenceError("selection output is outside the evidence layout")
    checkpoint = _checkpoint_reference(
        checkpoint_path,
        expected_sha256=checkpoint_sha256,
        checkpoint_id=checkpoint_id,
    )
    selection_report = _validate_selection_report(
        selection_report_path,
        expected_sha256=selection_report_sha256,
        checkpoint=checkpoint,
    )
    effective_config, effective_reference = _load_effective_config_reference(
        effective_config_path,
        expected_sha256=effective_config_sha256,
        run_identity=identity,
    )
    expected_contract = expected_tensor_contract(identity["runner"])
    if tensor_contract != expected_contract:
        raise PolicyExportEvidenceError("approved tensor contract does not match runner")
    evaluation_references = _evaluation_references(
        evaluation_result_paths,
        task=identity["task"],
        run_id=identity["run_id"],
        runner=identity["runner"],
        checkpoint=checkpoint,
    )
    receipt = {
        "version": 1,
        "status": "approved",
        "selected_by_user": True,
        "selection_id": selection_id,
        "approved_at": _approved_at(approved_at),
        "task": identity["task"],
        "run_id": identity["run_id"],
        "runner": identity["runner"],
        "algorithm": identity["algorithm"],
        "checkpoint": checkpoint,
        "selection_report": selection_report,
        "run_identity": {
            "document": identity,
            **identity_reference,
        },
        "effective_config": effective_reference,
        "effective_config_summary": {
            "resolved_identity": effective_config["resolved_identity"],
            "fingerprints": effective_config["fingerprints"],
        },
        "evaluation_results": evaluation_references,
        "tensor_contract": tensor_contract,
    }
    published = _write_new_json(output_path, receipt)
    return {
        "version": 1,
        "selection_id": selection_id,
        "selection_receipt": published,
        "checkpoint": checkpoint,
    }


def validate_checkpoint_selection(
    path: Path,
    *,
    expected_sha256: str,
) -> tuple[dict[str, Any], dict[str, Any]]:
    reference = _regular_file(
        path,
        label="checkpoint selection receipt",
        expected_sha256=expected_sha256,
    )
    receipt = _load_json(path, label="checkpoint selection receipt")
    if (
        receipt.get("version") != 1
        or receipt.get("status") != "approved"
        or receipt.get("selected_by_user") is not True
    ):
        raise PolicyExportEvidenceError("approved version-1 selection receipt required")
    selection_id = _identifier("selection_id", receipt.get("selection_id"))
    _approved_at(receipt.get("approved_at"))
    identity_binding = receipt.get("run_identity")
    if not isinstance(identity_binding, dict):
        raise PolicyExportEvidenceError("selection run identity binding is missing")
    identity_path = identity_binding.get("path")
    if not isinstance(identity_path, str):
        raise PolicyExportEvidenceError("selection run identity path is invalid")
    identity, identity_reference = _load_run_identity_reference(
        Path(identity_path),
        expected_sha256=identity_binding.get("sha256"),
    )
    if identity_binding.get("document") != identity:
        raise PolicyExportEvidenceError("embedded run identity differs from source")
    if identity_binding.get("identity_sha256") != identity["identity_sha256"]:
        raise PolicyExportEvidenceError("selection run identity SHA-256 mismatch")
    for field in ("task", "run_id", "runner", "algorithm"):
        if receipt.get(field) != identity.get(field):
            raise PolicyExportEvidenceError(f"selection {field} mismatch")
    repo_root = Path(identity["source"]["repository_root"])
    if path != expected_selection_path(
        repo_root,
        task=identity["task"],
        run_id=identity["run_id"],
        selection_id=selection_id,
    ):
        raise PolicyExportEvidenceError("selection receipt is outside the evidence layout")
    checkpoint_value = receipt.get("checkpoint")
    if not isinstance(checkpoint_value, dict):
        raise PolicyExportEvidenceError("selection checkpoint binding is missing")
    checkpoint = _checkpoint_reference(
        Path(checkpoint_value.get("path", "")),
        expected_sha256=checkpoint_value.get("sha256"),
        checkpoint_id=checkpoint_value.get("checkpoint_id"),
    )
    if checkpoint_value != checkpoint:
        raise PolicyExportEvidenceError("selection checkpoint metadata mismatch")
    report_value = receipt.get("selection_report")
    if not isinstance(report_value, dict):
        raise PolicyExportEvidenceError("selection report binding is missing")
    report_reference = _validate_selection_report(
        Path(report_value.get("path", "")),
        expected_sha256=report_value.get("sha256"),
        checkpoint=checkpoint,
    )
    if report_value != report_reference:
        raise PolicyExportEvidenceError("selection report metadata mismatch")
    config_value = receipt.get("effective_config")
    if not isinstance(config_value, dict):
        raise PolicyExportEvidenceError("selection effective config binding is missing")
    effective_config, config_reference = _load_effective_config_reference(
        Path(config_value.get("path", "")),
        expected_sha256=config_value.get("sha256"),
        run_identity=identity,
    )
    if config_value != config_reference:
        raise PolicyExportEvidenceError("selection effective config metadata mismatch")
    if receipt.get("effective_config_summary") != {
        "resolved_identity": effective_config["resolved_identity"],
        "fingerprints": effective_config["fingerprints"],
    }:
        raise PolicyExportEvidenceError("selection effective config summary mismatch")
    if receipt.get("tensor_contract") != expected_tensor_contract(identity["runner"]):
        raise PolicyExportEvidenceError("selection tensor contract mismatch")
    evaluation_values = receipt.get("evaluation_results")
    if not isinstance(evaluation_values, list):
        raise PolicyExportEvidenceError("selection evaluation references are missing")
    evaluations = _evaluation_references(
        [Path(item.get("path", "")) for item in evaluation_values if isinstance(item, dict)],
        task=identity["task"],
        run_id=identity["run_id"],
        runner=identity["runner"],
        checkpoint=checkpoint,
    )
    if evaluation_values != evaluations:
        raise PolicyExportEvidenceError("selection evaluation metadata mismatch")
    reference["selection_id"] = selection_id
    reference["identity"] = identity_reference
    return receipt, reference


@dataclass(frozen=True)
class ExportPlan:
    repo_root: Path
    task: str
    run_id: str
    export_id: str
    paths: dict[str, Path]
    checkpoint: dict[str, Any]
    selection_receipt: dict[str, Any]
    selection_reference: dict[str, Any]
    tensor_contract: dict[str, Any]
    onnx_export_contract: dict[str, Any]
    parity_contract: dict[str, Any]


def _require_new_export_targets(paths: dict[str, Path]) -> None:
    export_dir = paths["export_dir"]
    _reject_symlink_components(export_dir, label="export directory")
    if not export_dir.is_dir():
        raise PolicyExportEvidenceError(
            "export directory must be prepared by prepare_evidence_layout.py"
        )
    for label in ("jit", "onnx", "receipt"):
        path = paths[label]
        _reject_symlink_components(path, label=f"export {label}")
        if path.exists() or path.is_symlink():
            raise PolicyExportEvidenceError(f"export {label} target already exists")


def preflight_export(
    *,
    task: str,
    run_id: str,
    checkpoint_id: str,
    checkpoint_path: Path,
    checkpoint_sha256: str,
    export_id: str,
    selection_receipt_path: Path,
    selection_receipt_sha256: str,
    jit_path: Path,
    onnx_path: Path,
    receipt_path: Path,
    history_contract: str,
    normalization_contract: str,
    reset_contract: str,
    onnx_export_profile: str,
    parity_steps: int,
    reset_step: int,
    minimum_parity_samples: int,
    max_abs_action_error: float,
    num_envs: int,
    seed: int,
) -> ExportPlan:
    selection, selection_reference = validate_checkpoint_selection(
        selection_receipt_path,
        expected_sha256=selection_receipt_sha256,
    )
    if task != selection["task"] or run_id != selection["run_id"]:
        raise PolicyExportEvidenceError("export task/run does not match selection")
    checkpoint = _checkpoint_reference(
        checkpoint_path,
        expected_sha256=checkpoint_sha256,
        checkpoint_id=checkpoint_id,
    )
    if checkpoint != selection["checkpoint"]:
        raise PolicyExportEvidenceError("export checkpoint does not match selection")
    tensor_contract = dict(selection["tensor_contract"])
    supplied_contract = {
        "history_contract": history_contract,
        "normalization_contract": normalization_contract,
        "reset_contract": reset_contract,
        "actor_input_order": tensor_contract["actor_input_order"],
    }
    if supplied_contract != tensor_contract:
        raise PolicyExportEvidenceError("export tensor contract differs from selection")
    try:
        onnx_export_contract = get_onnx_export_contract(onnx_export_profile)
    except OnnxExportContractError as exc:
        raise PolicyExportEvidenceError(str(exc)) from exc
    integer_fields = {
        "parity_steps": parity_steps,
        "reset_step": reset_step,
        "minimum_parity_samples": minimum_parity_samples,
        "num_envs": num_envs,
    }
    if any(isinstance(value, bool) or not isinstance(value, int) for value in integer_fields.values()):
        raise PolicyExportEvidenceError("parity integer parameters are invalid")
    if not 2 <= parity_steps <= 64:
        raise PolicyExportEvidenceError("parity_steps must be between 2 and 64")
    if not 1 <= reset_step < parity_steps:
        raise PolicyExportEvidenceError("reset_step must be inside the parity window")
    if not 1 <= num_envs <= 64 or minimum_parity_samples <= 0:
        raise PolicyExportEvidenceError(
            "num_envs must be 1..64 and minimum_parity_samples must be positive"
        )
    if minimum_parity_samples > num_envs * parity_steps:
        raise PolicyExportEvidenceError("parity window cannot cover minimum samples")
    if (
        isinstance(max_abs_action_error, bool)
        or not isinstance(max_abs_action_error, (int, float))
        or not math.isfinite(float(max_abs_action_error))
        or max_abs_action_error < 0
    ):
        raise PolicyExportEvidenceError("max_abs_action_error must be finite and non-negative")
    if seed != selection["run_identity"]["document"]["seed"]:
        raise PolicyExportEvidenceError("export seed does not match run identity")
    repo_root = Path(
        selection["run_identity"]["document"]["source"]["repository_root"]
    )
    paths = expected_export_paths(
        repo_root,
        task=task,
        run_id=run_id,
        export_id=export_id,
    )
    for label, actual in (("jit", jit_path), ("onnx", onnx_path), ("receipt", receipt_path)):
        _reject_symlink_components(actual, label=f"export {label}")
        if actual != paths[label]:
            raise PolicyExportEvidenceError(f"export {label} is outside the evidence layout")
    _require_new_export_targets(paths)
    parity_contract = {
        "parity_steps": parity_steps,
        "reset_step": reset_step,
        "minimum_parity_samples": minimum_parity_samples,
        "max_abs_action_error": float(max_abs_action_error),
        "num_envs": num_envs,
        "seed": seed,
        "required_boundaries": ["initial", "pre_reset", "post_reset", "final"],
    }
    return ExportPlan(
        repo_root=repo_root,
        task=task,
        run_id=run_id,
        export_id=_identifier("export_id", export_id),
        paths=paths,
        checkpoint=checkpoint,
        selection_receipt=selection,
        selection_reference=selection_reference,
        tensor_contract=tensor_contract,
        onnx_export_contract=onnx_export_contract,
        parity_contract=parity_contract,
    )


class ExportPublisher:
    """Publish JIT, ONNX, and a validated receipt without overwriting."""

    def __init__(self, plan: ExportPlan) -> None:
        self.plan = plan
        self.claim_path = plan.paths["export_dir"] / ".publish-claim"
        self.attempt_dir = plan.paths["export_dir"] / ".attempt"
        self.jit_work_path = self.attempt_dir / "policy.pt"
        self.onnx_work_path = self.attempt_dir / "policy.onnx"
        self.receipt_work_path = self.attempt_dir / "receipt.json"
        self._entered = False
        self._published = False
        self._claim_identity: tuple[int, int] | None = None
        self._attempt_identity: tuple[int, int] | None = None

    @staticmethod
    def _identity(path: Path) -> tuple[int, int]:
        stat_result = path.stat(follow_symlinks=False)
        return stat_result.st_dev, stat_result.st_ino

    @staticmethod
    def _still_owned(path: Path, identity: tuple[int, int] | None) -> bool:
        if identity is None:
            return False
        try:
            return ExportPublisher._identity(path) == identity
        except FileNotFoundError:
            return False

    def __enter__(self) -> "ExportPublisher":
        _require_new_export_targets(self.plan.paths)
        try:
            with self.claim_path.open("x", encoding="utf-8") as stream:
                stream.write(self.plan.export_id + "\n")
        except FileExistsError as exc:
            raise PolicyExportEvidenceError("export publication is already claimed") from exc
        self._claim_identity = self._identity(self.claim_path)
        try:
            self.attempt_dir.mkdir()
        except OSError as exc:
            if self._still_owned(self.claim_path, self._claim_identity):
                self.claim_path.unlink()
            self._claim_identity = None
            raise PolicyExportEvidenceError(f"cannot create export attempt: {exc}") from exc
        self._attempt_identity = self._identity(self.attempt_dir)
        self._entered = True
        return self

    def close(self) -> None:
        if not self._entered:
            return
        if self._still_owned(self.attempt_dir, self._attempt_identity):
            shutil.rmtree(self.attempt_dir)
        if self._still_owned(self.claim_path, self._claim_identity):
            self.claim_path.unlink()
        self._entered = False
        self._claim_identity = None
        self._attempt_identity = None

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        self.close()

    def publish(self, receipt: dict[str, Any]) -> dict[str, Any]:
        if not self._entered or self._published:
            raise PolicyExportEvidenceError("export publisher is not active")
        _require_new_export_targets(self.plan.paths)
        jit = _regular_file(self.jit_work_path, label="attempt JIT")
        onnx = _regular_file(self.onnx_work_path, label="attempt ONNX")
        if jit["size_bytes"] <= 0 or onnx["size_bytes"] <= 0:
            raise PolicyExportEvidenceError("export artifacts must be non-empty")
        published = dict(receipt)
        published["receipt_path"] = str(self.plan.paths["receipt"])
        published["outputs"] = {
            "jit": {**jit, "path": str(self.plan.paths["jit"])},
            "onnx": {**onnx, "path": str(self.plan.paths["onnx"])},
        }
        with self.receipt_work_path.open("x", encoding="utf-8") as stream:
            json.dump(
                published,
                stream,
                indent=2,
                sort_keys=True,
                ensure_ascii=False,
                allow_nan=False,
            )
            stream.write("\n")
        created: list[tuple[Path, tuple[int, int]]] = []
        try:
            for source, target in (
                (self.jit_work_path, self.plan.paths["jit"]),
                (self.onnx_work_path, self.plan.paths["onnx"]),
            ):
                os.link(source, target, follow_symlinks=False)
                created.append((target, self._identity(target)))
            validate_export_bundle(
                self.receipt_work_path,
                canonical_receipt_path=self.plan.paths["receipt"],
            )
            os.link(
                self.receipt_work_path,
                self.plan.paths["receipt"],
                follow_symlinks=False,
            )
            created.append(
                (
                    self.plan.paths["receipt"],
                    self._identity(self.plan.paths["receipt"]),
                )
            )
            validate_export_bundle(self.plan.paths["receipt"])
        except (OSError, PolicyExportEvidenceError) as exc:
            for target, identity in reversed(created):
                if self._still_owned(target, identity):
                    target.unlink()
            raise PolicyExportEvidenceError(
                f"exclusive export publication failed: {exc}"
            ) from exc
        self._published = True
        return published


def close_export_resources(
    publisher: ExportPublisher,
    simulation_app: Any,
) -> None:
    """Release export publication state before shutting down Isaac Sim."""
    try:
        publisher.close()
    finally:
        simulation_app.close()


def _validate_parity_contract(value: Any) -> dict[str, Any]:
    expected_keys = {
        "parity_steps",
        "reset_step",
        "minimum_parity_samples",
        "max_abs_action_error",
        "num_envs",
        "seed",
        "required_boundaries",
    }
    if not isinstance(value, dict) or set(value) != expected_keys:
        raise PolicyExportEvidenceError("export parity contract is invalid")
    parity_steps = value["parity_steps"]
    reset_step = value["reset_step"]
    minimum = value["minimum_parity_samples"]
    num_envs = value["num_envs"]
    if any(
        isinstance(item, bool) or not isinstance(item, int)
        for item in (parity_steps, reset_step, minimum, num_envs, value["seed"])
    ):
        raise PolicyExportEvidenceError("export parity contract integers are invalid")
    if not 2 <= parity_steps <= 64 or not 1 <= reset_step < parity_steps:
        raise PolicyExportEvidenceError("export parity step contract is invalid")
    if not 1 <= num_envs <= 64 or not 1 <= minimum <= num_envs * parity_steps:
        raise PolicyExportEvidenceError("export parity sample contract is invalid")
    limit = value["max_abs_action_error"]
    if (
        isinstance(limit, bool)
        or not isinstance(limit, (int, float))
        or not math.isfinite(float(limit))
        or limit < 0
    ):
        raise PolicyExportEvidenceError("export parity error limit is invalid")
    if value["required_boundaries"] != [
        "initial",
        "pre_reset",
        "post_reset",
        "final",
    ]:
        raise PolicyExportEvidenceError("export parity boundary contract is invalid")
    return value


def _validate_shape(value: Any, *, label: str) -> list[int]:
    if (
        not isinstance(value, list)
        or not value
        or any(
            isinstance(item, bool) or not isinstance(item, int) or item <= 0
            for item in value
        )
    ):
        raise PolicyExportEvidenceError(f"{label} shape is invalid")
    return value


def _validate_receipt_onnx_contract(value: Any) -> dict[str, Any]:
    try:
        return validate_onnx_export_contract(value)
    except OnnxExportContractError as exc:
        raise PolicyExportEvidenceError(str(exc)) from exc


def _validate_onnx_model_shape(
    value: Any,
    *,
    label: str,
    batch_contract: str,
) -> list[int | str]:
    expected_batch: int | str = 1 if batch_contract == "static_batch_1" else "batch"
    if (
        not isinstance(value, list)
        or len(value) != 2
        or value[0] != expected_batch
        or isinstance(value[1], bool)
        or not isinstance(value[1], int)
        or value[1] <= 0
    ):
        raise PolicyExportEvidenceError(f"{label} shape is invalid")
    return value


def _validate_onnx_export_evidence(
    value: Any,
    *,
    contract: dict[str, Any],
) -> None:
    expected_keys = {
        "contract",
        "input_shape",
        "output_shape",
        "pre_simplify_node_count",
        "post_simplify_node_count",
        "simplifier_check",
    }
    if not isinstance(value, dict) or set(value) != expected_keys:
        raise PolicyExportEvidenceError("ONNX export evidence is invalid")
    if _validate_receipt_onnx_contract(value["contract"]) != contract:
        raise PolicyExportEvidenceError("ONNX export evidence contract mismatch")
    _validate_onnx_model_shape(
        value["input_shape"],
        label="ONNX model input",
        batch_contract=contract["batch_contract"],
    )
    _validate_onnx_model_shape(
        value["output_shape"],
        label="ONNX model output",
        batch_contract=contract["batch_contract"],
    )
    pre_count = value["pre_simplify_node_count"]
    post_count = value["post_simplify_node_count"]
    if any(
        isinstance(item, bool) or not isinstance(item, int) or item < 0
        for item in (pre_count, post_count)
    ) or post_count > pre_count:
        raise PolicyExportEvidenceError("ONNX node-count evidence is invalid")
    if contract["simplified"]:
        if value["simplifier_check"] is not True:
            raise PolicyExportEvidenceError("ONNX simplifier validation is incomplete")
    elif value["simplifier_check"] is not None or post_count != pre_count:
        raise PolicyExportEvidenceError("unsimplified ONNX evidence is inconsistent")


def _validate_parity(
    parity: Any,
    *,
    contract: dict[str, Any],
) -> None:
    if not isinstance(parity, dict):
        raise PolicyExportEvidenceError("export parity evidence is missing")
    sample_count = parity.get("sample_count")
    if (
        isinstance(sample_count, bool)
        or not isinstance(sample_count, int)
        or sample_count != contract["num_envs"] * contract["parity_steps"]
        or sample_count < contract["minimum_parity_samples"]
    ):
        raise PolicyExportEvidenceError("export parity sample count is insufficient")
    boundaries = parity.get("boundaries")
    if not isinstance(boundaries, list) or len(boundaries) != 4:
        raise PolicyExportEvidenceError("export parity boundaries are missing")
    expected_steps = {
        "initial": 0,
        "pre_reset": contract["reset_step"] - 1,
        "post_reset": contract["reset_step"],
        "final": contract["parity_steps"] - 1,
    }
    observed_steps: dict[str, int] = {}
    for item in boundaries:
        if not isinstance(item, dict):
            raise PolicyExportEvidenceError("export parity boundary is invalid")
        label = item.get("label")
        step = item.get("step")
        if label not in expected_steps or step != expected_steps[label]:
            raise PolicyExportEvidenceError("export parity boundary step is invalid")
        if label in observed_steps:
            raise PolicyExportEvidenceError("export parity boundary is duplicated")
        _sha256("boundary observation digest", item.get("observation_sha256"))
        _sha256("boundary native output digest", item.get("native_output_sha256"))
        _validate_shape(item.get("input_shape"), label="boundary input")
        _validate_shape(item.get("output_shape"), label="boundary output")
        observed_steps[label] = step
    if set(observed_steps) != set(expected_steps):
        raise PolicyExportEvidenceError("export parity reset boundaries are incomplete")
    _sha256("parity observation digest", parity.get("observation_batch_sha256"))
    _sha256("parity native output digest", parity.get("native_output_sha256"))
    native_error = parity.get("native_device_to_cpu_max_abs_action_error")
    if (
        isinstance(native_error, bool)
        or not isinstance(native_error, (int, float))
        or not math.isfinite(float(native_error))
    ):
        raise PolicyExportEvidenceError("Native device/CPU parity error is invalid")
    shapes: dict[str, tuple[list[int], list[int]]] = {}
    for kind in ("jit", "onnx"):
        evidence = parity.get(kind)
        if not isinstance(evidence, dict) or evidence.get("finite") is not True:
            raise PolicyExportEvidenceError(f"{kind} parity is not finite")
        error = evidence.get("max_abs_action_error")
        if (
            isinstance(error, bool)
            or not isinstance(error, (int, float))
            or not math.isfinite(float(error))
            or error > contract["max_abs_action_error"]
        ):
            raise PolicyExportEvidenceError(f"{kind} parity exceeds the approved limit")
        shapes[kind] = (
            _validate_shape(evidence.get("input_shape"), label=f"{kind} input"),
            _validate_shape(evidence.get("output_shape"), label=f"{kind} output"),
        )
    if shapes["jit"] != shapes["onnx"]:
        raise PolicyExportEvidenceError("JIT and ONNX parity shapes differ")


def validate_export_bundle(
    receipt_path: Path,
    *,
    canonical_receipt_path: Path | None = None,
) -> dict[str, Any]:
    receipt_reference = _regular_file(receipt_path, label="export receipt")
    receipt = _load_json(receipt_path, label="export receipt")
    version = receipt.get("version")
    if (
        isinstance(version, bool)
        or version not in (3, 4)
        or receipt.get("status") != "completed"
    ):
        raise PolicyExportEvidenceError("completed version-3 or version-4 export receipt required")
    export = receipt.get("export")
    inputs = receipt.get("inputs")
    outputs = receipt.get("outputs")
    if not isinstance(export, dict) or not isinstance(inputs, dict):
        raise PolicyExportEvidenceError("export receipt binding is incomplete")
    if not isinstance(outputs, dict) or set(outputs) != {"jit", "onnx"}:
        raise PolicyExportEvidenceError("export outputs binding is invalid")
    task = _identifier("export task", export.get("task"))
    run_id = _identifier("export run_id", export.get("run_id"))
    export_id = _identifier("export id", export.get("export_id"))
    selection_value = inputs.get("checkpoint_selection")
    if not isinstance(selection_value, dict):
        raise PolicyExportEvidenceError("export selection binding is missing")
    selection, selection_reference = validate_checkpoint_selection(
        Path(selection_value.get("path", "")),
        expected_sha256=selection_value.get("sha256"),
    )
    if selection_value.get("selection_id") != selection["selection_id"]:
        raise PolicyExportEvidenceError("export selection ID mismatch")
    if task != selection["task"] or run_id != selection["run_id"]:
        raise PolicyExportEvidenceError("export scope differs from selection")
    if export.get("runner") != selection["runner"]:
        raise PolicyExportEvidenceError("export runner differs from selection")
    if export.get("checkpoint_id") != selection["checkpoint"]["checkpoint_id"]:
        raise PolicyExportEvidenceError("export checkpoint ID differs from selection")
    checkpoint = inputs.get("checkpoint")
    if checkpoint != selection["checkpoint"]:
        raise PolicyExportEvidenceError("export checkpoint differs from selection")
    if inputs.get("run_identity") != selection["run_identity"]:
        raise PolicyExportEvidenceError("export run identity differs from selection")
    if inputs.get("effective_config") != selection["effective_config"]:
        raise PolicyExportEvidenceError("export effective config differs from selection")
    if inputs.get("tensor_contract") != selection["tensor_contract"]:
        raise PolicyExportEvidenceError("export tensor contract differs from selection")
    if version == 4:
        onnx_export_contract = _validate_receipt_onnx_contract(
            inputs.get("onnx_export_contract")
        )
        _validate_onnx_export_evidence(
            receipt.get("onnx_export"),
            contract=onnx_export_contract,
        )
    parity_contract = _validate_parity_contract(inputs.get("parity_contract"))
    _validate_parity(receipt.get("parity"), contract=parity_contract)
    repo_root = Path(
        selection["run_identity"]["document"]["source"]["repository_root"]
    )
    expected = expected_export_paths(
        repo_root,
        task=task,
        run_id=run_id,
        export_id=export_id,
    )
    declared_receipt_path = canonical_receipt_path or receipt_path
    if declared_receipt_path != expected["receipt"]:
        raise PolicyExportEvidenceError("export receipt is outside the evidence layout")
    if receipt.get("receipt_path") != str(declared_receipt_path):
        raise PolicyExportEvidenceError("embedded export receipt path mismatch")
    artifact_references: dict[str, dict[str, Any]] = {}
    for kind in ("jit", "onnx"):
        value = outputs[kind]
        if not isinstance(value, dict):
            raise PolicyExportEvidenceError(f"export {kind} reference is invalid")
        reference = _regular_file(
            Path(value.get("path", "")),
            label=f"export {kind}",
            expected_sha256=value.get("sha256"),
        )
        if Path(reference["path"]) != expected[kind] or value != reference:
            raise PolicyExportEvidenceError(f"export {kind} metadata/path mismatch")
        artifact_references[kind] = reference
    return {
        "status": "valid",
        "receipt": receipt_reference,
        "selection": selection_reference,
        "selection_document": selection,
        "checkpoint": selection["checkpoint"],
        "evaluations": selection["evaluation_results"],
        "outputs": artifact_references,
        "document": receipt,
    }


def _parse_json_object(raw: str, *, label: str) -> dict[str, Any]:
    try:
        value = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise PolicyExportEvidenceError(f"invalid {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise PolicyExportEvidenceError(f"{label} must decode to an object")
    return value


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    selection = subparsers.add_parser("record-selection")
    selection.add_argument("--selection-id", required=True)
    selection.add_argument("--approved-at", required=True)
    selection.add_argument("--checkpoint-id", required=True)
    selection.add_argument("--checkpoint", required=True)
    selection.add_argument("--checkpoint-sha256", required=True)
    selection.add_argument("--selection-report", required=True)
    selection.add_argument("--selection-report-sha256", required=True)
    selection.add_argument("--run-identity", required=True)
    selection.add_argument("--run-identity-file-sha256", required=True)
    selection.add_argument("--effective-config", required=True)
    selection.add_argument("--effective-config-sha256", required=True)
    selection.add_argument("--evaluation-result", action="append", default=[])
    selection.add_argument("--tensor-contract-json", required=True)
    selection.add_argument("--output", required=True)
    validate = subparsers.add_parser("validate-export")
    validate.add_argument("receipt")
    args = parser.parse_args()
    try:
        if args.command == "record-selection":
            result = record_checkpoint_selection(
                selection_id=args.selection_id,
                approved_at=args.approved_at,
                checkpoint_id=args.checkpoint_id,
                checkpoint_path=Path(args.checkpoint),
                checkpoint_sha256=args.checkpoint_sha256,
                selection_report_path=Path(args.selection_report),
                selection_report_sha256=args.selection_report_sha256,
                run_identity_path=Path(args.run_identity),
                run_identity_file_sha256=args.run_identity_file_sha256,
                effective_config_path=Path(args.effective_config),
                effective_config_sha256=args.effective_config_sha256,
                evaluation_result_paths=[Path(path) for path in args.evaluation_result],
                tensor_contract=_parse_json_object(
                    args.tensor_contract_json,
                    label="tensor contract JSON",
                ),
                output_path=Path(args.output),
            )
        else:
            result = validate_export_bundle(Path(args.receipt))
    except (PolicyExportEvidenceError, OSError) as exc:
        parser.error(str(exc))
    print(json.dumps(result, indent=2, sort_keys=True, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
