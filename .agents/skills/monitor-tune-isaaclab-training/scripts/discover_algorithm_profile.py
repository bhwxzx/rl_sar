#!/usr/bin/env python3
"""Discover an algorithm identity and propose a profile for an unknown trainer."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import sys
from pathlib import Path
from typing import Any

from algorithm_profiles import (
    DEFAULT_REGISTRY_PATH,
    ProfileError,
    load_registry,
    match_profile,
    normalize_metric_name,
    profile_fingerprint,
)
from capture_run_identity import RunIdentityError, validate_run_identity


ANSI_RE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
NUMERIC_LABEL_RE = re.compile(
    r"^\s*([^:]+):\s*[+-]?(?:(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?|nan|inf|-inf)s?\s*$",
    re.IGNORECASE,
)
CLASS_NAME_RE = re.compile(r"^\s*class_name\s*:\s*['\"]?([A-Za-z0-9_]+)", re.MULTILINE)
PROFILE_BACKENDS = {"rsl_rl", "skrl", "cusrl", "custom"}


def _validate_identity(data: Any) -> dict[str, Any]:
    try:
        validate_run_identity(data)
    except RunIdentityError as exc:
        raise ProfileError(str(exc)) from exc
    assert isinstance(data, dict)
    for field in ("backend", "algorithm", "runner"):
        if data[field].casefold() == "auto":
            raise ProfileError(f"run_identity.{field} cannot use legacy auto semantics")
    return data


def _load_run_identity(path: Path) -> dict[str, Any]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ProfileError(f"run identity does not exist: {path}") from exc
    except json.JSONDecodeError as exc:
        raise ProfileError(
            f"invalid run identity JSON at line {exc.lineno}: {exc.msg}"
        ) from exc
    return _validate_identity(data)


def _resolve_profile_backend(command: list[str], declared_backend: str) -> tuple[str, str]:
    joined = " ".join(str(item) for item in command).lower()
    normalized_declared = declared_backend.casefold()
    detected = [
        backend
        for backend in ("rsl_rl", "skrl", "cusrl")
        if backend in joined
    ]
    if len(detected) > 1:
        raise ProfileError(
            "training command contains multiple algorithm backend markers: "
            + ", ".join(detected)
        )
    if detected:
        backend = detected[0]
        if normalized_declared in PROFILE_BACKENDS and normalized_declared != backend:
            raise ProfileError(
                "run identity backend conflicts with the training command backend"
            )
        return backend, "training_command"
    if normalized_declared in PROFILE_BACKENDS:
        return normalized_declared, "run_identity"
    return "custom", "training_command_fallback"


def _validated_config(
    run_identity: dict[str, Any],
    path: Path | None,
) -> tuple[list[str], dict[str, str] | None]:
    if path is None:
        return [], None
    try:
        resolved_path = path.resolve(strict=True)
        content = resolved_path.read_bytes()
    except FileNotFoundError as exc:
        raise ProfileError(f"config file does not exist: {path}") from exc
    repository_root = Path(run_identity["source"]["repository_root"]).resolve()
    try:
        relative_path = resolved_path.relative_to(repository_root).as_posix()
    except ValueError as exc:
        raise ProfileError(
            "dumped config is outside the run identity repository root"
        ) from exc
    recorded = {
        entry["path"]: entry["sha256"]
        for entry in run_identity["config_files"]
    }
    if relative_path not in recorded:
        raise ProfileError(
            "dumped config is not listed in run_identity.config_files"
        )
    actual_sha256 = hashlib.sha256(content).hexdigest()
    if actual_sha256 != recorded[relative_path]:
        raise ProfileError(
            "dumped config sha256 does not match run_identity.config_files"
        )
    text = content.decode("utf-8", errors="replace")
    return CLASS_NAME_RE.findall(text), {
        "path": relative_path,
        "sha256": actual_sha256,
    }


def _single_config_candidate(label: str, candidates: list[str]) -> str | None:
    unique = sorted(set(candidates))
    if len(unique) > 1:
        raise ProfileError(
            f"dumped config contains ambiguous {label} class names: "
            + ", ".join(unique)
        )
    return unique[0] if unique else None


def _resolve_identity(
    run_identity: dict[str, Any],
    config_path: Path | None,
) -> tuple[dict[str, str], dict[str, Any]]:
    backend, backend_source = _resolve_profile_backend(
        run_identity["training"]["command"],
        run_identity["backend"],
    )
    declared_algorithm = run_identity["algorithm"]
    declared_runner = run_identity["runner"]
    class_names, config_file = _validated_config(run_identity, config_path)
    config_runner = _single_config_candidate(
        "runner",
        [name for name in class_names if "Runner" in name],
    )
    config_algorithm = _single_config_candidate(
        "algorithm",
        [
            name
            for name in class_names
            if name != config_runner and ("PPO" in name or name == "Distillation")
        ],
    )

    runner = declared_runner
    runner_source = "run_identity"
    if config_runner is not None:
        if declared_runner.casefold() != "unknown" and config_runner != declared_runner:
            raise ProfileError(
                "run identity runner conflicts with the dumped config runner"
            )
        runner = config_runner
        runner_source = "run_identity_and_config" if declared_runner == config_runner else "config"

    algorithm = declared_algorithm
    algorithm_source = "run_identity"
    if config_algorithm is not None:
        declared_is_class_name = (
            "PPO" in declared_algorithm or declared_algorithm == "Distillation"
        )
        if declared_is_class_name and config_algorithm != declared_algorithm:
            raise ProfileError(
                "run identity algorithm class conflicts with the dumped config"
            )
        algorithm = config_algorithm
        algorithm_source = (
            "run_identity_and_config"
            if declared_algorithm == config_algorithm
            else "config"
        )

    return (
        {"backend": backend, "name": algorithm, "runner_class": runner},
        {
            "declared_backend": run_identity["backend"],
            "declared_algorithm": declared_algorithm,
            "declared_runner": declared_runner,
            "backend_source": backend_source,
            "algorithm_source": algorithm_source,
            "runner_source": runner_source,
            "config_file": config_file,
            "config_class_names": class_names,
        },
    )


def _discover_metric_aliases(
    log_path: Path | None,
) -> tuple[dict[str, str], dict[str, Any] | None]:
    if log_path is None:
        return {}, None
    try:
        resolved_path = log_path.resolve(strict=True)
        stream = resolved_path.open("rb")
    except FileNotFoundError as exc:
        raise ProfileError(f"log file does not exist: {log_path}") from exc
    aliases: dict[str, str] = {}
    digest = hashlib.sha256()
    byte_count = 0
    with stream:
        before = os.fstat(stream.fileno())
        for raw_line in stream:
            digest.update(raw_line)
            byte_count += len(raw_line)
            if len(aliases) < 512:
                line = ANSI_RE.sub(
                    "",
                    raw_line.decode("utf-8", errors="replace").rstrip("\n"),
                )
                match = NUMERIC_LABEL_RE.match(line)
                if match:
                    label = match.group(1).strip()
                    aliases[label] = normalize_metric_name(label)
        after = os.fstat(stream.fileno())
    stable_fields = ("st_dev", "st_ino", "st_size", "st_mtime_ns")
    if (
        any(getattr(before, field) != getattr(after, field) for field in stable_fields)
        or byte_count != after.st_size
    ):
        raise ProfileError("log file changed while metric aliases were discovered")
    return dict(sorted(aliases.items())), {
        "path": str(resolved_path),
        "sha256": digest.hexdigest(),
        "size_bytes": byte_count,
    }


def _safe_id(value: str) -> str:
    normalized = re.sub(r"[^a-z0-9]+", "-", value.lower()).strip("-")
    return normalized or "unknown"


def discover(
    run_identity: dict[str, Any],
    registry: dict[str, Any],
    log_path: Path | None,
    config_path: Path | None,
) -> dict[str, Any]:
    """Return a matched profile or a candidate specific profile."""
    run_identity = _validate_identity(run_identity)
    identity, identity_resolution = _resolve_identity(run_identity, config_path)
    matched = match_profile(
        registry,
        identity["backend"],
        identity["name"],
        identity["runner_class"],
    )
    result: dict[str, Any] = {
        "schema_version": 2,
        "run_identity": {
            "task": run_identity["task"],
            "run_id": run_identity["run_id"],
            "host_id": run_identity["host_id"],
            "identity_sha256": run_identity["identity_sha256"],
        },
        "identity": identity,
        "identity_resolution": identity_resolution,
        "matched_profile": {
            "id": matched["id"],
            "profile_version": matched["profile_version"],
            "is_generic": matched["is_generic"],
            "fingerprint": profile_fingerprint(matched),
        },
    }
    if not matched["is_generic"]:
        result["status"] = "matched"
        return result

    parent_id = matched["id"]
    metric_aliases, metric_source = _discover_metric_aliases(log_path)
    candidate_id = (
        f"{_safe_id(identity['backend'])}-"
        f"{_safe_id(identity['name'])}-"
        f"{_safe_id(identity['runner_class'])}"
    )
    result["status"] = "candidate"
    result["candidate_profile"] = {
        "id": candidate_id,
        "profile_version": 2,
        "is_generic": False,
        "extends": parent_id,
        "match": {
            "backends": [identity["backend"]],
            "algorithm_names": [identity["name"]],
            "runner_classes": [identity["runner_class"]],
        },
        "metric_aliases": metric_aliases,
        "metric_source": metric_source,
        "protected_parameter_patterns": [],
        "evaluation_capabilities": {
            "play_entrypoint": matched["evaluation_capabilities"].get(
                "play_entrypoint"
            ),
            "supported_artifacts": ["native"],
            "history_contract": "review_required",
        },
    }
    result["required_next_action"] = "review_and_approve_persistent_profile"
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_identity", help="Validated host-local run identity JSON")
    parser.add_argument("--config", help="Optional dumped trainer configuration")
    parser.add_argument("--log", help="Optional console log for metric discovery")
    parser.add_argument("--registry", default=str(DEFAULT_REGISTRY_PATH))
    parser.add_argument("--output")
    args = parser.parse_args()
    try:
        result = discover(
            _load_run_identity(Path(args.run_identity)),
            load_registry(args.registry),
            Path(args.log) if args.log else None,
            Path(args.config) if args.config else None,
        )
    except ProfileError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    encoded = json.dumps(result, indent=2, sort_keys=True, ensure_ascii=False)
    if args.output:
        Path(args.output).write_text(encoded + "\n", encoding="utf-8")
    else:
        print(encoded)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
