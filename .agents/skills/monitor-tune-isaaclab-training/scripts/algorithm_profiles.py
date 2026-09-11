#!/usr/bin/env python3
"""Shared validation and resolution helpers for algorithm profiles."""

from __future__ import annotations

import hashlib
import json
import re
from pathlib import Path
from typing import Any


DEFAULT_REGISTRY_PATH = (
    Path(__file__).resolve().parent.parent / "references" / "algorithm-profiles.json"
)
PROFILE_KEYS = {
    "id",
    "profile_version",
    "is_generic",
    "extends",
    "match",
    "progress_patterns",
    "metric_aliases",
    "protected_parameter_patterns",
    "resume_required_args",
    "evaluation_capabilities",
}
MATCH_KEYS = {"backends", "algorithm_names", "runner_classes"}
PROGRESS_KEYS = {"name", "regex", "completion_offset"}
EVALUATION_KEYS = {
    "play_entrypoint",
    "supported_artifacts",
    "history_contract",
}
SUPPORTED_ARTIFACTS = {"native", "jit", "onnx"}
HISTORY_CONTRACTS = {
    "current_observation",
    "flat_time_major_history",
    "backend_defined",
    "review_required",
}


class ProfileError(ValueError):
    """Raised when an algorithm profile registry is invalid."""


def normalize_metric_name(label: str) -> str:
    """Convert an arbitrary console label to a stable metric key."""
    return re.sub(r"[^a-z0-9]+", "_", label.strip().lower()).strip("_")


def _expect_string_list(value: Any, path: str, allow_empty: bool = True) -> list[str]:
    if not isinstance(value, list) or (not allow_empty and not value):
        qualifier = "" if allow_empty else " non-empty"
        raise ProfileError(f"{path} must be a{qualifier} string array")
    for index, item in enumerate(value):
        if not isinstance(item, str) or not item:
            raise ProfileError(f"{path}[{index}] must be a non-empty string")
    return value


def _validate_profile(profile: Any, index: int) -> dict[str, Any]:
    path = f"profiles[{index}]"
    if not isinstance(profile, dict):
        raise ProfileError(f"{path} must be an object")
    unknown = sorted(set(profile) - PROFILE_KEYS)
    if unknown:
        raise ProfileError(f"{path} contains unknown field(s): {', '.join(unknown)}")
    profile_id = profile.get("id")
    if not isinstance(profile_id, str) or not re.fullmatch(r"[a-z0-9][a-z0-9-]*", profile_id):
        raise ProfileError(f"{path}.id must use lowercase letters, digits, and hyphens")
    version = profile.get("profile_version")
    if isinstance(version, bool) or not isinstance(version, int) or version < 1:
        raise ProfileError(f"{path}.profile_version must be a positive integer")
    if not isinstance(profile.get("is_generic"), bool):
        raise ProfileError(f"{path}.is_generic must be a boolean")
    if "extends" in profile and (not isinstance(profile["extends"], str) or not profile["extends"]):
        raise ProfileError(f"{path}.extends must be a non-empty string")

    match = profile.get("match")
    if not isinstance(match, dict):
        raise ProfileError(f"{path}.match must be an object")
    unknown_match = sorted(set(match) - MATCH_KEYS)
    if unknown_match:
        raise ProfileError(f"{path}.match contains unknown field(s): {', '.join(unknown_match)}")
    for key in MATCH_KEYS:
        _expect_string_list(match.get(key), f"{path}.match.{key}", allow_empty=False)

    progress_patterns = profile.get("progress_patterns", [])
    if not isinstance(progress_patterns, list):
        raise ProfileError(f"{path}.progress_patterns must be an array")
    for pattern_index, pattern in enumerate(progress_patterns):
        pattern_path = f"{path}.progress_patterns[{pattern_index}]"
        if not isinstance(pattern, dict):
            raise ProfileError(f"{pattern_path} must be an object")
        unknown_pattern = sorted(set(pattern) - PROGRESS_KEYS)
        if unknown_pattern:
            raise ProfileError(
                f"{pattern_path} contains unknown field(s): {', '.join(unknown_pattern)}"
            )
        if not isinstance(pattern.get("name"), str) or not pattern["name"]:
            raise ProfileError(f"{pattern_path}.name must be a non-empty string")
        regex = pattern.get("regex")
        if not isinstance(regex, str):
            raise ProfileError(f"{pattern_path}.regex must be a string")
        try:
            compiled = re.compile(regex)
        except re.error as exc:
            raise ProfileError(f"{pattern_path}.regex is invalid: {exc}") from exc
        if "current" not in compiled.groupindex:
            raise ProfileError(f"{pattern_path}.regex must define a named current group")
        offset = pattern.get("completion_offset")
        if isinstance(offset, bool) or not isinstance(offset, int) or not 0 <= offset <= 1:
            raise ProfileError(f"{pattern_path}.completion_offset must be 0 or 1")

    aliases = profile.get("metric_aliases", {})
    if not isinstance(aliases, dict):
        raise ProfileError(f"{path}.metric_aliases must be an object")
    for label, metric in aliases.items():
        if not isinstance(label, str) or not label or not isinstance(metric, str) or not metric:
            raise ProfileError(f"{path}.metric_aliases must map non-empty strings")

    protected = _expect_string_list(
        profile.get("protected_parameter_patterns", []),
        f"{path}.protected_parameter_patterns",
    )
    for pattern_index, pattern in enumerate(protected):
        try:
            re.compile(pattern)
        except re.error as exc:
            raise ProfileError(
                f"{path}.protected_parameter_patterns[{pattern_index}] is invalid: {exc}"
            ) from exc
    _expect_string_list(
        profile.get("resume_required_args", []),
        f"{path}.resume_required_args",
    )

    evaluation = profile.get("evaluation_capabilities")
    if evaluation is not None:
        if not isinstance(evaluation, dict):
            raise ProfileError(f"{path}.evaluation_capabilities must be an object")
        unknown_evaluation = sorted(set(evaluation) - EVALUATION_KEYS)
        if unknown_evaluation:
            raise ProfileError(
                f"{path}.evaluation_capabilities contains unknown field(s): "
                f"{', '.join(unknown_evaluation)}"
            )
        entrypoint = evaluation.get("play_entrypoint")
        if entrypoint is not None and (
            not isinstance(entrypoint, str) or not entrypoint
        ):
            raise ProfileError(
                f"{path}.evaluation_capabilities.play_entrypoint must be null "
                "or a non-empty string"
            )
        artifacts = _expect_string_list(
            evaluation.get("supported_artifacts", []),
            f"{path}.evaluation_capabilities.supported_artifacts",
            allow_empty=False,
        )
        unsupported = sorted(set(artifacts) - SUPPORTED_ARTIFACTS)
        if unsupported:
            raise ProfileError(
                f"{path}.evaluation_capabilities.supported_artifacts contains "
                f"unsupported value(s): {', '.join(unsupported)}"
            )
        if len(artifacts) != len(set(artifacts)):
            raise ProfileError(
                f"{path}.evaluation_capabilities.supported_artifacts must be unique"
            )
        if evaluation.get("history_contract") not in HISTORY_CONTRACTS:
            raise ProfileError(
                f"{path}.evaluation_capabilities.history_contract must be one of "
                f"{', '.join(sorted(HISTORY_CONTRACTS))}"
            )
    return profile


def validate_registry(registry: Any) -> dict[str, Any]:
    """Validate an algorithm profile registry."""
    if not isinstance(registry, dict) or set(registry) != {"schema_version", "profiles"}:
        raise ProfileError("registry must contain only schema_version and profiles")
    if registry.get("schema_version") != 1:
        raise ProfileError("registry schema_version must be 1")
    profiles = registry.get("profiles")
    if not isinstance(profiles, list) or not profiles:
        raise ProfileError("registry profiles must be a non-empty array")
    validated = [_validate_profile(profile, index) for index, profile in enumerate(profiles)]
    ids = [profile["id"] for profile in validated]
    if len(ids) != len(set(ids)):
        raise ProfileError("profile IDs must be unique")
    by_id = {profile["id"]: profile for profile in validated}
    for profile in validated:
        parent = profile.get("extends")
        if parent and parent not in by_id:
            raise ProfileError(f"profile {profile['id']} extends missing profile {parent}")
        seen = {profile["id"]}
        while parent:
            if parent in seen:
                raise ProfileError(f"profile inheritance cycle involving {parent}")
            seen.add(parent)
            parent = by_id[parent].get("extends")
    return registry


def load_registry(path: str | Path = DEFAULT_REGISTRY_PATH) -> dict[str, Any]:
    """Load and validate an algorithm profile registry."""
    registry_path = Path(path)
    try:
        registry = json.loads(registry_path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ProfileError(f"profile registry does not exist: {registry_path}") from exc
    except json.JSONDecodeError as exc:
        raise ProfileError(
            f"invalid profile registry JSON at line {exc.lineno}, column {exc.colno}: {exc.msg}"
        ) from exc
    return validate_registry(registry)


def _merge_unique(parent: list[Any], child: list[Any]) -> list[Any]:
    merged: list[Any] = []
    for item in [*parent, *child]:
        if item not in merged:
            merged.append(item)
    return merged


def resolve_profile(registry: dict[str, Any], profile_id: str) -> dict[str, Any]:
    """Resolve inheritance for a profile."""
    by_id = {profile["id"]: profile for profile in registry["profiles"]}
    if profile_id not in by_id:
        raise ProfileError(f"unknown profile ID: {profile_id}")
    chain: list[dict[str, Any]] = []
    current = by_id[profile_id]
    while current:
        chain.append(current)
        parent = current.get("extends")
        current = by_id[parent] if parent else None
    chain.reverse()

    resolved: dict[str, Any] = {
        "id": profile_id,
        "profile_version": by_id[profile_id]["profile_version"],
        "is_generic": by_id[profile_id]["is_generic"],
        "match": by_id[profile_id]["match"],
        "progress_patterns": [],
        "metric_aliases": {},
        "protected_parameter_patterns": [],
        "resume_required_args": [],
        "evaluation_capabilities": {},
    }
    for profile in chain:
        resolved["progress_patterns"] = _merge_unique(
            resolved["progress_patterns"], profile.get("progress_patterns", [])
        )
        resolved["metric_aliases"].update(profile.get("metric_aliases", {}))
        resolved["protected_parameter_patterns"] = _merge_unique(
            resolved["protected_parameter_patterns"],
            profile.get("protected_parameter_patterns", []),
        )
        resolved["resume_required_args"] = _merge_unique(
            resolved["resume_required_args"], profile.get("resume_required_args", [])
        )
        resolved["evaluation_capabilities"].update(
            profile.get("evaluation_capabilities", {})
        )
    return resolved


def profile_fingerprint(profile: dict[str, Any]) -> str:
    """Return a stable short fingerprint for a resolved profile."""
    encoded = json.dumps(profile, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()[:16]


def _match_values(choices: list[str], actual: str) -> tuple[bool, int]:
    lowered = {choice.casefold() for choice in choices}
    if actual.casefold() in lowered:
        return True, 10
    if "*" in choices:
        return True, 0
    return False, 0


def profile_matches(profile: dict[str, Any], backend: str, algorithm: str, runner: str) -> bool:
    """Return whether an unresolved profile matches an exact identity."""
    match = profile["match"]
    return all(
        _match_values(match[key], value)[0]
        for key, value in (
            ("backends", backend),
            ("algorithm_names", algorithm),
            ("runner_classes", runner),
        )
    )


def match_profile(
    registry: dict[str, Any],
    backend: str,
    algorithm: str,
    runner: str,
) -> dict[str, Any]:
    """Select the most specific matching profile."""
    candidates: list[tuple[int, str, dict[str, Any]]] = []
    for profile in registry["profiles"]:
        score = 0
        matches = True
        for key, value in (
            ("backends", backend),
            ("algorithm_names", algorithm),
            ("runner_classes", runner),
        ):
            matched, contribution = _match_values(profile["match"][key], value)
            matches = matches and matched
            score += contribution
        if matches:
            candidates.append((score, profile["id"], profile))
    if not candidates:
        raise ProfileError(
            f"no profile matches backend={backend}, algorithm={algorithm}, runner={runner}"
        )
    _, profile_id, _ = max(candidates, key=lambda item: (item[0], item[1]))
    return resolve_profile(registry, profile_id)
