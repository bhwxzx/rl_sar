#!/usr/bin/env python3
"""Capture deterministic local run identity without changing Git state."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import subprocess
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[4]
SAFE_IDENTIFIER_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
HEAD_RE = re.compile(r"^[0-9a-f]{40}$")
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
SCENARIO_FIELDS = {
    "scenario_id",
    "scenario_overrides",
    "command_schedule",
    "duration_steps",
    "num_envs",
    "seed",
}


class RunIdentityError(ValueError):
    """Raised when a run identity is incomplete, unsafe, or inconsistent."""


def _canonical_json(value: Any) -> str:
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    )


def _sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _validate_identifier(name: str, value: Any) -> None:
    if not isinstance(value, str) or not SAFE_IDENTIFIER_RE.fullmatch(value):
        raise RunIdentityError(f"{name} must be a safe ASCII identifier")


def _validate_string_array(name: str, value: Any, *, nonempty: bool) -> None:
    if (
        not isinstance(value, list)
        or (nonempty and not value)
        or any(not isinstance(item, str) or not item for item in value)
    ):
        requirement = "a non-empty" if nonempty else "an"
        raise RunIdentityError(f"{name} must be {requirement} array of non-empty strings")


def validate_scenario_contract(value: Any) -> None:
    if not isinstance(value, dict) or set(value) != SCENARIO_FIELDS:
        raise RunIdentityError(
            "evaluation scenario must contain exactly scenario_id, "
            "scenario_overrides, command_schedule, duration_steps, num_envs, and seed"
        )
    _validate_identifier("evaluation scenario_id", value["scenario_id"])
    if not isinstance(value["scenario_overrides"], dict):
        raise RunIdentityError("evaluation scenario_overrides must be an object")
    if not isinstance(value["command_schedule"], list):
        raise RunIdentityError("evaluation command_schedule must be an array")
    for field in ("duration_steps", "num_envs"):
        field_value = value[field]
        if isinstance(field_value, bool) or not isinstance(field_value, int) or field_value <= 0:
            raise RunIdentityError(f"evaluation {field} must be a positive integer")
    if isinstance(value["seed"], bool) or not isinstance(value["seed"], int):
        raise RunIdentityError("evaluation seed must be an integer")
    previous_end = -1
    for index, segment in enumerate(value["command_schedule"]):
        if not isinstance(segment, dict) or set(segment) != {
            "start_step",
            "end_step",
            "command",
        }:
            raise RunIdentityError(f"evaluation command schedule segment {index} is invalid")
        start = segment["start_step"]
        end = segment["end_step"]
        command = segment["command"]
        if (
            isinstance(start, bool)
            or not isinstance(start, int)
            or isinstance(end, bool)
            or not isinstance(end, int)
            or start != previous_end + 1
            or end < start
            or end >= value["duration_steps"]
        ):
            raise RunIdentityError(
                "evaluation command schedule must be ordered, contiguous, and within duration"
            )
        if (
            not isinstance(command, list)
            or len(command) != 3
            or any(
                isinstance(item, bool)
                or not isinstance(item, (int, float))
                or not math.isfinite(float(item))
                for item in command
            )
        ):
            raise RunIdentityError(
                "evaluation command schedule values must be finite [vx, vy, yaw_rate]"
            )
        previous_end = end
    if value["command_schedule"] and previous_end != value["duration_steps"] - 1:
        raise RunIdentityError(
            "evaluation command schedule must cover every evaluation step"
        )
    try:
        _canonical_json(value)
    except (TypeError, ValueError) as exc:
        raise RunIdentityError(f"evaluation scenario is not finite JSON: {exc}") from exc


def _reject_symlink_components(path: Path, *, label: str) -> None:
    if not path.is_absolute():
        raise RunIdentityError(f"{label} must be absolute")
    current = Path(path.anchor)
    for component in path.parts[1:]:
        current /= component
        if current.is_symlink():
            raise RunIdentityError(
                f"{label} contains a symlinked path component: {current}"
            )


def _safe_absolute_file(path: Path, *, label: str) -> Path:
    _reject_symlink_components(path, label=label)
    if not path.is_file():
        raise RunIdentityError(f"{label} is not a file: {path}")
    return path


def _repository_file(repo_root: Path, raw_path: str) -> tuple[Path, str]:
    candidate = Path(raw_path)
    if ".." in candidate.parts:
        raise RunIdentityError(f"config path cannot contain traversal: {raw_path}")
    if not candidate.is_absolute():
        candidate = repo_root / candidate
    candidate = _safe_absolute_file(candidate, label="config path")
    try:
        relative = candidate.relative_to(repo_root)
    except ValueError as exc:
        raise RunIdentityError(f"config path is outside repository: {candidate}") from exc
    if not relative.parts or relative.parts[0] == ".git":
        raise RunIdentityError(f"config path is not repository source: {candidate}")
    return candidate, relative.as_posix()


def _git_text(
    repo_root: Path,
    *args: str,
    allow_failure: bool = False,
) -> str:
    result = subprocess.run(
        ["git", "-c", "core.fsmonitor=false", "-C", str(repo_root), *args],
        check=False,
        capture_output=True,
        text=True,
        env={
            **os.environ,
            "GIT_OPTIONAL_LOCKS": "0",
            "GIT_TERMINAL_PROMPT": "0",
        },
    )
    if result.returncode != 0 and not allow_failure:
        raise RunIdentityError(
            f"local git {' '.join(args)} failed: {result.stderr.strip()}"
        )
    return result.stdout if result.returncode == 0 else ""


def _git_bytes(
    repo_root: Path,
    *args: str,
    allow_failure: bool = False,
) -> bytes | None:
    result = subprocess.run(
        ["git", "-c", "core.fsmonitor=false", "-C", str(repo_root), *args],
        check=False,
        capture_output=True,
        env={
            **os.environ,
            "GIT_OPTIONAL_LOCKS": "0",
            "GIT_TERMINAL_PROMPT": "0",
        },
    )
    if result.returncode != 0 and not allow_failure:
        raise RunIdentityError(
            f"local git {' '.join(args)} failed: "
            f"{result.stderr.decode('utf-8', errors='replace').strip()}"
        )
    return result.stdout if result.returncode == 0 else None


def _inspect_config_state(
    repo_root: Path,
    resolved_configs: list[tuple[Path, str]],
) -> tuple[list[str], list[str]]:
    dirty_paths: list[str] = []
    untracked_paths: list[str] = []
    for path, relative in resolved_configs:
        status = _git_text(
            repo_root,
            "status",
            "--porcelain=v1",
            "--untracked-files=all",
            "--",
            relative,
        )
        tracked = _git_text(
            repo_root,
            "ls-files",
            "--error-unmatch",
            "--",
            relative,
            allow_failure=True,
        )
        if not tracked:
            untracked_paths.append(relative)
            dirty_paths.append(relative)
            continue
        head_content = _git_bytes(
            repo_root,
            "show",
            f"HEAD:{relative}",
            allow_failure=True,
        )
        if status or head_content is None or path.read_bytes() != head_content:
            dirty_paths.append(relative)
    return sorted(set(dirty_paths)), sorted(set(untracked_paths))


def validate_run_identity(identity: Any) -> None:
    if not isinstance(identity, dict) or identity.get("version") not in {1, 2}:
        raise RunIdentityError("run_identity.version must be 1 or 2")
    for field in ("task", "run_id", "host_id", "backend", "algorithm", "runner"):
        _validate_identifier(f"run_identity.{field}", identity.get(field))
    seed = identity.get("seed")
    if isinstance(seed, bool) or not isinstance(seed, int):
        raise RunIdentityError("run_identity.seed must be an integer")

    source = identity.get("source")
    if not isinstance(source, dict):
        raise RunIdentityError("run_identity.source must be an object")
    repository_root = source.get("repository_root")
    if (
        not isinstance(repository_root, str)
        or not Path(repository_root).is_absolute()
        or ".." in Path(repository_root).parts
    ):
        raise RunIdentityError("run_identity.source.repository_root must be absolute")
    branch = source.get("branch")
    if not isinstance(branch, str) or not branch or len(branch) > 255:
        raise RunIdentityError("run_identity.source.branch must be a non-empty string")
    head = source.get("head")
    if not isinstance(head, str) or not HEAD_RE.fullmatch(head):
        raise RunIdentityError("run_identity.source.head must be a full lowercase commit")
    dirty = source.get("dirty")
    if not isinstance(dirty, bool):
        raise RunIdentityError("run_identity.source.dirty must be boolean")
    dirty_paths = source.get("dirty_paths")
    if (
        not isinstance(dirty_paths, list)
        or dirty_paths != sorted(set(dirty_paths))
        or any(
            not isinstance(path, str)
            or not path
            or Path(path).is_absolute()
            or ".." in Path(path).parts
            for path in dirty_paths
        )
    ):
        raise RunIdentityError(
            "run_identity.source.dirty_paths must be sorted unique repository-relative paths"
        )
    diff_sha256 = source.get("diff_sha256")
    if diff_sha256 is not None and (
        not isinstance(diff_sha256, str) or not SHA256_RE.fullmatch(diff_sha256)
    ):
        raise RunIdentityError("run_identity.source.diff_sha256 must be null or SHA-256")
    patch_evidence = source.get("patch_evidence")
    if patch_evidence is not None:
        if not isinstance(patch_evidence, dict):
            raise RunIdentityError("run_identity.source.patch_evidence must be an object or null")
        patch_path = patch_evidence.get("path")
        patch_hash = patch_evidence.get("sha256")
        if (
            not isinstance(patch_path, str)
            or not Path(patch_path).is_absolute()
            or ".." in Path(patch_path).parts
        ):
            raise RunIdentityError("run_identity.source.patch_evidence.path must be absolute")
        if not isinstance(patch_hash, str) or not SHA256_RE.fullmatch(patch_hash):
            raise RunIdentityError("run_identity.source.patch_evidence.sha256 must be SHA-256")
    evidence_count = int(diff_sha256 is not None) + int(patch_evidence is not None)
    if dirty and (not dirty_paths or evidence_count != 1):
        raise RunIdentityError(
            "dirty run identity requires dirty_paths and exactly one diff or patch evidence"
        )
    if not dirty and (dirty_paths or evidence_count):
        raise RunIdentityError("clean run identity cannot contain dirty evidence")

    training = identity.get("training")
    if not isinstance(training, dict):
        raise RunIdentityError("run_identity.training must be an object")
    _validate_string_array("run_identity.training.command", training.get("command"), nonempty=True)
    _validate_string_array(
        "run_identity.training.hydra_overrides",
        training.get("hydra_overrides"),
        nonempty=False,
    )
    command_iterator = iter(training["command"])
    if not all(any(token == override for token in command_iterator) for override in training["hydra_overrides"]):
        raise RunIdentityError(
            "run_identity Hydra overrides must appear in command order"
        )

    config_files = identity.get("config_files")
    if not isinstance(config_files, list) or not config_files:
        raise RunIdentityError("run_identity.config_files must be a non-empty array")
    config_paths: list[str] = []
    for entry in config_files:
        if not isinstance(entry, dict):
            raise RunIdentityError("run_identity.config_files entries must be objects")
        path = entry.get("path")
        sha256 = entry.get("sha256")
        if (
            not isinstance(path, str)
            or not path
            or Path(path).is_absolute()
            or ".." in Path(path).parts
        ):
            raise RunIdentityError("config file paths must be repository-relative")
        if not isinstance(sha256, str) or not SHA256_RE.fullmatch(sha256):
            raise RunIdentityError("config file sha256 must be SHA-256")
        config_paths.append(path)
    if config_paths != sorted(set(config_paths)):
        raise RunIdentityError("config file paths must be sorted and unique")

    if identity["version"] == 1:
        evaluation = identity.get("evaluation_scenario")
        if not isinstance(evaluation, dict):
            raise RunIdentityError("run_identity.evaluation_scenario must be an object")
        contract = evaluation.get("contract")
        validate_scenario_contract(contract)
        scenario_hash = evaluation.get("sha256")
        expected_scenario_hash = _sha256_bytes(_canonical_json(contract).encode("utf-8"))
        if scenario_hash != expected_scenario_hash:
            raise RunIdentityError("run_identity evaluation scenario SHA-256 mismatch")

    elif "evaluation_scenario" in identity:
        raise RunIdentityError("training context must not embed a scenario")

    identity_hash = identity.get("identity_sha256")
    payload = {key: value for key, value in identity.items() if key != "identity_sha256"}
    expected_identity_hash = _sha256_bytes(_canonical_json(payload).encode("utf-8"))
    if identity_hash != expected_identity_hash:
        raise RunIdentityError("run_identity identity_sha256 mismatch")


def capture_run_identity(
    repo_root: Path,
    *,
    task: str,
    run_id: str,
    host_id: str,
    backend: str,
    algorithm: str,
    runner: str,
    seed: int,
    training_command: list[str],
    hydra_overrides: list[str],
    config_paths: list[str],
    scenario_contract: dict[str, Any] | None = None,
    patch_evidence_path: Path | None = None,
) -> dict[str, Any]:
    """Capture a host-local identity using only read-only Git queries."""
    for name, value in (
        ("task", task),
        ("run-id", run_id),
        ("host-id", host_id),
        ("backend", backend),
        ("algorithm", algorithm),
        ("runner", runner),
    ):
        _validate_identifier(name, value)
    if isinstance(seed, bool) or not isinstance(seed, int):
        raise RunIdentityError("seed must be an integer")
    _validate_string_array("training command", training_command, nonempty=True)
    _validate_string_array("Hydra overrides", hydra_overrides, nonempty=False)
    command_iterator = iter(training_command)
    if not all(any(token == override for token in command_iterator) for override in hydra_overrides):
        raise RunIdentityError("Hydra overrides must appear in training command order")
    if scenario_contract is not None:
        validate_scenario_contract(scenario_contract)
    if not config_paths:
        raise RunIdentityError("at least one --config is required")

    _reject_symlink_components(repo_root, label="repository root")
    if not repo_root.is_dir():
        raise RunIdentityError("repository root must be an existing non-symlinked absolute directory")
    discovered_root = Path(
        _git_text(repo_root, "rev-parse", "--show-toplevel").strip()
    ).resolve()
    if discovered_root != repo_root:
        raise RunIdentityError(
            f"repository root mismatch: expected {repo_root}, Git reported {discovered_root}"
        )
    branch = _git_text(
        repo_root,
        "symbolic-ref",
        "--quiet",
        "--short",
        "HEAD",
        allow_failure=True,
    ).strip() or "DETACHED"
    head = _git_text(repo_root, "rev-parse", "HEAD").strip()
    if not HEAD_RE.fullmatch(head):
        raise RunIdentityError("Git HEAD is not a full lowercase commit")

    resolved_configs: list[tuple[Path, str]] = []
    for raw_path in config_paths:
        resolved_configs.append(_repository_file(repo_root, raw_path))
    resolved_configs.sort(key=lambda item: item[1])
    relative_paths = [relative for _, relative in resolved_configs]
    if len(relative_paths) != len(set(relative_paths)):
        raise RunIdentityError("--config paths must be unique")

    dirty_paths, untracked_paths = _inspect_config_state(
        repo_root,
        resolved_configs,
    )
    dirty = bool(dirty_paths)

    diff_sha256: str | None = None
    patch_evidence: dict[str, str] | None = None
    if dirty:
        if patch_evidence_path is not None:
            patch_path = _safe_absolute_file(
                patch_evidence_path,
                label="patch evidence",
            )
            expected_patch_parent = (
                repo_root
                / "learnings"
                / "policy_tuning"
                / task
                / run_id
                / "evidence"
                / "source"
            )
            if patch_path.parent != expected_patch_parent or patch_path.suffix != ".patch":
                raise RunIdentityError(
                    "patch evidence must be a .patch file in this run's evidence/source directory"
                )
            patch_bytes = patch_path.read_bytes()
            missing_patch_paths = [
                relative
                for relative in dirty_paths
                if relative.encode("utf-8") not in patch_bytes
            ]
            if missing_patch_paths:
                raise RunIdentityError(
                    "patch evidence does not name dirty path(s): "
                    + ", ".join(missing_patch_paths)
                )
            patch_evidence = {
                "path": str(patch_path),
                "sha256": _sha256_bytes(patch_bytes),
            }
        elif untracked_paths:
            raise RunIdentityError(
                "dirty untracked config requires --patch-evidence: "
                + ", ".join(untracked_paths)
            )
        else:
            diff = _git_bytes(
                repo_root,
                "diff",
                "--no-ext-diff",
                "--no-textconv",
                "--binary",
                "--full-index",
                "HEAD",
                "--",
                *relative_paths,
            )
            if not diff:
                raise RunIdentityError(
                    "dirty tracked config produced no diff; provide --patch-evidence"
                )
            assert isinstance(diff, bytes)
            diff_sha256 = _sha256_bytes(diff)
    elif patch_evidence_path is not None:
        raise RunIdentityError("clean source cannot use --patch-evidence")

    configs = sorted(
        (
            {"path": relative, "sha256": _sha256_file(path)}
            for path, relative in resolved_configs
        ),
        key=lambda item: item["path"],
    )
    final_branch = _git_text(
        repo_root,
        "symbolic-ref",
        "--quiet",
        "--short",
        "HEAD",
        allow_failure=True,
    ).strip() or "DETACHED"
    final_head = _git_text(repo_root, "rev-parse", "HEAD").strip()
    final_dirty_paths, final_untracked_paths = _inspect_config_state(
        repo_root,
        resolved_configs,
    )
    final_configs = sorted(
        (
            {"path": relative, "sha256": _sha256_file(path)}
            for path, relative in resolved_configs
        ),
        key=lambda item: item["path"],
    )
    state_changed = (
        final_branch != branch
        or final_head != head
        or final_dirty_paths != dirty_paths
        or final_untracked_paths != untracked_paths
        or final_configs != configs
    )
    if diff_sha256 is not None:
        final_diff = _git_bytes(
            repo_root,
            "diff",
            "--no-ext-diff",
            "--no-textconv",
            "--binary",
            "--full-index",
            "HEAD",
            "--",
            *relative_paths,
        )
        state_changed = state_changed or not isinstance(final_diff, bytes) or (
            _sha256_bytes(final_diff) != diff_sha256
        )
    if patch_evidence is not None:
        state_changed = state_changed or (
            _sha256_file(Path(patch_evidence["path"])) != patch_evidence["sha256"]
        )
    if state_changed:
        raise RunIdentityError("repository source changed during identity capture")
    scenario_hash = _sha256_bytes(
        _canonical_json(scenario_contract).encode("utf-8")
    )
    identity: dict[str, Any] = {
        "version": 1,
        "task": task,
        "run_id": run_id,
        "host_id": host_id,
        "backend": backend,
        "algorithm": algorithm,
        "runner": runner,
        "seed": seed,
        "source": {
            "repository_root": str(repo_root),
            "branch": branch,
            "head": head,
            "dirty": dirty,
            "dirty_paths": dirty_paths,
            "diff_sha256": diff_sha256,
            "patch_evidence": patch_evidence,
        },
        "training": {
            "command": training_command,
            "hydra_overrides": hydra_overrides,
        },
        "config_files": configs,
        "evaluation_scenario": {
            "contract": scenario_contract,
            "sha256": scenario_hash,
        },
    }
    if scenario_contract is None:
        identity["version"] = 2
        del identity["evaluation_scenario"]
    identity["identity_sha256"] = _sha256_bytes(
        _canonical_json(identity).encode("utf-8")
    )
    validate_run_identity(identity)
    return identity


def write_new_absolute_output(path: Path, encoded: str) -> None:
    if not path.is_absolute():
        raise RunIdentityError("--output must be a new absolute path")
    _reject_symlink_components(path.parent, label="--output")
    if not path.parent.is_dir():
        raise RunIdentityError("--output parent directory does not exist")
    if path.exists() or path.is_symlink():
        raise RunIdentityError("--output already exists")
    try:
        with path.open("x", encoding="utf-8") as stream:
            stream.write(encoded)
    except FileExistsError as exc:
        raise RunIdentityError("--output already exists") from exc


def _parse_json_argument(parser: argparse.ArgumentParser, name: str, raw: str) -> Any:
    try:
        return json.loads(raw)
    except json.JSONDecodeError as exc:
        parser.error(f"{name} is invalid JSON at column {exc.colno}: {exc.msg}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--host-id", required=True)
    parser.add_argument("--backend", required=True)
    parser.add_argument("--algorithm", required=True)
    parser.add_argument("--runner", required=True)
    parser.add_argument("--seed", required=True, type=int)
    parser.add_argument("--training-command-json", required=True)
    parser.add_argument("--hydra-overrides-json", default="[]")
    parser.add_argument("--config", action="append", default=[])
    parser.add_argument("--scenario-contract-json", help="Legacy v1 only; omit for reusable v2 context")
    parser.add_argument("--patch-evidence")
    parser.add_argument("--repo-root", default=str(REPO_ROOT))
    parser.add_argument("--output", help="New legacy identity path; v2 uses content-addressed storage")
    args = parser.parse_args()
    training_command = _parse_json_argument(
        parser,
        "--training-command-json",
        args.training_command_json,
    )
    hydra_overrides = _parse_json_argument(
        parser,
        "--hydra-overrides-json",
        args.hydra_overrides_json,
    )
    scenario_contract = _parse_json_argument(
        parser,
        "--scenario-contract-json",
        args.scenario_contract_json,
    ) if args.scenario_contract_json is not None else None
    try:
        identity = capture_run_identity(
            Path(args.repo_root),
            task=args.task,
            run_id=args.run_id,
            host_id=args.host_id,
            backend=args.backend,
            algorithm=args.algorithm,
            runner=args.runner,
            seed=args.seed,
            training_command=training_command,
            hydra_overrides=hydra_overrides,
            config_paths=args.config,
            scenario_contract=scenario_contract,
            patch_evidence_path=(
                Path(args.patch_evidence) if args.patch_evidence else None
            ),
        )
        encoded = json.dumps(
            identity,
            indent=2,
            sort_keys=True,
            ensure_ascii=False,
            allow_nan=False,
        ) + "\n"
        if identity["version"] == 2:
            from evidence_provenance import store_object, run_root
            if args.output:
                raise RunIdentityError("v2 context chooses its content-addressed output; omit --output")
            print(json.dumps(store_object(run_root(identity), "context", identity)))
        else:
            if not args.output:
                raise RunIdentityError("legacy identity requires --output")
            write_new_absolute_output(Path(args.output), encoded)
    except (OSError, RunIdentityError) as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
