"""Shared, versioned provenance paths and immutable content-addressed objects.

No simulator imports. Historical validation checks captured content; preflight
additionally checks the live evaluator sources. The mutable index is not evidence.
"""
from __future__ import annotations

import hashlib
import json
import os
import re
import subprocess
import tempfile
from pathlib import Path


def safe_path(path: Path) -> None:
    if not path.is_absolute() or ".." in path.parts:
        raise ValueError("path must be absolute without traversal")
    for part in (path, *path.parents):
        if part.is_symlink():
            raise ValueError(f"symlinked path: {part}")


def identifier(value: str) -> str:
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]{0,127}", value):
        raise ValueError("unsafe identifier")
    return value


def encode(value: dict) -> bytes:
    return (json.dumps(value, indent=2, sort_keys=True, ensure_ascii=False, allow_nan=False) + "\n").encode()


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def read_reference(reference: dict) -> dict:
    if not isinstance(reference, dict) or set(reference) != {"path", "sha256"}:
        raise ValueError("reference must contain path and sha256")
    path = Path(reference["path"])
    safe_path(path)
    data = path.read_bytes()
    if digest(data) != reference["sha256"]:
        raise ValueError(f"reference SHA-256 mismatch: {path}")
    value = json.loads(data)
    if not isinstance(value, dict):
        raise ValueError("reference must contain a JSON object")
    return value


def write_bytes(path: Path, data: bytes, *, reuse: bool = False, replace_index: bool = False) -> dict:
    safe_path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    safe_path(path)
    if replace_index and path.name != "index.md":
        raise ValueError("only index.md may be replaced")
    # Private same-filesystem temporary + exclusive link; readers see whole files.
    with tempfile.TemporaryDirectory(prefix=".publish-", dir=path.parent) as temporary:
        source = Path(temporary) / "content"
        with source.open("xb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        if replace_index:
            safe_path(path)
            os.replace(source, path)
        else:
            try:
                os.link(source, path, follow_symlinks=False)
            except FileExistsError:
                safe_path(path)
                if not reuse or not path.is_file() or path.read_bytes() != data:
                    raise ValueError(f"evidence already exists or differs: {path}")
    return {"path": str(path), "sha256": digest(data)}


def run_root(identity: dict) -> Path:
    return (Path(identity["source"]["repository_root"]) / "learnings" / "policy_tuning"
            / identifier(identity["task"]) / identifier(identity["run_id"]))


def store_object(root: Path, kind: str, value: dict) -> dict:
    if kind not in {"context", "config", "scenario", "source"}:
        raise ValueError("unsupported provenance kind")
    data = encode(value)
    path = root / "provenance" / f"{kind}-{digest(data)}.json"
    return write_bytes(path, data, reuse=True)


def require_source_path(path: Path, root: Path, kind: str) -> None:
    """Select old/new layout by the artifact's explicit schema, never by fallback."""
    safe_path(path)
    try:
        data = path.read_bytes()
        value = json.loads(data)
    except (OSError, ValueError) as exc:
        raise ValueError(f"cannot read source evidence: {path}") from exc
    if not isinstance(value, dict):
        raise ValueError("source evidence must be an object")
    if value.get("version") == 2 and kind in {"context", "config"}:
        expected = root / "provenance" / f"{kind}-{digest(path.read_bytes())}.json"
        if path != expected:
            raise ValueError("content-addressed provenance path mismatch")
    elif value.get("version") == 1 and kind in {"context", "config"}:
        prefix = "identity" if kind == "context" else "effective-config"
        if path.parent != root / "evidence" / "source" or not re.fullmatch(
            prefix + r"-[A-Za-z0-9][A-Za-z0-9._-]{0,127}\.json", path.name
        ):
            raise ValueError("legacy source evidence layout mismatch")
    else:
        raise ValueError("unsupported provenance schema")


def require_object(reference: dict, root: Path, kind: str) -> dict:
    value = read_reference(reference)
    if Path(reference["path"]) != root / "provenance" / f"{kind}-{reference['sha256']}.json":
        raise ValueError("provenance reference outside the current run")
    return value


def capture_evaluator_source(root: Path, repo: Path, files: list[Path]) -> dict:
    """Preserve exact source bytes once, including authorized untracked helpers."""
    safe_path(repo)
    before = subprocess.check_output(["git", "-C", str(repo), "rev-parse", "HEAD"], text=True).strip()
    captured = []
    for path in sorted(set(files)):
        safe_path(path)
        relative = str(path.relative_to(repo))
        data = path.read_bytes()
        captured.append({"path": relative, "sha256": digest(data), "content_utf8": data.decode("utf-8")})
    after = subprocess.check_output(["git", "-C", str(repo), "rev-parse", "HEAD"], text=True).strip()
    if before != after or any(digest((repo / f["path"]).read_bytes()) != f["sha256"] for f in captured):
        raise ValueError("evaluator source changed during capture")
    return store_object(root, "source", {"version": 1, "repository_root": str(repo), "head": before, "files": captured})


def capture_scenario(identity: dict, contract: dict, source: dict) -> dict:
    from capture_run_identity import validate_scenario_contract
    validate_scenario_contract(contract)
    if contract["seed"] != identity["seed"]:
        raise ValueError("scenario seed mismatch")
    return store_object(run_root(identity), "scenario", {
        "version": 1, "task": identity["task"], "run_id": identity["run_id"],
        "contract": contract, "evaluator_source": source,
    })


def validate_provenance(identity: dict, config_ref: dict, scenario_ref: dict, contract: dict, *, live: bool = False) -> None:
    from capture_effective_training_config import load_and_validate_effective_config
    from capture_run_identity import validate_run_identity
    validate_run_identity(identity)
    if identity["version"] != 2:
        raise ValueError("batch evaluations require training context version 2")
    root = run_root(identity)
    config = require_object(config_ref, root, "config")
    if config.get("version") != 2:
        raise ValueError("batch evaluations require config version 2")
    load_and_validate_effective_config(Path(config_ref["path"]), expected_sha256=config_ref["sha256"], run_identity=identity)
    scenario = require_object(scenario_ref, root, "scenario")
    if set(scenario) != {"version", "task", "run_id", "contract", "evaluator_source"} or scenario["version"] != 1:
        raise ValueError("invalid scenario evidence")
    if scenario["contract"] != contract or any(scenario[f] != identity[f] for f in ("task", "run_id")):
        raise ValueError("scenario evidence scope or contract mismatch")
    source = require_object(scenario["evaluator_source"], root, "source")
    if source.get("version") != 1 or source.get("repository_root") != identity["source"]["repository_root"]:
        raise ValueError("evaluator source scope mismatch")
    if not re.fullmatch(r"[0-9a-f]{40}", source.get("head", "")) or not source.get("files"):
        raise ValueError("evaluator source missing")
    seen = set()
    for entry in source["files"]:
        relative = Path(entry["path"])
        if relative.is_absolute() or ".." in relative.parts or str(relative) in seen:
            raise ValueError("unsafe or duplicate evaluator source")
        seen.add(str(relative))
        if digest(entry["content_utf8"].encode("utf-8")) != entry["sha256"]:
            raise ValueError("captured evaluator source SHA-256 mismatch")
        if live:
            path = Path(source["repository_root"]) / relative
            safe_path(path)
            if digest(path.read_bytes()) != entry["sha256"]:
                raise ValueError(f"live evaluator source drift: {relative}")
    if live:
        required = {
            "scripts/reinforcement_learning/rsl_rl/evaluate_policy.py",
            "scripts/reinforcement_learning/rsl_rl/policy_evaluation_evidence.py",
            "scripts/reinforcement_learning/rsl_rl/policy_evaluation_telemetry.py",
            ".agents/skills/monitor-tune-isaaclab-training/scripts/evidence_provenance.py",
        }
        if not required <= seen:
            raise ValueError("missing required evaluator source files")
