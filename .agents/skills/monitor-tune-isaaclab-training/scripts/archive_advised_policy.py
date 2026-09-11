#!/usr/bin/env python3
"""Atomically archive one user-approved JIT/ONNX pair without Git actions."""

from __future__ import annotations

import argparse
import ctypes
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from datetime import datetime
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from policy_export_evidence import (  # noqa: E402
    PolicyExportEvidenceError,
    validate_export_bundle,
)


SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
HARDWARE_NOTE = "仅可进入受监督实物测试；未经实物验证，不代表 hardware-ready。"


class ArchiveError(ValueError):
    """Raised when archive authorization or storage evidence is invalid."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ArchiveError(f"manifest does not exist: {path}") from exc
    except json.JSONDecodeError as exc:
        raise ArchiveError(f"invalid manifest JSON at line {exc.lineno}: {exc.msg}") from exc
    if not isinstance(value, dict):
        raise ArchiveError("manifest must be a JSON object")
    return value


def _git(root: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(root), *args],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise ArchiveError(result.stderr.strip() or "Git inspection failed")
    return result.stdout.strip()


def _validated_artifact(item: Any, kind: str) -> tuple[Path, str]:
    if not isinstance(item, dict):
        raise ArchiveError(f"artifacts.{kind} must be an object")
    path = Path(item.get("path", ""))
    expected = item.get("sha256")
    suffix = ".pt" if kind == "jit" else ".onnx"
    if not path.is_absolute() or path.is_symlink() or not path.is_file() or path.suffix != suffix:
        raise ArchiveError(f"{kind} artifact must be an absolute regular {suffix} file")
    if not isinstance(expected, str) or not SHA256_RE.fullmatch(expected):
        raise ArchiveError(f"artifacts.{kind}.sha256 must be lowercase SHA-256")
    if _sha256(path) != expected:
        raise ArchiveError(f"{kind} artifact SHA-256 changed")
    return path, expected


def _validate_manifest(manifest: dict[str, Any]) -> dict[str, Any]:
    if manifest.get("version") != 2 or manifest.get("archive_authorized") is not True:
        raise ArchiveError("version-2 manifest with archive_authorized=true is required")
    for field in ("task", "algorithm", "runner", "description_notes"):
        if not isinstance(manifest.get(field), str):
            raise ArchiveError(f"{field} must be a string")
    checkpoint = manifest.get("selected_checkpoint")
    if not isinstance(checkpoint, dict):
        raise ArchiveError("selected_checkpoint must be an object")
    checkpoint_path = Path(checkpoint.get("path", ""))
    checkpoint_hash = checkpoint.get("sha256")
    if not checkpoint_path.is_absolute() or checkpoint_path.is_symlink() or not checkpoint_path.is_file():
        raise ArchiveError("selected checkpoint must be an absolute regular file")
    if not isinstance(checkpoint_hash, str) or not SHA256_RE.fullmatch(checkpoint_hash):
        raise ArchiveError("selected checkpoint SHA-256 is invalid")
    if _sha256(checkpoint_path) != checkpoint_hash:
        raise ArchiveError("selected checkpoint SHA-256 changed")
    iteration = checkpoint.get("iteration")
    if isinstance(iteration, bool) or not isinstance(iteration, int) or iteration < 0:
        raise ArchiveError("selected checkpoint iteration must be non-negative")
    source = manifest.get("source")
    if not isinstance(source, dict) or not isinstance(source.get("commit"), str) or not source["commit"]:
        raise ArchiveError("source.commit is required")
    if not isinstance(source.get("dirty"), bool):
        raise ArchiveError("source.dirty must be boolean")
    if not isinstance(manifest.get("parameters"), dict):
        raise ArchiveError("parameters must be an object")
    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, dict):
        raise ArchiveError("artifacts must be an object")
    export_receipt = manifest.get("export_receipt")
    if not isinstance(export_receipt, dict):
        raise ArchiveError("export_receipt must be an object")
    export_path = export_receipt.get("path")
    export_sha256 = export_receipt.get("sha256")
    if not isinstance(export_path, str) or not isinstance(export_sha256, str):
        raise ArchiveError("export_receipt path and SHA-256 are required")
    try:
        export_validation = validate_export_bundle(Path(export_path))
    except PolicyExportEvidenceError as exc:
        raise ArchiveError(str(exc)) from exc
    if export_validation["receipt"]["sha256"] != export_sha256:
        raise ArchiveError("export receipt SHA-256 mismatch")
    export_document = export_validation["document"]
    selection_document = export_validation["selection_document"]
    for field in ("task", "algorithm", "runner"):
        if manifest[field] != selection_document[field]:
            raise ArchiveError(f"manifest {field} differs from export receipt")
    selected_checkpoint = selection_document["checkpoint"]
    if any(
        checkpoint.get(field) != selected_checkpoint.get(field)
        for field in ("path", "sha256", "iteration")
    ):
        raise ArchiveError("selected checkpoint differs from export receipt")
    output_values = export_document["outputs"]
    for kind in ("jit", "onnx"):
        expected_artifact = {
            "path": output_values[kind]["path"],
            "sha256": output_values[kind]["sha256"],
        }
        if artifacts.get(kind) != expected_artifact:
            raise ArchiveError(f"{kind} artifact differs from export receipt")
    evaluation = manifest.get("evaluation")
    if not isinstance(evaluation, dict) or not isinstance(evaluation.get("results"), list):
        raise ArchiveError("evaluation.results must be an array of path/SHA-256 references")
    expected_evaluations = [
        {"path": item["path"], "sha256": item["sha256"]}
        for item in selection_document["evaluation_results"]
    ]
    if evaluation["results"] != expected_evaluations:
        raise ArchiveError("evaluation results differ from checkpoint selection")
    identity_source = selection_document["run_identity"]["document"]["source"]
    if source != {
        "commit": identity_source["head"],
        "dirty": identity_source["dirty"],
    }:
        raise ArchiveError("manifest source differs from run identity")
    return export_validation


def _existing_pairs(collection: Path) -> set[tuple[str, str]]:
    pairs: set[tuple[str, str]] = set()
    for jit in collection.rglob("policy.pt"):
        onnx = jit.with_name("policy.onnx")
        if jit.is_symlink() or onnx.is_symlink() or not onnx.is_file():
            continue
        pairs.add((_sha256(jit), _sha256(onnx)))
    return pairs


def _validate_replacement(
    value: Any,
    *,
    destination: Path,
) -> dict[str, Any] | None:
    if value is None:
        if destination.exists() or destination.is_symlink():
            raise ArchiveError(f"archive destination already exists: {destination}")
        return None
    expected_keys = {"authorized", "path", "files"}
    if not isinstance(value, dict) or set(value) != expected_keys:
        raise ArchiveError("replace_existing contract is invalid")
    if value["authorized"] is not True or value["path"] != str(destination):
        raise ArchiveError("replace_existing authorization/path mismatch")
    if destination.is_symlink() or not destination.is_dir():
        raise ArchiveError("replace_existing target must be an existing regular directory")
    expected_names = {
        "policy.pt",
        "policy.onnx",
        "策略说明.txt",
        "archive_manifest.json",
    }
    files = value["files"]
    if not isinstance(files, dict) or set(files) != expected_names:
        raise ArchiveError("replace_existing must bind exactly four archive files")
    actual_names = {item.name for item in destination.iterdir()}
    if actual_names != expected_names:
        raise ArchiveError("replace_existing target contents differ from authorization")
    for name in sorted(expected_names):
        expected_hash = files[name]
        path = destination / name
        if (
            not isinstance(expected_hash, str)
            or not SHA256_RE.fullmatch(expected_hash)
            or path.is_symlink()
            or not path.is_file()
            or _sha256(path) != expected_hash
        ):
            raise ArchiveError(f"replace_existing hash/type mismatch: {name}")
    return value


def _exchange_directories(left: Path, right: Path) -> None:
    """Atomically exchange two directories on Linux without a missing-target gap."""
    renameat2 = getattr(ctypes.CDLL(None, use_errno=True), "renameat2", None)
    if renameat2 is None:
        raise ArchiveError("atomic directory exchange is unavailable on this platform")
    renameat2.argtypes = [
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_uint,
    ]
    renameat2.restype = ctypes.c_int
    at_fdcwd = -100
    rename_exchange = 2
    result = renameat2(
        at_fdcwd,
        os.fsencode(left),
        at_fdcwd,
        os.fsencode(right),
        rename_exchange,
    )
    if result != 0:
        error_number = ctypes.get_errno()
        raise ArchiveError(
            "atomic directory exchange failed: " + os.strerror(error_number)
        )


def _description(manifest: dict[str, Any], destination_name: str) -> str:
    checkpoint = manifest["selected_checkpoint"]
    artifacts = manifest["artifacts"]
    result_paths = [item["path"] for item in manifest["evaluation"]["results"]]
    lines = [
        "策略说明",
        "",
        f"归档目录: {destination_name}",
        f"任务: {manifest['task']}",
        f"算法: {manifest['algorithm']}",
        f"Runner: {manifest['runner']}",
        f"训练 checkpoint: {checkpoint['path']}",
        f"训练轮次: {checkpoint['iteration']}",
        f"Checkpoint SHA-256: {checkpoint['sha256']}",
        f"训练源码 commit: {manifest['source']['commit']}",
        f"训练源码 dirty: {str(manifest['source']['dirty']).lower()}",
        f"JIT SHA-256: {artifacts['jit']['sha256']}",
        f"ONNX SHA-256: {artifacts['onnx']['sha256']}",
        "",
        "训练参数:",
        json.dumps(manifest["parameters"], indent=2, sort_keys=True, ensure_ascii=False),
        "",
        "评估证据:",
        *[f"- {path}" for path in result_paths],
        "",
        "人工说明:",
        manifest["description_notes"],
        "",
        HARDWARE_NOTE,
        "",
    ]
    return "\n".join(lines)


def archive_policy(manifest: dict[str, Any], *, timestamp: str | None = None) -> dict[str, Any]:
    export_validation = _validate_manifest(manifest)
    storage_root = Path(manifest.get("storage_root", ""))
    collection_value = manifest.get("collection")
    if not storage_root.is_absolute() or not storage_root.is_dir() or storage_root.is_symlink():
        raise ArchiveError("storage_root must be an absolute non-symlink directory")
    if not isinstance(collection_value, str) or not collection_value or Path(collection_value).is_absolute():
        raise ArchiveError("collection must be a non-empty relative path")
    resolved_root = storage_root.resolve()
    git_root = Path(_git(resolved_root, "rev-parse", "--show-toplevel")).resolve()
    if git_root != resolved_root:
        raise ArchiveError("storage_root must be the exact Git top level")
    if _git(resolved_root, "status", "--porcelain"):
        raise ArchiveError("policy_storage worktree must be clean")
    collection_input = resolved_root / collection_value
    if collection_input.is_symlink() or not collection_input.is_dir():
        raise ArchiveError("collection must be an existing non-symlink directory inside storage_root")
    collection = collection_input.resolve()
    if resolved_root not in collection.parents:
        raise ArchiveError("collection must be an existing non-symlink directory inside storage_root")
    jit_path, jit_hash = _validated_artifact(manifest.get("artifacts", {}).get("jit"), "jit")
    onnx_path, onnx_hash = _validated_artifact(manifest.get("artifacts", {}).get("onnx"), "onnx")
    if (jit_hash, onnx_hash) in _existing_pairs(collection):
        raise ArchiveError("the JIT/ONNX pair already exists in this collection")
    destination_name = timestamp or datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
    if not re.fullmatch(r"\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}", destination_name):
        raise ArchiveError("timestamp must use YYYY-MM-DD-HH-MM-SS")
    destination = collection / destination_name
    replacement = _validate_replacement(
        manifest.get("replace_existing"),
        destination=destination,
    )
    temporary = Path(tempfile.mkdtemp(prefix=".advisor-archive-", dir=collection))
    try:
        os.chmod(temporary, 0o700)
        copied_jit = temporary / "policy.pt"
        copied_onnx = temporary / "policy.onnx"
        shutil.copy2(jit_path, copied_jit)
        shutil.copy2(onnx_path, copied_onnx)
        if _sha256(copied_jit) != jit_hash or _sha256(copied_onnx) != onnx_hash:
            raise ArchiveError("copied artifact verification failed")
        archive_manifest = dict(manifest)
        archive_manifest["validated_export"] = export_validation["document"]
        archive_manifest["hardware_ready"] = False
        archive_manifest["archive_path"] = str(destination)
        archive_manifest["hardware_boundary"] = HARDWARE_NOTE
        (temporary / "策略说明.txt").write_text(
            _description(manifest, destination_name), encoding="utf-8"
        )
        (temporary / "archive_manifest.json").write_text(
            json.dumps(archive_manifest, indent=2, sort_keys=True, ensure_ascii=False, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        if replacement is None:
            temporary.replace(destination)
        else:
            _exchange_directories(temporary, destination)
            shutil.rmtree(temporary)
    finally:
        if temporary.exists():
            shutil.rmtree(temporary)
    return {
        "version": 2,
        "archive_path": str(destination),
        "files": {
            "jit": {"path": str(destination / "policy.pt"), "sha256": jit_hash},
            "onnx": {"path": str(destination / "policy.onnx"), "sha256": onnx_hash},
            "description": str(destination / "策略说明.txt"),
            "manifest": str(destination / "archive_manifest.json"),
        },
        "git_action": "none",
        "replaced_existing": replacement is not None,
        "hardware_ready": False,
        "hardware_boundary": HARDWARE_NOTE,
        "export_receipt": manifest["export_receipt"],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest")
    parser.add_argument("--timestamp")
    parser.add_argument("--output")
    args = parser.parse_args()
    try:
        receipt = archive_policy(_load(Path(args.manifest)), timestamp=args.timestamp)
    except ArchiveError as exc:
        parser.error(str(exc))
    encoded = json.dumps(receipt, indent=2, sort_keys=True, ensure_ascii=False) + "\n"
    if args.output:
        output = Path(args.output)
        if not output.is_absolute() or output.exists():
            parser.error("--output must be a new absolute path")
        output.write_text(encoded, encoding="utf-8")
    else:
        print(encoded, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
