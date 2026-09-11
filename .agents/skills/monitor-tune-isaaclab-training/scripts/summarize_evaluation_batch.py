#!/usr/bin/env python3
"""Seal one evaluation batch and rebuild its run's navigation index."""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

from evidence_provenance import digest, encode, identifier, read_reference, safe_path, write_bytes

REPO_ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO_ROOT / "scripts/reinforcement_learning/rsl_rl"))
from policy_evaluation_evidence import validate_evaluation_bundle


def file_ref(path: Path) -> dict:
    safe_path(path)
    return {"path": str(path), "sha256": digest(path.read_bytes())}


def link(path: str | Path, directory: Path, label: str) -> str:
    return f"[{label}](<{os.path.relpath(path, directory)}>)"


def validate_batch(path: Path, *, expected_sha256: str | None = None) -> dict:
    reference = file_ref(path)
    if expected_sha256 is not None and reference["sha256"] != expected_sha256:
        raise ValueError("batch manifest SHA-256 mismatch")
    manifest = read_reference(reference)
    return _validate_manifest(manifest, path)


def _validate_manifest(manifest: dict, path: Path) -> dict:
    if manifest.get("version") != 1 or manifest.get("layout_version") != 2 or manifest.get("status") != "sealed":
        raise ValueError("batch must be a sealed version-1 manifest with layout_version 2")
    for field in ("task", "run_id", "batch_id"):
        identifier(manifest[field])
    root = Path(manifest["repository_root"]) / "learnings/policy_tuning" / manifest["task"] / manifest["run_id"]
    if path != root / "evaluations" / manifest["batch_id"] / "manifest.json":
        raise ValueError("batch manifest layout mismatch")
    report = manifest["report"]
    if Path(report["path"]) != path.parent / "report.md" or file_ref(Path(report["path"])) != report:
        raise ValueError("batch report SHA-256 or path mismatch")
    if not isinstance(manifest.get("cases"), list) or not manifest["cases"]:
        raise ValueError("batch cases missing")
    if manifest.get("origin") not in {"adopted", "executed"}:
        raise ValueError("invalid batch origin")
    planned = None
    if manifest["origin"] == "executed":
        from run_evaluation_batch import validate_contract
        contract_ref = manifest["contract"]
        if Path(contract_ref["path"]) != path.parent / "contract.json":
            raise ValueError("batch contract path mismatch")
        contract = read_reference(contract_ref)
        identity = validate_contract(contract)
        if any(manifest[f] != identity[f] for f in ("task", "run_id")) or contract["batch_id"] != manifest["batch_id"]:
            raise ValueError("batch contract scope mismatch")
        planned = {case["attempt_id"]: case for case in contract["cases"]}
        if [case["attempt_id"] for case in manifest["cases"]] != list(planned):
            raise ValueError("batch must report every supplied case in order")
    seen = set()
    for case in manifest["cases"]:
        identifier(case["attempt_id"])
        if case["attempt_id"] in seen:
            raise ValueError("duplicate batch attempt ID")
        seen.add(case["attempt_id"])
        if case["status"] not in {"completed", "failed", "not_run"}:
            raise ValueError("invalid batch case status")
        if planned is not None and case["scenario"] != planned[case["attempt_id"]]["scenario"]:
            raise ValueError("planned scenario mismatch")
        if case["status"] == "completed":
            result_ref = case["result"]
            validated = validate_evaluation_bundle(Path(result_ref["path"]))
            if validated["result"] != result_ref:
                raise ValueError("batch result SHA-256 mismatch")
            result = read_reference(result_ref)
            if any(result["evaluation"][f] != manifest[f] for f in ("task", "run_id")):
                raise ValueError("batch case run mismatch")
            if result["evaluation"]["evaluation_id"] != case["attempt_id"]:
                raise ValueError("batch attempt mismatch")
            if result["inputs"]["scenario"]["contract"] != case["scenario"]:
                raise ValueError("batch scenario mismatch")
            if planned is not None:
                if result["evaluation"].get("batch_id") != manifest["batch_id"]:
                    raise ValueError("batch ID mismatch")
                inputs = result["inputs"]
                if inputs["checkpoint"] != contract["checkpoint"] or inputs["effective_config"] != contract["effective_config"]:
                    raise ValueError("batch checkpoint/config binding mismatch")
                identity_ref = inputs["run_identity"]
                if {"path": identity_ref["path"], "sha256": identity_ref["file_sha256"]} != contract["run_identity"]:
                    raise ValueError("batch training context mismatch")
        elif case.get("result") is not None or not case.get("reason"):
            raise ValueError("unfinished case requires a reason and no completed result")
        log = case.get("console")
        if log is not None:
            if planned is not None and Path(log["path"]) != path.parent / "raw" / case["attempt_id"] / "console.log":
                raise ValueError("console log outside its attempt")
            if file_ref(Path(log["path"])) != log:
                raise ValueError("batch console SHA-256 mismatch")
        elif manifest["origin"] == "executed" and case["status"] != "not_run":
            raise ValueError("executed attempt requires a console log")
    for item in manifest.get("supporting_evidence", []):
        if file_ref(Path(item["path"])) != item:
            raise ValueError("supporting evidence SHA-256 mismatch")
    return manifest


def render_report(manifest: dict, directory: Path) -> str:
    lines = [f"# 评估批次 {manifest['batch_id']}", "", f"任务：`{manifest['task']}`；训练：`{manifest['run_id']}`。", "",
             "本报告汇总已校验的用例；completed 表示证据发布完成，策略表现须结合指标和视频判断。", "",
             "| 用例 / 尝试 | 状态 | 场景、步数 / 环境 / seed | 遥测 | 结果与视频 |",
             "| --- | --- | --- | --- | --- |"]
    policies = set()
    provenance = set()
    details = []
    for case in manifest["cases"]:
        contract = case["scenario"]
        outputs, telemetry = "—", "—"
        if case["status"] == "completed":
            result = read_reference(case["result"])
            policies.add((result["inputs"]["checkpoint"]["sha256"], result["evaluation"]["runner"]))
            provenance.add(("训练上下文", result["inputs"]["run_identity"]["path"]))
            for field, label in (("effective_config", "训练有效配置"), ("scenario_evidence", "场景来源")):
                if field in result["inputs"]:
                    provenance.add((label, result["inputs"][field]["path"]))
            telemetry = result.get("telemetry_status", "unknown")
            outputs = link(case["result"]["path"], directory, "结果")
            if result["outputs"]["video"]:
                outputs += " / " + link(result["outputs"]["video"]["path"], directory, "视频")
            metrics = {k: result.get("metrics", {}).get(k, "unavailable") for k in ("tracking_xy_rmse", "tracking_yaw_rmse", "termination_rate", "max_joint_velocity_utilization", "max_tilt")}
            details += [f"- `{case['attempt_id']}` 指标：`{json.dumps(metrics, ensure_ascii=False, sort_keys=True)}`"]
        if case.get("console"):
            outputs += " / " + link(case["console"]["path"], directory, "日志")
        lines.append(f"| {case['attempt_id']} | {case['status']} | {contract['scenario_id']}，{contract['duration_steps']} / {contract['num_envs']} / {contract['seed']} | {telemetry} | {outputs} |")
        details += [f"- `{case['attempt_id']}` 命令调度：`{json.dumps(contract['command_schedule'], ensure_ascii=False)}`；训练配置覆盖：`{json.dumps(contract['scenario_overrides'], ensure_ascii=False, sort_keys=True)}`。"]
        if case.get("reason"):
            details += [f"- `{case['attempt_id']}` 未完成原因：{case['reason']}"]
    lines += ["", "## 策略与场景", ""]
    lines += [f"- checkpoint SHA-256：`{sha}`；runner：`{runner}`。" for sha, runner in sorted(policies)]
    lines += [f"- {link(path, directory, label + ' ' + Path(path).name)}" for label, path in sorted(provenance)]
    lines += ["", *details, "", "## 观察与限制", ""]
    lines += [f"- {note}" for note in manifest.get("notes", [])]
    lines += ["- 未提供用户批准的收敛判据时，不作收敛判断；缺失遥测不能按零值解释。", "- 视频需经机器人持续在画面内的人工检查后才可作为动作证据。", "",
              "## 相关证据", ""]
    lines += [f"- {link(item['path'], directory, Path(item['path']).name)}" for item in manifest.get("supporting_evidence", [])]
    lines += ["", "## 建议与待授权事项", "", *[f"- {note}" for note in manifest.get("recommendations", [])], ""]
    return "\n".join(lines)


def rebuild_index(run: Path) -> Path:
    safe_path(run)
    lines = [f"# {run.name}", "", "此页可重新生成，仅用于导航；证据以各批次 manifest 及其校验链为准。", "", "## 评估批次", ""]
    for path in sorted((run / "evaluations").glob("*/manifest.json")):
        batch = validate_batch(path)
        counts = {s: sum(c["status"] == s for c in batch["cases"]) for s in ("completed", "failed", "not_run")}
        lines.append(f"- {link(path.parent / 'report.md', run, batch['batch_id'])}：{counts['completed']} 完成 / {counts['failed']} 失败 / {counts['not_run']} 未运行；{link(path, run, 'manifest')}。")
    lines += ["", "## 原始记录与生命周期", ""]
    for path in sorted((run / "evidence/training").glob("*.md")):
        lines.append(f"- {link(path, run, path.name)}")
    for directory in ("provenance", "training", "events", "evidence/checkpoint_selection", "evidence/export", "evidence/source"):
        if (run / directory).is_dir():
            lines.append(f"- {link(run / directory, run, directory)}")
    lines += ["", "旧版证据保持原位；本索引不会移动、压缩或重写已有结果。", ""]
    index = run / "index.md"
    write_bytes(index, "\n".join(lines).encode(), replace_index=True)
    return index


def seal_batch(run: Path, batch_id: str, cases: list[dict], *, origin: str = "adopted", notes: list[str] | None = None,
               supporting: list[Path] | None = None, recommendations: list[str] | None = None, contract: dict | None = None) -> dict:
    identifier(batch_id)
    safe_path(run)
    if origin not in {"adopted", "executed"}:
        raise ValueError("invalid batch origin")
    directory = run / "evaluations" / batch_id
    safe_path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    # A batch-level exclusive claim protects report + manifest as a pair.
    claim = directory / ".summary-claim"
    with claim.open("x"):
        pass
    try:
        if (directory / "report.md").exists() or (directory / "manifest.json").exists():
            raise ValueError("batch already sealed or partially published; use a new batch ID")
        manifest = {"version": 1, "layout_version": 2, "status": "sealed", "origin": origin,
                    "repository_root": str(run.parents[3]), "task": run.parent.name, "run_id": run.name,
                    "batch_id": batch_id, "contract": contract, "cases": cases, "notes": notes or [],
                    "recommendations": recommendations or [], "supporting_evidence": [file_ref(p) for p in supporting or []]}
        # Validate every completed bundle before creating human-facing claims.
        for case in cases:
            if case["status"] == "completed":
                validation = validate_evaluation_bundle(Path(case["result"]["path"]))
                if validation["result"] != case["result"]:
                    raise ValueError("batch result SHA-256 mismatch")
        report = write_bytes(directory / "report.md", render_report(manifest, directory).encode())
        manifest["report"] = report
        _validate_manifest(manifest, directory / "manifest.json")
        reference = write_bytes(directory / "manifest.json", encode(manifest))
        validate_batch(Path(reference["path"]))
        rebuild_index(run)
        return reference
    finally:
        claim.unlink()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-root", required=True, type=Path)
    parser.add_argument("--batch-id")
    parser.add_argument("--result", action="append", default=[], type=Path)
    parser.add_argument("--supporting-evidence", action="append", default=[], type=Path)
    parser.add_argument("--note", action="append", default=[])
    parser.add_argument("--recommendation", action="append", default=[])
    parser.add_argument("--index-only", action="store_true")
    args = parser.parse_args()
    try:
        if args.index_only:
            print(rebuild_index(args.run_root))
        else:
            if not args.batch_id or not args.result:
                parser.error("--batch-id and at least one --result are required")
            cases = []
            for path in args.result:
                validate_evaluation_bundle(path)
                result = json.loads(path.read_bytes())
                cases.append({"attempt_id": result["evaluation"]["evaluation_id"], "status": "completed",
                              "scenario": result["inputs"]["scenario"]["contract"], "result": file_ref(path), "console": None})
            print(json.dumps(seal_batch(args.run_root, args.batch_id, cases, notes=args.note,
                                       supporting=args.supporting_evidence, recommendations=args.recommendation)))
    except (ValueError, OSError, KeyError) as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
