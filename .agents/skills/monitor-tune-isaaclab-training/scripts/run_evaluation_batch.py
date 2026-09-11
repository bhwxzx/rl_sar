#!/usr/bin/env python3
"""Execute a finite, explicitly supplied evaluation batch sequentially.

This runner never generates cases, selects checkpoints, edits parameters or
controls training. Every attempt retains one console log, including failures.
"""
from __future__ import annotations

import argparse
import datetime
import json
import os
import signal
import subprocess
import sys
from pathlib import Path

from capture_run_identity import validate_run_identity
from evidence_provenance import (capture_evaluator_source, capture_scenario, encode, identifier,
                                 read_reference, require_source_path, run_root, safe_path, validate_provenance, write_bytes)
from prepare_evidence_layout import prepare_batch_layout
from summarize_evaluation_batch import REPO_ROOT, file_ref, seal_batch
from policy_evaluation_evidence import validate_evaluation_bundle, validate_scenario_contract


def validate_contract(contract: dict) -> dict:
    expected = {"version", "batch_id", "run_identity", "effective_config", "checkpoint", "cases"}
    if not isinstance(contract, dict) or set(contract) != expected or contract["version"] != 1:
        raise ValueError("batch contract must contain exactly version, batch_id, run_identity, effective_config, checkpoint, cases")
    identifier(contract["batch_id"])
    identity = read_reference(contract["run_identity"])
    validate_run_identity(identity)
    if identity["version"] != 2:
        raise ValueError("new batch requires reusable training context v2")
    require_source_path(Path(contract["run_identity"]["path"]), run_root(identity), "context")
    require_source_path(Path(contract["effective_config"]["path"]), run_root(identity), "config")
    from capture_effective_training_config import load_and_validate_effective_config
    load_and_validate_effective_config(Path(contract["effective_config"]["path"]), expected_sha256=contract["effective_config"]["sha256"], run_identity=identity)
    checkpoint = contract["checkpoint"]
    if file_ref(Path(checkpoint["path"])) != checkpoint:
        raise ValueError("checkpoint SHA-256 mismatch")
    if not Path(checkpoint["path"]).stem.startswith("model_"):
        raise ValueError("batch checkpoint must be model_<iteration>.pt")
    cases = contract["cases"]
    if not isinstance(cases, list) or not 1 <= len(cases) <= 32:
        raise ValueError("batch requires 1..32 explicitly supplied cases")
    seen = set()
    for case in cases:
        if not isinstance(case, dict) or set(case) != {"attempt_id", "scenario", "video", "timeout_seconds"}:
            raise ValueError("each case requires attempt_id, scenario, video, timeout_seconds")
        identifier(case["attempt_id"])
        if case["attempt_id"] in seen:
            raise ValueError("duplicate attempt ID")
        seen.add(case["attempt_id"])
        validate_scenario_contract(case["scenario"])
        if case["scenario"]["seed"] != identity["seed"]:
            raise ValueError("scenario seed mismatch")
        if case["scenario"]["num_envs"] > 64 or case["scenario"]["duration_steps"] > 100000:
            raise ValueError("case exceeds bounded evaluation limits")
        if not isinstance(case["video"], bool) or type(case["timeout_seconds"]) is not int or not 1 <= case["timeout_seconds"] <= 3600:
            raise ValueError("invalid video or timeout setting")
    return identity


def evaluation_argv(contract: dict, identity: dict, case: dict, paths: dict, scenario_ref: dict) -> list[str]:
    scenario = case["scenario"]
    checkpoint = contract["checkpoint"]
    command = [sys.executable, str(REPO_ROOT / "scripts/reinforcement_learning/rsl_rl/evaluate_policy.py"),
               "--task", identity["task"], "--run_id", identity["run_id"], "--batch_id", contract["batch_id"],
               "--evaluation_id", case["attempt_id"], "--candidate_id", Path(checkpoint["path"]).stem,
               "--checkpoint", checkpoint["path"], "--checkpoint_sha256", checkpoint["sha256"],
               "--artifact_kind", "native", "--artifact_path", checkpoint["path"], "--artifact_sha256", checkpoint["sha256"],
               "--run_identity_path", contract["run_identity"]["path"], "--run_identity_file_sha256", contract["run_identity"]["sha256"],
               "--effective_config_reference_json", json.dumps(contract["effective_config"]),
               "--scenario_evidence_reference_json", json.dumps(scenario_ref),
               "--scenario_id", scenario["scenario_id"], "--scenario_overrides_json", json.dumps(scenario["scenario_overrides"]),
               "--command_schedule_json", json.dumps(scenario["command_schedule"]),
               "--duration_steps", str(scenario["duration_steps"]), "--num_envs", str(scenario["num_envs"]),
               "--seed", str(scenario["seed"]), "--result_path", paths["play_result"],
               "--telemetry_path", paths["telemetry"], "--telemetry_stride", "1", "--headless", "--device", "cuda:0", "--require_idle_gpu"]
    command += ["--video_path", paths["video"], "--follow_robot_camera"] if case["video"] else ["--no_video"]
    return command


def run_process(argv: list[str], console: Path, timeout: int) -> tuple[int, str | None]:
    with console.open("x", encoding="utf-8") as stream:
        stream.write(json.dumps({"argv": argv}, ensure_ascii=False) + "\n")
        stream.flush()
        process = subprocess.Popen(argv, cwd=REPO_ROOT, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            return process.wait(timeout=timeout), None
        except (subprocess.TimeoutExpired, KeyboardInterrupt) as exc:
            # The new session contains only this evaluation, never training.
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            return process.returncode, type(exc).__name__


def record_batch_event(identity: dict, config_ref: dict, batch_ref: dict, batch_id: str) -> dict:
    from record_tuning_experience import write_event
    config = read_reference(config_ref)
    event = {
        "version": 5, "event_id": f"batch-{batch_id}", "event_type": "evaluation_batch",
        "task": identity["task"], "run_id": identity["run_id"], "algorithm": identity["algorithm"],
        "recorded_at": datetime.datetime.now(datetime.timezone.utc).isoformat(), "run_identity": identity,
        "context": {"observation_fingerprint": "unknown", "deployment_fingerprint": "unknown",
                    "reward_fingerprint": config["fingerprints"]["reward"]},
        "parameters": {}, "evidence": {
            "effective_config": {**config_ref, "effective_config_fingerprint": config["fingerprints"]["effective_config"], "reward_fingerprint": config["fingerprints"]["reward"]},
            "event": {"status": "available", **batch_ref},
            "outcome": {"status": "unavailable", "reason": "No approved baseline outcome comparison; this event records batch evidence only."}},
        "analysis": {"confidence": "low", "direct_parameter_change_supported": False},
        "next_suggestion": "Review the batch report and required telemetry before proposing further tests.",
    }
    return write_event(run_root(identity).parents[1], event)


def run_batch(contract: dict, *, execute=run_process) -> dict:
    identity = validate_contract(contract)
    if Path(identity["source"]["repository_root"]) != REPO_ROOT:
        raise ValueError("batch runner must execute in the context repository")
    if os.environ.get("CONDA_DEFAULT_ENV") != "isaacsim-5.1":
        raise ValueError("run with conda run -n isaacsim-5.1")
    root = run_root(identity)
    directory = root / "evaluations" / contract["batch_id"]
    safe_path(directory)
    directory.mkdir(parents=True, exist_ok=False)  # persistent reservation, including failed attempts
    source_files = [REPO_ROOT / "scripts/reinforcement_learning/rsl_rl" / name for name in
                    ("evaluate_policy.py", "policy_evaluation_evidence.py", "policy_evaluation_telemetry.py", "cli_args.py")]
    source_files += [Path(__file__).parent / name for name in
                     ("evidence_provenance.py", "capture_run_identity.py", "capture_effective_training_config.py", "run_evaluation_batch.py")]
    source = capture_evaluator_source(root, REPO_ROOT, source_files)
    # Persist exact finite inputs before starting any process.
    contract_ref = write_bytes(directory / "contract.json", encode(contract))
    results = []
    interrupted = False
    for case in contract["cases"]:
        item = {"attempt_id": case["attempt_id"], "scenario": case["scenario"], "result": None, "console": None}
        if interrupted:
            item.update(status="not_run", reason="Batch interrupted after an earlier attempt")
            results.append(item)
            continue
        layout = prepare_batch_layout(root.parents[1], task=identity["task"], run_id=identity["run_id"], batch_id=contract["batch_id"], evaluation_id=case["attempt_id"])
        console = Path(layout["paths"]["play_result"]).parent / "console.log"
        try:
            scenario_ref = capture_scenario(identity, case["scenario"], source)
            validate_provenance(identity, contract["effective_config"], scenario_ref, case["scenario"], live=True)
            code, reason = execute(evaluation_argv(contract, identity, case, layout["paths"], scenario_ref), console, case["timeout_seconds"])
            interrupted = reason == "KeyboardInterrupt"
            if code != 0 or reason:
                raise ValueError(f"evaluation exit={code}; {reason or 'see console log'}")
            validated = validate_evaluation_bundle(Path(layout["paths"]["play_result"]))
            item.update(status="completed", result=validated["result"])
        except (ValueError, OSError) as exc:
            if not console.exists():
                write_bytes(console, (str(exc) + "\n").encode())
            item.update(status="failed", reason=str(exc))
        item["console"] = file_ref(console)
        results.append(item)
    batch = seal_batch(root, contract["batch_id"], results, origin="executed", contract=contract_ref,
                       recommendations=["按批次报告检查失败项和动作表现；复测使用新的 batch_id 与 attempt_id。"])
    event = record_batch_event(identity, contract["effective_config"], batch, contract["batch_id"])
    return {"batch": batch, "event": event, "all_completed": all(case["status"] == "completed" for case in results)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("contract", type=Path)
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args()
    try:
        contract = json.loads(args.contract.read_bytes())
        validate_contract(contract)
        receipt = {"status": "valid", "cases": len(contract["cases"])} if args.validate_only else run_batch(contract)
        print(json.dumps(receipt, ensure_ascii=False))
        return 0 if receipt.get("all_completed", True) else 1
    except (ValueError, OSError, KeyError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    raise SystemExit(main())
