#!/usr/bin/env python3
"""Collect and analyze review-only LW runtime configuration profiles."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
from typing import Any, Iterable, NoReturn


HARDWARE_CONFIRMATION = (
    "I_CONFIRM_LW_IS_SUSPENDED_AND_MOTORS_MUST_REMAIN_DISABLED"
)
POLICIES = (
    "LW/robot_lab/leg_loco",
    "LW/robot_lab/wheel_loco",
    "LW/robot_lab/leg_to_wheel",
    "LW/robot_lab/wheel_to_leg",
)
POLICY_ASSETS = (
    "LW/base.yaml",
    "LW/robot_lab/leg_loco/config.yaml",
    "LW/robot_lab/leg_loco/policy.onnx",
    "LW/robot_lab/leg_to_wheel/config.yaml",
    "LW/robot_lab/leg_to_wheel/leg_to_wheel_transform_60hz.csv",
    "LW/robot_lab/leg_to_wheel/policy.onnx",
    "LW/robot_lab/wheel_loco/config.yaml",
    "LW/robot_lab/wheel_loco/policy.onnx",
    "LW/robot_lab/wheel_to_leg/config.yaml",
    "LW/robot_lab/wheel_to_leg/policy.onnx",
    "LW/robot_lab/wheel_to_leg/wheel_to_leg_transform_60hz.csv",
)


def fail(message: str) -> NoReturn:
    raise RuntimeError(message)


def file_digest(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_integer_list(value: str, name: str) -> list[int]:
    try:
        result = [int(item) for item in value.split(",") if item != ""]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"{name} must be comma-separated integers") from exc
    if not result:
        raise argparse.ArgumentTypeError(f"{name} must not be empty")
    return result


def prepare_output_directory(path: Path) -> None:
    if path.exists():
        if not path.is_dir():
            fail(f"output directory path is not a directory: {path}")
        if any(path.iterdir()):
            fail(f"refusing to use nonempty output directory: {path}")
    path.mkdir(parents=True, exist_ok=True)


def run_profiler(command: list[str]) -> None:
    completed = subprocess.run(command, check=False)
    if completed.returncode != 0:
        fail(
            f"profiler exited with status {completed.returncode}: "
            + " ".join(command)
        )


def collect_host(args: argparse.Namespace) -> None:
    output_dir = args.output_dir.resolve()
    prepare_output_directory(output_dir)
    allowed = set(os.sched_getaffinity(0))
    cpus = sorted(allowed) if args.cpus == "allowed" else parse_integer_list(
        args.cpus, "cpus"
    )
    priorities = parse_integer_list(args.realtime_priorities, "realtime-priorities")
    invalid = sorted(set(cpus) - allowed - {-1})
    if invalid:
        fail(f"requested CPUs are outside this process affinity mask: {invalid}")
    if any(priority < 0 for priority in priorities):
        fail("real-time priorities must be nonnegative")

    for cpu in cpus:
        for priority in priorities:
            report = output_dir / f"host-cpu{cpu}-rt{priority}.json"
            command = [
                str(args.profiler.resolve()),
                "--mode",
                "host-only",
                "--policy-root",
                str(args.policy_root.resolve()),
                "--output",
                str(report),
                "--duration-seconds",
                str(args.duration_seconds),
                "--cpu",
                str(cpu),
                "--realtime-priority",
                str(priority),
            ]
            if priority > 0 and args.require_realtime:
                command.append("--require-realtime")
            run_profiler(command)


def collect_hardware(args: argparse.Namespace) -> None:
    if args.confirmation != HARDWARE_CONFIRMATION:
        fail(
            "hardware collection requires --confirmation "
            + HARDWARE_CONFIRMATION
        )
    output = args.output.resolve()
    if output.exists():
        fail(f"refusing to overwrite existing report: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    command = [
        str(args.profiler.resolve()),
        "--mode",
        "hardware-observe",
        "--policy-root",
        str(args.policy_root.resolve()),
        "--output",
        str(output),
        "--duration-seconds",
        str(args.duration_seconds),
        "--cpu",
        str(args.cpu),
        "--realtime-priority",
        str(args.realtime_priority),
        "--right-port",
        args.right_port,
        "--left-port",
        args.left_port,
        "--imu-topic",
        args.imu_topic,
        "--hardware-confirmation",
        HARDWARE_CONFIRMATION,
    ]
    if args.require_realtime:
        command.append("--require-realtime")
    run_profiler(command)


def require_mapping(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        fail(f"{name} must be a mapping")
    return value


def require_string(mapping: dict[str, Any], key: str, context: str) -> str:
    value = mapping.get(key)
    if not isinstance(value, str) or not value:
        fail(f"{context}.{key} must be a nonempty string")
    return value


def require_number(mapping: dict[str, Any], key: str, context: str) -> float:
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        fail(f"{context}.{key} must be numeric")
    result = float(value)
    if not math.isfinite(result) or result < 0.0:
        fail(f"{context}.{key} must be finite and nonnegative")
    return result


def require_integer(mapping: dict[str, Any], key: str, context: str) -> int:
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        fail(f"{context}.{key} must be a nonnegative integer")
    return value


def require_positive_number(
    mapping: dict[str, Any], key: str, context: str
) -> float:
    result = require_number(mapping, key, context)
    if result <= 0.0:
        fail(f"{context}.{key} must be positive")
    return result


def load_policy_assets(report: dict[str, Any], path: Path) -> list[dict[str, str]]:
    values = report.get("policy_assets")
    if not isinstance(values, list):
        fail(f"profile policy_assets must be a list: {path}")
    records: list[dict[str, str]] = []
    for index, expected_path in enumerate(POLICY_ASSETS):
        if index >= len(values):
            fail(f"profile is missing policy asset {expected_path}: {path}")
        value = require_mapping(values[index], f"{path}.policy_assets[{index}]")
        actual_path = require_string(value, "path", "policy_asset")
        digest = require_string(value, "sha256", "policy_asset")
        if actual_path != expected_path:
            fail(f"profile policy assets are not in the approved order: {path}")
        if re.fullmatch(r"[0-9a-f]{64}", digest) is None:
            fail(f"profile policy asset SHA-256 is invalid: {path}")
        records.append({"path": actual_path, "sha256": digest})
    if len(values) != len(POLICY_ASSETS):
        fail(f"profile contains extra or duplicate policy assets: {path}")
    return records


def load_report(path: Path) -> dict[str, Any]:
    if (
        path.is_symlink()
        or not path.is_file()
        or path.resolve(strict=True) != path
    ):
        fail(f"profile report must be a regular non-symlink file: {path}")
    before_digest = file_digest(path)
    with path.open("r", encoding="utf-8") as source:
        report = json.load(source)
    if file_digest(path) != before_digest:
        fail(f"profile changed while it was being read: {path}")
    report = require_mapping(report, str(path))
    if type(report.get("schema_version")) is not int or report["schema_version"] != 2:
        fail(f"unsupported profile schema in {path}")
    if report.get("failed") is not False:
        fail(f"profile reports failure and cannot be used: {path}")
    source_commit = require_string(report, "source_commit", str(path))
    if re.fullmatch(r"[0-9a-f]{40}", source_commit) is None:
        fail(f"profile source_commit must be a full lowercase Git SHA-1: {path}")
    mode = report.get("mode")
    if mode not in {"host-only", "hardware-observe"}:
        fail(f"profile mode is not approved: {path}")
    host = require_mapping(report.get("host"), f"{path}.host")
    host_keys = ("node", "system", "release", "machine")
    if set(host) != set(host_keys):
        fail(f"profile host identity fields differ from schema: {path}")
    report["host"] = {
        key: require_string(host, key, f"{path}.host") for key in host_keys
    }
    policy_root_value = require_string(report, "policy_root", str(path))
    policy_root = Path(policy_root_value)
    if (
        not policy_root.is_absolute()
        or str(policy_root.resolve(strict=True)) != policy_root_value
    ):
        fail(f"profile policy_root is not an existing canonical path: {path}")
    report["policy_assets"] = load_policy_assets(report, path)
    duration = require_positive_number(
        report, "duration_per_policy_seconds", str(path)
    )
    policies = report.get("policies")
    if not isinstance(policies, list):
        fail(f"profile policies must be a list: {path}")
    if len(policies) != len(POLICIES):
        fail(f"profile must contain exactly four LW policy records: {path}")
    for index, expected_name in enumerate(POLICIES):
        policy_mapping = require_mapping(
            policies[index], f"{path}.policies[{index}]"
        )
        if policy_mapping.get("policy") != expected_name:
            fail(f"profile policies are not unique and in approved order: {path}")
        policy_duration = require_positive_number(
            policy_mapping, "duration_seconds", f"{path}.policies[{index}]"
        )
        if policy_duration != duration:
            fail(f"profile policy duration differs from report duration: {path}")
    hardware = require_mapping(report.get("hardware"), f"{path}.hardware")
    if mode == "host-only":
        if report.get("hardware_confirmation") is not False:
            fail(f"host-only profile claims hardware confirmation: {path}")
        if hardware.get("commands_sent") != "none":
            fail(f"host-only profile claims hardware output: {path}")
        serial = require_mapping(
            hardware.get("serial_write_duration"),
            f"{path}.hardware.serial_write_duration",
        )
        if (
            hardware.get("initial_disable_writes_complete") is not False
            or hardware.get("disable_keepalive_started") is not False
            or hardware.get("disable_only_output_enforced") is not False
            or require_integer(hardware, "initial_disable_packets", "hardware") != 0
            or require_integer(serial, "count", "serial_write_duration") != 0
            or require_integer(hardware, "serial_write_failures", "hardware") != 0
        ):
            fail(f"host-only profile contains contradictory hardware proof: {path}")
    else:
        if report.get("hardware_confirmation") is not True:
            fail(f"hardware profile is missing suspension confirmation: {path}")
        if hardware.get("commands_sent") != "motors_disable_only":
            fail(f"hardware profile does not claim disable-only output: {path}")
    report["_path"] = str(path)
    report["_sha256"] = before_digest
    return report


def load_base_scalars(path: Path) -> dict[str, Any]:
    """Read only simple scalars directly under the top-level LW mapping."""
    result: dict[str, Any] = {}
    inside_lw = False
    for line_number, raw_line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        content = raw_line.split("#", 1)[0].rstrip()
        if not content:
            continue
        if not raw_line.startswith((" ", "\t")):
            inside_lw = content == "LW:"
            continue
        if not inside_lw or not raw_line.startswith("  ") or raw_line.startswith("    "):
            continue
        if ":" not in content:
            continue
        key, value = content.strip().split(":", 1)
        value = value.strip()
        if not value:
            continue
        if value in {"true", "false"}:
            result[key] = value == "true"
            continue
        try:
            result[key] = int(value) if all(
                character not in value for character in ".eE"
            ) else float(value)
        except ValueError:
            # Lists and strings are intentionally outside this analyzer's scope.
            continue
    required = {
        "dt",
        "sensor_timeout",
        "serial_write_timeout",
        "control_loop_degraded_consecutive_misses",
        "control_loop_degraded_lateness",
    }
    missing = sorted(required - result.keys())
    if missing:
        fail(f"base.yaml is missing required scalar keys: {missing}")
    return result


def host_score(report: dict[str, Any]) -> tuple[float, ...]:
    missed = 0.0
    max_deadline = 0.0
    max_execution = 0.0
    max_inference = 0.0
    requested_cpu: int | None = None
    requested_priority: int | None = None
    for policy_value in report["policies"]:
        policy = require_mapping(policy_value, "policy")
        control = require_mapping(policy.get("control_timing"), "control_timing")
        inference = require_mapping(
            policy.get("inference_duration"), "inference_duration"
        )
        startup = require_mapping(policy.get("control_startup"), "control_startup")
        missed += require_number(control, "missed_deadlines", "control_timing")
        max_deadline = max(
            max_deadline,
            require_number(
                control, "maximum_deadline_lateness_us", "control_timing"
            ),
        )
        max_execution = max(
            max_execution,
            require_number(control, "maximum_execution_us", "control_timing"),
        )
        max_inference = max(
            max_inference,
            require_number(inference, "p999_us", "inference_duration"),
        )
        cpu_value = startup.get("requested_cpu")
        if type(cpu_value) is not int or cpu_value < -1:
            fail("control_startup.requested_cpu must be -1 or nonnegative")
        cpu = cpu_value
        priority = require_integer(
            startup, "requested_realtime_priority", "control_startup"
        )
        if requested_cpu is None:
            requested_cpu = cpu
            requested_priority = priority
        if cpu != requested_cpu or priority != requested_priority:
            fail("profile changed CPU or real-time priority between policies")
        if cpu >= 0 and startup.get("affinity_applied") is not True:
            fail(f"requested CPU affinity was not applied: {report['_path']}")
        if priority > 0 and startup.get("realtime_applied") is not True:
            fail(f"requested SCHED_FIFO was not applied: {report['_path']}")
    assert requested_cpu is not None and requested_priority is not None
    return (
        missed,
        max_deadline,
        max_execution,
        max_inference,
        float(requested_priority),
        float(requested_cpu),
    )


def rounded_up(value: float, quantum: float) -> float:
    return math.ceil(value / quantum) * quantum


def deployment_identity(report: dict[str, Any]) -> dict[str, Any]:
    return {
        "source_commit": report["source_commit"],
        "host": report["host"],
        "policy_root": report["policy_root"],
        "policy_assets": report["policy_assets"],
    }


def validate_current_policy_assets(identity: dict[str, Any]) -> None:
    root = Path(identity["policy_root"])
    for record in identity["policy_assets"]:
        asset = root / record["path"]
        if (
            asset.is_symlink()
            or not asset.is_file()
            or asset.resolve(strict=True) != asset
        ):
            fail(f"policy asset must be a regular non-symlink file: {asset}")
        if file_digest(asset) != record["sha256"]:
            fail(f"policy asset no longer matches the profile: {asset}")


def validate_inputs_unchanged(
    reports: list[dict[str, Any]],
    base_path: Path,
    base_digest: str,
    identity: dict[str, Any],
) -> None:
    if base_path.is_symlink() or not base_path.is_file():
        fail("base.yaml was replaced while generating the review report")
    if file_digest(base_path) != base_digest:
        fail("base.yaml changed while generating the review report")
    for report in reports:
        path = Path(report["_path"])
        if path.is_symlink() or not path.is_file():
            fail(f"profile was replaced while generating the review report: {path}")
        if file_digest(path) != report["_sha256"]:
            fail(
                "profile changed while generating the review report: "
                + report["_path"]
            )
    validate_current_policy_assets(identity)


def analyze(args: argparse.Namespace) -> None:
    base_path = args.base_yaml.resolve()
    if args.base_yaml.is_symlink() or not args.base_yaml.is_file():
        fail(f"base.yaml must be a regular non-symlink file: {args.base_yaml}")
    before_digest = file_digest(base_path)
    base = load_base_scalars(base_path)

    reports = [load_report(path.absolute()) for path in args.reports]
    report_paths = [report["_path"] for report in reports]
    if len(report_paths) != len(set(report_paths)):
        fail("the same profile report was supplied more than once")
    identity = deployment_identity(reports[0])
    for report in reports[1:]:
        if deployment_identity(report) != identity:
            fail("profile reports do not describe one exact deployment identity")
    expected_base = (Path(identity["policy_root"]) / "LW/base.yaml").resolve()
    if base_path != expected_base:
        fail("--base-yaml is not the profiled deployment base.yaml")
    base_record = identity["policy_assets"][0]
    if (
        base_record["path"] != "LW/base.yaml"
        or before_digest != base_record["sha256"]
    ):
        fail("base.yaml does not match the profiled deployment identity")
    validate_current_policy_assets(identity)

    host_reports = [report for report in reports if report.get("mode") == "host-only"]
    hardware_reports = [
        report for report in reports if report.get("mode") == "hardware-observe"
    ]
    if not host_reports:
        fail("at least one host-only report is required")
    if len(hardware_reports) > 1:
        fail("at most one hardware-observe report may be analyzed at a time")
    host_duration = host_reports[0]["duration_per_policy_seconds"]
    if any(
        report["duration_per_policy_seconds"] != host_duration
        for report in host_reports[1:]
    ):
        fail("host-only reports have non-comparable per-policy durations")

    ranked = sorted(
        ((host_score(report), report) for report in host_reports),
        key=lambda item: item[0],
    )
    selected_score, selected = ranked[0]
    selected_policy = require_mapping(selected["policies"][0], "policy")
    selected_startup = require_mapping(
        selected_policy["control_startup"], "control_startup"
    )
    selected_cpu = selected_startup["requested_cpu"]
    selected_priority = selected_startup["requested_realtime_priority"]
    dt_seconds = float(base["dt"])

    overlay: dict[str, Any] = {
        "control_loop_cpu": selected_cpu,
        "control_loop_realtime_priority": selected_priority,
        # Requiring SCHED_FIFO is an operational decision, never an inferred one.
        "control_loop_require_realtime": False,
        # Fatal timing remains disabled until physical hard-disable validation.
        "control_loop_fatal_consecutive_misses": 0,
        "control_loop_fatal_lateness": 0.0,
    }
    decisions: dict[str, Any] = {
        "control_loop_cpu": {
            "status": "provisional",
            "selected": selected_cpu,
            "ranking_score": list(selected_score),
            "source": selected["_path"],
        },
        "control_loop_realtime_priority": {
            "status": "provisional_explicit_candidates_only",
            "selected": selected_priority,
        },
        "control_loop_require_realtime": {
            "status": "manual_required",
            "candidate_overlay_kept_false": True,
        },
        "fatal_timing": {
            "status": "physical_validation_required",
            "candidate_overlay_kept_disabled": True,
        },
    }

    observed_deadline_ms = selected_score[1] / 1000.0
    if args.max_safe_control_gap_ms is None:
        overlay["control_loop_degraded_consecutive_misses"] = int(
            base["control_loop_degraded_consecutive_misses"]
        )
        overlay["control_loop_degraded_lateness"] = float(
            base["control_loop_degraded_lateness"]
        )
        decisions["degraded_timing"] = {
            "status": "manual_required",
            "reason": "--max-safe-control-gap-ms was not supplied",
        }
    else:
        candidate_ms = rounded_up(
            max(3.0 * dt_seconds * 1000.0, 1.25 * observed_deadline_ms), 0.1
        )
        if candidate_ms > args.max_safe_control_gap_ms:
            fail(
                "observed control timing plus margin exceeds the operator-supplied "
                "maximum safe control gap"
            )
        overlay["control_loop_degraded_lateness"] = candidate_ms / 1000.0
        overlay["control_loop_degraded_consecutive_misses"] = max(
            1, math.ceil(candidate_ms / (dt_seconds * 1000.0))
        )
        decisions["degraded_timing"] = {
            "status": "provisional",
            "observed_maximum_deadline_ms": observed_deadline_ms,
            "operator_maximum_safe_gap_ms": args.max_safe_control_gap_ms,
        }

    if not hardware_reports:
        overlay["sensor_timeout"] = float(base["sensor_timeout"])
        overlay["serial_write_timeout"] = float(base["serial_write_timeout"])
        decisions["sensor_timeout"] = {
            "status": "hardware_measurement_required"
        }
        decisions["serial_write_timeout"] = {
            "status": "hardware_measurement_required"
        }
    else:
        hardware_report = hardware_reports[0]
        if hardware_report.get("hardware_confirmation") is not True:
            fail("hardware report is missing the exact suspension confirmation")
        hardware = require_mapping(hardware_report.get("hardware"), "hardware")
        if hardware.get("commands_sent") != "motors_disable_only":
            fail("hardware report does not prove motors-disable-only output")
        if hardware.get("initial_disable_writes_complete") is not True:
            fail("hardware report does not prove complete initial disable writes")
        if require_integer(
            hardware, "initial_disable_packets", "hardware"
        ) < 20:
            fail("hardware report has insufficient initial disable packets")
        if hardware.get("disable_keepalive_started") is not True:
            fail("hardware report does not prove disable keepalive startup")
        if hardware.get("disable_only_output_enforced") is not True:
            fail("hardware report does not prove continuous disable-only output")
        if require_integer(
            hardware, "serial_write_failures", "hardware"
        ) != 0:
            fail("hardware report contains serial write failures")
        sensor_names = ("imu", "right_feedback", "left_feedback")
        if any(hardware.get(f"{name}_seen") is not True for name in sensor_names):
            fail("hardware report did not observe every required sensor source")
        sensor_distributions = [
            require_mapping(hardware.get(f"{name}_gap"), f"{name}_gap")
            for name in sensor_names
        ]
        if any(
            require_integer(distribution, "count", "sensor")
            < args.minimum_hardware_samples
            for distribution in sensor_distributions
        ):
            fail("hardware report has insufficient sensor-gap samples")
        serial = require_mapping(
            hardware.get("serial_write_duration"), "serial_write_duration"
        )
        if require_integer(serial, "count", "serial_write_duration") \
                < args.minimum_hardware_samples:
            fail("hardware report has insufficient serial-write samples")

        if args.max_safe_sensor_timeout_ms is None:
            overlay["sensor_timeout"] = float(base["sensor_timeout"])
            decisions["sensor_timeout"] = {
                "status": "manual_required",
                "reason": "--max-safe-sensor-timeout-ms was not supplied",
            }
        else:
            sensor_candidate_us = max(
                max(require_number(item, "p50_us", "sensor") for item in sensor_distributions)
                * 3.0,
                max(require_number(item, "p999_us", "sensor") for item in sensor_distributions)
                * 1.5,
                max(require_number(item, "maximum_us", "sensor") for item in sensor_distributions)
                * 1.25,
                max(
                    require_number(
                        hardware,
                        f"{name}_first_sample_delay_us",
                        "hardware",
                    )
                    for name in sensor_names
                )
                * 1.25,
                max(
                    require_number(
                        hardware,
                        f"{name}_final_age_us",
                        "hardware",
                    )
                    for name in sensor_names
                )
                * 1.25,
            )
            sensor_candidate_ms = rounded_up(sensor_candidate_us / 1000.0, 0.1)
            if sensor_candidate_ms > args.max_safe_sensor_timeout_ms:
                fail(
                    "observed sensor gaps plus margin exceed the operator-supplied "
                    "maximum safe sensor timeout"
                )
            overlay["sensor_timeout"] = sensor_candidate_ms / 1000.0
            decisions["sensor_timeout"] = {
                "status": "provisional_suspended_only",
                "candidate_ms": sensor_candidate_ms,
                "operator_maximum_safe_ms": args.max_safe_sensor_timeout_ms,
            }

        serial_candidate_ms = rounded_up(
            max(
                require_number(serial, "p999_us", "serial") * 1.5,
                require_number(serial, "maximum_us", "serial") * 1.25,
            )
            / 1000.0,
            0.1,
        )
        if serial_candidate_ms >= dt_seconds * 1000.0:
            fail("serial write candidate consumes the complete control period")
        overlay["serial_write_timeout"] = serial_candidate_ms / 1000.0
        decisions["serial_write_timeout"] = {
            "status": "provisional_suspended_only",
            "candidate_ms": serial_candidate_ms,
        }

    result = {
        "schema_version": 2,
        "review_only": True,
        "must_not_be_applied_without_human_review": True,
        "deployment_identity": identity,
        "base_configuration": {
            "path": str(base_path),
            "sha256": before_digest,
        },
        "input_reports": [
            {
                "path": report["_path"],
                "sha256": report["_sha256"],
                "mode": report["mode"],
                "duration_per_policy_seconds": report[
                    "duration_per_policy_seconds"
                ],
            }
            for report in reports
        ],
        "candidate_overlay": {"LW": overlay},
        "decisions": decisions,
        "ranked_host_reports": [
            {"score": list(score), "report": report["_path"]}
            for score, report in ranked
        ],
    }
    output_path = args.output.resolve()
    if output_path.exists():
        fail(f"refusing to overwrite existing analysis: {output_path}")
    validate_inputs_unchanged(reports, base_path, before_digest, identity)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("x", encoding="utf-8") as destination:
        json.dump(result, destination, ensure_ascii=False, indent=2)
        destination.write("\n")
    try:
        validate_inputs_unchanged(reports, base_path, before_digest, identity)
    except (OSError, RuntimeError):
        output_path.unlink(missing_ok=True)
        raise
    print(f"review-only candidate report written to {output_path}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Collect and analyze LW runtime configuration profiles"
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    host = subparsers.add_parser("collect-host")
    host.add_argument("--profiler", type=Path, required=True)
    host.add_argument("--policy-root", type=Path, required=True)
    host.add_argument("--output-dir", type=Path, required=True)
    host.add_argument("--duration-seconds", type=float, default=30.0)
    host.add_argument("--cpus", default="allowed")
    host.add_argument("--realtime-priorities", default="0")
    host.add_argument("--require-realtime", action="store_true")
    host.set_defaults(func=collect_host)

    hardware = subparsers.add_parser("collect-hardware")
    hardware.add_argument("--profiler", type=Path, required=True)
    hardware.add_argument("--policy-root", type=Path, required=True)
    hardware.add_argument("--output", type=Path, required=True)
    hardware.add_argument("--duration-seconds", type=float, default=60.0)
    hardware.add_argument("--cpu", type=int, required=True)
    hardware.add_argument("--realtime-priority", type=int, default=0)
    hardware.add_argument("--require-realtime", action="store_true")
    hardware.add_argument("--right-port", default="/dev/ttyLegRight")
    hardware.add_argument("--left-port", default="/dev/ttyLegLeft")
    hardware.add_argument("--imu-topic", default="/imu")
    hardware.add_argument("--confirmation", required=True)
    hardware.set_defaults(func=collect_hardware)

    analyzer = subparsers.add_parser("analyze")
    analyzer.add_argument("--base-yaml", type=Path, required=True)
    analyzer.add_argument("--reports", type=Path, nargs="+", required=True)
    analyzer.add_argument("--output", type=Path, required=True)
    analyzer.add_argument("--max-safe-sensor-timeout-ms", type=float)
    analyzer.add_argument("--max-safe-control-gap-ms", type=float)
    analyzer.add_argument("--minimum-hardware-samples", type=int, default=1000)
    analyzer.set_defaults(func=analyze)
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if getattr(args, "duration_seconds", 1.0) <= 0.0:
        parser.error("--duration-seconds must be positive")
    if getattr(args, "minimum_hardware_samples", 1) <= 0:
        parser.error("--minimum-hardware-samples must be positive")
    for name in ("max_safe_sensor_timeout_ms", "max_safe_control_gap_ms"):
        value = getattr(args, name, None)
        if value is not None and (not math.isfinite(value) or value <= 0.0):
            parser.error(f"--{name.replace('_', '-')} must be positive")
    try:
        args.func(args)
        return 0
    except (OSError, RuntimeError, subprocess.SubprocessError, json.JSONDecodeError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
