#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import tempfile


POLICIES = {
    "LW/robot_lab/leg_loco",
    "LW/robot_lab/wheel_loco",
    "LW/robot_lab/leg_to_wheel",
    "LW/robot_lab/wheel_to_leg",
}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profiler", type=Path, required=True)
    parser.add_argument("--policy-root", type=Path, required=True)
    args = parser.parse_args()

    with tempfile.TemporaryDirectory(prefix="lw-profile-integration-") as temporary:
        root = Path(temporary)
        report_path = root / "host.json"
        completed = subprocess.run(
            [
                str(args.profiler.resolve()),
                "--mode",
                "host-only",
                "--policy-root",
                str(args.policy_root.resolve()),
                "--output",
                str(report_path),
                "--duration-seconds",
                "0.05",
                "--cpu",
                "-1",
                "--realtime-priority",
                "0",
            ],
            check=False,
            timeout=30,
        )
        if completed.returncode != 0:
            raise RuntimeError(f"host-only profiler failed: {completed.returncode}")
        report = json.loads(report_path.read_text(encoding="utf-8"))
        if report.get("mode") != "host-only" or report.get("failed") is not False:
            raise RuntimeError("host-only report status differs")
        if {item.get("policy") for item in report.get("policies", [])} != POLICIES:
            raise RuntimeError("host-only report policy set differs")
        if not all(
            item["inference_duration"]["count"] > 0
            for item in report["policies"]
        ):
            raise RuntimeError("a policy did not execute inference")
        hardware = report.get("hardware", {})
        if hardware.get("commands_sent") != "none":
            raise RuntimeError("host-only report claims hardware commands")
        if hardware.get("serial_write_duration", {}).get("count") != 0:
            raise RuntimeError("host-only mode performed serial writes")

        refused_report = root / "hardware.json"
        refused = subprocess.run(
            [
                str(args.profiler.resolve()),
                "--mode",
                "hardware-observe",
                "--policy-root",
                str(args.policy_root.resolve()),
                "--output",
                str(refused_report),
                "--duration-seconds",
                "0.01",
                "--hardware-confirmation",
                "wrong",
            ],
            check=False,
            timeout=10,
        )
        if refused.returncode == 0 or refused_report.exists():
            raise RuntimeError("hardware mode accepted an invalid confirmation")

        unavailable_report = root / "unavailable-hardware.json"
        unavailable = subprocess.run(
            [
                str(args.profiler.resolve()),
                "--mode",
                "hardware-observe",
                "--policy-root",
                str(args.policy_root.resolve()),
                "--output",
                str(unavailable_report),
                "--duration-seconds",
                "0.01",
                "--right-port",
                str(root / "missing-right-port"),
                "--left-port",
                str(root / "missing-left-port"),
                "--hardware-confirmation",
                "I_CONFIRM_LW_IS_SUSPENDED_AND_MOTORS_MUST_REMAIN_DISABLED",
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )
        unavailable_output = unavailable.stdout + unavailable.stderr
        if unavailable.returncode == 0 or unavailable_report.exists():
            raise RuntimeError("hardware mode continued without serial disable")
        if "Preloading ONNX model" in unavailable_output:
            raise RuntimeError("model preload occurred before initial disable")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
