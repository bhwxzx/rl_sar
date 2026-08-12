#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import shutil
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "profile_lw_runtime_config.py"
SPEC = importlib.util.spec_from_file_location("profile_lw_runtime_config", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def distribution(count: int, p50: float, p999: float, maximum: float) -> dict:
    return {
        "count": count,
        "retained": count,
        "minimum_us": p50 / 2,
        "mean_us": p50,
        "p50_us": p50,
        "p95_us": p999 * 0.8,
        "p99_us": p999 * 0.9,
        "p999_us": p999,
        "maximum_us": maximum,
    }


def policy(
    name: str,
    cpu: int,
    priority: int,
    deadline_us: float,
    duration: float = 30.0,
) -> dict:
    startup = {
        "requested_cpu": cpu,
        "requested_realtime_priority": priority,
        "affinity_applied": cpu >= 0,
        "realtime_applied": priority > 0,
        "realtime_error": 0,
    }
    timing = {
        "cycles": 1000,
        "missed_deadlines": 0,
        "skipped_periods": 0,
        "average_wakeup_us": 10,
        "maximum_wakeup_us": 100,
        "maximum_deadline_lateness_us": deadline_us,
        "maximum_execution_us": 700,
        "level": 0,
    }
    return {
        "policy": name,
        "duration_seconds": duration,
        "inference_duration": distribution(1000, 1000, 1500, 1700),
        "control_timing": timing,
        "inference_timing": timing,
        "control_startup": startup,
        "inference_startup": {
            **startup,
            "requested_cpu": -1,
            "affinity_applied": False,
            "requested_realtime_priority": 0,
            "realtime_applied": False,
        },
    }


class ProfileAnalyzerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.policy_root = self.root / "policy"
        for relative in MODULE.POLICY_ASSETS:
            asset = self.policy_root / relative
            asset.parent.mkdir(parents=True, exist_ok=True)
            asset.write_text(f"test asset: {relative}\n", encoding="utf-8")
        self.base = self.policy_root / "LW/base.yaml"
        self.base.write_text(
            "LW:\n"
            "  dt: 0.005\n"
            "  sensor_timeout: 0.1\n"
            "  serial_write_timeout: 0.002\n"
            "  control_loop_degraded_consecutive_misses: 3\n"
            "  control_loop_degraded_lateness: 0.02\n",
            encoding="utf-8",
        )

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def policy_assets(self, root: Path | None = None) -> list[dict]:
        selected_root = self.policy_root if root is None else root
        return [
            {
                "path": relative,
                "sha256": MODULE.file_digest(selected_root / relative),
            }
            for relative in MODULE.POLICY_ASSETS
        ]

    def copy_policy_root(self, name: str) -> Path:
        destination = self.root / name
        shutil.copytree(self.policy_root, destination)
        return destination

    def write_report(
        self,
        name: str,
        mode: str,
        cpu: int,
        deadline: float,
        *,
        duration: float = 30.0,
        source_commit: str = "a" * 40,
        policy_root: Path | None = None,
        host: dict | None = None,
    ) -> Path:
        selected_root = self.policy_root if policy_root is None else policy_root
        report = {
            "schema_version": 2,
            "source_commit": source_commit,
            "mode": mode,
            "host": host or {
                "node": "lw-host",
                "system": "Linux",
                "release": "test-kernel",
                "machine": "x86_64",
            },
            "policy_root": str(selected_root.resolve()),
            "policy_assets": self.policy_assets(selected_root),
            "duration_per_policy_seconds": duration,
            "hardware_confirmation": mode == "hardware-observe",
            "failed": False,
            "policies": [
                policy(item, cpu, 0, deadline, duration)
                for item in MODULE.POLICIES
            ],
            "hardware": {
                "initial_disable_writes_complete": mode == "hardware-observe",
                "initial_disable_packets": 20 if mode == "hardware-observe" else 0,
                "disable_keepalive_started": mode == "hardware-observe",
                "disable_keepalive_period_ms": 5,
                "disable_only_output_enforced": mode == "hardware-observe",
                "imu_seen": mode == "hardware-observe",
                "imu_first_sample_delay_us": 5000,
                "imu_final_age_us": 4000,
                "imu_gap": distribution(2000, 5000, 6000, 7000),
                "right_feedback_seen": mode == "hardware-observe",
                "right_feedback_first_sample_delay_us": 6000,
                "right_feedback_final_age_us": 4500,
                "right_feedback_gap": distribution(2000, 5000, 6500, 7500),
                "left_feedback_seen": mode == "hardware-observe",
                "left_feedback_first_sample_delay_us": 5500,
                "left_feedback_final_age_us": 4200,
                "left_feedback_gap": distribution(2000, 5000, 6200, 7200),
                "serial_write_duration": (
                    distribution(2000, 300, 500, 600)
                    if mode == "hardware-observe"
                    else distribution(0, 0, 0, 0)
                ),
                "serial_write_failures": 0,
                "commands_sent": (
                    "motors_disable_only"
                    if mode == "hardware-observe"
                    else "none"
                ),
            },
        }
        path = self.root / name
        path.write_text(json.dumps(report), encoding="utf-8")
        return path

    def analyze(self, reports: list[Path], output: Path, sensor_bound=20.0) -> int:
        arguments = [
            "analyze",
            "--base-yaml",
            str(self.base),
            "--reports",
            *(str(report) for report in reports),
            "--output",
            str(output),
            "--max-safe-control-gap-ms",
            "20",
            "--minimum-hardware-samples",
            "100",
        ]
        if sensor_bound is not None:
            arguments.extend(
                ["--max-safe-sensor-timeout-ms", str(sensor_bound)]
            )
        return MODULE.main(arguments)

    def test_selects_best_cpu_and_keeps_fatal_disabled(self) -> None:
        slower = self.write_report("slow.yaml", "host-only", 2, 800)
        faster = self.write_report("fast.yaml", "host-only", 3, 400)
        hardware = self.write_report("hardware.yaml", "hardware-observe", 3, 450)
        output = self.root / "candidate.yaml"
        before = self.base.read_bytes()
        self.assertEqual(self.analyze([slower, faster, hardware], output), 0)
        result = json.loads(output.read_text(encoding="utf-8"))
        self.assertEqual(result["schema_version"], 2)
        self.assertEqual(result["deployment_identity"]["source_commit"], "a" * 40)
        self.assertEqual(
            result["base_configuration"]["sha256"],
            MODULE.file_digest(self.base),
        )
        self.assertEqual(len(result["input_reports"]), 3)
        overlay = result["candidate_overlay"]["LW"]
        self.assertEqual(overlay["control_loop_cpu"], 3)
        self.assertFalse(overlay["control_loop_require_realtime"])
        self.assertEqual(overlay["control_loop_fatal_consecutive_misses"], 0)
        self.assertEqual(overlay["control_loop_fatal_lateness"], 0.0)
        self.assertEqual(before, self.base.read_bytes())

    def test_without_hardware_retains_existing_io_values(self) -> None:
        host = self.write_report("host.yaml", "host-only", 1, 400)
        output = self.root / "candidate.yaml"
        self.assertEqual(self.analyze([host], output, sensor_bound=None), 0)
        result = json.loads(output.read_text(encoding="utf-8"))
        overlay = result["candidate_overlay"]["LW"]
        self.assertEqual(overlay["sensor_timeout"], 0.1)
        self.assertEqual(overlay["serial_write_timeout"], 0.002)
        self.assertEqual(
            result["decisions"]["sensor_timeout"]["status"],
            "hardware_measurement_required",
        )

    def test_rejects_unsafe_sensor_candidate(self) -> None:
        host = self.write_report("host.yaml", "host-only", 1, 400)
        hardware = self.write_report("hardware.yaml", "hardware-observe", 1, 400)
        output = self.root / "candidate.yaml"
        self.assertEqual(self.analyze([host, hardware], output, sensor_bound=5.0), 1)
        self.assertFalse(output.exists())

    def test_hardware_collection_requires_exact_confirmation(self) -> None:
        parser = MODULE.build_parser()
        args = parser.parse_args(
            [
                "collect-hardware",
                "--profiler",
                "/bin/true",
                "--policy-root",
                str(self.root),
                "--output",
                str(self.root / "hardware.yaml"),
                "--cpu",
                "0",
                "--confirmation",
                "wrong",
            ]
        )
        with self.assertRaises(RuntimeError):
            MODULE.collect_hardware(args)

    def test_rejects_report_missing_one_policy(self) -> None:
        host = self.write_report("host.json", "host-only", 1, 400)
        report = json.loads(host.read_text(encoding="utf-8"))
        report["policies"].pop()
        host.write_text(json.dumps(report), encoding="utf-8")
        output = self.root / "candidate.json"
        self.assertEqual(self.analyze([host], output, sensor_bound=None), 1)
        self.assertFalse(output.exists())

    def test_rejects_duplicate_and_reordered_policies(self) -> None:
        for mutation in ("duplicate", "reordered"):
            host = self.write_report(
                f"host-{mutation}.json", "host-only", 1, 400
            )
            report = json.loads(host.read_text(encoding="utf-8"))
            if mutation == "duplicate":
                report["policies"][-1] = report["policies"][0]
            else:
                report["policies"][0], report["policies"][1] = (
                    report["policies"][1],
                    report["policies"][0],
                )
            host.write_text(json.dumps(report), encoding="utf-8")
            output = self.root / f"candidate-{mutation}.json"
            self.assertEqual(self.analyze([host], output, sensor_bound=None), 1)
            self.assertFalse(output.exists())

    def test_rejects_mixed_deployment_identity(self) -> None:
        cases = (
            {
                "source_commit": "b" * 40,
            },
            {
                "host": {
                    "node": "other-host",
                    "system": "Linux",
                    "release": "test-kernel",
                    "machine": "x86_64",
                },
            },
            {
                "policy_root": self.copy_policy_root("other-policy"),
            },
        )
        for index, overrides in enumerate(cases):
            first = self.write_report(
                f"identity-{index}-a.json", "host-only", 1, 400
            )
            second = self.write_report(
                f"identity-{index}-b.json",
                "host-only",
                2,
                400,
                **overrides,
            )
            output = self.root / f"identity-{index}-candidate.json"
            self.assertEqual(
                self.analyze([first, second], output, sensor_bound=None), 1
            )
            self.assertFalse(output.exists())

    def test_rejects_mixed_policy_asset_digest(self) -> None:
        first = self.write_report("asset-a.json", "host-only", 1, 400)
        second = self.write_report("asset-b.json", "host-only", 2, 400)
        report = json.loads(second.read_text(encoding="utf-8"))
        report["policy_assets"][1]["sha256"] = "b" * 64
        second.write_text(json.dumps(report), encoding="utf-8")
        output = self.root / "asset-candidate.json"
        self.assertEqual(
            self.analyze([first, second], output, sensor_bound=None), 1
        )
        self.assertFalse(output.exists())

    def test_rejects_legacy_schema(self) -> None:
        host = self.write_report("legacy.json", "host-only", 1, 400)
        report = json.loads(host.read_text(encoding="utf-8"))
        report["schema_version"] = 1
        host.write_text(json.dumps(report), encoding="utf-8")
        output = self.root / "legacy-candidate.json"
        self.assertEqual(self.analyze([host], output, sensor_bound=None), 1)
        self.assertFalse(output.exists())

    def test_rejects_mixed_host_durations(self) -> None:
        first = self.write_report("duration-a.json", "host-only", 1, 400)
        second = self.write_report(
            "duration-b.json", "host-only", 2, 400, duration=60.0
        )
        output = self.root / "duration-candidate.json"
        self.assertEqual(
            self.analyze([first, second], output, sensor_bound=None), 1
        )
        self.assertFalse(output.exists())

    def test_allows_independent_hardware_duration(self) -> None:
        host = self.write_report(
            "duration-host.json", "host-only", 1, 400, duration=30.0
        )
        hardware = self.write_report(
            "duration-hardware.json",
            "hardware-observe",
            1,
            400,
            duration=60.0,
        )
        output = self.root / "duration-valid-candidate.json"
        self.assertEqual(self.analyze([host, hardware], output), 0)
        result = json.loads(output.read_text(encoding="utf-8"))
        self.assertEqual(
            [item["duration_per_policy_seconds"] for item in result["input_reports"]],
            [30.0, 60.0],
        )

    def test_rejects_stale_base_and_policy_assets(self) -> None:
        host = self.write_report("stale.json", "host-only", 1, 400)
        self.base.write_text(self.base.read_text(encoding="utf-8") + "# stale\n")
        output = self.root / "stale-candidate.json"
        self.assertEqual(self.analyze([host], output, sensor_bound=None), 1)
        self.assertFalse(output.exists())

    def test_rejects_unknown_or_contradictory_mode(self) -> None:
        unknown = self.write_report("unknown.json", "unknown", 1, 400)
        output = self.root / "unknown-candidate.json"
        self.assertEqual(self.analyze([unknown], output, sensor_bound=None), 1)
        self.assertFalse(output.exists())

        host = self.write_report("contradictory.json", "host-only", 1, 400)
        report = json.loads(host.read_text(encoding="utf-8"))
        report["hardware_confirmation"] = True
        host.write_text(json.dumps(report), encoding="utf-8")
        output = self.root / "contradictory-candidate.json"
        self.assertEqual(self.analyze([host], output, sensor_bound=None), 1)
        self.assertFalse(output.exists())

    def test_detects_report_change_after_loading(self) -> None:
        host = self.write_report("changing.json", "host-only", 1, 400)
        loaded = MODULE.load_report(host)
        identity = MODULE.deployment_identity(loaded)
        host.write_text(host.read_text(encoding="utf-8") + "\n", encoding="utf-8")
        with self.assertRaises(RuntimeError):
            MODULE.validate_inputs_unchanged(
                [loaded],
                self.base,
                MODULE.file_digest(self.base),
                identity,
            )

    def test_refuses_to_overwrite_analysis(self) -> None:
        host = self.write_report("host.json", "host-only", 1, 400)
        output = self.root / "candidate.json"
        output.write_text("preserve me\n", encoding="utf-8")
        self.assertEqual(self.analyze([host], output, sensor_bound=None), 1)
        self.assertEqual(output.read_text(encoding="utf-8"), "preserve me\n")

    def test_rejects_malformed_numeric_field(self) -> None:
        host = self.write_report("host.json", "host-only", 1, 400)
        report = json.loads(host.read_text(encoding="utf-8"))
        report["policies"][0]["control_startup"][
            "requested_realtime_priority"
        ] = "zero"
        host.write_text(json.dumps(report), encoding="utf-8")
        output = self.root / "candidate.json"
        self.assertEqual(self.analyze([host], output, sensor_bound=None), 1)
        self.assertFalse(output.exists())

    def test_rejects_hardware_without_initial_disable_proof(self) -> None:
        host = self.write_report("host.json", "host-only", 1, 400)
        hardware = self.write_report(
            "hardware.json", "hardware-observe", 1, 400
        )
        report = json.loads(hardware.read_text(encoding="utf-8"))
        report["hardware"]["initial_disable_writes_complete"] = False
        hardware.write_text(json.dumps(report), encoding="utf-8")
        output = self.root / "candidate.json"
        self.assertEqual(self.analyze([host, hardware], output), 1)
        self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
