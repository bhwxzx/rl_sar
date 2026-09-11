"""Regression tests for provenance reuse, publication and finite batch failures."""
import copy
import gzip
import json
import os
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import evidence_provenance as P
import run_evaluation_batch as B
import summarize_evaluation_batch as S
import policy_evaluation_evidence as E
import test_policy_export_evidence as fixture
from test_policy_export_evidence import CONFIG, EXPORT, IDENTITY


class EvaluationBatchTests(unittest.TestCase):
    def setUp(self):
        fixture.PolicyExportEvidenceTests.setUp(self)
        self.root = self.repo_root / "learnings/policy_tuning" / self.task / self.run_id
        self.context = copy.deepcopy(self.identity)
        self.context["version"] = 2
        del self.context["evaluation_scenario"]
        self.rehash(self.context)
        self.context_ref = P.store_object(self.root, "context", self.context)
        self.config = CONFIG.capture_effective_config(self.context, self.log_dir)
        self.config_ref = P.store_object(self.root, "config", self.config)
        files = []
        for name in ("scripts/reinforcement_learning/rsl_rl/evaluate_policy.py",
                     "scripts/reinforcement_learning/rsl_rl/policy_evaluation_evidence.py",
                     "scripts/reinforcement_learning/rsl_rl/policy_evaluation_telemetry.py",
                     ".agents/skills/monitor-tune-isaaclab-training/scripts/evidence_provenance.py"):
            path = self.repo_root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("fixture source\n")
            files.append({"path": name, "sha256": P.digest(path.read_bytes()), "content_utf8": path.read_text()})
        self.source = P.store_object(self.root, "source", {"version": 1, "repository_root": str(self.repo_root), "head": "1" * 40, "files": files})

    @staticmethod
    def rehash(identity):
        identity.pop("identity_sha256", None)
        identity["identity_sha256"] = IDENTITY._sha256_bytes(IDENTITY._canonical_json(identity).encode())

    def contract(self, batch="turning-001", count=6):
        cases = []
        for index in range(count):
            scenario = {**self.scenario, "scenario_id": f"turn-{index}", "command_schedule": [
                {"start_step": 0, "end_step": 7, "command": [0.0, 0.0, 0.5 if index % 2 else -0.5]}]}
            cases.append({"attempt_id": f"case-{index}-a01", "scenario": scenario, "video": False, "timeout_seconds": 30})
        return {"version": 1, "batch_id": batch, "run_identity": self.context_ref,
                "effective_config": self.config_ref, "checkpoint": {"path": str(self.checkpoint), "sha256": self.checkpoint_sha256}, "cases": cases}

    def publish(self, contract, case, paths):
        scenario_ref = P.capture_scenario(self.context, case["scenario"], self.source)
        plan = E.preflight_evaluation(repo_root=self.repo_root, task=self.task, run_id=self.run_id,
            batch_id=contract["batch_id"], evaluation_id=case["attempt_id"], result_path=Path(paths["play_result"]),
            telemetry_path=Path(paths["telemetry"]), video_path=None, checkpoint_path=self.checkpoint,
            checkpoint_sha256=self.checkpoint_sha256, artifact_kind="native", artifact_path=self.checkpoint,
            artifact_sha256=self.checkpoint_sha256, run_identity_path=Path(self.context_ref["path"]),
            run_identity_file_sha256=self.context_ref["sha256"], scenario_contract=case["scenario"],
            effective_config=self.config_ref, scenario_evidence=scenario_ref)
        binding = {"task": self.task, "run_id": self.run_id, "batch_id": contract["batch_id"],
                   "evaluation_id": case["attempt_id"], "candidate_id": "model_10", "runner": self.context["runner"]}
        inputs = {"checkpoint": plan.checkpoint, "artifact": {"kind": "native", **plan.artifact},
                  "run_identity": plan.run_identity, "effective_config": self.config_ref, "scenario_evidence": scenario_ref,
                  "scenario": {"contract": case["scenario"], "sha256": plan.scenario_sha256},
                  "resource_mode": {"training_overlap": False, "idle_gpu_required": True, "video_requested": False, "telemetry_requested": True}}
        result = {"version": 3, "layout_version": 2, "status": "completed", "evaluation": binding,
                  "inputs": inputs, "telemetry_status": "complete", "metrics": {"tracking_yaw_rmse": 0.5}}
        telemetry = {"version": 4, "evaluation": binding, "inputs": inputs, "samples": [{"step": 0, "value": 1.2345678901234567}]}
        with E.EvaluationPublisher(plan) as publisher:
            result = publisher.publish(result, telemetry=telemetry, video_source=None)
        return result, telemetry

    def execute_fake(self, argv, console, timeout):
        console.write_text("fixture evaluation\n")
        def arg(name):
            return argv[argv.index(name) + 1]
        contract = self.active_contract
        case = next(c for c in contract["cases"] if c["attempt_id"] == arg("--evaluation_id"))
        if case["attempt_id"] == "case-2-a01":
            return 1, "injected failure"
        self.publish(contract, case, {"play_result": arg("--result_path"), "telemetry": arg("--telemetry_path")})
        return 0, None

    def test_six_cases_one_event_failure_and_retry_preserve_evidence(self):
        self.active_contract = self.contract()
        with patch.object(B, "REPO_ROOT", self.repo_root), patch.dict(os.environ, {"CONDA_DEFAULT_ENV": "isaacsim-5.1"}), patch.object(B, "capture_evaluator_source", return_value=self.source):
            receipt = B.run_batch(self.active_contract, execute=self.execute_fake)
            self.assertFalse(receipt["all_completed"])
            batch = S.validate_batch(Path(receipt["batch"]["path"]))
            self.assertEqual([c["status"] for c in batch["cases"]].count("completed"), 5)
            self.assertEqual(len(list((self.root / "events").glob("*.json"))), 1)
            self.assertEqual(len(list((self.root / "evaluations/turning-001/raw").glob("*/console.log"))), 6)
            with self.assertRaises(FileExistsError):
                B.run_batch(self.active_contract, execute=self.execute_fake)
            old_manifest = Path(receipt["batch"]["path"]).read_bytes()
            self.active_contract = self.contract("turning-002", count=1)
            B.run_batch(self.active_contract, execute=self.execute_fake)
            self.assertEqual(old_manifest, Path(receipt["batch"]["path"]).read_bytes())
        self.assertEqual(len(list((self.root / "provenance").glob("config-*.json"))), 1)
        self.assertIn("turning-001/report.md", (self.root / "index.md").read_text())
        result_path = Path(batch["cases"][0]["result"]["path"])
        result = json.loads(result_path.read_bytes())
        telemetry = Path(result["outputs"]["telemetry"]["path"])
        telemetry.write_bytes(telemetry.read_bytes() + b"tamper")
        with self.assertRaises(ValueError):
            S.validate_batch(Path(receipt["batch"]["path"]))

    def test_lossless_gzip_and_new_selection_validator(self):
        contract = self.contract(count=1)
        case = contract["cases"][0]
        layout = B.prepare_batch_layout(self.root.parents[1], task=self.task, run_id=self.run_id, batch_id=contract["batch_id"], evaluation_id=case["attempt_id"])
        result, original = self.publish(contract, case, layout["paths"])
        compressed = Path(layout["paths"]["telemetry"]).read_bytes()
        self.assertEqual(json.loads(gzip.decompress(compressed)), original)
        self.assertEqual(compressed[4:8], b"\0\0\0\0")
        output = self.selection_path.parent / "selection-new-context.json"
        EXPORT.record_checkpoint_selection(selection_id="new-context", approved_at="2026-09-08T00:00:00Z",
            checkpoint_id="model_10", checkpoint_path=self.checkpoint, checkpoint_sha256=self.checkpoint_sha256,
            selection_report_path=self.report_path, selection_report_sha256=self.report_sha256,
            run_identity_path=Path(self.context_ref["path"]), run_identity_file_sha256=self.context_ref["sha256"],
            effective_config_path=Path(self.config_ref["path"]), effective_config_sha256=self.config_ref["sha256"],
            evaluation_result_paths=[Path(result["result_path"])], tensor_contract=self.tensor_contract, output_path=output)
        EXPORT.validate_checkpoint_selection(output, expected_sha256=P.digest(output.read_bytes()))
        with self.assertRaises(ValueError):
            B.prepare_batch_layout(self.root.parents[1], task=self.task, run_id=self.run_id, batch_id=contract["batch_id"], evaluation_id=case["attempt_id"])

    def test_context_source_drift_does_not_duplicate_config(self):
        changed = copy.deepcopy(self.context)
        changed["source"]["head"] = "3" * 40
        self.rehash(changed)
        other_context = P.store_object(self.root, "context", changed)
        other_config = CONFIG.capture_effective_config(changed, self.log_dir)
        self.assertNotEqual(other_context, self.context_ref)
        self.assertEqual(P.store_object(self.root, "config", other_config), self.config_ref)
        scenario = self.contract(count=1)["cases"][0]["scenario"]
        ref = P.capture_scenario(self.context, scenario, self.source)
        path = self.repo_root / "scripts/reinforcement_learning/rsl_rl/evaluate_policy.py"
        path.write_text("changed source")
        with self.assertRaisesRegex(ValueError, "drift"):
            P.validate_provenance(self.context, self.config_ref, ref, scenario, live=True)
        P.validate_provenance(self.context, self.config_ref, ref, scenario, live=False)

    def test_content_addressed_tamper_and_symlink_rejected(self):
        path = Path(self.config_ref["path"])
        path.write_text("{}")
        with self.assertRaises(ValueError):
            P.store_object(self.root, "config", self.config)
        target = self.root / "unsafe"
        target.symlink_to(self.root, target_is_directory=True)
        with self.assertRaises(ValueError):
            P.write_bytes(target / "new.json", b"{}")
        with self.assertRaises(ValueError):
            P.write_bytes(self.root / ".." / "escape.json", b"{}")

    def test_interruption_records_not_run_and_query_reads_v5(self):
        import query_tuning_experience as Q
        contract = self.contract("interrupt-001", count=3)
        def interrupt(argv, console, timeout):
            console.write_text("interrupted fixture")
            return -15, "KeyboardInterrupt"
        with patch.object(B, "REPO_ROOT", self.repo_root), patch.dict(os.environ, {"CONDA_DEFAULT_ENV": "isaacsim-5.1"}), patch.object(B, "capture_evaluator_source", return_value=self.source):
            receipt = B.run_batch(contract, execute=interrupt)
        batch = S.validate_batch(Path(receipt["batch"]["path"]))
        self.assertEqual([case["status"] for case in batch["cases"]], ["failed", "not_run", "not_run"])
        query = Q.query_tuning_experience(self.root.parents[1], run_identity=self.context,
            effective_config_path=Path(self.config_ref["path"]), effective_config_sha256=self.config_ref["sha256"],
            observation_fingerprint="known-obs", deployment_fingerprint="known-deployment")
        self.assertFalse(query["invalid_events"])
        self.assertFalse(query["candidate_events"])
        report = Path(batch["report"]["path"])
        report.write_text("tampered report")
        with self.assertRaisesRegex(ValueError, "report"):
            S.validate_batch(Path(receipt["batch"]["path"]))

    def test_empty_or_duplicate_batch_never_publishes_manifest(self):
        contract = self.contract(count=1)
        with self.assertRaises(ValueError):
            S.seal_batch(self.root, "empty-001", [])
        self.assertFalse((self.root / "evaluations/empty-001/manifest.json").exists())
        contract["cases"] *= 2
        with self.assertRaisesRegex(ValueError, "duplicate"):
            B.validate_contract(contract)


if __name__ == "__main__":
    unittest.main()
