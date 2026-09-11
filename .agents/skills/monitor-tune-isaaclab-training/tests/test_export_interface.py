#!/usr/bin/env python3

from __future__ import annotations

import ast
import subprocess
import sys
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "rsl_rl_export_policy.py"


class ExportInterfaceTests(unittest.TestCase):
    def test_help_uses_run_checkpoint_and_export_identifiers(self) -> None:
        result = subprocess.run(
            [sys.executable, str(SCRIPT), "--help"],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--run_id RUN_ID", result.stdout)
        self.assertIn("--checkpoint_id CHECKPOINT_ID", result.stdout)
        self.assertIn("--export_run_id EXPORT_RUN_ID", result.stdout)
        self.assertIn("--selection_receipt_path SELECTION_RECEIPT_PATH", result.stdout)
        self.assertIn("--selection_receipt_sha256 SELECTION_RECEIPT_SHA256", result.stdout)
        self.assertIn("--parity_steps PARITY_STEPS", result.stdout)
        self.assertIn("--reset_step RESET_STEP", result.stdout)
        self.assertIn("--reset_contract RESET_CONTRACT", result.stdout)
        self.assertIn("--onnx_export_profile", result.stdout)
        self.assertNotIn("--trial_id", result.stdout)
        self.assertNotIn("--candidate_id", result.stdout)

    def test_new_identifiers_are_required_parser_arguments(self) -> None:
        tree = ast.parse(SCRIPT.read_text(encoding="utf-8"))
        required: set[str] = set()
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call) or not node.args:
                continue
            function = node.func
            if not isinstance(function, ast.Attribute) or function.attr != "add_argument":
                continue
            option = node.args[0]
            if not isinstance(option, ast.Constant) or not isinstance(option.value, str):
                continue
            if any(
                keyword.arg == "required"
                and isinstance(keyword.value, ast.Constant)
                and keyword.value.value is True
                for keyword in node.keywords
            ):
                required.add(option.value)
        self.assertTrue(
            {
                "--run_id",
                "--checkpoint_id",
                "--export_run_id",
                "--selection_receipt_path",
                "--selection_receipt_sha256",
                "--parity_steps",
                "--reset_step",
                "--reset_contract",
                "--onnx_export_profile",
            }.issubset(required)
        )
        self.assertNotIn("--trial_id", required)
        self.assertNotIn("--candidate_id", required)

    def test_result_metadata_uses_only_current_identifiers(self) -> None:
        source = SCRIPT.read_text(encoding="utf-8")
        self.assertIn('"version": 4', source)
        self.assertIn("export_onnx_policy(", source)
        self.assertIn("run_onnx_policy(", source)
        self.assertIn('"run_id": export_plan.run_id', source)
        self.assertIn('"checkpoint_id": args_cli.checkpoint_id', source)
        self.assertIn('"export_id": export_plan.export_id', source)
        self.assertIn("export_publisher.publish(result)", source)
        self.assertNotIn("os.replace", source)
        self.assertNotIn("traced.save(str(Path(args_cli.jit_path)))", source)
        self.assertNotIn("trial_id", source)
        self.assertNotIn("candidate_id", source)

    def test_parity_walks_time_and_explicit_reset_boundary(self) -> None:
        source = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("for step in range(args_cli.parity_steps):", source)
        self.assertIn("if step == args_cli.reset_step:", source)
        self.assertIn("reset_result = env.reset()", source)
        self.assertIn('labels.append("pre_reset")', source)
        self.assertIn('labels.append("post_reset")', source)
        self.assertIn("_concatenate_observation_batches", source)


if __name__ == "__main__":
    unittest.main()
