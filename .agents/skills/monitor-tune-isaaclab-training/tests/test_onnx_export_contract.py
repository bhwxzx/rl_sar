#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


SKILL_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = SKILL_ROOT / "scripts" / "onnx_export_contract.py"


def load_module():
    spec = importlib.util.spec_from_file_location("onnx_export_contract_tested", SCRIPT)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


ONNX_EXPORT = load_module()


class OnnxExportContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        import torch

        torch.manual_seed(42)
        cls.torch = torch
        cls.model = torch.nn.Sequential(
            torch.nn.Linear(4, 8),
            torch.nn.ELU(),
            torch.nn.Linear(8, 2),
        ).eval()
        cls.example = torch.randn(1, 4, dtype=torch.float32)

    def test_static_batch_1_export_is_simplified_and_row_chunked(self) -> None:
        import onnx
        import onnxruntime as ort
        from onnxruntime.capi.onnxruntime_pybind11_state import InvalidArgument

        contract = ONNX_EXPORT.get_onnx_export_contract(
            ONNX_EXPORT.STATIC_BATCH_1_SIMPLIFIED
        )
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "policy.onnx"
            evidence = ONNX_EXPORT.export_onnx_policy(
                self.model,
                self.example,
                path,
                contract=contract,
            )
            model = onnx.load(str(path))
            self.assertEqual(model.graph.input[0].name, "obs")
            self.assertEqual(model.graph.output[0].name, "actions")
            self.assertEqual(
                next(item.version for item in model.opset_import if item.domain == ""),
                17,
            )
            self.assertEqual(evidence["input_shape"], [1, 4])
            self.assertEqual(evidence["output_shape"], [1, 2])
            self.assertTrue(evidence["simplifier_check"])
            self.assertLessEqual(
                evidence["post_simplify_node_count"],
                evidence["pre_simplify_node_count"],
            )

            samples = self.torch.randn(3, 4, dtype=self.torch.float32)
            with self.torch.no_grad():
                expected = self.model(samples)
            actual = ONNX_EXPORT.run_onnx_policy(
                path,
                samples,
                contract=contract,
            )
            self.torch.testing.assert_close(actual, expected, atol=1.0e-6, rtol=1.0e-6)

            session = ort.InferenceSession(
                str(path),
                providers=["CPUExecutionProvider"],
            )
            with self.assertRaises(InvalidArgument):
                session.run(["actions"], {"obs": samples.numpy()})

    def test_dynamic_profile_preserves_batch_support(self) -> None:
        contract = ONNX_EXPORT.get_onnx_export_contract(ONNX_EXPORT.DYNAMIC_BATCH)
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "policy.onnx"
            evidence = ONNX_EXPORT.export_onnx_policy(
                self.model,
                self.example,
                path,
                contract=contract,
            )
            self.assertEqual(evidence["input_shape"], ["batch", 4])
            self.assertEqual(evidence["output_shape"], ["batch", 2])
            self.assertIsNone(evidence["simplifier_check"])
            self.assertEqual(
                evidence["post_simplify_node_count"],
                evidence["pre_simplify_node_count"],
            )
            samples = self.torch.randn(3, 4, dtype=self.torch.float32)
            with self.torch.no_grad():
                expected = self.model(samples)
            actual = ONNX_EXPORT.run_onnx_policy(
                path,
                samples,
                contract=contract,
            )
            self.torch.testing.assert_close(actual, expected, atol=1.0e-6, rtol=1.0e-6)


if __name__ == "__main__":
    unittest.main()
