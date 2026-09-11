#!/usr/bin/env python3
"""Define, export, and validate explicit ONNX deployment contracts."""

from __future__ import annotations

from pathlib import Path
from typing import Any


STATIC_BATCH_1_SIMPLIFIED = "static_batch_1_simplified"
DYNAMIC_BATCH = "dynamic_batch"

_PROFILES: dict[str, dict[str, Any]] = {
    STATIC_BATCH_1_SIMPLIFIED: {
        "profile": STATIC_BATCH_1_SIMPLIFIED,
        "batch_contract": "static_batch_1",
        "opset_version": 17,
        "simplified": True,
        "input_name": "obs",
        "output_name": "actions",
        "input_dtype": "float32",
        "output_dtype": "float32",
    },
    DYNAMIC_BATCH: {
        "profile": DYNAMIC_BATCH,
        "batch_contract": "dynamic_batch",
        "opset_version": 18,
        "simplified": False,
        "input_name": "obs",
        "output_name": "actions",
        "input_dtype": "float32",
        "output_dtype": "float32",
    },
}


class OnnxExportContractError(ValueError):
    """Raised when an ONNX export violates its declared contract."""


def available_onnx_export_profiles() -> tuple[str, ...]:
    """Return the supported explicit ONNX export profiles."""
    return tuple(_PROFILES)


def get_onnx_export_contract(profile: str) -> dict[str, Any]:
    """Return a detached copy of one immutable profile contract."""
    try:
        return dict(_PROFILES[profile])
    except (KeyError, TypeError) as exc:
        raise OnnxExportContractError(
            "unsupported ONNX export profile: " + repr(profile)
        ) from exc


def validate_onnx_export_contract(value: Any) -> dict[str, Any]:
    """Require a contract to exactly match a supported profile."""
    if not isinstance(value, dict):
        raise OnnxExportContractError("ONNX export contract must be an object")
    expected = get_onnx_export_contract(value.get("profile"))
    if value != expected:
        raise OnnxExportContractError("ONNX export contract differs from its profile")
    return dict(expected)


def _dimension(value: Any) -> int | str:
    if value.dim_param:
        return value.dim_param
    if value.HasField("dim_value"):
        return int(value.dim_value)
    raise OnnxExportContractError("ONNX tensor dimension is unspecified")


def _tensor_shape(value: Any) -> list[int | str]:
    tensor_type = value.type.tensor_type
    return [_dimension(dimension) for dimension in tensor_type.shape.dim]


def _inspect_model(model: Any, contract: dict[str, Any]) -> dict[str, Any]:
    import onnx

    onnx.checker.check_model(model)
    if len(model.graph.input) != 1 or len(model.graph.output) != 1:
        raise OnnxExportContractError("ONNX model must have exactly one input and output")
    model_input = model.graph.input[0]
    model_output = model.graph.output[0]
    if model_input.name != contract["input_name"]:
        raise OnnxExportContractError("ONNX input name differs from export contract")
    if model_output.name != contract["output_name"]:
        raise OnnxExportContractError("ONNX output name differs from export contract")
    if (
        model_input.type.tensor_type.elem_type != onnx.TensorProto.FLOAT
        or model_output.type.tensor_type.elem_type != onnx.TensorProto.FLOAT
    ):
        raise OnnxExportContractError("ONNX input and output must be float32")
    input_shape = _tensor_shape(model_input)
    output_shape = _tensor_shape(model_output)
    if len(input_shape) != 2 or len(output_shape) != 2:
        raise OnnxExportContractError("ONNX input and output must be rank-2 tensors")
    expected_batch: int | str = (
        1 if contract["batch_contract"] == "static_batch_1" else "batch"
    )
    if input_shape[0] != expected_batch or output_shape[0] != expected_batch:
        raise OnnxExportContractError("ONNX batch dimensions differ from export contract")
    if any(
        isinstance(size, bool) or not isinstance(size, int) or size <= 0
        for size in (input_shape[1], output_shape[1])
    ):
        raise OnnxExportContractError("ONNX feature dimensions must be fixed and positive")
    opsets = {
        item.domain: int(item.version)
        for item in model.opset_import
    }
    if opsets.get("") != contract["opset_version"]:
        raise OnnxExportContractError("ONNX opset differs from export contract")
    return {
        "input_shape": input_shape,
        "output_shape": output_shape,
        "node_count": len(model.graph.node),
    }


def export_onnx_policy(
    model: Any,
    example_input: Any,
    output_path: Path,
    *,
    contract: dict[str, Any],
) -> dict[str, Any]:
    """Export one policy and return evidence from the reloaded final model."""
    import onnx
    import torch

    contract = validate_onnx_export_contract(contract)
    if (
        not isinstance(example_input, torch.Tensor)
        or example_input.dtype != torch.float32
        or example_input.ndim != 2
        or example_input.shape[0] != 1
    ):
        raise OnnxExportContractError(
            "ONNX export example must be a rank-2 float32 tensor with batch 1"
        )
    dynamic_axes = None
    if contract["batch_contract"] == "dynamic_batch":
        dynamic_axes = {
            contract["input_name"]: {0: "batch"},
            contract["output_name"]: {0: "batch"},
        }
    torch.onnx.export(
        model,
        example_input,
        str(output_path),
        input_names=[contract["input_name"]],
        output_names=[contract["output_name"]],
        dynamic_axes=dynamic_axes,
        opset_version=contract["opset_version"],
    )
    exported_model = onnx.load(str(output_path))
    pre_simplify = _inspect_model(exported_model, contract)
    simplifier_check: bool | None = None
    if contract["simplified"]:
        from onnxsim import simplify

        simplified_model, simplifier_check = simplify(exported_model)
        if simplifier_check is not True:
            raise OnnxExportContractError("onnxsim validation failed")
        onnx.save(simplified_model, str(output_path))
    final_model = onnx.load(str(output_path))
    final = _inspect_model(final_model, contract)
    if final["input_shape"] != pre_simplify["input_shape"]:
        raise OnnxExportContractError("onnxsim changed the input shape")
    if final["output_shape"] != pre_simplify["output_shape"]:
        raise OnnxExportContractError("onnxsim changed the output shape")
    if final["node_count"] > pre_simplify["node_count"]:
        raise OnnxExportContractError("onnxsim increased the ONNX node count")
    return {
        "contract": contract,
        "input_shape": final["input_shape"],
        "output_shape": final["output_shape"],
        "pre_simplify_node_count": pre_simplify["node_count"],
        "post_simplify_node_count": final["node_count"],
        "simplifier_check": simplifier_check,
    }


def run_onnx_policy(
    path: Path,
    policy_input: Any,
    *,
    contract: dict[str, Any],
) -> Any:
    """Run ONNX parity, row-by-row when the deployment contract is batch 1."""
    import numpy as np
    import onnxruntime as ort
    import torch

    contract = validate_onnx_export_contract(contract)
    if not isinstance(policy_input, torch.Tensor) or policy_input.ndim != 2:
        raise OnnxExportContractError("ONNX parity input must be a rank-2 tensor")
    array = policy_input.detach().cpu().numpy()
    session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
    if contract["batch_contract"] == "static_batch_1":
        outputs = [
            session.run(
                [contract["output_name"]],
                {contract["input_name"]: array[index : index + 1]},
            )[0]
            for index in range(array.shape[0])
        ]
        output = np.concatenate(outputs, axis=0)
    else:
        output = session.run(
            [contract["output_name"]],
            {contract["input_name"]: array},
        )[0]
    return torch.from_numpy(np.asarray(output))
