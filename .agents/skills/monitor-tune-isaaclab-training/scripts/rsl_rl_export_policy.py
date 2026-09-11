#!/usr/bin/env python3
"""Export one RSL-RL checkpoint to JIT/ONNX and prove open-loop parity."""

from __future__ import annotations

import argparse
import atexit
import hashlib
import subprocess
import sys
from collections.abc import Mapping
from pathlib import Path
from typing import Any


ORIGINAL_ARGV = tuple(sys.argv)
REPO_ROOT = Path(__file__).resolve().parents[4]
RSL_SCRIPT_DIR = REPO_ROOT / "scripts" / "reinforcement_learning" / "rsl_rl"
sys.path.insert(0, str(RSL_SCRIPT_DIR))

from policy_export_evidence import (  # noqa: E402
    ExportPublisher,
    PolicyExportEvidenceError,
    close_export_resources,
    preflight_export,
)
from onnx_export_contract import (  # noqa: E402
    available_onnx_export_profiles,
    export_onnx_policy,
    run_onnx_policy,
)

from isaaclab.app import AppLauncher

import cli_args  # isort: skip  # noqa: E402


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--task", required=True)
parser.add_argument("--agent", default="rsl_rl_cfg_entry_point")
parser.add_argument("--run_id", required=True)
parser.add_argument("--checkpoint_id", required=True)
parser.add_argument("--checkpoint_sha256", required=True)
parser.add_argument("--export_run_id", required=True)
parser.add_argument("--selection_receipt_path", required=True)
parser.add_argument("--selection_receipt_sha256", required=True)
parser.add_argument("--jit_path", required=True)
parser.add_argument("--onnx_path", required=True)
parser.add_argument("--result_path", required=True)
parser.add_argument("--max_abs_action_error", type=float, required=True)
parser.add_argument("--minimum_parity_samples", type=int, required=True)
parser.add_argument("--history_contract", required=True)
parser.add_argument("--normalization_contract", required=True)
parser.add_argument("--reset_contract", required=True)
parser.add_argument(
    "--onnx_export_profile",
    required=True,
    choices=available_onnx_export_profiles(),
)
parser.add_argument("--parity_steps", type=int, required=True)
parser.add_argument("--reset_step", type=int, required=True)
parser.add_argument("--num_envs", type=int, default=16)
parser.add_argument("--seed", type=int, required=True)
parser.add_argument("--require_idle_gpu", action="store_true")
cli_args.add_rsl_rl_args(parser)
if "-h" in ORIGINAL_ARGV or "--help" in ORIGINAL_ARGV:
    parser.print_help()
    raise SystemExit(0)
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()


def _bytes_sha256(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _assert_gpu_idle(device: str | None) -> None:
    if not args_cli.require_idle_gpu or device is None or not device.startswith("cuda"):
        return
    gpu_index = device.split(":", 1)[1] if ":" in device else "0"
    result = subprocess.run(
        [
            "nvidia-smi",
            "-i",
            gpu_index,
            "--query-compute-apps=pid",
            "--format=csv,noheader,nounits",
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError("cannot verify that the export GPU is idle")
    active = [
        line.strip() for line in result.stdout.splitlines()
        if line.strip().isdigit()
    ]
    if active:
        raise RuntimeError(
            "policy export GPU is not idle; active PID(s): "
            + ", ".join(active)
        )


try:
    export_plan = preflight_export(
        task=args_cli.task,
        run_id=args_cli.run_id,
        checkpoint_id=args_cli.checkpoint_id,
        checkpoint_path=Path(args_cli.checkpoint or ""),
        checkpoint_sha256=args_cli.checkpoint_sha256,
        export_id=args_cli.export_run_id,
        selection_receipt_path=Path(args_cli.selection_receipt_path),
        selection_receipt_sha256=args_cli.selection_receipt_sha256,
        jit_path=Path(args_cli.jit_path),
        onnx_path=Path(args_cli.onnx_path),
        receipt_path=Path(args_cli.result_path),
        history_contract=args_cli.history_contract,
        normalization_contract=args_cli.normalization_contract,
        reset_contract=args_cli.reset_contract,
        onnx_export_profile=args_cli.onnx_export_profile,
        parity_steps=args_cli.parity_steps,
        reset_step=args_cli.reset_step,
        minimum_parity_samples=args_cli.minimum_parity_samples,
        max_abs_action_error=args_cli.max_abs_action_error,
        num_envs=args_cli.num_envs,
        seed=args_cli.seed,
    )
    _assert_gpu_idle(args_cli.device)
    export_publisher = ExportPublisher(export_plan)
    export_publisher.__enter__()
except (PolicyExportEvidenceError, RuntimeError) as exc:
    parser.error(str(exc))
atexit.register(export_publisher.close)

checkpoint_path = Path(export_plan.checkpoint["path"])
jit_path = export_publisher.jit_work_path
onnx_path = export_publisher.onnx_work_path
result_path = export_plan.paths["receipt"]

sys.argv = [sys.argv[0]] + hydra_args
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402
import torch.nn as nn  # noqa: E402

from rsl_rl.runners import (  # noqa: E402
    DistillationRunner,
    OnPolicyRunner,
    OnPolicyRunnerAmp,
    OnPolicyRunnerAmpDwaq,
    OnPolicyRunnerAmpROA,
    OnPolicyRunnerDwaq,
    OnPolicyRunnerROA,
)

from isaaclab.envs import (  # noqa: E402
    DirectMARLEnv,
    DirectMARLEnvCfg,
    DirectRLEnvCfg,
    ManagerBasedRLEnvCfg,
    multi_agent_to_single_agent,
)
from isaaclab_rl.rsl_rl import (  # noqa: E402
    RslRlBaseRunnerCfg,
    RslRlVecEnvWrapper,
)
from isaaclab_tasks.utils.hydra import hydra_task_config  # noqa: E402

import robot_lab.tasks  # noqa: F401,E402


RUNNER_CLASSES = {
    "OnPolicyRunner": OnPolicyRunner,
    "DistillationRunner": DistillationRunner,
    "OnPolicyRunnerDwaq": OnPolicyRunnerDwaq,
    "OnPolicyRunnerAmp": OnPolicyRunnerAmp,
    "OnPolicyRunnerAmpDwaq": OnPolicyRunnerAmpDwaq,
    "OnPolicyRunnerROA": OnPolicyRunnerROA,
    "OnPolicyRunnerAmpROA": OnPolicyRunnerAmpROA,
}


class DWAQDeploymentWrapper(nn.Module):
    def __init__(self, policy: Any, num_obs: int):
        super().__init__()
        self.encoder_backbone = policy.encoder_backbone
        self.encode_mean_vel = policy.encode_mean_vel
        self.encode_mean_latent = policy.encode_mean_latent
        self.actor = policy.actor
        self.actor_obs_normalizer = policy.actor_obs_normalizer
        self.num_obs = num_obs

    def forward(self, history: torch.Tensor) -> torch.Tensor:
        current = history[:, -self.num_obs:]
        features = self.encoder_backbone(history)
        latent = torch.cat(
            (
                self.encode_mean_vel(features),
                self.encode_mean_latent(features),
            ),
            dim=-1,
        )
        actor_input = torch.cat((latent, current), dim=-1)
        if self.actor_obs_normalizer is not None:
            actor_input = self.actor_obs_normalizer(actor_input)
        return self.actor(actor_input)


class ROADeploymentWrapper(nn.Module):
    def __init__(self, policy: Any, num_obs: int):
        super().__init__()
        self.history_encoder = policy.history_encoder
        self.actor = policy.actor
        self.actor_obs_normalizer = policy.actor_obs_normalizer
        self.num_obs = num_obs

    def forward(self, history: torch.Tensor) -> torch.Tensor:
        current = history[:, -self.num_obs:]
        if self.actor_obs_normalizer is not None:
            current = self.actor_obs_normalizer(current)
        hist_latent, code_vel = self.history_encoder(history)
        return self.actor(torch.cat((current, code_vel, hist_latent), dim=-1))


class StandardDeploymentWrapper(nn.Module):
    def __init__(self, policy: Any):
        super().__init__()
        if hasattr(policy, "actor"):
            self.actor = policy.actor
        elif hasattr(policy, "student"):
            self.actor = policy.student
        else:
            raise RuntimeError("policy has neither actor nor student module")
        self.normalizer = getattr(
            policy,
            "actor_obs_normalizer",
            getattr(policy, "student_obs_normalizer", None),
        )

    def forward(self, observations: torch.Tensor) -> torch.Tensor:
        if self.normalizer is not None:
            observations = self.normalizer(observations)
        return self.actor(observations)


def _policy_tensor(observations: Any) -> torch.Tensor:
    if isinstance(observations, Mapping):
        if "policy" not in observations:
            raise RuntimeError("observation mapping does not contain policy")
        tensor = observations["policy"]
    else:
        tensor = observations
    if not isinstance(tensor, torch.Tensor):
        raise RuntimeError("policy observation is not a tensor")
    return tensor


def _cpu_observation_batch(observations: Any, sample_count: int) -> Any:
    if isinstance(observations, torch.Tensor):
        return observations[:sample_count].detach().cpu()
    if isinstance(observations, Mapping):
        return {
            key: _cpu_observation_batch(value, sample_count)
            for key, value in observations.items()
        }
    raise RuntimeError(
        "observation batch contains an unsupported non-tensor value"
    )


def _concatenate_observation_batches(batches: list[Any]) -> Any:
    if not batches:
        raise RuntimeError("parity observation batches are empty")
    first = batches[0]
    if isinstance(first, torch.Tensor):
        if any(not isinstance(batch, torch.Tensor) for batch in batches):
            raise RuntimeError("parity observation batch types changed")
        return torch.cat(batches, dim=0)
    if isinstance(first, Mapping):
        keys = set(first)
        if any(not isinstance(batch, Mapping) or set(batch) != keys for batch in batches):
            raise RuntimeError("parity observation mapping keys changed")
        return {
            key: _concatenate_observation_batches([batch[key] for batch in batches])
            for key in sorted(keys)
        }
    raise RuntimeError("parity observations contain an unsupported value")


def _action_tensor(value: Any) -> torch.Tensor:
    if isinstance(value, (tuple, list)):
        value = value[0]
    if not isinstance(value, torch.Tensor):
        raise RuntimeError("policy action is not a tensor")
    return value


def _tensor_digest(value: torch.Tensor) -> str:
    array = value.detach().cpu().contiguous().numpy()
    return _bytes_sha256(array.tobytes())


@hydra_task_config(args_cli.task, args_cli.agent)
def main(
    env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg,
    agent_cfg: RslRlBaseRunnerCfg,
) -> None:
    agent_cfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.seed = args_cli.seed
    env_cfg.sim.device = (
        args_cli.device if args_cli.device is not None else env_cfg.sim.device
    )
    raw_env = gym.make(args_cli.task, cfg=env_cfg)
    if isinstance(raw_env.unwrapped, DirectMARLEnv):
        raw_env = multi_agent_to_single_agent(raw_env)
    env = RslRlVecEnvWrapper(raw_env, clip_actions=agent_cfg.clip_actions)
    try:
        runner_class = RUNNER_CLASSES.get(agent_cfg.class_name)
        if runner_class is None:
            raise RuntimeError(f"unsupported RSL-RL runner: {agent_cfg.class_name}")
        runner = runner_class(
            env,
            agent_cfg.to_dict(),
            log_dir=None,
            device=agent_cfg.device,
        )
        runner.load(str(checkpoint_path))
        native_policy = runner.get_inference_policy(
            device=env.unwrapped.device
        )
        try:
            policy_module = runner.alg.policy
        except AttributeError:
            policy_module = runner.alg.actor_critic
        runner_name = agent_cfg.class_name
        if (
            "ROA" not in runner_name
            and "Dwaq" not in runner_name
            and bool(getattr(policy_module, "is_recurrent", False))
        ):
            raise RuntimeError(
                "automatic recurrent policy export requires a reviewed "
                "state-reset adapter"
            )
        observations = env.get_observations()
        cpu_observation_batches: list[Any] = []
        policy_input_batches: list[torch.Tensor] = []
        native_device_batches: list[torch.Tensor] = []
        boundaries: list[dict[str, Any]] = []
        actual_history_contract = ""
        policy_frame_width = 0
        for step in range(args_cli.parity_steps):
            if step == args_cli.reset_step:
                reset_result = env.reset()
                observations = (
                    reset_result[0]
                    if isinstance(reset_result, tuple)
                    else reset_result
                )
                if hasattr(policy_module, "reset"):
                    policy_module.reset(
                        torch.ones(
                            args_cli.num_envs,
                            dtype=torch.bool,
                            device=env.unwrapped.device,
                        )
                    )
            policy_observation = _policy_tensor(observations)
            step_history_contract = (
                "flat_time_major_history"
                if policy_observation.ndim > 2
                else "current_observation"
            )
            if not actual_history_contract:
                actual_history_contract = step_history_contract
                policy_frame_width = int(policy_observation.shape[-1])
            if step_history_contract != actual_history_contract:
                raise RuntimeError("policy observation history rank changed during parity")
            if actual_history_contract != args_cli.history_contract:
                raise RuntimeError(
                    "actual policy observation history contract differs from approval"
                )
            policy_input = (
                policy_observation.flatten(1)
                if policy_observation.ndim > 2
                else policy_observation
            )
            with torch.no_grad():
                native_device_action = _action_tensor(native_policy(observations))
            if policy_input.shape[0] != native_device_action.shape[0]:
                raise RuntimeError("policy input/action batch sizes differ")
            cpu_observation_batches.append(
                _cpu_observation_batch(observations, policy_input.shape[0])
            )
            cpu_input = policy_input.detach().cpu()
            cpu_action = native_device_action.detach().cpu()
            policy_input_batches.append(cpu_input)
            native_device_batches.append(cpu_action)
            labels: list[str] = []
            if step == 0:
                labels.append("initial")
            if step == args_cli.reset_step - 1:
                labels.append("pre_reset")
            if step == args_cli.reset_step:
                labels.append("post_reset")
            if step == args_cli.parity_steps - 1:
                labels.append("final")
            for label in labels:
                boundaries.append(
                    {
                        "label": label,
                        "step": step,
                        "observation_sha256": _tensor_digest(cpu_input),
                        "native_output_sha256": _tensor_digest(cpu_action),
                        "input_shape": list(cpu_input.shape),
                        "output_shape": list(cpu_action.shape),
                    }
                )
            if step < args_cli.parity_steps - 1 and step + 1 != args_cli.reset_step:
                observations, _, _, _ = env.step(native_device_action)

        policy_input = torch.cat(policy_input_batches, dim=0)
        native_device_action = torch.cat(native_device_batches, dim=0)
        cpu_observations = _concatenate_observation_batches(cpu_observation_batches)
        sample_count = int(policy_input.shape[0])
        if sample_count < args_cli.minimum_parity_samples:
            raise RuntimeError(
                "bounded temporal window does not cover minimum parity samples"
            )
        policy_module.cpu()
        policy_module.eval()
        with torch.no_grad():
            native_action = _action_tensor(native_policy(cpu_observations))
        native_action = native_action.detach().cpu()
        if native_device_action.shape != native_action.shape:
            raise RuntimeError(
                "Native device and CPU action shapes differ"
            )
        native_device_to_cpu_error = float(
            torch.max(
                torch.abs(native_device_action - native_action)
            ).item()
        )
        if "ROA" in runner_name:
            if actual_history_contract != "flat_time_major_history":
                raise RuntimeError("ROA export requires a time-major history")
            deployment_model: nn.Module = ROADeploymentWrapper(
                policy_module,
                policy_frame_width,
            ).eval()
            normalization_contract = "current_frame_only"
        elif "Dwaq" in runner_name:
            if actual_history_contract != "flat_time_major_history":
                raise RuntimeError("DWAQ export requires a time-major history")
            deployment_model = DWAQDeploymentWrapper(
                policy_module,
                policy_frame_width,
            ).eval()
            normalization_contract = "combined_actor_input"
        else:
            deployment_model = StandardDeploymentWrapper(policy_module).eval()
            normalization_contract = "backend_export_helper"
        if normalization_contract != args_cli.normalization_contract:
            raise RuntimeError(
                "actual normalization contract differs from approval"
            )
        example_input = policy_input[:1]
        traced = torch.jit.trace(deployment_model, example_input)
        traced.save(str(jit_path))
        onnx_export_evidence = export_onnx_policy(
            deployment_model,
            example_input,
            onnx_path,
            contract=export_plan.onnx_export_contract,
        )
        jit_model = torch.jit.load(str(jit_path), map_location="cpu").eval()
        with torch.no_grad():
            jit_action = _action_tensor(jit_model(policy_input)).cpu()
        onnx_action = run_onnx_policy(
            onnx_path,
            policy_input,
            contract=export_plan.onnx_export_contract,
        ).cpu()
        artifact_parity: dict[str, Any] = {}
        for kind, path, action in (
            ("jit", jit_path, jit_action),
            ("onnx", onnx_path, onnx_action),
        ):
            finite = bool(torch.isfinite(action).all().item())
            if action.shape != native_action.shape:
                raise RuntimeError(f"{kind} action shape differs from Native")
            error = float(torch.max(torch.abs(action - native_action)).item())
            if not finite or error > args_cli.max_abs_action_error:
                raise RuntimeError(
                    f"{kind} action parity gate failed: "
                    f"finite={finite}, max_abs_action_error={error:.9g}, "
                    f"limit={args_cli.max_abs_action_error:.9g}"
                )
            artifact_parity[kind] = {
                "input_shape": list(policy_input.shape),
                "output_shape": list(action.shape),
                "finite": finite,
                "max_abs_action_error": error,
            }
        result = {
            "version": 4,
            "status": "completed",
            "export": {
                "task": export_plan.task,
                "run_id": export_plan.run_id,
                "export_id": export_plan.export_id,
                "checkpoint_id": args_cli.checkpoint_id,
                "runner": runner_name,
            },
            "inputs": {
                "checkpoint": export_plan.checkpoint,
                "checkpoint_selection": {
                    "path": str(args_cli.selection_receipt_path),
                    "sha256": args_cli.selection_receipt_sha256,
                    "selection_id": export_plan.selection_receipt["selection_id"],
                },
                "run_identity": export_plan.selection_receipt["run_identity"],
                "effective_config": export_plan.selection_receipt[
                    "effective_config"
                ],
                "tensor_contract": export_plan.tensor_contract,
                "onnx_export_contract": export_plan.onnx_export_contract,
                "parity_contract": export_plan.parity_contract,
            },
            "onnx_export": onnx_export_evidence,
            "parity": {
                "sample_count": sample_count,
                "boundaries": boundaries,
                "observation_batch_sha256": _tensor_digest(policy_input),
                "native_output_sha256": _tensor_digest(native_action),
                "native_device_to_cpu_max_abs_action_error": (
                    native_device_to_cpu_error
                ),
                "history_contract": actual_history_contract,
                "normalization_contract": normalization_contract,
                **artifact_parity,
            },
        }
        export_publisher.publish(result)
        print(f"[INFO] Export receipt: {result_path}")
    finally:
        env.close()


try:
    main()
finally:
    close_export_resources(export_publisher, simulation_app)
