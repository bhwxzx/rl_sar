---
name: monitor-tune-isaaclab-training
description: Assess IsaacLab training and bounded policy evaluations, compare and export selected checkpoints, and advise parameter changes from evidence. Also generate MuJoCo Sim2Sim test reports on a deployment-only computer, including full evidence packages with receiver-confirmed model-resource hash reuse. Use for human-guided assessment or finite Sim2Sim reporting, not autonomous training campaigns.
---

# IsaacLab Training Advisor

Help the operator assess one live or completed training run, evaluate bounded
policy behavior, compare checkpoints, export a selected policy, and learn from
feedback. Training decisions and parameter edits remain with the user.

## Select the mode first

- **sim2sim-report**: For MuJoCo/deployment test reports, including a computer
  used only to produce those reports, read [sim2sim-report.md](references/sim2sim-report.md).
  Follow that workflow and the shared authorization rules below. It supports
  analyzing existing evidence or running a user-specified finite test. It does
  not require IsaacLab, a local training checkout, training argv, or training YAML.
  Use the deployment environment already available there; packaging uses only
  Python 3.10+ standard library. Do not invoke the Isaac Native batch runner.
- **IsaacLab advisor**: Use the routing table and IsaacLab workflow below for
  training, Native evaluation, export, or interpreting returned feedback here.

The skill folder is portable as a whole. Resolve its references and scripts from
its installed location, not from an assumed robot_lab checkout. An installation
used for `sim2sim-report` need not run or install the other capabilities.

## Route the task

Read only the references needed for the current action:

| Task | Read before acting |
| --- | --- |
| Establish or reuse run provenance | [run-identity.md](references/run-identity.md), [effective-training-config.md](references/effective-training-config.md) |
| Plan or summarize evaluations | [evidence-layout.md](references/evidence-layout.md), [human-guided-training-advisor.md](references/human-guided-training-advisor.md) |
| Monitor training or judge convergence | [human-guided-training-advisor.md](references/human-guided-training-advisor.md), [assessment-criteria-contract.md](references/assessment-criteria-contract.md) |
| Select, export, archive or replace a policy | [policy-export.md](references/policy-export.md) |
| Interpret historical feedback or propose parameters | [experience-query.md](references/experience-query.md) |

Resolve the most specific profile in `references/algorithm-profiles.json`.
A generic profile may parse progress but cannot supply missing algorithm-specific
observation, normalization, history, reset or deployment contracts.
Use `conda run -n isaacsim-5.1` for IsaacLab/RSL-RL commands and their validation;
this does not apply to the independent Sim2Sim mode or its packaging tests.

## Preserve identity and evidence

- Verify the actual task/run, backend, algorithm, runner, seed, log/checkpoint
  paths, branch/HEAD, dirty relevant source, process and GPU before acting.
- Reuse a previously user-chosen host ID and previously supplied exact argv;
  do not ask again when the session already establishes them. Never reconstruct
  missing exact argv from configuration guesses. Preserve ordered Hydra overrides.
- Capture effective `params/env.yaml` and `params/agent.yaml` before interpreting
  parameters. Logs, TensorBoard and W&B are activity/metric evidence.
- For new evaluation batches use reusable context v2 and effective config v2.
  Store identical verified content once; changed context/source gets a new hash.
  Each result binds context, config and its own scenario/evaluator-source evidence.
- Keep old evidence in place. Read legacy formats through their validators.
  Never weaken hashes, source scope or required telemetry for compatibility.

## Evaluate a bounded question

Use `scripts/run_evaluation_batch.py` for a finite list of explicitly supplied
Native cases. The operator's authorized question defines the cases and budget;
the runner never generates cases or launches training. It requires an idle GPU,
executes cases sequentially, retains one console log per attempt, publishes
lossless `telemetry.json.gz`, and seals one `manifest.json` and one `report.md`.
One batch creates one `evaluation_batch` event. Retries use new batch/attempt IDs.
Use `scripts/summarize_evaluation_batch.py` to group already completed evaluations
without moving or rewriting them. Read the run's `index.md` first; it is mutable
navigation, never hash-bound evidence. Finished reports and manifests are immutable.

Start video work with a short Native robot-in-frame smoke test. A terrain-only
video is not motion evidence. Only generate plots when they answer the question.
For turns, distinguish body yaw-rate tracking from accumulated world heading;
state averaging windows, resets and command transitions. One seed cannot prove a
universal speed threshold. Missing contact telemetry cannot establish foot contact.

For training overlap, use the existing single-evaluation route with explicit
session authorization, Native only, normally one environment, at most 2,000 steps,
and no video. Record pre/post training progress and throughput. Stop only the
evaluation if it interferes; never restart or signal training automatically.

Require bundle validation before using a result. `result.json` is published last;
partial work files are not completed evaluations. Inspect `telemetry_status`,
`missing_required_signals`, `signal_status` and `metric_availability`. AMP-ROA needs
complete required telemetry for complete assessment or Pareto eligibility.
Missing signals are unknown, never zero. Simulation evidence is not hardware readiness.

## Assess and advise

Training health needs two identity-compatible observations and monotonic log or
TensorBoard steps. GPU utilization and file timestamps alone cannot prove progress.
Use the health and assessment scripts described in the advisor reference.
Missing, unapproved or scope/hash-invalid criteria force `insufficient_evidence`
and `indeterminate`; never equate reward, elapsed training or normal completion
with convergence. Never invent acceptance thresholds.

Compare checkpoints under matching scenarios, duration, seed, environment count
and approved criteria. Show alternatives and uncertainty; export only the user's
selected checkpoint. Require Native/JIT/ONNX parity at multiple times including
before and after reset. Preserve AMP-ROA time-major history, current-frame-only
normalization and actor input `[current_obs, code_vel, hist_latent]`.

For feedback, first check export/tensor/reset contracts, runtime differences and
physical timing/calibration before proposing training changes. State the current
value, proposed change, expected effect, counter-metric, risk and compatible
historical evidence. Keep `direct_parameter_change_supported` false.

## Authorization and final response

Do not start/stop/resume training, edit parameters, run adaptive campaigns, deploy,
install packages, delete user files, or commit/push repositories without applicable
user authorization. Preserve unrelated dirty files. Authorization already supplied
in the session is sufficient; ask only for a missing decision.

Before archiving a selected policy into policy_storage, follow the exact manifest, clean-storage, fast-forward pull,
collision and duplicate checks in `policy-export.md`. Replacement requires separate
approval binding all four existing hashes. Archive authorization does not authorize
Git commit/push. Every policy description must state:

> 仅可进入受监督实物测试；未经实物验证，不代表 hardware-ready。

Report the finding, limitations, next bounded action and links to the index/report.
Do not create a second prose report, per-case summary files, or a new event merely
to restate the same batch. Preserve raw evidence; a later fact belongs in a new batch.
