# Human-guided IsaacLab training advisor

This workflow uses bounded evidence and user decisions. It does not depend on
autonomous campaign control or distributed coordination.

## Evidence order

Use evidence in this order:

1. monotonic training iteration, epoch, or TensorBoard scalar step;
2. finite short-, medium-, and long-window training metrics;
3. explicit completion or failure output;
4. Native closed-loop metrics and bounded telemetry from an exact checkpoint;
5. motion video that has passed a robot-in-frame check;
6. process, GPU, log mtime, checkpoint mtime, and W&B activity as auxiliary
   evidence only.

Never allow a lower-ranked source to overrule stale or non-finite monotonic
training evidence.

## Evidence paths and immutability

Follow [evidence-layout.md](evidence-layout.md). New evaluations use one batch per
bounded question: one immutable manifest/report, raw result + telemetry + optional
video and one console log per attempt. Summarize six related turning cases in one
comparison report and one v5 batch event, without six prose reports or duplicate
config dumps. Rebuild only `index.md`; add later facts under a new batch ID.
Existing raw evidence and v1-v4 events remain at their original paths.

A report states checkpoint/context/config identity; exact scenario overrides and
command schedule; scenario comparison metrics; selected plots/videos; limitations;
and recommendations or missing authorization. Include failed/not-run attempts.
For turning diagnosis use both body yaw rate and accumulated world heading with
explicit windows and resets. Avoid treating a transient spike or one seed as a
universal failure/threshold. Do not infer contact without measured contact signals.

## Host-local source identity

Follow [`run-identity.md`](run-identity.md) and run
`capture_run_identity.py` independently on each host. Record a user-controlled
host ID, local branch and full HEAD, exact training argv, ordered Hydra
overrides, relevant training configuration hashes. For new batches, store the canonical evaluator scenario
and its source snapshot separately; legacy v1 retains its scenario fingerprint. A dirty relevant source requires exactly one tracked diff hash or
controlled patch evidence. These records provide provenance only; they never
authorize remote Git writes or coordinate work between hosts.

## Two-snapshot health check

The first valid log or TensorBoard observation records a baseline and returns
`observing`; recent TensorBoard wall time does not make it `healthy`. On a later
check, pass the absolute path of the prior complete health JSON with
`--previous-health`. The collector verifies the same profile, log path, PID,
an earlier observation time, and at least one comparable progress source.

Classify the comparison as follows:

- `healthy`: a comparable log or TensorBoard step increased monotonically;
- `suspect`: comparable progress is unchanged but the stale interval is not
  confirmed, or auxiliary activity cannot prove progress;
- `stalled`: progress stayed unchanged for the configured stale duration while
  the expected process remains alive, regardless of GPU utilization;
- `unknown`: progress regressed, sources disagree, process identity mismatches,
  or the previous snapshot is not identity-compatible;
- `completed`: current profile progress reached its target;
- `stopped`: the identified process ended without confirmed completion.

A confirmed stall with GPU utilization above the configured low-utilization
threshold records `activity_without_progress: true`; that activity is auxiliary
and cannot override unchanged monotonic progress. Only a confirmed stall with
available low-GPU evidence may set `auto_recovery_candidate: true`. The marker
does not authorize or perform termination, signaling, restart, or resume.

The output includes `comparison` for the decision evidence and
`baseline_for_next_check` for the values captured by the current snapshot.
The explicit `--previous-log-progress`, `--previous-tensorboard-step`, and
`--previous-observed-at` options remain available for compatibility, but do not
provide the full profile, log, and PID identity validation of
`--previous-health`.

## Assessment criteria

`assess_training_run.py` consumes version-2 criteria. Use the numberless draft
at `assets/assessment-criteria-template.json` and follow
[`assessment-criteria-contract.md`](assessment-criteria-contract.md). Never
invent task thresholds or reuse an approval from another run.

The decision-bearing `contract` binds task, run, backend, profile, algorithm,
and runner; defines adjacent windows and required metrics; separates reporting-
only observed metrics from hard failures; and makes Play gates mandatory for a
convergence claim. The separate `approval` receipt records a timezone-aware
approval time and the canonical SHA-256 of the exact contract the user
approved. Editing the contract invalidates the receipt.

For each metric, compare the preceding window with the latest window. Relative
improvement is positive when movement follows the configured direction. A
metric is plateaued when the absolute relative change does not exceed its
tolerance. Only required metrics affect trend decisions. Observed metrics are
reported separately and cannot affect recommendations. Explicit approved hard
failures dominate plateau logic.

Missing criteria, a draft, malformed approval, contract hash mismatch, or any
scope mismatch forces `insufficient_evidence` and `indeterminate`. Non-finite
metrics and a confirmed stall remain visible as safety alerts, but cannot cause
a strong stop recommendation without the matching approved contract.

## Decision meanings

- `continue`: process evidence is `healthy` and at least one required metric
  has meaningful positive improvement without a hard failure. No other health
  state may produce this recommendation.
- `continue_and_recheck`: progress is `observing` or `suspect`, or the run is
  healthy but required trend or Play evidence is incomplete or mixed.
- `consider_stop_plateau`: enough adjacent windows exist and the configured
  number of required metrics are plateaued without meaningful improvement.
- `recommend_stop_invalid`: an explicitly approved non-finite, health-state, or
  training-metric hard failure.
- `insufficient_evidence`: progress is `unknown` or `stopped`, or the profile,
  metric direction, records, process identity, or criteria approval is
  unresolved.

Play gates qualify convergence rather than acting as an implicit stop rule. A
completed plateau with failed Play gates is `plateaued_with_defects`; missing
Play evidence is `indeterminate`. The tool emits advice only and never signals
a process.

## Training-overlap Play budget

When the user allows a short Play check during training:

- use one environment by default and never more than four;
- use at most 2,000 steps;
- evaluate Native only;
- disable video unless motion evidence is needed;
- record env-0 telemetry only and use a configurable stride;
- check GPU memory and training step before and after;
- stop the evaluator if it causes OOM, a material throughput drop, or training
  progress interruption; do not signal training.

The exact Native command uses the checkpoint for both artifact paths and hashes:

```bash
conda run -n isaacsim-5.1 python \
  scripts/reinforcement_learning/rsl_rl/evaluate_policy.py \
  --task EXACT_TASK --headless --device cuda:0 \
  --artifact_kind native \
  --checkpoint /absolute/model_N.pt \
  --checkpoint_sha256 SHA256 \
  --artifact_path /absolute/model_N.pt \
  --artifact_sha256 SHA256 \
  --candidate_id model-N --scenario_id quick-native \
  --duration_steps 500 --num_envs 1 --seed 42 \
  --allow_training_overlap --no_video \
  --run_id "$RUN_ID" --evaluation_id "$EVALUATION_ID" \
  --run_identity_path "$SOURCE_IDENTITY_PATH" \
  --run_identity_file_sha256 "$SOURCE_IDENTITY_FILE_SHA256" \
  --result_path "$PLAY_RESULT_PATH" \
  --telemetry_path "$TELEMETRY_PATH"
```

Telemetry contains the selected environment's command, reward, done/timeout,
base linear and angular velocity, projected gravity, root position, joint
position and velocity, applied torque, and action. Version 2 records, for every
signal, `required`, `available`, `complete`, `sample_count`,
`expected_sample_count`, `error_count`, and the first bounded `error`.
The published telemetry document is version 3 because it also repeats the
immutable evaluation/input binding; the individual signal-status contract is
unchanged.
Unavailable sample fields are `null`; no missing metric or signal may be
substituted with zero, reward, or another signal.

Overall `telemetry_status` is:

- `complete`: every runner-required signal has every expected sample;
- `partial`: at least one required signal has data but the required set is
  incomplete;
- `unavailable`: no required signal was captured;
- `not_requested`: no telemetry output was requested.

`missing_required_signals` lists every required signal that is incomplete, not
only signals with zero samples. Optional failures remain visible in
`signal_status` without changing the overall required-signal status.
`metric_availability` separately reports the sources of derived tracking,
tilt, joint-velocity, and torque metrics. A derived metric is omitted from
`metrics` unless every source was complete.

All runners require command, reward, done, timeout, and action for a complete
telemetry sample. `OnPolicyRunnerAmpROA` additionally requires joint names,
root linear and angular velocity, projected gravity, joint position and
velocity, and applied torque. If any AMP-ROA required signal is incomplete,
passing Play gates cannot produce `converged`; report `indeterminate` and the
missing signals. The checkpoint comparator must return `evaluation_required`
instead of a unique Pareto recommendation for such evidence.

These statuses describe simulation evidence only. Even complete Sim2Sim
telemetry cannot establish hardware readiness; only supervised physical tests
with their own telemetry can support that decision.

## Checkpoint comparison

First inventory regular `model_N.pt` files and reject files that are too recent
to be considered stable. Shortlisting is evidence selection, not final policy
selection. Include:

- the newest stable checkpoint;
- checkpoints nearest the best available configured training metrics;
- one or more checkpoints spanning the plateau region.

Run every shortlisted checkpoint under an identical evaluation contract. When
evaluation results are supplied, compare configured metrics by Pareto
dominance. A checkpoint dominates another only when it is no worse on every
available metric and strictly better on at least one. Missing required metrics
make a checkpoint ineligible. Results marked as requiring complete telemetry
are also ineligible while `telemetry_status` is not `complete`; the comparison
reports their status and missing signals under `incomplete_telemetry`.

For each matrix cell, allocate a fresh evaluation ID with the standard evidence
layout and invoke `evaluate_policy.py` with the exact `PLAY_RESULT_PATH`, optional
`TELEMETRY_PATH`/`VIDEO_PATH`, `SOURCE_IDENTITY_PATH`, and the source identity's
full-file SHA-256. Keep `--scenario_id`, `--scenario_overrides_json`,
`--command_schedule_json`, `--duration_steps`, `--num_envs`, and `--seed`
identical to the identity's canonical scenario contract. The evaluator publishes
the output bundle once and writes `result.json` last; an existing or partial
evaluation directory is not permission to retry with the same ID.

Before using a result for Pareto comparison, revalidate its version-2 binding:
checkpoint and artifact path/SHA-256, source-identity file and internal hashes,
scenario contract/fingerprint, resource mode, and telemetry/video output hashes.
Telemetry is acceptable only when its repeated `evaluation` and `inputs`
bindings exactly match the result. A missing completion marker, changed input,
hash mismatch, or output mismatch makes the cell ineligible rather than a zero
or failed metric.

After presenting the comparison, wait for the user to choose one stable
checkpoint. Allocate a fresh selection ID and export ID. Record the choice with
`policy_export_evidence.py record-selection`; bind the immutable comparison
report, checkpoint filename/iteration/hash, complete run identity, effective
configuration, every supporting evaluation bundle, and the reviewed
runner-specific tensor contract. Do not infer selection from a unique Pareto
recommendation.

Run `rsl_rl_export_policy.py` only under a separate export authorization. Pass
the approved selection receipt and hash, matching source/checkpoint/tensor
fields, canonical export paths, and a bounded temporal parity contract with an
explicit reset step and ONNX export profile. Treat only a new version-4 receipt
that passes `policy_export_evidence.py validate-export` as complete. Validated
legacy version-3 receipts remain eligible; JIT/ONNX files without a validated
receipt are partial attempt artifacts and are ineligible for use or archive.
See [`policy-export.md`](policy-export.md) for the exact protocol.

## Archive manifest

`archive_advised_policy.py` consumes a version-2 JSON object after separate
user archive authorization:

```json
{
  "version": 2,
  "archive_authorized": true,
  "storage_root": "/absolute/policy_storage",
  "collection": "LW/leg_loco",
  "task": "exact task",
  "algorithm": "AMP-ROA",
  "runner": "OnPolicyRunnerAmpROA",
  "selected_checkpoint": {
    "path": "/absolute/model_N.pt",
    "sha256": "64 hex characters",
    "iteration": 50000
  },
  "artifacts": {
    "jit": {"path": "/absolute/policy.pt", "sha256": "64 hex characters"},
    "onnx": {"path": "/absolute/policy.onnx", "sha256": "64 hex characters"}
  },
  "source": {"commit": "full commit", "dirty": true},
  "parameters": {"reward.path": -0.15},
  "evaluation": {
    "results": [
      {"path": "/absolute/result.json", "sha256": "64 hex characters"}
    ]
  },
  "export_receipt": {
    "path": "/absolute/evidence/export/export-001/receipt.json",
    "sha256": "64 hex characters"
  },
  "description_notes": "User-approved notes"
}
```

After the user approves that exact manifest and immediately before calling the
archiver, re-resolve the storage Git root, upstream, HEAD, worktree, and index.
Require a completely clean checkout, including no untracked paths, then run
`git -C "$STORAGE_ROOT" pull --ff-only`. Recheck HEAD, worktree, index,
destination collision, and duplicate JIT/ONNX pairs against the updated
checkout. Stop without archiving if the upstream is missing, the pull fails or
cannot fast-forward, the repository is dirty, or a target or duplicate appears.
Never stash, merge, rebase, reset, checkout, clean, or resolve storage state
automatically.

The archiver first revalidates the export receipt, selection receipt, complete
run identity, effective configuration, JIT/ONNX hashes, parity evidence, and
every closed-loop evaluation bundle. It requires repeated manifest fields to
match that evidence. It then verifies a clean storage Git worktree, existing
non-symlinked collection, duplicate artifact pairs, and destination collision.
It creates one atomic directory and performs no Git action; the required pull
belongs to the outer advisor workflow. Legacy version-1 path-only manifests
cannot support an archive operation.

For an explicitly requested replacement, never overwrite only one artifact.
Inventory the existing four-file bundle, obtain separate destructive approval,
and bind the exact destination plus all four old SHA-256 values under
`replace_existing`. After the mandatory pull, any file-set or hash drift
invalidates that approval. The archiver keeps collision rejection as the
default and uses an atomic whole-directory exchange only for the exact approved
bundle.

## Feedback and experience records

Record feedback as an immutable `feedback` event with `source` set to
`sim2sim` or `sim2real`. Include the exact policy identity when known,
deployment configuration, scenario, observations, safety events, evidence
paths, user assessment, root-cause classification, and next suggestion.

Before using an earlier lesson, run the read-only history query with the exact
current context:

```bash
conda run -n isaacsim-5.1 python \
  .agents/skills/monitor-tune-isaaclab-training/scripts/query_tuning_experience.py \
  --root "$ABSOLUTE_POLICY_TUNING_ROOT" \
  --run-identity "$SOURCE_IDENTITY_PATH" \
  --effective-config "$EFFECTIVE_CONFIG_PATH" \
  --effective-config-sha256 "$EFFECTIVE_CONFIG_SHA256" \
  --observation-fingerprint "$OBSERVATION_FINGERPRINT" \
  --deployment-fingerprint "$DEPLOYMENT_FINGERPRINT"
```

Read `historical_support.status`, every classification and completeness reason,
each event's confidence, and every evidence path and hash. Only
`candidate_events` has both compatible context and complete outcome evidence.
Conflicting, unknown, invalid, lifecycle-only, recommendation, or incomplete
history cannot become candidate outcome evidence. Even a candidate event is
insufficient by itself: combine it with current run evidence, state uncertainty,
and leave the parameter decision to the user.
See [experience-query.md](experience-query.md) for the complete query and
classification contract.

Every new experience event uses version 4, embeds the complete identity, and
references the revalidated effective configuration:

```json
{
  "version": 4,
  "event_id": "unique-ascii-id",
  "event_type": "run_snapshot",
  "recorded_at": "2026-07-31T18:00:00+08:00",
  "task": "exact-task",
  "run_id": "training-run-id",
  "algorithm": "AMP-ROA",
  "context": {
    "observation_fingerprint": "sha256 or explicit unknown",
    "reward_fingerprint": "sha256 or explicit unknown",
    "deployment_fingerprint": "sha256 or explicit unknown"
  },
  "parameters": {},
  "evidence": {
    "effective_config": {
      "path": "/absolute/.../evidence/source/effective-config-snapshot.json",
      "sha256": "64 lowercase hex characters",
      "effective_config_fingerprint": "64 lowercase hex characters",
      "reward_fingerprint": "64 lowercase hex characters"
    },
    "outcome": {
      "status": "unavailable",
      "reason": "run snapshot does not contain a baseline outcome comparison"
    }
  },
  "analysis": {"summary": "", "confidence": "low"},
  "next_suggestion": "",
  "run_identity": {
    "version": 1,
    "task": "exact-task",
    "run_id": "training-run-id",
    "host_id": "operator-chosen-host-id",
    "backend": "isaaclab",
    "algorithm": "AMP-ROA",
    "runner": "OnPolicyRunnerAmpROA",
    "seed": 42,
    "source": {
      "repository_root": "/absolute/robot_lab",
      "branch": "main",
      "head": "40 lowercase hex characters",
      "dirty": false,
      "dirty_paths": [],
      "diff_sha256": null,
      "patch_evidence": null
    },
    "training": {
      "command": ["python", "train.py", "--task=exact-task"],
      "hydra_overrides": []
    },
    "config_files": [
      {"path": "relative/config.py", "sha256": "64 lowercase hex characters"}
    ],
    "evaluation_scenario": {
      "contract": {
        "scenario_id": "quick-native",
        "scenario_overrides": {},
        "command_schedule": [],
        "duration_steps": 500,
        "num_envs": 1,
        "seed": 42
      },
      "sha256": "64 lowercase hex characters"
    },
    "identity_sha256": "64 lowercase hex characters"
  }
}
```

Allowed event types are `run_snapshot`, `assessment`, `decision`,
`checkpoint_evaluation`, `checkpoint_selection`, `export`, `archive`,
`feedback`, and `recommendation`. Records are append-only. Follow the
event-type and outcome contracts in [experience-query.md](experience-query.md).
Reuse a result only when task, algorithm, and all three context fingerprints
match and `outcome_evidence_complete` is true. A reward mismatch may produce a
verified informational configuration diff, but remains conflicting evidence.
Versions 1 and 2 remain unknown; version 3 may be context compatible but is
always outcome-incomplete.
