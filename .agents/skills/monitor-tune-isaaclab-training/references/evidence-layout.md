# Policy-tuning evidence layout

## Default: version 2 evaluation batches

```text
learnings/policy_tuning/<task>/<run-id>/
  index.md                         # mutable, regenerable navigation only
  provenance/
    context-<file-sha256>.json      # training identity v2, no scenario
    config-<file-sha256>.json       # effective config v2, no identity hash
    source-<file-sha256>.json       # exact evaluator source snapshot
    scenario-<file-sha256>.json     # contract plus source snapshot reference
  evaluations/<batch-id>/
    contract.json                  # finite execution input, executed batches only
    manifest.json                  # sealed cases/outcomes/refs, published last
    report.md                      # one human report for the question
    raw/<attempt-id>/
      result.json                  # completed result v3, layout_version 2
      telemetry.json.gz            # telemetry v4, gzip mtime=0, full precision
      video.mp4                    # only if requested
      console.log                  # one log per attempted case, including failures
      .attempt-used                # persistent reservation, never reuse this ID
  events/<timestamp>__<event-id>.json # v5 batch event
  evidence/                        # existing monitoring and lifecycle producers
    training/                      # only when training summaries are needed
    checkpoint_selection/
    export/
```

The lifecycle directory names stay compatible with the existing selection/export
receipt protocols; they accept both provenance schemas through one shared path
validator. No historical migration, deletion, recompression or rewritten receipts.
New batch layout is explicit (`layout_version: 2`); result/telemetry/context/config
versions distinguish contracts. Version-2 results, v1 identities/configs, old events
and export v3/v4 still require all their original hashes and scope checks.

`index.md` is the only replaceable generated output. Never reference its hash from
an immutable event. Rebuild it with `summarize_evaluation_batch.py --run-root
"$RUN_ROOT" --index-only`. Finished batch reports/manifests are immutable. Include
all completed, failed and not-run cases in one manifest; recommendations and
optional plot/video links belong in its report. Avoid parallel `summary.json` or
per-case prose. Training log summaries remain separate machine evidence when needed.

## Capture reusable provenance

Run `capture_run_identity.py` with the confirmed host/argv/config flags described
in `run-identity.md`, omitting `--scenario-contract-json` and `--output` for context
v2. It prints `{path, sha256}` for `provenance/context-<hash>.json`. Then run:

```bash
conda run -n isaacsim-5.1 python \
  .agents/skills/monitor-tune-isaaclab-training/scripts/capture_effective_training_config.py \
  "$CONTEXT_PATH" --log-dir "$ABSOLUTE_RUN_LOG_DIRECTORY"
```

The returned config reference names `provenance/config-<hash>.json`. Exact content
may be reused only after fresh validation. A changed training source or command
gets a new context; identical effective YAML may still reuse one config. Different
YAML bytes, scope, resolved runner/seed or semantic fingerprints cannot be merged.
`evidence_provenance.py` stores full evaluator source bytes once, including approved
untracked helper changes. Each scenario references this snapshot. Launch preflight
checks live evaluator hashes; historical reads validate captured content without
requiring the current checkout to remain at the old revision.

## Execute a finite Native batch

The input JSON contains exactly:

```json
{
  "version": 1,
  "batch_id": "turning-001",
  "run_identity": {"path": "<absolute context path>", "sha256": "<file sha256>"},
  "effective_config": {"path": "<absolute config path>", "sha256": "<file sha256>"},
  "checkpoint": {"path": "<absolute model_N.pt path>", "sha256": "<file sha256>"},
  "cases": [{
    "attempt_id": "left-direct-a01",
    "scenario": {
      "scenario_id": "left-direct",
      "scenario_overrides": {},
      "command_schedule": [{"start_step": 0, "end_step": 249, "command": [0, 0, 0.5]}],
      "duration_steps": 250, "num_envs": 1, "seed": 42
    },
    "video": false,
    "timeout_seconds": 600
  }]
}
```

This example is a schema illustration, not an approved test budget. Command segments
are inclusive and must cover the whole duration when supplied. Scenario seed must
match the run. Maximum safeguards: 32 cases, 64 environments, 100,000 steps per case,
3,600 seconds per process. Choose a much smaller task-appropriate budget normally.
The runner accepts no arbitrary shell commands and never adapts cases after failures.

```bash
conda run -n isaacsim-5.1 python \
  .agents/skills/monitor-tune-isaaclab-training/scripts/run_evaluation_batch.py \
  "$BATCH_CONTRACT" --validate-only
conda run -n isaacsim-5.1 python \
  .agents/skills/monitor-tune-isaaclab-training/scripts/run_evaluation_batch.py \
  "$BATCH_CONTRACT"
```

The runner reserves a fresh batch directory, captures the input contract and source,
then starts only its own evaluation subprocesses. It requires idle GPU and retains
console logs. A timeout stops only that evaluation process group; an interruption
marks remaining cases not-run. Hard process loss may leave an unsealed directory;
inspect its console/contract and use a new batch ID. Never erase an attempt to retry.
For manual new evaluations allocate with `prepare_evidence_layout.py --batch-id ...
--evaluation-id ...`; pass the returned result/telemetry/video paths plus
`--batch_id`, `--effective_config_reference_json`, and
`--scenario_evidence_reference_json` to the evaluator. The runner normally handles
those bindings, so prefer it for batches.

Publication retains exclusive claims/private attempts, no-overwrite links, SHA-256
of the actual compressed telemetry bytes, and `result.json` last. Compression is
lossless: no stride increase, sample removal or rounding. Consumers revalidate the
whole bundle and then read either legacy JSON or gzip. Reports do not replace raw
telemetry or establish approved convergence criteria.

## Group existing evidence without migration

```bash
conda run -n isaacsim-5.1 python \
  .agents/skills/monitor-tune-isaaclab-training/scripts/summarize_evaluation_batch.py \
  --run-root "$RUN_ROOT" --batch-id turning-review-001 \
  --result "$LEFT_RESULT" --result "$RIGHT_RESULT" \
  --supporting-evidence "$EXISTING_REPORT" \
  --note "State observed behavior and limits here"
```

An adopted batch validates every original result and stores references without
copying raw files or claiming that missing historical logs exist. Existing per-case
events remain unchanged; grouping them does not append duplicate experiences. New
executed batches append exactly one v5 `evaluation_batch` event. Selections, exports
and archives retain independent lifecycle events. Unknown observation/deployment
fingerprints remain explicit unknowns, so the automatic batch event cannot silently
become compatible parameter-change history.

# Legacy layout (v1)

For existing v1 evidence, use its deterministic repository-local tree for raw observations and keep it
separate from immutable tuning-experience events:

```text
learnings/policy_tuning/<task>/<run-id>/
├── evidence/
│   ├── criteria/criteria-<snapshot-id>.json
│   ├── health/health-<snapshot-id>.json
│   ├── source/identity-<snapshot-id>.json
│   ├── source/effective-config-<snapshot-id>.json
│   ├── source/source-<snapshot-id>.patch
│   ├── training/summary-<snapshot-id>.json
│   ├── training/assessment-<snapshot-id>.json
│   ├── checkpoint_selection/selection-<selection-id>.json
│   ├── play/<evaluation-id>/
│   │   ├── result.json
│   │   ├── telemetry.json
│   │   └── video.mp4
│   └── export/<export-id>/
│       ├── policy.pt
│       ├── policy.onnx
│       └── receipt.json
└── <timestamp>__<event-id>.json
```

## Prepare paths

Run the standard-library-only helper from the repository root before writing
evidence. Use a new snapshot ID for every observation and a new evaluation ID
for every Play attempt:

```bash
eval "$(
  python3 \
    .agents/skills/monitor-tune-isaaclab-training/scripts/prepare_evidence_layout.py \
    --task "$TASK" --run-id "$RUN_ID" \
    --snapshot-id "$SNAPSHOT_ID" --evaluation-id "$EVALUATION_ID" \
    --selection-id "$SELECTION_ID" --export-id "$EXPORT_ID" \
    --format shell
)"
```

The assignments provide `RUN_ROOT`, `EVIDENCE_ROOT`, `CRITERIA_PATH`,
`HEALTH_PATH`, `SOURCE_IDENTITY_PATH`, `EFFECTIVE_CONFIG_PATH`,
`SOURCE_PATCH_PATH`, `SUMMARY_PATH`, `ASSESSMENT_PATH`, `PLAY_RESULT_PATH`,
`TELEMETRY_PATH`, `VIDEO_PATH`, `CHECKPOINT_SELECTION_PATH`, `EXPORT_JIT_PATH`,
`EXPORT_ONNX_PATH`, and `EXPORT_RECEIPT_PATH`.
Omit `--evaluation-id` when no Play
artifacts are needed; shell output then explicitly unsets the three Play
variables so values from an earlier evaluation cannot leak into the current
snapshot. The default JSON format returns `null` for those paths and also
reports the created directories.
Omit `--selection-id` or `--export-id` when those artifacts are not needed;
their shell variables are explicitly unset for the same reason.

The helper accepts only bounded ASCII identifiers, rejects traversal and every
existing symlink component, creates directories but not evidence files, and
fails if any returned target already exists. It performs no Git operation.
Do not bypass a failure by deleting or overwriting evidence; choose a new
snapshot, evaluation, selection, or export ID.

## Write evidence once

Pass the returned paths directly to producers. `collect_training_health.py`
keeps stdout behavior when `--output` is omitted, but file output must be a new
absolute path. `summarize_training_log.py` always requires a new absolute
`--output` path. Both reject existing targets and symlinked parent components.

Create or copy a criteria draft into `CRITERIA_PATH` as a new file. After the
user approves its exact contract and hash, do not edit that file. Use the
matching `HEALTH_PATH`, `SUMMARY_PATH`, and `ASSESSMENT_PATH` for the same
snapshot. A later observation gets a new snapshot ID and a fresh set of paths.

After writing `SOURCE_IDENTITY_PATH`, pass it and the exact absolute RSL-RL run
directory to `capture_effective_training_config.py`; write only to the matching
new `EFFECTIVE_CONFIG_PATH`. The artifact embeds both effective YAML dumps and
their fingerprints. Do not substitute console, TensorBoard, or W&B metadata.

The evaluator requires `PLAY_RESULT_PATH` and accepts `TELEMETRY_PATH` and
`VIDEO_PATH` only when they exactly match its `--task`, `--run_id`, and
`--evaluation_id`. Also pass the matching immutable source identity through
`--run_identity_path` and `--run_identity_file_sha256`. Use `--no_video` and
omit `VIDEO_PATH` when video was not authorized or needed.

Before Isaac Sim starts, the evaluator verifies the checkpoint, deployment
artifact, run-identity file, complete scenario contract, all supplied hashes,
and every canonical output path. It rejects traversal, symlinked components,
existing final targets, and a scenario that conflicts with the identity. A
`.publish-claim` and private `.attempt/` directory serialize writers. Work
files stay in that attempt directory; final video and telemetry are published
with non-overwriting hard links, and `result.json` is linked last as the sole
completion marker. Normal failure removes only claim and attempt objects owned
by that process. A publication collision rolls back only final links created
by that process and never overwrites the competing file.

Version-2 results bind the exact checkpoint, artifact, source identity,
scenario fingerprint, and resource mode. Their `outputs` object binds every
published telemetry/video file by canonical absolute path and SHA-256.
Telemetry version 3 repeats the same `evaluation` and `inputs` objects. Downstream
consumers must revalidate the complete bundle rather than trusting path strings
or the presence of a partial work file.

Checkpoint selection receipts and export artifacts use their own immutable
directories. Create a selection receipt only after explicit user choice. The
exporter writes only below its private `.attempt/`, publishes JIT and ONNX with
non-overwriting hard links, and publishes the version-4 `receipt.json` last.
The validator remains backward-compatible with completed version-3 receipts.
See [`policy-export.md`](policy-export.md) for the complete source, parity, and
archive validation contract.

## Keep events immutable and separate

Raw criteria, health, summary, assessment, result, telemetry, and video files
stay below `evidence/`; source identity, effective configuration, and optional
controlled patch evidence stay below `evidence/source/`.
`record_tuning_experience.py` writes timestamped, append-only event JSON
directly below `RUN_ROOT`; never place those event files inside `evidence/`.

An event may reference raw evidence only by its absolute path plus a SHA-256.
Once referenced, the evidence file is immutable. Record later observations
under new snapshot or evaluation IDs and append a new event instead of editing
the earlier evidence or event.
