# Transactional policy selection, export, and archive

Selection/export/archive validators accept both legacy v1 identity/config paths and
new content-addressed v2 provenance through `evidence_provenance.require_source_path`.
Closed-loop result v3/gzip bundles are fully revalidated alongside legacy v2 bundles.
Selection and export receipt formats and lifecycle directories stay unchanged; this
is compatibility with additional validated inputs, not a relaxed parity/reset gate.


## Contents

- [Prepare evidence paths](#prepare-evidence-paths)
- [Record the user selection](#record-the-user-selection)
- [Export transaction](#export-transaction)
- [Parity contract](#parity-contract)
- [Export receipt and validation](#export-receipt-and-validation)
- [Archive gate](#archive-gate)

## Prepare evidence paths

Allocate a fresh selection ID and export ID with
`prepare_evidence_layout.py`. The final paths are:

- `CHECKPOINT_SELECTION_PATH`;
- `EXPORT_JIT_PATH`;
- `EXPORT_ONNX_PATH`;
- `EXPORT_RECEIPT_PATH`.

Never reuse either ID. Existing final files, a claim, or a partial export
directory are evidence of an earlier attempt, not permission to overwrite it.

## Record the user selection

Run `policy_export_evidence.py record-selection` only after the user explicitly
chooses one stable checkpoint. Supply the exact checkpoint-selection report and
SHA-256, checkpoint path/hash/filename stem, source identity, effective config,
every supporting version-2 evaluation result, and the reviewed tensor contract.

The checkpoint must be one stable inventory entry named
`model_<iteration>.pt`; `checkpoint_id` must equal its filename stem. The
receipt revalidates all evaluation bundles and requires their task, run,
runner, checkpoint path, and checkpoint hash to match. AMP-ROA also requires
complete evaluation telemetry.

The runner-specific tensor contracts are deterministic:

- ROA/AMP-ROA: flattened time-major history, current-frame-only normalization,
  environment history reset, actor input `[current_obs, code_vel, hist_latent]`;
- DWAQ/AMP-DWAQ: flattened time-major history, combined actor-input
  normalization, environment history reset;
- supported stateless runners: current observation, backend export-helper
  normalization, stateless environment reset.

The selection receipt embeds the complete run identity and binds the effective
config, report, checkpoint, evaluation results, and tensor contract by hash.

```bash
python3 .agents/skills/monitor-tune-isaaclab-training/scripts/policy_export_evidence.py \
  record-selection --selection-id "$SELECTION_ID" --approved-at "$APPROVED_AT" \
  --checkpoint-id "$CHECKPOINT_ID" --checkpoint "$CHECKPOINT" \
  --checkpoint-sha256 "$CHECKPOINT_SHA256" \
  --selection-report "$SELECTION_REPORT" \
  --selection-report-sha256 "$SELECTION_REPORT_SHA256" \
  --run-identity "$SOURCE_IDENTITY_PATH" \
  --run-identity-file-sha256 "$SOURCE_IDENTITY_FILE_SHA256" \
  --effective-config "$EFFECTIVE_CONFIG_PATH" \
  --effective-config-sha256 "$EFFECTIVE_CONFIG_SHA256" \
  --evaluation-result "$PLAY_RESULT_PATH" \
  --tensor-contract-json "$TENSOR_CONTRACT_JSON" \
  --output "$CHECKPOINT_SELECTION_PATH"
```

## Export transaction

Pass the new selection receipt and whole-file SHA-256 to
`rsl_rl_export_policy.py`. Also pass the same task, run, checkpoint ID/path/hash,
tensor-contract strings, a fresh export ID, canonical output paths, and the
bounded parity contract.

Before Isaac Sim starts, the exporter validates all source evidence and takes
an exclusive `.publish-claim`. JIT, ONNX, and receipt work files are generated
inside its owned `.attempt/`. It loads both artifacts, completes every parity
gate, and revalidates all input evidence before publishing. JIT and ONNX are
linked without overwrite; `receipt.json` is linked last as the only completion
marker. Normal failure removes only owned attempt objects and final links whose
inode still belongs to that publisher.

```bash
conda run -n isaacsim-5.1 python \
  .agents/skills/monitor-tune-isaaclab-training/scripts/rsl_rl_export_policy.py \
  --task "$TASK" --run_id "$RUN_ID" --export_run_id "$EXPORT_ID" \
  --checkpoint_id "$CHECKPOINT_ID" --checkpoint "$CHECKPOINT" \
  --checkpoint_sha256 "$CHECKPOINT_SHA256" \
  --selection_receipt_path "$CHECKPOINT_SELECTION_PATH" \
  --selection_receipt_sha256 "$CHECKPOINT_SELECTION_SHA256" \
  --jit_path "$EXPORT_JIT_PATH" --onnx_path "$EXPORT_ONNX_PATH" \
  --result_path "$EXPORT_RECEIPT_PATH" \
  --history_contract "$HISTORY_CONTRACT" \
  --normalization_contract "$NORMALIZATION_CONTRACT" \
  --reset_contract "$RESET_CONTRACT" \
  --onnx_export_profile static_batch_1_simplified \
  --parity_steps 8 --reset_step 4 \
  --minimum_parity_samples 8 --max_abs_action_error 1e-5 \
  --num_envs 1 --seed "$SEED" --require_idle_gpu --headless
```

## ONNX export contract

Choose the profile explicitly; there is no implicit default. Use
`static_batch_1_simplified` for a single-robot deployment. It fixes input and
output batch dimensions at 1, uses stable `obs` and `actions` names, exports
with opset 17, requires `onnxsim` to pass its model check, then reloads and
validates the final ONNX graph. Use `dynamic_batch` only when a consumer truly
needs batches larger than 1; that profile retains dynamic batch axes and opset
18 without simplification.

The static profile evaluates the multi-sample parity corpus one row at a time
and concatenates the actions, so it preserves the full temporal and reset
coverage. A static-batch artifact rejects direct batch sizes greater than 1 by
design.

## Parity contract

Use 2 through 64 temporal steps and 1 through 64 environments. Choose an
explicit reset step strictly inside the window. The exporter captures every
environment at every step and records at least these labeled boundaries:
`initial`, `pre_reset`, `post_reset`, and `final`.

Native device, Native CPU, JIT, and ONNX actions must have matching shapes and
finite values. JIT and ONNX maximum absolute action errors must not exceed the
approved limit. The receipt records observation/action digests, shape evidence,
boundary steps, sample count, reset contract, and Native device-to-CPU error.
This is bounded open-loop parity around real environment observations; it is
not deployment or hardware qualification.

## Export receipt and validation

A newly completed version-4 receipt binds:

- task, run, runner, checkpoint ID, and export ID;
- complete run identity and effective-config reference/fingerprints;
- approved checkpoint-selection receipt;
- tensor, ONNX export, and parity contracts;
- final ONNX input/output names, dtypes, shapes, opset/profile, simplifier
  result, and pre/post-simplification node counts;
- JIT/ONNX canonical paths, sizes, and SHA-256 values;
- multi-time/reset-boundary parity evidence.

Use `policy_export_evidence.py validate-export RECEIPT` before any downstream
use. Missing receipt, source drift, hash mismatch, incomplete boundaries,
parity failure, or a changed artifact invalidates the entire export.
Completed version-3 receipts created before this contract upgrade remain
validation-compatible, but every new export must publish version 4.

## Archive gate

Archive only after a separate user authorization. A version-2 archive manifest
must reference the export receipt by path and SHA-256 and list evaluation
results as `{path, sha256}` objects. After authorization and immediately before
the archive write, require a clean storage worktree and index, including no
untracked paths, then run `git -C "$STORAGE_ROOT" pull --ff-only`. Recheck the
updated HEAD and Git state, destination collision, and duplicate JIT/ONNX pair.
Stop without archiving if the upstream is missing, the pull fails or cannot
fast-forward, the repository is dirty, or a target or duplicate appears. Never
stash, merge, rebase, reset, checkout, clean, or resolve storage state
automatically.

The archiver revalidates the complete export and evaluation bundles, then
cross-checks checkpoint, task, runner, algorithm, source HEAD/dirty state, and
both artifacts before creating its atomic destination. It performs no Git
action; the required pull belongs to the outer advisor workflow. Legacy
version-1, path-only manifests are ineligible.

An existing destination remains an error by default. Replace one only after a
separate destructive-operation approval based on a read-only inventory. Add
this exact-shape object to the otherwise complete version-2 manifest:

```json
{
  "replace_existing": {
    "authorized": true,
    "path": "/absolute/policy_storage/collection/existing-directory",
    "files": {
      "policy.pt": "old SHA-256",
      "policy.onnx": "old SHA-256",
      "策略说明.txt": "old SHA-256",
      "archive_manifest.json": "old SHA-256"
    }
  }
}
```

Immediately after the required pull, recheck the exact four-file set and every
old hash. The archiver builds and verifies the new bundle in a private sibling,
atomically exchanges the directories, and deletes only the displaced bundle.
Any missing/extra file, symlink, changed hash, unavailable atomic exchange, or
cleanup failure aborts the operation. The original tracked bundle remains
recoverable from its prior Git commit, but never restore it automatically.
