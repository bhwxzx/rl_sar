# Effective training configuration evidence

## Version 2 config content for new batches

With a v2 training context, `capture_effective_training_config.py CONTEXT --log-dir
ABSOLUTE_RUN` creates v2 and prints `{path, sha256}`; omit `--output`. The object is
stored at `provenance/config-<file-sha256>.json` and reused only if bytes match exactly.
V2 removes `run_identity_sha256`, retaining task/run/host, absolute raw YAML paths,
full UTF-8 content, raw hashes/sizes, resolved runner/seed/algorithm, parameter and
reward inventories, and recomputed semantic fingerprints. Bind the context and config
references together in every result/event/selection. This permits two source contexts
with identical effective YAML to reference one config without conflating their sources.
V1 retains its exact identity-hash binding and original path contract. Both schemas
use the same strict YAML/content validator; no compatibility branch skips validation.
The legacy commands below use v1 and explicit output paths.


Use the RSL-RL files written before `runner.learn()` as the authoritative
effective configuration for a run:

- `<absolute-run-log>/params/env.yaml` contains the resolved environment,
  reward, scene, command, observation, randomization, and termination config;
- `<absolute-run-log>/params/agent.yaml` contains the resolved runner,
  algorithm, seed, iteration, logger, and optimization config.

Do not reconstruct these values from console metrics, TensorBoard scalars, or
W&B metadata. Those sources may corroborate progress or activity but are not
the effective configuration authority.

## Capture contract

First prepare a new evidence snapshot and create its validated host-local run
identity. Once both YAML files exist, run:

```bash
conda run -n isaacsim-5.1 python \
  .agents/skills/monitor-tune-isaaclab-training/scripts/capture_effective_training_config.py \
  "$SOURCE_IDENTITY_PATH" --log-dir "$ABSOLUTE_RUN_LOG_DIRECTORY" \
  --output "$EFFECTIVE_CONFIG_PATH"
```

The helper requires an absolute run directory below the identity repository's
`logs/rsl_rl/` tree. It rejects symlinked inputs, missing files, duplicate YAML
keys, non-finite values, source changes during capture, seed or runner mismatch,
task-command mismatch, run-ID-directory mismatch, experiment-directory
mismatch, and existing output. It reads YAML nodes without constructing
Python-tagged objects.

The version-1 evidence binds task, run, host, and identity SHA-256; embeds the
exact UTF-8 content, path, size, and SHA-256 of both YAML files; records every
reward term including disabled `null` terms; and reports selected runtime
training values. It provides semantic fingerprints for the full environment,
agent, reward subtree, and combined effective configuration.

Treat the full raw YAML as the reproducibility record. Treat extracted reward
weights and training values as a deterministic index for review. A fingerprint
match proves identical captured configuration bytes or semantic YAML content,
not that the live process used an uncaptured file. Keep the run identity's
repository source hashes and command evidence alongside this artifact.

## Revalidation and comparison

Before recording or querying version-3/4/5 experience, re-read the artifact as a
regular non-symlinked file, verify its whole-file SHA-256, and recompute the
embedded env/agent source hashes, reward inventory, selected training values,
resolved identity, and all four semantic fingerprints. Bind the artifact to
the complete run identity; a matching filename or copied JSON is insufficient.

Parameter comparison reparses both embedded YAML documents and emits a stable,
complete JSON-Pointer-ordered semantic diff. It separately indexes reward
weight and selected training-parameter changes. Exceeding the configured diff
entry bound is an error, not a truncated comparison. A difference describes
what changed between captured configurations; it does not show which change
caused an observed training outcome.

Lifecycle experience events use version 4; new batch events use version 5. Both reference this artifact by its
absolute path, whole-file SHA-256, effective-config fingerprint, and reward
fingerprint. Version 3 remains readable and may prove context compatibility,
but it cannot prove version-4 outcome completeness. Versions 1 and 2 remain
readable but cannot provide verified effective-configuration history.
