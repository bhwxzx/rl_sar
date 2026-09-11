# Host-local run identity

## Reusable training context (version 2, new batches)

Use the same host, task/run, source, exact argv, ordered Hydra, relevant training
config, runner and seed checks below. Omit `--scenario-contract-json` and `--output`
from the legacy command to create v2. The CLI prints a `{path, sha256}` reference
and exclusively stores `provenance/context-<file-sha256>.json`. Identical verified
bytes reuse that object; changed command, source, host or configuration yields a
new context. A previously confirmed host ID/argv remains valid authorization.

V2 omits `evaluation_scenario`; the remaining identity fields and internal identity
hash are validated as before. Do not weaken dirty-source evidence or omit relevant
training config files to obtain the same hash. Keep evaluator implementation sources
in the independently captured source snapshot, referenced by each scenario. The
finite batch runner preserves exact source bytes and verifies the live files before
launch. Its result binds context + effective config + scenario references.

Read [evidence-layout.md](evidence-layout.md) for the new batch protocol. The following
v1 scenario binding and command remain supported for existing/single evaluations.


For the legacy single-evaluation route, capture one version-1 run identity independently on every machine. This is
provenance evidence, not a distributed job protocol. Never use it to publish,
claim, synchronize, or control work on another host.

## Identity contract

Bind the identity to the exact task, run, backend, algorithm, runner, and seed.
Require a user-chosen safe `host_id`; do not silently derive a network hostname.
Record:

- the absolute local repository root, branch, and full HEAD;
- exact training argv as an ordered JSON array;
- exact Hydra overrides as a separate ordered JSON array whose tokens appear
  in the same order in the training argv;
- every relevant repository configuration path and its content SHA-256;
- the canonical evaluation scenario and its SHA-256;
- relevant dirty paths plus exactly one tracked diff SHA-256 or controlled
  patch path and SHA-256.

The evaluation scenario contains exactly `scenario_id`, `scenario_overrides`,
`command_schedule`, `duration_steps`, `num_envs`, and `seed`. Use the same
contract fields passed to `evaluate_policy.py`; do not hash an informal label.
Every evaluator invocation must pass the exact identity path plus its full-file
SHA-256. The evaluator recomputes the identity's internal SHA-256, requires the
task, run ID, seed, repository root, and full scenario contract to match, and
records both hashes in the immutable version-2 result. A matching scenario
label alone is insufficient.

List every configuration file that affects the run. Clean code outside those
files is represented by HEAD. If additional modified source affects the run,
include it as a `--config` input so its content and dirty state are captured.
Unrelated documentation or evidence files should not change source identity.

## Capture locally

First prepare a new snapshot with `prepare_evidence_layout.py`, then run:

```bash
python3 \
  .agents/skills/monitor-tune-isaaclab-training/scripts/capture_run_identity.py \
  --task "$TASK" --run-id "$RUN_ID" --host-id "$HOST_ID" \
  --backend "$BACKEND" --algorithm "$ALGORITHM" --runner "$RUNNER" \
  --seed "$SEED" \
  --training-command-json "$TRAINING_COMMAND_JSON" \
  --hydra-overrides-json "$HYDRA_OVERRIDES_JSON" \
  --config source/robot_lab/path/to/env_cfg.py \
  --config source/robot_lab/path/to/agent_cfg.py \
  --scenario-contract-json "$SCENARIO_CONTRACT_JSON" \
  --output "$SOURCE_IDENTITY_PATH"
```

The helper uses only local read-only Git queries: `rev-parse`, `symbolic-ref`,
`status`, `ls-files`, `show`, and `diff`. It disables optional locks, fsmonitor,
external diff, and textconv. It rejects repository mismatch, traversal,
symlinked inputs, duplicate config paths, unsafe identifiers, invalid hashes,
existing output, and source changes during capture.

For tracked dirty configs, it hashes `git diff --binary --full-index HEAD --`
over the exact config list. For a relevant untracked config or another case
where Git cannot produce the required tracked diff, first create a controlled
patch at the new `SOURCE_PATCH_PATH` through a separately authorized workflow,
then pass `--patch-evidence "$SOURCE_PATCH_PATH"`. The patch must stay in this
run's `evidence/source/` directory and name every relevant dirty path. The
helper only hashes the existing patch; it never creates, stages, applies,
commits, or transmits one.

## Record events

Embed the complete identity under `run_identity` in every new version-2 tuning
event. The event task, run ID, and algorithm must match the identity. The event
validator recomputes the scenario and overall identity hashes.

Version-1 events remain readable for compatibility, but they lack the MTA-005
host/source contract and cannot support a dual-host provenance claim. Compare
hosts only after each host independently records version-2 evidence. Matching
identity fields do not prove Git-mailbox readiness or authorize remote work.

## Evidence limits

Treat the identity as a deterministic attestation of supplied inputs, not a
host signature or proof of the live process. The helper does not attach to a
PID, inspect effective Hydra output, or prove that a user-chosen host ID is
globally unique. Cross-check the declared argv, overrides, task, seed, and
config list against the live process, log, and run directory through the
normal run-identity workflow before relying on the record.

Completeness depends on listing every modified source or configuration file
that affected the run. The helper detects ignored, untracked, status-visible,
and content-different listed files, but it cannot detect an influential file
that the operator omitted. The identity and scenario hashes detect later
mutation; they are fingerprints, not cryptographic signatures.

## Effective configuration is separate evidence

Repository configuration hashes describe the source inputs. They do not
replace the effective `params/env.yaml` and `params/agent.yaml` written after
Hydra and non-Hydra CLI overrides are applied. For every run handled by the
advisor, follow [`effective-training-config.md`](effective-training-config.md)
and bind those dumps to this identity in a separate immutable evidence file.

Require the effective env and agent seeds plus runner class to match this
identity. Preserve both layers: source identity explains which code and inputs
were supplied, while effective configuration records the resolved reward and
training values actually handed to the environment and runner.
