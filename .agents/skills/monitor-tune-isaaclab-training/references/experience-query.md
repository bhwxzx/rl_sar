# Read-only experience query

## Version 5 batch events

New finite batches write one `evaluation_batch` event under `<run>/events/` after
sealing the manifest. It uses the v4 identity/config/evidence availability contract
and adds a manifest reference that revalidates every completed case, all failure/log
records, the finite contract and report hashes. Context v2/config v2 are supported.
No per-case events are emitted. Adopted summaries of existing cases do not duplicate
historical events. Selection/export/archive events remain independent.

Query scans both legacy run-root JSON and `events/*.json`, applying the same bounded,
non-symlink and schema checks. V5 batch events are evidence records, not parameter
outcomes: `outcome.status` is unavailable and direct parameter changes remain false.
The automatic event records unknown observation/deployment fingerprints honestly;
unknown context cannot become compatible history. V1-v4 classifications and actual
baseline/change/result-window requirements remain unchanged.


Use `scripts/query_tuning_experience.py` to find historical tuning events that
may be relevant to the current run. The query is an inventory and
compatibility check. It never selects a parameter, generates an experiment,
or changes training state.

## Contents

- [Query contract](#query-contract)
- [Version-4 evidence contract](#version-4-evidence-contract)
- [Context classification](#context-classification)
- [Outcome completeness](#outcome-completeness)
- [Output and evidence limits](#output-and-evidence-limits)

## Query contract

Pass an existing absolute history root and the complete current context:

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

The query revalidates the current identity and effective-config artifact, then
derives task, algorithm, host ID, run ID, and reward fingerprint. It rejects a
current artifact outside that run's `evidence/source/` directory, a mismatched
whole-file SHA-256, or any internally inconsistent embedded YAML evidence.

The tool scans only direct event files matching
`<root>/<task>/<run-id>/*.json`. It does not recurse into `evidence/`, invoke
Git, contact another host, or write an output file. JSON is emitted only to
stdout. `--max-events` and `--max-event-bytes` bound the scan; exceeding the
event-count limit is an error, while oversized event files are reported as
invalid history.

Each event must:

- pass `record_tuning_experience.py` validation;
- be a regular non-symlinked file under non-symlinked task and run paths;
- match the task and run ID encoded by its storage directory;
- use the immutable filename derived from `recorded_at` and `event_id`.

Every version-3, version-4 or version-5 event must also reference one effective-config
artifact under its own `evidence/source/` directory. For events whose
algorithm, host, observation, and deployment context match the current query,
the tool verifies that artifact's path, whole-file SHA-256, run identity,
reward fingerprint, and internal semantic fingerprints before classifying it
as usable history.

The query hashes the exact event bytes. A file that changes during reading is
rejected rather than classified.

## Version-4 evidence contract

Write lifecycle events as version 4 and evaluation batch events as version 5. Versions 1 through 3 remain readable; version 3
can prove context compatibility but cannot satisfy the version-4 outcome
contract.

For `assessment`, `checkpoint_evaluation`, `checkpoint_selection`, `export`,
`archive`, and `feedback`, set `evidence.event` to either an available absolute
path/SHA-256 reference or an explicit unavailable reason. Available references
are revalidated with their native bundle contract and must match the event's
task, run, algorithm, runner, and evidence layout as applicable. Feedback also
declares `evidence.policy_binding` as a selection, export, or archive reference,
or explicitly unavailable. Never use a bare path as proof.

Every version-4 event declares `evidence.outcome`. Use an unavailable reason
when no causal result comparison was captured. An available outcome is allowed
only for `assessment`, `checkpoint_evaluation`, or `feedback`, and contains:

- a baseline run identity and effective-config reference;
- the exact complete effective-config diff under `parameter_changes`;
- a bounded `result_window` bound to `evidence.event` by path and SHA-256;
- a non-empty observed-effect summary and observation list.

`checkpoint_selection`, `export`, and `archive` can have complete lifecycle
evidence but are not outcome-bearing events. A `recommendation` is advice and
must never declare an available outcome.

## Context classification

The result contains `compatible_events`, `conflicting_events`,
`unknown_events`, and `invalid_events`:

- `compatible`: a version-3, version-4 or version-5 event has verified effective-config
  evidence and its algorithm, host ID, and all three context fingerprints
  exactly match;
- `conflicting`: at least one known field differs; `classification_reasons`
  lists every known mismatch and any additional unknown field;
- `unknown`: no known mismatch exists, but the query or event contains an
  explicit `unknown` or lacks host identity;
- `invalid`: JSON, schema, path binding, size, or immutability validation
  failed.

Version-1 and version-2 events remain readable but lack the required verified
effective-config binding, so they are always `unknown`. Never treat
`unknown == unknown` as a match.

A version-3/4/5 event with matching algorithm, host, observation, and deployment
context remains comparison-eligible when only the reward fingerprint differs.
It stays `conflicting`, but its `parameter_diff` shows the verified historical
configuration as baseline and the current configuration as current. Events
from another host or incompatible observation/deployment context are not
dereferenced on the current host.

## Outcome completeness

Each valid result reports `context_compatible`, `event_evidence_complete`,
`outcome_evidence_complete`, `tuning_candidate_evidence`, and explicit
completeness reasons. These fields are independent: matching context is not
evidence that a parameter change caused a measured outcome.

Only an event with both `context_compatible: true` and
`outcome_evidence_complete: true` appears in `candidate_events`. A version-3
event, recommendation, lifecycle-only event, or explicit unavailable outcome
cannot enter that list. Evidence from another host or incompatible
observation/deployment context is not dereferenced; its completeness is
reported as not checked rather than guessed.

## Output and evidence limits

Each valid result preserves the event's own confidence, parameter snapshot,
analysis summary, next suggestion, absolute event path, and event SHA-256.
`evidence_refs` extracts absolute paths stored in structured `path` or
`*_path` fields and includes a valid adjacent SHA-256 when present.

An evidence reference is not proof that the artifact still exists or is
available on the current host. Verify the referenced file and hash separately
before using it.

`effective_config_verification` reports whether a version-3/4/5 artifact was
verified, skipped because its context belongs elsewhere, or unavailable on a
legacy event. A verified `parameter_diff` is complete and deterministic. It
contains semantic JSON-Pointer changes plus separate reward-weight and selected
training-parameter changes. `--max-diff-entries` bounds complete comparison;
exceeding it makes that candidate invalid instead of silently truncating it.

Interpret `historical_support.status` as follows:

- `candidate_outcome_history_available`: at least one context-compatible event
  has complete outcome evidence and the scan is valid;
- `context_compatible_outcome_incomplete`: matching context exists but no
  complete outcome is available;
- `no_context_compatible_history`: no context-compatible event exists;
- `query_context_incomplete`: at least one current query field is unknown;
- `history_invalid`: at least one scanned event could not be validated.

`direct_parameter_change_supported` is always `false`. Even candidate outcome
history is advisory evidence only. Combine it with current run evidence, state
conflicts and uncertainty, and present any proposed change to the user for
approval.
