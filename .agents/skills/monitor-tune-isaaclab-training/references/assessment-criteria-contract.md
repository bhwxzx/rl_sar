# Assessment criteria v2 contract

This contract makes every strong training or convergence recommendation depend
on task-specific criteria that the user has reviewed and approved. The draft
template contains no LW_Leg thresholds and cannot authorize a decision.

## Contents

- [Decision boundary](#decision-boundary)
- [Contract fields](#contract-fields)
- [Approval receipt](#approval-receipt)
- [Safe approval workflow](#safe-approval-workflow)
- [Validation and assessment](#validation-and-assessment)

## Decision boundary

Only an `approved` contract whose hash and scope exactly match the current run
may produce `continue`, `consider_stop_plateau`, `recommend_stop_invalid`, or
`converged`. Missing, draft, malformed, modified, or mismatched criteria produce
`insufficient_evidence` and `indeterminate` convergence.

Non-finite values and a stalled health snapshot remain visible as
`safety_alerts` and set `operator_attention_required`, but do not become a
strong stop recommendation without a matching approved contract.

## Contract fields

Start from `assets/assessment-criteria-template.json`. Fill only the `contract`
while it remains a draft.

`scope` binds the decision to exact values for:

- `task` and `run_id`;
- `backend` and algorithm `profile_id`;
- `algorithm` and `runner`.

`windows` defines the adjacent window size, minimum total record count, and the
number of required metrics that must plateau before a plateau conclusion.

`required_metrics` contains decision-bearing metrics. Each entry has a
`direction` of `maximize` or `minimize` and a non-negative
`plateau_relative_tolerance`. These metrics alone drive improvement, plateau,
and convergence counts.

`observed_metrics` may contain `direction` and `description` for reporting. It
must not contain tolerances, limits, operators, values, or a `required` flag.
Observed trends never change a recommendation.

`hard_failures` contains three explicit mechanisms:

- `non_finite_metrics`: whether parsed non-finite metrics are a hard failure;
- `health_states`: health states, such as `stalled`, that are hard failures;
- `metric_limits`: named metric gates with an `op` and finite `value`.

For every metric limit, the latest-window mean must satisfy its expression. A
false expression is a hard failure. Supported operators are `<=`, `>=`, `<`,
and `>`.

`play_gates.required_for_convergence` must be `true` in an approved contract.
`play_gates.metrics` defines at least one Play metric gate. Missing Play results
therefore prevent a `converged` result.

## Approval receipt

The `approval` object is separate from `contract`:

```json
{
  "status": "approved",
  "approved_at": "2026-08-01T17:00:00+08:00",
  "approved_contract_sha256": "64 lowercase hexadecimal characters"
}
```

The hash is computed from canonical JSON of `contract` only: keys sorted,
compact separators, UTF-8, and no non-finite JSON numbers. Any contract edit
changes the computed hash and invalidates the approval receipt.

## Safe approval workflow

1. Copy the template to a user-chosen absolute path.
2. Fill the exact scope, windows, required metrics, observed metrics, hard
   failures, and Play gates while `approval.status` remains `draft`.
3. Run the validator and show the complete contract plus reported
   `contract_sha256` to the user.
4. Wait for explicit approval of that exact contract and hash.
5. Only after approval, set `status` to `approved`, record a timezone-aware
   `approved_at`, and copy the reported hash to
   `approved_contract_sha256`.
6. Run the validator again with the exact expected scope. Do not use the
   criteria unless it reports `eligible: true`.

Never infer approval from an earlier run or conversation. Never update the
approval hash automatically after changing the contract.

## Validation and assessment

Validate without changing the file:

```bash
conda run -n isaacsim-5.1 python \
  .agents/skills/monitor-tune-isaaclab-training/scripts/validate_assessment_criteria.py \
  /absolute/criteria.json \
  --task EXACT_TASK --run-id EXACT_RUN --backend EXACT_BACKEND \
  --profile-id EXACT_PROFILE --algorithm EXACT_ALGORITHM --runner EXACT_RUNNER
```

The report records the resolved absolute criteria path, whole-file SHA-256,
canonical contract SHA-256, approval receipt, scope, mismatches, and errors.
The assessment embeds the same provenance. The whole-file hash may change when
only the approval receipt changes; the contract hash identifies the exact
decision-bearing content.
