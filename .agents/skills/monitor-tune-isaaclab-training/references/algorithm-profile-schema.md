# Algorithm profiles and controlled upgrades

Use `algorithm-profiles.json` as the machine-readable registry. Profiles
separate common training supervision from algorithm-specific progress, metrics,
protected parameters, resume requirements, and deployment-evaluation inputs.

## Resolution

Resolve profiles by exact `backend`, `algorithm name`, and `runner class`. The most specific matching profile wins. Profiles may extend another profile to reuse common behavior.

Current built-in families include:

- RSL-RL PPO;
- RSL-RL Distillation;
- RSL-RL DWAQ;
- RSL-RL AMP;
- RSL-RL AMP-DWAQ;
- RSL-RL ROA;
- RSL-RL AMP-ROA;
- generic RSL-RL, SKRL, CU-SRL, and custom fallbacks.

Generic profiles permit supervision and recovery. Do not authorize tuning through a generic profile because its metric meanings, checkpoint state, and risky parameter paths are not yet proven for that algorithm.

## Profile fields

- `id`: stable lowercase identifier.
- `profile_version`: positive integer; increment when semantics change.
- `is_generic`: whether the profile is a fallback.
- `extends`: optional parent profile.
- `match`: arrays for backends, algorithm names, and runner classes; `*` is a wildcard.
- `progress_patterns`: regexes with named `current` and optional `target` groups.
- `metric_aliases`: console labels mapped to stable metric keys.
- `protected_parameter_patterns`: regexes identifying parameters that require
  a separately proposed edit and explicit user approval.
- `resume_required_args`: argv tokens that an approved resume command must contain.
- `evaluation_capabilities`: backend Play entry point, supported Native/JIT/ONNX
  artifacts, and the deployment observation-history contract.

`history_contract` is one of:

- `current_observation`: consume the current policy observation;
- `flat_time_major_history`: flatten `[batch, time, observation]` without
  changing time order;
- `backend_defined`: use a reviewed backend adapter;
- `review_required`: block final evaluation until a specific profile defines
  the contract.

Generic profiles remain ineligible for final strategy promotion. A new specific
profile must review Play loading, artifact input ordering, normalization, state
reset, and numerical parity before adding JIT or ONNX support.

Run `scripts/validate_algorithm_profiles.py` after every registry change.

## Unknown algorithm workflow

1. Capture and validate the complete current host-local run identity. Do not
   use placeholder or `auto` identity values.
2. Run `scripts/scan_algorithm_coverage.py` to detect new training/evaluation
   runner branches or configured identities lacking specific profiles.
3. Inspect the current entry point and dumped configuration.
4. Run `scripts/discover_algorithm_profile.py` with the run identity, log, and
   optional dumped config. The profile backend is the algorithm framework
   resolved from the training command; it is distinct from an environment or
   simulator backend such as `isaaclab` recorded by the run identity. A supplied
   config must match a repository-relative path and SHA-256 in
   `run_identity.config_files`; the discovery result records both.
5. Use the generic matched profile for monitor-only work.
6. Review the generated candidate's identity, metric aliases, progress
   semantics, checkpoint contents, resume behavior, protected parameters, Play
   entry point, history/normalization contract, artifact formats, and smoke-test
   requirements. When aliases come from a log, retain its resolved path,
   SHA-256, and byte count in the candidate for audit.
7. Propose an exact registry modification plan.
8. Apply the candidate only after explicit approval.
9. Validate the registry, run identity, parser, resume checks, and a
   representative smoke run.
10. Forward-test the upgraded skill with a fresh task.

Never let discovery write directly to the registry. Automatic discovery and candidate generation are safe; persistent self-modification remains approval-gated by the repository workflow.
