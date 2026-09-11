# Sim2Sim report mode

Use this mode on a MuJoCo/deployment computer to produce a report for the IsaacLab
operator. It is independent of the Isaac Native evaluator and training provenance
schemas. Resolve `$SKILL_DIR` to the directory containing the installed SKILL.md;
no script in this mode imports IsaacLab, robot_lab, torch, or MuJoCo.
The packaging helper does not collect data or launch simulations.

## Establish the question and local runtime

Distinguish analyzing existing results from executing a finite new test. Reuse
the user's selected policy, question, host ID, cases, duration, seeds and existing
authorization. If a new test lacks a necessary decision, propose the concrete
cases before launching it. Do not silently add A/B variants or training runs.

Inspect the actual deployment repository and its local instructions. Locate its
existing collector, executable, policy and model dependencies; use the deployment
environment already configured there. Do not copy a collector from an old evidence
package and assume its hard-coded paths, counts or assertions apply. If required
signals need instrumentation, propose the specific source changes and obtain any
approval required by the workspace before editing them. Report unavailable signals
when instrumentation is outside scope; do not fabricate a complete assessment.

Record local deployment identity: user-supplied host ID (reuse when known), full
Git HEAD, relevant dirty diff/source bytes, executable/collector source hashes,
MuJoCo/runtime versions, exact executed argv, seed, scene and resolved configuration.
Do not infer actual argv from config. Never require a training command or checkpoint
file absent from this machine. Bind the deployed policy by its actual SHA-256 and
available export manifest, task/run/checkpoint identifiers; label unverified upstream
fields as unknown. Do not substitute an unverified training checkpoint hash for the
deployed ONNX/JIT hash.

## Test and observation contract

Before new execution, state the finite cases, initial state/reset, command schedule,
duration, physics step, policy period, low-level/PD period, termination conditions
and authorized video requirements. Record timestep/decimation and actual timestamps,
not only requested rates. Stop only the owned test process on its timeout or safety
termination; retain partial evidence with its status. Do not modify the deployed
policy, production configuration or its input semantics to make a test pass.

Use the actual algorithm and tensor interface. Never assume all models use the
LW AMP-ROA layout. For that layout, explicitly verify rather than assume the known
41-value frame / 10-frame history / 410-value flattened input. Capture joint names
and both native and policy indices, position-relative-to-default semantics, wheel
position masking and noise, velocity scaling, quaternion convention and projected
gravity, previous actions, normalization, history order/initialization and reset.
Distinguish the tensor passed to inference from a reconstruction or debug buffer.

Collect what answers the question, retaining the existing full-rate/full-precision
evidence contract when rerunning an established diagnostic:

- Case, condition, reset ID, step/time, actual/planned commands, pre/post state and
  the sampling phase of each signal.
- Body quaternion, angular and linear velocity, position/height; define the sign,
  axes and formula for reported roll/pitch/backward lean.
- Joint q/default_q/dq in a named order, actual model input, raw action, scaled
  targets, low-level updates, clipping/saturation and applied actuator forces.
- Available contact identities/forces and physics substeps, with sampling rates.
  Do not infer contact from a picture or invent unrecorded substep signals.
- Reset/termination/safety events, exit code and one console log per attempt.

For observation mismatch questions, retain full actual inference inputs including
history for independent replay. An A/B comparison changes only the authorized
factor and matches policy, initial state, seed, timing, schedule and other config.
Preserve both conditions and early termination. Never fill the remainder of a
terminated case or call an offline reset test a closed-loop reset test.

## Analyze and seal one report

Reuse the deployment project's working diagnostic layout when available:
one `report.md`, machine-readable statistics/comparison, effective config and
identity, per-case results/raw telemetry/logs, requested video, and `manifest.json`.
Do not generate a second prose summary of the same test. New retries or new facts
belong in a new attempt/package; finalized files and their hashes stay immutable.

Report window boundaries, actual sample counts/coverage, resets and command
transitions. Include velocity/yaw tracking, posture, action/torque saturation and
stability metrics appropriate to the question. For pure turning, separate yaw-rate
tracking from accumulated heading and compare moving turns only when requested.
Separate observations, supported conclusions, unavailable evidence and proposed
next tests. A/B posture improvement does not prove that the whole transfer gap is
fixed or that locomotion remains stable. Simulation is not hardware readiness.

Validate file hashes, JSON/gzip readability, sample chronology, finite numeric
values, tensor dimensions and case completion/termination accounting before sealing.
If videos are requested, check robot-in-frame, timestamps and decoded duration;
label offline log replay explicitly. Keep failure evidence. Semantic validation
and the transfer helper's byte-integrity validation are different checks.

## Delivery: only model-resource reuse changes

Keep the existing telemetry frequency, precision, format, videos, report content
and `.tar.gz` delivery. Do not downsample, round, truncate, split delivery into
summary/detail tiers, or add transfer chunking. Retain the complete local evidence
directory. Use `scripts/sim2sim_report_bundle.py` to build a separate transfer archive
from that sealed directory without rewriting its original manifest or any payload.

Only explicitly listed mesh/texture dependencies may be omitted through reuse.
Always include MJCF/XML, URDF, effective configuration, code/diffs, logs, telemetry,
reports and existing identity/validation evidence. Do not add policy weights merely
for this feature or omit previously required weights. A screenshot is not a texture
resource. Identify actual dependencies from the scene, not by scanning extensions
alone. `resources.json` is an external UTF-8 JSON array of relative paths, e.g.:

```json
["config_snapshot/robot/assets/base_link.stl", "config_snapshot/robot/assets/body.png"]
```

Keep this list outside the sealed directory. The helper accepts common mesh/texture
extensions; it rejects config/model-description formats, unsafe paths and symlinks.
An unsupported resource stays in the full payload; extend the allowlist only after
identifying its role. No filename, timestamp, Git version, sender-local cache or
prior transmission alone proves that the receiver has a file.

### First transfer (or receiver inventory unknown)

```bash
python3 "$SKILL_DIR/scripts/sim2sim_report_bundle.py" pack \
  --source "$SEALED_RUN" --resources "$RESOURCE_LIST" --output "$NEW_ARCHIVE"
```

All files are included. The archive contains `transfer_manifest.json` and
`payload/<original relative paths>`. The helper also emits an archive SHA-256
sidecar; compare that digest on the receiving computer. Existing archives are never
overwritten. Do not send both the old full archive and the new archive for one run.

### Receiver verifies, restores and confirms cached resources

```bash
python3 "$SKILL_DIR/scripts/sim2sim_report_bundle.py" receive \
  --archive "$ARCHIVE" --sha256 "$EXPECTED_ARCHIVE_SHA256" \
  --output "$NEW_RECEIVED_DIRECTORY" --cache "$RESOURCE_CACHE"

python3 "$SKILL_DIR/scripts/sim2sim_report_bundle.py" inventory \
  --root "$NEW_RECEIVED_DIRECTORY/payload" --resources "$RESOURCE_LIST" \
  --cache "$RESOURCE_CACHE" --receiver-id "$RECEIVER_ID" \
  --output "$NEW_INVENTORY_JSON"
```

`receive` restores original paths under `payload/`, verifies every file, and leaves
the original evidence manifest unchanged. Historical absolute paths in reports
describe the originating host; consumers resolve the restored local bundle root
without editing those reports. `inventory` hashes actual received resources, stores
verified bytes under `<cache>/<sha256>`, and creates a new inventory with receiver
ID, hashes and sizes. Send the inventory and its SHA-256 back to the sender through
the existing transfer channel. These are fingerprints, not signatures or automatic
proof of identity. Neither helper contacts another machine.

### Subsequent transfer with that receiver's confirmed inventory

```bash
python3 "$SKILL_DIR/scripts/sim2sim_report_bundle.py" pack \
  --source "$NEXT_SEALED_RUN" --resources "$NEXT_RESOURCE_LIST" \
  --inventory "$RECEIVER_INVENTORY" --inventory-sha256 "$INVENTORY_SHA256" \
  --receiver-id "$RECEIVER_ID" --output "$NEXT_NEW_ARCHIVE"
```

Only a listed resource with matching SHA-256 and size in that receiver inventory
is omitted; changed/new resources are bundled. The transfer manifest records every
original path/hash/size and whether its bytes are bundled or expected from cache,
plus the inventory binding. Non-resource files are always bundled byte-for-byte.
On receipt use the same `receive` command with `--receiver-id "$RECEIVER_ID"`.
It rehashes cached resources; missing/corrupt items produce a machine-readable
list and nonzero exit without publishing a restored directory. Obtain exactly
those bytes or create a new full archive without inventory, then retry with a new
output path. Do not claim semantic completeness until the native evidence checks
also pass. Never alter or discard the original complete archive/evidence to repair
a transfer.

## Portability and final response

Copy the entire skill folder when installing on the deployment computer; keep its
relative references intact. Installation does not authorize package installation,
repository commits/pushes, production edits or remote actions. The user's request
to use `sim2sim-report` selects this mode without activating training workflows.
Report the main result, limitations, report/archive paths, archive hash, and any
required cached resources. Do not claim a transfer or receiver verification took
place when only the sender archive has been created.
