# LW Real-Deployment Issue Register

## Purpose

This file is the authoritative remediation order for the LW real-robot deployment review.

- Modify only one issue at a time.
- The user selects the issue to work on.
- Code changes require explicit user approval before editing.
- Do not include later issues or unrelated cleanup in an approved change.
- After verification, update the selected issue's status and resolution evidence.

## Status Values

- `pending`: Confirmed issue, not yet being changed.
- `in_progress`: Explicitly selected and approved by the user.
- `resolved`: Implemented and verified.
- `deferred`: Intentionally postponed.
- `wont_fix`: Deliberately rejected, with rationale recorded.

## Review Baseline

- Reviewed: 2026-07-29
- Repository HEAD during review: `e4a2765`
- Primary entry point: `src/rl_sar/src/rl_real_LW.cpp`
- Scope: real-robot entry point, FSM, core RL runtime, loop implementation, LW serial SDK, joystick input, motion loader, policy YAML/ONNX/CSV, CMake and ROS launch integration.
- Review method: static inspection, `cppcheck`, filesystem/build-artifact inspection, and offline ONNX Runtime metadata inspection.
- No robot node or serial device was started during the review.
- ONNX dimensions verified:
  - `leg_loco`: `410 -> 10`
  - `wheel_loco`: `195 -> 10`
  - `leg_to_wheel`: `59 -> 10`
  - `wheel_to_leg`: `59 -> 10`
- Both transition CSV files had consistent 17-column rows.
- Deployment baseline was not clean:
  - Three tracked LW ONNX files were modified.
  - `wheel_to_leg/policy.onnx` was untracked.
  - The installed `rl_real_LW` target resolved to a binary built on 2026-07-10, older than the current source change from 2026-07-23.

## Reprioritization Note

- Updated: 2026-08-04
- The current product scope is LW-only, while the repository should retain a
  clear, documented extension boundary for adding other robots later.
- The `Order` column is the single authoritative priority order. Rows are
  grouped by `Priority`; within the same priority, real-robot safety impact and
  dependencies decide the order. `Status` does not create a separate ordering
  scheme; filtering out resolved rows merely shows the next pending work.
- `LW-016` is ordered ahead of `LW-013` within P1 because a safety action that
  is stronger than its trigger warrants can itself cause a fall or prevent a
  controlled recovery. The review must preserve genuinely necessary hard
  stops rather than weakening protections indiscriminately.
- Existing issue IDs remain stable; only the authoritative execution order and
  risk-based priorities changed.

## Post-remediation Review Addendum

- Reviewed: 2026-08-12
- Repository HEAD during review: `57f184e`
- Review scope: all changes from `6761b82` through `57f184e`, including the
  shared LW runtime, real and Sim2Sim adapters, input handling, deployment
  bundle, Jetson build path, and suspended configuration profiler.
- Verification: the current root build completed and its full 31/31 CTest suite
  passed; the configuration analyzer's 8/8 unit tests, Bash syntax checks, and
  `git diff --check` passed. A warning-enabled build and targeted `cppcheck`
  inspection were also run.
- No robot node or serial device was started and no motor command was sent.
- The review confirmed the pending issues `LW-023` through `LW-031` below.

## 2026-08-13 Comprehensive Review Addendum

- Reviewed: 2026-08-13
- Repository HEAD during review: `769b6e8`
- Review scope: the project-owned FDILink IMU driver, real and Sim2Sim entry
  points, shared runtime and safety core, policy-output transport, optional
  actuator models, production bundle and launch integration, dependency
  bootstrap scripts, configuration, and automated tests.
- Verification: a fresh strict `-Wall -Wextra -Wpedantic -Werror` build and its
  full 38/38 CTest suite passed; all tracked Bash and Python files passed syntax
  checks; all 22 vendored LW description assets passed their SHA-256 manifest;
  `cppcheck` and `git diff --check` were also run.
- The existing root build was stale and produced two non-source failures; the
  fresh strict build established the current source baseline.
- An ASan/UBSan build completed, but the full sanitizer test run was not usable
  as a project verdict because the ROS Humble/Conda runtime combination raised
  an allocator mismatch inside external `librcl`/`rcutils` and a later test
  stalled. The temporary build was removed.
- No MuJoCo GUI, ROS real node, serial device, IMU, or motor was started.
- The review confirmed the new pending issues `LW-032` through `LW-041` below.

## Ordered Summary

| Order | ID | Priority | Status | Summary |
|---:|---|---|---|---|
| 1 | LW-023 | P0 / critical | resolved | Disable an enabled-on-power STM32 before fallible real-node startup work |
| 2 | LW-001 | P0 / critical | resolved | Make control-loop shutdown and exception handling fail-safe |
| 3 | LW-002 | P0 / critical | resolved | Require valid, fresh IMU and bilateral motor feedback before commanding |
| 4 | LW-003 | P0 / critical | resolved | Repair serial receive parsing and complete-write handling |
| 5 | LW-004 | P0 / critical | resolved | Remove invalid FSM transition targets and validate all transitions |
| 6 | LW-005 | P0 / critical | resolved | Enforce finite commands and state-aware attitude protection |
| 7 | LW-006 | P0 / critical | resolved | Latch joystick disconnects, clear commands, and validate indices |
| 8 | LW-044 | P1 / high | resolved | Make FDILink frame ingestion complete, initialized, and failure-visible |
| 9 | LW-045 | P1 / high | resolved | Decode FDILink payloads without packed-union or host-layout dependencies |
| 10 | LW-024 | P1 / high | resolved | Make the Sim2Sim physics-thread lifecycle bounded and joinable |
| 11 | LW-025 | P1 / high | resolved | Preserve keyboard velocity commands instead of replacing them every control cycle |
| 12 | LW-026 | P1 / high | resolved | Bind configuration candidates to one exact deployment and comparable reports |
| 13 | LW-027 | P1 / high | resolved | Make the ONNX Runtime dependency reproducible and integrity-verified |
| 14 | LW-007 | P1 / high | resolved | Remove cross-thread data races with coherent snapshots |
| 15 | LW-008 | P1 / high | resolved | Replace split policy queues with one coherent output frame |
| 16 | LW-009 | P1 / high | resolved | Use the configured 60 Hz wheel-to-leg reference rate |
| 17 | LW-010 | P1 / high | resolved | Make deployed binary, configuration, and models reproducible |
| 18 | LW-016 | P1 / high | resolved | Audit every safety trigger for proportional and recoverable behavior |
| 19 | LW-013 | P1 / high | resolved | Validate YAML, mappings, observation sizes, and model outputs |
| 20 | LW-011 | P1 / high | resolved | Make the control loop suitable for deterministic real-time execution |
| 21 | LW-017 | P1 / high | resolved | Bundle and verify the LW IMU/serial runtime dependencies |
| 22 | LW-019 | P1 / high | resolved | Enable a real-robot terminal keyboard recovery channel |
| 23 | LW-020 | P1 / high | resolved | Make the Jetson production inference bootstrap architecture-safe and ONNX-only |
| 24 | LW-021 | P1 / high | resolved | Make Sim2Sim and real deployment share one testable control and safety core |
| 25 | LW-022 | P1 / high | resolved | Measure suspended real-runtime behavior and generate review-only configuration candidates |
| 26 | LW-028 | P2 / medium | resolved | Replace unsafe Sim2Sim signal-handler work with a signal-safe shutdown request |
| 27 | LW-029 | P2 / medium | resolved | Resolve optional actuator models through the selected Sim2Sim policy root |
| 28 | LW-030 | P2 / medium | resolved | Keep inhibited commands and gait-phase observations coherent |
| 29 | LW-018 | P2 / medium | resolved | Unify the build entry point and Jetson detection |
| 30 | LW-012 | P2 / medium | resolved | Harden motion loading and correct its time convention |
| 31 | LW-015 | P2 / medium | resolved | Remove non-LW robot implementations while preserving future extension points |
| 32 | LW-031 | P2 / low | resolved | Restore a warning-clean strict build for maintained LW code and tests |
| 33 | LW-014 | P2 / low | resolved | Isolate and correct production debug/plot publishing |
| 34 | LW-032 | P0 / critical | resolved | Require complete, initialized, independently fresh IMU and AHRS data end to end |
| 35 | LW-033 | P1 / high | resolved | Bind policy-output freshness to the robot-state input snapshot |
| 36 | LW-034 | P1 / high | resolved | Verify downloaded inference runtimes against pinned trusted digests |
| 37 | LW-035 | P2 / medium | resolved | Bind the production launch file into deployment integrity verification |
| 38 | LW-036 | P2 / medium | resolved | Validate optional actuator-network outputs on every Sim2Sim inference |
| 39 | LW-037 | P2 / medium | resolved | Make Sim2Sim SIGTERM and normal ROS shutdown stop the render loop |
| 40 | LW-038 | P2 / medium | resolved | Remove repeated configuration decoding and allocation from the real control cycle |
| 41 | LW-039 | P2 / low | resolved | Reject actuator-model symlinks and policy-root escapes |
| 42 | LW-040 | P2 / low | resolved | Make the polymorphic RL base destruction contract safe |
| 43 | LW-041 | P2 / low | resolved | Make high-rate Sim2Sim plot publishing explicitly opt-in |
| 44 | LW-042 | P2 / medium | resolved | Make real debug telemetry nonblocking, source-fresh, and rate-bounded |
| 45 | LW-043 | P2 / medium | resolved | Retire the Sim2Sim actuator-model runtime while preserving offline training |

---

## [LW-001] Fail-safe loop shutdown and exception boundary

**Priority**: P0 / critical
**Status**: resolved
**Dependencies**: none

### Problem

`LoopFunc::start()` detaches its thread. `shutdown()` therefore cannot join it, even though the API prints that the loop ended. During destruction, a still-running control iteration can send another enable command after the disable packet or access an already-destroyed `RL_Real` object. Exceptions escaping a loop callback also terminate the process without a guaranteed safe command.

### Evidence

- `src/rl_sar/library/core/loop/loop.hpp:32-60`
- `src/rl_sar/src/rl_real_LW.cpp:118-124`
- `src/rl_sar/library/core/loop/loop.hpp:73-93`

### Intended Scope

- Keep loop threads joinable.
- Define a deterministic shutdown order.
- Stop all command-producing threads before sending the final disable command.
- Catch callback exceptions at the thread boundary and trigger a fail-safe path.
- Ensure the final disable operation is attempted only after no thread can re-enable motors.

### Acceptance Criteria

- `shutdown()` waits until the corresponding callback can no longer run.
- No control command can be sent after the final disable command.
- A forced callback exception produces a logged fault and safe shutdown instead of `std::terminate`.
- A mock-SDK test verifies command ordering during normal shutdown and exception shutdown.

### Resolution Evidence

- Resolved: 2026-07-29
- `LoopFunc` now owns a joinable thread, rejects double start, catches callback exceptions at the thread boundary, and joins during `shutdown()` or destruction.
- `RL_Real` now starts workers only after initialization is complete, stops the command-producing loop first, and sends its final disable frame through a latched serialized command gate.
- A loop exception now logs the originating loop and exception, closes the command gate, attempts an emergency disable, and requests ROS shutdown.
- `test_loop_lifecycle` covers join behavior, idempotent shutdown, double start, exception reporting, normal shutdown command ordering, and exception shutdown command ordering.
- Verified with:
  - `cmake --build build/rl_sar --target test_loop_lifecycle rl_real_LW -j2`
  - `ctest --test-dir build/rl_sar --output-on-failure -R '^loop_lifecycle$'`
- Result: `rl_real_LW` built successfully; `loop_lifecycle` passed (1/1).
- Hardware execution was intentionally not performed.

---

## [LW-002] Sensor and communication readiness/freshness gate

**Priority**: P0 / critical
**Status**: resolved
**Dependencies**: LW-001

### Problem

The return value of `InitSerial()` is ignored. `GetState()` returns early when no IMU exists, but `RobotControl()` still advances the FSM and sends commands. Motor feedback is considered updated when either the left or right board updates, and neither motor feedback nor IMU data has a freshness timeout. The ROS launch uses a fixed two-second delay rather than readiness.

### Evidence

- `src/rl_sar/src/rl_real_LW.cpp:45-60`
- `src/rl_sar/src/rl_real_LW.cpp:219-240`
- `src/rl_sar/src/rl_real_LW.cpp:393-445`
- `src/rl_sar/library/thirdparty/robot_sdk/lfr/LW_sdk/LW_sdk.hpp:213-217`
- `src/rl_sar/library/thirdparty/robot_sdk/lfr/LW_sdk/LW_sdk.hpp:270-277`
- `src/rl_sar/launch/rl_real_LW.launch.py:27-34`

### Intended Scope

- Refuse to start control if either serial port fails.
- Track independent left/right feedback timestamps and validity.
- Track IMU timestamp and validate quaternion/gyro freshness.
- Keep motors disabled until all required inputs are valid.
- Enter the fail-safe path when any required source becomes stale.
- Replace or supplement the fixed launch delay with an explicit readiness condition.

### Acceptance Criteria

- Missing IMU, missing right board, or missing left board prevents motor enable.
- Disconnecting any required source while active reaches the safe state within a defined timeout.
- Reconnection behavior is explicit and tested; it must not silently resume motion.
- Startup logs identify exactly which readiness condition is missing.

### Resolution Evidence

- Resolved: 2026-07-29
- Both serial ports must initialize successfully before any worker loop starts; failures identify the affected side, attempt a disable on any reachable side, and terminate startup.
- `LWSDK::RecvFdData()` now reports right and left valid-frame updates independently.
- IMU arrival and each board's valid feedback use independent steady-clock timestamps and a configurable `sensor_timeout` of `0.1 s`.
- Before all three sources are fresh, the control loop remains in a disabled waiting state and reports the exact missing sources without advancing the FSM.
- After readiness has been reached once, any source timeout permanently latches the command gate, sends 20 best-effort disable frames over approximately `100 ms`, and requests ROS shutdown. Reconnection cannot clear the latch.
- The fixed two-second launch delay was removed; actual sensor readiness now controls activation.
- `test_sensor_readiness` covers every missing startup source, the timeout boundary, every individual runtime timeout, and the permanent reconnection latch.
- Verified with:
  - `cmake --build build/rl_sar --target test_sensor_readiness test_loop_lifecycle rl_real_LW -j2`
  - `ctest --test-dir build/rl_sar --output-on-failure -R '^(loop_lifecycle|sensor_readiness)$'`
  - `python3 -m py_compile src/rl_sar/launch/rl_real_LW.launch.py`
  - targeted `cppcheck` of the real entry point and readiness test
- Result: `rl_real_LW` built successfully; both selected CTest tests passed; no new correctness warning was reported by `cppcheck`.
- Hardware execution was intentionally not performed.
- Safety boundary: host-side disable remains best-effort until `LW-003` verifies complete serial writes. A board-local communication watchdog and physical emergency stop are still required to cover cable loss, host failure, and power loss.

---

## [LW-003] Serial parser and transmitter robustness

**Priority**: P0 / critical
**Status**: resolved
**Dependencies**: LW-001

### Problem

When an RX buffer exceeds 4096 bytes, it is cleared and then immediately used in an unsigned subtraction, creating an out-of-bounds path. Port-configuration failures can leave a nonnegative file descriptor open. TX code treats any positive `write()` as success and does not handle partial writes, `EAGAIN`, or delivery failure consistently.

### Evidence

- `src/rl_sar/library/thirdparty/robot_sdk/lfr/LW_sdk/LW_sdk.hpp:91-129`
- `src/rl_sar/library/thirdparty/robot_sdk/lfr/LW_sdk/LW_sdk.hpp:131-189`
- `src/rl_sar/library/thirdparty/robot_sdk/lfr/LW_sdk/LW_sdk.hpp:219-268`

### Intended Scope

- Return immediately or safely resynchronize after clearing an oversized RX buffer.
- Close and invalidate a port after any configuration failure.
- Handle complete packet writes, partial writes, interruption, and nonblocking retry policy.
- Return structured per-board RX/TX status to the caller.

### Acceptance Criteria

- Corrupt input larger than 4096 bytes cannot access an empty buffer.
- Parser tests cover noise, split packets, multiple packets, bad CRC, and recovery.
- A command is successful only if the complete packet is written to each required board.
- Port setup failure cannot leave an apparently usable descriptor.

### Resolution Evidence

- Resolved: 2026-07-29
- Port initialization now configures a local candidate descriptor and publishes it to `LWSDK` only after every required setup step succeeds. All failure paths close the candidate and return the failed operation plus `errno`; unsupported low-latency ioctl behavior remains a nonfatal warning.
- Feedback uses a bounded streaming parser that preserves incomplete headers/frames, discards noise incrementally, copies packet bytes with `memcpy`, validates tail and CRC, parses every available frame, and applies only the latest valid frame.
- Each receive call drains at most 4096 bytes per side and returns independent byte, packet, discard, CRC, format, and read-error status.
- Command transmission tracks per-side offsets, retries partial writes and `EINTR`, waits through `EAGAIN/EWOULDBLOCK` with `poll()`, services both sides under one configurable `serial_write_timeout` of `0.002 s`, and succeeds only when both complete packets have entered the kernel serial queues.
- `RL_Real` checks the structured result for startup disable, waiting-state disable, motor-protection disable, normal control, emergency disable, and final shutdown. Normal-send failure is evaluated only after releasing `CommandGate`, avoiding recursive-lock deadlock before entering fail-safe.
- `test_lw_serial_sdk` covers split headers/frames, noise, multiple frames, bad CRC, bad tail, 8192-byte corrupt input and recovery, PTY bilateral RX/TX and mapping, repeated configuration failure without descriptor leakage, partial nonblocking writes, shared-deadline timeout, and continued delivery to a healthy side when its peer fails.
- Verified with:
  - `cmake --build build/rl_sar --target test_lw_serial_sdk test_sensor_readiness test_loop_lifecycle rl_real_LW rl_sim_LW -j2`
  - `ctest --test-dir build/rl_sar --output-on-failure --repeat until-fail:20 -R '^(lw_serial_sdk|sensor_readiness|loop_lifecycle)$'`
  - standalone `-Wall -Wextra -Wpedantic` compilation
  - standalone AddressSanitizer and UndefinedBehaviorSanitizer execution
  - targeted `cppcheck`
- Result: both LW executables built; all three selected tests passed for 20 consecutive runs; sanitizer execution found no memory or undefined-behavior failure; the descriptor-leak regression passed; `cppcheck` reported no new correctness warning.
- Hardware execution was intentionally not performed.
- Safety boundary: a complete host `write()` proves only that the kernel accepted the packet, not that the controller received or executed it. Board acknowledgements, board-local communication watchdogs, and a physical emergency stop remain necessary for end-to-end assurance.

---

## [LW-004] FSM transition correctness

**Priority**: P0 / critical
**Status**: resolved
**Dependencies**: LW-001

### Problem

Both morphology-transition states can return the nonexistent state name `RLFSMStateGetUp`. `FSM::Run()` then calls `states_.at(next)`, which throws. The loop has no safe exception boundary in the current implementation. Passive-mode help text also disagrees with the actual gamepad mapping.

### Evidence

- `src/rl_sar/fsm_robot/fsm_LW.hpp:460-475`
- `src/rl_sar/fsm_robot/fsm_LW.hpp:542-556`
- `src/rl_sar/library/core/fsm/fsm.hpp:62-95`
- `src/rl_sar/fsm_robot/fsm_LW.hpp:15-44`

### Intended Scope

- Replace invalid target names with the intended registered state names.
- Validate transition targets before using `.at()`.
- Make displayed operator instructions match actual mappings.
- Add a transition-table test covering every state and accepted input.

### Acceptance Criteria

- Every `CheckChange()` result is present in the factory's registered state set.
- Pressing every supported input in every state cannot throw.
- Invalid external requests are rejected without leaving the current safe state.

### Resolution Evidence

- Resolved: 2026-07-29
- The invalid `RLFSMStateGetUp` branches were removed from both morphology-transition states. As explicitly requested, `0/A` is ignored during leg-to-wheel and wheel-to-leg transitions instead of interrupting the active motion with a get-up state.
- Morphology transitions still accept `P/LB_X` for passive mode and `9/B` for get-down; successful motion completion still requests the corresponding registered locomotion state.
- `FSM::Run()` now resolves a `CheckChange()` target with `find()` before entering change mode. An unregistered target is logged and rejected while the FSM remains in its current state, so the transition path no longer throws through `unordered_map::at()`.
- Passive-mode operator guidance now matches the implemented wheel get-up mapping: keyboard `2` or gamepad `Y`.
- `test_lw_fsm_transitions` covers all eight registered LW states, every accepted keyboard/gamepad transition, every value in both input enums, ignored `0/A` input during morphology transitions, factory registration consistency, and rejection of invalid internal and external targets.
- Verified with:
  - `cmake --build build/rl_sar --target test_lw_fsm_transitions rl_real_LW rl_sim_LW -j2`
  - `ctest --test-dir build/rl_sar --output-on-failure --repeat until-fail:20 -R '^(loop_lifecycle|sensor_readiness|lw_serial_sdk|lw_fsm_transitions)$'`
  - targeted `cppcheck` of the FSM core, LW FSM, and transition test
- Result: both LW executables built successfully; all four selected tests passed for 20 consecutive runs; `cppcheck` reported only pre-existing style advisories and no new correctness warning.
- Hardware execution was intentionally not performed.

---

## [LW-005] Finite command validation and active protection

**Priority**: P0 / critical
**Status**: resolved
**Dependencies**: LW-001, LW-002

### Problem

`TorqueProtect()` only prints warnings. Policy action limits of ±100 permit very large position and wheel-velocity targets after scaling. NaN checking only covers right-leg action targets, does not reject infinities or bad gains, and still transmits the packet. Attitude protection is delayed and does not cover all motion states.

### Evidence

- `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp:358-375`
- `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp:386-436`
- `src/rl_sar/src/rl_real_LW.cpp:320-336`
- `src/rl_sar/src/rl_real_LW.cpp:447-479`
- `src/rl_sar/library/thirdparty/robot_sdk/lfr/LW_sdk/LW_sdk.hpp:244-267`
- `policy/LW/robot_lab/*/config.yaml`

### Intended Scope

- Require `std::isfinite()` for IMU quaternion/gyro, motor feedback
  position/velocity/estimated torque, raw policy actions, computed policy
  outputs, final robot commands, and low-level serial command fields.
- Reject negative proportional or derivative gains, without imposing a maximum
  gain.
- Reject invalid policy frames before enqueueing or transmitting them.
- Apply the 75-degree roll/pitch fail-safe only in get-down, both locomotion
  states, and both morphology-transition states.
- Do not apply attitude protection in passive or either get-up state.
- Keep the existing `[-100, 100]` action clipping, target position/velocity
  behavior, and warning-only predicted-torque protection unchanged.

### Acceptance Criteria

- NaN or infinity at any validated control stage triggers the permanent
  fail-safe path.
- Negative gains trigger the permanent fail-safe path.
- Invalid low-level command values are never written to either serial port.
- Roll or pitch beyond 75 degrees triggers the fail-safe path in exactly the
  five protected states and is ignored by attitude protection in passive and
  both get-up states.
- Large but finite actions, targets, gains, and predicted torque do not trigger
  fail-safe solely because of magnitude; existing action clipping remains
  unchanged.

### Resolution Evidence

- Resolved: 2026-07-29
- IMU quaternion/gyro and bilateral motor position, velocity, and estimated
  torque are checked for correct size and finite values before entering the
  controller.
- Raw model actions are checked before the existing clipping operation.
  Computed position, velocity, and torque outputs are checked before queueing.
  The final `RobotCommand` is checked before opening the command gate.
- The serial SDK independently rejects non-finite action/gain fields and
  negative gains before packet construction, returning bilateral `EINVAL`
  without writing bytes to either serial port.
- Attitude protection is evaluated before control and again after the FSM runs,
  so the first command produced after entering a protected state cannot bypass
  the 75-degree roll/pitch limit. Protection is enabled only for get-down, leg
  locomotion, wheel locomotion, leg-to-wheel, and wheel-to-leg.
- Per the approved scope, passive and both get-up states do not use attitude
  protection. No position/velocity range, maximum-gain, or predicted-torque
  fail-safe was added; existing `[-100, 100]` action clipping and warning-only
  `TorqueProtect()` behavior remain unchanged.
- `test_lw_control_safety` covers finite feedback, actions, outputs, final
  commands, negative gains, absence of magnitude limits, and the exact protected
  and unprotected state sets.
- `test_lw_serial_sdk` verifies that NaN, infinity, and negative gains produce
  zero serial bytes, while very large finite gains still transmit.
- Verified with:
  - `cmake --build build/rl_sar --target rl_real_LW rl_sim_LW test_lw_control_safety test_lw_serial_sdk test_lw_fsm_transitions test_sensor_readiness test_loop_lifecycle -j2`
  - `ctest --test-dir build/rl_sar --output-on-failure --repeat until-fail:20 -R '^(lw_control_safety|lw_serial_sdk|lw_fsm_transitions|sensor_readiness|loop_lifecycle)$'`
  - standalone `-Wall -Wextra -Wpedantic` compilation
  - standalone AddressSanitizer and UndefinedBehaviorSanitizer execution of the
    serial SDK test
  - targeted `cppcheck`
- Result: both LW executables built successfully; all five selected tests passed
  for 20 consecutive runs; sanitizer execution found no memory or
  undefined-behavior failure; `cppcheck` reported only a pre-existing
  constructor-initialization performance suggestion.
- Hardware execution was intentionally not performed.

---

## [LW-006] Joystick disconnect and input validation

**Priority**: P0 / critical
**Status**: resolved
**Dependencies**: LW-001, LW-002

### Problem

Joystick reads cannot distinguish no event from disconnect. Cached axis values remain nonzero, so loss of a wireless controller can preserve the last velocity command indefinitely. Button and axis numbers are used as array indices without bounds checks.

### Evidence

- `src/rl_sar/src/rl_real_LW.cpp:481-587`
- `src/rl_sar/library/thirdparty/joystick/joystick.cc:47-68`

### Intended Scope

- Do not add a deadman button or change the existing button, axis, scaling, or
  five-percent deadzone behavior.
- Distinguish an idle nonblocking device from EOF, disconnect, partial reads,
  and other read failures.
- On joystick unavailability, permanently latch joystick input off, clear all
  cached buttons and axes, and force velocity commands to zero.
- Keep motor control and the current FSM active instead of closing the command
  gate, disabling motors, or shutting down ROS.
- Do not automatically accept input after reconnection; restoring joystick
  input requires restarting the node.
- Validate button and axis indices before array access.

### Acceptance Criteria

- `EAGAIN`, `EWOULDBLOCK`, and `EINTR` do not falsely mark an idle joystick as
  disconnected.
- EOF, invalid descriptors, device errors, and partial reads permanently latch
  joystick input unavailable.
- A latched joystick fault clears cached input and forces `x/y/yaw` and Gamepad
  state to zero/none in the control path.
- Joystick loss does not send a motor-disable command, stop the current FSM, or
  shut down ROS.
- A stopped or missing joystick cannot initiate get-up, locomotion, or
  morphology transitions through stale Gamepad state.
- Out-of-range event numbers are ignored and logged without memory access
  outside arrays.
- Existing deadzone and valid input behavior remain unchanged.

### Resolution Evidence

- Resolved: 2026-07-29
- A LW-local nonblocking joystick reader now reports complete events, no-data,
  EOF/disconnect, and malformed/error results separately without modifying the
  third-party joystick submodule.
- LW real and simulation input paths use fixed-size button/axis arrays and
  validate every incoming event before indexing them.
- Startup open failure or a runtime terminal read result clears all cached
  joystick state and permanently latches joystick input unavailable. Subsequent
  calls cannot clear the latch or resume Gamepad commands.
- The 200 Hz control path clears `x/y/yaw` and both Gamepad state fields while
  the latch is set. The policy path independently publishes a zero command
  observation, so the next inference frame cannot reuse the prior joystick
  velocity.
- Per the approved safety behavior, joystick loss does not call
  `EnterFailSafe()`, close `CommandGate`, send motor-disable packets, or request
  ROS shutdown. The current FSM and motor support remain active. The fault gate
  does not clear keyboard state, but the current real executable does not start
  `KeyboardInterface()`, so terminal keyboard input is not an available
  recovery path.
- The existing axis normalization, `vel_command` scaling, input mapping, and
  strict five-percent deadzone are unchanged.
- `test_lw_joystick_safety` covers idle descriptors, complete events, EOF,
  invalid descriptors, null destinations, partial reads, button/axis boundaries,
  the five-percent deadzone boundary, full cache clearing, and the permanent
  fault latch.
- Verified with:
  - `cmake --build build/rl_sar --target test_lw_joystick_safety rl_real_LW rl_sim_LW -j2`
  - `ctest --test-dir build/rl_sar --output-on-failure --repeat until-fail:20 -R '^(loop_lifecycle|sensor_readiness|lw_serial_sdk|lw_fsm_transitions|lw_control_safety|lw_joystick_safety)$'`
  - standalone `-Wall -Wextra -Wpedantic` compilation
  - standalone AddressSanitizer and UndefinedBehaviorSanitizer execution
  - targeted `cppcheck`
- Result: both LW executables built successfully; all six selected tests passed
  for 20 consecutive runs; sanitizer execution found no memory or
  undefined-behavior failure; `cppcheck` reported only pre-existing
  constructor-initialization performance suggestions.
- Hardware execution was intentionally not performed.
- Safety boundary: if a wireless receiver remains present and reports neither
  release events nor a device/read error after radio-link loss, `/dev/input/js*`
  alone cannot prove that the handheld controller is disconnected.
- Safety boundary: stopping or restarting the real executable while the robot
  is upright sends a final motor-disable command and can cause a fall. After a
  latched joystick fault, the robot must be mechanically supported before
  shutdown; LW-006 does not provide an in-process controlled GetDown path.

---

## [LW-007] Coherent cross-thread state

**Priority**: P1 / high
**Status**: resolved
**Dependencies**: LW-001, LW-002

### Problem

The policy thread locks `state_mutex` while copying `robot_state`, but the control thread does not take that mutex while writing it. `control`, `params`, `rl_init_done`, `episode_length_buf`, plot data, and `motion_loader_lw` are also shared without a consistent synchronization policy. Policy switching can expose partially updated configuration and state.

### Evidence

- `src/rl_sar/src/rl_real_LW.cpp:127-203`
- `src/rl_sar/src/rl_real_LW.cpp:257-341`
- `src/rl_sar/src/rl_real_LW.cpp:393-445`
- `src/rl_sar/fsm_robot/fsm_LW.hpp:242-558`
- `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp:317-356`

### Intended Scope

- Define ownership for control state, robot state, active policy configuration, and motion reference.
- Publish immutable snapshots between threads or use a consistent lock/atomic scheme.
- Make policy activation and deactivation an atomic state transition.
- Ensure debug publishing reads a coherent snapshot.

### Acceptance Criteria

- No shared mutable control data is accessed without its documented synchronization mechanism.
- Policy switches cannot combine old observations with new configuration/model data.
- ThreadSanitizer or equivalent stress testing reports no races in the LW control path.

### Resolution Evidence

- Resolved: 2026-07-29
- The 200 Hz control thread is now the sole owner of `control`, the FSM,
  `robot_state`, `robot_command`, and the LW motion loader. The joystick worker
  publishes coherent velocity and sequenced Gamepad input through a protected
  mailbox; the simulation keyboard reader also runs in the control thread.
- Each inference frame reads one combined robot-state/control snapshot. Debug
  callbacks read dedicated complete snapshots instead of live control data, and
  the simulator actuator network consumes a generation-matched inference
  snapshot.
- All four policy YAML configurations and preloaded models are assembled before
  worker startup into read-only policy definitions. Activation publishes one
  atomic definition/model/motion-length context with a monotonically increasing
  generation; the inference thread keeps that same context for the whole frame
  and resets its private observation/history/output workspace only when the
  generation changes.
- Morphology references and inference progress are immutable, generation-tagged
  snapshots. A reference or progress value from another generation is rejected,
  so a policy frame cannot combine a new model/configuration with an old motion
  reference.
- Snapshot buffers copy into retained storage rather than transferring
  temporary vector ownership every control cycle, reducing allocation jitter in
  the 200 Hz path.
- `test_lw_runtime_sync` stresses whole-frame publication, atomic policy-context
  replacement, and coherent/sequenced input with one writer, four concurrent
  readers where applicable, and 50,000 iterations per worker.
- Verified with:
  - `cmake --build build/rl_sar --target rl_sdk test_lw_runtime_sync test_lw_fsm_transitions test_lw_control_safety test_lw_joystick_safety rl_real_LW rl_sim_LW -j2`
  - `ctest --test-dir build/rl_sar --output-on-failure -R 'loop_lifecycle|sensor_readiness|lw_serial_sdk|lw_fsm_transitions|lw_control_safety|lw_joystick_safety|lw_runtime_sync'`
  - `ctest --test-dir build/rl_sar --output-on-failure --repeat until-fail:20 -R 'lw_fsm_transitions|lw_runtime_sync'`
  - 50 additional consecutive standalone `test_lw_runtime_sync` runs
  - standalone AddressSanitizer and UndefinedBehaviorSanitizer execution
  - `-Wall -Wextra -Wpedantic` builds of the core library, synchronization
    test, and both LW executables
- Result: both LW executables and all selected targets built successfully; all
  seven selected CTest tests passed, the FSM/synchronization tests passed 20
  consecutive CTest runs, the synchronization test passed 50 additional runs,
  and sanitizer execution reported no memory or undefined-behavior failure.
  Strict-warning output contained only pre-existing initialization,
  unused-parameter, and third-party warnings.
- ThreadSanitizer instrumentation compiled successfully, but its runtime could
  not start in this container (`ThreadSanitizer: unexpected memory mapping`);
  the concurrent stress test, repeated runs, and ASan/UBSan execution were used
  as the available equivalent verification.
- Hardware execution was intentionally not performed.
- Scope boundary: the three existing position/velocity/torque queues remain
  separate and unversioned. A complete inference frame is internally bound to
  one generation, but stale or cross-paired queue consumption remains LW-008
  and was intentionally not changed here.

---

## [LW-008] Atomic policy output transport

**Priority**: P1 / high
**Status**: resolved
**Dependencies**: LW-007

### Problem

Position, velocity, and torque are pushed into separate queues. The consumer can pop a position before its velocity is available, discard it because of short-circuit evaluation, and later pair outputs from different inference frames. Queues are not cleared or versioned during policy changes.

### Evidence

- `src/rl_sar/src/rl_real_LW.cpp:320-333`
- `src/rl_sar/library/core/rl_sdk/rl_sdk.hpp:205-214`
- `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp:693-713`

### Intended Scope

- Replace split queues with one frame containing position, velocity, torque, timestamp, sequence number, and policy generation.
- Prefer explicit latest-frame semantics for real-time control.
- Reject stale frames and frames from an inactive policy generation.

### Acceptance Criteria

- A consumer can never observe a partial policy output.
- Policy transitions cannot consume frames from the prior policy.
- Delayed inference has a documented stale-frame response.

### Resolution Evidence

- Resolved: 2026-07-31
- LW policy inference now publishes position, velocity, torque, source
  timestamp, global sequence number, inference frame, and policy generation as
  one immutable latest-frame snapshot. The real and simulation producers no
  longer publish LW outputs through the three independent queues.
- Publication rejects incomplete payloads and generations that are not active.
  Activation and deactivation clear the latest-frame slot, and consumers
  validate the frame against the currently active generation before applying
  it. A policy-switch race may leave an old frame observable, but its generation
  cannot pass the consumer check and therefore cannot be commanded.
- Both the four LW FSM states and the simulator actuator-network path consume
  the same coherent frame. The generic queue-based `RLControl()` path remains
  available for non-LW robots and has a compatibility regression test.
- Freshness is measured with `steady_clock` from immediately before inference.
  The maximum accepted age is three policy periods
  (`3 * dt * decimation`, currently 60 ms). At the boundary the frame remains
  valid; after the boundary the consumer keeps the last already-applied command
  and gains, emits a rate-limited warning, and automatically resumes on the
  next complete fresh frame. It does not switch to passive, disable motors, or
  exit ROS solely because inference output is delayed.
- `test_lw_policy_output_transport` covers partial-frame and inactive-generation
  rejection, latest-frame overwrite behavior, generation clearing, freshness
  boundaries, concurrent readers, and non-LW queue compatibility.
- Verified with:
  - builds of `rl_sdk`, `rl_real_LW`, `rl_sim_LW`,
    `test_lw_policy_output_transport`, and the existing LW test targets
  - all eight selected CTest tests:
    `loop_lifecycle`, `sensor_readiness`, `lw_serial_sdk`,
    `lw_fsm_transitions`, `lw_control_safety`, `lw_joystick_safety`,
    `lw_runtime_sync`, and `lw_policy_output_transport`
  - 20 consecutive CTest runs each of `lw_fsm_transitions`,
    `lw_runtime_sync`, and `lw_policy_output_transport`
  - AddressSanitizer and UndefinedBehaviorSanitizer execution of
    `test_lw_policy_output_transport`
  - `-Wall -Wextra -Wpedantic` builds of the core library, the new transport
    test, and both LW executables; only pre-existing initialization,
    unused-parameter, and third-party warnings were reported
- Hardware execution was intentionally not performed.

---

## [LW-009] Wheel-to-leg motion reference rate

**Priority**: P1 / high
**Status**: resolved
**Dependencies**: LW-007, LW-008

### Problem

`wheel_to_leg/config.yaml` specifies `motion_fps: 60.0`, but the FSM constructs its motion loader at `1 / (dt * decimation)`, currently 50 Hz. This stretches the reference and changes computed reference velocities.

### Evidence

- `src/rl_sar/fsm_robot/fsm_LW.hpp:478-535`
- `policy/LW/robot_lab/wheel_to_leg/config.yaml:62-65`
- `src/rl_sar/library/core/motion_loader/motion_loader_lw.cpp:5-15`

### Intended Scope

- Use the configured motion-file frame rate consistently for both transition directions.
- Keep controller inference frequency separate from motion-file sampling frequency.
- Verify interpolation and velocity scaling against the training/export convention.

### Acceptance Criteria

- Both transition loaders report the configured 60 Hz source rate.
- Reference duration and velocities match an offline calculation from the CSV.
- A regression test compares selected timestamps against expected interpolated values.

### Resolution Evidence

- Resolved: 2026-07-31
- Both morphology-transition states now obtain the motion source rate from the
  immutable policy configuration's `motion_fps` field. Wheel-to-leg no longer
  derives a 50 Hz source rate from the independent 20 ms policy inference
  period.
- Policy inference remains at 50 Hz. `MotionLoaderLW` continues to interpolate
  the configured 60 Hz motion source at each inference timestamp, so this
  change does not alter control-loop or inference scheduling.
- Offline verification found 167 frames for leg-to-wheel and 170 frames for
  wheel-to-leg. Under the current loader convention their configured 60 Hz
  durations are 2.78333 s and 2.83333 s respectively; wheel-to-leg was
  previously stretched to 3.4 s. Joint reference velocities now use
  `(q[n+1] - q[n]) * 60` instead of a 50 Hz scale.
- `test_lw_motion_reference_rate` verifies that source FPS remains independent
  of policy `dt * decimation`, both YAML files select 60 Hz, CSV frame counts
  and widths are as expected, durations and forward-difference velocities
  match offline calculations, midpoint interpolation is correct, and the final
  CSV frame is reached.
- Verified with:
  - builds of `test_lw_motion_reference_rate`, `test_lw_fsm_transitions`,
    `rl_real_LW`, and `rl_sim_LW`
  - all nine selected CTest tests:
    `loop_lifecycle`, `sensor_readiness`, `lw_serial_sdk`,
    `lw_fsm_transitions`, `lw_control_safety`, `lw_joystick_safety`,
    `lw_runtime_sync`, `lw_policy_output_transport`, and
    `lw_motion_reference_rate`
  - 20 consecutive CTest runs each of `lw_fsm_transitions` and
    `lw_motion_reference_rate`
  - a `-Wall -Wextra -Wpedantic` build of the new test and both LW
    executables; only pre-existing initialization, unused-parameter, and
    third-party warnings were reported
  - targeted `cppcheck` of the new test with no findings
- Hardware execution was intentionally not performed.
- Scope boundary: `MotionLoaderLW` still uses its existing `num_frames * dt`
  duration convention. Defining and correcting that convention remains
  LW-012 and was intentionally not changed here.

---

## [LW-010] Reproducible deployment artifacts

**Priority**: P1 / high
**Status**: resolved
**Dependencies**: LW-004, LW-005, LW-009

### Problem

The installed launcher currently resolves to an executable older than the source. The executable embeds an absolute workspace `POLICY_DIR`, so it loads mutable working-tree models rather than a versioned installed policy set. Current LW model files are also modified/untracked.

### Evidence

- `src/rl_sar/CMakeLists.txt:42-46`
- `src/rl_sar/CMakeLists.txt:627-655`
- `src/rl_sar/CMakeLists.txt:672-678`
- Filesystem/build status recorded in the review baseline above.

### Intended Scope

- Resolve policies through the installed package share directory or an explicit deployment bundle.
- Build in a clean tree and record source commit, configuration hashes, and model hashes.
- Fail startup when the expected bundle is incomplete or mismatched.
- Ensure launch selects the freshly built target.

### Acceptance Criteria

- A deployment manifest identifies the binary, source commit, YAML, ONNX, and CSV hashes.
- Moving the installed workspace does not break policy lookup.
- Dirty or untracked production model files are not silently loaded.
- The launched binary contains all approved source changes.

### Resolution Evidence

- Resolved: 2026-08-04
- Implementation commits:
  - `8803e6d` adds the verified deployment bundle, approved ONNX models,
    manifest generator, policy-root isolation, fail-closed startup, and tests.
  - `8d18532` initializes pinned Git submodules in the clean temporary worktree.
  - `2cf311a` adds a verification-only startup path that performs the same
    deployment checks without initializing ROS, joystick, serial, or control
    workers.
- `build_lw_deployment.sh` built commit
  `2cf311adf55f3a2afffd4c18751cc11877a218de` from a clean detached worktree in
  `Release` mode and installed a non-symlink bundle at
  `build/lw010_release_2cf311a/`.
- The generated manifest records the source commit, the installed
  `rl_real_LW` SHA-256
  `5d7d92057fa6f1aced0e0f71d9a383754e42a5bcbc25d71c0b01ee2c72e0a667`,
  and hashes for the five YAML files, four ONNX models, and two transition CSV
  files.
- The installed executable and deployment tree contained no symbolic links.
  The real executable uses an explicit verified policy root and no longer
  contains `/home/lfr/rl_sar/policy`.
- The complete install prefix was copied to
  `build/lw010_relocated_2cf311a/`. After sourcing the relocated prefix,
  `ros2 pkg prefix rl_sar` resolved to that directory and
  `rl_real_LW --verify-deployment-only` passed all binary, source-commit,
  manifest, path-containment, and resource-hash checks.
- `test_lw_deployment_bundle` covers valid and relocated bundles, missing or
  tampered assets, tampered executables, source-commit mismatch, symbolic
  links, and non-normalized paths. The Python generator tests cover complete
  manifests, symlink rejection, non-Release builds, and invalid commits.
- Verified with a successful `rl_real_LW` build and 11 selected LW CTest tests:
  `loop_lifecycle`, `sensor_readiness`, `lw_serial_sdk`,
  `lw_deployment_bundle`, `lw_deployment_manifest_generator`,
  `lw_fsm_transitions`, `lw_control_safety`, `lw_joystick_safety`,
  `lw_runtime_sync`, `lw_policy_output_transport`, and
  `lw_motion_reference_rate`.
- Targeted `-Wall -Wextra -Wpedantic -Werror`, AddressSanitizer,
  UndefinedBehaviorSanitizer, `cppcheck`, Python syntax/tests, shell syntax,
  and `git diff --check` also passed.
- Hardware execution was intentionally not performed.
- Scope boundary: the manifest binds the installed executable and all LW
  policy/configuration artifacts required by this issue. Compatible external
  ROS, inference-runtime, Python, and operating-system shared libraries remain
  deployment prerequisites and are not vendored or hashed by this manifest.

---

## [LW-016] Safety-action proportionality and recovery audit

**Priority**: P1 / high
**Status**: resolved
**Dependencies**: LW-001, LW-002, LW-003, LW-005, LW-006, LW-011

### Problem

LW safety behavior has accumulated across loop exceptions, sensor freshness,
serial delivery, feedback and command validation, attitude limits, motor
protection, joystick loss, timing degradation, startup, and shutdown. There is
no single decision matrix proving that every trigger invokes an action with the
right severity and recovery policy.

`EnterFailSafe()` currently latches the command gate closed, sends 20 hard
motor-disable packets over roughly 100 ms, and requests ROS shutdown. That is
appropriate only when immediately removing torque is safer than retaining a
controlled state. Applied to a transient or recoverable condition, the same
behavior could make an upright robot fall or remove the operator's opportunity
to request `GetDown`. Conversely, weakening a genuinely fatal response could
leave unsafe commands active. The complete boundary therefore needs an
evidence-based proportionality audit rather than a blanket relaxation.

### Evidence

- `src/rl_sar/src/rl_real_LW.cpp:492-746`
- `src/rl_sar/src/rl_real_LW.cpp:776-787`
- `src/rl_sar/src/rl_real_LW.cpp:1298-1344`
- `src/rl_sar/library/core/loop/command_gate.hpp`
- `src/rl_sar/library/core/safety/lw_control_safety.hpp`
- `src/rl_sar/library/core/safety/lw_joystick_safety.hpp`
- Resolution evidence and documented safety boundaries for `LW-001` through
  `LW-006` and `LW-011`.

### Intended Scope

- Inventory every path that warns, zeros velocity, latches an input source,
  requests a state transition, sends damping/disable commands, closes
  `CommandGate`, calls `EnterFailSafe()`, or shuts down ROS.
- For every trigger, record the originating fault, robot/FSM state, persistence
  or debounce rule, assumed hardware condition, selected action, operator
  options, restart requirement, and worst credible consequence of both acting
  and not acting.
- Define an explicit action hierarchy such as diagnostic-only, bounded command
  reduction, zero-velocity latch with operator-controlled `GetDown`, controlled
  passive/damping behavior, and immediate hard disable plus shutdown. Do not
  assume every tier is safe in every state.
- Review thresholds, hysteresis, transient handling, retry limits, latch
  permanence, and recovery authority. A recoverable event must not silently
  escalate to an irreversible action without a documented reason.
- Require a specific physical-safety justification for every hard-disable path,
  including why immediate torque removal is safer than maintaining support or
  allowing a controlled descent in the affected state.
- Separate host guarantees from motor-board watchdog, acknowledgement, power,
  cable-loss, physical emergency-stop, and mechanical-support assumptions.
- Add focused tests for the approved decision matrix. Any resulting behavioral
  change remains confined to `LW-016` and requires the normal explicit code
  approval before implementation.

### Acceptance Criteria

- A reviewed decision matrix accounts for every production LW safety trigger
  and every terminal safety action; no call site is left with implicit
  severity or recovery semantics.
- Every `EnterFailSafe()`/hard-disable call site has evidence that immediate
  torque removal is the least dangerous available response for its fault and
  applicable FSM states.
- Transient or degraded conditions use no stronger action, no longer latch, and
  require no more operator intervention than the approved matrix specifies.
- Cases that retain FSM input state exactly when `GetDown` remains safe and
  available; cases with corrupt state/command data do not claim a controlled
  recovery that cannot be guaranteed.
- Logs distinguish degraded, recoverable, latched, and fatal states and state
  whether restart, `GetDown`, hardware disable, or physical support is required.
- Automated tests force each trigger and verify command-gate state, motor packet
  ordering, ROS-shutdown behavior, retained/cleared inputs, latch persistence,
  and permitted recovery actions without weakening unrelated protections.
- Deployment documentation states the remaining board-watchdog and physical
  emergency-stop assumptions and records any behavior that still requires
  suspended-robot validation.

### Resolution Evidence

- Resolved: 2026-08-09T11:32:37+08:00
- Commit: this commit
- Added `lw_safety_policy.hpp` as the single production decision matrix for 27
  runtime and lifecycle events. It assigns explicit S0 diagnostic, S1 input
  degradation, S2 controlled damping, S3 hard-disable, S4 hard-disable plus
  ROS shutdown, startup-abort, and orderly-shutdown actions. The supervisor
  latches severity monotonically and de-duplicates repeated diagnostic events.
- Joystick loss and joystick-loop exceptions now retain motor/FSM operation
  with input inhibition. Control timing degradation retains the approved
  zero-velocity latch and operator-requested `GetDown` path.
- Inference-loop exceptions, invalid policy action/output/configuration, and
  missing, incomplete, generation-mismatched, regressed, or stale policy
  output now permanently stop policy acceptance and use a `Kp=0`, `Kd=5`,
  zero-velocity/torque Passive damping command. A stale output discovered in
  the control thread overwrites the old command in that control cycle; all
  other S2 events are applied no later than the next executable control cycle.
  The FSM transition is requested and completed only by the control thread.
- Motor-board hardware faults now produce a one-shot S3 hard-disable latch and
  keep ROS alive for diagnostics. Control-loop/unknown-loop exceptions,
  explicitly enabled fatal timing thresholds, stale or invalid feedback,
  missing FSM state, protected-state attitude violation, invalid/null final
  commands, read failures, and incomplete sends remain S4. Hard-disable and
  ROS-shutdown latches are separate, so a later S4 event can still escalate an
  existing S3 latch.
- Startup serial, initial-disable, and loop-start failures retain best-effort
  disable followed by startup abort. Normal shutdown ordering is unchanged.
  Parser errors with continuing fresh frames and torque-limit reports remain
  diagnostic-only; feedback freshness is the escalation boundary.
- Added `test_lw_safety_policy`; expanded `test_lw_control_safety` and
  `test_lw_policy_output_transport` for complete matrix ordering/actions,
  source-specific loop handling, monotonic escalation, repeated-event
  de-duplication, failed-send escalation, normal shutdown, Passive damping
  contents, and bounded initial-wait/stale-output behavior.
- Debug verification built both `rl_real_LW` and `rl_sim_LW` and passed all 13
  registered CTest tests. A fresh Release configuration built both executables
  and passed the three directly affected tests. The three affected tests also
  passed under UndefinedBehaviorSanitizer.
- The pure safety-policy test passed under combined AddressSanitizer and
  UndefinedBehaviorSanitizer. Full `rl_sdk` ASan execution was not claimed:
  on this host the linked tests intermittently consumed a CPU indefinitely,
  and terminating them caused the ASan runtime to recursively print
  `DEADLYSIGNAL` without a usable application stack. Normal, Release, UBSan,
  and 50,000-frame concurrency runs all passed.
- A strict `-Wall -Wextra -Wpedantic -Werror` build passed for the new test,
  affected tests, core SDK, and `rl_real_LW` after only the repository's
  pre-existing `reorder`, aggregate-initializer, and unused-parameter warning
  classes were left as warnings. Targeted `cppcheck` found only the existing
  `CSVInit(std::string)` performance advisory and constructor/worker-binding
  heuristic; no new correctness finding was reported. `git diff --check`
  passed.
- `docs/LW_BUILD_DEPLOYMENT_CN.md` now explains the S0-S4 behavior in operator
  language and explicitly separates host packet-send guarantees from motor
  acknowledgement, board watchdog, physical emergency stop, mechanical
  support, Passive damping, and 75-degree attitude assumptions.
- Hardware execution was intentionally not performed. Before first ground
  experiment, S1-S4 fault injection, `motors_disable` acknowledgement/latency,
  serial-loss watchdog behavior, `Kd=5` Passive damping, and the 75-degree S4
  response still require suspended-robot validation with physical emergency
  stop, support, and exclusion distance in place.

---

## [LW-013] Configuration and dimension validation

**Priority**: P1 / high
**Status**: resolved
**Dependencies**: LW-005, LW-010

### Problem

The control path assumes every YAML vector has the correct length and every mapping/index is valid. Model input/output sizes are not explicitly checked against configuration before control starts. `InitObservations()` also initializes a `w,x,y,z` quaternion as `{0,0,0,1}` rather than the identity `{1,0,0,0}`.

### Evidence

- `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp:221-235`
- `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp:317-375`
- `src/rl_sar/src/rl_real_LW.cpp:127-203`
- `src/rl_sar/src/rl_real_LW.cpp:447-479`
- `policy/LW/base.yaml`
- `policy/LW/robot_lab/*/config.yaml`

### Intended Scope

- Validate required keys, vector lengths, mapping uniqueness, index ranges, positive timing values, and finite limits.
- Verify ONNX input/output shapes against computed observation/action dimensions.
- Correct the quaternion identity initialization.

### Acceptance Criteria

- Invalid configuration fails before serial command loops start.
- Every current LW configuration passes a standalone validation test.
- Model dimension mismatches produce a clear startup error.

### Resolution Evidence

- Resolved: 2026-08-09
- Added a shared LW configuration validator used by both `rl_real_LW` and
  `rl_sim_LW` before policy preload, serial initialization, or worker-loop
  startup. Missing keys, wrong types, invalid timing values, non-finite or
  incorrectly sized vectors, duplicate/out-of-range mappings, unsupported
  observation terms, invalid history settings, and inconsistent action limits
  now fail with the configuration path and field-specific reason.
- Policy observation dimensions are computed from the actual observation list
  and checked against `num_observations`; history selection determines the
  actual model input size. The four formal LW policies validate as
  `410 -> 10`, `195 -> 10`, `59 -> 10`, and `59 -> 10`.
- ONNX Runtime now exposes immutable input/output tensor metadata. LW preload
  requires one float32 rank-2 input and output, accepts batch size 1 or a
  dynamic batch, checks feature dimensions, and verifies the output length and
  finiteness during two warm-up inferences. Configuration/model failures now
  throw instead of being logged and ignored.
- Corrected the shared `w,x,y,z` observation quaternion identity from
  `{0,0,0,1}` to `{1,0,0,0}`.
- `test_lw_configuration_validation` covers all current YAML files and formal
  ONNX models, required fields, vector sizes, finite values, mapping uniqueness
  and bounds, observation names/dimensions, history settings, model dimension
  mismatches, and quaternion initialization.
- Verified with:
  - existing Debug build of `test_lw_configuration_validation`, `rl_real_LW`,
    and `rl_sim_LW`;
  - full Debug CTest suite: 14/14 passed;
  - clean Debug and clean Release builds of
    `test_lw_configuration_validation`, `rl_real_LW`, and `rl_sim_LW`;
  - clean Debug and Release `lw_configuration_validation` CTest: 1/1 passed in
    each build;
  - targeted `cppcheck` of the validator and its test: no warning reported;
  - `git diff --check`.
- No serial device, simulator UI, or real robot was started.

---

## [LW-011] Deterministic control-loop timing

**Priority**: P1 / high
**Status**: resolved
**Dependencies**: LW-001, LW-007

### Problem

LW-010 now guarantees a Release production bundle, but loop timing still
truncates callback duration to integer milliseconds and uses relative sleeps.
Callbacks run without a defined real-time scheduling policy, missed deadlines
are not measured or handled, and locomotion/transition status performs terminal
IO and flushes inside the 200 Hz control path.

### Evidence

- `src/rl_sar/CMakeLists.txt:34-40`
- `src/rl_sar/library/core/loop/loop.hpp:105-135`
- `src/rl_sar/library/core/logger/logger.hpp:47-92`
- `src/rl_sar/fsm_robot/fsm_LW.hpp:291-295`
- `src/rl_sar/fsm_robot/fsm_LW.hpp:371-375`
- `src/rl_sar/fsm_robot/fsm_LW.hpp:464-482`
- `src/rl_sar/fsm_robot/fsm_LW.hpp:570-588`

### Intended Scope

- Preserve the LW-010 Release production-build guarantee.
- Schedule against absolute deadlines with high-resolution durations.
- Define overrun detection and a safe overrun policy.
- Move terminal IO and progress formatting out of the control thread.
- Decide and document CPU affinity and real-time priority requirements.

### Acceptance Criteria

- Production builds use the approved optimized profile.
- A timing test reports control and inference jitter, maximum latency, and missed deadlines.
- Blocking terminal or ROS debug IO cannot delay motor command generation.

### Resolution Evidence

- Resolved: 2026-08-04T17:35:45+08:00
- Commit: this commit
- Approved Scope: absolute high-resolution scheduling without catch-up bursts;
  timing metrics and configurable CPU affinity/`SCHED_FIFO`; soft zero-velocity
  degradation after three consecutive skipped periods or one 20 ms deadline
  lateness; hard timing fail-safe disabled by default; periodic LW terminal
  output moved from the control callback to ROS timers in real and Sim2Sim.
- Changed Files: `policy/LW/base.yaml`, `docs/LW_BUILD_DEPLOYMENT_CN.md`,
  `src/rl_sar/CMakeLists.txt`, `src/rl_sar/fsm_robot/fsm_LW.hpp`, both LW
  headers and sources, `library/core/loop/loop.hpp`, `rl_sdk.hpp/.cpp`,
  `lw_runtime_sync.hpp`, `test_loop_timing.cpp`, and
  `test_lw_runtime_sync.cpp`.
- `LoopFunc` now schedules against `steady_clock` absolute deadlines, skips
  expired periods instead of replaying callbacks, exposes wakeup/deadline/
  execution statistics, validates scheduling settings before the first
  callback, and supports optional affinity and `SCHED_FIFO` with explicit
  required-versus-fallback behavior.
- LW real control latches `x/y/yaw` to zero at the approved degraded threshold
  while preserving FSM button events so the operator can request `GetDown`.
  Timing-triggered `EnterFailSafe()` remains inactive until a nonzero fatal
  threshold is deliberately configured.
- Periodic FSM status passes through a coherent nonblocking single-writer
  mailbox. Real and Sim2Sim ROS timers perform formatting and terminal IO away
  from the 200 Hz callback; real control also reports one-second timing
  summaries and scheduling fallback/degradation warnings.
- Verified with fresh Release configuration in
  `build/lw011_release_current`: both `rl_real_LW` and `rl_sim_LW` built and all
  12 registered tests passed. The new `loop_timing` test passed 10 consecutive
  runs and reports control/inference average and maximum wakeup lateness,
  maximum deadline lateness, maximum execution time, missed deadlines, and
  skipped periods.
- Standalone `-Wall -Wextra -Wpedantic` plus AddressSanitizer and
  UndefinedBehaviorSanitizer execution of `test_loop_timing` passed. Targeted
  `cppcheck` reported only the pre-existing `CSVInit(std::string)`
  pass-by-value performance suggestion.
- Hardware execution and deployment-host `SCHED_FIFO`/CPU-affinity tuning were
  intentionally not performed. Fatal timing thresholds remain zero until
  target-host measurements and board-watchdog behavior are reviewed.

---

## [LW-012] Motion-loader robustness and time convention

**Priority**: P2 / medium
**Status**: resolved
**Dependencies**: LW-009

### Problem

Motion duration is calculated as `num_frames * dt` even though the interval from the first to last frame is normally `(num_frames - 1) * dt`. A one-frame file dereferences an empty velocity vector, and inconsistent CSV row lengths are not rejected before indexed access.

### Evidence

- `src/rl_sar/library/core/motion_loader/motion_loader_lw.cpp:5-15`
- `src/rl_sar/library/core/motion_loader/motion_loader_lw.cpp:18-79`
- `src/rl_sar/library/core/motion_loader/motion_loader_lw.cpp:81-118`

### Intended Scope

- Define the exact frame-time convention.
- Reject empty, one-frame, nonfinite, or inconsistent-width motion data.
- Calculate velocities and duration according to the agreed convention.

### Acceptance Criteria

- Duration tests cover 0, 1, 2, and many frames.
- Malformed CSV input fails before any indexed access.
- First, intermediate, and final timestamp interpolation is verified.

### Resolution Evidence

- Resolved: 2026-08-09T15:53:18+08:00
- Commit: 本提交（基线 `6853b6c`）
- The loader now uses an explicit nonnegative `motion_time_offset_frames`
  contract. CSV row `i` is sampled at
  `(motion_time_offset_frames + i) / motion_fps`; both current transition
  configurations set the offset to `1` because their exporter removes the
  original `t=0` row. Future CSV files that retain that row can select offset
  `0` without another loader change.
- Times before the first retained sample hold the first row, times after the
  last sample hold the last row, and interpolation and forward-difference
  velocities use the configured motion rate. FSM completion now uses
  `motion_time >= motion_length`.
- Startup validation rejects invalid FPS/offset/joint-count parameters, empty
  or one-row files, blank or malformed fields, non-finite values, inconsistent
  row widths, unexpected joint counts, and invalid quaternions. Loaded
  quaternions are normalized and velocity storage is preallocated.
- `test_lw_motion_loader` covers offset `0` and `1`, duration, first/exact/
  intermediate/final timestamps, boundary holding, velocities, quaternion
  normalization, and malformed inputs. The reference-rate and configuration
  tests cover both formal 60 Hz CSV/configuration pairs and the new field.
- The targeted CTest selection passed 3/3 in the existing build. The complete
  existing suite passed 15/15.
- Fresh Debug and Release configurations in `build/lw012_debug_clean` and
  `build/lw012_release_clean` each built `test_lw_motion_loader`,
  `test_lw_motion_reference_rate`, `test_lw_configuration_validation`,
  `rl_real_LW`, and `rl_sim_LW`; the three selected tests passed 3/3 in each
  configuration.
- `git diff --check` passed. Targeted `cppcheck` reported no correctness
  warning and one non-blocking test-fixture initialization-list performance
  suggestion.
- No serial device, simulator UI, or real robot was started.

---

## [LW-015] LW-only repository scope and future robot extension boundary

**Priority**: P2 / medium
**Status**: resolved
**Dependencies**: LW-010

### Problem

The current repository still carries production policies for eleven non-LW
robots, eleven non-LW FSM headers, five non-LW real-robot entry points, and
multiple vendor SDKs/submodules. Several non-LW SDKs are configured by CMake
even though their executable targets are disabled. This increases dependency,
build, review, and maintenance surface for an LW-only product.
`src/rl_sar_zoo/` is also an approximately 588 MB nested Git repository with
LW plus eleven non-LW robot-description directories. Its LW MJCF/terrain
content has local changes required by the current Sim2Sim workflow but is not
tracked by the parent repository. Blind deletion or naive parent-repository
staging could lose those changes, record an unusable embedded repository, or
remove generic simulation/core facilities needed for future robot additions.

### Evidence

- `policy/{a1,b2,b2w,g1,go2,go2w,gr1t1,gr1t2,l4w4,lite3,tita}/`
- `src/rl_sar/fsm_robot/fsm_*.hpp`
- `src/rl_sar/src/rl_real_{a1,g1,go2,l4w4,lite3}.cpp`
- `src/rl_sar/library/thirdparty/robot_sdk/{unitree,deeprobotics,zhinao}/`
- `.gitmodules`
- `src/rl_sar/CMakeLists.txt:355-422`
- `README.md` and `README_CN.md`
- `scripts/download_robot_descriptions.sh`
- `src/rl_sar_zoo/`, including its nested Git metadata and LW local changes

### Intended Scope

- Produce an explicit tracked inventory that classifies robot-related paths as
  remove, retain as shared infrastructure, retain for LW, or external/user-owned.
- Remove tracked non-LW policies, FSM implementations, real-robot entry points,
  vendor SDKs/submodules, inactive CMake targets, and obsolete build/download
  and documentation references.
- Retain the LW policy/FSM/SDK, generic RL runtime, loop and inference
  infrastructure, FSM factory/registration mechanism, and generic simulation
  facilities needed for future robot integration.
- Reduce `fsm_all.hpp` to current LW registration while keeping a clear,
  documented extension point for registering another robot later.
- Include `src/rl_sar_zoo/` in the approved inventory. Preserve its current
  `LW_description` MJCF changes and new terrain assets, remove non-LW robot
  descriptions, and remove unrelated editor/download artifacts that are not
  part of the approved LW simulation input.
- Select and document one valid parent-repository integration strategy before
  staging the zoo: either vendor the cleaned LW description as ordinary tracked
  files with nested Git metadata removed, or configure a proper pinned
  submodule whose committed revision contains the approved LW changes. Do not
  create an unmanaged embedded-repository gitlink.
- Preserve upstream provenance and applicable license/version information for
  retained LW description assets.
- Keep current LW Sim2Sim and production-deployment workflows functional.
- Update `scripts/download_robot_descriptions.sh` and build behavior so a clean
  checkout obtains exactly the tracked/pinned LW description without silently
  downloading the former multi-robot zoo.
- Update the main documentation to describe LW as the current supported robot
  while retaining a concise guide for adding a future robot.
- Do not combine generic-core redesign or LW runtime behavior changes with this
  repository-scope cleanup.

### Acceptance Criteria

- An approved pre-deletion inventory identifies every tracked path and
  submodule to remove or retain, including every top-level zoo entry and every
  currently modified/untracked LW zoo asset.
- No active build/runtime reference to a removed robot or vendor SDK remains,
  except explicit historical/license text or future-extension documentation.
- A clean parent-repository checkout reproduces the approved
  `src/rl_sar_zoo/LW_description` without an unmanaged nested Git repository,
  manual download, or missing local MJCF/terrain changes.
- No non-LW robot-description directory remains in `src/rl_sar_zoo/`.
- A clean ROS 2 development build produces `rl_sim_LW`, and the LW Sim2Sim
  startup path remains available.
- A clean production build produces `rl_real_LW`; the deployment manifest and
  `--verify-deployment-only` acceptance pass.
- All existing LW unit/regression tests pass after the removal.
- The documentation identifies the bounded files/interfaces needed to add a
  future robot without restoring unrelated legacy implementations.
- The unrelated user-owned `.agents/skills/inspect-context-compactions/`
  directory remains unchanged and absent from the cleanup commit; all approved
  LW zoo files are intentionally tracked or pinned rather than silently omitted.

### Resolution Evidence

- **Resolved**: 2026-08-09
- The user approved the LW-only inventory and vendoring plan before any cleanup.
  Non-LW policies, FSMs, real adapters, SDKs/submodules, examples, inactive build
  wiring, and the legacy multi-robot zoo downloader were removed. Generic FSM,
  inference, loop, control, MuJoCo, joystick, message, and controller extension
  infrastructure remains documented in `README.md`, and the retained LW-only
  boundary is continuously enforced by `test_lw_repository_scope.py`.
- `src/rl_sar_zoo/LW_description` is now ordinary parent-repository content,
  based on zoo commit `349d14a700ecf248b3cdbec5e7bac30882b66e62` with upstream and license
  provenance recorded. Its 22-file SHA-256 manifest passes, no nested Git
  metadata or non-LW description remains, and the five pre-existing LW
  MJCF/terrain changes are byte-identical to the pre-cleanup backup. The terrain
  PNG SHA-256 is
  `6ffaba9cff2aa28640a72c578e9fbf1cea8a878b8cc26eafcccf985d78d75a2a`.
- A complete pre-cleanup zoo Git bundle and LW worktree archive were verified at
  `/tmp/lw015-zoo-backup-aTLmdg/`; their SHA-256 values are respectively
  `90dd2a285b47179378a30e68b1785d44edcd6e8d01fd63dbbf868ea0dbbb10d1`
  and `dd8d5a4ffbbdc9f7aa28ab5d6ffe7cc5328d319845b0e8d749498fda18058231`.
- The description validator passes in the working tree and in a detached clean
  checkout. Negative tests reject a changed manifest file, a non-LW description,
  and nested Git metadata. The standalone `lw_description` package configures,
  builds, and installs its MJCF, terrain, mesh, and URDF assets.
- The existing build and fresh Debug/Release builds each passed all 17 CTests.
  A separate detached clean-checkout Debug ROS 2 build produced
  `lw_description`, `rl_sim_LW`, and `rl_real_LW`, then passed 17/17 CTests.
- A detached clean-checkout Release production build created the deployment
  manifest and passed `rl_real_LW --verify-deployment-only`. It initialized only
  the pinned joystick submodule and used no robot-description download or non-LW
  SDK.
- `git diff --check`, Bash/Python syntax checks, and targeted `cppcheck` passed;
  `shellcheck` and `cmakelint` were unavailable. No serial device, simulator UI,
  or real robot was started.

---

## [LW-014] Debug and plot publishing isolation

**Priority**: P2 / low
**Status**: resolved
**Dependencies**: LW-007, LW-011

### Problem

Plot publishing is unconditionally enabled at compile time for production and
creates a 250 Hz ROS timer/publisher even when debug output is not wanted.
LW-007 already made its data source coherent and publication nonblocking, but
the message timestamp is still initialized only once instead of being refreshed
for each sample.

### Evidence

- `src/rl_sar/include/rl_real_LW.hpp:4`
- `src/rl_sar/src/rl_real_LW.cpp:129-178`
- `src/rl_sar/src/rl_real_LW.cpp:218-309`

### Intended Scope

- Make plotting opt-in through a build or runtime setting.
- Preserve the coherent snapshot and nonblocking real-time publisher introduced
  by LW-007.
- Refresh timestamps on every message.
- Avoid constructing the plot publisher/timer in production when plotting is
  disabled.

### Acceptance Criteria

- Production control can run with plotting completely disabled.
- Enabling plotting does not introduce control-path races or blocking.
- Published messages carry current timestamps and internally consistent samples.

### Resolution

- **Resolved**: 2026-08-11T12:01:35+08:00
- **Commit**: 本提交
- **Approved Scope**: 将 LW 实机调试发布改为默认关闭的显式 ROS/launch
  参数；保留 LW-007 的一致快照和非阻塞发布边界；逐条刷新消息时间戳；关闭时
  不创建 publisher、定时器或控制线程调试快照。
- **Changed Files**: `src/rl_sar/library/core/debug/lw_debug_publisher.*`、
  `src/rl_sar/include/rl_real_LW.hpp`、`src/rl_sar/src/rl_real_LW.cpp`、
  `src/rl_sar/launch/rl_real_LW.launch.py`、`src/rl_sar/CMakeLists.txt`、
  `src/rl_sar/test/test_lw_debug_publisher.cpp`、
  `docs/LW_BUILD_DEPLOYMENT_CN.md`。
- **Verification**: `lw_debug_publisher` 与 `lw_runtime_sync` 定向 CTest 在现有、
  全新 Debug 和全新 Release 三个构建中均通过；三套完整 CTest 均为 18/18；
  `rl_real_LW` 与 `rl_sim_LW` 在 Debug/Release clean build 中构建成功；
  `cppcheck`、Python/Bash 语法、launch 默认参数检查及 `git diff --check`
  通过；detached clean-worktree Release 部署构建、manifest 生成和
  `--verify-deployment-only` 通过。未启动串口、仿真界面或实机。
- **Remaining Follow-ups**: none

---

## [LW-017] Reproducible IMU and serial runtime deployment

**Priority**: P1 / high
**Status**: resolved
**Dependencies**: LW-010, LW-015

### Problem

The development helper builds the ROS `serial` library and `fdilink_ahrs`, but
the formal production builder selects only `rl_sar`. The installed LW launch
unconditionally resolves `fdilink_ahrs`, so a fresh or relocated production
prefix can pass `--verify-deployment-only` and still fail before the real node
starts because its IMU packages are absent. The package manifests also omit
dependencies that the AHRS CMake configuration requires, and the deployment
guide does not provide a complete device-alias and package-presence procedure.

### Evidence

- `build_LW.sh:7-8`
- `src/rl_sar/scripts/build_lw_deployment.sh:50-52`
- `src/rl_sar/launch/rl_real_LW.launch.py:14-17`
- `src/rl_sar/package.ros2.xml:11-19`
- `src/fdilink_ahrs_ROS2/package.xml:10-25`
- `src/fdilink_ahrs_ROS2/CMakeLists.txt:22-34`
- `src/fdilink_ahrs_ROS2/wheeltec_udev.sh:1-13`
- `src/rl_sar/src/rl_real_LW.cpp:171-172`

### Intended Scope

- Declare the project-owned IMU and serial dependency graph completely.
- Build and install `serial`, `fdilink_ahrs`, and `rl_sar` into the same clean
  production prefix.
- Bind the installed IMU/serial runtime files into deployment integrity checks.
- Provide a non-automatic IMU udev helper with the operator-approved `0777`
  device mode and document the
  distinct motor-board and IMU serial paths without inventing motor USB IDs.
- Keep ROS, operating-system libraries, USB drivers, and actual hardware
  activation outside the deployment manifest and automated acceptance.

### Acceptance Criteria

- A clean production prefix resolves `serial`, `fdilink_ahrs`, and `rl_sar`
  without sourcing a development workspace.
- `--verify-deployment-only` rejects missing, changed, escaping, or symlinked
  project-owned IMU/serial runtime files.
- Both production executables have no unresolved dynamic libraries.
- The Chinese deployment guide contains build, package, permission, stable
  device-name, offline-validation, and controlled `/imu` validation steps.
- Automated verification does not open serial devices or start ROS nodes.

### Resolution

- **Resolved**: 2026-08-11T12:34:05+08:00
- **Commit**: 本提交
- **Approved Scope**: 补全 LW 正式部署中的 `serial`、`fdilink_ahrs` 依赖
  图和同前缀安装；将项目内 IMU/串口运行文件加入清单哈希与离线校验；提供
  非自动执行、设备权限为用户指定 `0777` 的 IMU udev 辅助脚本；用中文记录
  电机板与 IMU 串口的构建、设备别名、权限和受控验证步骤。安装规则仍需
  root，安装后访问 IMU 不要求 root 或 `dialout`。
- **Changed Files**: `src/fdilink_ahrs_ROS2/{package.xml,CMakeLists.txt,wheeltec_udev.sh}`、
  `src/rl_sar/package.ros2.xml`、`src/rl_sar/scripts/{build_lw_deployment.sh,generate_lw_deployment_manifest.py}`、
  `src/rl_sar/library/core/deployment/lw_deployment_bundle.*`、
  `src/rl_sar/CMakeLists.txt`、`src/rl_sar/test/{test_generate_lw_deployment_manifest.py,test_lw_deployment_bundle.cpp,test_lw_runtime_dependencies.py}`、
  `docs/LW_BUILD_DEPLOYMENT_CN.md`、`.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **Verification**: 现有构建及从空目录创建的 Debug、Release 构建均通过新增
  三项定向 CTest 和完整 19/19 CTest；两个 clean build 均构建了 `serial`、
  `fdilink_ahrs`、`rl_real_LW` 和 `rl_sim_LW`。临时克隆中的 detached clean-
  worktree Release 正式构建安装了三个 ROS 包，生成 schema 2 清单并通过
  `--verify-deployment-only`；完整前缀重定位后，三个包均解析到新前缀，清单
  校验再次通过，两个运行程序的 `ldd` 均无 `not found`。Python/Bash 语法、
  XML 依赖测试、`cppcheck` 和 `git diff --check` 通过；`shellcheck`、
  `cmakelint` 不可用。未打开串口、未安装 udev 规则、未启动 ROS 节点、仿真
  界面或实机。用户随后明确要求将 IMU 设备模式恢复为 `0777` 且普通访问不
  依赖 `dialout`；四条规则、中文文档和批准范围已同步，Bash 语法、规则内容
  断言、三项部署定向 CTest 及 `git diff --check` 再次通过。
- **Remaining Follow-ups**: none

---

## [LW-018] Unified build entry point and Jetson detection

**Priority**: P2 / medium
**Status**: resolved
**Dependencies**: LW-015, LW-017

### Problem

`build_LW.sh` duplicates the normal build entry point with several separate
`--packages-select` invocations. A fresh workspace can therefore select a
package before its workspace dependency is installed or sourced. The helper
also installs system packages interactively and relies on a comment telling
Jetson users to export `IS_JETSON=true`. Jetson detection is inconsistent:
the inference downloader consumes the variable without resolving it, the
Jetson installer has broader local checks, and CMake checks only
`/etc/nv_tegra_release`.

### Evidence

- `build_LW.sh:3-9`
- `build.sh:96-135`
- `scripts/download_inference_runtime.sh:16-19,73-104`
- `scripts/install_pytorch_jetson.sh:30-42`
- `src/rl_sar/CMakeLists.txt:134-139`
- `docs/LW_BUILD_DEPLOYMENT_CN.md:119-122`

### Intended Scope

- Remove `build_LW.sh` and retain `build.sh` as the only development build
  entry point.
- Make selected-package builds include their complete workspace dependency
  closure and expose all ROS-version package manifests before selection.
- Resolve Jetson mode consistently through automatic Linux/aarch64 hardware
  indicators with a validated `IS_JETSON=true|false` override.
- Automatically install missing Debian/Ubuntu and ROS build dependencies on a
  first build, while avoiding package-manager work when everything is present.

### Acceptance Criteria

- A clean no-argument build succeeds for all workspace packages in dependency
  order.
- Clean selected builds of `fdilink_ahrs` and `rl_sar` include `serial` and the
  remaining declared workspace dependency closure.
- Native Jetson, non-Jetson aarch64, x86_64, valid overrides, and invalid or
  incompatible overrides are covered without requiring Jetson hardware.
- Shell, CMake, inference setup, and documentation report the same Jetson mode.
- A first build installs missing declared system/ROS dependencies and existing
  environments skip package-manager work.

### Resolution

- **Resolved**: 2026-08-11T13:31:54+08:00
- **Commit**: 本提交
- **Approved Scope**: 删除重复的 `build_LW.sh`，将 `build.sh` 作为唯一开发
  构建入口；使选包构建包含完整工作区依赖闭包；统一 Shell、CMake 与推理运行时
  的 Jetson 自动检测及显式覆盖；首次构建默认自动安装缺失的 Debian/Ubuntu、
  ROS 构建依赖并继续准备项目运行库，依赖齐全时不调用包管理器。
- **Changed Files**: `build.sh`（并删除 `build_LW.sh`）、
  `scripts/{common.sh,detect_jetson.sh,download_inference_runtime.sh,install_build_dependencies.sh,install_pytorch_jetson.sh}`、
  `src/rl_sar/CMakeLists.txt`、
  `src/rl_sar/test/{test_build_workflow.py,test_jetson_detection.py}`、
  `README_CN.md`、`docs/LW_BUILD_DEPLOYMENT_CN.md`、
  `.learnings/{LEARNINGS.md,LW_REAL_DEPLOYMENT_ISSUES.md}`。
- **Verification**: 隔离测试覆盖 x86_64、无 Jetson 标志的 aarch64、三类 Jetson
  标志、有效 `IS_JETSON` 覆盖、无效及不兼容覆盖；假 `dpkg-query`/`apt-get`/
  `sudo` 验证缺包时执行更新和安装，依赖齐全时完全跳过包管理器。最终源码的
  clean `./build.sh fdilink_ahrs` 按 `serial -> fdilink_ahrs` 构建 2 包，clean
  `./build.sh rl_sar` 按 `serial -> fdilink_ahrs -> rl_sar` 构建 3 包，无参数
  Debug 构建全部 6 包；独立 Debug、Release 的定向 CTest 和全量 21/21 CTest
  均通过。detached clean-worktree Release 正式部署构建安装三个 ROS 包、生成
  schema 2 清单并通过 `--verify-deployment-only`。Bash/Python 语法、
  `cppcheck` 和 `git diff --check` 通过；`shellcheck`、`cmakelint` 不可用。
  验收未在宿主机实际执行 apt 安装，也未使用 Jetson 或实机硬件。
- **Remaining Follow-ups**: none

---

## [LW-019] Real-robot terminal keyboard recovery channel

**Priority**: P1 / high
**Status**: resolved
**Dependencies**: LW-006, LW-007, LW-016

### Problem

The LW FSM accepts keyboard transitions and the shared SDK implements terminal
key decoding, but `rl_real_LW` never polls a keyboard. After a joystick fault
latches the Gamepad unavailable, the documented retained `GetDown` path is
therefore not reachable from a terminal. Starting the simulator's separate
keyboard loop in the real executable would also race with the 200 Hz control
thread, while a ROS 2 launch child cannot rely on its piped standard input as
an interactive terminal.

### Evidence

- `src/rl_sar/src/rl_real_LW.cpp:220-290,752-841`
- `src/rl_sar/src/rl_sim.cpp:152-180`
- `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp:915-1037`
- `src/rl_sar/fsm_robot/fsm_LW.hpp:119-121`
- `.learnings/LEARNINGS.md:59-87`

### Intended Scope

- Enable terminal keyboard input by default for `rl_real_LW`, with an explicit
  launch parameter for headless deployments.
- Read non-blockingly from the process controlling terminal and restore its
  termios state on every normal or exceptional exit path.
- Poll and apply keyboard input only in the real control thread; do not add a
  second writer to `Control`.
- Preserve the joystick fault latch and velocity-zero behavior while retaining
  keyboard FSM events, including the existing `9` to `GetDown` mapping.
- Document the interactive-terminal requirement and fail startup when keyboard
  input is requested but no controlling terminal is available.

### Acceptance Criteria

- `ros2 launch rl_sar rl_real_LW.launch.py` enables keyboard input by default
  when invoked from a controlling terminal.
- Terminal input is non-blocking, disables canonical input and echo without
  disabling signals, and restores the original termios state on destruction.
- The control thread consumes keyboard events before evaluating FSM
  transitions; no keyboard worker thread writes `Control` concurrently.
- A latched joystick fault does not clear a terminal `GetDown` event.
- `enable_keyboard:=false` supports an explicitly headless deployment, while
  enabled operation without a controlling terminal fails before control loops
  start.

### Resolution

- **Resolved**: 2026-08-11T15:31:22+08:00
- **Commit**: 本提交
- **Approved Scope**: 真机默认启用 `/dev/tty` 终端键盘，以非阻塞 RAII 方式
  配置和恢复 termios；只由 200 Hz 控制线程读取并更新 FSM 输入，不创建并发
  键盘线程；保留手柄断联后的键盘 `GetDown` 通道；为无交互终端提供显式
  `enable_keyboard:=false`，启用但无控制终端时在控制循环启动前失败。
- **Changed Files**: `src/rl_sar/library/core/safety/lw_terminal_keyboard.hpp`、
  `src/rl_sar/library/core/rl_sdk/rl_sdk.{hpp,cpp}`、
  `src/rl_sar/include/rl_real_LW.hpp`、`src/rl_sar/src/rl_real_LW.cpp`、
  `src/rl_sar/launch/rl_real_LW.launch.py`、`src/rl_sar/CMakeLists.txt`、
  `src/rl_sar/test/{test_lw_terminal_keyboard.cpp,test_lw_real_keyboard_integration.py}`、
  `README_CN.md`、`docs/LW_BUILD_DEPLOYMENT_CN.md`、
  `.learnings/{LEARNINGS.md,LW_REAL_DEPLOYMENT_ISSUES.md}`。
- **Verification**: 伪终端测试验证描述符非阻塞、关闭 `ICANON/ECHO`、保留
  `ISIG`、析构恢复原始 termios、数字键 `9` 与方向键解析以及无终端拒绝；
  集成测试验证键盘在 FSM 前由控制线程轮询、未新增 `loop_keyboard`、手柄故障
  门不清除键盘、launch 默认启用和文档契约。现有工作区及 clean Debug、
  Release 的定向测试与全量 23/23 CTest 均通过，两个 clean build 均构建全部
  6 包；`ros2 launch ... --show-args` 确认 `enable_keyboard` 默认 `true`。
  detached clean-worktree Release 正式部署构建和 `--verify-deployment-only`
  通过。Python/C++ 严格语法、`cppcheck`（仅定点抑制既有 `CSVInit` 按值传参
  提示）和 `git diff --check` 通过；`clang-tidy`、`cmakelint` 不可用。未打开
  真机串口、未启动 ROS 节点或电机控制，也未在真实控制终端上进行实机试键。
- **Remaining Follow-ups**: none

---

## [LW-020] Architecture-safe ONNX-only Jetson production inference

**Priority**: P1 / high
**Status**: resolved
**Dependencies**: LW-010, LW-018

### Problem

The Jetson build prepares LibTorch before ONNX Runtime even though every LW
production policy is ONNX. The PyTorch bootstrap maps every L4T R36 release to
one CUDA 12.6 wheel and omits documented PyTorch prerequisites, so an unrelated
Torch failure can stop a fresh real-robot build. Existing inference runtimes are
also accepted by directory shape alone; copying an x86_64 development tree to
Jetson therefore reuses incompatible ELF libraries. Finally, production
`rl_real_LW` links Torch whenever a local LibTorch directory happens to exist.

### Evidence

- `build.sh:18-31`
- `scripts/download_inference_runtime.sh:49-85,173-184,281-349`
- `scripts/install_pytorch_jetson.sh:30-99,102-181`
- `src/rl_sar/CMakeLists.txt:121-277,436-449`
- `policy/LW/robot_lab/*/config.yaml:2`

### Intended Scope

- Make the native Jetson build and every production deployment ONNX-only while
  preserving optional LibTorch actuator-network support on non-Jetson Sim2Sim.
- Validate the ELF machine type of existing and newly downloaded Linux
  inference libraries before CMake or deployment can consume them.
- Reject explicit Jetson LibTorch requests instead of guessing a PyTorch wheel
  from an L4T major version.
- Require a valid ONNX Runtime for Jetson and production configurations, and
  verify that the installed real executable has no Torch runtime dependency.
- Document the supported Jetson path and the prohibition on reusing x86_64
  inference artifacts.

### Acceptance Criteria

- A Jetson `./build.sh` prepares only the Linux aarch64 ONNX Runtime and never
  invokes a PyTorch installer.
- Linux x86_64 and AArch64 runtime libraries are accepted only on their matching
  architecture; missing, corrupt, and mismatched ELF files fail before linking.
- Jetson and `LW_PRODUCTION_DEPLOYMENT=ON` builds have `USE_TORCH=OFF`; non-Jetson
  development builds retain optional Torch support.
- The production `rl_real_LW` depends on ONNX Runtime and has no `libtorch`,
  `libtorch_cpu`, or `libc10` dynamic dependency.
- Documentation, targeted tests, full tests, static checks, and clean Debug and
  Release builds agree with the ONNX-only production contract.

### Resolution

- **Resolved**: 2026-08-11T17:09:09+08:00
- **Commit**: 本提交
- **Approved Scope**: Jetson 真机与正式部署仅准备并链接 ONNX Runtime，拒绝
  Jetson LibTorch 请求和错误 ELF 架构；非 Jetson 开发构建继续在新环境中
  自动准备 LibTorch 与 ONNX Runtime，保留 `rl_sim_LW --use_actuator_net`；
  正式部署验证 `rl_real_LW` 不依赖 Torch。Python 训练环境的 `torch` 不属于
  本项编译依赖范围。
- **Changed Files**: `build.sh`、
  `scripts/{download_inference_runtime.sh,validate_inference_runtime.sh}`（并删除
  `scripts/install_pytorch_jetson.sh`）、`src/rl_sar/CMakeLists.txt`、
  `src/rl_sar/scripts/build_lw_deployment.sh`、
  `src/rl_sar/test/{test_build_workflow.py,test_inference_runtime_architecture.py}`、
  `README_CN.md`、`docs/LW_BUILD_DEPLOYMENT_CN.md`、
  `.learnings/{LEARNINGS.md,LW_REAL_DEPLOYMENT_ISSUES.md}`。
- **Verification**: 架构校验单元测试 9/9、构建流程测试 12/12、Jetson 检测
  测试 8/8 通过；模拟 Jetson 显式请求 LibTorch 会在下载前失败，x86_64
  `all` 路径继续验证并准备 LibTorch 与 ONNX Runtime。现有构建及 clean
  Debug、Release 构建均通过全量 24/24 CTest；最终源码在三套构建目录中的
  LW-020 定向 CTest 均为 2/2。临时 clean clone 的 Release 正式部署构建和
  `--verify-deployment-only` 通过，最终 `rl_real_LW` 依赖 ONNX Runtime，
  `readelf`/`ldd` 未发现 `libtorch`、`libtorch_cpu` 或 `libc10`。Bash/Python
  语法与 `git diff --check` 通过；未修改 C++ 源文件，`shellcheck`、
  `cmakelint` 不可用。验收未在 Jetson、串口或真机上运行 ROS 节点或电机控制。
- **Remaining Follow-ups**: none

---

## [LW-021] Sim2Sim and real-runtime behavioral parity

**Priority**: P1 / high
**Status**: resolved
**Dependencies**: LW-007, LW-008, LW-011, LW-016, LW-020

### Problem

`rl_sim_LW` and `rl_real_LW` use the same LW FSM, policy definitions,
observation/output helpers, policy-output transport, nominal control period,
inference period, joint order, and joystick mapping. They nevertheless
implement their runtime pipelines in two separate entry-point sources. The
simulator proceeds directly from state acquisition through `StateController()`
to command application, while the real executable additionally applies
hardware-independent policy-action/output validation, stale-output fallback,
final-command validation, loop-exception handling, and control-timing safety
actions.

The default `RL::HandleLWPolicyOutputFault()` is a no-op, so a stale or
incomplete policy output can retain the previous command in Sim2Sim while the
real executable requests Passive damping. The simulator also reads, resets,
and writes `mjData` from control and debug callbacks without taking the
`sim->mtx` used by the physics thread around `mj_step()`. Its observations and
commands therefore are not guaranteed to form a coherent physics-step
snapshot.

The simulator additionally loads mutable source-tree policies, optionally uses
a Torch actuator network, always enables its plot path, and has no real-runtime
timing or safety-event callback. Existing tests validate the shared components
individually, but no deterministic end-to-end test feeds identical state/input
sequences through both runtime paths and compares FSM, observation, policy,
command, and hardware-independent safety results. A passing Sim2Sim run can
therefore serve as nominal behavior evidence but cannot currently establish
behavioral equivalence for real deployment.

### Evidence

- `src/rl_sar/src/rl_sim_LW.cpp:46-195,395-852,853-948`
- `src/rl_sar/src/rl_real_LW.cpp:100-298,456-898,899-1203`
- `src/rl_sar/include/rl_sim_LW.hpp:4-141`
- `src/rl_sar/include/rl_real_LW.hpp:8-172`
- `src/rl_sar/library/core/rl_sdk/rl_sdk.hpp:353-357`
- `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp:1207-1260`
- `src/rl_sar/library/thirdparty/mujoco_simulate/mujoco_utils.hpp:332-428`
- `src/rl_sar/CMakeLists.txt:774-845`
- Existing CTest registry: 24 component tests and no Sim2Sim/real parity test.

### Intended Scope

- Extract one platform-neutral LW runtime core used by both executables for
  input application, FSM execution, policy-input publication, inference,
  policy-action/output validation, stale-output handling, final-command
  validation, and hardware-independent safety decisions.
- Keep only sensor acquisition, actuator delivery, MuJoCo physics/UI, serial
  communication, ROS wiring, and platform lifecycle in thin real/simulation
  adapters. Do not weaken or reinterpret the approved LW-016 safety matrix.
- Give the simulator a safety adapter that both records the same decision and
  executes its MuJoCo equivalent without opening serial ports or shutting down
  unrelated host processes. S1 inhibits operator velocity while retaining the
  approved FSM recovery inputs; S2 applies the exact shared Passive command
  (`q=current`, `dq=0`, `tau=0`, `Kp=0`, `Kd=5`) through the simulator actuator
  path; S3/S4 latch command acceptance off and set every active MuJoCo actuator
  output to zero. S4 additionally records the requested terminal shutdown state
  while allowing the test harness to inspect the final trace.
- Make MuJoCo state acquisition and command application coherent with the
  physics thread by locking `sim->mtx` or publishing an immutable per-step
  snapshot. Remove unsynchronized reset, read, debug, and command access.
- Add a deterministic headless replay harness that supplies identical
  `RobotState`, operator input, timestamps, policy frames, and injected faults
  to the shared core and compares the complete observable trace.
- Run parity tests against the exact policy/configuration assets identified by
  the deployment bundle rather than silently accepting mutable source-tree
  divergence.
- Preserve simulation-only reset/pause controls and optional actuator-dynamics
  modeling outside the shared controller core; treat them as explicit adapter
  behavior, not parity-covered real-runtime behavior.
- Do not change policy models, YAML values, FSM transition semantics, serial
  protocol, motor gains, safety thresholds, CUDA/TensorRT support, or hardware
  activation as part of this issue.

### Acceptance Criteria

- `rl_sim_LW` and `rl_real_LW` invoke the same implementation for every
  platform-neutral control and inference stage; source-level duplication is
  limited to documented adapter responsibilities.
- For each Passive, get-up, get-down, locomotion, and morphology-transition
  scenario, identical replay inputs produce identical FSM states, policy
  generations, observation vectors, policy frames, and `RobotCommand` values
  within explicitly documented floating-point tolerances.
- NaN/Inf policy actions and outputs, incomplete or stale policy frames,
  regressed sequences, inference exceptions, control exceptions, and timing
  degradation produce the same hardware-independent safety decision and latch
  state in simulation and real-runtime tests.
- Sim2Sim visibly and numerically executes the approved action for each safety
  tier: S1 zeroes commanded velocity without clearing recovery buttons, S2
  enters and remains in Passive damping with `Kp=0` and `Kd=5`, and S3/S4
  produce zero active actuator effort after the terminal latch. Tests must not
  treat Passive damping and zero actuator output as equivalent actions.
- Hardware-specific events such as missing IMU, serial delivery failure, and
  motor-board faults are injected through adapter contracts; tests verify the
  real terminal action and the simulator's matching executed and recorded
  action without accessing hardware.
- No simulator callback accesses mutable `mjModel`/`mjData` concurrently with
  the physics thread. A stress test covers state snapshots, command writes,
  resets, and optional debug publication.
- The parity harness fails when policy/configuration hashes differ from the
  deployment bundle and passes with the exact committed bundle.
- Existing LW tests continue to pass in current, clean Debug, and clean Release
  builds; both executables build successfully; the formal Release deployment
  and `--verify-deployment-only` pass.
- Automated acceptance remains headless and does not open serial devices,
  start the real ROS node, send motor commands, or claim physical validation.

### Resolution

- **Resolved**: 2026-08-11T18:43:25+08:00
- **Commit**: 本提交
- **Approved Scope**: 将 Sim2Sim 与真机入口的平台无关控制、推理、输出校验、
  时序和安全决策收敛到同一运行时核心；仿真执行与真机同义的 S1 速度抑制、
  S2 Passive 阻尼以及 S3/S4 零执行器输出，并保留恢复按键；同步 MuJoCo
  读写；加入真实 ONNX 推理、故障注入、部署策略哈希和并发压力回放。传感器、
  执行器、串口、ROS、MuJoCo 生命周期及仿真专用复位/暂停/执行器网络继续由
  各自适配层负责；未改变策略、FSM 语义、串口协议、增益或安全阈值。
- **Changed Files**: `src/rl_sar/library/core/safety/{lw_runtime_core.hpp,lw_loop_config.hpp}`、
  `src/rl_sar/include/{rl_real_LW.hpp,rl_sim_LW.hpp}`、
  `src/rl_sar/src/{rl_real_LW.cpp,rl_sim_LW.cpp}`、`src/rl_sar/CMakeLists.txt`、
  `src/rl_sar/scripts/{build_lw_deployment.sh,verify_lw_policy_parity.py}`、
  `src/rl_sar/test/{test_lw_runtime_parity.cpp,test_lw_mujoco_synchronization.cpp,test_verify_lw_policy_parity.py,test_lw_real_keyboard_integration.py}`、
  `.learnings/{LEARNINGS.md,LW_REAL_DEPLOYMENT_ISSUES.md}`。
- **Verification**: 共享核心的确定性双适配器测试覆盖 FSM/命令一致性、真实
  `leg_loco` ONNX 推理的完整 observation/action/output 回放，以及 S1-S4、
  NaN/Inf、过期/缺帧/代际错误、控制与推理异常和时序故障；MuJoCo 无窗口
  四线程压力测试覆盖物理步进、状态快照、命令/复位和调试快照。当前工作树、
  clean Debug 和 clean Release 均成功构建两个入口并通过全量 27/27 CTest。
  隔离临时 clone 的 ONNX-only Release 正式部署构建、策略源文件/部署清单
  SHA-256 一致性检查及 `--verify-deployment-only` 全部通过，部署二进制无
  LibTorch 依赖。Python 编译、Bash 语法和 `git diff --check` 通过。所有验收
  均为 headless；未打开串口、未启动真机 ROS 节点、未发送电机命令，也未在
  Jetson 或真实机器人上执行物理验证。
- **Remaining Follow-ups**: none

---

## [LW-022] Suspended real-runtime profiling and configuration candidates

**Priority**: P1 / high
**Status**: resolved
**Dependencies**: LW-011, LW-016, LW-020, LW-021

### Problem

The current LW defaults for sensor freshness, paired serial-write deadline,
control-loop CPU affinity, real-time scheduling, and timing degradation were
chosen as portable conservative defaults rather than measured on the target
Jetson. Existing diagnostics expose only aggregate control-loop maxima and do
not retain sensor inter-arrival or serial enqueue latency distributions. A
Passive suspended run also leaves every locomotion policy inactive, so its CPU
load cannot represent production inference.

Blindly deriving production safety thresholds from an unloaded run is unsafe.
In particular, `control_loop_require_realtime` is an operational policy and
the fatal timing thresholds depend on verified board watchdog and physical
hard-disable behavior; neither can be selected from latency samples alone.

### Intended Scope

- Add a host-only profiler that runs the exact four bundled LW ONNX policies,
  observation/output path, motion references, 200 Hz control scheduling, and
  50 Hz inference without opening ROS, joystick, IMU, serial, or actuator
  devices.
- Add a separate suspended hardware-observation mode that refuses to start
  without an explicit confirmation flag, never enters an LW locomotion FSM
  state, and transmits only `motors_disable=true` packets while measuring IMU,
  right/left valid-feedback gaps and paired serial enqueue latency.
- Record bounded distribution statistics, control-loop startup/timing state,
  tested CPU/priority, policy identity, target environment and failure counts
  in machine-readable reports.
- Add an orchestrator/analyzer that can rank allowed CPUs and explicitly
  requested real-time priorities, then generate a review-only JSON report with
  a YAML-compatible candidate overlay. It must never modify
  `policy/LW/base.yaml` or a deployment bundle.
- Require an operator-supplied maximum safe sensor age and maximum safe control
  gap before suggesting the corresponding thresholds. Keep fatal timing
  thresholds disabled and mark `control_loop_require_realtime` as a manual
  deployment decision.
- Document that suspended shadow profiling produces provisional software and
  communication candidates, not final dynamic or physical safety validation.

### Acceptance Criteria

- Default profiling is host-only and performs no hardware I/O; hardware mode
  requires an exact, auditable confirmation, confirms an initial disable burst
  before policy preload, and continuously sends only motor disable packets.
- All four deployment policies execute actual ONNX inference without changing
  the FSM out of Passive or allowing policy output to reach actuators.
- Reports contain sample counts and percentile/max statistics for inference,
  sensor gaps and serial writes as applicable, plus loop misses, affinity and
  scheduling application results.
- Candidate generation rejects insufficient/failed measurements, never
  enables fatal timing, never silently requires SCHED_FIFO, and writes only to
  a new user-selected output path.
- Unit tests cover statistics, safety gating, candidate bounds, immutable input
  configuration and malformed reports. Existing tests and both LW executables
  continue to build and pass without hardware access.

### Resolution

- **Resolved**: 2026-08-11T19:54:33+08:00
- **Commit**: 本提交
- **Approved Scope**: 增加两阶段 LW 部署参数测量。host-only 阶段在目标主机
  运行四个正式 ONNX、共享控制/观测/输出路径及 200 Hz/50 Hz 线程而不访问
  硬件；吊装 hardware-observe 阶段保持 Passive、丢弃策略输出并只发送电机
  失能包，采集 IMU、左右反馈及成对串口写时序。分析器只生成需人工评审的
  候选报告，不修改 `base.yaml`，不自动启用实时强制或致命时序阈值。
  根据底层控制器上电即使能的现场约束，硬件模式在任何模型预加载前确认左右板
  初始失能包完整写入，并以独立 5 ms 保活持续只发送失能包；失败时不得开始或
  继续测算。反馈协议没有失能回执位，因此不把完整串口写入误称为物理执行确认。
- **Changed Files**: `src/rl_sar/src/lw_config_profiler.cpp`、
  `src/rl_sar/library/core/safety/lw_config_profile.hpp`、
  `src/rl_sar/scripts/{profile_lw_runtime_config.py,build_lw_deployment.sh,generate_lw_deployment_manifest.py}`、
  `src/rl_sar/library/core/deployment/lw_deployment_bundle.cpp`、
  `src/rl_sar/test/{test_lw_config_profile.cpp,test_lw_config_profiler_integration.py,test_profile_lw_runtime_config.py,test_lw_deployment_bundle.cpp}`、
  `src/rl_sar/CMakeLists.txt`、`docs/LW_BUILD_DEPLOYMENT_CN.md`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **Verification**: 当前工作树 Debug 全目标构建成功，`rl_sim_LW`、
  `rl_real_LW` 和 `lw_config_profiler` 均完成链接；完整 31/31 CTest 通过。
  集成测试实际加载四个正式 ONNX 并分别执行推理，确认 host-only 报告没有串口
  写或硬件命令，错误确认字符串在任何设备初始化前被拒绝；使用正确确认字符串
  但不可用串口时，采集器在任何 ONNX 预加载前失败。统计和分析器测试
  覆盖滚动上限、分位数、首帧/结束帧龄、配置输入不变、安全上限、采样不足、
  畸形字段、策略缺失及输出不可覆盖。隔离临时 Git 验证快照成功生成
  ONNX-only Release 正式部署，策略哈希、部署清单、动态库和
  `--verify-deployment-only` 均通过；部署包内采集器再次完成四策略 host-only
  推理，且采集器/分析器均纳入清单。Python 编译、Bash 语法和
  `git diff --check` 通过。未访问串口、未启动真机 ROS 节点、未发送电机命令；
  目标 Jetson 的 CPU/实时数据及吊装硬件数据必须按中文文档现场采集，不能由
  当前无硬件验收替代。
- **Remaining Follow-ups**: none

---

## [LW-023] Disable-before-startup safety boundary

**Priority**: P0 / critical
**Status**: resolved
**Dependencies**: LW-010, LW-017, LW-020

### Problem

The deployed STM32 enables motors at power-on, but the normal real-robot node
does not open the two motor-board serial ports or send its first disable packet
until after ROS/input setup, YAML validation, and all four policies and policy
contexts have been preloaded. A missing model, invalid configuration, terminal
failure, allocation failure, or other exception before serial initialization
can therefore leave the powered hardware enabled without any upper-layer
disable attempt. The LW-022 hardware profiler already establishes a stricter
disable-before-preload boundary, but the normal deployment entry point does not.

### Evidence

- `src/rl_sar/src/rl_real_LW.cpp:44-127`
- `src/rl_sar/src/rl_real_LW.cpp:143-169`
- `src/rl_sar/src/lw_config_profiler.cpp:401-425`

### Intended Scope

- Preserve offline deployment-integrity verification before hardware access.
- After verification, establish and confirm the real-node motor-disable output
  before policy preload or other lengthy/fallible runtime initialization.
- Keep disable output safe across every constructor failure after either serial
  port has been opened.
- Do not weaken the existing startup, command-gate, or final-disable checks.

### Acceptance Criteria

- With both serial ports available, no model preload starts before complete
  disable packets have been written to both boards.
- Every failure after serial initialization closes command delivery and attempts
  a bounded final disable sequence before releasing the ports.
- A partial serial initialization cannot start any worker loop or model preload.
- Tests prove the operation order and constructor-failure cleanup without
  accessing physical hardware.
- The deployment guide accurately states the earliest unavoidable enabled
  interval and the remaining need for physical isolation and emergency stop.

### Resolution

- **Resolved**: 2026-08-12T12:50:24+08:00
- **Commit**: 待本次提交
- **Approved Scope**: 在正式部署完整性校验通过后、ROS/终端/YAML/FSM/模型及
  `RL_Real` 堆分配之前，以栈上 RAII 守卫打开左右电机板串口并确认 20 个双侧
  完整失能包，随后以独立 5 ms 线程持续只发送失能；全部运行资源准备完成后
  join 保活、应用已验证的运行时写超时并补发交接失能，才允许启动工作循环。
  任一部分串口初始化或后续构造失败均关闭命令门、停止保活并尝试最终 20 包
  失能。保留部署离线校验、统一串口命令门和正常退出顺序，不把主机完整写入
  称为 STM32 物理执行确认。
- **Changed Files**: `src/rl_sar/library/core/safety/lw_startup_disable.hpp`、
  `src/rl_sar/include/rl_real_LW.hpp`、`src/rl_sar/src/rl_real_LW.cpp`、
  `src/rl_sar/test/{test_lw_startup_disable.cpp,test_lw_real_startup_disable_integration.py}`、
  `src/rl_sar/CMakeLists.txt`、`docs/LW_BUILD_DEPLOYMENT_CN.md`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **Verification**: 当前 Debug 全目标构建成功，完整 33/33 CTest 通过；启动失能
  PTY 生命周期测试和正式入口顺序集成测试连续 20 轮通过。全新 Release 构建
  成功生成 `rl_real_LW`，启动失能、正式入口顺序、串口 SDK、循环生命周期和
  安全策略测试 5/5 通过。新增 C++ 测试通过独立
  `-Wall -Wextra -Wpedantic -Werror` 构建、AddressSanitizer/
  UndefinedBehaviorSanitizer 和定向 `cppcheck`；Python 编译及
  `git diff --check` 通过。隔离 detached 临时验证提交成功生成 ONNX-only
  Release 正式部署，策略哈希、部署清单、动态依赖和
  `rl_real_LW --verify-deployment-only` 均通过，临时工作树随后已清理。
  全部自动化仅使用 PTY 或离线路径；未访问真实串口、未启动真机 ROS 节点、
  未发送真实电机命令，也未把主机 `write()` 当作硬件失能回执。部署文档明确
  保留上电至首批失能写入之间的不可避免窗口，以及可靠吊装、机械隔离和物理
  急停要求。
- **Remaining Follow-ups**: none

---

## [LW-024] Joinable and bounded Sim2Sim physics lifecycle

**Priority**: P1 / high
**Status**: resolved
**Dependencies**: LW-001, LW-021

### Problem

`rl_sim_LW` detaches the MuJoCo physics thread while passing it a raw pointer to
the owned `Simulate` object. The destructor does not wait for that thread, so
normal exit or a constructor exception can destroy `sim` while physics cleanup
still accesses it. Startup also waits forever for global MuJoCo data when model
loading fails, because the physics thread has no result channel and the polling
loop has no failure or timeout condition.

### Evidence

- `src/rl_sar/src/rl_sim_LW.cpp:74-112`
- `src/rl_sar/src/rl_sim_LW.cpp:250-269`
- `src/rl_sar/library/thirdparty/mujoco_simulate/mujoco_utils.hpp:436-459`

### Intended Scope

- Keep the physics thread joinable and make ownership explicit.
- Propagate model/data initialization success or failure to the constructor.
- Bound startup waiting and ensure partial construction cannot leave a worker
  accessing destroyed state.
- Preserve the shared LW runtime behavior established by LW-021.

### Acceptance Criteria

- Every normal, signal-requested, safety-requested, and exceptional exit stops
  and joins the physics thread before destroying `Simulate`, `mjData`, or
  `mjModel` state.
- An invalid or unloadable scene fails startup with a diagnostic instead of
  waiting indefinitely.
- Repeated start/exit and injected initialization failures pass under a suitable
  sanitizer or deterministic lifecycle stress test.
- No detached thread retains a pointer to an `RL_Real` or `Simulate` member.

### Resolution

- **Resolved**: 2026-08-12T13:33:29+08:00
- **Commit**: 待本次提交
- **Approved Scope**: 以 RAII 生命周期对象同步加载并唯一持有初始
  `mjModel`/`mjData`，再通过有界启动握手启动可 join 的物理 worker；删除
  `rl_sim_LW` 的 detached thread 和全局 `d` 无限轮询。为尚未启动或正在退出
  的渲染循环增加可取消模型交接，所有正常、窗口、安全请求和异常路径均先停止
  业务循环、唤醒并 join 物理 worker，之后才释放 MuJoCo 资源和 `Simulate`。
  线程异常跨边界传播到主线程；无效场景保留文件名和 MuJoCo 诊断。保持 LW-021
  的共享运行时及 S1–S4 安全动作语义，不处理 LW-028 的信号处理器重构。
- **Changed Files**: `src/rl_sar/library/core/simulation/lw_joinable_worker.hpp`、
  `src/rl_sar/library/thirdparty/mujoco_simulate/{mujoco_utils.hpp,simulate.h,simulate.cc}`、
  `src/rl_sar/include/rl_sim_LW.hpp`、`src/rl_sar/src/rl_sim_LW.cpp`、
  `src/rl_sar/test/{test_lw_joinable_worker.cpp,test_lw_mujoco_lifecycle.cpp,test_lw_sim_lifecycle_integration.py}`、
  `src/rl_sar/test/data/lw024_minimal.xml`、`src/rl_sar/CMakeLists.txt`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **Verification**: 当前 Debug 全目标构建成功，`rl_sim_LW` 完成链接，完整
  36/36 CTest 通过。全新 Release 构建成功，joinable worker、生命周期入口、
  MuJoCo 同步和真实无窗口 MuJoCo 生命周期测试 4/4 通过。真实 MuJoCo 测试
  覆盖无效场景诊断、渲染接管前取消、join、资源清空及幂等停止，连续 50 轮
  确定性压力通过；通用 worker 覆盖正常停止、启动超时、注入启动/运行异常和
  200 次内部重复生命周期，并在 AddressSanitizer/UndefinedBehaviorSanitizer
  下连续 20 轮通过。ASan/UBSan 版本 `rl_sim_LW` 构建成功；严格警告构建在仅
  降级 MuJoCo 既有 reorder、缺失初始化、未使用参数/函数和 sign-compare
  类别后通过。定向 `cppcheck`、Python 集成测试和 `git diff --check` 通过。
  未启动图形窗口、未访问真机硬件，也未改变 LW-028 信号处理器。
- **Remaining Follow-ups**: none

---

## [LW-025] Persistent and composable keyboard velocity input

**Priority**: P1 / high
**Status**: resolved
**Dependencies**: LW-006, LW-019, LW-021

### Problem

Each control cycle copies joystick-mailbox `x/y/yaw` into the shared control
state before keyboard processing. `W/S/A/D/Q/E` then changes that state for only
the current 5 ms cycle; the next cycle replaces it with the joystick value.
This contradicts the documented persistent keyboard velocity behavior and a
50 Hz inference loop can miss the short pulse entirely. `Space` consequently
does not provide the documented persistent stop semantics for keyboard input.

### Evidence

- `src/rl_sar/src/rl_real_LW.cpp:875-885`
- `src/rl_sar/src/rl_sim_LW.cpp:957-967`
- `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp:96-125`
- `docs/LW_BUILD_DEPLOYMENT_CN.md:474-485`

### Intended Scope

- Give keyboard velocity commands durable state with explicit merge/precedence
  rules relative to joystick velocity.
- Keep FSM button events edge-triggered and preserve the joystick-fault and
  timing-degraded zero-velocity latches.
- Apply identical hardware-independent behavior in real and Sim2Sim adapters.

### Acceptance Criteria

- A keyboard velocity step remains visible to subsequent control and inference
  cycles until changed or cleared according to the documented rules.
- `Space`, joystick disconnect, and timing degradation reliably force all three
  velocity axes to zero.
- Switching between keyboard and joystick input follows one documented,
  deterministic precedence rule without stale values reappearing.
- Automated parity tests cover persistence, zeroing, source switching, and FSM
  recovery buttons in real-like and Sim2Sim-like harnesses.

### Resolution

- **Resolved**: 2026-08-12T15:39:32+08:00
- **Commit**: 本提交
- **Approved Scope**: 用户明确决定以取消 LW 键盘速度功能替代原持久化与
  输入仲裁方案。真实机和 Sim2Sim 的共享 LW 运行核心均禁止
  `W/S/A/D/Q/E/Space` 修改 `x/y/yaw`，摇杆成为唯一人工速度来源；键盘
  FSM/模式事件（包括数字键 `9` 的 `GetDown`）继续进入状态机，手柄断联和
  控制时序降级的锁存零速语义保持不变。通用非 LW `StateController` 默认行为
  未改变。
- **Changed Files**: `src/rl_sar/library/core/rl_sdk/rl_sdk.{hpp,cpp}`、
  `src/rl_sar/library/core/safety/lw_runtime_core.hpp`、
  `src/rl_sar/test/{test_lw_runtime_parity.cpp,test_lw_real_keyboard_integration.py}`、
  `docs/LW_BUILD_DEPLOYMENT_CN.md`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **Verification**: 当前 Debug 工作树完整构建真实机和 Sim2Sim 入口并通过
  36/36 CTest；定向测试覆盖七个原速度键不改变摇杆三轴、键盘事件仍到达 FSM、
  S1 保留恢复键且锁住零速，以及非 LW 默认行为未被改变。Release 构建两个入口
  并通过终端键盘、真机键盘集成和运行时一致性测试；严格警告配置构建两个入口
  和一致性测试（仅输出该配置已知且已降级的 MuJoCo/旧适配层警告）；ASan/UBSan
  运行时一致性测试通过。Python 严格语法和 `git diff --check` 通过。未打开真机
  串口、未启动 ROS 节点、MuJoCo 图形窗口或电机控制，也未进行实机试键。
- **Remaining Follow-ups**: none

---

## [LW-026] Configuration-profile provenance and comparability

**Priority**: P1 / high
**Status**: resolved
**Dependencies**: LW-010, LW-022

### Problem

The profiler records `source_commit`, `policy_root`, host identity, and duration,
but the analyzer does not validate or compare those fields. Host and hardware
reports from different commits or policy trees can therefore be combined with
an unrelated `base.yaml`. The exact-policy check uses a set, so duplicate policy
records are accepted, and accumulated miss counts are ranked without requiring
comparable measurement durations. A mixed or stale report set can consequently
produce a plausible review file for the wrong deployment.

### Evidence

- `src/rl_sar/src/lw_config_profiler.cpp:930-983`
- `src/rl_sar/scripts/profile_lw_runtime_config.py:166-187`
- `src/rl_sar/scripts/profile_lw_runtime_config.py:294-320`
- `src/rl_sar/scripts/profile_lw_runtime_config.py:234-287`

### Intended Scope

- Bind every analyzed report and the selected base configuration to one exact,
  reviewable deployment identity.
- Require exactly one record for each approved policy and reject duplicates.
- Require comparable durations or normalize every duration-dependent ranking
  metric explicitly.
- Preserve review-only output and the existing physical-safety limits.

### Acceptance Criteria

- The analyzer rejects mixed source commits, policy roots/assets, incompatible
  base configurations, modes, and non-comparable report durations.
- Four policy records means exactly four unique records in the approved order or
  another explicitly validated canonical representation.
- Every accepted identity field is copied into the candidate report so a human
  can trace the candidate to its measurements and base file.
- Tests cover mixed commits, mixed policy roots/assets, duplicate policies,
  duration mismatch, stale base input, and valid same-deployment reports.

### Resolution

- **Resolved**: 2026-08-12T16:59:22+08:00
- **Commit**: 本提交
- **Approved Scope**: profiler 报告升级为 schema v2，在模型加载前固定源码提交、
  结构化主机身份、规范化策略根、每策略时长以及固定顺序的 11 个批准策略资产
  SHA-256，并在写报告前复核资产未变化。分析器仅接受同一提交、主机、策略根和
  资产摘要的报告，要求四个策略恰好各一条且顺序一致，要求全部 host 报告具有
  相同测量时长，并将 hardware 时长独立记录；`--base-yaml` 必须是该身份中的
  `LW/base.yaml` 且摘要一致。候选 schema v2 复制部署身份、base 和每个输入报告
  的路径/摘要/模式/时长，继续保持仅供评审、致命时序关闭及原物理安全限制。
- **Changed Files**: `src/rl_sar/src/lw_config_profiler.cpp`、
  `src/rl_sar/scripts/profile_lw_runtime_config.py`、`src/rl_sar/CMakeLists.txt`、
  `src/rl_sar/test/{test_profile_lw_runtime_config.py,test_lw_config_profiler_integration.py}`、
  `docs/LW_BUILD_DEPLOYMENT_CN.md`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **Verification**: 当前 Debug 工作树完整构建并通过 36/36 CTest；分析器 17 项
  单元测试覆盖旧 schema、混合提交/主机/策略根/资产、重复/缺失/乱序策略、
  host 时长不一致、独立 hardware 时长、陈旧 base、模式矛盾、输入变化及合法
  同部署组合。真实 profiler 集成测试核对结构化身份、有序资产摘要、每策略时长、
  host 无硬件输出以及硬件确认/缺失串口拒绝。Release 和严格警告配置均构建
  profiler 并通过定向 analyzer/profiler 测试；ASan/UBSan profiler 构建成功，
  host 测量及错误确认、缺失串口两个拒绝路径分别在受控单进程中通过。ASan 下
  同一 Python 测试连续启动多个 ONNX 子进程会出现环境性进程启动洪泛/10 秒超时，
  普通 Debug、Release、严格警告及各独立 ASan 路径均未复现功能失败。Python
  严格语法、定向 `cppcheck` 和 `git diff --check` 通过。未访问真机串口、未启动
  硬件观察、ROS 节点或电机控制。
- **Remaining Follow-ups**: none

---

## [LW-027] Reproducible ONNX Runtime deployment dependency

**Priority**: P1 / high
**Status**: resolved
**Dependencies**: LW-010, LW-017, LW-020

### Problem

Production LW executables embed an absolute build-host RPATH to the repository's
ONNX Runtime directory. The generated deployment prefix neither contains that
runtime nor records its hash in the manifest. Moving the prefix can therefore
break startup, while replacing the external runtime can change execution even
though `--verify-deployment-only` still accepts the bundle.

### Evidence

- `src/rl_sar/CMakeLists.txt:295-320`
- `src/rl_sar/scripts/generate_lw_deployment_manifest.py:25-36`
- `src/rl_sar/library/core/deployment/lw_deployment_bundle.cpp:38-53`
- `docs/LW_BUILD_DEPLOYMENT_CN.md:739-749`

### Intended Scope

- Define whether ONNX Runtime is bundled or is an explicitly external verified
  dependency, then enforce that contract consistently.
- Remove reliance on an unverified mutable build-tree library path.
- Include every required runtime library and relevant identity in deployment
  generation, integrity checking, and relocation tests.

### Acceptance Criteria

- A deployment either carries the required ONNX Runtime libraries under a
  relocatable runtime search path or rejects startup unless an external runtime
  with the recorded identity is present.
- Runtime libraries are architecture-validated and integrity-verified together
  with the executable and policies.
- Relocating the complete approved deployment prefix does not depend on the
  original source-tree path.
- Tampering with or omitting a required runtime library fails deployment
  verification before ROS, serial, or motor initialization.

### Resolution

- **Resolved**: 2026-08-12T17:24:40+08:00
- **Commit**: 本提交
- **Approved Scope**: 正式部署将实际使用的 ONNX Runtime 主库和共享 provider
  作为普通文件安装到 `lib/rl_sar/onnxruntime/`，两个生产可执行文件仅以
  `$ORIGIN/onnxruntime` 相对 RPATH 加载它们。manifest 升级为 schema v3，
  固定 ONNX Runtime 版本、规范化 CPU 架构、精确两库集合和各自 SHA-256；
  C++ 启动校验拒绝缺失、多余、符号链接、目录逃逸、错误 ELF64 架构和哈希
  不匹配。部署脚本拒绝源码树 ONNX RPATH 和部署外解析结果，并自动验收原部署
  前缀及完整重定位副本。
- **Changed Files**: `src/rl_sar/CMakeLists.txt`、
  `src/rl_sar/cmake/install_lw_deployment.cmake.in`、
  `src/rl_sar/library/core/deployment/{lw_deployment_bundle.cpp,lw_deployment_bundle.hpp}`、
  `src/rl_sar/scripts/{build_lw_deployment.sh,generate_lw_deployment_manifest.py}`、
  `src/rl_sar/test/{test_build_workflow.py,test_generate_lw_deployment_manifest.py,test_lw_deployment_bundle.cpp,test_verify_lw_policy_parity.py}`、
  `docs/LW_BUILD_DEPLOYMENT_CN.md`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **Verification**: 当前 Debug 工作树完整构建并通过 36/36 CTest；manifest
  生成器 13 项测试和 C++ bundle 18 条路径覆盖版本/架构、精确文件集合、缺失、
  符号链接、错误 ELF 架构、篡改、重定位及既有策略/运行文件约束。Release 和
  严格警告配置均构建主程序、配置测量工具与 bundle 测试并通过 4 项定向 CTest；
  ASan/UBSan bundle 测试通过。真实 `LW_PRODUCTION_DEPLOYMENT=ON` Release
  部署生成成功，manifest 记录 ONNX Runtime 1.22.0/x86_64 和两个库摘要；
  `readelf` 仅见 `$ORIGIN/onnxruntime`，`ldd` 对两个生产可执行文件均解析到
  当前部署前缀，原前缀和临时重定位副本的 `--verify-deployment-only` 均通过。
  真实失效注入中，缺 provider、缺主库和替换 provider 内容分别由启动校验、
  动态加载器和 SHA-256 校验在硬件初始化前拒绝。Python 严格语法、shell 语法、
  定向 `cppcheck` 和 `git diff --check` 通过。未访问 ROS 设备、真机串口或电机。
- **Remaining Follow-ups**: none

---

## [LW-028] Signal-safe Sim2Sim shutdown request

**Priority**: P2 / medium
**Status**: resolved
**Dependencies**: LW-024

### Problem

The Sim2Sim `SIGINT` handler writes through `std::cout` and dereferences a global
`RL_Real` pointer to reach the `Simulate` object. C++ streams, shared-object
lifetime checks, and general object access are not async-signal-safe; a signal
arriving during those operations can deadlock or race with destruction.

### Evidence

- `src/rl_sar/src/rl_sim_LW.cpp:74-75`
- `src/rl_sar/src/rl_sim_LW.cpp:1257-1277`

### Intended Scope

- Restrict the signal handler to a signal-safe notification mechanism.
- Perform logging, actuator zeroing, ROS shutdown, and object access in normal
  thread context.
- Coordinate with the joinable lifecycle introduced by LW-024.

### Acceptance Criteria

- The installed handler performs no C++ stream, allocation, locking, or object
  graph access.
- Repeated SIGINT injection during startup, steady simulation, and shutdown
  produces a deterministic clean exit without use-after-free or deadlock.
- Normal window-close and safety shutdown paths retain their existing behavior.

### Resolution

- **Resolved**: 2026-08-12T18:21:42+08:00
- **Commit**: 本提交
- **Approved Scope**: 删除 Sim2Sim 的异步 `SIGINT` handler 和全局
  `RL_Real*`。主线程在创建 ROS、MuJoCo 或工作线程前阻塞 `SIGINT`，专用可
  联结线程通过 `sigtimedwait` 同步接收并把一次性请求交给普通线程 coordinator；
  对象构造前收到的请求会锁存，绑定后通过 `weak_ptr` 调用既有
  `RequestSimulationStop()`。ROS 仅管理 `SIGTERM`。渲染循环返回后才记录日志、
  关闭 ROS 并联结线程；等待线程停止后保持 `SIGINT` 阻塞至进程退出，消除关闭
  尾部恢复默认动作的竞争窗。窗口关闭和既有安全停止路径保持不变。
- **Changed Files**: `src/rl_sar/src/rl_sim_LW.cpp`、
  `src/rl_sar/include/rl_sim_LW.hpp`、
  `src/rl_sar/library/core/simulation/lw_signal_shutdown.hpp`、
  `src/rl_sar/test/{test_lw_signal_shutdown.cpp,test_lw_sim_lifecycle_integration.py}`、
  `src/rl_sar/CMakeLists.txt`、`docs/LW_BUILD_DEPLOYMENT_CN.md`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **Verification**: 当前 Debug 工作树完整构建并通过 37/37 CTest。新增信号
  测试覆盖构造前锁存、稳态重复请求、无信号有界关闭、关闭期间并发信号、回调
  异常保留、原掩码恢复和进程退出前持续阻塞；源码集成测试确认不再存在自定义
  异步 handler 或全局对象指针，并保留 MuJoCo 待加载唤醒及窗口关闭测试。
  Release 和严格警告配置均构建 `rl_sim_LW` 与信号测试并通过 3 项定向 CTest；
  ASan/UBSan 信号测试连续 5 次通过，既有 MuJoCo 生命周期测试单独受控运行通过。
  真实 `rl_sim_LW` 进程连续 5 轮承受从稳态持续到进程退出的 SIGINT 洪泛，每轮
  679–733 次，均打印正常退出日志并返回 0。ThreadSanitizer 在本机运行时初始化
  即因 `unexpected memory mapping` 退出，未执行测试代码；定向 `cppcheck`、
  Python 严格语法和 `git diff --check` 通过。进程在进入 `main()` 前仍遵循
  操作系统默认 SIGINT 行为，该 pre-main 区间无法由程序内线程机制消除。未启动
  真机节点、未访问串口或电机。
- **Remaining Follow-ups**: none

---

## [LW-029] Sim2Sim actuator-model policy-root consistency

**Priority**: P2 / medium
**Status**: resolved
**Dependencies**: LW-010, LW-021

### Problem

`rl_sim_LW --policy-root` redirects the four primary LW policies, but optional
actuator-network models are still loaded through the compile-time `POLICY_DIR`.
The same process can therefore combine primary policies from one tree with
actuator models from another, or fail after relocation even when the selected
policy root is complete.

### Evidence

- `src/rl_sar/src/rl_sim_LW.cpp:54-67`
- `src/rl_sar/src/rl_sim_LW.cpp:134-152`

### Intended Scope

- Resolve every Sim2Sim policy-related asset through the selected, validated
  policy root.
- Fail explicitly when `--use_actuator_net` is requested but either model is
  missing or incompatible.
- Keep the optional actuator network outside the real deployment scope unless
  separately approved.

### Acceptance Criteria

- No `--use_actuator_net` asset path falls back to compile-time `POLICY_DIR`
  after an explicit policy root is selected.
- Tests demonstrate relocation, missing-model rejection, and consistent model
  selection without requiring a GUI run.
- Startup diagnostics identify the exact resolved actuator-model paths.

### Resolution

- **Resolved**: 2026-08-12T19:32:04+08:00
- **Commit**: 本提交
- **Approved Scope**: 将 Sim2Sim 可选执行器模型解析集中到独立组件，并以
  `SetPolicyRoot()` 保存的规范化策略根作为唯一来源。启用
  `--use_actuator_net` 时固定解析
  `LW/robot_lab/motors/{leg,foot}_actuator_net.pt`，逐项记录最终绝对路径，
  任一文件缺失、TorchScript 加载失败、拒绝 6 维输入、未产生恰好一个输出或
  产生非有限输出都会抛出带模型路径的启动错误；未启用该选项时仍不要求这两个
  可选资产。未修改真机执行路径。
- **Changed Files**: `src/rl_sar/library/core/simulation/lw_actuator_models.{hpp,cpp}`、
  `src/rl_sar/src/rl_sim_LW.cpp`、`src/rl_sar/include/rl_sim_LW.hpp`、
  `src/rl_sar/test/{test_lw_actuator_models.cpp,test_lw_sim_lifecycle_integration.py}`、
  `src/rl_sar/CMakeLists.txt`、`README.md`、`README_CN.md`、
  `docs/LW_BUILD_DEPLOYMENT_CN.md`、`.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **Verification**: 当前 Debug 工作树完整构建并通过 38/38 CTest。新增无 GUI
  测试把仓库实际 leg/foot `.pt` 文件复制到临时搬迁策略根并从该根成功加载、
  预热，同时确认所选根缺少 foot 文件而编译期根仍有同名文件时会明确拒绝、
  不会回退；伪模型覆盖未加载、非 TorchScript、6 维输入不兼容、多输出和非有限
  输出。当前仓库两个原始模型也通过相同 6→1 契约。独立 Release 构建
  `rl_sim_LW` 和测试目标并通过 2/2 定向 CTest；新组件与测试在
  `-Wall -Wextra -Wpedantic -Werror` 下构建并通过。ASan/UBSan 测试逻辑完成，
  禁用泄漏检测后无地址或未定义行为报告；启用 LeakSanitizer 时测试已打印通过，
  但进程退出阶段在 Conda `libstdc++` 的 `std::filesystem` 路径中报告 707 字节
  残留。定向 `cppcheck`、Python 严格语法和 `git diff --check` 通过。未启动
  MuJoCo GUI、真机节点，未访问串口或电机。
- **Remaining Follow-ups**: none

---

## [LW-030] Coherent inhibited-command gait observation

**Priority**: P2 / medium
**Status**: resolved
**Dependencies**: LW-021, LW-025

### Problem

When external input is faulty or the safety supervisor inhibits input, the
inference observation publishes zero `commands`, but `command_norm` and the
moving gait phase are computed from the pre-inhibition local control snapshot.
An inference frame can therefore contain a zero velocity command together with
a nonzero moving gait phase.

### Evidence

- `src/rl_sar/library/core/safety/lw_runtime_core.hpp:377-411`

### Intended Scope

- Derive command-dependent observation fields from one effective command after
  all inhibition and external-fault rules have been applied.
- Preserve current gait-phase timing for valid nonzero commands.
- Keep real, Sim2Sim, and host-profiler inference behavior identical.

### Acceptance Criteria

- A zeroed effective command always produces the documented stationary gait
  observation in the same inference frame.
- Tests cover normal movement, joystick fault, timing degradation, and external
  input fault without relying on scheduling luck.
- Nominal policy input/output parity remains unchanged for valid commands.

### Resolution

- **Resolved**: 2026-08-12T19:50:43+08:00
- **Commit**: 本提交
- **Approved Scope**: `LWRuntimeCore::runInferenceCycle()` 从每帧已发布的控制
  快照构造一次有效命令；外部输入故障或安全监督器输入抑制会统一清零有效
  `x/y/yaw`。策略 `commands`、命令范数、运动判定和最终 `gait_phase` 全部读取
  该有效命令，保证同一帧的零速度对应 `{0, 0}` 静止相位。按用户决定，内部
  相位时钟在短暂抑制期间保持连续而不重置；有效非零命令的阈值、推进速度和
  正弦/余弦计算顺序不变。real、Sim2Sim 和 host profiler 继续共用同一路径。
- **Changed Files**: `src/rl_sar/library/core/safety/lw_runtime_core.hpp`、
  `src/rl_sar/test/test_lw_runtime_parity.cpp`、
  `docs/LW_BUILD_DEPLOYMENT_CN.md`、`.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **Verification**: 当前 Debug 工作树完整构建并通过 38/38 CTest。共享运行时
  测试先发布非零策略输入，再于推理前分别注入 `JoystickUnavailable`、
  `ControlTimingDegraded` 和 `external_input_fault=true`，确定性确认每种情况的
  当帧 `commands={0,0,0}`、`gait_phase={0,0}`；正常路径保持原命令和精确相位
  数值，临时外部抑制后的恢复帧证明内部时钟没有重置。现有真实 ONNX
  real/Sim2Sim 推理奇偶性继续通过。独立 Release 构建成功链接
  `rl_real_LW`、`rl_sim_LW` 和 `lw_config_profiler`，运行时奇偶性与 profiler
  集成测试 2/2 通过。完整严格 `-Werror` 构建被 LW-031 已登记的既有
  `ObservationBuffer -Wreorder` 阻断；仅将该类别降为非致命后，本次目标在
  `-Wall -Wextra -Wpedantic -Werror` 其余规则下构建并通过。ASan/UBSan、定向
  `cppcheck` 和 `git diff --check` 通过。未启动 MuJoCo GUI、真机节点，未访问
  串口或电机。
- **Remaining Follow-ups**: none

---

## [LW-031] Warning-clean maintained LW build

**Priority**: P2 / low
**Status**: resolved
**Dependencies**: none

### Problem

A warning-enabled build exposes maintained-code warnings, including an
`ObservationBuffer` member-initialization order mismatch and a range-loop test
that binds `std::string` references to temporary conversions. The latter stops
the existing strict `-Werror` build before all targets are checked. This weakens
the value of compiler warnings as a regression gate and can hide new defects
among third-party MuJoCo warning noise.

### Evidence

- `src/rl_sar/library/core/observation_buffer/observation_buffer.hpp:55-62`
- `src/rl_sar/library/core/observation_buffer/observation_buffer.cpp:14-22`
- `src/rl_sar/test/test_lw_fsm_transitions.cpp:66-71`

### Intended Scope

- Correct warnings in maintained LW source and tests without behavior changes.
- Apply strict warning settings to maintained targets while treating vendored
  third-party code as a separately scoped dependency.
- Add a reproducible warning-clean check to the validation workflow.

### Acceptance Criteria

- Maintained LW source and test targets compile with the approved warning set
  and `-Werror`.
- Third-party warnings do not suppress or obscure maintained-code failures.
- The normal build and full CTest suite remain green.

### Resolution

- **Resolved**: 2026-08-12T20:18:05+08:00
- **Commit**: 本提交
- **Approved Scope**: 修正维护代码中全部审计警告且不改变行为：按声明顺序
  初始化 `ObservationBuffer`，FSM 测试按值构造字符串，real/Sim2Sim 的
  `LowCmd`/`LowState` 使用完整值初始化，并明确标记 real 构造函数保留参数未
  使用。新增默认关闭的 `LW_STRICT_WARNINGS` CMake 选项，对 GCC/Clang 维护
  目标施加 `-Wall -Wextra -Wpedantic -Werror`，对 MSVC 使用 `/W4 /WX`。
  joystick 与 MuJoCo simulate 源码拆分为独立 vendor 库，硬件 SDK、joystick
  和 MuJoCo 头文件通过 `SYSTEM` 边界传播；仅 vendor 编译目标在严格模式关闭
  自身诊断，不修改第三方源码，也不豁免维护目标。新增临时目录一键严格构建与
  全量 CTest 脚本。
- **Changed Files**: `src/rl_sar/CMakeLists.txt`、
  `src/rl_sar/library/core/observation_buffer/observation_buffer.cpp`、
  `src/rl_sar/include/{rl_real_LW.hpp,rl_sim_LW.hpp}`、
  `src/rl_sar/src/rl_real_LW.cpp`、
  `src/rl_sar/test/{test_lw_fsm_transitions.cpp,test_build_workflow.py}`、
  `scripts/validate_lw_strict_build.sh`、`README.md`、`README_CN.md`、
  `docs/LW_BUILD_DEPLOYMENT_CN.md`、`.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **Verification**: 初始完整 `-Wall -Wextra -Wpedantic` 审计统计 873 条诊断：
  维护代码包含 `-Wreorder`、`-Wrange-loop-construct`、聚合初始化和未使用参数，
  其余 850 余条来自 vendored MuJoCo/joystick。修复后运行
  `scripts/validate_lw_strict_build.sh`，全新 Debug 严格构建的所有维护库、测试、
  `rl_real_LW`、`rl_sim_LW` 和 `lw_config_profiler` 均在 `-Werror` 下成功，vendor
  输出无警告噪声，随后 38/38 CTest 通过；脚本自动清理临时目录。普通 Debug
  全构建及 38/38 CTest 同样通过。独立 Release 成功构建 real、Sim2Sim、MuJoCo
  生命周期和 FSM 目标，3/3 定向 CTest 通过。构建工作流静态测试 13/13、定向
  `cppcheck`、Python 严格语法、shell 语法和 `git diff --check` 通过。未启动
  MuJoCo GUI、真机节点，未访问串口或电机。
- **Remaining Follow-ups**: none

---

## [LW-032] End-to-end IMU and AHRS validity and freshness

**Priority**: P0 / critical
**Status**: resolved
**Dependencies**: LW-002, LW-003, LW-005, LW-017

### Problem

The project-owned FDILink driver can publish a newly timestamped `/imu` message
whose orientation is uninitialized or comes from an indefinitely old AHRS
frame. Its serial timeout and first-sequence flag are not initialized, and
individual serial reads do not reject short reads before parsing the shared
frame buffers. The real controller treats receipt of that combined ROS message
as proof that both angular velocity and orientation are fresh. Its feedback
validation rejects non-finite quaternion values but accepts a finite
non-quaternion such as `[0, 0, 0, 0]`, which attitude conversion reports as a
safe zero roll and pitch.

### Evidence

- `src/fdilink_ahrs_ROS2/include/ahrs_driver.h:63-81`
- `src/fdilink_ahrs_ROS2/src/ahrs_driver.cpp:8-50`
- `src/fdilink_ahrs_ROS2/src/ahrs_driver.cpp:74-101`
- `src/fdilink_ahrs_ROS2/src/ahrs_driver.cpp:110-205`
- `src/fdilink_ahrs_ROS2/src/ahrs_driver.cpp:303-440`
- `src/fdilink_ahrs_ROS2/src/ahrs_driver.cpp:444-495`
- `src/rl_sar/src/rl_real_LW.cpp:614-695`
- `src/rl_sar/library/core/safety/lw_control_safety.hpp:87-119`
- `src/rl_sar/library/core/safety/lw_control_safety.hpp:243-280`
- `scripts/validate_lw_strict_build.sh:25-31`
- `src/rl_sar/test/test_lw_runtime_dependencies.py:22-34`

### Intended Scope

- Preserve the third-party FDLink package unchanged. Treat its CRC- and
  frame-end-checked `/euler_angles` publish path as an independent AHRS
  liveness event, while retaining the documented limitation that the ROS topic
  does not expose the serial read count, device timestamp, or sequence.
- Remap FDLink `/imu` and `/euler_angles` into explicitly untrusted internal
  topics for the normal real launch; do not let raw `/imu` directly satisfy
  controller readiness.
- Grant each finite AHRS event one short-lived authorization for exactly one
  subsequent IMU message. Reject missing, consumed, regressing, or over-age
  authorization rather than refreshing cached orientation indefinitely.
- Reject non-finite, zero-norm, implausibly scaled, or otherwise invalid
  quaternions and non-finite angular velocity before readiness or attitude
  protection can authorize commands; normalize accepted quaternions.
- Configure motor-feedback freshness, trusted-IMU freshness, and IMU/AHRS pair
  age independently. Preserve 100 ms as an explicitly unapproved pre-measurement
  placeholder instead of inferring a limit from the nominal 400 Hz device or
  50 Hz policy frequency.
- Extend suspended hardware profiling to record raw IMU, valid AHRS, trusted
  IMU, pair-age, and bilateral feedback distributions and require separate
  operator-supplied safety ceilings before generating review-only candidates.

### Acceptance Criteria

- Raw FDLink `/imu` cannot directly produce a controller-ready sample; a
  never-seen, invalid, consumed, or stale AHRS authorization blocks it.
- One AHRS event authorizes at most one subsequent IMU message within the
  configured inclusive pair-age boundary.
- A finite invalid quaternion, including the zero quaternion, prevents command
  activation; non-finite orientation, Euler data, or angular velocity is also
  rejected before readiness.
- Startup without a trusted pair remains in the existing disable-only waiting
  state; loss after Ready expires on `trusted_imu_timeout` and latches the
  existing hard-disable-and-shutdown action.
- Hardware reports lacking raw IMU, valid AHRS, trusted IMU, pair samples, or
  bilateral feedback proof cannot generate a candidate.
- Automated tests demonstrate the guard, independent timeouts, configuration,
  launch remapping, profiler schema, and analyzer rejection paths without
  opening physical serial devices or sending motor commands; the full strict
  build remains green.

### Resolution

- **Resolved**: 2026-08-13T14:04:40+08:00
- **Commit**: 本提交
- **Approved Scope**: 按用户明确选择不修改第三方
  `src/fdilink_ahrs_ROS2`。完整 LW launch 将 FDLink 的 `/imu`、
  `/euler_angles` 重映射为 `/fdilink/raw_imu`、`/fdilink/raw_euler`；
  `rl_sar` 自有 guard 以有限 AHRS 事件一次性授权下一帧原始 IMU，限制配对
  时效，拒绝未授权、重复使用、时间倒退、非有限数据、零模或异常缩放四元数，
  并在交给 readiness 前归一化。电机反馈、可信 IMU、IMU/AHRS 配对分别使用
  `sensor_timeout`、`trusted_imu_timeout`、`imu_ahrs_pair_max_age`。硬件测算器
  复用同一 guard 并输出三类 IMU/AHRS 分布及配对时延；分析器要求对应的独立
  操作员安全上限，报告 schema 升级至 v3。
- **Changed Files**: `policy/LW/base.yaml`、`src/rl_sar/CMakeLists.txt`、
  `src/rl_sar/include/rl_real_LW.hpp`、
  `src/rl_sar/launch/rl_real_LW.launch.py`、
  `src/rl_sar/library/core/rl_sdk/lw_configuration_validation.cpp`、
  `src/rl_sar/library/core/safety/{lw_imu_ahrs_guard.hpp,sensor_readiness.hpp}`、
  `src/rl_sar/src/{rl_real_LW.cpp,lw_config_profiler.cpp}`、
  `src/rl_sar/scripts/profile_lw_runtime_config.py`、
  `src/rl_sar/test/{test_lw_imu_ahrs_guard.cpp,test_sensor_readiness.cpp,`
  `test_profile_lw_runtime_config.py,test_lw_config_profiler_integration.py,`
  `test_lw_real_startup_disable_integration.py}`、
  `docs/LW_BUILD_DEPLOYMENT_CN.md`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **Verification**: 普通 Debug 全构建成功且 39/39 CTest 通过；全新
  `scripts/validate_lw_strict_build.sh` 在
  `-Wall -Wextra -Wpedantic -Werror` 下完成所有维护目标并通过 39/39 CTest；
  profiler/analyzer 定向 18/18 Python 测试、launch Python 语法和加载构造、
  `git diff --check` 通过。未启动 ROS 节点、IMU、串口、MuJoCo GUI 或电机，
  `src/fdilink_ahrs_ROS2` 无任何修改。
- **Accepted Limitation**: 外部 guard 能证明 FDLink 刚经过其 AHRS CRC/帧尾
  检查并发布事件，但第三方无 Header 的 `/euler_angles` 不暴露设备时间戳、
  序号或 `serial::read()` 实际长度，因此不能从 ROS 层独立证明每次底层读取均
  完整。按用户决定，允许少量重复旧姿态，并以一次性授权、配对时限、内容校验
  和可信输出超时阻止无限续命；若以后要求逐字节串口完整性证明，必须另行批准
  FDLink 最小接口补丁或 `rl_sar` 串口代理。
- **Remaining Follow-ups**: 当前两个新增 IMU 时限均为 100 ms 未测量占位值。
  正式部署前必须在目标机吊装采集 schema v3 硬件报告，由现场安全评审分别给出
  最大电机反馈时效、最大可信 IMU 时效、最大配对时延和最大控制间断，再审查候选；
  不得从名义 400 Hz 或 50 Hz 策略频率直接推定。

---

## [LW-033] Policy input provenance and freshness

**Priority**: P1 / high
**Status**: resolved
**Dependencies**: LW-007, LW-008, LW-011, LW-021, LW-032

### Problem

`LWPolicyInputSnapshot` contains robot state and control values but no capture
time or sequence. The inference loop assigns the policy output's `source_time`
from the inference cycle rather than from the input snapshot. If the control
loop stalls while inference continues, repeated outputs based on the same old
robot state can therefore appear fresh. When control resumes,
`EvaluateLWPolicyOutput()` can accept one of those outputs even though its
source state predates the configured output-age limit.

### Evidence

- `src/rl_sar/library/core/rl_sdk/rl_sdk.hpp:222-237`
- `src/rl_sar/library/core/safety/lw_runtime_core.hpp:356-399`
- `src/rl_sar/library/core/safety/lw_runtime_core.hpp:452-459`
- `src/rl_sar/library/core/safety/lw_runtime_core.hpp:579-587`
- `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp:17-43`
- `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp:1213-1255`
- `policy/LW/base.yaml:23-38`

### Intended Scope

- Add a monotonic capture time and sequence/generation identity to every policy
  input snapshot.
- Propagate input provenance into the complete policy output frame.
- Define freshness against the state capture time and reject duplicate,
  regressing, future-dated, or over-age inputs and outputs.
- Coordinate stale-input handling with the existing proportional safety policy
  and control-loop timing degradation behavior.
- Test a stalled control producer with an active inference consumer, recovery,
  policy-generation switches, and age-boundary behavior.

### Acceptance Criteria

- A newly computed output based on an old state snapshot is never classified as
  ready solely because inference ran recently.
- The control path cannot consume an output whose originating state is older
  than the approved limit or belongs to another policy generation.
- Tests reproduce the control-stall scenario deterministically and verify the
  approved fallback and recovery semantics for both real and Sim2Sim adapters.
- Existing coherent-frame and runtime-parity tests remain green.

### Resolution

- **Resolved**: 2026-08-13T14:41:44+08:00
- **Commit**: 本提交
- **Approved Scope**: 每个 `LWPolicyInputSnapshot` 现在携带活动策略代际、全局
  单调序号以及 `GetState()` 完成时的单调时钟采集时间；推理线程按代际验证输入，
  对每个输入最多消费一次，重复输入不会推进策略帧、相位、历史、输出或进度。
  策略输出携带源输入序号与源状态采集时间，输入检查和推理完成后的二次检查均
  使用既有 `3 * dt * decimation` 数据年龄上限（当前配置为 60 ms），而不是以
  推理完成时间刷新年龄。输出传输和控制消费者拒绝缺失、回退或重复的新来源；
  200 Hz 控制循环仍允许在年龄窗口内保持同一个完整 50 Hz 输出。策略切换期间的
  旧代际输入和暂时重复输入只跳过；不完整、来源回退、未来时间或过期输入触发
  新的 `PolicyInputUnavailable` S2 Passive 阻尼锁存，必须重启，不改变既有硬失能
  分级或配置阈值。
- **Changed Files**: `src/rl_sar/library/core/rl_sdk/rl_sdk.hpp`、
  `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp`、
  `src/rl_sar/library/core/safety/lw_runtime_core.hpp`、
  `src/rl_sar/library/core/safety/lw_safety_policy.hpp`、
  `src/rl_sar/test/test_lw_policy_output_transport.cpp`、
  `src/rl_sar/test/test_lw_runtime_parity.cpp`、
  `src/rl_sar/test/test_lw_safety_policy.cpp`、
  `docs/LW_BUILD_DEPLOYMENT_CN.md`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **Verification**: 普通 Debug 完整构建成功且 39/39 CTest 通过；全新
  `scripts/validate_lw_strict_build.sh` 在
  `-Wall -Wextra -Wpedantic -Werror` 下完成所有维护目标并通过 39/39 CTest；
  `git diff --check` 通过。定向测试覆盖 60 ms 年龄边界、未来/过期/重复/回退
  输入、重复和回退输出来源、控制生产者停滞而推理消费者继续运行、停滞后的 S2
  real/Sim2Sim 一致性、重复输入后新输入恢复、策略代际切换，以及同一有效输出被
  多个控制调用保持使用。未启动 ROS 节点、IMU、串口、MuJoCo GUI 或电机，
  `src/fdilink_ahrs_ROS2` 无任何修改。
- **Remaining Follow-ups**: none

---

## [LW-034] Trusted inference-runtime download integrity

**Priority**: P1 / high
**Status**: resolved
**Dependencies**: LW-020, LW-027

### Problem

The inference-runtime bootstrap pins version strings and HTTPS URLs but does
not pin or verify trusted archive digests before extraction. Structural and ELF
architecture validation can accept a substituted archive with the expected
layout. The deployment manifest then hashes the bytes that happened to be
downloaded, which detects later mutation but does not establish that they were
the approved upstream release.

### Evidence

- `scripts/download_inference_runtime.sh:62-69`
- `scripts/download_inference_runtime.sh:112-157`
- `scripts/download_inference_runtime.sh:202-259`
- `scripts/validate_inference_runtime.sh:15-100`
- `src/rl_sar/scripts/generate_lw_deployment_manifest.py:38-41`
- `src/rl_sar/scripts/generate_lw_deployment_manifest.py:194-208`

### Intended Scope

- Pin a reviewed SHA-256 digest for every supported OS/architecture archive used
  by the bootstrap.
- Verify the complete downloaded archive before extraction or replacement of an
  existing runtime.
- Fail closed on missing, malformed, or mismatched digest data and remove
  incomplete temporary artifacts safely.
- Keep version, architecture, URL, expected digest, and deployed library hashes
  reviewably connected.
- Add offline tests using synthetic valid and tampered archives; do not require
  network downloads in the normal test suite.

### Acceptance Criteria

- No downloaded inference archive is extracted or installed until its digest
  matches the pinned value for the selected platform.
- A one-byte archive change is rejected before any approved runtime directory
  is replaced.
- Unsupported platform/version combinations fail explicitly rather than using
  an unverified fallback.
- Deployment generation continues to bind the installed runtime libraries and
  all bootstrap tests pass without network access.

### Resolution

- **Resolved**: 2026-08-13T15:49:27+08:00
- **Commit**: 本提交
- **Approved Scope**: 推理运行时支持矩阵收紧为 Linux x86_64 的 LibTorch
  2.3.0/ONNX Runtime 1.22.0，以及 Linux aarch64 的 ONNX Runtime 1.22.0；
  Darwin、Windows 和其他未审查组合不再使用回退 URL。清单固定运行时类型、
  版本、OS、规范化架构、精确官方 HTTPS URL、归档名/格式/根目录和完整归档
  SHA-256。三个摘要于 2026-08-13 从清单中的精确 PyTorch/ONNX Runtime 官方
  URL 流式读取完整归档并计算：LibTorch x86_64
  `f60009d2a74b6c8bdb174e398c70d217b7d12a4d3d358cd1db0690b32f6e193b`，
  ONNX x86_64
  `8344d55f93d5bc5021ce342db50f62079daf39aaafb5d311a451846228be49b3`，
  ONNX aarch64
  `bb76395092d150b52c7092dc6b8f2fe4d80f0f3bf0416d2f269193e347e24702`。
  下载器使用独立临时归档，摘要匹配前不解压；候选在同一文件系统的隔离目录完成
  安全路径、精确根目录、结构和 ELF 架构验证并写入来源证明后才替换结构损坏的
  目录。任一失败均清理候选并保留旧目录；结构有效但来源缺失、版本更高或来源
  不匹配的现有运行时会明确停止并要求单独审查升级，不自动覆盖。正式生产 CMake
  要求来源证明匹配仓库清单；部署 manifest 升级为 schema v4，绑定批准的 ONNX
  归档名/URL/SHA-256、来源文件哈希及实际部署库哈希，运行时验证器同时与编译进
  二进制的 x86_64/aarch64 批准归档身份比对。用户 Python 环境中的包不受影响。
- **Changed Files**: `.gitignore`、`README.md`、`README_CN.md`、
  `docs/LW_BUILD_DEPLOYMENT_CN.md`、
  `scripts/inference_runtime_archives.json`、
  `scripts/manage_inference_runtime.py`、
  `scripts/download_inference_runtime.sh`、`src/rl_sar/CMakeLists.txt`、
  `src/rl_sar/cmake/install_lw_deployment.cmake.in`、
  `src/rl_sar/scripts/generate_lw_deployment_manifest.py`、
  `src/rl_sar/library/core/deployment/lw_deployment_bundle.hpp`、
  `src/rl_sar/library/core/deployment/lw_deployment_bundle.cpp`、
  `src/rl_sar/test/test_inference_runtime_download_integrity.py`、
  `src/rl_sar/test/test_build_workflow.py`、
  `src/rl_sar/test/test_generate_lw_deployment_manifest.py`、
  `src/rl_sar/test/test_lw_deployment_bundle.cpp`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **Verification**: 普通 Debug 完整构建成功且 40/40 CTest 通过；全新
  `scripts/validate_lw_strict_build.sh` 在
  `-Wall -Wextra -Wpedantic -Werror` 下完成全部维护目标并通过 40/40 CTest；
  Python 严格语法、Shell 语法和 `git diff --check` 通过。新增 7 项纯离线测试
  使用合成 TGZ/ELF 覆盖正确安装及精确来源、单字节归档篡改在解压前拒绝、候选
  结构失败、结构有效但无来源时拒绝自动替换、无效旧目录只在候选完全验证后
  替换、重复/不支持清单失败和三项生产支持矩阵。manifest 生成与 C++ 启动验证
  测试覆盖来源缺失/不匹配、未批准版本/归档、来源文件及部署库篡改。正常测试套件
  未执行网络下载；未启动 ROS 节点、IMU、串口、MuJoCo GUI 或电机。
- **Accepted Limitation**: 固定摘要是对指定官方 HTTPS 资产完整字节的项目审查
  信任锚，不是上游数字签名。以后升级版本必须单独审查新 URL 与摘要并重新完成
  全量构建、Sim2Sim 和部署验收，不能把更高版本静默视为当前批准版本。
- **Remaining Follow-ups**: none

---

## [LW-035] Production launch-file integrity

**Priority**: P2 / medium
**Status**: resolved
**Dependencies**: LW-010, LW-017, LW-019, LW-023, LW-027, LW-034

### Problem

The production instructions start the robot through
`rl_real_LW.launch.py`, and that file controls whether the FDILink node starts
and whether keyboard/debug channels are enabled. The production manifest's
exact runtime file set does not include the installed `rl_sar` launch file.
Changing or replacing it after bundle generation therefore does not make
`--verify-deployment-only` fail even though the verified executable may be
started with materially different dependencies or parameters.

### Evidence

- `src/rl_sar/launch/rl_real_LW.launch.py:10-45`
- `src/rl_sar/CMakeLists.txt:1144-1149`
- `src/rl_sar/scripts/build_lw_deployment.sh:64-80`
- `src/rl_sar/scripts/build_lw_deployment.sh:145-149`
- `src/rl_sar/scripts/generate_lw_deployment_manifest.py:25-36`
- `src/rl_sar/library/core/deployment/lw_deployment_bundle.cpp:39-53`
- `src/rl_sar/test/test_lw_deployment_bundle.cpp:33-44`

### Intended Scope

- Add the exact installed real-robot launch file and any project-owned launch
  child it relies on to the production manifest's approved runtime set.
- Preserve symlink, containment, exact-set, and SHA-256 checks for those files.
- Add generator and runtime-verifier tests for missing, changed, extra, and
  symlinked launch assets.
- Keep development-only and Sim2Sim launch assets outside the production set
  unless separately justified.

### Acceptance Criteria

- The documented production launch path is cryptographically bound to the same
  source commit and bundle as `rl_real_LW`.
- Missing, modified, escaping, or symlinked required launch files make offline
  deployment verification fail before hardware access.
- A clean relocated production prefix still passes verification and resolves
  all required ROS packages within that prefix.

### Resolution

- **Resolved**: 2026-08-14T18:57:31+08:00
- **Commit**: 本提交
- **Approved Scope**: `LW_PRODUCTION_DEPLOYMENT=ON` 时只安装项目自有的
  `share/rl_sar/launch/rl_real_LW.launch.py`，不再把 Gazebo launch 或
  `worlds` 带入正式前缀；开发构建保持原有安装范围。manifest 的
  `runtime_files` 同时绑定该 launch 及
  `share/ament_index/resource_index/packages/rl_sar`，并继续绑定其引用的
  FDLink launch。生成器和 C++ 离线验证器都要求 `share/rl_sar/launch`
  是前缀内的真实目录、只含批准的单一普通文件，且 manifest 路径集合和
  SHA-256 完全匹配；缺失、修改、额外文件/目录、路径逃逸以及文件或目录符号
  链接均失败。生产构建在原始与迁移前缀内使用
  `PYTHONDONTWRITEBYTECODE=1 ros2 launch ... --show-args` 验证 `rl_sar`、
  FDLink 和 launch 参数解析而不启动节点。所有正式启动文档及构建输出均固定
  `PYTHONDONTWRITEBYTECODE=1`，避免 Python 在精确集合目录内生成或使用未绑定的
  `__pycache__`；省略时额外缓存会被完整性检查安全拒绝。manifest 格式未变化，
  保持 schema v4。
- **Changed Files**: `README.md`、`README_CN.md`、
  `docs/LW_BUILD_DEPLOYMENT_CN.md`、`src/rl_sar/CMakeLists.txt`、
  `src/rl_sar/scripts/build_lw_deployment.sh`、
  `src/rl_sar/scripts/generate_lw_deployment_manifest.py`、
  `src/rl_sar/library/core/deployment/lw_deployment_bundle.cpp`、
  `src/rl_sar/test/test_build_workflow.py`、
  `src/rl_sar/test/test_generate_lw_deployment_manifest.py`、
  `src/rl_sar/test/test_lw_deployment_bundle.cpp`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **Verification**: 三项定向 CTest
  `lw_deployment_bundle`、`lw_deployment_manifest_generator` 和
  `lw_build_workflow` 全部通过；普通 Debug 完整构建成功且 40/40 CTest 通过；
  全新 `scripts/validate_lw_strict_build.sh` 在
  `-Wall -Wextra -Wpedantic -Werror` 下构建成功并通过 40/40 CTest。第一次最终
  严格运行中与本项无关的 `lw_debug_publisher` 首次发布时序测试单次失败，其余
  39 项通过；未修改该测试，第二次全新严格构建和 40 项测试全部通过。另在隔离
  临时 Git 仓库中下载并核验 LW-034 批准的 ONNX Runtime，正式生成生产前缀；
  原始前缀和复制迁移前缀均完成 ROS 包解析、`--show-args` 与
  `--verify-deployment-only`，生产 launch 目录只含
  `rl_real_LW.launch.py` 且不存在 `worlds`。临时目录随后移入系统回收站。
  Python/Shell 语法和 `git diff --check` 通过；未启动 ROS 节点、IMU、串口、
  MuJoCo GUI 或电机，`src/fdilink_ahrs_ROS2` 无修改。
- **Remaining Follow-ups**: none

---

## [LW-036] Runtime actuator-network output validation

**Priority**: P2 / medium
**Status**: resolved
**Dependencies**: LW-005, LW-013, LW-016, LW-021, LW-029

### Problem

Optional Sim2Sim actuator models are checked once at startup with a zero input,
but every later inference result is indexed as `output[0]` without validating
its size or finiteness. An empty dynamic result can cause out-of-bounds access;
a NaN or infinity can enter `actuator_net_tau_` and then MuJoCo control. This
hook runs after the shared robot-command validation, so the auxiliary torque is
outside the existing final finite-command boundary.

### Evidence

- `src/rl_sar/library/core/simulation/lw_actuator_models.cpp:63-103`
- `src/rl_sar/src/rl_sim_LW.cpp:666-749`
- `src/rl_sar/library/core/safety/lw_runtime_core.hpp:317-331`
- `src/rl_sar/src/rl_sim_LW.cpp:910-935`
- `src/rl_sar/test/test_lw_actuator_models.cpp:89-155`
- `src/rl_sar/test/test_lw_actuator_models.cpp:214-251`

### Intended Scope

- Validate the output count and every output value on every actuator-model
  invocation, not only during warmup.
- Route inference exceptions and invalid dynamic outputs through one explicit
  proportional Sim2Sim safety event and prevent any invalid torque write.
- Validate the final combined actuator torque after all adapter hooks and before
  writing MuJoCo control.
- Add a stateful fake model that passes warmup and later returns empty,
  oversized, non-finite, and throwing results.

### Acceptance Criteria

- No actuator-model result is indexed or applied before its shape and values are
  validated.
- Empty, wrong-sized, non-finite, or throwing runtime results produce the
  approved safe action without undefined behavior or invalid MuJoCo controls.
- Valid optional actuator models retain their current behavior and real-robot
  command paths are unchanged.

### Resolution

- **Resolved**: 2026-08-14T20:06:59+08:00
- **Commit**: 本提交
- **Approved Scope**: 执行器模型启动预热与每次 Sim2Sim 运行时调用复用同一
  6 输入、单输出和有限性校验入口，捕获标准及未知推理异常。每个控制周期先
  使旧执行器网络 generation 失效，在独立候选帧中完成全部 leg/foot 关节推理，
  只有全部成功才原子提交力矩和 generation；策略输出无效或 S2 fallback 时不会
  复用旧网络力矩。最终网络/前馈或 MuJoCo PD 力矩先在预分配缓冲区中全部形成，
  验证有限并限幅后才统一写入 `mj_data->ctrl`。新增仅由 Sim2Sim 发出的
  `SimulationActuatorCommandInvalid`，映射为 S4
  `HardDisableAndShutdown`，立即清零 MuJoCo 执行器并停止仿真；真机命令路径
  未修改。
- **Changed Files**: `src/rl_sar/library/core/simulation/lw_actuator_models.hpp`、
  `src/rl_sar/library/core/simulation/lw_actuator_models.cpp`、
  `src/rl_sar/library/core/safety/lw_safety_policy.hpp`、
  `src/rl_sar/include/rl_sim_LW.hpp`、`src/rl_sar/src/rl_sim_LW.cpp`、
  `src/rl_sar/test/test_lw_actuator_models.cpp`、
  `src/rl_sar/test/test_lw_safety_policy.cpp`、
  `src/rl_sar/test/test_lw_sim_lifecycle_integration.py`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **Verification**: 定向构建 `test_lw_actuator_models`、
  `test_lw_safety_policy` 和 `rl_sim_LW` 成功；三项定向 CTest 连续 20 轮通过。
  有状态 fake model 覆盖预热后有效输出、空输出、超长输出、NaN、Inf 和异常；
  事务缓冲测试覆盖失败时 generation 清除且不泄漏部分结果，最终力矩测试覆盖
  有限限幅和无效候选不产生部分写入。当前 Debug 完整构建成功且 40/40 CTest
  通过；全新 `scripts/validate_lw_strict_build.sh` 在
  `-Wall -Wextra -Wpedantic -Werror` 下完成全部维护目标并通过 40/40 CTest。
  定向 `cppcheck` 和 `git diff --check` 通过。未启动 ROS 节点、MuJoCo GUI、
  IMU、串口或电机，用户所有的未跟踪技能目录未修改。
- **Remaining Follow-ups**: none

---

## [LW-037] Complete Sim2Sim SIGTERM and ROS shutdown

**Priority**: P2 / medium
**Status**: resolved
**Dependencies**: LW-024, LW-028

### Problem

Sim2Sim blocks SIGINT and handles it through `LWSigintWaiter`, but delegates
SIGTERM to the ROS signal handler. If SIGTERM causes `rclcpp::spin()` to return
normally, the ROS worker performs no simulation-stop request; only its
exception path calls `RequestSimulationStop()`. The main thread can remain
blocked in MuJoCo `RenderLoop()` until the window is closed manually.

### Evidence

- `src/rl_sar/library/core/simulation/lw_signal_shutdown.hpp:76-212`
- `src/rl_sar/src/rl_sim_LW.cpp:1265-1305`
- `src/rl_sar/src/rl_sim_LW.cpp:1319-1346`
- `src/rl_sar/test/test_lw_sim_lifecycle_integration.py:77-110`
- `src/rl_sar/test/test_lw_signal_shutdown.cpp:53-153`

### Intended Scope

- Make every normal ROS-spin exit, SIGTERM, SIGINT, ROS shutdown request, and
  ROS-thread exception request the same idempotent simulation stop.
- Preserve signal-safe handling and the established join order for ROS,
  rendering, physics, and business workers.
- Add a headless lifecycle test that exercises SIGTERM/normal ROS exit without
  requiring a GUI.

### Acceptance Criteria

- SIGTERM causes bounded, normal Sim2Sim termination without window interaction.
- Normal return from the ROS executor cannot leave `RenderLoop()` running.
- Repeated and concurrent shutdown requests remain idempotent, all workers are
  joined, and stored worker errors are still propagated.

### Resolution

- **Resolved**: 2026-08-14T20:28:39+08:00
- **Commit**: 本提交
- **Approved Scope**: 新增可测试的 ROS worker 包装器，使正常 spin 返回与异常
  都在保存 worker 异常后请求同一个 `LWSimShutdownCoordinator`；SIGINT 继续由
  同步等待线程安全处理，SIGTERM、显式 ROS shutdown、正常 spin 返回、ROS
  worker 异常和主线程生命周期异常统一进入幂等仿真停止路径。保留现有
  `RequestExit()` 唤醒机制、worker join 顺序和异常传播顺序，未修改第三方
  MuJoCo 代码。
- **Changed Files**: `src/rl_sar/CMakeLists.txt`、
  `src/rl_sar/library/core/simulation/lw_signal_shutdown.hpp`、
  `src/rl_sar/src/rl_sim_LW.cpp`、
  `src/rl_sar/test/test_lw_signal_shutdown.cpp`、
  `src/rl_sar/test/test_lw_ros_shutdown.cpp`、
  `src/rl_sar/test/test_lw_sim_lifecycle_integration.py`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **Verification**: 定向构建 `test_lw_signal_shutdown`、
  `test_lw_ros_shutdown` 和 `rl_sim_LW` 成功；正常 ROS shutdown、真实进程内
  SIGTERM、worker 异常及并发重复停止请求等四项定向 CTest 各连续 20 轮通过。
  当前 Debug 完整构建成功且 42/42 CTest 通过；全新
  `scripts/validate_lw_strict_build.sh` 在
  `-Wall -Wextra -Wpedantic -Werror` 下完成全部维护目标并通过 42/42 CTest。
  定向 `cppcheck`、Python 语法检查和 `git diff --check` 通过。未启动 MuJoCo
  GUI、真实机器人、IMU、串口或电机，用户所有的未跟踪技能目录未修改。
- **Remaining Follow-ups**: none

---

## [LW-038] Allocation-bounded real control cycle

**Priority**: P2 / medium
**Status**: resolved
**Dependencies**: LW-007, LW-011, LW-013, LW-031, LW-033

### Problem

The 200 Hz real control path repeatedly decodes YAML values into new vectors.
`GetState()` requests `joint_mapping` three times for every degree of freedom,
while command delivery and FSM interpolation also request container-valued
configuration inside control cycles. Snapshot publication additionally copies
vector-backed robot states under a mutex. These operations introduce allocator
and lock latency into the deterministic loop despite the existing timing
monitor and strict build gate.

### Evidence

- `src/rl_sar/library/core/rl_sdk/rl_sdk.hpp:160-176`
- `src/rl_sar/src/rl_real_LW.cpp:699-703`
- `src/rl_sar/src/rl_real_LW.cpp:717-758`
- `src/rl_sar/fsm_robot/fsm_LW.hpp:86-98`
- `src/rl_sar/fsm_robot/fsm_LW.hpp:160-171`
- `src/rl_sar/library/core/safety/lw_runtime_sync.hpp:10-42`
- `src/rl_sar/library/core/safety/lw_runtime_core.hpp:579-587`

### Intended Scope

- Decode and validate immutable base/policy configuration into typed retained
  storage before worker threads start.
- Remove repeated YAML lookups and container construction from real control and
  FSM execution paths.
- Reuse bounded storage for cross-thread state transport or document and test a
  provably bounded alternative.
- Add an allocation-count or equivalent deterministic regression test and use
  the existing suspended profiler to measure the target-host effect before
  changing deployed timing thresholds.

### Acceptance Criteria

- After startup and policy activation, steady-state real control iterations do
  not allocate because of configuration reads or state transport.
- No unbounded lock wait is introduced into the command deadline path.
- Cached values remain bound to the active validated policy generation and are
  refreshed atomically on an approved policy switch.
- Runtime parity, safety behavior, and full strict tests remain green; target-
  host timing evidence is recorded before any threshold recommendation.

### Resolution

- **Resolved**: 2026-08-14T21:20:17+08:00
- **Commit**: 本提交
- **Approved Scope**: 将 base 和 policy YAML 在校验阶段解码为强类型、只读运行时配置，
  并把 policy 配置与不可变定义及激活 generation 原子绑定；真实控制、LW FSM、
  共享推理运行时和配置 profiler 改用缓存值。策略输入状态传输复用预分配快照，
  控制侧使用非阻塞 `tryPublish()`，仅成功发布时递增序号，争用或跳帧继续由现有
  新鲜度与代际检查处理。增加稳定态分配计数、确定性锁争用、配置解码和代际绑定
  回归测试；未修改时序阈值，未启动硬件、ROS、串口或电机。
- **Changed Files**: `src/rl_sar/CMakeLists.txt`、
  `src/rl_sar/fsm_robot/fsm_LW.hpp`、
  `src/rl_sar/library/core/rl_sdk/lw_configuration_validation.cpp`、
  `src/rl_sar/library/core/rl_sdk/lw_configuration_validation.hpp`、
  `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp`、
  `src/rl_sar/library/core/rl_sdk/rl_sdk.hpp`、
  `src/rl_sar/library/core/safety/lw_runtime_core.hpp`、
  `src/rl_sar/library/core/safety/lw_runtime_sync.hpp`、
  `src/rl_sar/src/lw_config_profiler.cpp`、
  `src/rl_sar/src/rl_real_LW.cpp`、
  `src/rl_sar/src/rl_sim_LW.cpp`、
  `src/rl_sar/test/test_lw_allocation_bound.cpp`、
  `src/rl_sar/test/test_lw_configuration_validation.cpp`、
  `src/rl_sar/test/test_lw_runtime_parity.cpp`、
  `src/rl_sar/test/test_lw_runtime_sync.cpp`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **Verification**: 定向维护目标与 `rl_real_LW`、`rl_sim_LW`、
  `lw_config_profiler` 构建成功；`lw_allocation_bound`、`lw_runtime_sync`、
  `lw_configuration_validation`、`lw_runtime_parity`、`lw_fsm_transitions` 和
  `lw_control_safety` 各连续 20 轮通过，其中稳定态 10,000 次配置访问及策略输入
  快照传输的分配计数为零，锁争用测试验证控制侧发布立即返回且恢复后快照一致，
  代际测试验证切换时强类型定义原子替换。Debug 完整套件 43/43 通过；最终
  `scripts/validate_lw_strict_build.sh` 在
  `-Wall -Wextra -Wpedantic -Werror` 下完成全部目标并通过 43/43 CTest；定向
  `cppcheck`（仅精确抑制未修改的 `CSVInit(std::string)` 既有
  `passedByValue` 提示）和 `git diff --check` 通过。host-only profiler 实际加载、
  预热并运行 4 个正式 ONNX 策略各 0.2 秒，报告为 `failed=false`、
  `commands_sent=none`。没有依据该短时宿主机结果调整阈值；未来任何时序阈值建议
  仍须先取得目标 Jetson/硬件证据。
- **Remaining Follow-ups**: none

---

## [LW-039] Actuator-model policy-root containment

**Priority**: P2 / low
**Status**: resolved
**Dependencies**: LW-010, LW-029

### Problem

`ResolveLWActuatorModelPaths()` canonicalizes the selected policy root but only
lexically joins model paths and checks `is_regular_file()`, which follows
symbolic links. A symlinked leg or foot model can therefore resolve outside the
selected policy root while diagnostics still display the in-root lexical path.

### Evidence

- `src/rl_sar/library/core/simulation/lw_actuator_models.cpp:13-29`
- `src/rl_sar/library/core/simulation/lw_actuator_models.cpp:32-60`
- `src/rl_sar/test/test_lw_actuator_models.cpp:157-212`

### Intended Scope

- Reject symbolic links in every component of an optional actuator-model path.
- Canonicalize each model and prove containment within the selected canonical
  policy root before loading it.
- Test direct file symlinks, symlinked intermediate directories, and escapes to
  otherwise valid external TorchScript files.

### Acceptance Criteria

- No optional actuator model outside the selected policy root can be loaded
  through a symlink or path alias.
- Failure diagnostics identify the rejected lexical and resolved path without
  silently falling back to the compile-time policy directory.
- Normal relocated real directories continue to load both current models.

### Resolution

- **Resolved**: 2026-08-15T12:37:03+08:00
- **Commit**: 本提交
- **Approved Scope**: 继续以所选规范化 policy root 为唯一边界；对固定 leg/foot
  模型相对路径逐级执行非跟随式 `symlink_status()` 检查，拒绝最终模型文件和任一
  中间目录的符号链接。每个模型必须成功规范化、通过按路径组件比较的根目录包含
  检查，并且最终节点是普通非链接文件后才返回。错误同时报告词法路径、解析后
  路径（无法解析时报告明确原因）及命中的链接组件；机器人名额外拒绝 `.`、`..`
  和带父路径的别名。未增加编译期 `POLICY_DIR` 回退，未改变模型 6→1 契约、
  推理输出或真机路径。
- **Changed Files**:
  `src/rl_sar/library/core/simulation/lw_actuator_models.cpp`、
  `src/rl_sar/test/test_lw_actuator_models.cpp`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **Verification**: `test_lw_actuator_models` 与 `rl_sim_LW` 定向构建成功；
  `lw_actuator_models` 连续 20 轮通过，覆盖指向策略根外有效 TorchScript 的直接
  文件链接、中间目录链接、`..` 路径别名、缺失模型拒绝，以及普通搬迁目录中
  两个当前模型的成功解析、加载和 6→1 预热。完整 Debug 构建成功并通过 43/43
  CTest；全新 `scripts/validate_lw_strict_build.sh` 在
  `-Wall -Wextra -Wpedantic -Werror` 下完成全部目标并通过 43/43 CTest；定向
  `cppcheck` 和 `git diff --check` 通过。未启动 MuJoCo GUI、ROS、真机、串口或
  电机，用户未跟踪技能目录保持未修改。
- **Remaining Follow-ups**: none

---

## [LW-040] Safe polymorphic RL destruction contract

**Priority**: P2 / low
**Status**: resolved
**Dependencies**: LW-015, LW-024, LW-031

### Problem

`RL` is a polymorphic base with virtual runtime methods but has a non-virtual
destructor. No current deletion through `RL*` was found, so this is not an
active lifetime failure, but the documented extension boundary permits a future
owner to destroy a derived real or Sim2Sim object through the base and skip its
thread, serial, or simulation cleanup.

### Evidence

- `src/rl_sar/library/core/rl_sdk/rl_sdk.hpp:298-302`
- `src/rl_sar/library/core/rl_sdk/rl_sdk.hpp:320-365`
- `src/rl_sar/include/rl_real_LW.hpp`
- `src/rl_sar/include/rl_sim_LW.hpp:38-47`

### Intended Scope

- Make the ownership/destruction contract explicit and safe, normally with a
  virtual noexcept destructor unless a stronger non-polymorphic ownership
  restriction is selected and enforced.
- Verify derived destructors still perform their existing bounded shutdown in
  the required order.
- Add a focused compile/runtime regression test for the chosen contract.

### Acceptance Criteria

- It is impossible to invoke undefined or incomplete cleanup by deleting an LW
  runtime object through its supported base ownership type.
- The change does not alter runtime behavior, object ownership, or extension
  registration beyond the approved destruction contract.
- Strict warning, lifecycle, and full CTest suites remain green.

### Resolution

- **Resolved**: 2026-08-15T12:51:07+08:00
- **Commit**: 本提交
- **Approved Scope**: 将多态 `RL` 基类的公开析构函数改为
  `virtual noexcept = default`，真实和 Sim2Sim 的 `RL_Real` 析构函数显式声明为
  `noexcept override`，不改变析构函数体、对象创建方式、所有权或扩展注册。新增
  独立编译/运行时测试，静态验证虚析构和 `noexcept` 契约，并通过
  `std::unique_ptr<RL>` 销毁轻量派生探针，确认派生析构函数体及派生成员清理均
  完整执行。既有真实节点关门、worker 停止、最终失能顺序和 Sim2Sim worker、
  物理生命周期停止顺序保持不变。
- **Changed Files**: `src/rl_sar/CMakeLists.txt`、
  `src/rl_sar/include/rl_real_LW.hpp`、
  `src/rl_sar/include/rl_sim_LW.hpp`、
  `src/rl_sar/library/core/rl_sdk/rl_sdk.hpp`、
  `src/rl_sar/src/rl_real_LW.cpp`、
  `src/rl_sar/src/rl_sim_LW.cpp`、
  `src/rl_sar/test/test_lw_rl_destruction.cpp`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **Verification**: `test_lw_rl_destruction`、`rl_real_LW` 和 `rl_sim_LW`
  定向构建成功；`lw_rl_destruction`、`lw_signal_shutdown`、
  `lw_sim_lifecycle_integration`、`lw_mujoco_lifecycle` 和
  `lw_real_startup_disable_integration` 各连续 20 轮通过，验证基类所有权虚派发、
  派生成员清理，以及真实/Sim2Sim 原有有界停止顺序。完整 Debug 构建成功并通过
  44/44 CTest；全新 `scripts/validate_lw_strict_build.sh` 在
  `-Wall -Wextra -Wpedantic -Werror` 下完成全部目标并通过 44/44 CTest；定向
  `cppcheck` 和 `git diff --check` 通过。未启动 ROS 节点、MuJoCo GUI、真机、
  串口或电机，用户未跟踪技能目录保持未修改。
- **Remaining Follow-ups**: none

---

## [LW-041] Opt-in Sim2Sim plot publishing

**Priority**: P2 / low
**Status**: resolved
**Dependencies**: LW-014, LW-021, LW-031

### Problem

The Sim2Sim header unconditionally defines `PLOT`, so every maintained
`rl_sim_LW` build creates a `/joint_states` publisher and a 2 ms wall timer.
This makes high-rate diagnostic work part of the default simulation runtime and
can distort parity, timing, and profiling runs even when no consumer needs the
topic.

### Evidence

- `src/rl_sar/include/rl_sim_LW.hpp:4-10`
- `src/rl_sar/include/rl_sim_LW.hpp:119-124`
- `src/rl_sar/src/rl_sim_LW.cpp:223-232`
- `src/rl_sar/src/rl_sim_LW.cpp:278-432`

### Intended Scope

- Replace the header-level always-on macro with an explicit Sim2Sim runtime or
  build option that defaults off.
- Avoid creating the publisher, timer, message buffers, or callbacks when the
  option is disabled.
- Keep operator-status reporting separate from high-rate plot telemetry.
- Add configuration and lifecycle tests for both disabled and enabled modes.

### Acceptance Criteria

- Default Sim2Sim startup performs no 500 Hz plot publication work.
- An explicit documented opt-in restores the existing plot topic and payload.
- Real-robot debug publishing and shared control/safety parity remain unchanged.

### Resolution

- **Resolved**: 2026-08-15T13:17:13+08:00
- **Commit**: 本提交
- **Approved Scope**: 删除 Sim2Sim 头文件中始终启用的 `PLOT` 宏和未使用的
  matplotlib/plot-loop 声明，新增默认关闭的 `--enable-plot` 运行时开关；
  启用时默认 100 Hz，并允许通过 `--plot-rate-hz <integer>` 选择 1–200 Hz。
  频率参数必须与开关同时使用，缺失、非整数、越界或重复冲突值在启动时
  明确拒绝，参数顺序不受限制。默认模式不分配绘图快照缓冲、不创建 publisher
  或 timer，也不安装控制周期快照回调；显式启用后恢复既有
  `/LW_joint_states` topic 和 payload，启动日志输出有效 topic/频率。100 ms
  operator-status timer 保持独立，真机 Plot/debug 路径及共享控制、安全逻辑
  不变。
- **Changed Files**: `README.md`、`README_CN.md`、
  `docs/LW_BUILD_DEPLOYMENT_CN.md`、`docs/LW_QUICK_START_CN.md`、
  `src/rl_sar/CMakeLists.txt`、
  `src/rl_sar/include/rl_sim_LW.hpp`、
  `src/rl_sar/library/core/simulation/lw_sim_plot_config.hpp`、
  `src/rl_sar/src/rl_sim_LW.cpp`、
  `src/rl_sar/test/test_lw_sim_lifecycle_integration.py`、
  `src/rl_sar/test/test_lw_sim_plot_config.cpp`、
  `.learnings/LEARNINGS.md`、`.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **Verification**: `test_lw_sim_plot_config` 与 `lw_sim_lifecycle_integration` 各连续
  20 轮通过，覆盖默认关闭、显式启用、1–200 Hz 边界、无效或冲突参数拒绝、
  绘图资源按需创建和 operator-status 独立性。完整 Debug 构建成功并通过
  45/45 CTest；全新 `scripts/validate_lw_strict_build.sh` 在
  `-Wall -Wextra -Wpedantic -Werror` 下完成全部目标并通过 45/45
  CTest；定向 `cppcheck`、Python lifecycle integration 和 `git diff --check`
  通过。README、完整部署说明和快速开始文档中的 Plot 参数名称、默认值、
  范围和约束已交叉核对。未启动 MuJoCo GUI、ROS 节点、真机、串口或电机，
  用户未跟踪技能目录保持未修改。
- **Remaining Follow-ups**: none

---

## [LW-042] Nonblocking, source-fresh real debug telemetry

**Priority**: P2 / medium
**Status**: resolved
**Dependencies**: LW-007, LW-011, LW-014, LW-038

### Problem

The opt-in real-robot debug publisher is absent from the default control path,
but its enabled path does not fully satisfy the nonblocking contract recorded
for LW-014. The 200 Hz control callback calls `publishSnapshot()`, which uses
the blocking `LWSnapshotBuffer::publish()` while the ROS timer reads through
the same mutex. A preempted reader can therefore delay the control thread.

The publisher timer is also fixed at 4 ms (250 Hz), faster than the 5 ms
(200 Hz) source control cycle. The buffer carries no source sequence or capture
time, and `publishOnce()` can read the same snapshot repeatedly while assigning
a new publication timestamp each time. Consumers can consequently see repeated
control data as apparently fresh 250 Hz samples, with avoidable ROS work during
real-hardware debugging. The launch interface exposes only an enable switch,
and the existing test drives `publishOnce()` manually with a one-hour timer, so
it does not cover contention, duplicate suppression, or the production rate.

### Evidence

- `policy/LW/base.yaml:1-3`
- `src/rl_sar/src/rl_real_LW.cpp:229-247`
- `src/rl_sar/src/rl_real_LW.cpp:541-593`
- `src/rl_sar/library/core/debug/lw_debug_publisher.cpp:137-160`
- `src/rl_sar/library/core/safety/lw_runtime_sync.hpp:14-43`
- `src/rl_sar/launch/rl_real_LW.launch.py:39-48`
- `src/rl_sar/test/test_lw_debug_publisher.cpp:35-44`
- `src/rl_sar/test/test_lw_debug_publisher.cpp:158-187`

### Intended Scope

- Add a documented integer ROS/launch parameter for the real debug publication
  rate, defaulting to 50 Hz and accepting 1 through 200 Hz. Keep the existing
  enable switch default off and report the effective topic/rate when enabled.
- Make the control-side snapshot handoff strictly nonblocking; contention must
  drop a debug snapshot rather than wait on a timer-held mutex.
- Attach a monotonic source sequence (and capture-time metadata if needed) to
  each accepted snapshot, and publish only a source frame newer than the last
  published frame so an old sample is not restamped as new telemetry.
- Preserve the existing `/LW_joint_states` topic, payload mapping, coherent
  snapshot boundary, depth-one real-time publisher, and per-message timestamp.
- Add focused configuration, contention, freshness, lifecycle, and launch tests
  without starting a real node or accessing ROS hardware, serial ports, or
  motors. Synchronize all operator-facing parameter documentation.

### Acceptance Criteria

- With real debug disabled, no debug publisher, timer, snapshot handoff, or
  control-cycle copy exists, regardless of the configured inactive rate.
- With real debug enabled, the 200 Hz control callback never waits for the debug
  consumer; lock contention is observable only as a dropped debug sample.
- The effective debug rate is explicitly configurable from 1 through 200 Hz,
  defaults to 50 Hz, and invalid values fail startup before worker loops begin.
- A timer callback publishes each accepted source sequence at most once; it does
  not repeatedly restamp an unchanged control snapshot as fresh data.
- The existing topic and payload remain compatible, and shared control, safety,
  startup-disable, operator-status, and Sim2Sim behavior remain unchanged.

### Resolution

- **Resolved**: 2026-08-15T14:18:41+08:00
- **Commit**: 本提交
- **Approved Scope**: 为真机 launch/ROS 节点新增默认 50 Hz、只接受 1–200
  整数的 `debug_publish_rate_hz` 参数，并在任何 worker 启动前无条件校验；保持
  `enable_debug_publisher` 默认关闭，关闭时不创建 publisher、timer 或快照交接，
  也不在控制周期复制调试数据。启用时将控制侧快照交接改为 `tryPublish()`，锁
  争用立即丢弃调试帧而不等待；成功交接的帧携带单调源序号，timer 只发布比上次
  已发序号更新的最新帧，避免给未变化数据重复刷新时间戳。保留既有
  `/LW_joint_states`、26 字段 payload、depth-one realtime publisher、完整快照
  边界和发布时钟时间戳；Sim2Sim、控制、安全、启动失能和 operator-status
  行为不变。四份操作文档已同步参数、边界和丢帧语义。
- **Changed Files**: `README.md`、`README_CN.md`、
  `docs/LW_BUILD_DEPLOYMENT_CN.md`、`docs/LW_QUICK_START_CN.md`、
  `src/rl_sar/include/rl_real_LW.hpp`、
  `src/rl_sar/launch/rl_real_LW.launch.py`、
  `src/rl_sar/library/core/debug/lw_debug_publisher.cpp`、
  `src/rl_sar/library/core/debug/lw_debug_publisher.hpp`、
  `src/rl_sar/src/rl_real_LW.cpp`、
  `src/rl_sar/test/test_lw_debug_publisher.cpp`、
  `src/rl_sar/test/test_lw_real_startup_disable_integration.py`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **Verification**: `lw_debug_publisher`、`lw_runtime_sync` 和
  `lw_real_startup_disable_integration` 各连续 20 轮通过，覆盖默认关闭、
  1/50/200 Hz 有效边界、负数/0/201 无效边界、非阻塞 `tryPublish` 接线、同一
  源帧至多发布一次、新源帧继续发布、payload 和时间戳兼容，以及参数校验早于
  worker 启动。完整 Debug 构建成功并通过 45/45 CTest；全新
  `scripts/validate_lw_strict_build.sh` 在 `-Wall -Wextra -Wpedantic -Werror`
  下完成全部目标并通过 45/45 CTest。定向 `cppcheck`、Python 语法、launch
  `--show-args`、四份文档参数交叉核对和 `git diff --check` 通过。未启动真机
  节点、AHRS、串口、电机或 MuJoCo GUI；用户未跟踪技能目录
  保持未修改。
- **Remaining Follow-ups**: none

---

## [LW-043] Retire the Sim2Sim actuator-model runtime while preserving offline training

**Priority**: P2 / medium
**Status**: resolved
**Dependencies**: LW-020, LW-021, LW-036

### Problem

The optional `rl_sim_LW --use_actuator_net` path replaces selected MuJoCo
PD-plus-feedforward torques with TorchScript actuator-network output. This
creates a second Sim2Sim actuation path and retains a C++ LibTorch build,
download, validation, and runtime dependency solely for that optional feature.
The project still needs the independent Python training and evaluation workflow
and its two tracked `.pt` artifacts, but those offline assets do not require the
simulator to load or execute TorchScript.

### Evidence

- `src/rl_sar/src/rl_sim_LW.cpp`
- `src/rl_sar/include/rl_sim_LW.hpp`
- `src/rl_sar/library/core/simulation/lw_actuator_models.{hpp,cpp}`
- `src/rl_sar/library/core/inference_runtime/inference_runtime.{hpp,cpp}`
- `src/rl_sar/CMakeLists.txt`
- `scripts/download_inference_runtime.sh`
- `src/rl_sar/scripts/actuator_net.py`
- `policy/LW/robot_lab/motors/{leg,foot}_actuator_net.pt`

### Intended Scope

- Make Sim2Sim use the existing MuJoCo PD-plus-feedforward torque path for every
  joint and reject the removed `--use_actuator_net` option explicitly during
  startup.
- Preserve final finite-value and effort-limit validation before any MuJoCo
  control-array mutation by moving it into an actuator-model-independent module.
- Remove the C++ LibTorch backend and its build/download/validation surface;
  keep the ONNX Runtime policy backend.
- Preserve the Python actuator-model training/evaluation script, its install
  rule, Python `torch` usage, and both tracked model artifacts unchanged.
- Update maintained tests and current operator documentation without rewriting
  historical resolved issue records.

### Acceptance Criteria

- No maintained Sim2Sim code can load an actuator model or replace PD-plus-
  feedforward torque, and the old option fails before ROS, MuJoCo, or workers
  start.
- Invalid final candidate torques still trigger
  `SimulationActuatorCommandInvalid` before any `mj_data->ctrl` write.
- The maintained C++ build and runtime-management tools are ONNX-only and do not
  require LibTorch.
- The training script, its install rule, and both `.pt` files remain byte-for-
  byte unchanged and usable by the Python training/evaluation workflow.
- A clean build, full CTest suite, and strict build complete successfully.

### Resolution

- **Resolved**: 2026-08-16T12:35:00+08:00
- **Commit**: 本提交
- **Approved Scope**: 删除 `rl_sim_LW --use_actuator_net` 对 MuJoCo 底层力矩的
  运行时接管，所有关节统一使用 PD+前馈；旧参数在任何 ROS、MuJoCo
  或 worker 启动前明确拒绝。将最终力矩有限值/限幅/事务性写入校验迁入
  与执行器模型无关的 `lw_sim_torque_validation`，并删除已无消费者的共享
  `before_command_delivery` hook。C++ 推理、CMake、下载、来源清单和
  验证工具收敛为 ONNX-only；Python 训练/评估脚本、安装规则以及两个
  `.pt` 资产保持不变。当前 764 MB LibTorch 目录未删除，已可恢复地迁至
  `/home/lfr/rl_sar-runtime-backups/20260816-actuator-runtime-retirement/libtorch`，
  不覆盖既有备份。
- **Changed Files**: `.gitignore`、`README.md`、`build.sh`、
  `docs/{LW_BUILD_DEPLOYMENT_CN.md,LW_QUICK_START_CN.md}`、
  `scripts/{download_inference_runtime.sh,inference_runtime_archives.json,manage_inference_runtime.py,validate_inference_runtime.sh}`、
  `src/rl_sar/CMakeLists.txt`、`src/rl_sar/include/rl_sim_LW.hpp`、
  `src/rl_sar/library/core/inference_runtime/inference_runtime.{hpp,cpp}`、
  `src/rl_sar/library/core/safety/lw_runtime_core.hpp`、
  `src/rl_sar/library/core/simulation/{lw_actuator_models.hpp,lw_actuator_models.cpp,lw_sim_plot_config.hpp,lw_sim_torque_validation.hpp,lw_sim_torque_validation.cpp}`、
  `src/rl_sar/src/{rl_sim_LW.cpp,rl_real_LW.cpp,lw_config_profiler.cpp}`、
  `src/rl_sar/test/{test_build_workflow.py,test_inference_runtime.cpp,test_inference_runtime_architecture.py,test_inference_runtime_download_integrity.py,test_lw_actuator_models.cpp,test_lw_runtime_parity.cpp,test_lw_sim_lifecycle_integration.py,test_lw_sim_plot_config.cpp,test_lw_sim_torque_validation.cpp}`、
  `.learnings/{LEARNINGS.md,LW_REAL_DEPLOYMENT_ISSUES.md}`。
- **Verification**: 修改前基线 45/45 CTest 通过，其中旧 LibTorch 测试已对
  当前两个 TorchScript 模型完成加载及 6 维输入/单值输出契约验证。
  定向 7 项 CTest 通过；旧 `--use_actuator_net` 实际运行在 GUI/ROS 启动前
  以非零状态和明确诊断退出。执行全量 `./build.sh --clean` 后 `./build.sh`
  从零构建 6 个包成功，干净构建 45/45 CTest 通过；
  `scripts/validate_lw_strict_build.sh` 在 `-Wall -Wextra -Wpedantic -Werror`
  下构建全部维护目标并通过 45/45 CTest。`ldd` 确认
  `rl_sim_LW`、`rl_real_LW` 和 `lw_config_profiler` 均不依赖
  `libtorch`/`libc10`。Bash/Python 语法、ONNX 架构/下载完整性、通用力矩
  校验、源码/文档交叉检查和 `git diff --check` 通过。训练脚本和两个
  模型 SHA-256 与基线完全一致，安装后脚本存在；当前机器的现有
  Python 环境均未安装 `torch`，因此未新增依赖也未重复 Python 前向推理。
  未启动真机节点、AHRS、串口、电机或 MuJoCo GUI；用户未跟踪技能目录保持未修改。
- **Remaining Follow-ups**: none

---

## [LW-044] Complete and failure-visible FDILink frame ingestion

**Priority**: P1 / high
**Status**: resolved
**Dependencies**: LW-017, LW-032

### Problem

The production FDILink node passes an uninitialized `serial_timeout_` to the
serial library and reads `frist_sn_` before initialization. Its receive loop
records byte counts but continues parsing after short reads, so a partial frame
can be combined with stale or uninitialized storage before CRC and publication.
Serial-open failures also terminate with `exit(0)`, hiding a required runtime
failure from launch and service supervision.

### Evidence

- `src/fdilink_ahrs_ROS2/include/ahrs_driver.h:63-81`
- `src/fdilink_ahrs_ROS2/src/ahrs_driver.cpp:74-101`
- `src/fdilink_ahrs_ROS2/src/ahrs_driver.cpp:110-205`
- `src/fdilink_ahrs_ROS2/src/ahrs_driver.cpp:302-440`
- `src/serial_ros2/src/impl/unix.cc:533-606`

### Intended Scope

- Replace piecemeal union-backed parsing with a fixed-capacity, incrementally
  fed protocol parser that emits only complete frames with an approved
  type/length, valid header CRC8, valid payload CRC16, and the required end
  marker.
- Make arbitrary read fragmentation, leading noise, malformed frames, timeout
  reset, and deterministic resynchronization testable without ROS hardware.
- Initialize all sequence and sensor-cache state, update sequence accounting
  only after complete-frame validation, and never publish an IMU orientation or
  magnetic sample from a cache that has not received its corresponding frame.
- Use a deterministic bounded serial-read timeout and ensure shutdown cannot be
  held by an indeterminate wait.
- Return failure for serial-open, disconnect, and receive exceptions while
  preserving a successful status for normal ROS shutdown.
- Preserve production topics, remaps, coordinate transforms, the real-runtime
  IMU/AHRS guard, motor handling, policies, and the serial library.

### Acceptance Criteria

- No uninitialized scalar or sensor cache influences serial setup, sequence
  accounting, CRC validation, or a published message.
- A partial header or body, including bytes split at every frame boundary, is
  never published before completion; a timeout discards the incomplete frame
  rather than joining it to later bytes.
- Invalid type/length, CRC8, CRC16, and end-marker frames are rejected and a
  later valid frame is recovered without restarting the node.
- An IMU packet is not published before the first complete AHRS packet, and a
  magnetic sample is not published before the first complete IMU packet.
- An unavailable serial port exits promptly with a nonzero status; normal ROS
  shutdown closes the port and exits successfully within the bounded read
  timeout.
- The `fdilink_ahrs` package, full LW build/test suite, strict maintained build,
  deployment-related checks, syntax checks, and `git diff --check` pass without
  accessing a real serial device, IMU, or motor.

### Resolution

- **Resolved**: 2026-08-18T12:46:04+08:00
- **Commit**: 本提交
- **Approved Scope**: 将 FDILink 串口接收改为固定容量、增量式完整帧解析；
  只有类型、长度、CRC8、CRC16 和结束标记全部有效时才更新序号并发布。
  初始化串口、序号及传感器缓存状态，丢弃超时或短读留下的不完整帧；在
  AHRS/IMU 对应缓存首次有效前禁止发布依赖该缓存的消息。串口读取使用
  20 ms 有界超时，串口打开或接收异常以非零状态退出，正常 ROS 关闭在
  有界等待后成功退出；保持既有话题、坐标变换和下游真机安全守卫不变。
- **Changed Files**: `src/fdilink_ahrs_ROS2/CMakeLists.txt`、
  `src/fdilink_ahrs_ROS2/package.xml`、
  `src/fdilink_ahrs_ROS2/include/{ahrs_driver.h,fdilink_frame_parser.h}`、
  `src/fdilink_ahrs_ROS2/src/{ahrs_driver.cpp,fdilink_frame_parser.cpp}`、
  `src/fdilink_ahrs_ROS2/test/{test_fdilink_frame_parser.cpp,test_fdilink_process_lifecycle.py}`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **Verification**: 隔离构建 `serial`、`fdilink_ahrs`、`rl_sar` 三个包成功；
  FDILink 的 6 个 GTest 用例覆盖全部支持帧型、每个截断边界、噪声与连续帧、
  非法类型/长度/CRC8/CRC16/结束标记、超时复位和后续恢复，2 个 Python
  进程用例验证不存在的串口及时非零退出，以及空 PTY 上 SIGTERM 在 1 秒内
  成功退出。两项 CTest 均通过。变更目标在
  `-Wall -Wextra -Wpedantic -Werror` 下构建并再次通过两项 CTest；完整 LW
  CTest 45/45 通过，包含部署包、清单、运行时依赖、策略资产、构建工作流
  和启动禁用集成检查。`scripts/validate_lw_strict_build.sh` 构建全部维护目标
  并通过 45/45 CTest；`git diff --check` 通过。所有验证均未访问真机串口、
  AHRS、IMU 或电机；用户已有快速启动文档、错误记录和未跟踪技能目录保持不变。
- **Remaining Follow-ups**: none

---

## [LW-045] Explicit and architecture-safe FDILink payload decoding

**Priority**: P1 / high
**Status**: resolved
**Dependencies**: LW-044

### Problem

After LW-044 validates a complete frame, the driver copies its bytes into the
`read_tmp` member of a packed union and reads the inactive `frame` member. The
payload begins at byte offset seven, so its `float`, `double`, and `int64_t`
members are also unaligned. This relies on compiler union-punning extensions,
packed-member behavior, native IEEE-754 layout, and host byte order at the
production sensor boundary.

### Evidence

- `src/fdilink_ahrs_ROS2/include/fdilink_data_struct.h:6-186`
- `src/fdilink_ahrs_ROS2/src/ahrs_driver.cpp:18-40`
- `src/fdilink_ahrs_ROS2/src/ahrs_driver.cpp:213-385`

### Intended Scope

- Decode each supported FDILink payload from explicit little-endian byte
  offsets into ordinary aligned value types, using integer assembly and
  `memcpy` for IEEE-754 bit transfer rather than casts or packed objects.
- Make the driver cache decoded IMU, AHRS, INSGPS, and geodetic-position values
  and use the already validated frame serial number for sequence accounting.
- Remove the now-unused packed structs, unions, and layout-size assertions.
- Preserve frame validation, topics, message fields, units, coordinate
  transforms, publication ordering, timeout behavior, and downstream guards.
- Keep payload range and non-finite-value policy outside this issue.

### Acceptance Criteria

- No production FDILink payload is interpreted through a packed object,
  inactive union member, `reinterpret_cast`, or host-native multi-byte load.
- Fixed little-endian byte vectors decode every field of all four supported
  payload types, including signed values, `float`, `double`, and `int64_t`.
- Wrong type, wrong payload length, and truncated frames are rejected without
  mutating the caller's decoded output.
- Existing parser, process-lifecycle, ROS topic mapping, coordinate transforms,
  and sequence behavior remain unchanged.
- Changed targets pass warning-as-error and undefined/alignment sanitizer
  builds; FDILink tests, the full LW suite, deployment-related checks, syntax
  checks, and `git diff --check` pass without real hardware.

### Resolution

- **Resolved**: 2026-08-18T13:44:26+08:00
- **Commit**: 本提交
- **Approved Scope**: 将四类已验证 FDILink 帧按明确的小端字段偏移解码到
  普通对齐值对象；整数字节先组合为无符号数，IEEE-754 浮点通过 `memcpy`
  转移位模式，有符号 64 位值按协议补码转换。驱动只在完整解码成功后更新缓存，
  并直接使用已验证帧序号。删除 packed 结构、union 覆盖读取和布局尺寸断言；
  保持 CRC/帧解析、话题、单位、坐标变换、发布顺序和下游安全守卫不变，未加入
  数值范围或 NaN/Inf 策略。
- **Changed Files**: `src/fdilink_ahrs_ROS2/CMakeLists.txt`、
  `src/fdilink_ahrs_ROS2/include/{ahrs_driver.h,fdilink_payload_decoder.h}`、
  `src/fdilink_ahrs_ROS2/include/fdilink_data_struct.h`（删除）、
  `src/fdilink_ahrs_ROS2/src/{ahrs_driver.cpp,fdilink_payload_decoder.cpp}`、
  `src/fdilink_ahrs_ROS2/test/test_fdilink_payload_decoder.cpp`、
  `.learnings/LW_REAL_DEPLOYMENT_ISSUES.md`。
- **Verification**: 固定小端字节向量逐字段验证 IMU、AHRS、INSGPS 和
  Geodetic Position 的全部 `float`、`double` 与 `int64_t` 字段，并验证错误
  类型、错误长度和截断帧不修改输出。`serial`、`fdilink_ahrs`、`rl_sar`
  三包隔离构建成功；解析器、解码器和进程生命周期 3/3 CTest 通过，完整 LW
  CTest 45/45 通过。变更协议库、驱动和两项 C++ 测试在
  `-Wall -Wextra -Wpedantic -Werror -fsanitize=undefined,alignment`
  下构建，3/3 CTest 通过且无 sanitizer 报告；
  `scripts/validate_lw_strict_build.sh` 构建全部维护目标并通过 45/45 CTest。
  源码检查确认 FDILink 生产路径不再包含 packed/union 覆盖或
  `reinterpret_cast` 解码，`git diff --check` 通过。未访问真机串口、AHRS、
  IMU 或电机；用户未跟踪技能目录保持未修改。
- **Remaining Follow-ups**: none

---

## Resolution Template

When an issue is completed, update only that issue:

```markdown
**Status**: resolved

### Resolution
- Resolved: YYYY-MM-DDTHH:MM:SS+08:00
- Commit: <commit>
- Approved Scope: <what the user approved>
- Changed Files: <paths>
- Verification: <commands/tests and results>
- Remaining Follow-ups: <IDs only; do not modify them>
```
