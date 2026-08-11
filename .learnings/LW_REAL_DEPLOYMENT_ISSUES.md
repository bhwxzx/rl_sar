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

## Ordered Summary

| Order | ID | Priority | Status | Summary |
|---:|---|---|---|---|
| 1 | LW-001 | P0 / critical | resolved | Make control-loop shutdown and exception handling fail-safe |
| 2 | LW-002 | P0 / critical | resolved | Require valid, fresh IMU and bilateral motor feedback before commanding |
| 3 | LW-003 | P0 / critical | resolved | Repair serial receive parsing and complete-write handling |
| 4 | LW-004 | P0 / critical | resolved | Remove invalid FSM transition targets and validate all transitions |
| 5 | LW-005 | P0 / critical | resolved | Enforce finite commands and state-aware attitude protection |
| 6 | LW-006 | P0 / critical | resolved | Latch joystick disconnects, clear commands, and validate indices |
| 7 | LW-007 | P1 / high | resolved | Remove cross-thread data races with coherent snapshots |
| 8 | LW-008 | P1 / high | resolved | Replace split policy queues with one coherent output frame |
| 9 | LW-009 | P1 / high | resolved | Use the configured 60 Hz wheel-to-leg reference rate |
| 10 | LW-010 | P1 / high | resolved | Make deployed binary, configuration, and models reproducible |
| 11 | LW-016 | P1 / high | resolved | Audit every safety trigger for proportional and recoverable behavior |
| 12 | LW-013 | P1 / high | resolved | Validate YAML, mappings, observation sizes, and model outputs |
| 13 | LW-011 | P1 / high | resolved | Make the control loop suitable for deterministic real-time execution |
| 14 | LW-017 | P1 / high | resolved | Bundle and verify the LW IMU/serial runtime dependencies |
| 15 | LW-019 | P1 / high | resolved | Enable a real-robot terminal keyboard recovery channel |
| 16 | LW-020 | P1 / high | resolved | Make the Jetson production inference bootstrap architecture-safe and ONNX-only |
| 17 | LW-021 | P1 / high | resolved | Make Sim2Sim and real deployment share one testable control and safety core |
| 18 | LW-022 | P1 / high | resolved | Measure suspended real-runtime behavior and generate review-only configuration candidates |
| 19 | LW-018 | P2 / medium | resolved | Unify the build entry point and Jetson detection |
| 20 | LW-012 | P2 / medium | resolved | Harden motion loading and correct its time convention |
| 21 | LW-015 | P2 / medium | resolved | Remove non-LW robot implementations while preserving future extension points |
| 22 | LW-014 | P2 / low | resolved | Isolate and correct production debug/plot publishing |

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
  infrastructure remains documented in `docs/LW_REPOSITORY_SCOPE.md`.
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
- **Commit**: 待本次提交
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
