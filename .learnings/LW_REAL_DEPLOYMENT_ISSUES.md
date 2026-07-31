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
| 9 | LW-009 | P1 / high | pending | Use the configured 60 Hz wheel-to-leg reference rate |
| 10 | LW-010 | P1 / high | pending | Make deployed binary, configuration, and models reproducible |
| 11 | LW-011 | P2 / medium | pending | Make the control loop suitable for deterministic real-time execution |
| 12 | LW-012 | P2 / medium | pending | Harden motion loading and correct its time convention |
| 13 | LW-013 | P2 / medium | pending | Validate YAML, mappings, observation sizes, and model outputs |
| 14 | LW-014 | P2 / low | pending | Isolate and correct production debug/plot publishing |

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
**Status**: pending
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

---

## [LW-010] Reproducible deployment artifacts

**Priority**: P1 / high
**Status**: pending
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

---

## [LW-011] Deterministic control-loop timing

**Priority**: P2 / medium
**Status**: pending
**Dependencies**: LW-001, LW-007

### Problem

The project defaults to a Debug build, loop timing truncates execution duration to integer milliseconds and uses relative sleeps, callbacks run without real-time scheduling, and progress output performs terminal IO and flushes inside the 200 Hz control path. Production plotting is enabled by default.

### Evidence

- `src/rl_sar/CMakeLists.txt:34-40`
- `src/rl_sar/library/core/loop/loop.hpp:73-101`
- `src/rl_sar/library/core/logger/logger.hpp:47-92`
- `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp:635-690`
- `src/rl_sar/include/rl_real_LW.hpp:4-7`

### Intended Scope

- Use an explicit Release/RelWithDebInfo production profile.
- Schedule against absolute deadlines with high-resolution durations.
- Define overrun detection and a safe overrun policy.
- Move terminal IO and progress formatting out of the control thread.
- Decide and document CPU affinity and real-time priority requirements.

### Acceptance Criteria

- Production builds use the approved optimized profile.
- A timing test reports control and inference jitter, maximum latency, and missed deadlines.
- Blocking terminal or ROS debug IO cannot delay motor command generation.

---

## [LW-012] Motion-loader robustness and time convention

**Priority**: P2 / medium
**Status**: pending
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

---

## [LW-013] Configuration and dimension validation

**Priority**: P2 / medium
**Status**: pending
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

---

## [LW-014] Debug and plot publishing isolation

**Priority**: P2 / low
**Status**: pending
**Dependencies**: LW-007, LW-011

### Problem

Plot publishing is enabled at compile time for production, reads shared control and robot data without a coherent snapshot, and initializes the message timestamp only once rather than on each publication.

### Evidence

- `src/rl_sar/include/rl_real_LW.hpp:4-7`
- `src/rl_sar/src/rl_real_LW.cpp:62-110`
- `src/rl_sar/src/rl_real_LW.cpp:127-203`

### Intended Scope

- Make plotting opt-in through a build or runtime setting.
- Publish from a coherent nonblocking snapshot.
- Refresh timestamps on every message.

### Acceptance Criteria

- Production control can run with plotting completely disabled.
- Enabling plotting does not introduce control-path races or blocking.
- Published messages carry current timestamps and internally consistent samples.

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
