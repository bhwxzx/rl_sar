#include "lw_runtime_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{
constexpr size_t kNumDofs = 10;

void require(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

class ReplayState : public RLFSMState
{
public:
    ReplayState(RL& rl, std::string name, float offset)
        : RLFSMState(rl, std::move(name)), offset(offset)
    {
    }

    void Enter() override {}

    void Run() override
    {
        observed_keyboard = rl.control.current_keyboard;
        observed_gamepad = rl.control.current_gamepad;
        observed_velocity = {rl.control.x, rl.control.y, rl.control.yaw};
        for (size_t index = 0; index < kNumDofs; ++index)
        {
            fsm_command->motor_command.q[index] =
                fsm_state->motor_state.q[index] + offset;
            fsm_command->motor_command.dq[index] = 0.0f;
            fsm_command->motor_command.tau[index] = 0.1f * offset;
            fsm_command->motor_command.kp[index] = 20.0f;
            fsm_command->motor_command.kd[index] = 0.5f;
        }
    }

    void Exit() override {}

    Input::Keyboard observed_keyboard = Input::Keyboard::None;
    Input::Gamepad observed_gamepad = Input::Gamepad::None;
    std::vector<float> observed_velocity{0.0f, 0.0f, 0.0f};
    float offset;
};

class ReplayRL : public RL
{
public:
    ReplayRL()
    {
        params.config_node["num_of_dofs"] = static_cast<int>(kNumDofs);
        LWBaseRuntimeConfiguration runtime_configuration;
        runtime_configuration.num_dofs = kNumDofs;
        runtime_configuration.dt = 0.005f;
        runtime_configuration.decimation = 4;
        runtime_configuration.joint_mapping =
            {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
        runtime_configuration.wheel_indices = {2, 3};
        runtime_configuration.wheel_mask.assign(kNumDofs, 0);
        runtime_configuration.wheel_mask[2] = 1;
        runtime_configuration.wheel_mask[3] = 1;
        runtime_configuration.rl_kp.assign(kNumDofs, 20.0f);
        runtime_configuration.rl_kd.assign(kNumDofs, 0.5f);
        runtime_configuration.fixed_kp.assign(kNumDofs, 20.0f);
        runtime_configuration.fixed_kd.assign(kNumDofs, 0.5f);
        SetLWBaseRuntimeConfiguration(std::move(runtime_configuration));
        source.motor_state.resize(kNumDofs);
        robot_state.motor_state.resize(kNumDofs);
        robot_command.motor_command.resize(kNumDofs);
        for (size_t index = 0; index < kNumDofs; ++index)
        {
            source.motor_state.q[index] = 0.05f * static_cast<float>(index);
            source.motor_state.dq[index] = -0.01f * static_cast<float>(index);
            source.motor_state.tau_est[index] = 0.0f;
        }

        nominal = std::make_shared<ReplayState>(
            *this, "ReplayNominal", 0.25f);
        passive = std::make_shared<ReplayState>(
            *this, "RLFSMStatePassive", 0.0f);
        fsm.AddState(nominal);
        fsm.AddState(passive);
        fsm.SetInitialState("ReplayNominal");
    }

    std::vector<float> Forward() override
    {
        return {};
    }

    void GetState(RobotState<float>* state) override
    {
        *state = source;
    }

    void SetCommand(const RobotCommand<float>* command) override
    {
        delivered.push_back(*command);
    }

    RobotState<float> source;
    std::vector<RobotCommand<float>> delivered;
    std::shared_ptr<ReplayState> nominal;
    std::shared_ptr<ReplayState> passive;
};

struct AdapterTrace
{
    std::vector<LWSafetyAction> actions;
    std::vector<float> actuator_effort = std::vector<float>(kNumDofs, 1.0f);
    bool shutdown_requested = false;

    void execute(const LWSafetyDecision& decision)
    {
        actions.push_back(decision.action);
        if (decision.action == LWSafetyAction::HardDisable
            || decision.action == LWSafetyAction::HardDisableAndShutdown
            || decision.action == LWSafetyAction::AbortStartup)
        {
            std::fill(actuator_effort.begin(), actuator_effort.end(), 0.0f);
        }
        if (decision.action == LWSafetyAction::HardDisableAndShutdown
            || decision.action == LWSafetyAction::AbortStartup)
        {
            shutdown_requested = true;
        }
    }
};

struct Harness
{
    Harness()
    {
        core.bind(
            rl,
            [this](const LWSafetyDecision& decision, const std::string&)
            {
                adapter.execute(decision);
            });
        core.publishInitialPolicyInput();
    }

    void cycle(
        float x = 0.6f,
        Input::Keyboard keyboard = Input::Keyboard::None,
        Input::Gamepad gamepad = Input::Gamepad::None)
    {
        core.runControlCycle(
            LWControlCycleHooks{
                [this, x, gamepad]()
                {
                    rl.control.x = x;
                    rl.control.y = -0.2f;
                    rl.control.yaw = 0.3f;
                    rl.control.SetGamepad(gamepad);
                },
                [this, keyboard]()
                {
                    rl.control.SetKeyboard(keyboard);
                },
                []() { return true; },
                []() { return true; },
                {},
                {}});
    }

    ReplayRL rl;
    LWRuntimeCore core;
    AdapterTrace adapter;
};

void requireVectorEqual(
    const std::vector<float>& left,
    const std::vector<float>& right,
    const std::string& field)
{
    require(left.size() == right.size(), field + " size differs");
    for (size_t index = 0; index < left.size(); ++index)
    {
        require(
            std::fabs(left[index] - right[index]) <= 1.0e-6f,
            field + " differs at index " + std::to_string(index));
    }
}

void requireCommandEqual(
    const RobotCommand<float>& left,
    const RobotCommand<float>& right)
{
    requireVectorEqual(left.motor_command.q, right.motor_command.q, "q");
    requireVectorEqual(left.motor_command.dq, right.motor_command.dq, "dq");
    requireVectorEqual(left.motor_command.tau, right.motor_command.tau, "tau");
    requireVectorEqual(left.motor_command.kp, right.motor_command.kp, "kp");
    requireVectorEqual(left.motor_command.kd, right.motor_command.kd, "kd");
}

void requireObservationEqual(
    const Observations<float>& left,
    const Observations<float>& right)
{
    requireVectorEqual(left.lin_vel, right.lin_vel, "observation lin_vel");
    requireVectorEqual(left.ang_vel, right.ang_vel, "observation ang_vel");
    requireVectorEqual(left.gravity_vec, right.gravity_vec, "observation gravity");
    requireVectorEqual(left.commands, right.commands, "observation commands");
    requireVectorEqual(left.base_quat, right.base_quat, "observation quaternion");
    requireVectorEqual(left.dof_pos, right.dof_pos, "observation dof_pos");
    requireVectorEqual(left.dof_vel, right.dof_vel, "observation dof_vel");
    requireVectorEqual(left.actions, right.actions, "observation actions");
    requireVectorEqual(left.gait_phase, right.gait_phase, "observation gait phase");
}

void requireSafetyEqual(const Harness& real, const Harness& sim)
{
    const auto real_snapshot = real.core.safetySnapshot();
    const auto sim_snapshot = sim.core.safetySnapshot();
    require(
        real_snapshot.decision.latest_event
            == sim_snapshot.decision.latest_event,
        "latest safety event differs");
    require(
        real_snapshot.decision.highest_severity
            == sim_snapshot.decision.highest_severity,
        "highest safety severity differs");
    require(
        real_snapshot.decision.controlled_fallback_latched
            == sim_snapshot.decision.controlled_fallback_latched,
        "fallback latch differs");
    require(
        real_snapshot.terminal_latched == sim_snapshot.terminal_latched,
        "terminal latch differs");
    require(
        real_snapshot.shutdown_requested == sim_snapshot.shutdown_requested,
        "shutdown request differs");
    require(real.adapter.actions == sim.adapter.actions, "adapter actions differ");
    requireVectorEqual(
        real.adapter.actuator_effort,
        sim.adapter.actuator_effort,
        "adapter effort");
}

void prepareInferenceHarness(Harness& harness)
{
    constexpr const char* policy = "LW/robot_lab/leg_loco";
    harness.rl.SetPolicyRoot(POLICY_DIR);
    harness.rl.ReadYaml("LW", "base.yaml");
    harness.rl.PreloadModel(policy);
    harness.rl.PreloadLWPolicyContext(policy);
    harness.rl.ActivateLWPolicy(policy);
    harness.rl.control.gait_frequency = 1.5f;

    // Publish a nonzero command first so a subsequently latched fault exercises
    // the inference-frame boundary rather than relying on control-loop timing.
    harness.cycle(0.4f);
}

LWInferenceTraceSnapshot runInferenceObservationScenario(
    bool external_input_fault,
    LWSafetyEvent event = LWSafetyEvent::Count)
{
    Harness harness;
    prepareInferenceHarness(harness);
    if (event != LWSafetyEvent::Count)
    {
        harness.core.reportSafetyEvent(event);
    }
    harness.core.runInferenceCycle(external_input_fault);

    LWInferenceTraceSnapshot trace;
    require(
        harness.core.readInferenceTrace(trace),
        "inference observation scenario did not publish a trace");
    return trace;
}

void requireStationaryCommandObservation(
    const LWInferenceTraceSnapshot& trace,
    const std::string& scenario)
{
    requireVectorEqual(
        trace.observations.commands,
        {0.0f, 0.0f, 0.0f},
        scenario + " commands");
    requireVectorEqual(
        trace.observations.gait_phase,
        {0.0f, 0.0f},
        scenario + " gait phase");
}

void testEffectiveCommandKeepsGaitObservationCoherent()
{
    const LWInferenceTraceSnapshot nominal =
        runInferenceObservationScenario(false);
    requireVectorEqual(
        nominal.observations.commands,
        {0.4f, -0.2f, 0.3f},
        "nominal commands");
    constexpr float pi = 3.14159265358979323846f;
    constexpr float expected_phase_time = 0.005f * 4.0f * 1.5f;
    requireVectorEqual(
        nominal.observations.gait_phase,
        {std::sin(2.0f * pi * expected_phase_time),
         std::cos(2.0f * pi * expected_phase_time)},
        "nominal gait phase");

    requireStationaryCommandObservation(
        runInferenceObservationScenario(
            false, LWSafetyEvent::JoystickUnavailable),
        "joystick fault");
    requireStationaryCommandObservation(
        runInferenceObservationScenario(
            false, LWSafetyEvent::ControlTimingDegraded),
        "control timing degradation");
    requireStationaryCommandObservation(
        runInferenceObservationScenario(true),
        "external input fault");
}

void testTemporaryInhibitionPreservesPhaseClock()
{
    Harness harness;
    prepareInferenceHarness(harness);

    harness.core.runInferenceCycle(false);
    harness.cycle(0.4f);
    harness.core.runInferenceCycle(true);
    LWInferenceTraceSnapshot inhibited;
    require(
        harness.core.readInferenceTrace(inhibited),
        "temporary inhibition did not publish a trace");
    requireStationaryCommandObservation(inhibited, "temporary inhibition");

    harness.cycle(0.4f);
    harness.core.runInferenceCycle(false);
    LWInferenceTraceSnapshot recovered;
    require(
        harness.core.readInferenceTrace(recovered),
        "temporary inhibition recovery did not publish a trace");
    constexpr float pi = 3.14159265358979323846f;
    constexpr float expected_phase_time = 3.0f * 0.005f * 4.0f * 1.5f;
    requireVectorEqual(
        recovered.observations.gait_phase,
        {std::sin(2.0f * pi * expected_phase_time),
         std::cos(2.0f * pi * expected_phase_time)},
        "temporary inhibition recovery gait phase");
}

void testInputIsConsumedOnceAndHeldOutputRemainsUsable()
{
    Harness harness;
    prepareInferenceHarness(harness);

    harness.core.runInferenceCycle(false);
    LWInferenceTraceSnapshot first_trace;
    require(
        harness.core.readInferenceTrace(first_trace),
        "first input did not publish an inference trace");
    const auto first_output = harness.rl.LoadLWPolicyOutput();
    require(
        static_cast<bool>(first_output),
        "first input did not publish policy output");

    harness.core.runInferenceCycle(false);
    LWInferenceTraceSnapshot duplicate_trace;
    require(
        harness.core.readInferenceTrace(duplicate_trace),
        "duplicate input unexpectedly cleared the last trace");
    const auto duplicate_output = harness.rl.LoadLWPolicyOutput();
    require(
        duplicate_trace.frame == first_trace.frame
            && duplicate_trace.source_input_sequence
                == first_trace.source_input_sequence
            && duplicate_output
            && duplicate_output->sequence == first_output->sequence,
        "duplicate input advanced inference output");

    harness.rl.nominal->RLControlLW();
    const auto first_command = harness.rl.robot_command;
    std::fill(
        harness.rl.robot_command.motor_command.q.begin(),
        harness.rl.robot_command.motor_command.q.end(),
        -999.0f);
    harness.rl.nominal->RLControlLW();
    requireCommandEqual(first_command, harness.rl.robot_command);

    harness.cycle(0.4f);
    harness.core.runInferenceCycle(false);
    LWInferenceTraceSnapshot recovered_trace;
    require(
        harness.core.readInferenceTrace(recovered_trace)
            && recovered_trace.frame == first_trace.frame + 1
            && recovered_trace.source_input_sequence
                > first_trace.source_input_sequence
            && recovered_trace.source_state_time
                > first_trace.source_state_time,
        "fresh control input did not recover inference progress");
    require(
        !harness.core.safetySnapshot().decision.controlled_fallback_latched,
        "duplicate input incorrectly latched controlled fallback");
}

void testStalledControlInputTriggersS2Parity()
{
    Harness real;
    Harness sim;
    prepareInferenceHarness(real);
    prepareInferenceHarness(sim);
    real.core.runInferenceCycle(false);
    sim.core.runInferenceCycle(false);
    const auto real_output = real.rl.LoadLWPolicyOutput();
    const auto sim_output = sim.rl.LoadLWPolicyOutput();
    require(
        real_output && sim_output,
        "active inference consumer did not publish its first output");

    std::this_thread::sleep_for(std::chrono::milliseconds(70));
    real.core.runInferenceCycle(false);
    sim.core.runInferenceCycle(false);

    requireSafetyEqual(real, sim);
    const auto snapshot = real.core.safetySnapshot();
    require(
        snapshot.decision.latest_event
            == LWSafetyEvent::PolicyInputUnavailable
            && snapshot.decision.highest_severity
                == LWSafetySeverity::ControlledFallback
            && snapshot.decision.controlled_fallback_latched,
        "stalled control input did not latch S2 policy-input fallback");
    require(
        real.rl.LoadLWPolicyOutput()->sequence == real_output->sequence
            && sim.rl.LoadLWPolicyOutput()->sequence
                == sim_output->sequence,
        "stalled control producer allowed repeated output publication");
}

void testPolicyGenerationSwitchWaitsForMatchingInput()
{
    Harness harness;
    prepareInferenceHarness(harness);
    const std::uint64_t old_generation =
        harness.rl.LoadLWPolicyActivation()->generation;
    const std::uint64_t new_generation = harness.rl.ActivateLWPolicy(
        "LW/robot_lab/leg_loco");
    require(new_generation != old_generation, "policy generation did not change");

    harness.core.runInferenceCycle(false);
    LWInferenceTraceSnapshot trace;
    require(
        !harness.core.readInferenceTrace(trace)
            && !harness.rl.LoadLWPolicyOutput(),
        "new policy generation consumed an old-generation input");
    require(
        !harness.core.safetySnapshot().decision.controlled_fallback_latched,
        "expected generation transition latched fallback");

    harness.cycle(0.4f);
    harness.core.runInferenceCycle(false);
    require(
        harness.core.readInferenceTrace(trace)
            && trace.generation == new_generation
            && trace.frame == 1,
        "matching input did not start the new policy generation");
}

void testPolicyGenerationSwitchBindsTypedRuntimeConfiguration()
{
    Harness harness;
    constexpr const char* leg_policy = "LW/robot_lab/leg_loco";
    constexpr const char* wheel_policy = "LW/robot_lab/wheel_loco";
    harness.rl.SetPolicyRoot(POLICY_DIR);
    harness.rl.ReadYaml("LW", "base.yaml");
    harness.rl.PreloadModel(leg_policy);
    harness.rl.PreloadModel(wheel_policy);
    harness.rl.PreloadLWPolicyContext(leg_policy);
    harness.rl.PreloadLWPolicyContext(wheel_policy);

    const auto leg_definition =
        harness.rl.GetLWPolicyDefinition(leg_policy);
    const auto wheel_definition =
        harness.rl.GetLWPolicyDefinition(wheel_policy);
    require(
        leg_definition && wheel_definition,
        "typed policy definitions were not retained");
    require(
        leg_definition->runtime.observations.size()
            != wheel_definition->runtime.observations.size(),
        "test policies do not expose distinct typed configurations");

    const std::uint64_t leg_generation =
        harness.rl.ActivateLWPolicy(leg_policy);
    const auto leg_activation = harness.rl.LoadLWPolicyActivation();
    require(
        leg_activation
            && leg_activation->generation == leg_generation
            && leg_activation->definition == leg_definition.get(),
        "leg generation was not bound to its typed definition");

    const std::uint64_t wheel_generation =
        harness.rl.ActivateLWPolicy(wheel_policy);
    const auto wheel_activation = harness.rl.LoadLWPolicyActivation();
    require(
        wheel_activation
            && wheel_activation->generation == wheel_generation
            && wheel_generation != leg_generation
            && wheel_activation->definition == wheel_definition.get(),
        "wheel generation did not atomically replace the typed definition");
}

void testNominalReplayParity()
{
    Harness real;
    Harness sim;
    real.cycle();
    sim.cycle();
    require(real.rl.delivered.size() == 1, "real replay did not deliver");
    require(sim.rl.delivered.size() == 1, "sim replay did not deliver");
    requireCommandEqual(real.rl.delivered.back(), sim.rl.delivered.back());
    require(
        real.rl.fsm.current_state_->GetStateName()
            == sim.rl.fsm.current_state_->GetStateName(),
        "FSM state differs");
    requireSafetyEqual(real, sim);
}

void testExactPolicyInferenceReplayParity()
{
    Harness real;
    Harness sim;
    constexpr const char* policy = "LW/robot_lab/leg_loco";
    for (Harness* harness : {&real, &sim})
    {
        harness->rl.SetPolicyRoot(POLICY_DIR);
        harness->rl.ReadYaml("LW", "base.yaml");
        harness->rl.PreloadModel(policy);
        harness->rl.PreloadLWPolicyContext(policy);
        harness->rl.ActivateLWPolicy(policy);
        harness->rl.control.gait_frequency = 1.5f;
        harness->cycle(0.4f);
        harness->core.runInferenceCycle(false);
    }

    LWInferenceTraceSnapshot real_trace;
    LWInferenceTraceSnapshot sim_trace;
    require(real.core.readInferenceTrace(real_trace),
            "real inference trace was not published");
    require(sim.core.readInferenceTrace(sim_trace),
            "sim inference trace was not published");
    require(real_trace.generation == sim_trace.generation,
            "policy generation differs");
    require(real_trace.frame == sim_trace.frame, "policy frame differs");
    require(
        real_trace.source_input_sequence
            == sim_trace.source_input_sequence,
        "policy input sequence differs");
    requireObservationEqual(real_trace.observations, sim_trace.observations);
    requireVectorEqual(
        real_trace.output_dof_pos,
        sim_trace.output_dof_pos,
        "policy output q");
    requireVectorEqual(
        real_trace.output_dof_vel,
        sim_trace.output_dof_vel,
        "policy output dq");
    requireVectorEqual(
        real_trace.output_dof_tau,
        sim_trace.output_dof_tau,
        "policy output tau");
    requireSafetyEqual(real, sim);
}

void testS1PreservesRecoveryInputAndZerosVelocity()
{
    Harness real;
    Harness sim;
    real.core.reportSafetyEvent(LWSafetyEvent::ControlTimingDegraded);
    sim.core.reportSafetyEvent(LWSafetyEvent::ControlTimingDegraded);
    real.cycle(0.8f, Input::Keyboard::Num9, Input::Gamepad::B);
    sim.cycle(0.8f, Input::Keyboard::Num9, Input::Gamepad::B);

    for (Harness* harness : {&real, &sim})
    {
        require(
            harness->rl.nominal->observed_keyboard == Input::Keyboard::Num9,
            "S1 cleared the keyboard recovery input");
        require(
            harness->rl.nominal->observed_gamepad == Input::Gamepad::B,
            "S1 cleared the gamepad recovery input");
        requireVectorEqual(
            harness->rl.nominal->observed_velocity,
            {0.0f, 0.0f, 0.0f},
            "S1 velocity");
    }
    requireSafetyEqual(real, sim);
}

void testLWKeyboardVelocityCommandsAreIgnored()
{
    for (const auto keyboard : {
             Input::Keyboard::W,
             Input::Keyboard::S,
             Input::Keyboard::A,
             Input::Keyboard::D,
             Input::Keyboard::Q,
             Input::Keyboard::E,
             Input::Keyboard::Space})
    {
        Harness real;
        Harness sim;
        real.cycle(0.6f, keyboard);
        sim.cycle(0.6f, keyboard);

        for (Harness* harness : {&real, &sim})
        {
            require(
                harness->rl.nominal->observed_keyboard == keyboard,
                "LW FSM did not observe the keyboard event");
            requireVectorEqual(
                {harness->rl.control.x,
                 harness->rl.control.y,
                 harness->rl.control.yaw},
                {0.6f, -0.2f, 0.3f},
                "LW keyboard changed joystick velocity");
        }
        requireSafetyEqual(real, sim);
    }
}

void testNonLWStateControllerRetainsLegacyKeyboardVelocity()
{
    ReplayRL rl;
    rl.control.x = 0.6f;
    rl.control.SetKeyboard(Input::Keyboard::W);
    rl.StateController(&rl.robot_state, &rl.robot_command);
    require(
        std::fabs(rl.control.x - 0.7f) <= 1.0e-6f,
        "non-LW StateController lost its legacy keyboard velocity behavior");
}

void testS2ExecutesPassiveDamping()
{
    Harness real;
    Harness sim;
    real.core.reportSafetyEvent(LWSafetyEvent::InferenceLoopException);
    sim.core.reportSafetyEvent(LWSafetyEvent::InferenceLoopException);
    real.cycle();
    sim.cycle();

    for (Harness* harness : {&real, &sim})
    {
        require(harness->rl.delivered.size() == 1, "S2 command was not delivered");
        require(
            harness->rl.fsm.current_state_->GetStateName()
                == "RLFSMStatePassive",
            "S2 did not enter Passive");
        const auto& command = harness->rl.delivered.back().motor_command;
        for (size_t index = 0; index < kNumDofs; ++index)
        {
            require(command.q[index] == harness->rl.source.motor_state.q[index],
                    "S2 did not retain current q");
            require(command.dq[index] == 0.0f, "S2 dq was not zero");
            require(command.tau[index] == 0.0f, "S2 tau was not zero");
            require(command.kp[index] == 0.0f, "S2 Kp was not zero");
            require(command.kd[index] == 5.0f, "S2 Kd was not five");
        }
    }
    requireCommandEqual(real.rl.delivered.back(), sim.rl.delivered.back());
    requireSafetyEqual(real, sim);
}

void testS3AndS4LatchAndZeroActuators()
{
    for (const auto event : {
             LWSafetyEvent::MotorHardwareFault,
             LWSafetyEvent::ControlLoopException})
    {
        Harness real;
        Harness sim;
        real.core.reportSafetyEvent(event);
        sim.core.reportSafetyEvent(event);
        real.cycle();
        sim.cycle();
        require(real.rl.delivered.empty(), "terminal real command was delivered");
        require(sim.rl.delivered.empty(), "terminal sim command was delivered");
        requireSafetyEqual(real, sim);
        for (float effort : sim.adapter.actuator_effort)
        {
            require(effort == 0.0f, "terminal sim actuator was not zero");
        }
        const bool expect_shutdown =
            event == LWSafetyEvent::ControlLoopException;
        require(
            sim.adapter.shutdown_requested == expect_shutdown,
            "S3/S4 shutdown recording differs");
    }
}

void testInjectedFaultDecisionParity()
{
    const auto compare_event = [](LWSafetyEvent event)
    {
        Harness real;
        Harness sim;
        real.core.reportSafetyEvent(event);
        sim.core.reportSafetyEvent(event);
        requireSafetyEqual(real, sim);
    };

    compare_event(LWSafetyEvent::PolicyOutputUnavailable);
    compare_event(LWSafetyEvent::PolicyInputUnavailable);

    {
        Harness real;
        Harness sim;
        real.core.handleControlTiming(false, 3);
        sim.core.handleControlTiming(false, 3);
        requireSafetyEqual(real, sim);
    }
    {
        for (const auto status : {
                 LWPolicyOutputStatus::Incomplete,
                 LWPolicyOutputStatus::Stale,
                 LWPolicyOutputStatus::GenerationMismatch})
        {
            Harness real;
            Harness sim;
            real.core.handlePolicyOutputFault(status);
            sim.core.handlePolicyOutputFault(status);
            requireSafetyEqual(real, sim);
        }
    }
    {
        for (const std::string loop_name : {
                 "loop_rl", "loop_control", "unexpected_loop"})
        {
            Harness real;
            Harness sim;
            const auto error = std::make_exception_ptr(
                std::runtime_error("injected callback failure"));
            real.core.handleLoopError(loop_name, error);
            sim.core.handleLoopError(loop_name, error);
            requireSafetyEqual(real, sim);
        }
    }
    {
        Harness real;
        Harness sim;
        real.core.handleControlTiming(true, 7);
        sim.core.handleControlTiming(true, 7);
        requireSafetyEqual(real, sim);
        require(real.core.terminalLatched(), "fatal timing did not latch terminal");
    }
    {
        Harness real;
        Harness sim;
        std::vector<float> actions(kNumDofs, 0.0f);
        actions[2] = std::numeric_limits<float>::quiet_NaN();
        require(!real.core.acceptPolicyActions(actions, kNumDofs),
                "real accepted NaN action");
        require(!sim.core.acceptPolicyActions(actions, kNumDofs),
                "sim accepted NaN action");
        requireSafetyEqual(real, sim);
    }
    {
        Harness real;
        Harness sim;
        std::vector<float> q(kNumDofs, 0.0f);
        std::vector<float> dq(kNumDofs, 0.0f);
        std::vector<float> tau(kNumDofs, 0.0f);
        tau[4] = std::numeric_limits<float>::infinity();
        require(!real.core.acceptPolicyOutputs(q, dq, tau, kNumDofs),
                "real accepted Inf output");
        require(!sim.core.acceptPolicyOutputs(q, dq, tau, kNumDofs),
                "sim accepted Inf output");
        requireSafetyEqual(real, sim);
    }
    {
        Harness real;
        Harness sim;
        real.rl.source.imu.gyroscope[0] =
            std::numeric_limits<float>::quiet_NaN();
        sim.rl.source.imu.gyroscope[0] =
            std::numeric_limits<float>::quiet_NaN();
        real.cycle();
        sim.cycle();
        requireSafetyEqual(real, sim);
        require(real.core.terminalLatched(), "NaN feedback did not latch terminal");
    }
    {
        Harness real;
        Harness sim;
        real.rl.nominal->offset = std::numeric_limits<float>::quiet_NaN();
        sim.rl.nominal->offset = std::numeric_limits<float>::quiet_NaN();
        real.cycle();
        sim.cycle();
        requireSafetyEqual(real, sim);
        require(real.rl.delivered.empty(), "invalid real command was delivered");
        require(sim.rl.delivered.empty(), "invalid sim command was delivered");
        require(real.core.terminalLatched(), "invalid command did not latch terminal");
    }
}
} // namespace

int main()
{
    try
    {
        testNominalReplayParity();
        testExactPolicyInferenceReplayParity();
        testEffectiveCommandKeepsGaitObservationCoherent();
        testTemporaryInhibitionPreservesPhaseClock();
        testInputIsConsumedOnceAndHeldOutputRemainsUsable();
        testStalledControlInputTriggersS2Parity();
        testPolicyGenerationSwitchWaitsForMatchingInput();
        testPolicyGenerationSwitchBindsTypedRuntimeConfiguration();
        testS1PreservesRecoveryInputAndZerosVelocity();
        testLWKeyboardVelocityCommandsAreIgnored();
        testNonLWStateControllerRetainsLegacyKeyboardVelocity();
        testS2ExecutesPassiveDamping();
        testS3AndS4LatchAndZeroActuators();
        testInjectedFaultDecisionParity();
        std::cout << "LW runtime parity tests passed" << std::endl;
        return EXIT_SUCCESS;
    }
    catch (const std::exception& exception)
    {
        std::cerr << "LW runtime parity tests failed: "
                  << exception.what() << std::endl;
        return EXIT_FAILURE;
    }
}
