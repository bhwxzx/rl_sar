#ifndef LW_RUNTIME_CORE_HPP
#define LW_RUNTIME_CORE_HPP

#include "lw_control_safety.hpp"
#include "lw_safety_policy.hpp"
#include "rl_sdk.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

struct LWRuntimeSafetySnapshot
{
    LWSafetySnapshot decision;
    bool terminal_latched = false;
    bool shutdown_requested = false;
};

struct LWInferenceTraceSnapshot
{
    std::uint64_t generation = 0;
    std::uint64_t frame = 0;
    std::uint64_t source_input_sequence = 0;
    std::chrono::steady_clock::time_point source_state_time{};
    Observations<float> observations;
    std::vector<float> output_dof_pos;
    std::vector<float> output_dof_vel;
    std::vector<float> output_dof_tau;
};

struct LWControlCycleHooks
{
    std::function<void()> apply_input;
    std::function<void()> apply_keyboard;
    std::function<bool()> platform_ready;
    std::function<bool()> platform_precheck;
    std::function<void()> adapter_controls;
    std::function<void()> after_command_delivery;
};

struct LWInferenceCycleHooks
{
    std::function<void(
        Observations<float>&,
        const RobotState<float>&)> mutate_observation;
    std::function<void()> after_forward;
    std::function<void(
        const std::vector<float>&,
        const RobotState<float>&,
        const Observations<float>&,
        const std::vector<float>&,
        const std::vector<float>&)> after_publish;
};

// Platform-neutral LW control, inference, validation, and safety orchestration.
// Real hardware and MuJoCo supply only lifecycle and I/O hooks.
class LWRuntimeCore
{
public:
    using SafetySink = std::function<void(
        const LWSafetyDecision&,
        const std::string&)>;

    void bind(RL& rl, SafetySink safety_sink)
    {
        rl_ = &rl;
        safety_sink_ = std::move(safety_sink);
    }

    const LWSafetyDecision& reportSafetyEvent(
        LWSafetyEvent event,
        const std::string& reason = {}) noexcept
    {
        const auto& decision = safety_supervisor_.report(event);
        if (decision.action == LWSafetyAction::HardDisable
            || decision.action == LWSafetyAction::HardDisableAndShutdown
            || decision.action == LWSafetyAction::AbortStartup)
        {
            terminal_latched_.store(true, std::memory_order_release);
        }
        if (decision.action == LWSafetyAction::HardDisableAndShutdown
            || decision.action == LWSafetyAction::AbortStartup
            || decision.action == LWSafetyAction::OrderlyShutdown)
        {
            shutdown_requested_.store(true, std::memory_order_release);
        }

        if (safety_sink_)
        {
            try
            {
                safety_sink_(decision, reason);
            }
            catch (...)
            {
                // A safety adapter must never unwind into a real-time loop.
            }
        }
        return decision;
    }

    void handlePolicyOutputFault(LWPolicyOutputStatus) noexcept
    {
        reportSafetyEvent(
            LWSafetyEvent::PolicyOutputUnavailable,
            "[Safety] LW policy output became stale or incomplete");
    }

    void handlePolicyInputFault(LWPolicyInputStatus status) noexcept
    {
        reportSafetyEvent(
            LWSafetyEvent::PolicyInputUnavailable,
            std::string("[Safety] LW policy input provenance is ")
                + LWPolicyInputStatusName(status));
    }

    void handleLoopError(
        const std::string& loop_name,
        std::exception_ptr error,
        const std::function<void()>& joystick_fault = {}) noexcept
    {
        std::string message = "unknown exception";
        try
        {
            if (error)
            {
                std::rethrow_exception(error);
            }
        }
        catch (const std::exception& exception)
        {
            message = exception.what();
        }
        catch (...)
        {
        }

        if (loop_name == "loop_joystick")
        {
            if (joystick_fault)
            {
                try
                {
                    joystick_fault();
                }
                catch (...)
                {
                }
            }
            reportSafetyEvent(LWSafetyEvent::JoystickLoopException);
            return;
        }
        if (loop_name == "loop_rl")
        {
            reportSafetyEvent(
                LWSafetyEvent::InferenceLoopException,
                "[Loop] Inference callback exception - error: " + message);
            return;
        }
        reportSafetyEvent(
            loop_name == "loop_control"
                ? LWSafetyEvent::ControlLoopException
                : LWSafetyEvent::UnknownLoopException,
            "[Loop] Fatal callback exception - name: " + loop_name
                + ", error: " + message);
    }

    void handleControlTiming(
        bool fatal,
        std::uint64_t missed_deadlines) noexcept
    {
        if (!fatal)
        {
            reportSafetyEvent(LWSafetyEvent::ControlTimingDegraded);
            return;
        }
        reportSafetyEvent(
            LWSafetyEvent::ControlTimingFatal,
            "[Timing] Fatal control-loop timing fault after "
                + std::to_string(missed_deadlines)
                + " missed deadlines; hard-disable threshold was explicitly enabled");
    }

    bool inputInhibited() const noexcept
    {
        return static_cast<int>(safety_supervisor_.snapshot().highest_severity)
            >= static_cast<int>(LWSafetySeverity::InputDegraded);
    }

    bool controlledFallbackLatched() const noexcept
    {
        return safety_supervisor_.controlledFallbackLatched();
    }

    bool terminalLatched() const noexcept
    {
        return terminal_latched_.load(std::memory_order_acquire);
    }

    LWRuntimeSafetySnapshot safetySnapshot() const noexcept
    {
        return {
            safety_supervisor_.snapshot(),
            terminalLatched(),
            shutdown_requested_.load(std::memory_order_acquire)};
    }

    LWSafetySupervisor& safetySupervisor() noexcept
    {
        return safety_supervisor_;
    }

    bool readInferenceTrace(LWInferenceTraceSnapshot& trace) const
    {
        return inference_trace_.read(trace);
    }

    bool acceptPolicyActions(
        const std::vector<float>& actions,
        size_t num_dofs)
    {
        const LWValidationResult result =
            LWValidatePolicyActions(actions, num_dofs);
        if (result.valid())
        {
            return true;
        }
        reportSafetyEvent(
            LWSafetyEvent::PolicyActionInvalid,
            "[Safety] Invalid LW policy action: "
                + result.failureDescription());
        return false;
    }

    bool acceptPolicyOutputs(
        const std::vector<float>& positions,
        const std::vector<float>& velocities,
        const std::vector<float>& torques,
        size_t num_dofs)
    {
        const LWValidationResult result = LWValidatePolicyOutputs(
            positions,
            velocities,
            torques,
            num_dofs);
        if (result.valid())
        {
            return true;
        }
        reportSafetyEvent(
            LWSafetyEvent::PolicyOutputInvalid,
            "[Safety] Invalid LW policy output: "
                + result.failureDescription());
        return false;
    }

    void publishInitialPolicyInput()
    {
        requireBound();
        publishPolicyInput(std::chrono::steady_clock::now());
    }

    void runControlCycle(const LWControlCycleHooks& hooks)
    {
        requireBound();
        call(hooks.apply_input);
        call(hooks.apply_keyboard);
        inhibitVelocityIfRequired();

        rl_->GetState(&rl_->robot_state);
        const auto state_capture_time = std::chrono::steady_clock::now();
        if (terminalLatched())
        {
            return;
        }
        if (hooks.platform_ready && !hooks.platform_ready())
        {
            return;
        }
        if (!validateFeedbackAndAttitude())
        {
            return;
        }
        if (hooks.platform_precheck && !hooks.platform_precheck())
        {
            return;
        }

        const bool fallback_before_controller = controlledFallbackLatched();
        if (fallback_before_controller)
        {
            applyControlledFallbackCommand();
            if (rl_->fsm.current_state_
                && rl_->fsm.current_state_->GetStateName()
                    != "RLFSMStatePassive")
            {
                rl_->StateController(
                    &rl_->robot_state,
                    &rl_->robot_command,
                    false);
            }
        }
        else
        {
            rl_->StateController(
                &rl_->robot_state,
                &rl_->robot_command,
                false);
        }
        if (controlledFallbackLatched())
        {
            applyControlledFallbackCommand();
        }
        inhibitVelocityIfRequired();
        publishPolicyInput(state_capture_time);

        call(hooks.adapter_controls);
        if (!validateFeedbackAndAttitude()
            || !validateCommandForSend(rl_->robot_command)
            || terminalLatched())
        {
            return;
        }

        rl_->control.ClearInput();
        rl_->SetCommand(&rl_->robot_command);
        call(hooks.after_command_delivery);
    }

    void runInferenceCycle(
        bool external_input_fault,
        const LWInferenceCycleHooks& hooks = {})
    {
        requireBound();
        if (terminalLatched() || controlledFallbackLatched())
        {
            return;
        }
        const auto activation = rl_->LoadLWPolicyActivation();
        if (!activation || !activation->definition)
        {
            return;
        }
        if (!inference_activation_
            || inference_activation_->generation != activation->generation)
        {
            inference_activation_ = activation;
            resetInferenceWorkspace(*activation);
        }

        if (!policy_input_snapshot_.read(inference_policy_input_))
        {
            return;
        }
        const LWPolicyInputSnapshot& policy_input =
            inference_policy_input_;
        const auto policy_data_max_age =
            rl_->GetLWPolicyOutputMaxAge(*activation);
        const auto input_now = std::chrono::steady_clock::now();
        const LWPolicyInputStatus input_status = EvaluateLWPolicyInput(
            &policy_input,
            activation->generation,
            input_now,
            policy_data_max_age,
            last_inference_input_generation_,
            last_inference_input_sequence_,
            last_inference_state_capture_time_);
        if (input_status != LWPolicyInputStatus::Ready)
        {
            if (LWPolicyInputRequiresFallback(input_status))
            {
                handlePolicyInputFault(input_status);
            }
            return;
        }
        const RobotState<float>& local_state = policy_input.robot_state;
        const LWControlSnapshot& local_control = policy_input.control;
        LWControlSnapshot effective_control = local_control;
        if (external_input_fault || inputInhibited())
        {
            effective_control.x = 0.0f;
            effective_control.y = 0.0f;
            effective_control.yaw = 0.0f;
        }

        inference_motion_reference_ = rl_->LoadLWMotionReference();
        const auto& policy_configuration =
            activation->definition->runtime;
        const bool needs_motion_reference =
            policy_configuration.needs_motion_reference;
        if (needs_motion_reference
            && (!inference_motion_reference_
                || inference_motion_reference_->generation
                    != activation->generation))
        {
            return;
        }
        last_inference_input_generation_ = policy_input.generation;
        last_inference_input_sequence_ = policy_input.sequence;
        last_inference_state_capture_time_ =
            policy_input.state_capture_time;
        ++inference_frame_;
        inference_obs_.ang_vel = local_state.imu.gyroscope;
        inference_obs_.commands = {
            effective_control.x,
            effective_control.y,
            effective_control.yaw};
        inference_obs_.base_quat = local_state.imu.quaternion;
        inference_obs_.dof_pos = local_state.motor_state.q;
        inference_obs_.dof_vel = local_state.motor_state.dq;
        if (hooks.mutate_observation)
        {
            hooks.mutate_observation(inference_obs_, local_state);
        }

        inference_gait_phase_time_ +=
            policy_configuration.period_seconds
            * effective_control.gait_frequency;
        while (inference_gait_phase_time_ >= 1.0f)
        {
            inference_gait_phase_time_ -= 1.0f;
        }
        const float command_norm = std::sqrt(
            effective_control.x * effective_control.x
            + effective_control.y * effective_control.y
            + effective_control.yaw * effective_control.yaw);
        const float is_moving = command_norm > 0.1f ? 1.0f : 0.0f;
        constexpr float pi = 3.14159265358979323846f;
        inference_obs_.gait_phase = {
            is_moving * std::sin(2.0f * pi * inference_gait_phase_time_),
            is_moving * std::cos(2.0f * pi * inference_gait_phase_time_)};

        inference_obs_.actions = forward();
        call(hooks.after_forward);
        if (terminalLatched() || controlledFallbackLatched())
        {
            return;
        }

        rl_->ComputeLWOutput(
            policy_configuration,
            inference_obs_,
            inference_obs_.actions,
            inference_output_dof_pos_,
            inference_output_dof_vel_,
            inference_output_dof_tau_);
        const size_t num_dofs = policy_configuration.num_dofs;
        if (!acceptPolicyOutputs(
                inference_output_dof_pos_,
                inference_output_dof_vel_,
                inference_output_dof_tau_,
                num_dofs))
        {
            return;
        }

        if (rl_->TorqueProtect(
                inference_output_dof_tau_,
                policy_configuration))
        {
            reportSafetyEvent(LWSafetyEvent::TorqueLimitWarning);
        }
        const auto inference_completed_at =
            std::chrono::steady_clock::now();
        if (inference_completed_at < policy_input.state_capture_time
            || inference_completed_at - policy_input.state_capture_time
                > policy_data_max_age)
        {
            handlePolicyInputFault(
                inference_completed_at < policy_input.state_capture_time
                    ? LWPolicyInputStatus::Future
                    : LWPolicyInputStatus::Stale);
            return;
        }
        if (!rl_->PublishLWPolicyOutput(
            {policy_input.generation,
             0,
             inference_frame_,
             policy_input.sequence,
             policy_input.state_capture_time,
             inference_output_dof_pos_,
             inference_output_dof_vel_,
             inference_output_dof_tau_}))
        {
            return;
        }
        rl_->PublishLWPolicyProgress(
            activation->generation,
            inference_frame_);
        inference_trace_.publish(
            {activation->generation,
             inference_frame_,
             policy_input.sequence,
             policy_input.state_capture_time,
             inference_obs_,
             inference_output_dof_pos_,
             inference_output_dof_vel_,
             inference_output_dof_tau_});
        if (hooks.after_publish)
        {
            hooks.after_publish(
                inference_output_dof_tau_,
                local_state,
                inference_obs_,
                inference_output_dof_pos_,
                inference_output_dof_vel_);
        }
    }

    std::vector<float> forward()
    {
        requireBound();
        if (!inference_activation_
            || !inference_activation_->definition
            || !inference_activation_->definition->model)
        {
            return {};
        }
        const auto& definition = *inference_activation_->definition;
        const auto& policy_configuration = definition.runtime;
        const auto clamped_obs = rl_->ComputeLWObservation(
            policy_configuration,
            inference_obs_,
            inference_obs_dims_,
            inference_motion_reference_.get(),
            inference_frame_,
            inference_activation_->motion_length);

        std::vector<float> actions;
        const auto& history_indices =
            policy_configuration.observations_history;
        if (!history_indices.empty())
        {
            if (inference_frame_ == 1)
            {
                inference_history_obs_buf_.reset({0}, clamped_obs);
            }
            inference_history_obs_buf_.insert(clamped_obs);
            inference_history_obs_ =
                inference_history_obs_buf_.get_obs_vec(history_indices);
            actions = definition.model->forward({inference_history_obs_});
        }
        else
        {
            actions = definition.model->forward({clamped_obs});
        }

        const size_t num_dofs = policy_configuration.num_dofs;
        if (!acceptPolicyActions(actions, num_dofs))
        {
            return {};
        }

        const auto& upper =
            policy_configuration.clip_actions_upper;
        const auto& lower =
            policy_configuration.clip_actions_lower;
        if (!upper.empty() && !lower.empty())
        {
            const LWValidationResult upper_result =
                LWValidateFiniteVector("clip_actions_upper", upper, num_dofs);
            const LWValidationResult lower_result =
                LWValidateFiniteVector("clip_actions_lower", lower, num_dofs);
            if (!upper_result.valid() || !lower_result.valid())
            {
                const LWValidationResult& clip_result =
                    upper_result.valid() ? lower_result : upper_result;
                reportSafetyEvent(
                    LWSafetyEvent::PolicyConfigurationInvalid,
                    "[Safety] Invalid LW action clipping configuration: "
                        + clip_result.failureDescription());
                return {};
            }
            return clamp(actions, lower, upper);
        }
        return actions;
    }

private:
    static void call(const std::function<void()>& hook)
    {
        if (hook)
        {
            hook();
        }
    }

    void requireBound() const
    {
        if (!rl_)
        {
            throw std::logic_error("LW runtime core is not bound");
        }
    }

    void inhibitVelocityIfRequired()
    {
        if (!inputInhibited())
        {
            return;
        }
        rl_->control.x = 0.0f;
        rl_->control.y = 0.0f;
        rl_->control.yaw = 0.0f;
    }

    void publishPolicyInput(
        std::chrono::steady_clock::time_point state_capture_time)
    {
        const auto activation = rl_->LoadLWPolicyActivation();
        control_policy_input_.generation =
            activation ? activation->generation : 0;
        control_policy_input_.sequence =
            next_policy_input_sequence_;
        control_policy_input_.state_capture_time = state_capture_time;
        control_policy_input_.robot_state = rl_->robot_state;
        control_policy_input_.control =
            {rl_->control.x,
             rl_->control.y,
             rl_->control.yaw,
             rl_->control.gait_frequency};
        if (policy_input_snapshot_.tryPublish(control_policy_input_))
        {
            ++next_policy_input_sequence_;
        }
    }

    bool validateFeedbackAndAttitude()
    {
        const size_t num_dofs =
            rl_->GetLWBaseRuntimeConfiguration().num_dofs;
        const LWValidationResult feedback_result =
            LWValidateFeedbackState(rl_->robot_state, num_dofs);
        if (!feedback_result.valid())
        {
            reportSafetyEvent(
                LWSafetyEvent::FeedbackInvalid,
                "[Safety] Invalid LW feedback: "
                    + feedback_result.failureDescription());
            return false;
        }
        if (!rl_->fsm.current_state_)
        {
            reportSafetyEvent(
                LWSafetyEvent::FsmStateMissing,
                "[Safety] LW FSM has no current state");
            return false;
        }

        constexpr float attitude_threshold_degrees = 75.0f;
        const std::string& state_name =
            rl_->fsm.current_state_->GetStateName();
        const LWAttitudeValidationResult attitude_result = LWValidateAttitude(
            state_name,
            rl_->robot_state.imu.quaternion,
            attitude_threshold_degrees);
        if (!attitude_result.safe)
        {
            reportSafetyEvent(
                LWSafetyEvent::AttitudeLimitExceeded,
                "[Safety] LW attitude limit exceeded in " + state_name + ": "
                    + attitude_result.failureDescription(
                        attitude_threshold_degrees));
            return false;
        }
        return true;
    }

    bool validateCommandForSend(const RobotCommand<float>& command)
    {
        const size_t num_dofs =
            rl_->GetLWBaseRuntimeConfiguration().num_dofs;
        const LWValidationResult command_result =
            LWValidateRobotCommand(command, num_dofs);
        if (!command_result.valid())
        {
            reportSafetyEvent(
                LWSafetyEvent::RobotCommandInvalid,
                "[Safety] Invalid LW command: "
                    + command_result.failureDescription());
            return false;
        }
        return true;
    }

    void applyControlledFallbackCommand()
    {
        rl_->control.x = 0.0f;
        rl_->control.y = 0.0f;
        rl_->control.yaw = 0.0f;
        rl_->control.ClearInput();
        if (!controlled_fallback_applied_.exchange(
                true, std::memory_order_acq_rel))
        {
            rl_->DeactivateLWPolicy();
        }
        if (rl_->fsm.current_state_
            && rl_->fsm.current_state_->GetStateName()
                != "RLFSMStatePassive")
        {
            rl_->fsm.RequestStateChange("RLFSMStatePassive");
        }
        LWBuildPassiveDampingCommand(
            rl_->robot_state,
            rl_->robot_command,
            rl_->GetLWBaseRuntimeConfiguration().num_dofs);
    }

    void resetInferenceWorkspace(const LWPolicyActivation& activation)
    {
        const auto& policy_configuration = activation.definition->runtime;
        const size_t num_dofs = policy_configuration.num_dofs;
        inference_frame_ = 0;
        last_inference_input_generation_ = 0;
        last_inference_input_sequence_ = 0;
        last_inference_state_capture_time_ = {};
        inference_gait_phase_time_ = 0.0f;
        inference_motion_reference_.reset();
        inference_obs_ = {};
        inference_obs_.lin_vel = {0.0f, 0.0f, 0.0f};
        inference_obs_.ang_vel = {0.0f, 0.0f, 0.0f};
        inference_obs_.gravity_vec = {0.0f, 0.0f, -1.0f};
        inference_obs_.commands = {0.0f, 0.0f, 0.0f};
        inference_obs_.base_quat = {1.0f, 0.0f, 0.0f, 0.0f};
        inference_obs_.dof_pos =
            policy_configuration.default_dof_pos;
        inference_obs_.dof_vel.assign(num_dofs, 0.0f);
        inference_obs_.actions.assign(num_dofs, 0.0f);
        inference_obs_.gait_phase = {0.0f, 1.0f};
        inference_output_dof_pos_ = inference_obs_.dof_pos;
        inference_output_dof_vel_.assign(num_dofs, 0.0f);
        inference_output_dof_tau_.assign(num_dofs, 0.0f);
        inference_history_obs_.clear();

        rl_->ComputeLWObservation(
            policy_configuration,
            inference_obs_,
            inference_obs_dims_,
            nullptr,
            0,
            activation.motion_length);
        const auto& history_indices =
            policy_configuration.observations_history;
        if (!history_indices.empty())
        {
            const int history_length =
                *std::max_element(
                    history_indices.begin(),
                    history_indices.end())
                + 1;
            inference_history_obs_buf_ = ObservationBuffer(
                1,
                inference_obs_dims_,
                history_length,
                policy_configuration.observations_history_priority);
        }
    }

    RL* rl_ = nullptr;
    SafetySink safety_sink_;
    LWSafetySupervisor safety_supervisor_;
    std::atomic<bool> terminal_latched_{false};
    std::atomic<bool> shutdown_requested_{false};
    std::atomic<bool> controlled_fallback_applied_{false};
    LWSnapshotBuffer<LWPolicyInputSnapshot> policy_input_snapshot_;
    LWSnapshotBuffer<LWInferenceTraceSnapshot> inference_trace_;

    std::shared_ptr<const LWPolicyActivation> inference_activation_;
    LWPolicyInputSnapshot control_policy_input_;
    LWPolicyInputSnapshot inference_policy_input_;
    std::shared_ptr<const LWMotionReferenceSnapshot> inference_motion_reference_;
    Observations<float> inference_obs_;
    std::vector<int> inference_obs_dims_;
    ObservationBuffer inference_history_obs_buf_;
    std::vector<float> inference_history_obs_;
    std::vector<float> inference_output_dof_pos_;
    std::vector<float> inference_output_dof_vel_;
    std::vector<float> inference_output_dof_tau_;
    std::uint64_t inference_frame_ = 0;
    std::uint64_t next_policy_input_sequence_ = 1;
    std::uint64_t last_inference_input_generation_ = 0;
    std::uint64_t last_inference_input_sequence_ = 0;
    std::chrono::steady_clock::time_point
        last_inference_state_capture_time_{};
    float inference_gait_phase_time_ = 0.0f;
};

#endif // LW_RUNTIME_CORE_HPP
