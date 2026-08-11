
#include "rl_real_LW.hpp"
#include "lw_deployment_bundle.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cerrno>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>

using namespace std::chrono_literals;

namespace
{
std::chrono::nanoseconds ReadNonnegativeDuration(
    const YamlParams& params,
    const std::string& key,
    float default_seconds)
{
    const float seconds = params.Get<float>(key, default_seconds);
    if (!std::isfinite(seconds) || seconds < 0.0f)
    {
        throw std::runtime_error("LW " + key + " must be finite and nonnegative");
    }
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(seconds));
}

std::uint64_t ReadNonnegativeCount(
    const YamlParams& params,
    const std::string& key,
    int default_value)
{
    const int value = params.Get<int>(key, default_value);
    if (value < 0)
    {
        throw std::runtime_error("LW " + key + " must be nonnegative");
    }
    return static_cast<std::uint64_t>(value);
}

LoopConfig BuildLWControlLoopConfig(const YamlParams& params)
{
    LoopConfig config = LoopConfig::FromSeconds(params.Get<float>("dt"));
    config.cpu_affinity = params.Get<int>("control_loop_cpu", -1);
    config.realtime_priority =
        params.Get<int>("control_loop_realtime_priority", 0);
    config.require_realtime =
        params.Get<bool>("control_loop_require_realtime", false);
    config.timing_policy.degraded_consecutive_misses =
        ReadNonnegativeCount(
            params,
            "control_loop_degraded_consecutive_misses",
            3);
    config.timing_policy.degraded_lateness = ReadNonnegativeDuration(
        params,
        "control_loop_degraded_lateness",
        0.02f);
    config.timing_policy.fatal_consecutive_misses =
        ReadNonnegativeCount(
            params,
            "control_loop_fatal_consecutive_misses",
            0);
    config.timing_policy.fatal_lateness = ReadNonnegativeDuration(
        params,
        "control_loop_fatal_lateness",
        0.0f);
    return config;
}

const char* LWOperatorModeName(LWOperatorMode mode)
{
    switch (mode)
    {
    case LWOperatorMode::Passive: return "passive";
    case LWOperatorMode::GetUpLeg: return "get-up-leg";
    case LWOperatorMode::GetUpWheel: return "get-up-wheel";
    case LWOperatorMode::GetDown: return "get-down";
    case LWOperatorMode::LegLocomotion: return "leg-locomotion";
    case LWOperatorMode::WheelLocomotion: return "wheel-locomotion";
    case LWOperatorMode::LegToWheel: return "leg-to-wheel";
    case LWOperatorMode::WheelToLeg: return "wheel-to-leg";
    case LWOperatorMode::Unknown: return "unknown";
    }
    return "unknown";
}

bool LWOperatorModeHasProgress(LWOperatorMode mode)
{
    return mode == LWOperatorMode::GetUpLeg
        || mode == LWOperatorMode::GetUpWheel
        || mode == LWOperatorMode::GetDown
        || mode == LWOperatorMode::LegToWheel
        || mode == LWOperatorMode::WheelToLeg;
}
} // namespace

RL_Real::RL_Real(
    int argc,
    char **argv,
    const std::string& policy_root)
{
    this->SetPolicyRoot(policy_root);
    ros2_node = std::make_shared<rclcpp::Node>("rl_real_LW_node");
    const bool enable_keyboard = ros2_node->declare_parameter<bool>(
        "enable_keyboard",
        true);
    if (enable_keyboard)
    {
        this->terminal_keyboard_ = std::make_unique<LWTerminalKeyboard>();
        std::cout << LOGGER::INFO
                  << "[Input] Terminal keyboard enabled on /dev/tty"
                  << std::endl;
    }
    else
    {
        std::cerr << LOGGER::WARNING
                  << "[Input] Terminal keyboard disabled; no keyboard GetDown "
                     "recovery channel is available"
                  << std::endl;
    }
    // subscribe qos config
    auto subscribers_qos = rclcpp::SystemDefaultsQoS();
    subscribers_qos.keep_last(1);
    subscribers_qos.best_effort();
    this->imu_subscriber_ = ros2_node->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", subscribers_qos,
        [this] (const sensor_msgs::msg::Imu::SharedPtr imu_msg) {this->ImuCallback(imu_msg);}
    );

    this->SetupSysJoystick("/dev/input/js0", 16);

    // read params from yaml
    this->ang_vel_axis = "body";
    this->robot_name = "LW";
    this->ReadYaml(this->robot_name, "base.yaml");
    ValidateLWBaseConfiguration(
        this->params.config_node,
        this->ResolvePolicyPath(this->robot_name + "/base.yaml"));
    const float sensor_timeout_seconds = this->params.Get<float>("sensor_timeout");
    if (!std::isfinite(sensor_timeout_seconds) || sensor_timeout_seconds <= 0.0f)
    {
        throw std::runtime_error("LW sensor_timeout must be a finite positive value");
    }
    this->sensor_readiness_monitor_.setTimeout(
        std::chrono::duration_cast<SafetyClock::duration>(
            std::chrono::duration<float>(sensor_timeout_seconds)));
    const float serial_write_timeout_seconds =
        this->params.Get<float>("serial_write_timeout");
    if (!std::isfinite(serial_write_timeout_seconds)
        || serial_write_timeout_seconds <= 0.0f)
    {
        throw std::runtime_error("LW serial_write_timeout must be a finite positive value");
    }
    this->lw_sdk.SetWriteTimeout(
        std::chrono::duration_cast<LWSDK::Duration>(
            std::chrono::duration<float>(serial_write_timeout_seconds)));

    // 提前加载所有的模型到内存
    this->PreloadModel(this->robot_name + "/robot_lab/leg_loco");
    this->PreloadModel(this->robot_name + "/robot_lab/wheel_loco");
    this->PreloadModel(this->robot_name + "/robot_lab/leg_to_wheel");
    this->PreloadModel(this->robot_name + "/robot_lab/wheel_to_leg");
    this->PreloadLWPolicyContext(this->robot_name + "/robot_lab/leg_loco");
    this->PreloadLWPolicyContext(this->robot_name + "/robot_lab/wheel_loco");
    this->PreloadLWPolicyContext(this->robot_name + "/robot_lab/leg_to_wheel");
    this->PreloadLWPolicyContext(this->robot_name + "/robot_lab/wheel_to_leg");

    // auto load FSM by robot_name
    if (FSMManager::GetInstance().IsTypeSupported(this->robot_name))
    {
        auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
        if (fsm_ptr)
        {
            this->fsm = *fsm_ptr;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "[FSM] No FSM registered for robot: " << this->robot_name << std::endl;
    }

    // init robot
    this->lw_sdk.InitCmdData(this->lw_low_command);
    const LWSerialInitStatus serial_status =
        this->lw_sdk.InitSerial("/dev/ttyLegRight", "/dev/ttyLegLeft");
    if (!serial_status.bothInitialized())
    {
        safety_supervisor_.report(
            LWSafetyEvent::StartupSerialInitializationFailed);
        std::cerr << LOGGER::ERROR
                  << "[Safety] LW serial initialization failed: "
                  << serial_status.failureSummary()
                  << "; control loops will not start"
                  << std::endl;
        SendEmergencyDisableBurst();
        throw std::runtime_error("failed to initialize both LW serial ports");
    }
    const LWSendResult startup_disable = disable_lw_robot();
    if (!startup_disable.complete())
    {
        safety_supervisor_.report(
            LWSafetyEvent::StartupInitialDisableIncomplete);
        std::cerr << LOGGER::ERROR
                  << "[Safety] Initial disable command was incomplete: "
                  << startup_disable.failureSummary() << std::endl;
        SendEmergencyDisableBurst();
        throw std::runtime_error("failed to send complete initial disable command");
    }
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    this->InitOutputs();
    this->InitControl();
    this->control.gait_frequency = this->params.Get<std::vector<float>>("gait_command")[0];
    this->gait_phase_time = 0.0f;
    policy_input_snapshot_.publish(
        {this->robot_state,
         {this->control.x,
          this->control.y,
          this->control.yaw,
          this->control.gait_frequency}});

    // Build every resource before starting worker threads. The callbacks capture
    // this object, so they must not observe a partially initialized RL_Real.
    const auto loop_error_handler = [this](const std::string& loop_name, std::exception_ptr error)
    {
        this->HandleLoopError(loop_name, error);
    };
    const auto loop_timing_handler = [this](
        const std::string& loop_name,
        LoopTimingLevel level,
        const LoopTimingSnapshot& timing)
    {
        this->HandleLoopTiming(loop_name, level, timing);
    };
    this->loop_joystick = std::make_shared<LoopFunc>(
        "loop_joystick", 0.01, std::bind(&RL_Real::GetSysJoystick, this), -1, loop_error_handler);
    LoopConfig control_loop_config = BuildLWControlLoopConfig(this->params);
    this->loop_control = std::make_shared<LoopFunc>(
        "loop_control",
        control_loop_config,
        std::bind(&RL_Real::RobotControl, this),
        loop_error_handler,
        loop_timing_handler);
    this->loop_rl = std::make_shared<LoopFunc>(
        "loop_rl",
        this->params.Get<float>("dt") * this->params.Get<int>("decimation"),
        std::bind(&RL_Real::RunModel, this),
        -1,
        loop_error_handler);

    this->runtime_diagnostics_timer_ = ros2_node->create_wall_timer(
        100ms,
        std::bind(&RL_Real::RuntimeDiagnosticsCallback, this));

    const bool enable_debug_publisher =
        ros2_node->declare_parameter<bool>(
            "enable_debug_publisher",
            false);
    this->debug_publisher_ = LWDebugPublisher::CreateIfEnabled(
        enable_debug_publisher,
        ros2_node,
        LWDebugPublisherConfig{
            this->params.Get<int>("num_of_dofs"),
            this->params.Get<std::vector<int>>("joint_mapping"),
            this->params.Get<std::vector<int>>("wheel_indices"),
            this->params.Get<std::vector<float>>("rl_kp"),
            this->params.Get<std::vector<float>>("rl_kd"),
            4ms});
    if (this->debug_publisher_)
    {
        std::cout << LOGGER::INFO
                  << "[Debug] Publishing /LW_joint_states at 250 Hz"
                  << std::endl;
    }
#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif

    try
    {
        this->loop_joystick->start();
        this->loop_rl->start();
        this->loop_control->start();
    }
    catch (...)
    {
        safety_supervisor_.report(LWSafetyEvent::StartupLoopStartFailed);
        command_gate_.close();
        this->loop_control->shutdown();
        this->loop_rl->shutdown();
        this->loop_joystick->shutdown();
        SendEmergencyDisableBurst();
        throw;
    }
}

RL_Real::~RL_Real()
{
    safety_supervisor_.report(LWSafetyEvent::NormalShutdown);
    // Close the gate before joining so no new enable command can race with the
    // final disable frame. Stop the command producer first.
    command_gate_.close();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
    this->loop_joystick->shutdown();
    this->terminal_keyboard_.reset();
    const LWSendResult final_disable = disable_lw_robot(true);
    if (!final_disable.complete())
    {
        std::cerr << LOGGER::ERROR
                  << "[Safety] Final shutdown disable was incomplete: "
                  << final_disable.failureSummary() << std::endl;
    }
    std::cout << LOGGER::INFO << "RL_Real exit" << std::endl;
}

void RL_Real::RuntimeDiagnosticsCallback()
{
    const LWSafetySnapshot safety = safety_supervisor_.snapshot();
    if (safety.sequence != 0
        && safety.sequence != last_safety_event_sequence_)
    {
        last_safety_event_sequence_ = safety.sequence;
        const auto& decision = LWSafetyDecisionFor(safety.latest_event);
        std::ostream& output =
            decision.severity >= LWSafetySeverity::InputDegraded
            ? std::cerr
            : std::cout;
        output << (decision.severity >= LWSafetySeverity::InputDegraded
                       ? LOGGER::WARNING
                       : LOGGER::INFO)
               << "[Safety] event=" << decision.name
               << ", severity=" << LWSafetySeverityName(decision.severity)
               << ", highest_latched="
               << LWSafetySeverityName(safety.highest_severity)
               << ", action=" << LWSafetyActionName(decision.action)
               << ", restart_required="
               << (decision.restart_required ? "yes" : "no")
               << std::endl;
    }

    LWOperatorStatusSnapshot operator_status;
    if (ReadLWOperatorStatus(operator_status)
        && (!operator_status_seen_
            || operator_status.sequence != last_operator_status_sequence_))
    {
        operator_status_seen_ = true;
        last_operator_status_sequence_ = operator_status.sequence;

        std::ostringstream message;
        message << LOGGER::INFO << "[LW] mode="
                << LWOperatorModeName(operator_status.mode)
                << std::fixed << std::setprecision(3)
                << ", x=" << operator_status.x
                << ", y=" << operator_status.y
                << ", yaw=" << operator_status.yaw
                << ", gait=" << operator_status.gait_frequency;
        if (LWOperatorModeHasProgress(operator_status.mode))
        {
            message << std::setprecision(1)
                    << ", progress=" << operator_status.progress * 100.0f
                    << '%';
        }
        std::cout << message.str() << std::endl;
    }

    if (!loop_control)
    {
        return;
    }

    const LoopStartupSnapshot startup = loop_control->startupSnapshot();
    if (!realtime_fallback_logged_
        && startup.requested_realtime_priority > 0
        && !startup.realtime_applied)
    {
        realtime_fallback_logged_ = true;
        std::cerr << LOGGER::WARNING
                  << "[Timing] SCHED_FIFO priority "
                  << startup.requested_realtime_priority
                  << " was not applied (error " << startup.realtime_error
                  << "); loop_control is running with SCHED_OTHER"
                  << std::endl;
    }

    if (!timing_degraded_logged_
        && control_timing_degraded_latched_.load(std::memory_order_acquire))
    {
        timing_degraded_logged_ = true;
        std::cerr << LOGGER::WARNING
                  << "[Timing] Control timing is degraded: x/y/yaw are latched "
                     "to zero, FSM buttons remain active so GetDown can be "
                     "requested, and restart is required to clear the latch"
                  << std::endl;
    }

    ++runtime_diagnostics_ticks_;
    if (runtime_diagnostics_ticks_ % 10 != 0)
    {
        return;
    }

    const LoopTimingSnapshot timing = loop_control->timingSnapshot();
    const double average_wakeup_us = timing.cycles == 0
        ? 0.0
        : std::chrono::duration<double, std::micro>(
              timing.total_wakeup_lateness).count()
              / static_cast<double>(timing.cycles);
    std::ostringstream message;
    message << LOGGER::INFO << "[Timing] loop_control cycles=" << timing.cycles
            << ", wakeup_us(avg/max)=" << std::fixed << std::setprecision(1)
            << average_wakeup_us << '/'
            << std::chrono::duration<double, std::micro>(
                   timing.max_wakeup_lateness).count()
            << ", deadline_late_us(max)="
            << std::chrono::duration<double, std::micro>(
                   timing.max_deadline_lateness).count()
            << ", execution_us(max)="
            << std::chrono::duration<double, std::micro>(
                   timing.max_execution_time).count()
            << ", missed_deadlines=" << timing.missed_deadlines
            << ", skipped_periods=" << timing.skipped_periods;
    std::cout << message.str() << std::endl;
}

LWSendResult RL_Real::disable_lw_robot(bool latch_commands)
{
    LowCmd cmd = {0};
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); i++)
    {
        cmd.motorCmd[i].action_set = 0.0f;
        cmd.motorCmd[i].Kp = 0.0f;
        cmd.motorCmd[i].Kd = 0.0f;
    }
    cmd.motors_disable = true;

    LWSendResult result;
    const auto send_disable = [this, &cmd, &result]()
    {
        result = this->lw_sdk.SendCmdData(cmd);
    };

    if (latch_commands)
    {
        command_gate_.closeAndSend(send_disable);
    }
    else
    {
        command_gate_.sendSerialized(send_disable);
    }
    return result;
}

void RL_Real::HandleLoopError(const std::string& loop_name, std::exception_ptr error) noexcept
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
        LatchJoystickFault({LWJoystickSampleStatus::Error, EFAULT, -1});
        safety_supervisor_.report(LWSafetyEvent::JoystickLoopException);
        return;
    }
    if (loop_name == "loop_rl")
    {
        ApplySafetyEvent(
            LWSafetyEvent::InferenceLoopException,
            "[Loop] Inference callback exception - error: " + message);
        return;
    }

    ApplySafetyEvent(
        loop_name == "loop_control"
            ? LWSafetyEvent::ControlLoopException
            : LWSafetyEvent::UnknownLoopException,
        "[Loop] Fatal callback exception - name: " + loop_name
            + ", error: " + message);
}

void RL_Real::HandleLoopTiming(
    const std::string& loop_name,
    LoopTimingLevel level,
    const LoopTimingSnapshot& timing) noexcept
{
    if (loop_name != "loop_control")
    {
        return;
    }

    if (level == LoopTimingLevel::Degraded)
    {
        safety_supervisor_.report(LWSafetyEvent::ControlTimingDegraded);
        control_timing_degraded_latched_.store(
            true, std::memory_order_release);
        return;
    }

    if (level == LoopTimingLevel::Fatal)
    {
        control_timing_degraded_latched_.store(
            true, std::memory_order_release);
        ApplySafetyEvent(
            LWSafetyEvent::ControlTimingFatal,
            "[Timing] Fatal control-loop timing fault after "
            + std::to_string(timing.missed_deadlines)
            + " missed deadlines; hard-disable threshold was explicitly enabled");
    }
}

void RL_Real::SendEmergencyDisableBurst() noexcept
{
    constexpr int disable_attempts = 20;
    constexpr auto disable_interval = 5ms;

    command_gate_.close();
    bool send_error_logged = false;
    for (int attempt = 0; attempt < disable_attempts; ++attempt)
    {
        try
        {
            const LWSendResult result = disable_lw_robot(true);
            if (!result.complete() && !send_error_logged)
            {
                std::cerr << LOGGER::ERROR
                          << "[Safety] Emergency disable was incomplete: "
                          << result.failureSummary() << std::endl;
                send_error_logged = true;
            }
        }
        catch (const std::exception& exception)
        {
            if (!send_error_logged)
            {
                std::cerr << LOGGER::ERROR << "[Safety] Failed to send emergency disable: "
                          << exception.what() << std::endl;
                send_error_logged = true;
            }
        }
        catch (...)
        {
            if (!send_error_logged)
            {
                std::cerr << LOGGER::ERROR << "[Safety] Failed to send emergency disable"
                          << std::endl;
                send_error_logged = true;
            }
        }

        if (attempt + 1 < disable_attempts)
        {
            std::this_thread::sleep_for(disable_interval);
        }
    }
}

void RL_Real::HandleLWPolicyOutputFault(
    LWPolicyOutputStatus status) noexcept
{
    (void)status;
    ApplySafetyEvent(
        LWSafetyEvent::PolicyOutputUnavailable,
        "[Safety] LW policy output became stale or incomplete");
}

void RL_Real::ApplySafetyEvent(
    LWSafetyEvent event,
    const std::string& reason) noexcept
{
    const auto& decision = safety_supervisor_.report(event);
    if (decision.action == LWSafetyAction::HardDisable)
    {
        EnterFailSafe(reason, false);
        return;
    }
    if (decision.action == LWSafetyAction::HardDisableAndShutdown)
    {
        EnterFailSafe(reason, true);
    }
}

void RL_Real::EnterFailSafe(
    const std::string& reason,
    bool request_shutdown) noexcept
{
    std::lock_guard<std::mutex> lock(fail_safe_mutex_);
    const bool first_hard_disable =
        !fatal_error_latched_.exchange(true, std::memory_order_acq_rel);

    if (first_hard_disable)
    {
        std::cerr << LOGGER::ERROR << reason << std::endl;
        std::cerr << LOGGER::ERROR
                  << "[Safety] Commands permanently latched off; restart is required"
                  << std::endl;

        SendEmergencyDisableBurst();
    }

    if (!request_shutdown
        || shutdown_requested_.exchange(true, std::memory_order_acq_rel))
    {
        return;
    }
    try
    {
        if (rclcpp::ok())
        {
            rclcpp::shutdown();
        }
    }
    catch (const std::exception& exception)
    {
        std::cerr << LOGGER::ERROR << "[Safety] Failed to request ROS shutdown: "
                  << exception.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << LOGGER::ERROR << "[Safety] Failed to request ROS shutdown" << std::endl;
    }
}

bool RL_Real::HandleSensorReadiness()
{
    if (sensor_readiness_status_.decision == SensorReadinessDecision::Ready)
    {
        if (!sensor_ready_logged_)
        {
            std::cout << LOGGER::INFO
                      << "[Safety] Fresh IMU and bilateral motor feedback confirmed; control is ready"
                      << std::endl;
            sensor_ready_logged_ = true;
        }
        return true;
    }

    const std::string missing_sources = sensor_readiness_status_.missingSources();
    if (sensor_readiness_status_.decision == SensorReadinessDecision::FaultLatched)
    {
        ApplySafetyEvent(
            LWSafetyEvent::SensorDataStale,
            "[Safety] Required runtime data became stale: " + missing_sources);
        return false;
    }

    const auto now = SafetyClock::now();
    const bool log_due =
        missing_sources != last_missing_sources_
        || last_readiness_log_time_ == SafetyClock::time_point{}
        || now - last_readiness_log_time_ >= 1s;
    if (log_due)
    {
        std::cout << LOGGER::WARNING
                  << "[Safety] Motors remain disabled; waiting for fresh: "
                  << missing_sources << std::endl;
        last_missing_sources_ = missing_sources;
        last_readiness_log_time_ = now;
    }

    const LWSendResult disable_result = disable_lw_robot();
    if (!disable_result.complete())
    {
        ApplySafetyEvent(
            LWSafetyEvent::WaitingDisableIncomplete,
            "[Safety] Waiting-state disable command was incomplete: "
            + disable_result.failureSummary());
    }
    return false;
}

bool RL_Real::ValidateFeedbackAndAttitude()
{
    const size_t num_dofs =
        static_cast<size_t>(this->params.Get<int>("num_of_dofs"));
    const LWValidationResult feedback_result =
        LWValidateFeedbackState(this->robot_state, num_dofs);
    if (!feedback_result.valid())
    {
        ApplySafetyEvent(
            LWSafetyEvent::FeedbackInvalid,
            "[Safety] Invalid LW feedback: "
            + feedback_result.failureDescription());
        return false;
    }

    if (!this->fsm.current_state_)
    {
        ApplySafetyEvent(
            LWSafetyEvent::FsmStateMissing,
            "[Safety] LW FSM has no current state");
        return false;
    }

    constexpr float attitude_threshold_degrees = 75.0f;
    const std::string& state_name =
        this->fsm.current_state_->GetStateName();
    const LWAttitudeValidationResult attitude_result =
        LWValidateAttitude(
            state_name,
            this->robot_state.imu.quaternion,
            attitude_threshold_degrees);
    if (!attitude_result.safe)
    {
        ApplySafetyEvent(
            LWSafetyEvent::AttitudeLimitExceeded,
            "[Safety] LW attitude limit exceeded in " + state_name + ": "
            + attitude_result.failureDescription(attitude_threshold_degrees));
        return false;
    }
    return true;
}

bool RL_Real::ValidateCommandForSend(const RobotCommand<float>& command)
{
    const size_t num_dofs =
        static_cast<size_t>(this->params.Get<int>("num_of_dofs"));
    const LWValidationResult command_result =
        LWValidateRobotCommand(command, num_dofs);
    if (!command_result.valid())
    {
        ApplySafetyEvent(
            LWSafetyEvent::RobotCommandInvalid,
            "[Safety] Invalid LW command: "
            + command_result.failureDescription());
        return false;
    }
    return true;
}

void RL_Real::ApplyControlledFallbackCommand()
{
    this->control.x = 0.0f;
    this->control.y = 0.0f;
    this->control.yaw = 0.0f;
    this->control.ClearInput();

    if (!controlled_fallback_applied_.exchange(
            true, std::memory_order_acq_rel))
    {
        DeactivateLWPolicy();
    }
    if (this->fsm.current_state_
        && this->fsm.current_state_->GetStateName()
            != "RLFSMStatePassive")
    {
        this->fsm.RequestStateChange("RLFSMStatePassive");
    }

    LWBuildPassiveDampingCommand(
        this->robot_state,
        this->robot_command,
        static_cast<size_t>(this->params.Get<int>("num_of_dofs")));
}

void RL_Real::RobotControl()
{
#ifdef CONTROL_TIME_PRINT
    auto t_start = std::chrono::high_resolution_clock::now();
#endif
    ApplyPendingInput();
    if (this->terminal_keyboard_)
    {
        this->KeyboardInterface(
            this->terminal_keyboard_->descriptor(),
            false);
    }
    if (control_timing_degraded_latched_.load(std::memory_order_acquire))
    {
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
    }
    ApplyJoystickFaultGate();
    this->GetState(&this->robot_state);
    if (fatal_error_latched_.load(std::memory_order_acquire))
    {
        return;
    }
    if (!HandleSensorReadiness())
    {
        return;
    }
    if (!ValidateFeedbackAndAttitude())
    {
        return;
    }

    // Motor-board faults make host-side controlled fallback unverifiable.
    // Hard-disable once and keep ROS diagnostics alive for inspection.
    if(this->lw_sdk.MotorsProtect(this->lw_low_state)){
        ApplySafetyEvent(
            LWSafetyEvent::MotorHardwareFault,
            "[Safety] LW motor-board fault reported");
        return;
    }
    
#ifdef CONTROL_TIME_PRINT
    auto t_after_get = std::chrono::high_resolution_clock::now();
#endif

    const bool fallback_before_state_controller =
        safety_supervisor_.controlledFallbackLatched();
    if (fallback_before_state_controller)
    {
        ApplyControlledFallbackCommand();
        if (this->fsm.current_state_
            && this->fsm.current_state_->GetStateName()
                != "RLFSMStatePassive")
        {
            // Complete the requested transition in the control thread. The
            // command is overwritten with damping again below before send.
            this->StateController(&this->robot_state, &this->robot_command);
        }
    }
    else
    {
        this->StateController(&this->robot_state, &this->robot_command);
    }
    if (safety_supervisor_.controlledFallbackLatched())
    {
        // A stale output may be discovered inside StateController(). Replace
        // that cycle's old policy command instead of holding it indefinitely.
        ApplyControlledFallbackCommand();
    }
    if (control_timing_degraded_latched_.load(std::memory_order_acquire))
    {
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
    }
    policy_input_snapshot_.publish(
        {this->robot_state,
         {this->control.x,
          this->control.y,
          this->control.yaw,
          this->control.gait_frequency}});
    // The FSM may enter a protected state during StateController(). Validate
    // again so its first command cannot bypass state-specific attitude checks.
    if (!ValidateFeedbackAndAttitude())
    {
        return;
    }
    if (!ValidateCommandForSend(this->robot_command))
    {
        return;
    }

    this->control.ClearInput();

    this->SetCommand(&this->robot_command);
    if (this->debug_publisher_)
    {
        this->debug_publisher_->publishSnapshot(
            LWDebugSnapshot{
                this->lw_low_state,
                this->lw_low_command,
                {this->robot_state.imu.gyroscope[0],
                 this->robot_state.imu.gyroscope[1],
                 this->robot_state.imu.gyroscope[2]},
                {this->robot_state.imu.quaternion[0],
                 this->robot_state.imu.quaternion[1],
                 this->robot_state.imu.quaternion[2],
                 this->robot_state.imu.quaternion[3]},
                this->control.x,
                this->control.y,
                this->control.yaw});
    }
#ifdef CONTROL_TIME_PRINT
    auto t_end = std::chrono::high_resolution_clock::now();

    double total_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    double read_ms = std::chrono::duration<double, std::milli>(t_after_get - t_start).count();
    static int count = 0;
    if (++count % 50 == 0) {
        std::cout << "[Latency Test] Total Control Pipeline: " << total_ms << "ms "
                  << "(SDK Read: " << read_ms << "ms)" << std::endl;
        if (total_ms > this->params.Get<float>("dt") * 1000.0f) {
            std::cout << LOGGER::WARNING << "!!! Control Overrun !!!" << std::endl;
        }
    }
#endif
}

void RL_Real::RunModel()
{
    if (fatal_error_latched_.load(std::memory_order_acquire)
        || safety_supervisor_.controlledFallbackLatched())
    {
        return;
    }
    const auto activation = LoadLWPolicyActivation();
    if (!activation || !activation->definition)
    {
        return;
    }
    if (!inference_activation_
        || inference_activation_->generation
            != activation->generation)
    {
        inference_activation_ = activation;
        ResetInferenceWorkspace(*activation);
    }

    LWPolicyInputSnapshot policy_input;
    if (!policy_input_snapshot_.read(policy_input))
    {
        return;
    }
    const RobotState<float>& local_state =
        policy_input.robot_state;
    const LWControlSnapshot& local_control =
        policy_input.control;

    inference_motion_reference_ = LoadLWMotionReference();
    const auto observations =
        activation->definition->params.Get<std::vector<std::string>>(
            "observations");
    const bool needs_motion_reference =
        std::find(
            observations.begin(),
            observations.end(),
            "whole_body_tracking/motion_command")
        != observations.end();
    if (needs_motion_reference
        && (!inference_motion_reference_
            || inference_motion_reference_->generation
                != activation->generation))
    {
        return;
    }
    const auto output_source_time =
        std::chrono::steady_clock::now();

#ifdef FOWARD_TIME_PRINT
    auto t_start = std::chrono::high_resolution_clock::now();
#endif
    ++inference_frame_;
    inference_obs_.ang_vel = local_state.imu.gyroscope;
    inference_obs_.commands =
        joystick_fault_latch_.faulted()
        ? std::vector<float>{0.0f, 0.0f, 0.0f}
        : std::vector<float>{
            local_control.x,
            local_control.y,
            local_control.yaw};
    inference_obs_.base_quat = local_state.imu.quaternion;
    inference_obs_.dof_pos = local_state.motor_state.q;
    inference_obs_.dof_vel = local_state.motor_state.dq;

    const YamlParams& policy_params =
        activation->definition->params;
    inference_gait_phase_time_ +=
        policy_params.Get<float>("dt")
        * policy_params.Get<int>("decimation")
        * local_control.gait_frequency;
    if (inference_gait_phase_time_ >= 1.0f)
    {
        inference_gait_phase_time_ -= 1.0f;
    }
    const float command_norm = std::sqrt(
        local_control.x * local_control.x
        + local_control.y * local_control.y
        + local_control.yaw * local_control.yaw);
    const float is_moving = command_norm > 0.1f ? 1.0f : 0.0f;
    inference_obs_.gait_phase = {
        is_moving * std::sin(
            2 * static_cast<float>(M_PI)
            * inference_gait_phase_time_),
        is_moving * std::cos(
            2 * static_cast<float>(M_PI)
            * inference_gait_phase_time_)};

    inference_obs_.actions = Forward();
    if (fatal_error_latched_.load(std::memory_order_acquire)
        || safety_supervisor_.controlledFallbackLatched())
    {
        return;
    }
#ifdef FOWARD_TIME_PRINT
    auto t_end = std::chrono::high_resolution_clock::now();
    double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    float rl_dt_ms =
        policy_params.Get<float>("dt")
        * policy_params.Get<int>("decimation")
        * 1000.0f;
        
    static double max_ms = 0;
    static int count = 0;
    if (elapsed_ms > max_ms) max_ms = elapsed_ms;
    if (elapsed_ms > rl_dt_ms) {
        std::cout << "  <-- !!! TIMEOUT !!!" << std::endl;
    }

    if (++count % 100 == 0) {
        std::cout << "[RL Inference] Curr: " << elapsed_ms << "ms, Max: " << max_ms
                  << "ms, Limit: " << rl_dt_ms << "ms";
        std::cout << std::endl;
    }
#endif

    ComputeLWOutput(
        policy_params,
        inference_obs_,
        inference_obs_.actions,
        inference_output_dof_pos_,
        inference_output_dof_vel_,
        inference_output_dof_tau_);
    const LWValidationResult output_result =
        LWValidatePolicyOutputs(
            inference_output_dof_pos_,
            inference_output_dof_vel_,
            inference_output_dof_tau_,
            static_cast<size_t>(
                policy_params.Get<int>("num_of_dofs")));
    if (!output_result.valid())
    {
        ApplySafetyEvent(
            LWSafetyEvent::PolicyOutputInvalid,
            "[Safety] Invalid LW policy output: "
            + output_result.failureDescription());
        return;
    }

    if (TorqueProtect(inference_output_dof_tau_, policy_params))
    {
        safety_supervisor_.report(LWSafetyEvent::TorqueLimitWarning);
    }
    PublishLWPolicyOutput(
        {activation->generation,
         0,
         inference_frame_,
         output_source_time,
         inference_output_dof_pos_,
         inference_output_dof_vel_,
         inference_output_dof_tau_});
    PublishLWPolicyProgress(
        activation->generation,
        inference_frame_);

#ifdef CSV_LOGGER
    this->CSVLogger(
        inference_output_dof_tau_,
        local_state.motor_state.tau_est,
        inference_obs_.dof_pos,
        inference_output_dof_pos_,
        inference_obs_.dof_vel);
#endif
}

std::vector<float> RL_Real::Forward()
{
    if (!inference_activation_
        || !inference_activation_->definition
        || !inference_activation_->definition->model)
    {
        return {};
    }
    const auto& definition =
        *inference_activation_->definition;
    const auto& policy_params = definition.params;
    const auto clamped_obs = ComputeLWObservation(
        policy_params,
        inference_obs_,
        inference_obs_dims_,
        inference_motion_reference_.get(),
        inference_frame_,
        inference_activation_->motion_length);

    std::vector<float> actions;
    const auto history_indices =
        policy_params.Get<std::vector<int>>(
            "observations_history");
    if (!history_indices.empty())
    {
        if (inference_frame_ == 1)
        {
            inference_history_obs_buf_.reset(
                {0},
                clamped_obs);
        }
        inference_history_obs_buf_.insert(clamped_obs);
        inference_history_obs_ =
            inference_history_obs_buf_.get_obs_vec(
                history_indices);
        actions = definition.model->forward(
            {inference_history_obs_});
    }
    else
    {
        actions = definition.model->forward({clamped_obs});
    }

    const size_t num_dofs =
        static_cast<size_t>(
            policy_params.Get<int>("num_of_dofs"));
    const LWValidationResult action_result =
        LWValidatePolicyActions(actions, num_dofs);
    if (!action_result.valid())
    {
        ApplySafetyEvent(
            LWSafetyEvent::PolicyActionInvalid,
            "[Safety] Invalid LW policy action: "
            + action_result.failureDescription());
        return {};
    }

    const auto upper =
        policy_params.Get<std::vector<float>>(
            "clip_actions_upper");
    const auto lower =
        policy_params.Get<std::vector<float>>(
            "clip_actions_lower");
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
            ApplySafetyEvent(
                LWSafetyEvent::PolicyConfigurationInvalid,
                "[Safety] Invalid LW action clipping configuration: "
                + clip_result.failureDescription());
            return {};
        }
        return clamp(actions, lower, upper);
    }
    return actions;
}

void RL_Real::ResetInferenceWorkspace(
    const LWPolicyActivation& activation)
{
    const YamlParams& policy_params =
        activation.definition->params;
    const size_t num_dofs = static_cast<size_t>(
        policy_params.Get<int>("num_of_dofs"));

    inference_frame_ = 0;
    inference_gait_phase_time_ = 0.0f;
    inference_motion_reference_.reset();
    inference_obs_ = {};
    inference_obs_.lin_vel = {0.0f, 0.0f, 0.0f};
    inference_obs_.ang_vel = {0.0f, 0.0f, 0.0f};
    inference_obs_.gravity_vec = {0.0f, 0.0f, -1.0f};
    inference_obs_.commands = {0.0f, 0.0f, 0.0f};
    inference_obs_.base_quat = {1.0f, 0.0f, 0.0f, 0.0f};
    inference_obs_.dof_pos =
        policy_params.Get<std::vector<float>>(
            "default_dof_pos");
    inference_obs_.dof_vel.assign(num_dofs, 0.0f);
    inference_obs_.actions.assign(num_dofs, 0.0f);
    inference_obs_.gait_phase = {0.0f, 1.0f};
    inference_output_dof_pos_ = inference_obs_.dof_pos;
    inference_output_dof_vel_.assign(num_dofs, 0.0f);
    inference_output_dof_tau_.assign(num_dofs, 0.0f);
    inference_history_obs_.clear();

    ComputeLWObservation(
        policy_params,
        inference_obs_,
        inference_obs_dims_,
        nullptr,
        0,
        activation.motion_length);
    const auto history_indices =
        policy_params.Get<std::vector<int>>(
            "observations_history");
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
            policy_params.Get<std::string>(
                "observations_history_priority"));
    }
}

void RL_Real::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    auto sample = std::make_shared<TimedImuSample>();
    sample->message = std::move(msg);
    sample->received_at = SafetyClock::now();
    received_imu_sample_.set(std::move(sample));
}

void RL_Real::GetState(RobotState<float> *state)
{
    const LWFeedbackUpdate feedback_update = this->lw_sdk.RecvFdData(this->lw_low_state);
    const auto now = SafetyClock::now();
    if (feedback_update.readFailed())
    {
        ApplySafetyEvent(
            LWSafetyEvent::FeedbackReadFailed,
            "[Safety] LW feedback read failed: " + feedback_update.failureSummary());
        return;
    }
    if (feedback_update.hasParserErrors()
        && (last_serial_diagnostic_log_time_ == SafetyClock::time_point{}
            || now - last_serial_diagnostic_log_time_ >= 1s))
    {
        safety_supervisor_.report(LWSafetyEvent::FeedbackParserError);
        std::cout << LOGGER::WARNING << "[Serial] Feedback parser errors: "
                  << feedback_update.parserErrorSummary() << std::endl;
        last_serial_diagnostic_log_time_ = now;
    }
    if (feedback_update.right.updated)
    {
        sensor_readiness_monitor_.markRightFeedbackReceived(now);
    }
    if (feedback_update.left.updated)
    {
        sensor_readiness_monitor_.markLeftFeedbackReceived(now);
    }

    std::shared_ptr<TimedImuSample> imu_sample;
    received_imu_sample_.get(imu_sample);
    if (imu_sample && imu_sample->message)
    {
        sensor_readiness_monitor_.markImuReceived(imu_sample->received_at);
    }

    sensor_readiness_status_ = sensor_readiness_monitor_.evaluate(now);
    if (!sensor_readiness_status_.allFresh())
    {
        return;
    }

    const auto& imu = *imu_sample->message;

    state->imu.quaternion[0] = imu.orientation.w;
    state->imu.quaternion[1] = imu.orientation.x;
    state->imu.quaternion[2] = imu.orientation.y;
    state->imu.quaternion[3] = imu.orientation.z;

// ----------------- 新增的宏定义滤波逻辑 -----------------
#ifdef ENABLE_IMU_GYRO_FILTER
    if (this->is_first_imu_) {
        // 第一帧数据不滤波，直接赋值，作为滤波器的初始基准
        this->filtered_gyro_[0] = imu.angular_velocity.x;
        this->filtered_gyro_[1] = imu.angular_velocity.y;
        this->filtered_gyro_[2] = imu.angular_velocity.z;
        this->is_first_imu_ = false;
    } else {
        // 一阶低通滤波公式： y(t) = alpha * x(t) + (1 - alpha) * y(t-1)
        this->filtered_gyro_[0] = this->gyro_filter_alpha_ * imu.angular_velocity.x + (1.0f - this->gyro_filter_alpha_) * this->filtered_gyro_[0];
        this->filtered_gyro_[1] = this->gyro_filter_alpha_ * imu.angular_velocity.y + (1.0f - this->gyro_filter_alpha_) * this->filtered_gyro_[1];
        this->filtered_gyro_[2] = this->gyro_filter_alpha_ * imu.angular_velocity.z + (1.0f - this->gyro_filter_alpha_) * this->filtered_gyro_[2];
    }
    
    // 将滤波后的结果赋值给 state
    state->imu.gyroscope[0] = this->filtered_gyro_[0];
    state->imu.gyroscope[1] = this->filtered_gyro_[1];
    state->imu.gyroscope[2] = this->filtered_gyro_[2];
#else
    // 原有逻辑（未开启滤波）
    state->imu.gyroscope[0] = imu.angular_velocity.x;
    state->imu.gyroscope[1] = imu.angular_velocity.y;
    state->imu.gyroscope[2] = imu.angular_velocity.z;
#endif
// --------------------------------------------------------

    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        state->motor_state.q[i] = this->lw_low_state.motorState[this->params.Get<std::vector<int>>("joint_mapping")[i]].pos_now;
        state->motor_state.dq[i] = this->lw_low_state.motorState[this->params.Get<std::vector<int>>("joint_mapping")[i]].vel_now;
        state->motor_state.tau_est[i] = this->lw_low_state.motorState[this->params.Get<std::vector<int>>("joint_mapping")[i]].tau_now;
    }
}

void RL_Real::SetCommand(const RobotCommand<float> *command)
{
    if (command == nullptr || !ValidateCommandForSend(*command))
    {
        if (command == nullptr)
        {
            ApplySafetyEvent(
                LWSafetyEvent::NullRobotCommand,
                "[Safety] Null LW command");
        }
        return;
    }

    LWSendResult send_result;
    const bool command_sent = command_gate_.sendIfOpen([this, command, &send_result]()
    {
        auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
        auto wheel_indices = this->params.Get<std::vector<int>>("wheel_indices");
        int num_dofs = this->params.Get<int>("num_of_dofs");

        for (int i = 0; i < num_dofs; ++i)
        {
            int motor_id = joint_mapping[i];

            this->lw_low_command.motorCmd[motor_id].Kp = command->motor_command.kp[i];
            this->lw_low_command.motorCmd[motor_id].Kd = command->motor_command.kd[i];

            bool is_wheel = false;
            for (int k : wheel_indices)
            {
                if (i == k)
                {
                    is_wheel = true;
                    break;
                }
            }

            if (is_wheel)
            {
                this->lw_low_command.motorCmd[motor_id].action_set = command->motor_command.dq[i];
            }
            else
            {
                this->lw_low_command.motorCmd[motor_id].action_set = command->motor_command.q[i];
            }
        }

        this->lw_low_command.motors_disable = false;
        send_result = this->lw_sdk.SendCmdData(this->lw_low_command);
    });

    // The gate mutex has been released here. Entering fail-safe while holding
    // it would deadlock when the emergency disable tries to serialize a send.
    if (command_sent && !send_result.complete())
    {
        ApplySafetyEvent(
            LWSafetyEvent::ControlCommandIncomplete,
            "[Safety] Control command was incomplete: " + send_result.failureSummary());
    }
}

void RL_Real::SetupSysJoystick(const std::string& device, int bits)
{
    this->sys_js = std::make_unique<LWJoystickDevice>(device);
    if (!this->sys_js->isFound())
    {
        std::cout << LOGGER::ERROR << "Joystick [" << device << "] open failed." << std::endl;
        LatchJoystickFault(
            {LWJoystickSampleStatus::Disconnected, EBADF, -1});
    }

    this->sys_js_max_value = (1 << (bits - 1));
}

void RL_Real::LatchJoystickFault(
    const LWJoystickSampleResult& result) noexcept
{
    safety_supervisor_.report(LWSafetyEvent::JoystickUnavailable);
    LWClearJoystickState(this->sys_js_button, this->sys_js_axis);
    joystick_input_mailbox_.clear(Input::Gamepad::None);
    this->sys_js_active = false;

    if (joystick_fault_latch_.latch())
    {
        std::cerr << LOGGER::ERROR
                  << "[Safety] Joystick input latched unavailable"
                  << ", status=" << static_cast<int>(result.status)
                  << ", errno=" << result.error_number
                  << ", bytes=" << result.bytes_read
                  << ". Velocity commands will remain zero; motor control and "
                     "the current FSM remain active. Restart is required to "
                     "restore joystick input."
                  << std::endl;
    }
}

void RL_Real::ApplyPendingInput()
{
    const auto input = joystick_input_mailbox_.read();
    this->control.x = input.x;
    this->control.y = input.y;
    this->control.yaw = input.yaw;
    if (input.event_sequence != consumed_gamepad_sequence_)
    {
        consumed_gamepad_sequence_ = input.event_sequence;
        this->control.SetGamepad(input.event);
    }
}

void RL_Real::ApplyJoystickFaultGate() noexcept
{
    if (!joystick_fault_latch_.faulted())
    {
        return;
    }

    this->control.x = 0.0f;
    this->control.y = 0.0f;
    this->control.yaw = 0.0f;
    this->control.current_gamepad = Input::Gamepad::None;
    this->control.last_gamepad = Input::Gamepad::None;
}

void RL_Real::GetSysJoystick()
{
    LWBeginJoystickCycle(this->sys_js_button);

    if (joystick_fault_latch_.faulted())
    {
        return;
    }

    if (!this->sys_js || !this->sys_js->isFound())
    {
        LatchJoystickFault(
            {LWJoystickSampleStatus::Disconnected, EBADF, -1});
        return;
    }

    while (true)
    {
        const LWJoystickSampleResult sample_result =
            this->sys_js->sample(&this->sys_js_event);
        if (sample_result.status == LWJoystickSampleStatus::NoData)
        {
            break;
        }
        if (!sample_result.hasEvent())
        {
            LatchJoystickFault(sample_result);
            return;
        }

        const LWJoystickEventResult event_result =
            LWApplyJoystickEvent(
                this->sys_js_event,
                this->sys_js_button,
                this->sys_js_axis,
                this->sys_js_max_value,
                this->axis_deadzone);
        if (event_result == LWJoystickEventResult::ButtonOutOfRange
            || event_result == LWJoystickEventResult::AxisOutOfRange)
        {
            static size_t invalid_event_count = 0;
            ++invalid_event_count;
            if (invalid_event_count == 1 || invalid_event_count % 100 == 0)
            {
                std::cerr << LOGGER::WARNING
                          << "[Safety] Ignoring out-of-range joystick event"
                          << ", type=" << static_cast<int>(this->sys_js_event.type)
                          << ", number=" << static_cast<int>(this->sys_js_event.number)
                          << ", occurrences=" << invalid_event_count
                          << std::endl;
            }
        }
        else if (event_result == LWJoystickEventResult::InvalidConfiguration)
        {
            LatchJoystickFault(
                {LWJoystickSampleStatus::Error, EINVAL, 0});
            return;
        }
    }

    // 修改为自己的手柄映射
    if (this->sys_js_button[0].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::A);
    if (this->sys_js_button[1].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::B);
    if (this->sys_js_button[3].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::X);
    if (this->sys_js_button[4].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::Y);
    if (this->sys_js_button[6].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB);
    if (this->sys_js_button[7].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::RB);
    if (this->sys_js_button[13].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::LStick);
    if (this->sys_js_button[14].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::RStick);
    if (this->sys_js_axis[7] < 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::DPadUp);
    if (this->sys_js_axis[7] > 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::DPadDown);
    if (this->sys_js_axis[6] < 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::DPadLeft);
    if (this->sys_js_axis[6] > 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::DPadRight);
    if (this->sys_js_button[6].pressed && this->sys_js_button[0].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB_A);
    if (this->sys_js_button[6].pressed && this->sys_js_button[1].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB_B);
    if (this->sys_js_button[6].pressed && this->sys_js_button[3].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB_X);
    if (this->sys_js_button[6].pressed && this->sys_js_button[4].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB_Y);
    if (this->sys_js_button[6].pressed && this->sys_js_button[13].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB_LStick);
    if (this->sys_js_button[6].pressed && this->sys_js_button[14].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB_RStick);
    if (this->sys_js_button[6].pressed && this->sys_js_axis[7] < 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB_DPadUp);
    if (this->sys_js_button[6].pressed && this->sys_js_axis[7] > 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB_DPadDown);
    if (this->sys_js_button[6].pressed && this->sys_js_axis[6] > 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB_DPadRight);
    if (this->sys_js_button[6].pressed && this->sys_js_axis[6] < 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB_DPadLeft);
    if (this->sys_js_button[7].pressed && this->sys_js_button[0].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::RB_A);
    if (this->sys_js_button[7].pressed && this->sys_js_button[1].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::RB_B);
    if (this->sys_js_button[7].pressed && this->sys_js_button[3].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::RB_X);
    if (this->sys_js_button[7].pressed && this->sys_js_button[4].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::RB_Y);
    if (this->sys_js_button[7].pressed && this->sys_js_button[13].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::RB_LStick);
    if (this->sys_js_button[7].pressed && this->sys_js_button[14].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::RB_RStick);
    if (this->sys_js_button[7].pressed && this->sys_js_axis[7] < 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::RB_DPadUp);
    if (this->sys_js_button[7].pressed && this->sys_js_axis[7] > 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::RB_DPadDown);
    if (this->sys_js_button[7].pressed && this->sys_js_axis[6] > 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::RB_DPadRight);
    if (this->sys_js_button[7].pressed && this->sys_js_axis[6] < 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::RB_DPadLeft);
    if (this->sys_js_button[6].pressed && this->sys_js_button[7].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB_RB);

    // 通过sys_js_max_value将各项指令归一化
    // float ly = -float(this->sys_js_axis[1]) / float(this->sys_js_max_value);
    // float lx = -float(this->sys_js_axis[0]) / float(this->sys_js_max_value);
    // float rx = -float(this->sys_js_axis[3]) / float(this->sys_js_max_value);

    float ly = (-float(this->sys_js_axis[1]) / float(this->sys_js_max_value)) * this->params.Get<std::vector<float>>("vel_command")[0];
    float lx = (-float(this->sys_js_axis[0]) / float(this->sys_js_max_value)) * this->params.Get<std::vector<float>>("vel_command")[1];
    float rx = (-float(this->sys_js_axis[2]) / float(this->sys_js_max_value)) * this->params.Get<std::vector<float>>("vel_command")[2];

    bool has_input = (ly != 0.0f || lx != 0.0f || rx != 0.0f);

    if (has_input)
    {
        joystick_input_mailbox_.publishVelocity(ly, lx, rx);
        this->sys_js_active = true;
    }
    else if (this->sys_js_active)
    {
        joystick_input_mailbox_.publishVelocity(
            0.0f,
            0.0f,
            0.0f);
        this->sys_js_active = false;
    }

    // if (this->control.current_gamepad == Input::Gamepad::DPadUp )
    // {
    //     if (!this->control.dpad_handled) { // 如果还没处理过
    //         this->control.gait_frequency += 0.1f;
    //         this->control.dpad_handled = true; // 标记为已处理
    //     }
    // }
    // else if (this->control.current_gamepad == Input::Gamepad::DPadDown )
    // {
    //     if (!this->control.dpad_handled) { // 如果还没处理过
    //         this->control.gait_frequency -= 0.1f;
    //         this->control.dpad_handled = true; // 标记为已处理
    //     }
    // }
    // else 
    // {
    //     this->control.dpad_handled = false;
    // }
}

int main(int argc, char **argv)
{
    const bool verify_deployment_only =
        argc == 2
        && std::string(argv[1]) == "--verify-deployment-only";
    bool ros_initialized = false;
    try
    {
        const std::filesystem::path package_share =
            ament_index_cpp::get_package_share_directory("rl_sar");
        const std::filesystem::path running_executable =
            std::filesystem::canonical("/proc/self/exe");
        const LWDeploymentBundleInfo deployment =
            LWDeploymentBundle::Verify(
                package_share,
                running_executable,
                RL_SAR_SOURCE_COMMIT);
        std::cout << LOGGER::INFO
                  << "[Deployment] Verified LW bundle for source commit "
                  << deployment.source_commit
                  << " at " << deployment.bundle_root << std::endl;
        if (verify_deployment_only)
        {
            std::cout << LOGGER::INFO
                      << "[Deployment] Verification-only check passed"
                      << std::endl;
            return 0;
        }

        rclcpp::init(argc, argv);
        ros_initialized = true;
        auto rl_sar = std::make_shared<RL_Real>(
            argc, argv, deployment.policy_root.string());
        rclcpp::spin(rl_sar->ros2_node);
        rclcpp::shutdown();
        return 0;
    }
    catch (const std::exception& exception)
    {
        std::cerr << LOGGER::ERROR << "[Startup] rl_real_LW failed: "
                  << exception.what() << std::endl;
        if (ros_initialized && rclcpp::ok())
        {
            rclcpp::shutdown();
        }
        return 1;
    }
}
