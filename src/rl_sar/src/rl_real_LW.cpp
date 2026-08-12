
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
    const std::string& policy_root,
    LWStartupDisableGuard& startup_disable)
    : startup_disable_(&startup_disable)
{
    (void)argc;
    (void)argv;
    startup_disable_->requireHealthy();
    runtime_core_.bind(
        *this,
        [this](const LWSafetyDecision& decision, const std::string& reason)
        {
            if (decision.action == LWSafetyAction::HardDisable)
            {
                EnterFailSafe(reason, false);
            }
            else if (decision.action
                     == LWSafetyAction::HardDisableAndShutdown)
            {
                EnterFailSafe(reason, true);
            }
        });
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
    const LWSDK::Duration runtime_serial_write_timeout =
        std::chrono::duration_cast<LWSDK::Duration>(
            std::chrono::duration<float>(serial_write_timeout_seconds));

    // 提前加载所有的模型到内存
    const auto preload_model = [this](const std::string& policy)
    {
        startup_disable_->requireHealthy();
        this->PreloadModel(policy);
        startup_disable_->requireHealthy();
    };
    const auto preload_context = [this](const std::string& policy)
    {
        startup_disable_->requireHealthy();
        this->PreloadLWPolicyContext(policy);
        startup_disable_->requireHealthy();
    };
    preload_model(this->robot_name + "/robot_lab/leg_loco");
    preload_model(this->robot_name + "/robot_lab/wheel_loco");
    preload_model(this->robot_name + "/robot_lab/leg_to_wheel");
    preload_model(this->robot_name + "/robot_lab/wheel_to_leg");
    preload_context(this->robot_name + "/robot_lab/leg_loco");
    preload_context(this->robot_name + "/robot_lab/wheel_loco");
    preload_context(this->robot_name + "/robot_lab/leg_to_wheel");
    preload_context(this->robot_name + "/robot_lab/wheel_to_leg");

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
    startup_disable_->sdk().InitCmdData(this->lw_low_command);
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    this->InitOutputs();
    this->InitControl();
    this->control.gait_frequency = this->params.Get<std::vector<float>>("gait_command")[0];
    this->gait_phase_time = 0.0f;
    runtime_core_.publishInitialPolicyInput();

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

    startup_disable_->requireHealthy();
    startup_disable_->handOffToRuntime(runtime_serial_write_timeout);
    try
    {
        this->loop_joystick->start();
        this->loop_rl->start();
        this->loop_control->start();
    }
    catch (...)
    {
        runtime_core_.reportSafetyEvent(LWSafetyEvent::StartupLoopStartFailed);
        startup_disable_->commandGate().close();
        this->loop_control->shutdown();
        this->loop_rl->shutdown();
        this->loop_joystick->shutdown();
        SendEmergencyDisableBurst();
        throw;
    }
}

RL_Real::~RL_Real()
{
    runtime_core_.reportSafetyEvent(LWSafetyEvent::NormalShutdown);
    // Close the gate before joining so no new enable command can race with the
    // final disable frame. Stop the command producer first.
    startup_disable_->commandGate().close();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
    this->loop_joystick->shutdown();
    this->terminal_keyboard_.reset();
    startup_disable_->finalize();
    std::cout << LOGGER::INFO << "RL_Real exit" << std::endl;
}

void RL_Real::RuntimeDiagnosticsCallback()
{
    const LWSafetySnapshot safety = runtime_core_.safetySnapshot().decision;
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
    return startup_disable_->sendDisable(latch_commands);
}

void RL_Real::HandleLoopError(const std::string& loop_name, std::exception_ptr error) noexcept
{
    runtime_core_.handleLoopError(
        loop_name,
        error,
        [this]()
        {
            LatchJoystickFault(
                {LWJoystickSampleStatus::Error, EFAULT, -1});
        });
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

    if (level == LoopTimingLevel::Degraded
        || level == LoopTimingLevel::Fatal)
    {
        control_timing_degraded_latched_.store(
            true, std::memory_order_release);
        runtime_core_.handleControlTiming(
            level == LoopTimingLevel::Fatal,
            timing.missed_deadlines);
    }
}

void RL_Real::SendEmergencyDisableBurst() noexcept
{
    startup_disable_->sendEmergencyDisableBurst();
}

void RL_Real::HandleLWPolicyOutputFault(
    LWPolicyOutputStatus status) noexcept
{
    runtime_core_.handlePolicyOutputFault(status);
}

void RL_Real::ApplySafetyEvent(
    LWSafetyEvent event,
    const std::string& reason) noexcept
{
    runtime_core_.reportSafetyEvent(event, reason);
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

void RL_Real::RobotControl()
{
#ifdef CONTROL_TIME_PRINT
    auto t_start = std::chrono::high_resolution_clock::now();
#endif
    runtime_core_.runControlCycle(
        LWControlCycleHooks{
            [this]()
            {
                ApplyPendingInput();
                ApplyJoystickFaultGate();
            },
            [this]()
            {
                if (terminal_keyboard_)
                {
                    this->KeyboardInterface(
                        terminal_keyboard_->descriptor(),
                        false);
                }
            },
            [this]() { return HandleSensorReadiness(); },
            [this]()
            {
                if (!startup_disable_->sdk().MotorsProtect(lw_low_state))
                {
                    return true;
                }
                ApplySafetyEvent(
                    LWSafetyEvent::MotorHardwareFault,
                    "[Safety] LW motor-board fault reported");
                return false;
            },
            {},
            {},
            [this]()
            {
                if (!debug_publisher_)
                {
                    return;
                }
                debug_publisher_->publishSnapshot(
                    LWDebugSnapshot{
                        lw_low_state,
                        lw_low_command,
                        {robot_state.imu.gyroscope[0],
                         robot_state.imu.gyroscope[1],
                         robot_state.imu.gyroscope[2]},
                        {robot_state.imu.quaternion[0],
                         robot_state.imu.quaternion[1],
                         robot_state.imu.quaternion[2],
                         robot_state.imu.quaternion[3]},
                        control.x,
                        control.y,
                        control.yaw});
            }});
#ifdef CONTROL_TIME_PRINT
    auto t_end = std::chrono::high_resolution_clock::now();

    double total_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    static int count = 0;
    if (++count % 50 == 0) {
        std::cout << "[Latency Test] Total Control Pipeline: " << total_ms << "ms"
                  << std::endl;
        if (total_ms > this->params.Get<float>("dt") * 1000.0f) {
            std::cout << LOGGER::WARNING << "!!! Control Overrun !!!" << std::endl;
        }
    }
#endif
}

void RL_Real::RunModel()
{
    LWInferenceCycleHooks hooks;
#ifdef CSV_LOGGER
    hooks.after_publish = [this](
        const std::vector<float>& torque,
        const RobotState<float>& state,
        const Observations<float>& observations,
        const std::vector<float>& positions,
        const std::vector<float>&)
    {
        CSVLogger(
            torque,
            state.motor_state.tau_est,
            observations.dof_pos,
            positions,
            observations.dof_vel);
    };
#endif
    runtime_core_.runInferenceCycle(
        joystick_fault_latch_.faulted(),
        hooks);
}

std::vector<float> RL_Real::Forward()
{
    return runtime_core_.forward();
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
    const LWFeedbackUpdate feedback_update =
        startup_disable_->sdk().RecvFdData(this->lw_low_state);
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
        runtime_core_.reportSafetyEvent(LWSafetyEvent::FeedbackParserError);
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
    if (command == nullptr)
    {
        ApplySafetyEvent(
            LWSafetyEvent::NullRobotCommand,
            "[Safety] Null LW command");
        return;
    }

    LWSendResult send_result;
    const bool command_sent = startup_disable_->commandGate().sendIfOpen(
        [this, command, &send_result]()
        {
            auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
            auto wheel_indices = this->params.Get<std::vector<int>>("wheel_indices");
            int num_dofs = this->params.Get<int>("num_of_dofs");

            for (int i = 0; i < num_dofs; ++i)
            {
                int motor_id = joint_mapping[i];

                this->lw_low_command.motorCmd[motor_id].Kp =
                    command->motor_command.kp[i];
                this->lw_low_command.motorCmd[motor_id].Kd =
                    command->motor_command.kd[i];

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
                    this->lw_low_command.motorCmd[motor_id].action_set =
                        command->motor_command.dq[i];
                }
                else
                {
                    this->lw_low_command.motorCmd[motor_id].action_set =
                        command->motor_command.q[i];
                }
            }

            this->lw_low_command.motors_disable = false;
            send_result =
                startup_disable_->sdk().SendCmdData(this->lw_low_command);
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
    runtime_core_.reportSafetyEvent(LWSafetyEvent::JoystickUnavailable);
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

        // The powered STM32 starts enabled. Establish and maintain a complete
        // bilateral disable output before ROS, input, YAML, model, or heap-
        // allocated RL_Real initialization can fail.
        LWStartupDisableGuard startup_disable;
        rclcpp::init(argc, argv);
        ros_initialized = true;
        auto rl_sar = std::make_shared<RL_Real>(
            argc,
            argv,
            deployment.policy_root.string(),
            startup_disable);
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
