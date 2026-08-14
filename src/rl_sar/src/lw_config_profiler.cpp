#include "fsm.hpp"
#include "lw_config_profile.hpp"
#include "lw_deployment_bundle.hpp"
#include "lw_loop_config.hpp"
#include "lw_imu_ahrs_guard.hpp"
#include "lw_runtime_core.hpp"
#include "motion_loader_lw.hpp"
#include "rl_sdk.hpp"
#include "LW_sdk.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sys/utsname.h>
#include <unistd.h>

#ifndef RL_SAR_SOURCE_COMMIT
#define RL_SAR_SOURCE_COMMIT "unverified"
#endif

using namespace std::chrono_literals;

namespace
{
constexpr const char* kHardwareConfirmation =
    "I_CONFIRM_LW_IS_SUSPENDED_AND_MOTORS_MUST_REMAIN_DISABLED";
constexpr std::size_t kNumDofs = 10;

enum class ProfileMode
{
    HostOnly,
    HardwareObserve,
};

struct ProfileOptions
{
    ProfileMode mode = ProfileMode::HostOnly;
    std::filesystem::path policy_root;
    std::filesystem::path output;
    double duration_seconds = 30.0;
    int cpu = -1;
    int realtime_priority = 0;
    bool require_realtime = false;
    bool hardware_confirmed = false;
    std::string right_port = "/dev/ttyLegRight";
    std::string left_port = "/dev/ttyLegLeft";
    std::string imu_topic = "/imu";
    std::string ahrs_topic = "/euler_angles";
};

struct ProfilePolicyResult
{
    std::string policy;
    LWProfileDistributionSnapshot inference;
    LoopTimingSnapshot control_timing;
    LoopTimingSnapshot inference_timing;
    LoopStartupSnapshot control_startup;
    LoopStartupSnapshot inference_startup;
};

struct HardwareProfileResult
{
    LWProfileTimedSourceSnapshot imu;
    LWProfileTimedSourceSnapshot ahrs;
    LWProfileTimedSourceSnapshot trusted_imu;
    LWProfileDistributionSnapshot imu_ahrs_pair_age;
    LWProfileTimedSourceSnapshot right_feedback;
    LWProfileTimedSourceSnapshot left_feedback;
    LWProfileDistributionSnapshot serial_writes;
    std::uint64_t serial_write_failures = 0;
};

struct ProfileHostIdentity
{
    std::string node;
    std::string system;
    std::string release;
    std::string machine;
};

std::string requireValue(int& index, int argc, char** argv)
{
    if (index + 1 >= argc)
    {
        throw std::invalid_argument(
            std::string("missing value for ") + argv[index]);
    }
    return argv[++index];
}

int parseInt(const std::string& text, const char* name)
{
    std::size_t consumed = 0;
    const int value = std::stoi(text, &consumed);
    if (consumed != text.size())
    {
        throw std::invalid_argument(std::string(name) + " is not an integer");
    }
    return value;
}

double parsePositiveDouble(const std::string& text, const char* name)
{
    std::size_t consumed = 0;
    const double value = std::stod(text, &consumed);
    if (consumed != text.size() || !std::isfinite(value) || value <= 0.0)
    {
        throw std::invalid_argument(
            std::string(name) + " must be a finite positive number");
    }
    return value;
}

void printUsage()
{
    std::cout
        << "Usage: lw_config_profiler --policy-root PATH --output REPORT.json [options]\n"
        << "\n"
        << "Modes:\n"
        << "  --mode host-only          No ROS or hardware access (default)\n"
        << "  --mode hardware-observe   Subscribe to IMU, read motor feedback,\n"
        << "                            and send only motor-disable packets\n"
        << "\n"
        << "Options:\n"
        << "  --duration-seconds N\n"
        << "  --cpu N\n"
        << "  --realtime-priority N\n"
        << "  --require-realtime\n"
        << "  --right-port PATH --left-port PATH\n"
        << "  --imu-topic NAME --ahrs-topic NAME\n"
        << "  --hardware-confirmation " << kHardwareConfirmation << "\n"
        << "\n"
        << "The output path must not already exist. This tool never modifies base.yaml.\n";
}

ProfileOptions parseOptions(int argc, char** argv)
{
    ProfileOptions options;
    for (int index = 1; index < argc; ++index)
    {
        const std::string argument = argv[index];
        if (argument == "--help" || argument == "-h")
        {
            printUsage();
            std::exit(EXIT_SUCCESS);
        }
        if (argument == "--mode")
        {
            const std::string value = requireValue(index, argc, argv);
            if (value == "host-only")
            {
                options.mode = ProfileMode::HostOnly;
            }
            else if (value == "hardware-observe")
            {
                options.mode = ProfileMode::HardwareObserve;
            }
            else
            {
                throw std::invalid_argument("unknown profile mode: " + value);
            }
        }
        else if (argument == "--policy-root")
        {
            options.policy_root = requireValue(index, argc, argv);
        }
        else if (argument == "--output")
        {
            options.output = requireValue(index, argc, argv);
        }
        else if (argument == "--duration-seconds")
        {
            options.duration_seconds = parsePositiveDouble(
                requireValue(index, argc, argv), "duration-seconds");
        }
        else if (argument == "--cpu")
        {
            options.cpu = parseInt(
                requireValue(index, argc, argv), "cpu");
        }
        else if (argument == "--realtime-priority")
        {
            options.realtime_priority = parseInt(
                requireValue(index, argc, argv), "realtime-priority");
        }
        else if (argument == "--require-realtime")
        {
            options.require_realtime = true;
        }
        else if (argument == "--right-port")
        {
            options.right_port = requireValue(index, argc, argv);
        }
        else if (argument == "--left-port")
        {
            options.left_port = requireValue(index, argc, argv);
        }
        else if (argument == "--imu-topic")
        {
            options.imu_topic = requireValue(index, argc, argv);
        }
        else if (argument == "--ahrs-topic")
        {
            options.ahrs_topic = requireValue(index, argc, argv);
        }
        else if (argument == "--hardware-confirmation")
        {
            options.hardware_confirmed =
                requireValue(index, argc, argv) == kHardwareConfirmation;
        }
        else
        {
            throw std::invalid_argument("unknown argument: " + argument);
        }
    }

    if (options.policy_root.empty())
    {
        throw std::invalid_argument("--policy-root is required");
    }
    if (options.output.empty())
    {
        throw std::invalid_argument("--output is required");
    }
    if (std::filesystem::exists(options.output))
    {
        throw std::invalid_argument(
            "refusing to overwrite existing report: "
            + options.output.string());
    }
    if (options.cpu < -1)
    {
        throw std::invalid_argument("--cpu must be -1 or nonnegative");
    }
    if (options.realtime_priority < 0)
    {
        throw std::invalid_argument(
            "--realtime-priority must be nonnegative");
    }
    if (options.require_realtime && options.realtime_priority == 0)
    {
        throw std::invalid_argument(
            "--require-realtime requires a positive real-time priority");
    }
    if (options.mode == ProfileMode::HardwareObserve
        && !options.hardware_confirmed)
    {
        throw std::invalid_argument(
            std::string("hardware-observe requires --hardware-confirmation ")
            + kHardwareConfirmation);
    }
    return options;
}

void emitDistribution(
    std::ostream& output,
    const LWProfileDistributionSnapshot& snapshot)
{
    output << "{\"count\":" << snapshot.count
           << ",\"retained\":" << snapshot.retained
           << ",\"minimum_us\":" << snapshot.minimum_us
           << ",\"mean_us\":" << snapshot.mean_us
           << ",\"p50_us\":" << snapshot.p50_us
           << ",\"p95_us\":" << snapshot.p95_us
           << ",\"p99_us\":" << snapshot.p99_us
           << ",\"p999_us\":" << snapshot.p999_us
           << ",\"maximum_us\":" << snapshot.maximum_us << '}';
}

void emitJsonString(std::ostream& output, const std::string& value)
{
    output << '"';
    for (const unsigned char character : value)
    {
        switch (character)
        {
        case '"': output << "\\\""; break;
        case '\\': output << "\\\\"; break;
        case '\b': output << "\\b"; break;
        case '\f': output << "\\f"; break;
        case '\n': output << "\\n"; break;
        case '\r': output << "\\r"; break;
        case '\t': output << "\\t"; break;
        default:
            if (character < 0x20)
            {
                output << "\\u00" << std::hex << std::setw(2)
                       << std::setfill('0')
                       << static_cast<unsigned int>(character)
                       << std::dec << std::setfill(' ');
            }
            else
            {
                output << static_cast<char>(character);
            }
        }
    }
    output << '"';
}

void emitLoopTiming(std::ostream& output, const LoopTimingSnapshot& timing)
{
    const double average_wakeup_us = timing.cycles == 0
        ? 0.0
        : std::chrono::duration<double, std::micro>(
              timing.total_wakeup_lateness).count()
              / static_cast<double>(timing.cycles);
    output << "{\"cycles\":" << timing.cycles
           << ",\"missed_deadlines\":" << timing.missed_deadlines
           << ",\"skipped_periods\":" << timing.skipped_periods
           << ",\"average_wakeup_us\":" << average_wakeup_us
           << ",\"maximum_wakeup_us\":"
           << std::chrono::duration<double, std::micro>(
                  timing.max_wakeup_lateness).count()
           << ",\"maximum_deadline_lateness_us\":"
           << std::chrono::duration<double, std::micro>(
                  timing.max_deadline_lateness).count()
           << ",\"maximum_execution_us\":"
           << std::chrono::duration<double, std::micro>(
                  timing.max_execution_time).count()
           << ",\"level\":" << static_cast<int>(timing.level) << '}';
}

void emitLoopStartup(
    std::ostream& output,
    const LoopStartupSnapshot& startup)
{
    output << "{\"requested_cpu\":" << startup.requested_cpu_affinity
           << ",\"requested_realtime_priority\":"
           << startup.requested_realtime_priority
           << ",\"affinity_applied\":"
           << (startup.affinity_applied ? "true" : "false")
           << ",\"realtime_applied\":"
           << (startup.realtime_applied ? "true" : "false")
           << ",\"realtime_error\":" << startup.realtime_error << '}';
}

ProfileHostIdentity hostIdentity()
{
    struct utsname info{};
    if (::uname(&info) != 0)
    {
        throw std::runtime_error("failed to read host identity");
    }
    return {
        info.nodename,
        info.sysname,
        info.release,
        info.machine};
}

class ProfilePassiveState final : public RLFSMState
{
public:
    explicit ProfilePassiveState(RL& rl)
        : RLFSMState(rl, "RLFSMStatePassive")
    {
    }

    void Enter() override {}

    void Run() override
    {
        for (std::size_t index = 0; index < kNumDofs; ++index)
        {
            fsm_command->motor_command.q[index] =
                fsm_state->motor_state.q[index];
            fsm_command->motor_command.dq[index] = 0.0f;
            fsm_command->motor_command.tau[index] = 0.0f;
            fsm_command->motor_command.kp[index] = 0.0f;
            fsm_command->motor_command.kd[index] = 5.0f;
        }
    }

    void Exit() override {}
};

class LWConfigProfiler final : public RL
{
public:
    LWConfigProfiler(ProfileOptions options, int argc, char** argv)
        : options_(std::move(options))
    {
        options_.policy_root = std::filesystem::canonical(options_.policy_root);
        policy_assets_ = snapshotPolicyAssets();
        host_identity_ = hostIdentity();
        runtime_core_.bind(
            *this,
            [this](const LWSafetyDecision& decision, const std::string& reason)
            {
                if (decision.severity >= LWSafetySeverity::HardDisable)
                {
                    fail("profile safety event "
                         + std::string(LWSafetyActionName(decision.action))
                         + (reason.empty() ? "" : ": " + reason));
                }
            });
        SetPolicyRoot(options_.policy_root);
        robot_name = "LW";
        ang_vel_axis = "body";
        ReadYaml(robot_name, "base.yaml");
        SetLWBaseRuntimeConfiguration(
            ValidateLWBaseConfiguration(
                params.config_node,
                ResolvePolicyPath("LW/base.yaml")));
        const float imu_ahrs_pair_max_age_seconds =
            params.Get<float>("imu_ahrs_pair_max_age");
        imu_ahrs_guard_.setPairMaxAge(
            std::chrono::duration_cast<LWImuAhrsGuard::Duration>(
                std::chrono::duration<float>(
                    imu_ahrs_pair_max_age_seconds)));
        const float serial_write_timeout_seconds =
            params.Get<float>("serial_write_timeout");
        lw_sdk_.SetWriteTimeout(
            std::chrono::duration_cast<LWSDK::Duration>(
                std::chrono::duration<float>(serial_write_timeout_seconds)));
        try
        {
            if (options_.mode == ProfileMode::HardwareObserve)
            {
                initializeHardwareDisableOutput();
            }

            InitJointNum(kNumDofs);
            InitOutputs();
            InitControl();
            control.gait_frequency =
                params.Get<std::vector<float>>("gait_command")[0];
            initializeSyntheticState();

            passive_state_ = std::make_shared<ProfilePassiveState>(*this);
            fsm.AddState(passive_state_);
            fsm.SetInitialState("RLFSMStatePassive");

            for (const std::string& policy : policyPaths())
            {
                PreloadModel(policy);
                requireDisableKeepaliveHealthy();
                PreloadLWPolicyContext(policy);
                requireDisableKeepaliveHealthy();
            }
            runtime_core_.publishInitialPolicyInput();

            if (options_.mode == ProfileMode::HardwareObserve)
            {
                hardware_started_at_ = std::chrono::steady_clock::now();
                initializeHardwareObservation(argc, argv);
                requireDisableKeepaliveHealthy();
            }
        }
        catch (...)
        {
            stopExecutor();
            stopDisableKeepalive();
            if (hardware_serial_attempted_)
            {
                sendDisableBurst();
            }
            throw;
        }
    }

    ~LWConfigProfiler()
    {
        stopExecutor();
        stopDisableKeepalive();
        if (options_.mode == ProfileMode::HardwareObserve)
        {
            sendDisableBurst();
        }
    }

    int run()
    {
        for (const std::string& policy : policyPaths())
        {
            if (failed_.load(std::memory_order_acquire))
            {
                break;
            }
            runPolicy(policy);
        }
        if (options_.mode == ProfileMode::HardwareObserve)
        {
            hardware_result_ = snapshotHardware();
        }
        stopExecutor();
        if (options_.mode == ProfileMode::HardwareObserve)
        {
            stopDisableKeepalive();
            if (!sendDisableBurst())
            {
                fail("one or more final motor-disable writes failed");
            }
            hardware_result_.serial_writes = serial_writes_.snapshot();
            hardware_result_.serial_write_failures =
                serial_write_failures_.load(std::memory_order_acquire);
        }
        try
        {
            if (!samePolicyAssets(policy_assets_, snapshotPolicyAssets()))
            {
                fail("policy assets changed during profiling");
            }
        }
        catch (const std::exception& exception)
        {
            fail(std::string("failed to revalidate policy assets: ")
                 + exception.what());
        }
        writeReport();
        if (failed_.load(std::memory_order_acquire))
        {
            std::cerr << "[LW Profile] failed: " << failureMessage() << '\n';
            return EXIT_FAILURE;
        }
        std::cout << "[LW Profile] report written to " << options_.output
                  << '\n';
        return EXIT_SUCCESS;
    }

    std::vector<float> Forward() override
    {
        return runtime_core_.forward();
    }

    void GetState(RobotState<float>* state) override
    {
        if (options_.mode == ProfileMode::HostOnly)
        {
            updateSyntheticState();
            *state = synthetic_state_;
            state_ready_.store(true, std::memory_order_release);
            return;
        }

        const LWFeedbackUpdate update = lw_sdk_.RecvFdData(lw_low_state_);
        const auto now = std::chrono::steady_clock::now();
        if (update.readFailed())
        {
            throw std::runtime_error(
                "motor feedback read failed: " + update.failureSummary());
        }
        if (update.right.updated)
        {
            right_feedback_.mark(now);
        }
        if (update.left.updated)
        {
            left_feedback_.mark(now);
        }

        sensor_msgs::msg::Imu::SharedPtr imu;
        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            imu = latest_imu_;
        }
        const bool ready = imu
            && right_feedback_.hasBeenSeen()
            && left_feedback_.hasBeenSeen();
        state_ready_.store(ready, std::memory_order_release);
        if (!ready)
        {
            return;
        }

        state->imu.quaternion = {
            static_cast<float>(imu->orientation.w),
            static_cast<float>(imu->orientation.x),
            static_cast<float>(imu->orientation.y),
            static_cast<float>(imu->orientation.z)};
        state->imu.gyroscope = {
            static_cast<float>(imu->angular_velocity.x),
            static_cast<float>(imu->angular_velocity.y),
            static_cast<float>(imu->angular_velocity.z)};
        const auto mapping = params.Get<std::vector<int>>("joint_mapping");
        for (std::size_t index = 0; index < kNumDofs; ++index)
        {
            const int source = mapping[index];
            state->motor_state.q[index] =
                lw_low_state_.motorState[source].pos_now;
            state->motor_state.dq[index] =
                lw_low_state_.motorState[source].vel_now;
            state->motor_state.tau_est[index] =
                lw_low_state_.motorState[source].tau_now;
        }
    }

    void SetCommand(const RobotCommand<float>*) override
    {
        // Policy and Passive commands are deliberately discarded. Hardware
        // observation sends a separately constructed motors_disable packet.
    }

private:
    static const std::vector<std::string>& policyPaths()
    {
        static const std::vector<std::string> policies = {
            "LW/robot_lab/leg_loco",
            "LW/robot_lab/wheel_loco",
            "LW/robot_lab/leg_to_wheel",
            "LW/robot_lab/wheel_to_leg"};
        return policies;
    }

    static const std::vector<std::string>& policyAssetPaths()
    {
        static const std::vector<std::string> assets = {
            "LW/base.yaml",
            "LW/robot_lab/leg_loco/config.yaml",
            "LW/robot_lab/leg_loco/policy.onnx",
            "LW/robot_lab/leg_to_wheel/config.yaml",
            "LW/robot_lab/leg_to_wheel/leg_to_wheel_transform_60hz.csv",
            "LW/robot_lab/leg_to_wheel/policy.onnx",
            "LW/robot_lab/wheel_loco/config.yaml",
            "LW/robot_lab/wheel_loco/policy.onnx",
            "LW/robot_lab/wheel_to_leg/config.yaml",
            "LW/robot_lab/wheel_to_leg/policy.onnx",
            "LW/robot_lab/wheel_to_leg/wheel_to_leg_transform_60hz.csv"};
        return assets;
    }

    std::vector<LWDeploymentFileRecord> snapshotPolicyAssets() const
    {
        std::vector<LWDeploymentFileRecord> records;
        records.reserve(policyAssetPaths().size());
        for (const std::string& relative : policyAssetPaths())
        {
            const std::filesystem::path asset =
                options_.policy_root / relative;
            if (std::filesystem::canonical(asset) != asset.lexically_normal())
            {
                throw std::runtime_error(
                    "policy asset contains a symbolic-link component: "
                    + asset.string());
            }
            records.push_back({
                relative,
                LWDeploymentBundle::Sha256File(asset)});
        }
        return records;
    }

    static bool samePolicyAssets(
        const std::vector<LWDeploymentFileRecord>& left,
        const std::vector<LWDeploymentFileRecord>& right)
    {
        if (left.size() != right.size())
        {
            return false;
        }
        for (std::size_t index = 0; index < left.size(); ++index)
        {
            if (left[index].path != right[index].path
                || left[index].sha256 != right[index].sha256)
            {
                return false;
            }
        }
        return true;
    }

    void initializeSyntheticState()
    {
        synthetic_state_.motor_state.resize(kNumDofs);
        synthetic_state_.imu.quaternion = {1.0f, 0.0f, 0.0f, 0.0f};
        synthetic_state_.imu.gyroscope = {0.0f, 0.0f, 0.0f};
        synthetic_state_.imu.accelerometer = {0.0f, 0.0f, 9.81f};
        const auto defaults =
            params.Get<std::vector<float>>("default_dof_pos_leg");
        for (std::size_t index = 0; index < kNumDofs; ++index)
        {
            synthetic_state_.motor_state.q[index] = defaults[index];
        }
    }

    void updateSyntheticState()
    {
        const double seconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - profile_started_at_).count();
        for (std::size_t index = 0; index < kNumDofs; ++index)
        {
            const float offset = static_cast<float>(
                0.01 * std::sin(seconds + 0.17 * static_cast<double>(index)));
            synthetic_state_.motor_state.q[index] += offset * 0.001f;
            synthetic_state_.motor_state.dq[index] = offset;
        }
        synthetic_state_.imu.gyroscope[2] =
            static_cast<float>(0.01 * std::sin(seconds));
    }

    void initializeHardwareDisableOutput()
    {
        lw_sdk_.InitCmdData(disable_command_);
        disable_command_.motors_disable = true;
        hardware_serial_attempted_ = true;
        const LWSerialInitStatus status = lw_sdk_.InitSerial(
            options_.right_port.c_str(), options_.left_port.c_str());
        if (!status.bothInitialized())
        {
            sendDisableBurst();
            throw std::runtime_error(
                "failed to initialize profiler serial ports: "
                + status.failureSummary());
        }
        for (int attempt = 0; attempt < 20; ++attempt)
        {
            sendDisable();
            ++initial_disable_packets_;
            if (attempt + 1 < 20)
            {
                std::this_thread::sleep_for(5ms);
            }
        }
        initial_disable_writes_complete_ = true;
        startDisableKeepalive();
    }

    void initializeHardwareObservation(int argc, char** argv)
    {
        rclcpp::init(argc, argv);
        ros_initialized_ = true;
        node_ = std::make_shared<rclcpp::Node>("lw_config_profiler");
        auto qos = rclcpp::SystemDefaultsQoS();
        qos.keep_last(1);
        qos.best_effort();
        imu_subscription_ = node_->create_subscription<sensor_msgs::msg::Imu>(
            options_.imu_topic,
            qos,
            [this](sensor_msgs::msg::Imu::SharedPtr message)
            {
                const auto now = std::chrono::steady_clock::now();
                imu_gaps_.mark(now);
                const LWImuAhrsGuardDecision decision =
                    imu_ahrs_guard_.observeImu(
                        now,
                        {message->orientation.w,
                         message->orientation.x,
                         message->orientation.y,
                         message->orientation.z},
                        {message->angular_velocity.x,
                         message->angular_velocity.y,
                         message->angular_velocity.z});
                if (decision.pair_age_observed)
                {
                    imu_ahrs_pair_age_.record(decision.pair_age);
                }
                if (!decision.accepted())
                {
                    return;
                }
                trusted_imu_gaps_.mark(now);
                auto trusted =
                    std::make_shared<sensor_msgs::msg::Imu>(*message);
                trusted->orientation.w /= decision.quaternion_norm;
                trusted->orientation.x /= decision.quaternion_norm;
                trusted->orientation.y /= decision.quaternion_norm;
                trusted->orientation.z /= decision.quaternion_norm;
                std::lock_guard<std::mutex> lock(imu_mutex_);
                latest_imu_ = std::move(trusted);
            });
        ahrs_subscription_ =
            node_->create_subscription<geometry_msgs::msg::Vector3>(
                options_.ahrs_topic,
                qos,
                [this](geometry_msgs::msg::Vector3::SharedPtr message)
                {
                    const auto now = std::chrono::steady_clock::now();
                    if (imu_ahrs_guard_.observeAhrs(
                            now,
                            {message->x, message->y, message->z}))
                    {
                        ahrs_gaps_.mark(now);
                    }
                });
        executor_thread_ = std::thread([this]() { rclcpp::spin(node_); });
    }

    void startDisableKeepalive()
    {
        disable_keepalive_stop_.store(false, std::memory_order_release);
        disable_keepalive_started_ = true;
        disable_keepalive_thread_ = std::thread(
            [this]()
            {
                auto next = std::chrono::steady_clock::now();
                while (!disable_keepalive_stop_.load(
                    std::memory_order_acquire))
                {
                    try
                    {
                        sendDisable();
                    }
                    catch (const std::exception& exception)
                    {
                        fail(std::string("motor-disable keepalive failed: ")
                             + exception.what());
                        break;
                    }
                    catch (...)
                    {
                        fail("motor-disable keepalive failed");
                        break;
                    }
                    next += 5ms;
                    std::this_thread::sleep_until(next);
                }
            });
    }

    void stopDisableKeepalive() noexcept
    {
        disable_keepalive_stop_.store(true, std::memory_order_release);
        if (disable_keepalive_thread_.joinable())
        {
            disable_keepalive_thread_.join();
        }
    }

    void requireDisableKeepaliveHealthy() const
    {
        if (options_.mode == ProfileMode::HardwareObserve
            && failed_.load(std::memory_order_acquire))
        {
            throw std::runtime_error(
                "motor-disable keepalive failed during profiler initialization");
        }
    }

    void stopExecutor() noexcept
    {
        if (ros_initialized_ && rclcpp::ok())
        {
            rclcpp::shutdown();
        }
        if (executor_thread_.joinable())
        {
            executor_thread_.join();
        }
        ros_initialized_ = false;
    }

    void sendDisable()
    {
        const auto started = std::chrono::steady_clock::now();
        const LWSendResult result = lw_sdk_.SendCmdData(disable_command_);
        serial_writes_.record(std::chrono::steady_clock::now() - started);
        if (!result.complete())
        {
            ++serial_write_failures_;
            throw std::runtime_error(
                "motor-disable write was incomplete: "
                + result.failureSummary());
        }
    }

    bool sendDisableBurst() noexcept
    {
        if (disable_burst_sent_.exchange(true, std::memory_order_acq_rel))
        {
            return serial_write_failures_.load(std::memory_order_acquire) == 0;
        }
        bool all_complete = true;
        for (int attempt = 0; attempt < 20; ++attempt)
        {
            try
            {
                sendDisable();
            }
            catch (...)
            {
                all_complete = false;
            }
            if (attempt + 1 < 20)
            {
                std::this_thread::sleep_for(5ms);
            }
        }
        return all_complete;
    }

    void prepareMotionReference(const std::string& policy)
    {
        motion_loader_.reset();
        motion_length_ = 0.0f;
        const auto definition = GetLWPolicyDefinition(policy);
        if (!definition)
        {
            throw std::runtime_error("missing preloaded policy: " + policy);
        }
        const auto& policy_configuration = definition->runtime;
        const bool needs_motion =
            policy_configuration.needs_motion_reference;
        if (needs_motion)
        {
            motion_loader_ = std::make_unique<MotionLoaderLW>(
                ResolvePolicyPath(
                    policy + "/"
                    + policy_configuration.motion_file),
                policy_configuration.motion_fps,
                policy_configuration.motion_time_offset_frames,
                kNumDofs);
            motion_loader_->Reset(robot_state.imu.quaternion);
            motion_length_ = motion_loader_->GetDuration();
        }
        policy_generation_ = ActivateLWPolicy(policy, motion_length_);
        policy_started_at_ = std::chrono::steady_clock::now();
        publishMotionReference();
    }

    void publishMotionReference()
    {
        if (!motion_loader_)
        {
            return;
        }
        double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - policy_started_at_).count();
        if (motion_length_ > 0.0f)
        {
            elapsed = std::fmod(elapsed, static_cast<double>(motion_length_));
        }
        motion_loader_->Update(static_cast<float>(elapsed));
        PublishLWMotionReference(
            {policy_generation_,
             motion_loader_->GetJointPos(),
             motion_loader_->GetJointVel(),
             motion_loader_->GetAnchorQuat(),
             motion_loader_->GetInitQuat()});
    }

    void runPolicy(const std::string& policy)
    {
        prepareMotionReference(policy);
        auto inference_samples = std::make_shared<LWProfileDistribution>();
        LoopConfig control_config = BuildLWControlLoopConfig(params);
        control_config.cpu_affinity = options_.cpu;
        control_config.realtime_priority = options_.realtime_priority;
        control_config.require_realtime = options_.require_realtime;
        // Profiling observes misses; it must not turn a sample into a runtime
        // safety action or terminate before the requested duration.
        control_config.timing_policy = {};

        const auto error_handler = [this](
            const std::string& loop_name,
            std::exception_ptr error)
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
            fail(loop_name + ": " + message);
        };

        LoopFunc control_loop(
            "lw_profile_control",
            control_config,
            [this]()
            {
                runtime_core_.runControlCycle(
                    LWControlCycleHooks{
                        {}, {},
                        [this]()
                        {
                            return options_.mode == ProfileMode::HostOnly
                                || state_ready_.load(
                                    std::memory_order_acquire);
                        },
                        []() { return true; }, {}, {}, {}});
            },
            error_handler);
        LoopFunc inference_loop(
            "lw_profile_inference",
            params.Get<float>("dt") * params.Get<int>("decimation"),
            [this, inference_samples]()
            {
                publishMotionReference();
                const auto started = std::chrono::steady_clock::now();
                runtime_core_.runInferenceCycle(false);
                inference_samples->record(
                    std::chrono::steady_clock::now() - started);
            },
            -1,
            error_handler);

        profile_started_at_ = std::chrono::steady_clock::now();
        inference_loop.start();
        control_loop.start();
        {
            std::unique_lock<std::mutex> lock(failure_mutex_);
            failure_condition_.wait_for(
                lock,
                std::chrono::duration<double>(options_.duration_seconds),
                [this]() { return failed_.load(std::memory_order_acquire); });
        }
        control_loop.shutdown();
        inference_loop.shutdown();

        ProfilePolicyResult result;
        result.policy = policy;
        result.inference = inference_samples->snapshot();
        result.control_timing = control_loop.timingSnapshot();
        result.inference_timing = inference_loop.timingSnapshot();
        result.control_startup = control_loop.startupSnapshot();
        result.inference_startup = inference_loop.startupSnapshot();
        policy_results_.push_back(std::move(result));
        DeactivateLWPolicy();
    }

    void fail(const std::string& message) noexcept
    {
        bool expected = false;
        if (failed_.compare_exchange_strong(
                expected, true, std::memory_order_acq_rel))
        {
            std::lock_guard<std::mutex> lock(failure_mutex_);
            failure_message_ = message;
        }
        failure_condition_.notify_all();
    }

    HardwareProfileResult snapshotHardware() const
    {
        HardwareProfileResult result;
        result.imu = imu_gaps_.snapshotSince(hardware_started_at_);
        result.ahrs = ahrs_gaps_.snapshotSince(hardware_started_at_);
        result.trusted_imu =
            trusted_imu_gaps_.snapshotSince(hardware_started_at_);
        result.imu_ahrs_pair_age = imu_ahrs_pair_age_.snapshot();
        result.right_feedback =
            right_feedback_.snapshotSince(hardware_started_at_);
        result.left_feedback =
            left_feedback_.snapshotSince(hardware_started_at_);
        result.serial_writes = serial_writes_.snapshot();
        result.serial_write_failures =
            serial_write_failures_.load(std::memory_order_acquire);
        return result;
    }

    std::string failureMessage() const
    {
        std::lock_guard<std::mutex> lock(failure_mutex_);
        return failure_message_;
    }

    void writeReport() const
    {
        std::ofstream output(options_.output, std::ios::out | std::ios::trunc);
        if (!output)
        {
            throw std::runtime_error(
                "failed to create profile report: "
                + options_.output.string());
        }
        output << std::setprecision(17)
               << "{\n\"schema_version\":3,\n\"source_commit\":";
        emitJsonString(output, RL_SAR_SOURCE_COMMIT);
        output << ",\n\"mode\":";
        emitJsonString(
            output,
            options_.mode == ProfileMode::HostOnly
                ? "host-only" : "hardware-observe");
        output << ",\n\"host\":{\"node\":";
        emitJsonString(output, host_identity_.node);
        output << ",\"system\":";
        emitJsonString(output, host_identity_.system);
        output << ",\"release\":";
        emitJsonString(output, host_identity_.release);
        output << ",\"machine\":";
        emitJsonString(output, host_identity_.machine);
        output << '}';
        output << ",\n\"policy_root\":";
        emitJsonString(
            output,
            std::filesystem::canonical(options_.policy_root).string());
        output << ",\n\"policy_assets\":[\n";
        for (std::size_t index = 0; index < policy_assets_.size(); ++index)
        {
            if (index != 0)
            {
                output << ",\n";
            }
            output << "{\"path\":";
            emitJsonString(output, policy_assets_[index].path);
            output << ",\"sha256\":";
            emitJsonString(output, policy_assets_[index].sha256);
            output << '}';
        }
        output << "\n],\n\"duration_per_policy_seconds\":"
               << options_.duration_seconds
               << ",\n\"hardware_confirmation\":"
               << (options_.hardware_confirmed ? "true" : "false")
               << ",\n\"failed\":"
               << (failed_.load(std::memory_order_acquire) ? "true" : "false")
               << ",\n\"failure\":";
        emitJsonString(output, failureMessage());
        output << ",\n\"policies\":[\n";
        bool first = true;
        for (const auto& result : policy_results_)
        {
            if (!first)
            {
                output << ",\n";
            }
            first = false;
            output << "{\"policy\":";
            emitJsonString(output, result.policy);
            output << ",\"duration_seconds\":"
                   << options_.duration_seconds
                   << ",\"inference_duration\":";
            emitDistribution(output, result.inference);
            output << ",\"control_timing\":";
            emitLoopTiming(output, result.control_timing);
            output << ",\"inference_timing\":";
            emitLoopTiming(output, result.inference_timing);
            output << ",\"control_startup\":";
            emitLoopStartup(output, result.control_startup);
            output << ",\"inference_startup\":";
            emitLoopStartup(output, result.inference_startup);
            output << '}';
        }
        output << "\n],\n\"hardware\":{\"initial_disable_writes_complete\":"
               << (initial_disable_writes_complete_ ? "true" : "false")
               << ",\"initial_disable_packets\":"
               << initial_disable_packets_
               << ",\"disable_keepalive_started\":"
               << (disable_keepalive_started_ ? "true" : "false")
               << ",\"disable_keepalive_period_ms\":5"
               << ",\"disable_only_output_enforced\":"
               << (initial_disable_writes_complete_
                       && disable_keepalive_started_
                       && hardware_result_.serial_write_failures == 0
                       ? "true" : "false")
               << ",\"imu_seen\":"
               << (hardware_result_.imu.seen ? "true" : "false")
               << ",\"imu_first_sample_delay_us\":"
               << hardware_result_.imu.first_sample_delay_us
               << ",\"imu_final_age_us\":"
               << hardware_result_.imu.final_age_us
               << ",\"imu_gap\":";
        emitDistribution(output, hardware_result_.imu.gaps);
        output << ",\"ahrs_seen\":"
               << (hardware_result_.ahrs.seen ? "true" : "false")
               << ",\"ahrs_first_sample_delay_us\":"
               << hardware_result_.ahrs.first_sample_delay_us
               << ",\"ahrs_final_age_us\":"
               << hardware_result_.ahrs.final_age_us
               << ",\"ahrs_gap\":";
        emitDistribution(output, hardware_result_.ahrs.gaps);
        output << ",\"trusted_imu_seen\":"
               << (hardware_result_.trusted_imu.seen ? "true" : "false")
               << ",\"trusted_imu_first_sample_delay_us\":"
               << hardware_result_.trusted_imu.first_sample_delay_us
               << ",\"trusted_imu_final_age_us\":"
               << hardware_result_.trusted_imu.final_age_us
               << ",\"trusted_imu_gap\":";
        emitDistribution(output, hardware_result_.trusted_imu.gaps);
        output << ",\"imu_ahrs_pair_age\":";
        emitDistribution(output, hardware_result_.imu_ahrs_pair_age);
        output << ",\"right_feedback_seen\":"
               << (hardware_result_.right_feedback.seen ? "true" : "false")
               << ",\"right_feedback_first_sample_delay_us\":"
               << hardware_result_.right_feedback.first_sample_delay_us
               << ",\"right_feedback_final_age_us\":"
               << hardware_result_.right_feedback.final_age_us
               << ",\"right_feedback_gap\":";
        emitDistribution(output, hardware_result_.right_feedback.gaps);
        output << ",\"left_feedback_seen\":"
               << (hardware_result_.left_feedback.seen ? "true" : "false")
               << ",\"left_feedback_first_sample_delay_us\":"
               << hardware_result_.left_feedback.first_sample_delay_us
               << ",\"left_feedback_final_age_us\":"
               << hardware_result_.left_feedback.final_age_us
               << ",\"left_feedback_gap\":";
        emitDistribution(output, hardware_result_.left_feedback.gaps);
        output << ",\"serial_write_duration\":";
        emitDistribution(output, hardware_result_.serial_writes);
        output << ",\"serial_write_failures\":"
               << hardware_result_.serial_write_failures
               << ",\"commands_sent\":";
        emitJsonString(
            output,
            options_.mode == ProfileMode::HostOnly
                ? "none" : "motors_disable_only");
        output << "}\n}\n";
        if (!output)
        {
            throw std::runtime_error(
                "failed to write profile report: "
                + options_.output.string());
        }
    }

    ProfileOptions options_;
    ProfileHostIdentity host_identity_;
    std::vector<LWDeploymentFileRecord> policy_assets_;
    LWRuntimeCore runtime_core_;
    std::shared_ptr<ProfilePassiveState> passive_state_;
    RobotState<float> synthetic_state_;
    std::chrono::steady_clock::time_point profile_started_at_{};

    std::unique_ptr<MotionLoaderLW> motion_loader_;
    float motion_length_ = 0.0f;
    std::uint64_t policy_generation_ = 0;
    std::chrono::steady_clock::time_point policy_started_at_{};
    std::vector<ProfilePolicyResult> policy_results_;

    LWSDK lw_sdk_;
    LowState lw_low_state_{};
    LowCmd disable_command_{};
    LWProfileTimedSource imu_gaps_;
    LWProfileTimedSource ahrs_gaps_;
    LWProfileTimedSource trusted_imu_gaps_;
    LWProfileDistribution imu_ahrs_pair_age_;
    LWProfileTimedSource right_feedback_;
    LWProfileTimedSource left_feedback_;
    LWProfileDistribution serial_writes_;
    std::atomic<std::uint64_t> serial_write_failures_{0};
    std::atomic<bool> state_ready_{false};
    std::atomic<bool> disable_burst_sent_{false};
    bool hardware_serial_attempted_ = false;
    bool initial_disable_writes_complete_ = false;
    std::uint64_t initial_disable_packets_ = 0;
    bool disable_keepalive_started_ = false;
    std::atomic<bool> disable_keepalive_stop_{false};
    std::thread disable_keepalive_thread_;
    std::chrono::steady_clock::time_point hardware_started_at_{};
    HardwareProfileResult hardware_result_;

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr ahrs_subscription_;
    sensor_msgs::msg::Imu::SharedPtr latest_imu_;
    std::mutex imu_mutex_;
    LWImuAhrsGuard imu_ahrs_guard_{std::chrono::milliseconds(100)};
    std::thread executor_thread_;
    bool ros_initialized_ = false;

    std::atomic<bool> failed_{false};
    mutable std::mutex failure_mutex_;
    std::condition_variable failure_condition_;
    std::string failure_message_;
};
} // namespace

int main(int argc, char** argv)
{
    try
    {
        ProfileOptions options = parseOptions(argc, argv);
        LWConfigProfiler profiler(std::move(options), argc, argv);
        return profiler.run();
    }
    catch (const std::exception& exception)
    {
        std::cerr << "[LW Profile] " << exception.what() << '\n';
        if (rclcpp::ok())
        {
            rclcpp::shutdown();
        }
        return EXIT_FAILURE;
    }
}
