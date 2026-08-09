#ifndef RL_REAL_LW_HPP
#define RL_REAL_LW_HPP

#define PLOT
// #define ENABLE_IMU_GYRO_FILTER 
// #define CSV_LOGGER
// #define CONTROL_TIME_PRINT
// #define FOWARD_TIME_PRINT

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "inference_runtime.hpp"
#include "command_gate.hpp"
#include "loop.hpp"
#include "sensor_readiness.hpp"
#include "lw_control_safety.hpp"
#include "lw_joystick_safety.hpp"
#include "lw_safety_policy.hpp"
#include "fsm_LW.hpp"

#include "LW_sdk.hpp"
#include <atomic>
#include <csignal>
#include <chrono>
#include <exception>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <realtime_tools/realtime_box.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include "joystick.hh"
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

class RL_Real : public RL
{
public:
    RL_Real(int argc, char **argv, const std::string& policy_root);
    ~RL_Real();

    std::shared_ptr<rclcpp::Node> ros2_node;

private:
    // rl functions
    std::vector<float> Forward() override;
    void GetState(RobotState<float> *state) override;
    void SetCommand(const RobotCommand<float> *command) override;
    void RunModel();
    void RobotControl();
    void ApplyPendingInput();
    void HandleLoopError(const std::string& loop_name, std::exception_ptr error) noexcept;
    void HandleLoopTiming(
        const std::string& loop_name,
        LoopTimingLevel level,
        const LoopTimingSnapshot& timing) noexcept;
    void HandleLWPolicyOutputFault(
        LWPolicyOutputStatus status) noexcept override;
    void ApplySafetyEvent(
        LWSafetyEvent event,
        const std::string& reason) noexcept;
    void EnterFailSafe(
        const std::string& reason,
        bool request_shutdown) noexcept;
    void SendEmergencyDisableBurst() noexcept;
    void ApplyControlledFallbackCommand();
    bool HandleSensorReadiness();
    bool ValidateFeedbackAndAttitude();
    bool ValidateCommandForSend(const RobotCommand<float>& command);

    // loop
    std::shared_ptr<LoopFunc> loop_joystick;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;
    std::shared_ptr<LoopFunc> loop_plot;
    std::atomic<bool> control_timing_degraded_latched_{false};

    // LW interface
    LWSDK lw_sdk;
    LowCmd lw_low_command = {0};
    LowState lw_low_state = {0};
    CommandGate command_gate_;
    LWSafetySupervisor safety_supervisor_;
    std::atomic<bool> fatal_error_latched_{false};
    std::atomic<bool> shutdown_requested_{false};
    std::atomic<bool> controlled_fallback_applied_{false};
    std::mutex fail_safe_mutex_;
    LWSendResult disable_lw_robot(bool latch_commands = false);

    // joystick
    std::unique_ptr<LWJoystickDevice> sys_js;
    JoystickEvent sys_js_event;

    std::array<LWJoystickButton, LW_JOYSTICK_BUTTON_COUNT> sys_js_button{};
    std::array<int, LW_JOYSTICK_AXIS_COUNT> sys_js_axis{};
    LWJoystickFaultLatch joystick_fault_latch_;
    LWInputMailbox<Input::Gamepad> joystick_input_mailbox_;
    std::uint64_t consumed_gamepad_sequence_ = 0;
    bool sys_js_active = false;
    float axis_deadzone = 0.05f;
    int sys_js_max_value = (1 << (16 - 1)); // 即 2的15次方 = 32768
    void SetupSysJoystick(const std::string& device, int bits);
    void GetSysJoystick();
    void LatchJoystickFault(const LWJoystickSampleResult& result) noexcept;
    void ApplyJoystickFaultGate() noexcept;

    // Imu
    using SafetyClock = SensorReadinessMonitor::Clock;
    struct TimedImuSample
    {
        sensor_msgs::msg::Imu::SharedPtr message;
        SafetyClock::time_point received_at;
    };

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_ = nullptr;
    realtime_tools::RealtimeBox<std::shared_ptr<TimedImuSample>> received_imu_sample_{nullptr};
    SensorReadinessMonitor sensor_readiness_monitor_{std::chrono::milliseconds(100)};
    SensorReadinessStatus sensor_readiness_status_;
    SafetyClock::time_point last_readiness_log_time_{};
    SafetyClock::time_point last_serial_diagnostic_log_time_{};
    std::string last_missing_sources_;
    bool sensor_ready_logged_ = false;
    void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
#ifdef ENABLE_IMU_GYRO_FILTER
    // 滤波系数 alpha 取值范围 (0, 1]。
    // 越接近 1 则越信任当前真实值（滤波效果弱，延迟小）；
    // 越接近 0 则越信任历史值（滤波效果强，延迟大）。建议根据实际震动情况调节（如 0.2 ~ 0.5）。
    float gyro_filter_alpha_ = 0.3f; 
    
    // 记录上一时刻的滤波结果
    std::vector<float> filtered_gyro_ = {0.0f, 0.0f, 0.0f};
    
    // 标记是否是第一次接收到 IMU 数据，防止从 0 开始产生初始跳变
    bool is_first_imu_ = true; 
#endif

     // plot
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> jointstate_plot_publisher_ = nullptr;
    std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> realtime_debug_publisher_ = nullptr;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr runtime_diagnostics_timer_;
    void jointstate_plot_callback(void);
    void RuntimeDiagnosticsCallback();
    std::uint64_t last_operator_status_sequence_ = 0;
    std::uint64_t runtime_diagnostics_ticks_ = 0;
    bool operator_status_seen_ = false;
    bool timing_degraded_logged_ = false;
    bool realtime_fallback_logged_ = false;
    std::uint64_t last_safety_event_sequence_ = 0;

    // others
    void disable_robot(void);
    struct RealDebugSnapshot
    {
        RobotState<float> robot_state;
        LowState low_state{};
        LowCmd low_command{};
        LWControlSnapshot control;
    };

    LWSnapshotBuffer<LWPolicyInputSnapshot> policy_input_snapshot_;
    LWSnapshotBuffer<RealDebugSnapshot> debug_snapshot_;

    std::shared_ptr<const LWPolicyActivation> inference_activation_;
    std::shared_ptr<const LWMotionReferenceSnapshot> inference_motion_reference_;
    Observations<float> inference_obs_;
    std::vector<int> inference_obs_dims_;
    ObservationBuffer inference_history_obs_buf_;
    std::vector<float> inference_history_obs_;
    std::vector<float> inference_output_dof_pos_;
    std::vector<float> inference_output_dof_vel_;
    std::vector<float> inference_output_dof_tau_;
    std::uint64_t inference_frame_ = 0;
    float inference_gait_phase_time_ = 0.0f;
    void ResetInferenceWorkspace(
        const LWPolicyActivation& activation);

};

#endif // RL_REAL_LW_HPP
