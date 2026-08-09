/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_SDK_HPP
#define RL_SDK_HPP

#include <iostream>
#include <string>
#include <exception>
#include <unistd.h>
#include <algorithm>
#include <tbb/concurrent_queue.h>
#include <vector>
#include <memory>
#include <fstream>
#include <mutex>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <filesystem>

#include <yaml-cpp/yaml.h>
#include "fsm.hpp"
#include "observation_buffer.hpp"
#include "vector_math.hpp"
#include "inference_runtime.hpp"
#include "lw_configuration_validation.hpp"
#include "logger.hpp"
#include "motion_loader.hpp"
#include "motion_loader_lw.hpp"
#include "lw_runtime_sync.hpp"

#include <unordered_map>

template <typename T>
struct RobotCommand
{
    struct MotorCommand
    {
        std::vector<int> mode;
        std::vector<T> q;
        std::vector<T> dq;
        std::vector<T> tau;
        std::vector<T> kp;
        std::vector<T> kd;

        void resize(size_t num_joints)
        {
            mode.resize(num_joints, 0);
            q.resize(num_joints, 0.0f);
            dq.resize(num_joints, 0.0f);
            tau.resize(num_joints, 0.0f);
            kp.resize(num_joints, 0.0f);
            kd.resize(num_joints, 0.0f);
        }
    } motor_command;
};

template <typename T>
struct RobotState
{
    struct IMU
    {
        std::vector<T> quaternion = {1.0f, 0.0f, 0.0f, 0.0f}; // w, x, y, z
        std::vector<T> gyroscope = {0.0f, 0.0f, 0.0f};
        std::vector<T> accelerometer = {0.0f, 0.0f, 0.0f};
    } imu;

    struct MotorState
    {
        std::vector<T> q;
        std::vector<T> dq;
        std::vector<T> ddq;
        std::vector<T> tau_est;
        std::vector<T> cur;

        void resize(size_t num_joints)
        {
            q.resize(num_joints, 0.0f);
            dq.resize(num_joints, 0.0f);
            ddq.resize(num_joints, 0.0f);
            tau_est.resize(num_joints, 0.0f);
            cur.resize(num_joints, 0.0f);
        }
    } motor_state;
};

namespace Input
{
    // Recommend: Num0-GetUp Num9-GetDown N-ToggleNavMode
    //            R-SimReset Enter-SimToggle
    //            M-MotorEnable K-MotorDisable P-MotorPassive
    //            Num1-BaseLocomotion Num2-Num8-Skills(7)
    //            WS-AxisX AD-AxisY QE-AxisYaw Space-AxisClear
    enum class Keyboard
    {
        None = 0,
        A, B, C, D, E, F, G, H, I, J, K, L, M,
        N, O, P, Q, R, S, T, U, V, W, X, Y, Z,
        Num0, Num1, Num2, Num3, Num4, Num5, Num6, Num7, Num8, Num9,
        Space, Enter, Escape,
        Up, Down, Left, Right
    };

    // Recommend: A-GetUp B-GetDown X-ToggleNavMode Y-None
    //            RB_Y-SimReset RB_X-SimToggle
    //            LB_A-MotorEnable LB_B-MotorDisable LB_X-MotorPassive
    //            RB_DPadUp-BaseLocomotion RB_DPadOthers/LB_DPadOthers-Skills(7)
    //            LY-AxisX LX-AxisY RX-AxisYaw
    enum class Gamepad
    {
        None = 0,
        A, B, X, Y, LB, RB, LStick, RStick, DPadUp, DPadDown, DPadLeft, DPadRight,
        LB_A, LB_B, LB_X, LB_Y, LB_LStick, LB_RStick, LB_DPadUp, LB_DPadDown, LB_DPadLeft, LB_DPadRight,
        RB_A, RB_B, RB_X, RB_Y, RB_LStick, RB_RStick, RB_DPadUp, RB_DPadDown, RB_DPadLeft, RB_DPadRight,
        LB_RB
    };
}

struct Control
{
    Input::Keyboard current_keyboard = Input::Keyboard::None, last_keyboard = Input::Keyboard::None;
    Input::Gamepad current_gamepad = Input::Gamepad::None, last_gamepad = Input::Gamepad::None;

    float x = 0.0f;
    float y = 0.0f;
    float yaw = 0.0f;
    float gait_frequency = 0.0f;
    //float swing_height = 0.0f;
    bool dpad_handled = false;
    bool navigation_mode = false;

    void SetKeyboard(Input::Keyboard keyboad)
    {
        if (current_keyboard != keyboad)
        {
            last_keyboard = current_keyboard;
            current_keyboard = keyboad;
        }
    }

    void SetGamepad(Input::Gamepad gamepad)
    {
        if (current_gamepad != gamepad)
        {
            last_gamepad = current_gamepad;
            current_gamepad = gamepad;
        }
    }

    void ClearInput()
    {
        current_keyboard = last_keyboard;
        current_gamepad = Input::Gamepad::None;
    }
};

struct YamlParams
{
    YAML::Node config_node;

    // Get config value by key
    // WARNING: For vectors/containers, store result in a variable before using iterators/references:
    //   ✓ auto vec = params.Get<std::vector<int>>("key"); vec.begin()
    //   ✗ params.Get<std::vector<int>>("key").begin()  // dangling reference!
    template<typename T>
    T Get(const std::string& key, const T& default_value = T()) const
    {
        if (config_node[key])
        {
            return config_node[key].as<T>();
        }
        return default_value;
    }

    bool Has(const std::string& key) const
    {
        return config_node[key].IsDefined();
    }
};

struct LWMotionReferenceSnapshot
{
    std::uint64_t generation = 0;
    std::vector<float> joint_pos;
    std::vector<float> joint_vel;
    std::vector<float> anchor_quat;
    std::vector<float> init_quat;
};

struct LWPolicyDefinition
{
    std::string path;
    YamlParams params;
    std::shared_ptr<InferenceRuntime::Model> model;
};

struct LWPolicyActivation
{
    std::shared_ptr<const LWPolicyDefinition> definition;
    std::uint64_t generation = 0;
    float motion_length = 0.0f;
    std::chrono::steady_clock::time_point activated_at{};
};

struct LWPolicyProgressSnapshot
{
    std::uint64_t generation = 0;
    std::uint64_t frame = 0;
};

struct LWControlSnapshot
{
    float x = 0.0f;
    float y = 0.0f;
    float yaw = 0.0f;
    float gait_frequency = 0.0f;
};

struct LWPolicyInputSnapshot
{
    RobotState<float> robot_state;
    LWControlSnapshot control;
};

struct LWPolicyOutputFrame
{
    std::uint64_t generation = 0;
    std::uint64_t sequence = 0;
    std::uint64_t frame = 0;
    std::chrono::steady_clock::time_point source_time{};
    std::vector<float> dof_pos;
    std::vector<float> dof_vel;
    std::vector<float> dof_tau;
};

enum class LWPolicyOutputStatus
{
    Ready,
    Missing,
    GenerationMismatch,
    Incomplete,
    Stale
};

inline bool LWPolicyOutputRequiresFallback(
    LWPolicyOutputStatus status,
    bool initial_wait_expired) noexcept
{
    return status == LWPolicyOutputStatus::Stale
        || status == LWPolicyOutputStatus::Incomplete
        || (status != LWPolicyOutputStatus::Ready
            && initial_wait_expired);
}

bool LWPolicyOutputPayloadComplete(
    const LWPolicyOutputFrame& output,
    size_t expected_dofs) noexcept;
LWPolicyOutputStatus EvaluateLWPolicyOutput(
    const LWPolicyOutputFrame* output,
    std::uint64_t active_generation,
    size_t expected_dofs,
    std::chrono::steady_clock::time_point now,
    std::chrono::steady_clock::duration max_age) noexcept;

class LWPolicyOutputTransport
{
public:
    bool publish(
        LWPolicyOutputFrame output,
        std::uint64_t active_generation,
        size_t expected_dofs);
    std::shared_ptr<const LWPolicyOutputFrame> load() const noexcept;
    void clear() noexcept;

private:
    LWAtomicSnapshot<LWPolicyOutputFrame> latest_;
    std::atomic<std::uint64_t> next_sequence_{1};
};

template <typename T>
struct Observations
{
    std::vector<T> lin_vel;
    std::vector<T> ang_vel;
    std::vector<T> gravity_vec;
    std::vector<T> commands;
    std::vector<T> base_quat;
    std::vector<T> dof_pos;
    std::vector<T> dof_vel;
    std::vector<T> actions;
    std::vector<T> gait_phase;
    // std::vector<T> gait_command;
};

class RL
{
public:
    RL() {};
    ~RL() {};

    YamlParams params;
    Observations<float> obs;
    std::vector<int> obs_dims;

    RobotState<float> robot_state;
    RobotCommand<float> robot_command;
    tbb::concurrent_queue<std::vector<float>> output_dof_pos_queue;
    tbb::concurrent_queue<std::vector<float>> output_dof_vel_queue;
    tbb::concurrent_queue<std::vector<float>> output_dof_tau_queue;

    FSM fsm;
    RobotState<float> start_state;
    RobotState<float> now_state;
    bool rl_init_done = false;

    // init
    void InitObservations();
    void InitOutputs();
    void InitControl();
    void InitRL(std::string robot_config_path);
    void InitJointNum(size_t num_joints);
    void SetPolicyRoot(const std::filesystem::path& policy_root);
    std::string ResolvePolicyPath(const std::string& relative_path) const;
    // 预加载模型函数
    void PreloadModel(const std::string& robot_config_path);
    // 存放已经加载好的 ONNX 模型的字典
    std::unordered_map<std::string, std::shared_ptr<InferenceRuntime::Model>> preloaded_models_;
    std::unordered_map<std::string, YAML::Node> preloaded_lw_policy_configs_;
    void PreloadLWPolicyContext(const std::string& robot_config_path);
    std::shared_ptr<const LWPolicyDefinition> GetLWPolicyDefinition(
        const std::string& robot_config_path) const;
    std::uint64_t ActivateLWPolicy(
        const std::string& robot_config_path,
        float motion_length = 0.0f);
    void DeactivateLWPolicy();
    std::shared_ptr<const LWPolicyActivation> LoadLWPolicyActivation() const noexcept;
    void PublishLWMotionReference(LWMotionReferenceSnapshot reference);
    void PublishCurrentLWMotionReference(std::uint64_t generation);
    std::shared_ptr<const LWMotionReferenceSnapshot> LoadLWMotionReference() const noexcept;
    void PublishLWPolicyProgress(std::uint64_t generation, std::uint64_t frame);
    std::shared_ptr<const LWPolicyProgressSnapshot> LoadLWPolicyProgress() const noexcept;
    void PublishLWOperatorStatus(
        LWOperatorMode mode,
        float progress = 0.0f) noexcept;
    bool ReadLWOperatorStatus(LWOperatorStatusSnapshot& status) const noexcept;
    bool PublishLWPolicyOutput(LWPolicyOutputFrame output);
    std::shared_ptr<const LWPolicyOutputFrame> LoadLWPolicyOutput() const noexcept;
    std::chrono::steady_clock::duration GetLWPolicyOutputMaxAge(
        const LWPolicyActivation& activation) const;
    virtual void HandleLWPolicyOutputFault(
        LWPolicyOutputStatus status) noexcept
    {
        (void)status;
    }

#ifdef RL_REQUIRE_EXPLICIT_POLICY_ROOT
    std::filesystem::path policy_root_;
#else
    std::filesystem::path policy_root_{POLICY_DIR};
#endif

    // rl functions
    virtual std::vector<float> Forward() = 0;
    std::vector<float> ComputeObservation();
    std::vector<float> ComputeLWObservation(
        const YamlParams& policy_params,
        Observations<float>& policy_obs,
        std::vector<int>& policy_obs_dims,
        const LWMotionReferenceSnapshot* motion_reference,
        std::uint64_t policy_frame,
        float motion_length) const;
    virtual void GetState(RobotState<float> *state) = 0;
    virtual void SetCommand(const RobotCommand<float> *command) = 0;
    void StateController(const RobotState<float> *state, RobotCommand<float> *command);
    void ComputeOutput(const std::vector<float> &actions, std::vector<float> &output_dof_pos, std::vector<float> &output_dof_vel, std::vector<float> &output_dof_tau);
    void ComputeLWOutput(
        const YamlParams& policy_params,
        const Observations<float>& policy_obs,
        const std::vector<float>& actions,
        std::vector<float>& output_dof_pos,
        std::vector<float>& output_dof_vel,
        std::vector<float>& output_dof_tau) const;

    // yaml params
    void ReadYaml(const std::string& file_path, const std::string& file_name);

    // csv logger
    std::string csv_filename;
    void CSVInit(std::string robot_name);
    void CSVLogger(const std::vector<float> &torque, const std::vector<float> &tau_est, const std::vector<float> &joint_pos, const std::vector<float> &joint_pos_target, const std::vector<float> &joint_vel);

    // control
    Control control;
    void KeyboardInterface();

    // history buffer
    ObservationBuffer history_obs_buf;
    std::vector<float> history_obs;

    // others
    int motiontime = 0;
    std::string robot_name, config_name;
    bool simulation_running = true;
    std::string ang_vel_axis = "body";  // "world" or "body"
    unsigned long long episode_length_buf = 0;
    float motion_length = 0.0;
    float gait_phase_time = 0.0f;
    int InverseJointMapping(int idx) const;

    // Motion tracking (for mimic/dance tasks)
    std::unique_ptr<MotionLoader> motion_loader;
    std::unique_ptr<MotionLoaderLW> motion_loader_lw;

    // protect func
    bool TorqueProtect(const std::vector<float> &origin_output_dof_tau);
    bool TorqueProtect(
        const std::vector<float>& origin_output_dof_tau,
        const YamlParams& policy_params) const;
    void AttitudeProtect(const std::vector<float> &quaternion, float pitch_threshold, float roll_threshold);

    // rl module
    std::shared_ptr<InferenceRuntime::Model> model;
    // output buffer
    std::vector<float> output_dof_tau;
    std::vector<float> output_dof_pos;
    std::vector<float> output_dof_vel;

    // thread safety
    std::mutex model_mutex;

private:
    std::unordered_map<
        std::string,
        std::shared_ptr<const LWPolicyDefinition>> lw_policy_definitions_;
    LWAtomicSnapshot<LWPolicyActivation> lw_policy_activation_;
    LWAtomicSnapshot<LWMotionReferenceSnapshot> lw_motion_reference_;
    LWAtomicSnapshot<LWPolicyProgressSnapshot> lw_policy_progress_;
    LWOperatorStatusMailbox lw_operator_status_;
    std::atomic<std::uint64_t> lw_operator_status_sequence_{1};
    LWPolicyOutputTransport lw_policy_output_transport_;
    std::atomic<std::uint64_t> lw_next_policy_generation_{1};
};

class RLFSMState : public FSMState
{
public:
    RLFSMState(RL& rl, const std::string& name)
        : FSMState(name), rl(rl), fsm_state(nullptr), fsm_command(nullptr) {}

    RL& rl;
    const RobotState<float> *fsm_state;
    RobotCommand<float> *fsm_command;

    bool Interpolate(
        float& percent,
        const std::vector<float>& start_pos,
        const std::vector<float>& target_pos,
        float duration_seconds,
        const std::string& description = "",
        bool use_fixed_gains = true,
        LWOperatorMode lw_status_mode = LWOperatorMode::Unknown
    );

    void RLControl();
    void RLControlLW();

private:
    std::uint64_t last_lw_output_generation_ = 0;
    std::uint64_t last_lw_output_sequence_ = 0;
    bool lw_output_stale_ = false;
};

#endif // RL_SDK_HPP
