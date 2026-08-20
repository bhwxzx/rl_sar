#ifndef LW_CONFIGURATION_VALIDATION_HPP
#define LW_CONFIGURATION_VALIDATION_HPP

#include "inference_runtime.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

struct LWPolicyDimensions
{
    std::size_t num_dofs = 0;
    std::size_t observation = 0;
    std::size_t model_input = 0;
    std::size_t model_output = 0;
};

struct LWBaseRuntimeConfiguration
{
    std::size_t num_dofs = 0;
    float dt = 0.0f;
    int decimation = 0;
    std::vector<std::string> joint_names;
    std::vector<int> joint_mapping;
    std::vector<int> wheel_indices;
    std::vector<std::uint8_t> wheel_mask;
    std::vector<float> rl_kp;
    std::vector<float> rl_kd;
    std::vector<float> fixed_kp;
    std::vector<float> fixed_kd;
    std::vector<float> torque_limits;
    std::vector<float> default_dof_pos_leg;
    std::vector<float> default_dof_pos_wheel;
    std::vector<float> gait_command;
    std::vector<float> vel_command;
};

struct LWPolicyRuntimeConfiguration
{
    std::size_t num_dofs = 0;
    float dt = 0.0f;
    int decimation = 0;
    float period_seconds = 0.0f;
    float output_max_age_seconds = 0.0f;
    float clip_obs = 0.0f;
    float ang_vel_scale = 0.0f;
    float dof_pos_scale = 0.0f;
    float dof_vel_scale = 0.0f;
    std::string model_name;
    std::vector<std::string> observations;
    std::vector<int> observations_history;
    std::string observations_history_priority;
    std::vector<float> commands_scale;
    std::vector<float> action_scale;
    std::vector<float> clip_actions_lower;
    std::vector<float> clip_actions_upper;
    std::vector<float> rl_kp;
    std::vector<float> rl_kd;
    std::vector<float> fixed_kp;
    std::vector<float> fixed_kd;
    std::vector<float> torque_limits;
    std::vector<float> default_dof_pos;
    std::vector<int> joint_mapping;
    std::vector<int> wheel_indices;
    std::vector<std::uint8_t> wheel_mask;
    bool needs_motion_reference = false;
    std::string motion_file;
    float motion_fps = 0.0f;
    int motion_time_offset_frames = 0;
    std::vector<int> motion_joint_mapping;
};

struct LWValidatedPolicyConfiguration
{
    YAML::Node merged;
    LWPolicyDimensions dimensions;
    LWPolicyRuntimeConfiguration runtime;
};

LWBaseRuntimeConfiguration ValidateLWBaseConfiguration(
    const YAML::Node& config,
    const std::string& source);

LWValidatedPolicyConfiguration ValidateLWPolicyConfiguration(
    const YAML::Node& base_config,
    const YAML::Node& policy_config,
    const std::string& source);

void ValidateLWModelContract(
    InferenceRuntime::Model& model,
    const LWPolicyDimensions& dimensions,
    const std::string& source,
    int warmup_iterations = 2);

#endif // LW_CONFIGURATION_VALIDATION_HPP
