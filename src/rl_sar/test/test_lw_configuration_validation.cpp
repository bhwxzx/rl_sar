#include "lw_configuration_validation.hpp"
#include "rl_sdk.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
namespace fs = std::filesystem;

void require(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

void requireFailure(
    const std::function<void()>& operation,
    const std::string& expected_text)
{
    try
    {
        operation();
    }
    catch (const std::exception& exception)
    {
        require(
            std::string(exception.what()).find(expected_text)
                != std::string::npos,
            "unexpected validation error: " + std::string(exception.what()));
        return;
    }
    throw std::runtime_error(
        "expected validation failure containing: " + expected_text);
}

YAML::Node loadConfig(const fs::path& file, const std::string& key)
{
    const YAML::Node config = YAML::LoadFile(file.string())[key];
    require(config && config.IsMap(), "failed to load " + file.string());
    return config;
}

class FakeONNXModel : public InferenceRuntime::Model
{
public:
    FakeONNXModel(
        std::int64_t input_features,
        std::int64_t output_features,
        std::int64_t input_batch = 1,
        std::int64_t output_batch = 1)
        : inputs_{{"observations",
                   {input_batch, input_features},
                   InferenceRuntime::TensorElementType::Float32}},
          outputs_{{"actions",
                    {output_batch, output_features},
                    InferenceRuntime::TensorElementType::Float32}},
          output_features_(output_features)
    {
    }

    bool load(const std::string&) override
    {
        return true;
    }

    bool is_loaded() const override
    {
        return true;
    }

    void forwardInto(
        const InferenceRuntime::TensorView*,
        std::size_t,
        InferenceRuntime::MutableTensorView output) override
    {
        require(
            output.size == static_cast<std::size_t>(output_features_),
            "fake model output size differs");
        std::fill_n(output.data, output.size, 0.0f);
    }

    std::string get_model_type() const override
    {
        return "onnx";
    }

    const std::vector<InferenceRuntime::TensorMetadata>&
    input_metadata() const override
    {
        return inputs_;
    }

    const std::vector<InferenceRuntime::TensorMetadata>&
    output_metadata() const override
    {
        return outputs_;
    }

private:
    std::vector<InferenceRuntime::TensorMetadata> inputs_;
    std::vector<InferenceRuntime::TensorMetadata> outputs_;
    std::int64_t output_features_;
};

class TestRL : public RL
{
public:
    std::vector<float> Forward() override
    {
        return {};
    }

    void GetState(RobotState<float>*) override
    {
    }

    void SetCommand(const RobotCommand<float>*) override
    {
    }
};

std::vector<float> referenceObservation(
    const LWPolicyRuntimeConfiguration& configuration,
    const Observations<float>& observations,
    const LWMotionReferenceSnapshot* motion_reference)
{
    std::vector<std::vector<float>> terms;
    for (const std::string& observation : configuration.observations)
    {
        if (observation == "ang_vel")
        {
            terms.push_back(observations.ang_vel * configuration.ang_vel_scale);
        }
        else if (observation == "gravity_vec")
        {
            terms.push_back(QuatRotateInverse(
                observations.base_quat, observations.gravity_vec));
        }
        else if (observation == "commands")
        {
            terms.push_back(observations.commands * configuration.commands_scale);
        }
        else if (observation == "dof_pos")
        {
            auto relative = observations.dof_pos - configuration.default_dof_pos;
            for (const int wheel : configuration.wheel_indices)
            {
                relative[wheel] = 0.0F;
            }
            terms.push_back(relative * configuration.dof_pos_scale);
        }
        else if (observation == "dof_vel")
        {
            terms.push_back(observations.dof_vel * configuration.dof_vel_scale);
        }
        else if (observation == "actions")
        {
            terms.push_back(observations.actions);
        }
        else if (observation == "gait_phase")
        {
            terms.push_back(observations.gait_phase);
        }
        else if (observation == "whole_body_tracking/motion_command")
        {
            std::vector<float> command;
            for (const int source : configuration.motion_joint_mapping)
            {
                command.push_back(motion_reference->joint_pos[source]);
            }
            for (const int source : configuration.motion_joint_mapping)
            {
                command.push_back(motion_reference->joint_vel[source]);
            }
            terms.push_back(std::move(command));
        }
        else if (observation
                 == "whole_body_tracking/motion_anchor_ori_b")
        {
            const auto robot = MotionLoaderLW::ComputeTorsoQuat(
                observations.base_quat);
            const auto anchor = QuaternionMultiply(
                motion_reference->init_quat,
                motion_reference->anchor_quat);
            const auto relative = QuaternionMultiply(
                QuaternionConjugate(robot), anchor);
            terms.push_back(MatrixFirstTwoColumns(
                QuaternionToRotationMatrix(relative)));
        }
    }
    std::vector<float> flattened;
    for (const auto& term : terms)
    {
        flattened.insert(flattened.end(), term.begin(), term.end());
    }
    return clamp(flattened, -configuration.clip_obs, configuration.clip_obs);
}

void testContiguousObservationAssemblyMatchesPreviousOrdering()
{
    const fs::path policy_root(POLICY_DIR);
    const YAML::Node base = loadConfig(policy_root / "LW/base.yaml", "LW");
    TestRL runtime;
    Observations<float> observations;
    observations.ang_vel = {0.2F, -0.3F, 0.4F};
    observations.gravity_vec = {0.0F, 0.0F, -1.0F};
    observations.commands = {0.5F, -0.25F, 0.75F};
    observations.base_quat = {0.91F, 0.12F, -0.18F, 0.34F};
    observations.dof_pos.resize(10);
    observations.dof_vel.resize(10);
    observations.actions.resize(10);
    for (std::size_t index = 0; index < 10; ++index)
    {
        observations.dof_pos[index] = 0.07F * static_cast<float>(index);
        observations.dof_vel[index] = -0.03F * static_cast<float>(index);
        observations.actions[index] = 0.02F * static_cast<float>(index + 1);
    }
    observations.gait_phase = {0.6F, -0.8F};
    LWMotionReferenceSnapshot motion;
    motion.joint_pos.resize(10);
    motion.joint_vel.resize(10);
    for (std::size_t index = 0; index < 10; ++index)
    {
        motion.joint_pos[index] = 0.11F * static_cast<float>(index + 1);
        motion.joint_vel[index] = -0.04F * static_cast<float>(index + 1);
    }
    motion.anchor_quat = {0.96F, 0.08F, -0.14F, 0.21F};
    motion.init_quat = {0.98F, -0.05F, 0.09F, 0.16F};

    for (const char* path : {
             "LW/robot_lab/leg_loco",
             "LW/robot_lab/wheel_loco",
             "LW/robot_lab/leg_to_wheel",
             "LW/robot_lab/wheel_to_leg"})
    {
        const fs::path config_path = policy_root / path / "config.yaml";
        const auto validated = ValidateLWPolicyConfiguration(
            base,
            loadConfig(config_path, path),
            config_path.string());
        const auto* reference = validated.runtime.needs_motion_reference
            ? &motion
            : nullptr;
        const auto expected = referenceObservation(
            validated.runtime, observations, reference);
        std::vector<float> actual(validated.dimensions.observation, 0.0F);
        const float* const data = actual.data();
        runtime.ComputeLWObservationInto(
            validated.runtime, observations, reference, actual);
        require(actual.data() == data, std::string(path) + " replaced output");
        require(actual.size() == expected.size(), std::string(path) + " size differs");
        for (std::size_t index = 0; index < actual.size(); ++index)
        {
            require(
                std::fabs(actual[index] - expected[index]) <= 1.0e-6F,
                std::string(path) + " observation differs at index "
                    + std::to_string(index));
        }
    }
}

void testCurrentLWConfigurationsAndModels()
{
    const fs::path policy_root(POLICY_DIR);
    const YAML::Node base =
        loadConfig(policy_root / "LW/base.yaml", "LW");
    const auto base_runtime =
        ValidateLWBaseConfiguration(base, "LW/base.yaml");
    require(base_runtime.num_dofs == 10, "base runtime DOF count differs");
    require(
        base_runtime.joint_names.size() == base_runtime.num_dofs
            && base_runtime.joint_names.front() == "right_hip_joint"
            && base_runtime.joint_names.back() == "left_wheel_joint",
        "base runtime joint names were not retained");
    require(
        base_runtime.joint_mapping.size() == base_runtime.num_dofs,
        "base runtime joint mapping was not retained");
    require(
        base_runtime.wheel_mask.size() == base_runtime.num_dofs
            && base_runtime.wheel_mask[8] != 0
            && base_runtime.wheel_mask[9] != 0,
        "base runtime wheel mask was not decoded");

    struct ExpectedPolicy
    {
        std::string relative_path;
        std::size_t observation;
        std::size_t input;
        std::vector<float> output;
    };
    const std::vector<ExpectedPolicy> policies = {
        {"LW/robot_lab/leg_loco", 41, 410,
         {0x1.8b4598p-1F, 0x1.f2ce2cp-1F, 0x1.f70562p+0F,
          0x1.e1fd02p+1F, 0x1.719a36p+0F, 0x1.a9b186p+0F,
          0x1.56c2cap+1F, -0x1.f2ad64p+0F, 0x1.d5ac8p-1F,
          0x1.28096cp-1F}},
        {"LW/robot_lab/wheel_loco", 39, 195,
         {0x1.18241ap-1F, -0x1.aa31ep-1F, -0x1.a82328p-2F,
          0x1.6c19f6p+1F, 0x1.ee7c34p-4F, -0x1.249d04p-1F,
          -0x1.5bb052p-2F, -0x1.0cf112p-3F, -0x1.0b6872p+2F,
          0x1.42712ap+2F}},
        {"LW/robot_lab/leg_to_wheel", 59, 59,
         {-0x1.585e7p-2F, -0x1.3f41a4p+0F, -0x1.49eb6cp+0F,
          -0x1.a6f982p-1F, 0x1.8bfbeap+0F, -0x1.60f2bp+1F,
          -0x1.8798c4p-2F, 0x1.80d9bp-2F, -0x1.2d3aacp+1F,
          0x1.1eea08p+0F}},
        {"LW/robot_lab/wheel_to_leg", 59, 59,
         {-0x1.292a8cp+1F, 0x1.882dbp+0F, 0x1.7a6574p-2F,
          0x1.f2e5bep-2F, 0x1.b518acp-1F, -0x1.e03cacp+0F,
          -0x1.f3abfap-1F, 0x1.2a31f2p-2F, 0x1.dde794p+2F,
          -0x1.4bd34ap+2F}},
    };

    for (const auto& policy : policies)
    {
        const fs::path config_path =
            policy_root / policy.relative_path / "config.yaml";
        const YAML::Node policy_config =
            loadConfig(config_path, policy.relative_path);
        const auto validated = ValidateLWPolicyConfiguration(
            base, policy_config, config_path.string());
        require(
            validated.dimensions.observation == policy.observation,
            policy.relative_path + " computed the wrong observation size");
        require(
            validated.dimensions.model_input == policy.input,
            policy.relative_path + " computed the wrong model input size");
        require(
            validated.dimensions.model_output == 10,
            policy.relative_path + " computed the wrong action size");
        require(
            validated.runtime.num_dofs == 10
                && validated.runtime.period_seconds > 0.0f
                && validated.runtime.output_max_age_seconds
                    == 3.0f * validated.runtime.period_seconds,
            policy.relative_path + " typed timing configuration differs");
        require(
            validated.runtime.observations.size()
                == policy_config["observations"].size(),
            policy.relative_path + " observation list was not retained");
        require(
            validated.runtime.observation_layout.size()
                == validated.runtime.observations.size()
                && validated.runtime.observation_layout.back().offset
                        + validated.runtime.observation_layout.back().size
                    == policy.observation,
            policy.relative_path + " observation layout differs");
        require(
            validated.runtime.wheel_mask.size() == 10,
            policy.relative_path + " wheel mask was not decoded");
        const bool expected_motion =
            policy.relative_path.find("_to_") != std::string::npos;
        require(
            validated.runtime.needs_motion_reference == expected_motion,
            policy.relative_path + " motion requirement differs");

        const fs::path model_path =
            policy_root / policy.relative_path / "policy.onnx";
        auto model = InferenceRuntime::ModelFactory::load_model(
            model_path.string());
        require(model != nullptr, "failed to load " + model_path.string());
        ValidateLWModelContract(
            *model, validated.dimensions, model_path.string(), 1);
        std::vector<float> input(policy.input);
        for (std::size_t index = 0; index < input.size(); ++index)
        {
            input[index] = static_cast<float>(
                static_cast<int>(index % 23) - 11) * 0.03125F;
        }
        std::vector<float> output(policy.output.size(), 0.0F);
        const float* const output_data = output.data();
        const InferenceRuntime::TensorView input_view = {
            input.data(), input.size()};
        model->forwardInto(
            &input_view,
            1,
            {output.data(), output.size()});
        require(
            output.data() == output_data,
            policy.relative_path + " replaced caller-owned output storage");
        for (std::size_t index = 0; index < output.size(); ++index)
        {
            require(
                std::fabs(output[index] - policy.output[index]) <= 1.0e-6F,
                policy.relative_path + " output differs at index "
                    + std::to_string(index));
        }
    }
}

void testSparseHistoryUsesSelectedFrameCountForModelInput()
{
    const fs::path policy_root(POLICY_DIR);
    const YAML::Node base =
        loadConfig(policy_root / "LW/base.yaml", "LW");
    const fs::path config_path =
        policy_root / "LW/robot_lab/leg_loco/config.yaml";
    YAML::Node sparse_history =
        YAML::Clone(loadConfig(config_path, "LW/robot_lab/leg_loco"));
    sparse_history["observations_history"] = std::vector<int>{9};

    const auto validated = ValidateLWPolicyConfiguration(
        base,
        sparse_history,
        "sparse-history-policy");
    require(
        validated.runtime.observations_history == std::vector<int>{9},
        "sparse history frame was not retained");
    require(
        validated.dimensions.model_input
            == validated.dimensions.observation,
        "one selected sparse frame did not produce one observation frame");
}

void testMotionObservationContracts()
{
    const fs::path policy_root(POLICY_DIR);
    const YAML::Node base =
        loadConfig(policy_root / "LW/base.yaml", "LW");
    const fs::path config_path =
        policy_root / "LW/robot_lab/leg_to_wheel/config.yaml";
    const YAML::Node original = loadConfig(
        config_path, "LW/robot_lab/leg_to_wheel");

    const auto require_contract =
        [&](const std::vector<std::string>& observations,
            int expected_dimension,
            bool expected_motion_reference,
            const std::string& source)
        {
            YAML::Node candidate = YAML::Clone(original);
            candidate["observations"] = observations;
            candidate["num_observations"] = expected_dimension;
            const auto validated = ValidateLWPolicyConfiguration(
                base, candidate, source);
            require(
                validated.dimensions.observation
                    == static_cast<std::size_t>(expected_dimension),
                source + " observation dimension differs");
            require(
                validated.runtime.needs_motion_reference
                    == expected_motion_reference,
                source + " motion-reference contract differs");
        };

    require_contract(
        {"whole_body_tracking/motion_command"},
        20,
        true,
        "motion-command-only");
    require_contract(
        {"whole_body_tracking/motion_anchor_ori_b"},
        6,
        true,
        "motion-anchor-only");
    require_contract(
        {"whole_body_tracking/motion_command",
         "whole_body_tracking/motion_anchor_ori_b"},
        26,
        true,
        "motion-command-and-anchor");
    require_contract({"ang_vel"}, 3, false, "non-motion");

    YAML::Node removed_phase = YAML::Clone(original);
    removed_phase["observations"] =
        std::vector<std::string>{"RoboMimic_Deploy/phase"};
    removed_phase["num_observations"] = 1;
    requireFailure(
        [&]() {
            ValidateLWPolicyConfiguration(
                base, removed_phase, "removed-motion-phase");
        },
        "unsupported observation 'RoboMimic_Deploy/phase'");
}

void testInvalidBaseConfigurationIsRejected()
{
    const fs::path policy_root(POLICY_DIR);
    const YAML::Node original =
        loadConfig(policy_root / "LW/base.yaml", "LW");

    YAML::Node missing = YAML::Clone(original);
    missing.remove("dt");
    requireFailure(
        [&]() { ValidateLWBaseConfiguration(missing, "missing-base"); },
        "missing required key 'dt'");

    YAML::Node short_vector = YAML::Clone(original);
    short_vector["rl_kp"] = std::vector<float>(9, 1.0f);
    requireFailure(
        [&]() {
            ValidateLWBaseConfiguration(short_vector, "short-base");
        },
        "rl_kp' must contain 10 values");

    YAML::Node non_finite = YAML::Clone(original);
    auto torque_limits =
        non_finite["torque_limits"].as<std::vector<float>>();
    torque_limits[3] = std::numeric_limits<float>::infinity();
    non_finite["torque_limits"] = torque_limits;
    requireFailure(
        [&]() {
            ValidateLWBaseConfiguration(non_finite, "non-finite-base");
        },
        "non-finite value at index 3");

    YAML::Node duplicate_mapping = YAML::Clone(original);
    duplicate_mapping["joint_mapping"] =
        std::vector<int>{0, 1, 2, 3, 4, 5, 6, 7, 8, 8};
    requireFailure(
        [&]() {
            ValidateLWBaseConfiguration(
                duplicate_mapping, "duplicate-base");
        },
        "duplicate index 8");

    YAML::Node out_of_range = YAML::Clone(original);
    out_of_range["wheel_indices"] = std::vector<int>{8, 10};
    requireFailure(
        [&]() {
            ValidateLWBaseConfiguration(out_of_range, "range-base");
        },
        "outside [0, 10)");
}

void testInvalidPolicyConfigurationIsRejected()
{
    const fs::path policy_root(POLICY_DIR);
    const YAML::Node base =
        loadConfig(policy_root / "LW/base.yaml", "LW");
    const fs::path config_path =
        policy_root / "LW/robot_lab/leg_loco/config.yaml";
    const YAML::Node original =
        loadConfig(config_path, "LW/robot_lab/leg_loco");

    YAML::Node missing = YAML::Clone(original);
    missing.remove("action_scale");
    requireFailure(
        [&]() {
            ValidateLWPolicyConfiguration(base, missing, "missing-policy");
        },
        "missing required key 'action_scale'");

    YAML::Node unknown_observation = YAML::Clone(original);
    unknown_observation["observations"] =
        std::vector<std::string>{"ang_vel", "misspelled_observation"};
    requireFailure(
        [&]() {
            ValidateLWPolicyConfiguration(
                base, unknown_observation, "unknown-policy");
        },
        "unsupported observation 'misspelled_observation'");

    YAML::Node wrong_dimension = YAML::Clone(original);
    wrong_dimension["num_observations"] = 40;
    requireFailure(
        [&]() {
            ValidateLWPolicyConfiguration(
                base, wrong_dimension, "dimension-policy");
        },
        "num_observations expected 41, got 40");

    YAML::Node bad_priority = YAML::Clone(original);
    bad_priority["observations_history_priority"] = "frames";
    requireFailure(
        [&]() {
            ValidateLWPolicyConfiguration(
                base, bad_priority, "priority-policy");
        },
        "must be 'time' or 'term'");

    YAML::Node duplicate_history = YAML::Clone(original);
    duplicate_history["observations_history"] =
        std::vector<int>{2, 1, 1, 0};
    requireFailure(
        [&]() {
            ValidateLWPolicyConfiguration(
                base, duplicate_history, "history-policy");
        },
        "duplicate index 1");

    const fs::path motion_config_path =
        policy_root / "LW/robot_lab/leg_to_wheel/config.yaml";
    const YAML::Node motion_original = loadConfig(
        motion_config_path, "LW/robot_lab/leg_to_wheel");
    YAML::Node missing_time_offset = YAML::Clone(motion_original);
    missing_time_offset.remove("motion_time_offset_frames");
    requireFailure(
        [&]() {
            ValidateLWPolicyConfiguration(
                base,
                missing_time_offset,
                "missing-motion-time-offset");
        },
        "missing required key 'motion_time_offset_frames'");

    YAML::Node negative_time_offset = YAML::Clone(motion_original);
    negative_time_offset["motion_time_offset_frames"] = -1;
    requireFailure(
        [&]() {
            ValidateLWPolicyConfiguration(
                base,
                negative_time_offset,
                "negative-motion-time-offset");
        },
        "motion_time_offset_frames' must be nonnegative");
}

void testModelDimensionMismatchIsRejected()
{
    const LWPolicyDimensions dimensions{10, 59, 59, 10};
    FakeONNXModel bad_input(58, 10);
    requireFailure(
        [&]() {
            ValidateLWModelContract(
                bad_input, dimensions, "bad-input.onnx", 0);
        },
        "input feature dimension expected 59, got 58");

    FakeONNXModel bad_output(59, 9);
    requireFailure(
        [&]() {
            ValidateLWModelContract(
                bad_output, dimensions, "bad-output.onnx", 0);
        },
        "output feature dimension expected 10, got 9");

    FakeONNXModel dynamic_batch(59, 10, -1);
    requireFailure(
        [&]() {
            ValidateLWModelContract(
                dynamic_batch, dimensions, "dynamic-batch.onnx", 0);
        },
        "input batch dimension must be fixed at 1, got -1");

    FakeONNXModel dynamic_output_batch(59, 10, 1, -1);
    requireFailure(
        [&]() {
            ValidateLWModelContract(
                dynamic_output_batch,
                dimensions,
                "dynamic-output-batch.onnx",
                0);
        },
        "output batch dimension must be fixed at 1, got -1");
}

void testObservationQuaternionUsesWxyzIdentity()
{
    TestRL runtime;
    runtime.params.config_node["num_of_dofs"] = 10;
    runtime.params.config_node["default_dof_pos"] =
        std::vector<float>(10, 0.0f);
    runtime.params.config_node["observations"] =
        std::vector<std::string>{};
    runtime.params.config_node["clip_obs"] = 1.0f;
    runtime.InitObservations();
    require(
        runtime.obs.base_quat
            == std::vector<float>({1.0f, 0.0f, 0.0f, 0.0f}),
        "InitObservations did not use the w,x,y,z identity quaternion");
}
} // namespace

int main()
{
    try
    {
        testCurrentLWConfigurationsAndModels();
        testContiguousObservationAssemblyMatchesPreviousOrdering();
        testSparseHistoryUsesSelectedFrameCountForModelInput();
        testMotionObservationContracts();
        testInvalidBaseConfigurationIsRejected();
        testInvalidPolicyConfigurationIsRejected();
        testModelDimensionMismatchIsRejected();
        testObservationQuaternionUsesWxyzIdentity();
    }
    catch (const std::exception& exception)
    {
        std::cerr << "test_lw_configuration_validation failed: "
                  << exception.what() << std::endl;
        return 1;
    }
    std::cout << "test_lw_configuration_validation passed" << std::endl;
    return 0;
}
