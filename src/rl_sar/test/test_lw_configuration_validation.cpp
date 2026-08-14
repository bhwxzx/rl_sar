#include "lw_configuration_validation.hpp"
#include "rl_sdk.hpp"

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
    FakeONNXModel(std::int64_t input_features, std::int64_t output_features)
        : inputs_{{"observations",
                   {1, input_features},
                   InferenceRuntime::TensorElementType::Float32}},
          outputs_{{"actions",
                    {1, output_features},
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

    std::vector<float> forward(
        const std::vector<std::vector<float>>&) override
    {
        return std::vector<float>(
            static_cast<std::size_t>(output_features_), 0.0f);
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

void testCurrentLWConfigurationsAndModels()
{
    const fs::path policy_root(POLICY_DIR);
    const YAML::Node base =
        loadConfig(policy_root / "LW/base.yaml", "LW");
    const auto base_runtime =
        ValidateLWBaseConfiguration(base, "LW/base.yaml");
    require(base_runtime.num_dofs == 10, "base runtime DOF count differs");
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
    };
    const std::vector<ExpectedPolicy> policies = {
        {"LW/robot_lab/leg_loco", 41, 410},
        {"LW/robot_lab/wheel_loco", 39, 195},
        {"LW/robot_lab/leg_to_wheel", 59, 59},
        {"LW/robot_lab/wheel_to_leg", 59, 59},
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
    }
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
