#include "lw_configuration_validation.hpp"
#include "rl_sdk.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <functional>
#include <iostream>
#include <iomanip>
#include <limits>
#include <memory>
#include <stdexcept>
#include <sstream>
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

// Fixed-input FP32 regression: absolute tolerance near zero plus a small
// relative allowance for larger outputs across CPU inference implementations.
// This is a test comparison only; model weights and stored baselines stay fixed.
void requireModelOutputClose(
    float actual, float expected, const std::string& policy, std::size_t index)
{
    const double error = std::fabs(
        static_cast<double>(actual) - static_cast<double>(expected));
    const double allowed_error = 2.0e-6
        + 1.0e-6 * std::fabs(static_cast<double>(expected));
    if (std::isfinite(actual) && std::isfinite(expected) && error <= allowed_error)
    {
        return;
    }
    std::ostringstream message;
    message << std::setprecision(std::numeric_limits<double>::max_digits10)
            << policy << " output differs at index " << index
            << ": expected=" << expected << ", actual=" << actual
            << ", abs_error=" << error << ", allowed_error=" << allowed_error;
    throw std::runtime_error(message.str());
}

void testModelOutputTolerance()
{
    // Adjacent representable float values straddle independently specified
    // tolerance boundaries, including both signs and the near-zero case.
    const std::vector<std::pair<float, double>> boundaries = {
        {0.0F, 2.0e-6}, {1.0F, 3.0e-6}, {-1.0F, 3.0e-6}, {100.0F, 102.0e-6}};
    for (const auto& boundary : boundaries)
    {
        for (const int direction : {-1, 1})
        {
            const float expected = boundary.first;
            float inside = static_cast<float>(
                static_cast<double>(expected) + direction * boundary.second);
            if (std::fabs(static_cast<double>(inside) - expected) > boundary.second)
            {
                inside = std::nextafter(inside, expected);
            }
            const float outside = std::nextafter(
                inside, direction > 0 ? std::numeric_limits<float>::infinity()
                                      : -std::numeric_limits<float>::infinity());
            requireModelOutputClose(inside, expected, "boundary", 7);
            requireFailure(
                [&] { requireModelOutputClose(outside, expected, "boundary", 7); },
                "boundary output differs at index 7");
        }
    }
    // The measured Jetson deviations are allowed without replacing the oracle.
    requireModelOutputClose(-0.5564388036727905F, -0.5564398169517517F, "leg", 2);
    requireModelOutputClose(6.323897361755371F, 6.323895454406738F, "leg", 6);
    requireFailure(
        [] { requireModelOutputClose(0.0001F, 0.0F, "changed-output", 0); },
        "changed-output");
    for (const float invalid : {std::numeric_limits<float>::quiet_NaN(),
                               std::numeric_limits<float>::infinity(),
                               -std::numeric_limits<float>::infinity()})
    {
        requireFailure([&] { requireModelOutputClose(invalid, 0.0F, "invalid", 0); },
                       "invalid output differs");
        requireFailure([&] { requireModelOutputClose(0.0F, invalid, "invalid", 0); },
                       "invalid output differs");
        requireFailure([&] { requireModelOutputClose(invalid, invalid, "invalid", 0); },
                       "invalid output differs");
    }
    for (const auto* field : {"expected=", "actual=", "abs_error=", "allowed_error="})
    {
        requireFailure([] { requireModelOutputClose(1.0F, 0.0F, "diagnostic", 3); },
                       field);
    }
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
    const LWValidatedBaseConfiguration validated_base(base, "LW/base.yaml");
    require(base_runtime.sensor_timeout == base["sensor_timeout"].as<float>()
                && base_runtime.trusted_imu_timeout == base["trusted_imu_timeout"].as<float>()
                && base_runtime.imu_ahrs_pair_max_age == base["imu_ahrs_pair_max_age"].as<float>()
                && base_runtime.serial_write_timeout == base["serial_write_timeout"].as<float>(),
            "typed base timeouts differ from YAML");
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
    // Locomotion ONNX baselines cross-checked independently against the
    // archived JIT weights/forward structure with NumPy (not the JIT engine).
    const std::vector<ExpectedPolicy> policies = {
        // Archive: LW/leg_loco/2026-09-16-15-45-12
        // ONNX SHA-256: c5d94cd109557baaf3b3a57b9e146a95555ea7c323b538fcf8830aea829f42eb
        {"LW/robot_lab/leg_loco", 41, 410,
         {-0x1.3bc284p-1F, -0x1.1c118p+0F, -0x1.1ce5aep-1F,
          -0x1.38e3bp+0F, 0x1.6629bep+1F, 0x1.934b88p+1F,
          0x1.94bab4p+2F, -0x1.7ee4fep+0F, 0x1.240122p+2F,
          0x1.3be86ep+0F}},
        // Archive: LW/wheel_loco/2026-09-19-11-41-48
        // ONNX SHA-256: 689395ee03276cb3b2dcaa37db58c2b4921069df56c8b884007ce3abbafcbfcf
        {"LW/robot_lab/wheel_loco", 39, 390,
         {0x1.8f17ep-2F, -0x1.a5d8d8p-6F, -0x1.a60e3cp+0F,
          -0x1.8fd52p+0F, 0x1.e9292ap-3F, -0x1.1a8b06p-1F,
          0x1.43e334p-2F, 0x1.fd61cap-1F, -0x1.1f4694p+0F,
          0x1.4d0d7cp+2F}},
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
        const auto reused = validated_base.validatePolicy(policy_config, config_path.string());
        require(YAML::Dump(reused.merged) == YAML::Dump(validated.merged),
                policy.relative_path + " reused base changed merged configuration");
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
            requireModelOutputClose(
                output[index], policy.output[index], policy.relative_path, index);
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

void testValidatedBaseSnapshotAndRawEntry()
{
    const fs::path policy_root(POLICY_DIR);
    YAML::Node base = loadConfig(policy_root / "LW/base.yaml", "LW");
    const YAML::Node policy = loadConfig(
        policy_root / "LW/robot_lab/leg_loco/config.yaml", "LW/robot_lab/leg_loco");
    const LWValidatedBaseConfiguration validated(base, "base-snapshot");
    const auto original = validated.validatePolicy(policy, "policy");
    base["dt"] = -1.0f;
    base["joint_names"][0] = "mutated";
    auto first = validated.validatePolicy(policy, "policy");
    require(YAML::Dump(first.merged) == YAML::Dump(original.merged),
            "source YAML mutation changed validated snapshot");
    first.merged["dt"] = -2.0f;
    first.merged["joint_names"][0] = "also-mutated";
    require(YAML::Dump(validated.validatePolicy(policy, "policy").merged)
                == YAML::Dump(original.merged),
            "returned YAML mutation changed validated snapshot");
    requireFailure([&]() { ValidateLWPolicyConfiguration(base, policy, "raw-policy"); }, "dt");

    TestRL runtime;
    runtime.SetPolicyRoot(policy_root);
    runtime.params.config_node = base;
    runtime.SetLWBaseRuntimeConfiguration(validated);
    runtime.PreloadModel("LW/robot_lab/leg_loco");
    require(YAML::Dump(runtime.preloaded_lw_policy_configs_.at("LW/robot_lab/leg_loco").merged)
                == YAML::Dump(original.merged),
            "preload did not use the installed base snapshot");
    // Installing an unvalidated runtime value must not retain the old snapshot.
    runtime.SetLWBaseRuntimeConfiguration(validated.runtime());
    requireFailure([&]() { runtime.PreloadModel("LW/robot_lab/wheel_loco"); }, "dt");
}

void testBaseTimeoutValidation()
{
    const fs::path policy_root(POLICY_DIR);
    const YAML::Node base = loadConfig(policy_root / "LW/base.yaml", "LW");
    const YAML::Node policy = loadConfig(
        policy_root / "LW/robot_lab/leg_loco/config.yaml", "LW/robot_lab/leg_loco");
    for (const std::string key : {"policy_entry_angle_deg", "policy_entry_stable_time"}) {
        for (float invalid : {0.f, -1.f, std::numeric_limits<float>::quiet_NaN(),
                              std::numeric_limits<float>::infinity()}) {
            auto candidate=YAML::Clone(base);candidate[key]=invalid;
            requireFailure([&]() { LWValidatedBaseConfiguration checked(candidate,"entry"); },key);
        }
        auto legacy=YAML::Clone(base);legacy.remove(key);
        LWValidatedBaseConfiguration checked(legacy,"entry-defaults");
        require(checked.runtime().policy_entry_angle_deg==8.f
                && checked.runtime().policy_entry_stable_time==.5f,"entry defaults mismatch");
    }
    auto excessive=YAML::Clone(base);excessive["policy_entry_angle_deg"]=90.f;
    requireFailure([&]() { LWValidatedBaseConfiguration checked(excessive,"entry"); },"policy_entry_angle_deg");
    for (const std::string key : {"sensor_timeout", "trusted_imu_timeout",
                                  "imu_ahrs_pair_max_age", "serial_write_timeout"})
    {
        const auto reject = [&](const YAML::Node& candidate)
        {
            requireFailure([&]() { LWValidatedBaseConfiguration checked(candidate, "timeouts"); }, key);
            requireFailure([&]() { ValidateLWPolicyConfiguration(candidate, policy, "raw-policy"); }, key);
        };
        YAML::Node missing = YAML::Clone(base);
        missing.remove(key);
        reject(missing);
        YAML::Node wrong_type = YAML::Clone(base);
        wrong_type[key] = "not-a-number";
        reject(wrong_type);
        for (const float invalid : {0.0f, -1.0f,
                                    std::numeric_limits<float>::quiet_NaN(),
                                    std::numeric_limits<float>::infinity(),
                                    -std::numeric_limits<float>::infinity()})
        {
            YAML::Node candidate = YAML::Clone(base);
            candidate[key] = invalid;
            reject(candidate);
        }
    }
}

void testActionClippingConfigurationIsValidatedAtLoad()
{
    const fs::path policy_root(POLICY_DIR);
    const YAML::Node base = loadConfig(policy_root / "LW/base.yaml", "LW");
    const YAML::Node original = loadConfig(
        policy_root / "LW/robot_lab/leg_loco/config.yaml",
        "LW/robot_lab/leg_loco");
    for (const std::string key : {"clip_actions_lower", "clip_actions_upper"})
    {
        const auto reject = [&](const YAML::Node& candidate)
        {
            requireFailure(
                [&]() { ValidateLWPolicyConfiguration(base, candidate, "clipping"); },
                key);
        };
        YAML::Node missing = YAML::Clone(original);
        missing.remove(key);
        reject(missing);
        YAML::Node scalar = YAML::Clone(original);
        scalar[key] = "not-a-vector";
        reject(scalar);
        for (const std::size_t size : {0U, 9U, 11U})
        {
            YAML::Node candidate = YAML::Clone(original);
            candidate[key] = std::vector<float>(size, 0.0f);
            reject(candidate);
        }
        for (const float invalid : {std::numeric_limits<float>::quiet_NaN(),
                                    std::numeric_limits<float>::infinity(),
                                    -std::numeric_limits<float>::infinity()})
        {
            YAML::Node candidate = YAML::Clone(original);
            auto bounds = candidate[key].as<std::vector<float>>();
            bounds[3] = invalid;
            candidate[key] = bounds;
            reject(candidate);
        }
    }
    YAML::Node reversed = YAML::Clone(original);
    auto lower = reversed["clip_actions_lower"].as<std::vector<float>>();
    lower[0] = reversed["clip_actions_upper"][0].as<float>() + 1.0f;
    reversed["clip_actions_lower"] = lower;
    requireFailure(
        [&]() { ValidateLWPolicyConfiguration(base, reversed, "reversed-clipping"); },
        "clip_actions_lower exceeds clip_actions_upper");

    reversed["clip_actions_upper"][0] = lower[0];
    const auto equal = ValidateLWPolicyConfiguration(base, reversed, "equal-clipping");
    require(equal.runtime.clip_actions_lower[0] == equal.runtime.clip_actions_upper[0],
            "equal clipping bounds must remain valid");
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
        testModelOutputTolerance();
        testCurrentLWConfigurationsAndModels();
        testContiguousObservationAssemblyMatchesPreviousOrdering();
        testSparseHistoryUsesSelectedFrameCountForModelInput();
        testMotionObservationContracts();
        testInvalidBaseConfigurationIsRejected();
        testInvalidPolicyConfigurationIsRejected();
        testValidatedBaseSnapshotAndRawEntry();
        testBaseTimeoutValidation();
        testActionClippingConfigurationIsValidatedAtLoad();
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
