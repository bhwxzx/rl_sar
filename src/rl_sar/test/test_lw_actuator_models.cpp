#include "lw_actuator_models.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <unistd.h>

namespace
{
namespace fs = std::filesystem;

void Require(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

void RequireFailure(
    const std::function<void()>& operation,
    const std::vector<std::string>& expected_text)
{
    try
    {
        operation();
    }
    catch (const std::exception& exception)
    {
        const std::string message = exception.what();
        for (const std::string& text : expected_text)
        {
            Require(
                message.find(text) != std::string::npos,
                "unexpected actuator-model error: " + message);
        }
        return;
    }
    throw std::runtime_error("expected actuator-model validation failure");
}

class TemporaryDirectory
{
public:
    TemporaryDirectory()
    {
        const auto nonce = std::chrono::steady_clock::now()
                               .time_since_epoch()
                               .count();
        path_ = fs::temp_directory_path()
            / ("lw-actuator-model-test-" + std::to_string(getpid())
               + "-" + std::to_string(nonce));
        fs::create_directories(path_);
    }

    ~TemporaryDirectory()
    {
        std::error_code ignored;
        fs::remove_all(path_, ignored);
    }

    const fs::path& path() const noexcept
    {
        return path_;
    }

private:
    fs::path path_;
};

void CreatePlaceholder(const fs::path& path)
{
    fs::create_directories(path.parent_path());
    std::ofstream output(path);
    output << "test model placeholder";
    Require(output.good(), "failed to create " + path.string());
}

class FakeTorchModel : public InferenceRuntime::Model
{
public:
    enum class Output
    {
        Valid,
        Empty,
        TooLarge,
        NonFinite,
        Infinite,
        Throw
    };

    explicit FakeTorchModel(
        Output output = Output::Valid,
        bool loaded = true,
        std::string model_type = "torch")
        : output_(output),
          loaded_(loaded),
          model_type_(std::move(model_type))
    {
    }

    bool load(const std::string&) override
    {
        return loaded_;
    }

    bool is_loaded() const override
    {
        return loaded_;
    }

    std::vector<float> forward(
        const std::vector<std::vector<float>>& inputs) override
    {
        observed_input_size_ =
            inputs.empty() ? 0 : inputs.front().size();
        if (output_ == Output::Throw)
        {
            throw std::runtime_error("incompatible input shape");
        }
        if (output_ == Output::Empty)
        {
            return {};
        }
        if (output_ == Output::TooLarge)
        {
            return {0.0f, 0.0f};
        }
        if (output_ == Output::NonFinite)
        {
            return {std::numeric_limits<float>::quiet_NaN()};
        }
        if (output_ == Output::Infinite)
        {
            return {std::numeric_limits<float>::infinity()};
        }
        return {valid_output_};
    }

    std::string get_model_type() const override
    {
        return model_type_;
    }

    std::size_t observed_input_size() const noexcept
    {
        return observed_input_size_;
    }

    void set_output(Output output) noexcept
    {
        output_ = output;
    }

    void set_valid_output(float output) noexcept
    {
        valid_output_ = output;
    }

private:
    Output output_;
    bool loaded_;
    std::string model_type_;
    std::size_t observed_input_size_ = 0;
    float valid_output_ = 0.0f;
};

void TestRelocatedPolicyRootSelectsBothModels()
{
    TemporaryDirectory temporary;
    const fs::path selected_root = temporary.path() / "relocated-policy";
    const fs::path motors = selected_root / "LW/robot_lab/motors";
#ifdef USE_TORCH
    fs::create_directories(motors);
    fs::copy_file(
        fs::path(POLICY_DIR)
            / "LW/robot_lab/motors/leg_actuator_net.pt",
        motors / "leg_actuator_net.pt");
    fs::copy_file(
        fs::path(POLICY_DIR)
            / "LW/robot_lab/motors/foot_actuator_net.pt",
        motors / "foot_actuator_net.pt");
#else
    CreatePlaceholder(motors / "leg_actuator_net.pt");
    CreatePlaceholder(motors / "foot_actuator_net.pt");
#endif

    const LWActuatorModelPaths paths =
        ResolveLWActuatorModelPaths(selected_root, "LW");
    const fs::path canonical_root = fs::canonical(selected_root);
    Require(
        paths.leg
            == canonical_root
                / "LW/robot_lab/motors/leg_actuator_net.pt",
        "leg actuator model did not follow the relocated policy root");
    Require(
        paths.foot
            == canonical_root
                / "LW/robot_lab/motors/foot_actuator_net.pt",
        "foot actuator model did not follow the relocated policy root");
#ifdef USE_TORCH
    Require(
        LoadLWActuatorModel(paths.leg)
            && LoadLWActuatorModel(paths.foot),
        "relocated actuator models were not loadable");
#endif
}

void TestMissingModelIsRejectedWithResolvedPath()
{
    TemporaryDirectory temporary;
    const fs::path selected_root = temporary.path() / "selected-policy";
    const fs::path motors = selected_root / "LW/robot_lab/motors";
    CreatePlaceholder(motors / "leg_actuator_net.pt");
    fs::create_directories(selected_root);

    const fs::path expected_missing =
        fs::canonical(selected_root)
        / "LW/robot_lab/motors/foot_actuator_net.pt";
    RequireFailure(
        [&]() { ResolveLWActuatorModelPaths(selected_root, "LW"); },
        {"Required LW actuator model", expected_missing.string()});
}

void TestCompatibleModelUsesSixInputsAndOneOutput()
{
    FakeTorchModel model;
    ValidateLWActuatorModelContract(model, "/selected/leg_actuator_net.pt");
    Require(
        model.observed_input_size() == 6,
        "actuator-model validation used the wrong input width");
}

void TestIncompatibleModelsAreRejected()
{
    const fs::path model_path("/selected/foot_actuator_net.pt");

    FakeTorchModel unloaded(FakeTorchModel::Output::Valid, false);
    RequireFailure(
        [&]() { ValidateLWActuatorModelContract(unloaded, model_path); },
        {model_path.string(), "not loaded"});

    FakeTorchModel wrong_type(
        FakeTorchModel::Output::Valid, true, "onnx");
    RequireFailure(
        [&]() { ValidateLWActuatorModelContract(wrong_type, model_path); },
        {model_path.string(), "TorchScript"});

    FakeTorchModel bad_input(FakeTorchModel::Output::Throw);
    RequireFailure(
        [&]() { ValidateLWActuatorModelContract(bad_input, model_path); },
        {model_path.string(), "expected 6-value input"});

    FakeTorchModel bad_output(FakeTorchModel::Output::TooLarge);
    RequireFailure(
        [&]() { ValidateLWActuatorModelContract(bad_output, model_path); },
        {model_path.string(), "exactly one output"});

    FakeTorchModel non_finite(FakeTorchModel::Output::NonFinite);
    RequireFailure(
        [&]() { ValidateLWActuatorModelContract(non_finite, model_path); },
        {model_path.string(), "non-finite"});
}

void TestRuntimeOutputsAreValidatedAfterWarmup()
{
    const fs::path model_path("/selected/leg_actuator_net.pt");
    FakeTorchModel model;
    ValidateLWActuatorModelContract(model, model_path);

    model.set_valid_output(1.25f);
    Require(
        EvaluateLWActuatorModelOutput(
            model, std::vector<float>(6, 0.5f), model_path)
            == 1.25f,
        "valid runtime actuator output changed");

    for (const auto output : {
             FakeTorchModel::Output::Empty,
             FakeTorchModel::Output::TooLarge})
    {
        model.set_output(output);
        RequireFailure(
            [&]() {
                EvaluateLWActuatorModelOutput(
                    model, std::vector<float>(6, 0.0f), model_path);
            },
            {model_path.string(), "exactly one output"});
    }
    for (const auto output : {
             FakeTorchModel::Output::NonFinite,
             FakeTorchModel::Output::Infinite})
    {
        model.set_output(output);
        RequireFailure(
            [&]() {
                EvaluateLWActuatorModelOutput(
                    model, std::vector<float>(6, 0.0f), model_path);
            },
            {model_path.string(), "non-finite"});
    }
    model.set_output(FakeTorchModel::Output::Throw);
    RequireFailure(
        [&]() {
            EvaluateLWActuatorModelOutput(
                model, std::vector<float>(6, 0.0f), model_path);
        },
        {model_path.string(), "incompatible input shape"});
}

void TestActuatorTorqueFrameCommitsAtomically()
{
    LWActuatorTorqueFrame frame;
    frame.resize(3);
    auto& first = frame.beginUpdate();
    first = {1.0f, 2.0f, 3.0f};
    frame.commit(7);
    Require(frame.generation() == 7, "valid torque generation was not committed");
    Require(
        frame.values() == std::vector<float>({1.0f, 2.0f, 3.0f}),
        "valid torque frame was not committed");

    auto& failed = frame.beginUpdate();
    failed[0] = 9.0f;
    Require(frame.generation() == 0, "failed update retained an old generation");
    Require(
        frame.values() == std::vector<float>({1.0f, 2.0f, 3.0f}),
        "partial torque candidate leaked into committed values");

    failed = {4.0f, 5.0f, 6.0f};
    frame.commit(8);
    Require(frame.generation() == 8, "replacement generation was not committed");
    Require(
        frame.values() == std::vector<float>({4.0f, 5.0f, 6.0f}),
        "replacement torque frame was not committed atomically");
}

void TestFinalActuatorTorquesValidateBeforeMutation()
{
    std::vector<float> bounded(3, 99.0f);
    auto result = PrepareLWActuatorTorques(
        {2.0f, -3.0f, 0.5f},
        {1.0f, 2.0f, 1.0f},
        bounded);
    Require(result.valid(), "finite final torques were rejected");
    Require(
        bounded == std::vector<float>({1.0f, -2.0f, 0.5f}),
        "finite final torques were not clamped correctly");

    const std::vector<float> unchanged(3, 77.0f);
    for (const float invalid : {
             std::numeric_limits<float>::quiet_NaN(),
             std::numeric_limits<float>::infinity()})
    {
        bounded = unchanged;
        result = PrepareLWActuatorTorques(
            {1.0f, invalid, 2.0f},
            {3.0f, 3.0f, 3.0f},
            bounded);
        Require(!result.valid(), "non-finite final torque was accepted");
        Require(result.index == 1, "wrong invalid final-torque index reported");
        Require(bounded == unchanged, "invalid torque caused a partial mutation");
    }

    bounded = unchanged;
    result = PrepareLWActuatorTorques(
        {1.0f, 2.0f, 3.0f},
        {3.0f, -1.0f, 3.0f},
        bounded);
    Require(!result.valid(), "invalid torque limit was accepted");
    Require(bounded == unchanged, "invalid limit caused a partial mutation");
}

void TestCurrentModelsLoadWithoutGui()
{
#ifdef USE_TORCH
    const LWActuatorModelPaths paths =
        ResolveLWActuatorModelPaths(POLICY_DIR, "LW");
    const std::shared_ptr<InferenceRuntime::Model> leg =
        LoadLWActuatorModel(paths.leg);
    const std::shared_ptr<InferenceRuntime::Model> foot =
        LoadLWActuatorModel(paths.foot);
    Require(leg && foot, "current actuator models were not loaded");
#endif
}
}

int main()
{
    try
    {
        TestRelocatedPolicyRootSelectsBothModels();
        TestMissingModelIsRejectedWithResolvedPath();
        TestCompatibleModelUsesSixInputsAndOneOutput();
        TestIncompatibleModelsAreRejected();
        TestRuntimeOutputsAreValidatedAfterWarmup();
        TestActuatorTorqueFrameCommitsAtomically();
        TestFinalActuatorTorquesValidateBeforeMutation();
        TestCurrentModelsLoadWithoutGui();
    }
    catch (const std::exception& exception)
    {
        std::cerr << "test_lw_actuator_models failed: "
                  << exception.what() << std::endl;
        return 1;
    }
    std::cout << "test_lw_actuator_models passed" << std::endl;
    return 0;
}
