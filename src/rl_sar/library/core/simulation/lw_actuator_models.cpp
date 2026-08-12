#include "lw_actuator_models.hpp"

#include <cmath>
#include <stdexcept>
#include <system_error>
#include <vector>

namespace
{
constexpr std::size_t kActuatorModelInputSize = 6;
constexpr std::size_t kActuatorModelOutputSize = 1;

std::filesystem::path RequireModelFile(
    const std::filesystem::path& policy_root,
    const std::filesystem::path& relative_path)
{
    const std::filesystem::path model_path =
        (policy_root / relative_path).lexically_normal();
    std::error_code error;
    const bool is_model_file =
        std::filesystem::is_regular_file(model_path, error);
    if (error || !is_model_file)
    {
        throw std::runtime_error(
            "Required LW actuator model is not a readable file: "
            + model_path.string());
    }
    return model_path;
}
}

LWActuatorModelPaths ResolveLWActuatorModelPaths(
    const std::filesystem::path& policy_root,
    const std::string& robot_name)
{
    if (robot_name.empty()
        || std::filesystem::path(robot_name).is_absolute()
        || std::filesystem::path(robot_name).filename() != robot_name)
    {
        throw std::runtime_error(
            "Invalid robot name for LW actuator models: " + robot_name);
    }

    std::error_code error;
    const std::filesystem::path canonical_root =
        std::filesystem::canonical(policy_root, error);
    if (error || !std::filesystem::is_directory(canonical_root))
    {
        throw std::runtime_error(
            "Actuator-model policy root is not a readable directory: "
            + policy_root.string());
    }

    const std::filesystem::path motors =
        std::filesystem::path(robot_name) / "robot_lab" / "motors";
    return {
        RequireModelFile(
            canonical_root, motors / "leg_actuator_net.pt"),
        RequireModelFile(
            canonical_root, motors / "foot_actuator_net.pt")};
}

void ValidateLWActuatorModelContract(
    InferenceRuntime::Model& model,
    const std::filesystem::path& model_path)
{
    const std::string source = model_path.string();
    if (!model.is_loaded())
    {
        throw std::runtime_error(
            "LW actuator model is not loaded: " + source);
    }
    if (model.get_model_type() != "torch")
    {
        throw std::runtime_error(
            "LW actuator model must be TorchScript: " + source);
    }

    std::vector<float> output;
    try
    {
        output = model.forward(
            {std::vector<float>(kActuatorModelInputSize, 0.0f)});
    }
    catch (const std::exception& exception)
    {
        throw std::runtime_error(
            "LW actuator model rejected the expected 6-value input '"
            + source + "': " + exception.what());
    }
    if (output.size() != kActuatorModelOutputSize)
    {
        throw std::runtime_error(
            "LW actuator model must produce exactly one output '"
            + source + "': got " + std::to_string(output.size()));
    }
    if (!std::isfinite(output.front()))
    {
        throw std::runtime_error(
            "LW actuator model produced a non-finite warmup output: "
            + source);
    }
}

std::shared_ptr<InferenceRuntime::Model> LoadLWActuatorModel(
    const std::filesystem::path& model_path)
{
    std::unique_ptr<InferenceRuntime::Model> model =
        InferenceRuntime::ModelFactory::load_model(model_path.string());
    if (!model)
    {
        throw std::runtime_error(
            "Failed to load LW actuator model: " + model_path.string());
    }
    ValidateLWActuatorModelContract(*model, model_path);
    return std::shared_ptr<InferenceRuntime::Model>(std::move(model));
}
