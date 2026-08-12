#ifndef LW_ACTUATOR_MODELS_HPP
#define LW_ACTUATOR_MODELS_HPP

#include "inference_runtime.hpp"

#include <filesystem>
#include <memory>
#include <string>

struct LWActuatorModelPaths
{
    std::filesystem::path leg;
    std::filesystem::path foot;
};

LWActuatorModelPaths ResolveLWActuatorModelPaths(
    const std::filesystem::path& policy_root,
    const std::string& robot_name);

void ValidateLWActuatorModelContract(
    InferenceRuntime::Model& model,
    const std::filesystem::path& model_path);

std::shared_ptr<InferenceRuntime::Model> LoadLWActuatorModel(
    const std::filesystem::path& model_path);

#endif // LW_ACTUATOR_MODELS_HPP
