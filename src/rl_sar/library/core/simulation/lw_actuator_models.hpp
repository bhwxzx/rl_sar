#ifndef LW_ACTUATOR_MODELS_HPP
#define LW_ACTUATOR_MODELS_HPP

#include "inference_runtime.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

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

float EvaluateLWActuatorModelOutput(
    InferenceRuntime::Model& model,
    const std::vector<float>& input,
    const std::filesystem::path& model_path);

std::shared_ptr<InferenceRuntime::Model> LoadLWActuatorModel(
    const std::filesystem::path& model_path);

class LWActuatorTorqueFrame
{
public:
    void resize(std::size_t size);
    std::vector<float>& beginUpdate() noexcept;
    void commit(std::uint64_t generation);

    const std::vector<float>& values() const noexcept;
    std::uint64_t generation() const noexcept;

private:
    std::vector<float> candidate_;
    std::vector<float> committed_;
    std::uint64_t generation_ = 0;
};

enum class LWActuatorTorqueFailure
{
    None,
    SizeMismatch,
    NonFiniteCandidate,
    InvalidLimit
};

struct LWActuatorTorqueValidation
{
    LWActuatorTorqueFailure failure = LWActuatorTorqueFailure::None;
    std::size_t index = 0;

    bool valid() const noexcept
    {
        return failure == LWActuatorTorqueFailure::None;
    }

    const char* failureName() const noexcept;
};

LWActuatorTorqueValidation PrepareLWActuatorTorques(
    const std::vector<float>& candidates,
    const std::vector<float>& limits,
    std::vector<float>& bounded) noexcept;

#endif // LW_ACTUATOR_MODELS_HPP
