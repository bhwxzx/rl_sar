#include "lw_actuator_models.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <system_error>
#include <vector>

namespace
{
constexpr std::size_t kActuatorModelInputSize = 6;
constexpr std::size_t kActuatorModelOutputSize = 1;

bool IsWithin(
    const std::filesystem::path& root,
    const std::filesystem::path& candidate)
{
    auto root_component = root.begin();
    auto candidate_component = candidate.begin();
    while (root_component != root.end()
           && candidate_component != candidate.end())
    {
        if (*root_component != *candidate_component)
        {
            return false;
        }
        ++root_component;
        ++candidate_component;
    }
    return root_component == root.end();
}

std::string DescribeResolvedPath(
    const std::filesystem::path& resolved_path,
    const std::error_code& error)
{
    if (error)
    {
        return "<unresolved: " + error.message() + ">";
    }
    return resolved_path.string();
}

[[noreturn]] void RejectModelPath(
    const std::string& reason,
    const std::filesystem::path& lexical_path,
    const std::filesystem::path& resolved_path,
    const std::error_code& resolution_error)
{
    throw std::runtime_error(
        reason + "; lexical path: " + lexical_path.string()
        + "; resolved path: "
        + DescribeResolvedPath(resolved_path, resolution_error));
}

std::filesystem::path RequireModelFile(
    const std::filesystem::path& policy_root,
    const std::filesystem::path& relative_path)
{
    const std::filesystem::path normalized_relative =
        relative_path.lexically_normal();
    if (relative_path.empty()
        || relative_path.is_absolute()
        || normalized_relative != relative_path)
    {
        throw std::runtime_error(
            "Invalid relative LW actuator-model path: "
            + relative_path.string());
    }
    for (const auto& component : normalized_relative)
    {
        if (component == "." || component == "..")
        {
            throw std::runtime_error(
                "Invalid relative LW actuator-model path: "
                + relative_path.string());
        }
    }

    const std::filesystem::path lexical_path =
        (policy_root / normalized_relative).lexically_normal();
    std::error_code resolution_error;
    const std::filesystem::path resolved_path =
        std::filesystem::canonical(lexical_path, resolution_error);

    std::filesystem::path current = policy_root;
    std::filesystem::file_status final_status;
    for (const auto& component : normalized_relative)
    {
        current /= component;
        std::error_code status_error;
        final_status = std::filesystem::symlink_status(
            current,
            status_error);
        if (status_error)
        {
            RejectModelPath(
                "Required LW actuator model path cannot be inspected: "
                    + current.string() + ": " + status_error.message(),
                lexical_path,
                resolved_path,
                resolution_error);
        }
        if (std::filesystem::is_symlink(final_status))
        {
            RejectModelPath(
                "LW actuator model path contains a symbolic link: "
                    + current.string(),
                lexical_path,
                resolved_path,
                resolution_error);
        }
    }

    if (resolution_error
        || !std::filesystem::is_regular_file(final_status))
    {
        RejectModelPath(
            "Required LW actuator model is not a readable regular file",
            lexical_path,
            resolved_path,
            resolution_error);
    }
    if (!IsWithin(policy_root, resolved_path))
    {
        RejectModelPath(
            "LW actuator model resolves outside the selected policy root: "
                + policy_root.string(),
            lexical_path,
            resolved_path,
            resolution_error);
    }
    return resolved_path;
}
}

LWActuatorModelPaths ResolveLWActuatorModelPaths(
    const std::filesystem::path& policy_root,
    const std::string& robot_name)
{
    const std::filesystem::path robot_component(robot_name);
    if (robot_name.empty()
        || robot_component.is_absolute()
        || robot_component.has_parent_path()
        || robot_component == "."
        || robot_component == ".."
        || robot_component.filename() != robot_component)
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
    static_cast<void>(EvaluateLWActuatorModelOutput(
        model,
        std::vector<float>(kActuatorModelInputSize, 0.0f),
        model_path));
}

float EvaluateLWActuatorModelOutput(
    InferenceRuntime::Model& model,
    const std::vector<float>& input,
    const std::filesystem::path& model_path)
{
    if (input.size() != kActuatorModelInputSize)
    {
        throw std::runtime_error(
            "LW actuator model requires exactly 6 input values '"
            + model_path.string() + "': got "
            + std::to_string(input.size()));
    }
    if (!std::all_of(input.begin(), input.end(), [](float value)
        {
            return std::isfinite(value);
        }))
    {
        throw std::runtime_error(
            "LW actuator model received a non-finite input: "
            + model_path.string());
    }

    std::vector<float> output;
    try
    {
        output = model.forward({input});
    }
    catch (const std::exception& exception)
    {
        throw std::runtime_error(
            "LW actuator model rejected the expected 6-value input '"
            + model_path.string() + "': " + exception.what());
    }
    catch (...)
    {
        throw std::runtime_error(
            "LW actuator model rejected the expected 6-value input '"
            + model_path.string() + "' with an unknown exception");
    }
    if (output.size() != kActuatorModelOutputSize)
    {
        throw std::runtime_error(
            "LW actuator model must produce exactly one output '"
            + model_path.string() + "': got "
            + std::to_string(output.size()));
    }
    if (!std::isfinite(output.front()))
    {
        throw std::runtime_error(
            "LW actuator model produced a non-finite output: "
            + model_path.string());
    }
    return output.front();
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

void LWActuatorTorqueFrame::resize(std::size_t size)
{
    candidate_.assign(size, 0.0f);
    committed_.assign(size, 0.0f);
    generation_ = 0;
}

std::vector<float>& LWActuatorTorqueFrame::beginUpdate() noexcept
{
    generation_ = 0;
    std::fill(candidate_.begin(), candidate_.end(), 0.0f);
    return candidate_;
}

void LWActuatorTorqueFrame::commit(std::uint64_t generation)
{
    if (generation == 0 || candidate_.size() != committed_.size())
    {
        throw std::runtime_error(
            "Invalid LW actuator torque-frame commit");
    }
    committed_ = candidate_;
    generation_ = generation;
}

const std::vector<float>& LWActuatorTorqueFrame::values() const noexcept
{
    return committed_;
}

std::uint64_t LWActuatorTorqueFrame::generation() const noexcept
{
    return generation_;
}

const char* LWActuatorTorqueValidation::failureName() const noexcept
{
    switch (failure)
    {
    case LWActuatorTorqueFailure::None: return "none";
    case LWActuatorTorqueFailure::SizeMismatch: return "size-mismatch";
    case LWActuatorTorqueFailure::NonFiniteCandidate:
        return "non-finite-candidate";
    case LWActuatorTorqueFailure::InvalidLimit: return "invalid-limit";
    }
    return "unknown";
}

LWActuatorTorqueValidation PrepareLWActuatorTorques(
    const std::vector<float>& candidates,
    const std::vector<float>& limits,
    std::vector<float>& bounded) noexcept
{
    if (candidates.size() != limits.size()
        || candidates.size() != bounded.size())
    {
        return {LWActuatorTorqueFailure::SizeMismatch, 0};
    }

    for (std::size_t index = 0; index < candidates.size(); ++index)
    {
        if (!std::isfinite(candidates[index]))
        {
            return {
                LWActuatorTorqueFailure::NonFiniteCandidate,
                index};
        }
        if (!std::isfinite(limits[index]) || limits[index] < 0.0f)
        {
            return {LWActuatorTorqueFailure::InvalidLimit, index};
        }
    }

    for (std::size_t index = 0; index < candidates.size(); ++index)
    {
        bounded[index] = std::clamp(
            candidates[index], -limits[index], limits[index]);
    }
    return {};
}
