#ifndef LW_CONFIGURATION_VALIDATION_HPP
#define LW_CONFIGURATION_VALIDATION_HPP

#include "inference_runtime.hpp"

#include <cstddef>
#include <string>

#include <yaml-cpp/yaml.h>

struct LWPolicyDimensions
{
    std::size_t num_dofs = 0;
    std::size_t observation = 0;
    std::size_t model_input = 0;
    std::size_t model_output = 0;
};

struct LWValidatedPolicyConfiguration
{
    YAML::Node merged;
    LWPolicyDimensions dimensions;
};

void ValidateLWBaseConfiguration(
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
