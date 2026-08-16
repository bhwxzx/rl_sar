#include "lw_sim_torque_validation.hpp"

#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
void Require(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

void TestValidTorquesAreClamped()
{
    std::vector<float> bounded(3, 99.0f);
    const auto result = PrepareLWSimTorques(
        {2.0f, -3.0f, 0.5f},
        {1.0f, 2.0f, 1.0f},
        bounded);
    Require(result.valid(), "finite final torques were rejected");
    Require(
        bounded == std::vector<float>({1.0f, -2.0f, 0.5f}),
        "finite final torques were not clamped correctly");
}

void TestInvalidInputsNeverPartiallyMutateOutput()
{
    const std::vector<float> unchanged(3, 77.0f);
    std::vector<float> bounded = unchanged;

    auto result = PrepareLWSimTorques(
        {1.0f, std::numeric_limits<float>::quiet_NaN(), 2.0f},
        {3.0f, 3.0f, 3.0f},
        bounded);
    Require(!result.valid(), "non-finite final torque was accepted");
    Require(result.index == 1, "wrong invalid final-torque index reported");
    Require(
        result.failure == LWSimTorqueFailure::NonFiniteCandidate,
        "wrong non-finite failure reported");
    Require(bounded == unchanged, "invalid torque caused a partial mutation");

    result = PrepareLWSimTorques(
        {1.0f, 2.0f, 3.0f},
        {3.0f, -1.0f, 3.0f},
        bounded);
    Require(!result.valid(), "invalid torque limit was accepted");
    Require(
        result.failure == LWSimTorqueFailure::InvalidLimit,
        "wrong invalid-limit failure reported");
    Require(bounded == unchanged, "invalid limit caused a partial mutation");

    result = PrepareLWSimTorques({1.0f}, {1.0f, 2.0f}, bounded);
    Require(!result.valid(), "size mismatch was accepted");
    Require(
        result.failure == LWSimTorqueFailure::SizeMismatch,
        "wrong size-mismatch failure reported");
    Require(bounded == unchanged, "size mismatch caused a mutation");
}
} // namespace

int main()
{
    try
    {
        TestValidTorquesAreClamped();
        TestInvalidInputsNeverPartiallyMutateOutput();
    }
    catch (const std::exception& exception)
    {
        std::cerr << "test_lw_sim_torque_validation failed: "
                  << exception.what() << std::endl;
        return 1;
    }
    std::cout << "test_lw_sim_torque_validation passed" << std::endl;
    return 0;
}
