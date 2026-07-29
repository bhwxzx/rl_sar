#ifndef LW_CONTROL_SAFETY_HPP
#define LW_CONTROL_SAFETY_HPP

#include "rl_sdk.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

enum class LWValidationCode
{
    Valid,
    SizeMismatch,
    NonFinite,
    NegativeGain
};

struct LWValidationResult
{
    LWValidationCode code = LWValidationCode::Valid;
    std::string field;
    size_t index = 0;
    size_t expected_size = 0;
    size_t actual_size = 0;
    float value = 0.0f;

    bool valid() const noexcept
    {
        return code == LWValidationCode::Valid;
    }

    std::string failureDescription() const
    {
        std::ostringstream stream;
        if (code == LWValidationCode::SizeMismatch)
        {
            stream << field << " size=" << actual_size
                   << ", expected=" << expected_size;
        }
        else if (code == LWValidationCode::NonFinite)
        {
            stream << field << "[" << index << "] is not finite: " << value;
        }
        else if (code == LWValidationCode::NegativeGain)
        {
            stream << field << "[" << index << "] is negative: " << value;
        }
        return stream.str();
    }
};

inline LWValidationResult LWValidateFiniteVector(
    const std::string& field,
    const std::vector<float>& values,
    size_t expected_size)
{
    if (values.size() != expected_size)
    {
        LWValidationResult result;
        result.code = LWValidationCode::SizeMismatch;
        result.field = field;
        result.expected_size = expected_size;
        result.actual_size = values.size();
        return result;
    }

    for (size_t index = 0; index < values.size(); ++index)
    {
        if (!std::isfinite(values[index]))
        {
            LWValidationResult result;
            result.code = LWValidationCode::NonFinite;
            result.field = field;
            result.index = index;
            result.value = values[index];
            return result;
        }
    }
    return {};
}

inline LWValidationResult LWValidateFeedbackState(
    const RobotState<float>& state,
    size_t num_dofs)
{
    const std::vector<std::pair<std::string, const std::vector<float>*>> fields = {
        {"imu.quaternion", &state.imu.quaternion},
        {"imu.gyroscope", &state.imu.gyroscope},
        {"motor.q", &state.motor_state.q},
        {"motor.dq", &state.motor_state.dq},
        {"motor.tau_est", &state.motor_state.tau_est},
    };
    const std::vector<size_t> expected_sizes = {
        4,
        3,
        num_dofs,
        num_dofs,
        num_dofs,
    };

    for (size_t index = 0; index < fields.size(); ++index)
    {
        const LWValidationResult result =
            LWValidateFiniteVector(
                fields[index].first,
                *fields[index].second,
                expected_sizes[index]);
        if (!result.valid())
        {
            return result;
        }
    }
    return {};
}

inline LWValidationResult LWValidatePolicyActions(
    const std::vector<float>& actions,
    size_t num_dofs)
{
    return LWValidateFiniteVector("policy.action", actions, num_dofs);
}

inline LWValidationResult LWValidatePolicyOutputs(
    const std::vector<float>& positions,
    const std::vector<float>& velocities,
    const std::vector<float>& torques,
    size_t num_dofs)
{
    const LWValidationResult position_result =
        LWValidateFiniteVector("policy.output_q", positions, num_dofs);
    if (!position_result.valid())
    {
        return position_result;
    }

    const LWValidationResult velocity_result =
        LWValidateFiniteVector("policy.output_dq", velocities, num_dofs);
    if (!velocity_result.valid())
    {
        return velocity_result;
    }

    return LWValidateFiniteVector("policy.output_tau", torques, num_dofs);
}

inline LWValidationResult LWValidateRobotCommand(
    const RobotCommand<float>& command,
    size_t num_dofs)
{
    const std::vector<std::pair<std::string, const std::vector<float>*>> fields = {
        {"command.q", &command.motor_command.q},
        {"command.dq", &command.motor_command.dq},
        {"command.tau", &command.motor_command.tau},
        {"command.kp", &command.motor_command.kp},
        {"command.kd", &command.motor_command.kd},
    };

    for (const auto& field : fields)
    {
        const LWValidationResult result =
            LWValidateFiniteVector(field.first, *field.second, num_dofs);
        if (!result.valid())
        {
            return result;
        }
    }

    for (size_t index = 0; index < num_dofs; ++index)
    {
        if (command.motor_command.kp[index] < 0.0f)
        {
            LWValidationResult result;
            result.code = LWValidationCode::NegativeGain;
            result.field = "command.kp";
            result.index = index;
            result.value = command.motor_command.kp[index];
            return result;
        }
        if (command.motor_command.kd[index] < 0.0f)
        {
            LWValidationResult result;
            result.code = LWValidationCode::NegativeGain;
            result.field = "command.kd";
            result.index = index;
            result.value = command.motor_command.kd[index];
            return result;
        }
    }
    return {};
}

inline bool LWStateUsesAttitudeProtection(const std::string& state_name)
{
    return state_name == "RLFSMStateGetDown"
        || state_name == "RLFSMStateRLLocomotion_Leg"
        || state_name == "RLFSMStateRLLocomotion_Wheel"
        || state_name == "RLFSMStateRL_LegToWheel"
        || state_name == "RLFSMStateRL_WheelToLeg";
}

struct LWAttitudeValidationResult
{
    bool protection_enabled = false;
    bool safe = true;
    float roll_degrees = 0.0f;
    float pitch_degrees = 0.0f;

    std::string failureDescription(float threshold_degrees) const
    {
        std::ostringstream stream;
        stream << "roll=" << roll_degrees
               << " deg, pitch=" << pitch_degrees
               << " deg, threshold=" << threshold_degrees << " deg";
        return stream.str();
    }
};

inline LWAttitudeValidationResult LWValidateAttitude(
    const std::string& state_name,
    const std::vector<float>& quaternion,
    float threshold_degrees)
{
    LWAttitudeValidationResult result;
    result.protection_enabled = LWStateUsesAttitudeProtection(state_name);
    if (!result.protection_enabled)
    {
        return result;
    }

    if (quaternion.size() != 4 || !std::isfinite(threshold_degrees)
        || threshold_degrees <= 0.0f)
    {
        result.safe = false;
        result.roll_degrees = std::numeric_limits<float>::quiet_NaN();
        result.pitch_degrees = std::numeric_limits<float>::quiet_NaN();
        return result;
    }

    const float w = quaternion[0];
    const float x = quaternion[1];
    const float y = quaternion[2];
    const float z = quaternion[3];
    const float sin_roll = 2.0f * (w * x + y * z);
    const float cos_roll = 1.0f - 2.0f * (x * x + y * y);
    const float sin_pitch =
        std::clamp(2.0f * (w * y - z * x), -1.0f, 1.0f);
    constexpr float radians_to_degrees = 57.29577951308232f;
    result.roll_degrees = std::atan2(sin_roll, cos_roll) * radians_to_degrees;
    result.pitch_degrees = std::asin(sin_pitch) * radians_to_degrees;
    result.safe =
        std::isfinite(result.roll_degrees)
        && std::isfinite(result.pitch_degrees)
        && std::fabs(result.roll_degrees) <= threshold_degrees
        && std::fabs(result.pitch_degrees) <= threshold_degrees;
    return result;
}

#endif // LW_CONTROL_SAFETY_HPP
