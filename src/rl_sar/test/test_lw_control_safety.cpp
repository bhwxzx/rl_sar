#include "lw_control_safety.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
constexpr size_t kNumDofs = 10;

void require(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

RobotState<float> makeValidState()
{
    RobotState<float> state;
    state.motor_state.resize(kNumDofs);
    return state;
}

RobotCommand<float> makeValidCommand()
{
    RobotCommand<float> command;
    command.motor_command.resize(kNumDofs);
    return command;
}

std::vector<float> rollQuaternion(float degrees)
{
    constexpr float degrees_to_radians = 0.017453292519943295f;
    const float half_angle = 0.5f * degrees * degrees_to_radians;
    return {std::cos(half_angle), std::sin(half_angle), 0.0f, 0.0f};
}

std::vector<float> pitchQuaternion(float degrees)
{
    constexpr float degrees_to_radians = 0.017453292519943295f;
    const float half_angle = 0.5f * degrees * degrees_to_radians;
    return {std::cos(half_angle), 0.0f, std::sin(half_angle), 0.0f};
}

void testFeedbackFiniteValidation()
{
    RobotState<float> state = makeValidState();
    require(
        LWValidateFeedbackState(state, kNumDofs).valid(),
        "valid feedback was rejected");

    state.imu.quaternion[2] = std::numeric_limits<float>::quiet_NaN();
    auto result = LWValidateFeedbackState(state, kNumDofs);
    require(result.code == LWValidationCode::NonFinite, "IMU NaN was accepted");
    require(result.field == "imu.quaternion" && result.index == 2, "IMU NaN location was lost");

    state = makeValidState();
    state.imu.gyroscope[1] = std::numeric_limits<float>::infinity();
    result = LWValidateFeedbackState(state, kNumDofs);
    require(result.code == LWValidationCode::NonFinite, "gyro infinity was accepted");

    state = makeValidState();
    state.motor_state.q[3] = -std::numeric_limits<float>::infinity();
    result = LWValidateFeedbackState(state, kNumDofs);
    require(result.code == LWValidationCode::NonFinite, "position infinity was accepted");

    state = makeValidState();
    state.motor_state.dq[4] = std::numeric_limits<float>::quiet_NaN();
    result = LWValidateFeedbackState(state, kNumDofs);
    require(result.code == LWValidationCode::NonFinite, "velocity NaN was accepted");

    state = makeValidState();
    state.motor_state.tau_est[5] = std::numeric_limits<float>::infinity();
    result = LWValidateFeedbackState(state, kNumDofs);
    require(result.code == LWValidationCode::NonFinite, "torque infinity was accepted");

    state = makeValidState();
    state.motor_state.q.pop_back();
    result = LWValidateFeedbackState(state, kNumDofs);
    require(result.code == LWValidationCode::SizeMismatch, "feedback size mismatch was accepted");
}

void testPolicyFiniteValidationWithoutRangeChecks()
{
    std::vector<float> actions(kNumDofs, 0.0f);
    actions[0] = 1.0e20f;
    require(
        LWValidatePolicyActions(actions, kNumDofs).valid(),
        "large finite action was treated as a range violation");

    actions[1] = std::numeric_limits<float>::quiet_NaN();
    require(
        !LWValidatePolicyActions(actions, kNumDofs).valid(),
        "policy action NaN was accepted");

    std::vector<float> positions(kNumDofs, 1.0e20f);
    std::vector<float> velocities(kNumDofs, -1.0e20f);
    std::vector<float> torques(kNumDofs, 1.0e20f);
    require(
        LWValidatePolicyOutputs(positions, velocities, torques, kNumDofs).valid(),
        "large finite policy output was treated as a range violation");

    torques[7] = std::numeric_limits<float>::infinity();
    const auto result =
        LWValidatePolicyOutputs(positions, velocities, torques, kNumDofs);
    require(result.code == LWValidationCode::NonFinite, "policy torque infinity was accepted");
    require(result.field == "policy.output_tau" && result.index == 7, "policy output location was lost");
}

void testCommandFiniteAndNegativeGainValidation()
{
    RobotCommand<float> command = makeValidCommand();
    command.motor_command.q[0] = 1.0e20f;
    command.motor_command.dq[1] = -1.0e20f;
    command.motor_command.tau[2] = 1.0e20f;
    command.motor_command.kp[3] = 1.0e20f;
    command.motor_command.kd[4] = 1.0e20f;
    require(
        LWValidateRobotCommand(command, kNumDofs).valid(),
        "large finite command or gain was treated as a range violation");

    command.motor_command.q[5] = std::numeric_limits<float>::quiet_NaN();
    require(
        !LWValidateRobotCommand(command, kNumDofs).valid(),
        "command position NaN was accepted");

    command = makeValidCommand();
    command.motor_command.kp[6] = -0.01f;
    auto result = LWValidateRobotCommand(command, kNumDofs);
    require(result.code == LWValidationCode::NegativeGain, "negative Kp was accepted");
    require(result.field == "command.kp" && result.index == 6, "negative Kp location was lost");

    command = makeValidCommand();
    command.motor_command.kd[7] = -0.01f;
    result = LWValidateRobotCommand(command, kNumDofs);
    require(result.code == LWValidationCode::NegativeGain, "negative Kd was accepted");

    command = makeValidCommand();
    command.motor_command.kd.pop_back();
    result = LWValidateRobotCommand(command, kNumDofs);
    require(result.code == LWValidationCode::SizeMismatch, "command size mismatch was accepted");
}

void testStateSpecificAttitudeProtection()
{
    constexpr float threshold = 75.0f;
    const std::vector<std::string> unprotected_states = {
        "RLFSMStatePassive",
        "RLFSMStateGetUp_Leg",
        "RLFSMStateGetUp_Wheel",
    };
    for (const auto& state : unprotected_states)
    {
        const auto result =
            LWValidateAttitude(state, rollQuaternion(90.0f), threshold);
        require(!result.protection_enabled, state + " unexpectedly enabled attitude protection");
        require(result.safe, state + " rejected attitude while protection was disabled");
    }

    const std::vector<std::string> protected_states = {
        "RLFSMStateGetDown",
        "RLFSMStateRLLocomotion_Leg",
        "RLFSMStateRLLocomotion_Wheel",
        "RLFSMStateRL_LegToWheel",
        "RLFSMStateRL_WheelToLeg",
    };
    for (const auto& state : protected_states)
    {
        auto result =
            LWValidateAttitude(state, rollQuaternion(74.0f), threshold);
        require(result.protection_enabled, state + " did not enable attitude protection");
        require(result.safe, state + " rejected an in-range roll");

        result = LWValidateAttitude(state, rollQuaternion(76.0f), threshold);
        require(!result.safe, state + " accepted an excessive roll");

        result = LWValidateAttitude(state, pitchQuaternion(-76.0f), threshold);
        require(!result.safe, state + " accepted an excessive pitch");
    }
}
} // namespace

int main()
{
    try
    {
        testFeedbackFiniteValidation();
        testPolicyFiniteValidationWithoutRangeChecks();
        testCommandFiniteAndNegativeGainValidation();
        testStateSpecificAttitudeProtection();
        std::cout << "LW control safety tests passed" << std::endl;
        return EXIT_SUCCESS;
    }
    catch (const std::exception& exception)
    {
        std::cerr << "LW control safety tests failed: "
                  << exception.what() << std::endl;
        return EXIT_FAILURE;
    }
}
