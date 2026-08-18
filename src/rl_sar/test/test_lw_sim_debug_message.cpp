#include "lw_sim_debug_message.hpp"

#include <atomic>
#include <cstdlib>
#include <iostream>
#include <new>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
std::atomic<bool> count_allocations{false};
std::atomic<std::size_t> allocation_count{0};
}

void* operator new(std::size_t size)
{
    if (count_allocations.load(std::memory_order_relaxed))
    {
        allocation_count.fetch_add(1, std::memory_order_relaxed);
    }
    if (void* memory = std::malloc(size))
    {
        return memory;
    }
    throw std::bad_alloc();
}

void* operator new[](std::size_t size)
{
    return ::operator new(size);
}

void operator delete(void* memory) noexcept
{
    std::free(memory);
}

void operator delete[](void* memory) noexcept
{
    std::free(memory);
}

void operator delete(void* memory, std::size_t) noexcept
{
    std::free(memory);
}

void operator delete[](void* memory, std::size_t) noexcept
{
    std::free(memory);
}

namespace
{
void require(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

LWSimDebugMessageConfig validConfig()
{
    return {
        LW_SIM_DEBUG_NUM_DOFS,
        {0, 1, 2, 3, 4, 5, 6, 7, 8, 9},
        {8, 9}};
}

RobotState<float> makeState()
{
    RobotState<float> state;
    state.motor_state.resize(LW_SIM_DEBUG_NUM_DOFS);
    for (std::size_t index = 0; index < LW_SIM_DEBUG_NUM_DOFS; ++index)
    {
        state.motor_state.q[index] = static_cast<float>(index) + 0.1f;
        state.motor_state.dq[index] = static_cast<float>(index) + 0.2f;
        state.motor_state.tau_est[index] = static_cast<float>(index) + 0.3f;
    }
    return state;
}

RobotCommand<float> makeCommand()
{
    RobotCommand<float> command;
    command.motor_command.resize(LW_SIM_DEBUG_NUM_DOFS);
    for (std::size_t index = 0; index < LW_SIM_DEBUG_NUM_DOFS; ++index)
    {
        command.motor_command.q[index] = 100.0f + static_cast<float>(index);
        command.motor_command.dq[index] = 200.0f + static_cast<float>(index);
    }
    return command;
}

LWSimDebugTelemetry fullTelemetry()
{
    LWSimDebugTelemetry telemetry;
    telemetry.gait_available = true;
    telemetry.left_foot_position = {1.0, 2.0, 3.0};
    telemetry.left_contact = 1.0;
    telemetry.right_foot_position = {4.0, 5.0, 6.0};
    telemetry.right_contact = 0.0;
    telemetry.frame_position_available = true;
    telemetry.frame_position = {7.0, 8.0, 9.0};
    telemetry.frame_velocity_available = true;
    telemetry.frame_velocity = {10.0, 11.0, 12.0};
    telemetry.angular_velocity_available = true;
    telemetry.angular_velocity = {13.0, 14.0, 15.0};
    telemetry.quaternion_available = true;
    telemetry.quaternion = {16.0, 17.0, 18.0, 19.0};
    return telemetry;
}

void testFixedLayoutAndValues()
{
    const std::vector<std::string> expected_names = {
        "right_hip_now", "left_hip_now", "right_thigh_now", "left_thigh_now",
        "right_shank_now", "left_shank_now", "right_foot_now", "left_foot_now",
        "right_wheel_now", "left_wheel_now",
        "right_hip_target", "left_hip_target", "right_thigh_target", "left_thigh_target",
        "right_shank_target", "left_shank_target", "right_foot_target", "left_foot_target",
        "right_wheel_target", "left_wheel_target",
        "l_foot_x", "l_foot_y", "l_foot_z", "l_contact",
        "r_foot_x", "r_foot_y", "r_foot_z", "r_contact",
        "cmd_vel_x", "cmd_vel_yaw",
        "base_p_x", "base_p_y", "base_p_z",
        "base_v_x", "base_v_y", "base_v_z",
        "base_w_x", "base_w_y", "base_w_z",
        "base_q_w", "base_q_x", "base_q_y", "base_q_z"};
    LWSimDebugMessageCache cache(validConfig());
    const RobotState<float> state = makeState();
    const RobotCommand<float> command = makeCommand();
    LWControlSnapshot control;
    control.x = 1.25f;
    control.yaw = -0.75f;

    require(
        cache.Populate(state, command, control, fullTelemetry()),
        "valid debug sample was rejected");
    const auto& message = cache.message();
    require(message.name == expected_names, "debug field names changed");
    require(
        message.position.size() == LW_SIM_DEBUG_FIELD_COUNT
            && message.velocity.size() == LW_SIM_DEBUG_FIELD_COUNT
            && message.effort.size() == LW_SIM_DEBUG_FIELD_COUNT,
        "debug numeric arrays do not have 43 fields");

    for (std::size_t index = 0; index < LW_SIM_DEBUG_NUM_DOFS; ++index)
    {
        require(message.position[index] == state.motor_state.q[index],
            "current position changed");
        require(message.velocity[index] == state.motor_state.dq[index],
            "current velocity changed");
        require(message.effort[index] == state.motor_state.tau_est[index],
            "current effort changed");
        const bool wheel = index == 8 || index == 9;
        require(
            message.position[10 + index]
                == (wheel ? 0.0 : command.motor_command.q[index]),
            "target position changed");
        require(
            message.velocity[10 + index]
                == (wheel ? command.motor_command.dq[index] : 0.0),
            "target velocity changed");
    }

    const std::vector<double> expected_gait =
        {1.0, 2.0, 3.0, 1.0, 4.0, 5.0, 6.0, 0.0};
    for (std::size_t index = 0; index < expected_gait.size(); ++index)
    {
        require(message.position[20 + index] == expected_gait[index],
            "gait field changed");
    }
    const std::vector<double> expected_tracking = {
        1.25, -0.75,
        7.0, 8.0, 9.0,
        10.0, 11.0, 12.0,
        13.0, 14.0, 15.0,
        16.0, 17.0, 18.0, 19.0};
    for (std::size_t index = 0; index < expected_tracking.size(); ++index)
    {
        require(message.position[28 + index] == expected_tracking[index],
            "tracking field changed");
    }
}

void testMissingTelemetryClearsStaleValues()
{
    LWSimDebugMessageCache cache(validConfig());
    const RobotState<float> state = makeState();
    const RobotCommand<float> command = makeCommand();
    LWControlSnapshot control;
    control.x = 2.0f;
    control.yaw = 3.0f;
    require(cache.Populate(state, command, control, fullTelemetry()),
        "warm telemetry sample was rejected");
    require(cache.Populate(state, command, control, LWSimDebugTelemetry{}),
        "missing telemetry sample was rejected");

    const auto& position = cache.message().position;
    for (std::size_t index = 20; index < 28; ++index)
    {
        require(position[index] == 0.0, "missing gait data remained stale");
    }
    require(position[28] == 2.0 && position[29] == 3.0,
        "commands were not retained without optional telemetry");
    for (std::size_t index = 30; index < position.size(); ++index)
    {
        require(position[index] == 0.0,
            "missing tracking data remained stale");
    }
}

void expectInvalid(LWSimDebugMessageConfig config)
{
    bool threw = false;
    try
    {
        LWSimDebugMessageCache cache(config);
    }
    catch (const std::invalid_argument&)
    {
        threw = true;
    }
    require(threw, "invalid debug configuration was accepted");
}

void testInvalidConfigurations()
{
    auto config = validConfig();
    config.num_dofs = 9;
    expectInvalid(config);

    config = validConfig();
    config.joint_mapping.pop_back();
    expectInvalid(config);

    config = validConfig();
    config.joint_mapping[9] = 10;
    expectInvalid(config);

    config = validConfig();
    config.joint_mapping[9] = 8;
    expectInvalid(config);

    config = validConfig();
    config.wheel_indices.pop_back();
    expectInvalid(config);

    config = validConfig();
    config.wheel_indices = {8, 8};
    expectInvalid(config);

    config = validConfig();
    config.wheel_indices = {8, 10};
    expectInvalid(config);
}

void testPopulateDoesNotAllocateAfterWarmup()
{
    LWSimDebugMessageCache cache(validConfig());
    const RobotState<float> state = makeState();
    const RobotCommand<float> command = makeCommand();
    const LWControlSnapshot control{};
    const LWSimDebugTelemetry telemetry = fullTelemetry();
    require(cache.Populate(state, command, control, telemetry),
        "debug cache warmup failed");

    bool populated = true;
    allocation_count.store(0, std::memory_order_relaxed);
    count_allocations.store(true, std::memory_order_release);
    for (int iteration = 0; iteration < 10000; ++iteration)
    {
        populated = cache.Populate(state, command, control, telemetry)
            && populated;
    }
    count_allocations.store(false, std::memory_order_release);

    require(populated, "debug cache population failed");
    require(allocation_count.load(std::memory_order_relaxed) == 0,
        "debug cache allocated after warmup");
}
} // namespace

int main()
{
    try
    {
        testFixedLayoutAndValues();
        testMissingTelemetryClearsStaleValues();
        testInvalidConfigurations();
        testPopulateDoesNotAllocateAfterWarmup();
    }
    catch (const std::exception& exception)
    {
        count_allocations.store(false, std::memory_order_release);
        std::cerr << "test_lw_sim_debug_message failed: "
                  << exception.what() << std::endl;
        return 1;
    }
    std::cout << "test_lw_sim_debug_message passed" << std::endl;
    return 0;
}
