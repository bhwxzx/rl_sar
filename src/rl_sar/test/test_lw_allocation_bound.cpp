#include "rl_sdk.hpp"

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
constexpr std::size_t kNumDofs = 10;

void require(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

class AllocationTestRL : public RL
{
public:
    std::vector<float> Forward() override
    {
        return {};
    }

    void GetState(RobotState<float>*) override
    {
    }

    void SetCommand(const RobotCommand<float>*) override
    {
    }
};

class AllocationTestState : public RLFSMState
{
public:
    explicit AllocationTestState(RL& rl)
        : RLFSMState(rl, "AllocationTestState")
    {
    }

    void Enter() override
    {
    }

    void Run() override
    {
    }

    void Exit() override
    {
    }
};

LWBaseRuntimeConfiguration makeBaseConfiguration()
{
    LWBaseRuntimeConfiguration configuration;
    configuration.num_dofs = kNumDofs;
    configuration.dt = 0.005f;
    configuration.decimation = 4;
    configuration.joint_mapping = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    configuration.wheel_indices = {8, 9};
    configuration.wheel_mask.assign(kNumDofs, 0);
    configuration.wheel_mask[8] = 1;
    configuration.wheel_mask[9] = 1;
    configuration.rl_kp.assign(kNumDofs, 20.0f);
    configuration.rl_kd.assign(kNumDofs, 0.5f);
    configuration.fixed_kp.assign(kNumDofs, 40.0f);
    configuration.fixed_kd.assign(kNumDofs, 1.0f);
    return configuration;
}

void testSteadyStateConfigurationAndStateTransportDoNotAllocate()
{
    AllocationTestRL rl;
    rl.SetLWBaseRuntimeConfiguration(makeBaseConfiguration());
    rl.InitJointNum(kNumDofs);

    AllocationTestState state(rl);
    state.fsm_state = &rl.robot_state;
    state.fsm_command = &rl.robot_command;
    std::vector<float> start(kNumDofs, 0.0f);
    std::vector<float> target(kNumDofs, 0.5f);
    float percent = 0.01f;

    LWPolicyInputSnapshot published;
    published.generation = 1;
    published.sequence = 1;
    published.state_capture_time = std::chrono::steady_clock::now();
    published.robot_state.motor_state.resize(kNumDofs);
    LWPolicyInputSnapshot received;
    received.robot_state.motor_state.resize(kNumDofs);
    LWSnapshotBuffer<LWPolicyInputSnapshot> buffer;
    buffer.publish(published);
    require(buffer.read(received), "failed to warm the policy input buffer");

    bool operations_succeeded = true;
    allocation_count.store(0, std::memory_order_relaxed);
    count_allocations.store(true, std::memory_order_release);
    for (std::uint64_t iteration = 0; iteration < 10000; ++iteration)
    {
        published.sequence = iteration + 2;
        published.robot_state.motor_state.q[0] =
            static_cast<float>(iteration);
        operations_succeeded =
            buffer.tryPublish(published)
            && buffer.read(received)
            && received.sequence == published.sequence
            && state.Interpolate(
                percent,
                start,
                target,
                1000.0f,
                "",
                true,
                LWOperatorMode::Unknown)
            && operations_succeeded;
    }
    count_allocations.store(false, std::memory_order_release);

    require(operations_succeeded, "steady-state operation failed");
    require(
        allocation_count.load(std::memory_order_relaxed) == 0,
        "configuration or policy-input transport allocated after warmup");
}
} // namespace

int main()
{
    try
    {
        testSteadyStateConfigurationAndStateTransportDoNotAllocate();
    }
    catch (const std::exception& exception)
    {
        count_allocations.store(false, std::memory_order_release);
        std::cerr << "test_lw_allocation_bound failed: "
                  << exception.what() << std::endl;
        return 1;
    }
    std::cout << "test_lw_allocation_bound passed" << std::endl;
    return 0;
}
