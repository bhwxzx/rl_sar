#include "rl_sdk.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
using Clock = std::chrono::steady_clock;

void require(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

class TestRL : public RL
{
public:
    std::vector<float> Forward() override
    {
        return {};
    }

    void GetState(RobotState<float>*) override {}
    void SetCommand(const RobotCommand<float>*) override {}
};

class TestControlState : public RLFSMState
{
public:
    explicit TestControlState(RL& rl)
        : RLFSMState(rl, "generic-control-test")
    {
    }

    void Enter() override {}
    void Run() override {}
    void Exit() override {}
    std::string CheckChange() override
    {
        return state_name_;
    }
};

LWPolicyOutputFrame makeFrame(
    std::uint64_t generation,
    std::uint64_t frame,
    size_t dofs,
    Clock::time_point source_time)
{
    const float tag = static_cast<float>(frame);
    return {
        generation,
        0,
        frame,
        source_time,
        std::vector<float>(dofs, tag),
        std::vector<float>(dofs, 2.0f * tag),
        std::vector<float>(dofs, 3.0f * tag)};
}

void testRejectsPartialAndWrongGenerationFrames()
{
    constexpr size_t dofs = 10;
    constexpr std::uint64_t generation = 7;
    LWPolicyOutputTransport transport;
    const auto now = Clock::now();

    require(
        !transport.publish(
            makeFrame(generation - 1, 1, dofs, now),
            generation,
            dofs),
        "transport accepted an inactive generation");

    auto missing_timestamp =
        makeFrame(generation, 1, dofs, {});
    require(
        !transport.publish(
            std::move(missing_timestamp),
            generation,
            dofs),
        "transport accepted a frame without a source timestamp");

    auto missing_position =
        makeFrame(generation, 1, dofs, now);
    missing_position.dof_pos.pop_back();
    require(
        !transport.publish(
            std::move(missing_position),
            generation,
            dofs),
        "transport accepted a partial position vector");

    auto missing_velocity =
        makeFrame(generation, 1, dofs, now);
    missing_velocity.dof_vel.clear();
    require(
        !transport.publish(
            std::move(missing_velocity),
            generation,
            dofs),
        "transport accepted a partial velocity vector");

    auto missing_torque =
        makeFrame(generation, 1, dofs, now);
    missing_torque.dof_tau.resize(dofs - 1);
    require(
        !transport.publish(
            std::move(missing_torque),
            generation,
            dofs),
        "transport accepted a partial torque vector");
    require(
        !transport.load(),
        "rejected frames changed the latest-frame slot");
}

void testLatestFrameAndGenerationSwitchSemantics()
{
    constexpr size_t dofs = 10;
    LWPolicyOutputTransport transport;
    const auto now = Clock::now();

    require(
        transport.publish(
            makeFrame(1, 10, dofs, now),
            1,
            dofs),
        "first complete frame was rejected");
    const auto first = transport.load();
    require(first && first->sequence == 1,
            "first frame did not receive sequence one");

    require(
        transport.publish(
            makeFrame(1, 11, dofs, now),
            1,
            dofs),
        "second complete frame was rejected");
    const auto latest = transport.load();
    require(
        latest
            && latest->sequence == 2
            && latest->frame == 11
            && latest->dof_pos[0] == 11.0f
            && latest->dof_vel[0] == 22.0f
            && latest->dof_tau[0] == 33.0f,
        "latest-frame slot retained or mixed an older frame");

    transport.clear();
    require(
        !transport.load(),
        "policy deactivation retained an output frame");
    require(
        !transport.publish(
            makeFrame(1, 12, dofs, now),
            2,
            dofs),
        "old generation republished after a switch");
    require(
        transport.publish(
            makeFrame(2, 1, dofs, now),
            2,
            dofs),
        "new generation frame was rejected");
    require(
        transport.load()->generation == 2,
        "new generation did not replace the cleared slot");
}

void testFrameFreshnessClassification()
{
    constexpr size_t dofs = 10;
    constexpr std::uint64_t generation = 3;
    const auto source_time = Clock::now();
    const auto max_age = std::chrono::milliseconds(60);
    auto output =
        makeFrame(generation, 1, dofs, source_time);
    output.sequence = 1;

    require(
        EvaluateLWPolicyOutput(
            nullptr,
            generation,
            dofs,
            source_time,
            max_age)
            == LWPolicyOutputStatus::Missing,
        "missing frame was not classified");
    require(
        EvaluateLWPolicyOutput(
            &output,
            generation + 1,
            dofs,
            source_time,
            max_age)
            == LWPolicyOutputStatus::GenerationMismatch,
        "generation mismatch was not classified");

    auto incomplete = output;
    incomplete.dof_tau.pop_back();
    require(
        EvaluateLWPolicyOutput(
            &incomplete,
            generation,
            dofs,
            source_time,
            max_age)
            == LWPolicyOutputStatus::Incomplete,
        "partial frame was not classified");
    require(
        EvaluateLWPolicyOutput(
            &output,
            generation,
            dofs,
            source_time + max_age,
            max_age)
            == LWPolicyOutputStatus::Ready,
        "frame at the age boundary was rejected");
    require(
        EvaluateLWPolicyOutput(
            &output,
            generation,
            dofs,
            source_time + max_age
                + std::chrono::nanoseconds(1),
            max_age)
            == LWPolicyOutputStatus::Stale,
        "frame beyond the age boundary was accepted");
    require(
        EvaluateLWPolicyOutput(
            &output,
            generation,
            dofs,
            source_time - std::chrono::nanoseconds(1),
            max_age)
            == LWPolicyOutputStatus::Stale,
        "future-dated frame was accepted");
}

void testConcurrentReadersNeverObservePartialFrames()
{
    constexpr size_t dofs = 10;
    constexpr std::uint64_t generation = 9;
    constexpr std::uint64_t iterations = 50000;
    LWPolicyOutputTransport transport;
    std::atomic<bool> start{false};
    std::atomic<bool> failed{false};

    std::thread writer([&]()
    {
        while (!start.load(std::memory_order_acquire))
        {
        }
        for (std::uint64_t frame = 1;
             frame <= iterations;
             ++frame)
        {
            if (!transport.publish(
                    makeFrame(
                        generation,
                        frame,
                        dofs,
                        Clock::now()),
                    generation,
                    dofs))
            {
                failed.store(true, std::memory_order_release);
                return;
            }
        }
    });

    std::vector<std::thread> readers;
    for (int reader = 0; reader < 4; ++reader)
    {
        readers.emplace_back([&]()
        {
            while (!start.load(std::memory_order_acquire))
            {
            }
            for (std::uint64_t iteration = 0;
                 iteration < iterations;
                 ++iteration)
            {
                const auto output = transport.load();
                if (!output)
                {
                    continue;
                }
                const float tag =
                    static_cast<float>(output->frame);
                if (output->generation != generation
                    || output->sequence == 0
                    || output->dof_pos.size() != dofs
                    || output->dof_vel.size() != dofs
                    || output->dof_tau.size() != dofs)
                {
                    failed.store(
                        true,
                        std::memory_order_release);
                    return;
                }
                for (size_t i = 0; i < dofs; ++i)
                {
                    if (output->dof_pos[i] != tag
                        || output->dof_vel[i]
                            != 2.0f * tag
                        || output->dof_tau[i]
                            != 3.0f * tag)
                    {
                        failed.store(
                            true,
                            std::memory_order_release);
                        return;
                    }
                }
            }
        });
    }

    start.store(true, std::memory_order_release);
    writer.join();
    for (auto& reader : readers)
    {
        reader.join();
    }

    const auto latest = transport.load();
    require(
        !failed.load(std::memory_order_acquire),
        "concurrent reader observed a partial or mixed frame");
    require(
        latest && latest->frame == iterations,
        "latest-frame transport accumulated an older backlog");
}

void testGenericControlRetainsNonLWQueueCompatibility()
{
    TestRL rl;
    rl.params.config_node["num_of_dofs"] = 2;
    rl.params.config_node["rl_kp"] =
        std::vector<float>{10.0f, 20.0f};
    rl.params.config_node["rl_kd"] =
        std::vector<float>{1.0f, 2.0f};

    RobotCommand<float> command;
    command.motor_command.resize(2);
    TestControlState state(rl);
    state.fsm_command = &command;

    rl.output_dof_pos_queue.push({0.25f, -0.5f});
    rl.output_dof_vel_queue.push({1.5f, -2.0f});
    state.RLControl();

    require(
        command.motor_command.q
            == std::vector<float>({0.25f, -0.5f})
            && command.motor_command.dq
                == std::vector<float>({1.5f, -2.0f})
            && command.motor_command.kp
                == std::vector<float>({10.0f, 20.0f})
            && command.motor_command.kd
                == std::vector<float>({1.0f, 2.0f}),
        "LW transport changes broke the generic queue consumer");
}
} // namespace

int main()
{
    try
    {
        testRejectsPartialAndWrongGenerationFrames();
        testLatestFrameAndGenerationSwitchSemantics();
        testFrameFreshnessClassification();
        testConcurrentReadersNeverObservePartialFrames();
        testGenericControlRetainsNonLWQueueCompatibility();
        std::cout << "LW policy output transport tests passed"
                  << std::endl;
        return EXIT_SUCCESS;
    }
    catch (const std::exception& exception)
    {
        std::cerr << "LW policy output transport tests failed: "
                  << exception.what() << std::endl;
        return EXIT_FAILURE;
    }
}
