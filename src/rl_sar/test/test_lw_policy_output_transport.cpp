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
    Clock::time_point source_state_time)
{
    const float tag = static_cast<float>(frame);
    return {
        generation,
        0,
        frame,
        frame,
        source_state_time,
        std::vector<float>(dofs, tag),
        std::vector<float>(dofs, 2.0f * tag),
        std::vector<float>(dofs, 3.0f * tag)};
}

void testRejectsPartialAndWrongGenerationFrames()
{
    constexpr size_t dofs = 10;
    constexpr std::uint64_t generation = 7;
    LWPolicyOutputTransport transport;
    transport.configure(dofs);
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

    auto missing_input_sequence =
        makeFrame(generation, 1, dofs, now);
    missing_input_sequence.source_input_sequence = 0;
    require(
        !transport.publish(
            std::move(missing_input_sequence),
            generation,
            dofs),
        "transport accepted a frame without an input sequence");

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
    transport.configure(dofs);
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
            makeFrame(
                1,
                11,
                dofs,
                now + std::chrono::nanoseconds(1)),
            1,
            dofs),
        "second complete frame was rejected");
    const auto latest = transport.load();
    require(
        latest
            && latest->sequence == 2
            && latest->frame == 11
            && latest->source_input_sequence == 11
            && latest->dof_pos[0] == 11.0f
            && latest->dof_vel[0] == 22.0f
            && latest->dof_tau[0] == 33.0f,
        "latest-frame slot retained or mixed an older frame");

    require(
        !transport.publish(
            makeFrame(
                1,
                11,
                dofs,
                now + std::chrono::nanoseconds(2)),
            1,
            dofs),
        "transport accepted duplicate input provenance");
    require(
        !transport.publish(
            makeFrame(
                1,
                12,
                dofs,
                now + std::chrono::nanoseconds(1)),
            1,
            dofs),
        "transport accepted regressing state provenance");

    require(
        transport.load()->generation == 1,
        "latest generation changed without a new publication");
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

void testInputFreshnessAndProvenanceClassification()
{
    constexpr std::uint64_t generation = 5;
    const auto capture_time = Clock::now();
    const auto max_age = std::chrono::milliseconds(60);
    LWPolicyInputSnapshot input{
        generation,
        10,
        capture_time,
        {},
        {}};

    const auto evaluate = [&](const LWPolicyInputSnapshot* candidate,
                              Clock::time_point now,
                              std::uint64_t last_generation = 0,
                              std::uint64_t last_sequence = 0,
                              Clock::time_point last_time = {})
    {
        return EvaluateLWPolicyInput(
            candidate,
            generation,
            now,
            max_age,
            last_generation,
            last_sequence,
            last_time);
    };

    require(
        evaluate(nullptr, capture_time) == LWPolicyInputStatus::Missing,
        "missing input was not classified");
    auto wrong_generation = input;
    wrong_generation.generation = generation - 1;
    require(
        evaluate(&wrong_generation, capture_time)
            == LWPolicyInputStatus::GenerationMismatch,
        "input generation mismatch was not classified");
    auto incomplete = input;
    incomplete.sequence = 0;
    require(
        evaluate(&incomplete, capture_time)
            == LWPolicyInputStatus::Incomplete,
        "input without a sequence was not classified");
    require(
        evaluate(&input, capture_time + max_age)
            == LWPolicyInputStatus::Ready,
        "input at the age boundary was rejected");
    require(
        evaluate(
            &input,
            capture_time + max_age + std::chrono::nanoseconds(1))
            == LWPolicyInputStatus::Stale,
        "over-age input was accepted");
    require(
        evaluate(&input, capture_time - std::chrono::nanoseconds(1))
            == LWPolicyInputStatus::Future,
        "future-dated input was accepted");
    require(
        evaluate(&input, capture_time, generation, 10, capture_time)
            == LWPolicyInputStatus::Duplicate,
        "duplicate input was not classified");

    auto regressing_sequence = input;
    regressing_sequence.sequence = 9;
    require(
        evaluate(
            &regressing_sequence,
            capture_time,
            generation,
            10,
            capture_time)
            == LWPolicyInputStatus::Regressing,
        "regressing input sequence was accepted");
    auto repeated_time = input;
    repeated_time.sequence = 11;
    require(
        evaluate(
            &repeated_time,
            capture_time,
            generation,
            10,
            capture_time)
            == LWPolicyInputStatus::Regressing,
        "new sequence with repeated state time was accepted");

    require(
        !LWPolicyInputRequiresFallback(LWPolicyInputStatus::Duplicate),
        "duplicate input incorrectly required fallback");
    require(
        !LWPolicyInputRequiresFallback(
            LWPolicyInputStatus::GenerationMismatch),
        "generation switch incorrectly required fallback");
    for (const auto status : {
             LWPolicyInputStatus::Incomplete,
             LWPolicyInputStatus::Regressing,
             LWPolicyInputStatus::Future,
             LWPolicyInputStatus::Stale})
    {
        require(
            LWPolicyInputRequiresFallback(status),
            std::string("unsafe input did not require fallback: ")
                + LWPolicyInputStatusName(status));
    }
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

    require(
        LWPolicyOutputRequiresFallback(LWPolicyOutputStatus::Stale, false),
        "stale policy output did not require fallback");
    require(
        LWPolicyOutputRequiresFallback(LWPolicyOutputStatus::Incomplete, false),
        "incomplete policy output did not require fallback");
    require(
        !LWPolicyOutputRequiresFallback(LWPolicyOutputStatus::Missing, false),
        "initial policy-output wait triggered fallback too early");
    require(
        LWPolicyOutputRequiresFallback(LWPolicyOutputStatus::Missing, true),
        "expired initial policy-output wait retained the old command");
    require(
        !LWPolicyOutputRequiresFallback(LWPolicyOutputStatus::Ready, true),
        "ready policy output incorrectly triggered fallback");
}

void testConcurrentReaderNeverObservesPartialFrames()
{
    constexpr size_t dofs = 10;
    constexpr std::uint64_t generation = 9;
    constexpr std::uint64_t iterations = 50000;
    LWPolicyOutputTransport transport;
    transport.configure(dofs);
    const auto source_epoch = Clock::now();
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
                        source_epoch
                            + std::chrono::nanoseconds(frame)),
                    generation,
                    dofs))
            {
                failed.store(true, std::memory_order_release);
                return;
            }
        }
    });

    std::thread reader([&]()
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
                || output->source_input_sequence != output->frame
                || output->source_state_time.time_since_epoch().count() == 0
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

    start.store(true, std::memory_order_release);
    writer.join();
    reader.join();

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
        testInputFreshnessAndProvenanceClassification();
        testFrameFreshnessClassification();
        testConcurrentReaderNeverObservesPartialFrames();
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
