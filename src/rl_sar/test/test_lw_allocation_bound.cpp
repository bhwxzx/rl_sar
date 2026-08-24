#include "fsm_LW.hpp"
#include "lw_runtime_core.hpp"

#include <algorithm>
#include <atomic>
#include <cstdlib>
#include <filesystem>
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

    void GetState(RobotState<float>* state) override
    {
        *state = source_state;
    }

    void SetCommand(const RobotCommand<float>*) override
    {
        ++commands_delivered;
    }

    RobotState<float> source_state;
    std::uint64_t commands_delivered = 0;
};

class AllocationFreeModel : public InferenceRuntime::Model
{
public:
    bool load(const std::string&) override
    {
        return true;
    }

    bool is_loaded() const override
    {
        return true;
    }

    void forwardInto(
        const InferenceRuntime::TensorView* inputs,
        std::size_t input_count,
        InferenceRuntime::MutableTensorView output) override
    {
        if (inputs == nullptr || input_count != 1
            || inputs[0].size != expected_input_size
            || output.size != kNumDofs)
        {
            throw std::logic_error("allocation model contract mismatch");
        }
        last_input_size = inputs[0].size;
        ++calls;
        std::fill_n(output.data, output.size, 0.0F);
    }

    std::string get_model_type() const override
    {
        return "test";
    }

    std::size_t expected_input_size = 0;
    std::size_t last_input_size = 0;
    std::size_t calls = 0;
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
        for (std::size_t index = 0; index < kNumDofs; ++index)
        {
            fsm_command->motor_command.q[index] =
                fsm_state->motor_state.q[index];
            fsm_command->motor_command.dq[index] = 0.0f;
            fsm_command->motor_command.tau[index] = 0.0f;
            fsm_command->motor_command.kp[index] = 20.0f;
            fsm_command->motor_command.kd[index] = 0.5f;
        }
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

void initializeValidState(AllocationTestRL& rl)
{
    rl.InitJointNum(kNumDofs);
    rl.source_state.motor_state.resize(kNumDofs);
    rl.source_state.imu.quaternion = {1.0f, 0.0f, 0.0f, 0.0f};
    rl.source_state.imu.gyroscope = {0.0f, 0.0f, 0.0f};
    for (std::size_t index = 0; index < kNumDofs; ++index)
    {
        rl.source_state.motor_state.q[index] =
            0.01f * static_cast<float>(index);
        rl.source_state.motor_state.dq[index] = 0.0f;
        rl.source_state.motor_state.tau_est[index] = 0.0f;
    }
    rl.robot_state = rl.source_state;
}

void testWarmedCompleteControlCycleDoesNotAllocate()
{
    AllocationTestRL rl;
    rl.SetLWBaseRuntimeConfiguration(makeBaseConfiguration());
    initializeValidState(rl);
    const auto state = std::make_shared<AllocationTestState>(rl);
    rl.fsm.AddState(state);
    rl.fsm.SetInitialState("AllocationTestState");

    LWRuntimeCore core;
    core.bind(rl, {});
    core.publishInitialPolicyInput();
    std::uint64_t hook_calls = 0;
    auto run_cycle = [&]()
    {
        core.runControlCycle(
            LWControlCycleHooks{
                [&]() { ++hook_calls; },
                [&]() { ++hook_calls; },
                []() { return true; },
                []() { return true; },
                [&]() { ++hook_calls; },
                [&]() { ++hook_calls; }});
    };
    run_cycle();

    allocation_count.store(0, std::memory_order_relaxed);
    count_allocations.store(true, std::memory_order_release);
    for (std::size_t iteration = 0; iteration < 10000; ++iteration)
    {
        run_cycle();
    }
    count_allocations.store(false, std::memory_order_release);

    require(
        rl.commands_delivered == 10001,
        "complete control cycle did not deliver every command");
    require(
        hook_calls == 4 * 10001,
        "complete control cycle did not invoke every adapter hook");
    require(
        allocation_count.load(std::memory_order_relaxed) == 0,
        "warmed complete control cycle allocated");
}

void prepareTransitionPolicies(AllocationTestRL& rl)
{
    const std::filesystem::path policy_root =
        std::filesystem::canonical(POLICY_DIR);
    rl.SetPolicyRoot(policy_root);
    rl.robot_name = "LW";
    rl.ReadYaml("LW", "base.yaml");
    rl.SetLWBaseRuntimeConfiguration(
        ValidateLWBaseConfiguration(
            rl.params.config_node,
            (policy_root / "LW/base.yaml").string()));
    initializeValidState(rl);
    for (const std::string policy : {
             "LW/robot_lab/leg_to_wheel",
             "LW/robot_lab/wheel_to_leg"})
    {
        rl.PreloadModel(policy);
        rl.PreloadLWPolicyContext(policy);
    }
}

template <typename TransitionState>
void requireWarmedTransitionRunDoesNotAllocate(
    AllocationTestRL& rl,
    const std::string& policy)
{
    TransitionState state(&rl);
    state.fsm_state = &rl.robot_state;
    state.fsm_command = &rl.robot_command;
    state.Enter();
    const LWPolicyActivation* activation = rl.LoadLWPolicyActivation();
    require(
        activation && activation->definition
            && activation->definition->path == policy,
        "transition did not activate the expected policy");

    const auto source_time = std::chrono::steady_clock::now();
    LWPolicyOutputFrame output;
    output.generation = activation->generation;
    output.frame = 1;
    output.source_input_sequence = 1;
    output.source_state_time = source_time;
    output.dof_pos.assign(kNumDofs, 0.0f);
    output.dof_vel.assign(kNumDofs, 0.0f);
    output.dof_tau.assign(kNumDofs, 0.0f);
    require(
        rl.PublishLWPolicyOutput(output, *activation),
        "transition output warmup publication failed");
    rl.PublishLWPolicyProgress(activation->generation, 1);
    state.Run();

    allocation_count.store(0, std::memory_order_relaxed);
    count_allocations.store(true, std::memory_order_release);
    for (std::size_t iteration = 0; iteration < 100; ++iteration)
    {
        state.Run();
    }
    count_allocations.store(false, std::memory_order_release);

    require(
        allocation_count.load(std::memory_order_relaxed) == 0,
        policy + " transition Run allocated after warmup");
    state.Exit();
}

void testWarmedTransitionRunsDoNotAllocate()
{
    AllocationTestRL rl;
    prepareTransitionPolicies(rl);
    requireWarmedTransitionRunDoesNotAllocate<
        LW_fsm::RLFSMStateRL_LegToWheel>(
        rl,
        "LW/robot_lab/leg_to_wheel");
    requireWarmedTransitionRunDoesNotAllocate<
        LW_fsm::RLFSMStateRL_WheelToLeg>(
        rl,
        "LW/robot_lab/wheel_to_leg");
}

void publishMotionReference(
    AllocationTestRL& rl,
    std::uint64_t generation)
{
    LWMotionReferenceSnapshot reference;
    reference.generation = generation;
    reference.joint_pos.assign(kNumDofs, 0.0F);
    reference.joint_vel.assign(kNumDofs, 0.0F);
    reference.anchor_quat = {1.0F, 0.0F, 0.0F, 0.0F};
    reference.init_quat = {1.0F, 0.0F, 0.0F, 0.0F};
    rl.PublishLWMotionReference(std::move(reference));
}

void testWarmedInferencePipelineDoesNotAllocate()
{
    AllocationTestRL rl;
    const std::filesystem::path policy_root =
        std::filesystem::canonical(POLICY_DIR);
    rl.SetPolicyRoot(policy_root);
    rl.robot_name = "LW";
    rl.ReadYaml("LW", "base.yaml");
    rl.SetLWBaseRuntimeConfiguration(
        ValidateLWBaseConfiguration(
            rl.params.config_node,
            (policy_root / "LW/base.yaml").string()));
    initializeValidState(rl);
    const auto state = std::make_shared<AllocationTestState>(rl);
    rl.fsm.AddState(state);
    rl.fsm.SetInitialState("AllocationTestState");

    struct PolicyCase
    {
        const char* path;
        std::shared_ptr<AllocationFreeModel> model;
        bool needs_motion = false;
    };
    std::vector<PolicyCase> policies;
    for (const char* path : {
             "LW/robot_lab/leg_loco",
             "LW/robot_lab/wheel_loco",
             "LW/robot_lab/leg_to_wheel",
             "LW/robot_lab/wheel_to_leg"})
    {
        const auto config_path = policy_root / path / "config.yaml";
        const YAML::Node policy_config =
            YAML::LoadFile(config_path.string())[path];
        auto validated = ValidateLWPolicyConfiguration(
            rl.params.config_node,
            policy_config,
            config_path.string());
        auto model = std::make_shared<AllocationFreeModel>();
        model->expected_input_size = validated.dimensions.model_input;
        const bool needs_motion = validated.runtime.needs_motion_reference;
        rl.preloaded_lw_policy_configs_[path] = std::move(validated);
        rl.preloaded_models_[path] = model;
        rl.PreloadLWPolicyContext(path);
        policies.push_back({path, std::move(model), needs_motion});
    }

    LWRuntimeCore core;
    core.bind(rl, {});
    core.publishInitialPolicyInput();
    for (const PolicyCase& policy : policies)
    {
        const std::uint64_t generation = rl.ActivateLWPolicy(policy.path);
        const LWPolicyActivation* activation = rl.LoadLWPolicyActivation();
        require(
            activation && activation->definition,
            "allocation policy activation is missing");
        rl.source_state.motor_state.q =
            activation->definition->runtime.default_dof_pos;
        rl.robot_state = rl.source_state;
        if (policy.needs_motion)
        {
            publishMotionReference(rl, generation);
        }
        core.runControlCycle({});
        core.runInferenceCycle(false);
        const std::size_t warmed_calls = policy.model->calls;
        require(warmed_calls == 1, "inference warmup did not run");

        allocation_count.store(0, std::memory_order_relaxed);
        count_allocations.store(true, std::memory_order_release);
        for (std::size_t iteration = 0; iteration < 100; ++iteration)
        {
            core.runControlCycle({});
            core.runInferenceCycle(false);
        }
        count_allocations.store(false, std::memory_order_release);

        require(
            allocation_count.load(std::memory_order_relaxed) == 0,
            std::string(policy.path)
                + " warmed inference pipeline allocated "
                + std::to_string(
                    allocation_count.load(std::memory_order_relaxed))
                + " times");
        require(
            policy.model->calls == warmed_calls + 100
                && policy.model->last_input_size
                    == policy.model->expected_input_size,
            std::string(policy.path)
                + " inference input contract differs after switching");
    }
}
} // namespace

int main()
{
    try
    {
        testSteadyStateConfigurationAndStateTransportDoNotAllocate();
        testWarmedCompleteControlCycleDoesNotAllocate();
        testWarmedTransitionRunsDoNotAllocate();
        testWarmedInferencePipelineDoesNotAllocate();
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
