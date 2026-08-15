#include "rl_sdk.hpp"

#include <iostream>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

namespace
{
struct DestructionState
{
    bool derived_destructor_ran = false;
    bool derived_member_destroyed = false;
};

class DestructionSentinel
{
public:
    explicit DestructionSentinel(DestructionState& state) noexcept
        : state_(state)
    {
    }

    ~DestructionSentinel() noexcept
    {
        state_.derived_member_destroyed = true;
    }

private:
    DestructionState& state_;
};

class DestructionProbe final : public RL
{
public:
    explicit DestructionProbe(DestructionState& state) noexcept
        : state_(state), sentinel_(state)
    {
    }

    ~DestructionProbe() noexcept override
    {
        state_.derived_destructor_ran = true;
    }

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

private:
    DestructionState& state_;
    DestructionSentinel sentinel_;
};

static_assert(
    std::has_virtual_destructor<RL>::value,
    "RL must support destruction through its polymorphic base type");
static_assert(
    noexcept(std::declval<RL&>().~RL()),
    "RL destruction must be noexcept");
static_assert(
    std::is_nothrow_destructible<DestructionProbe>::value,
    "derived RL destruction must preserve the noexcept contract");

void Require(bool condition, const char* message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

void TestBaseOwnershipRunsCompleteDerivedDestruction()
{
    DestructionState state;
    std::unique_ptr<RL> runtime =
        std::make_unique<DestructionProbe>(state);
    runtime.reset();

    Require(
        state.derived_destructor_ran,
        "base ownership skipped the derived RL destructor");
    Require(
        state.derived_member_destroyed,
        "base ownership skipped derived RL member cleanup");
}
} // namespace

int main()
{
    try
    {
        TestBaseOwnershipRunsCompleteDerivedDestruction();
    }
    catch (const std::exception& exception)
    {
        std::cerr << "test_lw_rl_destruction failed: "
                  << exception.what() << std::endl;
        return 1;
    }
    std::cout << "test_lw_rl_destruction passed" << std::endl;
    return 0;
}
