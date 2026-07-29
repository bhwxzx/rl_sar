#include "fsm_LW.hpp"

#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace
{
void require(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

class CoreTestState : public FSMState
{
public:
    CoreTestState(std::string name, std::string next)
        : FSMState(std::move(name)), next_(std::move(next))
    {
    }

    void Enter() override {}
    void Run() override {}
    void Exit() override {}
    std::string CheckChange() override
    {
        return next_;
    }

private:
    std::string next_;
};

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

void testCoreRejectsUnregisteredTargets()
{
    FSM fsm;
    const auto safe_state =
        std::make_shared<CoreTestState>("SafeState", "MissingState");
    fsm.AddState(safe_state);
    fsm.SetInitialState("SafeState");

    fsm.Run();
    require(fsm.current_state_ == safe_state, "invalid internal target changed current state");
    require(fsm.next_state_ == safe_state, "invalid internal target changed next state");
    require(fsm.mode_ == FSM::Mode::NORMAL, "invalid internal target changed FSM mode");

    fsm.RequestStateChange("MissingExternalState");
    require(fsm.current_state_ == safe_state, "invalid external request changed current state");
    require(fsm.next_state_ == safe_state, "invalid external request changed next state");
    require(fsm.mode_ == FSM::Mode::NORMAL, "invalid external request changed FSM mode");
}

using StateMap = std::unordered_map<std::string, std::shared_ptr<FSMState>>;

StateMap createLWStates(RL& rl, std::unordered_set<std::string>& registered_names)
{
    LWFSMFactory factory("RLFSMStatePassive");
    StateMap states;
    for (const auto& name : factory.GetSupportedStates())
    {
        registered_names.insert(name);
        auto state = factory.CreateState(&rl, name);
        require(state != nullptr, "factory did not create registered state " + name);
        require(
            state->GetStateName() == name,
            "factory state name does not match registered name " + name);
        states.emplace(name, std::move(state));
    }
    require(states.size() == factory.GetSupportedStates().size(), "duplicate LW state name");
    return states;
}

void setReadyProgress(const std::shared_ptr<FSMState>& state)
{
    if (auto get_up_leg =
            std::dynamic_pointer_cast<LW_fsm::RLFSMStateGetUp_Leg>(state))
    {
        get_up_leg->percent_getup = 1.0f;
    }
    else if (auto get_up_wheel =
                 std::dynamic_pointer_cast<LW_fsm::RLFSMStateGetUp_Wheel>(state))
    {
        get_up_wheel->percent_getup = 1.0f;
    }
}

void clearInput(RL& rl)
{
    rl.control.current_keyboard = Input::Keyboard::None;
    rl.control.last_keyboard = Input::Keyboard::None;
    rl.control.current_gamepad = Input::Gamepad::None;
    rl.control.last_gamepad = Input::Gamepad::None;
}

void requireKnownTarget(
    const std::shared_ptr<FSMState>& state,
    const std::unordered_set<std::string>& registered_names,
    const std::string& context)
{
    const std::string target = state->CheckChange();
    require(
        registered_names.count(target) == 1,
        context + " returned unregistered state " + target);
}

struct TransitionCase
{
    const char* state;
    Input::Keyboard keyboard;
    Input::Gamepad gamepad;
    const char* expected;
    bool ready = false;
};

void testAcceptedTransitionTable(
    RL& rl,
    const StateMap& states,
    const std::unordered_set<std::string>& registered_names)
{
    const std::vector<TransitionCase> transitions = {
        {"RLFSMStatePassive", Input::Keyboard::Num0, Input::Gamepad::None,
         "RLFSMStateGetUp_Leg"},
        {"RLFSMStatePassive", Input::Keyboard::None, Input::Gamepad::A,
         "RLFSMStateGetUp_Leg"},
        {"RLFSMStatePassive", Input::Keyboard::Num2, Input::Gamepad::None,
         "RLFSMStateGetUp_Wheel"},
        {"RLFSMStatePassive", Input::Keyboard::None, Input::Gamepad::Y,
         "RLFSMStateGetUp_Wheel"},

        {"RLFSMStateGetUp_Leg", Input::Keyboard::P, Input::Gamepad::None,
         "RLFSMStatePassive"},
        {"RLFSMStateGetUp_Leg", Input::Keyboard::None, Input::Gamepad::LB_X,
         "RLFSMStatePassive"},
        {"RLFSMStateGetUp_Leg", Input::Keyboard::Num2, Input::Gamepad::None,
         "RLFSMStateGetUp_Wheel"},
        {"RLFSMStateGetUp_Leg", Input::Keyboard::None, Input::Gamepad::Y,
         "RLFSMStateGetUp_Wheel"},
        {"RLFSMStateGetUp_Leg", Input::Keyboard::Num1, Input::Gamepad::None,
         "RLFSMStateRLLocomotion_Leg", true},
        {"RLFSMStateGetUp_Leg", Input::Keyboard::None, Input::Gamepad::RB_DPadUp,
         "RLFSMStateRLLocomotion_Leg", true},
        {"RLFSMStateGetUp_Leg", Input::Keyboard::Num9, Input::Gamepad::None,
         "RLFSMStateGetDown", true},
        {"RLFSMStateGetUp_Leg", Input::Keyboard::None, Input::Gamepad::B,
         "RLFSMStateGetDown", true},

        {"RLFSMStateGetUp_Wheel", Input::Keyboard::P, Input::Gamepad::None,
         "RLFSMStatePassive"},
        {"RLFSMStateGetUp_Wheel", Input::Keyboard::None, Input::Gamepad::LB_X,
         "RLFSMStatePassive"},
        {"RLFSMStateGetUp_Wheel", Input::Keyboard::Num0, Input::Gamepad::None,
         "RLFSMStateGetUp_Leg"},
        {"RLFSMStateGetUp_Wheel", Input::Keyboard::None, Input::Gamepad::A,
         "RLFSMStateGetUp_Leg"},
        {"RLFSMStateGetUp_Wheel", Input::Keyboard::Num3, Input::Gamepad::None,
         "RLFSMStateRLLocomotion_Wheel", true},
        {"RLFSMStateGetUp_Wheel", Input::Keyboard::None, Input::Gamepad::RB_DPadDown,
         "RLFSMStateRLLocomotion_Wheel", true},
        {"RLFSMStateGetUp_Wheel", Input::Keyboard::Num9, Input::Gamepad::None,
         "RLFSMStateGetDown", true},
        {"RLFSMStateGetUp_Wheel", Input::Keyboard::None, Input::Gamepad::B,
         "RLFSMStateGetDown", true},

        {"RLFSMStateGetDown", Input::Keyboard::P, Input::Gamepad::None,
         "RLFSMStatePassive"},
        {"RLFSMStateGetDown", Input::Keyboard::None, Input::Gamepad::LB_X,
         "RLFSMStatePassive"},
        {"RLFSMStateGetDown", Input::Keyboard::Num0, Input::Gamepad::None,
         "RLFSMStateGetUp_Leg"},
        {"RLFSMStateGetDown", Input::Keyboard::None, Input::Gamepad::A,
         "RLFSMStateGetUp_Leg"},
        {"RLFSMStateGetDown", Input::Keyboard::Num2, Input::Gamepad::None,
         "RLFSMStateGetUp_Wheel"},
        {"RLFSMStateGetDown", Input::Keyboard::None, Input::Gamepad::Y,
         "RLFSMStateGetUp_Wheel"},

        {"RLFSMStateRLLocomotion_Leg", Input::Keyboard::P, Input::Gamepad::None,
         "RLFSMStatePassive"},
        {"RLFSMStateRLLocomotion_Leg", Input::Keyboard::None, Input::Gamepad::LB_X,
         "RLFSMStatePassive"},
        {"RLFSMStateRLLocomotion_Leg", Input::Keyboard::Num9, Input::Gamepad::None,
         "RLFSMStateGetDown"},
        {"RLFSMStateRLLocomotion_Leg", Input::Keyboard::None, Input::Gamepad::B,
         "RLFSMStateGetDown"},
        {"RLFSMStateRLLocomotion_Leg", Input::Keyboard::Num1,
         Input::Gamepad::None, "RLFSMStateRLLocomotion_Leg"},
        {"RLFSMStateRLLocomotion_Leg", Input::Keyboard::None,
         Input::Gamepad::RB_DPadUp, "RLFSMStateRLLocomotion_Leg"},
        {"RLFSMStateRLLocomotion_Leg", Input::Keyboard::Num4, Input::Gamepad::None,
         "RLFSMStateRL_LegToWheel"},
        {"RLFSMStateRLLocomotion_Leg", Input::Keyboard::None,
         Input::Gamepad::RB_DPadLeft, "RLFSMStateRL_LegToWheel"},

        {"RLFSMStateRLLocomotion_Wheel", Input::Keyboard::P, Input::Gamepad::None,
         "RLFSMStatePassive"},
        {"RLFSMStateRLLocomotion_Wheel", Input::Keyboard::None,
         Input::Gamepad::LB_X, "RLFSMStatePassive"},
        {"RLFSMStateRLLocomotion_Wheel", Input::Keyboard::Num9,
         Input::Gamepad::None, "RLFSMStateGetDown"},
        {"RLFSMStateRLLocomotion_Wheel", Input::Keyboard::None,
         Input::Gamepad::B, "RLFSMStateGetDown"},
        {"RLFSMStateRLLocomotion_Wheel", Input::Keyboard::Num3,
         Input::Gamepad::None, "RLFSMStateRLLocomotion_Wheel"},
        {"RLFSMStateRLLocomotion_Wheel", Input::Keyboard::None,
         Input::Gamepad::RB_DPadDown, "RLFSMStateRLLocomotion_Wheel"},
        {"RLFSMStateRLLocomotion_Wheel", Input::Keyboard::Num5,
         Input::Gamepad::None, "RLFSMStateRL_WheelToLeg"},
        {"RLFSMStateRLLocomotion_Wheel", Input::Keyboard::None,
         Input::Gamepad::RB_DPadRight, "RLFSMStateRL_WheelToLeg"},

        {"RLFSMStateRL_LegToWheel", Input::Keyboard::P, Input::Gamepad::None,
         "RLFSMStatePassive"},
        {"RLFSMStateRL_LegToWheel", Input::Keyboard::None, Input::Gamepad::LB_X,
         "RLFSMStatePassive"},
        {"RLFSMStateRL_LegToWheel", Input::Keyboard::Num9, Input::Gamepad::None,
         "RLFSMStateGetDown"},
        {"RLFSMStateRL_LegToWheel", Input::Keyboard::None, Input::Gamepad::B,
         "RLFSMStateGetDown"},

        {"RLFSMStateRL_WheelToLeg", Input::Keyboard::P, Input::Gamepad::None,
         "RLFSMStatePassive"},
        {"RLFSMStateRL_WheelToLeg", Input::Keyboard::None, Input::Gamepad::LB_X,
         "RLFSMStatePassive"},
        {"RLFSMStateRL_WheelToLeg", Input::Keyboard::Num9, Input::Gamepad::None,
         "RLFSMStateGetDown"},
        {"RLFSMStateRL_WheelToLeg", Input::Keyboard::None, Input::Gamepad::B,
         "RLFSMStateGetDown"},
    };

    for (const auto& transition : transitions)
    {
        clearInput(rl);
        const auto state = states.at(transition.state);
        if (transition.ready)
        {
            setReadyProgress(state);
        }
        rl.control.current_keyboard = transition.keyboard;
        rl.control.current_gamepad = transition.gamepad;
        const std::string actual = state->CheckChange();
        require(
            actual == transition.expected,
            std::string(transition.state) + " returned " + actual
                + " instead of " + transition.expected);
        require(
            registered_names.count(actual) == 1,
            std::string(transition.state) + " returned unregistered target " + actual);
    }
}

void testEveryInputReturnsRegisteredState(
    RL& rl,
    const StateMap& states,
    const std::unordered_set<std::string>& registered_names)
{
    for (const auto& entry : states)
    {
        const auto& state = entry.second;
        setReadyProgress(state);

        for (int value = static_cast<int>(Input::Keyboard::None);
             value <= static_cast<int>(Input::Keyboard::Right);
             ++value)
        {
            clearInput(rl);
            rl.control.current_keyboard = static_cast<Input::Keyboard>(value);
            requireKnownTarget(
                state, registered_names, entry.first + " keyboard input");
        }

        for (int value = static_cast<int>(Input::Gamepad::None);
             value <= static_cast<int>(Input::Gamepad::LB_RB);
             ++value)
        {
            clearInput(rl);
            rl.control.current_gamepad = static_cast<Input::Gamepad>(value);
            requireKnownTarget(
                state, registered_names, entry.first + " gamepad input");
        }
    }
}

void testMorphologyTransitionsIgnoreGetUpInput(
    RL& rl,
    const StateMap& states)
{
    for (const char* name :
         {"RLFSMStateRL_LegToWheel", "RLFSMStateRL_WheelToLeg"})
    {
        const auto state = states.at(name);

        clearInput(rl);
        rl.control.current_keyboard = Input::Keyboard::Num0;
        require(
            state->CheckChange() == name,
            std::string(name) + " accepted keyboard get-up during transition");

        clearInput(rl);
        rl.control.current_gamepad = Input::Gamepad::A;
        require(
            state->CheckChange() == name,
            std::string(name) + " accepted gamepad get-up during transition");
    }
}

void testGetDownCompletesToPassive(RL& rl, const StateMap& states)
{
    const auto get_down =
        std::dynamic_pointer_cast<LW_fsm::RLFSMStateGetDown>(
            states.at("RLFSMStateGetDown"));
    require(get_down != nullptr, "get-down state has unexpected type");

    clearInput(rl);
    get_down->percent_getdown = 1.0f;
    require(
        get_down->CheckChange() == "RLFSMStatePassive",
        "completed get-down did not return to passive");
}
} // namespace

int main()
{
    try
    {
        testCoreRejectsUnregisteredTargets();

        TestRL rl;
        std::unordered_set<std::string> registered_names;
        const auto states = createLWStates(rl, registered_names);
        testAcceptedTransitionTable(rl, states, registered_names);
        testEveryInputReturnsRegisteredState(rl, states, registered_names);
        testMorphologyTransitionsIgnoreGetUpInput(rl, states);
        testGetDownCompletesToPassive(rl, states);

        std::cout << "LW FSM transition tests passed" << std::endl;
        return EXIT_SUCCESS;
    }
    catch (const std::exception& exception)
    {
        std::cerr << "LW FSM transition tests failed: "
                  << exception.what() << std::endl;
        return EXIT_FAILURE;
    }
}
