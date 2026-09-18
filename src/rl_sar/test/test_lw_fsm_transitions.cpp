#include "fsm_LW.hpp"

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <unistd.h>

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
    std::string_view CheckChange() override
    {
        return next_;
    }

private:
    std::string next_;
};

class TestRL : public RL
{
public:
    LWPolicyEntryGuard::Time entry_now{std::chrono::seconds(100)};
    LWPolicyEntryGuard::Time LWEntryCheckNow() const override { return entry_now; }
    std::vector<float> Forward() override
    {
        return {};
    }

    void GetState(RobotState<float>*) override {}
    void SetCommand(const RobotCommand<float>*) override {}
};

class TemporaryPolicyRoot
{
public:
    TemporaryPolicyRoot()
        : root(
              std::filesystem::temp_directory_path()
              / ("lw-fsm-motion-preload-"
                 + std::to_string(getpid())))
    {
        std::error_code error;
        std::filesystem::remove_all(root, error);
        std::filesystem::create_directories(root / "LW/robot_lab");
        copyFile("LW/base.yaml");
    }

    ~TemporaryPolicyRoot()
    {
        std::error_code error;
        std::filesystem::remove_all(root, error);
    }

    void copyTransition(
        const std::string& name,
        const std::string& motion_file,
        bool include_motion = true)
    {
        const std::filesystem::path relative =
            std::filesystem::path("LW/robot_lab") / name;
        std::filesystem::create_directories(root / relative);
        copyFile((relative / "config.yaml").generic_string());
        copyFile((relative / "policy.onnx").generic_string());
        if (include_motion)
        {
            copyFile((relative / motion_file).generic_string());
        }
    }

    void removeMotion(
        const std::string& name,
        const std::string& motion_file) const
    {
        require(
            std::filesystem::remove(
                root / "LW/robot_lab" / name / motion_file),
            "failed to remove preloaded transition motion");
    }

    std::filesystem::path root;

private:
    void copyFile(const std::string& relative)
    {
        const std::filesystem::path source =
            std::filesystem::path(POLICY_DIR) / relative;
        const std::filesystem::path destination = root / relative;
        std::filesystem::create_directories(destination.parent_path());
        std::filesystem::copy_file(
            source,
            destination,
            std::filesystem::copy_options::overwrite_existing);
    }
};

void configurePolicyRoot(TestRL& rl, const std::filesystem::path& root)
{
    rl.SetPolicyRoot(root);
    rl.robot_name = "LW";
    rl.ReadYaml("LW", "base.yaml");
}

void preloadTransition(TestRL& rl, const std::string& policy)
{
    rl.PreloadModel(policy);
    rl.PreloadLWPolicyContext(policy);
    require(
        rl.GetPreloadedLWMotionPlayer(policy) != nullptr,
        "transition motion player was not preloaded: " + policy);
    const auto definition = rl.GetLWPolicyDefinition(policy);
    require(
        definition && definition->prepared_motion,
        "transition policy did not retain prepared motion: " + policy);
}

template <typename TransitionState>
void enterPreloadedTransition(
    TestRL& rl,
    const std::string& expected_policy)
{
    RobotState<float> robot_state;
    robot_state.motor_state.resize(10);
    robot_state.imu.quaternion = {1.0f, 0.0f, 0.0f, 0.0f};
    RobotCommand<float> robot_command;
    robot_command.motor_command.resize(10);

    TransitionState state(&rl);
    state.fsm_state = &robot_state;
    state.fsm_command = &robot_command;
    MotionLoaderLW* const prepared_player =
        rl.GetPreloadedLWMotionPlayer(expected_policy);
    state.Enter();

    const auto activation = rl.LoadLWPolicyActivation();
    require(activation != nullptr, "preloaded transition did not activate");
    require(
        activation->definition->path == expected_policy,
        "preloaded transition activated the wrong policy");
    require(
        rl.motion_loader_lw == prepared_player,
        "transition did not reuse the startup-preloaded player");
    require(rl.motion_length > 0.0f, "transition motion length is invalid");
    const auto reference = rl.LoadLWMotionReference();
    require(reference != nullptr, "transition did not publish its first reference");
    require(reference->joint_pos.size() == 10, "reference position size is invalid");
    require(reference->joint_vel.size() == 10, "reference velocity size is invalid");

    state.Exit();
    require(
        rl.motion_loader_lw == nullptr,
        "transition exit retained an active motion player");
    require(
        rl.LoadLWPolicyActivation() == nullptr,
        "transition exit retained policy activation");
}

void testMorphologyEnterUsesStartupPreload()
{
    TemporaryPolicyRoot policy_root;
    policy_root.copyTransition(
        "leg_to_wheel", "leg_to_wheel_transform_60hz.csv");
    policy_root.copyTransition(
        "wheel_to_leg", "wheel_to_leg_transform_60hz.csv");

    TestRL rl;
    configurePolicyRoot(rl, policy_root.root);
    const std::string leg_to_wheel = "LW/robot_lab/leg_to_wheel";
    const std::string wheel_to_leg = "LW/robot_lab/wheel_to_leg";
    preloadTransition(rl, leg_to_wheel);
    preloadTransition(rl, wheel_to_leg);

    policy_root.removeMotion(
        "leg_to_wheel", "leg_to_wheel_transform_60hz.csv");
    policy_root.removeMotion(
        "wheel_to_leg", "wheel_to_leg_transform_60hz.csv");

    enterPreloadedTransition<LW_fsm::RLFSMStateRL_LegToWheel>(
        rl, leg_to_wheel);
    enterPreloadedTransition<LW_fsm::RLFSMStateRL_WheelToLeg>(
        rl, wheel_to_leg);
}

void testMissingMotionFailsDuringPreload()
{
    TemporaryPolicyRoot policy_root;
    policy_root.copyTransition(
        "leg_to_wheel",
        "leg_to_wheel_transform_60hz.csv",
        false);

    TestRL rl;
    configurePolicyRoot(rl, policy_root.root);
    const std::string policy = "LW/robot_lab/leg_to_wheel";
    rl.PreloadModel(policy);
    bool rejected = false;
    try
    {
        rl.PreloadLWPolicyContext(policy);
    }
    catch (const std::runtime_error& exception)
    {
        rejected =
            std::string(exception.what()).find("Failed to open motion file")
            != std::string::npos;
    }
    require(rejected, "missing transition motion did not fail during preload");
    require(
        rl.GetLWPolicyDefinition(policy) == nullptr,
        "failed motion preload published a policy definition");
    require(
        rl.GetPreloadedLWMotionPlayer(policy) == nullptr,
        "failed motion preload published a motion player");
}

void testPolicyRootResolution(TestRL& rl)
{
    const std::filesystem::path root =
        std::filesystem::canonical(POLICY_DIR);
    rl.SetPolicyRoot(root);
    require(
        rl.ResolvePolicyPath("LW/base.yaml")
            == (root / "LW/base.yaml").string(),
        "policy root did not resolve a valid relative path");

    for (const std::string invalid : {
             "../policy/LW/base.yaml",
             "LW/../base.yaml",
             "/tmp/base.yaml",
             "LW//base.yaml",
         })
    {
        bool rejected = false;
        try
        {
            rl.ResolvePolicyPath(invalid);
        }
        catch (const std::runtime_error&)
        {
            rejected = true;
        }
        require(rejected, "unsafe policy path was accepted: " + invalid);
    }
}

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
        auto typed = std::dynamic_pointer_cast<RLFSMState>(state);
        typed->fsm_state = &rl.robot_state;
        typed->fsm_command = &rl.robot_command;
        states.emplace(name, std::move(state));
    }
    require(states.size() == factory.GetSupportedStates().size(), "duplicate LW state name");
    return states;
}

void setReadyProgress(RL& rl, const std::shared_ptr<FSMState>& state)
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
    if (state->GetStateName() == "RLFSMStateGetUp_Leg"
        || state->GetStateName() == "RLFSMStateGetUp_Wheel") {
        auto& test = dynamic_cast<TestRL&>(rl);
        rl.control.current_keyboard = Input::Keyboard::None;
        rl.control.current_gamepad = Input::Gamepad::None;
        for (int i=0; i<=100; ++i) {
            test.entry_now += std::chrono::milliseconds(5);
            rl.robot_state.imu.sample_time = test.entry_now;
            state->CheckChange();
        }
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
    const std::string_view target = state->CheckChange();
    const std::string target_string(target);
    require(
        registered_names.count(target_string) == 1,
        context + " returned unregistered state " + target_string);
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
            setReadyProgress(rl, state);
        }
        rl.control.current_keyboard = transition.keyboard;
        rl.control.current_gamepad = transition.gamepad;
        const std::string_view actual = state->CheckChange();
        const std::string actual_string(actual);
        require(
            actual == transition.expected,
            std::string(transition.state) + " returned " + actual_string
                + " instead of " + transition.expected);
        require(
            registered_names.count(actual_string) == 1,
            std::string(transition.state)
                + " returned unregistered target " + actual_string);
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
        setReadyProgress(rl, state);

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

std::vector<float> tilt(float roll, float pitch=0)
{
    constexpr double rad=0.017453292519943295;
    double cr=std::cos(roll*rad/2), sr=std::sin(roll*rad/2);
    double cp=std::cos(pitch*rad/2), sp=std::sin(pitch*rad/2);
    return {float(cr*cp),float(sr*cp),float(cr*sp),float(-sr*sp)};
}

void testEntryGuardSamples()
{
    using namespace std::chrono_literals;
    LWPolicyEntryGuard g;
    auto now=LWPolicyEntryGuard::Time(100s);
    const auto feed=[&](const std::vector<float>& q, bool completed=true) {
        g.observe(q,now,now,completed,8,.5,.02);
    };
    for (float angle : {8.f,-8.f}) {
        g.reset();
        for (int i=0;i<=100;++i) { now+=5ms; feed(tilt(angle,angle)); }
        require(g.ready(), "8-degree boundary rejected");
        now+=5ms; feed(tilt(angle>0?8.01f:-8.01f));
        require(!g.ready(), "outside angle accepted");
    }
    g.reset();feed(tilt(0));
    const auto cached=now;
    now+=10ms;g.observe(tilt(0),cached,now,true,8,.5,.02);
    require(g.stable_seconds==0, "cached sample advanced stability");
    now+=500ms;g.observe(tilt(0),cached,now,true,8,.5,.02);
    require(!g.ready() && g.reason==LWPolicyEntryGuard::Reason::Stale, "stale sample accepted");
    feed(tilt(0));
    for(int i=0;i<99;++i) { now+=5ms;feed(tilt(0)); }
    require(!g.ready(), "admitted before hold duration");
    now+=5ms;feed(tilt(0));require(g.ready(), "continuous fresh samples not admitted");
    now+=30ms;feed(tilt(0));require(!g.ready(), "sample gap did not reset hold");
    for(int i=0;i<100;++i) { now+=5ms;feed(tilt(0)); }
    require(g.ready(), "recovery interval not admitted");
    now+=5ms;feed(tilt(0),false);require(!g.ready(), "incomplete getup admitted");
    for (auto q : {std::vector<float>{0,0,0,0}, std::vector<float>{1,0,0},
                   std::vector<float>{NAN,0,0,0},std::vector<float>{INFINITY,0,0,0}}) {
        now+=5ms;feed(q);require(!g.ready(), "invalid quaternion accepted");
    }
    g.observe(tilt(0),now+1ms,now,true,8,.5,.02);
    require(!g.ready(), "future sample accepted");
}

void testEntryTransitions()
{
    using namespace std::chrono_literals;
    for (const std::string kind : {"Leg","Wheel"}) {
        TestRL rl;
        rl.lw_policy_entry_guard_enabled = true;
        rl.SetLWBaseRuntimeConfiguration(LWValidatedBaseConfiguration(
            YAML::LoadFile(std::string(POLICY_DIR)+"/LW/base.yaml")["LW"], "entry-test"));
        rl.InitJointNum(10);
        std::unordered_set<std::string> names;
        auto states=createLWStates(rl,names);
        const std::string getup="RLFSMStateGetUp_"+kind;
        const std::string loco="RLFSMStateRLLocomotion_"+kind;
        auto current=states.at(getup);
        for (const auto& pair:states) rl.fsm.AddState(pair.second);
        rl.fsm.previous_state_=states.at("RLFSMStatePassive");
        rl.fsm.SetInitialState(getup);
        if(kind=="Leg") std::dynamic_pointer_cast<LW_fsm::RLFSMStateGetUp_Leg>(current)->percent_pre_getup=1;
        else std::dynamic_pointer_cast<LW_fsm::RLFSMStateGetUp_Wheel>(current)->percent_pre_getup=1;
        setReadyProgress(rl,current);
        const auto key=kind=="Leg"?Input::Keyboard::Num1:Input::Keyboard::Num3;
        rl.control.current_keyboard=key;
        require(current->CheckChange()==loco,"ready entry rejected");
        // A spike arrives after the request but before next-cycle Enter.
        rl.fsm.RequestStateChange(loco);
        rl.entry_now+=5ms;rl.robot_state.imu.sample_time=rl.entry_now;
        rl.robot_state.imu.quaternion=tilt(13);
        rl.fsm.Run();
        require(rl.fsm.current_state_==current && rl.fsm.mode_==FSM::Mode::NORMAL,
                "staged transition bypassed new tilt");
        require(rl.control.current_keyboard==Input::Keyboard::None,
                "rejected keyboard request retained");
        clearInput(rl);
        rl.control.current_gamepad=Input::Gamepad::B;
        require(current->CheckChange()=="RLFSMStateGetDown","entry gate blocked getdown");
        clearInput(rl);rl.control.current_keyboard=Input::Keyboard::P;
        require(current->CheckChange()=="RLFSMStatePassive","entry gate blocked passive");
        clearInput(rl);
        std::ostringstream captured;auto* previous=std::cout.rdbuf(captured.rdbuf());
        for(int i=0;i<200;++i) {
            rl.entry_now+=5ms;rl.robot_state.imu.sample_time=rl.entry_now;
            current->CheckChange();
        }
        std::cout.rdbuf(previous);
        size_t count=0,pos=0;
        while((pos=captured.str().find("[PolicyEntry]",pos))!=std::string::npos) {++count;pos+=13;}
        require(count==2,"warning rate is not 2Hz while rejection persists");
        rl.robot_state.imu.quaternion=tilt(0);
        setReadyProgress(rl,current);
        require(current->CheckChange()==getup,"automatically started on recovery");
        rl.control.current_gamepad=kind=="Leg"?Input::Gamepad::RB_DPadUp:Input::Gamepad::RB_DPadDown;
        require(current->CheckChange()==loco,"fresh gamepad request rejected after recovery");
        require(current->CanTransitionTo(loco),"ready staged transition rejected");
        // The simulator does not opt in: even a tilted/no-timestamp state
        // must retain its original transition behavior.
        rl.lw_policy_entry_guard_enabled=false;
        rl.robot_state.imu.quaternion=tilt(13);
        rl.robot_state.imu.sample_time={};
        rl.control.current_keyboard=key;
        require(current->CheckChange()==loco && current->CanTransitionTo(loco),
                "real-only guard changed simulation transitions");
        rl.lw_policy_entry_guard_enabled=true;
        // Re-entering GetUp must discard prior stability.
        rl.fsm.previous_state_=states.at("RLFSMStatePassive");current->Enter();
        require(!current->CanTransitionTo(loco),"re-entry reused old admission");
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
        testEntryGuardSamples();
        testEntryTransitions();
        testCoreRejectsUnregisteredTargets();
        testMorphologyEnterUsesStartupPreload();
        testMissingMotionFailsDuringPreload();

        TestRL rl;
        testPolicyRootResolution(rl);
        rl.SetLWBaseRuntimeConfiguration(LWValidatedBaseConfiguration(
            YAML::LoadFile(std::string(POLICY_DIR) + "/LW/base.yaml")["LW"], "entry-test"));
        rl.InitJointNum(10);
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
