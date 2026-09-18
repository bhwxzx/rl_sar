#ifndef LW_FSM_HPP
#define LW_FSM_HPP

#include "fsm.hpp"
#include "rl_sdk.hpp"
#include "lw_policy_entry_guard.hpp"

namespace LW_fsm
{

inline float GetLWMotionSourceFPS(
    const YamlParams& policy_params)
{
    return policy_params.Get<float>("motion_fps");
}

inline float GetLWMotionSourceFPS(
    const LWPolicyRuntimeConfiguration& policy_configuration) noexcept
{
    return policy_configuration.motion_fps;
}

class RLFSMStatePassive : public RLFSMState
{
public:
    RLFSMStatePassive(RL *rl) : RLFSMState(*rl, "RLFSMStatePassive") {}

    void Enter() override
    {
        rl.PublishLWOperatorStatus(LWOperatorMode::Passive);
    }

    void Run() override
    {
        const std::size_t num_dofs =
            rl.GetLWBaseRuntimeConfiguration().num_dofs;
        for (std::size_t i = 0; i < num_dofs; ++i)
        {
            // fsm_command->motor_command.q[i] = fsm_state->motor_state.q[i];
            fsm_command->motor_command.dq[i] = 0;
            fsm_command->motor_command.kp[i] = 0;
            fsm_command->motor_command.kd[i] = 5;
            fsm_command->motor_command.tau[i] = 0;
        }
    }

    void Exit() override {}

    std::string_view CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp_Leg";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num2 || rl.control.current_gamepad == Input::Gamepad::Y)
        {
            return "RLFSMStateGetUp_Wheel";
        }
        return state_name_;
    }
};

// Shared by the two GetUp states; never activates a policy on recovery.
class LWGetUpEntryCheck
{
public:
    LWPolicyEntryGuard guard;
    void reset() { guard.reset(); warning_active_ = false; last_warning_ = {}; }
    void update(RL& rl, const RobotState<float>& state, bool completed)
    {
        if (!rl.lw_policy_entry_guard_enabled) { reset(); return; }
        const auto& c = rl.GetLWBaseRuntimeConfiguration();
        guard.observe(state.imu.quaternion, state.imu.sample_time,
                      rl.LWEntryCheckNow(), completed, c.policy_entry_angle_deg,
                      c.policy_entry_stable_time, c.trusted_imu_timeout);
        if (warning_active_ && guard.ready()) {
            std::cout << LOGGER::INFO << "[PolicyEntry] 已满足启动条件，请重新按键启动策略" << std::endl;
            warning_active_ = false;
        }
        if (warning_active_) warn(rl);
    }
    bool request(RL& rl, const char* policy)
    {
        if (!rl.lw_policy_entry_guard_enabled || guard.ready()) return true;
        policy_ = policy;
        warning_active_ = true;
        // Discard the rejected request, including the keyboard's remembered key.
        rl.control.current_keyboard = rl.control.last_keyboard = Input::Keyboard::None;
        rl.control.current_gamepad = rl.control.last_gamepad = Input::Gamepad::None;
        warn(rl);
        return false;
    }
private:
    void warn(RL& rl)
    {
        const auto now = rl.LWEntryCheckNow();
        if (last_warning_ != LWPolicyEntryGuard::Time{}
            && now-last_warning_ < std::chrono::milliseconds(500)) return;
        last_warning_ = now;
        const auto& c = rl.GetLWBaseRuntimeConfiguration();
        std::cout << LOGGER::WARNING << "[PolicyEntry] 拒绝启动 " << policy_
            << ": " << guard.description() << ", roll=" << guard.roll
            << " deg, pitch=" << guard.pitch << " deg, 要求各轴绝对值<="
            << c.policy_entry_angle_deg << " deg, 连续稳定=" << guard.stable_seconds
            << "/" << c.policy_entry_stable_time << " s；保持GetUp" << std::endl;
    }
    bool warning_active_ = false;
    const char* policy_ = "locomotion";
    LWPolicyEntryGuard::Time last_warning_{};
};

class RLFSMStateGetUp_Leg : public RLFSMState
{
public:
    RLFSMStateGetUp_Leg(RL *rl) : RLFSMState(*rl, "RLFSMStateGetUp_Leg") {}

    float percent_pre_getup = 0.0f;
    float percent_getup = 0.0f;
    std::vector<float> pre_running_pos = {
        0.0, 0.0,
        0.0, 0.0,
        -1.178, 1.178,
        0.00, 0.00,
        0.0, 0.0
    };
    bool stand_from_passive = true;
    LWGetUpEntryCheck entry_check;

    void Enter() override
    {
        entry_check.reset();
        percent_pre_getup = 0.0f;
        percent_getup = 0.0f;
        if (rl.fsm.previous_state_->GetStateName() == "RLFSMStatePassive")
        {
            stand_from_passive = true;
        }
        else
        {
            stand_from_passive = false;
        }
        rl.now_state = *fsm_state;
        rl.start_state = rl.now_state;
    }

    void Run() override
    {
        if(stand_from_passive)
        {

            if (Interpolate(percent_pre_getup, rl.now_state.motor_state.q, pre_running_pos, 2.0f, "Pre Getting up", true, LWOperatorMode::GetUpLeg)) return;
            // 这里的params是从base.yaml中读取的
            if (Interpolate(percent_getup, pre_running_pos, rl.GetLWBaseRuntimeConfiguration().default_dof_pos_leg, 2.0f, "Getting up", true, LWOperatorMode::GetUpLeg)) return;
        }
        else
        {
            if (Interpolate(percent_getup, rl.now_state.motor_state.q, rl.GetLWBaseRuntimeConfiguration().default_dof_pos_leg, 3.0f, "Getting up", true, LWOperatorMode::GetUpLeg)) return;
        }
    }

    void Exit() override {}

    bool CanTransitionTo(std::string_view target) override
    {
        if (target != "RLFSMStateRLLocomotion_Leg") return true;
        if (rl.lw_policy_entry_guard_enabled)
            entry_check.update(rl, *fsm_state, percent_getup >= 1.0f);
        return entry_check.request(rl, "leg_loco");
    }

    std::string_view CheckChange() override
    {
        if (rl.lw_policy_entry_guard_enabled)
            entry_check.update(rl, *fsm_state, percent_getup >= 1.0f);
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        if (rl.control.current_keyboard == Input::Keyboard::Num2 || rl.control.current_gamepad == Input::Gamepad::Y)
        {
            return "RLFSMStateGetUp_Wheel";
        }
        if (percent_getup >= 1.0f)
        {
            if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
            {
                if (entry_check.request(rl, "leg_loco"))
                    return "RLFSMStateRLLocomotion_Leg";
                return state_name_;
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
            {
                return "RLFSMStateGetDown";
            }
        }
        return state_name_;
    }
};

class RLFSMStateGetUp_Wheel : public RLFSMState
{
public:
    RLFSMStateGetUp_Wheel(RL *rl) : RLFSMState(*rl, "RLFSMStateGetUp_Wheel") {}

    float percent_pre_getup = 0.0f;
    float percent_getup = 0.0f;
    std::vector<float> pre_running_pos = {
        0.0, 0.0,
        0.0, 0.0,
        0.0, 0.0,
        0.00, 0.00,
        0.0, 0.0
    };
    bool stand_from_passive = true;
    LWGetUpEntryCheck entry_check;

    void Enter() override
    {
        entry_check.reset();
        percent_pre_getup = 0.0f;
        percent_getup = 0.0f;
        if (rl.fsm.previous_state_->GetStateName() == "RLFSMStatePassive")
        {
            stand_from_passive = true;
        }
        else
        {
            stand_from_passive = false;
        }
        rl.now_state = *fsm_state;
        rl.start_state = rl.now_state;
    }

    void Run() override
    {
        if(stand_from_passive)
        {

            if (Interpolate(percent_pre_getup, rl.now_state.motor_state.q, pre_running_pos, 2.0f, "Pre Getting up", true, LWOperatorMode::GetUpWheel)) return;
            if (Interpolate(percent_getup, pre_running_pos, rl.GetLWBaseRuntimeConfiguration().default_dof_pos_wheel, 2.0f, "Getting up", true, LWOperatorMode::GetUpWheel)) return;
        }
        else
        {
            if (Interpolate(percent_getup, rl.now_state.motor_state.q, rl.GetLWBaseRuntimeConfiguration().default_dof_pos_wheel, 3.0f, "Getting up", true, LWOperatorMode::GetUpWheel)) return;
        }
    }

    void Exit() override {}

    bool CanTransitionTo(std::string_view target) override
    {
        if (target != "RLFSMStateRLLocomotion_Wheel") return true;
        if (rl.lw_policy_entry_guard_enabled)
            entry_check.update(rl, *fsm_state, percent_getup >= 1.0f);
        return entry_check.request(rl, "wheel_loco");
    }

    std::string_view CheckChange() override
    {
        if (rl.lw_policy_entry_guard_enabled)
            entry_check.update(rl, *fsm_state, percent_getup >= 1.0f);
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp_Leg";
        }
        if (percent_getup >= 1.0f)
        {
            if (rl.control.current_keyboard == Input::Keyboard::Num3 || rl.control.current_gamepad == Input::Gamepad::RB_DPadDown)
            {
                if (entry_check.request(rl, "wheel_loco"))
                    return "RLFSMStateRLLocomotion_Wheel";
                return state_name_;
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
            {
                return "RLFSMStateGetDown";
            }
        }
        return state_name_;
    }
};

class RLFSMStateGetDown : public RLFSMState
{
public:
    RLFSMStateGetDown(RL *rl) : RLFSMState(*rl, "RLFSMStateGetDown") {}

    float percent_getdown = 0.0f;
    std::vector<float> pre_running_pos = {
        0.0, 0.0,
        0.0, 0.0,
        -1.178, 1.178,
        0.00, 0.00,
        0.0, 0.0
    };

    void Enter() override
    {
        percent_getdown = 0.0f;
        rl.now_state = *fsm_state;
    }

    void Run() override
    {
        // Interpolate(percent_getdown, rl.now_state.motor_state.q, rl.start_state.motor_state.q, 3.0f, "Getting down", true);
        Interpolate(percent_getdown, rl.now_state.motor_state.q, pre_running_pos, 3.0f, "Getting down", true, LWOperatorMode::GetDown);
    }

    void Exit() override {}

    std::string_view CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X || percent_getdown >= 1.0f)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp_Leg";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num2 || rl.control.current_gamepad == Input::Gamepad::Y)
        {
            return "RLFSMStateGetUp_Wheel";
        }
        return state_name_;
    }
};

class RLFSMStateRLLocomotion_Leg : public RLFSMState
{
public:
    RLFSMStateRLLocomotion_Leg(RL *rl) : RLFSMState(*rl, "RLFSMStateRLLocomotion_Leg") {}

    float percent_transition = 0.0f;
    int print_count = 0;

    void Enter() override
    {
        percent_transition = 0.0f;
        // read params from yaml
        rl.config_name = "robot_lab";
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name + "/leg_loco";
        try
        {
            if (!rl.GetLWPolicyDefinition(robot_config_path))
            {
                throw std::runtime_error(
                    "policy context was not preloaded: "
                    + robot_config_path);
            }
            rl.InitControl();
            rl.ActivateLWPolicy(robot_config_path);
            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "Policy activation failed: " << e.what() << std::endl;
            rl.DeactivateLWPolicy();
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        // position transition from last default_dof_pos to current default_dof_pos
        // if (Interpolate(percent_transition, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 0.5f, "Policy transition", true)) return;

        if (print_count++ % 20 == 0) // 10Hz status publication
        {
            rl.PublishLWOperatorStatus(LWOperatorMode::LegLocomotion);
        }
        RLControlLW();
    }

    void Exit() override
    {
        rl.DeactivateLWPolicy();
    }

    std::string_view CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRLLocomotion_Leg";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num4 || rl.control.current_gamepad == Input::Gamepad::RB_DPadLeft)
        {
            return "RLFSMStateRL_LegToWheel";
        }
        return state_name_;
    }
};

class RLFSMStateRLLocomotion_Wheel : public RLFSMState
{
public:
    RLFSMStateRLLocomotion_Wheel(RL *rl) : RLFSMState(*rl, "RLFSMStateRLLocomotion_Wheel") {}

    float percent_transition = 0.0f;
    int print_count = 0;

    void Enter() override
    {
        percent_transition = 0.0f;
        // read params from yaml
        rl.config_name = "robot_lab";
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name + "/wheel_loco";
        try
        {
            if (!rl.GetLWPolicyDefinition(robot_config_path))
            {
                throw std::runtime_error(
                    "policy context was not preloaded: "
                    + robot_config_path);
            }
            rl.InitControl();
            rl.ActivateLWPolicy(robot_config_path);
            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "Policy activation failed: " << e.what() << std::endl;
            rl.DeactivateLWPolicy();
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        // position transition from last default_dof_pos to current default_dof_pos
        // if (Interpolate(percent_transition, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 0.5f, "Policy transition", true)) return;

        if (print_count++ % 20 == 0) // 10Hz status publication
        {
            rl.PublishLWOperatorStatus(LWOperatorMode::WheelLocomotion);
        }
        RLControlLW();
    }

    void Exit() override
    {
        rl.DeactivateLWPolicy();
    }

    std::string_view CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num3 || rl.control.current_gamepad == Input::Gamepad::RB_DPadDown)
        {
            return "RLFSMStateRLLocomotion_Wheel";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num5 || rl.control.current_gamepad == Input::Gamepad::RB_DPadRight)
        {
            return "RLFSMStateRL_WheelToLeg";
        }
        return state_name_;
    }
};

class RLFSMStateRL_LegToWheel : public RLFSMState
{
public:
    RLFSMStateRL_LegToWheel(RL *rl) : RLFSMState(*rl, "RLFSMStateRL_LegToWheel") {}
    std::uint64_t policy_generation = 0;
    std::uint64_t policy_frame = 0;

    void Enter() override
    {
        // read params from yaml
        rl.config_name = "robot_lab/leg_to_wheel";
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            const auto definition =
                rl.GetLWPolicyDefinition(robot_config_path);
            if (!definition)
            {
                throw std::runtime_error(
                    "policy context was not preloaded: "
                    + robot_config_path);
            }

            rl.motion_loader_lw =
                rl.GetPreloadedLWMotionPlayer(robot_config_path);
            if (!rl.motion_loader_lw
                || !definition->prepared_motion)
            {
                throw std::runtime_error(
                    "motion context was not preloaded: "
                    + robot_config_path);
            }
            rl.motion_length = rl.motion_loader_lw->GetDuration();

            rl.motion_loader_lw->Reset(fsm_state->imu.quaternion);
            rl.InitControl();
            policy_generation =
                rl.ActivateLWPolicy(
                    robot_config_path,
                    rl.motion_length);
            policy_frame = 0;
            rl.PublishCurrentLWMotionReference(policy_generation);

            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "Policy activation failed: " << e.what() << std::endl;
            rl.DeactivateLWPolicy();
            rl.motion_loader_lw = nullptr;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        // position transition from last default_dof_pos to current default_dof_pos
        // if (Interpolate(percent_transition, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 0.5f, "Policy transition", true)) return;

        // Calculate motion time and progress
        const auto activation = rl.LoadLWPolicyActivation();
        const auto progress = rl.LoadLWPolicyProgress();
        if (!activation
            || activation->generation != policy_generation)
        {
            return;
        }
        if (progress
            && progress->generation == policy_generation
            && progress->frame >= policy_frame)
        {
            policy_frame = progress->frame;
        }
        const auto& policy_configuration =
            activation->definition->runtime;
        float motion_time =
            policy_frame
            * policy_configuration.period_seconds;
        motion_time = std::fmin(motion_time, rl.motion_length);
        float percent = motion_time / rl.motion_length;
        rl.PublishLWOperatorStatus(LWOperatorMode::LegToWheel, percent);

        rl.motion_loader_lw->Update(motion_time);
        rl.PublishCurrentLWMotionReference(policy_generation);

        RLControlLW();

        if (motion_time >= rl.motion_length)
        {
            rl.fsm.RequestStateChange("RLFSMStateRLLocomotion_Wheel");
        }
    }

    void Exit() override
    {
        rl.DeactivateLWPolicy();
        rl.motion_loader_lw = nullptr;
    }

    std::string_view CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        return state_name_;
    }
};

class RLFSMStateRL_WheelToLeg : public RLFSMState
{
public:
    RLFSMStateRL_WheelToLeg(RL *rl) : RLFSMState(*rl, "RLFSMStateRL_WheelToLeg") {}
    std::uint64_t policy_generation = 0;
    std::uint64_t policy_frame = 0;

    void Enter() override
    {
        // read params from yaml
        rl.config_name = "robot_lab/wheel_to_leg";
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            const auto definition =
                rl.GetLWPolicyDefinition(robot_config_path);
            if (!definition)
            {
                throw std::runtime_error(
                    "policy context was not preloaded: "
                    + robot_config_path);
            }

            rl.motion_loader_lw =
                rl.GetPreloadedLWMotionPlayer(robot_config_path);
            if (!rl.motion_loader_lw
                || !definition->prepared_motion)
            {
                throw std::runtime_error(
                    "motion context was not preloaded: "
                    + robot_config_path);
            }
            rl.motion_length = rl.motion_loader_lw->GetDuration();

            rl.motion_loader_lw->Reset(fsm_state->imu.quaternion);
            rl.InitControl();
            policy_generation =
                rl.ActivateLWPolicy(
                    robot_config_path,
                    rl.motion_length);
            policy_frame = 0;
            rl.PublishCurrentLWMotionReference(policy_generation);

            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "Policy activation failed: " << e.what() << std::endl;
            rl.DeactivateLWPolicy();
            rl.motion_loader_lw = nullptr;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        // position transition from last default_dof_pos to current default_dof_pos
        // if (Interpolate(percent_transition, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 0.5f, "Policy transition", true)) return;

        // Calculate motion time and progress
        const auto activation = rl.LoadLWPolicyActivation();
        const auto progress = rl.LoadLWPolicyProgress();
        if (!activation
            || activation->generation != policy_generation)
        {
            return;
        }
        if (progress
            && progress->generation == policy_generation
            && progress->frame >= policy_frame)
        {
            policy_frame = progress->frame;
        }
        const auto& policy_configuration =
            activation->definition->runtime;
        float motion_time =
            policy_frame
            * policy_configuration.period_seconds;
        motion_time = std::fmin(motion_time, rl.motion_length);
        float percent = motion_time / rl.motion_length;
        rl.PublishLWOperatorStatus(LWOperatorMode::WheelToLeg, percent);

        rl.motion_loader_lw->Update(motion_time);
        rl.PublishCurrentLWMotionReference(policy_generation);

        RLControlLW();

        if (motion_time >= rl.motion_length)
        {
            rl.fsm.RequestStateChange("RLFSMStateRLLocomotion_Leg");
        }
    }

    void Exit() override
    {
        rl.DeactivateLWPolicy();
        rl.motion_loader_lw = nullptr;
    }

    std::string_view CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        return state_name_;
    }
};

} // namespace LW_fsm

class LWFSMFactory : public FSMFactory
{
public:
    LWFSMFactory(const std::string& initial) : initial_state_(initial) {}
    std::shared_ptr<FSMState> CreateState(void *context, const std::string &state_name) override
    {
        RL *rl = static_cast<RL *>(context);
        if (state_name == "RLFSMStatePassive")
            return std::make_shared<LW_fsm::RLFSMStatePassive>(rl);
        else if (state_name == "RLFSMStateGetUp_Leg")
            return std::make_shared<LW_fsm::RLFSMStateGetUp_Leg>(rl);
        else if (state_name == "RLFSMStateGetUp_Wheel")
            return std::make_shared<LW_fsm::RLFSMStateGetUp_Wheel>(rl);
        else if (state_name == "RLFSMStateGetDown")
            return std::make_shared<LW_fsm::RLFSMStateGetDown>(rl);
        else if (state_name == "RLFSMStateRLLocomotion_Leg")
            return std::make_shared<LW_fsm::RLFSMStateRLLocomotion_Leg>(rl);
        else if (state_name == "RLFSMStateRLLocomotion_Wheel")
            return std::make_shared<LW_fsm::RLFSMStateRLLocomotion_Wheel>(rl);
        else if (state_name == "RLFSMStateRL_LegToWheel")
            return std::make_shared<LW_fsm::RLFSMStateRL_LegToWheel>(rl);
        else if (state_name == "RLFSMStateRL_WheelToLeg")
            return std::make_shared<LW_fsm::RLFSMStateRL_WheelToLeg>(rl);
        return nullptr;
    }
    std::string GetType() const override { return "LW"; }
    std::vector<std::string> GetSupportedStates() const override
    {
        return {
            "RLFSMStatePassive",
            "RLFSMStateGetUp_Leg",
            "RLFSMStateGetUp_Wheel",
            "RLFSMStateGetDown",
            "RLFSMStateRLLocomotion_Leg",
            "RLFSMStateRLLocomotion_Wheel",
            "RLFSMStateRL_LegToWheel",
            "RLFSMStateRL_WheelToLeg"
        };
    }
    std::string GetInitialState() const override { return initial_state_; }
private:
    std::string initial_state_;
};

REGISTER_FSM_FACTORY(LWFSMFactory, "RLFSMStatePassive")

#endif // LW_FSM_HPP
