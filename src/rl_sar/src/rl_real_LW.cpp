
#include "rl_real_LW.hpp"

using namespace std::chrono_literals;

RL_Real::RL_Real(int argc, char **argv)
{
    ros2_node = std::make_shared<rclcpp::Node>("rl_real_LW_node");
    this->imu_subscriber_ = ros2_node->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", rclcpp::SystemDefaultsQoS(),
        [this] (const sensor_msgs::msg::Imu::SharedPtr imu_msg) {this->ImuCallback(imu_msg);}
    );

    this->SetupSysJoystick("/dev/input/js0", 16);

    // read params from yaml
    this->ang_vel_axis = "body";
    this->robot_name = "LW";
    this->ReadYaml(this->robot_name, "base.yaml");

    // auto load FSM by robot_name
    if (FSMManager::GetInstance().IsTypeSupported(this->robot_name))
    {
        auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
        if (fsm_ptr)
        {
            this->fsm = *fsm_ptr;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "[FSM] No FSM registered for robot: " << this->robot_name << std::endl;
    }

    // init robot
    this->lw_sdk.InitSerial("/dev/ttyACM0");
    this->lw_sdk.InitCmdData(this->lw_low_command);
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    this->InitOutputs();
    this->InitControl();
    this->control.gait_frequency = this->params.Get<std::vector<float>>("gait_command")[0];
    this->gait_phase_time = 0.0f;

    // loop
    this->loop_joystick = std::make_shared<LoopFunc>("loop_joystick", 0.01, std::bind(&RL_Real::GetSysJoystick, this));
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.Get<float>("dt"), std::bind(&RL_Real::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.Get<float>("dt") * this->params.Get<int>("decimation"), std::bind(&RL_Real::RunModel, this));
    this->loop_joystick->start();
    this->loop_control->start();
    this->loop_rl->start();

#ifdef PLOT
    this->jointstate_plot_publisher_ = ros2_node->create_publisher<sensor_msgs::msg::JointState>(
        "/LW_joint_states", rclcpp::SystemDefaultsQoS()
    );
    this->timer_ = ros2_node->create_wall_timer(
        2ms, std::bind(&RL_Real::jointstate_plot_callback, this)
    );
#endif
#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif

}

RL_Real::~RL_Real()
{
    this->loop_joystick->shutdown();
    this->loop_rl->shutdown();
    this->loop_control->shutdown();
    disable_lw_robot();
    std::cout << LOGGER::INFO << "RL_Real exit" << std::endl;
}

void RL_Real::jointstate_plot_callback(void)
{
    sensor_msgs::msg::JointState msg;
    // 关节顺序与base.yaml里一致
    std::vector<std::string> joint_now_names = {
        "right_hip_now", "left_hip_now",
        "right_thigh_now", "left_thigh_now",
        "right_shank_now", "left_shank_now",
        "right_foot_now", "left_foot_now",
        "right_wheel_now", "left_wheel_now"
    };
    std::vector<std::string> joint_target_names = {
        "right_hip_target", "left_hip_target",
        "right_thigh_target", "left_thigh_target",
        "right_shank_target", "left_shank_target",
        "right_foot_target", "left_foot_target",
        "right_wheel_target", "left_wheel_target"
    };
    std::vector<std::string> joint_names;
    joint_names.reserve(joint_now_names.size() + joint_target_names.size());
    joint_names.insert(joint_names.end(), joint_now_names.begin(), joint_now_names.end());
    joint_names.insert(joint_names.end(), joint_target_names.begin(), joint_target_names.end());

    size_t total_size = joint_names.size();
    msg.name.resize(total_size); 
    msg.position.resize(total_size);
    msg.velocity.resize(total_size);
    msg.effort.resize(total_size);
    msg.name = joint_names;

    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        msg.position[i] = this->lw_low_state.motorState[this->params.Get<std::vector<int>>("joint_mapping")[i]].pos_now;
        msg.velocity[i] = this->lw_low_state.motorState[this->params.Get<std::vector<int>>("joint_mapping")[i]].vel_now;
        msg.effort[i] = this->lw_low_state.motorState[this->params.Get<std::vector<int>>("joint_mapping")[i]].tau_now;
    }
    for (int i = this->params.Get<int>("num_of_dofs"); i < 2*this->params.Get<int>("num_of_dofs"); ++i)
    {
        msg.position[i] = this->lw_low_command.motorCmd[this->params.Get<std::vector<int>>("joint_mapping")[i]].action_set;
    }
    for (int i : this->params.Get<std::vector<int>>("wheel_indices"))
    {
        msg.position[i + this->params.Get<int>("num_of_dofs")] = 0.0f;
        msg.velocity[i + this->params.Get<int>("num_of_dofs")] = this->lw_low_command.motorCmd[this->params.Get<std::vector<int>>("joint_mapping")[i]].action_set;
    }

    this->jointstate_plot_publisher_->publish(msg);

}

void RL_Real::disable_lw_robot(void)
{
    LowCmd cmd;
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); i++)
    {
        cmd.motorCmd[i].action_set = 0.0f;
        cmd.motorCmd[i].Kp = 0.0f;
        cmd.motorCmd[i].Kd = 0.0f;
    }
    cmd.motors_disable = true;
    this->lw_sdk.SendCmdData(cmd);
}

void RL_Real::RobotControl()
{
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        this->GetState(&this->robot_state);
    } 

    this->StateController(&this->robot_state, &this->robot_command);

    this->control.ClearInput();

    this->SetCommand(&this->robot_command);
}

void RL_Real::RunModel()
{
    if (this->rl_init_done)
    {
        RobotState<float> local_state; 

        {
            std::lock_guard<std::mutex> lock(state_mutex);
            local_state = this->robot_state;
        }
        this->episode_length_buf += 1;
        this->obs.ang_vel = local_state.imu.gyroscope;
        this->obs.commands = {this->control.x, this->control.y, this->control.yaw};
        this->obs.base_quat = local_state.imu.quaternion;
        this->obs.dof_pos = local_state.motor_state.q;
        this->obs.dof_vel = local_state.motor_state.dq;

        this->gait_phase_time += this->params.Get<float>("dt") * this->params.Get<int>("decimation") * this->control.gait_frequency;
        if (this->gait_phase_time >= 1.0f)
        {
            this->gait_phase_time -= 1.0f; 
        }
        this->obs.gait_phase = {std::sin(2 * static_cast<float>(M_PI) * (this->gait_phase_time)), 
                                std::cos(2 * static_cast<float>(M_PI) * (this->gait_phase_time))};
            
        this->obs.gait_command = {this->control.gait_frequency, 
                                  this->params.Get<std::vector<float>>("gait_command")[1], 
                                  this->params.Get<std::vector<float>>("gait_command")[2],
                                  this->params.Get<std::vector<float>>("gait_command")[3]};

        this->obs.actions = this->Forward();
        this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);

        if (!this->output_dof_pos.empty())
        {
            output_dof_pos_queue.push(this->output_dof_pos);
        }
        if (!this->output_dof_vel.empty())
        {
            output_dof_vel_queue.push(this->output_dof_vel);
        }
        if (!this->output_dof_tau.empty())
        {
            output_dof_tau_queue.push(this->output_dof_tau);
        }

        this->TorqueProtect(this->output_dof_tau);
        this->AttitudeProtect(this->robot_state.imu.quaternion, 75.0f, 75.0f);

#ifdef CSV_LOGGER
        std::vector<float> tau_est = this->robot_state.motor_state.tau_est;
        this->CSVLogger(this->output_dof_tau, tau_est, this->obs.dof_pos, this->output_dof_pos, this->obs.dof_vel);
#endif
    }
}

std::vector<float> RL_Real::Forward()
{
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);

    // If model is being reinitialized, return previous actions to avoid blocking
    if (!lock.owns_lock())
    {
        std::cout << LOGGER::WARNING << "Model is being reinitialized, using previous actions" << std::endl;
        return this->obs.actions;
    }

    std::vector<float> clamped_obs = this->ComputeObservation();

    std::vector<float> actions;
    if (!this->params.Get<std::vector<int>>("observations_history").empty())
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.Get<std::vector<int>>("observations_history"));
        actions = this->model->forward({this->history_obs});
    }
    else
    {
        actions = this->model->forward({clamped_obs});
    }

    if (!this->params.Get<std::vector<float>>("clip_actions_upper").empty() && !this->params.Get<std::vector<float>>("clip_actions_lower").empty())
    {
        return clamp(actions, this->params.Get<std::vector<float>>("clip_actions_lower"), this->params.Get<std::vector<float>>("clip_actions_upper"));
    }
    else
    {
        return actions;
    }
}

void RL_Real::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex);
    this->imu = *msg;
}

void RL_Real::GetState(RobotState<float> *state)
{
    state->imu.quaternion[0] = imu.orientation.w;
    state->imu.quaternion[1] = imu.orientation.x;
    state->imu.quaternion[2] = imu.orientation.y;
    state->imu.quaternion[3] = imu.orientation.z;

    state->imu.gyroscope[0] = imu.angular_velocity.x;
    state->imu.gyroscope[1] = imu.angular_velocity.y;
    state->imu.gyroscope[2] = imu.angular_velocity.z;

    if (this->lw_sdk.RecvFdData(this->lw_low_state))
    {
        for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
        {
            state->motor_state.q[i] = this->lw_low_state.motorState[this->params.Get<std::vector<int>>("joint_mapping")[i]].pos_now;
            state->motor_state.dq[i] = this->lw_low_state.motorState[this->params.Get<std::vector<int>>("joint_mapping")[i]].vel_now;
            state->motor_state.tau_est[i] = this->lw_low_state.motorState[this->params.Get<std::vector<int>>("joint_mapping")[i]].tau_now;
        }
    }

}

void RL_Real::SetCommand(const RobotCommand<float> *command)
{
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        this->lw_low_command.motorCmd[this->params.Get<std::vector<int>>("joint_mapping")[i]].action_set = command->motor_command.q[i];
        this->lw_low_command.motorCmd[this->params.Get<std::vector<int>>("joint_mapping")[i]].Kp = command->motor_command.kp[i];
        this->lw_low_command.motorCmd[this->params.Get<std::vector<int>>("joint_mapping")[i]].Kd = command->motor_command.kd[i];
    }
    for (int i : this->params.Get<std::vector<int>>("wheel_indices"))
    {
        this->lw_low_command.motorCmd[this->params.Get<std::vector<int>>("joint_mapping")[i]].action_set = command->motor_command.dq[i];
    }
    this->lw_low_command.motors_disable = false;

    this->lw_sdk.SendCmdData(this->lw_low_command);
}

void RL_Real::SetupSysJoystick(const std::string& device, int bits)
{
    this->sys_js = std::make_unique<Joystick>(device);
    if (!this->sys_js->isFound())
    {
        std::cout << LOGGER::ERROR << "Joystick [" << device << "] open failed." << std::endl;
        // exit(1);
    }

    this->sys_js_max_value = (1 << (bits - 1));
}

void RL_Real::GetSysJoystick()
{
    // Clear all button event states
    for (int i = 0; i < 20; ++i)
    {
        this->sys_js_button[i].on_press = false;
        this->sys_js_button[i].on_release = false;
    }

    // Check if joystick is valid before using
    if (!this->sys_js)
    {
        return;
    }

    while (this->sys_js->sample(&this->sys_js_event))
    {
        if (this->sys_js_event.isButton())
        {
            this->sys_js_button[this->sys_js_event.number].update(this->sys_js_event.value);
        }
        else if (this->sys_js_event.isAxis())
        {
            double normalized = double(this->sys_js_event.value) / this->sys_js_max_value;
            if (std::abs(normalized) < this->axis_deadzone)
            {
                this->sys_js_axis[this->sys_js_event.number] = 0;
            }
            else
            {
                this->sys_js_axis[this->sys_js_event.number] = this->sys_js_event.value;
            }
        }
    }

    // 修改为自己的手柄映射
    if (this->sys_js_button[0].on_press) this->control.SetGamepad(Input::Gamepad::A);
    if (this->sys_js_button[1].on_press) this->control.SetGamepad(Input::Gamepad::B);
    if (this->sys_js_button[2].on_press) this->control.SetGamepad(Input::Gamepad::X);
    if (this->sys_js_button[3].on_press) this->control.SetGamepad(Input::Gamepad::Y);
    if (this->sys_js_button[4].on_press) this->control.SetGamepad(Input::Gamepad::LB);
    if (this->sys_js_button[5].on_press) this->control.SetGamepad(Input::Gamepad::RB);
    if (this->sys_js_button[9].on_press) this->control.SetGamepad(Input::Gamepad::LStick);
    if (this->sys_js_button[10].on_press) this->control.SetGamepad(Input::Gamepad::RStick);
    if (this->sys_js_axis[7] < 0) this->control.SetGamepad(Input::Gamepad::DPadUp);
    if (this->sys_js_axis[7] > 0) this->control.SetGamepad(Input::Gamepad::DPadDown);
    if (this->sys_js_axis[6] < 0) this->control.SetGamepad(Input::Gamepad::DPadLeft);
    if (this->sys_js_axis[6] > 0) this->control.SetGamepad(Input::Gamepad::DPadRight);
    if (this->sys_js_button[4].pressed && this->sys_js_button[0].on_press) this->control.SetGamepad(Input::Gamepad::LB_A);
    if (this->sys_js_button[4].pressed && this->sys_js_button[1].on_press) this->control.SetGamepad(Input::Gamepad::LB_B);
    if (this->sys_js_button[4].pressed && this->sys_js_button[2].on_press) this->control.SetGamepad(Input::Gamepad::LB_X);
    if (this->sys_js_button[4].pressed && this->sys_js_button[3].on_press) this->control.SetGamepad(Input::Gamepad::LB_Y);
    if (this->sys_js_button[4].pressed && this->sys_js_button[9].on_press) this->control.SetGamepad(Input::Gamepad::LB_LStick);
    if (this->sys_js_button[4].pressed && this->sys_js_button[10].on_press) this->control.SetGamepad(Input::Gamepad::LB_RStick);
    if (this->sys_js_button[4].pressed && this->sys_js_axis[7] < 0) this->control.SetGamepad(Input::Gamepad::LB_DPadUp);
    if (this->sys_js_button[4].pressed && this->sys_js_axis[7] > 0) this->control.SetGamepad(Input::Gamepad::LB_DPadDown);
    if (this->sys_js_button[4].pressed && this->sys_js_axis[6] > 0) this->control.SetGamepad(Input::Gamepad::LB_DPadRight);
    if (this->sys_js_button[4].pressed && this->sys_js_axis[6] < 0) this->control.SetGamepad(Input::Gamepad::LB_DPadLeft);
    if (this->sys_js_button[5].pressed && this->sys_js_button[0].on_press) this->control.SetGamepad(Input::Gamepad::RB_A);
    if (this->sys_js_button[5].pressed && this->sys_js_button[1].on_press) this->control.SetGamepad(Input::Gamepad::RB_B);
    if (this->sys_js_button[5].pressed && this->sys_js_button[2].on_press) this->control.SetGamepad(Input::Gamepad::RB_X);
    if (this->sys_js_button[5].pressed && this->sys_js_button[3].on_press) this->control.SetGamepad(Input::Gamepad::RB_Y);
    if (this->sys_js_button[5].pressed && this->sys_js_button[9].on_press) this->control.SetGamepad(Input::Gamepad::RB_LStick);
    if (this->sys_js_button[5].pressed && this->sys_js_button[10].on_press) this->control.SetGamepad(Input::Gamepad::RB_RStick);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[7] < 0) this->control.SetGamepad(Input::Gamepad::RB_DPadUp);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[7] > 0) this->control.SetGamepad(Input::Gamepad::RB_DPadDown);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[6] > 0) this->control.SetGamepad(Input::Gamepad::RB_DPadRight);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[6] < 0) this->control.SetGamepad(Input::Gamepad::RB_DPadLeft);
    if (this->sys_js_button[4].pressed && this->sys_js_button[5].on_press) this->control.SetGamepad(Input::Gamepad::LB_RB);

    // 通过sys_js_max_value将各项指令归一化
    // float ly = -float(this->sys_js_axis[1]) / float(this->sys_js_max_value);
    // float lx = -float(this->sys_js_axis[0]) / float(this->sys_js_max_value);
    // float rx = -float(this->sys_js_axis[3]) / float(this->sys_js_max_value);

    float ly = (-float(this->sys_js_axis[1]) / float(this->sys_js_max_value)) * this->params.Get<std::vector<float>>("vel_command")[0];
    float lx = (-float(this->sys_js_axis[0]) / float(this->sys_js_max_value)) * this->params.Get<std::vector<float>>("vel_command")[1];
    float rx = (-float(this->sys_js_axis[3]) / float(this->sys_js_max_value)) * this->params.Get<std::vector<float>>("vel_command")[2];

    bool has_input = (ly != 0.0f || lx != 0.0f || rx != 0.0f);

    if (has_input)
    {
        this->control.x = ly;
        this->control.y = lx;
        this->control.yaw = rx;
        this->sys_js_active = true;
    }
    else if (this->sys_js_active)
    {
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
        this->sys_js_active = false;
    }

    if (this->control.current_gamepad == Input::Gamepad::DPadUp )
    {
        if (!this->control.dpad_handled) { // 如果还没处理过
            this->control.gait_frequency += 0.5f;
            this->control.dpad_handled = true; // 标记为已处理
        }
    }
    else if (this->control.current_gamepad == Input::Gamepad::DPadDown )
    {
        if (!this->control.dpad_handled) { // 如果还没处理过
            this->control.gait_frequency -= 0.5f;
            this->control.dpad_handled = true; // 标记为已处理
        }
    }
    else 
    {
        this->control.dpad_handled = false;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto rl_sar = std::make_shared<RL_Real>(argc, argv);
    rclcpp::spin(rl_sar->ros2_node);
    rclcpp::shutdown();
    return 0;
}
