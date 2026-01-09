
#include "rl_real_LW.hpp"

using namespace std::chrono_literals;

RL_Real::RL_Real(int argc, char **argv)
{
    ros2_node = std::make_shared<rclcpp::Node>("rl_real_LW_node");
    // subscribe qos config
    auto subscribers_qos = rclcpp::SystemDefaultsQoS();
    subscribers_qos.keep_last(1);
    subscribers_qos.best_effort();
    this->imu_subscriber_ = ros2_node->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", subscribers_qos,
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
        "/LW_joint_states", 1
    );
    this->realtime_debug_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(jointstate_plot_publisher_);
    
    this->realtime_debug_publisher_->lock();
    auto & debug_msg = realtime_debug_publisher_->msg_;
    debug_msg.header.stamp = ros2_node->get_clock()->now();
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
    std::vector<std::string> imu_states = {
        "imu_ang_vel",
    };
    std::vector<std::string> joint_names;
    joint_names.reserve(joint_now_names.size() + joint_target_names.size() + imu_states.size());
    joint_names.insert(joint_names.end(), joint_now_names.begin(), joint_now_names.end());
    joint_names.insert(joint_names.end(), joint_target_names.begin(), joint_target_names.end());
    joint_names.insert(joint_names.end(), imu_states.begin(), imu_states.end());

    size_t total_size = joint_names.size();
    debug_msg.name.resize(total_size); 
    debug_msg.position.resize(total_size);
    debug_msg.velocity.resize(total_size);
    debug_msg.effort.resize(total_size);
    debug_msg.name = joint_names;
    this->realtime_debug_publisher_->unlock();

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
    if (this->realtime_debug_publisher_->trylock())
    {
        auto & msg = realtime_debug_publisher_->msg_;

        int num_of_dofs = this->params.Get<int>("num_of_dofs");
        auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
        auto wheel_indices = this->params.Get<std::vector<int>>("wheel_indices");
        auto rl_kp = this->params.Get<std::vector<float>>("rl_kp");
        auto rl_kd = this->params.Get<std::vector<float>>("rl_kd");

        for (int i = 0; i < num_of_dofs; ++i)
        {
            msg.velocity[i] = this->lw_low_state.motorState[joint_mapping[i]].vel_now;
            msg.effort[i] = this->lw_low_state.motorState[joint_mapping[i]].tau_now;
            if (i == wheel_indices[0] || i == wheel_indices[1] ) // 两个轮子
            {
                msg.position[i] = 0.0f;
            }
            else
            {
                msg.position[i] = this->lw_low_state.motorState[joint_mapping[i]].pos_now;
            }
        }
        for (int i = num_of_dofs; i < 2*num_of_dofs; ++i)
        {
            if ((i-num_of_dofs) == wheel_indices[0] || (i-num_of_dofs) == wheel_indices[1])
            {
                msg.velocity[i] = this->lw_low_command.motorCmd[joint_mapping[i - num_of_dofs]].action_set;
                msg.effort[i] = rl_kp[i - num_of_dofs]*(0.0f - this->lw_low_state.motorState[joint_mapping[i - num_of_dofs]].pos_now) +
                                rl_kd[i - num_of_dofs]*(this->lw_low_command.motorCmd[joint_mapping[i - num_of_dofs]].action_set - this->lw_low_state.motorState[joint_mapping[i - num_of_dofs]].vel_now);
            }
            else{
                msg.position[i] = this->lw_low_command.motorCmd[joint_mapping[i - num_of_dofs]].action_set;
                msg.effort[i] = rl_kp[i - num_of_dofs]*(this->lw_low_command.motorCmd[joint_mapping[i - num_of_dofs]].action_set - this->lw_low_state.motorState[joint_mapping[i - num_of_dofs]].pos_now) +
                                rl_kd[i - num_of_dofs]*(0.0f - this->lw_low_state.motorState[joint_mapping[i - num_of_dofs]].vel_now);
            }
        }
        

        msg.position[2*num_of_dofs] = this->robot_state.imu.gyroscope[0];
        msg.velocity[2*num_of_dofs] = this->robot_state.imu.gyroscope[1];
        msg.effort[2*num_of_dofs] = this->robot_state.imu.gyroscope[2];

        this->realtime_debug_publisher_->unlockAndPublish();
    }
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
#ifdef CONTROL_TIME_PRINT
    auto t_start = std::chrono::high_resolution_clock::now();
#endif
    this->GetState(&this->robot_state);
#ifdef CONTROL_TIME_PRINT
    auto t_after_get = std::chrono::high_resolution_clock::now();
#endif

    this->StateController(&this->robot_state, &this->robot_command);

    this->control.ClearInput();

    this->SetCommand(&this->robot_command);
#ifdef CONTROL_TIME_PRINT
    auto t_end = std::chrono::high_resolution_clock::now();

    double total_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    double read_ms = std::chrono::duration<double, std::milli>(t_after_get - t_start).count();
    static int count = 0;
    if (++count % 50 == 0) {
        std::cout << "[Latency Test] Total Control Pipeline: " << total_ms << "ms "
                  << "(SDK Read: " << read_ms << "ms)" << std::endl;
        if (total_ms > this->params.Get<float>("dt") * 1000.0f) {
            std::cout << LOGGER::WARNING << "!!! Control Overrun !!!" << std::endl;
        }
    }
#endif
}

void RL_Real::RunModel()
{
    if (this->rl_init_done)
    {
#ifdef FOWARD_TIME_PRINT
        auto t_start = std::chrono::high_resolution_clock::now();
#endif
        RobotState<float> local_state; 

        {
            std::lock_guard<std::mutex> lock(state_mutex); // 保护 robot_state 的完整性
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
#ifdef FOWARD_TIME_PRINT
        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

        // --- 超时检测逻辑 ---
        float rl_dt_ms = this->params.Get<float>("dt") * this->params.Get<int>("decimation") * 1000.0f;
        
        static double max_ms = 0;
        static int count = 0;
        if (elapsed_ms > max_ms) max_ms = elapsed_ms;
        if (elapsed_ms > rl_dt_ms) {
                std::cout << "  <-- !!! TIMEOUT !!!" << std::endl;
            }

        // 每 100 次打印一次统计信息，避免频繁 IO 影响性能
        if (++count % 100 == 0) {
            std::cout << "[RL Inference] Curr: " << elapsed_ms << "ms, Max: " << max_ms 
                      << "ms, Limit: " << rl_dt_ms << "ms";
            std::cout << std::endl;
        }
#endif

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
    // clamped观测打印
    // std::cout << clamped_obs << std::endl;

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
    received_imu_msg_ptr_.set(std::move(msg));
}

void RL_Real::GetState(RobotState<float> *state)
{
    std::shared_ptr<sensor_msgs::msg::Imu> imu_msg_ptr;
    received_imu_msg_ptr_.get(imu_msg_ptr);
    if (!imu_msg_ptr) {
        std::cout << LOGGER::WARNING << "No IMU data received yet!" << std::endl;
        return;
    }
    auto imu =  *imu_msg_ptr;

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
    auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
    auto wheel_indices = this->params.Get<std::vector<int>>("wheel_indices");
    int num_dofs = this->params.Get<int>("num_of_dofs");

    for (int i = 0; i < num_dofs; ++i)
    {
        int motor_id = joint_mapping[i];
        
        this->lw_low_command.motorCmd[motor_id].Kp = command->motor_command.kp[i];
        this->lw_low_command.motorCmd[motor_id].Kd = command->motor_command.kd[i];

        bool is_wheel = false;
        for (int k : wheel_indices) {
            if (i == k) {
                is_wheel = true;
                break;
            }
        }

        if (is_wheel) {

            this->lw_low_command.motorCmd[motor_id].action_set = command->motor_command.dq[i];

        } else {
            this->lw_low_command.motorCmd[motor_id].action_set = command->motor_command.q[i];
        }
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
    if (this->sys_js_button[3].on_press) this->control.SetGamepad(Input::Gamepad::X);
    if (this->sys_js_button[4].on_press) this->control.SetGamepad(Input::Gamepad::Y);
    if (this->sys_js_button[6].on_press) this->control.SetGamepad(Input::Gamepad::LB);
    if (this->sys_js_button[7].on_press) this->control.SetGamepad(Input::Gamepad::RB);
    if (this->sys_js_button[13].on_press) this->control.SetGamepad(Input::Gamepad::LStick);
    if (this->sys_js_button[14].on_press) this->control.SetGamepad(Input::Gamepad::RStick);
    if (this->sys_js_axis[7] < 0) this->control.SetGamepad(Input::Gamepad::DPadUp);
    if (this->sys_js_axis[7] > 0) this->control.SetGamepad(Input::Gamepad::DPadDown);
    if (this->sys_js_axis[6] < 0) this->control.SetGamepad(Input::Gamepad::DPadLeft);
    if (this->sys_js_axis[6] > 0) this->control.SetGamepad(Input::Gamepad::DPadRight);
    if (this->sys_js_button[6].pressed && this->sys_js_button[0].on_press) this->control.SetGamepad(Input::Gamepad::LB_A);
    if (this->sys_js_button[6].pressed && this->sys_js_button[1].on_press) this->control.SetGamepad(Input::Gamepad::LB_B);
    if (this->sys_js_button[6].pressed && this->sys_js_button[3].on_press) this->control.SetGamepad(Input::Gamepad::LB_X);
    if (this->sys_js_button[6].pressed && this->sys_js_button[4].on_press) this->control.SetGamepad(Input::Gamepad::LB_Y);
    if (this->sys_js_button[6].pressed && this->sys_js_button[13].on_press) this->control.SetGamepad(Input::Gamepad::LB_LStick);
    if (this->sys_js_button[6].pressed && this->sys_js_button[14].on_press) this->control.SetGamepad(Input::Gamepad::LB_RStick);
    if (this->sys_js_button[6].pressed && this->sys_js_axis[7] < 0) this->control.SetGamepad(Input::Gamepad::LB_DPadUp);
    if (this->sys_js_button[6].pressed && this->sys_js_axis[7] > 0) this->control.SetGamepad(Input::Gamepad::LB_DPadDown);
    if (this->sys_js_button[6].pressed && this->sys_js_axis[6] > 0) this->control.SetGamepad(Input::Gamepad::LB_DPadRight);
    if (this->sys_js_button[6].pressed && this->sys_js_axis[6] < 0) this->control.SetGamepad(Input::Gamepad::LB_DPadLeft);
    if (this->sys_js_button[7].pressed && this->sys_js_button[0].on_press) this->control.SetGamepad(Input::Gamepad::RB_A);
    if (this->sys_js_button[7].pressed && this->sys_js_button[1].on_press) this->control.SetGamepad(Input::Gamepad::RB_B);
    if (this->sys_js_button[7].pressed && this->sys_js_button[3].on_press) this->control.SetGamepad(Input::Gamepad::RB_X);
    if (this->sys_js_button[7].pressed && this->sys_js_button[4].on_press) this->control.SetGamepad(Input::Gamepad::RB_Y);
    if (this->sys_js_button[7].pressed && this->sys_js_button[13].on_press) this->control.SetGamepad(Input::Gamepad::RB_LStick);
    if (this->sys_js_button[7].pressed && this->sys_js_button[14].on_press) this->control.SetGamepad(Input::Gamepad::RB_RStick);
    if (this->sys_js_button[7].pressed && this->sys_js_axis[7] < 0) this->control.SetGamepad(Input::Gamepad::RB_DPadUp);
    if (this->sys_js_button[7].pressed && this->sys_js_axis[7] > 0) this->control.SetGamepad(Input::Gamepad::RB_DPadDown);
    if (this->sys_js_button[7].pressed && this->sys_js_axis[6] > 0) this->control.SetGamepad(Input::Gamepad::RB_DPadRight);
    if (this->sys_js_button[7].pressed && this->sys_js_axis[6] < 0) this->control.SetGamepad(Input::Gamepad::RB_DPadLeft);
    if (this->sys_js_button[6].pressed && this->sys_js_button[7].on_press) this->control.SetGamepad(Input::Gamepad::LB_RB);

    // 通过sys_js_max_value将各项指令归一化
    // float ly = -float(this->sys_js_axis[1]) / float(this->sys_js_max_value);
    // float lx = -float(this->sys_js_axis[0]) / float(this->sys_js_max_value);
    // float rx = -float(this->sys_js_axis[3]) / float(this->sys_js_max_value);

    float ly = (-float(this->sys_js_axis[1]) / float(this->sys_js_max_value)) * this->params.Get<std::vector<float>>("vel_command")[0];
    float lx = (-float(this->sys_js_axis[0]) / float(this->sys_js_max_value)) * this->params.Get<std::vector<float>>("vel_command")[1];
    float rx = (-float(this->sys_js_axis[2]) / float(this->sys_js_max_value)) * this->params.Get<std::vector<float>>("vel_command")[2];

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
            this->control.gait_frequency += 0.1f;
            this->control.dpad_handled = true; // 标记为已处理
        }
    }
    else if (this->control.current_gamepad == Input::Gamepad::DPadDown )
    {
        if (!this->control.dpad_handled) { // 如果还没处理过
            this->control.gait_frequency -= 0.1f;
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
