
#include "rl_sim_LW.hpp"

#include <iomanip>
#include <sstream>

using namespace std::chrono_literals;
RL_Real* RL_Real::instance = nullptr;

namespace
{
const char* LWOperatorModeName(LWOperatorMode mode)
{
    switch (mode)
    {
    case LWOperatorMode::Passive: return "passive";
    case LWOperatorMode::GetUpLeg: return "get-up-leg";
    case LWOperatorMode::GetUpWheel: return "get-up-wheel";
    case LWOperatorMode::GetDown: return "get-down";
    case LWOperatorMode::LegLocomotion: return "leg-locomotion";
    case LWOperatorMode::WheelLocomotion: return "wheel-locomotion";
    case LWOperatorMode::LegToWheel: return "leg-to-wheel";
    case LWOperatorMode::WheelToLeg: return "wheel-to-leg";
    case LWOperatorMode::Unknown: return "unknown";
    }
    return "unknown";
}

bool LWOperatorModeHasProgress(LWOperatorMode mode)
{
    return mode == LWOperatorMode::GetUpLeg
        || mode == LWOperatorMode::GetUpWheel
        || mode == LWOperatorMode::GetDown
        || mode == LWOperatorMode::LegToWheel
        || mode == LWOperatorMode::WheelToLeg;
}
} // namespace

// // ===================== [定义抗冲击实验状态变量] =====================
// static double g_impact_start_time = -1.0;  // 记录冲击开始的物理时间
// static double g_impact_force_x = 0.0;      // X方向(前后)的冲击力 (N)
// static double g_impact_force_y = 0.0;      // Y方向(侧向)的冲击力 (N)
// static double g_impact_duration = 0.2;     // 冲击持续时间 (例如 0.2秒 = 200ms)
// // ============================================================================

RL_Real::RL_Real(int argc, char **argv)
{
    ros2_node = std::make_shared<rclcpp::Node>("rl_real_LW_node");
    // this->imu_subscriber_ = ros2_node->create_subscription<sensor_msgs::msg::Imu>(
    //     "/imu", rclcpp::SystemDefaultsQoS(),
    //     [this] (const sensor_msgs::msg::Imu::SharedPtr imu_msg) {this->ImuCallback(imu_msg);}
    // );

    // Set static instance pointer early for signal handler
    instance = this;

    // scan for libraries in the plugin directory to load additional plugins
    scanPluginLibraries();

    mjv_defaultCamera(&this->cam);

    mjv_defaultOption(&this->opt);

    mjv_defaultPerturb(&this->pert);

    // simulate object encapsulates the UI
    sim = std::make_unique<mj::Simulate>(
        std::make_unique<mj::GlfwAdapter>(),
        &cam, &opt, &pert, /* is_passive = */ false);

    this->robot_name = "LW";
    this->scene_name = "scene"; // "scene" "scene_terrain"
    std::string filename = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/../rl_sar_zoo/" + this->robot_name + "_description/mjcf/" + this->scene_name + ".xml";

    // start physics thread
    std::thread physicsthreadhandle(&PhysicsThread, sim.get(), filename.c_str());
    physicsthreadhandle.detach();

    while (1)
    {
        if (d)
        {
            std::cout << LOGGER::INFO << "[MuJoCo] Data prepared" << std::endl;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    this->mj_model = m;
    this->mj_data = d;

    this->SetupSysJoystick("/dev/input/js1", 16);

    // read params from yaml
    this->ang_vel_axis = "body";
    
    this->ReadYaml(this->robot_name, "base.yaml");

    // 提前加载所有的模型到内存
    this->PreloadModel(this->robot_name + "/robot_lab/leg_loco");
    this->PreloadModel(this->robot_name + "/robot_lab/wheel_loco");
    this->PreloadModel(this->robot_name + "/robot_lab/leg_to_wheel");
    this->PreloadModel(this->robot_name + "/robot_lab/wheel_to_leg");
    this->PreloadLWPolicyContext(this->robot_name + "/robot_lab/leg_loco");
    this->PreloadLWPolicyContext(this->robot_name + "/robot_lab/wheel_loco");
    this->PreloadLWPolicyContext(this->robot_name + "/robot_lab/leg_to_wheel");
    this->PreloadLWPolicyContext(this->robot_name + "/robot_lab/wheel_to_leg");

    // 解析命令行参数
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--use_actuator_net") {
            this->use_actuator_net_ = true;
            std::cout << LOGGER::INFO << "Actuator Network Mode Enabled!" << std::endl;
        }
    }

    // 如果开启了执行器网络，加载 ONNX 并初始化 Buffer
    if (this->use_actuator_net_) {
        // 这里的路径请根据实际情况修改
        std::string leg_mlp_path = std::string(POLICY_DIR) + "/" + this->robot_name + "/robot_lab/motors/leg_actuator_net.pt";
        std::string foot_mlp_path = std::string(POLICY_DIR) + "/" + this->robot_name + "/robot_lab/motors/foot_actuator_net.pt";
        this->leg_actuator_model_ = InferenceRuntime::ModelFactory::load_model(leg_mlp_path);
        this->foot_actuator_model_ = InferenceRuntime::ModelFactory::load_model(foot_mlp_path);
        
        if (!this->leg_actuator_model_ || !this->foot_actuator_model_) {
            std::cout << LOGGER::ERROR << "Failed to load Actuator Network ONNX!" << std::endl;
        }

        int num_dofs = this->params.Get<int>("num_of_dofs");
        int decimation = this->params.Get<int>("decimation");
        int history_len = 2 * decimation + 1; // 队列大小设为 9 (decimation=4)

        for(int i = 0; i < history_len; ++i) {
            this->pos_err_history_.push_front(std::vector<float>(num_dofs, 0.0f));
            this->vel_history_.push_front(std::vector<float>(num_dofs, 0.0f));
        }

        this->actuator_net_tau_.resize(num_dofs, 0.0f);
    }

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
    this->lw_sdk.InitCmdData(this->lw_low_command);
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    this->InitOutputs();
    this->InitControl();
    this->control.gait_frequency = this->params.Get<std::vector<float>>("gait_command")[0];
    this->gait_phase_time = 0.0f;
    policy_input_snapshot_.publish(
        {this->robot_state,
         {this->control.x,
          this->control.y,
          this->control.yaw,
          this->control.gait_frequency}});

    // loop
    this->loop_joystick = std::make_shared<LoopFunc>("loop_joystick", 0.01, std::bind(&RL_Real::GetSysJoystick, this));
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.Get<float>("dt"), std::bind(&RL_Real::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.Get<float>("dt") * this->params.Get<int>("decimation"), std::bind(&RL_Real::RunModel, this));
    this->loop_joystick->start();
    this->loop_control->start();
    this->loop_rl->start();
    this->operator_status_timer_ = ros2_node->create_wall_timer(
        100ms,
        std::bind(&RL_Real::OperatorStatusCallback, this));
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
    instance = nullptr;
    this->loop_joystick->shutdown();
    this->loop_rl->shutdown();
    this->loop_control->shutdown();
    //disable_lw_robot();
    std::cout << LOGGER::INFO << "RL_Real exit" << std::endl;
}

void RL_Real::jointstate_plot_callback(void)
{
    SimDebugSnapshot snapshot;
    if (!debug_snapshot_.read(snapshot))
    {
        return;
    }
    sensor_msgs::msg::JointState msg;
    // 强烈建议加上时间戳，保证 ROS 2 Bag 的时间对齐
    msg.header.stamp = this->ros2_node->get_clock()->now();

    // ================= 1. 定义消息名称列表 =================
    // 关节顺序与 base.yaml 里一致
    std::vector<std::string> joint_now_names = {
        "right_hip_now", "left_hip_now", "right_thigh_now", "left_thigh_now",
        "right_shank_now", "left_shank_now", "right_foot_now", "left_foot_now",
        "right_wheel_now", "left_wheel_now"
    };
    std::vector<std::string> joint_target_names = {
        "right_hip_target", "left_hip_target", "right_thigh_target", "left_thigh_target",
        "right_shank_target", "left_shank_target", "right_foot_target", "left_foot_target",
        "right_wheel_target", "left_wheel_target"
    };
    std::vector<std::string> gait_data_names = {
        "l_foot_x", "l_foot_y", "l_foot_z", "l_contact",
        "r_foot_x", "r_foot_y", "r_foot_z", "r_contact"
    };
    std::vector<std::string> tracking_data_names = {
        "cmd_vel_x", "cmd_vel_yaw",           // 期望线速度 (X) 和 期望角速度 (Z)
        "base_p_x", "base_p_y", "base_p_z",   // 机身全局位置
        "base_v_x", "base_v_y", "base_v_z",   // 机身全局线速度
        "base_w_x", "base_w_y", "base_w_z",   // 机身全局角速度
        "base_q_w", "base_q_x", "base_q_y", "base_q_z" // 机身全局姿态四元数
    };

    // 合并所有名称
    std::vector<std::string> joint_names;
    joint_names.reserve(joint_now_names.size() + joint_target_names.size() + 
                        gait_data_names.size() + tracking_data_names.size());
    joint_names.insert(joint_names.end(), joint_now_names.begin(), joint_now_names.end());
    joint_names.insert(joint_names.end(), joint_target_names.begin(), joint_target_names.end());
    joint_names.insert(joint_names.end(), gait_data_names.begin(), gait_data_names.end());
    joint_names.insert(joint_names.end(), tracking_data_names.begin(), tracking_data_names.end());

    // ================= 2. 初始化 msg 数组 =================
    size_t total_size = joint_names.size();
    msg.name = joint_names;
    msg.position.resize(total_size, 0.0); // 默认填充 0.0
    msg.velocity.resize(total_size, 0.0);
    msg.effort.resize(total_size, 0.0);

    // ================= 3. 填充关节与命令状态 =================
    int num_of_dofs = this->params.Get<int>("num_of_dofs");
    auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
    auto wheel_indices = this->params.Get<std::vector<int>>("wheel_indices");

    for (int i = 0; i < num_of_dofs; ++i)
    {
        msg.position[i] = snapshot.robot_state.motor_state.q[i];
        msg.velocity[i] = snapshot.robot_state.motor_state.dq[i];
        msg.effort[i] = snapshot.robot_state.motor_state.tau_est[i];
    }
    for (int i = num_of_dofs; i < 2 * num_of_dofs; ++i)
    {
        msg.position[i] =
            snapshot.robot_command.motor_command.q[
                i - num_of_dofs];
    }
    for (int i : wheel_indices)
    {
        msg.position[i + num_of_dofs] = 0.0f;
        msg.velocity[i + num_of_dofs] =
            snapshot.robot_command.motor_command.dq[i];
    }

    // ================= 4. 填充步态数据 (脚端信息) =================
    static int id_l_site = mj_name2id(mj_model, mjOBJ_SITE, "left_foot_site");
    static int id_r_site = mj_name2id(mj_model, mjOBJ_SITE, "right_foot_site");
    static int id_l_sensor = mj_name2id(mj_model, mjOBJ_SENSOR, "left_foot_force_sensor");
    static int id_r_sensor = mj_name2id(mj_model, mjOBJ_SENSOR, "right_foot_force_sensor");

    int offset_gait = joint_now_names.size() + joint_target_names.size();

    if (id_l_site >= 0 && id_r_site >= 0 && id_l_sensor >= 0 && id_r_sensor >= 0) 
    {
        // 获取 Z 轴方向的受力 (+2 对应 Z 轴)
        double l_force = mj_data->sensordata[mj_model->sensor_adr[id_l_sensor] + 2];
        double r_force = mj_data->sensordata[mj_model->sensor_adr[id_r_sensor] + 2];
        double force_threshold = 10.0; 

        msg.position[offset_gait + 0] = mj_data->site_xpos[3 * id_l_site + 0];
        msg.position[offset_gait + 1] = mj_data->site_xpos[3 * id_l_site + 1];
        msg.position[offset_gait + 2] = mj_data->site_xpos[3 * id_l_site + 2];
        msg.position[offset_gait + 3] = (l_force > force_threshold) ? 1.0 : 0.0;

        msg.position[offset_gait + 4] = mj_data->site_xpos[3 * id_r_site + 0];
        msg.position[offset_gait + 5] = mj_data->site_xpos[3 * id_r_site + 1];
        msg.position[offset_gait + 6] = mj_data->site_xpos[3 * id_r_site + 2];
        msg.position[offset_gait + 7] = (r_force > force_threshold) ? 1.0 : 0.0;
    }

    // ================= 5. 填充全局追踪与位姿信息 =================
    static int id_frame_pos = mj_name2id(mj_model, mjOBJ_SENSOR, "frame_pos");
    static int id_frame_vel = mj_name2id(mj_model, mjOBJ_SENSOR, "frame_vel");
    static int id_imu_gyro  = mj_name2id(mj_model, mjOBJ_SENSOR, "imu_gyro");
    static int id_imu_quat  = mj_name2id(mj_model, mjOBJ_SENSOR, "imu_quat"); // 新增四元数ID获取

    int offset_track = offset_gait + gait_data_names.size();

    // 记录期望指令
    msg.position[offset_track + 0] = snapshot.control.x;
    msg.position[offset_track + 1] = snapshot.control.yaw;

    // 提取传感器数据
    if (id_frame_pos >= 0) {
        int adr = mj_model->sensor_adr[id_frame_pos];
        msg.position[offset_track + 2] = mj_data->sensordata[adr + 0]; // base_p_x
        msg.position[offset_track + 3] = mj_data->sensordata[adr + 1]; // base_p_y
        msg.position[offset_track + 4] = mj_data->sensordata[adr + 2]; // base_p_z
    }
    
    if (id_frame_vel >= 0) {
        int adr = mj_model->sensor_adr[id_frame_vel];
        msg.position[offset_track + 5] = mj_data->sensordata[adr + 0]; // base_v_x
        msg.position[offset_track + 6] = mj_data->sensordata[adr + 1]; // base_v_y
        msg.position[offset_track + 7] = mj_data->sensordata[adr + 2]; // base_v_z
    }

    if (id_imu_gyro >= 0) {
        int adr = mj_model->sensor_adr[id_imu_gyro];
        msg.position[offset_track + 8] = mj_data->sensordata[adr + 0]; // base_w_x
        msg.position[offset_track + 9] = mj_data->sensordata[adr + 1]; // base_w_y
        msg.position[offset_track + 10] = mj_data->sensordata[adr + 2]; // base_w_z
    }

    if (id_imu_quat >= 0) {
        int adr = mj_model->sensor_adr[id_imu_quat];
        msg.position[offset_track + 11] = mj_data->sensordata[adr + 0]; // base_q_w
        msg.position[offset_track + 12] = mj_data->sensordata[adr + 1]; // base_q_x
        msg.position[offset_track + 13] = mj_data->sensordata[adr + 2]; // base_q_y
        msg.position[offset_track + 14] = mj_data->sensordata[adr + 3]; // base_q_z
    }

    // ==============================================================================
    this->jointstate_plot_publisher_->publish(msg);

}

void RL_Real::OperatorStatusCallback()
{
    LWOperatorStatusSnapshot status;
    if (!ReadLWOperatorStatus(status)
        || (operator_status_seen_
            && status.sequence == last_operator_status_sequence_))
    {
        return;
    }

    operator_status_seen_ = true;
    last_operator_status_sequence_ = status.sequence;
    std::ostringstream message;
    message << LOGGER::INFO << "[LW Sim2Sim] mode="
            << LWOperatorModeName(status.mode)
            << std::fixed << std::setprecision(3)
            << ", x=" << status.x
            << ", y=" << status.y
            << ", yaw=" << status.yaw
            << ", gait=" << status.gait_frequency;
    if (LWOperatorModeHasProgress(status.mode))
    {
        message << std::setprecision(1)
                << ", progress=" << status.progress * 100.0f << '%';
    }
    std::cout << message.str() << std::endl;
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

    ApplyPendingInput();
    this->KeyboardInterface();
    ApplyJoystickFaultGate();
    this->GetState(&this->robot_state);

    this->StateController(&this->robot_state, &this->robot_command);
    policy_input_snapshot_.publish(
        {this->robot_state,
         {this->control.x,
          this->control.y,
          this->control.yaw,
          this->control.gait_frequency}});

    if (this->control.current_keyboard == Input::Keyboard::R || this->control.current_gamepad == Input::Gamepad::RB_Y)
    {
        if (this->mj_model && this->mj_data)
        {   
            if(this->robot_name == "LW")
            {
                int id = mj_name2id(this->mj_model, mjOBJ_KEY, "home_leg");
                mj_resetDataKeyframe(this->mj_model, this->mj_data, id);
            }
            else
            {
                mj_resetData(this->mj_model, this->mj_data);
            }

            mj_forward(this->mj_model, this->mj_data);
        }
    }
    if (this->control.current_keyboard == Input::Keyboard::T || this->control.current_gamepad == Input::Gamepad::RB_A)
    {
        if (this->mj_model && this->mj_data)
        {   
            if(this->robot_name == "LW")
            {
                int id = mj_name2id(this->mj_model, mjOBJ_KEY, "home_wheel");
                mj_resetDataKeyframe(this->mj_model, this->mj_data, id);
            }
            else
            {
                mj_resetData(this->mj_model, this->mj_data);
            }

            mj_forward(this->mj_model, this->mj_data);
        }
    }
    if (this->control.current_keyboard == Input::Keyboard::Enter || this->control.current_gamepad == Input::Gamepad::RB_X)
    {
        if (simulation_running)
        {
            sim->run = 0;
            std::cout << std::endl << LOGGER::INFO << "Simulation Stop" << std::endl;
        }
        else
        {
            sim->run = 1;
            std::cout << std::endl << LOGGER::INFO << "Simulation Start" << std::endl;
        }
        simulation_running = !simulation_running;
    }

    this->control.ClearInput();

    const auto activation = LoadLWPolicyActivation();
    const auto inference_output = LoadLWPolicyOutput();
    const auto now = std::chrono::steady_clock::now();
    const bool has_current_inference_output =
        activation
        && activation->definition
        && EvaluateLWPolicyOutput(
               inference_output.get(),
               activation->generation,
               static_cast<size_t>(
                   activation->definition->params.Get<int>(
                       "num_of_dofs")),
               now,
               GetLWPolicyOutputMaxAge(*activation))
            == LWPolicyOutputStatus::Ready;
    if (this->use_actuator_net_
        && this->leg_actuator_model_
        && this->foot_actuator_model_
        && has_current_inference_output
        && inference_output->dof_pos.size()
            == static_cast<size_t>(
                this->params.Get<int>("num_of_dofs")))
    {
        int num_dofs = this->params.Get<int>("num_of_dofs");
        std::vector<float> current_pos_err(num_dofs, 0.0f);
        std::vector<float> current_vel(num_dofs, 0.0f);
        
        // 获取最新瞬间的误差和速度
        for (int i = 0; i < num_dofs; ++i) {
            // this->output_dof_pos 是算出的目标点，robot_state.q 是 200Hz 最新的物理状态
            current_pos_err[i] = inference_output->dof_pos[i] - this->robot_state.motor_state.q[i];
            current_vel[i] = this->robot_state.motor_state.dq[i];
        }
        // 在执行器网络启动的最初期，预填充历史
        if (inference_output->frame <= 1) {
            for (size_t k = 0; k < this->pos_err_history_.size(); ++k) {
                this->pos_err_history_[k] = current_pos_err;
                this->vel_history_[k] = current_vel;
            }
        }
        
        // 高频滚动历史缓冲区
        this->pos_err_history_.pop_back();
        this->pos_err_history_.push_front(current_pos_err);
        
        this->vel_history_.pop_back();
        this->vel_history_.push_front(current_vel);
        
        int decimation = this->params.Get<int>("decimation"); // 通常是 4
        
        for (int i : leg_train_indices) {
            // 跨步长抓取历史特征 (对应 0ms, -20ms, -40ms)
            std::vector<float> mlp_input = {
                this->pos_err_history_[0][i] * 1.0f,               // 0ms
                this->pos_err_history_[decimation][i] * 1.0f,      // -20ms
                this->pos_err_history_[2 * decimation][i] * 1.0f,  // -40ms
                this->vel_history_[0][i] * 1.0f,
                this->vel_history_[decimation][i] * 1.0f,
                this->vel_history_[2 * decimation][i] * 1.0f
            };
            
            auto output = this->leg_actuator_model_->forward({mlp_input});
            this->actuator_net_tau_[i] = output[0] * 1.0f; 
        }

        for (int i : foot_train_indices) {
            // 跨步长抓取历史特征 (对应 0ms, -20ms, -40ms)
            std::vector<float> mlp_input = {
                this->pos_err_history_[0][i] * 1.0f,               // 0ms
                this->pos_err_history_[decimation][i] * 1.0f,      // -20ms
                this->pos_err_history_[2 * decimation][i] * 1.0f,  // -40ms
                this->vel_history_[0][i] * 1.0f,
                this->vel_history_[decimation][i] * 1.0f,
                this->vel_history_[2 * decimation][i] * 1.0f
            };
            
            auto output = this->foot_actuator_model_->forward({mlp_input});
            this->actuator_net_tau_[i] = output[0] * 1.0f; 
        }
        actuator_net_generation_ = activation->generation;
    }

    // // ===================== [执行机身外部冲击力] =====================
    // if (this->mj_model && this->mj_data)
    // {
    //     // 获取机身(浮动基座)的 ID
    //     int body_id = mj_name2id(this->mj_model, mjOBJ_BODY, "base_link");

    //     if (g_impact_start_time > 0)
    //     {
    //         double t_elapsed = this->mj_data->time - g_impact_start_time;
    //         if (t_elapsed < g_impact_duration)
    //         {
    //             // 施加六轴力/力矩:[Fx, Fy, Fz, Tx, Ty, Tz]
    //             this->mj_data->xfrc_applied[6 * body_id + 0] = g_impact_force_x;
    //             this->mj_data->xfrc_applied[6 * body_id + 1] = g_impact_force_y;
    //             this->mj_data->xfrc_applied[6 * body_id + 2] = 0.0; // Z 轴不受力
    //         }
    //         else
    //         {
    //             // 冲击结束，力清零
    //             this->mj_data->xfrc_applied[6 * body_id + 0] = 0.0;
    //             this->mj_data->xfrc_applied[6 * body_id + 1] = 0.0;
    //             this->mj_data->xfrc_applied[6 * body_id + 2] = 0.0;
    //             g_impact_start_time = -1.0;
    //             std::cout << "[INFO] 🛑 冲击结束，力已释放" << std::endl;
    //         }
    //     }
    // }
    // // ============================================================================

    this->SetCommand(&this->robot_command);
    debug_snapshot_.publish(
        SimDebugSnapshot{
            this->robot_state,
            this->robot_command,
            {this->control.x,
             this->control.y,
             this->control.yaw,
             this->control.gait_frequency}});

}

void RL_Real::RunModel()
{
    const auto activation = LoadLWPolicyActivation();
    if (!activation || !activation->definition)
    {
        return;
    }
    if (!inference_activation_
        || inference_activation_->generation
            != activation->generation)
    {
        inference_activation_ = activation;
        ResetInferenceWorkspace(*activation);
    }

    LWPolicyInputSnapshot policy_input;
    if (!policy_input_snapshot_.read(policy_input))
    {
        return;
    }
    const RobotState<float>& local_state =
        policy_input.robot_state;
    const LWControlSnapshot& local_control =
        policy_input.control;

    inference_motion_reference_ = LoadLWMotionReference();
    const auto observations =
        activation->definition->params.Get<std::vector<std::string>>(
            "observations");
    const bool needs_motion_reference =
        std::find(
            observations.begin(),
            observations.end(),
            "whole_body_tracking/motion_command")
        != observations.end();
    if (needs_motion_reference
        && (!inference_motion_reference_
            || inference_motion_reference_->generation
                != activation->generation))
    {
        return;
    }
    const auto output_source_time =
        std::chrono::steady_clock::now();

    ++inference_frame_;

#ifdef ADD_ANGVEL_NOISE
    static std::default_random_engine generator;
    std::normal_distribution<float> distribution(0.0, 0.4f);
    inference_obs_.ang_vel.resize(
        local_state.imu.gyroscope.size());
    for (size_t i = 0;
         i < local_state.imu.gyroscope.size();
         ++i)
    {
        inference_obs_.ang_vel[i] =
            local_state.imu.gyroscope[i]
            + distribution(generator);
    }
#else
    inference_obs_.ang_vel = local_state.imu.gyroscope;
#endif
    inference_obs_.commands =
        joystick_fault_latch_.faulted()
        ? std::vector<float>{0.0f, 0.0f, 0.0f}
        : std::vector<float>{
            local_control.x,
            local_control.y,
            local_control.yaw};
    inference_obs_.base_quat = local_state.imu.quaternion;
    inference_obs_.dof_pos = local_state.motor_state.q;

#ifdef ADD_JOINTVEL_NOISE
    static std::default_random_engine velocity_generator;
    std::normal_distribution<float> velocity_distribution(
        0.0,
        1.5f);
    inference_obs_.dof_vel.resize(
        local_state.motor_state.dq.size());
    for (size_t i = 0;
         i < local_state.motor_state.dq.size();
         ++i)
    {
        inference_obs_.dof_vel[i] =
            local_state.motor_state.dq[i]
            + velocity_distribution(velocity_generator);
    }
#else
    inference_obs_.dof_vel = local_state.motor_state.dq;
#endif

    const YamlParams& policy_params =
        activation->definition->params;
    inference_gait_phase_time_ +=
        policy_params.Get<float>("dt")
        * policy_params.Get<int>("decimation")
        * local_control.gait_frequency;
    if (inference_gait_phase_time_ >= 1.0f)
    {
        inference_gait_phase_time_ -= 1.0f;
    }
    const float command_norm = std::sqrt(
        local_control.x * local_control.x
        + local_control.y * local_control.y
        + local_control.yaw * local_control.yaw);
    const float is_moving =
        command_norm > 0.1f ? 1.0f : 0.0f;
    inference_obs_.gait_phase = {
        is_moving * std::sin(
            2 * static_cast<float>(M_PI)
            * inference_gait_phase_time_),
        is_moving * std::cos(
            2 * static_cast<float>(M_PI)
            * inference_gait_phase_time_)};

    inference_obs_.actions = Forward();

#ifdef ENABLE_FORWARD_LATENCY
    std::this_thread::sleep_for(std::chrono::milliseconds(35));
#endif

    ComputeLWOutput(
        policy_params,
        inference_obs_,
        inference_obs_.actions,
        inference_output_dof_pos_,
        inference_output_dof_vel_,
        inference_output_dof_tau_);

    TorqueProtect(inference_output_dof_tau_, policy_params);
    PublishLWPolicyOutput(
        {activation->generation,
         0,
         inference_frame_,
         output_source_time,
         inference_output_dof_pos_,
         inference_output_dof_vel_,
         inference_output_dof_tau_});
    PublishLWPolicyProgress(
        activation->generation,
        inference_frame_);

#ifdef CSV_LOGGER
    this->CSVLogger(
        inference_output_dof_tau_,
        local_state.motor_state.tau_est,
        inference_obs_.dof_pos,
        inference_output_dof_pos_,
        inference_obs_.dof_vel);
#endif
}

std::vector<float> RL_Real::Forward()
{
    if (!inference_activation_
        || !inference_activation_->definition
        || !inference_activation_->definition->model)
    {
        return {};
    }
    const auto& definition =
        *inference_activation_->definition;
    const auto& policy_params = definition.params;
    const auto clamped_obs = ComputeLWObservation(
        policy_params,
        inference_obs_,
        inference_obs_dims_,
        inference_motion_reference_.get(),
        inference_frame_,
        inference_activation_->motion_length);

    std::vector<float> actions;
    const auto history_indices =
        policy_params.Get<std::vector<int>>(
            "observations_history");
    if (!history_indices.empty())
    {
        if (inference_frame_ == 1)
        {
            inference_history_obs_buf_.reset(
                {0},
                clamped_obs);
        }
        inference_history_obs_buf_.insert(clamped_obs);
        inference_history_obs_ =
            inference_history_obs_buf_.get_obs_vec(
                history_indices);
        actions = definition.model->forward(
            {inference_history_obs_});
    }
    else
    {
        actions = definition.model->forward({clamped_obs});
    }

    const auto upper =
        policy_params.Get<std::vector<float>>(
            "clip_actions_upper");
    const auto lower =
        policy_params.Get<std::vector<float>>(
            "clip_actions_lower");
    if (!upper.empty() && !lower.empty())
    {
        return clamp(actions, lower, upper);
    }
    return actions;
}

void RL_Real::ResetInferenceWorkspace(
    const LWPolicyActivation& activation)
{
    const YamlParams& policy_params =
        activation.definition->params;
    const size_t num_dofs = static_cast<size_t>(
        policy_params.Get<int>("num_of_dofs"));

    inference_frame_ = 0;
    inference_gait_phase_time_ = 0.0f;
    inference_motion_reference_.reset();
    inference_obs_ = {};
    inference_obs_.lin_vel = {0.0f, 0.0f, 0.0f};
    inference_obs_.ang_vel = {0.0f, 0.0f, 0.0f};
    inference_obs_.gravity_vec = {0.0f, 0.0f, -1.0f};
    inference_obs_.commands = {0.0f, 0.0f, 0.0f};
    inference_obs_.base_quat = {1.0f, 0.0f, 0.0f, 0.0f};
    inference_obs_.dof_pos =
        policy_params.Get<std::vector<float>>(
            "default_dof_pos");
    inference_obs_.dof_vel.assign(num_dofs, 0.0f);
    inference_obs_.actions.assign(num_dofs, 0.0f);
    inference_obs_.gait_phase = {0.0f, 1.0f};
    inference_output_dof_pos_ = inference_obs_.dof_pos;
    inference_output_dof_vel_.assign(num_dofs, 0.0f);
    inference_output_dof_tau_.assign(num_dofs, 0.0f);
    inference_history_obs_.clear();

    ComputeLWObservation(
        policy_params,
        inference_obs_,
        inference_obs_dims_,
        nullptr,
        0,
        activation.motion_length);
    const auto history_indices =
        policy_params.Get<std::vector<int>>(
            "observations_history");
    if (!history_indices.empty())
    {
        const int history_length =
            *std::max_element(
                history_indices.begin(),
                history_indices.end())
            + 1;
        inference_history_obs_buf_ = ObservationBuffer(
            1,
            inference_obs_dims_,
            history_length,
            policy_params.Get<std::string>(
                "observations_history_priority"));
    }
}

// void RL_Real::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
// {
//     this->imu = *msg;
// }

void RL_Real::GetState(RobotState<float> *state)
{
    if (mj_data)
    {
        // xml的sensor顺序为： jointpos jointvel jointtorque quat gyro accmeter
        state->imu.quaternion[0] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 0];
        state->imu.quaternion[1] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 1];
        state->imu.quaternion[2] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 2];
        state->imu.quaternion[3] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 3];

        state->imu.gyroscope[0] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 4];
        state->imu.gyroscope[1] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 5];
        state->imu.gyroscope[2] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 6];

        for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
        {
            state->motor_state.q[i] = mj_data->sensordata[this->params.Get<std::vector<int>>("joint_mapping")[i]];
            state->motor_state.dq[i] = mj_data->sensordata[this->params.Get<std::vector<int>>("joint_mapping")[i] + this->params.Get<int>("num_of_dofs")];
            state->motor_state.tau_est[i] = mj_data->sensordata[this->params.Get<std::vector<int>>("joint_mapping")[i] + 2 * this->params.Get<int>("num_of_dofs")];
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
    // this->lw_sdk.SendCmdData(this->lw_low_command);

    if (mj_data)
    {
        const auto activation =
            this->LoadLWPolicyActivation();
        for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
        {
            float target_tau = 0.0f;
            
            // 判断当前关节是否需要被执行器网络接管
            bool is_actuator_net_controlled = false;
            if (this->use_actuator_net_
                && activation
                && actuator_net_generation_
                    == activation->generation) {
                if (std::find(this->leg_train_indices.begin(), this->leg_train_indices.end(), i) != this->leg_train_indices.end()) {
                    is_actuator_net_controlled = true;
                }
                if (std::find(this->foot_train_indices.begin(), this->foot_train_indices.end(), i) != this->foot_train_indices.end()) {
                    is_actuator_net_controlled = true;
                }
            }

            if (is_actuator_net_controlled) {
                // 使用执行器网络的输出力矩 (附加可能存在的前馈 FF 力矩)
                target_tau = this->actuator_net_tau_[i] + command->motor_command.tau[i]; 
            } else {
                // 使用理想的 MuJoCo PD 力矩 (用于轮子或其他没被接管的关节)
                target_tau = command->motor_command.tau[i] +
                    command->motor_command.kp[i] * (command->motor_command.q[i] - mj_data->sensordata[this->params.Get<std::vector<int>>("joint_mapping")[i]]) +
                    command->motor_command.kd[i] * (command->motor_command.dq[i] - mj_data->sensordata[this->params.Get<std::vector<int>>("joint_mapping")[i] + this->params.Get<int>("num_of_dofs")]);
            }

            // 限幅并写入 MuJoCo
            mj_data->ctrl[this->params.Get<std::vector<int>>("joint_mapping")[i]] = 
                clamp<float>(target_tau, -this->params.Get<std::vector<float>>("torque_limits")[i], this->params.Get<std::vector<float>>("torque_limits")[i]);
        }
    }
}

void RL_Real::SetupSysJoystick(const std::string& device, int bits)
{
    this->sys_js = std::make_unique<LWJoystickDevice>(device);
    if (!this->sys_js->isFound())
    {
        std::cout << LOGGER::ERROR << "Joystick [" << device << "] open failed." << std::endl;
        LatchJoystickFault(
            {LWJoystickSampleStatus::Disconnected, EBADF, -1});
    }

    this->sys_js_max_value = (1 << (bits - 1));
}

void RL_Real::LatchJoystickFault(
    const LWJoystickSampleResult& result) noexcept
{
    LWClearJoystickState(this->sys_js_button, this->sys_js_axis);
    joystick_input_mailbox_.clear(Input::Gamepad::None);
    this->sys_js_active = false;

    if (joystick_fault_latch_.latch())
    {
        std::cerr << LOGGER::ERROR
                  << "[Safety] Joystick input latched unavailable"
                  << ", status=" << static_cast<int>(result.status)
                  << ", errno=" << result.error_number
                  << ", bytes=" << result.bytes_read
                  << ". Velocity commands will remain zero and the current "
                     "simulation FSM will remain active. Restart is required "
                     "to restore joystick input."
                  << std::endl;
    }
}

void RL_Real::ApplyPendingInput()
{
    const auto input = joystick_input_mailbox_.read();
    this->control.x = input.x;
    this->control.y = input.y;
    this->control.yaw = input.yaw;
    if (input.event_sequence != consumed_gamepad_sequence_)
    {
        consumed_gamepad_sequence_ = input.event_sequence;
        this->control.SetGamepad(input.event);
    }
}

void RL_Real::ApplyJoystickFaultGate() noexcept
{
    if (!joystick_fault_latch_.faulted())
    {
        return;
    }

    this->control.x = 0.0f;
    this->control.y = 0.0f;
    this->control.yaw = 0.0f;
    this->control.current_gamepad = Input::Gamepad::None;
    this->control.last_gamepad = Input::Gamepad::None;
}

void RL_Real::GetSysJoystick()
{
    LWBeginJoystickCycle(this->sys_js_button);

    if (joystick_fault_latch_.faulted())
    {
        return;
    }

    if (!this->sys_js || !this->sys_js->isFound())
    {
        LatchJoystickFault(
            {LWJoystickSampleStatus::Disconnected, EBADF, -1});
        return;
    }

    while (true)
    {
        const LWJoystickSampleResult sample_result =
            this->sys_js->sample(&this->sys_js_event);
        if (sample_result.status == LWJoystickSampleStatus::NoData)
        {
            break;
        }
        if (!sample_result.hasEvent())
        {
            LatchJoystickFault(sample_result);
            return;
        }

        const LWJoystickEventResult event_result =
            LWApplyJoystickEvent(
                this->sys_js_event,
                this->sys_js_button,
                this->sys_js_axis,
                this->sys_js_max_value,
                this->axis_deadzone);
        if (event_result == LWJoystickEventResult::ButtonOutOfRange
            || event_result == LWJoystickEventResult::AxisOutOfRange)
        {
            static size_t invalid_event_count = 0;
            ++invalid_event_count;
            if (invalid_event_count == 1 || invalid_event_count % 100 == 0)
            {
                std::cerr << LOGGER::WARNING
                          << "[Safety] Ignoring out-of-range joystick event"
                          << ", type=" << static_cast<int>(this->sys_js_event.type)
                          << ", number=" << static_cast<int>(this->sys_js_event.number)
                          << ", occurrences=" << invalid_event_count
                          << std::endl;
            }
        }
        else if (event_result == LWJoystickEventResult::InvalidConfiguration)
        {
            LatchJoystickFault(
                {LWJoystickSampleStatus::Error, EINVAL, 0});
            return;
        }
    }

#ifdef JOYSTICK_1
    // 修改为自己的手柄映射
    if (this->sys_js_button[0].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::A);
    if (this->sys_js_button[1].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::B);
    if (this->sys_js_button[3].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::X);
    if (this->sys_js_button[4].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::Y);
    if (this->sys_js_button[6].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB);
    if (this->sys_js_button[7].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::RB);
    if (this->sys_js_button[13].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::LStick);
    if (this->sys_js_button[14].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::RStick);
    if (this->sys_js_axis[7] < 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::DPadUp);
    if (this->sys_js_axis[7] > 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::DPadDown);
    if (this->sys_js_axis[6] < 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::DPadLeft);
    if (this->sys_js_axis[6] > 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::DPadRight);
    if (this->sys_js_button[6].pressed && this->sys_js_button[0].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB_A);
    if (this->sys_js_button[6].pressed && this->sys_js_button[1].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB_B);
    if (this->sys_js_button[6].pressed && this->sys_js_button[3].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB_X);
    if (this->sys_js_button[6].pressed && this->sys_js_button[4].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB_Y);
    if (this->sys_js_button[6].pressed && this->sys_js_button[13].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB_LStick);
    if (this->sys_js_button[6].pressed && this->sys_js_button[14].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB_RStick);
    if (this->sys_js_button[6].pressed && this->sys_js_axis[7] < 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB_DPadUp);
    if (this->sys_js_button[6].pressed && this->sys_js_axis[7] > 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB_DPadDown);
    if (this->sys_js_button[6].pressed && this->sys_js_axis[6] > 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB_DPadRight);
    if (this->sys_js_button[6].pressed && this->sys_js_axis[6] < 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB_DPadLeft);
    if (this->sys_js_button[7].pressed && this->sys_js_button[0].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::RB_A);
    if (this->sys_js_button[7].pressed && this->sys_js_button[1].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::RB_B);
    if (this->sys_js_button[7].pressed && this->sys_js_button[3].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::RB_X);
    if (this->sys_js_button[7].pressed && this->sys_js_button[4].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::RB_Y);
    if (this->sys_js_button[7].pressed && this->sys_js_button[13].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::RB_LStick);
    if (this->sys_js_button[7].pressed && this->sys_js_button[14].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::RB_RStick);
    if (this->sys_js_button[7].pressed && this->sys_js_axis[7] < 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::RB_DPadUp);
    if (this->sys_js_button[7].pressed && this->sys_js_axis[7] > 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::RB_DPadDown);
    if (this->sys_js_button[7].pressed && this->sys_js_axis[6] > 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::RB_DPadRight);
    if (this->sys_js_button[7].pressed && this->sys_js_axis[6] < 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::RB_DPadLeft);
    if (this->sys_js_button[6].pressed && this->sys_js_button[7].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB_RB);

    // 通过sys_js_max_value将各项指令归一化
    float ly = (-float(this->sys_js_axis[1]) / float(this->sys_js_max_value)) * this->params.Get<std::vector<float>>("vel_command")[0];
    float lx = (-float(this->sys_js_axis[0]) / float(this->sys_js_max_value)) * this->params.Get<std::vector<float>>("vel_command")[1];
    float rx = (-float(this->sys_js_axis[2]) / float(this->sys_js_max_value)) * this->params.Get<std::vector<float>>("vel_command")[2];

#endif

#ifdef JOYSTICK_2
    // 修改为自己的手柄映射
    if (this->sys_js_button[0].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::A);
    if (this->sys_js_button[1].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::B);
    if (this->sys_js_button[2].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::X);
    if (this->sys_js_button[3].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::Y);
    if (this->sys_js_button[4].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB);
    if (this->sys_js_button[5].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::RB);
    if (this->sys_js_button[9].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::LStick);
    if (this->sys_js_button[10].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::RStick);
    if (this->sys_js_axis[7] < 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::DPadUp);
    if (this->sys_js_axis[7] > 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::DPadDown);
    if (this->sys_js_axis[6] < 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::DPadLeft);
    if (this->sys_js_axis[6] > 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::DPadRight);
    if (this->sys_js_button[4].pressed && this->sys_js_button[0].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB_A);
    if (this->sys_js_button[4].pressed && this->sys_js_button[1].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB_B);
    if (this->sys_js_button[4].pressed && this->sys_js_button[2].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB_X);
    if (this->sys_js_button[4].pressed && this->sys_js_button[3].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB_Y);
    if (this->sys_js_button[4].pressed && this->sys_js_button[9].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB_LStick);
    if (this->sys_js_button[4].pressed && this->sys_js_button[10].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB_RStick);
    if (this->sys_js_button[4].pressed && this->sys_js_axis[7] < 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB_DPadUp);
    if (this->sys_js_button[4].pressed && this->sys_js_axis[7] > 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB_DPadDown);
    if (this->sys_js_button[4].pressed && this->sys_js_axis[6] > 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB_DPadRight);
    if (this->sys_js_button[4].pressed && this->sys_js_axis[6] < 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB_DPadLeft);
    if (this->sys_js_button[5].pressed && this->sys_js_button[0].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::RB_A);
    if (this->sys_js_button[5].pressed && this->sys_js_button[1].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::RB_B);
    if (this->sys_js_button[5].pressed && this->sys_js_button[2].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::RB_X);
    if (this->sys_js_button[5].pressed && this->sys_js_button[3].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::RB_Y);
    if (this->sys_js_button[5].pressed && this->sys_js_button[9].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::RB_LStick);
    if (this->sys_js_button[5].pressed && this->sys_js_button[10].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::RB_RStick);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[7] < 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::RB_DPadUp);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[7] > 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::RB_DPadDown);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[6] > 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::RB_DPadRight);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[6] < 0) joystick_input_mailbox_.publishEvent(Input::Gamepad::RB_DPadLeft);
    if (this->sys_js_button[4].pressed && this->sys_js_button[5].on_press) joystick_input_mailbox_.publishEvent(Input::Gamepad::LB_RB);

    // 通过sys_js_max_value将各项指令归一化
    // float ly = -float(this->sys_js_axis[1]) / float(this->sys_js_max_value);
    // float lx = -float(this->sys_js_axis[0]) / float(this->sys_js_max_value);
    // float rx = -float(this->sys_js_axis[3]) / float(this->sys_js_max_value);

    float ly = (-float(this->sys_js_axis[1]) / float(this->sys_js_max_value)) * this->params.Get<std::vector<float>>("vel_command")[0];
    float lx = (-float(this->sys_js_axis[0]) / float(this->sys_js_max_value)) * this->params.Get<std::vector<float>>("vel_command")[1];
    float rx = (-float(this->sys_js_axis[3]) / float(this->sys_js_max_value)) * this->params.Get<std::vector<float>>("vel_command")[2];
#endif

    bool has_input = (ly != 0.0f || lx != 0.0f || rx != 0.0f);

    if (has_input)
    {
        joystick_input_mailbox_.publishVelocity(ly, lx, rx);
        this->sys_js_active = true;
    }
    else if (this->sys_js_active)
    {
        joystick_input_mailbox_.publishVelocity(
            0.0f,
            0.0f,
            0.0f);
        this->sys_js_active = false;
    }

    // // =====================[手柄一键触发速度追踪测试] =====================
    // static double test_start_time = -1.0;

    // // 设定触发按键
    // // 当你按下该键，且当前没有在测试中时，触发实验
    // if (this->sys_js_button[3].on_press && test_start_time < 0) {
    //     test_start_time = this->mj_data->time; // 记录测试起点的绝对仿真时间
    //     std::cout << "\n[INFO] 🚀 自动速度追踪实验开始！" << std::endl;
    // }

    // if (test_start_time > 0) {
    //     // 计算实验已经进行的时间
    //     double t = this->mj_data->time - test_start_time;

    //     // 设计经典的多段阶跃曲线 (Staircase Command)
    //     if (t < 1.0) {
    //         this->control.x = 0.0f;   // 原地站立预备
    //     } 
    //     else if (t < 4.0) {
    //         this->control.x = 0.3f;   // 前向巡航加速 
    //     }
    //     else if (t < 7.0) {
    //         this->control.x = 0.6f;   // 前向巡航加速 
    //     }
    //     else if (t < 10.0) {
    //         this->control.x = 0.9f;   // 前向巡航加速 
    //     }
    //     else if (t < 13.0) {
    //         this->control.x = 1.2f;   // 前向巡航加速 
    //     }
    //     else if (t < 16.0) {
    //         this->control.x = 1.5f;   // 前向巡航加速 
    //     }
    //     else {
    //         this->control.x = 0.0f;   // 实验结束
    //         this->control.yaw = 0.0f;
    //         test_start_time = -1.0;   // 状态复位，允许按键再次触发
    //         std::cout << "[INFO] ✅ 自动速度追踪实验结束！" << std::endl;
    //     }

    //     this->control.y = 0.0f;
    //     // this->control.yaw = 0.0f;
    //     this->sys_js_active = true;
    // } else {
    //     // 非测试状态下，保持静止
    //     this->control.x = 0.0f;
    //     this->control.y = 0.0f;
    //     this->control.yaw = 0.0f;
    //     this->sys_js_active = false;
    // }
    
    // // =====================[手柄一键触发冲击测试] =====================
    // // 设定冲击力量级 (例如 150 牛顿)
    // double impact_magnitude = 200.0; 
    
    // // // 触发右侧向冲击 
    // // if (this->sys_js_button[3].on_press && g_impact_start_time < 0) {
    // //     g_impact_start_time = this->mj_data->time;
    // //     g_impact_force_x = 0.0;
    // //     g_impact_force_y = impact_magnitude;  
    // //     std::cout << "\n[INFO] 💥 触发侧向(Y+)冲击力: " << impact_magnitude << " N" << std::endl;
    // // }
    // // // 触发左侧向冲击 
    // // if (this->sys_js_button[3].on_press && g_impact_start_time < 0) {
    // //     g_impact_start_time = this->mj_data->time;
    // //     g_impact_force_x = 0.0;
    // //     g_impact_force_y = -impact_magnitude;  
    // //     std::cout << "\n[INFO] 💥 触发侧向(Y-)冲击力: " << -impact_magnitude << " N" << std::endl;
    // // }
    // // 触发前方冲击
    // if (this->sys_js_button[3].on_press && g_impact_start_time < 0) {
    //     g_impact_start_time = this->mj_data->time;
    //     g_impact_force_x = -impact_magnitude;
    //     g_impact_force_y = 0.0;  
    //     std::cout << "\n[INFO] 💥 触发后向(X-)冲击力: " << -impact_magnitude << " N" << std::endl;
    // }
    // ============================================================================

    // if (this->control.current_gamepad == Input::Gamepad::DPadUp )
    // {
    //     if (!this->control.dpad_handled) { // 如果还没处理过
    //         this->control.gait_frequency += 0.1f;
    //         this->control.dpad_handled = true; // 标记为已处理
    //     }
    // }
    // else if (this->control.current_gamepad == Input::Gamepad::DPadDown )
    // {
    //     if (!this->control.dpad_handled) { // 如果还没处理过
    //         this->control.gait_frequency -= 0.1f;
    //         this->control.dpad_handled = true; // 标记为已处理
    //     }
    // }
    // else 
    // {
    //     this->control.dpad_handled = false;
    // }
}

// Signal handler for Ctrl+C
void signalHandler(int signum)
{
    std::cout << LOGGER::INFO << "Received signal " << signum << ", exiting..." << std::endl;
    if (RL_Real::instance && RL_Real::instance->sim)
    {
        RL_Real::instance->sim->exitrequest.store(1);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto rl_sar = std::make_shared<RL_Real>(argc, argv);
    std::thread ros_thread([&]() {
        rclcpp::spin(rl_sar->ros2_node);
    });

    signal(SIGINT, signalHandler);
    if (rl_sar->sim) {
        rl_sar->sim->RenderLoop(); 
    }

    rclcpp::shutdown();
    if (ros_thread.joinable()) {
        ros_thread.join();
    }
    return 0;
}
