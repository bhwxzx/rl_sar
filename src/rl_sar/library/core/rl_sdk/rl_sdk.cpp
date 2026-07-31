/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_sdk.hpp"

namespace
{
const char* LWPolicyOutputStatusName(
    LWPolicyOutputStatus status) noexcept
{
    switch (status)
    {
    case LWPolicyOutputStatus::Ready:
        return "ready";
    case LWPolicyOutputStatus::Missing:
        return "missing";
    case LWPolicyOutputStatus::GenerationMismatch:
        return "generation_mismatch";
    case LWPolicyOutputStatus::Incomplete:
        return "incomplete";
    case LWPolicyOutputStatus::Stale:
        return "stale";
    }
    return "unknown";
}
} // namespace

bool LWPolicyOutputPayloadComplete(
    const LWPolicyOutputFrame& output,
    size_t expected_dofs) noexcept
{
    return output.dof_pos.size() == expected_dofs
        && output.dof_vel.size() == expected_dofs
        && output.dof_tau.size() == expected_dofs;
}

LWPolicyOutputStatus EvaluateLWPolicyOutput(
    const LWPolicyOutputFrame* output,
    std::uint64_t active_generation,
    size_t expected_dofs,
    std::chrono::steady_clock::time_point now,
    std::chrono::steady_clock::duration max_age) noexcept
{
    if (output == nullptr)
    {
        return LWPolicyOutputStatus::Missing;
    }
    if (output->generation != active_generation)
    {
        return LWPolicyOutputStatus::GenerationMismatch;
    }
    if (output->sequence == 0
        || output->source_time.time_since_epoch().count() == 0
        || !LWPolicyOutputPayloadComplete(*output, expected_dofs))
    {
        return LWPolicyOutputStatus::Incomplete;
    }
    if (now < output->source_time
        || now - output->source_time > max_age)
    {
        return LWPolicyOutputStatus::Stale;
    }
    return LWPolicyOutputStatus::Ready;
}

bool LWPolicyOutputTransport::publish(
    LWPolicyOutputFrame output,
    std::uint64_t active_generation,
    size_t expected_dofs)
{
    if (output.generation != active_generation
        || output.source_time.time_since_epoch().count() == 0
        || !LWPolicyOutputPayloadComplete(output, expected_dofs))
    {
        return false;
    }
    output.sequence = next_sequence_.fetch_add(
        1,
        std::memory_order_relaxed);
    latest_.store(
        std::make_shared<LWPolicyOutputFrame>(
            std::move(output)));
    return true;
}

std::shared_ptr<const LWPolicyOutputFrame>
LWPolicyOutputTransport::load() const noexcept
{
    return latest_.load();
}

void LWPolicyOutputTransport::clear() noexcept
{
    latest_.clear();
}

void RL::StateController(const RobotState<float>* state, RobotCommand<float>* command)
{
    auto updateState = [&](std::shared_ptr<FSMState> statePtr)
    {
        if (auto rl_fsm_state = std::dynamic_pointer_cast<RLFSMState>(statePtr))
        {
            rl_fsm_state->fsm_state = state;
            rl_fsm_state->fsm_command = command;
        }
    };
    for (auto& pair : fsm.states_)
    {
        updateState(pair.second);
    }

    fsm.Run();

    this->motiontime++;

    if (this->control.current_keyboard == Input::Keyboard::W)
    {
        this->control.x += 0.1f;
    }
    if (this->control.current_keyboard == Input::Keyboard::S)
    {
        this->control.x -= 0.1f;
    }
    if (this->control.current_keyboard == Input::Keyboard::A)
    {
        this->control.y += 0.1f;
    }
    if (this->control.current_keyboard == Input::Keyboard::D)
    {
        this->control.y -= 0.1f;
    }
    if (this->control.current_keyboard == Input::Keyboard::Q)
    {
        this->control.yaw += 0.1f;
    }
    if (this->control.current_keyboard == Input::Keyboard::E)
    {
        this->control.yaw -= 0.1f;
    }
    if (this->control.current_keyboard == Input::Keyboard::Space)
    {
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
    }
    if (this->control.current_keyboard == Input::Keyboard::N || this->control.current_gamepad == Input::Gamepad::X)
    {
        this->control.navigation_mode = !this->control.navigation_mode;
        std::cout << std::endl << LOGGER::INFO << "Navigation mode: " << (this->control.navigation_mode ? "ON" : "OFF") << std::endl;
    }
}

std::vector<float> RL::ComputeObservation()
{
    std::vector<std::vector<float>> obs_list;

    for (const std::string &observation : this->params.Get<std::vector<std::string>>("observations"))
    {
        // ============= Base Observations =============
        if (observation == "lin_vel")
        {
            obs_list.push_back(this->obs.lin_vel * this->params.Get<float>("lin_vel_scale"));
        }
        else if (observation == "ang_vel")
        {
            // In ROS1 Gazebo, the coordinate system for angular velocity is in the world coordinate system.
            // In ROS2 Gazebo, mujoco and real robot, the coordinate system for angular velocity is in the body coordinate system.
            if (this->ang_vel_axis == "body")
            {
                obs_list.push_back(this->obs.ang_vel * this->params.Get<float>("ang_vel_scale"));
            }
            else if (this->ang_vel_axis == "world")
            {
                obs_list.push_back(QuatRotateInverse(this->obs.base_quat, this->obs.ang_vel) * this->params.Get<float>("ang_vel_scale"));
            }
        }
        else if (observation == "gravity_vec")
        {
            obs_list.push_back(QuatRotateInverse(this->obs.base_quat, this->obs.gravity_vec));
        }
        else if (observation == "commands")
        {
            obs_list.push_back(this->obs.commands * this->params.Get<std::vector<float>>("commands_scale"));
        }
        else if (observation == "dof_pos")
        {
            // 观测的是相对默认位置值
            std::vector<float> dof_pos_rel = this->obs.dof_pos - this->params.Get<std::vector<float>>("default_dof_pos");
            // 对于轮子相对位置大小为0
            for (int i : this->params.Get<std::vector<int>>("wheel_indices"))
            {
                dof_pos_rel[i] = 0.0f;
            }
            obs_list.push_back(dof_pos_rel * this->params.Get<float>("dof_pos_scale"));
        }
        else if (observation == "dof_vel")
        {
            obs_list.push_back(this->obs.dof_vel * this->params.Get<float>("dof_vel_scale"));
        }
        else if (observation == "actions")
        {
            obs_list.push_back(this->obs.actions);
        }
        // ============= Other Observations =============
        else if (observation == "gait_phase")
        {
            obs_list.push_back(this->obs.gait_phase);
        }
        // else if (observation == "gait_command")
        // {
        //     obs_list.push_back(this->obs.gait_command);
        // }
        else if (observation == "whole_body_tracking/motion_command")
        {
            std::vector<float> motion_cmd;
            if (this->robot_name=="g1" && this->motion_loader)
            {
                auto joint_pos_sdk = this->motion_loader->GetJointPos();
                auto joint_vel_sdk = this->motion_loader->GetJointVel();
                auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
                std::vector<float> joint_pos_training(joint_mapping.size());
                std::vector<float> joint_vel_training(joint_mapping.size());
                for (size_t i = 0; i < joint_mapping.size(); ++i)
                {
                    joint_pos_training[i] = joint_pos_sdk[joint_mapping[i]];
                    joint_vel_training[i] = joint_vel_sdk[joint_mapping[i]];
                }
                motion_cmd.insert(motion_cmd.end(), joint_pos_training.begin(), joint_pos_training.end());
                motion_cmd.insert(motion_cmd.end(), joint_vel_training.begin(), joint_vel_training.end());
            }
            else if(this->robot_name=="LW" && this->motion_loader_lw)
            {
                auto joint_pos_sdk = this->motion_loader_lw->GetJointPos();
                auto joint_vel_sdk = this->motion_loader_lw->GetJointVel();
                auto joint_mapping = this->params.Get<std::vector<int>>("motion_joint_mapping");
                std::vector<float> joint_pos_training(joint_mapping.size());
                std::vector<float> joint_vel_training(joint_mapping.size());
                for (size_t i = 0; i < joint_mapping.size(); ++i)
                {
                    joint_pos_training[i] = joint_pos_sdk[joint_mapping[i]];
                    joint_vel_training[i] = joint_vel_sdk[joint_mapping[i]];
                }
                motion_cmd.insert(motion_cmd.end(), joint_pos_training.begin(), joint_pos_training.end());
                motion_cmd.insert(motion_cmd.end(), joint_vel_training.begin(), joint_vel_training.end());
            }
            else
            {
                motion_cmd.resize(this->params.Get<int>("num_of_dofs") * 2, 0.0f);
            }
            obs_list.push_back(motion_cmd);
        }
        else if (observation == "whole_body_tracking/motion_anchor_ori_b")
        {
            std::vector<float> anchor_ori(6, 0.0f);
            if (this->robot_name=="g1" && this->motion_loader)
            {
                auto waist_sdk_indices = this->params.Get<std::vector<int>>("waist_joint_indices");
                std::vector<float> waist_angles = {
                    this->obs.dof_pos[InverseJointMapping(waist_sdk_indices[0])],
                    this->obs.dof_pos[InverseJointMapping(waist_sdk_indices[1])],
                    this->obs.dof_pos[InverseJointMapping(waist_sdk_indices[2])]
                };
                std::vector<float> robot_torso_quat_w = MotionLoader::ComputeTorsoQuat(this->obs.base_quat, waist_angles);
                std::vector<float> ref_torso_quat_w = this->motion_loader->GetAnchorQuat();
                std::vector<float> init_quat = this->motion_loader->GetInitQuat();
                std::vector<float> motion_anchor_quat_w = QuaternionMultiply(init_quat, ref_torso_quat_w);
                std::vector<float> robot_quat_inv = QuaternionConjugate(robot_torso_quat_w);
                std::vector<float> relative_quat = QuaternionMultiply(robot_quat_inv, motion_anchor_quat_w);
                std::vector<float> rot_matrix = QuaternionToRotationMatrix(relative_quat);
                anchor_ori = MatrixFirstTwoColumns(rot_matrix);
            }
            else if(this->robot_name=="LW" && this->motion_loader_lw)
            {
                std::vector<float> robot_torso_quat_w = MotionLoaderLW::ComputeTorsoQuat(this->obs.base_quat);
                std::vector<float> ref_torso_quat_w = this->motion_loader_lw->GetAnchorQuat();
                std::vector<float> init_quat = this->motion_loader_lw->GetInitQuat();
                std::vector<float> motion_anchor_quat_w = QuaternionMultiply(init_quat, ref_torso_quat_w);
                std::vector<float> robot_quat_inv = QuaternionConjugate(robot_torso_quat_w);
                std::vector<float> relative_quat = QuaternionMultiply(robot_quat_inv, motion_anchor_quat_w);
                std::vector<float> rot_matrix = QuaternionToRotationMatrix(relative_quat);
                anchor_ori = MatrixFirstTwoColumns(rot_matrix);
            }
            obs_list.push_back(anchor_ori);
        }
        else if (observation == "RoboMimic_Deploy/phase")
        {
            float motion_time = this->episode_length_buf * this->params.Get<float>("dt") * this->params.Get<int>("decimation");
            float count = motion_time;
            float phase = count / this->motion_length;
            std::vector<float> phase_vec = {phase};
            obs_list.push_back(phase_vec);
        }
    }

    this->obs_dims.clear();
    for (const auto& obs : obs_list)
    {
       this->obs_dims.push_back(obs.size());
    }

    std::vector<float> obs;
    for (const auto& obs_vec : obs_list)
    {
        obs.insert(obs.end(), obs_vec.begin(), obs_vec.end());
    }
    std::vector<float> clamped_obs = clamp(obs, -this->params.Get<float>("clip_obs"), this->params.Get<float>("clip_obs"));
    return clamped_obs;
}

std::vector<float> RL::ComputeLWObservation(
    const YamlParams& policy_params,
    Observations<float>& policy_obs,
    std::vector<int>& policy_obs_dims,
    const LWMotionReferenceSnapshot* motion_reference,
    std::uint64_t policy_frame,
    float motion_length) const
{
    std::vector<std::vector<float>> obs_list;
    for (const std::string& observation :
         policy_params.Get<std::vector<std::string>>("observations"))
    {
        if (observation == "ang_vel")
        {
            obs_list.push_back(
                policy_obs.ang_vel
                * policy_params.Get<float>("ang_vel_scale"));
        }
        else if (observation == "gravity_vec")
        {
            obs_list.push_back(
                QuatRotateInverse(
                    policy_obs.base_quat,
                    policy_obs.gravity_vec));
        }
        else if (observation == "commands")
        {
            obs_list.push_back(
                policy_obs.commands
                * policy_params.Get<std::vector<float>>("commands_scale"));
        }
        else if (observation == "dof_pos")
        {
            std::vector<float> dof_pos_rel =
                policy_obs.dof_pos
                - policy_params.Get<std::vector<float>>("default_dof_pos");
            for (int index :
                 policy_params.Get<std::vector<int>>("wheel_indices"))
            {
                dof_pos_rel[index] = 0.0f;
            }
            obs_list.push_back(
                dof_pos_rel
                * policy_params.Get<float>("dof_pos_scale"));
        }
        else if (observation == "dof_vel")
        {
            obs_list.push_back(
                policy_obs.dof_vel
                * policy_params.Get<float>("dof_vel_scale"));
        }
        else if (observation == "actions")
        {
            obs_list.push_back(policy_obs.actions);
        }
        else if (observation == "gait_phase")
        {
            obs_list.push_back(policy_obs.gait_phase);
        }
        else if (observation == "whole_body_tracking/motion_command")
        {
            std::vector<float> motion_command;
            if (motion_reference != nullptr)
            {
                const auto mapping =
                    policy_params.Get<std::vector<int>>(
                        "motion_joint_mapping");
                std::vector<float> joint_pos(mapping.size());
                std::vector<float> joint_vel(mapping.size());
                for (size_t i = 0; i < mapping.size(); ++i)
                {
                    joint_pos[i] =
                        motion_reference->joint_pos[mapping[i]];
                    joint_vel[i] =
                        motion_reference->joint_vel[mapping[i]];
                }
                motion_command.insert(
                    motion_command.end(),
                    joint_pos.begin(),
                    joint_pos.end());
                motion_command.insert(
                    motion_command.end(),
                    joint_vel.begin(),
                    joint_vel.end());
            }
            else
            {
                motion_command.resize(
                    policy_params.Get<int>("num_of_dofs") * 2,
                    0.0f);
            }
            obs_list.push_back(std::move(motion_command));
        }
        else if (observation
                 == "whole_body_tracking/motion_anchor_ori_b")
        {
            std::vector<float> anchor_orientation(6, 0.0f);
            if (motion_reference != nullptr)
            {
                const std::vector<float> robot_torso_quat =
                    MotionLoaderLW::ComputeTorsoQuat(
                        policy_obs.base_quat);
                const std::vector<float> motion_anchor_quat =
                    QuaternionMultiply(
                        motion_reference->init_quat,
                        motion_reference->anchor_quat);
                const std::vector<float> relative_quat =
                    QuaternionMultiply(
                        QuaternionConjugate(robot_torso_quat),
                        motion_anchor_quat);
                anchor_orientation = MatrixFirstTwoColumns(
                    QuaternionToRotationMatrix(relative_quat));
            }
            obs_list.push_back(std::move(anchor_orientation));
        }
        else if (observation == "RoboMimic_Deploy/phase")
        {
            const float policy_dt =
                policy_params.Get<float>("dt")
                * policy_params.Get<int>("decimation");
            const float phase =
                motion_length > 0.0f
                ? static_cast<float>(policy_frame) * policy_dt
                    / motion_length
                : 0.0f;
            obs_list.push_back({phase});
        }
    }

    policy_obs_dims.clear();
    std::vector<float> flattened;
    for (const auto& observation : obs_list)
    {
        policy_obs_dims.push_back(
            static_cast<int>(observation.size()));
        flattened.insert(
            flattened.end(),
            observation.begin(),
            observation.end());
    }
    const float clip = policy_params.Get<float>("clip_obs");
    return clamp(flattened, -clip, clip);
}

void RL::InitObservations()
{
    this->obs.lin_vel = {0.0f, 0.0f, 0.0f};
    this->obs.ang_vel = {0.0f, 0.0f, 0.0f};
    this->obs.gravity_vec = {0.0f, 0.0f, -1.0f};
    this->obs.commands = {0.0f, 0.0f, 0.0f};
    this->obs.base_quat = {0.0f, 0.0f, 0.0f, 1.0f};
    this->obs.dof_pos = this->params.Get<std::vector<float>>("default_dof_pos");
    this->obs.dof_vel.clear();
    this->obs.dof_vel.resize(this->params.Get<int>("num_of_dofs"), 0.0f);
    this->obs.actions.clear();
    this->obs.actions.resize(this->params.Get<int>("num_of_dofs"), 0.0f);
    this->obs.gait_phase = {0.0f, 1.0f}; // sin(0) cos(0)
    // this->obs.gait_command = this->params.Get<std::vector<float>>("gait_command");
    this->ComputeObservation();
}

void RL::InitOutputs()
{
    int num_of_dofs = this->params.Get<int>("num_of_dofs");
    this->output_dof_tau.clear();
    this->output_dof_tau.resize(num_of_dofs, 0.0f);
    this->output_dof_pos = this->params.Get<std::vector<float>>("default_dof_pos");
    this->output_dof_vel.clear();
    this->output_dof_vel.resize(num_of_dofs, 0.0f);
}

void RL::InitControl()
{
    this->control.x = 0.0f;
    this->control.y = 0.0f;
    this->control.yaw = 0.0f;
}

void RL::InitJointNum(size_t num_joints)
{
    this->robot_state.motor_state.resize(num_joints);
    this->start_state.motor_state.resize(num_joints);
    this->now_state.motor_state.resize(num_joints);
    this->robot_command.motor_command.resize(num_joints);
}

void RL::PreloadModel(const std::string& robot_config_path)
{
    // 读取对应的 YAML 拿到模型名字
    std::string config_path = std::string(POLICY_DIR) + "/" + robot_config_path + "/config.yaml";
    YAML::Node config;
    try {
        config = YAML::LoadFile(config_path)[robot_config_path];
    } catch (...) {
        std::cout << LOGGER::ERROR << "Failed to preload yaml: " << config_path << std::endl;
        return;
    }

    std::string model_name = config["model_name"].as<std::string>();
    std::string model_path = std::string(POLICY_DIR) + "/" + robot_config_path + "/" + model_name;
    
    std::cout << LOGGER::INFO << "Preloading ONNX model into memory: " << model_path << std::endl;
    
    // 加载 ONNX 到显存/内存
    auto model = InferenceRuntime::ModelFactory::load_model(model_path);
    if (!model) {
        std::cout << LOGGER::ERROR << "Failed to preload model!" << std::endl;
        return;
    }

    // 模型预热 (Warm-up)：把计算图彻底唤醒
    int num_obs = config["num_observations"].as<int>();
    int history_len = 1;
    if (config["observations_history"]) {
        auto hist_vec = config["observations_history"].as<std::vector<int>>();
        if (!hist_vec.empty()) {
            history_len = *std::max_element(hist_vec.begin(), hist_vec.end()) + 1;
        }
    }
    std::vector<float> dummy_input(num_obs * history_len, 0.0f);
    // --- 开始计时 ---
    auto start_time = std::chrono::steady_clock::now();
    for(int i = 0; i < 2; i++) {
        model->forward({dummy_input});
    }
    // --- 结束计时 ---
    auto end_time = std::chrono::steady_clock::now();
    // 计算耗时 (毫秒)
    std::chrono::duration<double, std::milli> duration_ms = end_time - start_time;
    
    // 打印预热耗时
    std::cout << LOGGER::INFO << "Model warmup took " 
              << std::fixed << std::setprecision(3) << duration_ms.count() 
              << " ms." << std::endl;

    // 存入字典缓存
    this->preloaded_models_[robot_config_path] = std::shared_ptr<InferenceRuntime::Model>(std::move(model));
    std::cout << LOGGER::INFO << "Preload & Warmup finished for: " << robot_config_path << std::endl;
}

void RL::PreloadLWPolicyContext(
    const std::string& robot_config_path)
{
    const std::string config_path =
        std::string(POLICY_DIR) + "/" + robot_config_path
        + "/config.yaml";
    const YAML::Node loaded =
        YAML::LoadFile(config_path)[robot_config_path];
    if (!loaded || !loaded.IsMap())
    {
        throw std::runtime_error(
            "Invalid LW policy configuration: " + config_path);
    }

    auto definition = std::make_shared<LWPolicyDefinition>();
    definition->path = robot_config_path;
    definition->params.config_node = YAML::Clone(
        this->params.config_node);
    for (auto it = loaded.begin(); it != loaded.end(); ++it)
    {
        const std::string key = it->first.as<std::string>();
        definition->params.config_node[key] = YAML::Clone(it->second);
    }

    const auto model_it =
        this->preloaded_models_.find(robot_config_path);
    if (model_it == this->preloaded_models_.end()
        || !model_it->second)
    {
        throw std::runtime_error(
            "LW policy model was not preloaded: "
            + robot_config_path);
    }
    definition->model = model_it->second;
    this->lw_policy_definitions_[robot_config_path] =
        std::move(definition);
}

std::shared_ptr<const LWPolicyDefinition>
RL::GetLWPolicyDefinition(
    const std::string& robot_config_path) const
{
    const auto it =
        this->lw_policy_definitions_.find(robot_config_path);
    if (it == this->lw_policy_definitions_.end())
    {
        return nullptr;
    }
    return it->second;
}

std::uint64_t RL::ActivateLWPolicy(
    const std::string& robot_config_path,
    float motion_length)
{
    const auto definition =
        GetLWPolicyDefinition(robot_config_path);
    if (!definition)
    {
        throw std::runtime_error(
            "LW policy context was not preloaded: "
            + robot_config_path);
    }

    auto activation = std::make_shared<LWPolicyActivation>();
    activation->definition = definition;
    activation->generation =
        lw_next_policy_generation_.fetch_add(
            1,
            std::memory_order_relaxed);
    activation->motion_length = motion_length;
    activation->activated_at = std::chrono::steady_clock::now();
    lw_policy_progress_.store(
        std::make_shared<LWPolicyProgressSnapshot>(
            LWPolicyProgressSnapshot{
                activation->generation,
                0}));
    lw_motion_reference_.clear();
    lw_policy_output_transport_.clear();
    lw_policy_activation_.store(activation);
    return activation->generation;
}

void RL::DeactivateLWPolicy()
{
    lw_policy_activation_.clear();
    lw_motion_reference_.clear();
    lw_policy_output_transport_.clear();
}

std::shared_ptr<const LWPolicyActivation>
RL::LoadLWPolicyActivation() const noexcept
{
    return lw_policy_activation_.load();
}

void RL::PublishLWMotionReference(
    LWMotionReferenceSnapshot reference)
{
    const auto activation = LoadLWPolicyActivation();
    if (!activation
        || reference.generation != activation->generation)
    {
        return;
    }
    lw_motion_reference_.store(
        std::make_shared<LWMotionReferenceSnapshot>(
            std::move(reference)));
}

void RL::PublishCurrentLWMotionReference(
    std::uint64_t generation)
{
    if (!this->motion_loader_lw)
    {
        return;
    }
    PublishLWMotionReference(
        LWMotionReferenceSnapshot{
            generation,
            this->motion_loader_lw->GetJointPos(),
            this->motion_loader_lw->GetJointVel(),
            this->motion_loader_lw->GetAnchorQuat(),
            this->motion_loader_lw->GetInitQuat()});
}

std::shared_ptr<const LWMotionReferenceSnapshot>
RL::LoadLWMotionReference() const noexcept
{
    return lw_motion_reference_.load();
}

void RL::PublishLWPolicyProgress(
    std::uint64_t generation,
    std::uint64_t frame)
{
    const auto activation = LoadLWPolicyActivation();
    if (!activation || activation->generation != generation)
    {
        return;
    }
    lw_policy_progress_.store(
        std::make_shared<LWPolicyProgressSnapshot>(
            LWPolicyProgressSnapshot{generation, frame}));
}

std::shared_ptr<const LWPolicyProgressSnapshot>
RL::LoadLWPolicyProgress() const noexcept
{
    return lw_policy_progress_.load();
}

bool RL::PublishLWPolicyOutput(LWPolicyOutputFrame output)
{
    const auto activation = LoadLWPolicyActivation();
    if (!activation
        || !activation->definition
        || output.generation != activation->generation)
    {
        return false;
    }
    const size_t expected_dofs = static_cast<size_t>(
        activation->definition->params.Get<int>("num_of_dofs"));
    return lw_policy_output_transport_.publish(
        std::move(output),
        activation->generation,
        expected_dofs);
}

std::shared_ptr<const LWPolicyOutputFrame>
RL::LoadLWPolicyOutput() const noexcept
{
    return lw_policy_output_transport_.load();
}

std::chrono::steady_clock::duration
RL::GetLWPolicyOutputMaxAge(
    const LWPolicyActivation& activation) const
{
    const YamlParams& policy_params =
        activation.definition->params;
    const float max_age_seconds =
        3.0f
        * policy_params.Get<float>("dt")
        * policy_params.Get<int>("decimation");
    return std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(
            std::chrono::duration<float>(max_age_seconds));
}

void RL::InitRL(std::string robot_config_path)
{
    std::lock_guard<std::mutex> lock(this->model_mutex);

    this->ReadYaml(robot_config_path, "config.yaml");

    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    this->InitObservations();
    this->InitOutputs();
    this->InitControl();

    const auto& observations_history = this->params.Get<std::vector<int>>("observations_history"); 
    if (!observations_history.empty())
    {
        int history_length = *std::max_element(observations_history.begin(), observations_history.end()) + 1;
        this->history_obs_buf = ObservationBuffer(1, this->obs_dims, history_length, this->params.Get<std::string>("observations_history_priority"));
    }

    // ==========================================
    // 直接从内存缓存中取模型
    // ==========================================
    if (this->preloaded_models_.find(robot_config_path) != this->preloaded_models_.end())
    {
        // 瞬间切换指针
        this->model = this->preloaded_models_[robot_config_path]; 
    }
    else
    {
        // 如果没预加载，就现场读
        std::cout << LOGGER::WARNING << "Model not preloaded! Loading from disk (WILL CAUSE LAG): " << robot_config_path << std::endl;
        std::string model_path = std::string(POLICY_DIR) + "/" + robot_config_path + "/" + this->params.Get<std::string>("model_name");
        auto loaded_model = InferenceRuntime::ModelFactory::load_model(model_path);
        this->model = std::shared_ptr<InferenceRuntime::Model>(std::move(loaded_model));
    }

    if (!this->model)
    {
        throw std::runtime_error("Failed to load model from: " + robot_config_path);
    }
}

void RL::ComputeOutput(const std::vector<float> &actions, std::vector<float> &output_dof_pos, std::vector<float> &output_dof_vel, std::vector<float> &output_dof_tau)
{
    std::vector<float> actions_scaled = actions * this->params.Get<std::vector<float>>("action_scale");
    std::vector<float> pos_actions_scaled = actions_scaled;
    std::vector<float> vel_actions_scaled(actions.size(), 0.0f);
    for (int i : this->params.Get<std::vector<int>>("wheel_indices"))
    {
        pos_actions_scaled[i] = 0.0f;
        vel_actions_scaled[i] = actions_scaled[i];
    }
    std::vector<float> all_actions_scaled = pos_actions_scaled + vel_actions_scaled;
    output_dof_pos = pos_actions_scaled + this->params.Get<std::vector<float>>("default_dof_pos");
    output_dof_vel = vel_actions_scaled;
    // 这一步力矩计算好像有问题，存疑
    // output_dof_tau = this->params.Get<std::vector<float>>("rl_kp") * (all_actions_scaled + this->params.Get<std::vector<float>>("default_dof_pos") - this->obs.dof_pos) - this->params.Get<std::vector<float>>("rl_kd") * this->obs.dof_vel;
    output_dof_tau = this->params.Get<std::vector<float>>("rl_kp") * (pos_actions_scaled + this->params.Get<std::vector<float>>("default_dof_pos") - this->obs.dof_pos) + this->params.Get<std::vector<float>>("rl_kd") * (vel_actions_scaled - this->obs.dof_vel);
    // output_dof_tau = clamp(output_dof_tau, -this->params.Get<std::vector<float>>("torque_limits"), this->params.Get<std::vector<float>>("torque_limits"));
}

void RL::ComputeLWOutput(
    const YamlParams& policy_params,
    const Observations<float>& policy_obs,
    const std::vector<float>& actions,
    std::vector<float>& output_dof_pos,
    std::vector<float>& output_dof_vel,
    std::vector<float>& output_dof_tau) const
{
    const std::vector<float> actions_scaled =
        actions
        * policy_params.Get<std::vector<float>>("action_scale");
    std::vector<float> pos_actions_scaled = actions_scaled;
    std::vector<float> vel_actions_scaled(actions.size(), 0.0f);
    for (int index :
         policy_params.Get<std::vector<int>>("wheel_indices"))
    {
        pos_actions_scaled[index] = 0.0f;
        vel_actions_scaled[index] = actions_scaled[index];
    }

    const auto default_dof_pos =
        policy_params.Get<std::vector<float>>("default_dof_pos");
    output_dof_pos = pos_actions_scaled + default_dof_pos;
    output_dof_vel = vel_actions_scaled;
    output_dof_tau =
        policy_params.Get<std::vector<float>>("rl_kp")
            * (pos_actions_scaled + default_dof_pos
               - policy_obs.dof_pos)
        + policy_params.Get<std::vector<float>>("rl_kd")
            * (vel_actions_scaled - policy_obs.dof_vel);
}

int RL::InverseJointMapping(int idx) const
{
    auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
    for (size_t i = 0; i < joint_mapping.size(); ++i) {
        if (joint_mapping[i] == idx) return (int)i;
    }
    return -1;
}

void RL::TorqueProtect(const std::vector<float>& origin_output_dof_tau)
{
    TorqueProtect(origin_output_dof_tau, this->params);
}

void RL::TorqueProtect(
    const std::vector<float>& origin_output_dof_tau,
    const YamlParams& policy_params) const
{
    std::vector<int> out_of_range_indices;
    std::vector<float> out_of_range_values;
    const auto torque_limits =
        policy_params.Get<std::vector<float>>("torque_limits");
    for (size_t i = 0; i < origin_output_dof_tau.size(); ++i)
    {
        float torque_value = origin_output_dof_tau[i];
        float limit_lower = -torque_limits[i];
        float limit_upper = torque_limits[i];

        if (torque_value < limit_lower || torque_value > limit_upper)
        {
            out_of_range_indices.push_back(i);
            out_of_range_values.push_back(torque_value);
        }
    }
    if (!out_of_range_indices.empty())
    {
        for (size_t i = 0; i < out_of_range_indices.size(); ++i)
        {
            int index = out_of_range_indices[i];
            float value = out_of_range_values[i];
            float limit_lower = -torque_limits[index];
            float limit_upper = torque_limits[index];

            std::cout << LOGGER::WARNING << "Torque(" << index + 1 << ")=" << value << " out of range(" << limit_lower << ", " << limit_upper << ")" << std::endl;
        }
        // Just a reminder, no protection
        // this->control.SetKeyboard(Input::Keyboard::P);
        // std::cout << LOGGER::INFO << "Switching to STATE_POS_GETDOWN"<< std::endl;
    }
}

void RL::AttitudeProtect(const std::vector<float> &quaternion, float pitch_threshold, float roll_threshold)
{
    // Use QuaternionToEuler from vector_math.hpp
    std::vector<float> euler = QuaternionToEuler(quaternion);
    float roll = euler[0] * 57.2958f;   // Convert to degrees
    float pitch = euler[1] * 57.2958f;

    if (std::fabs(roll) > roll_threshold)
    {
        this->control.SetKeyboard(Input::Keyboard::P);
        std::cout << LOGGER::WARNING << "Roll exceeds " << roll_threshold << " degrees. Current: " << roll << " degrees." << std::endl;
    }
    if (std::fabs(pitch) > pitch_threshold)
    {
        this->control.SetKeyboard(Input::Keyboard::P);
        std::cout << LOGGER::WARNING << "Pitch exceeds " << pitch_threshold << " degrees. Current: " << pitch << " degrees." << std::endl;
    }
}

#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

static int kbhit()
{
    static bool initialized = false;
    static termios original_term;

    // Initialize terminal to non-canonical mode on first call
    if (!initialized)
    {
        tcgetattr(STDIN_FILENO, &original_term);

        termios new_term = original_term;
        new_term.c_lflag &= ~(ICANON | ECHO);  // Disable canonical mode and echo
        new_term.c_cc[VMIN] = 0;   // Non-blocking read
        new_term.c_cc[VTIME] = 0;  // No timeout

        tcsetattr(STDIN_FILENO, TCSANOW, &new_term);

        // Register cleanup function to restore terminal on exit
        static bool cleanup_registered = false;
        if (!cleanup_registered)
        {
            std::atexit([]() {
                tcsetattr(STDIN_FILENO, TCSANOW, &original_term);
            });
            cleanup_registered = true;
        }

        initialized = true;
    }

    // Non-blocking read of a single character
    char c;
    int result = read(STDIN_FILENO, &c, 1);

    return (result == 1) ? (unsigned char)c : -1;
}

void RL::KeyboardInterface()
{
    int c = kbhit();
    if (c > 0)
    {
        switch (c)
        {
        case '0': this->control.SetKeyboard(Input::Keyboard::Num0); break;
        case '1': this->control.SetKeyboard(Input::Keyboard::Num1); break;
        case '2': this->control.SetKeyboard(Input::Keyboard::Num2); break;
        case '3': this->control.SetKeyboard(Input::Keyboard::Num3); break;
        case '4': this->control.SetKeyboard(Input::Keyboard::Num4); break;
        case '5': this->control.SetKeyboard(Input::Keyboard::Num5); break;
        case '6': this->control.SetKeyboard(Input::Keyboard::Num6); break;
        case '7': this->control.SetKeyboard(Input::Keyboard::Num7); break;
        case '8': this->control.SetKeyboard(Input::Keyboard::Num8); break;
        case '9': this->control.SetKeyboard(Input::Keyboard::Num9); break;
        case 'a': case 'A': this->control.SetKeyboard(Input::Keyboard::A); break;
        case 'b': case 'B': this->control.SetKeyboard(Input::Keyboard::B); break;
        case 'c': case 'C': this->control.SetKeyboard(Input::Keyboard::C); break;
        case 'd': case 'D': this->control.SetKeyboard(Input::Keyboard::D); break;
        case 'e': case 'E': this->control.SetKeyboard(Input::Keyboard::E); break;
        case 'f': case 'F': this->control.SetKeyboard(Input::Keyboard::F); break;
        case 'g': case 'G': this->control.SetKeyboard(Input::Keyboard::G); break;
        case 'h': case 'H': this->control.SetKeyboard(Input::Keyboard::H); break;
        case 'i': case 'I': this->control.SetKeyboard(Input::Keyboard::I); break;
        case 'j': case 'J': this->control.SetKeyboard(Input::Keyboard::J); break;
        case 'k': case 'K': this->control.SetKeyboard(Input::Keyboard::K); break;
        case 'l': case 'L': this->control.SetKeyboard(Input::Keyboard::L); break;
        case 'm': case 'M': this->control.SetKeyboard(Input::Keyboard::M); break;
        case 'n': case 'N': this->control.SetKeyboard(Input::Keyboard::N); break;
        case 'o': case 'O': this->control.SetKeyboard(Input::Keyboard::O); break;
        case 'p': case 'P': this->control.SetKeyboard(Input::Keyboard::P); break;
        case 'q': case 'Q': this->control.SetKeyboard(Input::Keyboard::Q); break;
        case 'r': case 'R': this->control.SetKeyboard(Input::Keyboard::R); break;
        case 's': case 'S': this->control.SetKeyboard(Input::Keyboard::S); break;
        case 't': case 'T': this->control.SetKeyboard(Input::Keyboard::T); break;
        case 'u': case 'U': this->control.SetKeyboard(Input::Keyboard::U); break;
        case 'v': case 'V': this->control.SetKeyboard(Input::Keyboard::V); break;
        case 'w': case 'W': this->control.SetKeyboard(Input::Keyboard::W); break;
        case 'x': case 'X': this->control.SetKeyboard(Input::Keyboard::X); break;
        case 'y': case 'Y': this->control.SetKeyboard(Input::Keyboard::Y); break;
        case 'z': case 'Z': this->control.SetKeyboard(Input::Keyboard::Z); break;
        case ' ': this->control.SetKeyboard(Input::Keyboard::Space); break;
        case '\n': case '\r': this->control.SetKeyboard(Input::Keyboard::Enter); break;
        case 27:  // Escape sequence (for arrow keys on Unix/Linux/macOS)
        {
            char seq[2];
            // Try to read escape sequence non-blockingly
            if (read(STDIN_FILENO, &seq[0], 1) == 1)
            {
                if (seq[0] == '[')
                {
                    if (read(STDIN_FILENO, &seq[1], 1) == 1)
                    {
                        switch (seq[1])
                        {
                        case 'A': this->control.SetKeyboard(Input::Keyboard::Up); break;
                        case 'B': this->control.SetKeyboard(Input::Keyboard::Down); break;
                        case 'C': this->control.SetKeyboard(Input::Keyboard::Right); break;
                        case 'D': this->control.SetKeyboard(Input::Keyboard::Left); break;
                        default: break;
                        }
                    }
                }
                else
                {
                    // Plain escape key
                    this->control.SetKeyboard(Input::Keyboard::Escape);
                }
            }
            else
            {
                // Plain escape key
                this->control.SetKeyboard(Input::Keyboard::Escape);
            }
        } break;
        default:  break;
        }
    }
}

template <typename T>
std::vector<T> ReadVectorFromYaml(const YAML::Node &node)
{
    std::vector<T> values;
    for (const auto &val : node)
    {
        values.push_back(val.as<T>());
    }
    return values;
}

void RL::ReadYaml(const std::string& file_path, const std::string& file_name)
{
    std::string config_path = std::string(POLICY_DIR) + "/" + file_path + "/" + file_name;
    YAML::Node config;
    try
    {
        config = YAML::LoadFile(config_path)[file_path];
    }
    catch (YAML::BadFile &e)
    {
        std::cout << LOGGER::ERROR << "The file '" << config_path << "' does not exist" << std::endl;
        return;
    }

    for (auto it = config.begin(); it != config.end(); ++it)
    {
        std::string key = it->first.as<std::string>();
        this->params.config_node[key] = it->second;
    }
}

void RL::CSVInit(std::string robot_path)
{
    csv_filename = std::string(POLICY_DIR) + "/" + robot_path + "/motor";

    // Uncomment these lines if need timestamp for file name
    // auto now = std::chrono::system_clock::now();
    // std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    // std::stringstream ss;
    // ss << std::put_time(std::localtime(&now_c), "%Y%m%d%H%M%S");
    // std::string timestamp = ss.str();
    // csv_filename += "_" + timestamp;

    csv_filename += ".csv";
    std::ofstream file(csv_filename.c_str());

    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << "tau_cal_" << i << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << "tau_est_" << i << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << "joint_pos_" << i << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << "joint_pos_target_" << i << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << "joint_vel_" << i << ","; }

    file << std::endl;

    file.close();
}

void RL::CSVLogger(const std::vector<float>& torque, const std::vector<float>& tau_est, const std::vector<float>& joint_pos, const std::vector<float>& joint_pos_target, const std::vector<float>& joint_vel)
{
    std::ofstream file(csv_filename.c_str(), std::ios_base::app);

    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << torque[i] << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << tau_est[i] << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << joint_pos[i] << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << joint_pos_target[i] << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << joint_vel[i] << ","; }

    file << std::endl;

    file.close();
}

bool RLFSMState::Interpolate(
    float& percent,
    const std::vector<float>& start_pos,
    const std::vector<float>& target_pos,
    float duration_seconds,
    const std::string& description,
    bool use_fixed_gains)
{
    if (percent >= 1.0f)
    {
        return false;
    }

    if (percent == 0.0f)
    {
        float max_diff = 0.0f;
        for (size_t i = 0; i < start_pos.size() && i < target_pos.size(); ++i)
        {
            max_diff = std::max(max_diff, std::abs(start_pos[i] - target_pos[i]));
        }

        if (max_diff < 0.1f)
        {
            percent = 1.0f;
        }
    }

    int required_frames = std::max(1, static_cast<int>(std::ceil(duration_seconds / rl.params.Get<float>("dt"))));
    float step = 1.0f / required_frames;

    percent += step;
    percent = std::min(percent, 1.0f);

    auto kp = use_fixed_gains ? rl.params.Get<std::vector<float>>("fixed_kp") : rl.params.Get<std::vector<float>>("rl_kp");
    auto kd = use_fixed_gains ? rl.params.Get<std::vector<float>>("fixed_kd") : rl.params.Get<std::vector<float>>("rl_kd");

    for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
    {
        fsm_command->motor_command.q[i] = (1 - percent) * start_pos[i] + percent * target_pos[i];
        fsm_command->motor_command.dq[i] = 0;
        fsm_command->motor_command.kp[i] = kp[i];
        fsm_command->motor_command.kd[i] = kd[i];
        fsm_command->motor_command.tau[i] = 0;
    }

    if (!description.empty())
    {
        LOGGER::PrintProgress(percent, description);
    }

    if (percent >= 1.0f)
    {
        return false;
    }

    return true;
}

void RLFSMState::RLControl()
{
    std::vector<float> _output_dof_pos, _output_dof_vel;
    if (rl.output_dof_pos_queue.try_pop(_output_dof_pos) && rl.output_dof_vel_queue.try_pop(_output_dof_vel))
    {
        const auto rl_kp =
            rl.params.Get<std::vector<float>>("rl_kp");
        const auto rl_kd =
            rl.params.Get<std::vector<float>>("rl_kd");
        for (int i = 0;
             i < rl.params.Get<int>("num_of_dofs");
             ++i)
        {
            if (!_output_dof_pos.empty())
            {
                fsm_command->motor_command.q[i] = _output_dof_pos[i];
            }
            if (!_output_dof_vel.empty())
            {
                fsm_command->motor_command.dq[i] = _output_dof_vel[i];
            }
            fsm_command->motor_command.kp[i] = rl_kp[i];
            fsm_command->motor_command.kd[i] = rl_kd[i];
            fsm_command->motor_command.tau[i] = 0;
        }
    }
}

void RLFSMState::RLControlLW()
{
    const auto activation = rl.LoadLWPolicyActivation();
    if (!activation || !activation->definition)
    {
        return;
    }

    const auto output = rl.LoadLWPolicyOutput();
    const auto now = std::chrono::steady_clock::now();
    const auto max_age =
        rl.GetLWPolicyOutputMaxAge(*activation);
    const size_t expected_dofs = static_cast<size_t>(
        activation->definition->params.Get<int>("num_of_dofs"));
    LWPolicyOutputStatus status = EvaluateLWPolicyOutput(
        output.get(),
        activation->generation,
        expected_dofs,
        now,
        max_age);

    if (status == LWPolicyOutputStatus::Ready
        && last_lw_output_generation_
            == activation->generation
        && output->sequence < last_lw_output_sequence_)
    {
        status = LWPolicyOutputStatus::Stale;
    }

    if (status != LWPolicyOutputStatus::Ready)
    {
        const bool initial_wait_expired =
            now >= activation->activated_at
            && now - activation->activated_at > max_age;
        if (status == LWPolicyOutputStatus::Stale
            || status == LWPolicyOutputStatus::Incomplete
            || initial_wait_expired)
        {
            lw_output_stale_ = true;
            constexpr auto warning_interval =
                std::chrono::seconds(1);
            if (last_lw_output_warning_
                    .time_since_epoch().count() == 0
                || now - last_lw_output_warning_
                    >= warning_interval)
            {
                last_lw_output_warning_ = now;
                long long output_age_ms = -1;
                if (output
                    && output->source_time
                        .time_since_epoch().count() != 0
                    && now >= output->source_time)
                {
                    output_age_ms =
                        std::chrono::duration_cast<
                            std::chrono::milliseconds>(
                            now - output->source_time)
                            .count();
                }
                std::cout << LOGGER::WARNING
                          << "LW policy output unavailable"
                          << ", generation="
                          << activation->generation
                          << ", status="
                          << LWPolicyOutputStatusName(status)
                          << ", max_age_ms="
                          << std::chrono::duration_cast<
                                 std::chrono::milliseconds>(
                                 max_age)
                                 .count()
                          << ", output_age_ms="
                          << output_age_ms
                          << ". Holding the last applied command."
                          << std::endl;
            }
        }
        return;
    }

    if (lw_output_stale_)
    {
        std::cout << LOGGER::INFO
                  << "LW policy output recovered"
                  << ", generation="
                  << activation->generation
                  << ", sequence="
                  << output->sequence
                  << std::endl;
        lw_output_stale_ = false;
    }

    if (last_lw_output_generation_
        != activation->generation)
    {
        last_lw_output_generation_ =
            activation->generation;
        last_lw_output_sequence_ = 0;
    }
    last_lw_output_sequence_ = output->sequence;

    const YamlParams& policy_params =
        activation->definition->params;
    const auto rl_kp =
        policy_params.Get<std::vector<float>>("rl_kp");
    const auto rl_kd =
        policy_params.Get<std::vector<float>>("rl_kd");
    for (size_t i = 0; i < expected_dofs; ++i)
    {
        fsm_command->motor_command.q[i] =
            output->dof_pos[i];
        fsm_command->motor_command.dq[i] =
            output->dof_vel[i];
        fsm_command->motor_command.kp[i] = rl_kp[i];
        fsm_command->motor_command.kd[i] = rl_kd[i];
        fsm_command->motor_command.tau[i] = 0.0f;
    }
}
