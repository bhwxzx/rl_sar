#ifndef RL_SIM_LW_HPP
#define RL_SIM_LW_HPP

#define PLOT
// #define ENABLE_FORWARD_LATENCY
// #define ADD_ANGVEL_NOISE
// #define ADD_JOINTVEL_NOISE
#define JOYSTICK_1
// #define JOYSTICK_2
// #define CSV_LOGGER

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "inference_runtime.hpp"
#include "loop.hpp"
#include "lw_joystick_safety.hpp"
#include "fsm_LW.hpp"

#include "LW_sdk.hpp"
#include <csignal>
#include <random>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "joystick.hh"
#include "matplotlibcpp.h"
#include <mujoco/mujoco.h>
#include "mujoco_utils.hpp"
namespace plt = matplotlibcpp;

class RL_Real : public RL
{
public:
    RL_Real(int argc, char **argv);
    ~RL_Real();

    std::shared_ptr<rclcpp::Node> ros2_node;
    std::unique_ptr<mj::Simulate> sim;
    static RL_Real* instance;

private:
    // rl functions
    std::vector<float> Forward() override;
    void GetState(RobotState<float> *state) override;
    void SetCommand(const RobotCommand<float> *command) override;
    void RunModel();
    void RobotControl();

    // loop
    std::shared_ptr<LoopFunc> loop_joystick;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;
    std::shared_ptr<LoopFunc> loop_plot;
    std::shared_ptr<LoopFunc> loop_keyboard;

    // mujoco
    mjvCamera cam;
    mjvOption opt;
    mjvPerturb pert;
    mjData *mj_data;
    mjModel *mj_model;
    std::string scene_name;

    // LW interface
    LWSDK lw_sdk;
    LowCmd lw_low_command = {0};
    LowState lw_low_state = {0};
    void disable_lw_robot();

    // joystick
    std::unique_ptr<LWJoystickDevice> sys_js;
    JoystickEvent sys_js_event;

    std::array<LWJoystickButton, LW_JOYSTICK_BUTTON_COUNT> sys_js_button{};
    std::array<int, LW_JOYSTICK_AXIS_COUNT> sys_js_axis{};
    LWJoystickFaultLatch joystick_fault_latch_;
    bool sys_js_active = false;
    float axis_deadzone = 0.05f;
    int sys_js_max_value = (1 << (16 - 1)); // 即 2的15次方 = 32768
    void SetupSysJoystick(const std::string& device, int bits);
    void GetSysJoystick();
    void LatchJoystickFault(const LWJoystickSampleResult& result) noexcept;
    void ApplyJoystickFaultGate() noexcept;

    // Imu
    // sensor_msgs::msg::Imu imu;
    // rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    // void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);

    // plot
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointstate_plot_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    void jointstate_plot_callback(void);

    // actuator net
    bool use_actuator_net_ = false; // 是否使用执行器网络的 Flag
    std::shared_ptr<InferenceRuntime::Model> leg_actuator_model_; // 模型指针
    std::shared_ptr<InferenceRuntime::Model> foot_actuator_model_; // 模型指针
    std::vector<int> leg_train_indices = {0, 1, 2, 3, 4, 5}; 
    std::vector<int> foot_train_indices = {6, 7};
    std::deque<std::vector<float>> pos_err_history_; 
    std::deque<std::vector<float>> vel_history_;     
    std::vector<float> actuator_net_tau_;

    // others
    void disable_robot(void);
    std::mutex state_mutex;

};

#endif // RL_SIM_LW_HPP
