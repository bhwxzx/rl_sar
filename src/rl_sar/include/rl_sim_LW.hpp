#ifndef RL_SIM_LW_HPP
#define RL_SIM_LW_HPP

#define PLOT
#define ENABLE_FORWARD_LATENCY
#define ADD_JOINTVEL_NOISE
// #define JOYSTICK_1
#define JOYSTICK_2
// #define CSV_LOGGER

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "inference_runtime.hpp"
#include "loop.hpp"
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

class Button
{
public:
    Button() {}

    void update(bool state)
    {
        on_press = state ? state != pressed : false;
        on_release = state ? false : state != pressed;
        pressed = state;
    }

    bool pressed = false;
    bool on_press = false;
    bool on_release = false;
};

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
    std::unique_ptr<Joystick> sys_js;
    JoystickEvent sys_js_event;

    Button sys_js_button[20];
    int sys_js_axis[10] = {0};
    bool sys_js_active = false;
    float axis_deadzone = 0.05f;
    int sys_js_max_value = (1 << (16 - 1)); // 即 2的15次方 = 32768
    void SetupSysJoystick(const std::string& device, int bits);
    void GetSysJoystick();

    // Imu
    // sensor_msgs::msg::Imu imu;
    // rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    // void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);

     // plot
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointstate_plot_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    void jointstate_plot_callback(void);

    // others
    void disable_robot(void);
    std::mutex state_mutex;

};

#endif // RL_SIM_LW_HPP