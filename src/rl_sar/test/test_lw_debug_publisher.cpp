#include "lw_debug_publisher.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace std::chrono_literals;

namespace
{
void require(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

void requireNear(double actual, double expected, const std::string& message)
{
    if (std::abs(actual - expected) > 1.0e-5)
    {
        throw std::runtime_error(
            message + ": expected " + std::to_string(expected)
            + ", got " + std::to_string(actual));
    }
}

LWDebugPublisherConfig makeConfig(const std::string& topic_name)
{
    LWDebugPublisherConfig config{
        10,
        {0, 1, 2, 3, 4, 5, 6, 7, 8, 9},
        {8, 9},
        std::vector<float>(10, 2.0f),
        std::vector<float>(10, 0.5f),
        std::chrono::hours(1)};
    config.topic_name = topic_name;
    return config;
}

LWDebugSnapshot makeSnapshot(float base)
{
    LWDebugSnapshot snapshot;
    for (int index = 0; index < MOTOR_COUNTS; ++index)
    {
        snapshot.low_state.motorState[index].pos_now = base + index;
        snapshot.low_state.motorState[index].vel_now = 2.0f * base + index;
        snapshot.low_state.motorState[index].tau_now = 3.0f * base + index;
        snapshot.low_command.motorCmd[index].action_set = 4.0f * base + index;
    }
    snapshot.gyroscope = {
        5.0f * base,
        5.0f * base + 1.0f,
        5.0f * base + 2.0f};
    snapshot.quaternion = {
        6.0f * base,
        6.0f * base + 1.0f,
        6.0f * base + 2.0f,
        6.0f * base + 3.0f};
    snapshot.command_x = 7.0f * base;
    snapshot.command_y = 7.0f * base + 1.0f;
    snapshot.command_yaw = 7.0f * base + 2.0f;
    return snapshot;
}

void spinUntilReceived(
    const std::shared_ptr<rclcpp::Node>& node,
    const std::vector<sensor_msgs::msg::JointState>& messages,
    std::size_t expected_count)
{
    for (int attempt = 0;
         attempt < 200 && messages.size() < expected_count;
         ++attempt)
    {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(2ms);
    }
    require(messages.size() >= expected_count, "debug message was not received");
}

void verifyMessage(
    const sensor_msgs::msg::JointState& message,
    float base)
{
    require(message.name.size() == 26, "unexpected debug name count");
    require(message.position.size() == 26, "unexpected position count");
    require(message.velocity.size() == 26, "unexpected velocity count");
    require(message.effort.size() == 26, "unexpected effort count");
    require(message.name[0] == "right_hip_now", "current joint name changed");
    require(message.name[10] == "right_hip_target", "target joint name changed");
    require(message.name[25] == "cmd_vel", "command field name changed");

    requireNear(message.position[0], base, "current position is incoherent");
    requireNear(message.velocity[0], 2.0f * base, "current velocity is incoherent");
    requireNear(message.effort[0], 3.0f * base, "current effort is incoherent");
    requireNear(message.position[10], 4.0f * base, "target position is incoherent");
    requireNear(message.position[8], 0.0f, "wheel position should remain zero");
    requireNear(
        message.velocity[18],
        4.0f * base + 8.0f,
        "wheel target is incoherent");
    requireNear(message.position[20], 5.0f * base, "gyroscope x is incoherent");
    requireNear(
        message.velocity[20],
        5.0f * base + 1.0f,
        "gyroscope y is incoherent");
    requireNear(
        message.effort[20],
        5.0f * base + 2.0f,
        "gyroscope z is incoherent");
    requireNear(message.position[21], 6.0f * base, "quaternion w is incoherent");
    requireNear(
        message.position[24],
        6.0f * base + 3.0f,
        "quaternion z is incoherent");
    requireNear(message.position[25], 7.0f * base, "command x is incoherent");
    requireNear(
        message.velocity[25],
        7.0f * base + 1.0f,
        "command y is incoherent");
    requireNear(
        message.effort[25],
        7.0f * base + 2.0f,
        "command yaw is incoherent");
}
} // namespace

int main(int argc, char** argv)
{
    try
    {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<rclcpp::Node>("test_lw_debug_publisher");
        const std::string topic_name =
            "/LW_joint_states_test_" + std::to_string(getpid());

        auto disabled = LWDebugPublisher::CreateIfEnabled(false, node, {});
        require(!disabled, "disabled debug publisher was constructed");
        require(
            node->count_publishers(topic_name) == 0,
            "disabled mode created a ROS publisher");

        std::vector<sensor_msgs::msg::JointState> messages;
        auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
            topic_name,
            rclcpp::QoS(1),
            [&messages](const sensor_msgs::msg::JointState::SharedPtr message)
            {
                messages.push_back(*message);
            });
        auto enabled = LWDebugPublisher::CreateIfEnabled(
            true,
            node,
            makeConfig(topic_name));
        require(enabled != nullptr, "enabled debug publisher was not constructed");
        require(!enabled->publishOnce(), "publisher emitted without a snapshot");

        enabled->publishSnapshot(makeSnapshot(10.0f));
        const auto first_before = node->get_clock()->now();
        require(enabled->publishOnce(), "first debug publish failed");
        const auto first_after = node->get_clock()->now();
        spinUntilReceived(node, messages, 1);
        verifyMessage(messages[0], 10.0f);
        const rclcpp::Time first_stamp(messages[0].header.stamp);
        require(
            first_stamp >= first_before && first_stamp <= first_after,
            "first message timestamp was not refreshed at publish time");

        std::this_thread::sleep_for(2ms);
        enabled->publishSnapshot(makeSnapshot(20.0f));
        const auto second_before = node->get_clock()->now();
        require(enabled->publishOnce(), "second debug publish failed");
        const auto second_after = node->get_clock()->now();
        spinUntilReceived(node, messages, 2);
        verifyMessage(messages[1], 20.0f);
        const rclcpp::Time second_stamp(messages[1].header.stamp);
        require(
            second_stamp >= second_before && second_stamp <= second_after,
            "second message timestamp was not refreshed at publish time");
        require(second_stamp > first_stamp, "message timestamps did not advance");

        enabled.reset();
        subscription.reset();
        rclcpp::shutdown();
        std::cout << "LW debug publisher tests passed" << std::endl;
        return EXIT_SUCCESS;
    }
    catch (const std::exception& exception)
    {
        std::cerr << "LW debug publisher test failed: "
                  << exception.what() << std::endl;
        if (rclcpp::ok())
        {
            rclcpp::shutdown();
        }
        return EXIT_FAILURE;
    }
}
