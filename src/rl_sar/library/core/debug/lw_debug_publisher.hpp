#ifndef LW_DEBUG_PUBLISHER_HPP
#define LW_DEBUG_PUBLISHER_HPP

#include "LW_sdk.hpp"
#include "lw_runtime_sync.hpp"

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

struct LWDebugSnapshot
{
    LowState low_state{};
    LowCmd low_command{};
    std::array<float, 3> gyroscope{};
    std::array<float, 4> quaternion{1.0f, 0.0f, 0.0f, 0.0f};
    float command_x = 0.0f;
    float command_y = 0.0f;
    float command_yaw = 0.0f;
};

struct LWDebugPublisherConfig
{
    int num_dofs = 0;
    std::vector<int> joint_mapping;
    std::vector<int> wheel_indices;
    std::vector<float> rl_kp;
    std::vector<float> rl_kd;
    std::chrono::nanoseconds publish_period{std::chrono::milliseconds(4)};
    std::string topic_name{"/LW_joint_states"};
};

class LWDebugPublisher
{
public:
    static std::unique_ptr<LWDebugPublisher> CreateIfEnabled(
        bool enabled,
        const std::shared_ptr<rclcpp::Node>& node,
        LWDebugPublisherConfig config);

    void publishSnapshot(const LWDebugSnapshot& snapshot);
    bool publishOnce();

private:
    LWDebugPublisher(
        std::shared_ptr<rclcpp::Node> node,
        LWDebugPublisherConfig config);

    void initializeMessage();
    void populateMessage(
        sensor_msgs::msg::JointState& message,
        const LWDebugSnapshot& snapshot);

    std::shared_ptr<rclcpp::Node> node_;
    LWDebugPublisherConfig config_;
    LWSnapshotBuffer<LWDebugSnapshot> snapshot_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    std::shared_ptr<
        realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>
        realtime_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // LW_DEBUG_PUBLISHER_HPP
