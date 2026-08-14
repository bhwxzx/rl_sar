#include "lw_signal_shutdown.hpp"

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include <unistd.h>

namespace
{
using namespace std::chrono_literals;

void Require(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

template<typename Predicate>
bool WaitUntil(Predicate predicate)
{
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (!predicate() && std::chrono::steady_clock::now() < deadline)
    {
        std::this_thread::sleep_for(1ms);
    }
    return predicate();
}
}

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cerr << "expected normal or sigterm mode" << std::endl;
        return 2;
    }
    const std::string mode(argv[1]);
    if (mode != "normal" && mode != "sigterm")
    {
        std::cerr << "unknown shutdown mode: " << mode << std::endl;
        return 2;
    }

    try
    {
        rclcpp::init(
            argc,
            argv,
            rclcpp::InitOptions(),
            rclcpp::SignalHandlerOptions::SigTerm);
        const auto node = std::make_shared<rclcpp::Node>(
            "lw_ros_shutdown_test_" + mode);
        LWSimShutdownCoordinator coordinator;
        std::atomic<int> stop_count{0};
        coordinator.Bind([&]() { ++stop_count; });

        std::exception_ptr ros_error;
        std::atomic<bool> ros_done{false};
        std::thread ros_thread([&]() {
            ros_error = RunLWSimShutdownBoundWorker(
                coordinator,
                [&]() { rclcpp::spin(node); });
            ros_done.store(true);
        });

        std::this_thread::sleep_for(20ms);
        bool trigger_succeeded = true;
        if (mode == "sigterm")
        {
            trigger_succeeded = kill(getpid(), SIGTERM) == 0;
        }
        else
        {
            rclcpp::shutdown();
        }

        const bool stop_observed = WaitUntil(
            [&]() { return stop_count.load() == 1; });
        const bool bounded_exit = WaitUntil(
            [&]() { return ros_done.load(); });
        if (!bounded_exit && rclcpp::ok())
        {
            rclcpp::shutdown();
        }
        if (ros_thread.joinable())
        {
            ros_thread.join();
        }
        if (rclcpp::ok())
        {
            rclcpp::shutdown();
        }

        Require(trigger_succeeded, "failed to trigger ROS shutdown");
        Require(stop_observed, "ROS exit did not request simulation stop");
        Require(bounded_exit, "ROS worker did not exit within two seconds");
        Require(stop_count.load() == 1, "ROS exit requested simulation stop repeatedly");
        if (ros_error)
        {
            std::rethrow_exception(ros_error);
        }
        std::cout << "LW ROS " << mode << " shutdown test passed" << std::endl;
        return 0;
    }
    catch (const std::exception& exception)
    {
        if (rclcpp::ok())
        {
            rclcpp::shutdown();
        }
        std::cerr << "test_lw_ros_shutdown failed: "
                  << exception.what() << std::endl;
        return 1;
    }
}
