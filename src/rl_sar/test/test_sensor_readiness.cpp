#include "sensor_readiness.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
using namespace std::chrono_literals;

void require(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

void testWaitsForEveryRequiredSource()
{
    SensorReadinessMonitor monitor(100ms);
    const auto now = SensorReadinessMonitor::TimePoint{};

    auto status = monitor.evaluate(now);
    require(status.decision == SensorReadinessDecision::Waiting, "empty monitor did not wait");
    require(
        status.missingSources() == "IMU, right motor board, left motor board",
        "empty monitor did not report every missing source");

    monitor.markImuReceived(now);
    monitor.markRightFeedbackReceived(now);
    status = monitor.evaluate(now);
    require(status.decision == SensorReadinessDecision::Waiting, "monitor ignored missing left board");
    require(status.missingSources() == "left motor board", "wrong missing source was reported");

    monitor.markLeftFeedbackReceived(now);
    status = monitor.evaluate(now);
    require(status.decision == SensorReadinessDecision::Ready, "all fresh sources did not become ready");
}

void testEverySingleSourceCanBlockStartup()
{
    const auto now = SensorReadinessMonitor::TimePoint{};

    SensorReadinessMonitor missing_imu(100ms);
    missing_imu.markRightFeedbackReceived(now);
    missing_imu.markLeftFeedbackReceived(now);
    require(
        missing_imu.evaluate(now).missingSources() == "IMU",
        "missing IMU was not identified");

    SensorReadinessMonitor missing_right(100ms);
    missing_right.markImuReceived(now);
    missing_right.markLeftFeedbackReceived(now);
    require(
        missing_right.evaluate(now).missingSources() == "right motor board",
        "missing right board was not identified");

    SensorReadinessMonitor missing_left(100ms);
    missing_left.markImuReceived(now);
    missing_left.markRightFeedbackReceived(now);
    require(
        missing_left.evaluate(now).missingSources() == "left motor board",
        "missing left board was not identified");
}

void testRuntimeTimeoutLatchesFault()
{
    SensorReadinessMonitor monitor(100ms);
    const auto initial = SensorReadinessMonitor::TimePoint{};

    monitor.markImuReceived(initial);
    monitor.markRightFeedbackReceived(initial);
    monitor.markLeftFeedbackReceived(initial);
    require(
        monitor.evaluate(initial).decision == SensorReadinessDecision::Ready,
        "monitor did not enter ready state");
    require(
        monitor.evaluate(initial + 100ms).decision == SensorReadinessDecision::Ready,
        "source expired at the inclusive timeout boundary");

    const auto stale = initial + 101ms;
    auto status = monitor.evaluate(stale);
    require(
        status.decision == SensorReadinessDecision::FaultLatched,
        "runtime timeout did not latch a fault");
    require(monitor.faultLatched(), "fault latch was not retained");

    monitor.markImuReceived(stale);
    monitor.markRightFeedbackReceived(stale);
    monitor.markLeftFeedbackReceived(stale);
    status = monitor.evaluate(stale);
    require(
        status.decision == SensorReadinessDecision::FaultLatched,
        "reconnection silently cleared the fault latch");
    require(status.allFresh(), "reconnected sources were not recognized as fresh");
}

void testEveryRuntimeSourceCanLatchFault()
{
    const auto initial = SensorReadinessMonitor::TimePoint{};
    const auto stale = initial + 101ms;

    const auto require_fault_for = [&](const std::string& missing_source)
    {
        SensorReadinessMonitor monitor(100ms);
        monitor.markImuReceived(initial);
        monitor.markRightFeedbackReceived(initial);
        monitor.markLeftFeedbackReceived(initial);
        require(
            monitor.evaluate(initial).decision == SensorReadinessDecision::Ready,
            "monitor did not become ready before single-source timeout");

        if (missing_source != "IMU")
        {
            monitor.markImuReceived(stale);
        }
        if (missing_source != "right motor board")
        {
            monitor.markRightFeedbackReceived(stale);
        }
        if (missing_source != "left motor board")
        {
            monitor.markLeftFeedbackReceived(stale);
        }

        const auto status = monitor.evaluate(stale);
        require(
            status.decision == SensorReadinessDecision::FaultLatched,
            missing_source + " timeout did not latch a fault");
        require(
            status.missingSources() == missing_source,
            missing_source + " timeout reported the wrong source");
    };

    require_fault_for("IMU");
    require_fault_for("right motor board");
    require_fault_for("left motor board");
}

void testInvalidTimeoutIsRejected()
{
    bool rejected = false;
    try
    {
        SensorReadinessMonitor monitor(SensorReadinessMonitor::Duration::zero());
    }
    catch (const std::invalid_argument&)
    {
        rejected = true;
    }
    require(rejected, "non-positive timeout was accepted");
}
} // namespace

int main()
{
    try
    {
        testWaitsForEveryRequiredSource();
        testEverySingleSourceCanBlockStartup();
        testRuntimeTimeoutLatchesFault();
        testEveryRuntimeSourceCanLatchFault();
        testInvalidTimeoutIsRejected();
        std::cout << "sensor readiness tests passed" << std::endl;
        return 0;
    }
    catch (const std::exception& exception)
    {
        std::cerr << "sensor readiness test failed: " << exception.what() << std::endl;
        return 1;
    }
}
