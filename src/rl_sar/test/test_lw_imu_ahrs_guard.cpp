#include "lw_imu_ahrs_guard.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
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

const std::array<double, 3> kEuler{0.1, -0.2, 0.3};
const std::array<double, 4> kQuaternion{1.0, 0.0, 0.0, 0.0};
const std::array<double, 3> kGyroscope{0.1, 0.2, 0.3};

void testRequiresOneFreshAhrsPerImu()
{
    LWImuAhrsGuard guard(20ms);
    const auto start = LWImuAhrsGuard::TimePoint{};

    require(
        !guard.observeImu(start, kQuaternion, kGyroscope).accepted(),
        "IMU without AHRS was accepted");
    require(guard.observeAhrs(start, kEuler), "finite AHRS was rejected");
    const auto accepted = guard.observeImu(start + 3ms, kQuaternion, kGyroscope);
    require(accepted.accepted(), "fresh paired IMU was rejected");
    require(accepted.pair_age_observed, "accepted pair age was not observed");
    require(accepted.pair_age == 3ms, "pair age was not retained");
    require(
        !guard.observeImu(start + 4ms, kQuaternion, kGyroscope).accepted(),
        "one AHRS event authorized more than one IMU");
}

void testRejectsExpiredAndRegressingPairs()
{
    LWImuAhrsGuard guard(20ms);
    const auto start = LWImuAhrsGuard::TimePoint{} + 100ms;
    guard.observeAhrs(start, kEuler);
    require(
        guard.observeImu(start + 20ms, kQuaternion, kGyroscope).accepted(),
        "inclusive pair age boundary was rejected");

    guard.observeAhrs(start + 30ms, kEuler);
    const auto expired =
        guard.observeImu(start + 51ms, kQuaternion, kGyroscope);
    require(
        expired.status == LWImuAhrsGuardStatus::StaleAhrs,
        "expired AHRS authorization was accepted");
    require(
        expired.pair_age_observed && expired.pair_age == 21ms,
        "expired pair age was not retained for profiling");

    guard.observeAhrs(start + 70ms, kEuler);
    require(
        guard.observeImu(start + 69ms, kQuaternion, kGyroscope).status
            == LWImuAhrsGuardStatus::StaleAhrs,
        "regressing callback time was accepted");
}

void testRejectsInvalidDataAndConsumesAuthorization()
{
    LWImuAhrsGuard guard(20ms);
    const auto start = LWImuAhrsGuard::TimePoint{};
    const double nan = std::numeric_limits<double>::quiet_NaN();

    require(
        !guard.observeAhrs(start, {nan, 0.0, 0.0}),
        "non-finite AHRS was accepted");
    require(
        guard.observeImu(start, kQuaternion, kGyroscope).status
            == LWImuAhrsGuardStatus::InvalidAhrs,
        "invalid AHRS state was not retained");

    guard.observeAhrs(start + 1ms, kEuler);
    require(
        guard.observeImu(start + 2ms, {0.0, 0.0, 0.0, 0.0}, kGyroscope).status
            == LWImuAhrsGuardStatus::InvalidImu,
        "zero quaternion was accepted");
    require(
        !guard.observeImu(start + 3ms, kQuaternion, kGyroscope).accepted(),
        "invalid IMU did not consume its one-shot authorization");

    guard.observeAhrs(start + 4ms, kEuler);
    require(
        guard.observeImu(start + 5ms, {1.2, 0.0, 0.0, 0.0}, kGyroscope).status
            == LWImuAhrsGuardStatus::InvalidImu,
        "implausibly scaled quaternion was accepted");

    guard.observeAhrs(start + 6ms, kEuler);
    require(
        guard.observeImu(start + 7ms, kQuaternion, {nan, 0.0, 0.0}).status
            == LWImuAhrsGuardStatus::InvalidImu,
        "non-finite angular velocity was accepted");
}

void testRejectsInvalidConfiguration()
{
    bool rejected = false;
    try
    {
        LWImuAhrsGuard guard(LWImuAhrsGuard::Duration::zero());
    }
    catch (const std::invalid_argument&)
    {
        rejected = true;
    }
    require(rejected, "non-positive pair maximum age was accepted");
}
} // namespace

int main()
{
    try
    {
        testRequiresOneFreshAhrsPerImu();
        testRejectsExpiredAndRegressingPairs();
        testRejectsInvalidDataAndConsumesAuthorization();
        testRejectsInvalidConfiguration();
        std::cout << "LW IMU/AHRS guard tests passed" << std::endl;
        return 0;
    }
    catch (const std::exception& exception)
    {
        std::cerr << "LW IMU/AHRS guard test failed: "
                  << exception.what() << std::endl;
        return 1;
    }
}
