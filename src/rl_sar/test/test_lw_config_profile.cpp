#include "lw_config_profile.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace
{
void require(bool condition, const char* message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

void testDistribution()
{
    LWProfileDistribution distribution(4);
    for (int value = 1; value <= 6; ++value)
    {
        distribution.recordMicroseconds(static_cast<double>(value));
    }
    const auto snapshot = distribution.snapshot();
    require(snapshot.count == 6, "complete sample count was not retained");
    require(snapshot.retained == 4, "rolling window did not remain bounded");
    require(snapshot.minimum_us == 1.0, "minimum differs");
    require(snapshot.maximum_us == 6.0, "maximum differs");
    require(std::fabs(snapshot.mean_us - 3.5) < 1.0e-9, "mean differs");
    // The rolling window contains 5, 6, 3, 4 after wrap-around.
    require(std::fabs(snapshot.p50_us - 4.5) < 1.0e-9, "median differs");
}

void testTimedSource()
{
    LWProfileTimedSource source;
    const auto start =
        std::chrono::steady_clock::now() - std::chrono::milliseconds(12);
    source.mark(start);
    source.mark(start + std::chrono::milliseconds(5));
    source.mark(start + std::chrono::milliseconds(12));
    const auto snapshot = source.snapshot();
    require(source.hasBeenSeen(), "timed source was not marked seen");
    require(snapshot.count == 2, "timed source gap count differs");
    require(snapshot.minimum_us == 5000.0, "minimum gap differs");
    require(snapshot.maximum_us == 7000.0, "maximum gap differs");
    const auto timed_snapshot = source.snapshotSince(
        start - std::chrono::milliseconds(3));
    require(timed_snapshot.seen, "timed snapshot lost seen state");
    require(
        timed_snapshot.first_sample_delay_us == 3000.0,
        "first sample delay differs");
    require(timed_snapshot.final_age_us >= 0.0, "final age is negative");
}

void testInvalidSample()
{
    LWProfileDistribution distribution;
    bool rejected = false;
    try
    {
        distribution.recordMicroseconds(-1.0);
    }
    catch (const std::invalid_argument&)
    {
        rejected = true;
    }
    require(rejected, "negative sample was accepted");
}
} // namespace

int main()
{
    try
    {
        testDistribution();
        testTimedSource();
        testInvalidSample();
        std::cout << "LW configuration profile statistics tests passed\n";
        return EXIT_SUCCESS;
    }
    catch (const std::exception& exception)
    {
        std::cerr << "LW configuration profile statistics tests failed: "
                  << exception.what() << '\n';
        return EXIT_FAILURE;
    }
}
