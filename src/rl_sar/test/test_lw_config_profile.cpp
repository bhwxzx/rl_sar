#include "lw_config_profile.hpp"

#include <chrono>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <future>
#include <stdexcept>
#include <thread>

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
        start - std::chrono::milliseconds(3),
        start + std::chrono::milliseconds(15));
    require(timed_snapshot.seen, "timed snapshot lost seen state");
    require(
        timed_snapshot.first_sample_delay_us == 3000.0,
        "first sample delay differs");
    require(timed_snapshot.last_sample_offset_us == 15000.0, "last sample differs");
    require(timed_snapshot.final_age_us == 3000.0, "fixed-cutoff age differs");
}

void testCommonCutoffAndLateCallbacks()
{
    using Clock = LWProfileSamplingWindow::Clock;
    auto now = Clock::time_point{} + std::chrono::seconds(1);
    LWProfileSamplingWindow window([&] { return now; });
    LWProfileTimedSource right;
    LWProfileTimedSource left;
    LWProfileTimedSource missing;
    require(!window.record([](auto) {}), "sampling admitted before start");
    window.start();
    for (int index = 0; index < 3; ++index)
    {
        now += std::chrono::milliseconds(5);
        window.record([&](auto time) { right.mark(time); left.mark(time); });
    }
    now += std::chrono::milliseconds(2);
    window.close();
    const auto cutoff = window.snapshot();
    now += std::chrono::hours(1); // shutdown/sorting time must not change age
    window.close();
    require(window.snapshot().end == cutoff.end, "repeated close moved cutoff");
    require(!window.record([&](auto time) { right.mark(time); left.mark(time); }),
            "late feedback entered the closed window");
    const auto r = right.snapshotSince(cutoff.start, cutoff.end);
    now += std::chrono::hours(1);
    const auto l = left.snapshotSince(cutoff.start, cutoff.end);
    require(r.final_age_us == 2000.0 && l.final_age_us == r.final_age_us,
            "sequential snapshots changed equal-source ages");
    require(r.gaps.count == 2 && l.gaps.count == 2, "late samples changed gaps");
    require(r.gaps.maximum_us == 5000.0, "real gap evidence changed");
    const auto empty = missing.snapshotSince(cutoff.start, cutoff.end);
    require(!empty.seen && empty.gaps.count == 0 && empty.final_age_us == 0.0,
            "missing source fabricated age evidence");
    bool rejected = false;
    try { right.snapshotSince(cutoff.start, cutoff.start); }
    catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "snapshot accepted samples after its cutoff");
}

void testCloseDrainsInFlightTransaction()
{
    using Clock = LWProfileSamplingWindow::Clock;
    std::atomic<int> milliseconds{0};
    LWProfileSamplingWindow window([&]
    {
        return Clock::time_point{} + std::chrono::milliseconds(milliseconds.load());
    });
    LWProfileTimedSource raw;
    LWProfileTimedSource trusted;
    LWProfileDistribution pairs;
    window.start();
    milliseconds.store(5);
    std::promise<void> raw_recorded;
    std::promise<void> finish_transaction;
    auto finish = finish_transaction.get_future();
    std::thread callback([&]
    {
        window.record([&](auto time)
        {
            raw.mark(time);
            raw_recorded.set_value();
            finish.wait();
            pairs.recordMicroseconds(1000.0);
            trusted.mark(time);
        });
    });
    raw_recorded.get_future().wait();
    std::promise<void> close_attempted;
    auto closing = std::async(std::launch::async, [&]
    {
        close_attempted.set_value();
        window.close(); // also used by the profiler's failure path
    });
    close_attempted.get_future().wait();
    const bool waited = closing.wait_for(std::chrono::milliseconds(10))
        == std::future_status::timeout;
    milliseconds.store(8);
    finish_transaction.set_value();
    callback.join();
    closing.get();
    require(waited, "cutoff did not wait for the admitted transaction");
    const auto cutoff = window.snapshot();
    const auto r = raw.snapshotSince(cutoff.start, cutoff.end);
    const auto t = trusted.snapshotSince(cutoff.start, cutoff.end);
    require(r.seen && t.seen && pairs.snapshot().count == 1,
            "cutoff split raw/trusted/pair evidence");
    require(r.final_age_us == 3000.0 && t.final_age_us == 3000.0,
            "in-flight timestamps differ");
    require(!window.record([&](auto time) { raw.mark(time); }),
            "failed/closed sampling accepted a later callback");
}

void testCapturedDistributionIsIndependent()
{
    LWProfileDistribution samples(4);
    samples.recordMicroseconds(5.0);
    samples.recordMicroseconds(3.0);
    auto captured = samples.capture();
    samples.recordMicroseconds(100.0);
    const auto summary = LWProfileDistribution::summarize(std::move(captured));
    require(summary.count == 2 && summary.maximum_us == 5.0 && summary.p50_us == 4.0,
            "statistics changed after capture");
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
        testCommonCutoffAndLateCallbacks();
        testCloseDrainsInFlightTransaction();
        testCapturedDistributionIsIndependent();
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
