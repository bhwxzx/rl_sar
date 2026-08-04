#include "loop.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <future>
#include <iostream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

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

template <typename Function>
void requireInvalidArgument(Function&& function, const std::string& message)
{
    bool rejected = false;
    try
    {
        function();
    }
    catch (const std::invalid_argument&)
    {
        rejected = true;
    }
    require(rejected, message);
}

template <typename Predicate>
bool waitFor(Predicate&& predicate, std::chrono::milliseconds timeout)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (predicate())
        {
            return true;
        }
        std::this_thread::sleep_for(1ms);
    }
    return predicate();
}

void testInvalidConfigurationIsRejectedBeforeCallback()
{
    requireInvalidArgument(
        []() { (void)LoopConfig::FromSeconds(0.0f); },
        "zero period was accepted");
    requireInvalidArgument(
        []() { (void)LoopConfig::FromSeconds(-0.01f); },
        "negative period was accepted");
    requireInvalidArgument(
        []()
        {
            (void)LoopConfig::FromSeconds(
                std::numeric_limits<float>::quiet_NaN());
        },
        "NaN period was accepted");

    LoopConfig invalid_cpu = LoopConfig::FromSeconds(0.01f);
    invalid_cpu.cpu_affinity = -2;
    requireInvalidArgument(
        [&]() { LoopFunc loop("invalid_cpu", invalid_cpu, []() {}); },
        "invalid CPU affinity was accepted");

    LoopConfig required_without_priority = LoopConfig::FromSeconds(0.01f);
    required_without_priority.require_realtime = true;
    requireInvalidArgument(
        [&]()
        {
            LoopFunc loop(
                "required_without_priority",
                required_without_priority,
                []() {});
        },
        "required realtime scheduling without a priority was accepted");

    LoopConfig negative_threshold = LoopConfig::FromSeconds(0.01f);
    negative_threshold.timing_policy.degraded_lateness = -1ns;
    requireInvalidArgument(
        [&]()
        {
            LoopFunc loop("negative_threshold", negative_threshold, []() {});
        },
        "negative timing threshold was accepted");

#ifdef __linux__
    std::atomic<int> callback_count{0};
    LoopConfig invalid_priority = LoopConfig::FromSeconds(0.01f);
    invalid_priority.realtime_priority = 1000;
    LoopFunc invalid_priority_loop(
        "invalid_priority",
        invalid_priority,
        [&]() { ++callback_count; });
    requireInvalidArgument(
        [&]() { invalid_priority_loop.start(); },
        "invalid SCHED_FIFO priority was accepted");
    require(
        callback_count.load() == 0,
        "callback ran before thread scheduling validation completed");
#endif
}

void testAbsoluteScheduleDoesNotAccumulateCallbackDuration()
{
    std::mutex mutex;
    std::vector<std::chrono::steady_clock::time_point> starts;
    std::promise<void> samples_ready;
    auto ready = samples_ready.get_future();
    std::atomic<bool> signaled{false};

    LoopFunc loop("absolute_schedule", 0.01f, [&]()
    {
        {
            std::lock_guard<std::mutex> lock(mutex);
            starts.push_back(std::chrono::steady_clock::now());
            if (starts.size() >= 15 && !signaled.exchange(true))
            {
                samples_ready.set_value();
            }
        }
        std::this_thread::sleep_for(5ms);
    });

    loop.start();
    require(
        ready.wait_for(1s) == std::future_status::ready,
        "absolute scheduler did not produce timing samples");
    loop.shutdown();

    std::lock_guard<std::mutex> lock(mutex);
    require(starts.size() >= 15, "too few absolute scheduling samples");
    const auto elapsed = starts[14] - starts[0];
    require(
        elapsed >= 100ms,
        "absolute scheduler produced an unexpected catch-up burst");
    require(
        elapsed < 190ms,
        "callback duration accumulated into each absolute period");
}

void testOverrunStatisticsAndNoCatchUpBurst()
{
    std::mutex mutex;
    std::vector<std::chrono::steady_clock::time_point> starts;
    std::atomic<int> callback_count{0};

    LoopConfig config = LoopConfig::FromSeconds(0.005f);
    LoopFunc loop("overrun_statistics", config, [&]()
    {
        {
            std::lock_guard<std::mutex> lock(mutex);
            starts.push_back(std::chrono::steady_clock::now());
        }
        ++callback_count;
        std::this_thread::sleep_for(12ms);
    });

    loop.start();
    require(
        waitFor([&]() { return callback_count.load() >= 4; }, 1s),
        "overrun loop did not run enough cycles");
    loop.shutdown();

    const LoopTimingSnapshot timing = loop.timingSnapshot();
    require(timing.cycles >= 4, "cycle count was not reported");
    require(timing.missed_deadlines > 0, "missed deadlines were not reported");
    require(timing.skipped_periods > 0, "skipped periods were not reported");
    require(
        timing.max_execution_time >= 10ms,
        "maximum callback execution time was not reported");
    require(
        timing.max_deadline_lateness > 0ns,
        "maximum deadline lateness was not reported");

    std::lock_guard<std::mutex> lock(mutex);
    for (std::size_t index = 1; index < starts.size(); ++index)
    {
        require(
            starts[index] - starts[index - 1] >= 10ms,
            "scheduler attempted a catch-up callback burst");
    }
}

void testThreeConsecutiveMissedPeriodsTriggerDegradedOnce()
{
    std::promise<void> degraded;
    auto degraded_ready = degraded.get_future();
    std::atomic<int> degraded_count{0};

    LoopConfig config = LoopConfig::FromSeconds(0.002f);
    config.timing_policy.degraded_consecutive_misses = 3;
    LoopFunc loop(
        "degraded_consecutive",
        config,
        []() { std::this_thread::sleep_for(3ms); },
        nullptr,
        [&](const std::string&, LoopTimingLevel level, const LoopTimingSnapshot&)
        {
            if (level == LoopTimingLevel::Degraded)
            {
                if (degraded_count.fetch_add(1) == 0)
                {
                    degraded.set_value();
                }
            }
        });

    loop.start();
    require(
        degraded_ready.wait_for(1s) == std::future_status::ready,
        "three consecutive missed periods did not trigger degraded timing");
    std::this_thread::sleep_for(20ms);
    loop.shutdown();

    const LoopTimingSnapshot timing = loop.timingSnapshot();
    require(
        timing.skipped_periods >= 3,
        "degraded timing fired before three periods were skipped");
    require(
        timing.level == LoopTimingLevel::Degraded,
        "degraded timing level was not latched");
    require(degraded_count.load() == 1, "degraded callback was not sticky");
}

void testSingleTwentyMillisecondLatenessTriggersDegraded()
{
    std::promise<void> degraded;
    auto degraded_ready = degraded.get_future();

    LoopConfig config = LoopConfig::FromSeconds(0.005f);
    config.timing_policy.degraded_lateness = 20ms;
    LoopFunc loop(
        "degraded_lateness",
        config,
        []() { std::this_thread::sleep_for(26ms); },
        nullptr,
        [&](const std::string&, LoopTimingLevel level, const LoopTimingSnapshot&)
        {
            if (level == LoopTimingLevel::Degraded)
            {
                degraded.set_value();
            }
        });

    loop.start();
    require(
        degraded_ready.wait_for(1s) == std::future_status::ready,
        "single 20 ms deadline lateness did not trigger degraded timing");
    loop.shutdown();
    require(
        loop.timingSnapshot().max_deadline_lateness >= 20ms,
        "20 ms deadline lateness was not retained in timing metrics");
}

void testFatalTimingIsDisabledByDefault()
{
    std::atomic<int> timing_events{0};
    std::atomic<int> callback_count{0};
    LoopConfig config = LoopConfig::FromSeconds(0.005f);
    LoopFunc loop(
        "fatal_disabled",
        config,
        [&]()
        {
            ++callback_count;
            std::this_thread::sleep_for(26ms);
        },
        nullptr,
        [&](const std::string&, LoopTimingLevel, const LoopTimingSnapshot&)
        {
            ++timing_events;
        });

    loop.start();
    require(
        waitFor([&]() { return callback_count.load() >= 2; }, 1s),
        "fatal-disabled loop did not run enough cycles");
    loop.shutdown();

    require(timing_events.load() == 0, "default policy raised a timing event");
    require(
        loop.timingSnapshot().level == LoopTimingLevel::Normal,
        "default policy enabled a fatal timing latch");
}

void testExplicitFatalThresholdTriggersFatal()
{
    std::promise<void> fatal;
    auto fatal_ready = fatal.get_future();
    std::atomic<int> fatal_count{0};
    LoopConfig config = LoopConfig::FromSeconds(0.005f);
    config.timing_policy.fatal_lateness = 10ms;
    LoopFunc loop(
        "fatal_enabled",
        config,
        []() { std::this_thread::sleep_for(16ms); },
        nullptr,
        [&](const std::string&, LoopTimingLevel level, const LoopTimingSnapshot&)
        {
            if (level == LoopTimingLevel::Fatal
                && fatal_count.fetch_add(1) == 0)
            {
                fatal.set_value();
            }
        });

    loop.start();
    require(
        fatal_ready.wait_for(1s) == std::future_status::ready,
        "explicit fatal timing threshold did not trigger");
    std::this_thread::sleep_for(20ms);
    loop.shutdown();
    require(
        loop.timingSnapshot().level == LoopTimingLevel::Fatal,
        "fatal timing level was not latched");
    require(fatal_count.load() == 1, "fatal callback was not sticky");
}

void testShutdownInterruptsAbsoluteWait()
{
    std::atomic<int> callback_count{0};
    LoopFunc loop("interrupt_wait", 5.0f, [&]() { ++callback_count; });
    loop.start();
    require(
        waitFor([&]() { return callback_count.load() >= 1; }, 100ms),
        "long-period loop did not execute its first callback");

    const auto started = std::chrono::steady_clock::now();
    loop.shutdown();
    require(
        std::chrono::steady_clock::now() - started < 500ms,
        "shutdown did not interrupt the absolute deadline wait");
}

void testShutdownDoesNotPromoteFinishingCallbackTiming()
{
    std::mutex mutex;
    std::condition_variable condition;
    bool callback_entered = false;
    bool release_callback = false;
    std::atomic<int> timing_events{0};

    LoopConfig config = LoopConfig::FromSeconds(0.001f);
    config.timing_policy.fatal_lateness = 1ms;
    LoopFunc loop(
        "shutdown_timing",
        config,
        [&]()
        {
            std::unique_lock<std::mutex> lock(mutex);
            callback_entered = true;
            condition.notify_all();
            condition.wait(lock, [&]() { return release_callback; });
        },
        nullptr,
        [&](const std::string&, LoopTimingLevel, const LoopTimingSnapshot&)
        {
            ++timing_events;
        });

    loop.start();
    {
        std::unique_lock<std::mutex> lock(mutex);
        require(
            condition.wait_for(lock, 1s, [&]() { return callback_entered; }),
            "shutdown timing callback did not start");
    }

    auto shutdown = std::async(std::launch::async, [&]() { loop.shutdown(); });
    std::this_thread::sleep_for(10ms);
    {
        std::lock_guard<std::mutex> lock(mutex);
        release_callback = true;
    }
    condition.notify_all();
    require(
        shutdown.wait_for(1s) == std::future_status::ready,
        "shutdown timing loop did not stop");
    shutdown.get();
    require(
        timing_events.load() == 0,
        "normal shutdown promoted a finishing callback timing fault");
    require(
        loop.timingSnapshot().level == LoopTimingLevel::Normal,
        "normal shutdown latched a timing fault");
}

void printTimingReport(const char* name, const LoopTimingSnapshot& timing)
{
    const double average_wakeup_us = timing.cycles == 0
        ? 0.0
        : std::chrono::duration<double, std::micro>(
              timing.total_wakeup_lateness).count()
              / static_cast<double>(timing.cycles);
    std::cout << name << " timing: cycles=" << timing.cycles
              << ", wakeup_us(avg/max)=" << average_wakeup_us << '/'
              << std::chrono::duration<double, std::micro>(
                     timing.max_wakeup_lateness).count()
              << ", deadline_late_us(max)="
              << std::chrono::duration<double, std::micro>(
                     timing.max_deadline_lateness).count()
              << ", execution_us(max)="
              << std::chrono::duration<double, std::micro>(
                     timing.max_execution_time).count()
              << ", missed=" << timing.missed_deadlines
              << ", skipped=" << timing.skipped_periods << '\n';
}
} // namespace

int main()
{
    try
    {
        testInvalidConfigurationIsRejectedBeforeCallback();
        testAbsoluteScheduleDoesNotAccumulateCallbackDuration();
        testOverrunStatisticsAndNoCatchUpBurst();
        testThreeConsecutiveMissedPeriodsTriggerDegradedOnce();
        testSingleTwentyMillisecondLatenessTriggersDegraded();
        testFatalTimingIsDisabledByDefault();
        testExplicitFatalThresholdTriggersFatal();
        testShutdownInterruptsAbsoluteWait();
        testShutdownDoesNotPromoteFinishingCallbackTiming();

        LoopFunc control_report("control_report", 0.005f, []() {});
        LoopFunc inference_report("inference_report", 0.02f, []() {});
        control_report.start();
        inference_report.start();
        std::this_thread::sleep_for(60ms);
        control_report.shutdown();
        inference_report.shutdown();
        printTimingReport("control", control_report.timingSnapshot());
        printTimingReport("inference", inference_report.timingSnapshot());

        std::cout << "loop timing tests passed" << std::endl;
        return 0;
    }
    catch (const std::exception& exception)
    {
        std::cerr << "loop timing test failed: " << exception.what()
                  << std::endl;
        return 1;
    }
}
