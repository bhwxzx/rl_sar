#include "lw_joinable_worker.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <exception>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

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

void TestShutdownUnblocksAndJoins()
{
    std::mutex mutex;
    std::condition_variable condition;
    bool stop_requested = false;
    std::atomic<bool> worker_exited{false};

    LWJoinableWorker worker([&]() {
        {
            std::lock_guard<std::mutex> lock(mutex);
            stop_requested = true;
        }
        condition.notify_all();
    });
    worker.start(
        []() {},
        [&]() {
            std::unique_lock<std::mutex> lock(mutex);
            condition.wait(lock, [&]() { return stop_requested; });
            worker_exited.store(true);
        },
        1s);

    Require(worker.joinable(), "worker must remain joinable while running");
    worker.shutdown();
    Require(worker_exited.load(), "shutdown must wait for worker exit");
    Require(!worker.joinable(), "shutdown must join the worker");
    worker.shutdown();
}

void TestStartupFailureIsPropagated()
{
    std::atomic<int> stop_requests{0};
    LWJoinableWorker worker([&]() { ++stop_requests; });
    bool failed = false;
    try
    {
        worker.start(
            []() { throw std::runtime_error("injected startup failure"); },
            []() {},
            1s);
    }
    catch (const std::runtime_error& error)
    {
        failed = std::string(error.what()) == "injected startup failure";
    }
    Require(failed, "startup exception must be propagated exactly");
    Require(!worker.joinable(), "failed startup must join its worker");
    Require(stop_requests.load() >= 1, "failed startup must request stop");
}

void TestStartupTimeoutIsBounded()
{
    std::mutex mutex;
    std::condition_variable condition;
    bool stop_requested = false;
    LWJoinableWorker worker([&]() {
        {
            std::lock_guard<std::mutex> lock(mutex);
            stop_requested = true;
        }
        condition.notify_all();
    });

    const auto started = std::chrono::steady_clock::now();
    bool timed_out = false;
    try
    {
        worker.start(
            [&]() {
                std::unique_lock<std::mutex> lock(mutex);
                condition.wait(lock, [&]() { return stop_requested; });
            },
            []() {},
            20ms);
    }
    catch (const std::runtime_error& error)
    {
        timed_out = std::string(error.what()) == "worker startup timed out";
    }
    const auto elapsed = std::chrono::steady_clock::now() - started;
    Require(timed_out, "startup wait must report its timeout");
    Require(elapsed < 1s, "startup timeout must remain bounded");
    Require(!worker.joinable(), "timed-out startup must join its worker");
}

void TestWorkerFailureRequestsStop()
{
    std::atomic<bool> stop_requested{false};
    LWJoinableWorker worker([&]() { stop_requested.store(true); });
    worker.start(
        []() {},
        []() { throw std::runtime_error("injected worker failure"); },
        1s);

    for (int attempt = 0; attempt < 100 && !stop_requested.load(); ++attempt)
    {
        std::this_thread::sleep_for(1ms);
    }
    worker.shutdown();
    Require(stop_requested.load(), "worker failure must request shutdown");

    bool failed = false;
    try
    {
        worker.rethrowWorkerError();
    }
    catch (const std::runtime_error& error)
    {
        failed = std::string(error.what()) == "injected worker failure";
    }
    Require(failed, "worker exception must cross the thread boundary");
}

void TestRepeatedLifecycle()
{
    for (int cycle = 0; cycle < 200; ++cycle)
    {
        std::atomic<bool> stop_requested{false};
        LWJoinableWorker worker([&]() { stop_requested.store(true); });
        worker.start(
            []() {},
            [&]() {
                while (!stop_requested.load())
                {
                    std::this_thread::yield();
                }
            },
            1s);
        worker.shutdown();
        Require(!worker.joinable(), "repeated lifecycle left a joinable worker");
    }
}
} // namespace

int main()
{
    try
    {
        TestShutdownUnblocksAndJoins();
        TestStartupFailureIsPropagated();
        TestStartupTimeoutIsBounded();
        TestWorkerFailureRequestsStop();
        TestRepeatedLifecycle();
    }
    catch (const std::exception& error)
    {
        std::cerr << "test_lw_joinable_worker failed: " << error.what()
                  << std::endl;
        return 1;
    }
    std::cout << "test_lw_joinable_worker passed" << std::endl;
    return 0;
}
