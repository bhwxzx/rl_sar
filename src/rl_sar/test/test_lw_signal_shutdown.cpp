#include "lw_signal_shutdown.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

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
void WaitUntil(Predicate predicate, const std::string& message)
{
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (!predicate() && std::chrono::steady_clock::now() < deadline)
    {
        std::this_thread::sleep_for(1ms);
    }
    Require(predicate(), message);
}

void SendSigint()
{
    Require(kill(getpid(), SIGINT) == 0, "failed to send SIGINT");
}

bool SigintIsBlocked()
{
    sigset_t current_mask;
    Require(
        pthread_sigmask(SIG_SETMASK, nullptr, &current_mask) == 0,
        "failed to inspect signal mask");
    return sigismember(&current_mask, SIGINT) == 1;
}

void TestStartupSignalIsLatchedUntilBind()
{
    LWSimShutdownCoordinator coordinator;
    LWSigintWaiter waiter([&]() { coordinator.Request(); });
    SendSigint();
    WaitUntil(
        [&]() { return coordinator.requested(); },
        "startup SIGINT was not latched");

    std::atomic<int> stop_count{0};
    coordinator.Bind([&]() { ++stop_count; });
    Require(stop_count.load() == 1, "latched SIGINT did not stop after bind");
}

void TestRepeatedSigintIsIdempotent()
{
    LWSimShutdownCoordinator coordinator;
    std::atomic<int> stop_count{0};
    coordinator.Bind([&]() { ++stop_count; });
    LWSigintWaiter waiter([&]() { coordinator.Request(); });

    for (int index = 0; index < 32; ++index)
    {
        SendSigint();
    }
    WaitUntil(
        [&]() { return coordinator.requested(); },
        "steady-state SIGINT was not delivered");
    std::this_thread::sleep_for(30ms);
    Require(stop_count.load() == 1, "repeated SIGINT was not idempotent");
}

void TestShutdownIsBoundedWithoutSignal()
{
    std::atomic<int> callback_count{0};
    const bool initially_blocked = SigintIsBlocked();
    const auto started = std::chrono::steady_clock::now();
    {
        LWSigintWaiter waiter([&]() { ++callback_count; });
        Require(SigintIsBlocked(), "SIGINT was not blocked in the main thread");
    }
    const auto elapsed = std::chrono::steady_clock::now() - started;
    Require(elapsed < 1s, "SIGINT waiter shutdown was not bounded");
    Require(callback_count.load() == 0, "normal shutdown invoked SIGINT callback");
    Require(
        SigintIsBlocked() == initially_blocked,
        "SIGINT waiter did not restore the original signal mask");
}

void TestRepeatedSigintDuringShutdownIsSafe()
{
    LWSimShutdownCoordinator coordinator;
    std::atomic<int> stop_count{0};
    coordinator.Bind([&]() { ++stop_count; });
    LWSigintWaiter waiter([&]() { coordinator.Request(); });
    std::thread sender(
        []()
        {
            for (int index = 0; index < 64; ++index)
            {
                SendSigint();
                std::this_thread::sleep_for(1ms);
            }
        });
    WaitUntil(
        [&]() { return coordinator.requested(); },
        "shutdown SIGINT was not delivered");
    waiter.Shutdown();
    sender.join();
    coordinator.Unbind();
    Require(stop_count.load() == 1, "shutdown SIGINT invoked callback repeatedly");
}

void TestCallbackFailureIsReported()
{
    LWSigintWaiter waiter(
        []() { throw std::runtime_error("test callback failure"); });
    SendSigint();
    std::this_thread::sleep_for(30ms);
    bool reported = false;
    try
    {
        waiter.RethrowWaitError();
    }
    catch (const std::runtime_error& error)
    {
        reported = std::string(error.what()) == "test callback failure";
    }
    Require(reported, "SIGINT callback failure was not retained");
}

void TestShutdownBoundWorkerStopsAfterNormalReturn()
{
    LWSimShutdownCoordinator coordinator;
    std::atomic<int> stop_count{0};
    coordinator.Bind([&]() { ++stop_count; });

    const std::exception_ptr error = RunLWSimShutdownBoundWorker(
        coordinator,
        []() {});
    Require(!error, "normal worker return reported an exception");
    Require(stop_count.load() == 1, "normal worker return did not stop simulation");
}

void TestShutdownBoundWorkerStopsAndRetainsException()
{
    LWSimShutdownCoordinator coordinator;
    std::atomic<int> stop_count{0};
    coordinator.Bind([&]() { ++stop_count; });

    const std::exception_ptr error = RunLWSimShutdownBoundWorker(
        coordinator,
        []() { throw std::runtime_error("ROS worker failure"); });
    Require(stop_count.load() == 1, "throwing worker did not stop simulation");
    bool retained = false;
    try
    {
        std::rethrow_exception(error);
    }
    catch (const std::runtime_error& exception)
    {
        retained = std::string(exception.what()) == "ROS worker failure";
    }
    Require(retained, "worker exception was not retained");
}

void TestConcurrentShutdownRequestsAreIdempotent()
{
    LWSimShutdownCoordinator coordinator;
    std::atomic<int> stop_count{0};
    coordinator.Bind([&]() { ++stop_count; });

    std::vector<std::thread> requesters;
    for (int index = 0; index < 32; ++index)
    {
        requesters.emplace_back([&]() { coordinator.Request(); });
    }
    for (std::thread& requester : requesters)
    {
        requester.join();
    }
    Require(stop_count.load() == 1, "concurrent shutdown was not idempotent");
}

void TestProcessShutdownCanKeepSigintBlocked()
{
    sigset_t original_mask;
    Require(
        pthread_sigmask(SIG_SETMASK, nullptr, &original_mask) == 0,
        "failed to save original signal mask");
    {
        LWSigintWaiter waiter([]() {});
        waiter.ShutdownAndKeepBlocked();
    }
    Require(SigintIsBlocked(), "process shutdown restored SIGINT unexpectedly");
    Require(
        pthread_sigmask(SIG_SETMASK, &original_mask, nullptr) == 0,
        "failed to restore test signal mask");
}

}

int main()
{
    try
    {
        TestStartupSignalIsLatchedUntilBind();
        TestRepeatedSigintIsIdempotent();
        TestShutdownIsBoundedWithoutSignal();
        TestRepeatedSigintDuringShutdownIsSafe();
        TestCallbackFailureIsReported();
        TestShutdownBoundWorkerStopsAfterNormalReturn();
        TestShutdownBoundWorkerStopsAndRetainsException();
        TestConcurrentShutdownRequestsAreIdempotent();
        TestProcessShutdownCanKeepSigintBlocked();
    }
    catch (const std::exception& error)
    {
        std::cerr << "test_lw_signal_shutdown failed: "
                  << error.what() << std::endl;
        return 1;
    }
    std::cout << "test_lw_signal_shutdown passed" << std::endl;
    return 0;
}
