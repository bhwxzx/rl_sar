#include "command_gate.hpp"
#include "loop.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <iostream>
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

void testShutdownWaitsForCallback()
{
    std::mutex mutex;
    std::condition_variable condition;
    bool callback_entered = false;
    bool release_callback = false;
    bool callback_exited = false;

    LoopFunc loop("blocking_callback", 0.001f, [&]()
    {
        std::unique_lock<std::mutex> lock(mutex);
        callback_entered = true;
        condition.notify_all();
        condition.wait(lock, [&]() { return release_callback; });
        callback_exited = true;
    });

    loop.start();
    {
        std::unique_lock<std::mutex> lock(mutex);
        require(
            condition.wait_for(lock, 1s, [&]() { return callback_entered; }),
            "loop callback did not start");
    }

    auto shutdown = std::async(std::launch::async, [&]() { loop.shutdown(); });
    require(
        shutdown.wait_for(30ms) == std::future_status::timeout,
        "shutdown returned before the active callback exited");

    {
        std::lock_guard<std::mutex> lock(mutex);
        release_callback = true;
    }
    condition.notify_all();

    require(
        shutdown.wait_for(1s) == std::future_status::ready,
        "shutdown did not join the loop thread");
    shutdown.get();
    require(callback_exited, "callback did not finish before shutdown returned");

    // shutdown is intentionally idempotent.
    loop.shutdown();
}

void testCallbackExceptionIsReported()
{
    std::promise<std::string> reported_error;
    auto error = reported_error.get_future();
    std::atomic<int> report_count{0};

    LoopFunc loop(
        "throwing_callback",
        0.001f,
        []() { throw std::runtime_error("expected callback failure"); },
        -1,
        [&](const std::string& loop_name, std::exception_ptr exception)
        {
            require(loop_name == "throwing_callback", "wrong loop name reported");
            ++report_count;
            try
            {
                std::rethrow_exception(exception);
            }
            catch (const std::exception& caught)
            {
                reported_error.set_value(caught.what());
            }
        });

    loop.start();
    require(
        error.wait_for(1s) == std::future_status::ready,
        "callback exception was not reported");
    require(
        error.get() == "expected callback failure",
        "callback exception message was not preserved");
    loop.shutdown();
    require(report_count.load() == 1, "callback exception was reported more than once");
}

void testCallbackExceptionLatchesDisable()
{
    CommandGate gate;
    std::mutex mutex;
    std::vector<std::string> commands;
    std::promise<void> disabled;
    auto disable_complete = disabled.get_future();

    LoopFunc loop(
        "faulting_command_loop",
        0.001f,
        [&]()
        {
            gate.sendIfOpen([&]()
            {
                std::lock_guard<std::mutex> lock(mutex);
                commands.emplace_back("normal");
            });
            throw std::runtime_error("command loop fault");
        },
        -1,
        [&](const std::string&, std::exception_ptr)
        {
            gate.closeAndSend([&]()
            {
                std::lock_guard<std::mutex> lock(mutex);
                commands.emplace_back("disable");
            });
            disabled.set_value();
        });

    loop.start();
    require(
        disable_complete.wait_for(1s) == std::future_status::ready,
        "callback exception did not trigger disable");
    disable_complete.get();
    loop.shutdown();

    require(
        !gate.sendIfOpen([&]() { commands.emplace_back("late-normal"); }),
        "exception shutdown accepted a command after disable");

    const std::vector<std::string> expected = {
        "normal",
        "disable",
    };
    require(commands == expected, "exception shutdown did not leave disable last");
}

void testDoubleStartIsRejected()
{
    LoopFunc loop("double_start", 1.0f, []() {});
    loop.start();

    bool rejected = false;
    try
    {
        loop.start();
    }
    catch (const std::logic_error&)
    {
        rejected = true;
    }

    loop.shutdown();
    require(rejected, "starting a live loop twice was not rejected");
}

void testDisableIsTheLastSerializedCommand()
{
    CommandGate gate;
    std::mutex mutex;
    std::condition_variable condition;
    bool normal_send_entered = false;
    bool release_normal_send = false;
    std::vector<std::string> events;

    std::thread normal_sender([&]()
    {
        require(gate.sendIfOpen([&]()
        {
            std::unique_lock<std::mutex> lock(mutex);
            events.emplace_back("normal-start");
            normal_send_entered = true;
            condition.notify_all();
            condition.wait(lock, [&]() { return release_normal_send; });
            events.emplace_back("normal-end");
        }), "normal command was unexpectedly rejected");
    });

    {
        std::unique_lock<std::mutex> lock(mutex);
        require(
            condition.wait_for(lock, 1s, [&]() { return normal_send_entered; }),
            "normal command did not enter the send section");
    }

    auto disable_sender = std::async(std::launch::async, [&]()
    {
        gate.closeAndSend([&]()
        {
            std::lock_guard<std::mutex> lock(mutex);
            events.emplace_back("disable");
        });
    });

    require(
        disable_sender.wait_for(30ms) == std::future_status::timeout,
        "disable command did not wait for the in-flight normal command");

    {
        std::lock_guard<std::mutex> lock(mutex);
        release_normal_send = true;
    }
    condition.notify_all();

    normal_sender.join();
    require(
        disable_sender.wait_for(1s) == std::future_status::ready,
        "disable command did not complete");
    disable_sender.get();

    require(gate.isClosed(), "command gate was not latched closed");
    require(
        !gate.sendIfOpen([&]() { events.emplace_back("late-normal"); }),
        "normal command was accepted after disable");

    const std::vector<std::string> expected = {
        "normal-start",
        "normal-end",
        "disable",
    };
    require(events == expected, "disable was not the final serialized command");
}
} // namespace

int main()
{
    try
    {
        testShutdownWaitsForCallback();
        testCallbackExceptionIsReported();
        testCallbackExceptionLatchesDisable();
        testDoubleStartIsRejected();
        testDisableIsTheLastSerializedCommand();
        std::cout << "loop lifecycle tests passed" << std::endl;
        return 0;
    }
    catch (const std::exception& exception)
    {
        std::cerr << "loop lifecycle test failed: " << exception.what() << std::endl;
        return 1;
    }
}
