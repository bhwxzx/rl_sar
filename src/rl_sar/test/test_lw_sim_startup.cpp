#include "lw_sim_startup.hpp"

#include <array>
#include <atomic>
#include <future>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

namespace {
void Check(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

void TestReadinessAndSingleStart()
{
    LWSimStartup gate;
    int starts = 0, rollbacks = 0;
    const auto start = [&](int stage) { Check(stage == starts++, "startup order"); };
    const auto rollback = [&]() noexcept { ++rollbacks; };
    gate.StartIfReady(start, rollback, [] { return false; });
    Check(starts == 0, "started before initialization");
    gate.PublishReady();
    gate.PublishReady();
    gate.StartIfReady(start, rollback, [] { return false; });
    gate.PublishReady();
    gate.StartIfReady(start, rollback, [] { return false; });
    Check(starts == 3 && rollbacks == 0, "duplicate runtime startup");
    Check(gate.WaitForRuntime([] { return false; }), "runtime permission missing");
}

void TestEarlyExit()
{
    for (bool ready : {false, true})
    {
        LWSimStartup gate;
        if (ready) gate.PublishReady();
        gate.Cancel();
        gate.PublishReady();
        gate.StartIfReady([](int) { throw std::runtime_error("started after cancel"); },
                          []() noexcept {}, [] { return false; });
        Check(!gate.WaitForRuntime([] { return false; }), "cancelled startup revived");
    }
    LWSimStartup external_exit;
    Check(!external_exit.WaitForRuntime([] { return true; }), "window exit not observed");
    external_exit.PublishReady();
    Check(external_exit.state() == LWSimStartup::State::Cancelled, "late ready revived exit");
}

void TestCancelDuringEachStart()
{
    for (int cancel_stage = 0; cancel_stage < 3; ++cancel_stage)
    {
        LWSimStartup gate;
        gate.PublishReady();
        std::promise<void> reached, resume;
        auto resumed = resume.get_future();
        int starts = 0, rollbacks = 0;
        std::thread starter([&] {
            gate.StartIfReady([&](int stage) {
                ++starts;
                if (stage == cancel_stage)
                {
                    reached.set_value();
                    resumed.wait();
                }
            }, [&]() noexcept { ++rollbacks; }, [] { return false; });
        });
        reached.get_future().wait();
        gate.Cancel();
        resume.set_value();
        starter.join();
        Check(starts == cancel_stage + 1 && rollbacks == 1, "partial start not rolled back");
        Check(!gate.WaitForRuntime([] { return false; }), "cancel allowed physics");
    }
}

void TestFailedStartJoinsWorkers()
{
    for (int fail_stage = 0; fail_stage < 3; ++fail_stage)
    {
        LWSimStartup gate;
        gate.PublishReady();
        std::atomic<bool> stop{false};
        std::array<std::thread, 3> workers;
        std::vector<int> joined;
        bool caught = false;
        try
        {
            gate.StartIfReady([&](int stage) {
                workers[stage] = std::thread([&] {
                    while (!stop.load()) std::this_thread::yield();
                });
                if (stage == fail_stage) throw std::runtime_error("injected start failure");
            }, [&]() noexcept {
                stop.store(true);
                for (int stage = 2; stage >= 0; --stage)
                {
                    if (workers[stage].joinable())
                    {
                        workers[stage].join();
                        joined.push_back(stage);
                    }
                }
            }, [] { return false; });
        }
        catch (const std::runtime_error&) { caught = true; }
        Check(caught, "startup exception swallowed");
        Check(joined.size() == static_cast<std::size_t>(fail_stage + 1), "worker leaked");
        for (int i = 0; i <= fail_stage; ++i)
            Check(joined[i] == fail_stage - i, "rollback order");
        Check(!gate.WaitForRuntime([] { return false; }), "failed startup allowed physics");
    }
}

void TestPhysicsWaitAndRelease()
{
    for (bool cancel : {false, true})
    {
        LWSimStartup gate;
        std::promise<void> ready;
        bool advanced = false;
        std::thread physics([&] {
            gate.PublishReady();
            ready.set_value();
            advanced = gate.WaitForRuntime([] { return false; });
        });
        ready.get_future().wait();
        Check(gate.state() == LWSimStartup::State::Ready, "physics bypassed runtime permission");
        if (cancel) gate.Cancel();
        else gate.StartIfReady([](int) {}, []() noexcept {}, [] { return false; });
        physics.join();
        Check(advanced != cancel, "wrong physics release result");
    }
}
}  // namespace

int main()
{
    try
    {
        TestReadinessAndSingleStart();
        TestEarlyExit();
        TestCancelDuringEachStart();
        TestFailedStartJoinsWorkers();
        TestPhysicsWaitAndRelease();
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
