#include "lw_runtime_sync.hpp"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
void require(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

struct CoherentFrame
{
    std::uint64_t sequence = 0;
    std::vector<std::uint64_t> values;
};

void testSnapshotBufferPublishesWholeFrames()
{
    LWSnapshotBuffer<CoherentFrame> buffer;
    std::atomic<bool> start{false};
    std::atomic<bool> failed{false};

    std::thread writer([&]()
    {
        while (!start.load(std::memory_order_acquire))
        {
        }
        for (std::uint64_t sequence = 1;
             sequence <= 50000;
             ++sequence)
        {
            buffer.publish(
                CoherentFrame{
                    sequence,
                    std::vector<std::uint64_t>(32, sequence)});
        }
    });

    std::vector<std::thread> readers;
    for (int reader = 0; reader < 4; ++reader)
    {
        readers.emplace_back([&]()
        {
            while (!start.load(std::memory_order_acquire))
            {
            }
            for (int iteration = 0;
                 iteration < 50000;
                 ++iteration)
            {
                CoherentFrame frame;
                if (!buffer.read(frame))
                {
                    continue;
                }
                if (frame.values.size() != 32)
                {
                    failed.store(true, std::memory_order_release);
                    return;
                }
                for (std::uint64_t value : frame.values)
                {
                    if (value != frame.sequence)
                    {
                        failed.store(
                            true,
                            std::memory_order_release);
                        return;
                    }
                }
            }
        });
    }

    start.store(true, std::memory_order_release);
    writer.join();
    for (auto& reader : readers)
    {
        reader.join();
    }
    require(
        !failed.load(std::memory_order_acquire),
        "snapshot buffer exposed a partially published frame");
}

struct CopyGate
{
    std::mutex mutex;
    std::condition_variable condition;
    bool entered = false;
    bool release = false;
};

struct BlockingReadFrame
{
    std::uint64_t value = 0;
    bool block_on_assignment = false;
    std::shared_ptr<CopyGate> gate;

    BlockingReadFrame& operator=(const BlockingReadFrame& other)
    {
        if (block_on_assignment && gate)
        {
            std::unique_lock<std::mutex> lock(gate->mutex);
            gate->entered = true;
            gate->condition.notify_all();
            gate->condition.wait(lock, [&]() { return gate->release; });
        }
        value = other.value;
        return *this;
    }
};

void testTryPublishNeverWaitsForAReader()
{
    LWSnapshotBuffer<BlockingReadFrame> buffer;
    BlockingReadFrame initial;
    initial.value = 1;
    buffer.publish(initial);

    const auto gate = std::make_shared<CopyGate>();
    BlockingReadFrame blocked_destination;
    blocked_destination.block_on_assignment = true;
    blocked_destination.gate = gate;
    std::thread reader([&]() { buffer.read(blocked_destination); });

    {
        std::unique_lock<std::mutex> lock(gate->mutex);
        gate->condition.wait(lock, [&]() { return gate->entered; });
    }
    BlockingReadFrame update;
    update.value = 2;
    require(
        !buffer.tryPublish(update),
        "tryPublish waited for or overwrote a reader-held snapshot");

    {
        std::lock_guard<std::mutex> lock(gate->mutex);
        gate->release = true;
    }
    gate->condition.notify_all();
    reader.join();

    require(buffer.tryPublish(update), "tryPublish did not recover after contention");
    BlockingReadFrame received;
    require(buffer.read(received), "updated snapshot was unavailable");
    require(received.value == 2, "updated snapshot was incoherent");
}

struct ImmutableContext
{
    std::uint64_t generation = 0;
    std::string config;
    std::string model;
};

void testAtomicSnapshotKeepsContextCoherent()
{
    LWAtomicSnapshot<ImmutableContext> slot;
    std::atomic<bool> start{false};
    std::atomic<bool> failed{false};

    std::thread writer([&]()
    {
        while (!start.load(std::memory_order_acquire))
        {
        }
        for (std::uint64_t generation = 1;
             generation <= 50000;
             ++generation)
        {
            const std::string tag = std::to_string(generation);
            slot.store(
                std::make_shared<ImmutableContext>(
                    ImmutableContext{
                        generation,
                        "config-" + tag,
                        "model-" + tag}));
        }
    });

    std::vector<std::thread> readers;
    for (int reader = 0; reader < 4; ++reader)
    {
        readers.emplace_back([&]()
        {
            while (!start.load(std::memory_order_acquire))
            {
            }
            for (int iteration = 0;
                 iteration < 50000;
                 ++iteration)
            {
                const auto context = slot.load();
                if (!context)
                {
                    continue;
                }
                const std::string tag =
                    std::to_string(context->generation);
                if (context->config != "config-" + tag
                    || context->model != "model-" + tag)
                {
                    failed.store(
                        true,
                        std::memory_order_release);
                    return;
                }
            }
        });
    }

    start.store(true, std::memory_order_release);
    writer.join();
    for (auto& reader : readers)
    {
        reader.join();
    }
    require(
        !failed.load(std::memory_order_acquire),
        "atomic policy slot mixed generations");
}

enum class TestEvent
{
    None,
    Start,
    Stop
};

void testInputMailboxKeepsVelocityCoherentAndEventsSequenced()
{
    LWInputMailbox<TestEvent> mailbox;
    std::atomic<bool> start{false};
    std::atomic<bool> failed{false};

    std::thread writer([&]()
    {
        while (!start.load(std::memory_order_acquire))
        {
        }
        for (int value = 1; value <= 50000; ++value)
        {
            const float x = static_cast<float>(value);
            mailbox.publishVelocity(x, 2.0f * x, 3.0f * x);
            if (value % 1000 == 0)
            {
                mailbox.publishEvent(TestEvent::Start);
            }
        }
    });

    std::thread reader([&]()
    {
        while (!start.load(std::memory_order_acquire))
        {
        }
        std::uint64_t last_event_sequence = 0;
        for (int iteration = 0;
             iteration < 50000;
             ++iteration)
        {
            const auto snapshot = mailbox.read();
            if (snapshot.y != 2.0f * snapshot.x
                || snapshot.yaw != 3.0f * snapshot.x
                || snapshot.event_sequence
                    < last_event_sequence)
            {
                failed.store(true, std::memory_order_release);
                return;
            }
            last_event_sequence = snapshot.event_sequence;
        }
    });

    start.store(true, std::memory_order_release);
    writer.join();
    reader.join();
    require(
        !failed.load(std::memory_order_acquire),
        "input mailbox exposed incoherent velocity or event state");

    const auto before_clear = mailbox.read();
    mailbox.clear(TestEvent::None);
    const auto after_clear = mailbox.read();
    require(
        after_clear.x == 0.0f
            && after_clear.y == 0.0f
            && after_clear.yaw == 0.0f
            && after_clear.event == TestEvent::None,
        "input clear retained a stale command");
    require(
        after_clear.event_sequence
            == before_clear.event_sequence + 1,
        "input clear was not published as a new event generation");
}

void testOperatorStatusMailboxPublishesCoherentFramesWithoutBlocking()
{
    LWOperatorStatusMailbox mailbox;
    std::atomic<bool> start{false};
    std::atomic<bool> failed{false};

    std::thread writer([&]()
    {
        while (!start.load(std::memory_order_acquire))
        {
        }
        for (std::uint64_t sequence = 1; sequence <= 50000; ++sequence)
        {
            const float value = static_cast<float>(sequence);
            mailbox.publish(
                {sequence,
                 LWOperatorMode::WheelLocomotion,
                 value,
                 2.0f * value,
                 3.0f * value,
                 4.0f * value,
                 5.0f * value});
        }
    });

    std::vector<std::thread> readers;
    for (int reader = 0; reader < 4; ++reader)
    {
        readers.emplace_back([&]()
        {
            while (!start.load(std::memory_order_acquire))
            {
            }
            std::uint64_t last_sequence = 0;
            for (int iteration = 0; iteration < 50000; ++iteration)
            {
                LWOperatorStatusSnapshot status;
                if (!mailbox.read(status))
                {
                    continue;
                }
                if (status.mode != LWOperatorMode::WheelLocomotion
                    || status.sequence < last_sequence
                    || status.y != 2.0f * status.x
                    || status.yaw != 3.0f * status.x
                    || status.gait_frequency != 4.0f * status.x
                    || status.progress != 5.0f * status.x)
                {
                    failed.store(true, std::memory_order_release);
                    return;
                }
                last_sequence = status.sequence;
            }
        });
    }

    start.store(true, std::memory_order_release);
    writer.join();
    for (auto& reader : readers)
    {
        reader.join();
    }
    require(
        !failed.load(std::memory_order_acquire),
        "operator status mailbox exposed a partially published frame");
}
} // namespace

int main()
{
    try
    {
        testSnapshotBufferPublishesWholeFrames();
        testTryPublishNeverWaitsForAReader();
        testAtomicSnapshotKeepsContextCoherent();
        testInputMailboxKeepsVelocityCoherentAndEventsSequenced();
        testOperatorStatusMailboxPublishesCoherentFramesWithoutBlocking();
        std::cout << "LW runtime synchronization tests passed"
                  << std::endl;
        return EXIT_SUCCESS;
    }
    catch (const std::exception& exception)
    {
        std::cerr << "LW runtime synchronization tests failed: "
                  << exception.what() << std::endl;
        return EXIT_FAILURE;
    }
}
