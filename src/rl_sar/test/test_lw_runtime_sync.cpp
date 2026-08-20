#include "lw_runtime_sync.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <future>
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

struct BlockingPublishFrame
{
    std::uint64_t value = 0;
    bool block_copy = false;
    std::shared_ptr<CopyGate> gate;

    BlockingPublishFrame& operator=(const BlockingPublishFrame& other)
    {
        if (other.block_copy && other.gate)
        {
            std::unique_lock<std::mutex> lock(other.gate->mutex);
            other.gate->entered = true;
            other.gate->condition.notify_all();
            other.gate->condition.wait(
                lock, [&]() { return other.gate->release; });
        }
        value = other.value;
        block_copy = false;
        gate.reset();
        return *this;
    }
};

void testSpscReaderNeverWaitsForAnInProgressPublish()
{
    LWSpscLatestValue<BlockingPublishFrame> slot;
    slot.publish(BlockingPublishFrame{1, false, nullptr});
    const BlockingPublishFrame* initial = slot.readLatest();
    require(initial && initial->value == 1, "initial SPSC value missing");

    const auto gate = std::make_shared<CopyGate>();
    BlockingPublishFrame update{2, true, gate};
    std::thread writer([&]()
    {
        slot.publish(update);
    });
    {
        std::unique_lock<std::mutex> lock(gate->mutex);
        gate->condition.wait(lock, [&]() { return gate->entered; });
    }
    std::promise<void> read_started;
    auto read_started_future = read_started.get_future();
    std::promise<const BlockingPublishFrame*> read_result;
    auto read_future = read_result.get_future();
    std::thread reader([&]()
    {
        read_started.set_value();
        read_result.set_value(slot.readLatest());
    });
    read_started_future.wait();
    const bool reader_returned =
        read_future.wait_for(std::chrono::milliseconds(500))
            == std::future_status::ready;
    const BlockingPublishFrame* retained =
        reader_returned ? read_future.get() : nullptr;
    if (reader_returned)
    {
        reader.join();
    }
    require(
        !reader_returned || (retained && retained->value == 1),
        "reader exposed an in-progress SPSC publish");
    {
        std::lock_guard<std::mutex> lock(gate->mutex);
        gate->release = true;
    }
    gate->condition.notify_all();
    writer.join();
    if (!reader_returned)
    {
        reader.join();
    }
    require(
        reader_returned,
        "reader waited for an in-progress SPSC publisher");

    const BlockingPublishFrame* published = slot.readLatest();
    require(
        published && published->value == 2,
        "completed SPSC publish was not visible");
}

void testSpscLatestValueKeepsFramesCoherent()
{
    LWSpscLatestValue<CoherentFrame> slot;
    std::atomic<bool> start{false};
    std::atomic<bool> failed{false};
    std::thread writer([&]()
    {
        while (!start.load(std::memory_order_acquire))
        {
        }
        for (std::uint64_t sequence = 1; sequence <= 50000; ++sequence)
        {
            slot.publish(
                CoherentFrame{
                    sequence,
                    std::vector<std::uint64_t>(32, sequence)});
        }
    });
    std::thread reader([&]()
    {
        while (!start.load(std::memory_order_acquire))
        {
        }
        for (int iteration = 0; iteration < 50000; ++iteration)
        {
            const CoherentFrame* frame = slot.readLatest();
            if (!frame)
            {
                continue;
            }
            if (frame->values.size() != 32)
            {
                failed.store(true, std::memory_order_release);
                return;
            }
            for (std::uint64_t value : frame->values)
            {
                if (value != frame->sequence)
                {
                    failed.store(true, std::memory_order_release);
                    return;
                }
            }
        }
    });
    start.store(true, std::memory_order_release);
    writer.join();
    reader.join();
    require(
        !failed.load(std::memory_order_acquire),
        "SPSC latest-value slot exposed a mixed frame");
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
        LWInputMailbox<TestEvent>::Snapshot snapshot;
        for (int iteration = 0;
             iteration < 50000;
             ++iteration)
        {
            if (!mailbox.read(snapshot))
            {
                continue;
            }
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

    LWInputMailbox<TestEvent>::Snapshot before_clear;
    require(mailbox.read(before_clear), "input snapshot missing before clear");
    mailbox.clear(TestEvent::None);
    LWInputMailbox<TestEvent>::Snapshot after_clear;
    require(mailbox.read(after_clear), "input snapshot missing after clear");
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
        testSpscReaderNeverWaitsForAnInProgressPublish();
        testSpscLatestValueKeepsFramesCoherent();
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
