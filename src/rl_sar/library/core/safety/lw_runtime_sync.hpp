#ifndef LW_RUNTIME_SYNC_HPP
#define LW_RUNTIME_SYNC_HPP

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <utility>

template <typename T>
class LWSnapshotBuffer
{
public:
    void publish(const T& value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        value_ = value;
        valid_ = true;
    }

    bool read(T& value) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!valid_)
        {
            return false;
        }
        value = value_;
        return true;
    }

    bool valid() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return valid_;
    }

private:
    mutable std::mutex mutex_;
    T value_{};
    bool valid_ = false;
};

template <typename T>
class LWAtomicSnapshot
{
public:
    std::shared_ptr<const T> load() const noexcept
    {
        return std::atomic_load_explicit(
            &value_,
            std::memory_order_acquire);
    }

    void store(std::shared_ptr<const T> value) noexcept
    {
        std::atomic_store_explicit(
            &value_,
            std::move(value),
            std::memory_order_release);
    }

    void clear() noexcept
    {
        store(nullptr);
    }

private:
    std::shared_ptr<const T> value_;
};

template <typename Event>
struct LWInputSnapshot
{
    float x = 0.0f;
    float y = 0.0f;
    float yaw = 0.0f;
    Event event{};
    std::uint64_t event_sequence = 0;
};

template <typename Event>
class LWInputMailbox
{
public:
    using Snapshot = LWInputSnapshot<Event>;

    void publishVelocity(float x, float y, float yaw)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_.x = x;
        snapshot_.y = y;
        snapshot_.yaw = yaw;
    }

    void publishEvent(Event event)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_.event = event;
        ++snapshot_.event_sequence;
    }

    void clear(Event none)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_.x = 0.0f;
        snapshot_.y = 0.0f;
        snapshot_.yaw = 0.0f;
        snapshot_.event = none;
        ++snapshot_.event_sequence;
    }

    Snapshot read() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return snapshot_;
    }

private:
    mutable std::mutex mutex_;
    Snapshot snapshot_{};
};

enum class LWOperatorMode : int
{
    Unknown = 0,
    Passive,
    GetUpLeg,
    GetUpWheel,
    GetDown,
    LegLocomotion,
    WheelLocomotion,
    LegToWheel,
    WheelToLeg,
};

struct LWOperatorStatusSnapshot
{
    std::uint64_t sequence = 0;
    LWOperatorMode mode = LWOperatorMode::Unknown;
    float x = 0.0f;
    float y = 0.0f;
    float yaw = 0.0f;
    float gait_frequency = 0.0f;
    float progress = 0.0f;
};

class LWOperatorStatusMailbox
{
public:
    // One control-loop writer publishes status while ROS diagnostics may read
    // concurrently. Sequentially consistent atomics make the revision check a
    // coherent, nonblocking snapshot boundary without a control-path mutex.
    void publish(const LWOperatorStatusSnapshot& status) noexcept
    {
        revision_.fetch_add(1);
        sequence_.store(status.sequence);
        mode_.store(static_cast<int>(status.mode));
        x_.store(status.x);
        y_.store(status.y);
        yaw_.store(status.yaw);
        gait_frequency_.store(status.gait_frequency);
        progress_.store(status.progress);
        revision_.fetch_add(1);
    }

    bool read(LWOperatorStatusSnapshot& status) const noexcept
    {
        for (int attempt = 0; attempt < 8; ++attempt)
        {
            const std::uint64_t before = revision_.load();
            if ((before & 1U) != 0U)
            {
                continue;
            }

            LWOperatorStatusSnapshot candidate;
            candidate.sequence = sequence_.load();
            candidate.mode = static_cast<LWOperatorMode>(mode_.load());
            candidate.x = x_.load();
            candidate.y = y_.load();
            candidate.yaw = yaw_.load();
            candidate.gait_frequency = gait_frequency_.load();
            candidate.progress = progress_.load();

            const std::uint64_t after = revision_.load();
            if (before == after && (after & 1U) == 0U && after != 0)
            {
                status = candidate;
                return true;
            }
        }
        return false;
    }

private:
    std::atomic<std::uint64_t> revision_{0};
    std::atomic<std::uint64_t> sequence_{0};
    std::atomic<int> mode_{static_cast<int>(LWOperatorMode::Unknown)};
    std::atomic<float> x_{0.0f};
    std::atomic<float> y_{0.0f};
    std::atomic<float> yaw_{0.0f};
    std::atomic<float> gait_frequency_{0.0f};
    std::atomic<float> progress_{0.0f};
};

#endif // LW_RUNTIME_SYNC_HPP
