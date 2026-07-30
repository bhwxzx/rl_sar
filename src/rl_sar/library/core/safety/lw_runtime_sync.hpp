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

#endif // LW_RUNTIME_SYNC_HPP
