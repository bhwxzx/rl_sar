#ifndef LW_CONFIG_PROFILE_HPP
#define LW_CONFIG_PROFILE_HPP

#include "loop.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <vector>

struct LWProfileDistributionSnapshot
{
    std::uint64_t count = 0;
    std::size_t retained = 0;
    double minimum_us = 0.0;
    double mean_us = 0.0;
    double p50_us = 0.0;
    double p95_us = 0.0;
    double p99_us = 0.0;
    double p999_us = 0.0;
    double maximum_us = 0.0;
};

struct LWProfileTimedSourceSnapshot
{
    bool seen = false;
    double first_sample_delay_us = 0.0;
    double last_sample_offset_us = 0.0;
    double final_age_us = 0.0;
    LWProfileDistributionSnapshot gaps;
};

// Serializes only sensor-statistic transactions, never serial disable output.
// The injected clock makes cutoff/late-callback behavior testable without sleeps.
class LWProfileSamplingWindow
{
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;
    struct Snapshot
    {
        bool started = false;
        bool closed = false;
        TimePoint start{};
        TimePoint end{};
    };

    explicit LWProfileSamplingWindow(
        std::function<TimePoint()> now = [] { return Clock::now(); })
        : now_(std::move(now)) {}

    void start()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (window_.started)
        {
            throw std::logic_error("profile sampling window already started");
        }
        window_.start = now_();
        window_.started = true;
    }

    template <typename Callback>
    bool record(Callback&& callback)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!window_.started || window_.closed)
        {
            return false;
        }
        callback(now_());
        return true;
    }

    void close()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (window_.started && !window_.closed)
        {
            // Acquiring the lock drains any admitted transaction first.
            window_.end = now_();
            window_.closed = true;
        }
    }

    Snapshot snapshot() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return window_;
    }

private:
    std::function<TimePoint()> now_;
    mutable std::mutex mutex_;
    Snapshot window_;
};

// Keeps aggregate extrema/mean for the complete run and a bounded rolling
// sample window for percentile estimates. Long profiles therefore cannot grow
// memory without bound, and the report states how many samples were retained.
class LWProfileDistribution
{
public:
    explicit LWProfileDistribution(std::size_t capacity = 200000)
        : samples_(capacity, 0.0), capacity_(capacity)
    {
        if (capacity == 0)
        {
            throw std::invalid_argument(
                "LW profile distribution capacity must be positive");
        }
    }

    template <typename Rep, typename Period>
    void record(std::chrono::duration<Rep, Period> duration)
    {
        recordMicroseconds(
            std::chrono::duration<double, std::micro>(duration).count());
    }

    void recordMicroseconds(double value)
    {
        if (!std::isfinite(value) || value < 0.0)
        {
            throw std::invalid_argument(
                "LW profile samples must be finite and nonnegative");
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (count_ == 0)
        {
            minimum_us_ = value;
            maximum_us_ = value;
        }
        else
        {
            minimum_us_ = std::min(minimum_us_, value);
            maximum_us_ = std::max(maximum_us_, value);
        }
        total_us_ += static_cast<long double>(value);
        samples_[static_cast<std::size_t>(count_ % capacity_)] = value;
        ++count_;
    }

    struct Captured
    {
        LWProfileDistributionSnapshot summary;
        std::vector<double> samples;
    };

    Captured capture() const
    {
        Captured captured;
        auto& result = captured.summary;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            result.count = count_;
            result.retained = static_cast<std::size_t>(
                std::min<std::uint64_t>(count_, capacity_));
            if (count_ == 0)
            {
                return captured;
            }
            result.minimum_us = minimum_us_;
            result.maximum_us = maximum_us_;
            result.mean_us = static_cast<double>(
                total_us_ / static_cast<long double>(count_));
            captured.samples.assign(
                samples_.begin(),
                samples_.begin()
                    + static_cast<std::ptrdiff_t>(result.retained));
        }
        return captured;
    }

    static LWProfileDistributionSnapshot summarize(Captured captured)
    {
        auto& retained_samples = captured.samples;
        auto& result = captured.summary;
        std::sort(retained_samples.begin(), retained_samples.end());
        result.p50_us = percentile(retained_samples, 0.50);
        result.p95_us = percentile(retained_samples, 0.95);
        result.p99_us = percentile(retained_samples, 0.99);
        result.p999_us = percentile(retained_samples, 0.999);
        return result;
    }

    LWProfileDistributionSnapshot snapshot() const
    {
        return summarize(capture());
    }

private:
    static double percentile(
        const std::vector<double>& sorted,
        double fraction)
    {
        if (sorted.empty())
        {
            return 0.0;
        }
        const double position =
            fraction * static_cast<double>(sorted.size() - 1);
        const auto lower = static_cast<std::size_t>(std::floor(position));
        const auto upper = static_cast<std::size_t>(std::ceil(position));
        const double blend = position - static_cast<double>(lower);
        return sorted[lower] * (1.0 - blend) + sorted[upper] * blend;
    }

    mutable std::mutex mutex_;
    std::vector<double> samples_;
    std::uint64_t capacity_ = 0;
    std::uint64_t count_ = 0;
    long double total_us_ = 0.0;
    double minimum_us_ = 0.0;
    double maximum_us_ = 0.0;
};

struct LWProfileTimedSource
{
    void mark(std::chrono::steady_clock::time_point now)
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (seen)
        {
            gaps.record(now - last);
        }
        else
        {
            first = now;
        }
        last = now;
        seen = true;
    }

    bool hasBeenSeen() const
    {
        std::lock_guard<std::mutex> lock(mutex);
        return seen;
    }

    LWProfileDistributionSnapshot snapshot() const
    {
        return gaps.snapshot();
    }

    LWProfileTimedSourceSnapshot snapshotSince(
        std::chrono::steady_clock::time_point started,
        std::chrono::steady_clock::time_point ended) const
    {
        LWProfileTimedSourceSnapshot result;
        LWProfileDistribution::Captured captured;
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (ended < started || (seen && (first < started || last > ended)))
            {
                throw std::invalid_argument("source outside profile sampling window");
            }
            result.seen = seen;
            captured = gaps.capture();
            if (seen)
            {
                result.first_sample_delay_us =
                    std::chrono::duration<double, std::micro>(first - started).count();
                result.last_sample_offset_us =
                    std::chrono::duration<double, std::micro>(last - started).count();
                result.final_age_us =
                    std::chrono::duration<double, std::micro>(ended - last).count();
            }
        }
        // Sorting must not hold either the source or sampling-window lock.
        result.gaps = LWProfileDistribution::summarize(std::move(captured));
        return result;
    }

    mutable std::mutex mutex;
    bool seen = false;
    std::chrono::steady_clock::time_point first{};
    std::chrono::steady_clock::time_point last{};
    LWProfileDistribution gaps;
};

#endif // LW_CONFIG_PROFILE_HPP
