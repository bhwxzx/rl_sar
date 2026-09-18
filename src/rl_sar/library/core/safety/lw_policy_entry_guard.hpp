#ifndef LW_POLICY_ENTRY_GUARD_HPP
#define LW_POLICY_ENTRY_GUARD_HPP

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

// Startup admission only. Existing runtime/fatal safety decisions retain priority.
class LWPolicyEntryGuard
{
public:
    using Clock = std::chrono::steady_clock;
    using Time = Clock::time_point;
    enum class Reason { Incomplete, Invalid, Stale, Tilted, Settling, Ready };
    Reason reason = Reason::Incomplete;
    double roll = 0.0, pitch = 0.0, stable_seconds = 0.0;

    void reset() { *this = LWPolicyEntryGuard{}; }
    bool ready() const { return reason == Reason::Ready; }
    const char* description() const
    {
        switch (reason) {
        case Reason::Incomplete: return "GetUp尚未完成";
        case Reason::Invalid: return "姿态数据无效";
        case Reason::Stale: return "姿态过期或采样不连续";
        case Reason::Tilted: return "机身倾角超限";
        case Reason::Settling: return "连续稳定时间不足";
        case Reason::Ready: return "已满足启动条件";
        }
        return "未知";
    }
    void observe(const std::vector<float>& q, Time sample, Time now,
                 bool completed, double limit, double hold, double max_age)
    {
        roll = pitch = std::numeric_limits<double>::quiet_NaN();
        if (!completed) { reject(Reason::Incomplete); return; }
        if (!std::isfinite(limit) || limit <= 0 || limit >= 90
            || !std::isfinite(hold) || hold <= 0
            || !std::isfinite(max_age) || max_age <= 0 || q.size() != 4) {
            reject(Reason::Invalid); return;
        }
        double norm2 = 0;
        for (float v : q) { if (!std::isfinite(v)) { reject(Reason::Invalid); return; } norm2 += double(v)*v; }
        if (norm2 < 0.25 || norm2 > 2.25) { reject(Reason::Invalid); return; }
        const double n = std::sqrt(norm2);
        const double w=q[0]/n, x=q[1]/n, y=q[2]/n, z=q[3]/n;
        constexpr double degrees = 57.29577951308232;
        roll = std::atan2(2*(w*x+y*z), 1-2*(x*x+y*y))*degrees;
        pitch = std::asin(std::clamp(2*(w*y-z*x), -1.0, 1.0))*degrees;
        if (sample == Time{} || sample > now || seconds(now-sample) > max_age
            || (last_now_ != Time{} && now < last_now_)) {
            reject(Reason::Stale); last_now_ = now; return;
        }
        last_now_ = now;
        // Float quaternion conversion can place an exact boundary a few ulps outside.
        if (std::fabs(roll) > limit + 1e-5 || std::fabs(pitch) > limit + 1e-5) {
            reject(Reason::Tilted); return;
        }
        if (last_sample_ != Time{} && (sample < last_sample_
            || seconds(sample-last_sample_) > max_age)) {
            reject(Reason::Stale);
        }
        if (first_sample_ == Time{}) first_sample_ = now;
        last_sample_ = sample;
        // Re-reading a cached quaternion cannot advance the stable interval.
        stable_seconds = std::max(0.0, seconds(sample-first_sample_));
        reason = stable_seconds + 1e-9 >= hold ? Reason::Ready : Reason::Settling;
    }
private:
    static double seconds(Clock::duration d) { return std::chrono::duration<double>(d).count(); }
    void reject(Reason why) { reason=why; first_sample_={}; last_sample_={}; stable_seconds=0; }
    Time first_sample_{}, last_sample_{}, last_now_{};
};
#endif
