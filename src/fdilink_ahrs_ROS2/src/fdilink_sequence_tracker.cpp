#include "fdilink_sequence_tracker.h"

#include <limits>

namespace FDILink
{

std::uint64_t saturatingSequenceCountAdd(
    std::uint64_t current, std::uint64_t increment) noexcept
{
  const std::uint64_t maximum = std::numeric_limits<std::uint64_t>::max();
  if (increment > maximum - current)
  {
    return maximum;
  }
  return current + increment;
}

SequenceObservation SequenceTracker::observe(std::uint8_t received) noexcept
{
  SequenceObservation observation;
  observation.received = received;

  if (!initialized_)
  {
    observation.expected = received;
    last_received_ = received;
    initialized_ = true;
    return observation;
  }

  observation.expected = static_cast<std::uint8_t>(last_received_ + 1U);
  const std::uint8_t delta =
      static_cast<std::uint8_t>(received - last_received_);
  if (delta == 0U)
  {
    observation.event = SequenceEvent::Duplicate;
    statistics_.duplicates =
        saturatingSequenceCountAdd(statistics_.duplicates, 1U);
    return observation;
  }

  last_received_ = received;
  if (delta == 1U)
  {
    observation.event = SequenceEvent::InOrder;
    return observation;
  }

  if (delta <= 127U)
  {
    observation.event = SequenceEvent::ForwardGap;
    observation.missing = static_cast<std::uint8_t>(delta - 1U);
    statistics_.confirmed_lost = saturatingSequenceCountAdd(
        statistics_.confirmed_lost, observation.missing);
    return observation;
  }

  observation.event = SequenceEvent::Discontinuity;
  statistics_.discontinuities =
      saturatingSequenceCountAdd(statistics_.discontinuities, 1U);
  return observation;
}

const SequenceStatistics& SequenceTracker::statistics() const noexcept
{
  return statistics_;
}

}  // namespace FDILink
