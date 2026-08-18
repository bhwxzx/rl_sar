#ifndef FDILINK_SEQUENCE_TRACKER_H_
#define FDILINK_SEQUENCE_TRACKER_H_

#include <cstdint>

namespace FDILink
{

enum class SequenceEvent
{
  First,
  InOrder,
  ForwardGap,
  Duplicate,
  Discontinuity,
};

struct SequenceObservation
{
  SequenceEvent event = SequenceEvent::First;
  std::uint8_t expected = 0;
  std::uint8_t received = 0;
  std::uint8_t missing = 0;
};

struct SequenceStatistics
{
  std::uint64_t confirmed_lost = 0;
  std::uint64_t duplicates = 0;
  std::uint64_t discontinuities = 0;
};

std::uint64_t saturatingSequenceCountAdd(
    std::uint64_t current, std::uint64_t increment) noexcept;

class SequenceTracker
{
public:
  SequenceObservation observe(std::uint8_t received) noexcept;
  const SequenceStatistics& statistics() const noexcept;

private:
  SequenceStatistics statistics_{};
  std::uint8_t last_received_ = 0;
  bool initialized_ = false;
};

}  // namespace FDILink

#endif  // FDILINK_SEQUENCE_TRACKER_H_
