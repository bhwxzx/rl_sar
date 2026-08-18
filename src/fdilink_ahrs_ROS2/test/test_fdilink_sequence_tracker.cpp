#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

#include "fdilink_sequence_tracker.h"

namespace
{

TEST(FDILinkSequenceTracker, EstablishesBaselineThenAcceptsInOrderFrames)
{
  FDILink::SequenceTracker tracker;

  const auto first = tracker.observe(42U);
  EXPECT_EQ(first.event, FDILink::SequenceEvent::First);
  EXPECT_EQ(first.expected, 42U);
  EXPECT_EQ(first.received, 42U);

  const auto next = tracker.observe(43U);
  EXPECT_EQ(next.event, FDILink::SequenceEvent::InOrder);
  EXPECT_EQ(next.expected, 43U);
  EXPECT_EQ(next.missing, 0U);
}

TEST(FDILinkSequenceTracker, TreatsWraparoundAsInOrder)
{
  FDILink::SequenceTracker tracker;
  tracker.observe(255U);

  const auto observation = tracker.observe(0U);
  EXPECT_EQ(observation.event, FDILink::SequenceEvent::InOrder);
  EXPECT_EQ(observation.expected, 0U);
  EXPECT_EQ(tracker.statistics().confirmed_lost, 0U);
}

TEST(FDILinkSequenceTracker, CountsOnlyConfirmedMissingFramesInForwardGaps)
{
  FDILink::SequenceTracker tracker;
  tracker.observe(10U);

  auto observation = tracker.observe(12U);
  EXPECT_EQ(observation.event, FDILink::SequenceEvent::ForwardGap);
  EXPECT_EQ(observation.missing, 1U);
  EXPECT_EQ(tracker.statistics().confirmed_lost, 1U);

  observation = tracker.observe(16U);
  EXPECT_EQ(observation.event, FDILink::SequenceEvent::ForwardGap);
  EXPECT_EQ(observation.missing, 3U);
  EXPECT_EQ(tracker.statistics().confirmed_lost, 4U);
}

TEST(FDILinkSequenceTracker, CountsDuplicateWithoutInventingLoss)
{
  FDILink::SequenceTracker tracker;
  tracker.observe(10U);

  const auto observation = tracker.observe(10U);
  EXPECT_EQ(observation.event, FDILink::SequenceEvent::Duplicate);
  EXPECT_EQ(observation.expected, 11U);
  EXPECT_EQ(tracker.statistics().duplicates, 1U);
  EXPECT_EQ(tracker.statistics().confirmed_lost, 0U);

  EXPECT_EQ(
      tracker.observe(11U).event, FDILink::SequenceEvent::InOrder);
}

TEST(FDILinkSequenceTracker, ClassifiesBackwardAndHalfRangeJumpsAsDiscontinuities)
{
  FDILink::SequenceTracker tracker;
  tracker.observe(100U);

  auto observation = tracker.observe(99U);
  EXPECT_EQ(observation.event, FDILink::SequenceEvent::Discontinuity);
  EXPECT_EQ(tracker.statistics().confirmed_lost, 0U);

  observation = tracker.observe(227U);
  EXPECT_EQ(observation.event, FDILink::SequenceEvent::Discontinuity);
  EXPECT_EQ(tracker.statistics().discontinuities, 2U);
  EXPECT_EQ(tracker.statistics().confirmed_lost, 0U);
}

TEST(FDILinkSequenceTracker, RebasesAfterDiscontinuity)
{
  FDILink::SequenceTracker tracker;
  tracker.observe(200U);
  EXPECT_EQ(
      tracker.observe(100U).event, FDILink::SequenceEvent::Discontinuity);

  const auto recovered = tracker.observe(101U);
  EXPECT_EQ(recovered.event, FDILink::SequenceEvent::InOrder);
  EXPECT_EQ(recovered.expected, 101U);
}

TEST(FDILinkSequenceTracker, SaturatingCounterAdditionCannotWrap)
{
  const std::uint64_t maximum = std::numeric_limits<std::uint64_t>::max();
  EXPECT_EQ(FDILink::saturatingSequenceCountAdd(maximum, 1U), maximum);
  EXPECT_EQ(FDILink::saturatingSequenceCountAdd(maximum - 1U, 2U), maximum);
  EXPECT_EQ(FDILink::saturatingSequenceCountAdd(maximum - 2U, 2U), maximum);
}

}  // namespace
