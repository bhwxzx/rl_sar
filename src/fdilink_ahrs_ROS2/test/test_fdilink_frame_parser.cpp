#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "crc_table.h"
#include "fdilink_frame_parser.h"

namespace
{

std::vector<std::uint8_t> makeFrame(
    std::uint8_t type,
    std::uint8_t payload_size,
    std::uint8_t serial_number)
{
  std::vector<std::uint8_t> frame(
      FDILink::FDILINK_HEADER_SIZE
          + static_cast<std::size_t>(payload_size) + 1,
      0);
  frame[0] = FDILink::FRAME_HEAD;
  frame[1] = type;
  frame[2] = payload_size;
  frame[3] = serial_number;
  for (std::size_t index = 0; index < payload_size; ++index)
  {
    frame[FDILink::FDILINK_HEADER_SIZE + index] =
        static_cast<std::uint8_t>((index * 17U + serial_number) & 0xffU);
  }
  frame[4] = CRC8_Table(frame.data(), 4);
  const std::uint16_t crc16 = CRC16_Table(
      frame.data() + FDILink::FDILINK_HEADER_SIZE, payload_size);
  frame[5] = static_cast<std::uint8_t>(crc16 >> 8);
  frame[6] = static_cast<std::uint8_t>(crc16 & 0xffU);
  frame.back() = FDILink::FRAME_END;
  return frame;
}

std::vector<FDILink::ValidatedFrame> feed(
    FDILink::FrameParser& parser,
    const std::vector<std::uint8_t>& bytes,
    int* rejected = nullptr)
{
  std::vector<FDILink::ValidatedFrame> frames;
  for (const std::uint8_t byte : bytes)
  {
    FDILink::ValidatedFrame frame;
    const FDILink::FrameParserEvent event = parser.consume(byte, frame);
    if (event == FDILink::FrameParserEvent::FrameReady)
    {
      frames.push_back(frame);
    }
    else if (event == FDILink::FrameParserEvent::Rejected
             && rejected != nullptr)
    {
      ++(*rejected);
    }
  }
  return frames;
}

TEST(FDILinkFrameParser, AcceptsEverySupportedFrameType)
{
  const std::array<std::pair<std::uint8_t, std::uint8_t>, 4> cases{{
      {FDILink::TYPE_IMU, FDILink::IMU_LEN},
      {FDILink::TYPE_AHRS, FDILink::AHRS_LEN},
      {FDILink::TYPE_INSGPS, FDILink::INSGPS_LEN},
      {FDILink::TYPE_GEODETIC_POS, FDILink::GEODETIC_POS_LEN},
  }};

  FDILink::FrameParser parser;
  std::uint8_t serial_number = 10;
  for (const auto& item : cases)
  {
    const auto source = makeFrame(item.first, item.second, serial_number);
    const auto frames = feed(parser, source);
    ASSERT_EQ(frames.size(), 1U);
    EXPECT_EQ(frames[0].type, item.first);
    EXPECT_EQ(frames[0].payload_size, item.second);
    EXPECT_EQ(frames[0].serial_number, serial_number);
    EXPECT_EQ(frames[0].frame_size, source.size());
    EXPECT_TRUE(std::equal(
        source.begin(), source.end(), frames[0].bytes.begin()));
    ++serial_number;
  }
}

TEST(FDILinkFrameParser, DoesNotEmitAnyIncompletePrefix)
{
  const auto source =
      makeFrame(FDILink::TYPE_AHRS, FDILink::AHRS_LEN, 21);

  for (std::size_t split = 0; split < source.size(); ++split)
  {
    FDILink::FrameParser parser;
    const std::vector<std::uint8_t> prefix(
        source.begin(), source.begin() + static_cast<std::ptrdiff_t>(split));
    EXPECT_TRUE(feed(parser, prefix).empty()) << "split=" << split;

    const std::vector<std::uint8_t> suffix(
        source.begin() + static_cast<std::ptrdiff_t>(split), source.end());
    const auto frames = feed(parser, suffix);
    ASSERT_EQ(frames.size(), 1U) << "split=" << split;
    EXPECT_EQ(frames[0].serial_number, 21);
  }
}

TEST(FDILinkFrameParser, HandlesNoiseAndBackToBackFrames)
{
  std::vector<std::uint8_t> stream{0x00, 0x7f, 0xfe};
  const auto first = makeFrame(FDILink::TYPE_IMU, FDILink::IMU_LEN, 30);
  const auto second = makeFrame(FDILink::TYPE_AHRS, FDILink::AHRS_LEN, 31);
  stream.insert(stream.end(), first.begin(), first.end());
  stream.insert(stream.end(), second.begin(), second.end());

  FDILink::FrameParser parser;
  const auto frames = feed(parser, stream);
  ASSERT_EQ(frames.size(), 2U);
  EXPECT_EQ(frames[0].type, FDILink::TYPE_IMU);
  EXPECT_EQ(frames[1].type, FDILink::TYPE_AHRS);
}

TEST(FDILinkFrameParser, RejectsMalformedFramesAndRecovers)
{
  const auto valid = makeFrame(FDILink::TYPE_IMU, FDILink::IMU_LEN, 40);
  std::vector<std::vector<std::uint8_t>> invalid;

  auto bad_type = valid;
  bad_type[1] = 0x99;
  invalid.push_back(bad_type);

  auto bad_length = valid;
  bad_length[2] = FDILink::AHRS_LEN;
  invalid.push_back(bad_length);

  auto bad_crc8 = valid;
  bad_crc8[4] ^= 0x01;
  invalid.push_back(bad_crc8);

  auto bad_crc16 = valid;
  bad_crc16[5] ^= 0x01;
  invalid.push_back(bad_crc16);

  auto bad_end = valid;
  bad_end.back() = 0x00;
  invalid.push_back(bad_end);

  for (const auto& malformed : invalid)
  {
    FDILink::FrameParser parser;
    int rejected = 0;
    EXPECT_TRUE(feed(parser, malformed, &rejected).empty());
    EXPECT_GT(rejected, 0);

    const auto recovered = feed(parser, valid, &rejected);
    ASSERT_EQ(recovered.size(), 1U);
    EXPECT_EQ(recovered[0].serial_number, 40);
  }
}

TEST(FDILinkFrameParser, TimeoutResetPreventsCrossReadFrameJoining)
{
  const auto source =
      makeFrame(FDILink::TYPE_AHRS, FDILink::AHRS_LEN, 50);
  const std::size_t split = source.size() / 2;

  FDILink::FrameParser parser;
  EXPECT_TRUE(feed(
      parser,
      std::vector<std::uint8_t>(source.begin(), source.begin() + split))
                  .empty());
  ASSERT_TRUE(parser.hasPartialFrame());
  parser.reset();
  EXPECT_FALSE(parser.hasPartialFrame());

  EXPECT_TRUE(feed(
      parser,
      std::vector<std::uint8_t>(source.begin() + split, source.end()))
                  .empty());
  const auto recovered = feed(parser, source);
  ASSERT_EQ(recovered.size(), 1U);
}

TEST(FDILinkFrameParser, ConsumesIgnoredFramesWithoutPublishingSensorData)
{
  FDILink::FrameParser parser;
  const auto source = makeFrame(FDILink::TYPE_GROUND, 12, 60);
  const auto frames = feed(parser, source);
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0].type, FDILink::TYPE_GROUND);
  EXPECT_EQ(frames[0].serial_number, 60);
}

}  // namespace
