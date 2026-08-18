#include "fdilink_frame_parser.h"

#include <algorithm>

#include "crc_table.h"

namespace FDILink
{

bool FrameParser::isSupportedType(std::uint8_t type) noexcept
{
  return type == TYPE_IMU || type == TYPE_AHRS || type == TYPE_INSGPS
      || type == TYPE_GEODETIC_POS;
}

bool FrameParser::isIgnoredType(std::uint8_t type) noexcept
{
  return type == TYPE_GROUND || type == TYPE_GROUND_EXTENDED;
}

bool FrameParser::lengthMatchesType(
    std::uint8_t type, std::uint8_t payload_size) noexcept
{
  switch (type)
  {
    case TYPE_IMU:
      return payload_size == IMU_LEN;
    case TYPE_AHRS:
      return payload_size == AHRS_LEN;
    case TYPE_INSGPS:
      return payload_size == INSGPS_LEN;
    case TYPE_GEODETIC_POS:
      return payload_size == GEODETIC_POS_LEN;
    default:
      return isIgnoredType(type);
  }
}

FrameParserEvent FrameParser::reject(std::uint8_t current_byte) noexcept
{
  reset();
  if (current_byte == FRAME_HEAD)
  {
    buffer_[0] = FRAME_HEAD;
    size_ = 1;
  }
  return FrameParserEvent::Rejected;
}

FrameParserEvent FrameParser::consume(
    std::uint8_t byte, ValidatedFrame& output)
{
  if (size_ == 0)
  {
    if (byte == FRAME_HEAD)
    {
      buffer_[0] = byte;
      size_ = 1;
    }
    return FrameParserEvent::None;
  }

  if (size_ >= buffer_.size())
  {
    return reject(byte);
  }

  buffer_[size_++] = byte;

  if (size_ == 2)
  {
    if (!isSupportedType(buffer_[1]) && !isIgnoredType(buffer_[1]))
    {
      return reject(byte);
    }
    return FrameParserEvent::None;
  }

  if (size_ == 3)
  {
    if (!lengthMatchesType(buffer_[1], buffer_[2]))
    {
      return reject(byte);
    }
    expected_size_ = FDILINK_HEADER_SIZE
        + static_cast<std::size_t>(buffer_[2]) + 1;
    if (expected_size_ > buffer_.size())
    {
      return reject(byte);
    }
    return FrameParserEvent::None;
  }

  if (size_ == 5)
  {
    const std::uint8_t expected_crc8 = CRC8_Table(buffer_.data(), 4);
    if (buffer_[4] != expected_crc8)
    {
      return reject(byte);
    }
  }

  if (expected_size_ == 0 || size_ < expected_size_)
  {
    return FrameParserEvent::None;
  }
  if (size_ != expected_size_ || buffer_[expected_size_ - 1] != FRAME_END)
  {
    return reject(byte);
  }

  const std::uint16_t expected_crc16 =
      (static_cast<std::uint16_t>(buffer_[5]) << 8)
      | static_cast<std::uint16_t>(buffer_[6]);
  const std::uint16_t actual_crc16 = CRC16_Table(
      buffer_.data() + FDILINK_HEADER_SIZE, buffer_[2]);
  if (actual_crc16 != expected_crc16)
  {
    return reject(byte);
  }

  output = ValidatedFrame{};
  output.type = buffer_[1];
  output.payload_size = buffer_[2];
  output.serial_number = buffer_[3];
  output.frame_size = expected_size_;
  std::copy_n(buffer_.begin(), expected_size_, output.bytes.begin());
  reset();
  return FrameParserEvent::FrameReady;
}

void FrameParser::reset() noexcept
{
  size_ = 0;
  expected_size_ = 0;
}

bool FrameParser::hasPartialFrame() const noexcept
{
  return size_ != 0;
}

}  // namespace FDILink
