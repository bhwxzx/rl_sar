#include "fdilink_payload_decoder.h"

#include <cstddef>
#include <cstring>
#include <limits>

namespace FDILink
{
namespace
{

static_assert(sizeof(float) == 4, "FDILink requires 32-bit float");
static_assert(sizeof(double) == 8, "FDILink requires 64-bit double");
static_assert(sizeof(std::int64_t) == 8, "FDILink requires 64-bit int64_t");
static_assert(
    std::numeric_limits<float>::is_iec559,
    "FDILink requires IEC 559 float");
static_assert(
    std::numeric_limits<double>::is_iec559,
    "FDILink requires IEC 559 double");

std::uint32_t readUint32LittleEndian(const std::uint8_t* bytes) noexcept
{
  return static_cast<std::uint32_t>(bytes[0])
      | (static_cast<std::uint32_t>(bytes[1]) << 8U)
      | (static_cast<std::uint32_t>(bytes[2]) << 16U)
      | (static_cast<std::uint32_t>(bytes[3]) << 24U);
}

std::uint64_t readUint64LittleEndian(const std::uint8_t* bytes) noexcept
{
  std::uint64_t value = 0;
  for (std::size_t index = 0; index < 8; ++index)
  {
    value |= static_cast<std::uint64_t>(bytes[index]) << (index * 8U);
  }
  return value;
}

float readFloatLittleEndian(const std::uint8_t* bytes) noexcept
{
  const std::uint32_t bits = readUint32LittleEndian(bytes);
  float value = 0.0F;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

double readDoubleLittleEndian(const std::uint8_t* bytes) noexcept
{
  const std::uint64_t bits = readUint64LittleEndian(bytes);
  double value = 0.0;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

std::int64_t readInt64LittleEndian(const std::uint8_t* bytes) noexcept
{
  const std::uint64_t bits = readUint64LittleEndian(bytes);
  if (bits <= static_cast<std::uint64_t>(
      std::numeric_limits<std::int64_t>::max()))
  {
    return static_cast<std::int64_t>(bits);
  }
  return -1 - static_cast<std::int64_t>(~bits);
}

bool hasExpectedLayout(
    const ValidatedFrame& frame,
    std::uint8_t expected_type,
    std::uint8_t expected_payload_size) noexcept
{
  const std::size_t expected_frame_size =
      FDILINK_HEADER_SIZE
      + static_cast<std::size_t>(expected_payload_size) + 1;
  return frame.type == expected_type
      && frame.payload_size == expected_payload_size
      && frame.frame_size == expected_frame_size
      && frame.bytes[0] == FRAME_HEAD
      && frame.bytes[1] == expected_type
      && frame.bytes[2] == expected_payload_size
      && frame.bytes[expected_frame_size - 1] == FRAME_END;
}

float readFloatAndAdvance(const std::uint8_t*& cursor) noexcept
{
  const float value = readFloatLittleEndian(cursor);
  cursor += sizeof(float);
  return value;
}

}  // namespace

bool decodeImuPayload(
    const ValidatedFrame& frame, ImuPayload& output) noexcept
{
  if (!hasExpectedLayout(frame, TYPE_IMU, IMU_LEN))
  {
    return false;
  }

  const std::uint8_t* cursor = frame.bytes.data() + FDILINK_HEADER_SIZE;
  ImuPayload decoded;
  decoded.gyroscope_x = readFloatAndAdvance(cursor);
  decoded.gyroscope_y = readFloatAndAdvance(cursor);
  decoded.gyroscope_z = readFloatAndAdvance(cursor);
  decoded.accelerometer_x = readFloatAndAdvance(cursor);
  decoded.accelerometer_y = readFloatAndAdvance(cursor);
  decoded.accelerometer_z = readFloatAndAdvance(cursor);
  decoded.magnetometer_x = readFloatAndAdvance(cursor);
  decoded.magnetometer_y = readFloatAndAdvance(cursor);
  decoded.magnetometer_z = readFloatAndAdvance(cursor);
  decoded.imu_temperature = readFloatAndAdvance(cursor);
  decoded.pressure = readFloatAndAdvance(cursor);
  decoded.pressure_temperature = readFloatAndAdvance(cursor);
  decoded.timestamp = readInt64LittleEndian(cursor);
  output = decoded;
  return true;
}

bool decodeAhrsPayload(
    const ValidatedFrame& frame, AhrsPayload& output) noexcept
{
  if (!hasExpectedLayout(frame, TYPE_AHRS, AHRS_LEN))
  {
    return false;
  }

  const std::uint8_t* cursor = frame.bytes.data() + FDILINK_HEADER_SIZE;
  AhrsPayload decoded;
  decoded.roll_speed = readFloatAndAdvance(cursor);
  decoded.pitch_speed = readFloatAndAdvance(cursor);
  decoded.heading_speed = readFloatAndAdvance(cursor);
  decoded.roll = readFloatAndAdvance(cursor);
  decoded.pitch = readFloatAndAdvance(cursor);
  decoded.heading = readFloatAndAdvance(cursor);
  decoded.quaternion_w = readFloatAndAdvance(cursor);
  decoded.quaternion_x = readFloatAndAdvance(cursor);
  decoded.quaternion_y = readFloatAndAdvance(cursor);
  decoded.quaternion_z = readFloatAndAdvance(cursor);
  decoded.timestamp = readInt64LittleEndian(cursor);
  output = decoded;
  return true;
}

bool decodeInsGpsPayload(
    const ValidatedFrame& frame, InsGpsPayload& output) noexcept
{
  if (!hasExpectedLayout(frame, TYPE_INSGPS, INSGPS_LEN))
  {
    return false;
  }

  const std::uint8_t* cursor = frame.bytes.data() + FDILINK_HEADER_SIZE;
  InsGpsPayload decoded;
  decoded.body_velocity_x = readFloatAndAdvance(cursor);
  decoded.body_velocity_y = readFloatAndAdvance(cursor);
  decoded.body_velocity_z = readFloatAndAdvance(cursor);
  decoded.body_acceleration_x = readFloatAndAdvance(cursor);
  decoded.body_acceleration_y = readFloatAndAdvance(cursor);
  decoded.body_acceleration_z = readFloatAndAdvance(cursor);
  decoded.location_north = readFloatAndAdvance(cursor);
  decoded.location_east = readFloatAndAdvance(cursor);
  decoded.location_down = readFloatAndAdvance(cursor);
  decoded.velocity_north = readFloatAndAdvance(cursor);
  decoded.velocity_east = readFloatAndAdvance(cursor);
  decoded.velocity_down = readFloatAndAdvance(cursor);
  decoded.acceleration_north = readFloatAndAdvance(cursor);
  decoded.acceleration_east = readFloatAndAdvance(cursor);
  decoded.acceleration_down = readFloatAndAdvance(cursor);
  decoded.pressure_altitude = readFloatAndAdvance(cursor);
  decoded.timestamp = readInt64LittleEndian(cursor);
  output = decoded;
  return true;
}

bool decodeGeodeticPositionPayload(
    const ValidatedFrame& frame,
    GeodeticPositionPayload& output) noexcept
{
  if (!hasExpectedLayout(frame, TYPE_GEODETIC_POS, GEODETIC_POS_LEN))
  {
    return false;
  }

  const std::uint8_t* cursor = frame.bytes.data() + FDILINK_HEADER_SIZE;
  GeodeticPositionPayload decoded;
  decoded.latitude = readDoubleLittleEndian(cursor);
  cursor += sizeof(double);
  decoded.longitude = readDoubleLittleEndian(cursor);
  cursor += sizeof(double);
  decoded.height = readDoubleLittleEndian(cursor);
  cursor += sizeof(double);
  decoded.horizontal_accuracy = readFloatAndAdvance(cursor);
  decoded.vertical_accuracy = readFloatAndAdvance(cursor);
  output = decoded;
  return true;
}

}  // namespace FDILink
