#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

#include "fdilink_payload_decoder.h"

namespace
{

template <std::size_t PayloadSize>
FDILink::ValidatedFrame makeValidatedFrame(
    std::uint8_t type,
    const std::array<std::uint8_t, PayloadSize>& payload)
{
  static_assert(PayloadSize <= 255, "FDILink payload length must fit uint8_t");
  FDILink::ValidatedFrame frame;
  frame.type = type;
  frame.serial_number = 0x5a;
  frame.payload_size = static_cast<std::uint8_t>(PayloadSize);
  frame.frame_size = FDILink::FDILINK_HEADER_SIZE + PayloadSize + 1;
  frame.bytes[0] = FDILink::FRAME_HEAD;
  frame.bytes[1] = type;
  frame.bytes[2] = frame.payload_size;
  frame.bytes[3] = frame.serial_number;
  std::copy(
      payload.begin(), payload.end(),
      frame.bytes.begin() + FDILink::FDILINK_HEADER_SIZE);
  frame.bytes[frame.frame_size - 1] = FDILink::FRAME_END;
  return frame;
}

TEST(FDILinkPayloadDecoder, DecodesEveryImuFieldFromFixedLittleEndianBytes)
{
  const std::array<std::uint8_t, FDILink::IMU_LEN> payload{{
      0x00, 0x00, 0x80, 0x3f,
      0x00, 0x00, 0x00, 0xc0,
      0x00, 0x00, 0x00, 0x3f,
      0x00, 0x00, 0x80, 0xbe,
      0x00, 0x00, 0x40, 0x40,
      0x00, 0x00, 0x80, 0xc0,
      0x00, 0x00, 0xa0, 0x40,
      0x00, 0x00, 0xc0, 0xc0,
      0x00, 0x00, 0xe0, 0x40,
      0x00, 0x00, 0x00, 0x41,
      0x00, 0x00, 0x10, 0x41,
      0x00, 0x00, 0x20, 0xc1,
      0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01,
  }};
  const auto frame = makeValidatedFrame(FDILink::TYPE_IMU, payload);

  FDILink::ImuPayload decoded;
  ASSERT_TRUE(FDILink::decodeImuPayload(frame, decoded));
  EXPECT_FLOAT_EQ(decoded.gyroscope_x, 1.0F);
  EXPECT_FLOAT_EQ(decoded.gyroscope_y, -2.0F);
  EXPECT_FLOAT_EQ(decoded.gyroscope_z, 0.5F);
  EXPECT_FLOAT_EQ(decoded.accelerometer_x, -0.25F);
  EXPECT_FLOAT_EQ(decoded.accelerometer_y, 3.0F);
  EXPECT_FLOAT_EQ(decoded.accelerometer_z, -4.0F);
  EXPECT_FLOAT_EQ(decoded.magnetometer_x, 5.0F);
  EXPECT_FLOAT_EQ(decoded.magnetometer_y, -6.0F);
  EXPECT_FLOAT_EQ(decoded.magnetometer_z, 7.0F);
  EXPECT_FLOAT_EQ(decoded.imu_temperature, 8.0F);
  EXPECT_FLOAT_EQ(decoded.pressure, 9.0F);
  EXPECT_FLOAT_EQ(decoded.pressure_temperature, -10.0F);
  EXPECT_EQ(decoded.timestamp, INT64_C(0x0102030405060708));
}

TEST(FDILinkPayloadDecoder, DecodesEveryAhrsFieldFromFixedLittleEndianBytes)
{
  const std::array<std::uint8_t, FDILink::AHRS_LEN> payload{{
      0x00, 0x00, 0x80, 0xbf,
      0x00, 0x00, 0x00, 0x40,
      0x00, 0x00, 0x40, 0xc0,
      0x00, 0x00, 0x80, 0x40,
      0x00, 0x00, 0xa0, 0xc0,
      0x00, 0x00, 0xc0, 0x40,
      0x00, 0x00, 0x00, 0x3f,
      0x00, 0x00, 0x00, 0xbf,
      0x00, 0x00, 0x80, 0x3e,
      0x00, 0x00, 0x80, 0xbe,
      0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  }};
  const auto frame = makeValidatedFrame(FDILink::TYPE_AHRS, payload);

  FDILink::AhrsPayload decoded;
  ASSERT_TRUE(FDILink::decodeAhrsPayload(frame, decoded));
  EXPECT_FLOAT_EQ(decoded.roll_speed, -1.0F);
  EXPECT_FLOAT_EQ(decoded.pitch_speed, 2.0F);
  EXPECT_FLOAT_EQ(decoded.heading_speed, -3.0F);
  EXPECT_FLOAT_EQ(decoded.roll, 4.0F);
  EXPECT_FLOAT_EQ(decoded.pitch, -5.0F);
  EXPECT_FLOAT_EQ(decoded.heading, 6.0F);
  EXPECT_FLOAT_EQ(decoded.quaternion_w, 0.5F);
  EXPECT_FLOAT_EQ(decoded.quaternion_x, -0.5F);
  EXPECT_FLOAT_EQ(decoded.quaternion_y, 0.25F);
  EXPECT_FLOAT_EQ(decoded.quaternion_z, -0.25F);
  EXPECT_EQ(decoded.timestamp, -2);
}

TEST(FDILinkPayloadDecoder, DecodesEveryInsGpsFieldFromFixedLittleEndianBytes)
{
  const std::array<std::uint8_t, FDILink::INSGPS_LEN> payload{{
      0x00, 0x00, 0x80, 0x3f,
      0x00, 0x00, 0x00, 0x40,
      0x00, 0x00, 0x40, 0x40,
      0x00, 0x00, 0x80, 0x40,
      0x00, 0x00, 0xa0, 0x40,
      0x00, 0x00, 0xc0, 0x40,
      0x00, 0x00, 0xe0, 0x40,
      0x00, 0x00, 0x00, 0x41,
      0x00, 0x00, 0x10, 0x41,
      0x00, 0x00, 0x20, 0x41,
      0x00, 0x00, 0x30, 0x41,
      0x00, 0x00, 0x40, 0x41,
      0x00, 0x00, 0x50, 0x41,
      0x00, 0x00, 0x60, 0x41,
      0x00, 0x00, 0x70, 0x41,
      0x00, 0x00, 0x80, 0xc1,
      0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11,
  }};
  const auto frame = makeValidatedFrame(FDILink::TYPE_INSGPS, payload);

  FDILink::InsGpsPayload decoded;
  ASSERT_TRUE(FDILink::decodeInsGpsPayload(frame, decoded));
  EXPECT_FLOAT_EQ(decoded.body_velocity_x, 1.0F);
  EXPECT_FLOAT_EQ(decoded.body_velocity_y, 2.0F);
  EXPECT_FLOAT_EQ(decoded.body_velocity_z, 3.0F);
  EXPECT_FLOAT_EQ(decoded.body_acceleration_x, 4.0F);
  EXPECT_FLOAT_EQ(decoded.body_acceleration_y, 5.0F);
  EXPECT_FLOAT_EQ(decoded.body_acceleration_z, 6.0F);
  EXPECT_FLOAT_EQ(decoded.location_north, 7.0F);
  EXPECT_FLOAT_EQ(decoded.location_east, 8.0F);
  EXPECT_FLOAT_EQ(decoded.location_down, 9.0F);
  EXPECT_FLOAT_EQ(decoded.velocity_north, 10.0F);
  EXPECT_FLOAT_EQ(decoded.velocity_east, 11.0F);
  EXPECT_FLOAT_EQ(decoded.velocity_down, 12.0F);
  EXPECT_FLOAT_EQ(decoded.acceleration_north, 13.0F);
  EXPECT_FLOAT_EQ(decoded.acceleration_east, 14.0F);
  EXPECT_FLOAT_EQ(decoded.acceleration_down, 15.0F);
  EXPECT_FLOAT_EQ(decoded.pressure_altitude, -16.0F);
  EXPECT_EQ(decoded.timestamp, INT64_C(0x1122334455667788));
}

TEST(
    FDILinkPayloadDecoder,
    DecodesEveryGeodeticPositionFieldFromFixedLittleEndianBytes)
{
  const std::array<std::uint8_t, FDILink::GEODETIC_POS_LEN> payload{{
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x3f,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x3f,
      0x00, 0x00, 0x40, 0x40,
      0x00, 0x00, 0x80, 0xc0,
  }};
  const auto frame =
      makeValidatedFrame(FDILink::TYPE_GEODETIC_POS, payload);

  FDILink::GeodeticPositionPayload decoded;
  ASSERT_TRUE(FDILink::decodeGeodeticPositionPayload(frame, decoded));
  EXPECT_DOUBLE_EQ(decoded.latitude, 1.0);
  EXPECT_DOUBLE_EQ(decoded.longitude, -2.0);
  EXPECT_DOUBLE_EQ(decoded.height, 0.5);
  EXPECT_FLOAT_EQ(decoded.horizontal_accuracy, 3.0F);
  EXPECT_FLOAT_EQ(decoded.vertical_accuracy, -4.0F);
}

TEST(FDILinkPayloadDecoder, RejectsLayoutMismatchWithoutMutatingOutput)
{
  const std::array<std::uint8_t, FDILink::IMU_LEN> payload{};
  const auto valid = makeValidatedFrame(FDILink::TYPE_IMU, payload);
  FDILink::ImuPayload decoded;
  decoded.gyroscope_x = 123.0F;
  decoded.timestamp = 456;

  auto wrong_type = valid;
  wrong_type.type = FDILink::TYPE_AHRS;
  EXPECT_FALSE(FDILink::decodeImuPayload(wrong_type, decoded));
  EXPECT_FLOAT_EQ(decoded.gyroscope_x, 123.0F);
  EXPECT_EQ(decoded.timestamp, 456);

  auto wrong_length = valid;
  wrong_length.payload_size = FDILink::AHRS_LEN;
  EXPECT_FALSE(FDILink::decodeImuPayload(wrong_length, decoded));
  EXPECT_FLOAT_EQ(decoded.gyroscope_x, 123.0F);
  EXPECT_EQ(decoded.timestamp, 456);

  auto truncated = valid;
  --truncated.frame_size;
  EXPECT_FALSE(FDILink::decodeImuPayload(truncated, decoded));
  EXPECT_FLOAT_EQ(decoded.gyroscope_x, 123.0F);
  EXPECT_EQ(decoded.timestamp, 456);
}

}  // namespace
