#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

#include "fdilink_payload_validation.h"

namespace
{

const std::array<float, 3> kInvalidFloats{{
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::infinity(),
    -std::numeric_limits<float>::infinity(),
}};

const std::array<double, 3> kInvalidDoubles{{
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(),
}};

FDILink::AhrsPayload makeValidAhrsPayload()
{
  FDILink::AhrsPayload payload;
  payload.quaternion_w = 1.0F;
  return payload;
}

TEST(FDILinkPayloadValidation, ValidatesImuMotionAndMagneticIndependently)
{
  const std::array<float FDILink::ImuPayload::*, 6> motion_fields{{
      &FDILink::ImuPayload::gyroscope_x,
      &FDILink::ImuPayload::gyroscope_y,
      &FDILink::ImuPayload::gyroscope_z,
      &FDILink::ImuPayload::accelerometer_x,
      &FDILink::ImuPayload::accelerometer_y,
      &FDILink::ImuPayload::accelerometer_z,
  }};
  for (const auto field : motion_fields)
  {
    for (const float invalid : kInvalidFloats)
    {
      FDILink::ImuPayload payload;
      payload.*field = invalid;
      const auto result = FDILink::validateImuPayload(payload);
      EXPECT_FALSE(result.motion_valid);
      EXPECT_TRUE(result.magnetic_valid);
    }
  }

  const std::array<float FDILink::ImuPayload::*, 3> magnetic_fields{{
      &FDILink::ImuPayload::magnetometer_x,
      &FDILink::ImuPayload::magnetometer_y,
      &FDILink::ImuPayload::magnetometer_z,
  }};
  for (const auto field : magnetic_fields)
  {
    for (const float invalid : kInvalidFloats)
    {
      FDILink::ImuPayload payload;
      payload.*field = invalid;
      const auto result = FDILink::validateImuPayload(payload);
      EXPECT_TRUE(result.motion_valid);
      EXPECT_FALSE(result.magnetic_valid);
    }
  }
}

TEST(FDILinkPayloadValidation, IgnoresUnpublishedImuFields)
{
  const std::array<float FDILink::ImuPayload::*, 3> unpublished_fields{{
      &FDILink::ImuPayload::imu_temperature,
      &FDILink::ImuPayload::pressure,
      &FDILink::ImuPayload::pressure_temperature,
  }};
  for (const auto field : unpublished_fields)
  {
    FDILink::ImuPayload payload;
    payload.*field = std::numeric_limits<float>::quiet_NaN();
    const auto result = FDILink::validateImuPayload(payload);
    EXPECT_TRUE(result.motion_valid);
    EXPECT_TRUE(result.magnetic_valid);
  }
}

TEST(FDILinkPayloadValidation, RejectsEveryNonFinitePublishedAhrsField)
{
  const std::array<float FDILink::AhrsPayload::*, 7> published_fields{{
      &FDILink::AhrsPayload::roll,
      &FDILink::AhrsPayload::pitch,
      &FDILink::AhrsPayload::heading,
      &FDILink::AhrsPayload::quaternion_w,
      &FDILink::AhrsPayload::quaternion_x,
      &FDILink::AhrsPayload::quaternion_y,
      &FDILink::AhrsPayload::quaternion_z,
  }};
  for (const auto field : published_fields)
  {
    for (const float invalid : kInvalidFloats)
    {
      auto payload = makeValidAhrsPayload();
      payload.*field = invalid;
      EXPECT_EQ(
          FDILink::validateAhrsPayload(payload),
          FDILink::AhrsPayloadValidationStatus::NonFinite);
    }
  }
}

TEST(FDILinkPayloadValidation, EnforcesQuaternionNormRange)
{
  auto payload = makeValidAhrsPayload();
  EXPECT_EQ(
      FDILink::validateAhrsPayload(payload),
      FDILink::AhrsPayloadValidationStatus::Valid);

  payload.quaternion_w = 0.5F;
  EXPECT_EQ(
      FDILink::validateAhrsPayload(payload),
      FDILink::AhrsPayloadValidationStatus::QuaternionNormOutOfRange);

  payload.quaternion_w = 1.5F;
  EXPECT_EQ(
      FDILink::validateAhrsPayload(payload),
      FDILink::AhrsPayloadValidationStatus::QuaternionNormOutOfRange);

  payload = FDILink::AhrsPayload{};
  payload.quaternion_w = std::nextafter(
      static_cast<float>(FDILink::MINIMUM_QUATERNION_NORM),
      std::numeric_limits<float>::infinity());
  EXPECT_EQ(
      FDILink::validateAhrsPayload(payload),
      FDILink::AhrsPayloadValidationStatus::Valid);
  payload.quaternion_w = std::nextafter(
      static_cast<float>(FDILink::MINIMUM_QUATERNION_NORM), 0.0F);
  EXPECT_EQ(
      FDILink::validateAhrsPayload(payload),
      FDILink::AhrsPayloadValidationStatus::QuaternionNormOutOfRange);

  payload.quaternion_w = std::nextafter(
      static_cast<float>(FDILink::MAXIMUM_QUATERNION_NORM), 0.0F);
  EXPECT_EQ(
      FDILink::validateAhrsPayload(payload),
      FDILink::AhrsPayloadValidationStatus::Valid);
  payload.quaternion_w = std::nextafter(
      static_cast<float>(FDILink::MAXIMUM_QUATERNION_NORM),
      std::numeric_limits<float>::infinity());
  EXPECT_EQ(
      FDILink::validateAhrsPayload(payload),
      FDILink::AhrsPayloadValidationStatus::QuaternionNormOutOfRange);
}

TEST(FDILinkPayloadValidation, IgnoresUnpublishedAhrsFields)
{
  const std::array<float FDILink::AhrsPayload::*, 3> unpublished_fields{{
      &FDILink::AhrsPayload::roll_speed,
      &FDILink::AhrsPayload::pitch_speed,
      &FDILink::AhrsPayload::heading_speed,
  }};
  for (const auto field : unpublished_fields)
  {
    auto payload = makeValidAhrsPayload();
    payload.*field = std::numeric_limits<float>::quiet_NaN();
    EXPECT_EQ(
        FDILink::validateAhrsPayload(payload),
        FDILink::AhrsPayloadValidationStatus::Valid);
  }
}

TEST(FDILinkPayloadValidation, RejectsEveryNonFinitePublishedInsGpsField)
{
  const std::array<float FDILink::InsGpsPayload::*, 9> published_fields{{
      &FDILink::InsGpsPayload::body_velocity_x,
      &FDILink::InsGpsPayload::body_velocity_y,
      &FDILink::InsGpsPayload::body_velocity_z,
      &FDILink::InsGpsPayload::location_north,
      &FDILink::InsGpsPayload::location_east,
      &FDILink::InsGpsPayload::location_down,
      &FDILink::InsGpsPayload::velocity_north,
      &FDILink::InsGpsPayload::velocity_east,
      &FDILink::InsGpsPayload::velocity_down,
  }};
  for (const auto field : published_fields)
  {
    for (const float invalid : kInvalidFloats)
    {
      FDILink::InsGpsPayload payload;
      payload.*field = invalid;
      EXPECT_EQ(
          FDILink::validateInsGpsPayload(payload),
          FDILink::InsGpsPayloadValidationStatus::NonFinite);
    }
  }
}

TEST(FDILinkPayloadValidation, IgnoresUnpublishedInsGpsFields)
{
  const std::array<float FDILink::InsGpsPayload::*, 7> unpublished_fields{{
      &FDILink::InsGpsPayload::body_acceleration_x,
      &FDILink::InsGpsPayload::body_acceleration_y,
      &FDILink::InsGpsPayload::body_acceleration_z,
      &FDILink::InsGpsPayload::acceleration_north,
      &FDILink::InsGpsPayload::acceleration_east,
      &FDILink::InsGpsPayload::acceleration_down,
      &FDILink::InsGpsPayload::pressure_altitude,
  }};
  for (const auto field : unpublished_fields)
  {
    FDILink::InsGpsPayload payload;
    payload.*field = std::numeric_limits<float>::quiet_NaN();
    EXPECT_EQ(
        FDILink::validateInsGpsPayload(payload),
        FDILink::InsGpsPayloadValidationStatus::Valid);
  }
}

TEST(FDILinkPayloadValidation, ValidatesGeodeticFiniteValuesAndRanges)
{
  const std::array<double FDILink::GeodeticPositionPayload::*, 3>
      published_fields{{
          &FDILink::GeodeticPositionPayload::latitude,
          &FDILink::GeodeticPositionPayload::longitude,
          &FDILink::GeodeticPositionPayload::height,
      }};
  for (const auto field : published_fields)
  {
    for (const double invalid : kInvalidDoubles)
    {
      FDILink::GeodeticPositionPayload payload;
      payload.*field = invalid;
      EXPECT_EQ(
          FDILink::validateGeodeticPositionPayload(payload),
          FDILink::GeodeticPayloadValidationStatus::NonFinite);
    }
  }

  FDILink::GeodeticPositionPayload payload;
  payload.latitude = FDILink::MINIMUM_LATITUDE_RADIANS;
  payload.longitude = FDILink::MINIMUM_LONGITUDE_RADIANS;
  EXPECT_EQ(
      FDILink::validateGeodeticPositionPayload(payload),
      FDILink::GeodeticPayloadValidationStatus::Valid);
  payload.latitude = FDILink::MAXIMUM_LATITUDE_RADIANS;
  payload.longitude = FDILink::MAXIMUM_LONGITUDE_RADIANS;
  EXPECT_EQ(
      FDILink::validateGeodeticPositionPayload(payload),
      FDILink::GeodeticPayloadValidationStatus::Valid);

  payload.latitude = std::nextafter(
      FDILink::MINIMUM_LATITUDE_RADIANS,
      -std::numeric_limits<double>::infinity());
  EXPECT_EQ(
      FDILink::validateGeodeticPositionPayload(payload),
      FDILink::GeodeticPayloadValidationStatus::LatitudeOutOfRange);
  payload.latitude = 0.0;
  payload.longitude = std::nextafter(
      FDILink::MAXIMUM_LONGITUDE_RADIANS,
      std::numeric_limits<double>::infinity());
  EXPECT_EQ(
      FDILink::validateGeodeticPositionPayload(payload),
      FDILink::GeodeticPayloadValidationStatus::LongitudeOutOfRange);
}

TEST(FDILinkPayloadValidation, IgnoresUnpublishedGeodeticAccuracyFields)
{
  FDILink::GeodeticPositionPayload payload;
  payload.horizontal_accuracy = std::numeric_limits<float>::quiet_NaN();
  payload.vertical_accuracy = std::numeric_limits<float>::infinity();
  EXPECT_EQ(
      FDILink::validateGeodeticPositionPayload(payload),
      FDILink::GeodeticPayloadValidationStatus::Valid);
}

}  // namespace
