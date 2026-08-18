#include "fdilink_payload_validation.h"

#include <array>
#include <cmath>
#include <cstddef>

namespace FDILink
{
namespace
{

template <typename Value, std::size_t Size>
bool allFinite(const std::array<Value, Size>& values) noexcept
{
  for (const Value value : values)
  {
    if (!std::isfinite(value))
    {
      return false;
    }
  }
  return true;
}

}  // namespace

ImuPayloadValidation validateImuPayload(const ImuPayload& payload) noexcept
{
  ImuPayloadValidation result;
  result.motion_valid = allFinite(std::array<float, 6>{{
      payload.gyroscope_x,
      payload.gyroscope_y,
      payload.gyroscope_z,
      payload.accelerometer_x,
      payload.accelerometer_y,
      payload.accelerometer_z,
  }});
  result.magnetic_valid = allFinite(std::array<float, 3>{{
      payload.magnetometer_x,
      payload.magnetometer_y,
      payload.magnetometer_z,
  }});
  return result;
}

AhrsPayloadValidationStatus validateAhrsPayload(
    const AhrsPayload& payload) noexcept
{
  if (!allFinite(std::array<float, 7>{{
      payload.roll,
      payload.pitch,
      payload.heading,
      payload.quaternion_w,
      payload.quaternion_x,
      payload.quaternion_y,
      payload.quaternion_z,
  }}))
  {
    return AhrsPayloadValidationStatus::NonFinite;
  }

  const double norm_squared =
      static_cast<double>(payload.quaternion_w) * payload.quaternion_w
      + static_cast<double>(payload.quaternion_x) * payload.quaternion_x
      + static_cast<double>(payload.quaternion_y) * payload.quaternion_y
      + static_cast<double>(payload.quaternion_z) * payload.quaternion_z;
  const double minimum_squared =
      MINIMUM_QUATERNION_NORM * MINIMUM_QUATERNION_NORM;
  const double maximum_squared =
      MAXIMUM_QUATERNION_NORM * MAXIMUM_QUATERNION_NORM;
  if (norm_squared < minimum_squared || norm_squared > maximum_squared)
  {
    return AhrsPayloadValidationStatus::QuaternionNormOutOfRange;
  }
  return AhrsPayloadValidationStatus::Valid;
}

InsGpsPayloadValidationStatus validateInsGpsPayload(
    const InsGpsPayload& payload) noexcept
{
  if (!allFinite(std::array<float, 9>{{
      payload.body_velocity_x,
      payload.body_velocity_y,
      payload.body_velocity_z,
      payload.location_north,
      payload.location_east,
      payload.location_down,
      payload.velocity_north,
      payload.velocity_east,
      payload.velocity_down,
  }}))
  {
    return InsGpsPayloadValidationStatus::NonFinite;
  }
  return InsGpsPayloadValidationStatus::Valid;
}

GeodeticPayloadValidationStatus validateGeodeticPositionPayload(
    const GeodeticPositionPayload& payload) noexcept
{
  if (!allFinite(std::array<double, 3>{{
      payload.latitude,
      payload.longitude,
      payload.height,
  }}))
  {
    return GeodeticPayloadValidationStatus::NonFinite;
  }
  if (payload.latitude < MINIMUM_LATITUDE_RADIANS
      || payload.latitude > MAXIMUM_LATITUDE_RADIANS)
  {
    return GeodeticPayloadValidationStatus::LatitudeOutOfRange;
  }
  if (payload.longitude < MINIMUM_LONGITUDE_RADIANS
      || payload.longitude > MAXIMUM_LONGITUDE_RADIANS)
  {
    return GeodeticPayloadValidationStatus::LongitudeOutOfRange;
  }
  return GeodeticPayloadValidationStatus::Valid;
}

}  // namespace FDILink
