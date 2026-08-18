#ifndef FDILINK_PAYLOAD_VALIDATION_H_
#define FDILINK_PAYLOAD_VALIDATION_H_

#include "fdilink_payload_decoder.h"

namespace FDILink
{

constexpr double MINIMUM_QUATERNION_NORM = 0.9;
constexpr double MAXIMUM_QUATERNION_NORM = 1.1;
constexpr double MINIMUM_LATITUDE_RADIANS = -1.5707963267948966;
constexpr double MAXIMUM_LATITUDE_RADIANS = 1.5707963267948966;
constexpr double MINIMUM_LONGITUDE_RADIANS = -3.1415926535897932;
constexpr double MAXIMUM_LONGITUDE_RADIANS = 3.1415926535897932;

struct ImuPayloadValidation
{
  bool motion_valid = false;
  bool magnetic_valid = false;
};

enum class AhrsPayloadValidationStatus
{
  Valid,
  NonFinite,
  QuaternionNormOutOfRange,
};

enum class InsGpsPayloadValidationStatus
{
  Valid,
  NonFinite,
};

enum class GeodeticPayloadValidationStatus
{
  Valid,
  NonFinite,
  LatitudeOutOfRange,
  LongitudeOutOfRange,
};

ImuPayloadValidation validateImuPayload(const ImuPayload& payload) noexcept;
AhrsPayloadValidationStatus validateAhrsPayload(
    const AhrsPayload& payload) noexcept;
InsGpsPayloadValidationStatus validateInsGpsPayload(
    const InsGpsPayload& payload) noexcept;
GeodeticPayloadValidationStatus validateGeodeticPositionPayload(
    const GeodeticPositionPayload& payload) noexcept;

}  // namespace FDILink

#endif  // FDILINK_PAYLOAD_VALIDATION_H_
