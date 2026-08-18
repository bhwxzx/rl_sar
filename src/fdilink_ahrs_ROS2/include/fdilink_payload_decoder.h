#ifndef FDILINK_PAYLOAD_DECODER_H_
#define FDILINK_PAYLOAD_DECODER_H_

#include <cstdint>

#include "fdilink_frame_parser.h"

namespace FDILink
{

struct ImuPayload
{
  float gyroscope_x = 0.0F;
  float gyroscope_y = 0.0F;
  float gyroscope_z = 0.0F;
  float accelerometer_x = 0.0F;
  float accelerometer_y = 0.0F;
  float accelerometer_z = 0.0F;
  float magnetometer_x = 0.0F;
  float magnetometer_y = 0.0F;
  float magnetometer_z = 0.0F;
  float imu_temperature = 0.0F;
  float pressure = 0.0F;
  float pressure_temperature = 0.0F;
  std::int64_t timestamp = 0;
};

struct AhrsPayload
{
  float roll_speed = 0.0F;
  float pitch_speed = 0.0F;
  float heading_speed = 0.0F;
  float roll = 0.0F;
  float pitch = 0.0F;
  float heading = 0.0F;
  float quaternion_w = 0.0F;
  float quaternion_x = 0.0F;
  float quaternion_y = 0.0F;
  float quaternion_z = 0.0F;
  std::int64_t timestamp = 0;
};

struct InsGpsPayload
{
  float body_velocity_x = 0.0F;
  float body_velocity_y = 0.0F;
  float body_velocity_z = 0.0F;
  float body_acceleration_x = 0.0F;
  float body_acceleration_y = 0.0F;
  float body_acceleration_z = 0.0F;
  float location_north = 0.0F;
  float location_east = 0.0F;
  float location_down = 0.0F;
  float velocity_north = 0.0F;
  float velocity_east = 0.0F;
  float velocity_down = 0.0F;
  float acceleration_north = 0.0F;
  float acceleration_east = 0.0F;
  float acceleration_down = 0.0F;
  float pressure_altitude = 0.0F;
  std::int64_t timestamp = 0;
};

struct GeodeticPositionPayload
{
  double latitude = 0.0;
  double longitude = 0.0;
  double height = 0.0;
  float horizontal_accuracy = 0.0F;
  float vertical_accuracy = 0.0F;
};

bool decodeImuPayload(
    const ValidatedFrame& frame, ImuPayload& output) noexcept;
bool decodeAhrsPayload(
    const ValidatedFrame& frame, AhrsPayload& output) noexcept;
bool decodeInsGpsPayload(
    const ValidatedFrame& frame, InsGpsPayload& output) noexcept;
bool decodeGeodeticPositionPayload(
    const ValidatedFrame& frame,
    GeodeticPositionPayload& output) noexcept;

}  // namespace FDILink

#endif  // FDILINK_PAYLOAD_DECODER_H_
