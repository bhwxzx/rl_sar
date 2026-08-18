#include <ahrs_driver.h>

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace FDILink
{
namespace
{
constexpr std::uint32_t kSerialReadTimeoutMs = 20;
constexpr std::size_t kSerialReadBufferSize = 256;

static_assert(
    sizeof(imu_frame_read) == FDILINK_HEADER_SIZE + IMU_LEN + 1,
    "IMU frame storage must match the wire format");
static_assert(
    sizeof(ahrs_frame_read) == FDILINK_HEADER_SIZE + AHRS_LEN + 1,
    "AHRS frame storage must match the wire format");
static_assert(
    sizeof(insgps_frame_read) == FDILINK_HEADER_SIZE + INSGPS_LEN + 1,
    "INSGPS frame storage must match the wire format");
static_assert(
    sizeof(Geodetic_Position_frame_read)
        == FDILINK_HEADER_SIZE + GEODETIC_POS_LEN + 1,
    "geodetic-position frame storage must match the wire format");

template <typename FrameStorage>
void copyValidatedFrame(const ValidatedFrame& source, FrameStorage& destination)
{
  if (source.frame_size != sizeof(destination.read_tmp))
  {
    throw std::runtime_error("validated FDILink frame has an unexpected size");
  }
  std::memcpy(destination.read_tmp, source.bytes.data(), source.frame_size);
}
}  // namespace

ahrsBringup::ahrsBringup()
: rclcpp::Node("ahrs_bringup")
{
  if_debug_ = declare_parameter<bool>("if_debug_", false);

  const std::int64_t device_type =
      declare_parameter<std::int64_t>("device_type_", 1);
  if (device_type != 0 && device_type != 1)
  {
    throw std::invalid_argument("device_type_ must be 0 or 1");
  }
  device_type_ = static_cast<int>(device_type);

  imu_topic = declare_parameter<std::string>("imu_topic", "/imu");
  imu_frame_id_ =
      declare_parameter<std::string>("imu_frame_id_", "gyro_link");
  mag_pose_2d_topic = declare_parameter<std::string>(
      "mag_pose_2d_topic", "/mag_pose_2d");
  Euler_angles_topic = declare_parameter<std::string>(
      "Euler_angles_topic", "/euler_angles");
  gps_topic = declare_parameter<std::string>("gps_topic", "/gps/fix");
  Magnetic_topic =
      declare_parameter<std::string>("Magnetic_topic", "/magnetic");
  twist_topic =
      declare_parameter<std::string>("twist_topic", "/system_speed");
  NED_odom_topic = declare_parameter<std::string>(
      "NED_odom_topic", "/NED_odometry");
  serial_port_ = declare_parameter<std::string>(
      "serial_port_", "/dev/fdilink_ahrs");

  const std::int64_t serial_baud =
      declare_parameter<std::int64_t>("serial_baud_", 921600);
  if (serial_baud <= 0
      || serial_baud > static_cast<std::int64_t>(
          std::numeric_limits<std::uint32_t>::max()))
  {
    throw std::invalid_argument("serial_baud_ must fit a positive uint32");
  }
  serial_baud_ = static_cast<std::uint32_t>(serial_baud);

  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic, 10);
  gps_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(gps_topic, 10);
  mag_pose_pub_ =
      create_publisher<geometry_msgs::msg::Pose2D>(mag_pose_2d_topic, 10);
  Euler_angles_pub_ =
      create_publisher<geometry_msgs::msg::Vector3>(Euler_angles_topic, 10);
  Magnetic_pub_ =
      create_publisher<geometry_msgs::msg::Vector3>(Magnetic_topic, 10);
  twist_pub_ =
      create_publisher<geometry_msgs::msg::Twist>(twist_topic, 10);
  NED_odom_pub_ =
      create_publisher<nav_msgs::msg::Odometry>(NED_odom_topic, 10);

  try
  {
    serial_.setPort(serial_port_);
    serial_.setBaudrate(serial_baud_);
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.setParity(serial::parity_none);
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    serial::Timeout timeout =
        serial::Timeout::simpleTimeout(kSerialReadTimeoutMs);
    serial_.setTimeout(timeout);
    serial_.open();
  }
  catch (const serial::IOException& error)
  {
    throw std::runtime_error(
        "cannot open FDILink serial port " + serial_port_ + ": "
        + error.what());
  }

  if (!serial_.isOpen())
  {
    throw std::runtime_error(
        "FDILink serial port did not open: " + serial_port_);
  }
  RCLCPP_INFO(
      get_logger(),
      "FDILink serial port initialized: %s at %u baud, timeout %u ms",
      serial_port_.c_str(), serial_baud_, kSerialReadTimeoutMs);
}

ahrsBringup::~ahrsBringup()
{
  try
  {
    if (serial_.isOpen())
    {
      serial_.close();
    }
  }
  catch (const std::exception& error)
  {
    std::fprintf(
        stderr, "Failed to close FDILink serial port cleanly: %s\n",
        error.what());
  }
}

void ahrsBringup::processLoop()
{
  RCLCPP_INFO(get_logger(), "ahrsBringup::processLoop: start");
  std::array<std::uint8_t, kSerialReadBufferSize> read_buffer{};

  while (rclcpp::ok())
  {
    if (!serial_.isOpen())
    {
      throw std::runtime_error("FDILink serial port closed during receive");
    }

    const std::size_t bytes_available = serial_.available();
    const std::size_t requested = bytes_available == 0
        ? 1
        : std::min(bytes_available, read_buffer.size());
    const std::size_t bytes_read =
        serial_.read(read_buffer.data(), requested);

    if (bytes_read == 0)
    {
      if (frame_parser_.hasPartialFrame())
      {
        frame_parser_.reset();
        if (if_debug_)
        {
          RCLCPP_WARN(
              get_logger(),
              "FDILink frame timed out before completion; partial data discarded");
        }
      }
      continue;
    }

    for (std::size_t index = 0; index < bytes_read; ++index)
    {
      ValidatedFrame frame;
      const FrameParserEvent event =
          frame_parser_.consume(read_buffer[index], frame);
      if (event == FrameParserEvent::Rejected)
      {
        ++crc_error_;
        if (if_debug_)
        {
          RCLCPP_WARN(get_logger(), "Rejected malformed FDILink frame");
        }
      }
      else if (event == FrameParserEvent::FrameReady)
      {
        handleValidatedFrame(frame);
      }
    }

    if (bytes_read < requested && frame_parser_.hasPartialFrame())
    {
      frame_parser_.reset();
      if (if_debug_)
      {
        RCLCPP_WARN(
            get_logger(),
            "FDILink short read ended an incomplete frame; partial data discarded");
      }
    }
  }

  frame_parser_.reset();
  RCLCPP_INFO(get_logger(), "ahrsBringup::processLoop: stop");
}

void ahrsBringup::handleValidatedFrame(const ValidatedFrame& frame)
{
  switch (frame.type)
  {
    case TYPE_IMU:
      copyValidatedFrame(frame, imu_frame_);
      checkSN(TYPE_IMU);
      has_valid_imu_ = true;
      if (has_valid_ahrs_)
      {
        publishImuFrame();
      }
      else if (if_debug_)
      {
        RCLCPP_WARN(
            get_logger(),
            "Ignoring IMU frame until a complete AHRS frame is available");
      }
      break;
    case TYPE_AHRS:
      copyValidatedFrame(frame, ahrs_frame_);
      checkSN(TYPE_AHRS);
      has_valid_ahrs_ = true;
      publishAhrsFrame();
      break;
    case TYPE_INSGPS:
      copyValidatedFrame(frame, insgps_frame_);
      checkSN(TYPE_INSGPS);
      publishInsGpsFrame();
      break;
    case TYPE_GEODETIC_POS:
      copyValidatedFrame(frame, Geodetic_Position_frame_);
      checkSN(TYPE_GEODETIC_POS);
      publishGeodeticPositionFrame();
      break;
    case TYPE_GROUND:
    case TYPE_GROUND_EXTENDED:
      if (!first_sequence_received_)
      {
        read_sn_ = frame.serial_number;
        first_sequence_received_ = true;
      }
      else
      {
        const std::uint8_t expected =
            static_cast<std::uint8_t>(read_sn_ + 1);
        if (expected != frame.serial_number)
        {
          sn_lost_ += static_cast<std::uint8_t>(
              frame.serial_number - expected);
        }
        read_sn_ = frame.serial_number;
      }
      break;
    default:
      throw std::runtime_error("validated an unsupported FDILink frame type");
  }
}

void ahrsBringup::publishImuFrame()
{
  sensor_msgs::msg::Imu imu_data;
  imu_data.header.stamp = now();
  imu_data.header.frame_id = imu_frame_id_;

  const Eigen::Quaterniond q_ahrs(
      ahrs_frame_.frame.data.data_pack.Qw,
      ahrs_frame_.frame.data.data_pack.Qx,
      ahrs_frame_.frame.data.data_pack.Qy,
      ahrs_frame_.frame.data.data_pack.Qz);
  const Eigen::Quaterniond q_r =
      Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
  const Eigen::Quaterniond q_rr =
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitX());

  if (device_type_ == 0)
  {
    imu_data.orientation.w = ahrs_frame_.frame.data.data_pack.Qw;
    imu_data.orientation.x = ahrs_frame_.frame.data.data_pack.Qx;
    imu_data.orientation.y = ahrs_frame_.frame.data.data_pack.Qy;
    imu_data.orientation.z = ahrs_frame_.frame.data.data_pack.Qz;
    imu_data.angular_velocity.x = imu_frame_.frame.data.data_pack.gyroscope_x;
    imu_data.angular_velocity.y = imu_frame_.frame.data.data_pack.gyroscope_y;
    imu_data.angular_velocity.z = imu_frame_.frame.data.data_pack.gyroscope_z;
    imu_data.linear_acceleration.x =
        imu_frame_.frame.data.data_pack.accelerometer_x;
    imu_data.linear_acceleration.y =
        imu_frame_.frame.data.data_pack.accelerometer_y;
    imu_data.linear_acceleration.z =
        imu_frame_.frame.data.data_pack.accelerometer_z;
  }
  else
  {
    const Eigen::Quaterniond q_out = q_r * q_ahrs * q_rr;
    imu_data.orientation.w = q_out.w();
    imu_data.orientation.x = q_out.x();
    imu_data.orientation.y = q_out.y();
    imu_data.orientation.z = q_out.z();
    imu_data.angular_velocity.x = imu_frame_.frame.data.data_pack.gyroscope_x;
    imu_data.angular_velocity.y = -imu_frame_.frame.data.data_pack.gyroscope_y;
    imu_data.angular_velocity.z = -imu_frame_.frame.data.data_pack.gyroscope_z;
    imu_data.linear_acceleration.x =
        imu_frame_.frame.data.data_pack.accelerometer_x;
    imu_data.linear_acceleration.y =
        -imu_frame_.frame.data.data_pack.accelerometer_y;
    imu_data.linear_acceleration.z =
        -imu_frame_.frame.data.data_pack.accelerometer_z;
  }
  imu_pub_->publish(imu_data);
}

void ahrsBringup::publishAhrsFrame()
{
  geometry_msgs::msg::Pose2D pose_2d;
  pose_2d.theta = ahrs_frame_.frame.data.data_pack.Heading;
  mag_pose_pub_->publish(pose_2d);

  geometry_msgs::msg::Vector3 euler_angles;
  euler_angles.x = ahrs_frame_.frame.data.data_pack.Roll;
  euler_angles.y = ahrs_frame_.frame.data.data_pack.Pitch;
  euler_angles.z = ahrs_frame_.frame.data.data_pack.Heading;
  Euler_angles_pub_->publish(euler_angles);

  if (has_valid_imu_)
  {
    geometry_msgs::msg::Vector3 magnetic;
    magnetic.x = imu_frame_.frame.data.data_pack.magnetometer_x;
    magnetic.y = imu_frame_.frame.data.data_pack.magnetometer_y;
    magnetic.z = imu_frame_.frame.data.data_pack.magnetometer_z;
    Magnetic_pub_->publish(magnetic);
  }
}

void ahrsBringup::publishGeodeticPositionFrame()
{
  sensor_msgs::msg::NavSatFix gps_data;
  gps_data.header.stamp = now();
  gps_data.header.frame_id = "navsat_link";
  gps_data.latitude =
      Geodetic_Position_frame_.frame.data.data_pack.Latitude / DEG_TO_RAD;
  gps_data.longitude =
      Geodetic_Position_frame_.frame.data.data_pack.Longitude / DEG_TO_RAD;
  gps_data.altitude = Geodetic_Position_frame_.frame.data.data_pack.Height;
  gps_pub_->publish(gps_data);
}

void ahrsBringup::publishInsGpsFrame()
{
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = now();
  odom_msg.pose.pose.position.x =
      insgps_frame_.frame.data.data_pack.Location_North;
  odom_msg.pose.pose.position.y =
      insgps_frame_.frame.data.data_pack.Location_East;
  odom_msg.pose.pose.position.z =
      insgps_frame_.frame.data.data_pack.Location_Down;
  odom_msg.twist.twist.linear.x =
      insgps_frame_.frame.data.data_pack.Velocity_North;
  odom_msg.twist.twist.linear.y =
      insgps_frame_.frame.data.data_pack.Velocity_East;
  odom_msg.twist.twist.linear.z =
      insgps_frame_.frame.data.data_pack.Velocity_Down;
  NED_odom_pub_->publish(odom_msg);

  geometry_msgs::msg::Twist speed_msg;
  speed_msg.linear.x = insgps_frame_.frame.data.data_pack.BodyVelocity_X;
  speed_msg.linear.y = insgps_frame_.frame.data.data_pack.BodyVelocity_Y;
  speed_msg.linear.z = insgps_frame_.frame.data.data_pack.BodyVelocity_Z;
  twist_pub_->publish(speed_msg);
}

void ahrsBringup::magCalculateYaw(
    double roll,
    double pitch,
    double& magyaw,
    double magx,
    double magy,
    double magz)
{
  const double temp1 = magy * cos(roll) + magz * sin(roll);
  const double temp2 = magx * cos(pitch)
      + magy * sin(pitch) * sin(roll)
      - magz * sin(pitch) * cos(roll);
  magyaw = atan2(-temp1, temp2);
  if (magyaw < 0)
  {
    magyaw += 2 * PI;
  }
}

void ahrsBringup::checkSN(int type)
{
  std::uint8_t serial_number = 0;
  switch (type)
  {
    case TYPE_IMU:
      serial_number = imu_frame_.frame.header.serial_num;
      break;
    case TYPE_AHRS:
      serial_number = ahrs_frame_.frame.header.serial_num;
      break;
    case TYPE_INSGPS:
      serial_number = insgps_frame_.frame.header.serial_num;
      break;
    case TYPE_GEODETIC_POS:
      serial_number = Geodetic_Position_frame_.frame.header.serial_num;
      break;
    default:
      return;
  }

  if (!first_sequence_received_)
  {
    read_sn_ = serial_number;
    first_sequence_received_ = true;
    return;
  }

  const std::uint8_t expected = static_cast<std::uint8_t>(read_sn_ + 1);
  if (expected != serial_number)
  {
    sn_lost_ += static_cast<std::uint8_t>(serial_number - expected);
    if (if_debug_)
    {
      RCLCPP_WARN(
          get_logger(),
          "Detected FDILink sequence loss: expected %u, received %u",
          static_cast<unsigned int>(expected),
          static_cast<unsigned int>(serial_number));
    }
  }
  read_sn_ = serial_number;
}

}  // namespace FDILink

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<FDILink::ahrsBringup>();
    node->processLoop();
    rclcpp::shutdown();
    return EXIT_SUCCESS;
  }
  catch (const std::exception& error)
  {
    std::fprintf(stderr, "FDILink AHRS driver failed: %s\n", error.what());
    if (rclcpp::ok())
    {
      rclcpp::shutdown();
    }
    return EXIT_FAILURE;
  }
}
