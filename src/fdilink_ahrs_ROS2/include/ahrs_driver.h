#ifndef BASE_DRIVER_H_
#define BASE_DRIVER_H_


#include <inttypes.h>
#include <array>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <iostream>
#include <unistd.h>
#include <serial/serial.h> //ROS的串口包 http://wjwwood.io/serial/doc/1.1.0/index.html
#include <math.h>
#include <fstream>
#include <fdilink_frame_parser.h>
#include <fdilink_payload_decoder.h>
#include <fdilink_payload_validation.h>
//#include <sensor_msgs/Imu.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <string>
//#include <ros/package.h>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <crc_table.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Core>
//#include <boost/thread.h>

using namespace std;
using namespace Eigen;
namespace FDILink
{
#define PI 3.141592653589793
#define DEG_TO_RAD 0.017453292519943295


class ahrsBringup : public rclcpp::Node
{
public:
  ahrsBringup();
  ~ahrsBringup();
  void processLoop();
  void magCalculateYaw(double roll, double pitch, double &magyaw, double magx, double magy, double magz);

private:
  void handleValidatedFrame(const ValidatedFrame& frame);
  void reportSemanticRejection(std::uint8_t type, const char* reason);
  void updateSequence(std::uint8_t serial_number);
  void publishImuFrame();
  void publishAhrsFrame();
  void publishGeodeticPositionFrame();
  void publishInsGpsFrame();

  bool if_debug_ = false;
  //sum info
  int sn_lost_ = 0;
  int crc_error_ = 0;
  std::uint64_t semantic_error_ = 0;
  uint8_t read_sn_ = 0;
  bool first_sequence_received_ = false;
  int device_type_ = 1;

  //serial
  serial::Serial serial_; //声明串口对象
  std::string serial_port_;
  std::uint32_t serial_baud_ = 921600;
  FrameParser frame_parser_;
  //data
  ImuPayload imu_payload_{};
  std::array<float, 3> magnetic_field_{};
  AhrsPayload ahrs_payload_{};
  InsGpsPayload insgps_payload_{};
  GeodeticPositionPayload geodetic_position_payload_{};
  bool has_valid_imu_ = false;
  bool has_valid_magnetic_ = false;
  bool has_valid_ahrs_ = false;
  //frame name
  //std::string imu_frame_id="gyro_link";
  std::string imu_frame_id_;
  std::string insgps_frame_id_;
  std::string latlon_frame_id_;
  //topic
  std::string imu_topic="/imu", mag_pose_2d_topic="/mag_pose_2d";
  std::string latlon_topic="latlon";
  std::string Euler_angles_topic="/Euler_angles", Magnetic_topic="/Magnetic";
  std::string gps_topic="/gps/fix",twist_topic="/system_speed",NED_odom_topic="/NED_odometry";


  //Publisher
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr  imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr mag_pose_pub_;

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr Euler_angles_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr Magnetic_pub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  twist_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr NED_odom_pub_;



}; //ahrsBringup
} // namespace FDILink

#endif
