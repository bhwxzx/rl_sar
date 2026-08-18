cmake_minimum_required(VERSION 3.5)
project(fdilink_ahrs)


# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(serial REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  include
  src
  ${Eigen3_INCLUDE_DIRS} 
)


add_library(fdilink_protocol STATIC
  src/crc_table.cpp
  src/fdilink_frame_parser.cpp
)
target_include_directories(fdilink_protocol PUBLIC include)

add_executable(ahrs_driver_node src/ahrs_driver.cpp)
target_link_libraries(ahrs_driver_node fdilink_protocol)
ament_target_dependencies(ahrs_driver_node rclcpp rclpy  std_msgs  sensor_msgs  serial tf2_ros tf2 nav_msgs geometry_msgs)


add_executable(imu_tf_node src/imu_tf.cpp)
ament_target_dependencies(imu_tf_node rclcpp rclpy std_msgs sensor_msgs serial tf2_ros tf2 tf2_geometry_msgs)

if(BUILD_TESTING)
  ament_add_gtest(test_fdilink_frame_parser
    test/test_fdilink_frame_parser.cpp
  )
  target_link_libraries(test_fdilink_frame_parser fdilink_protocol)

  add_test(
    NAME fdilink_process_lifecycle
    COMMAND
      "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/test_fdilink_process_lifecycle.py"
      "$<TARGET_FILE:ahrs_driver_node>"
  )
  set_tests_properties(fdilink_process_lifecycle PROPERTIES
    ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1"
    TIMEOUT 10
  )
endif()

# find_package(Boost 1.55.0 REQUIRED COMPONENTS system filesystem)
# include_directories(ahrs_driver_node ${Boost_INCLOUDE_DIRS})
# link_directories(ahrs_driver_node ${Boost_LIBRARY_DIRS})
# target_link_libraries(ahrs_driver_node ${Boost_LIBRSRIES})

install(TARGETS
  ahrs_driver_node
  imu_tf_node

  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS wheeltec_udev.sh
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
