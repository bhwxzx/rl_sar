#!/bin/bash

# 对于jetson 需要 export IS_JETSON=true
sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers ros-$ROS_DISTRO-control-toolbox ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-xacro

source ./install/setup.bash
./build.sh serial
./build.sh fdilink_ahrs
./build.sh robot_msgs robot_joint_controller
./build.sh rl_sar