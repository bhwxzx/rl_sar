#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${script_dir}/common.sh"

mode=install
if [[ ${1:-} == "--print-packages" ]]; then
    mode=print
elif [[ $# -ne 0 ]]; then
    echo "Usage: $0 [--print-packages]" >&2
    exit 2
fi

packages=(
    build-essential
    cmake
    curl
    libboost-all-dev
    libeigen3-dev
    libglfw3-dev
    libgl1-mesa-dev
    libssl-dev
    libtbb-dev
    libyaml-cpp-dev
    python3-colcon-common-extensions
    python3-dev
    python3-pip
    pkg-config
    unzip
    wget
)

if [[ -n ${ROS_DISTRO:-} ]]; then
    packages+=(
        "ros-${ROS_DISTRO}-ament-index-cpp"
        "ros-${ROS_DISTRO}-control-toolbox"
        "ros-${ROS_DISTRO}-controller-interface"
        "ros-${ROS_DISTRO}-geometry-msgs"
        "ros-${ROS_DISTRO}-hardware-interface"
        "ros-${ROS_DISTRO}-joint-state-publisher-gui"
        "ros-${ROS_DISTRO}-nav-msgs"
        "ros-${ROS_DISTRO}-pluginlib"
        "ros-${ROS_DISTRO}-rcl-interfaces"
        "ros-${ROS_DISTRO}-rclcpp"
        "ros-${ROS_DISTRO}-rclpy"
        "ros-${ROS_DISTRO}-realtime-tools"
        "ros-${ROS_DISTRO}-robot-state-publisher"
        "ros-${ROS_DISTRO}-ros2-control"
        "ros-${ROS_DISTRO}-ros2-controllers"
        "ros-${ROS_DISTRO}-rosidl-default-generators"
        "ros-${ROS_DISTRO}-sensor-msgs"
        "ros-${ROS_DISTRO}-std-msgs"
        "ros-${ROS_DISTRO}-teleop-twist-keyboard"
        "ros-${ROS_DISTRO}-tf2"
        "ros-${ROS_DISTRO}-tf2-geometry-msgs"
        "ros-${ROS_DISTRO}-tf2-ros"
        "ros-${ROS_DISTRO}-urdf"
        "ros-${ROS_DISTRO}-xacro"
    )
fi

if [[ "$mode" == print ]]; then
    printf '%s\n' "${packages[@]}"
    exit 0
fi

if [[ ${RL_SAR_SKIP_SYSTEM_DEPENDENCIES:-false} == true ]]; then
    print_warning "Skipping automatic system dependency installation by request"
    exit 0
fi

if [[ $(uname -s) != Linux || ! -x "$(command -v apt-get 2>/dev/null || true)" ]]; then
    print_error "Automatic dependency installation requires apt-get on Linux"
    print_info "Install the packages listed by: $0 --print-packages"
    exit 1
fi
if ! command -v dpkg-query &> /dev/null; then
    print_error "dpkg-query is required to inspect installed dependencies"
    exit 1
fi

missing_packages=()
for package in "${packages[@]}"; do
    if [[ $(dpkg-query -W -f='${Status}' "$package" 2>/dev/null || true) \
          != "install ok installed" ]]; then
        missing_packages+=("$package")
    fi
done

if [[ ${#missing_packages[@]} -eq 0 ]]; then
    print_success "System build dependencies are already installed"
    exit 0
fi

print_info "Installing missing system dependencies: ${missing_packages[*]}"
if [[ ${EUID} -eq 0 ]]; then
    apt-get update
    DEBIAN_FRONTEND=noninteractive apt-get install -y "${missing_packages[@]}"
else
    if ! command -v sudo &> /dev/null; then
        print_error "sudo is required to install missing system dependencies"
        exit 1
    fi
    sudo apt-get update
    sudo env DEBIAN_FRONTEND=noninteractive \
        apt-get install -y "${missing_packages[@]}"
fi

print_success "System build dependencies installed"
