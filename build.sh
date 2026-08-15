#!/bin/bash
set -e

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Load common utilities
source "${SCRIPT_DIR}/scripts/common.sh"

# ========================
# Configuration
# ========================

# ========================
# Build Functions
# ========================

# 调用 scripts/download_inference_runtime.sh 检查/下载安装推理相关库
setup_inference_runtime() { 
    print_header "[Setting up Inference Runtime]"

    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
    DOWNLOAD_SCRIPT="${SCRIPT_DIR}/scripts/download_inference_runtime.sh"
    local runtime_target=all
    if [ "${IS_JETSON}" = true ]; then
        runtime_target=onnx
    fi

    if [ -f "$DOWNLOAD_SCRIPT" ]; then
        print_info "Checking inference libraries (${runtime_target})..."
        bash "$DOWNLOAD_SCRIPT" "$runtime_target" || {
            print_error "Failed to setup inference libraries"
            exit 1
        }
        print_success "Inference runtime setup completed!"
    else
        print_warning "Download script not found: $DOWNLOAD_SCRIPT"
    fi
}

setup_system_dependencies() {
    print_header "[Setting up System Dependencies]"

    local dependency_script="${SCRIPT_DIR}/scripts/install_build_dependencies.sh"
    if [ ! -f "$dependency_script" ]; then
        print_error "Dependency installer not found: $dependency_script"
        exit 1
    fi
    bash "$dependency_script" || {
        print_error "Failed to setup system build dependencies"
        exit 1
    }
}

prepare_build_platform() {
    if ! resolve_jetson_platform; then
        exit 1
    fi
    print_info "Jetson mode: ${IS_JETSON} (${JETSON_DETECTION_SOURCE})"
}

setup_mujoco() {
    print_header "[Setting up MuJoCo]"

    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
    DOWNLOAD_MUJOCO_SCRIPT="${SCRIPT_DIR}/scripts/download_mujoco.sh"

    if [ -f "$DOWNLOAD_MUJOCO_SCRIPT" ]; then
        print_info "Checking MuJoCo library..."
        bash "$DOWNLOAD_MUJOCO_SCRIPT" || {
            print_error "Failed to setup MuJoCo"
            exit 1
        }
        print_success "MuJoCo setup completed!"
    else
        print_warning "MuJoCo download script not found: $DOWNLOAD_MUJOCO_SCRIPT"
    fi
}

validate_lw_description() {
    print_header "[Checking LW Description]"

    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
    VALIDATE_LW_DESCRIPTION_SCRIPT="${SCRIPT_DIR}/scripts/validate_lw_description.sh"

    if [ -f "$VALIDATE_LW_DESCRIPTION_SCRIPT" ]; then
        print_info "Validating tracked LW description files..."
        bash "$VALIDATE_LW_DESCRIPTION_SCRIPT" || {
            print_error "LW description validation failed"
            exit 1
        }
        print_success "LW description validation completed!"
    else
        print_error "LW description validator not found: $VALIDATE_LW_DESCRIPTION_SCRIPT"
        exit 1
    fi
}

run_mujoco_build() {
    print_header "[Running MuJoCo Build]"
    print_info "Building with MuJoCo simulator support..."
    print_separator

    cmake src/rl_sar/ -B cmake_build -DUSE_CMAKE=ON -DUSE_MUJOCO=ON
    cmake --build cmake_build -j$(nproc 2>/dev/null || echo 4)

    print_success "MuJoCo build completed!"
}

run_ros_build() {
    local packages=("$@")
    local package_list=$(IFS=' '; echo "${packages[*]}")

    print_header "[Running ROS Build]"

    # Clean existing symlinks
    clean_existing_symlinks "${packages[@]}"

    # Detect incompatible artifacts
    detect_incompatible_build_artifacts

    # Colcon must see every dual-ROS package before resolving dependency closure.
    create_symlinks_for_all_packages

    # Execute build
    if [ ${#packages[@]} -eq 0 ]; then
        if [[ "$ROS_DISTRO" == "noetic" ]]; then
            print_header "[Using catkin build]"
            print_info "Building all packages..."
            catkin build
        else
            print_header "[Using colcon build]"
            print_info "Building all packages..."
            colcon build --merge-install --symlink-install
        fi
    else
        if [[ "$ROS_DISTRO" == "noetic" ]]; then
            print_header "[Using catkin build]"
            print_info "Building specific packages: $package_list"
            catkin build $package_list
        else
            print_header "[Using colcon build]"
            print_info "Building specific packages: $package_list"
            colcon build --merge-install --symlink-install \
                --packages-up-to "${packages[@]}"
        fi
    fi

    print_success "ROS build completed!"
}

# ========================
# Clean Functions
# ========================

clean_workspace() {
    local packages=("$@")

    print_header "[Cleaning Workspace]"

    # Show what will be cleaned
    print_info "The following will be cleaned:"
    if [ ${#packages[@]} -eq 0 ]; then
        echo "  - All package.xml symlinks in directory src/"
    else
        echo "  - Package.xml symlinks for: ${packages[*]}"
    fi
    echo "  - directory build/"
    echo "  - directory cmake_build/"
    echo "  - directory devel/"
    echo "  - directory install/"
    echo "  - directory log/"
    echo "  - directory logs/"
    echo "  - directory .catkin_tools/"

    # Ask for confirmation
    if [ ${#packages[@]} -eq 0 ]; then
        if ! ask_confirmation "Are you sure you want to clean ALL symlinks and build artifacts?"; then
            print_warning "Clean operation cancelled."
            exit 0
        fi
    else
        if ! ask_confirmation "Are you sure you want to clean symlinks for specified packages and build artifacts?"; then
            print_warning "Clean operation cancelled."
            exit 0
        fi
    fi

    # Remove package.xml symlinks
    if [ ${#packages[@]} -eq 0 ]; then
        print_info "Removing all package.xml symlinks..."
        find src -name "package.xml" -type l -delete
        print_success "Removed all symlinks"
    else
        print_info "Removing symlinks for specific packages..."
        for package_name in "${packages[@]}"; do
            package_dir=$(find src -name "$package_name" -type d | head -n 1)
            if [ -n "$package_dir" ]; then
                if [ -L "$package_dir/package.xml" ]; then
                    rm -f "$package_dir/package.xml"
                    print_success "Removed symlink from $package_name"
                else
                    print_warning "No symlink found for $package_name"
                fi
            else
                print_error "Package '$package_name' not found in src directory"
            fi
        done
    fi

    # Clean build artifacts
    print_info "Cleaning build artifacts..."
    rm -rf build/ cmake_build/ devel/ install/ log/ logs/ .catkin_tools/

    print_success "Clean completed!"
}

clean_existing_symlinks() {
    local packages=("$@")

    print_header "[Cleaning Existing Symlinks]"

    if [ ${#packages[@]} -eq 0 ]; then
        print_info "Removing all existing package.xml symlinks..."
        find src -name "package.xml" -type l -delete
        print_success "Removed all existing symlinks"
    else
        print_info "Removing existing symlinks for specified packages..."
        removed_packages=()
        for package_name in "${packages[@]}"; do
            package_dir=$(find src -name "$package_name" -type d | head -n 1)
            if [ -n "$package_dir" ] && [ -L "$package_dir/package.xml" ]; then
                rm -f "$package_dir/package.xml"
                removed_packages+=("$package_name")
            fi
        done

        if [ ${#removed_packages[@]} -gt 0 ]; then
            print_success "Removed existing symlinks from: ${removed_packages[*]}"
        else
            print_warning "No existing symlinks found"
        fi
    fi
}

# ========================
# ROS Specific Functions
# ========================

detect_incompatible_build_artifacts() {
    print_header "[Checking for Incompatible Build Artifacts]"

    local needs_cleanup=false

    # Check for ROS1 artifacts when using ROS2
    if [[ "$ROS_DISTRO" != "noetic" ]]; then
        if [ -d "devel" ] || [ -d ".catkin_tools" ]; then
            print_warning "Found ROS1 build artifacts (devel/ or .catkin_tools/) while using ROS2. Cleaning workspace..."
            needs_cleanup=true
        fi
    fi

    # Check for ROS2 artifacts when using ROS1
    if [[ "$ROS_DISTRO" == "noetic" ]]; then
        if [ -d "install" ] || [ -d "log" ]; then
            print_warning "Found ROS2 build artifacts (install/ or log/) while using ROS1. Cleaning workspace..."
            needs_cleanup=true
        fi
    fi

    if [ "$needs_cleanup" = true ]; then
        clean_workspace
    else
        print_success "No incompatible build artifacts found"
    fi
}

create_symlinks_for_package() {
    local package_dir="$1"
    local package_name=$(basename "$package_dir")

    if [ -d "$package_dir" ]; then
        if [ -f "$package_dir/package.ros1.xml" ] && [ -f "$package_dir/package.ros2.xml" ]; then
            [ -e "$package_dir/package.xml" ] && rm -f "$package_dir/package.xml"

            if [[ "$ROS_DISTRO" == "noetic" ]]; then
                ln -s package.ros1.xml "$package_dir/package.xml"
                return 0
            elif [[ "$ROS_DISTRO" == "foxy" || "$ROS_DISTRO" == "humble" ]]; then
                ln -s package.ros2.xml "$package_dir/package.xml"
                return 0
            else
                print_error "Unknown ROS version: $ROS_DISTRO"
                return 1
            fi
        fi
    fi
    return 1
}

create_symlinks_for_all_packages() {
    print_header "[Creating Symlinks for All Packages]"

    created_packages=()
    while IFS= read -r -d '' package_dir; do
        package_dir=$(dirname "$package_dir")
        package_name=$(basename "$package_dir")
        if create_symlinks_for_package "$package_dir"; then
            created_packages+=("$package_name")
        fi
    done < <(find src -name "package.ros1.xml" -print0)

    if [ ${#created_packages[@]} -gt 0 ]; then
        print_success "Created symlinks for: ${created_packages[*]}"
    else
        print_warning "No packages with dual ROS support found"
    fi
}

# ========================
# Main Script
# ========================

show_usage() {
    print_header "[Build System Usage]"
    print_header
    echo -e "Usage: $0 [OPTIONS] [PACKAGE_NAMES...]"
    echo ""
    echo -e "${COLOR_INFO}Options:${COLOR_RESET}"
    echo -e "  -c, --clean      Clean workspace (remove symlinks and build artifacts)"
    echo -e "  -mj,--mujoco     Build with MuJoCo simulator support (CMake only)"
    echo -e "  -h, --help       Show this help message"
    echo ""
    echo -e "${COLOR_INFO}Examples:${COLOR_RESET}"
    echo -e "  $0                    # Build all ROS packages"
    echo -e "  $0 package1 package2  # Build specific ROS packages"
    echo -e "  $0 -c                 # Clean all symlinks and build artifacts"
    echo -e "  $0 --clean package1   # Clean specific package and build artifacts"
    echo -e "  $0 -mj                # Build with CMake and MuJoCo simulator support"
}

main() {
    local packages=()
    local clean_mode=false
    local mujoco_mode=false

    # Parse command line arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            -c|--clean) clean_mode=true; shift ;;
            -mj|--mujoco) mujoco_mode=true; shift ;;
            -h|--help) show_usage; exit 0 ;;
            --) shift; packages+=("$@"); break ;;
            -*) print_error "Unknown option: $1"; show_usage; exit 1 ;;
            *) packages+=("$1"); shift ;;
        esac
    done

    # Handle MuJoCo build mode
    if [ "$mujoco_mode" = true ]; then
        prepare_build_platform
        setup_system_dependencies
        setup_inference_runtime
        validate_lw_description
        setup_mujoco
        run_mujoco_build
        exit 0
    fi

    # Handle clean mode
    if [ "$clean_mode" = true ]; then
        clean_workspace "${packages[@]}"
        exit 0
    fi

    # Handle ROS build
    if [ -z "$ROS_DISTRO" ]; then
        print_error "ROS environment not detected. Please source your ROS setup.bash first."
        print_info "For production deployment, use src/rl_sar/scripts/build_lw_deployment.sh."
        exit 1
    fi

    prepare_build_platform
    setup_system_dependencies
    setup_inference_runtime
    if [ "$IS_JETSON" = false ]; then
        if [ ${#packages[@]} -eq 0 ]; then
            setup_mujoco
        else
            for package in "${packages[@]}"; do
                if [ "$package" = rl_sar ]; then
                    setup_mujoco
                    break
                fi
            done
        fi
    fi
    validate_lw_description
    run_ros_build "${packages[@]}"
}

main "$@"
