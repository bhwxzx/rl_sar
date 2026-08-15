#!/bin/bash
set -e

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WORKSPACE_ROOT="$SCRIPT_DIR"

# Load common utilities
source "${SCRIPT_DIR}/scripts/common.sh"

# Keep every relative build and cleanup path bound to this workspace even when
# the script is launched from another directory.
cd "$WORKSPACE_ROOT"

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

run_ros_build() {
    local packages=("$@")
    local package_list=$(IFS=' '; echo "${packages[*]}")

    print_header "[Running ROS Build]"

    # Detect incompatible artifacts
    detect_incompatible_build_artifacts

    # Execute build
    if [ ${#packages[@]} -eq 0 ]; then
        print_header "[Using colcon build]"
        print_info "Building all packages..."
        colcon build --symlink-install
    else
        print_header "[Using colcon build]"
        print_info "Building specific packages: $package_list"
        colcon build --symlink-install \
            --packages-up-to "${packages[@]}"
    fi

    print_success "ROS build completed!"
}

# ========================
# Clean Functions
# ========================

clean_all_workspace() {
    print_header "[Cleaning Workspace]"

    print_info "The following will be cleaned:"
    echo "  - directory build/"
    echo "  - directory cmake_build/"
    echo "  - directory install/"
    echo "  - directory log/"
    echo "  - directory logs/"

    if ! ask_confirmation "Are you sure you want to clean ALL build artifacts?"; then
        print_warning "Clean operation cancelled."
        exit 0
    fi

    print_info "Cleaning build artifacts..."
    rm -rf -- \
        "$WORKSPACE_ROOT/build" \
        "$WORKSPACE_ROOT/cmake_build" \
        "$WORKSPACE_ROOT/install" \
        "$WORKSPACE_ROOT/log" \
        "$WORKSPACE_ROOT/logs"

    print_success "Clean completed!"
}

validate_ros2_isolated_layout() {
    local install_root="$WORKSPACE_ROOT/install"
    local layout_marker="$install_root/.colcon_install_layout"
    local layout=""
    local artifact_root=""

    for artifact_root in "$WORKSPACE_ROOT/build" "$install_root"; do
        if [ -L "$artifact_root" ]; then
            print_error "Package cleanup refuses a symlinked artifact root: $artifact_root"
            return 1
        fi
        if [ -e "$artifact_root" ] && [ ! -d "$artifact_root" ]; then
            print_error "Package cleanup requires a directory artifact root: $artifact_root"
            return 1
        fi
    done

    if [ -e "$layout_marker" ]; then
        if [ ! -f "$layout_marker" ] || [ -L "$layout_marker" ]; then
            print_error "Invalid Colcon install layout marker: $layout_marker"
            return 1
        fi
        IFS= read -r layout < "$layout_marker" || true
        if [ "$layout" != "isolated" ]; then
            print_error "Package cleanup requires an isolated Colcon install layout; found '${layout:-unknown}'."
            print_info "Run './build.sh --clean' once, then './build.sh' to migrate safely."
            return 1
        fi
    elif [ -d "$install_root" ] && [ -n "$(find "$install_root" -mindepth 1 -print -quit)" ]; then
        print_error "Package cleanup cannot identify the existing install layout."
        print_info "Run './build.sh --clean' once, then './build.sh' to recreate it safely."
        return 1
    fi
}

resolve_ros2_clean_packages() {
    local requested=("$@")
    local discovered_output=""
    local affected_output=""
    local package_name=""
    local -A discovered=()
    local -A affected_seen=()

    if ! discovered_output=$(colcon list --names-only); then
        print_error "Failed to discover ROS 2 packages before cleanup."
        return 1
    fi
    while IFS= read -r package_name; do
        if [ -n "$package_name" ]; then
            if [[ "$package_name" = "." || "$package_name" = ".."
               || "$package_name" = */* ]]; then
                print_error "Colcon returned an unsafe package name: '$package_name'"
                return 1
            fi
            discovered["$package_name"]=1
        fi
    done <<< "$discovered_output"

    for package_name in "${requested[@]}"; do
        if [ -z "${discovered[$package_name]+present}" ]; then
            print_error "Unknown package '$package_name'; no files were removed."
            return 1
        fi
    done

    if ! affected_output=$(colcon list --names-only --topological-order \
        --packages-above "${requested[@]}"); then
        print_error "Failed to resolve packages which depend on: ${requested[*]}"
        return 1
    fi

    ROS2_CLEAN_PACKAGES=()
    while IFS= read -r package_name; do
        if [ -n "$package_name" ]; then
            if [ -z "${discovered[$package_name]+present}" ]; then
                print_error "Colcon returned an undiscovered affected package: '$package_name'"
                return 1
            fi
            if [ -z "${affected_seen[$package_name]+present}" ]; then
                affected_seen["$package_name"]=1
                ROS2_CLEAN_PACKAGES+=("$package_name")
            fi
        fi
    done <<< "$affected_output"
    if [ ${#ROS2_CLEAN_PACKAGES[@]} -eq 0 ]; then
        print_error "No packages were resolved for cleanup; no files were removed."
        return 1
    fi
}

clean_ros2_packages() {
    local requested=("$@")
    local package_name=""
    local index=0

    validate_ros2_isolated_layout
    resolve_ros2_clean_packages "${requested[@]}"

    print_header "[Cleaning Selected ROS 2 Packages]"
    print_info "Requested packages: ${requested[*]}"
    print_info "The following packages will be cleaned, including reverse dependencies:"
    for package_name in "${ROS2_CLEAN_PACKAGES[@]}"; do
        echo "  - build/$package_name"
        echo "  - install/$package_name"
    done
    print_info "Shared log directories and source package.xml files will be preserved."

    if ! ask_confirmation "Proceed with this package-scoped cleanup?"; then
        print_warning "Clean operation cancelled."
        return 0
    fi

    # Delete dependents before their dependencies. Package names come only
    # from Colcon discovery, and build/install roots were rejected if symlinked.
    for ((index=${#ROS2_CLEAN_PACKAGES[@]} - 1; index >= 0; index--)); do
        package_name="${ROS2_CLEAN_PACKAGES[$index]}"
        rm -rf -- \
            "$WORKSPACE_ROOT/build/$package_name" \
            "$WORKSPACE_ROOT/install/$package_name"
    done
    print_success "Selected ROS 2 package cleanup completed!"
}

clean_selected_packages() {
    local packages=("$@")
    local package_name=""

    for package_name in "${packages[@]}"; do
        if [[ ! "$package_name" =~ ^[A-Za-z][A-Za-z0-9_-]*$ ]]; then
            print_error "Invalid package name '$package_name'; no files were removed."
            return 1
        fi
    done

    validate_ros2_environment "Source ROS 2 before cleaning selected packages."
    clean_ros2_packages "${packages[@]}"
}

clean_workspace() {
    local packages=("$@")

    if [ ${#packages[@]} -eq 0 ]; then
        clean_all_workspace
    else
        clean_selected_packages "${packages[@]}"
    fi
}

# ========================
# ROS Specific Functions
# ========================

validate_ros2_environment() {
    local missing_message="$1"

    if [ -z "${ROS_DISTRO:-}" ]; then
        print_error "ROS 2 environment not detected. $missing_message"
        return 1
    fi
    case "$ROS_DISTRO" in
        foxy|humble)
            ;;
        *)
            print_error "Unsupported ROS distribution: $ROS_DISTRO"
            print_info "This workspace supports ROS 2 Foxy and Humble only."
            return 1
            ;;
    esac
}

detect_incompatible_build_artifacts() {
    print_header "[Checking for Incompatible Build Artifacts]"

    if [ -f "$WORKSPACE_ROOT/install/.colcon_install_layout" ]; then
        local install_layout=""
        IFS= read -r install_layout \
            < "$WORKSPACE_ROOT/install/.colcon_install_layout" || true
        if [ "$install_layout" = "merged" ]; then
            print_error "The existing ROS 2 workspace uses the legacy merged install layout."
            print_info "Run './build.sh --clean' once before rebuilding with isolated installs."
            return 1
        fi
    fi
    print_success "No incompatible build artifacts found"
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
    echo -e "  -c, --clean      Clean all artifacts, or selected packages and reverse dependencies"
    echo -e "  -h, --help       Show this help message"
    echo ""
    echo -e "${COLOR_INFO}Examples:${COLOR_RESET}"
    echo -e "  $0                    # Build all ROS 2 packages"
    echo -e "  $0 package1 package2  # Build specific ROS 2 packages"
    echo -e "  $0 -c                 # Clean all build artifacts"
    echo -e "  $0 --clean package1   # Clean a package and packages which depend on it"
}

main() {
    local packages=()
    local clean_mode=false

    # Parse command line arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            -c|--clean) clean_mode=true; shift ;;
            -h|--help) show_usage; exit 0 ;;
            --) shift; packages+=("$@"); break ;;
            -*) print_error "Unknown option: $1"; show_usage; exit 1 ;;
            *) packages+=("$1"); shift ;;
        esac
    done

    # Handle clean mode
    if [ "$clean_mode" = true ]; then
        clean_workspace "${packages[@]}"
        exit 0
    fi

    # Handle ROS 2 build
    if ! validate_ros2_environment "Please source your ROS 2 setup.bash first."; then
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
