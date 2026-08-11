cmake_minimum_required(VERSION 3.10)
project(rl_sar VERSION 4.0.0 LANGUAGES CXX)

# Set C++ standard globally
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# System environment detection
set(SYSTEM_TYPE ${CMAKE_SYSTEM_NAME})

set(USE_CMAKE OFF CACHE BOOL "Use Cmake build system")
if(USE_CMAKE)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
endif()

if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "noetic")
        add_compile_definitions(USE_ROS1)
    elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        add_compile_definitions(USE_ROS2)
    endif()
else()
    message(WARNING "ROS_DISTRO not set, assuming non-ROS mode")
    add_compile_definitions(USE_CMAKE)
endif()

# MuJoCo simulator option
set(USE_MUJOCO OFF CACHE BOOL "Build with MuJoCo simulator support")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()
string(TOUPPER "${CMAKE_BUILD_TYPE}" UPPER_BUILD_TYPE)
if(UPPER_BUILD_TYPE STREQUAL "DEBUG")
    add_compile_options(-g)
endif()

# Get project root directory
get_filename_component(PROJECT_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

option(
    LW_PRODUCTION_DEPLOYMENT
    "Build a verified, relocatable LW real-robot deployment bundle"
    OFF
)
set(RL_SAR_SOURCE_COMMIT "unverified")
if(LW_PRODUCTION_DEPLOYMENT)
    if(NOT CMAKE_BUILD_TYPE STREQUAL "Release")
        message(FATAL_ERROR
            "LW_PRODUCTION_DEPLOYMENT requires CMAKE_BUILD_TYPE=Release")
    endif()
    find_package(Git REQUIRED)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} -C "${PROJECT_ROOT_DIR}" rev-parse HEAD
        RESULT_VARIABLE LW_GIT_REVISION_RESULT
        OUTPUT_VARIABLE RL_SAR_SOURCE_COMMIT
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    string(LENGTH "${RL_SAR_SOURCE_COMMIT}" LW_SOURCE_COMMIT_LENGTH)
    if(NOT LW_GIT_REVISION_RESULT EQUAL 0
       OR NOT LW_SOURCE_COMMIT_LENGTH EQUAL 40
       OR NOT RL_SAR_SOURCE_COMMIT MATCHES "^[0-9a-f]+$")
        message(FATAL_ERROR
            "Cannot determine the source commit for LW deployment")
    endif()
    execute_process(
        COMMAND ${GIT_EXECUTABLE} -C "${PROJECT_ROOT_DIR}"
                status --porcelain --untracked-files=all
        RESULT_VARIABLE LW_GIT_STATUS_RESULT
        OUTPUT_VARIABLE LW_GIT_STATUS
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(NOT LW_GIT_STATUS_RESULT EQUAL 0 OR NOT LW_GIT_STATUS STREQUAL "")
        message(FATAL_ERROR
            "LW production deployment must be built from a clean Git worktree")
    endif()
endif()

add_definitions(-DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DPOLICY_DIR="${PROJECT_ROOT_DIR}/policy")
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "noetic")
        find_package(catkin REQUIRED COMPONENTS
            controller_manager
            genmsg
            joint_state_controller
            robot_state_publisher
            roscpp
            # gazebo_ros
            std_msgs
            tf
            geometry_msgs
            robot_msgs
            robot_joint_controller
            rospy
        )
        catkin_package(
            CATKIN_DEPENDS
            robot_joint_controller
            rospy
        )
    elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        find_package(ament_cmake REQUIRED)
        find_package(ament_index_cpp REQUIRED)
        find_package(rclcpp REQUIRED)
        find_package(geometry_msgs REQUIRED)
        find_package(sensor_msgs REQUIRED)
        find_package(realtime_tools REQUIRED)
        ament_package()
    endif()
    add_compile_options(${GAZEBO_CXX_FLAGS})
    # find_package(gazebo REQUIRED)
endif()

# Inference Runtime

set(
    INFERENCE_RUNTIME_DIR
    "${PROJECT_ROOT_DIR}/library/inference_runtime"
    CACHE PATH "Inference runtime dependency root"
)

# ONNX Runtime Setup
set(USE_ONNX "OFF")
set(ONNX_RUNTIME_LIB "")
set(ONNX_RUNTIME_DIR "${INFERENCE_RUNTIME_DIR}/onnxruntime")

# Resolve the same Jetson mode used by the shell build and dependency scripts.
execute_process(
    COMMAND bash "${PROJECT_ROOT_DIR}/scripts/detect_jetson.sh"
    RESULT_VARIABLE JETSON_DETECTION_RESULT
    OUTPUT_VARIABLE JETSON_DETECTION_OUTPUT
    ERROR_VARIABLE JETSON_DETECTION_ERROR
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(NOT JETSON_DETECTION_RESULT EQUAL 0)
    message(FATAL_ERROR
        "Jetson platform detection failed: "
        "${JETSON_DETECTION_OUTPUT}${JETSON_DETECTION_ERROR}")
endif()
if(JETSON_DETECTION_OUTPUT MATCHES "^true;(.+)$")
    set(IS_JETSON TRUE)
    set(JETSON_DETECTION_SOURCE "${CMAKE_MATCH_1}")
elseif(JETSON_DETECTION_OUTPUT MATCHES "^false;(.+)$")
    set(IS_JETSON FALSE)
    set(JETSON_DETECTION_SOURCE "${CMAKE_MATCH_1}")
else()
    message(FATAL_ERROR
        "Unexpected Jetson detection output: ${JETSON_DETECTION_OUTPUT}")
endif()
message(STATUS
    "Jetson mode: ${IS_JETSON} (${JETSON_DETECTION_SOURCE})")

# if(EXISTS "${ONNX_RUNTIME_DIR}/include" AND EXISTS "${ONNX_RUNTIME_DIR}/lib" AND NOT IS_JETSON)
if(EXISTS "${ONNX_RUNTIME_DIR}/include" AND EXISTS "${ONNX_RUNTIME_DIR}/lib")
    set(USE_ONNX "ON")
    add_compile_definitions(USE_ONNX)

    message(STATUS "Found ONNX Runtime at: ${ONNX_RUNTIME_DIR}")

    # Read ONNX Runtime version
    if(EXISTS "${ONNX_RUNTIME_DIR}/VERSION_NUMBER")
        file(READ "${ONNX_RUNTIME_DIR}/VERSION_NUMBER" ONNX_RUNTIME_VERSION)
        string(STRIP "${ONNX_RUNTIME_VERSION}" ONNX_RUNTIME_VERSION)
    endif()

    include_directories("${ONNX_RUNTIME_DIR}/include")

    # Detect library extension
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        set(LIB_EXT "so")
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
        set(LIB_EXT "dylib")
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
        set(LIB_EXT "lib")
    endif()

    # Find library
    if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
        set(LIB_NAMES onnxruntime.lib)
    else()
        set(LIB_NAMES libonnxruntime.${LIB_EXT} onnxruntime.${LIB_EXT})
    endif()

    find_library(ONNX_RUNTIME_LIB_PATH
        NAMES ${LIB_NAMES}
        PATHS "${ONNX_RUNTIME_DIR}/lib"
        NO_DEFAULT_PATH
    )

    if(ONNX_RUNTIME_LIB_PATH)
        if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
            add_library(onnxruntime_prebuilt STATIC IMPORTED GLOBAL)
        else()
            add_library(onnxruntime_prebuilt SHARED IMPORTED GLOBAL)
        endif()

        set_target_properties(onnxruntime_prebuilt PROPERTIES
            IMPORTED_LOCATION "${ONNX_RUNTIME_LIB_PATH}"
            INTERFACE_INCLUDE_DIRECTORIES "${ONNX_RUNTIME_DIR}/include"
        )

        set(ONNX_RUNTIME_LIB onnxruntime_prebuilt)
        message(STATUS "ONNX Runtime library: ${ONNX_RUNTIME_LIB_PATH}")
    else()
        message(WARNING "ONNX Runtime library not found")
        set(USE_ONNX "OFF")
    endif()
else()
    message(WARNING "ONNX Runtime not found. Please run: bash download_inference_runtime.sh onnx")
endif()

# LibTorch Setup
set(USE_TORCH "OFF")
set(TORCH_LIBRARIES "")
set(LIBTORCH_DIR "${INFERENCE_RUNTIME_DIR}/libtorch")

if(EXISTS "${LIBTORCH_DIR}/include" AND EXISTS "${LIBTORCH_DIR}/lib")
    set(USE_TORCH "ON")
    add_compile_definitions(USE_TORCH)

    message(STATUS "Found LibTorch at: ${LIBTORCH_DIR}")

    # For Jetson with old CMake: set common Jetson CUDA architectures to avoid CUDA17 requirement
    if(CMAKE_VERSION VERSION_LESS "3.18")
        # Common Jetson architectures: 5.3(Nano), 6.2(TX2), 7.2(Xavier), 8.7(Orin)
        set(TORCH_CUDA_ARCH_LIST "5.3;6.2;7.2;8.7" CACHE STRING "CUDA architectures")
        set(ENV{TORCH_CUDA_ARCH_LIST} "5.3;6.2;7.2;8.7")
        message(STATUS "CMake ${CMAKE_VERSION} detected - using common Jetson CUDA architectures")
    endif()

    set(CMAKE_PREFIX_PATH "${LIBTORCH_DIR};${CMAKE_PREFIX_PATH}")
    find_package(Torch QUIET)

    if(Torch_FOUND)
        message(STATUS "LibTorch version: ${Torch_VERSION}")
        message(STATUS "Torch libraries: ${TORCH_LIBRARIES}")
    else()
        set(USE_TORCH "OFF")
        message(WARNING "LibTorch found but find_package(Torch) failed")
    endif()
else()
    message(WARNING "LibTorch not found. Please run: bash download_inference_runtime.sh libtorch")
endif()

# RPATH Configuration for inference libraries
if(USE_TORCH OR USE_ONNX)
    set(CMAKE_SKIP_BUILD_RPATH FALSE)
    set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)

    set(PREBUILT_LIB_PATHS "")
    if(USE_TORCH)
        list(APPEND PREBUILT_LIB_PATHS "${LIBTORCH_DIR}/lib")
    endif()
    if(USE_ONNX)
        list(APPEND PREBUILT_LIB_PATHS "${ONNX_RUNTIME_DIR}/lib")
    endif()
    # OpenMP path will be added later after it's found on macOS
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${PREBUILT_LIB_PATHS}")

    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--disable-new-dtags")
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--disable-new-dtags")
    endif()

    message(STATUS "Configured RPATH:")
    foreach(path ${PREBUILT_LIB_PATHS})
        message(STATUS "  - ${path}")
    endforeach()
endif()

find_package(TBB REQUIRED)
find_package(Threads REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

include(CTest)
if(BUILD_TESTING)
    add_executable(test_loop_lifecycle
        test/test_loop_lifecycle.cpp
    )
    target_include_directories(test_loop_lifecycle PRIVATE
        library/core/logger
        library/core/loop
    )
    target_link_libraries(test_loop_lifecycle
        Threads::Threads
    )
    add_test(NAME loop_lifecycle COMMAND test_loop_lifecycle)

    add_executable(test_loop_timing
        test/test_loop_timing.cpp
    )
    target_include_directories(test_loop_timing PRIVATE
        library/core/logger
        library/core/loop
    )
    target_link_libraries(test_loop_timing
        Threads::Threads
    )
    add_test(NAME loop_timing COMMAND test_loop_timing)

    add_executable(test_sensor_readiness
        test/test_sensor_readiness.cpp
    )
    target_include_directories(test_sensor_readiness PRIVATE
        library/core/safety
    )
    add_test(NAME sensor_readiness COMMAND test_sensor_readiness)

    if(SYSTEM_TYPE STREQUAL "Linux")
        add_executable(test_lw_serial_sdk
            test/test_lw_serial_sdk.cpp
        )
        target_include_directories(test_lw_serial_sdk PRIVATE
            library/thirdparty/robot_sdk/lfr/LW_sdk
        )
        add_test(NAME lw_serial_sdk COMMAND test_lw_serial_sdk)
    endif()
endif()

# Add Python library path to RPATH
if(Python3_LIBRARIES)
    get_filename_component(PYTHON_LIB_DIR "${Python3_LIBRARIES}" DIRECTORY)
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${PYTHON_LIB_DIR}")
    message(STATUS "Added Python library path to RPATH: ${PYTHON_LIB_DIR}")
endif()

if(SYSTEM_TYPE STREQUAL "Linux")
    set(YAML_CPP_LIBRARIES yaml-cpp)
link_directories(/usr/local/lib)
include_directories(${YAML_CPP_INCLUDE_DIR})
    find_package(OpenSSL REQUIRED)
    add_library(
        lw_deployment_bundle
        library/core/deployment/lw_deployment_bundle.cpp
    )
    target_include_directories(
        lw_deployment_bundle
        PUBLIC library/core/deployment
    )
    target_link_libraries(
        lw_deployment_bundle
        PUBLIC ${YAML_CPP_LIBRARIES} OpenSSL::Crypto
    )
elseif(SYSTEM_TYPE STREQUAL "Darwin")
    find_library(YAML_CPP_LIB yaml-cpp REQUIRED)
    set(YAML_CPP_LIBRARIES ${YAML_CPP_LIB})
    link_directories(/opt/homebrew/lib)
    if(YAML_CPP_INCLUDE_DIR)
        include_directories(${YAML_CPP_INCLUDE_DIR})
    endif()

    # Find OpenMP (required by LibTorch on macOS)
    find_library(OMP_LIB NAMES omp PATHS /opt/homebrew/lib /opt/homebrew/opt/libomp/lib)
    if(OMP_LIB)
        message(STATUS "Found OpenMP library: ${OMP_LIB}")
        set(OPENMP_LIBRARIES ${OMP_LIB})
        # Add OpenMP path to RPATH (for LibTorch dependency)
        get_filename_component(OMP_LIB_DIR "${OPENMP_LIBRARIES}" DIRECTORY)
        set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${OMP_LIB_DIR}")
        message(STATUS "Added OpenMP to RPATH: ${OMP_LIB_DIR}")
    else()
        message(WARNING "OpenMP not found. Please install: brew install libomp")
    endif()
endif()

# LW_sdk
add_library(LW_sdk INTERFACE)
target_include_directories(LW_sdk INTERFACE
    library/thirdparty/robot_sdk/lfr/LW_sdk
)
target_link_libraries(LW_sdk INTERFACE Threads::Threads)

if(NOT USE_CMAKE AND "$ENV{ROS_DISTRO}" MATCHES "foxy|humble")
    add_library(
        lw_debug_publisher
        library/core/debug/lw_debug_publisher.cpp
    )
    target_include_directories(lw_debug_publisher PUBLIC
        library/core/debug
        library/core/safety
        library/thirdparty/robot_sdk/lfr/LW_sdk
    )
    ament_target_dependencies(lw_debug_publisher
        rclcpp
        sensor_msgs
        realtime_tools
    )

    if(BUILD_TESTING)
        add_executable(
            test_lw_debug_publisher
            test/test_lw_debug_publisher.cpp
        )
        target_link_libraries(
            test_lw_debug_publisher
            lw_debug_publisher
        )
        ament_target_dependencies(test_lw_debug_publisher
            rclcpp
            sensor_msgs
        )
        add_test(NAME lw_debug_publisher COMMAND test_lw_debug_publisher)
    endif()
endif()

if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "noetic")
        include_directories(${catkin_INCLUDE_DIRS})
    endif()
endif()

include_directories(
    include
    library/core/matplotlibcpp
    library/core/observation_buffer
    library/core/rl_sdk
    library/core/loop
    library/core/safety
    library/core/debug
    library/core/fsm
    library/core/vector_math
    library/core/motion_loader
    library/core/inference_runtime
    library/core/logger
    fsm_robot
)

# library for inference runtime
add_library(inference_runtime library/core/inference_runtime/inference_runtime.cpp)
if(USE_TORCH)
    target_link_libraries(inference_runtime PUBLIC ${TORCH_LIBRARIES})
    target_compile_definitions(inference_runtime PRIVATE USE_TORCH)
    # Link OpenMP on macOS (required by LibTorch)
    if(SYSTEM_TYPE STREQUAL "Darwin" AND OPENMP_LIBRARIES)
        target_link_libraries(inference_runtime PUBLIC ${OPENMP_LIBRARIES})
    endif()
endif()
if(USE_ONNX)
    target_link_libraries(inference_runtime PUBLIC ${ONNX_RUNTIME_LIB})
    target_compile_definitions(inference_runtime PRIVATE USE_ONNX)
endif()
if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        install(TARGETS inference_runtime DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

# library for LW motion loader
add_library(motion_loader_lw library/core/motion_loader/motion_loader_lw.cpp)

# library for rl sdk
add_library(
    lw_configuration_validation
    library/core/rl_sdk/lw_configuration_validation.cpp
)
target_link_libraries(lw_configuration_validation PUBLIC
    inference_runtime
    ${YAML_CPP_LIBRARIES}
)
add_library(rl_sdk library/core/rl_sdk/rl_sdk.cpp)
add_library(
    rl_sdk_lw_deployment
    library/core/rl_sdk/rl_sdk.cpp
)
target_compile_definitions(
    rl_sdk_lw_deployment
    PUBLIC RL_REQUIRE_EXPLICIT_POLICY_ROOT
)
foreach(RL_SDK_TARGET rl_sdk rl_sdk_lw_deployment)
    target_link_libraries(${RL_SDK_TARGET} PUBLIC
        motion_loader_lw
        inference_runtime
        lw_configuration_validation
        Python3::Python
        Python3::Module
        TBB::tbb
    )
endforeach()
find_package(Python3 COMPONENTS NumPy)
if(Python3_NumPy_FOUND)
    target_link_libraries(rl_sdk PUBLIC Python3::NumPy)
    target_link_libraries(rl_sdk_lw_deployment PUBLIC Python3::NumPy)
else()
    target_compile_definitions(rl_sdk PUBLIC WITHOUT_NUMPY)
    target_compile_definitions(rl_sdk_lw_deployment PUBLIC WITHOUT_NUMPY)
endif()
if(BUILD_TESTING)
    if(SYSTEM_TYPE STREQUAL "Linux")
        add_executable(
            test_lw_deployment_bundle
            test/test_lw_deployment_bundle.cpp
        )
        target_link_libraries(
            test_lw_deployment_bundle
            PRIVATE lw_deployment_bundle
        )
        add_test(
            NAME lw_deployment_bundle
            COMMAND test_lw_deployment_bundle
        )
        add_test(
            NAME lw_deployment_manifest_generator
            COMMAND ${Python3_EXECUTABLE}
                    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_generate_lw_deployment_manifest.py
                    ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_lw_deployment_manifest.py
        )
        add_test(
            NAME lw_runtime_dependencies
            COMMAND ${Python3_EXECUTABLE}
                    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_lw_runtime_dependencies.py
                    ${CMAKE_CURRENT_SOURCE_DIR}/package.ros2.xml
                    ${PROJECT_ROOT_DIR}/src/fdilink_ahrs_ROS2/package.xml
        )
        add_test(
            NAME lw_jetson_detection
            COMMAND ${Python3_EXECUTABLE}
                    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_jetson_detection.py
                    ${PROJECT_ROOT_DIR}/scripts/detect_jetson.sh
        )
        add_test(
            NAME lw_build_workflow
            COMMAND ${Python3_EXECUTABLE}
                    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_build_workflow.py
                    ${PROJECT_ROOT_DIR}/build.sh
                    ${PROJECT_ROOT_DIR}/build_LW.sh
                    ${PROJECT_ROOT_DIR}/README_CN.md
                    ${PROJECT_ROOT_DIR}/docs/LW_BUILD_DEPLOYMENT_CN.md
                    ${PROJECT_ROOT_DIR}/scripts/install_build_dependencies.sh
        )
        add_executable(
            test_lw_terminal_keyboard
            test/test_lw_terminal_keyboard.cpp
        )
        target_link_libraries(
            test_lw_terminal_keyboard
            PRIVATE
                rl_sdk
                observation_buffer
                ${YAML_CPP_LIBRARIES}
        )
        add_test(
            NAME lw_terminal_keyboard
            COMMAND test_lw_terminal_keyboard
        )
        add_test(
            NAME lw_real_keyboard_integration
            COMMAND ${Python3_EXECUTABLE}
                    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_lw_real_keyboard_integration.py
                    ${CMAKE_CURRENT_SOURCE_DIR}/src/rl_real_LW.cpp
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/rl_real_LW.hpp
                    ${CMAKE_CURRENT_SOURCE_DIR}/launch/rl_real_LW.launch.py
                    ${PROJECT_ROOT_DIR}/docs/LW_BUILD_DEPLOYMENT_CN.md
        )
    endif()

    add_executable(test_lw_fsm_transitions
        test/test_lw_fsm_transitions.cpp
    )
    target_link_libraries(test_lw_fsm_transitions PRIVATE
        rl_sdk
        observation_buffer
        ${YAML_CPP_LIBRARIES}
    )
    add_test(NAME lw_fsm_transitions COMMAND test_lw_fsm_transitions)

    add_executable(test_lw_control_safety
        test/test_lw_control_safety.cpp
    )
    target_link_libraries(test_lw_control_safety PRIVATE
        rl_sdk
        observation_buffer
        ${YAML_CPP_LIBRARIES}
    )
    add_test(NAME lw_control_safety COMMAND test_lw_control_safety)

    add_executable(test_lw_safety_policy
        test/test_lw_safety_policy.cpp
    )
    target_include_directories(test_lw_safety_policy PRIVATE
        library/core/safety
    )
    add_test(NAME lw_safety_policy COMMAND test_lw_safety_policy)

    add_executable(test_lw_joystick_safety
        test/test_lw_joystick_safety.cpp
    )
    add_test(NAME lw_joystick_safety COMMAND test_lw_joystick_safety)

    add_executable(test_lw_runtime_sync
        test/test_lw_runtime_sync.cpp
    )
    target_link_libraries(test_lw_runtime_sync PRIVATE
        Threads::Threads
    )
    add_test(NAME lw_runtime_sync COMMAND test_lw_runtime_sync)

    add_executable(test_lw_policy_output_transport
        test/test_lw_policy_output_transport.cpp
    )
    target_link_libraries(test_lw_policy_output_transport PRIVATE
        rl_sdk
        observation_buffer
        ${YAML_CPP_LIBRARIES}
        Threads::Threads
    )
    add_test(
        NAME lw_policy_output_transport
        COMMAND test_lw_policy_output_transport
    )

    add_executable(test_lw_motion_reference_rate
        test/test_lw_motion_reference_rate.cpp
    )
    target_link_libraries(test_lw_motion_reference_rate PRIVATE
        rl_sdk
        observation_buffer
        ${YAML_CPP_LIBRARIES}
    )
    add_test(
        NAME lw_motion_reference_rate
        COMMAND test_lw_motion_reference_rate
    )

    add_executable(test_lw_motion_loader
        test/test_lw_motion_loader.cpp
    )
    target_link_libraries(test_lw_motion_loader PRIVATE
        motion_loader_lw
    )
    add_test(
        NAME lw_motion_loader
        COMMAND test_lw_motion_loader
    )

    add_executable(test_lw_configuration_validation
        test/test_lw_configuration_validation.cpp
    )
    target_link_libraries(test_lw_configuration_validation PRIVATE
        rl_sdk
        observation_buffer
        ${YAML_CPP_LIBRARIES}
    )
    add_test(
        NAME lw_configuration_validation
        COMMAND test_lw_configuration_validation
    )

    add_test(
        NAME lw_repository_scope
        COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/test/test_lw_repository_scope.py
                ${PROJECT_ROOT_DIR}
    )
endif()
if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        install(TARGETS rl_sdk DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

# library for observation buffer
add_library(observation_buffer library/core/observation_buffer/observation_buffer.cpp)
if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        install(TARGETS observation_buffer DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

# executable for rl sim LW
if(SYSTEM_TYPE STREQUAL "Linux"
   AND NOT IS_JETSON
   AND NOT LW_PRODUCTION_DEPLOYMENT)
    if(NOT USE_CMAKE AND "$ENV{ROS_DISTRO}" MATCHES "foxy|humble")
        add_compile_definitions(USE_MUJOCO)
        set(MUJOCO_DIR "${PROJECT_ROOT_DIR}/library/mujoco")

        # Linux/Windows use standard structure
        if(NOT EXISTS "${MUJOCO_DIR}/include/mujoco/mujoco.h")
            message(FATAL_ERROR "MuJoCo not found. Please run: bash scripts/download_mujoco.sh")
        endif()
        set(MUJOCO_INCLUDE_DIR "${MUJOCO_DIR}/include")
        set(MUJOCO_LIB_DIR "${MUJOCO_DIR}/lib")

        # Add MuJoCo library path to global RPATH
        set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${MUJOCO_LIB_DIR}")
        message(STATUS "Added MuJoCo to RPATH: ${MUJOCO_LIB_DIR}")

        # Find GLFW
        find_package(glfw3 REQUIRED)

        # Platform-specific library names
        set(MUJOCO_LIB_NAME "libmujoco.so.3.2.7")

        # Import MuJoCo library with RPATH
        add_library(mujoco_prebuilt SHARED IMPORTED)
        set_target_properties(mujoco_prebuilt PROPERTIES
            IMPORTED_LOCATION "${MUJOCO_LIB_DIR}/${MUJOCO_LIB_NAME}"
            INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}"
            IMPORTED_NO_SONAME TRUE
        )

        # Include directories
        include_directories(
            ${MUJOCO_INCLUDE_DIR}
            library/thirdparty/joystick
        )

        # Source files
        file(GLOB MUJOCO_SIMULATE_SRC library/thirdparty/mujoco_simulate/*.cc)
        set(JOYSTICK_SRC library/thirdparty/joystick/joystick.cc)

        # Build executable
        add_executable(rl_sim_LW src/rl_sim_LW.cpp ${MUJOCO_SIMULATE_SRC} ${JOYSTICK_SRC})

        # Add include directories for mujoco_simulate and joystick headers
        target_include_directories(rl_sim_LW PRIVATE
            library/thirdparty/mujoco_simulate
            library/thirdparty/joystick
        )

        target_link_libraries(rl_sim_LW
            rl_sdk observation_buffer ${YAML_CPP_LIBRARIES} LW_sdk
            Threads::Threads mujoco_prebuilt glfw
        )
        if(BUILD_TESTING)
            add_executable(
                test_lw_description
                test/test_lw_description.cpp
            )
            target_compile_definitions(
                test_lw_description
                PRIVATE
                    "LW_DESCRIPTION_MJCF_DIR=\"${PROJECT_ROOT_DIR}/src/rl_sar_zoo/LW_description/mjcf\""
            )
            target_link_libraries(
                test_lw_description
                PRIVATE mujoco_prebuilt
            )
            add_test(NAME lw_description COMMAND test_lw_description)
        endif()
        ament_target_dependencies(rl_sim_LW
                rclcpp
                geometry_msgs
                sensor_msgs
        )
        install(TARGETS rl_sim_LW DESTINATION lib/${PROJECT_NAME})
        message(STATUS "MuJoCo configured: ${MUJOCO_DIR}")
    endif()
endif()

# executable for rl real LW
if(SYSTEM_TYPE STREQUAL "Linux")
    if(NOT USE_CMAKE AND "$ENV{ROS_DISTRO}" MATCHES "foxy|humble")
        if(NOT DEFINED JOYSTICK_SRC)
            set(JOYSTICK_SRC library/thirdparty/joystick/joystick.cc)
        endif()
        add_executable(rl_real_LW 
            src/rl_real_LW.cpp
            ${JOYSTICK_SRC}
        )
        target_include_directories(rl_real_LW PRIVATE
            library/thirdparty/joystick
        )
        target_link_libraries(rl_real_LW
            LW_sdk
            lw_debug_publisher
            rl_sdk_lw_deployment
            lw_deployment_bundle
            observation_buffer
            ${YAML_CPP_LIBRARIES}
            Threads::Threads
        )
        ament_target_dependencies(rl_real_LW
            ament_index_cpp
            rclcpp
            geometry_msgs
            sensor_msgs
            realtime_tools
        )
        target_compile_definitions(
            rl_real_LW
            PRIVATE RL_SAR_SOURCE_COMMIT="${RL_SAR_SOURCE_COMMIT}"
        )
        install(TARGETS rl_real_LW DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

# executable for actuator net
if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "noetic")
        catkin_install_python(PROGRAMS
            scripts/actuator_net.py
            DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
    elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        install(PROGRAMS
            scripts/actuator_net.py
            DESTINATION lib/${PROJECT_NAME}
        )
    endif()
endif()

if(NOT USE_CMAKE)
    if($ENV{ROS_DISTRO} MATCHES "foxy|humble")
        install(
            DIRECTORY launch worlds
            DESTINATION share/${PROJECT_NAME}/${dir}
        )
        if(LW_PRODUCTION_DEPLOYMENT)
            set(LW_DEPLOYMENT_POLICY_FILES
                LW/base.yaml
                LW/robot_lab/leg_loco/config.yaml
                LW/robot_lab/leg_loco/policy.onnx
                LW/robot_lab/leg_to_wheel/config.yaml
                LW/robot_lab/leg_to_wheel/leg_to_wheel_transform_60hz.csv
                LW/robot_lab/leg_to_wheel/policy.onnx
                LW/robot_lab/wheel_loco/config.yaml
                LW/robot_lab/wheel_loco/policy.onnx
                LW/robot_lab/wheel_to_leg/config.yaml
                LW/robot_lab/wheel_to_leg/policy.onnx
                LW/robot_lab/wheel_to_leg/wheel_to_leg_transform_60hz.csv
            )
            foreach(LW_POLICY_FILE IN LISTS LW_DEPLOYMENT_POLICY_FILES)
                get_filename_component(
                    LW_POLICY_DIRECTORY "${LW_POLICY_FILE}" DIRECTORY)
                install(
                    FILES "${PROJECT_ROOT_DIR}/policy/${LW_POLICY_FILE}"
                    DESTINATION
                        "share/${PROJECT_NAME}/deployment/LW/policy/${LW_POLICY_DIRECTORY}"
                )
            endforeach()
            configure_file(
                cmake/install_lw_deployment.cmake.in
                "${CMAKE_CURRENT_BINARY_DIR}/install_lw_deployment.cmake"
                @ONLY
            )
            install(
                SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/install_lw_deployment.cmake"
            )
        else()
            install(
                DIRECTORY ${PROJECT_ROOT_DIR}/policy
                DESTINATION share/${PROJECT_NAME}/${dir}
            )
        endif()
    endif()
endif()

# ========================
# mujoco
# ========================
if(USE_MUJOCO)
    add_compile_definitions(USE_MUJOCO)
    set(MUJOCO_DIR "${PROJECT_ROOT_DIR}/library/mujoco")

    # Check MuJoCo installation (different paths for different platforms)
    if(SYSTEM_TYPE STREQUAL "Darwin")
        # macOS uses framework structure
        if(NOT EXISTS "${MUJOCO_DIR}/Headers/mujoco.h")
            message(FATAL_ERROR "MuJoCo not found. Please run: bash scripts/download_mujoco.sh")
        endif()

        # Create compatible include structure in build directory for macOS
        # This allows <mujoco/mujoco.h> to work on macOS where headers are in Headers/ directory
        set(MUJOCO_COMPAT_INCLUDE_DIR "${CMAKE_BINARY_DIR}/mujoco_include")
        file(MAKE_DIRECTORY "${MUJOCO_COMPAT_INCLUDE_DIR}/mujoco")
        file(GLOB MUJOCO_HEADERS "${MUJOCO_DIR}/Headers/*.h")
        file(COPY ${MUJOCO_HEADERS} DESTINATION "${MUJOCO_COMPAT_INCLUDE_DIR}/mujoco")

        set(MUJOCO_INCLUDE_DIR "${MUJOCO_COMPAT_INCLUDE_DIR}")
        set(MUJOCO_LIB_DIR "${MUJOCO_DIR}/Versions/Current")
    else()
        # Linux/Windows use standard structure
        if(NOT EXISTS "${MUJOCO_DIR}/include/mujoco/mujoco.h")
            message(FATAL_ERROR "MuJoCo not found. Please run: bash scripts/download_mujoco.sh")
        endif()
        set(MUJOCO_INCLUDE_DIR "${MUJOCO_DIR}/include")
        set(MUJOCO_LIB_DIR "${MUJOCO_DIR}/lib")
    endif()

    # Add MuJoCo library path to global RPATH
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${MUJOCO_LIB_DIR}")
    message(STATUS "Added MuJoCo to RPATH: ${MUJOCO_LIB_DIR}")

    # Find GLFW
    find_package(glfw3 REQUIRED)

    # Platform-specific library names
    if(SYSTEM_TYPE STREQUAL "Linux")
        set(MUJOCO_LIB_NAME "libmujoco.so.3.2.7")
    elseif(SYSTEM_TYPE STREQUAL "Darwin")
        set(MUJOCO_LIB_NAME "libmujoco.3.2.7.dylib")
    elseif(SYSTEM_TYPE STREQUAL "Windows")
        set(MUJOCO_LIB_NAME "mujoco.dll")
    else()
        message(FATAL_ERROR "Unsupported platform: ${SYSTEM_TYPE}")
    endif()

    # Import MuJoCo library with RPATH
    add_library(mujoco_prebuilt SHARED IMPORTED)
    set_target_properties(mujoco_prebuilt PROPERTIES
        IMPORTED_LOCATION "${MUJOCO_LIB_DIR}/${MUJOCO_LIB_NAME}"
        INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}"
        IMPORTED_NO_SONAME TRUE
    )

    # Include directories
    include_directories(
        ${MUJOCO_INCLUDE_DIR}
        library/thirdparty/joystick
    )

    # Source files
    file(GLOB MUJOCO_SIMULATE_SRC library/thirdparty/mujoco_simulate/*.cc)
    set(JOYSTICK_SRC library/thirdparty/joystick/joystick.cc)

    # On macOS, also include Objective-C++ files for CoreVideo support
    if(SYSTEM_TYPE STREQUAL "Darwin")
        file(GLOB MUJOCO_SIMULATE_MM library/thirdparty/mujoco_simulate/*.mm)
        list(APPEND MUJOCO_SIMULATE_SRC ${MUJOCO_SIMULATE_MM})
    endif()

    # Build executable
    add_executable(rl_sim_mujoco src/rl_sim_mujoco.cpp ${MUJOCO_SIMULATE_SRC} ${JOYSTICK_SRC})

    # Suppress deprecation warnings for macOS 15.0 API changes in MuJoCo Simulate
    if(SYSTEM_TYPE STREQUAL "Darwin")
        set_source_files_properties(
            ${MUJOCO_SIMULATE_MM}
            PROPERTIES COMPILE_FLAGS "-Wno-deprecated-declarations"
        )
    endif()

    # Add include directories for mujoco_simulate and joystick headers
    target_include_directories(rl_sim_mujoco PRIVATE
        library/thirdparty/mujoco_simulate
        library/thirdparty/joystick
    )

    target_link_libraries(rl_sim_mujoco
        rl_sdk observation_buffer ${YAML_CPP_LIBRARIES}
        Threads::Threads mujoco_prebuilt glfw
    )

    # macOS specific frameworks and libraries
    if(SYSTEM_TYPE STREQUAL "Darwin")
        # OpenMP support
        if(OPENMP_LIBRARIES)
            target_link_libraries(rl_sim_mujoco ${OPENMP_LIBRARIES})
        endif()

        # Required frameworks for GlfwCoreVideo
        target_link_libraries(rl_sim_mujoco
            "-framework CoreVideo"
            "-framework OpenGL"
            "-framework AppKit"
        )
    endif()

    install(TARGETS rl_sim_mujoco DESTINATION bin)
    message(STATUS "MuJoCo configured: ${MUJOCO_DIR}")
endif()

# ========================
# only for test
# ========================
# add_executable(test_observation_buffer test/test_observation_buffer.cpp)
# target_link_libraries(test_observation_buffer
#     observation_buffer
# )
# install(TARGETS test_observation_buffer DESTINATION lib/${PROJECT_NAME})
# add_executable(test_vector_math test/test_vector_math.cpp)
# install(TARGETS test_vector_math DESTINATION lib/${PROJECT_NAME})

# add_executable(test_inference_stability test/test_inference_stability.cpp)
# target_link_libraries(test_inference_stability
#     inference_runtime
#     ${TORCH_LIBRARIES}
#     ${ONNX_RUNTIME_LIB}
# )
# install(TARGETS test_inference_stability DESTINATION lib/${PROJECT_NAME})

message(STATUS "========================================")
message(STATUS "       RL_SAR Build Configuration       ")
message(STATUS "========================================")
message(STATUS "Project: ${PROJECT_NAME} v${PROJECT_VERSION}")
message(STATUS "System: ${SYSTEM_TYPE}")
message(STATUS "Architecture: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "USE_CMAKE: ${USE_CMAKE}")
message(STATUS "----------------------------------------")
message(STATUS "Backend Configuration:")
message(STATUS "USE_TORCH: ${USE_TORCH}")
message(STATUS "USE_ONNX: ${USE_ONNX}")
if(USE_TORCH)
    if(Torch_VERSION)
        message(STATUS "LibTorch version: ${Torch_VERSION}")
    else()
        message(STATUS "LibTorch: Available")
    endif()
    if(IS_JETSON)
        message(STATUS "LibTorch: Jetson device detected")
        if(JETPACK_VERSION)
            message(STATUS "JetPack version: ${JETPACK_VERSION}")
        endif()
    endif()
endif()
if(USE_ONNX)
    if(ONNX_RUNTIME_VERSION)
        message(STATUS "ONNX Runtime version: ${ONNX_RUNTIME_VERSION}")
    elseif(onnxruntime_VERSION)
        message(STATUS "ONNX Runtime version: ${onnxruntime_VERSION}")
    else()
        message(STATUS "ONNX Runtime: Available")
    endif()
endif()
message(STATUS "----------------------------------------")
message(STATUS "Simulator Configuration:")
message(STATUS "USE_MUJOCO: ${USE_MUJOCO}")
if(USE_MUJOCO)
    message(STATUS "MuJoCo: Enabled")
endif()
message(STATUS "========================================")
