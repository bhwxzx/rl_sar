cmake_minimum_required(VERSION 3.10)
project(rl_sar VERSION 4.0.0 LANGUAGES CXX)

# Set C++ standard globally
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(
    LW_STRICT_WARNINGS
    "Treat warnings in maintained LW targets as build errors"
    OFF
)
if(LW_STRICT_WARNINGS)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        add_compile_options(
            "$<$<COMPILE_LANGUAGE:CXX>:-Wall>"
            "$<$<COMPILE_LANGUAGE:CXX>:-Wextra>"
            "$<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>"
            "$<$<COMPILE_LANGUAGE:CXX>:-Werror>"
        )
    elseif(MSVC)
        add_compile_options(
            "$<$<COMPILE_LANGUAGE:CXX>:/W4>"
            "$<$<COMPILE_LANGUAGE:CXX>:/WX>"
        )
    else()
        message(FATAL_ERROR
            "LW_STRICT_WARNINGS is unsupported by ${CMAKE_CXX_COMPILER_ID}")
    endif()
endif()

function(lw_silence_vendor_warnings target_name)
    if(NOT TARGET ${target_name})
        message(FATAL_ERROR "Unknown LW vendor target: ${target_name}")
    endif()
    if(LW_STRICT_WARNINGS)
        if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
            target_compile_options(
                ${target_name} PRIVATE
                "$<$<COMPILE_LANGUAGE:CXX>:-w>"
            )
        elseif(MSVC)
            target_compile_options(
                ${target_name} PRIVATE
                "$<$<COMPILE_LANGUAGE:CXX>:/w>"
            )
        endif()
    endif()
endfunction()

# System environment detection
set(SYSTEM_TYPE ${CMAKE_SYSTEM_NAME})

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

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
ament_package()

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
set(RUNTIME_VALIDATOR "${PROJECT_ROOT_DIR}/scripts/validate_inference_runtime.sh")

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

set(RUNTIME_VALIDATION_ARCH "${CMAKE_SYSTEM_PROCESSOR}")
if(RUNTIME_VALIDATION_ARCH STREQUAL "")
    set(RUNTIME_VALIDATION_ARCH "${CMAKE_HOST_SYSTEM_PROCESSOR}")
endif()

if(EXISTS "${ONNX_RUNTIME_DIR}/include" AND EXISTS "${ONNX_RUNTIME_DIR}/lib")
    execute_process(
        COMMAND bash "${RUNTIME_VALIDATOR}" onnx
                "${ONNX_RUNTIME_DIR}" "${RUNTIME_VALIDATION_ARCH}"
        RESULT_VARIABLE ONNX_VALIDATION_RESULT
        OUTPUT_VARIABLE ONNX_VALIDATION_OUTPUT
        ERROR_VARIABLE ONNX_VALIDATION_ERROR
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_STRIP_TRAILING_WHITESPACE
    )
    if(NOT ONNX_VALIDATION_RESULT EQUAL 0)
        message(FATAL_ERROR
            "ONNX Runtime validation failed: "
            "${ONNX_VALIDATION_OUTPUT}${ONNX_VALIDATION_ERROR}")
    endif()

    message(STATUS "${ONNX_VALIDATION_OUTPUT}")

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
        set(USE_ONNX "ON")
        add_compile_definitions(USE_ONNX)
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

        if(LW_PRODUCTION_DEPLOYMENT)
            if(NOT ONNX_RUNTIME_VERSION MATCHES "^[0-9]+\\.[0-9]+\\.[0-9]+$")
                message(FATAL_ERROR
                    "LW production deployment requires a valid ONNX Runtime "
                    "VERSION_NUMBER")
            endif()
            execute_process(
                COMMAND
                    "${Python3_EXECUTABLE}"
                    "${PROJECT_ROOT_DIR}/scripts/manage_inference_runtime.py"
                    check
                    --catalog
                    "${PROJECT_ROOT_DIR}/scripts/inference_runtime_archives.json"
                    --kind onnx
                    --version "${ONNX_RUNTIME_VERSION}"
                    --os Linux
                    --architecture "${RUNTIME_VALIDATION_ARCH}"
                    --runtime-root "${INFERENCE_RUNTIME_DIR}"
                    --validator
                    "${PROJECT_ROOT_DIR}/scripts/validate_inference_runtime.sh"
                RESULT_VARIABLE LW_ONNX_PROVENANCE_RESULT
                OUTPUT_VARIABLE LW_ONNX_PROVENANCE_OUTPUT
                ERROR_VARIABLE LW_ONNX_PROVENANCE_ERROR
            )
            if(NOT LW_ONNX_PROVENANCE_RESULT EQUAL 0)
                message(FATAL_ERROR
                    "LW production deployment requires an approved ONNX "
                    "Runtime archive provenance: "
                    "${LW_ONNX_PROVENANCE_OUTPUT}${LW_ONNX_PROVENANCE_ERROR}")
            endif()
            set(
                LW_ONNX_RUNTIME_ORIGIN_FILE
                "${ONNX_RUNTIME_DIR}/.rl_sar_runtime_origin.json"
            )
            get_filename_component(
                LW_ONNX_RUNTIME_REAL_LIBRARY
                "${ONNX_RUNTIME_LIB_PATH}"
                REALPATH
            )
            set(
                LW_ONNX_RUNTIME_PROVIDER_LIBRARY
                "${ONNX_RUNTIME_DIR}/lib/libonnxruntime_providers_shared.so"
            )
            if(NOT EXISTS "${LW_ONNX_RUNTIME_REAL_LIBRARY}"
               OR IS_SYMLINK "${LW_ONNX_RUNTIME_REAL_LIBRARY}")
                message(FATAL_ERROR
                    "Resolved ONNX Runtime library must be a regular file: "
                    "${LW_ONNX_RUNTIME_REAL_LIBRARY}")
            endif()
            if(NOT EXISTS "${LW_ONNX_RUNTIME_PROVIDER_LIBRARY}"
               OR IS_SYMLINK "${LW_ONNX_RUNTIME_PROVIDER_LIBRARY}")
                message(FATAL_ERROR
                    "ONNX Runtime provider library must be a regular file: "
                    "${LW_ONNX_RUNTIME_PROVIDER_LIBRARY}")
            endif()
        endif()
    else()
        if(IS_JETSON OR LW_PRODUCTION_DEPLOYMENT)
            message(FATAL_ERROR "ONNX Runtime library not found")
        else()
            message(WARNING "ONNX Runtime library not found")
        endif()
    endif()
else()
    if(IS_JETSON OR LW_PRODUCTION_DEPLOYMENT)
        message(FATAL_ERROR
            "ONNX Runtime is required for Jetson and LW production builds. "
            "Run: scripts/download_inference_runtime.sh onnx")
    else()
        message(WARNING
            "ONNX Runtime not found. Please run: "
            "scripts/download_inference_runtime.sh onnx")
    endif()
endif()

# RPATH Configuration for inference libraries
if(USE_ONNX)
    set(CMAKE_SKIP_BUILD_RPATH FALSE)
    set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)

    set(PREBUILT_LIB_PATHS "")
    list(APPEND PREBUILT_LIB_PATHS "${ONNX_RUNTIME_DIR}/lib")
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
    add_executable(test_lw_joinable_worker
        test/test_lw_joinable_worker.cpp
    )
    target_include_directories(test_lw_joinable_worker PRIVATE
        library/core/simulation
    )
    target_link_libraries(test_lw_joinable_worker PRIVATE Threads::Threads)
    add_test(NAME lw_joinable_worker COMMAND test_lw_joinable_worker)
    if(SYSTEM_TYPE STREQUAL "Linux")
        add_executable(test_lw_signal_shutdown
            test/test_lw_signal_shutdown.cpp
        )
        target_include_directories(test_lw_signal_shutdown PRIVATE
            library/core/simulation
        )
        target_link_libraries(test_lw_signal_shutdown PRIVATE Threads::Threads)
        add_test(NAME lw_signal_shutdown COMMAND test_lw_signal_shutdown)
        add_executable(test_lw_ros_shutdown
            test/test_lw_ros_shutdown.cpp
        )
        target_include_directories(test_lw_ros_shutdown PRIVATE
            library/core/simulation
        )
        target_link_libraries(test_lw_ros_shutdown Threads::Threads)
        ament_target_dependencies(test_lw_ros_shutdown rclcpp)
        add_test(
            NAME lw_ros_shutdown_normal
            COMMAND test_lw_ros_shutdown normal
        )
        add_test(
            NAME lw_ros_shutdown_sigterm
            COMMAND test_lw_ros_shutdown sigterm
        )
        set_tests_properties(
            lw_ros_shutdown_normal
            lw_ros_shutdown_sigterm
            PROPERTIES TIMEOUT 10
        )
    endif()
    add_test(
        NAME lw_sim_lifecycle_integration
        COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/test/test_lw_sim_lifecycle_integration.py
    )

    add_executable(test_lw_sim_plot_config
        test/test_lw_sim_plot_config.cpp
    )
    target_include_directories(test_lw_sim_plot_config PRIVATE
        library/core/simulation
    )
    add_test(NAME lw_sim_plot_config COMMAND test_lw_sim_plot_config)

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

    add_executable(test_lw_imu_ahrs_guard
        test/test_lw_imu_ahrs_guard.cpp
    )
    target_include_directories(test_lw_imu_ahrs_guard PRIVATE
        library/core/safety
    )
    add_test(NAME lw_imu_ahrs_guard COMMAND test_lw_imu_ahrs_guard)

    if(SYSTEM_TYPE STREQUAL "Linux")
        add_executable(test_lw_serial_sdk
            test/test_lw_serial_sdk.cpp
        )
        target_include_directories(test_lw_serial_sdk PRIVATE
            library/thirdparty/robot_sdk/lfr/LW_sdk
        )
        add_test(NAME lw_serial_sdk COMMAND test_lw_serial_sdk)

        add_executable(test_lw_startup_disable
            test/test_lw_startup_disable.cpp
        )
        target_include_directories(test_lw_startup_disable PRIVATE
            library/core/loop
            library/core/safety
            library/thirdparty/robot_sdk/lfr/LW_sdk
        )
        target_link_libraries(test_lw_startup_disable
            Threads::Threads
        )
        add_test(NAME lw_startup_disable COMMAND test_lw_startup_disable)
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

endif()

# LW_sdk
add_library(LW_sdk INTERFACE)
target_include_directories(LW_sdk SYSTEM INTERFACE
    library/thirdparty/robot_sdk/lfr/LW_sdk
)
target_link_libraries(LW_sdk INTERFACE Threads::Threads)

if(SYSTEM_TYPE STREQUAL "Linux")
    add_library(
        lw_joystick_vendor
        library/thirdparty/joystick/joystick.cc
    )
    target_include_directories(lw_joystick_vendor SYSTEM PUBLIC
        library/thirdparty/joystick
    )
    lw_silence_vendor_warnings(lw_joystick_vendor)
endif()

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
    library/core/simulation
    fsm_robot
)

# library for inference runtime
add_library(inference_runtime library/core/inference_runtime/inference_runtime.cpp)
if(USE_ONNX)
    target_link_libraries(inference_runtime PUBLIC ${ONNX_RUNTIME_LIB})
    target_compile_definitions(inference_runtime PRIVATE USE_ONNX)
endif()

add_library(
    lw_sim_torque_validation
    library/core/simulation/lw_sim_torque_validation.cpp
)
target_include_directories(lw_sim_torque_validation PUBLIC
    library/core/simulation
)
install(TARGETS inference_runtime DESTINATION lib/${PROJECT_NAME})

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
        add_library(
            lw_deployment_bundle_test_support
            library/core/deployment/lw_deployment_bundle.cpp
        )
        target_include_directories(
            lw_deployment_bundle_test_support
            PUBLIC library/core/deployment
        )
        target_link_libraries(
            lw_deployment_bundle_test_support
            PUBLIC ${YAML_CPP_LIBRARIES} OpenSSL::Crypto
        )
        if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
            set(LW_TEST_ONNX_MAIN_SHA256
                "d2d4b1f4378d317a0b2f109a57766ec7713ef4ac160d14d1614467d624a8943c")
            set(LW_TEST_ONNX_PROVIDER_SHA256
                "0f9923a1343635e15bf5fc36f3d8687cdcc0f83e5b4e318c0aeb5a0852588525")
        else()
            set(LW_TEST_ONNX_MAIN_SHA256
                "8b27c6b3fd950a70ff1374e817a9467f4be5868981555b2df8454c15c2bbbfdc")
            set(LW_TEST_ONNX_PROVIDER_SHA256
                "6d5d7742974c2e3558e5c3092d881aaecaa78af8003e32539c77c5acae45ba43")
        endif()
        target_compile_definitions(
            lw_deployment_bundle_test_support
            PRIVATE
                "RL_SAR_TEST_APPROVED_ONNX_MAIN_SHA256=\"${LW_TEST_ONNX_MAIN_SHA256}\""
                "RL_SAR_TEST_APPROVED_ONNX_PROVIDER_SHA256=\"${LW_TEST_ONNX_PROVIDER_SHA256}\""
        )
        add_executable(
            test_lw_deployment_bundle
            test/test_lw_deployment_bundle.cpp
        )
        target_link_libraries(
            test_lw_deployment_bundle
            PRIVATE lw_deployment_bundle_test_support
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
                    ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
                    ${PROJECT_ROOT_DIR}/src/fdilink_ahrs_ROS2/package.xml
        )
        add_test(
            NAME lw_policy_asset_parity
            COMMAND ${Python3_EXECUTABLE}
                    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_verify_lw_policy_parity.py
                    ${CMAKE_CURRENT_SOURCE_DIR}/scripts/verify_lw_policy_parity.py
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
                    ${PROJECT_ROOT_DIR}/README.md
                    ${PROJECT_ROOT_DIR}/docs/LW_BUILD_DEPLOYMENT_CN.md
                    ${PROJECT_ROOT_DIR}/docs/LW_QUICK_START_CN.md
                    ${PROJECT_ROOT_DIR}/scripts/install_build_dependencies.sh
                    ${PROJECT_ROOT_DIR}/scripts/download_inference_runtime.sh
                    ${PROJECT_ROOT_DIR}/scripts/install_pytorch_jetson.sh
                    ${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt
                    ${CMAKE_CURRENT_SOURCE_DIR}/scripts/build_lw_deployment.sh
                    ${PROJECT_ROOT_DIR}/scripts/validate_lw_strict_build.sh
        )
        add_test(
            NAME lw_inference_runtime_architecture
            COMMAND ${Python3_EXECUTABLE}
                    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_inference_runtime_architecture.py
                    ${PROJECT_ROOT_DIR}/scripts/validate_inference_runtime.sh
        )
        add_test(
            NAME lw_inference_runtime_download_integrity
            COMMAND ${Python3_EXECUTABLE}
                    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_inference_runtime_download_integrity.py
                    ${PROJECT_ROOT_DIR}/scripts/manage_inference_runtime.py
                    ${PROJECT_ROOT_DIR}/scripts/validate_inference_runtime.sh
                    ${PROJECT_ROOT_DIR}/scripts/inference_runtime_archives.json
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
                    ${CMAKE_CURRENT_SOURCE_DIR}/library/core/safety/lw_runtime_core.hpp
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

    add_executable(test_lw_runtime_parity
        test/test_lw_runtime_parity.cpp
    )
    target_link_libraries(test_lw_runtime_parity PRIVATE
        rl_sdk
        observation_buffer
        ${YAML_CPP_LIBRARIES}
    )
    add_test(NAME lw_runtime_parity COMMAND test_lw_runtime_parity)

    add_executable(test_lw_allocation_bound
        test/test_lw_allocation_bound.cpp
    )
    target_link_libraries(test_lw_allocation_bound PRIVATE
        rl_sdk
        observation_buffer
        ${YAML_CPP_LIBRARIES}
    )
    add_test(NAME lw_allocation_bound COMMAND test_lw_allocation_bound)

    add_executable(test_lw_rl_destruction
        test/test_lw_rl_destruction.cpp
    )
    target_link_libraries(test_lw_rl_destruction PRIVATE
        rl_sdk
        observation_buffer
        ${YAML_CPP_LIBRARIES}
    )
    add_test(NAME lw_rl_destruction COMMAND test_lw_rl_destruction)

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
    if(TARGET lw_joystick_vendor)
        target_link_libraries(
            test_lw_joystick_safety
            PRIVATE lw_joystick_vendor
        )
    endif()
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

    if(USE_ONNX)
        add_executable(
            test_inference_runtime
            test/test_inference_runtime.cpp
        )
        target_link_libraries(
            test_inference_runtime
            PRIVATE inference_runtime
        )
        add_test(
            NAME inference_runtime_contract
            COMMAND test_inference_runtime
        )
    endif()

    add_executable(
        test_lw_sim_torque_validation
        test/test_lw_sim_torque_validation.cpp
    )
    target_link_libraries(
        test_lw_sim_torque_validation
        PRIVATE lw_sim_torque_validation
    )
    add_test(
        NAME lw_sim_torque_validation
        COMMAND test_lw_sim_torque_validation
    )

    add_executable(
        test_lw_config_profile
        test/test_lw_config_profile.cpp
    )
    target_link_libraries(
        test_lw_config_profile
        PRIVATE Threads::Threads
    )
    add_test(NAME lw_config_profile COMMAND test_lw_config_profile)

    add_test(
        NAME lw_runtime_config_analyzer
        COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/test/test_profile_lw_runtime_config.py
    )

    add_test(
        NAME lw_repository_scope
        COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/test/test_lw_repository_scope.py
                ${PROJECT_ROOT_DIR}
    )
endif()
install(TARGETS rl_sdk DESTINATION lib/${PROJECT_NAME})

# library for observation buffer
add_library(observation_buffer library/core/observation_buffer/observation_buffer.cpp)
install(TARGETS observation_buffer DESTINATION lib/${PROJECT_NAME})

# executable for rl sim LW
if(SYSTEM_TYPE STREQUAL "Linux"
   AND NOT IS_JETSON
   AND NOT LW_PRODUCTION_DEPLOYMENT)
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

        # Source files
        file(GLOB MUJOCO_SIMULATE_SRC library/thirdparty/mujoco_simulate/*.cc)
        add_library(
            lw_mujoco_simulate_vendor
            ${MUJOCO_SIMULATE_SRC}
        )
        target_include_directories(lw_mujoco_simulate_vendor SYSTEM PUBLIC
            library/thirdparty/mujoco_simulate
        )
        target_link_libraries(lw_mujoco_simulate_vendor PUBLIC
            Threads::Threads mujoco_prebuilt glfw
        )
        lw_silence_vendor_warnings(lw_mujoco_simulate_vendor)

        # Build executable
        add_executable(rl_sim_LW src/rl_sim_LW.cpp)

        target_link_libraries(rl_sim_LW
            rl_sdk observation_buffer lw_sim_torque_validation
            ${YAML_CPP_LIBRARIES} LW_sdk
            Threads::Threads mujoco_prebuilt glfw
            lw_mujoco_simulate_vendor lw_joystick_vendor
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

            add_executable(
                test_lw_mujoco_synchronization
                test/test_lw_mujoco_synchronization.cpp
            )
            target_compile_definitions(
                test_lw_mujoco_synchronization
                PRIVATE
                    "LW_TEST_SCENE=\"${PROJECT_ROOT_DIR}/src/rl_sar_zoo/LW_description/mjcf/scene.xml\""
            )
            target_link_libraries(
                test_lw_mujoco_synchronization
                PRIVATE mujoco_prebuilt Threads::Threads
            )
            add_test(
                NAME lw_mujoco_synchronization
                COMMAND test_lw_mujoco_synchronization
            )

            add_executable(
                test_lw_mujoco_lifecycle
                test/test_lw_mujoco_lifecycle.cpp
            )
            target_compile_definitions(
                test_lw_mujoco_lifecycle
                PRIVATE
                    "LW_TEST_SCENE=\"${CMAKE_CURRENT_SOURCE_DIR}/test/data/lw024_minimal.xml\""
            )
            target_include_directories(
                test_lw_mujoco_lifecycle
                PRIVATE
                    library/core/simulation
            )
            target_link_libraries(
                test_lw_mujoco_lifecycle
                PRIVATE
                    lw_mujoco_simulate_vendor
                    mujoco_prebuilt
                    Threads::Threads
            )
            add_test(
                NAME lw_mujoco_lifecycle
                COMMAND test_lw_mujoco_lifecycle
            )
        endif()
        ament_target_dependencies(rl_sim_LW
                rclcpp
                geometry_msgs
                sensor_msgs
        )
        install(TARGETS rl_sim_LW DESTINATION lib/${PROJECT_NAME})
        message(STATUS "MuJoCo configured: ${MUJOCO_DIR}")
endif()

# executable for rl real LW
if(SYSTEM_TYPE STREQUAL "Linux")
    add_executable(rl_real_LW
            src/rl_real_LW.cpp
        )
        add_executable(
            lw_config_profiler
            src/lw_config_profiler.cpp
        )
        target_link_libraries(rl_real_LW
            LW_sdk
            lw_joystick_vendor
            lw_debug_publisher
            rl_sdk_lw_deployment
            lw_deployment_bundle
            observation_buffer
            ${YAML_CPP_LIBRARIES}
            Threads::Threads
        )
        target_link_libraries(lw_config_profiler
            LW_sdk
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
        ament_target_dependencies(lw_config_profiler
            rclcpp
            sensor_msgs
        )
        target_compile_definitions(
            rl_real_LW
            PRIVATE RL_SAR_SOURCE_COMMIT="${RL_SAR_SOURCE_COMMIT}"
        )
        target_compile_definitions(
            lw_config_profiler
            PRIVATE RL_SAR_SOURCE_COMMIT="${RL_SAR_SOURCE_COMMIT}"
        )
        install(
            TARGETS rl_real_LW lw_config_profiler
            DESTINATION lib/${PROJECT_NAME}
        )
        if(LW_PRODUCTION_DEPLOYMENT)
            set(LW_ONNX_RUNTIME_INSTALL_DIRECTORY
                "lib/${PROJECT_NAME}/onnxruntime")
            set_target_properties(
                rl_real_LW lw_config_profiler
                PROPERTIES
                    BUILD_WITH_INSTALL_RPATH FALSE
                    BUILD_RPATH "${ONNX_RUNTIME_DIR}/lib"
                    INSTALL_RPATH "$ORIGIN/onnxruntime"
                    INSTALL_RPATH_USE_LINK_PATH FALSE
            )
            install(
                FILES "${LW_ONNX_RUNTIME_REAL_LIBRARY}"
                DESTINATION "${LW_ONNX_RUNTIME_INSTALL_DIRECTORY}"
                RENAME "libonnxruntime.so.1"
            )
            install(
                FILES "${LW_ONNX_RUNTIME_PROVIDER_LIBRARY}"
                DESTINATION "${LW_ONNX_RUNTIME_INSTALL_DIRECTORY}"
            )
            install(
                FILES "${LW_ONNX_RUNTIME_ORIGIN_FILE}"
                DESTINATION "${LW_ONNX_RUNTIME_INSTALL_DIRECTORY}"
                RENAME "origin.json"
            )
        endif()
        if(BUILD_TESTING)
            add_test(NAME lw_config_profiler_help COMMAND lw_config_profiler --help)
            add_test(
                NAME lw_real_startup_disable_integration
                COMMAND ${Python3_EXECUTABLE}
                        ${CMAKE_CURRENT_SOURCE_DIR}/test/test_lw_real_startup_disable_integration.py
            )
            add_test(
                NAME lw_config_profiler_integration
                COMMAND ${Python3_EXECUTABLE}
                        ${CMAKE_CURRENT_SOURCE_DIR}/test/test_lw_config_profiler_integration.py
                        --profiler $<TARGET_FILE:lw_config_profiler>
                        --policy-root ${PROJECT_ROOT_DIR}/policy
            )
        endif()
endif()

# executable for actuator net
install(PROGRAMS
    scripts/actuator_net.py
    scripts/profile_lw_runtime_config.py
    DESTINATION lib/${PROJECT_NAME}
)

if(LW_PRODUCTION_DEPLOYMENT)
        install(
            FILES launch/rl_real_LW.launch.py
            DESTINATION share/${PROJECT_NAME}/launch
        )
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
            DIRECTORY launch
            DESTINATION share/${PROJECT_NAME}/${dir}
        )
        install(
            DIRECTORY ${PROJECT_ROOT_DIR}/policy
            DESTINATION share/${PROJECT_NAME}/${dir}
        )
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
message(STATUS "----------------------------------------")
message(STATUS "Backend Configuration:")
message(STATUS "USE_ONNX: ${USE_ONNX}")
if(USE_ONNX)
    if(ONNX_RUNTIME_VERSION)
        message(STATUS "ONNX Runtime version: ${ONNX_RUNTIME_VERSION}")
    elseif(onnxruntime_VERSION)
        message(STATUS "ONNX Runtime version: ${onnxruntime_VERSION}")
    else()
        message(STATUS "ONNX Runtime: Available")
    endif()
endif()
message(STATUS "========================================")
