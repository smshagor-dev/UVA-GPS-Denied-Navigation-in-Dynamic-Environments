# System Designer and Developer: Md Shahanur Islam Shagor
# Project: UVA GPS Denied Navigation in Dynamic Environments
# Technology: C++, Python, Go, CMake

cmake_minimum_required(VERSION 3.20)

if(WIN32 AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(_DRONE_VCPKG_TOOLCHAIN "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
    if(EXISTS "${_DRONE_VCPKG_TOOLCHAIN}")
        set(CMAKE_TOOLCHAIN_FILE "${_DRONE_VCPKG_TOOLCHAIN}" CACHE STRING "vcpkg toolchain file")
    elseif(EXISTS "C:/tools/vcpkg-full/scripts/buildsystems/vcpkg.cmake")
        set(CMAKE_TOOLCHAIN_FILE "C:/tools/vcpkg-full/scripts/buildsystems/vcpkg.cmake" CACHE STRING "vcpkg toolchain file")
    endif()
endif()

project(DroneSwarmSensorFusion VERSION 2.0.0 LANGUAGES C CXX)

# 
# C++20 Standard â€“ required for concepts, ranges, and coroutines
# 
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Build type defaults
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

message(STATUS "Build type : ${CMAKE_BUILD_TYPE}")
message(STATUS "Compiler   : ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")

# 
# Compiler Flags
# 
if(MSVC)
    add_compile_options(
        /W4
        /EHsc
        /permissive-
        /wd4100
        /utf-8
        $<$<CONFIG:Release>:/O2>
        $<$<CONFIG:Release>:/DNDEBUG>
        $<$<CONFIG:RelWithDebInfo>:/O2>
        $<$<CONFIG:RelWithDebInfo>:/Zi>
        $<$<CONFIG:Debug>:/Od>
        $<$<CONFIG:Debug>:/Zi>
    )
else()
    add_compile_options(
        -Wall -Wextra -Wpedantic
        -Wno-unused-parameter
        $<$<CONFIG:Release>:-O3 -march=native -DNDEBUG>
        $<$<CONFIG:Debug>:-g3 -fsanitize=address,undefined>
        $<$<CONFIG:RelWithDebInfo>:-O2 -g>
    )

    add_link_options(
        $<$<CONFIG:Debug>:-fsanitize=address,undefined>
    )
endif()

# 
# CMake Module Path
# 
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

if(WIN32)
    foreach(_drone_prefix
        "$ENV{VCPKG_ROOT}/installed/x64-windows"
        "C:/tools/vcpkg-full/installed/x64-windows"
    )
        if(EXISTS "${_drone_prefix}")
            list(APPEND CMAKE_PREFIX_PATH "${_drone_prefix}" "${_drone_prefix}/share")
        endif()
    endforeach()
endif()

# 
# External Dependencies
# 

# 1. Eigen3  linear algebra for EKF/VIO
find_package(Eigen3 3.4...6 REQUIRED NO_MODULE)
message(STATUS "Eigen3 found: ${EIGEN3_VERSION}")

# 2. OpenCV  vision, feature extraction, RTSP
find_package(OpenCV 4.5 REQUIRED
    COMPONENTS core imgproc highgui calib3d features2d video dnn)
message(STATUS "OpenCV found: ${OpenCV_VERSION}")

# 3. PCL  point cloud processing for LiDAR SLAM
find_package(PCL 1.12 REQUIRED
    COMPONENTS common io filters segmentation kdtree search)
message(STATUS "PCL found: ${PCL_VERSION}")

# 4. Fast-DDS (eProsima)  V2X mesh communication
find_package(nlohmann_json QUIET CONFIG)
find_package(fastdds QUIET CONFIG)
find_package(fastrtps QUIET CONFIG)
if(fastdds_FOUND OR fastrtps_FOUND)
    message(STATUS "Fast-DDS transport found")
    set(HAS_FASTDDS_TRANSPORT TRUE)
else()
    message(STATUS "Fast-DDS transport not found - built-in UDP swarm transport will be used")
    set(HAS_FASTDDS_TRANSPORT FALSE)
endif()

# 5. spdlog  structured logging
find_package(spdlog REQUIRED)
message(STATUS "spdlog found")

# 6. pybind11  Python/C++ GUI bridge
find_package(pybind11 QUIET)
if(pybind11_FOUND)
    message(STATUS "pybind11 found: ${pybind11_VERSION}")
    set(HAS_PYBIND TRUE)
else()
    include(FetchContent)
    FetchContent_Declare(
        pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11.git
        GIT_TAG        v2.13.6
        GIT_SHALLOW    TRUE
    )
    FetchContent_MakeAvailable(pybind11)
    if(TARGET pybind11::module)
        message(STATUS "pybind11 fetched: ${pybind11_VERSION}")
        set(HAS_PYBIND TRUE)
    else()
        message(STATUS "pybind11 unavailable - Python bindings disabled")
        set(HAS_PYBIND FALSE)
    endif()
endif()

# 7. TensorRT (optional, Jetson Nano / NVIDIA platform)
find_library(TENSORRT_LIB nvinfer
    HINTS
        $ENV{TENSORRT_ROOT}/lib
        $ENV{TENSORRT_ROOT}/lib/x64
        /usr/lib/aarch64-linux-gnu
        /usr/lib/x86_64-linux-gnu)
find_path(TENSORRT_INCLUDE NvInfer.h
    HINTS
        $ENV{TENSORRT_ROOT}/include
        /usr/include/aarch64-linux-gnu
        /usr/local/include)
if(TENSORRT_LIB AND TENSORRT_INCLUDE)
    message(STATUS "TensorRT found: ${TENSORRT_LIB}")
    set(HAS_TENSORRT TRUE)
else()
    message(STATUS "TensorRT not found - YOLOv8n inference will use OpenCV DNN fallback")
    set(HAS_TENSORRT FALSE)
endif()

# 8. Threads
find_package(Threads REQUIRED)

# 
# Global Include Directories
# 
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/crypto
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

if(HAS_TENSORRT)
    include_directories(${TENSORRT_INCLUDE})
endif()

# 
# Preprocessor Definitions
# 
add_compile_definitions(
    $<$<BOOL:${HAS_FASTDDS_TRANSPORT}>:DRONE_HAS_FASTDDS_TRANSPORT>
    $<$<BOOL:${HAS_TENSORRT}>:DRONE_HAS_TENSORRT>
    DRONE_LOG_LEVEL=1
)

# 
# Library: sensor_fusion_core
# 
set(CORE_SOURCES
    src/autonomy/DecisionEngine.cpp
    src/autonomy/ExperienceMemory.cpp
    src/localization/LocalizationFusion.cpp
    src/localization/TDOAIngestor.cpp
    src/localization/TDOALocalizer.cpp
    src/localization/TimeSyncTracker.cpp
    src/localization/UWBSerialDriver.cpp
    src/swarm/SwarmSecurity.cpp
    src/sensors/BarometerSensor.cpp
    src/sensors/IMUSensor.cpp
    src/sensors/OpticalFlowSensor.cpp
    src/sensors/RangefinderSensor.cpp
    src/sensors/CameraSensor.cpp
    src/sensors/LidarSensor.cpp
    src/sensors/MotorSensor.cpp
    src/telemetry/ControlPlaneTelemetryClient.cpp
    src/vio/EKFEstimator.cpp
    src/vio/VIOPipeline.cpp
    src/slam/KeyframeManager.cpp
    src/slam/MapPlanner.cpp
    src/slam/OccupancyGridMap.cpp
    src/hal/JetsonHAL.cpp
    src/swarm/V2XMeshNetwork.cpp
    third_party/crypto/monocypher.c
    third_party/crypto/sha3.c
)

add_library(sensor_fusion_core STATIC ${CORE_SOURCES})

target_link_libraries(sensor_fusion_core PUBLIC
    Eigen3::Eigen
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    Threads::Threads
    spdlog::spdlog
    $<$<BOOL:${fastdds_FOUND}>:fastdds>
    $<$<BOOL:${fastrtps_FOUND}>:fastrtps>
    $<$<BOOL:${HAS_TENSORRT}>:${TENSORRT_LIB}>
    $<$<BOOL:${MSVC}>:bcrypt>
    $<$<BOOL:${WIN32}>:crypt32>
    $<$<BOOL:${WIN32}>:ncrypt>
    $<$<BOOL:${WIN32}>:Ws2_32>
)

target_compile_definitions(sensor_fusion_core PRIVATE
    ${PCL_DEFINITIONS}
)

# 
# Executable: drone_node  (main entry point per drone)
# 
add_executable(drone_node src/main.cpp)
target_link_libraries(drone_node PRIVATE sensor_fusion_core)

# 
# Pybind11 Module: drone_bridge  (Python GUI â†” C++ core)
# 
if(HAS_PYBIND)
    pybind11_add_module(drone_bridge src/drone_bridge.cpp)
    target_link_libraries(drone_bridge PRIVATE sensor_fusion_core)
    message(STATUS "drone_bridge Python module: ENABLED")
endif()

# 
# Tests
# 
option(BUILD_TESTS "Build unit tests" ON)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# 
# Install Rules
# 
install(TARGETS drone_node sensor_fusion_core
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)

# 
# Summary
# 
message(STATUS "")
message(STATUS "â•”â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•—")
message(STATUS "â•‘   Drone Swarm Sensor Fusion  Build Config   â•‘")
message(STATUS "â• â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•£")
message(STATUS "â•‘  Fast-DDS native : ${HAS_FASTDDS_TRANSPORT}")
message(STATUS "â•‘  TensorRT        : ${HAS_TENSORRT}")
message(STATUS "â•‘  pybind11 GUI    : ${HAS_PYBIND}")
message(STATUS "â•‘  Build Type      : ${CMAKE_BUILD_TYPE}")
message(STATUS "â•šâ•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•")
message(STATUS "")
# System Designer and Developer: Md Shahanur Islam Shagor
# Project: UVA GPS Denied Navigation in Dynamic Environments
# Technology: C++, Python, Go, CMake
# System Designer and Developer: Md Shahanur Islam Shagor
# Project: UVA GPS Denied Navigation in Dynamic Environments
# Technology: C++, Python, Go, CMake
