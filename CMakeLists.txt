cmake_minimum_required(VERSION 3.20)
project(DroneSwarmSensorFusion VERSION 2.0.0 LANGUAGES CXX)

# ─────────────────────────────────────────────────────────────────────────────
# C++20 Standard – required for concepts, ranges, and coroutines
# ─────────────────────────────────────────────────────────────────────────────
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

# ─────────────────────────────────────────────────────────────────────────────
# Compiler Flags
# ─────────────────────────────────────────────────────────────────────────────
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

# ─────────────────────────────────────────────────────────────────────────────
# CMake Module Path
# ─────────────────────────────────────────────────────────────────────────────
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# ─────────────────────────────────────────────────────────────────────────────
# External Dependencies
# ─────────────────────────────────────────────────────────────────────────────

# 1. Eigen3 — linear algebra for EKF/VIO
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
message(STATUS "Eigen3 found: ${EIGEN3_VERSION}")

# 2. OpenCV — vision, feature extraction, RTSP
find_package(OpenCV 4.5 REQUIRED
    COMPONENTS core imgproc highgui calib3d features2d video dnn)
message(STATUS "OpenCV found: ${OpenCV_VERSION}")

# 3. PCL — point cloud processing for LiDAR SLAM
find_package(PCL 1.12 REQUIRED
    COMPONENTS common io filters segmentation visualization kdtree search)
message(STATUS "PCL found: ${PCL_VERSION}")

# 4. Fast-DDS (eProsima) — V2X mesh communication
find_package(fastrtps QUIET)
if(fastrtps_FOUND)
    message(STATUS "Fast-DDS found: ${fastrtps_VERSION}")
    set(HAS_FASTDDS TRUE)
else()
    message(WARNING "Fast-DDS not found – V2X features will be disabled")
    set(HAS_FASTDDS FALSE)
endif()

# 5. spdlog — structured logging
find_package(spdlog REQUIRED)
message(STATUS "spdlog found")

# 6. pybind11 — Python/C++ GUI bridge
find_package(pybind11 QUIET)
if(pybind11_FOUND)
    message(STATUS "pybind11 found: ${pybind11_VERSION}")
    set(HAS_PYBIND TRUE)
else()
    message(WARNING "pybind11 not found – Python bindings disabled")
    set(HAS_PYBIND FALSE)
endif()

# 7. TensorRT (optional, Jetson Nano / NVIDIA platform)
find_library(TENSORRT_LIB nvinfer HINTS /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu)
find_path(TENSORRT_INCLUDE NvInfer.h HINTS /usr/include/aarch64-linux-gnu /usr/local/include)
if(TENSORRT_LIB AND TENSORRT_INCLUDE)
    message(STATUS "TensorRT found: ${TENSORRT_LIB}")
    set(HAS_TENSORRT TRUE)
else()
    message(WARNING "TensorRT not found – YOLOv8n inference will use OpenCV DNN fallback")
    set(HAS_TENSORRT FALSE)
endif()

# 8. Threads
find_package(Threads REQUIRED)

# ─────────────────────────────────────────────────────────────────────────────
# Global Include Directories
# ─────────────────────────────────────────────────────────────────────────────
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

if(HAS_TENSORRT)
    include_directories(${TENSORRT_INCLUDE})
endif()

# ─────────────────────────────────────────────────────────────────────────────
# Preprocessor Definitions
# ─────────────────────────────────────────────────────────────────────────────
add_compile_definitions(
    $<$<BOOL:${HAS_FASTDDS}>:DRONE_HAS_FASTDDS>
    $<$<BOOL:${HAS_TENSORRT}>:DRONE_HAS_TENSORRT>
    DRONE_LOG_LEVEL=1
)

# ─────────────────────────────────────────────────────────────────────────────
# Library: sensor_fusion_core
# ─────────────────────────────────────────────────────────────────────────────
set(CORE_SOURCES
    src/autonomy/DecisionEngine.cpp
    src/sensors/IMUSensor.cpp
    src/sensors/CameraSensor.cpp
    src/sensors/LidarSensor.cpp
    src/vio/EKFEstimator.cpp
    src/vio/VIOPipeline.cpp
    src/slam/KeyframeManager.cpp
    src/hal/JetsonHAL.cpp
)

if(HAS_FASTDDS)
    list(APPEND CORE_SOURCES
        src/swarm/V2XMeshNetwork.cpp
    )
endif()

add_library(sensor_fusion_core SHARED ${CORE_SOURCES})

target_link_libraries(sensor_fusion_core PUBLIC
    Eigen3::Eigen
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    Threads::Threads
    spdlog::spdlog
    $<$<BOOL:${HAS_FASTDDS}>:fastrtps>
    $<$<BOOL:${HAS_TENSORRT}>:${TENSORRT_LIB}>
)

target_compile_definitions(sensor_fusion_core PRIVATE
    ${PCL_DEFINITIONS}
)

# ─────────────────────────────────────────────────────────────────────────────
# Executable: drone_node  (main entry point per drone)
# ─────────────────────────────────────────────────────────────────────────────
add_executable(drone_node src/main.cpp)
target_link_libraries(drone_node PRIVATE sensor_fusion_core)

# ─────────────────────────────────────────────────────────────────────────────
# Pybind11 Module: drone_bridge  (Python GUI ↔ C++ core)
# ─────────────────────────────────────────────────────────────────────────────
if(HAS_PYBIND)
    pybind11_add_module(drone_bridge src/drone_bridge.cpp)
    target_link_libraries(drone_bridge PRIVATE sensor_fusion_core)
    message(STATUS "drone_bridge Python module: ENABLED")
endif()

# ─────────────────────────────────────────────────────────────────────────────
# Tests
# ─────────────────────────────────────────────────────────────────────────────
option(BUILD_TESTS "Build unit tests" ON)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# ─────────────────────────────────────────────────────────────────────────────
# Install Rules
# ─────────────────────────────────────────────────────────────────────────────
install(TARGETS drone_node sensor_fusion_core
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)

# ─────────────────────────────────────────────────────────────────────────────
# Summary
# ─────────────────────────────────────────────────────────────────────────────
message(STATUS "")
message(STATUS "╔══════════════════════════════════════════════╗")
message(STATUS "║   Drone Swarm Sensor Fusion — Build Config   ║")
message(STATUS "╠══════════════════════════════════════════════╣")
message(STATUS "║  Fast-DDS (V2X)  : ${HAS_FASTDDS}")
message(STATUS "║  TensorRT        : ${HAS_TENSORRT}")
message(STATUS "║  pybind11 GUI    : ${HAS_PYBIND}")
message(STATUS "║  Build Type      : ${CMAKE_BUILD_TYPE}")
message(STATUS "╚══════════════════════════════════════════════╝")
message(STATUS "")
