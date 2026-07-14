# System Designer and Developer: Md Shahanur Islam Shagor
# Project: UVA GPS Denied Navigation in Dynamic Environments
# Technology: C++, Python, Go, CMake

cmake_minimum_required(VERSION 3.24)

if(POLICY CMP0144)
    cmake_policy(SET CMP0144 NEW)
endif()

if(WIN32 AND NOT DEFINED CMAKE_TOOLCHAIN_FILE AND DEFINED ENV{VCPKG_ROOT})
    file(TO_CMAKE_PATH "$ENV{VCPKG_ROOT}" _drone_vcpkg_root)
    if(EXISTS "${_drone_vcpkg_root}/scripts/buildsystems/vcpkg.cmake")
        set(CMAKE_TOOLCHAIN_FILE
            "${_drone_vcpkg_root}/scripts/buildsystems/vcpkg.cmake"
            CACHE STRING "vcpkg toolchain file")
    endif()
endif()

project(DroneSwarmSensorFusion VERSION 2.0.0 LANGUAGES C CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(CompilerWarnings)
include(Sanitizers)
include(Dependencies)

option(DRONE_BUILD_TESTS "Build native tests" ON)
option(DRONE_ENABLE_FASTDDS "Enable Fast-DDS transport support when available" ON)
option(DRONE_ENABLE_TENSORRT "Enable TensorRT inference support when available" OFF)
option(DRONE_ENABLE_PYTHON_BINDINGS "Build the pybind11 bridge module" ON)
option(DRONE_FETCH_PYBIND11 "Fetch pybind11 when no package is available" ON)
option(DRONE_FETCH_GOOGLETEST "Fetch GoogleTest when no package is available" ON)
option(DRONE_WARNINGS_AS_ERRORS "Treat project warnings as errors" OFF)
option(DRONE_ENABLE_ASAN "Enable AddressSanitizer on supported toolchains" OFF)
option(DRONE_ENABLE_UBSAN "Enable UndefinedBehaviorSanitizer on supported toolchains" OFF)
option(DRONE_ENABLE_CPACK "Enable archive packaging via CPack" OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(MSVC)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
endif()

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

function(drone_set_target_output_dirs target runtime_subdir library_subdir archive_subdir)
    foreach(config Debug Release RelWithDebInfo MinSizeRel)
        string(TOUPPER "${config}" config_upper)
        set_target_properties(${target} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY_${config_upper} "${PROJECT_BINARY_DIR}/${runtime_subdir}/${config}"
            LIBRARY_OUTPUT_DIRECTORY_${config_upper} "${PROJECT_BINARY_DIR}/${library_subdir}/${config}"
            ARCHIVE_OUTPUT_DIRECTORY_${config_upper} "${PROJECT_BINARY_DIR}/${archive_subdir}/${config}"
            PDB_OUTPUT_DIRECTORY_${config_upper} "${PROJECT_BINARY_DIR}/${archive_subdir}/${config}"
        )
    endforeach()
endfunction()

function(drone_append_build_interface_paths out_var)
    set(_wrapped_paths "")
    foreach(path IN LISTS ARGN)
        list(APPEND _wrapped_paths "$<BUILD_INTERFACE:${path}>")
    endforeach()
    set(${out_var} "${_wrapped_paths}" PARENT_SCOPE)
endfunction()

function(drone_append_install_interface_paths out_var)
    set(_wrapped_paths "")
    foreach(path IN LISTS ARGN)
        list(APPEND _wrapped_paths "$<INSTALL_INTERFACE:${path}>")
    endforeach()
    set(${out_var} "${_wrapped_paths}" PARENT_SCOPE)
endfunction()

drone_resolve_dependencies()

drone_append_build_interface_paths(DRONE_OPENCV_BUILD_INCLUDE_DIRS ${DRONE_OPENCV_INCLUDE_DIRS})
drone_append_build_interface_paths(DRONE_PCL_BUILD_INCLUDE_DIRS ${DRONE_PCL_INCLUDE_DIRS})
drone_append_install_interface_paths(DRONE_OPENCV_INSTALL_INCLUDE_DIRS ${DRONE_OPENCV_INCLUDE_DIRS})
drone_append_install_interface_paths(DRONE_PCL_INSTALL_INCLUDE_DIRS ${DRONE_PCL_INCLUDE_DIRS})
if(DRONE_HAVE_TENSORRT)
    drone_append_build_interface_paths(DRONE_TENSORRT_BUILD_INCLUDE_DIRS ${DRONE_TENSORRT_INCLUDE_DIR})
    drone_append_install_interface_paths(DRONE_TENSORRT_INSTALL_INCLUDE_DIRS ${DRONE_TENSORRT_INCLUDE_DIR})
else()
    set(DRONE_TENSORRT_BUILD_INCLUDE_DIRS "")
    set(DRONE_TENSORRT_INSTALL_INCLUDE_DIRS "")
endif()

set(DRONE_CORE_SOURCES
    src/autonomy/DecisionEngine.cpp
    src/autonomy/ExperienceMemory.cpp
    src/localization/LocalizationFusion.cpp
    src/localization/TDOAIngestor.cpp
    src/localization/TDOALocalizer.cpp
    src/localization/TimeSyncTracker.cpp
    src/localization/UWBSerialDriver.cpp
    src/runtime/RuntimeMode.cpp
    src/security/PeerPacketAuth.cpp
    src/swarm/EdgeConsensusManager.cpp
    src/swarm/EdgePeerProtocol.cpp
    src/swarm/SwarmStateCache.cpp
    src/safety/SafetyManager.cpp
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

add_library(sensor_fusion_core STATIC)
target_sources(sensor_fusion_core PRIVATE ${DRONE_CORE_SOURCES})
target_compile_features(sensor_fusion_core PUBLIC cxx_std_20)
set_target_properties(sensor_fusion_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(sensor_fusion_core
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/third_party/crypto>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_include_directories(sensor_fusion_core
    SYSTEM PUBLIC
        ${DRONE_OPENCV_BUILD_INCLUDE_DIRS}
        ${DRONE_PCL_BUILD_INCLUDE_DIRS}
        ${DRONE_TENSORRT_BUILD_INCLUDE_DIRS}
        ${DRONE_OPENCV_INSTALL_INCLUDE_DIRS}
        ${DRONE_PCL_INSTALL_INCLUDE_DIRS}
        ${DRONE_TENSORRT_INSTALL_INCLUDE_DIRS}
)
target_link_libraries(sensor_fusion_core
    PUBLIC
        Eigen3::Eigen
        Threads::Threads
        spdlog::spdlog
        ${DRONE_SECURITY_LIBS}
        ${DRONE_OPENCV_LIBS}
        ${DRONE_PCL_LIBS}
    PRIVATE
        ${DRONE_FASTDDS_LINK_LIBS}
        $<$<BOOL:${DRONE_HAVE_TENSORRT}>:${DRONE_TENSORRT_LIB}>
        $<$<BOOL:${WIN32}>:Ws2_32>
        $<$<BOOL:${WIN32}>:bcrypt>
        $<$<BOOL:${WIN32}>:crypt32>
        $<$<BOOL:${WIN32}>:ncrypt>
)
target_compile_definitions(sensor_fusion_core
    PUBLIC
        $<$<BOOL:${DRONE_HAVE_FASTDDS_TRANSPORT}>:DRONE_HAS_FASTDDS_TRANSPORT>
        $<$<BOOL:${DRONE_HAVE_TENSORRT}>:DRONE_HAS_TENSORRT>
        ${DRONE_PCL_COMPILE_DEFINITIONS}
)
drone_apply_project_warnings(sensor_fusion_core)
drone_enable_sanitizers(sensor_fusion_core)
drone_set_target_output_dirs(sensor_fusion_core "bin" "lib" "lib")

add_executable(drone_node src/main.cpp)
target_link_libraries(drone_node PRIVATE sensor_fusion_core)
drone_apply_project_warnings(drone_node)
drone_enable_sanitizers(drone_node)
drone_set_target_output_dirs(drone_node "bin" "lib" "lib")

if(DRONE_ENABLE_PYTHON_BINDINGS AND DRONE_HAVE_PYTHON_BINDINGS)
    pybind11_add_module(drone_bridge src/drone_bridge.cpp)
    target_include_directories(drone_bridge SYSTEM PRIVATE
        ${DRONE_OPENCV_INCLUDE_DIRS}
        ${DRONE_PCL_INCLUDE_DIRS}
        $<$<BOOL:${DRONE_HAVE_TENSORRT}>:${DRONE_TENSORRT_INCLUDE_DIR}>
    )
    target_compile_definitions(drone_bridge PRIVATE ${DRONE_PCL_COMPILE_DEFINITIONS})
    target_link_libraries(drone_bridge PRIVATE
        sensor_fusion_core
        ${DRONE_OPENCV_LIBS}
        ${DRONE_PCL_LIBS}
        $<$<BOOL:${DRONE_HAVE_TENSORRT}>:${DRONE_TENSORRT_LIB}>
    )
    drone_apply_project_warnings(drone_bridge)
    drone_enable_sanitizers(drone_bridge)
    drone_set_target_output_dirs(drone_bridge "python" "python" "python")
endif()

if(DRONE_BUILD_TESTS)
    drone_enable_googletest()
    enable_testing()
    add_subdirectory(tests)
endif()

install(TARGETS drone_node sensor_fusion_core
    EXPORT DroneSwarmTargets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

if(TARGET drone_bridge)
    install(TARGETS drone_bridge
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
endif()

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY third_party/crypto/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.h"
)

install(FILES
    LICENSE
    NOTICE
    README.md
    DEPLOYMENT.md
    DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}
)

install(FILES
    config/runtime.example.json
    config/anchors.example.json
    config/lidar.example.json
    config/detector_labels.example.json
    config/swarm_edge_protocol.example.json
    DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/config
)

configure_package_config_file(
    "${PROJECT_SOURCE_DIR}/cmake/DroneSwarmSensorFusionConfig.cmake.in"
    "${PROJECT_BINARY_DIR}/DroneSwarmSensorFusionConfig.cmake"
    INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
)

write_basic_package_version_file(
    "${PROJECT_BINARY_DIR}/DroneSwarmSensorFusionConfigVersion.cmake"
    VERSION "${PROJECT_VERSION}"
    COMPATIBILITY SameMajorVersion
)

install(EXPORT DroneSwarmTargets
    FILE DroneSwarmSensorFusionTargets.cmake
    NAMESPACE DroneSwarm::
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
)

install(FILES
    "${PROJECT_BINARY_DIR}/DroneSwarmSensorFusionConfig.cmake"
    "${PROJECT_BINARY_DIR}/DroneSwarmSensorFusionConfigVersion.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
)

if(DRONE_ENABLE_CPACK)
    set(CPACK_PACKAGE_NAME "${PROJECT_NAME}")
    set(CPACK_PACKAGE_VENDOR "Md Shahanur Islam Shagor")
    set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
    set(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/LICENSE")
    set(CPACK_RESOURCE_FILE_README "${PROJECT_SOURCE_DIR}/README.md")
    if(WIN32)
        set(CPACK_GENERATOR "ZIP")
    else()
        set(CPACK_GENERATOR "TGZ")
    endif()
    include(CPack)
endif()

message(STATUS "")
message(STATUS "DroneSwarmSensorFusion configuration summary")
message(STATUS "  Build type          : ${CMAKE_BUILD_TYPE}")
message(STATUS "  Compiler            : ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  Tests               : ${DRONE_BUILD_TESTS}")
message(STATUS "  Python bindings     : ${DRONE_PYTHON_BINDINGS_STATUS}")
message(STATUS "  Fast-DDS transport  : ${DRONE_FASTDDS_STATUS}")
message(STATUS "  TensorRT inference  : ${DRONE_TENSORRT_STATUS}")
message(STATUS "  Install prefix      : ${CMAKE_INSTALL_PREFIX}")
message(STATUS "")
