# System Designer and Developer: Md Shahanur Islam Shagor
# Project: UVA GPS Denied Navigation in Dynamic Environments
# Technology: C++, Python, Go, CMake

cmake_minimum_required(VERSION 3.24)

include(GoogleTest)

set(GTEST_DISCOVERY_MODE PRE_TEST)

add_library(drone_test_support INTERFACE)
target_include_directories(drone_test_support INTERFACE
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/third_party/crypto"
)
target_include_directories(drone_test_support SYSTEM INTERFACE
    ${DRONE_OPENCV_INCLUDE_DIRS}
    ${DRONE_PCL_INCLUDE_DIRS}
)
target_link_libraries(drone_test_support INTERFACE
    GTest::gtest_main
)

function(drone_register_gtest target labels)
    drone_apply_project_warnings(${target})
    drone_enable_coverage(${target})
    drone_enable_sanitizers(${target})
    drone_set_target_output_dirs(${target} "tests" "tests" "tests")
    gtest_discover_tests(${target}
        DISCOVERY_TIMEOUT 30
        DISCOVERY_MODE ${GTEST_DISCOVERY_MODE}
        PROPERTIES LABELS "${labels}"
    )
endfunction()

add_executable(test_ekf test_ekf.cpp)
target_link_libraries(test_ekf PRIVATE
    drone_test_support
    sensor_fusion_core
    Eigen3::Eigen
)
drone_register_gtest(test_ekf "unit;navigation")

add_executable(test_sensors test_sensors.cpp)
target_link_libraries(test_sensors PRIVATE
    drone_test_support
    sensor_fusion_core
)
drone_register_gtest(test_sensors "unit;sensor;simulation")

add_executable(test_slam test_slam.cpp)
target_link_libraries(test_slam PRIVATE
    drone_test_support
    sensor_fusion_core
    Eigen3::Eigen
    ${DRONE_OPENCV_LIBS}
)
drone_register_gtest(test_slam "unit;simulation")

add_executable(test_autonomy
    test_autonomy.cpp
    ../src/autonomy/DecisionEngine.cpp
    ../src/autonomy/ExperienceMemory.cpp
    ../src/localization/TDOALocalizer.cpp
    ../src/sensors/CameraSensor.cpp
    ../src/safety/SafetyManager.cpp
)
target_link_libraries(test_autonomy PRIVATE
    drone_test_support
    Eigen3::Eigen
    ${DRONE_OPENCV_LIBS}
    ${DRONE_PCL_LIBS}
    spdlog::spdlog
)
target_compile_definitions(test_autonomy PRIVATE ${DRONE_PCL_COMPILE_DEFINITIONS})
drone_register_gtest(test_autonomy "unit;navigation;simulation")

add_executable(test_navigation_intelligence
    test_navigation_intelligence.cpp
    ../src/autonomy/ExperienceMemory.cpp
    ../src/localization/LocalizationFusion.cpp
    ../src/localization/TDOAIngestor.cpp
    ../src/localization/TDOALocalizer.cpp
    ../src/localization/TimeSyncTracker.cpp
    ../src/runtime/RuntimeMode.cpp
    ../src/safety/SafetyManager.cpp
    ../src/security/PeerPacketAuth.cpp
    ../src/sensors/CameraSensor.cpp
    ../src/sensors/IMUSensor.cpp
    ../src/sensors/LidarSensor.cpp
    ../src/slam/KeyframeManager.cpp
    ../src/slam/MapPlanner.cpp
    ../src/slam/OccupancyGridMap.cpp
    ../src/vio/EKFEstimator.cpp
    ../src/vio/VIOPipeline.cpp
    ../src/swarm/EdgeConsensusManager.cpp
    ../src/swarm/EdgePeerProtocol.cpp
    ../src/swarm/SwarmStateCache.cpp
    ../src/swarm/V2XMeshNetwork.cpp
    ../src/swarm/SwarmSecurity.cpp
    ../third_party/crypto/monocypher.c
    ../third_party/crypto/sha3.c
)
target_link_libraries(test_navigation_intelligence PRIVATE
    drone_test_support
    Eigen3::Eigen
    ${DRONE_OPENCV_LIBS}
    ${DRONE_PCL_LIBS}
    ${DRONE_SECURITY_LIBS}
    spdlog::spdlog
    $<$<BOOL:${MSVC}>:Ws2_32>
    $<$<BOOL:${MSVC}>:bcrypt>
)
target_compile_definitions(test_navigation_intelligence PRIVATE ${DRONE_PCL_COMPILE_DEFINITIONS})
drone_register_gtest(test_navigation_intelligence "unit;navigation;sensor;swarm;telemetry;security;simulation")

add_executable(test_swarm_security
    test_swarm_security.cpp
    ../src/sensors/MotorSensor.cpp
    ../src/security/PeerPacketAuth.cpp
    ../src/swarm/EdgeConsensusManager.cpp
    ../src/swarm/EdgePeerProtocol.cpp
    ../src/swarm/SwarmStateCache.cpp
    ../src/swarm/SwarmSecurity.cpp
    ../src/swarm/V2XMeshNetwork.cpp
    ../third_party/crypto/monocypher.c
    ../third_party/crypto/sha3.c
)
target_link_libraries(test_swarm_security PRIVATE
    drone_test_support
    Eigen3::Eigen
    ${DRONE_OPENCV_LIBS}
    ${DRONE_PCL_LIBS}
    ${DRONE_SECURITY_LIBS}
    spdlog::spdlog
    $<$<BOOL:${MSVC}>:bcrypt>
)
target_compile_definitions(test_swarm_security PRIVATE ${DRONE_PCL_COMPILE_DEFINITIONS})
drone_register_gtest(test_swarm_security "unit;security;swarm;simulation")

add_executable(test_boot_trust
    test_boot_trust.cpp
    ../third_party/crypto/sha3.c
)
target_link_libraries(test_boot_trust PRIVATE drone_test_support)
drone_register_gtest(test_boot_trust "unit;security")

add_executable(test_telemetry test_telemetry.cpp)
target_link_libraries(test_telemetry PRIVATE
    drone_test_support
    sensor_fusion_core
)
drone_register_gtest(test_telemetry "unit;telemetry")

add_executable(test_edge_swarm
    test_edge_swarm.cpp
    ../src/security/PeerPacketAuth.cpp
    ../src/swarm/EdgeConsensusManager.cpp
    ../src/swarm/EdgePeerProtocol.cpp
    ../src/swarm/SwarmStateCache.cpp
)
target_link_libraries(test_edge_swarm PRIVATE
    drone_test_support
    Eigen3::Eigen
)
drone_register_gtest(test_edge_swarm "unit;security;swarm")

if(DRONE_HAVE_FASTDDS_TRANSPORT)
    add_executable(test_v2x test_v2x.cpp)
    target_link_libraries(test_v2x PRIVATE
        drone_test_support
        sensor_fusion_core
        ${DRONE_FASTDDS_LINK_LIBS}
    )
    drone_apply_project_warnings(test_v2x)
    drone_enable_coverage(test_v2x)
    drone_enable_sanitizers(test_v2x)
    drone_set_target_output_dirs(test_v2x "tests" "tests" "tests")
    add_test(NAME test_v2x COMMAND $<TARGET_FILE:test_v2x>)
    set_tests_properties(test_v2x PROPERTIES LABELS "integration;swarm;simulation")
endif()
