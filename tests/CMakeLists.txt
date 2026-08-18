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

set(DRONE_PHASE17_TEST_CORE sensor_fusion_core)
if(DRONE_ENABLE_PHASE17_HEADLESS_TSAN)
    add_library(phase17_estimator_headless_core STATIC
        ../src/vio/EKFEstimator.cpp
        ../src/vio/EKFStateEstimatorAdapter.cpp
        ../src/vio/EstimatorCoordinator.cpp
        ../src/vio/MeasurementEnvelope.cpp
        ../src/vio/Phase17ESKFEstimator.cpp
        ../src/vio/StateEstimator.cpp
        ../third_party/crypto/monocypher.c
        ../third_party/crypto/sha3.c
    )
    target_compile_features(phase17_estimator_headless_core PUBLIC cxx_std_20)
    set_target_properties(phase17_estimator_headless_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
    target_include_directories(phase17_estimator_headless_core
        PUBLIC
            $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
            $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/third_party/crypto>
    )
    target_link_libraries(phase17_estimator_headless_core
        PUBLIC
            Eigen3::Eigen
            Threads::Threads
            spdlog::spdlog
    )
    drone_apply_project_warnings(phase17_estimator_headless_core)
    drone_enable_coverage(phase17_estimator_headless_core)
    drone_enable_sanitizers(phase17_estimator_headless_core)
    drone_set_target_output_dirs(phase17_estimator_headless_core "tests" "tests" "tests")
    set(DRONE_PHASE17_TEST_CORE phase17_estimator_headless_core)
endif()

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

add_executable(test_phase16_shadow test_phase16_shadow.cpp)
target_link_libraries(test_phase16_shadow PRIVATE
    drone_test_support
    sensor_fusion_core
    Eigen3::Eigen
)
drone_register_gtest(test_phase16_shadow "unit;navigation;replay;phase16")

add_executable(test_phase17_eskf test_phase17_eskf.cpp)
target_link_libraries(test_phase17_eskf PRIVATE
    drone_test_support
    ${DRONE_PHASE17_TEST_CORE}
    Eigen3::Eigen
)
drone_register_gtest(test_phase17_eskf "unit;navigation;replay;phase17")

add_executable(test_phase18_zupt test_phase18_zupt.cpp)
target_link_libraries(test_phase18_zupt PRIVATE
    drone_test_support
    ${DRONE_PHASE17_TEST_CORE}
    Eigen3::Eigen
)
drone_register_gtest(test_phase18_zupt "unit;navigation;replay;phase18")

add_executable(test_phase19_fej test_phase19_fej.cpp)
target_link_libraries(test_phase19_fej PRIVATE
    drone_test_support
    ${DRONE_PHASE17_TEST_CORE}
    Eigen3::Eigen
)
drone_register_gtest(test_phase19_fej "unit;navigation;replay;phase19")

add_executable(test_phase20_msckf test_phase20_msckf.cpp)
target_link_libraries(test_phase20_msckf PRIVATE
    drone_test_support
    ${DRONE_PHASE17_TEST_CORE}
    Eigen3::Eigen
)
drone_register_gtest(test_phase20_msckf "unit;navigation;replay;phase20")

add_executable(test_phase21_triangulation test_phase21_triangulation.cpp)
target_link_libraries(test_phase21_triangulation PRIVATE
    drone_test_support
    ${DRONE_PHASE17_TEST_CORE}
    Eigen3::Eigen
)
drone_register_gtest(test_phase21_triangulation "unit;navigation;replay;phase21")

add_executable(test_phase22_msckf_update test_phase22_msckf_update.cpp)
target_link_libraries(test_phase22_msckf_update PRIVATE
    drone_test_support
    ${DRONE_PHASE17_TEST_CORE}
    Eigen3::Eigen
)
drone_register_gtest(test_phase22_msckf_update "unit;navigation;replay;phase22")

add_executable(ekf_phase15_replay ekf_phase15_replay.cpp)
target_link_libraries(ekf_phase15_replay PRIVATE
    sensor_fusion_core
    Eigen3::Eigen
)
drone_apply_project_warnings(ekf_phase15_replay)
drone_enable_coverage(ekf_phase15_replay)
drone_enable_sanitizers(ekf_phase15_replay)
drone_set_target_output_dirs(ekf_phase15_replay "tests" "tests" "tests")
add_test(NAME ekf_phase15_replay COMMAND $<TARGET_FILE:ekf_phase15_replay>)
set_tests_properties(ekf_phase15_replay PROPERTIES LABELS "integration;navigation;replay")

add_executable(ekf_phase16_replay ekf_phase16_replay.cpp)
target_link_libraries(ekf_phase16_replay PRIVATE
    sensor_fusion_core
    Eigen3::Eigen
)
drone_apply_project_warnings(ekf_phase16_replay)
drone_enable_coverage(ekf_phase16_replay)
drone_enable_sanitizers(ekf_phase16_replay)
drone_set_target_output_dirs(ekf_phase16_replay "tests" "tests" "tests")
add_test(NAME ekf_phase16_replay COMMAND $<TARGET_FILE:ekf_phase16_replay>)
set_tests_properties(ekf_phase16_replay PROPERTIES LABELS "integration;navigation;replay;phase16")

add_executable(ekf_phase17_replay ekf_phase17_replay.cpp)
target_link_libraries(ekf_phase17_replay PRIVATE
    ${DRONE_PHASE17_TEST_CORE}
    Eigen3::Eigen
)
drone_apply_project_warnings(ekf_phase17_replay)
drone_enable_coverage(ekf_phase17_replay)
drone_enable_sanitizers(ekf_phase17_replay)
drone_set_target_output_dirs(ekf_phase17_replay "tests" "tests" "tests")
add_test(NAME ekf_phase17_replay COMMAND $<TARGET_FILE:ekf_phase17_replay>)
set_tests_properties(ekf_phase17_replay PROPERTIES LABELS "integration;navigation;replay;phase17")

add_executable(ekf_phase18_replay ekf_phase18_replay.cpp)
target_link_libraries(ekf_phase18_replay PRIVATE
    ${DRONE_PHASE17_TEST_CORE}
    Eigen3::Eigen
)
drone_apply_project_warnings(ekf_phase18_replay)
drone_enable_coverage(ekf_phase18_replay)
drone_enable_sanitizers(ekf_phase18_replay)
drone_set_target_output_dirs(ekf_phase18_replay "tests" "tests" "tests")
add_test(NAME ekf_phase18_replay COMMAND $<TARGET_FILE:ekf_phase18_replay>)
set_tests_properties(ekf_phase18_replay PROPERTIES LABELS "integration;navigation;replay;phase18")

add_executable(ekf_phase19_replay ekf_phase19_replay.cpp)
target_link_libraries(ekf_phase19_replay PRIVATE
    ${DRONE_PHASE17_TEST_CORE}
    Eigen3::Eigen
)
drone_apply_project_warnings(ekf_phase19_replay)
drone_enable_coverage(ekf_phase19_replay)
drone_enable_sanitizers(ekf_phase19_replay)
drone_set_target_output_dirs(ekf_phase19_replay "tests" "tests" "tests")
add_test(NAME ekf_phase19_replay COMMAND $<TARGET_FILE:ekf_phase19_replay>)
set_tests_properties(ekf_phase19_replay PROPERTIES LABELS "integration;navigation;replay;phase19")

add_executable(ekf_phase20_replay ekf_phase20_replay.cpp)
target_link_libraries(ekf_phase20_replay PRIVATE
    ${DRONE_PHASE17_TEST_CORE}
    Eigen3::Eigen
)
drone_apply_project_warnings(ekf_phase20_replay)
drone_enable_coverage(ekf_phase20_replay)
drone_enable_sanitizers(ekf_phase20_replay)
drone_set_target_output_dirs(ekf_phase20_replay "tests" "tests" "tests")
add_test(NAME ekf_phase20_replay COMMAND $<TARGET_FILE:ekf_phase20_replay>)
set_tests_properties(ekf_phase20_replay PROPERTIES LABELS "integration;navigation;replay;phase20")

add_executable(ekf_phase21_replay ekf_phase21_replay.cpp)
target_link_libraries(ekf_phase21_replay PRIVATE
    ${DRONE_PHASE17_TEST_CORE}
    Eigen3::Eigen
)
drone_apply_project_warnings(ekf_phase21_replay)
drone_enable_coverage(ekf_phase21_replay)
drone_enable_sanitizers(ekf_phase21_replay)
drone_set_target_output_dirs(ekf_phase21_replay "tests" "tests" "tests")
add_test(NAME ekf_phase21_replay COMMAND $<TARGET_FILE:ekf_phase21_replay>)
set_tests_properties(ekf_phase21_replay PROPERTIES LABELS "integration;navigation;replay;phase21")

add_executable(ekf_phase22_replay ekf_phase22_replay.cpp)
target_link_libraries(ekf_phase22_replay PRIVATE
    ${DRONE_PHASE17_TEST_CORE}
    Eigen3::Eigen
)
drone_apply_project_warnings(ekf_phase22_replay)
drone_enable_coverage(ekf_phase22_replay)
drone_enable_sanitizers(ekf_phase22_replay)
drone_set_target_output_dirs(ekf_phase22_replay "tests" "tests" "tests")
add_test(NAME ekf_phase22_replay COMMAND $<TARGET_FILE:ekf_phase22_replay>)
set_tests_properties(ekf_phase22_replay PROPERTIES LABELS "integration;navigation;replay;phase22")

add_executable(phase16_benchmark phase16_benchmark.cpp)
target_link_libraries(phase16_benchmark PRIVATE
    sensor_fusion_core
    Eigen3::Eigen
)
drone_apply_project_warnings(phase16_benchmark)
drone_enable_coverage(phase16_benchmark)
drone_enable_sanitizers(phase16_benchmark)
drone_set_target_output_dirs(phase16_benchmark "tests" "tests" "tests")

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
    ../src/vio/EKFStateEstimatorAdapter.cpp
    ../src/vio/EstimatorCoordinator.cpp
    ../src/vio/MeasurementEnvelope.cpp
    ../src/vio/Phase17ESKFEstimator.cpp
    ../src/vio/StateEstimator.cpp
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
