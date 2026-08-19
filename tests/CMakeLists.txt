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
target_link_libraries(drone_test_support INTERFACE GTest::gtest_main)

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
    target_include_directories(phase17_estimator_headless_core PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/third_party/crypto>
    )
    target_link_libraries(phase17_estimator_headless_core PUBLIC Eigen3::Eigen Threads::Threads spdlog::spdlog)
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
target_link_libraries(test_ekf PRIVATE drone_test_support sensor_fusion_core Eigen3::Eigen)
drone_register_gtest(test_ekf "unit;navigation")

add_executable(test_phase16_shadow test_phase16_shadow.cpp)
target_link_libraries(test_phase16_shadow PRIVATE drone_test_support sensor_fusion_core Eigen3::Eigen)
drone_register_gtest(test_phase16_shadow "unit;navigation;replay;phase16")

foreach(test_target IN ITEMS
    test_phase17_eskf
    test_phase18_zupt
    test_phase19_fej
    test_phase20_msckf
    test_phase21_triangulation
    test_phase22_msckf_update
    test_msckf_marginalization_integration
    test_shadow_only_measurement
)
    add_executable(${test_target} ${test_target}.cpp)
    target_link_libraries(${test_target} PRIVATE drone_test_support ${DRONE_PHASE17_TEST_CORE} Eigen3::Eigen)
    drone_register_gtest(${test_target} "unit;navigation;replay")
endforeach()

add_executable(test_visual_feature_track_manager test_visual_feature_track_manager.cpp)
target_link_libraries(test_visual_feature_track_manager PRIVATE drone_test_support Eigen3::Eigen)
drone_register_gtest(test_visual_feature_track_manager "unit;navigation;visual-feature-tracking")

add_executable(test_visual_feature_track_ingest test_visual_feature_track_ingest.cpp)
target_link_libraries(test_visual_feature_track_ingest PRIVATE
    drone_test_support
    ${DRONE_PHASE17_TEST_CORE}
    Eigen3::Eigen
)
drone_register_gtest(test_visual_feature_track_ingest "unit;navigation;visual-feature-tracking;shadow-only")

foreach(replay_target IN ITEMS
    ekf_phase15_replay
    ekf_phase16_replay
    ekf_phase17_replay
    ekf_phase18_replay
    ekf_phase19_replay
    ekf_phase20_replay
    ekf_phase21_replay
    ekf_phase22_replay
)
    add_executable(${replay_target} ${replay_target}.cpp)
    if(replay_target STREQUAL "ekf_phase15_replay" OR replay_target STREQUAL "ekf_phase16_replay")
        target_link_libraries(${replay_target} PRIVATE sensor_fusion_core Eigen3::Eigen)
    else()
        target_link_libraries(${replay_target} PRIVATE ${DRONE_PHASE17_TEST_CORE} Eigen3::Eigen)
    endif()
    drone_apply_project_warnings(${replay_target})
    drone_enable_coverage(${replay_target})
    drone_enable_sanitizers(${replay_target})
    drone_set_target_output_dirs(${replay_target} "tests" "tests" "tests")
    add_test(NAME ${replay_target} COMMAND $<TARGET_FILE:${replay_target}>)
    set_tests_properties(${replay_target} PROPERTIES LABELS "integration;navigation;replay")
endforeach()

add_executable(phase16_benchmark phase16_benchmark.cpp)
target_link_libraries(phase16_benchmark PRIVATE sensor_fusion_core Eigen3::Eigen)
drone_apply_project_warnings(phase16_benchmark)
drone_enable_coverage(phase16_benchmark)
drone_enable_sanitizers(phase16_benchmark)
drone_set_target_output_dirs(phase16_benchmark "tests" "tests" "tests")

add_executable(test_sensors test_sensors.cpp)
target_link_libraries(test_sensors PRIVATE drone_test_support sensor_fusion_core)
drone_register_gtest(test_sensors "unit;sensor;simulation")

add_executable(test_slam test_slam.cpp)
target_link_libraries(test_slam PRIVATE drone_test_support sensor_fusion_core Eigen3::Eigen ${DRONE_OPENCV_LIBS})
drone_register_gtest(test_slam "unit;simulation")

add_executable(test_autonomy
    test_autonomy.cpp
    ../src/autonomy/DecisionEngine.cpp
    ../src/autonomy/ExperienceMemory.cpp
    ../src/localization/TDOALocalizer.cpp
    ../src/sensors/CameraSensor.cpp
    ../src/safety/SafetyManager.cpp
)
target_link_libraries(test_autonomy PRIVATE drone_test_support Eigen3::Eigen ${DRONE_OPENCV_LIBS} ${DRONE_PCL_LIBS} spdlog::spdlog)
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
    drone_test_support Eigen3::Eigen ${DRONE_OPENCV_LIBS} ${DRONE_PCL_LIBS} ${DRONE_SECURITY_LIBS}
    spdlog::spdlog $<$<BOOL:${MSVC}>:Ws2_32> $<$<BOOL:${MSVC}>:bcrypt>
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
    drone_test_support Eigen3::Eigen ${DRONE_OPENCV_LIBS} ${DRONE_PCL_LIBS} ${DRONE_SECURITY_LIBS}
    spdlog::spdlog $<$<BOOL:${MSVC}>:bcrypt>
)
target_compile_definitions(test_swarm_security PRIVATE ${DRONE_PCL_COMPILE_DEFINITIONS})
drone_register_gtest(test_swarm_security "unit;security;swarm;simulation")

add_executable(test_boot_trust test_boot_trust.cpp ../third_party/crypto/sha3.c)
target_link_libraries(test_boot_trust PRIVATE drone_test_support)
drone_register_gtest(test_boot_trust "unit;security")

add_executable(test_telemetry test_telemetry.cpp)
target_link_libraries(test_telemetry PRIVATE drone_test_support sensor_fusion_core)
drone_register_gtest(test_telemetry "unit;telemetry")

add_executable(test_edge_swarm
    test_edge_swarm.cpp
    ../src/security/PeerPacketAuth.cpp
    ../src/swarm/EdgeConsensusManager.cpp
    ../src/swarm/EdgePeerProtocol.cpp
    ../src/swarm/SwarmStateCache.cpp
)
target_link_libraries(test_edge_swarm PRIVATE drone_test_support Eigen3::Eigen)
drone_register_gtest(test_edge_swarm "unit;security;swarm")

if(DRONE_HAVE_FASTDDS_TRANSPORT)
    add_executable(test_v2x test_v2x.cpp)
    target_link_libraries(test_v2x PRIVATE drone_test_support sensor_fusion_core ${DRONE_FASTDDS_LINK_LIBS})
    drone_apply_project_warnings(test_v2x)
    drone_enable_coverage(test_v2x)
    drone_enable_sanitizers(test_v2x)
    drone_set_target_output_dirs(test_v2x "tests" "tests" "tests")
    add_test(NAME test_v2x COMMAND $<TARGET_FILE:test_v2x>)
    set_tests_properties(test_v2x PROPERTIES LABELS "integration;swarm;simulation")
endif()
