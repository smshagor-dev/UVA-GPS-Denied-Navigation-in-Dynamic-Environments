# System Designer and Developer: Md Shahanur Islam Shagor
# Project: UVA GPS Denied Navigation in Dynamic Environments
# Technology: C++, Python, Go, CMake

cmake_minimum_required(VERSION 3.24)
include(GoogleTest)
set(GTEST_DISCOVERY_MODE PRE_TEST)

add_library(drone_test_support INTERFACE)
target_include_directories(drone_test_support INTERFACE "${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/third_party/crypto")
target_include_directories(drone_test_support SYSTEM INTERFACE ${DRONE_OPENCV_INCLUDE_DIRS} ${DRONE_PCL_INCLUDE_DIRS})
target_link_libraries(drone_test_support INTERFACE GTest::gtest_main)

set(DRONE_PHASE17_TEST_CORE sensor_fusion_core)
if(DRONE_ENABLE_PHASE17_HEADLESS_TSAN)
    add_library(phase17_estimator_headless_core STATIC
        ../src/vio/EKFEstimator.cpp ../src/vio/EKFStateEstimatorAdapter.cpp ../src/vio/EstimatorCoordinator.cpp
        ../src/vio/MeasurementEnvelope.cpp ../src/vio/Phase17ESKFEstimator.cpp ../src/vio/StateEstimator.cpp
        ../third_party/crypto/monocypher.c ../third_party/crypto/sha3.c)
    target_compile_features(phase17_estimator_headless_core PUBLIC cxx_std_20)
    set_target_properties(phase17_estimator_headless_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
    target_include_directories(phase17_estimator_headless_core PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include> $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/third_party/crypto>)
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
    gtest_discover_tests(${target} DISCOVERY_TIMEOUT 30 DISCOVERY_MODE ${GTEST_DISCOVERY_MODE} PROPERTIES LABELS "${labels}")
endfunction()

function(drone_estimator_gtest target source labels)
    add_executable(${target} ${source})
    target_link_libraries(${target} PRIVATE drone_test_support ${DRONE_PHASE17_TEST_CORE} Eigen3::Eigen)
    drone_register_gtest(${target} "${labels}")
endfunction()

add_executable(test_ekf test_ekf.cpp)
target_link_libraries(test_ekf PRIVATE drone_test_support sensor_fusion_core Eigen3::Eigen)
drone_register_gtest(test_ekf "unit;navigation")
add_executable(test_phase16_shadow test_phase16_shadow.cpp)
target_link_libraries(test_phase16_shadow PRIVATE drone_test_support sensor_fusion_core Eigen3::Eigen)
drone_register_gtest(test_phase16_shadow "unit;navigation;replay;phase16")
drione_placeholder()
