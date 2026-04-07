cmake_minimum_required(VERSION 3.20)

# Fetch GoogleTest
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(googletest)

include(GoogleTest)

# ── Test: EKF Estimator ───────────────────────────────────────────────────────
add_executable(test_ekf test_ekf.cpp)
target_link_libraries(test_ekf PRIVATE
    sensor_fusion_core
    GTest::gtest_main
    Eigen3::Eigen
)
gtest_discover_tests(test_ekf)

# ── Test: Sensor Base ─────────────────────────────────────────────────────────
add_executable(test_sensors test_sensors.cpp)
target_link_libraries(test_sensors PRIVATE
    sensor_fusion_core
    GTest::gtest_main
)
gtest_discover_tests(test_sensors)

# ── Test: V2X Mesh ────────────────────────────────────────────────────────────
add_executable(test_slam test_slam.cpp)
target_link_libraries(test_slam PRIVATE
    sensor_fusion_core
    GTest::gtest_main
    Eigen3::Eigen
    ${OpenCV_LIBS}
)
gtest_discover_tests(test_slam)

add_executable(test_autonomy
    test_autonomy.cpp
    ../src/autonomy/DecisionEngine.cpp
)
target_include_directories(test_autonomy PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(test_autonomy PRIVATE
    GTest::gtest_main
    Eigen3::Eigen
    ${OpenCV_LIBS}
)
gtest_discover_tests(test_autonomy)

if(HAS_FASTDDS)
    add_executable(test_v2x test_v2x.cpp)
    target_link_libraries(test_v2x PRIVATE
        sensor_fusion_core
        GTest::gtest_main
        fastrtps
    )
    gtest_discover_tests(test_v2x)
endif()
