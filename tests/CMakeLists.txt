# System Designer and Developer: Md Shahanur Islam Shagor
# Project: UVA GPS Denied Navigation in Dynamic Environments
# Technology: C++, Python, Go, CMake

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

set(GTEST_DISCOVERY_MODE POST_BUILD)
if(NOT CMAKE_CONFIGURATION_TYPES)
    set(GTEST_DISCOVERY_MODE PRE_TEST)
endif()

#  Test: EKF Estimator â”€
add_executable(test_ekf test_ekf.cpp)
target_link_libraries(test_ekf PRIVATE
    sensor_fusion_core
    GTest::gtest_main
    Eigen3::Eigen
)
gtest_discover_tests(test_ekf DISCOVERY_TIMEOUT 30 DISCOVERY_MODE ${GTEST_DISCOVERY_MODE})

#  Test: Sensor Base 
add_executable(test_sensors test_sensors.cpp)
target_link_libraries(test_sensors PRIVATE
    sensor_fusion_core
    GTest::gtest_main
)
gtest_discover_tests(test_sensors DISCOVERY_TIMEOUT 30 DISCOVERY_MODE ${GTEST_DISCOVERY_MODE})

#  Test: V2X Mesh 
add_executable(test_slam test_slam.cpp)
target_link_libraries(test_slam PRIVATE
    sensor_fusion_core
    GTest::gtest_main
    Eigen3::Eigen
    ${OpenCV_LIBS}
)
gtest_discover_tests(test_slam DISCOVERY_TIMEOUT 30 DISCOVERY_MODE ${GTEST_DISCOVERY_MODE})

add_executable(test_autonomy
    test_autonomy.cpp
    ../src/autonomy/DecisionEngine.cpp
    ../src/autonomy/ExperienceMemory.cpp
    ../src/localization/TDOALocalizer.cpp
)
target_include_directories(test_autonomy PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(test_autonomy PRIVATE
    GTest::gtest_main
    Eigen3::Eigen
    ${OpenCV_LIBS}
    spdlog::spdlog
)
gtest_discover_tests(test_autonomy DISCOVERY_TIMEOUT 30 DISCOVERY_MODE ${GTEST_DISCOVERY_MODE})

add_executable(test_navigation_intelligence
    test_navigation_intelligence.cpp
    ../src/autonomy/ExperienceMemory.cpp
    ../src/localization/LocalizationFusion.cpp
    ../src/localization/TDOAIngestor.cpp
    ../src/localization/TDOALocalizer.cpp
    ../src/localization/TimeSyncTracker.cpp
    ../src/slam/KeyframeManager.cpp
    ../src/slam/MapPlanner.cpp
    ../src/slam/OccupancyGridMap.cpp
    ../src/swarm/V2XMeshNetwork.cpp
    ../src/swarm/SwarmSecurity.cpp
    ../third_party/crypto/monocypher.c
    ../third_party/crypto/sha3.c
)
target_include_directories(test_navigation_intelligence PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(test_navigation_intelligence PRIVATE
    GTest::gtest_main
    Eigen3::Eigen
    ${OpenCV_LIBS}
    spdlog::spdlog
    ${PCL_LIBRARIES}
    $<$<BOOL:${MSVC}>:Ws2_32>
    $<$<BOOL:${MSVC}>:bcrypt>
)
gtest_discover_tests(test_navigation_intelligence DISCOVERY_TIMEOUT 30 DISCOVERY_MODE ${GTEST_DISCOVERY_MODE})

add_executable(test_swarm_security
    test_swarm_security.cpp
    ../src/sensors/MotorSensor.cpp
    ../src/swarm/SwarmSecurity.cpp
    ../src/swarm/V2XMeshNetwork.cpp
    ../third_party/crypto/monocypher.c
    ../third_party/crypto/sha3.c
)
target_include_directories(test_swarm_security PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/crypto
)
target_link_libraries(test_swarm_security PRIVATE
    GTest::gtest_main
    Eigen3::Eigen
    ${OpenCV_LIBS}
    spdlog::spdlog
    $<$<BOOL:${MSVC}>:bcrypt>
)
gtest_discover_tests(test_swarm_security DISCOVERY_TIMEOUT 30 DISCOVERY_MODE ${GTEST_DISCOVERY_MODE})

if(HAS_FASTDDS_TRANSPORT)
    add_executable(test_v2x test_v2x.cpp)
    target_link_libraries(test_v2x PRIVATE
        sensor_fusion_core
        GTest::gtest_main
        $<$<BOOL:${fastdds_FOUND}>:fastdds>
        $<$<BOOL:${fastrtps_FOUND}>:fastrtps>
    )
    add_test(NAME test_v2x COMMAND $<TARGET_FILE:test_v2x>)
endif()
# System Designer and Developer: Md Shahanur Islam Shagor
# Project: UVA GPS Denied Navigation in Dynamic Environments
# Technology: C++, Python, Go, CMake
# System Designer and Developer: Md Shahanur Islam Shagor
# Project: UVA GPS Denied Navigation in Dynamic Environments
# Technology: C++, Python, Go, CMake
