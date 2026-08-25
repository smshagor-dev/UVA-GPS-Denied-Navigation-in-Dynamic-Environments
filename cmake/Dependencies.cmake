include_guard(GLOBAL)

include(FetchContent)

set(DRONE_FASTDDS_LINK_LIBS "")
set(DRONE_OPENCV_INCLUDE_DIRS "")
set(DRONE_OPENCV_LIBS "")
set(DRONE_PCL_INCLUDE_DIRS "")
set(DRONE_PCL_LIBS "")
set(DRONE_PCL_COMPILE_DEFINITIONS "")
set(DRONE_SECURITY_LIBS "")
set(DRONE_TENSORRT_INCLUDE_DIR "")
set(DRONE_TENSORRT_LIB "")
set(DRONE_HAVE_FASTDDS_TRANSPORT FALSE)
set(DRONE_HAVE_TENSORRT FALSE)
set(DRONE_HAVE_PYTHON_BINDINGS FALSE)
set(DRONE_FASTDDS_STATUS "DISABLED")
set(DRONE_TENSORRT_STATUS "DISABLED")
set(DRONE_PYTHON_BINDINGS_STATUS "DISABLED")

function(drone_enable_googletest)
    if(TARGET GTest::gtest_main)
        return()
    endif()

    find_package(GTest CONFIG QUIET)
    if(TARGET GTest::gtest_main)
        message(STATUS "GoogleTest: FOUND - package config mode")
        return()
    endif()

    if(NOT DRONE_FETCH_GOOGLETEST)
        message(FATAL_ERROR
            "GoogleTest is required when DRONE_BUILD_TESTS=ON. "
            "Install GTest or enable DRONE_FETCH_GOOGLETEST.")
    endif()

    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(googletest)
    message(STATUS "GoogleTest: FETCHED - FetchContent fallback enabled")
endfunction()

function(drone_resolve_dependencies)
    find_package(Threads REQUIRED)

    find_package(Eigen3 3.4 QUIET NO_MODULE)
    if(NOT Eigen3_FOUND)
        find_path(DRONE_EIGEN_INCLUDE_DIR
            NAMES Eigen/Core signature_of_eigen3_matrix_library
            HINTS
                "$ENV{EIGEN3_INCLUDE_DIR}"
                "$ENV{EIGEN3_ROOT}"
            PATH_SUFFIXES eigen3
        )

        if(DRONE_EIGEN_INCLUDE_DIR)
            add_library(Eigen3::Eigen INTERFACE IMPORTED)
            set_target_properties(Eigen3::Eigen PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES "${DRONE_EIGEN_INCLUDE_DIR}"
            )
            set(Eigen3_FOUND TRUE)
            set(EIGEN3_VERSION "header-only fallback")
            set(EIGEN3_VERSION "header-only fallback" PARENT_SCOPE)
            message(STATUS "Eigen3: FOUND - include-only fallback at ${DRONE_EIGEN_INCLUDE_DIR}")
        endif()
    endif()

    if(NOT Eigen3_FOUND)
        message(FATAL_ERROR
            "Eigen3 is required but was not found.\n"
            "Windows: set VCPKG_ROOT and use the manifest-backed presets.\n"
            "Linux: install libeigen3-dev or provide EIGEN3_ROOT / EIGEN3_INCLUDE_DIR.")
    endif()
    if(NOT DEFINED EIGEN3_VERSION)
        if(DEFINED Eigen3_VERSION)
            set(EIGEN3_VERSION "${Eigen3_VERSION}" PARENT_SCOPE)
        else()
            set(EIGEN3_VERSION "package config" PARENT_SCOPE)
        endif()
    endif()
    message(STATUS "Eigen3: FOUND - version ${EIGEN3_VERSION}")

    find_package(OpenCV 4.5 REQUIRED COMPONENTS core imgproc highgui calib3d features2d video dnn)
    set(DRONE_OPENCV_INCLUDE_DIRS "${OpenCV_INCLUDE_DIRS}" PARENT_SCOPE)
    set(DRONE_OPENCV_LIBS "${OpenCV_LIBS}" PARENT_SCOPE)
    message(STATUS "OpenCV: FOUND - version ${OpenCV_VERSION}")

    find_package(PCL 1.12 REQUIRED COMPONENTS common io filters segmentation kdtree search)
    set(DRONE_PCL_INCLUDE_DIRS "${PCL_INCLUDE_DIRS}" PARENT_SCOPE)
    set(DRONE_PCL_LIBS "${PCL_LIBRARIES}" PARENT_SCOPE)
    set(DRONE_PCL_COMPILE_DEFINITIONS "${PCL_DEFINITIONS}" PARENT_SCOPE)
    message(STATUS "PCL: FOUND - version ${PCL_VERSION}")

    find_package(spdlog REQUIRED CONFIG)
    message(STATUS "spdlog: FOUND - version ${spdlog_VERSION}")

    if(NOT WIN32)
        find_package(OpenSSL REQUIRED COMPONENTS Crypto)
        set(DRONE_SECURITY_LIBS OpenSSL::Crypto PARENT_SCOPE)
        message(STATUS "OpenSSL: FOUND - version ${OpenSSL_VERSION}")
    endif()

    if(DRONE_ENABLE_FASTDDS)
        find_package(fastdds QUIET CONFIG)
        find_package(fastrtps QUIET CONFIG)
        set(_drone_fastdds_libs "")
        if(fastdds_FOUND)
            list(APPEND _drone_fastdds_libs fastdds)
        endif()
        if(fastrtps_FOUND)
            list(APPEND _drone_fastdds_libs fastrtps)
        endif()

        if(_drone_fastdds_libs)
            set(DRONE_FASTDDS_LINK_LIBS "${_drone_fastdds_libs}" PARENT_SCOPE)
            set(DRONE_HAVE_FASTDDS_TRANSPORT TRUE PARENT_SCOPE)
            set(DRONE_FASTDDS_STATUS "FOUND - DDS transport enabled" PARENT_SCOPE)
            message(STATUS "Fast-DDS: FOUND - DDS transport enabled")
        else()
            set(DRONE_FASTDDS_STATUS "NOT FOUND - UDP transport fallback enabled" PARENT_SCOPE)
            message(STATUS "Fast-DDS: NOT FOUND - UDP transport fallback enabled")
        endif()
    else()
        set(DRONE_FASTDDS_STATUS "DISABLED - DRONE_ENABLE_FASTDDS=OFF" PARENT_SCOPE)
        message(STATUS "Fast-DDS: DISABLED - DRONE_ENABLE_FASTDDS=OFF")
    endif()

    if(DRONE_ENABLE_TENSORRT)
        find_path(DRONE_TENSORRT_INCLUDE_DIR
            NAMES NvInfer.h
            HINTS "$ENV{TENSORRT_ROOT}/include"
            PATHS /usr/include/aarch64-linux-gnu /usr/local/include
        )
        find_library(DRONE_TENSORRT_LIB
            NAMES nvinfer
            HINTS "$ENV{TENSORRT_ROOT}/lib" "$ENV{TENSORRT_ROOT}/lib/x64"
            PATHS /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu
        )

        if(DRONE_TENSORRT_INCLUDE_DIR AND DRONE_TENSORRT_LIB)
            set(DRONE_TENSORRT_INCLUDE_DIR "${DRONE_TENSORRT_INCLUDE_DIR}" PARENT_SCOPE)
            set(DRONE_TENSORRT_LIB "${DRONE_TENSORRT_LIB}" PARENT_SCOPE)
            set(DRONE_HAVE_TENSORRT TRUE PARENT_SCOPE)
            set(DRONE_TENSORRT_STATUS "FOUND - TensorRT inference enabled" PARENT_SCOPE)
            message(STATUS "TensorRT: FOUND - TensorRT inference enabled")
        else()
            set(DRONE_TENSORRT_STATUS "NOT FOUND - OpenCV DNN fallback enabled" PARENT_SCOPE)
            message(STATUS "TensorRT: NOT FOUND - OpenCV DNN fallback enabled")
        endif()
    else()
        set(DRONE_TENSORRT_STATUS "DISABLED - OpenCV DNN fallback enabled" PARENT_SCOPE)
        message(STATUS "TensorRT: DISABLED - OpenCV DNN fallback enabled")
    endif()

    if(DRONE_ENABLE_PYTHON_BINDINGS)
        set(PYBIND11_FINDPYTHON ON)
        find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)
        find_package(pybind11 CONFIG QUIET)

        if(pybind11_FOUND)
            set(DRONE_HAVE_PYTHON_BINDINGS TRUE PARENT_SCOPE)
            set(DRONE_PYTHON_BINDINGS_STATUS "FOUND - pybind11 package config" PARENT_SCOPE)
            message(STATUS "Python bindings: FOUND - pybind11 package config")
        elseif(DRONE_FETCH_PYBIND11)
            FetchContent_Declare(
                pybind11
                GIT_REPOSITORY https://github.com/pybind/pybind11.git
                GIT_TAG v2.13.6
                GIT_SHALLOW TRUE
            )
            FetchContent_MakeAvailable(pybind11)
            if(TARGET pybind11::module)
                set(DRONE_HAVE_PYTHON_BINDINGS TRUE PARENT_SCOPE)
                set(DRONE_PYTHON_BINDINGS_STATUS "FETCHED - pybind11 FetchContent fallback" PARENT_SCOPE)
                message(STATUS "Python bindings: FETCHED - pybind11 FetchContent fallback")
            else()
                set(DRONE_PYTHON_BINDINGS_STATUS "NOT FOUND - python bindings disabled" PARENT_SCOPE)
                message(STATUS "Python bindings: NOT FOUND - python bindings disabled")
            endif()
        else()
            set(DRONE_PYTHON_BINDINGS_STATUS "NOT FOUND - python bindings disabled" PARENT_SCOPE)
            message(STATUS "Python bindings: NOT FOUND - python bindings disabled")
        endif()
    else()
        set(DRONE_PYTHON_BINDINGS_STATUS "DISABLED - DRONE_ENABLE_PYTHON_BINDINGS=OFF" PARENT_SCOPE)
        message(STATUS "Python bindings: DISABLED - DRONE_ENABLE_PYTHON_BINDINGS=OFF")
    endif()
endfunction()
