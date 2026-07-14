# cmake/jetson_toolchain.cmake
# Cross-compilation toolchain for NVIDIA Jetson Nano / Orin (aarch64)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# Use native compiler (running directly on Jetson)
set(CMAKE_C_COMPILER   gcc)
set(CMAKE_CXX_COMPILER g++)

# Jetson-specific optimization flags
add_compile_options(
    -mcpu=cortex-a57      # Jetson Nano CPU
    -mfpu=neon-fp-armv8
    -mfloat-abi=hard
    -ftree-vectorize
    -ffast-math
)

set(DRONE_JETSON_CUDA_ROOT "$ENV{DRONE_JETSON_CUDA_ROOT}" CACHE PATH "Jetson CUDA root")
if(NOT DRONE_JETSON_CUDA_ROOT)
    set(DRONE_JETSON_CUDA_ROOT "/usr/local/cuda" CACHE PATH "Jetson CUDA root" FORCE)
endif()
set(CUDA_TOOLKIT_ROOT_DIR "${DRONE_JETSON_CUDA_ROOT}" CACHE PATH "CUDA toolkit root for Jetson")
if(EXISTS "${CUDA_TOOLKIT_ROOT_DIR}/bin/nvcc")
    set(CMAKE_CUDA_COMPILER "${CUDA_TOOLKIT_ROOT_DIR}/bin/nvcc" CACHE FILEPATH "CUDA compiler for Jetson")
endif()

set(DRONE_JETSON_TENSORRT_ROOT "$ENV{DRONE_JETSON_TENSORRT_ROOT}" CACHE PATH "Jetson TensorRT root")
if(NOT DRONE_JETSON_TENSORRT_ROOT)
    set(DRONE_JETSON_TENSORRT_ROOT "/usr/lib/aarch64-linux-gnu" CACHE PATH "Jetson TensorRT root" FORCE)
endif()
set(TENSORRT_ROOT "${DRONE_JETSON_TENSORRT_ROOT}" CACHE PATH "TensorRT root for Jetson")

set(DRONE_JETSON_TENSORRT_INCLUDE_DIR "$ENV{DRONE_JETSON_TENSORRT_INCLUDE_DIR}" CACHE PATH "Jetson TensorRT include directory")
if(NOT DRONE_JETSON_TENSORRT_INCLUDE_DIR)
    set(DRONE_JETSON_TENSORRT_INCLUDE_DIR "/usr/include/aarch64-linux-gnu" CACHE PATH "Jetson TensorRT include directory" FORCE)
endif()
set(TENSORRT_INCLUDE_DIR "${DRONE_JETSON_TENSORRT_INCLUDE_DIR}" CACHE PATH "TensorRT include directory for Jetson")

set(DRONE_JETSON_OPENCV_DIR "$ENV{DRONE_JETSON_OPENCV_DIR}" CACHE PATH "Jetson OpenCV CMake package directory")
if(NOT DRONE_JETSON_OPENCV_DIR)
    set(DRONE_JETSON_OPENCV_DIR "/usr/lib/aarch64-linux-gnu/cmake/opencv4" CACHE PATH "Jetson OpenCV CMake package directory" FORCE)
endif()
set(OpenCV_DIR "${DRONE_JETSON_OPENCV_DIR}" CACHE PATH "OpenCV package directory for Jetson")

message(STATUS "Jetson Toolchain loaded")
message(STATUS "  CUDA: ${CUDA_TOOLKIT_ROOT_DIR}")
message(STATUS "  TRT:  ${TENSORRT_ROOT}")
