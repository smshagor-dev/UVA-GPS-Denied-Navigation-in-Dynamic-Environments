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

# CUDA paths (JetPack)
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda)
set(CMAKE_CUDA_COMPILER   /usr/local/cuda/bin/nvcc)

# TensorRT hints
set(TENSORRT_ROOT /usr/lib/aarch64-linux-gnu)
set(TENSORRT_INCLUDE_DIR /usr/include/aarch64-linux-gnu)

# OpenCV with CUDA support (JetPack build)
set(OpenCV_DIR /usr/lib/aarch64-linux-gnu/cmake/opencv4)

message(STATUS "Jetson Toolchain loaded")
message(STATUS "  CUDA: ${CUDA_TOOLKIT_ROOT_DIR}")
message(STATUS "  TRT:  ${TENSORRT_ROOT}")
