# GPS-Denied Drone Swarm — Multi-Modal Sensor Fusion
### C++20 / CMake / PySide6 / ESP32-CAM / TensorRT

```
╔══════════════════════════════════════════════════════════════════╗
║              GPS-DENIED DRONE SWARM  v2.0                        ║
║  LiDAR · IMU · ESP32-CAM (YOLOv8n) · Thermal · V2X Mesh        ║
╚══════════════════════════════════════════════════════════════════╝
```

## Architecture Overview

```
┌─────────────────────────────────────────────────────────┐
│                    DRONE NODE (C++20)                   │
│                                                         │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌────────┐ │
│  │  LiDAR   │  │ ESP32-CAM│  │   IMU    │  │Thermal │ │
│  │ (PCL)    │  │(RTSP/UDP)│  │(I2C 400Hz│  │MLX90640│ │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └───┬────┘ │
│       │              │              │             │      │
│  ┌────▼──────────────▼──────────────▼─────────────▼───┐ │
│  │           Error-State EKF  (Eigen3)                 │ │
│  │    State: [pos vel quat ba bg]  (16-dim)            │ │
│  │    Propagate @ IMU rate  (~400 Hz)                  │ │
│  │    Update    @ Camera    (~30  Hz)                  │ │
│  └────────────────────┬────────────────────────────────┘ │
│                        │ PoseEstimate                    │
│  ┌─────────────────────▼───────────────────────────────┐ │
│  │     Keyframe SLAM (ORB features, PCL cloud merge)   │ │
│  └─────────────────────┬───────────────────────────────┘ │
│                         │ Compressed KF                  │
│  ┌──────────────────────▼──────────────────────────────┐ │
│  │   V2X Mesh Network (Fast-DDS / UDP multicast)       │ │
│  │   Leader-Follower  |  Bully election  |  Formation  │ │
│  └─────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────┘
         │ pybind11 / TCP socket
┌────────▼──────────────────────────────────────────────────┐
│           PySide6 Lab Dashboard  (Python)                 │
│  ┌──────────┐ ┌───────────┐ ┌──────────┐ ┌────────────┐  │
│  │ 3D Map   │ │  Thermal  │ │  Drift   │ │  Health    │  │
│  │ pyqtgraph│ │ Heatmap   │ │  Graph   │ │  Cards     │  │
│  └──────────┘ └───────────┘ └──────────┘ └────────────┘  │
└───────────────────────────────────────────────────────────┘
```

## Project Structure

```
drone_swarm/
├── CMakeLists.txt              ← Multi-target CMake (C++20, Ninja)
├── README.md
│
├── include/
│   ├── sensors/
│   │   ├── SensorBase.hpp      ← Abstract base (lifecycle, threading)
│   │   ├── IMUSensor.hpp       ← MPU-6050 / ICM-42688-P  (I2C, 400Hz)
│   │   ├── LidarSensor.hpp     ← VLP-16 / RPLIDAR A3  (UDP, PCL)
│   │   ├── CameraSensor.hpp    ← ESP32-CAM RTSP + YOLOv8n TRT
│   │   └── ThermalSensor.hpp   ← MLX90640 32×24 IR  (I2C)
│   ├── vio/
│   │   ├── EKFEstimator.hpp    ← Error-state EKF  (Eigen3, 16-dim state)
│   │   └── VIOPipeline.hpp     ← Sensor orchestrator + event queue
│   ├── slam/
│   │   └── KeyframeManager.hpp ← ORB KF selection + compressed sharing
│   ├── swarm/
│   │   └── V2XMeshNetwork.hpp  ← Fast-DDS mesh, Bully election, formation
│   └── hal/
│       └── JetsonHAL.hpp       ← I2C/UART/ESP32 HAL + SystemStats
│
├── src/
│   ├── main.cpp                ← Drone node entry point
│   ├── drone_bridge.cpp        ← pybind11 Python ↔ C++ bridge
│   ├── sensors/                ← Sensor .cpp implementations
│   ├── vio/
│   │   └── EKFEstimator.cpp    ← Full EKF implementation
│   └── swarm/
│
├── firmware/
│   └── esp32_cam/
│       └── esp32_cam_firmware.ino  ← RTSP + UDP MJPEG + OTA
│
├── scripts/
│   └── drone_setup.py          ← Build/flash/run automation (argparse)
│
├── gui/
│   └── dashboard.py            ← PySide6 + pyqtgraph dashboard
│
├── tests/
│   └── CMakeLists.txt
│
├── cmake/
│   └── jetson_toolchain.cmake
│
└── logs/                       ← Rotating log files (spdlog)
```

## Quick Start

```bash
# 1. Install dependencies
python3 scripts/drone_setup.py setup

# 2. Build (Release)
python3 scripts/drone_setup.py build

# 3. Flash ESP32-CAM
python3 scripts/drone_setup.py flash --port=/dev/ttyUSB0

# 4. Launch drone node (Drone ID=1)
python3 scripts/drone_setup.py run --id=1 --esp32=192.168.4.1

# 5. Open lab dashboard
python3 scripts/drone_setup.py gui
```

## Key Technical Decisions

| Concern | Solution |
|---|---|
| Memory safety | `std::unique_ptr` / `std::shared_ptr` throughout |
| Concurrency | `std::thread` + `std::mutex` per sensor; event queue for VIO |
| Drift | Error-state EKF with Mahalanobis outlier gating |
| Bandwidth | Only ORB descriptors shared (< 4KB/KF), not raw images |
| Inference | TensorRT INT8 on Jetson; OpenCV DNN fallback on x86 |
| Comms | Fast-DDS multicast; UDP fallback if DDS absent |
| Logging | spdlog rotating files + colored console |
| GUI bridge | pybind11 module; TCP socket fallback for remote monitoring |

## Hardware Requirements

- **Jetson Nano** (4GB) or **Raspberry Pi 4** (8GB) per drone
- **ESP32-CAM** (AI-Thinker) — OV2640 camera
- **RPLiDAR A3** or **Velodyne VLP-16**
- **MPU-6050** or **ICM-42688-P** IMU (I2C)
- **MLX90640** 32×24 thermal sensor (I2C)
- **WiFi** 5GHz for V2X mesh (802.11ac recommended)
