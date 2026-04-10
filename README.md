# System Designer and Developer: Md Shahanur Islam Shagor
# Project: UVA GPS Denied Navigation in Dynamic Environments
# Technology: C++, Python, Go, CMake

# GPS-Denied Drone Swarm Sensor Fusion

High-performance multi-drone software stack for operating a swarm in GPS-denied environments using onboard sensor fusion, local autonomy, swarm networking, and a real-time operator dashboard.

This project combines:

- `C++20` for onboard real-time perception, estimation, and safety-oriented control
- `Python / PySide6` for the live monitoring and operator control dashboard
- `Go` for the fleet-scale control plane and digital-twin style backend

The system is designed for environments where GNSS is weak, jammed, or unavailable, and drones must rely on `IMU + camera + LiDAR + V2X mesh` to localize, coordinate, and execute missions.

## 1. Project Goal

The goal of this project is to build a scalable drone swarm stack that can:

- estimate motion without GPS
- detect and track objects
- avoid static and dynamic obstacles
- elect leaders and maintain formations
- coordinate missions across many drones
- expose full fleet state to an operator dashboard

At small scale, one drone can run the onboard pipeline and be monitored locally.

At larger scale, the same onboard logic can connect to the `Go control plane` so drones can be grouped into clusters, commanded, monitored, and audited centrally.

## 2. Core Features

### Onboard C++ features

- error-state `EKF` for motion estimation
- `VIO` pipeline for camera + IMU based odometry
- `TDOA` localization module for anchor-based GPS-denied ranging fallback
- localization confidence scoring with degraded / lost-state detection
- keyframe-based `SLAM` support
- `ExperienceMemory` layer for per-drone historical risk summarization
- `DecisionEngine` for autonomy decisions
- built-in `V2X` swarm networking with optional native `Fast-DDS` transport when installed
- secure swarm transport helpers in the swarm security module
- local safety / degraded-mode behavior when hardware or optional dependencies are unavailable

### Fleet / backend features

- telemetry ingest service
- fleet snapshot endpoint
- mission scheduling endpoints
- command fan-out endpoint
- health monitoring endpoint
- event/log aggregation endpoint
- discovery endpoint
- in-memory digital twin state store

### Dashboard features

- dark-themed real-time `PySide6` UI
- 3D swarm trajectory view
- EKF drift plot
- swarm status table
- command console
- CPU / GPU / battery / link health cards
- localization source / confidence visibility
- local `pybind11` mode when bridge builds are available
- `Go backend` mode for fleet-scale use
- simulation fallback when neither backend path is available
- persistent Python dashboard datastore for settings, command history, and last fleet snapshot restore

## 3. System Architecture

```text
                  +----------------------------------+
                  |         PySide6 Dashboard        |
                  |  3D map | drift | health | cmd   |
                  +-----------------+----------------+
                                    |
                       HTTP / WebSocket-like polling
                                    |
                  +----------------------------------+
                  |         Go Control Plane         |
                  | telemetry | missions | commands  |
                  | registry  | health   | twin      |
                  +-----------------+----------------+
                                    |
                    telemetry / command distribution
                                    |
        +---------------------------------------------------------+
        |                  Drone Node (C++20)                     |
        | sensors -> VIO / EKF / TDOA -> autonomy -> swarm -> safety |
        +---------------------------------------------------------+
            |          |            |            |
          IMU        Camera       LiDAR        V2X Mesh
```

## 4. Why Three Languages

### C++

Use C++ where latency and determinism matter:

- sensor drivers
- EKF propagation
- visual-inertial estimation
- TDOA multilateration
- LiDAR processing
- onboard autonomy
- local safety logic

### Python

Use Python where operator productivity and rich desktop UI matter:

- `PySide6` dashboard
- visualization
- rapid operator tool iteration
- local bridge/debug workflows

### Go

Use Go where fleet orchestration and concurrent network I/O matter:

- control plane
- telemetry ingest
- mission orchestration
- command fan-out
- health monitoring
- state caching
- dashboard backend

## 5. Repository Layout

```text
drone_swarm/
  CMakeLists.txt
  go.mod
  main.py
  README.md
  CONTRIBUTE.md
  DEPLOYMENT.md
cmd/
  control-plane/
    main.go
docs/
  GO_SWARM_ARCHITECTURE.md
firmware/
  esp32_cam/
gui/
  dashboard.py
include/
  autonomy/
  hal/
  localization/
  sensors/
  slam/
  swarm/
  vio/
internal/
  controlplane/
    server.go
    state.go
    types.go
scripts/
  drone_setup.py
src/
  autonomy/
  hal/
  localization/
  sensors/
  slam/
  swarm/
  vio/
  drone_bridge.cpp
  main.cpp
tests/
```

## 6. Main Runtime Components

### 6.1 `src/main.cpp`

Main onboard drone node.

Responsibilities:

- initialize sensors
- start `VIO / EKF`
- initialize TDOA anchors
- maintain per-drone experience memory
- run autonomy loop
- publish health, memory, TDOA, and decision logs
- enable swarm networking through built-in UDP transport and native `Fast-DDS` transport when available

Important note:

The current runtime path supports demo-generated TDOA, CSV playback, UDP text ingest, and serial UWB-style ingest. If no external measurements are provided, it still falls back to synthetic demo measurements.

### 6.2 `src/drone_bridge.cpp`

`pybind11` module that exposes selected C++ functionality to Python.

Responsibilities:

- expose VIO pose
- expose system stats
- expose optional swarm controls

### 6.3 `gui/dashboard.py`

Operator dashboard.

Responsibilities:

- show real-time drone state
- issue formation / hold / return-home / emergency commands
- render trajectories and drift
- connect either to local `pybind11` or the Go control plane
- fall back to simulation when needed

### 6.4 `cmd/control-plane/main.go`

Fleet backend entrypoint.

Responsibilities:

- start the integrated control-plane server
- expose fleet API endpoints
- support telemetry ingest and command routing
- seed a digital-twin style fleet snapshot for demo use

### 6.5 `main.py`

Project launcher.

Responsibilities:

- start Go control plane if available
- start C++ drone node if available
- start the Python dashboard
- shut all processes down together

## 7. Estimation and Control Calculations

This project is not only a UI or communication system. It contains real estimation and swarm-control logic. The core calculations are summarized below.

### 7.1 EKF state

The onboard error-state EKF tracks:

```text
x = [p, v, q, b_a, b_g]
```

Where:

- `p` = position
- `v` = velocity
- `q` = attitude quaternion
- `b_a` = accelerometer bias
- `b_g` = gyroscope bias

### 7.2 IMU propagation

At each IMU step:

```text
a_corrected = a_measured - b_a
w_corrected = w_measured - b_g
v_k+1 = v_k + (R(q_k) * a_corrected + g) * dt
p_k+1 = p_k + v_k * dt + 0.5 * (R(q_k) * a_corrected + g) * dt^2
q_k+1 = q_k boxplus (w_corrected * dt)
```

### 7.3 Visual update

Camera / VIO updates correct drift by minimizing innovation:

```text
r = z - h(x)
K = P H^T (H P H^T + R)^-1
x = x + K r
P = (I - K H) P
```

### 7.4 TDOA localization

If synchronized anchors or ranging radios are available, the drone can estimate position from time-difference-of-arrival:

```text
d_i - d_ref = c * (t_i - t_ref)
```

The current implementation uses iterative least-squares / Gauss-Newton style multilateration across multiple anchors.

### 7.5 Formation control

For a follower drone:

```text
v_cmd = k_p * (p_target - p_current)
```

Collision avoidance is then blended into the final velocity command.

### 7.6 Relative-velocity avoidance

For a peer:

```text
v_rel = v_peer - v_self
closing_speed = -(v_rel dot d_hat)
time_to_collision = distance / closing_speed
```

If `time_to_collision` falls below a prediction horizon, avoidance weight is increased.

### 7.7 Experience memory / risk prior

The onboard autonomy maintains a lightweight historical memory per drone:

```text
memory = f(drift trend, battery burn, obstacle frequency, target frequency, dominant labels)
```

Per drone, the memory layer summarizes:

- `drift_trend_m_per_min`
- `battery_burn_pct_per_min`
- `obstacle_frequency`
- `target_frequency`
- `dominant_label`
- `risk_score`

### 7.8 Drift monitoring

Dashboard drift graph tracks EKF drift over time:

```text
drift = || p_estimated - p_reference ||
```

### 7.9 Secure swarm communication

The swarm security module provides secure-envelope style handling for inter-drone traffic, including:

- payload protection helpers
- sender authenticity checks
- replay rejection logic
- chain-linked command validation concepts

These features are present in the codebase. Swarm traffic can run on the built-in UDP transport everywhere, and can additionally use native `Fast-DDS` transport when that package is installed.

## 8. Scaling Considerations

The project is designed to scale beyond a lab demo.

### 8.1 Why the Go backend matters

If many drones send frequent updates, the dashboard should not connect directly to every drone.

Instead:

- drones publish or forward telemetry
- the Go backend aggregates fleet state
- the dashboard reads from a single backend API

### 8.2 Clustered swarm strategy

A clustered model scales better than one flat swarm:

- one cluster leader can summarize local state
- formation changes can be pushed per cluster
- health alerts can be aggregated per cluster
- the operator UI can drill down from fleet -> cluster -> drone

## 9. Go Control Plane Responsibilities

The repository contains a Go control-plane implementation under:

- `cmd/control-plane`
- `internal/controlplane`

Current implementation status:

- integrated HTTP control-plane service is present
- telemetry, fleet, mission, command, health, event, and discovery endpoints are present
- in-memory digital twin state store is present
- dashboard can connect to the backend using `--backend-url`

Current API routes:

- `GET /api/v1/fleet`
- `POST /api/v1/telemetry`
- `POST /api/v1/commands`
- `GET,POST /api/v1/missions`
- `GET /api/v1/health`
- `GET /api/v1/events`
- `GET /api/v1/discovery`

## 10. Dashboard Operating Modes

### Local lab mode

Use when testing one machine or one drone:

- dashboard reads from `pybind11` if `drone_bridge` is importable
- local C++ process provides pose / stats directly

### Fleet mode

Use when testing many drones:

- dashboard connects to Go backend
- backend provides aggregated fleet state
- dashboard should not directly open per-drone connections

### Simulation fallback mode

Use when the bridge module or backend is unavailable:

- dashboard synthesizes fleet activity for UI validation
- useful for design/demo work without full runtime dependencies

### Persistent dashboard state

The Python dashboard now keeps a local datastore so the next launch can restore:

- recent command history
- last selected drone
- last known fleet snapshot
- last used backend URL
- local mission overrides and added simulated drones

## 11. Launch Options

### Option A: unified launcher

```powershell
$env:DRONE_SWARM_SECRET="replace-with-strong-shared-secret"
python main.py
```

What it does:

- starts Go control plane if available
- starts `drone_node.exe` if available
- starts the Python dashboard
- writes logs under `logs/launcher/`

Useful flags:

```powershell
python main.py --dry-run
python main.py --skip-go
python main.py --skip-cpp
python main.py --skip-gui
```

### Option B: manual startup

#### 1. Go control plane

```powershell
go run ./cmd/control-plane
```

#### 2. C++ drone node

```powershell
$env:DRONE_SWARM_SECRET="replace-with-strong-shared-secret"
build\Release\drone_node.exe --id=1 --esp32=192.168.4.1 --lidar=192.168.1.201:2368 --tdoa-csv=tdoa_measurements.csv
```

#### 3. Dashboard in fleet mode

```powershell
$env:DRONE_OPERATOR_ID="operator-console-1"
$env:DRONE_OPERATOR_ROLE="operator"
$env:DRONE_OPERATOR_SECRET="replace-with-a-strong-operator-secret"
python gui/dashboard.py --backend-url http://127.0.0.1:8080
```

For hardened HTTPS or mTLS fleet mode:

```powershell
$env:DRONE_OPERATOR_ID="operator-console-1"
$env:DRONE_OPERATOR_ROLE="operator"
$env:DRONE_TLS_ENABLED="true"
$env:DRONE_TLS_CA_FILE="certs/ca.crt"
$env:DRONE_TLS_CLIENT_CERT_FILE="certs/operator-client.crt"
$env:DRONE_TLS_CLIENT_KEY_FILE="certs/operator-client.key"
python gui/dashboard.py --backend-url https://127.0.0.1:8080
```

Before first hardened run, generate local certificates:

```powershell
python scripts/generate_tls_certs.py --force
```

The generated operator client certificate uses common name `operator-console-1` by default. In mTLS mode, that identity must match `DRONE_OPERATOR_ID` or command submission will be rejected.
Role-based command authorization is also enforced by the control plane through `DRONE_OPERATOR_ROLE`. Use `operator` for standard mission control, `commander` for election privileges, and `maintenance` for maintenance-only actions.
For multi-operator approval workflows, configure the control-plane with `DRONE_OPERATOR_CREDENTIALS` using `operator_id:role:secret;...`. Critical actions such as `election` and `emergency_land` now require a second distinct authenticated operator to approve the exact same request.

The onboard C++ runtime now also evaluates a drone security state from link integrity, timing trust, localization loss, and hardened-profile trust posture. In bridge mode this is exposed as `security_state`, `security_summary`, `link_integrity_score`, and health flags to the dashboard.
It also keeps a drone-side remote command inbox and policy gate: secure swarm commands such as formation hold, mission sync, and emergency stop are accepted or rejected on the drone itself based on the current onboard security state.
The node can now also post native telemetry directly to the Go backend when `DRONE_ENABLE_BACKEND_TELEMETRY=true`, which lets the backend/dashboard receive the drone's onboard security posture without depending on the Python bridge.

#### 4. Dashboard in local mode

```powershell
python gui/dashboard.py
```

## 12. Build Notes

### C++

Main required dependencies:

- `Eigen3`
- `OpenCV`
- `PCL`
- `spdlog`

Optional dependencies:

- `pybind11` is auto-fetched if not already installed
- native `Fast-DDS` transport is enabled when the package is installed
- `TensorRT` is only available on systems with NVIDIA runtime support

### Python

Main dashboard dependencies:

- `PySide6`
- `pyqtgraph`
- `PyOpenGL`
- `numpy`

### Go

Main requirement:

- `Go 1.22+`

## 13. Hardware Targets

Recommended onboard hardware:

- `Jetson Nano`
- `Jetson Orin`
- `Raspberry Pi 4` for reduced workloads

Typical sensors:

- `ESP32-CAM`
- LiDAR such as `RPLIDAR`
- IMU devices such as `MPU-6050` or `ICM-42688-P`
- ESC / motor telemetry feed

## 14. Current Engineering Status

Implemented in the repository:

- onboard sensor fusion pipeline
- autonomy decision engine
- experience-memory driven decision bias
- TDOA multilateration module
- external TDOA CSV / UDP / serial ingest paths for real measurement playback/integration
- localization confidence, source tracking, and degraded/lost navigation modes
- secure swarm security module
- built-in swarm networking path with native `Fast-DDS` support when installed
- `pybind11` bridge with local Windows build support
- real-time `PySide6` dashboard
- Go control-plane service
- unified process launcher
- loop-relocalization hook and map-planner waypoint generation
- OpenCV DNN fallback inference path for non-TensorRT machines

Known limitations:

- native `TensorRT` still requires an NVIDIA GPU/runtime and is not available on AMD-only machines
- local C++ builds still depend on native packages being installed and discoverable by CMake
- if external TDOA/UWB data is not supplied, `main.cpp` still falls back to synthetic demo measurements
- several checked-in build folders may be stale across path or machine changes
- `scripts/drone_setup.py` is mainly Linux/Jetson oriented

Verification completed in this workspace on `2026-04-08`:

- `python -m py_compile main.py gui/dashboard.py scripts/drone_setup.py` passed
- `go test ./...` passed
- `cmake -S . -B build-runtime-check -DBUILD_TESTS=OFF` passed
- `cmake --build build-runtime-check --config Release --target drone_node drone_bridge` passed
- `drone_bridge` imported successfully from the built module with Windows DLL directories added
- `test_autonomy.exe` passed
- `test_navigation_intelligence.exe` passed
- native `Fast-DDS` transport was detected after local package installation

What that means:

- Python, Go, and the main C++ runtime path are currently healthy in this workspace
- native `Fast-DDS` transport is now available on this machine
- `TensorRT` remains hardware-dependent rather than code-blocked
- old stale build outputs should still not be treated as reliable proof of current build health

## 15. Roadmap

Next logical steps:

1. automate Windows DLL-path setup for local `drone_bridge` imports
2. package serial UWB vendor parsers for specific hardware models
3. add more automated Go/API tests
4. deepen loop closure and global map optimization
5. continue maturing the backend toward production deployment patterns

## 16. Related Documents

- [docs/GO_SWARM_ARCHITECTURE.md](/d:/Final%20Project/drone_swarm/docs/GO_SWARM_ARCHITECTURE.md)
- [CONTRIBUTE.md](/d:/Final%20Project/drone_swarm/CONTRIBUTE.md)
- [DEPLOYMENT.md](/d:/Final%20Project/drone_swarm/DEPLOYMENT.md)

## 17. Summary

This is a full-stack drone swarm project, not only a flight stack and not only a dashboard.

It combines:

- onboard estimation and autonomy in `C++`
- real-time operator visualization in `Python`
- fleet orchestration in `Go`

That split lets the project support:

- local single-drone workflows
- backend-connected multi-drone monitoring
- future scaling toward larger coordinated fleets
