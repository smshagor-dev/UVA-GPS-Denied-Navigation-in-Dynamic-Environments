# System Designer and Developer: Md Shahanur Islam Shagor
# Project: UVA GPS Denied Navigation in Dynamic Environments
# Technology: C++, Python, Go, CMake

# GPS-Denied Drone Swarm Sensor Fusion

High-performance multi-drone software stack for operating a swarm in GPS-denied environments using onboard sensor fusion, local autonomy, swarm networking, and a real-time operator dashboard.

This project combines:

- `C++20` for onboard real-time perception, estimation, and safety-critical control
- `Python / PySide6` for the live monitoring and operator control dashboard
- `Go` for the fleet-scale control plane required for 1000+ drones

The system is designed for environments where GNSS is weak, jammed, or unavailable, and the drones must rely on `IMU + camera + LiDAR + V2X mesh` to localize, coordinate, and execute missions.

## 1. Project Goal

The goal of this project is to build a scalable drone swarm stack that can:

- estimate motion without GPS
- detect and track objects
- avoid static and dynamic obstacles
- elect leaders and maintain formations
- coordinate missions across many drones
- expose the full fleet state to an operator dashboard

At small scale, one drone can run the full onboard pipeline and be monitored locally.

At large scale, the same onboard logic can be connected to a `Go control plane` so that `1000+` drones can be grouped into clusters, scheduled, commanded, monitored, and audited centrally.

## 2. Core Features

### Onboard C++ features

- Error-state `EKF` for motion estimation
- `VIO` pipeline for camera + IMU based odometry
- `TDOA` localization module for anchor-based GPS-denied ranging fallback
- keyframe-based `SLAM` map sharing
- `YOLOv8n` object-detection driven autonomy
- personal ML-style `ExperienceMemory` layer for per-drone historical risk summarization
- `V2X` swarm networking with leader election and formations
- decentralized `MCSS` leader election using battery, motor-health, link-quality, CPU headroom, and thermal headroom scoring
- secure swarm transport with `AES-256` encryption, `PBKDF2` key derivation, `Ed25519` signatures, replay protection, and chain-linked leader command frames
- collision avoidance with peer, LiDAR, relative velocity, and local-minima escape logic
- local safety modes such as hold, return-home, and emergency-land

### Fleet / backend features

- telemetry ingest service
- swarm registry and drone discovery
- mission scheduling
- formation assignment
- command fan-out
- health monitoring
- event/log aggregation
- digital twin state cache
- dashboard API

### Dashboard features

- dark-themed real-time PySide6 UI
- 3D swarm trajectory view
- EKF drift plot
- swarm status table
- command console
- CPU / GPU / battery / link health cards
- direct `pybind11` mode for local lab use
- `Go backend` mode for fleet-scale use

## 3. System Architecture

```text
                  +----------------------------------+
                  |         PySide6 Dashboard        |
                  |  3D map | drift | health | cmd   |
                  +-----------------+----------------+
                                    |
                       HTTP / WebSocket / gRPC
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
        | sensors -> VIO / EKF / TDOA -> autonomy -> secure swarm -> safety |
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
- secure inter-drone transport
- collision avoidance
- flight safety logic

### Python

Use Python where operator productivity and rich desktop UI matter:

- PySide6 dashboard
- visualization
- rapid operator tool iteration
- bridge/debug workflows

### Go

Use Go where very large fleet orchestration and network concurrency matter:

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
cmd/
  control-plane/
    main.go
docs/
  GO_SWARM_ARCHITECTURE.md
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
- start VIO / EKF
- initialize TDOA anchors / ranging fallback
- maintain per-drone experience memory
- start swarm networking
- run autonomy loop
- publish health, memory, TDOA, and decision logs
- enforce secure swarm message acceptance / rejection rules

### 6.2 `src/drone_bridge.cpp`

`pybind11` module that exposes selected C++ functionality to Python.

Responsibilities:

- expose VIO pose
- expose system stats
- expose swarm commands and formation control

### 6.3 `gui/dashboard.py`

Operator dashboard.

Responsibilities:

- show real-time drone state
- issue election / formation / emergency commands
- render trajectories and drift
- connect either to local `pybind11` or the Go control plane

### 6.4 `cmd/control-plane/main.go`

Fleet backend entrypoint.

Responsibilities:

- start integrated control-plane server
- expose fleet API endpoints
- support telemetry ingest and command routing

### 6.5 `main.py`

Project launcher.

Responsibilities:

- start Go control plane if available
- start C++ drone node
- start Python dashboard
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

Typical dimensions:

- position: `3`
- velocity: `3`
- quaternion: `4`
- accel bias: `3`
- gyro bias: `3`

Total nominal state size: `16`

### 7.2 IMU propagation

At each IMU step:

```text
a_corrected = a_measured - b_a
w_corrected = w_measured - b_g
v_k+1 = v_k + (R(q_k) * a_corrected + g) * dt
p_k+1 = p_k + v_k * dt + 0.5 * (R(q_k) * a_corrected + g) * dt^2
q_k+1 = q_k boxplus (w_corrected * dt)
```

This allows high-rate prediction even when camera updates are sparse.

### 7.3 Visual update

Camera / VIO updates correct drift by minimizing innovation:

```text
r = z - h(x)
K = P H^T (H P H^T + R)^-1
x = x + K r
P = (I - K H) P
```

This is the core reason the project can operate without GPS.

### 7.4 TDOA localization

If synchronized anchors or ranging radios are available, the drone can estimate position from time-difference-of-arrival:

```text
d_i - d_ref = c * (t_i - t_ref)
```

Where:

- `d_i` = range to anchor `i`
- `d_ref` = range to the reference anchor
- `c` = propagation speed of the signal
- `t_i - t_ref` = arrival-time difference

The current implementation uses iterative least-squares / Gauss-Newton multilateration across `4+` anchors.

This gives the stack a second GPS-denied localization path:

- primary path: `VIO + EKF`
- secondary path: `TDOA anchor geometry`
- autonomy support path: return-home and caution logic can use TDOA confidence

### 7.5 Formation control

For a follower drone:

```text
v_cmd = k_p * (p_target - p_current)
```

Then collision avoidance is blended:

```text
v_total = v_tracking + v_avoid
```

Where `v_avoid` comes from peer repulsion, static LiDAR obstacles, relative velocity hazard weighting, and tangential escape when local minima occurs.

### 7.6 Relative-velocity avoidance

For a peer:

```text
v_rel = v_peer - v_self
closing_speed = -(v_rel dot d_hat)
time_to_collision = distance / closing_speed
```

If `time_to_collision` is below a prediction horizon, avoidance weight is increased.

This is better than only checking distance because two fast drones may still collide even if current separation seems acceptable.

### 7.7 Personal ML memory / experience prior

The onboard autonomy now maintains a lightweight historical memory per drone:

```text
memory = f(drift trend, battery burn, obstacle frequency, target frequency, dominant labels)
```

This is an embedded ML-style summarization layer rather than a heavy neural network. It stores what the drone repeatedly experiences and turns that into a usable risk prior.

Per drone, the memory layer computes:

- `drift_trend_m_per_min`
- `battery_burn_pct_per_min`
- `obstacle_frequency`
- `target_frequency`
- `dominant_label`
- `risk_score`

The high-level AI uses this to:

- slow down search in risky sectors
- keep followers more conservative when repeated hazards are observed
- preserve context across autonomy loop iterations

### 7.8 Drift monitoring

Dashboard drift graph tracks EKF drift over time:

```text
drift = || p_estimated - p_reference ||
```

Reference may come from loop closure, anchor reset, or trusted pose checkpoints depending on deployment mode.

### 7.9 Secure swarm communication

To reduce drone hijack risk, the swarm transport now applies a secure envelope around inter-drone traffic.

Framework elements:

- `AES-256` encrypted payload transport
- `PBKDF2-SHA256` key derivation from the shared swarm secret
- `Ed25519` signatures for sender authenticity
- replay rejection using monotonic secure sequence tracking
- tamper rejection using authenticated message checks
- chain-linked leader command frames for formation / mission / stop commands

The sync model uses a triple digest:

- `SHA-1` for the accepted past-frame commitment
- `SHA-256` for the present-frame payload commitment
- `SHA3-256` for the future-commitment check

If those sync proofs do not match, the frame is rejected and the sender is not trusted for that message.

Leader-to-follower high-priority messages such as:

- `FORMATION_CMD`
- `LEADER_ELECT`
- `MISSION_SYNC`
- `EMERGENCY_STOP`

are also wrapped in a ledger-style hash chain so that a broken command history is rejected instead of replayed.

## 8. Scaling Calculations for 1000+ Drones

The project is designed to scale far beyond a lab demo. The numbers below explain why the Go control plane is needed.

### 8.1 Flat dashboard polling is not enough

If one dashboard tries to directly manage `1000` drones and each drone sends `20 Hz` state updates:

```text
1000 drones * 20 updates/s = 20,000 updates/s
```

If each normalized telemetry payload is roughly `300 bytes`:

```text
20,000 * 300 bytes = 6,000,000 bytes/s
```

That is about:

```text
~6 MB/s raw telemetry
~48 Mb/s before protocol overhead, bursts, logs, and retries
```

This is exactly why the dashboard should not directly talk to every drone.

### 8.2 Clustered swarm strategy

Treating 1000 drones as one flat swarm is operationally expensive. A better model is clustered control.

If one cluster contains `20 drones`:

```text
1000 / 20 = 50 clusters
```

Then:

- one cluster leader can summarize local state
- formation changes can be pushed per cluster
- health alerts can be aggregated per cluster
- operator UI can drill down from fleet -> cluster -> drone

### 8.3 Leader count

With one primary leader and one backup per cluster:

```text
50 clusters * 2 key nodes = 100 high-priority leadership roles
```

This is much easier to manage than trying to coordinate 1000 peers equally.

### 8.4 Telemetry aggregation benefit

If the Go control plane stores full telemetry internally but publishes only `2 Hz` dashboard summaries:

```text
1000 drones * 2 updates/s = 2,000 dashboard-visible updates/s
```

That is a `10x` reduction from the raw `20,000 updates/s` ingest rate.

This is one of the main architectural wins of the Go backend.

## 9. Go Control Plane Responsibilities

The repository already contains a Go control-plane skeleton under:

- `cmd/control-plane`
- `internal/controlplane`

The design covers these fleet-scale roles:

- `central control plane`
- `telemetry ingest server`
- `swarm registry / drone discovery service`
- `mission scheduler`
- `formation assignment / fleet orchestration`
- `health monitoring backend`
- `command fan-out service`
- `WebSocket / gRPC style dashboard gateway`
- `log aggregation and event pipeline`
- `digital twin / live state cache`

Current implementation status:

- integrated HTTP control-plane service is present
- telemetry, mission, command, health, event, and discovery endpoints are present
- in-memory digital twin state store is present
- dashboard can connect to the backend using `--backend-url`

## 10. Dashboard Operating Modes

### Local lab mode

Use when testing one machine or one drone:

- dashboard reads from `pybind11`
- local C++ process provides pose / stats directly

### Fleet mode

Use when testing many drones:

- dashboard connects to Go backend
- backend provides aggregated fleet state
- dashboard should not directly open per-drone connections

## 11. Launch Options

### Option A: unified launcher

```bash
set DRONE_SWARM_SECRET=replace-with-strong-shared-secret
python main.py
```

What it does:

- starts Go control plane if available
- starts `drone_node.exe` if available
- starts the Python dashboard
- writes logs under `logs/launcher/`

Useful flags:

```bash
python main.py --dry-run
python main.py --skip-go
python main.py --skip-cpp
python main.py --skip-gui
```

### Option B: manual startup

#### 1. Go control plane

```bash
go run ./cmd/control-plane
```

#### 2. C++ drone node

```bash
set DRONE_SWARM_SECRET=replace-with-strong-shared-secret
build-dashboard/drone_node.exe --id=1 --esp32=192.168.4.1 --lidar=192.168.1.201:2368
```

#### 3. Dashboard in fleet mode

```bash
python gui/dashboard.py --backend-url http://127.0.0.1:8080
```

#### 4. Dashboard in local mode

```bash
python gui/dashboard.py
```

## 12. Build Notes

### C++

Main dependencies:

- `Eigen3`
- `OpenCV`
- `PCL`
- `spdlog`
- `pybind11`

### Python

Main dependencies:

- `PySide6`
- `pyqtgraph`
- `PyOpenGL`
- `numpy`

### Go

Main requirement:

- `Go 1.22+`

Note:

The current Windows environment used during development may have C++ and Python ready while `go` is not installed on `PATH`. In that case:

- C++ node can still run
- dashboard can still run
- control plane code exists but `go run` will require Go installation

## 13. Hardware Targets

Recommended onboard hardware:

- `Jetson Nano`
- `Jetson Orin`
- `Raspberry Pi 4` for reduced workloads

Typical sensors:

- `ESP32-CAM`
- `RPLIDAR A3` or similar LiDAR
- `MPU-6050` / `ICM-42688-P`
- ESC / motor telemetry or a dedicated motor-health sensor feed
- thermal sensor where needed

## 14. Current Engineering Status

Implemented in the repository:

- onboard sensor fusion pipeline
- autonomy decision engine
- experience-memory driven decision bias
- TDOA multilateration module
- secure swarm crypto envelope with AES-256 / PBKDF2 / Ed25519
- swarm formation and avoidance logic
- pybind11 bridge
- real-time PySide6 dashboard
- Go control-plane skeleton
- unified process launcher

Known limitations:

- Fast-DDS may be unavailable on some Windows setups, so V2X may fall back or be partially disabled
- Go backend source is integrated, but local execution still depends on Go toolchain availability
- current Go backend is an integrated service, not yet split into independent production microservices
- current `main.cpp` runtime path uses demo/synthetic TDOA measurements derived from the current pose; real UWB / RF anchor packet ingest still needs hardware integration

Recent verification completed:

- `build-tests/drone_node.exe` builds successfully
- `build-tests/tests/test_autonomy.exe` passes
- `build-tests/tests/test_navigation_intelligence.exe` passes
- `build-tests/tests/test_swarm_security.exe` passes

## 15. Roadmap

Next logical steps:

1. add protobuf / gRPC contracts between drone node and Go backend
2. publish C++ telemetry directly to Go telemetry ingest
3. add persistent stores such as Redis + PostgreSQL / ClickHouse
4. replace synthetic TDOA demo measurements with real UWB / RF anchor packet ingest
5. add WebSocket streaming for the fleet dashboard
6. split Go backend into deployable services
7. add mission replay, audit, and operator auth

## 16. Related Documents

- `docs/GO_SWARM_ARCHITECTURE.md`

## 17. Summary

This is a full-stack drone swarm project, not only a flight stack and not only a dashboard.

It combines:

- onboard estimation and autonomy in `C++`
- real-time operator visualization in `Python`
- large-fleet orchestration in `Go`

That split is the main reason the project can grow from:

- `1 local drone`

to

- `1000+ coordinated drones`

without collapsing under UI load, telemetry volume, or control complexity.
# System Designer and Developer: Md Shahanur Islam Shagor
# Project: UVA GPS Denied Navigation in Dynamic Environments
# Technology: C++, Python, Go, CMake
# System Designer and Developer: Md Shahanur Islam Shagor
# Project: UVA GPS Denied Navigation in Dynamic Environments
# Technology: C++, Python, Go, CMake
