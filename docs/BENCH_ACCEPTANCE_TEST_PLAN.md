# Bench Acceptance Test Plan

## Purpose

This plan defines the minimum bench acceptance gate before any tethered or free-flight GPS-denied testing.

It is intentionally conservative:

- passing bench checks does **not** mean the system is flight ready
- failing any required bench check means the system must be treated as **not ready for flight**
- simulation/demo fallback is not acceptable for bench acceptance unless the test category explicitly says so

## Bench Gate Rule

Do not claim flight readiness unless:

- runtime mode is `bench` or `production`
- synthetic/demo localization fallback is disabled
- real anchors are loaded
- required LiDAR is configured and available
- camera and IMU open successfully
- backend is reachable in non-simulation mode
- dashboard can visibly show non-simulation backend mode

Suggested command:

```powershell
python scripts/bench_check.py --runtime-config config/runtime.json --backend-url http://127.0.0.1:8080
```

Expected result:

- `BENCH VERDICT: PASS`

If the script reports `FAIL`, do not proceed to flight testing.

## Test Categories

### 1. IMU live data test

Goal:
- prove the real IMU device opens and produces live data without synthetic fallback

Procedure:
- verify `DRONE_ENABLE_IMU=true`
- verify `DRONE_IMU_DEVICE` points to the actual bus/device
- start the node in `bench` mode
- confirm the IMU opens and live measurements are observed
- confirm no synthetic IMU fallback is active in logs

Acceptance criteria:
- IMU device opens successfully
- live sample timestamps advance
- accelerometer and gyro values change with physical motion
- no simulation/fallback warning is present

Failure examples:
- device missing
- read permission denied
- constant zero/fixed values
- synthetic fallback active

### 2. Camera live stream test

Goal:
- prove the real camera stream is reachable and produces frames

Procedure:
- verify `DRONE_ENABLE_CAMERA=true`
- verify `DRONE_CAMERA_STREAM_URL` or `DRONE_ESP32_IP`
- open the stream and read at least one frame
- confirm the node reports live camera input rather than demo fallback

Acceptance criteria:
- stream opens successfully
- at least one frame is received
- frame timestamps advance during continuous polling
- no simulation placeholder path is active

Failure examples:
- RTSP stream cannot open
- stream opens but no frames arrive
- repeated stale frame

### 3. LiDAR live scan test

Goal:
- prove the configured LiDAR path is real, configured, and producing scan data

Procedure:
- verify `config/lidar.json` exists and is referenced by runtime config
- verify LiDAR host/port/model/frame settings
- start the node in `bench` or `production`
- confirm LiDAR socket/parsing path initializes
- confirm scan points and obstacle points are produced

Acceptance criteria:
- LiDAR config loads successfully
- required LiDAR initializes
- live scan packets arrive
- obstacle list is non-empty when targets are placed in front of the sensor

Failure examples:
- missing config
- invalid model/port
- parser rejects all packets
- required LiDAR unavailable

### 4. UWB/TDOA live measurement test

Goal:
- prove external real anchor measurements are reaching the node

Procedure:
- verify runtime mode is not `simulation`
- verify real anchor config is loaded
- verify live UDP or serial TDOA/UWB ingest path is active
- confirm anchor visibility count and real measurement updates in logs/telemetry

Acceptance criteria:
- at least 4 anchors are loaded
- live measurements are ingested from UDP or serial
- localization data source is `real`, not `simulation`
- production mode does not rely on playback-only input

Failure examples:
- missing anchors
- duplicate/invalid anchors
- no live measurements
- fallback to synthetic or playback-only path

### 5. EKF stability test

Goal:
- prove the estimator remains numerically stable on live bench data

Procedure:
- run the node stationary on the bench
- observe pose, drift, confidence, and lost/degraded state
- gently excite the IMU and confirm the estimate responds and settles

Acceptance criteria:
- no NaN/inf values
- localization state remains `nominal` or recovers cleanly
- drift does not grow without bound while stationary
- safety manager does not enter unstable oscillatory state

Failure examples:
- unbounded drift
- repeated lost/recover flapping
- estimator divergence

### 6. VIO tracking quality test

Goal:
- prove the visual frontend is tracking real features well enough for bench validation

Procedure:
- point the camera at a textured scene
- observe feature count, inlier ratio, reprojection error, and visual confidence
- then reduce texture/lighting and verify confidence drops safely

Acceptance criteria:
- tracked features are consistently non-trivial
- inlier ratio remains above the degraded threshold in a normal textured scene
- reprojection error stays bounded
- low visual quality reduces safety envelope rather than silently continuing at full speed

Failure examples:
- persistent low feature count
- RANSAC rejecting nearly all updates
- confidence remains high despite poor image quality

### 7. Backend telemetry test

Goal:
- prove the onboard node can reach the Go backend with real telemetry

Procedure:
- verify `DRONE_ENABLE_BACKEND_TELEMETRY=true`
- verify `DRONE_BACKEND_URL` and `DRONE_SWARM_SECRET`
- start backend in `production`
- confirm `/api/v1/fleet` shows the real drone with `source=real`

Acceptance criteria:
- backend reachable
- telemetry posts do not crash the node
- `backend_mode` is visible and not `simulation`
- `simulation_enabled=false`

Failure examples:
- backend unreachable
- telemetry rejected or stale
- real drone not visible in fleet snapshot

### 8. Dashboard real-data mode test

Goal:
- prove the operator can distinguish real backend data from simulation/fallback

Procedure:
- connect the dashboard to the Go backend
- verify operator-visible mode banner
- verify no simulation warning when the backend is real
- verify stale/no-real-drone warnings appear when expected

Acceptance criteria:
- backend mode is visible
- simulation is clearly flagged if present
- real-data mode is distinguishable from playback/simulation
- stale telemetry warning appears when telemetry stops updating

Failure examples:
- backend mode hidden
- dashboard silently looks healthy with no real drones
- stale telemetry not visible

### 9. Command safety test

Goal:
- prove the safety/security layers block unsafe commands before flight

Procedure:
- attempt mission/waypoint behavior with localization lost
- attempt remote commands under unsafe security state
- trigger emergency-stop path
- verify indoor speed limiting under degraded visual confidence

Acceptance criteria:
- localization lost blocks mission command execution
- unsafe security state rejects remote command
- emergency land overrides all other commands
- missing required sensor blocks arming/autonomous flight

Failure examples:
- waypoint mission still allowed during localization loss
- remote command accepted under unsafe trust state
- emergency stop not dominant

## Bench Script Scope

`scripts/bench_check.py` is a fast preflight gate. It checks:

- required config files exist
- runtime mode is not `simulation`
- anchors are defined
- LiDAR is configured
- camera stream opens
- IMU device opens
- backend is reachable
- backend mode is visible and non-simulation for dashboard use

It prints per-check `PASS` or `FAIL` with reasons and ends with a bench verdict.

## Limits

This plan and script do **not** prove:

- good localization drift over long trajectories
- safe closed-loop flight behavior
- tethered recovery behavior
- multi-drone radio robustness
- safe free-flight operation

Those remain separate acceptance gates after bench validation.
