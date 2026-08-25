# Onboard Sensor Telemetry Publisher

## Purpose

The onboard `drone_node` publishes per-drone telemetry to the Go control-plane using the same nested sensor schema consumed by the PySide6 dashboard.

This publisher is designed to:

- preserve `real` / `playback` / `simulation` / `unavailable` source tags
- avoid sending raw image frames
- cap LiDAR payload size before uplink
- continue flying safely if the backend is slow or unavailable

## Core Payload Fields

Top-level payload includes:

- `drone_id`
- `source`
- `timestamp`
- `cluster_id`
- `role`
- `position`
- `velocity`
- `attitude_rpy`
- `thrust_vector`
- `localization_source`
- `localization_data_source`
- `localization_state`
- `localization_confidence`
- `safety_state`
- `safety_summary`
- `security_state`
- `security_summary`
- `battery_pct`
- `rssi_dbm`
- `cpu_temp_c`
- `gpu_load_pct`

## Nested Sensor Payloads

### Camera

- `status`
- `fps`
- `frame_age_ms`
- `resolution`
- `dropped_frames`
- `source`
- `latest_frame_ref`

Raw image bytes are not sent.

### IMU

- `status`
- `sample_rate_hz`
- `last_sample_age_ms`
- `accel`
- `gyro`
- `health`
- `source`

### LiDAR

- `status`
- `packet_rate_hz`
- `scan_age_ms`
- `point_count`
- `points_2d`
- `min_range_m`
- `max_range_m`
- `source`

`points_2d` is capped to `256` points before uplink.

### TDOA / UWB

- `status`
- `source`
- `visible_anchor_count`
- `anchors`
- `estimated_position`
- `calibration_warning`

### Replay

- `status`
- `active`
- `file_name`
- `progress`
- `current_time`
- `confidence_series`
- `source`

Replay is only marked active when playback telemetry is actually in use.

## Source Tagging Rules

- `real`: only when the actual device/stream/measurement is active
- `playback`: only when CSV/log playback is the active source
- `simulation`: only in simulation mode or explicit simulated sensor path
- `unavailable`: when no valid data exists

The publisher must never silently upgrade simulated or missing telemetry to `real`.

## Publish Interval

- controlled by `DRONE_BACKEND_TELEMETRY_INTERVAL_MS`
- default loop integration keeps publish attempts periodic but non-blocking to flight logic

## Backend Failure Behavior

- publish runs in a background async task
- the flight/control loop continues even if the backend is slow
- failures are logged through telemetry client status and warning logs
- retries remain backoff-controlled by `ControlPlaneTelemetryClient`

## Local Build And Validation

Use the repo-wide validator to prove the Python, Go, and C++ paths are buildable together:

```powershell
python scripts/local_validate.py
```

That script now hard-fails if `cmake` is missing, so the C++ path is explicitly validated instead of silently skipped.

To build manually:

```powershell
cmake -S . -B build-local -DBUILD_TESTS=ON
cmake --build build-local --config Release --target drone_node
ctest --test-dir build-local --output-on-failure
```

See [LOCAL_BUILD_AND_BENCH_DEMO_GUIDE.md](LOCAL_BUILD_AND_BENCH_DEMO_GUIDE.md) for Windows and Linux dependency setup.

## Telemetry Smoke Tests

Simulation-tagged schema smoke test:

```powershell
python scripts/telemetry_smoke_test.py --backend-url http://127.0.0.1:8080
```

Production-mode unavailable-source smoke test:

```powershell
python scripts/production_telemetry_smoke_test.py --backend-url http://127.0.0.1:8080
```

Recommended backend startup for both:

```powershell
$env:DRONE_BACKEND_MODE="production"
$env:DRONE_BACKEND_SIMULATION_ENABLED="false"
go run ./cmd/control-plane
```

What these smoke tests prove:

- dashboard-compatible nested telemetry can be posted end-to-end
- simulation payloads stay marked `source=simulation`
- unavailable production payloads stay marked `source=unavailable`
- production backend does not seed a fake simulation fleet
- LiDAR payload capping is enforced by the backend snapshot path

## Safety Note

Backend telemetry is observability only.
Loss of backend connectivity must not stop the onboard safety manager, autonomy loop, or local failsafe behavior.

## Example Payload

```json
{
  "drone_id": 1,
  "source": "real",
  "localization_data_source": "real",
  "safety_state": "NORMAL",
  "camera": {
    "status": "live",
    "fps": 29.5,
    "frame_age_ms": 42.0,
    "resolution": "1280x720",
    "dropped_frames": 1,
    "source": "real",
    "latest_frame_ref": "frame-1842"
  },
  "imu": {
    "status": "live",
    "sample_rate_hz": 200.0,
    "last_sample_age_ms": 3.0,
    "accel": { "x": 0.02, "y": -0.01, "z": 9.81 },
    "gyro": { "x": 0.01, "y": 0.00, "z": -0.03 },
    "health": "good",
    "source": "real"
  },
  "lidar": {
    "status": "live",
    "packet_rate_hz": 10.0,
    "scan_age_ms": 61.0,
    "point_count": 3,
    "points_2d": [
      { "x": 1.0, "y": 0.5, "intensity": 0.8 }
    ],
    "min_range_m": 0.3,
    "max_range_m": 20.0,
    "source": "real"
  }
}
```

## Current Readiness Level

- onboard telemetry publisher schema: ready for local backend validation
- cross-language validation flow: ready through `scripts/local_validate.py`
- bench-demo backend smoke coverage: ready
- target-hardware production validation: still pending real Linux flight-computer and sensor evidence
