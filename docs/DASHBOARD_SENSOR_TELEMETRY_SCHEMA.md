# Dashboard Sensor Telemetry Schema

## Overview

The Go control-plane fleet snapshot now supports optional per-drone sensor payloads so the PySide6 dashboard can render real camera, IMU, LiDAR, TDOA/UWB, and replay status without faking live telemetry.

Allowed source tags:

- `real`
- `playback`
- `simulation`
- `unavailable`

## Fleet Telemetry JSON Shape

Each `POST /api/v1/telemetry` payload may include these optional nested fields inside one drone telemetry object:

```json
{
  "drone_id": 7,
  "cluster_id": "cluster-01",
  "role": "LEADER",
  "source": "real",
  "position": [1.2, 0.3, 2.8],
  "velocity": [0.0, 0.1, 0.0],
  "mission_state": "fly",
  "safety_state": "NORMAL",
  "safety_summary": "Nominal safety envelope",
  "localization_source": "vision-inertial",
  "localization_data_source": "real",
  "localization_state": "nominal",
  "localization_confidence": 0.91,
  "camera": {
    "status": "live",
    "fps": 29.7,
    "frame_age_ms": 41.0,
    "resolution": "1280x720",
    "dropped_frames": 1,
    "source": "real",
    "preview_url": "http://127.0.0.1:9090/preview/drone-7.jpg",
    "latest_frame_ref": "frame-0001842"
  },
  "imu": {
    "status": "live",
    "sample_rate_hz": 200.0,
    "last_sample_age_ms": 3.5,
    "accel": { "x": 0.02, "y": -0.01, "z": 9.81 },
    "gyro": { "x": 0.01, "y": 0.00, "z": -0.03 },
    "health": "good",
    "source": "real"
  },
  "lidar": {
    "status": "live",
    "packet_rate_hz": 10.0,
    "scan_age_ms": 62.0,
    "point_count": 3,
    "points_2d": [
      { "x": 1.0, "y": 0.5, "intensity": 0.8 },
      { "x": 1.4, "y": -0.2, "intensity": 0.7 },
      { "x": 2.2, "y": 0.9, "intensity": 0.6 }
    ],
    "min_range_m": 0.3,
    "max_range_m": 20.0,
    "source": "real"
  },
  "tdoa": {
    "status": "live",
    "source": "real",
    "visible_anchor_count": 4,
    "anchors": [
      { "id": "A0", "x": 0.0, "y": 0.0, "z": 2.5, "visible": true, "last_seen_ms": 18.0 },
      { "id": "A1", "x": 8.0, "y": 0.0, "z": 2.5, "visible": true, "last_seen_ms": 17.0 }
    ],
    "estimated_position": { "x": 1.2, "y": 0.3, "z": 2.8 },
    "calibration_warning": ""
  },
  "replay": {
    "status": "unavailable",
    "active": false,
    "file_name": "",
    "progress": 0.0,
    "current_time": 0.0,
    "confidence_series": [],
    "source": "unavailable"
  }
}
```

## Example Playback Payload

```json
{
  "drone_id": 42,
  "source": "playback",
  "localization_data_source": "playback",
  "camera": {
    "status": "playback",
    "fps": 24.0,
    "frame_age_ms": 85.0,
    "resolution": "1280x720",
    "dropped_frames": 0,
    "source": "playback",
    "latest_frame_ref": "log-frame-42"
  },
  "replay": {
    "status": "playback",
    "active": true,
    "file_name": "session-42.log",
    "progress": 0.42,
    "current_time": 12.5,
    "confidence_series": [0.91, 0.89, 0.86],
    "source": "playback"
  }
}
```

## Example Simulation Payload

```json
{
  "drone_id": 3,
  "source": "simulation",
  "localization_data_source": "simulation",
  "camera": {
    "status": "simulation",
    "source": "simulation",
    "latest_frame_ref": "sim-frame-3"
  },
  "imu": {
    "status": "simulation",
    "sample_rate_hz": 100.0,
    "last_sample_age_ms": 10.0,
    "accel": { "x": 0.0, "y": 0.0, "z": 9.81 },
    "gyro": { "x": 0.0, "y": 0.0, "z": 0.0 },
    "health": "simulation",
    "source": "simulation"
  }
}
```

## Fleet Snapshot Rules

- `GET /api/v1/fleet` returns the same nested sensor objects per drone after sanitization.
- Oversized arrays are truncated safely before storage.
- Unknown extra fields are rejected by backend JSON decoding.
- Sensor `source` values are preserved when valid and normalized to the allowed source-tag set.

Backend validation notes:

- `scripts/telemetry_smoke_test.py` posts a simulation-tagged payload and verifies the schema round-trips through `POST /api/v1/telemetry` and `GET /api/v1/fleet`.
- the smoke test also verifies the LiDAR truncation rule by posting more than `256` 2D points and checking the stored snapshot is capped.
- `scripts/production_telemetry_smoke_test.py` verifies the production backend keeps `backend_mode=production`, does not seed fake simulation drones, and preserves dashboard-visible fields even when sensors are marked `source=unavailable`.

## Payload Size Limits

- `lidar.points_2d`: maximum `256` points
- `tdoa.anchors`: maximum `32` anchors
- `replay.confidence_series`: maximum `256` samples
- `camera.preview_url`: maximum `512` characters
- `camera.latest_frame_ref`: maximum `256` characters
- request body: still capped by backend request reader limit

## Dashboard Rendering Rules

- `real` data may be rendered as live operational telemetry.
- `playback` data must remain visibly marked as playback, not live flight.
- `simulation` data must remain visibly marked as simulation/demo.
- `unavailable` must show disconnected or unavailable states.
- Camera panels may display `preview_url` or `latest_frame_ref`, but not invent missing frames.
- Replay panels may render confidence trends only when replay payload exists.

## Why Raw Video Frames Are Not Stored In Fleet Snapshot

Raw frames are intentionally excluded from the fleet snapshot because they:

- are too large for frequent control-plane telemetry polling
- can overload backend memory and JSON responses
- are better served from a dedicated preview/stream endpoint
- would blur the separation between control telemetry and media transport

Use `preview_url` or `latest_frame_ref` to point the dashboard to an external preview/stream system instead.

## Local Validation And Readiness

Recommended local validation flow:

```powershell
python scripts/local_validate.py
python scripts/telemetry_smoke_test.py --backend-url http://127.0.0.1:8080
python scripts/production_telemetry_smoke_test.py --backend-url http://127.0.0.1:8080
```

For a production-mode bench demo, start the backend with:

```powershell
$env:DRONE_BACKEND_MODE="production"
$env:DRONE_BACKEND_SIMULATION_ENABLED="false"
go run ./cmd/control-plane
```

Current readiness level:

- dashboard-compatible telemetry schema: ready for local validation
- backend schema round-trip smoke coverage: ready
- real sensor truthfulness: ready for bench/demo use because simulation and unavailable sources remain visibly labeled
- real flight readiness: not implied by this schema validation alone
