#!/usr/bin/env python3
"""Verify simulation-tagged telemetry appears in the Go backend fleet snapshot."""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
MAX_LIDAR_POINTS = 256


def request_json(
    method: str, url: str, payload: dict | None = None
) -> tuple[int, dict]:
    data = None
    headers = {"Accept": "application/json"}
    if payload is not None:
        data = json.dumps(payload).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(url, data=data, headers=headers, method=method)
    try:
        with urllib.request.urlopen(req, timeout=5) as response:
            body = response.read().decode("utf-8")
            return response.status, json.loads(body) if body else {}
    except urllib.error.HTTPError as exc:
        body = exc.read().decode("utf-8", errors="replace")
        parsed = {}
        if body:
            try:
                parsed = json.loads(body)
            except json.JSONDecodeError:
                parsed = {"raw": body}
        return exc.code, parsed


def fail(message: str) -> int:
    print(f"FAIL: {message}")
    return 1


def simulation_payload() -> dict:
    drone_id = int(time.time()) % 100000 + 7000
    lidar_points = [
        {"x": float(idx) * 0.02, "y": float(idx) * -0.01, "intensity": 0.5}
        for idx in range(MAX_LIDAR_POINTS + 12)
    ]
    return {
        "drone_id": drone_id,
        "cluster_id": "cluster-bench-demo",
        "role": "LEADER",
        "source": "simulation",
        "connectivity": "Mesh",
        "reachable": True,
        "position": [1.2, -0.4, 1.8],
        "velocity": [0.0, 0.1, 0.0],
        "attitude_rpy": [0.01, -0.02, 0.3],
        "thrust_vector": [0.0, 0.0, 9.81],
        "mission_state": "bench-demo",
        "drift_m": 0.04,
        "battery_pct": 88.0,
        "rssi_dbm": -49.0,
        "cpu_temp_c": 51.0,
        "gpu_load_pct": 21.0,
        "localization_source": "vision-depth-fused",
        "localization_data_source": "simulation",
        "localization_state": "nominal",
        "localization_confidence": 0.93,
        "tdoa_confidence": 0.71,
        "confidence_trend": 0.01,
        "visible_anchor_count": 4,
        "occupancy_ratio": 0.14,
        "sync_confidence": 0.97,
        "imu_camera_offset_ms": 1.8,
        "security_state": "TRUSTED",
        "security_summary": "Synthetic bench-demo payload",
        "safety_state": "NORMAL",
        "safety_summary": "Simulation-only smoke test payload",
        "remote_command_allowed": True,
        "telemetry_uplink_allowed": True,
        "link_integrity_score": 0.95,
        "firmware_measurement": "smoke-test-sim",
        "firmware_version": "2.0.0-smoke",
        "secure_boot_state": "LAB_BOOT",
        "boot_trust_summary": "Synthetic smoke-test payload",
        "rollback_counter": 1,
        "maintenance_mode": False,
        "update_channel_state": "idle",
        "last_remote_command_status": "not-issued",
        "health_flags": ["synthetic_payload"],
        "camera": {
            "status": "simulation",
            "fps": 30.0,
            "frame_age_ms": 20.0,
            "resolution": "1280x720",
            "dropped_frames": 0,
            "source": "simulation",
            "preview_url": "http://127.0.0.1:9090/preview/smoke-sim.jpg",
            "latest_frame_ref": "smoke-sim-frame",
        },
        "imu": {
            "status": "simulation",
            "sample_rate_hz": 200.0,
            "last_sample_age_ms": 4.0,
            "accel": {"x": 0.0, "y": 0.0, "z": 9.81},
            "gyro": {"x": 0.01, "y": 0.0, "z": -0.01},
            "health": "simulation",
            "source": "simulation",
        },
        "lidar": {
            "status": "simulation",
            "packet_rate_hz": 10.0,
            "scan_age_ms": 18.0,
            "point_count": len(lidar_points),
            "points_2d": lidar_points,
            "min_range_m": 0.3,
            "max_range_m": 20.0,
            "source": "simulation",
        },
        "tdoa": {
            "status": "simulation",
            "source": "simulation",
            "visible_anchor_count": 4,
            "anchors": [
                {
                    "id": "A0",
                    "x": 0.0,
                    "y": 0.0,
                    "z": 2.5,
                    "visible": True,
                    "last_seen_ms": 9.0,
                },
                {
                    "id": "A1",
                    "x": 8.0,
                    "y": 0.0,
                    "z": 2.5,
                    "visible": True,
                    "last_seen_ms": 10.0,
                },
                {
                    "id": "A2",
                    "x": 0.0,
                    "y": 8.0,
                    "z": 2.5,
                    "visible": True,
                    "last_seen_ms": 11.0,
                },
                {
                    "id": "A3",
                    "x": 8.0,
                    "y": 8.0,
                    "z": 2.5,
                    "visible": True,
                    "last_seen_ms": 12.0,
                },
            ],
            "estimated_position": {"x": 1.2, "y": -0.4, "z": 1.8},
            "calibration_warning": "simulation smoke-test payload",
        },
        "replay": {
            "status": "simulation",
            "active": False,
            "file_name": "",
            "progress": 0.0,
            "current_time": 0.0,
            "confidence_series": [0.95, 0.96, 0.94],
            "source": "simulation",
        },
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--backend-url",
        required=True,
        help="Base Go control-plane URL, for example http://127.0.0.1:8080",
    )
    args = parser.parse_args()

    base_url = args.backend_url.rstrip("/")
    payload = simulation_payload()

    status, post_body = request_json("POST", f"{base_url}/api/v1/telemetry", payload)
    if status != 202:
        return fail(
            f"telemetry POST was not accepted: status={status} body={post_body}"
        )

    status, fleet = request_json("GET", f"{base_url}/api/v1/fleet")
    if status != 200:
        return fail(f"fleet GET failed: status={status} body={fleet}")

    drones = fleet.get("drones", [])
    target = next(
        (drone for drone in drones if drone.get("drone_id") == payload["drone_id"]),
        None,
    )
    if target is None:
        return fail(
            f"posted simulation telemetry for drone_id={payload['drone_id']} did not appear in fleet snapshot"
        )
    if target.get("source") != "simulation":
        return fail(
            f"expected top-level source=simulation, got {target.get('source')!r}"
        )
    if target.get("source") == "real":
        return fail("simulation smoke payload was mislabeled as real")
    if target.get("localization_data_source") != "simulation":
        return fail(
            f"expected localization_data_source=simulation, got {target.get('localization_data_source')!r}"
        )
    if target.get("camera", {}).get("source") != "simulation":
        return fail("camera source did not stay marked as simulation")
    if target.get("imu", {}).get("source") != "simulation":
        return fail("imu source did not stay marked as simulation")
    if target.get("lidar", {}).get("source") != "simulation":
        return fail("lidar source did not stay marked as simulation")
    if target.get("tdoa", {}).get("source") != "simulation":
        return fail("tdoa source did not stay marked as simulation")
    if target.get("replay", {}).get("source") != "simulation":
        return fail("replay source did not stay marked as simulation")

    lidar_points = target.get("lidar", {}).get("points_2d", [])
    if len(lidar_points) != MAX_LIDAR_POINTS:
        return fail(
            f"expected LiDAR points to be capped at {MAX_LIDAR_POINTS}, got {len(lidar_points)}"
        )
    if target.get("lidar", {}).get("point_count") != MAX_LIDAR_POINTS:
        return fail(
            f"expected sanitized LiDAR point_count={MAX_LIDAR_POINTS}, got {target.get('lidar', {}).get('point_count')}"
        )
    if fleet.get("real_drone_count", -1) < 0:
        return fail("fleet snapshot did not include real_drone_count")

    print("PASS: simulation telemetry round-trip succeeded.")
    print(
        f"backend_mode={fleet.get('backend_mode')} simulation_enabled={fleet.get('simulation_enabled')} real_drone_count={fleet.get('real_drone_count')}"
    )
    print(
        f"drone_id={payload['drone_id']} source={target.get('source')} lidar_points={len(lidar_points)}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
