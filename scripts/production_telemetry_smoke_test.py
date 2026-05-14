#!/usr/bin/env python3
"""Verify production-mode backend telemetry handling without fake hardware data."""

from __future__ import annotations

import argparse
import json
import time
import urllib.error
import urllib.request


def request_json(method: str, url: str, payload: dict | None = None) -> tuple[int, dict]:
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


def looks_like_seeded_demo_fleet(drones: list[dict]) -> bool:
    if len(drones) < 5:
        return False
    drone_ids = {drone.get("drone_id") for drone in drones}
    expected_ids = {1, 2, 3, 4, 5}
    if not expected_ids.issubset(drone_ids):
        return False
    seeded = [
        drone
        for drone in drones
        if drone.get("drone_id") in expected_ids
        and drone.get("cluster_id") == "cluster-01"
        and drone.get("source") == "simulation"
    ]
    return len(seeded) == 5


def production_unavailable_payload() -> dict:
    drone_id = int(time.time()) % 100000 + 9000
    return {
        "drone_id": drone_id,
        "cluster_id": "cluster-bench-prod",
        "role": "FOLLOWER",
        "source": "unavailable",
        "connectivity": "Mesh",
        "reachable": False,
        "position": [0.0, 0.0, 0.0],
        "velocity": [0.0, 0.0, 0.0],
        "attitude_rpy": [0.0, 0.0, 0.0],
        "thrust_vector": [0.0, 0.0, 9.81],
        "mission_state": "standby",
        "drift_m": 0.0,
        "battery_pct": 100.0,
        "rssi_dbm": -90.0,
        "cpu_temp_c": 35.0,
        "gpu_load_pct": 0.0,
        "localization_source": "vision-inertial",
        "localization_data_source": "unavailable",
        "localization_state": "lost",
        "localization_confidence": 0.0,
        "tdoa_confidence": 0.0,
        "confidence_trend": 0.0,
        "visible_anchor_count": 0,
        "occupancy_ratio": 0.0,
        "sync_confidence": 0.0,
        "imu_camera_offset_ms": 0.0,
        "security_state": "TRUSTED",
        "security_summary": "Production smoke test unavailable payload",
        "safety_state": "HOLD",
        "safety_summary": "No real sensors attached during production smoke test",
        "remote_command_allowed": False,
        "telemetry_uplink_allowed": True,
        "link_integrity_score": 0.0,
        "firmware_measurement": "smoke-test-prod",
        "firmware_version": "2.0.0-smoke",
        "secure_boot_state": "LAB_BOOT",
        "boot_trust_summary": "Production unavailable-source smoke payload",
        "rollback_counter": 1,
        "maintenance_mode": False,
        "update_channel_state": "idle",
        "last_remote_command_status": "not-issued",
        "health_flags": ["no_real_sensors_attached"],
        "camera": {
            "status": "unavailable",
            "fps": 0.0,
            "frame_age_ms": 0.0,
            "resolution": "",
            "dropped_frames": 0,
            "source": "unavailable",
            "latest_frame_ref": "",
        },
        "imu": {
            "status": "unavailable",
            "sample_rate_hz": 0.0,
            "last_sample_age_ms": 0.0,
            "accel": {"x": 0.0, "y": 0.0, "z": 0.0},
            "gyro": {"x": 0.0, "y": 0.0, "z": 0.0},
            "health": "unavailable",
            "source": "unavailable",
        },
        "lidar": {
            "status": "unavailable",
            "packet_rate_hz": 0.0,
            "scan_age_ms": 0.0,
            "point_count": 0,
            "points_2d": [],
            "min_range_m": 0.0,
            "max_range_m": 0.0,
            "source": "unavailable",
        },
        "tdoa": {
            "status": "unavailable",
            "source": "unavailable",
            "visible_anchor_count": 0,
            "anchors": [],
            "estimated_position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "calibration_warning": "no anchors connected in production smoke test",
        },
        "replay": {
            "status": "unavailable",
            "active": False,
            "file_name": "",
            "progress": 0.0,
            "current_time": 0.0,
            "confidence_series": [],
            "source": "unavailable",
        },
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend-url", required=True, help="Base Go control-plane URL, for example http://127.0.0.1:8080")
    args = parser.parse_args()

    base_url = args.backend_url.rstrip("/")

    status, before = request_json("GET", f"{base_url}/api/v1/fleet")
    if status != 200:
        return fail(f"initial fleet GET failed: status={status} body={before}")
    if before.get("backend_mode") != "production":
        return fail(f"expected backend_mode=production before posting telemetry, got {before.get('backend_mode')!r}")
    if before.get("simulation_enabled") is not False:
        return fail(f"expected simulation_enabled=false in production mode, got {before.get('simulation_enabled')!r}")
    if looks_like_seeded_demo_fleet(before.get("drones", [])):
        return fail("production backend already has seeded simulation drones; restart it in production mode with simulation disabled")

    payload = production_unavailable_payload()
    status, post_body = request_json("POST", f"{base_url}/api/v1/telemetry", payload)
    if status != 202:
        return fail(f"telemetry POST was not accepted: status={status} body={post_body}")

    status, after = request_json("GET", f"{base_url}/api/v1/fleet")
    if status != 200:
        return fail(f"fleet GET after telemetry failed: status={status} body={after}")
    if after.get("backend_mode") != "production":
        return fail(f"expected backend_mode=production after telemetry, got {after.get('backend_mode')!r}")
    if after.get("simulation_enabled") is not False:
        return fail(f"expected simulation_enabled=false after telemetry, got {after.get('simulation_enabled')!r}")
    if looks_like_seeded_demo_fleet(after.get("drones", [])):
        return fail("production backend contains the seeded simulation demo fleet after POST")

    target = next((drone for drone in after.get("drones", []) if drone.get("drone_id") == payload["drone_id"]), None)
    if target is None:
        return fail(f"posted production unavailable payload for drone_id={payload['drone_id']} did not appear in fleet snapshot")
    if target.get("source") != "unavailable":
        return fail(f"expected top-level source=unavailable, got {target.get('source')!r}")
    if after.get("real_drone_count") != 0:
        return fail(f"expected real_drone_count=0 for unavailable-only payload, got {after.get('real_drone_count')!r}")
    if "stale_drone_count" not in after:
        return fail("fleet snapshot did not expose stale_drone_count")

    required_fields = [
        "backend_mode",
        "simulation_enabled",
        "real_drone_count",
        "stale_drone_count",
        "drones",
    ]
    missing_snapshot_fields = [field for field in required_fields if field not in after]
    if missing_snapshot_fields:
        return fail(f"fleet snapshot is missing dashboard-visible fields: {missing_snapshot_fields}")

    required_drone_fields = [
        "source",
        "localization_data_source",
        "camera",
        "imu",
        "lidar",
        "tdoa",
        "replay",
    ]
    missing_drone_fields = [field for field in required_drone_fields if field not in target]
    if missing_drone_fields:
        return fail(f"fleet drone payload is missing dashboard-visible fields: {missing_drone_fields}")

    print("PASS: production telemetry unavailable-source path behaved as expected.")
    print(f"backend_mode={after.get('backend_mode')} simulation_enabled={after.get('simulation_enabled')} real_drone_count={after.get('real_drone_count')}")
    print(f"drone_id={payload['drone_id']} source={target.get('source')} localization_data_source={target.get('localization_data_source')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
