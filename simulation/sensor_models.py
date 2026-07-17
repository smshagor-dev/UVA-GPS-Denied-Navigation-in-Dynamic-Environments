from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from simulation.vehicle_state import VehicleState


@dataclass
class SensorFrame:
    imu: dict[str, Any]
    gps: dict[str, Any]
    camera: dict[str, Any]
    vio: dict[str, Any]
    lidar: dict[str, Any]
    telemetry_link: dict[str, Any]
    command_channel: dict[str, Any]


def generate_sensor_frame(state: VehicleState, step_index: int, dt_s: float) -> SensorFrame:
    phase = step_index * dt_s
    yaw_rate = 0.03 * math.sin(phase)
    accel_x = 0.05 * math.cos(phase)
    accel_y = 0.03 * math.sin(phase * 0.5)
    imu = {
        "status": state.sensor_status["imu"],
        "sample_rate_hz": 200.0,
        "accel": {"x": accel_x, "y": accel_y, "z": 9.81},
        "gyro": {"x": 0.0, "y": 0.0, "z": yaw_rate},
        "age_ms": 5.0,
    }
    gps = {
        "status": state.sensor_status["gps"],
        "position": {"x": state.x, "y": state.y, "z": state.z},
        "fix_type": "3D" if state.sensor_status["gps"] == "live" else "unavailable",
        "age_ms": 40.0,
    }
    camera = {
        "status": state.sensor_status["camera"],
        "fps": 30.0,
        "frame_age_ms": 18.0,
        "dropped_frames": 0 if state.sensor_status["camera"] == "live" else 12,
    }
    vio = {
        "status": "active" if state.sensor_status["camera"] == "live" else "degraded",
        "track_count": 120 if state.sensor_status["camera"] == "live" else 22,
        "confidence": state.localization_confidence,
    }
    lidar = {
        "status": state.sensor_status["lidar"],
        "packet_rate_hz": 10.0,
        "scan_age_ms": 12.0 if state.sensor_status["lidar"] == "live" else 250.0,
        "point_count": 32 if state.sensor_status["lidar"] == "live" else 0,
    }
    telemetry_link = {
        "status": state.sensor_status["telemetry_link"],
        "delay_ms": state.telemetry_delay_ms,
        "packet_loss_pct": state.packet_loss_pct,
    }
    command_channel = {
        "status": state.sensor_status["command_channel"],
        "accepted": state.remote_command_allowed,
    }
    return SensorFrame(
        imu=imu,
        gps=gps,
        camera=camera,
        vio=vio,
        lidar=lidar,
        telemetry_link=telemetry_link,
        command_channel=command_channel,
    )

