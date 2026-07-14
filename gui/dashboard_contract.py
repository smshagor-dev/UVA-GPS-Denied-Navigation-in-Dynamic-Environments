"""Headless-safe dashboard data contracts and parser helpers."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any


def safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def safe_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def safe_text(value: Any, default: str = "") -> str:
    if value is None:
        return default
    text = str(value).strip()
    return text if text else default


def safe_dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def safe_list(value: Any) -> list[Any]:
    return value if isinstance(value, list) else []


@dataclass(slots=True)
class DroneState:
    drone_id: int
    cluster_id: str
    role: str
    mission_state: str
    position: tuple[float, float, float]
    velocity: tuple[float, float, float]
    drift_m: float
    battery_pct: float
    connectivity: str
    reachable: bool
    rssi_dbm: float
    cpu_temp_c: float
    gpu_load_pct: float
    commanded_altitude_m: float = 0.0
    commanded_speed_mps: float = 0.0
    manual_target_position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    manual_control_active: bool = False
    source: str = "real"
    stale: bool = False
    attitude_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    thrust_vector: tuple[float, float, float] = (0.0, 0.0, 9.81)
    localization_source: str = "vision-inertial"
    localization_data_source: str = "unavailable"
    localization_state: str = "nominal"
    localization_confidence: float = 1.0
    tdoa_confidence: float = 0.0
    confidence_trend: float = 0.0
    relocalization_count: int = 0
    visible_anchor_count: int = 0
    occupancy_ratio: float = 0.0
    sync_confidence: float = 1.0
    imu_camera_offset_ms: float = 0.0
    peer_count: int = 0
    stale_peer_count: int = 0
    mesh_topology_mode: str = "adaptive_mesh"
    local_consensus_state: str = "single_node"
    local_consensus_epoch: int = 0
    peer_latency_ms: float = 0.0
    mesh_bandwidth_kbps: float = 0.0
    edge_serialization_mode: str = "json"
    edge_average_packet_size_bytes: float = 0.0
    edge_bandwidth_savings_estimate_pct: float = 0.0
    edge_packet_encode_latency_us: float = 0.0
    disconnected_operation: bool = False
    edge_health_status: str = "nominal"
    edge_autonomy_state: str = "backend_assisted"
    edge_inference_status: str = "idle"
    edge_inference_fps: float = 0.0
    edge_inference_confidence: float = 0.0
    local_obstacle_count: int = 0
    shared_obstacle_count: int = 0
    peer_clock_offset_ms: float = 0.0
    anchor_visibility_ratio: float = 0.0
    tdoa_weight: float = 0.0
    planned_waypoint_count: int = 0
    last_relocalized_keyframe: int = 0
    security_state: str = "TRUSTED"
    security_summary: str = "All trust signals nominal"
    security_transition_reason: str = "initial-trust"
    remote_command_allowed: bool = True
    telemetry_uplink_allowed: bool = True
    link_integrity_score: float = 1.0
    trust_epoch: int = 1
    last_auth_failure_at_s: float = 0.0
    tamper_score: float = 0.0
    firmware_measurement: str = "lab-local-build"
    firmware_version: str = "0.0.0"
    secure_boot_state: str = "LAB_BOOT"
    boot_trust_summary: str = "Lab boot trust bypassed"
    rollback_counter: int = 0
    maintenance_mode: bool = False
    update_channel_state: str = "idle"
    last_remote_command_status: str = "no remote command"
    health_flags: list[str] = field(default_factory=list)
    camera_status: str = "unavailable"
    camera: dict[str, Any] = field(default_factory=dict)
    imu_status: str = "unavailable"
    imu: dict[str, Any] = field(default_factory=dict)
    lidar_status: str = "unavailable"
    lidar: dict[str, Any] = field(default_factory=dict)
    tdoa_status: str = "unavailable"
    tdoa: dict[str, Any] = field(default_factory=dict)
    replay_status: str = "unavailable"
    replay: dict[str, Any] = field(default_factory=dict)
    motor_health: float = 1.0
    leadership_score: float = 0.0
    election_ready: bool = True
    timestamp: float = field(default_factory=time.time)


@dataclass(slots=True)
class DashboardSnapshot:
    states: list[DroneState]
    backend_mode: str
    leader_id: int | None
    avg_latency_ms: float
    packet_loss_pct: float
    cpu_temp_c: float
    gpu_load_pct: float
    avg_peer_latency_ms: float = 0.0
    avg_mesh_bandwidth_kbps: float = 0.0
    mesh_topology_mode: str = "adaptive_mesh"
    election_state: str = "stable"
    clusters: list[dict[str, Any]] = field(default_factory=list)
    critical_alerts: int = 0
    health: dict[str, Any] = field(default_factory=dict)
    missions: list[dict[str, Any]] = field(default_factory=list)
    events: list[dict[str, Any]] = field(default_factory=list)
    services: list[str] = field(default_factory=list)
    simulation_enabled: bool = False
    real_drone_count: int = 0
    stale_drone_count: int = 0
    timestamp: float = field(default_factory=time.time)


def mission_start_allowed(snapshot: DashboardSnapshot) -> bool:
    states = snapshot.states
    if not states:
        return False
    return (
        snapshot.real_drone_count > 0
        and snapshot.stale_drone_count == 0
        and all(state.localization_confidence >= 0.65 for state in states)
        and all(state.localization_data_source == "real" for state in states)
        and all(state.remote_command_allowed for state in states)
    )


def parse_sensor_source(payload: dict[str, Any], fallback: str = "unavailable") -> str:
    return safe_text(payload.get("source", fallback), fallback)


def parse_camera_payload(payload: Any) -> tuple[str, dict[str, Any]]:
    data = safe_dict(payload)
    status = safe_text(data.get("status", "unavailable"), "unavailable")
    return status, {
        "status": status,
        "fps": safe_float(data.get("fps", 0.0)),
        "frame_age_ms": safe_float(data.get("frame_age_ms", 0.0)),
        "resolution": safe_text(data.get("resolution", "N/A"), "N/A"),
        "dropped_frames": safe_int(data.get("dropped_frames", 0), 0),
        "source": parse_sensor_source(data),
        "preview_url": safe_text(data.get("preview_url", ""), ""),
        "latest_frame_ref": safe_text(data.get("latest_frame_ref", ""), ""),
    }


def parse_imu_payload(payload: Any) -> tuple[str, dict[str, Any]]:
    data = safe_dict(payload)
    accel = safe_dict(data.get("accel"))
    gyro = safe_dict(data.get("gyro"))
    status = safe_text(data.get("status", "unavailable"), "unavailable")
    return status, {
        "status": status,
        "sample_rate_hz": safe_float(data.get("sample_rate_hz", 0.0)),
        "last_sample_age_ms": safe_float(data.get("last_sample_age_ms", 0.0)),
        "accel": (
            safe_float(accel.get("x", 0.0)),
            safe_float(accel.get("y", 0.0)),
            safe_float(accel.get("z", 0.0)),
        ),
        "gyro": (
            safe_float(gyro.get("x", 0.0)),
            safe_float(gyro.get("y", 0.0)),
            safe_float(gyro.get("z", 0.0)),
        ),
        "health": safe_text(data.get("health", "unknown"), "unknown"),
        "source": parse_sensor_source(data),
    }


def parse_lidar_payload(payload: Any) -> tuple[str, dict[str, Any]]:
    data = safe_dict(payload)
    points = [
        (
            safe_float(item.get("x", 0.0)),
            safe_float(item.get("y", 0.0)),
            safe_float(item.get("intensity", 0.0)),
        )
        for item in safe_list(data.get("points_2d"))
        if isinstance(item, dict)
    ]
    status = safe_text(data.get("status", "unavailable"), "unavailable")
    return status, {
        "status": status,
        "packet_rate_hz": safe_float(data.get("packet_rate_hz", 0.0)),
        "scan_age_ms": safe_float(data.get("scan_age_ms", 0.0)),
        "point_count": safe_int(data.get("point_count", len(points)), len(points)),
        "points_2d": points,
        "min_range_m": safe_float(data.get("min_range_m", 0.0)),
        "max_range_m": safe_float(data.get("max_range_m", 0.0)),
        "source": parse_sensor_source(data),
    }


def parse_tdoa_payload(payload: Any) -> tuple[str, dict[str, Any]]:
    data = safe_dict(payload)
    anchors = [
        {
            "id": safe_text(item.get("id", "A?"), "A?"),
            "x": safe_float(item.get("x", 0.0)),
            "y": safe_float(item.get("y", 0.0)),
            "z": safe_float(item.get("z", 0.0)),
            "visible": bool(item.get("visible", False)),
            "last_seen_ms": safe_float(item.get("last_seen_ms", 0.0)),
        }
        for item in safe_list(data.get("anchors"))
        if isinstance(item, dict)
    ]
    estimated = safe_dict(data.get("estimated_position"))
    status = safe_text(data.get("status", "unavailable"), "unavailable")
    return status, {
        "status": status,
        "source": parse_sensor_source(data),
        "visible_anchor_count": safe_int(
            data.get("visible_anchor_count", len([a for a in anchors if a["visible"]])),
            0,
        ),
        "anchors": anchors,
        "estimated_position": (
            safe_float(estimated.get("x", 0.0)),
            safe_float(estimated.get("y", 0.0)),
            safe_float(estimated.get("z", 0.0)),
        ),
        "calibration_warning": safe_text(data.get("calibration_warning", ""), ""),
    }


def parse_replay_payload(payload: Any) -> tuple[str, dict[str, Any]]:
    data = safe_dict(payload)
    series = [
        safe_float(value, 0.0) for value in safe_list(data.get("confidence_series"))
    ]
    status = safe_text(data.get("status", "unavailable"), "unavailable")
    return status, {
        "status": status,
        "active": bool(data.get("active", False)),
        "file_name": safe_text(data.get("file_name", ""), ""),
        "progress": safe_float(data.get("progress", 0.0)),
        "current_time": safe_float(data.get("current_time", 0.0)),
        "confidence_series": series,
        "source": parse_sensor_source(data),
    }
