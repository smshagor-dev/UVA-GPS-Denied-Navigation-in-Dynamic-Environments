from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import phase6_performance_suite as suite
from simulation.fault_injector import ScheduledFault
from simulation.sensor_models import generate_sensor_frame
from simulation.vehicle_state import VehicleState


@dataclass
class ScenarioSpec:
    name: str
    description: str
    steps: int
    dt_s: float
    vehicle: VehicleState
    faults: list[ScheduledFault]
    use_backend: bool = True
    warmup_steps: int = 1


def utc_now() -> str:
    return datetime.now(UTC).isoformat().replace("+00:00", "Z")


def state_to_payload(state: VehicleState, timestamp_text: str) -> dict[str, Any]:
    payload = suite.production_payload(state.drone_id)
    payload.update(
        {
            "cluster_id": state.cluster_id,
            "role": state.role,
            "source": "simulation",
            "position": [state.x, state.y, state.z],
            "velocity": [state.vx, state.vy, state.vz],
            "attitude_rpy": [0.0, 0.0, state.yaw],
            "mission_state": state.mission_state,
            "battery_pct": state.battery_pct,
            "localization_source": state.localization_source,
            "localization_data_source": state.localization_data_source,
            "localization_state": state.localization_state,
            "localization_confidence": state.localization_confidence,
            "tdoa_confidence": state.tdoa_confidence,
            "visible_anchor_count": state.visible_anchor_count,
            "sync_confidence": state.sync_confidence,
            "peer_count": state.peer_count,
            "stale_peer_count": state.stale_peer_count,
            "local_obstacle_count": state.local_obstacle_count,
            "shared_obstacle_count": state.shared_obstacle_count,
            "safety_state": state.safety_state,
            "safety_summary": state.safety_summary,
            "security_state": state.security_state,
            "remote_command_allowed": state.remote_command_allowed,
            "telemetry_uplink_allowed": state.telemetry_uplink_allowed,
            "timestamp": timestamp_text,
        }
    )
    payload["camera"]["status"] = state.sensor_status["camera"]
    payload["camera"]["source"] = "simulation"
    payload["imu"]["status"] = state.sensor_status["imu"]
    payload["imu"]["health"] = state.sensor_status["imu"]
    payload["imu"]["source"] = "simulation"
    payload["lidar"]["status"] = state.sensor_status["lidar"]
    payload["lidar"]["source"] = "simulation"
    payload["tdoa"]["status"] = "simulation"
    payload["tdoa"]["source"] = "simulation"
    payload["health_flags"] = [
        f"mission_state:{state.mission_state}",
        f"command_channel:{state.command_channel_state}",
        f"estimator:{state.estimator_health}",
    ]
    return payload


def _fleet_snapshot(base_url: str) -> dict[str, Any]:
    status, body = suite.request_json("GET", f"{base_url}/api/v1/fleet")
    if status != 200:
        raise RuntimeError(f"fleet GET failed: status={status} body={body}")
    return body


def _find_drone(snapshot: dict[str, Any], drone_id: int) -> dict[str, Any]:
    for drone in snapshot.get("drones", []):
        if drone.get("drone_id") == drone_id:
            return drone
    return {}


def _measure_endpoint_latency(
    base_url: str,
    method: str,
    path: str,
    payload: dict[str, Any] | None = None,
    *,
    accepted_statuses: tuple[int, ...] = (200, 202),
) -> tuple[dict[str, Any], float]:
    started = time.perf_counter()
    status, body = suite.request_json(method, f"{base_url}{path}", payload)
    elapsed_ms = (time.perf_counter() - started) * 1000.0
    if status not in accepted_statuses:
        raise RuntimeError(f"{method} {path} failed: status={status} body={body}")
    body["_http_status"] = status
    return body, elapsed_ms


def run_backend_scenario(spec: ScenarioSpec, base_url: str) -> dict[str, Any]:
    state = spec.vehicle
    faults_by_step = {fault.at_step: fault for fault in spec.faults}
    timeline: list[dict[str, Any]] = []
    latencies_ms: list[float] = []
    transitions: list[dict[str, Any]] = []
    active_fault = "none"
    recovery_step: int | None = None
    degraded_step: int | None = None
    last_safety_state = state.safety_state
    base_epoch = datetime.now(UTC).timestamp()
    for step_index in range(spec.steps):
        timestamp_text = datetime.fromtimestamp(base_epoch + (step_index * spec.dt_s), tz=UTC).isoformat().replace("+00:00", "Z")
        if step_index in faults_by_step:
            scheduled = faults_by_step[step_index]
            scheduled.apply(state)
            active_fault = scheduled.name
            timeline.append({"step": step_index, "event": "fault_applied", "fault": scheduled.name, "timestamp": timestamp_text})
            if state.safety_state != "NORMAL" and degraded_step is None:
                degraded_step = step_index
        state.advance(spec.dt_s)
        sensor_frame = generate_sensor_frame(state, step_index, spec.dt_s)
        payload = state_to_payload(state, timestamp_text)
        post_body, post_latency_ms = _measure_endpoint_latency(base_url, "POST", "/api/v1/telemetry", payload)
        _ = post_body
        snapshot, fleet_latency_ms = _measure_endpoint_latency(base_url, "GET", "/api/v1/fleet")
        drone = _find_drone(snapshot, state.drone_id)
        health, health_latency_ms = _measure_endpoint_latency(base_url, "GET", "/api/v1/health")
        readiness, readiness_latency_ms = _measure_endpoint_latency(
            base_url,
            "GET",
            "/api/v1/ready",
            accepted_statuses=(200, 503),
        )
        step_latency = post_latency_ms + fleet_latency_ms + health_latency_ms + readiness_latency_ms
        latencies_ms.append(step_latency)
        observed_safety_state = drone.get("safety_state", state.safety_state)
        if observed_safety_state != last_safety_state:
            transitions.append(
                {
                    "step": step_index,
                    "from": last_safety_state,
                    "to": observed_safety_state,
                    "timestamp": timestamp_text,
                }
            )
            last_safety_state = observed_safety_state
        if degraded_step is not None and observed_safety_state == "NORMAL" and recovery_step is None and active_fault == "recovery":
            recovery_step = step_index
        timeline.append(
            {
                "step": step_index,
                "timestamp": timestamp_text,
                "active_fault": active_fault,
                "input_state": {
                    "position": [state.x, state.y, state.z],
                    "localization_confidence": state.localization_confidence,
                    "telemetry_delay_ms": state.telemetry_delay_ms,
                    "packet_loss_pct": state.packet_loss_pct,
                    "sensor_status": dict(state.sensor_status),
                    "command_channel_state": state.command_channel_state,
                },
                "sensor_frame": asdict(sensor_frame),
                "system_response": {
                    "fleet_safety_state": observed_safety_state,
                    "fleet_mission_state": drone.get("mission_state", state.mission_state),
                    "fleet_localization_state": drone.get("localization_state", state.localization_state),
                    "readiness_status": readiness.get("status"),
                    "readiness_http_status": readiness.get("_http_status"),
                    "health_online_drones": health.get("online_drones"),
                    "fleet_stale_drones": snapshot.get("stale_drone_count"),
                },
                "latency_ms": {
                    "post": post_latency_ms,
                    "fleet": fleet_latency_ms,
                    "health": health_latency_ms,
                    "ready": readiness_latency_ms,
                    "total": step_latency,
                },
            }
        )
    if degraded_step is not None and recovery_step is None and last_safety_state == "NORMAL":
        recovery_step = spec.steps - 1
    detection_steps = degraded_step - spec.warmup_steps if degraded_step is not None else 0
    recovery_steps = (recovery_step - degraded_step) if degraded_step is not None and recovery_step is not None else 0
    return {
        "name": spec.name,
        "description": spec.description,
        "steps": spec.steps,
        "dt_s": spec.dt_s,
        "timeline": timeline,
        "safety_state_transitions": transitions,
        "detection_time_s": max(0.0, detection_steps * spec.dt_s),
        "recovery_time_s": max(0.0, recovery_steps * spec.dt_s),
        "latency_ms": {
            "average": sum(latencies_ms) / len(latencies_ms) if latencies_ms else 0.0,
            "max": max(latencies_ms) if latencies_ms else 0.0,
        },
        "final_state": timeline[-1]["system_response"] if timeline else {},
        "pass": True,
    }


def run_local_scenario(spec: ScenarioSpec) -> dict[str, Any]:
    state = spec.vehicle
    faults_by_step = {fault.at_step: fault for fault in spec.faults}
    timeline: list[dict[str, Any]] = []
    transitions: list[dict[str, Any]] = []
    degraded_step: int | None = None
    recovery_step: int | None = None
    last_safety_state = state.safety_state
    base_epoch = datetime.now(UTC).timestamp()
    for step_index in range(spec.steps):
        timestamp_text = datetime.fromtimestamp(base_epoch + (step_index * spec.dt_s), tz=UTC).isoformat().replace("+00:00", "Z")
        if step_index in faults_by_step:
            faults_by_step[step_index].apply(state)
        state.advance(spec.dt_s)
        sensor_frame = generate_sensor_frame(state, step_index, spec.dt_s)
        if state.safety_state != "NORMAL" and degraded_step is None:
            degraded_step = step_index
        if degraded_step is not None and state.safety_state == "NORMAL" and recovery_step is None:
            recovery_step = step_index
        if state.safety_state != last_safety_state:
            transitions.append({"step": step_index, "from": last_safety_state, "to": state.safety_state, "timestamp": timestamp_text})
            last_safety_state = state.safety_state
        timeline.append(
            {
                "step": step_index,
                "timestamp": timestamp_text,
                "input_state": {
                    "position": [state.x, state.y, state.z],
                    "telemetry_delay_ms": state.telemetry_delay_ms,
                    "packet_loss_pct": state.packet_loss_pct,
                    "localization_confidence": state.localization_confidence,
                },
                "sensor_frame": asdict(sensor_frame),
                "system_response": {
                    "safety_state": state.safety_state,
                    "mission_state": state.mission_state,
                    "localization_state": state.localization_state,
                    "command_channel_state": state.command_channel_state,
                },
            }
        )
    detection_steps = degraded_step - spec.warmup_steps if degraded_step is not None else 0
    recovery_steps = (recovery_step - degraded_step) if degraded_step is not None and recovery_step is not None else 0
    return {
        "name": spec.name,
        "description": spec.description,
        "steps": spec.steps,
        "dt_s": spec.dt_s,
        "timeline": timeline,
        "safety_state_transitions": transitions,
        "detection_time_s": max(0.0, detection_steps * spec.dt_s),
        "recovery_time_s": max(0.0, recovery_steps * spec.dt_s),
        "final_state": timeline[-1]["system_response"] if timeline else {},
        "pass": True,
    }


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
