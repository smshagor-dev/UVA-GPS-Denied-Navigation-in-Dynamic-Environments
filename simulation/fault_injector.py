from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from simulation.vehicle_state import VehicleState


FaultAction = Callable[[VehicleState], None]


@dataclass(frozen=True)
class ScheduledFault:
    at_step: int
    name: str
    apply: FaultAction


def gps_denied_fault() -> FaultAction:
    def apply(state: VehicleState) -> None:
        state.sensor_status["gps"] = "denied"
        state.localization_source = "vio-tdoa-fused"
        state.localization_data_source = "simulation"
        state.mission_state = "gps_denied_navigation"
        state.safety_summary = "GPS denied; estimator relying on VIO/TDOA"
    return apply


def sensor_dropout_fault(sensor: str) -> FaultAction:
    def apply(state: VehicleState) -> None:
        state.sensor_status[sensor] = "dropout"
        state.localization_confidence = min(state.localization_confidence, 0.44)
        state.localization_state = "degraded"
        state.safety_state = "DEGRADED_LOCALIZATION"
        state.safety_summary = f"{sensor} dropout degraded localization confidence"
        state.mission_state = "sensor_dropout"
    return apply


def telemetry_delay_fault(delay_ms: float) -> FaultAction:
    def apply(state: VehicleState) -> None:
        state.sensor_status["telemetry_link"] = "delayed"
        state.telemetry_delay_ms = delay_ms
        state.safety_state = "LINK_LOST" if delay_ms >= 400.0 else "DEGRADED_LOCALIZATION"
        state.safety_summary = f"Telemetry delay elevated to {delay_ms:.0f} ms"
        state.mission_state = "telemetry_delay"
    return apply


def packet_loss_fault(loss_pct: float) -> FaultAction:
    def apply(state: VehicleState) -> None:
        state.sensor_status["telemetry_link"] = "lossy"
        state.packet_loss_pct = loss_pct
        state.stale_peer_count = max(state.stale_peer_count, 1)
        state.safety_state = "LINK_LOST" if loss_pct >= 40.0 else "DEGRADED_LOCALIZATION"
        state.safety_summary = f"Packet loss increased to {loss_pct:.0f}%"
        state.mission_state = "packet_loss"
    return apply


def estimator_degradation_fault(confidence: float) -> FaultAction:
    def apply(state: VehicleState) -> None:
        state.localization_confidence = confidence
        state.tdoa_confidence = min(state.tdoa_confidence, confidence + 0.08)
        state.localization_state = "lost" if confidence < 0.25 else "degraded"
        state.estimator_health = "lost" if confidence < 0.25 else "degraded"
        state.safety_state = "EMERGENCY_LAND" if confidence < 0.25 else "DEGRADED_LOCALIZATION"
        state.security_state = "LAND_IMMEDIATELY" if confidence < 0.25 else state.security_state
        state.remote_command_allowed = False if confidence < 0.25 else state.remote_command_allowed
        state.safety_summary = "Estimator confidence degraded below mission threshold"
        state.mission_state = "estimator_degradation"
    return apply


def invalid_localization_fault() -> FaultAction:
    def apply(state: VehicleState) -> None:
        state.localization_confidence = 0.05
        state.localization_state = "lost"
        state.visible_anchor_count = 0
        state.safety_state = "EMERGENCY_LAND"
        state.security_state = "LAND_IMMEDIATELY"
        state.remote_command_allowed = False
        state.safety_summary = "Invalid localization rejected by validation layer"
        state.mission_state = "invalid_localization"
    return apply


def command_rejection_fault() -> FaultAction:
    def apply(state: VehicleState) -> None:
        state.command_channel_state = "rejected"
        state.sensor_status["command_channel"] = "rejected"
        state.remote_command_allowed = False
        state.safety_summary = "Command rejected by policy gate"
    return apply


def recovery_action() -> FaultAction:
    def apply(state: VehicleState) -> None:
        state.sensor_status["imu"] = "live"
        state.sensor_status["gps"] = "live"
        state.sensor_status["camera"] = "live"
        state.sensor_status["lidar"] = "live"
        state.sensor_status["telemetry_link"] = "healthy"
        state.sensor_status["command_channel"] = "ready"
        state.localization_source = "vision-inertial"
        state.localization_data_source = "simulation"
        state.localization_state = "nominal"
        state.localization_confidence = 0.91
        state.tdoa_confidence = 0.74
        state.visible_anchor_count = 4
        state.telemetry_delay_ms = 0.0
        state.packet_loss_pct = 0.0
        state.estimator_health = "nominal"
        state.safety_state = "NORMAL"
        state.safety_summary = "System recovered to nominal operation"
        state.security_state = "TRUSTED"
        state.remote_command_allowed = True
        state.telemetry_uplink_allowed = True
        state.command_channel_state = "ready"
        state.mission_state = "recovered"
        state.stale_peer_count = 0
    return apply

