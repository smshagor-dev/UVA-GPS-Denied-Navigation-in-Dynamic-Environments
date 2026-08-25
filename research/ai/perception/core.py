from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any

from research.ai.common import clamp
from research.ai.inference import BaseModelAdapter, InferenceResult
from simulation.sensor_models import SensorFrame
from simulation.vehicle_state import VehicleState


@dataclass(frozen=True)
class Detection:
    label: str
    confidence: float
    source: str
    details: dict[str, Any]


@dataclass(frozen=True)
class PerceptionInput:
    timestamp: str
    vehicle_state: dict[str, Any]
    sensor_frame: dict[str, Any]
    telemetry_context: dict[str, Any]
    expected_labels: list[str]


@dataclass(frozen=True)
class PerceptionResult:
    detections: list[Detection]
    decision_input: dict[str, Any]
    inference: InferenceResult
    latency_ms: float
    confidence_score: float


class TelemetryPerceptionPipeline:
    def __init__(self, model: BaseModelAdapter) -> None:
        self.model = model

    def build_input(
        self,
        timestamp: str,
        state: VehicleState,
        sensor_frame: SensorFrame,
        expected_labels: list[str],
    ) -> PerceptionInput:
        return PerceptionInput(
            timestamp=timestamp,
            vehicle_state={
                "drone_id": state.drone_id,
                "position": [state.x, state.y, state.z],
                "velocity": [state.vx, state.vy, state.vz],
                "mission_state": state.mission_state,
                "battery_pct": state.battery_pct,
                "localization_confidence": state.localization_confidence,
                "localization_state": state.localization_state,
            },
            sensor_frame={
                "imu": sensor_frame.imu,
                "gps": sensor_frame.gps,
                "camera": sensor_frame.camera,
                "vio": sensor_frame.vio,
                "lidar": sensor_frame.lidar,
                "telemetry_link": sensor_frame.telemetry_link,
                "command_channel": sensor_frame.command_channel,
            },
            telemetry_context={
                "peer_count": state.peer_count,
                "stale_peer_count": state.stale_peer_count,
                "local_obstacle_count": state.local_obstacle_count,
                "shared_obstacle_count": state.shared_obstacle_count,
                "telemetry_delay_ms": state.telemetry_delay_ms,
                "packet_loss_pct": state.packet_loss_pct,
            },
            expected_labels=expected_labels,
        )

    def _detect(self, perception_input: PerceptionInput) -> list[Detection]:
        detections: list[Detection] = []
        local_obs = int(perception_input.telemetry_context["local_obstacle_count"])
        shared_obs = int(perception_input.telemetry_context["shared_obstacle_count"])
        localization_confidence = float(perception_input.vehicle_state["localization_confidence"])
        telemetry_delay_ms = float(perception_input.telemetry_context["telemetry_delay_ms"])
        packet_loss_pct = float(perception_input.telemetry_context["packet_loss_pct"])
        sensor_frame = perception_input.sensor_frame

        if (local_obs + shared_obs) > 0:
            detections.append(
                Detection(
                    label="obstacle_cluster",
                    confidence=clamp(0.55 + ((local_obs + shared_obs) * 0.05), 0.0, 0.98),
                    source="lidar-telemetry",
                    details={"local_count": local_obs, "shared_count": shared_obs},
                )
            )
        if localization_confidence < 0.6:
            detections.append(
                Detection(
                    label="localization_degradation",
                    confidence=clamp(0.9 - localization_confidence, 0.0, 0.97),
                    source="vio-estimator",
                    details={"localization_confidence": localization_confidence},
                )
            )
        if telemetry_delay_ms >= 180.0 or packet_loss_pct >= 15.0:
            detections.append(
                Detection(
                    label="telemetry_degradation",
                    confidence=clamp(0.52 + (telemetry_delay_ms / 1000.0) + (packet_loss_pct / 200.0), 0.0, 0.96),
                    source="telemetry-link",
                    details={"delay_ms": telemetry_delay_ms, "packet_loss_pct": packet_loss_pct},
                )
            )
        if sensor_frame["camera"]["status"] != "live" or sensor_frame["gps"]["status"] != "live":
            detections.append(
                Detection(
                    label="sensor_dropout",
                    confidence=0.88,
                    source="sensor-health",
                    details={
                        "camera": sensor_frame["camera"]["status"],
                        "gps": sensor_frame["gps"]["status"],
                    },
                )
            )
        if not detections:
            detections.append(
                Detection(label="nominal_operation", confidence=0.91, source="telemetry-perception", details={})
            )
        return detections

    def run(self, perception_input: PerceptionInput) -> PerceptionResult:
        started = time.perf_counter()
        detections = self._detect(perception_input)
        obstacle_density = clamp(
            (float(perception_input.telemetry_context["local_obstacle_count"]) + float(perception_input.telemetry_context["shared_obstacle_count"])) / 10.0,
            0.0,
            1.0,
        )
        telemetry_penalty = clamp(
            (float(perception_input.telemetry_context["telemetry_delay_ms"]) / 500.0)
            + (float(perception_input.telemetry_context["packet_loss_pct"]) / 100.0),
            0.0,
            1.0,
        )
        inference = self.model.infer(
            {
                "obstacle_density": obstacle_density,
                "localization_confidence": float(perception_input.vehicle_state["localization_confidence"]),
                "link_quality": clamp(1.0 - telemetry_penalty, 0.0, 1.0),
                "mission_pressure": 0.35 if "telemetry_degradation" in [item.label for item in detections] else 0.12,
            }
        )
        top_confidence = max(item.confidence for item in detections)
        decision_input = {
            "top_detection": max(detections, key=lambda item: item.confidence).label,
            "detection_labels": [item.label for item in detections],
            "confidence_score": clamp((top_confidence + inference.confidence) / 2.0, 0.0, 1.0),
            "prediction": inference.prediction,
            "prediction_backend": inference.backend,
            "fallback_required": top_confidence < 0.55 or inference.confidence < 0.6,
        }
        latency_ms = ((time.perf_counter() - started) * 1000.0) + inference.latency_ms
        return PerceptionResult(
            detections=detections,
            decision_input=decision_input,
            inference=inference,
            latency_ms=latency_ms,
            confidence_score=float(decision_input["confidence_score"]),
        )

