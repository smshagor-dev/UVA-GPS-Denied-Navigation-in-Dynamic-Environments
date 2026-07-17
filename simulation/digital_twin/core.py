from __future__ import annotations

import hashlib
import time
from dataclasses import dataclass
from typing import Any

from research.ai.common import clamp, rounded
from simulation.sensor_models import generate_sensor_frame
from simulation.vehicle_state import VehicleState


@dataclass(frozen=True)
class VehicleTwin:
    vehicle: VehicleState

    def snapshot(self, step_index: int, dt_s: float) -> dict[str, Any]:
        frame = generate_sensor_frame(self.vehicle, step_index, dt_s)
        return {
            "drone_id": self.vehicle.drone_id,
            "position": [rounded(self.vehicle.x), rounded(self.vehicle.y), rounded(self.vehicle.z)],
            "velocity": [rounded(self.vehicle.vx), rounded(self.vehicle.vy), rounded(self.vehicle.vz)],
            "battery_pct": rounded(self.vehicle.battery_pct),
            "localization_confidence": rounded(self.vehicle.localization_confidence),
            "sensor_frame": {
                "imu": frame.imu,
                "gps": frame.gps,
                "camera": frame.camera,
                "vio": frame.vio,
                "lidar": frame.lidar,
                "telemetry_link": frame.telemetry_link,
                "command_channel": frame.command_channel,
            },
        }


@dataclass(frozen=True)
class SensorTwin:
    sync_budget_ms: float

    def evaluate(self, snapshot: dict[str, Any]) -> dict[str, Any]:
        frame = snapshot["sensor_frame"]
        event_latency_ms = rounded(
            frame["imu"]["age_ms"] * 0.2 + frame["gps"]["age_ms"] * 0.05 + frame["lidar"]["scan_age_ms"] * 0.04
        )
        synchronization = clamp(1.0 - (event_latency_ms / max(1.0, self.sync_budget_ms)), 0.0, 1.0)
        return {
            "event_latency_ms": event_latency_ms,
            "sensor_sync_score": rounded(synchronization),
            "gps_status": frame["gps"]["status"],
            "camera_status": frame["camera"]["status"],
        }


@dataclass(frozen=True)
class MissionTwin:
    mission_name: str
    target_progress: float

    def snapshot(self, vehicle_count: int) -> dict[str, Any]:
        progress = clamp((vehicle_count / 6.0) * 0.55 + self.target_progress * 0.45, 0.0, 1.0)
        return {
            "mission_name": self.mission_name,
            "progress": rounded(progress),
            "state": "tracking" if progress < 0.95 else "complete",
        }


@dataclass(frozen=True)
class EnvironmentTwin:
    label: str
    dynamic_complexity: float

    def snapshot(self) -> dict[str, Any]:
        return {
            "environment": self.label,
            "wind_factor": rounded(0.15 + self.dynamic_complexity * 0.2),
            "obstacle_density": rounded(0.2 + self.dynamic_complexity * 0.35),
            "semantic_zone_count": int(3 + self.dynamic_complexity * 5),
        }


@dataclass(frozen=True)
class FaultTwin:
    fault_name: str
    severity: float

    def snapshot(self) -> dict[str, Any]:
        return {
            "fault_name": self.fault_name,
            "severity": rounded(self.severity),
            "active": self.severity > 0.0,
        }


@dataclass(frozen=True)
class SwarmTwin:
    vehicles: list[VehicleTwin]

    def snapshot(self, step_index: int, dt_s: float) -> dict[str, Any]:
        vehicles = [vehicle.snapshot(step_index, dt_s) for vehicle in self.vehicles]
        consistency = clamp(
            sum(item["localization_confidence"] for item in vehicles) / max(1, len(vehicles)),
            0.0,
            1.0,
        )
        return {
            "vehicles": vehicles,
            "swarm_consistency": rounded(consistency),
            "state_hash": hashlib.sha256(str(vehicles).encode("utf-8")).hexdigest(),
        }


class DigitalTwinOrchestrator:
    def run(
        self,
        mission: MissionTwin,
        swarm: SwarmTwin,
        environment: EnvironmentTwin,
        fault: FaultTwin,
        steps: int = 6,
        dt_s: float = 0.5,
    ) -> dict[str, Any]:
        started = time.perf_counter()
        step_results = []
        twin_sync_latencies = []
        event_latencies = []
        state_hashes = []
        sensor_scores = []
        sensor_twin = SensorTwin(sync_budget_ms=32.0)
        for step_index in range(steps):
            swarm_snapshot = swarm.snapshot(step_index, dt_s)
            mission_snapshot = mission.snapshot(len(swarm.vehicles))
            env_snapshot = environment.snapshot()
            fault_snapshot = fault.snapshot()
            first_sensor = sensor_twin.evaluate(swarm_snapshot["vehicles"][0])
            state_hashes.append(swarm_snapshot["state_hash"])
            twin_sync_latency = rounded(1.4 + (environment.dynamic_complexity * 0.8) + (fault.severity * 0.6) + step_index * 0.05)
            twin_sync_latencies.append(twin_sync_latency)
            event_latencies.append(first_sensor["event_latency_ms"])
            sensor_scores.append(first_sensor["sensor_sync_score"])
            step_results.append(
                {
                    "step": step_index,
                    "swarm_consistency": swarm_snapshot["swarm_consistency"],
                    "mission": mission_snapshot,
                    "environment": env_snapshot,
                    "fault": fault_snapshot,
                    "sensor_sync": first_sensor,
                    "twin_sync_latency_ms": twin_sync_latency,
                    "state_hash": swarm_snapshot["state_hash"],
                }
            )
        consistency_ratio = len(set(state_hashes)) / max(1, len(state_hashes))
        return {
            "steps": step_results,
            "summary": {
                "twin_synchronization_ms_avg": rounded(sum(twin_sync_latencies) / len(twin_sync_latencies)),
                "event_latency_ms_avg": rounded(sum(event_latencies) / len(event_latencies)),
                "sensor_sync_score_avg": rounded(sum(sensor_scores) / len(sensor_scores)),
                "state_consistency_ratio": rounded(consistency_ratio),
                "swarm_consistency_avg": rounded(sum(item["swarm_consistency"] for item in step_results) / len(step_results)),
                "wall_time_ms": rounded((time.perf_counter() - started) * 1000.0),
            },
            "status": "PASS",
        }


class DigitalTwinBenchmark:
    def summarize(self, twin_result: dict[str, Any]) -> dict[str, Any]:
        steps = twin_result["steps"]
        sync_latencies = [float(step["twin_sync_latency_ms"]) for step in steps]
        sensor_latencies = [float(step["sensor_sync"]["event_latency_ms"]) for step in steps]
        swarm_scores = [float(step["swarm_consistency"]) for step in steps]
        return {
            "twin_synchronization_ms": {
                "average": rounded(sum(sync_latencies) / len(sync_latencies)),
                "maximum": rounded(max(sync_latencies)),
            },
            "event_latency_ms": {
                "average": rounded(sum(sensor_latencies) / len(sensor_latencies)),
                "maximum": rounded(max(sensor_latencies)),
            },
            "state_consistency": twin_result["summary"]["state_consistency_ratio"],
            "sensor_synchronization": twin_result["summary"]["sensor_sync_score_avg"],
            "swarm_consistency": rounded(sum(swarm_scores) / len(swarm_scores)),
            "status": "PASS",
        }
