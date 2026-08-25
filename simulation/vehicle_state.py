from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class VehicleState:
    drone_id: int
    cluster_id: str
    role: str
    x: float = 0.0
    y: float = 0.0
    z: float = 1.5
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw: float = 0.0
    battery_pct: float = 87.0
    localization_source: str = "vision-inertial"
    localization_data_source: str = "simulation"
    localization_state: str = "nominal"
    localization_confidence: float = 0.92
    tdoa_confidence: float = 0.75
    visible_anchor_count: int = 4
    sync_confidence: float = 0.97
    peer_count: int = 2
    stale_peer_count: int = 0
    local_obstacle_count: int = 0
    shared_obstacle_count: int = 0
    safety_state: str = "NORMAL"
    safety_summary: str = "Nominal operation"
    security_state: str = "TRUSTED"
    remote_command_allowed: bool = True
    telemetry_uplink_allowed: bool = True
    mission_state: str = "hold"
    command_channel_state: str = "ready"
    telemetry_delay_ms: float = 0.0
    packet_loss_pct: float = 0.0
    estimator_health: str = "nominal"
    sensor_status: dict[str, str] = field(
        default_factory=lambda: {
            "imu": "live",
            "gps": "live",
            "camera": "live",
            "lidar": "live",
            "telemetry_link": "healthy",
            "command_channel": "ready",
        }
    )

    def advance(self, dt_s: float) -> None:
        self.x += self.vx * dt_s
        self.y += self.vy * dt_s
        self.z = max(0.0, self.z + self.vz * dt_s)
        self.battery_pct = max(0.0, self.battery_pct - (0.004 * dt_s))

