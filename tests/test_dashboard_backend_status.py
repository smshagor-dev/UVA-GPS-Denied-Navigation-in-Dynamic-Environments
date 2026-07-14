import unittest

from gui.backend_status import (
    compose_backend_mode,
    compose_operator_status,
    summarize_localization_data_source,
)
from gui.dashboard_contract import (
    DashboardSnapshot,
    DroneState,
    mission_start_allowed,
    parse_camera_payload,
    parse_imu_payload,
    parse_lidar_payload,
    parse_replay_payload,
    parse_tdoa_payload,
)


class BackendStatusTests(unittest.TestCase):
    def _snapshot(self, **state_overrides) -> DashboardSnapshot:
        defaults = {
            "drone_id": 1,
            "cluster_id": "cluster-01",
            "role": "LEADER",
            "mission_state": "standby",
            "position": (0.0, 0.0, 1.0),
            "velocity": (0.0, 0.0, 0.0),
            "drift_m": 0.05,
            "battery_pct": 91.0,
            "connectivity": "Mesh",
            "reachable": True,
            "rssi_dbm": -52.0,
            "cpu_temp_c": 48.0,
            "gpu_load_pct": 24.0,
            "localization_data_source": "real",
            "localization_confidence": 0.92,
            "remote_command_allowed": True,
        }
        defaults.update(state_overrides)
        state = DroneState(**defaults)
        return DashboardSnapshot(
            states=[state],
            backend_mode="production",
            leader_id=1,
            avg_latency_ms=8.0,
            packet_loss_pct=0.4,
            cpu_temp_c=48.0,
            gpu_load_pct=24.0,
            real_drone_count=1,
            stale_drone_count=0,
        )

    def test_base_mode_is_preserved_without_fallback(self) -> None:
        self.assertEqual(compose_backend_mode("go-control-plane"), "go-control-plane")

    def test_simulation_fallback_is_operator_visible(self) -> None:
        self.assertEqual(
            compose_backend_mode("go-control-plane", "go-control-plane-unreachable"),
            "go-control-plane-fallback:go-control-plane-unreachable",
        )

    def test_empty_inputs_default_to_simulation(self) -> None:
        self.assertEqual(compose_backend_mode("", ""), "simulation")

    def test_operator_status_shows_simulation_warning(self) -> None:
        self.assertIn(
            "LOCALIZATION:SIMULATION",
            compose_operator_status("simulation", "simulation"),
        )

    def test_operator_status_warns_when_no_real_drones_connected(self) -> None:
        self.assertIn(
            "NO REAL DRONES",
            compose_operator_status(
                "production", "real", simulation_enabled=False, real_drone_count=0
            ),
        )

    def test_operator_status_warns_when_telemetry_is_stale(self) -> None:
        self.assertIn(
            "STALE TELEMETRY:2",
            compose_operator_status(
                "production",
                "real",
                simulation_enabled=False,
                real_drone_count=1,
                stale_drone_count=2,
            ),
        )

    def test_operator_status_marks_edge_swarm_mode(self) -> None:
        self.assertIn(
            "EDGE SWARM ACTIVE",
            compose_operator_status(
                "edge_swarm", "real", simulation_enabled=False, real_drone_count=2
            ),
        )

    def test_source_summary_prioritizes_simulation(self) -> None:
        self.assertEqual(
            summarize_localization_data_source(["real", "simulation"]), "simulation"
        )

    def test_mission_start_allowed_requires_real_safe_state(self) -> None:
        self.assertTrue(mission_start_allowed(self._snapshot()))

    def test_mission_start_allowed_rejects_simulation_or_low_confidence(self) -> None:
        self.assertFalse(
            mission_start_allowed(self._snapshot(localization_data_source="simulation"))
        )
        self.assertFalse(
            mission_start_allowed(self._snapshot(localization_confidence=0.42))
        )

    def test_sensor_payload_parsers_handle_missing_fields(self) -> None:
        self.assertEqual(parse_camera_payload({})[0], "unavailable")
        self.assertEqual(parse_imu_payload({})[1]["accel"], (0.0, 0.0, 0.0))
        self.assertEqual(parse_lidar_payload({})[1]["points_2d"], [])
        self.assertEqual(parse_tdoa_payload({})[1]["anchors"], [])
        self.assertEqual(parse_replay_payload({})[1]["confidence_series"], [])

    def test_sensor_payload_parsers_preserve_real_and_simulation_sources(self) -> None:
        self.assertEqual(
            parse_camera_payload({"status": "live", "source": "real"})[1]["source"],
            "real",
        )
        self.assertEqual(
            parse_lidar_payload({"status": "simulation", "source": "simulation"})[1][
                "source"
            ],
            "simulation",
        )
        self.assertEqual(
            parse_tdoa_payload({"status": "playback", "source": "playback"})[1][
                "source"
            ],
            "playback",
        )


if __name__ == "__main__":
    unittest.main()
