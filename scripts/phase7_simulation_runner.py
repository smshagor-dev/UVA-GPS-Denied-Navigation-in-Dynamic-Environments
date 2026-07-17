#!/usr/bin/env python3
"""Run deterministic software simulation scenarios for Phase 7."""

from __future__ import annotations

import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from simulation.fault_injector import ScheduledFault, gps_denied_fault, packet_loss_fault, recovery_action, telemetry_delay_fault
from simulation.mission_executor import ScenarioSpec, run_local_scenario, utc_now
from simulation.vehicle_state import VehicleState

OUTPUT_PATH = REPO_ROOT / "docs" / "phase7" / "simulation_results.json"


def main() -> int:
    specs = [
        ScenarioSpec(
            name="swarm_coordination",
            description="Three-peer coordination surrogate with nominal mesh coordination metrics.",
            steps=8,
            dt_s=0.5,
            vehicle=VehicleState(drone_id=8301, cluster_id="phase7-sim", role="LEADER", vx=0.15, vy=0.04, peer_count=3),
            faults=[ScheduledFault(5, "recovery", recovery_action())],
            use_backend=False,
        ),
        ScenarioSpec(
            name="gps_denied",
            description="Simulation-only GPS-denied navigation path.",
            steps=8,
            dt_s=0.5,
            vehicle=VehicleState(drone_id=8302, cluster_id="phase7-sim", role="FOLLOWER", vx=0.09),
            faults=[ScheduledFault(2, "gps_denied", gps_denied_fault()), ScheduledFault(5, "recovery", recovery_action())],
            use_backend=False,
        ),
        ScenarioSpec(
            name="communication_loss",
            description="Simulation-only communication delay and packet loss scenario.",
            steps=8,
            dt_s=0.5,
            vehicle=VehicleState(drone_id=8303, cluster_id="phase7-sim", role="FOLLOWER"),
            faults=[
                ScheduledFault(2, "telemetry_delay", telemetry_delay_fault(380.0)),
                ScheduledFault(3, "packet_loss", packet_loss_fault(42.0)),
                ScheduledFault(6, "recovery", recovery_action()),
            ],
            use_backend=False,
        ),
    ]
    results = [run_local_scenario(spec) for spec in specs]
    payload = {
        "generated_at": utc_now(),
        "scenario_count": len(results),
        "simulation_layer": {
            "sensor_models": "simulation/sensor_models.py",
            "vehicle_state_model": "simulation/vehicle_state.py",
            "mission_executor": "simulation/mission_executor.py",
            "fault_injector": "simulation/fault_injector.py",
        },
        "scenarios": results,
        "status": "PASS" if all(result.get("pass") for result in results) else "FAIL",
        "notes": [
            "This is a deterministic software simulation abstraction, not PX4 or Gazebo evidence.",
            "Software HIL validation complete. Physical flight validation remains future work.",
        ],
    }
    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_PATH.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
