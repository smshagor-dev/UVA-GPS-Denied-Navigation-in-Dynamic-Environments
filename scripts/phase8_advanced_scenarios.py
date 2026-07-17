#!/usr/bin/env python3
"""Run deterministic advanced autonomy scenarios for Phase 8."""

from __future__ import annotations

import json
import platform
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import phase6_performance_suite as suite
from simulation.fault_injector import ScheduledFault, estimator_degradation_fault, packet_loss_fault, recovery_action, telemetry_delay_fault
from simulation.mission_executor import ScenarioSpec, run_backend_scenario, utc_now
from simulation.vehicle_state import VehicleState


DOC_ROOT = REPO_ROOT / "docs" / "phase8"
RESULTS_ROOT = REPO_ROOT / "results" / "phase8"
OUTPUT_PATH = DOC_ROOT / "advanced_scenario_results.json"


def build_specs() -> list[ScenarioSpec]:
    return [
        ScenarioSpec(
            name="multi_agent_coordination",
            description="Two-role coordination surrogate with stable peer coordination and shared formation state.",
            steps=8,
            dt_s=0.5,
            vehicle=VehicleState(
                drone_id=8401,
                cluster_id="phase8-advanced",
                role="LEADER",
                vx=0.18,
                vy=0.06,
                peer_count=4,
                mission_state="multi_agent_coordination",
            ),
            faults=[ScheduledFault(6, "recovery", recovery_action())],
        ),
        ScenarioSpec(
            name="dynamic_environment_adaptation",
            description="Obstacle density and localization degradation force adaptive autonomy response.",
            steps=8,
            dt_s=0.5,
            vehicle=VehicleState(
                drone_id=8402,
                cluster_id="phase8-advanced",
                role="FOLLOWER",
                local_obstacle_count=2,
                shared_obstacle_count=3,
                mission_state="dynamic_environment_adaptation",
            ),
            faults=[
                ScheduledFault(2, "dynamic_change", estimator_degradation_fault(0.42)),
                ScheduledFault(5, "recovery", recovery_action()),
            ],
        ),
        ScenarioSpec(
            name="long_duration_autonomy",
            description="Extended-step deterministic autonomy run with bounded battery drift and stable safety state.",
            steps=20,
            dt_s=0.5,
            vehicle=VehicleState(
                drone_id=8403,
                cluster_id="phase8-advanced",
                role="LEADER",
                vx=0.07,
                vy=0.02,
                mission_state="long_duration_autonomy",
            ),
            faults=[],
        ),
        ScenarioSpec(
            name="communication_degradation_recovery",
            description="Combined delay and packet loss recovery under sustained autonomy.",
            steps=8,
            dt_s=0.5,
            vehicle=VehicleState(
                drone_id=8404,
                cluster_id="phase8-advanced",
                role="FOLLOWER",
                mission_state="communication_degradation_recovery",
            ),
            faults=[
                ScheduledFault(2, "delay", telemetry_delay_fault(420.0)),
                ScheduledFault(3, "loss", packet_loss_fault(38.0)),
                ScheduledFault(6, "recovery", recovery_action()),
            ],
        ),
        ScenarioSpec(
            name="multi_objective_mission_planning",
            description="Mission prioritization surrogate balancing coordination, safety margin, and communication resilience.",
            steps=8,
            dt_s=0.5,
            vehicle=VehicleState(
                drone_id=8405,
                cluster_id="phase8-advanced",
                role="LEADER",
                vx=0.11,
                vy=-0.03,
                local_obstacle_count=1,
                shared_obstacle_count=2,
                peer_count=3,
                mission_state="multi_objective_mission_planning",
            ),
            faults=[
                ScheduledFault(2, "planning_tradeoff", telemetry_delay_fault(220.0)),
                ScheduledFault(5, "recovery", recovery_action()),
            ],
        ),
    ]


def summarize(results: list[dict[str, object]]) -> dict[str, object]:
    reliability = sum(1 for result in results if result.get("pass")) / len(results) if results else 0.0
    avg_detection = sum(float(result.get("detection_time_s", 0.0)) for result in results) / len(results) if results else 0.0
    avg_recovery = sum(float(result.get("recovery_time_s", 0.0)) for result in results) / len(results) if results else 0.0
    return {
        "scenario_reliability": reliability,
        "average_detection_time_s": avg_detection,
        "average_recovery_time_s": avg_recovery,
    }


def main() -> int:
    DOC_ROOT.mkdir(parents=True, exist_ok=True)
    RESULTS_ROOT.mkdir(parents=True, exist_ok=True)
    backend_port = suite.allocate_free_port()
    backend_log = RESULTS_ROOT / "phase8_advanced_backend.log"
    process, startup_ms = suite.start_backend(
        backend_port,
        backend_log,
        mode="production",
        simulation_enabled=False,
        extra_env={"DRONE_BACKEND_STALE_SEC": "2"},
    )
    results: list[dict[str, object]] = []
    try:
        base_url = f"http://127.0.0.1:{backend_port}"
        for spec in build_specs():
            results.append(run_backend_scenario(spec, base_url))
    finally:
        suite.stop_backend(process)

    payload = {
        "generated_at": utc_now(),
        "environment": {
            "platform": platform.platform(),
            "python_version": sys.version,
        },
        "backend_startup_ms": startup_ms,
        "scenario_count": len(results),
        "scenarios": results,
        "summary": summarize(results),
        "status": "PASS" if all(result.get("pass") for result in results) else "FAIL",
        "notes": [
            "Advanced autonomy scenarios are deterministic software experiments executed against the live Go control-plane backend.",
            "No physical flight, hardware qualification, or ML accuracy claims are implied by these results.",
        ],
    }
    OUTPUT_PATH.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
