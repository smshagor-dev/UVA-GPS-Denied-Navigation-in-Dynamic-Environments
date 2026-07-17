#!/usr/bin/env python3
"""Run deterministic Phase 7 failure injection scenarios and export evidence."""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import phase6_performance_suite as suite
from simulation.fault_injector import (
    ScheduledFault,
    command_rejection_fault,
    estimator_degradation_fault,
    invalid_localization_fault,
    packet_loss_fault,
    recovery_action,
    sensor_dropout_fault,
    telemetry_delay_fault,
)
from simulation.mission_executor import ScenarioSpec, run_backend_scenario, utc_now
from simulation.vehicle_state import VehicleState


DOC_ROOT = REPO_ROOT / "docs" / "phase7"
RESULTS_ROOT = REPO_ROOT / "results" / "phase7"
OUTPUT_PATH = DOC_ROOT / "failure_injection_results.json"


def build_failure_specs() -> list[ScenarioSpec]:
    return [
        ScenarioSpec(
            name="delayed_packets",
            description="Delayed telemetry packets trigger degraded then recovered state.",
            steps=7,
            dt_s=0.5,
            vehicle=VehicleState(
                drone_id=8201, cluster_id="phase7-failure", role="LEADER"
            ),
            faults=[
                ScheduledFault(2, "delay", telemetry_delay_fault(500.0)),
                ScheduledFault(5, "recovery", recovery_action()),
            ],
        ),
        ScenarioSpec(
            name="missing_telemetry",
            description="Packet loss triggers stale communication behavior.",
            steps=7,
            dt_s=0.5,
            vehicle=VehicleState(
                drone_id=8202, cluster_id="phase7-failure", role="FOLLOWER"
            ),
            faults=[
                ScheduledFault(2, "packet_loss", packet_loss_fault(55.0)),
                ScheduledFault(5, "recovery", recovery_action()),
            ],
        ),
        ScenarioSpec(
            name="stale_sensor_data",
            description="LiDAR dropout creates degraded localization and sensor fault visibility.",
            steps=7,
            dt_s=0.5,
            vehicle=VehicleState(
                drone_id=8203, cluster_id="phase7-failure", role="FOLLOWER"
            ),
            faults=[
                ScheduledFault(2, "lidar_dropout", sensor_dropout_fault("lidar")),
                ScheduledFault(5, "recovery", recovery_action()),
            ],
        ),
        ScenarioSpec(
            name="invalid_localization",
            description="Invalid localization is rejected and escalates to emergency fallback.",
            steps=7,
            dt_s=0.5,
            vehicle=VehicleState(
                drone_id=8204, cluster_id="phase7-failure", role="LEADER"
            ),
            faults=[
                ScheduledFault(2, "invalid_localization", invalid_localization_fault()),
                ScheduledFault(5, "recovery", recovery_action()),
            ],
        ),
        ScenarioSpec(
            name="degraded_estimator_confidence",
            description="Low estimator confidence produces emergency fallback visibility.",
            steps=7,
            dt_s=0.5,
            vehicle=VehicleState(
                drone_id=8205, cluster_id="phase7-failure", role="LEADER"
            ),
            faults=[
                ScheduledFault(2, "estimator_drop", estimator_degradation_fault(0.19)),
                ScheduledFault(5, "recovery", recovery_action()),
            ],
        ),
        ScenarioSpec(
            name="command_rejection",
            description="Command rejection is surfaced through policy state and command channel visibility.",
            steps=7,
            dt_s=0.5,
            vehicle=VehicleState(
                drone_id=8206, cluster_id="phase7-failure", role="FOLLOWER"
            ),
            faults=[
                ScheduledFault(2, "command_rejection", command_rejection_fault()),
                ScheduledFault(5, "recovery", recovery_action()),
            ],
        ),
    ]


def summarize(results: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "detection_time_s_max": max(
            result.get("detection_time_s", 0.0) for result in results
        ),
        "recovery_time_s_max": max(
            result.get("recovery_time_s", 0.0) for result in results
        ),
        "final_states": {
            result["name"]: result.get("final_state", {}) for result in results
        },
    }


def main() -> int:
    DOC_ROOT.mkdir(parents=True, exist_ok=True)
    RESULTS_ROOT.mkdir(parents=True, exist_ok=True)
    backend_port = suite.allocate_free_port()
    backend_log = RESULTS_ROOT / "phase7_failure_backend.log"
    process, startup_ms = suite.start_backend(
        backend_port,
        backend_log,
        mode="production",
        simulation_enabled=False,
        extra_env={"DRONE_BACKEND_STALE_SEC": "2"},
    )
    results: list[dict[str, Any]] = []
    try:
        base_url = f"http://127.0.0.1:{backend_port}"
        for spec in build_failure_specs():
            results.append(run_backend_scenario(spec, base_url))
    finally:
        suite.stop_backend(process)
    payload = {
        "generated_at": utc_now(),
        "backend_startup_ms": startup_ms,
        "scenario_count": len(results),
        "scenarios": results,
        "summary": summarize(results),
        "status": "PASS" if all(result.get("pass") for result in results) else "FAIL",
    }
    OUTPUT_PATH.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
