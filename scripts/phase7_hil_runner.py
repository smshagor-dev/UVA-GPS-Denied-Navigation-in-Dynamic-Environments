#!/usr/bin/env python3
"""Run a deterministic software HIL evidence suite for Phase 7."""

from __future__ import annotations

import json
import platform
import sys
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import phase6_performance_suite as suite
from simulation.fault_injector import (
    ScheduledFault,
    estimator_degradation_fault,
    gps_denied_fault,
    packet_loss_fault,
    recovery_action,
    sensor_dropout_fault,
    telemetry_delay_fault,
)
from simulation.mission_executor import ScenarioSpec, run_backend_scenario, utc_now
from simulation.vehicle_state import VehicleState


DOC_ROOT = REPO_ROOT / "docs" / "phase7"
RESULTS_ROOT = REPO_ROOT / "results" / "phase7"
OUTPUT_PATH = DOC_ROOT / "hil_results.json"


def build_scenarios() -> list[ScenarioSpec]:
    return [
        ScenarioSpec(
            name="hil_gps_denied_navigation",
            description="Software HIL navigation loop with GPS denied and VIO/TDOA fallback.",
            steps=8,
            dt_s=0.5,
            vehicle=VehicleState(
                drone_id=8101, cluster_id="phase7-hil", role="LEADER", vx=0.2, vy=0.05
            ),
            faults=[
                ScheduledFault(2, "gps_denied", gps_denied_fault()),
                ScheduledFault(5, "recovery", recovery_action()),
            ],
        ),
        ScenarioSpec(
            name="hil_sensor_dropout",
            description="Software HIL sensor dropout with degraded localization and recovery.",
            steps=8,
            dt_s=0.5,
            vehicle=VehicleState(
                drone_id=8102, cluster_id="phase7-hil", role="FOLLOWER", vx=0.1
            ),
            faults=[
                ScheduledFault(2, "camera_dropout", sensor_dropout_fault("camera")),
                ScheduledFault(5, "recovery", recovery_action()),
            ],
        ),
        ScenarioSpec(
            name="hil_telemetry_delay",
            description="Software HIL delayed telemetry link with recovery to nominal backend state.",
            steps=8,
            dt_s=0.5,
            vehicle=VehicleState(
                drone_id=8103, cluster_id="phase7-hil", role="FOLLOWER"
            ),
            faults=[
                ScheduledFault(2, "telemetry_delay", telemetry_delay_fault(450.0)),
                ScheduledFault(5, "recovery", recovery_action()),
            ],
        ),
        ScenarioSpec(
            name="hil_packet_loss",
            description="Software HIL packet loss forcing degraded then recovered communication state.",
            steps=8,
            dt_s=0.5,
            vehicle=VehicleState(
                drone_id=8104, cluster_id="phase7-hil", role="FOLLOWER"
            ),
            faults=[
                ScheduledFault(2, "packet_loss", packet_loss_fault(45.0)),
                ScheduledFault(5, "recovery", recovery_action()),
            ],
        ),
        ScenarioSpec(
            name="hil_estimator_degradation",
            description="Software HIL estimator degradation with emergency fallback and recovery.",
            steps=8,
            dt_s=0.5,
            vehicle=VehicleState(
                drone_id=8105, cluster_id="phase7-hil", role="LEADER", vx=0.08, vy=-0.04
            ),
            faults=[
                ScheduledFault(
                    2, "estimator_degradation", estimator_degradation_fault(0.18)
                ),
                ScheduledFault(6, "recovery", recovery_action()),
            ],
        ),
    ]


def main() -> int:
    DOC_ROOT.mkdir(parents=True, exist_ok=True)
    RESULTS_ROOT.mkdir(parents=True, exist_ok=True)
    backend_port = suite.allocate_free_port()
    backend_log = RESULTS_ROOT / "phase7_hil_backend.log"
    process, startup_ms = suite.start_backend(
        backend_port,
        backend_log,
        mode="production",
        simulation_enabled=False,
        extra_env={"DRONE_BACKEND_STALE_SEC": "2"},
    )
    scenarios = build_scenarios()
    results: list[dict[str, Any]] = []
    try:
        base_url = f"http://127.0.0.1:{backend_port}"
        for scenario in scenarios:
            results.append(run_backend_scenario(scenario, base_url))
    finally:
        suite.stop_backend(process)

    payload = {
        "generated_at": utc_now(),
        "environment": {
            "platform": platform.platform(),
            "python_version": sys.version,
        },
        "backend": {
            "mode": "production",
            "simulation_enabled": False,
            "startup_ms": startup_ms,
            "log_path": str(backend_log.relative_to(REPO_ROOT)),
        },
        "scenario_count": len(results),
        "scenarios": results,
        "status": "PASS" if all(result.get("pass") for result in results) else "FAIL",
        "notes": [
            "Software HIL validation complete. Physical flight validation remains future work.",
            "This harness simulates deterministic sensor, telemetry, and command behavior against the live Go control-plane backend.",
        ],
    }
    OUTPUT_PATH.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
