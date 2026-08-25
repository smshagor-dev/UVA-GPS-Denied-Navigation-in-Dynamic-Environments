#!/usr/bin/env python3
"""Phase 9 AI-assisted mission planning validation."""

from __future__ import annotations

import platform
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from research.ai.common import summarize_latency, utc_now, write_json
from research.ai.inference import ModelMetadata, MockOnnxAdapter
from research.ai.planning import AIMissionPlanner, MissionScenario, RouteOption


DOC_ROOT = REPO_ROOT / "docs" / "phase9"
OUTPUT_JSON = DOC_ROOT / "mission_planning_results.json"
OUTPUT_MD = DOC_ROOT / "MISSION_PLANNING_REPORT.md"


def build_scenarios() -> list[MissionScenario]:
    drones = [
        {"drone_id": 9201, "battery_pct": 88.0, "role": "LEADER", "load_score": 0.2},
        {"drone_id": 9202, "battery_pct": 83.0, "role": "FOLLOWER", "load_score": 0.25},
        {"drone_id": 9203, "battery_pct": 79.0, "role": "FOLLOWER", "load_score": 0.15},
        {"drone_id": 9204, "battery_pct": 76.0, "role": "FOLLOWER", "load_score": 0.3},
    ]
    return [
        MissionScenario(
            name="dynamic_route_selection",
            objective="reach corridor exit while minimizing obstacle and link risk",
            routes=[
                RouteOption("corridor_alpha", 180.0, 0.28, 0.34, 0.22),
                RouteOption("corridor_bravo", 160.0, 0.46, 0.42, 0.35),
            ],
            available_drones=drones,
            mission_pressure=0.35,
            link_quality=0.88,
            failure_mode="nominal",
            expected_route="corridor_alpha",
            expected_success=True,
            expected_recovery=False,
        ),
        MissionScenario(
            name="risk_aware_replan",
            objective="maintain mission progress under communication degradation",
            routes=[
                RouteOption("mesh_safe_lane", 210.0, 0.32, 0.44, 0.12),
                RouteOption("fast_but_exposed", 150.0, 0.51, 0.28, 0.68),
            ],
            available_drones=drones,
            mission_pressure=0.57,
            link_quality=0.42,
            failure_mode="link_loss",
            expected_route="mesh_safe_lane",
            expected_success=True,
            expected_recovery=True,
        ),
        MissionScenario(
            name="resource_aware_allocation",
            objective="reallocate mapping task after battery drop",
            routes=[
                RouteOption("reserve_supported", 190.0, 0.30, 0.26, 0.24),
                RouteOption("battery_aggressive", 155.0, 0.35, 0.59, 0.21),
            ],
            available_drones=drones,
            mission_pressure=0.63,
            link_quality=0.74,
            failure_mode="battery_drop",
            expected_route="reserve_supported",
            expected_success=True,
            expected_recovery=True,
        ),
        MissionScenario(
            name="failure_aware_sensor_fault",
            objective="degrade safely when localization confidence falls",
            routes=[
                RouteOption("safe_corridor", 205.0, 0.24, 0.31, 0.18),
                RouteOption("high_gain_lane", 145.0, 0.58, 0.36, 0.45),
            ],
            available_drones=drones,
            mission_pressure=0.49,
            link_quality=0.68,
            failure_mode="sensor_fault",
            expected_route="safe_corridor",
            expected_success=True,
            expected_recovery=True,
        ),
    ]


def main() -> int:
    DOC_ROOT.mkdir(parents=True, exist_ok=True)
    planner = AIMissionPlanner(
        MockOnnxAdapter(
            ModelMetadata(
                model_name="phase9-planner-mock",
                model_version="1.0.0",
                backend="onnx-mock",
                task="mission_planning",
                input_schema="phase9/planner_input_v1",
                output_schema="phase9/planner_output_v1",
            )
        )
    )
    scenarios = build_scenarios()
    decisions = []
    latencies: list[float] = []
    successes = 0
    recoveries = 0
    for scenario in scenarios:
        decision = planner.plan(scenario)
        latencies.append(decision.latency_ms)
        successes += 1 if decision.success else 0
        recoveries += 1 if decision.recovery_success else 0
        decisions.append(
            {
                "scenario": decision.scenario,
                "selected_route": decision.selected_route,
                "route_score": decision.route_score,
                "task_allocation": decision.task_allocation,
                "replan_triggered": decision.replan_triggered,
                "recovery_plan": decision.recovery_plan,
                "success": decision.success,
                "recovery_success": decision.recovery_success,
                "latency_ms": decision.latency_ms,
                "confidence_score": decision.confidence_score,
                "model_prediction": decision.model_prediction,
            }
        )
    payload = {
        "generated_at": utc_now(),
        "environment": {"platform": platform.platform(), "python_version": sys.version},
        "scenario_count": len(decisions),
        "decisions": decisions,
        "summary": {
            "success_rate": successes / len(decisions),
            "recovery_rate": recoveries / len(decisions),
            "decision_timing_ms": summarize_latency(latencies),
        },
        "status": (
            "PASS"
            if successes == len(decisions) and recoveries == len(decisions)
            else "FAIL"
        ),
    }
    write_json(OUTPUT_JSON, payload)
    OUTPUT_MD.write_text(
        "\n".join(
            [
                "# Phase 9 Mission Planning Report",
                "",
                "Date: July 16, 2026",
                "",
                "## Summary",
                "",
                f"- planning scenarios: `{len(decisions)}`",
                f"- success rate: `{payload['summary']['success_rate']:.3f}`",
                f"- recovery rate: `{payload['summary']['recovery_rate']:.3f}`",
                f"- decision timing p95: `{payload['summary']['decision_timing_ms']['p95_ms']:.3f} ms`",
                "",
                "## Evidence",
                "",
                f"- machine-readable artifact: `{OUTPUT_JSON.relative_to(REPO_ROOT)}`",
                "- dynamic route selection recorded",
                "- risk-aware replanning recorded",
                "- multi-drone task allocation recorded",
                "",
                "## Verdict",
                "",
                f"Status: {payload['status']}",
                "",
            ]
        ),
        encoding="utf-8",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
