#!/usr/bin/env python3
"""Phase 9 swarm intelligence simulation."""

from __future__ import annotations

import platform
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from research.ai.common import summarize_latency, utc_now, write_json
from simulation.swarm_ai import SwarmAgent, SwarmCoordinator, SwarmScenario


DOC_ROOT = REPO_ROOT / "docs" / "phase9"
OUTPUT_JSON = DOC_ROOT / "swarm_intelligence_results.json"
OUTPUT_MD = DOC_ROOT / "SWARM_INTELLIGENCE_REPORT.md"


def base_agents() -> list[SwarmAgent]:
    return [
        SwarmAgent(9301, "LEADER", 0.0, 0.0, 88.0, 0.92, 0.91, 0.18),
        SwarmAgent(9302, "FOLLOWER", 1.2, 0.4, 84.0, 0.89, 0.86, 0.24),
        SwarmAgent(9303, "FOLLOWER", -1.1, 0.5, 81.0, 0.83, 0.82, 0.21),
        SwarmAgent(9304, "FOLLOWER", 0.1, -1.3, 79.0, 0.8, 0.79, 0.27),
    ]


def build_scenarios() -> list[SwarmScenario]:
    return [
        SwarmScenario(
            name="formation_management",
            objective="maintain distributed formation with stable leader coordination",
            agents=base_agents(),
            communication_fault=0.05,
            leader_fault=False,
            task_pressure=0.3,
            expected_stable=True,
        ),
        SwarmScenario(
            name="leader_fault_recovery",
            objective="replace leader and preserve cooperative task execution",
            agents=base_agents(),
            communication_fault=0.12,
            leader_fault=True,
            task_pressure=0.45,
            expected_stable=True,
        ),
        SwarmScenario(
            name="communication_aware_coordination",
            objective="adapt task assignment under degraded mesh quality",
            agents=base_agents(),
            communication_fault=0.28,
            leader_fault=False,
            task_pressure=0.52,
            expected_stable=True,
        ),
    ]


def main() -> int:
    DOC_ROOT.mkdir(parents=True, exist_ok=True)
    coordinator = SwarmCoordinator()
    scenarios = []
    latencies: list[float] = []
    formation_scores: list[float] = []
    recovery_times: list[float] = []
    passes = 0
    for scenario in build_scenarios():
        result = coordinator.simulate(scenario)
        scenarios.append(result)
        latencies.append(float(result["latency_ms"]))
        formation_scores.append(float(result["formation_stability"]))
        recovery_times.append(float(result["recovery_time_s"]))
        passes += 1 if result["pass"] else 0
    payload = {
        "generated_at": utc_now(),
        "environment": {"platform": platform.platform(), "python_version": sys.version},
        "scenario_count": len(scenarios),
        "scenarios": scenarios,
        "summary": {
            "pass_rate": passes / len(scenarios),
            "formation_stability_avg": sum(formation_scores) / len(formation_scores),
            "fault_recovery_avg_s": sum(recovery_times) / len(recovery_times),
            "latency_ms": summarize_latency(latencies),
        },
        "status": "PASS" if passes == len(scenarios) else "FAIL",
    }
    write_json(OUTPUT_JSON, payload)
    OUTPUT_MD.write_text(
        "\n".join(
            [
                "# Phase 9 Swarm Intelligence Report",
                "",
                "Date: July 16, 2026",
                "",
                "## Summary",
                "",
                f"- scenarios: `{len(scenarios)}`",
                f"- pass rate: `{payload['summary']['pass_rate']:.3f}`",
                f"- average formation stability: `{payload['summary']['formation_stability_avg']:.3f}`",
                f"- average recovery time: `{payload['summary']['fault_recovery_avg_s']:.3f} s`",
                "",
                "## Validation Scope",
                "",
                "- multi-agent coordination",
                "- formation management",
                "- leader fault recovery",
                "- communication-aware task assignment",
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
