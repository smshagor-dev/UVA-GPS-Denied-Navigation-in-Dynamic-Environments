#!/usr/bin/env python3
"""Phase 11 multi-agent decision framework evidence generator."""

from __future__ import annotations

import platform
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from research.ai.common import summarize_latency, utc_now, write_json
from research.multi_agent import AgentDecisionContext, MultiAgentDecisionFramework, MultiAgentScenario


DOC_ROOT = REPO_ROOT / "docs" / "phase11"
OUTPUT_JSON = DOC_ROOT / "multi_agent_results.json"
OUTPUT_MD = DOC_ROOT / "MULTI_AGENT_REPORT.md"


def build_scenarios() -> list[MultiAgentScenario]:
    return [
        MultiAgentScenario(
            name="cooperative_mapping",
            objective="allocate mapping and relay tasks under nominal mesh conditions",
            agents=[
                AgentDecisionContext(1101, "LEADER", 92.0, 0.91, 0.93, 0.18, (0.0, 0.0)),
                AgentDecisionContext(1102, "FOLLOWER", 86.0, 0.88, 0.89, 0.22, (1.1, 0.3)),
                AgentDecisionContext(1103, "FOLLOWER", 81.0, 0.84, 0.86, 0.27, (-0.8, 0.5)),
                AgentDecisionContext(1104, "FOLLOWER", 79.0, 0.8, 0.82, 0.21, (0.3, -1.2)),
            ],
            task_demand=0.42,
            conflict_level=0.18,
            communication_noise=0.06,
            estimator_degradation=0.04,
            requires_leader_rotation=False,
        ),
        MultiAgentScenario(
            name="leader_reselection",
            objective="recover mission coordination during leader degradation",
            agents=[
                AgentDecisionContext(1111, "LEADER", 78.0, 0.66, 0.58, 0.42, (0.0, 0.0)),
                AgentDecisionContext(1112, "FOLLOWER", 88.0, 0.9, 0.91, 0.2, (0.8, 0.4)),
                AgentDecisionContext(1113, "FOLLOWER", 84.0, 0.86, 0.87, 0.24, (-0.7, 0.2)),
                AgentDecisionContext(1114, "FOLLOWER", 82.0, 0.81, 0.83, 0.19, (0.2, -0.9)),
            ],
            task_demand=0.51,
            conflict_level=0.34,
            communication_noise=0.11,
            estimator_degradation=0.15,
            requires_leader_rotation=True,
        ),
        MultiAgentScenario(
            name="conflict_resolution_under_delay",
            objective="resolve mission intent conflicts under degraded communication",
            agents=[
                AgentDecisionContext(1121, "LEADER", 89.0, 0.82, 0.88, 0.25, (0.0, 0.0)),
                AgentDecisionContext(1122, "FOLLOWER", 85.0, 0.74, 0.8, 0.31, (1.0, 0.1)),
                AgentDecisionContext(1123, "FOLLOWER", 76.0, 0.71, 0.79, 0.22, (-1.0, 0.4)),
                AgentDecisionContext(1124, "FOLLOWER", 83.0, 0.77, 0.81, 0.29, (0.2, -1.0)),
            ],
            task_demand=0.58,
            conflict_level=0.52,
            communication_noise=0.23,
            estimator_degradation=0.18,
            requires_leader_rotation=False,
        ),
    ]


def main() -> int:
    DOC_ROOT.mkdir(parents=True, exist_ok=True)
    framework = MultiAgentDecisionFramework()
    scenarios = [framework.evaluate(scenario) for scenario in build_scenarios()]
    latencies = [float(item["latency_ms"]) for item in scenarios]
    consensus = [float(item["consensus_score"]) for item in scenarios]
    cooperation = [float(item["cooperation_score"]) for item in scenarios]
    passes = sum(1 for item in scenarios if item["pass"])
    payload = {
        "generated_at": utc_now(),
        "environment": {"platform": platform.platform(), "python_version": sys.version},
        "scenario_count": len(scenarios),
        "scenarios": scenarios,
        "summary": {
            "pass_rate": passes / len(scenarios),
            "consensus_score_avg": sum(consensus) / len(consensus),
            "cooperation_score_avg": sum(cooperation) / len(cooperation),
            "latency_ms": summarize_latency(latencies),
        },
        "status": "PASS" if passes == len(scenarios) else "FAIL",
    }
    write_json(OUTPUT_JSON, payload)
    OUTPUT_MD.write_text(
        "\n".join(
            [
                "# Phase 11 Multi-Agent Report",
                "",
                "Date: July 17, 2026",
                "",
                "## Summary",
                "",
                f"- scenarios: `{len(scenarios)}`",
                f"- pass rate: `{payload['summary']['pass_rate']:.3f}`",
                f"- average consensus score: `{payload['summary']['consensus_score_avg']:.3f}`",
                f"- average cooperation score: `{payload['summary']['cooperation_score_avg']:.3f}`",
                f"- decision latency p95: `{payload['summary']['latency_ms']['p95_ms']:.3f} ms`",
                "",
                "## Implemented Capabilities",
                "",
                "- independent drone agents",
                "- cooperative decision making",
                "- consensus scoring",
                "- distributed task allocation",
                "- dynamic leader election",
                "- shared world state handoff",
                "- agent negotiation margins",
                "- mission conflict resolution",
                "",
                "## Artifact",
                "",
                f"- `{OUTPUT_JSON.relative_to(REPO_ROOT)}`",
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
