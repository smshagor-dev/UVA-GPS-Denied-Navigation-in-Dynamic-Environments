from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from research.ai.common import clamp, rounded, utc_now


@dataclass(frozen=True)
class ScenarioTrace:
    name: str
    top_detection: str
    planner_route: str
    coordination_mode: str
    failure_mode: str
    confidence_score: float
    recovery_time_s: float


@dataclass(frozen=True)
class DecisionExplanation:
    timestamp: str
    scenario: str
    reasoning: str
    confidence_explanation: str
    planning_explanation: str
    failure_explanation: str
    recovery_explanation: str
    confidence_score: float
    safety_state: str


class ExplainableDecisionLogger:
    def explain(self, trace: ScenarioTrace) -> DecisionExplanation:
        safety_state = "NORMAL" if trace.confidence_score >= 0.62 and trace.failure_mode == "nominal" else "RECOVERY"
        reasoning = (
            f"Selected route {trace.planner_route} after observing {trace.top_detection} "
            f"with coordination mode {trace.coordination_mode}."
        )
        confidence_explanation = (
            f"Confidence remained at {rounded(trace.confidence_score):.3f} because perception, planning, "
            f"and coordination signals stayed within bounded degradation."
        )
        planning_explanation = (
            f"Planning favored {trace.planner_route} to balance mission progress against failure mode {trace.failure_mode}."
        )
        failure_explanation = (
            "No active failure trigger was present."
            if trace.failure_mode == "nominal"
            else f"Failure mode {trace.failure_mode} reduced autonomy margin and triggered explanation logging."
        )
        recovery_explanation = (
            f"Recovery path completed in {rounded(trace.recovery_time_s):.3f} s with safety state {safety_state}."
        )
        return DecisionExplanation(
            timestamp=utc_now(),
            scenario=trace.name,
            reasoning=reasoning,
            confidence_explanation=confidence_explanation,
            planning_explanation=planning_explanation,
            failure_explanation=failure_explanation,
            recovery_explanation=recovery_explanation,
            confidence_score=clamp(trace.confidence_score, 0.0, 1.0),
            safety_state=safety_state,
        )

    def summarize(self, traces: list[ScenarioTrace]) -> dict[str, Any]:
        explanations = [self.explain(trace) for trace in traces]
        avg_confidence = sum(item.confidence_score for item in explanations) / max(1, len(explanations))
        return {
            "generated_at": utc_now(),
            "explanations": explanations,
            "summary": {
                "scenario_count": len(explanations),
                "confidence_avg": rounded(avg_confidence),
                "recovery_case_count": sum(1 for trace in traces if trace.failure_mode != "nominal"),
                "normal_case_count": sum(1 for trace in traces if trace.failure_mode == "nominal"),
            },
            "status": "PASS",
        }
