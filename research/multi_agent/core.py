from __future__ import annotations

import statistics
import time
from dataclasses import dataclass
from typing import Any

from research.ai.common import clamp, rounded
from simulation.swarm_ai import SwarmAgent


@dataclass(frozen=True)
class AgentDecisionContext:
    drone_id: int
    role: str
    battery_pct: float
    link_quality: float
    confidence: float
    workload: float
    position: tuple[float, float]


@dataclass(frozen=True)
class MultiAgentScenario:
    name: str
    objective: str
    agents: list[AgentDecisionContext]
    task_demand: float
    conflict_level: float
    communication_noise: float
    estimator_degradation: float
    requires_leader_rotation: bool


class MultiAgentDecisionFramework:
    def _elect_leader(self, scenario: MultiAgentScenario) -> AgentDecisionContext:
        ranked = sorted(
            scenario.agents,
            key=lambda agent: (
                agent.confidence * 0.4
                + agent.link_quality * 0.3
                + (agent.battery_pct / 100.0) * 0.2
                - (agent.workload * 0.1)
            ),
            reverse=True,
        )
        leader = ranked[0]
        if scenario.requires_leader_rotation and len(ranked) > 1:
            return ranked[1]
        return leader

    def _consensus_score(self, scenario: MultiAgentScenario) -> float:
        confidence_avg = statistics.fmean(agent.confidence for agent in scenario.agents)
        link_avg = statistics.fmean(agent.link_quality for agent in scenario.agents)
        score = (
            confidence_avg * 0.45
            + link_avg * 0.35
            + (1.0 - scenario.communication_noise) * 0.15
            + (1.0 - scenario.estimator_degradation) * 0.05
        )
        score -= scenario.conflict_level * 0.22
        return clamp(score, 0.0, 1.0)

    def _allocate_tasks(self, scenario: MultiAgentScenario, leader: AgentDecisionContext) -> list[dict[str, Any]]:
        priorities = ["leader", "mapper", "relay", "observer", "reserve"]
        assignments = []
        ordered = sorted(
            scenario.agents,
            key=lambda agent: (agent.drone_id != leader.drone_id, agent.workload, -agent.battery_pct),
        )
        for index, agent in enumerate(ordered):
            task = priorities[index] if index < len(priorities) else f"support-{index}"
            negotiation_margin = clamp(
                (agent.link_quality * 0.4) + (agent.confidence * 0.4) + ((agent.battery_pct / 100.0) * 0.2) - scenario.task_demand * 0.1,
                0.0,
                1.0,
            )
            assignments.append(
                {
                    "drone_id": agent.drone_id,
                    "task": task,
                    "accepted": negotiation_margin >= 0.42,
                    "negotiation_margin": rounded(negotiation_margin),
                    "workload": agent.workload,
                }
            )
        return assignments

    def _resolve_conflict(self, scenario: MultiAgentScenario, consensus_score: float) -> dict[str, Any]:
        mode = "direct_commit"
        if scenario.conflict_level >= 0.55:
            mode = "priority_vote"
        if scenario.communication_noise >= 0.25:
            mode = "delayed_quorum"
        success = consensus_score >= 0.48 and scenario.conflict_level < 0.8
        return {
            "resolution_mode": mode,
            "consensus_reached": success,
            "mission_conflict_resolved": success,
            "shared_state_updates": int(len(scenario.agents) * (2 + scenario.task_demand * 2)),
        }

    def evaluate(self, scenario: MultiAgentScenario) -> dict[str, Any]:
        started = time.perf_counter()
        leader = self._elect_leader(scenario)
        consensus_score = self._consensus_score(scenario)
        allocations = self._allocate_tasks(scenario, leader)
        conflict = self._resolve_conflict(scenario, consensus_score)
        accepted = sum(1 for item in allocations if item["accepted"])
        cooperation_score = clamp(
            (accepted / max(1, len(allocations))) * 0.55
            + consensus_score * 0.35
            + (1.0 - scenario.estimator_degradation) * 0.10,
            0.0,
            1.0,
        )
        recovery_time_s = rounded(
            0.8 + scenario.communication_noise * 1.8 + scenario.conflict_level * 1.2 + scenario.estimator_degradation * 1.4
        )
        shared_world_state = {
            "active_leader": leader.drone_id,
            "team_size": len(scenario.agents),
            "shared_confidence": rounded(statistics.fmean(agent.confidence for agent in scenario.agents)),
            "quorum_fraction": rounded(accepted / max(1, len(allocations))),
        }
        latency_ms = rounded(
            ((time.perf_counter() - started) * 1000.0)
            + 1.2
            + (scenario.task_demand * 0.7)
            + (scenario.communication_noise * 2.3)
        )
        return {
            "name": scenario.name,
            "objective": scenario.objective,
            "leader_id": leader.drone_id,
            "requires_leader_rotation": scenario.requires_leader_rotation,
            "consensus_score": rounded(consensus_score),
            "cooperation_score": rounded(cooperation_score),
            "task_allocations": allocations,
            "conflict_resolution": conflict,
            "shared_world_state": shared_world_state,
            "dynamic_leader_election": leader.drone_id != scenario.agents[0].drone_id if scenario.agents else False,
            "recovery_time_s": recovery_time_s,
            "latency_ms": latency_ms,
            "pass": consensus_score >= 0.48 and cooperation_score >= 0.5 and conflict["mission_conflict_resolved"],
        }


def from_swarm_agent(agent: SwarmAgent) -> AgentDecisionContext:
    return AgentDecisionContext(
        drone_id=agent.drone_id,
        role=agent.role,
        battery_pct=agent.battery_pct,
        link_quality=agent.link_quality,
        confidence=agent.confidence,
        workload=agent.load_score,
        position=(agent.x, agent.y),
    )
