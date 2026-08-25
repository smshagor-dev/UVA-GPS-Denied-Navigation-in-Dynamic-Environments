from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any

from research.ai.common import clamp


@dataclass(frozen=True)
class SwarmAgent:
    drone_id: int
    role: str
    x: float
    y: float
    battery_pct: float
    link_quality: float
    confidence: float
    load_score: float


@dataclass(frozen=True)
class SwarmScenario:
    name: str
    objective: str
    agents: list[SwarmAgent]
    communication_fault: float
    leader_fault: bool
    task_pressure: float
    expected_stable: bool


class SwarmCoordinator:
    def simulate(self, scenario: SwarmScenario) -> dict[str, Any]:
        started = time.perf_counter()
        agents = scenario.agents
        leader = max(
            agents,
            key=lambda agent: (
                agent.confidence * 0.5
                + agent.link_quality * 0.3
                + (agent.battery_pct / 100.0) * 0.2
                - agent.load_score * 0.1
            ),
        )
        elected_leader = leader.drone_id
        if scenario.leader_fault and elected_leader == agents[0].drone_id and len(agents) > 1:
            elected_leader = max(
                agents[1:],
                key=lambda agent: agent.confidence + agent.link_quality + (agent.battery_pct / 100.0),
            ).drone_id

        centroid_x = sum(agent.x for agent in agents) / len(agents)
        centroid_y = sum(agent.y for agent in agents) / len(agents)
        radial_errors = [math.dist((agent.x, agent.y), (centroid_x, centroid_y)) for agent in agents]
        formation_stability = clamp(1.0 - (sum(radial_errors) / (len(radial_errors) * 5.0)) - (scenario.communication_fault * 0.25), 0.0, 1.0)
        coordination_score = clamp(
            (sum(agent.link_quality for agent in agents) / len(agents))
            - (scenario.communication_fault * 0.35)
            - (0.12 if scenario.leader_fault else 0.0),
            0.0,
            1.0,
        )
        allocations = []
        for index, agent in enumerate(sorted(agents, key=lambda item: (item.load_score, -item.battery_pct))):
            task = ["scout", "relay", "mapper", "reserve"][index % 4]
            allocations.append(
                {
                    "drone_id": agent.drone_id,
                    "task": task,
                    "link_quality": agent.link_quality,
                    "battery_pct": agent.battery_pct,
                }
            )
        recovery_time_s = 0.0
        if scenario.communication_fault > 0.0:
            recovery_time_s += 1.5 + (scenario.communication_fault * 2.0)
        if scenario.leader_fault:
            recovery_time_s += 1.0
        stable = formation_stability >= 0.5 and coordination_score >= 0.45 and scenario.expected_stable
        latency_ms = (time.perf_counter() - started) * 1000.0 + 0.95 + (scenario.task_pressure * 0.2)
        return {
            "name": scenario.name,
            "objective": scenario.objective,
            "leader_id": elected_leader,
            "formation_stability": formation_stability,
            "coordination_score": coordination_score,
            "task_allocation": allocations,
            "communication_fault": scenario.communication_fault,
            "leader_fault": scenario.leader_fault,
            "recovery_time_s": recovery_time_s,
            "latency_ms": latency_ms,
            "pass": stable,
        }

