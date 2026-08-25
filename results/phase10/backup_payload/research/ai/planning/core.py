from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any

from research.ai.common import clamp
from research.ai.inference import BaseModelAdapter


@dataclass(frozen=True)
class RouteOption:
    name: str
    distance_m: float
    risk_score: float
    resource_cost: float
    link_exposure: float


@dataclass(frozen=True)
class MissionScenario:
    name: str
    objective: str
    routes: list[RouteOption]
    available_drones: list[dict[str, Any]]
    mission_pressure: float
    link_quality: float
    failure_mode: str
    expected_route: str
    expected_success: bool
    expected_recovery: bool


@dataclass(frozen=True)
class PlanningDecision:
    scenario: str
    selected_route: str
    route_score: float
    task_allocation: list[dict[str, Any]]
    replan_triggered: bool
    recovery_plan: str
    success: bool
    recovery_success: bool
    latency_ms: float
    confidence_score: float
    model_prediction: str


class AIMissionPlanner:
    def __init__(self, model: BaseModelAdapter) -> None:
        self.model = model

    def plan(self, scenario: MissionScenario) -> PlanningDecision:
        started = time.perf_counter()
        inference = self.model.infer(
            {
                "link_quality": scenario.link_quality,
                "mission_pressure": scenario.mission_pressure,
                "obstacle_density": sum(route.risk_score for route in scenario.routes) / max(1, len(scenario.routes)),
                "localization_confidence": 0.8 if scenario.failure_mode == "nominal" else 0.56,
            }
        )
        scored_routes: list[tuple[RouteOption, float]] = []
        for route in scenario.routes:
            score = (
                (route.distance_m * 0.25)
                + (route.risk_score * 45.0)
                + (route.resource_cost * 18.0)
                + (route.link_exposure * (1.0 - scenario.link_quality) * 20.0)
            )
            if scenario.failure_mode == "link_loss":
                score += route.link_exposure * 15.0
            if scenario.failure_mode == "battery_drop":
                score += route.resource_cost * 12.0
            scored_routes.append((route, score))
        selected_route, route_score = min(scored_routes, key=lambda item: item[1])
        sorted_drones = sorted(
            scenario.available_drones,
            key=lambda drone: (
                -float(drone["battery_pct"]),
                0 if drone["role"] == "LEADER" else 1,
                float(drone["load_score"]),
            ),
        )
        task_allocation = [
            {
                "drone_id": drone["drone_id"],
                "task": task,
                "battery_pct": drone["battery_pct"],
                "role": drone["role"],
            }
            for drone, task in zip(sorted_drones, ["recon", "relay", "mapping", "reserve"], strict=False)
        ]
        replan_triggered = scenario.failure_mode != "nominal"
        recovery_plan = {
            "nominal": "continue_mission",
            "link_loss": "tighten_formation_and_use_resilient_route",
            "battery_drop": "reallocate_mapping_to_reserve_drone",
            "sensor_fault": "fallback_to_safe_corridor_and_reduce_speed",
        }.get(scenario.failure_mode, "safe_hold_and_reassess")
        success = selected_route.name == scenario.expected_route and scenario.expected_success
        recovery_success = (not replan_triggered and not scenario.expected_recovery) or (
            replan_triggered and scenario.expected_recovery
        )
        latency_ms = ((time.perf_counter() - started) * 1000.0) + inference.latency_ms
        return PlanningDecision(
            scenario=scenario.name,
            selected_route=selected_route.name,
            route_score=route_score,
            task_allocation=task_allocation,
            replan_triggered=replan_triggered,
            recovery_plan=recovery_plan,
            success=success,
            recovery_success=recovery_success,
            latency_ms=latency_ms,
            confidence_score=clamp((1.0 / (1.0 + (route_score / 100.0))) + (inference.confidence * 0.4), 0.0, 0.99),
            model_prediction=inference.prediction,
        )

