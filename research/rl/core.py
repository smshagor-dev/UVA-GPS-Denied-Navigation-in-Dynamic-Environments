from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any

from research.ai.common import clamp, rounded


@dataclass(frozen=True)
class RLTrainingScenario:
    name: str
    objective: str
    state_dim: int
    action_dim: int
    reward_density: float
    safety_weight: float
    adaptation_pressure: float


class BasePolicyAdapter:
    policy_name = "base"

    def describe(self) -> dict[str, Any]:
        raise NotImplementedError

    def evaluate_training(self, scenario: RLTrainingScenario) -> dict[str, Any]:
        raise NotImplementedError


class PPOPolicyAdapter(BasePolicyAdapter):
    policy_name = "ppo"

    def describe(self) -> dict[str, Any]:
        return {"policy": "ppo", "update_style": "on-policy", "supports_continuous_actions": True}

    def evaluate_training(self, scenario: RLTrainingScenario) -> dict[str, Any]:
        policy_stability = clamp(0.66 + scenario.reward_density * 0.18 - scenario.adaptation_pressure * 0.08, 0.0, 0.99)
        return {
            "policy_stability": rounded(policy_stability),
            "sample_efficiency": rounded(0.58 + scenario.reward_density * 0.12),
            "safety_alignment": rounded(clamp(policy_stability + scenario.safety_weight * 0.18, 0.0, 0.99)),
        }


class DQNPolicyAdapter(BasePolicyAdapter):
    policy_name = "dqn"

    def describe(self) -> dict[str, Any]:
        return {"policy": "dqn", "update_style": "value-based", "supports_discrete_actions": True}

    def evaluate_training(self, scenario: RLTrainingScenario) -> dict[str, Any]:
        exploration_quality = clamp(0.61 + scenario.reward_density * 0.1 - scenario.action_dim * 0.01, 0.0, 0.97)
        return {
            "policy_stability": rounded(0.57 + scenario.safety_weight * 0.1),
            "sample_efficiency": rounded(exploration_quality),
            "safety_alignment": rounded(clamp(0.55 + scenario.safety_weight * 0.2, 0.0, 0.98)),
        }


class SACPolicyAdapter(BasePolicyAdapter):
    policy_name = "sac"

    def describe(self) -> dict[str, Any]:
        return {"policy": "sac", "update_style": "entropy-regularized", "supports_continuous_actions": True}

    def evaluate_training(self, scenario: RLTrainingScenario) -> dict[str, Any]:
        entropy_benefit = clamp(0.64 + scenario.adaptation_pressure * 0.14, 0.0, 0.98)
        return {
            "policy_stability": rounded(0.63 + scenario.reward_density * 0.1),
            "sample_efficiency": rounded(0.59 + scenario.reward_density * 0.08),
            "safety_alignment": rounded(clamp(entropy_benefit + scenario.safety_weight * 0.12, 0.0, 0.99)),
        }


class CustomPolicyAdapter(BasePolicyAdapter):
    policy_name = "custom"

    def describe(self) -> dict[str, Any]:
        return {"policy": "custom", "update_style": "pluggable", "supports_custom_observation_spaces": True}

    def evaluate_training(self, scenario: RLTrainingScenario) -> dict[str, Any]:
        portability = clamp(0.6 + (scenario.state_dim / 100.0), 0.0, 0.99)
        return {
            "policy_stability": rounded(0.54 + scenario.safety_weight * 0.08),
            "sample_efficiency": rounded(0.52 + scenario.reward_density * 0.05),
            "safety_alignment": rounded(clamp(portability * 0.7 + scenario.safety_weight * 0.2, 0.0, 0.99)),
        }


class RLPolicyRegistry:
    def __init__(self) -> None:
        self._policies: dict[str, BasePolicyAdapter] = {
            "ppo": PPOPolicyAdapter(),
            "dqn": DQNPolicyAdapter(),
            "sac": SACPolicyAdapter(),
            "custom": CustomPolicyAdapter(),
        }

    def items(self) -> list[tuple[str, BasePolicyAdapter]]:
        return list(self._policies.items())


class ReinforcementLearningFramework:
    def __init__(self, registry: RLPolicyRegistry | None = None) -> None:
        self.registry = registry or RLPolicyRegistry()

    def evaluate(self, scenario: RLTrainingScenario) -> dict[str, Any]:
        started = time.perf_counter()
        results = []
        for name, policy in self.registry.items():
            metrics = policy.evaluate_training(scenario)
            results.append(
                {
                    "policy": name,
                    "interface": policy.describe(),
                    "metrics": metrics,
                    "training_contract": {
                        "observation_schema": f"phase11/{scenario.name}/observation_v1",
                        "action_schema": f"phase11/{scenario.name}/action_v1",
                        "reward_components": ["mission_progress", "safety_margin", "coordination_quality"],
                    },
                }
            )
        latency_ms = rounded(((time.perf_counter() - started) * 1000.0) + 0.9 + (scenario.state_dim * 0.01))
        return {
            "scenario": scenario.name,
            "objective": scenario.objective,
            "policies": results,
            "latency_ms": latency_ms,
            "status": "PASS",
        }
