from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from research.ai.common import clamp


@dataclass(frozen=True)
class ModelMetadata:
    model_name: str
    model_version: str
    backend: str
    task: str
    input_schema: str
    output_schema: str


@dataclass(frozen=True)
class InferenceResult:
    prediction: str
    confidence: float
    backend: str
    latency_ms: float
    metadata: ModelMetadata
    features: dict[str, Any]


class BaseModelAdapter:
    def __init__(self, metadata: ModelMetadata) -> None:
        self.metadata = metadata

    def infer(self, features: dict[str, Any]) -> InferenceResult:
        raise NotImplementedError


class MockPyTorchAdapter(BaseModelAdapter):
    def infer(self, features: dict[str, Any]) -> InferenceResult:
        obstacle_score = float(features.get("obstacle_density", 0.0))
        confidence_seed = float(features.get("localization_confidence", 0.0))
        confidence = clamp(0.55 + (confidence_seed * 0.35) + (obstacle_score * 0.1), 0.0, 0.99)
        if obstacle_score >= 0.7:
            prediction = "avoid_obstacle_cluster"
        elif confidence_seed < 0.45:
            prediction = "fallback_localization"
        else:
            prediction = "continue_nominal_track"
        return InferenceResult(
            prediction=prediction,
            confidence=confidence,
            backend="pytorch-mock",
            latency_ms=1.85 + (obstacle_score * 0.45),
            metadata=self.metadata,
            features=features,
        )


class MockOnnxAdapter(BaseModelAdapter):
    def infer(self, features: dict[str, Any]) -> InferenceResult:
        link_quality = float(features.get("link_quality", 1.0))
        mission_pressure = float(features.get("mission_pressure", 0.0))
        confidence = clamp(0.62 + (link_quality * 0.25) - (mission_pressure * 0.08), 0.0, 0.98)
        if link_quality < 0.55:
            prediction = "communication_resilient_plan"
        elif mission_pressure > 0.7:
            prediction = "resource_constrained_plan"
        else:
            prediction = "balanced_plan"
        return InferenceResult(
            prediction=prediction,
            confidence=confidence,
            backend="onnx-mock",
            latency_ms=1.35 + ((1.0 - link_quality) * 0.4),
            metadata=self.metadata,
            features=features,
        )

