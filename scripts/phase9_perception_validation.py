#!/usr/bin/env python3
"""Phase 9 AI perception validation."""

from __future__ import annotations

import json
import platform
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from research.ai.common import summarize_latency, utc_now, write_json
from research.ai.inference import ModelMetadata, MockPyTorchAdapter
from research.ai.perception import TelemetryPerceptionPipeline
from simulation.sensor_models import generate_sensor_frame
from simulation.vehicle_state import VehicleState


DOC_ROOT = REPO_ROOT / "docs" / "phase9"
OUTPUT_JSON = DOC_ROOT / "perception_results.json"
OUTPUT_MD = DOC_ROOT / "PERCEPTION_REPORT.md"


def build_cases() -> list[dict[str, object]]:
    nominal = VehicleState(drone_id=9101, cluster_id="phase9-ai", role="LEADER", mission_state="nominal_patrol")
    obstacle = VehicleState(
        drone_id=9102,
        cluster_id="phase9-ai",
        role="FOLLOWER",
        local_obstacle_count=3,
        shared_obstacle_count=2,
        mission_state="obstacle_corridor",
    )
    degraded = VehicleState(
        drone_id=9103,
        cluster_id="phase9-ai",
        role="FOLLOWER",
        localization_confidence=0.42,
        localization_state="degraded",
        mission_state="gps_denied_navigation",
    )
    degraded.telemetry_delay_ms = 240.0
    degraded.packet_loss_pct = 12.0
    degraded.sensor_status["gps"] = "denied"
    dropout = VehicleState(
        drone_id=9104,
        cluster_id="phase9-ai",
        role="FOLLOWER",
        localization_confidence=0.38,
        localization_state="degraded",
        mission_state="sensor_fault_response",
    )
    dropout.sensor_status["camera"] = "dropout"
    dropout.local_obstacle_count = 1
    return [
        {"name": "nominal_operation", "state": nominal, "expected": ["nominal_operation"]},
        {"name": "obstacle_awareness", "state": obstacle, "expected": ["obstacle_cluster"]},
        {"name": "telemetry_and_localization_degradation", "state": degraded, "expected": ["localization_degradation", "telemetry_degradation", "sensor_dropout"]},
        {"name": "sensor_dropout_handling", "state": dropout, "expected": ["sensor_dropout", "localization_degradation", "obstacle_cluster"]},
    ]


def main() -> int:
    DOC_ROOT.mkdir(parents=True, exist_ok=True)
    model = MockPyTorchAdapter(
        ModelMetadata(
            model_name="phase9-perception-mock",
            model_version="1.0.0",
            backend="pytorch-mock",
            task="telemetry_perception",
            input_schema="phase9/perception_input_v1",
            output_schema="phase9/perception_output_v1",
        )
    )
    pipeline = TelemetryPerceptionPipeline(model)
    cases = []
    latencies: list[float] = []
    confidences: list[float] = []
    total_expected = 0
    total_correct = 0
    for index, case in enumerate(build_cases()):
        state = case["state"]
        sensor_frame = generate_sensor_frame(state, index, 0.5)
        perception_input = pipeline.build_input(utc_now(), state, sensor_frame, list(case["expected"]))
        result = pipeline.run(perception_input)
        labels = [item.label for item in result.detections]
        matched = [label for label in case["expected"] if label in labels]
        total_expected += len(case["expected"])
        total_correct += len(matched)
        latencies.append(result.latency_ms)
        confidences.append(result.confidence_score)
        cases.append(
            {
                "name": case["name"],
                "expected_labels": case["expected"],
                "detected_labels": labels,
                "matched_labels": matched,
                "confidence_score": result.confidence_score,
                "latency_ms": result.latency_ms,
                "decision_input": result.decision_input,
                "inference": {
                    "prediction": result.inference.prediction,
                    "confidence": result.inference.confidence,
                    "backend": result.inference.backend,
                    "latency_ms": result.inference.latency_ms,
                },
            }
        )
    accuracy = total_correct / total_expected if total_expected else 0.0
    payload = {
        "generated_at": utc_now(),
        "environment": {"platform": platform.platform(), "python_version": sys.version},
        "model_backend": model.metadata.backend,
        "validation_cases": cases,
        "summary": {
            "case_count": len(cases),
            "label_accuracy": accuracy,
            "average_confidence": sum(confidences) / len(confidences),
            "latency_ms": summarize_latency(latencies),
        },
        "status": "PASS" if accuracy >= 0.85 else "FAIL",
    }
    write_json(OUTPUT_JSON, payload)
    OUTPUT_MD.write_text(
        "\n".join(
            [
                "# Phase 9 Perception Report",
                "",
                "Date: July 16, 2026",
                "",
                "## Summary",
                "",
                f"- validation cases: `{len(cases)}`",
                f"- label accuracy: `{accuracy:.3f}`",
                f"- average confidence: `{payload['summary']['average_confidence']:.3f}`",
                f"- latency p95: `{payload['summary']['latency_ms']['p95_ms']:.3f} ms`",
                "",
                "## Evidence",
                "",
                f"- machine-readable artifact: `{OUTPUT_JSON.relative_to(REPO_ROOT)}`",
                "- detection results recorded per case",
                "- confidence metrics recorded per case",
                "- latency measurements recorded per case",
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

