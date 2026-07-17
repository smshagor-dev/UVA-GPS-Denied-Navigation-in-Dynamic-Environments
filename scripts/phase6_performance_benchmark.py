#!/usr/bin/env python3
"""Run Phase 6 benchmark-focused measurements and save raw results."""

from __future__ import annotations

import json
import time
import urllib.request
from dataclasses import asdict
from datetime import UTC, datetime
from pathlib import Path

import phase6_performance_suite as suite


OUTPUT_PATH = suite.DOC_ROOT / "benchmark_results.json"


def benchmark_metrics_get(base_url: str, iterations: int) -> suite.Metric:
    samples_ms: list[float] = []
    for _ in range(iterations):
        started = time.perf_counter()
        request = urllib.request.Request(f"{base_url}/metrics", headers={"Accept": "text/plain"}, method="GET")
        with urllib.request.urlopen(request, timeout=10) as response:
            body = response.read().decode("utf-8", errors="replace")
            status = response.status
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        if status != 200 or "drone_swarm_controlplane_ready" not in body:
            raise RuntimeError(f"metrics GET failed: status={status} body={body}")
        samples_ms.append(elapsed_ms)
    return suite.summarize("backend_metrics_get", "http", samples_ms, {"url": f"{base_url}/metrics"})


def benchmark_command_posts(base_url: str, iterations: int) -> suite.Metric:
    samples_ms: list[float] = []
    accepted = 0
    for index in range(iterations):
        payload = {
            "action": "return_home",
            "payload": {
                "target_ids": [1],
                "cluster_id": "cluster-01",
                "request_id": f"phase6-command-{index}",
            },
        }
        started = time.perf_counter()
        status, body = suite.request_json("POST", f"{base_url}/api/v1/commands", payload)
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        if status != 202:
            raise RuntimeError(f"command POST failed: status={status} payload={body}")
        samples_ms.append(elapsed_ms)
        accepted += 1
    throughput_hz = accepted / (sum(samples_ms) / 1000.0) if samples_ms else 0.0
    return suite.summarize(
        "backend_command_post",
        "http",
        samples_ms,
        {"url": f"{base_url}/api/v1/commands", "accepted": accepted, "throughput_hz": throughput_hz},
    )


def main() -> int:
    suite.DOC_ROOT.mkdir(parents=True, exist_ok=True)
    metrics: list[suite.Metric] = []

    metrics.append(suite.benchmark_config_load(suite.REPO_ROOT / "config" / "runtime.json", 20))
    metrics.append(suite.benchmark_config_load(suite.REPO_ROOT / "config" / "anchors.json", 20))
    metrics.append(suite.benchmark_config_load(suite.REPO_ROOT / "config" / "lidar.json", 20))
    metrics.append(
        suite.benchmark_native_test(
            "ekf_update_path",
            suite.TEST_ROOT / "test_ekf.exe",
            ["--gtest_filter=EKFTest.FreeFallVelocity"],
            30,
        )
    )
    metrics.append(
        suite.benchmark_native_test(
            "lidar_processing_path",
            suite.TEST_ROOT / "test_sensors.exe",
            ["--gtest_filter=LidarSensor.ObstacleListGeneratedFromValidScan"],
            30,
        )
    )
    metrics.append(
        suite.benchmark_native_test(
            "sensor_fusion_path",
            suite.TEST_ROOT / "test_navigation_intelligence.exe",
            ["--gtest_filter=LocalizationFusion.PrefersTdoaWhenVioDriftIsHigh"],
            20,
        )
    )
    metrics.append(
        suite.benchmark_native_test(
            "telemetry_serialization_path",
            suite.TEST_ROOT / "test_telemetry.exe",
            ["--gtest_filter=ControlPlaneTelemetryClient.SerializesTelemetryPayload"],
            40,
        )
    )
    metrics.append(
        suite.benchmark_native_test(
            "security_packet_auth_path",
            suite.TEST_ROOT / "test_edge_swarm.exe",
            ["--gtest_filter=PeerPacketAuth.HmacSignedPacketVerifies"],
            40,
        )
    )

    production_port = suite.allocate_free_port()
    production_log = suite.DOC_ROOT / "backend_benchmark.log"
    benchmark_process, startup_ms = suite.start_backend(production_port, production_log)
    try:
        base_url = f"http://127.0.0.1:{production_port}"
        metrics.append(suite.summarize("backend_startup", "process", [startup_ms], {"mode": "production"}))
        metrics.append(suite.benchmark_backend_reads(base_url, 30))
        metrics.append(suite.benchmark_telemetry_posts(base_url, 30))
        metrics.append(benchmark_metrics_get(base_url, 30))
    finally:
        suite.stop_backend(benchmark_process)

    simulation_port = suite.allocate_free_port()
    simulation_log = suite.DOC_ROOT / "backend_benchmark_commands.log"
    sim_process, _ = suite.start_backend(
        simulation_port,
        simulation_log,
        mode="simulation",
        simulation_enabled=True,
    )
    try:
        base_url = f"http://127.0.0.1:{simulation_port}"
        metrics.append(benchmark_command_posts(base_url, 30))
    finally:
        suite.stop_backend(sim_process)

    payload = {
        "generated_at": datetime.now(UTC).isoformat().replace("+00:00", "Z"),
        "metrics": [asdict(metric) for metric in metrics],
        "notes": [
            "Phase 6 benchmark results were collected on the local Windows workspace.",
            "Backend command latency was measured against the lab-profile control plane using unsigned commands in benchmark mode.",
        ],
    }
    OUTPUT_PATH.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
