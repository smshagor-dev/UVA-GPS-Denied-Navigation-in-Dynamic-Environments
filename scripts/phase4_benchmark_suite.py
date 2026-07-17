#!/usr/bin/env python3
"""Run repeatable Phase 4 software benchmarks and save raw results."""

from __future__ import annotations

import json
import os
import statistics
import subprocess
import sys
import time
import tracemalloc
import urllib.error
import urllib.request
from dataclasses import asdict, dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
OUTPUT_PATH = REPO_ROOT / "docs" / "phase4" / "benchmark_results.json"
BUILD_ROOT = REPO_ROOT / "build" / "validation-msvc"
TEST_ROOT = BUILD_ROOT / "tests" / "Release"
BENCHMARK_BACKEND_PORT = 18080


@dataclass
class Metric:
    name: str
    kind: str
    iterations: int
    total_ms: float
    mean_ms: float
    median_ms: float
    p95_ms: float
    extra: dict[str, Any]


def percentile(values: list[float], ratio: float) -> float:
    if not values:
        return 0.0
    if len(values) == 1:
        return values[0]
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(round((len(ordered) - 1) * ratio))))
    return ordered[index]


def summarize(name: str, kind: str, samples_ms: list[float], extra: dict[str, Any]) -> Metric:
    return Metric(
        name=name,
        kind=kind,
        iterations=len(samples_ms),
        total_ms=sum(samples_ms),
        mean_ms=statistics.fmean(samples_ms) if samples_ms else 0.0,
        median_ms=statistics.median(samples_ms) if samples_ms else 0.0,
        p95_ms=percentile(samples_ms, 0.95),
        extra=extra,
    )


def request_json(method: str, url: str, payload: dict[str, Any] | None = None) -> tuple[int, dict[str, Any]]:
    data = None
    headers = {"Accept": "application/json"}
    if payload is not None:
        data = json.dumps(payload).encode("utf-8")
        headers["Content-Type"] = "application/json"
    request = urllib.request.Request(url, data=data, headers=headers, method=method)
    try:
        with urllib.request.urlopen(request, timeout=5) as response:
            body = response.read().decode("utf-8")
            return response.status, json.loads(body) if body else {}
    except urllib.error.HTTPError as exc:
        body = exc.read().decode("utf-8", errors="replace")
        try:
            parsed = json.loads(body) if body else {}
        except json.JSONDecodeError:
            parsed = {"raw": body}
        return exc.code, parsed


def benchmark_native_test(name: str, executable: Path, args: list[str], repeat: int) -> Metric:
    if not executable.exists():
        raise FileNotFoundError(f"missing benchmark executable: {executable}")
    samples_ms: list[float] = []
    command = [str(executable), *args]
    for _ in range(repeat):
        started = time.perf_counter()
        completed = subprocess.run(
            command,
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            check=False,
        )
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        if completed.returncode != 0:
            raise RuntimeError(
                f"{name} failed with exit code {completed.returncode}\n{completed.stdout}\n{completed.stderr}"
            )
        samples_ms.append(elapsed_ms)
    return summarize(
        name,
        "native",
        samples_ms,
        {"command": command, "repeat": repeat},
    )


def benchmark_config_load(path: Path, repeat: int) -> Metric:
    raw_text = path.read_text(encoding="utf-8")
    samples_ms: list[float] = []
    peak_kib = 0.0
    for _ in range(repeat):
        tracemalloc.start()
        started = time.perf_counter()
        parsed = json.loads(raw_text)
        samples_ms.append((time.perf_counter() - started) * 1000.0)
        current, peak = tracemalloc.get_traced_memory()
        peak_kib = max(peak_kib, peak / 1024.0)
        tracemalloc.stop()
        if not isinstance(parsed, dict):
            raise RuntimeError(f"{path} did not parse to an object")
    return summarize(
        f"config_load::{path.relative_to(REPO_ROOT)}",
        "python-json",
        samples_ms,
        {"peak_kib": peak_kib, "bytes": len(raw_text.encode('utf-8'))},
    )


def production_payload(drone_id: int) -> dict[str, Any]:
    return {
        "drone_id": drone_id,
        "cluster_id": "cluster-phase4",
        "role": "LEADER",
        "source": "real",
        "connectivity": "Mesh",
        "reachable": True,
        "position": [1.0, 2.0, 3.0],
        "velocity": [0.1, 0.0, 0.0],
        "attitude_rpy": [0.0, 0.0, 0.0],
        "thrust_vector": [0.0, 0.0, 9.81],
        "mission_state": "phase4-benchmark",
        "drift_m": 0.05,
        "battery_pct": 87.0,
        "rssi_dbm": -52.0,
        "cpu_temp_c": 49.0,
        "gpu_load_pct": 24.0,
        "localization_source": "vision-inertial",
        "localization_data_source": "real",
        "localization_state": "nominal",
        "localization_confidence": 0.91,
        "tdoa_confidence": 0.72,
        "confidence_trend": 0.01,
        "visible_anchor_count": 4,
        "occupancy_ratio": 0.12,
        "sync_confidence": 0.97,
        "imu_camera_offset_ms": 1.2,
        "peer_count": 2,
        "stale_peer_count": 0,
        "mesh_topology_mode": "adaptive_mesh",
        "local_consensus_state": "leader_sync",
        "local_consensus_epoch": 5,
        "peer_latency_ms": 7.0,
        "mesh_bandwidth_kbps": 130.0,
        "edge_serialization_mode": "cbor",
        "edge_average_packet_size_bytes": 128.0,
        "edge_bandwidth_savings_estimate_pct": 32.0,
        "edge_packet_encode_latency_us": 14.0,
        "auth_mode": "hmac_sha256",
        "auth_failures": 0,
        "unsigned_packets": 0,
        "last_auth_result": "accepted",
        "pqc_ready_status": "planned",
        "disconnected_operation": False,
        "edge_health_status": "nominal",
        "edge_autonomy_state": "distributed_hold",
        "edge_inference_status": "active",
        "edge_inference_fps": 14.0,
        "edge_inference_confidence": 0.81,
        "local_obstacle_count": 2,
        "shared_obstacle_count": 3,
        "security_state": "TRUSTED",
        "security_summary": "Phase 4 benchmark payload",
        "remote_command_allowed": True,
        "telemetry_uplink_allowed": True,
        "link_integrity_score": 0.95,
        "firmware_measurement": "phase4-benchmark-build",
        "firmware_version": "2.0.0",
        "secure_boot_state": "LAB_BOOT",
        "boot_trust_summary": "Phase 4 benchmark payload",
        "rollback_counter": 1,
        "maintenance_mode": False,
        "update_channel_state": "idle",
        "last_remote_command_status": "not-issued",
        "health_flags": ["phase4_benchmark"],
        "camera": {
            "status": "live",
            "fps": 30.0,
            "frame_age_ms": 18.0,
            "resolution": "1280x720",
            "dropped_frames": 0,
            "source": "real",
            "latest_frame_ref": "phase4-benchmark-frame",
        },
        "imu": {
            "status": "live",
            "sample_rate_hz": 200.0,
            "last_sample_age_ms": 5.0,
            "accel": {"x": 0.0, "y": 0.0, "z": 9.81},
            "gyro": {"x": 0.01, "y": 0.0, "z": -0.01},
            "health": "good",
            "source": "real",
        },
        "lidar": {
            "status": "live",
            "packet_rate_hz": 10.0,
            "scan_age_ms": 12.0,
            "point_count": 3,
            "points_2d": [
                {"x": 0.4, "y": 0.2, "intensity": 0.6},
                {"x": 0.8, "y": -0.3, "intensity": 0.7},
                {"x": 1.2, "y": 0.1, "intensity": 0.8},
            ],
            "min_range_m": 0.3,
            "max_range_m": 20.0,
            "source": "real",
        },
        "tdoa": {
            "status": "live",
            "source": "real",
            "visible_anchor_count": 4,
            "anchors": [
                {"id": "A0", "x": 0.0, "y": 0.0, "z": 2.5, "visible": True, "last_seen_ms": 6.0},
                {"id": "A1", "x": 8.0, "y": 0.0, "z": 2.5, "visible": True, "last_seen_ms": 7.0},
                {"id": "A2", "x": 0.0, "y": 8.0, "z": 2.5, "visible": True, "last_seen_ms": 8.0},
                {"id": "A3", "x": 8.0, "y": 8.0, "z": 2.5, "visible": True, "last_seen_ms": 9.0},
            ],
            "estimated_position": {"x": 1.0, "y": 2.0, "z": 3.0},
        },
        "replay": {"status": "unavailable", "active": False, "source": "unavailable"},
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }


def start_backend(port: int) -> tuple[subprocess.Popen[str], float]:
    env = dict(os.environ)
    env["DRONE_BACKEND_MODE"] = "production"
    env["DRONE_BACKEND_SIMULATION_ENABLED"] = "false"
    env["DRONE_SWARM_ADDR"] = f":{port}"
    log_path = REPO_ROOT / "docs" / "phase4" / "backend_benchmark.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", encoding="utf-8") as handle:
        handle.write("")
    handle = log_path.open("a", encoding="utf-8")
    started = time.perf_counter()
    process = subprocess.Popen(
        ["go", "run", "./cmd/control-plane"],
        cwd=REPO_ROOT,
        env=env,
        stdout=handle,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    health_url = f"http://127.0.0.1:{port}/api/v1/health"
    deadline_s = 60.0
    while (time.perf_counter() - started) < deadline_s:
        if process.poll() is not None:
            raise RuntimeError(f"backend exited early with code {process.returncode}")
        try:
            status, _ = request_json("GET", health_url)
            if status == 200:
                return process, (time.perf_counter() - started) * 1000.0
        except Exception:  # noqa: BLE001
            pass
        time.sleep(0.1)
    process.terminate()
    raise TimeoutError(f"backend did not become healthy within {deadline_s:.0f} seconds")


def stop_backend(process: subprocess.Popen[str]) -> None:
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)


def run_smoke_in_isolated_backend(script_path: str, port: int) -> dict[str, Any]:
    backend, _startup_ms = start_backend(port)
    try:
        return run_smoke_script(script_path, f"http://127.0.0.1:{port}")
    finally:
        stop_backend(backend)


def benchmark_backend_reads(base_url: str, iterations: int) -> Metric:
    samples_ms: list[float] = []
    for _ in range(iterations):
        started = time.perf_counter()
        status, payload = request_json("GET", f"{base_url}/api/v1/fleet")
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        if status != 200:
            raise RuntimeError(f"fleet GET failed: status={status} payload={payload}")
        samples_ms.append(elapsed_ms)
    return summarize("backend_fleet_get", "http", samples_ms, {"url": f"{base_url}/api/v1/fleet"})


def benchmark_telemetry_posts(base_url: str, iterations: int) -> Metric:
    samples_ms: list[float] = []
    accepted = 0
    for index in range(iterations):
        payload = production_payload(9000 + index)
        started = time.perf_counter()
        status, body = request_json("POST", f"{base_url}/api/v1/telemetry", payload)
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        if status != 202:
            raise RuntimeError(f"telemetry POST failed: status={status} payload={body}")
        samples_ms.append(elapsed_ms)
        accepted += 1
    throughput_hz = accepted / (sum(samples_ms) / 1000.0) if samples_ms else 0.0
    return summarize(
        "backend_telemetry_post",
        "http",
        samples_ms,
        {
            "url": f"{base_url}/api/v1/telemetry",
            "accepted": accepted,
            "throughput_hz": throughput_hz,
        },
    )


def run_smoke_script(script_path: str, base_url: str) -> dict[str, Any]:
    completed = subprocess.run(
        [sys.executable, script_path, "--backend-url", base_url],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    return {
        "script": script_path,
        "returncode": completed.returncode,
        "stdout": completed.stdout.strip(),
        "stderr": completed.stderr.strip(),
    }


def main() -> int:
    metrics: list[Metric] = []
    smoke_results: list[dict[str, Any]] = []

    metrics.append(benchmark_config_load(REPO_ROOT / "config" / "runtime.json", 20))
    metrics.append(benchmark_config_load(REPO_ROOT / "config" / "anchors.json", 50))
    metrics.append(benchmark_config_load(REPO_ROOT / "config" / "lidar.json", 50))

    metrics.append(
        benchmark_native_test(
            "ekf_update_path",
            TEST_ROOT / "test_ekf.exe",
            ["--gtest_filter=EKFTest.FreeFallVelocity"],
            50,
        )
    )
    metrics.append(
        benchmark_native_test(
            "lidar_processing_path",
            TEST_ROOT / "test_sensors.exe",
            ["--gtest_filter=LidarSensor.ObstacleListGeneratedFromValidScan"],
            50,
        )
    )
    metrics.append(
        benchmark_native_test(
            "sensor_fusion_path",
            TEST_ROOT / "test_navigation_intelligence.exe",
            ["--gtest_filter=LocalizationFusion.PrefersTdoaWhenVioDriftIsHigh"],
            30,
        )
    )
    metrics.append(
        benchmark_native_test(
            "telemetry_serialization_path",
            TEST_ROOT / "test_telemetry.exe",
            ["--gtest_filter=ControlPlaneTelemetryClient.SerializesTelemetryPayload"],
            60,
        )
    )
    metrics.append(
        benchmark_native_test(
            "security_packet_auth_path",
            TEST_ROOT / "test_edge_swarm.exe",
            ["--gtest_filter=PeerPacketAuth.HmacSignedPacketVerifies"],
            60,
        )
    )

    benchmark_base_url = f"http://127.0.0.1:{BENCHMARK_BACKEND_PORT}"
    backend, startup_ms = start_backend(BENCHMARK_BACKEND_PORT)
    try:
        metrics.append(
            summarize(
                "backend_startup",
                "process",
                [startup_ms],
                {"mode": "production", "simulation_enabled": False},
            )
        )
        metrics.append(benchmark_backend_reads(benchmark_base_url, 25))
        metrics.append(benchmark_telemetry_posts(benchmark_base_url, 25))
    finally:
        stop_backend(backend)

    smoke_results.append(
        run_smoke_in_isolated_backend("scripts/telemetry_smoke_test.py", 18081)
    )
    smoke_results.append(
        run_smoke_in_isolated_backend("scripts/production_telemetry_smoke_test.py", 18082)
    )

    report = {
        "generated_at": datetime.now(UTC).isoformat().replace("+00:00", "Z"),
        "metrics": [asdict(metric) for metric in metrics],
        "smoke_results": smoke_results,
        "notes": [
            "All benchmark numbers are wall-clock measurements collected on the local Windows workspace.",
            "Memory and allocator micro-metrics were limited to Python JSON-load tracemalloc measurements in this environment.",
        ],
    }
    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_PATH.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"wrote {OUTPUT_PATH}")
    for metric in metrics:
        print(
            f"{metric.name}: mean_ms={metric.mean_ms:.3f} median_ms={metric.median_ms:.3f} p95_ms={metric.p95_ms:.3f}"
        )
    failed_smokes = [item for item in smoke_results if item["returncode"] != 0]
    return 1 if failed_smokes else 0


if __name__ == "__main__":
    raise SystemExit(main())
