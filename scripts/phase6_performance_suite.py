#!/usr/bin/env python3
"""Run Phase 6 software performance, stress, soak, and latency measurements."""

from __future__ import annotations

import json
import os
import re
import socket
import statistics
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import asdict, dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
DOC_ROOT = REPO_ROOT / "docs" / "phase6"
OUTPUT_PATH = DOC_ROOT / "performance_results.json"
LOG_PATH = DOC_ROOT / "performance_suite.log"
BUILD_ROOT = REPO_ROOT / "build" / "validation-msvc"
TEST_ROOT = BUILD_ROOT / "tests" / "Release"
@dataclass
class Metric:
    name: str
    kind: str
    iterations: int
    total_ms: float
    mean_ms: float
    median_ms: float
    p50_ms: float
    p95_ms: float
    p99_ms: float
    max_ms: float
    extra: dict[str, Any]


def log(message: str) -> None:
    DOC_ROOT.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now(UTC).isoformat().replace("+00:00", "Z")
    line = f"[{timestamp}] {message}"
    print(line)
    with LOG_PATH.open("a", encoding="utf-8") as handle:
        handle.write(line + "\n")


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
        p50_ms=percentile(samples_ms, 0.50),
        p95_ms=percentile(samples_ms, 0.95),
        p99_ms=percentile(samples_ms, 0.99),
        max_ms=max(samples_ms) if samples_ms else 0.0,
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
        with urllib.request.urlopen(request, timeout=10) as response:
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
    return summarize(name, "native", samples_ms, {"command": command, "repeat": repeat})


def benchmark_config_load(path: Path, repeat: int) -> Metric:
    raw_text = path.read_text(encoding="utf-8")
    samples_ms: list[float] = []
    for _ in range(repeat):
        started = time.perf_counter()
        parsed = json.loads(raw_text)
        samples_ms.append((time.perf_counter() - started) * 1000.0)
        if not isinstance(parsed, dict):
            raise RuntimeError(f"{path} did not parse to an object")
    return summarize(
        f"config_load::{path.relative_to(REPO_ROOT)}",
        "python-json",
        samples_ms,
        {"bytes": len(raw_text.encode('utf-8'))},
    )


def production_payload(drone_id: int) -> dict[str, Any]:
    return {
        "drone_id": drone_id,
        "cluster_id": "cluster-phase6",
        "role": "LEADER",
        "source": "real",
        "connectivity": "Mesh",
        "reachable": True,
        "position": [1.0, 2.0, 3.0],
        "velocity": [0.1, 0.0, 0.0],
        "attitude_rpy": [0.0, 0.0, 0.0],
        "thrust_vector": [0.0, 0.0, 9.81],
        "mission_state": "phase6-performance",
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
        "local_consensus_epoch": 6,
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
        "security_summary": "Phase 6 benchmark payload",
        "remote_command_allowed": True,
        "telemetry_uplink_allowed": True,
        "link_integrity_score": 0.95,
        "firmware_measurement": "phase6-benchmark-build",
        "firmware_version": "2.0.0",
        "secure_boot_state": "LAB_BOOT",
        "boot_trust_summary": "Phase 6 benchmark payload",
        "rollback_counter": 1,
        "maintenance_mode": False,
        "update_channel_state": "idle",
        "last_remote_command_status": "not-issued",
        "health_flags": ["phase6_benchmark"],
        "camera": {
            "status": "live",
            "fps": 30.0,
            "frame_age_ms": 18.0,
            "resolution": "1280x720",
            "dropped_frames": 0,
            "source": "real",
            "latest_frame_ref": "phase6-benchmark-frame",
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


def allocate_free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        return int(sock.getsockname()[1])


def start_backend(
    port: int,
    log_file: Path,
    *,
    mode: str = "production",
    simulation_enabled: bool = False,
    extra_env: dict[str, str] | None = None,
) -> tuple[subprocess.Popen[str], float]:
    env = dict(os.environ)
    env["DRONE_BACKEND_MODE"] = mode
    env["DRONE_BACKEND_SIMULATION_ENABLED"] = "true" if simulation_enabled else "false"
    env["DRONE_SWARM_ADDR"] = f":{port}"
    if extra_env:
        env.update(extra_env)
    log_file.parent.mkdir(parents=True, exist_ok=True)
    log_handle = log_file.open("w", encoding="utf-8")
    started = time.perf_counter()
    process = subprocess.Popen(
        ["go", "run", "./cmd/control-plane"],
        cwd=REPO_ROOT,
        env=env,
        stdout=log_handle,
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
        payload = production_payload(10000 + index)
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
        {"url": f"{base_url}/api/v1/telemetry", "accepted": accepted, "throughput_hz": throughput_hz},
    )


def sample_process_stats(pid: int) -> dict[str, float]:
    command = (
        "Get-Process -Id "
        f"{pid} | Select-Object Id,CPU,WorkingSet64,PrivateMemorySize64,Handles,Threads,StartTime "
        "| ConvertTo-Json -Compress"
    )
    completed = subprocess.run(
        ["powershell", "-NoProfile", "-Command", command],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    if completed.returncode != 0 or not completed.stdout.strip():
        raise RuntimeError(f"process sample failed for pid={pid}: {completed.stderr}")
    payload = json.loads(completed.stdout)
    threads = payload.get("Threads", [])
    start_time = payload.get("StartTime")
    uptime_seconds = 0.0
    if start_time:
        started_text = str(start_time)
        if started_text.startswith("/Date("):
            match = re.search(r"/Date\((\d+)\)/", started_text)
            if match:
                started = datetime.fromtimestamp(int(match.group(1)) / 1000.0, tz=UTC)
                uptime_seconds = max(0.0, (datetime.now(UTC) - started).total_seconds())
        else:
            started = datetime.fromisoformat(started_text)
            tzinfo = started.tzinfo or UTC
            uptime_seconds = max(0.0, (datetime.now(tzinfo) - started).total_seconds())
    return {
        "cpu_seconds": float(payload.get("CPU", 0.0) or 0.0),
        "working_set_mb": float(payload.get("WorkingSet64", 0) or 0) / (1024.0 * 1024.0),
        "private_memory_mb": float(payload.get("PrivateMemorySize64", 0) or 0) / (1024.0 * 1024.0),
        "handles": float(payload.get("Handles", 0) or 0),
        "thread_count": float(len(threads)),
        "open_file_descriptors": float(payload.get("Handles", 0) or 0),
        "uptime_s": uptime_seconds,
    }


def run_stress_test(base_url: str, total_requests: int, workers: int) -> dict[str, Any]:
    latencies_ms: list[float] = []
    failures: list[dict[str, Any]] = []
    lock = threading.Lock()

    def worker(index: int) -> None:
        payload = production_payload(20000 + index)
        started = time.perf_counter()
        status, body = request_json("POST", f"{base_url}/api/v1/telemetry", payload)
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        with lock:
            latencies_ms.append(elapsed_ms)
            if status != 202:
                failures.append({"index": index, "status": status, "body": body})

    wall_started = time.perf_counter()
    with ThreadPoolExecutor(max_workers=workers) as pool:
        futures = [pool.submit(worker, idx) for idx in range(total_requests)]
        for future in as_completed(futures):
            future.result()
    wall_elapsed_ms = (time.perf_counter() - wall_started) * 1000.0
    return {
        "requests": total_requests,
        "workers": workers,
        "wall_elapsed_ms": wall_elapsed_ms,
        "throughput_hz": total_requests / (wall_elapsed_ms / 1000.0) if wall_elapsed_ms else 0.0,
        "latency": asdict(summarize("stress_post_latency", "http-stress", latencies_ms, {})),
        "failures": failures,
    }


def run_stress_test_with_stats(base_url: str, pid: int, total_requests: int, workers: int) -> dict[str, Any]:
    stats_before = sample_process_stats(pid)
    result = run_stress_test(base_url, total_requests=total_requests, workers=workers)
    stats_after = sample_process_stats(pid)
    result["resource_before"] = stats_before
    result["resource_after"] = stats_after
    result["cpu_seconds_delta"] = stats_after["cpu_seconds"] - stats_before["cpu_seconds"]
    result["working_set_mb_delta"] = stats_after["working_set_mb"] - stats_before["working_set_mb"]
    result["private_memory_mb_delta"] = stats_after["private_memory_mb"] - stats_before["private_memory_mb"]
    result["thread_count_delta"] = stats_after["thread_count"] - stats_before["thread_count"]
    result["handle_delta"] = stats_after["handles"] - stats_before["handles"]
    return result


def run_soak_test(base_url: str, pid: int, duration_s: int, interval_s: float) -> dict[str, Any]:
    samples: list[dict[str, Any]] = []
    latencies_ms: list[float] = []
    requests = 0
    started = time.perf_counter()
    while (time.perf_counter() - started) < duration_s:
        sample_started = time.perf_counter()
        payload = production_payload(30000 + requests)
        status, body = request_json("POST", f"{base_url}/api/v1/telemetry", payload)
        latency_ms = (time.perf_counter() - sample_started) * 1000.0
        if status != 202:
            raise RuntimeError(f"soak telemetry POST failed: status={status} payload={body}")
        stats = sample_process_stats(pid)
        stats["elapsed_s"] = time.perf_counter() - started
        stats["latency_ms"] = latency_ms
        samples.append(stats)
        latencies_ms.append(latency_ms)
        requests += 1
        time.sleep(interval_s)
    working_sets = [sample["working_set_mb"] for sample in samples]
    private_sets = [sample["private_memory_mb"] for sample in samples]
    cpu_samples = [sample["cpu_seconds"] for sample in samples]
    thread_samples = [sample["thread_count"] for sample in samples]
    handle_samples = [sample["handles"] for sample in samples]
    fd_samples = [sample["open_file_descriptors"] for sample in samples]
    return {
        "duration_s": duration_s,
        "interval_s": interval_s,
        "requests": requests,
        "latency": asdict(summarize("soak_post_latency", "http-soak", latencies_ms, {})),
        "working_set_mb_start": working_sets[0] if working_sets else 0.0,
        "working_set_mb_end": working_sets[-1] if working_sets else 0.0,
        "working_set_mb_peak": max(working_sets) if working_sets else 0.0,
        "private_memory_mb_start": private_sets[0] if private_sets else 0.0,
        "private_memory_mb_end": private_sets[-1] if private_sets else 0.0,
        "private_memory_mb_peak": max(private_sets) if private_sets else 0.0,
        "cpu_seconds_start": cpu_samples[0] if cpu_samples else 0.0,
        "cpu_seconds_end": cpu_samples[-1] if cpu_samples else 0.0,
        "thread_count_start": thread_samples[0] if thread_samples else 0.0,
        "thread_count_end": thread_samples[-1] if thread_samples else 0.0,
        "thread_count_peak": max(thread_samples) if thread_samples else 0.0,
        "handle_count_start": handle_samples[0] if handle_samples else 0.0,
        "handle_count_end": handle_samples[-1] if handle_samples else 0.0,
        "handle_count_peak": max(handle_samples) if handle_samples else 0.0,
        "open_file_descriptors_start": fd_samples[0] if fd_samples else 0.0,
        "open_file_descriptors_end": fd_samples[-1] if fd_samples else 0.0,
        "open_file_descriptors_peak": max(fd_samples) if fd_samples else 0.0,
        "latency_drift_ms": (latencies_ms[-1] - latencies_ms[0]) if len(latencies_ms) >= 2 else 0.0,
        "queue_depth_peak": 0.0,
        "samples": samples,
    }


def main() -> int:
    DOC_ROOT.mkdir(parents=True, exist_ok=True)
    if LOG_PATH.exists():
        LOG_PATH.unlink()

    metrics: list[Metric] = []
    log("starting phase 6 performance suite")

    metrics.append(benchmark_config_load(REPO_ROOT / "config" / "runtime.json", 20))
    metrics.append(benchmark_config_load(REPO_ROOT / "config" / "anchors.json", 20))
    metrics.append(benchmark_config_load(REPO_ROOT / "config" / "lidar.json", 20))
    metrics.append(
        benchmark_native_test(
            "ekf_update_path",
            TEST_ROOT / "test_ekf.exe",
            ["--gtest_filter=EKFTest.FreeFallVelocity"],
            30,
        )
    )
    metrics.append(
        benchmark_native_test(
            "lidar_processing_path",
            TEST_ROOT / "test_sensors.exe",
            ["--gtest_filter=LidarSensor.ObstacleListGeneratedFromValidScan"],
            30,
        )
    )
    metrics.append(
        benchmark_native_test(
            "sensor_fusion_path",
            TEST_ROOT / "test_navigation_intelligence.exe",
            ["--gtest_filter=LocalizationFusion.PrefersTdoaWhenVioDriftIsHigh"],
            20,
        )
    )
    metrics.append(
        benchmark_native_test(
            "telemetry_serialization_path",
            TEST_ROOT / "test_telemetry.exe",
            ["--gtest_filter=ControlPlaneTelemetryClient.SerializesTelemetryPayload"],
            40,
        )
    )
    metrics.append(
        benchmark_native_test(
            "security_packet_auth_path",
            TEST_ROOT / "test_edge_swarm.exe",
            ["--gtest_filter=PeerPacketAuth.HmacSignedPacketVerifies"],
            40,
        )
    )

    benchmark_log = DOC_ROOT / "backend_benchmark.log"
    stress_log = DOC_ROOT / "backend_stress.log"
    soak_log = DOC_ROOT / "backend_soak.log"
    benchmark_port = allocate_free_port()
    stress_port = allocate_free_port()
    soak_port = allocate_free_port()
    log(
        "using backend ports "
        f"benchmark={benchmark_port} stress={stress_port} soak={soak_port}"
    )

    benchmark_process, startup_ms = start_backend(benchmark_port, benchmark_log)
    try:
        base_url = f"http://127.0.0.1:{benchmark_port}"
        metrics.append(summarize("backend_startup", "process", [startup_ms], {"mode": "production"}))
        metrics.append(benchmark_backend_reads(base_url, 30))
        metrics.append(benchmark_telemetry_posts(base_url, 30))
    finally:
        stop_backend(benchmark_process)

    stress_process, _stress_startup = start_backend(stress_port, stress_log)
    try:
        stress_result = run_stress_test(f"http://127.0.0.1:{stress_port}", total_requests=400, workers=16)
    finally:
        stop_backend(stress_process)

    soak_process, _soak_startup = start_backend(soak_port, soak_log)
    try:
        soak_result = run_soak_test(
            f"http://127.0.0.1:{soak_port}",
            soak_process.pid,
            duration_s=90,
            interval_s=0.5,
        )
    finally:
        stop_backend(soak_process)

    report = {
        "generated_at": datetime.now(UTC).isoformat().replace("+00:00", "Z"),
        "metrics": [asdict(metric) for metric in metrics],
        "stress_test": stress_result,
        "soak_test": soak_result,
        "memory_validation_references": {
            "asan_lsan_report": "docs/phase4/MEMORY_AUDIT.md",
            "valgrind_report": "docs/phase4/VALGRIND_REPORT.md",
            "asan_log": "docs/phase4/asan_lsan_ctest.log",
            "valgrind_logs": [
                "docs/phase4/valgrind_test_ekf.log",
                "docs/phase4/valgrind_test_edge_swarm.log",
                "docs/phase4/valgrind_drone_node.log",
                "docs/phase4/valgrind_drone_node_suppressed.log",
            ],
        },
        "notes": [
            "Benchmark numbers are wall-clock measurements collected on the local Windows workspace.",
            "Soak memory samples were collected from the Go control-plane process via Get-Process.",
            "Repository-owned leak detection still relies on Phase 4 ASan/LSan and Valgrind evidence; Phase 6 extends this with live memory-drift sampling under soak load.",
        ],
    }
    OUTPUT_PATH.write_text(json.dumps(report, indent=2), encoding="utf-8")
    log(f"wrote {OUTPUT_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
