#!/usr/bin/env python3
"""Run reproducible Phase 7 mission scenarios against the control-plane backend."""

from __future__ import annotations

import hashlib
import json
import platform
import statistics
import subprocess
import sys
import time
from dataclasses import asdict
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import phase6_performance_suite as suite


REPO_ROOT = suite.REPO_ROOT
DOC_ROOT = REPO_ROOT / "docs" / "phase7"
RESULTS_ROOT = REPO_ROOT / "results" / "phase7"
EXPERIMENT_ROOT = REPO_ROOT / "experiments"
OUTPUT_PATH = DOC_ROOT / "mission_results.json"
RUN_LOG_PATH = RESULTS_ROOT / "phase7_mission_scenarios.log"


def utc_now() -> str:
    return datetime.now(UTC).isoformat().replace("+00:00", "Z")


def log(message: str) -> None:
    RESULTS_ROOT.mkdir(parents=True, exist_ok=True)
    line = f"[{utc_now()}] {message}"
    print(line)
    with RUN_LOG_PATH.open("a", encoding="utf-8") as handle:
        handle.write(line + "\n")


def hash_file(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {"path": str(path.relative_to(REPO_ROOT)), "exists": False}
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    return {
        "path": str(path.relative_to(REPO_ROOT)),
        "exists": True,
        "sha256": digest,
        "bytes": path.stat().st_size,
        "modified_at": datetime.fromtimestamp(path.stat().st_mtime, tz=UTC).isoformat().replace("+00:00", "Z"),
    }


def percentile(values: list[float], ratio: float) -> float:
    return suite.percentile(values, ratio)


def sample_metrics_process(pid: int) -> dict[str, float]:
    try:
        return suite.sample_process_stats(pid)
    except Exception:  # noqa: BLE001
        return {}


def git_revision() -> str:
    completed = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    if completed.returncode != 0:
        return "unknown"
    return completed.stdout.strip() or "unknown"


def request_metrics(base_url: str) -> dict[str, Any]:
    started = time.perf_counter()
    request = suite.urllib.request.Request(f"{base_url}/metrics", headers={"Accept": "text/plain"}, method="GET")
    with suite.urllib.request.urlopen(request, timeout=10) as response:
        body = response.read().decode("utf-8", errors="replace")
        status = response.status
    elapsed_ms = (time.perf_counter() - started) * 1000.0
    return {"status": status, "latency_ms": elapsed_ms, "body_size_bytes": len(body.encode("utf-8"))}


def scenario_payload(
    drone_id: int,
    *,
    cluster_id: str,
    role: str,
    mission_state: str,
    source: str = "simulation",
    localization_source: str = "vision-inertial",
    localization_data_source: str = "simulation",
    localization_state: str = "nominal",
    localization_confidence: float = 0.9,
    tdoa_confidence: float = 0.7,
    visible_anchor_count: int = 4,
    peer_count: int = 2,
    stale_peer_count: int = 0,
    local_obstacle_count: int = 1,
    shared_obstacle_count: int = 1,
    safety_state: str = "NORMAL",
    safety_summary: str = "Nominal software-only mission scenario",
    security_state: str = "TRUSTED",
    remote_command_allowed: bool = True,
    telemetry_uplink_allowed: bool = True,
    health_flags: list[str] | None = None,
) -> dict[str, Any]:
    payload = suite.production_payload(drone_id)
    payload.update(
        {
            "cluster_id": cluster_id,
            "role": role,
            "source": source,
            "mission_state": mission_state,
            "localization_source": localization_source,
            "localization_data_source": localization_data_source,
            "localization_state": localization_state,
            "localization_confidence": localization_confidence,
            "tdoa_confidence": tdoa_confidence,
            "visible_anchor_count": visible_anchor_count,
            "peer_count": peer_count,
            "stale_peer_count": stale_peer_count,
            "local_obstacle_count": local_obstacle_count,
            "shared_obstacle_count": shared_obstacle_count,
            "safety_state": safety_state,
            "safety_summary": safety_summary,
            "security_state": security_state,
            "remote_command_allowed": remote_command_allowed,
            "telemetry_uplink_allowed": telemetry_uplink_allowed,
            "health_flags": health_flags or [],
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        }
    )
    return payload


def post_telemetry(base_url: str, payload: dict[str, Any]) -> dict[str, Any]:
    started = time.perf_counter()
    status, body = suite.request_json("POST", f"{base_url}/api/v1/telemetry", payload)
    elapsed_ms = (time.perf_counter() - started) * 1000.0
    return {"status": status, "body": body, "latency_ms": elapsed_ms}


def get_fleet(base_url: str) -> dict[str, Any]:
    started = time.perf_counter()
    status, body = suite.request_json("GET", f"{base_url}/api/v1/fleet")
    elapsed_ms = (time.perf_counter() - started) * 1000.0
    if status != 200:
        raise RuntimeError(f"fleet GET failed: status={status} body={body}")
    body["_latency_ms"] = elapsed_ms
    return body


def get_health(base_url: str) -> dict[str, Any]:
    started = time.perf_counter()
    status, body = suite.request_json("GET", f"{base_url}/api/v1/health")
    elapsed_ms = (time.perf_counter() - started) * 1000.0
    if status != 200:
        raise RuntimeError(f"health GET failed: status={status} body={body}")
    body["_latency_ms"] = elapsed_ms
    return body


def get_readiness(base_url: str) -> dict[str, Any]:
    started = time.perf_counter()
    status, body = suite.request_json("GET", f"{base_url}/api/v1/ready")
    elapsed_ms = (time.perf_counter() - started) * 1000.0
    if status != 200:
        raise RuntimeError(f"ready GET failed: status={status} body={body}")
    body["_latency_ms"] = elapsed_ms
    return body


def expect(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def find_drone(fleet: dict[str, Any], drone_id: int) -> dict[str, Any]:
    drones = fleet.get("drones", [])
    target = next((drone for drone in drones if drone.get("drone_id") == drone_id), None)
    if target is None:
        raise AssertionError(f"drone {drone_id} not found in fleet snapshot")
    return target


def formation_scenario(base_url: str, experiment_id: str) -> dict[str, Any]:
    cluster_id = "phase7-formation"
    drone_ids = [7101, 7102, 7103]
    post_samples: list[float] = []
    timeline: list[dict[str, Any]] = []
    for offset, drone_id in enumerate(drone_ids):
        role = "LEADER" if offset == 0 else "FOLLOWER"
        payload = scenario_payload(
            drone_id,
            cluster_id=cluster_id,
            role=role,
            mission_state="formation_maintenance",
            peer_count=2,
            stale_peer_count=0,
            source="simulation",
            localization_data_source="simulation",
            health_flags=[f"experiment:{experiment_id}", "scenario:formation", "coordination:stable"],
        )
        result = post_telemetry(base_url, payload)
        expect(result["status"] == 202, f"formation telemetry rejected for drone {drone_id}")
        post_samples.append(float(result["latency_ms"]))
        timeline.append({"event": "telemetry_posted", "drone_id": drone_id, "latency_ms": result["latency_ms"]})
    fleet = get_fleet(base_url)
    cluster = next((item for item in fleet.get("clusters", []) if item.get("cluster_id") == cluster_id), None)
    expect(cluster is not None, "formation cluster was not created")
    expect(cluster.get("drone_count") == 3, f"expected 3 drones in formation cluster, got {cluster}")
    expect(cluster.get("leader_id") == drone_ids[0], f"unexpected leader in formation cluster: {cluster}")
    expect(fleet.get("leader_id") == drone_ids[0], f"unexpected fleet leader: {fleet.get('leader_id')}")
    health = get_health(base_url)
    return {
        "name": "formation_maintenance",
        "pass": True,
        "latency_ms": summarize_latency(post_samples + [fleet["_latency_ms"], health["_latency_ms"]]),
        "timeline": timeline,
        "observations": {
            "cluster_id": cluster_id,
            "cluster_state": cluster,
            "fleet_leader_id": fleet.get("leader_id"),
            "online_drones": health.get("online_drones"),
            "critical_alerts": health.get("critical_alerts"),
        },
    }


def gps_denied_scenario(base_url: str, experiment_id: str) -> dict[str, Any]:
    drone_id = 7201
    payload = scenario_payload(
        drone_id,
        cluster_id="phase7-gps-denied",
        role="LEADER",
        mission_state="gps_denied_navigation",
        source="real",
        localization_source="vio-tdoa-fused",
        localization_data_source="real",
        localization_state="nominal",
        localization_confidence=0.94,
        tdoa_confidence=0.82,
        visible_anchor_count=4,
        peer_count=1,
        local_obstacle_count=2,
        shared_obstacle_count=3,
        health_flags=[f"experiment:{experiment_id}", "scenario:gps_denied", "fusion:active"],
    )
    post_result = post_telemetry(base_url, payload)
    expect(post_result["status"] == 202, "gps-denied telemetry rejected")
    fleet = get_fleet(base_url)
    target = find_drone(fleet, drone_id)
    expect(target.get("source") == "real", f"expected source=real for gps-denied scenario, got {target.get('source')}")
    expect(target.get("localization_source") == "vio-tdoa-fused", "gps-denied localization source mismatch")
    expect(target.get("localization_state") == "nominal", "gps-denied localization state mismatch")
    expect(float(target.get("localization_confidence", 0.0)) >= 0.9, "gps-denied confidence below target")
    expect(int(target.get("visible_anchor_count", 0)) >= 4, "gps-denied anchor visibility not preserved")
    metrics = request_metrics(base_url)
    expect(metrics["status"] == 200, "metrics endpoint unavailable during gps-denied scenario")
    return {
        "name": "gps_denied_navigation",
        "pass": True,
        "latency_ms": summarize_latency([post_result["latency_ms"], fleet["_latency_ms"], metrics["latency_ms"]]),
        "timeline": [{"event": "telemetry_posted", "drone_id": drone_id, "latency_ms": post_result["latency_ms"]}],
        "observations": {
            "drone_id": drone_id,
            "localization_source": target.get("localization_source"),
            "localization_confidence": target.get("localization_confidence"),
            "visible_anchor_count": target.get("visible_anchor_count"),
            "metrics_latency_ms": metrics["latency_ms"],
        },
    }


def communication_loss_scenario(base_url: str, experiment_id: str) -> dict[str, Any]:
    drone_id = 7301
    cluster_id = "phase7-link-loss"
    first = post_telemetry(
        base_url,
        scenario_payload(
            drone_id,
            cluster_id=cluster_id,
            role="LEADER",
            mission_state="comm_loss_recovery",
            source="real",
            localization_data_source="real",
            peer_count=0,
            stale_peer_count=0,
            safety_state="NORMAL",
            health_flags=[f"experiment:{experiment_id}", "scenario:comm_loss", "link:healthy"],
        ),
    )
    expect(first["status"] == 202, "communication-loss seed telemetry rejected")
    time.sleep(3.2)
    stale_fleet = get_fleet(base_url)
    stale_target = find_drone(stale_fleet, drone_id)
    expect(bool(stale_target.get("stale")), f"expected stale drone after timeout, got {stale_target}")
    expect(int(stale_fleet.get("stale_drone_count", 0)) >= 1, "stale fleet count did not increase")
    recovered = post_telemetry(
        base_url,
        scenario_payload(
            drone_id,
            cluster_id=cluster_id,
            role="LEADER",
            mission_state="comm_loss_recovery",
            source="real",
            localization_data_source="real",
            peer_count=0,
            stale_peer_count=0,
            safety_state="LINK_LOST",
            safety_summary="Link timeout observed and recovery signal injected",
            health_flags=[f"experiment:{experiment_id}", "scenario:comm_loss", "link:recovered"],
        ),
    )
    expect(recovered["status"] == 202, "communication-loss recovery telemetry rejected")
    recovered_fleet = get_fleet(base_url)
    recovered_target = find_drone(recovered_fleet, drone_id)
    expect(not bool(recovered_target.get("stale")), f"expected stale flag to clear after recovery, got {recovered_target}")
    readiness = get_readiness(base_url)
    return {
        "name": "communication_loss_recovery",
        "pass": True,
        "latency_ms": summarize_latency(
            [first["latency_ms"], stale_fleet["_latency_ms"], recovered["latency_ms"], recovered_fleet["_latency_ms"], readiness["_latency_ms"]]
        ),
        "timeline": [
            {"event": "telemetry_posted", "stage": "initial", "latency_ms": first["latency_ms"]},
            {"event": "stale_detected", "latency_ms": stale_fleet["_latency_ms"]},
            {"event": "telemetry_posted", "stage": "recovery", "latency_ms": recovered["latency_ms"]},
        ],
        "observations": {
            "stale_detected": True,
            "stale_drone_count": stale_fleet.get("stale_drone_count"),
            "recovered_ready_status": readiness.get("status"),
            "recovered_ready_reason": readiness.get("reason"),
        },
    }


def obstacle_degradation_scenario(base_url: str, experiment_id: str) -> dict[str, Any]:
    drone_id = 7401
    payload = scenario_payload(
        drone_id,
        cluster_id="phase7-obstacle",
        role="LEADER",
        mission_state="obstacle_degradation",
        source="real",
        localization_source="vision-inertial",
        localization_data_source="real",
        localization_state="degraded",
        localization_confidence=0.41,
        tdoa_confidence=0.35,
        visible_anchor_count=2,
        peer_count=1,
        stale_peer_count=1,
        local_obstacle_count=8,
        shared_obstacle_count=11,
        safety_state="DEGRADED_LOCALIZATION",
        safety_summary="Obstacle density increased while localization confidence dropped",
        security_state="TRUSTED",
        remote_command_allowed=True,
        health_flags=[f"experiment:{experiment_id}", "scenario:obstacle", "localization:degraded"],
    )
    post_result = post_telemetry(base_url, payload)
    expect(post_result["status"] == 202, "obstacle degradation telemetry rejected")
    fleet = get_fleet(base_url)
    target = find_drone(fleet, drone_id)
    expect(target.get("safety_state") == "DEGRADED_LOCALIZATION", "degraded safety state not preserved")
    expect(float(target.get("localization_confidence", 1.0)) < 0.5, "expected degraded localization confidence")
    expect(int(target.get("local_obstacle_count", 0)) >= 8, "expected dense local obstacle count")
    health = get_health(base_url)
    return {
        "name": "obstacle_avoidance_degradation",
        "pass": True,
        "latency_ms": summarize_latency([post_result["latency_ms"], fleet["_latency_ms"], health["_latency_ms"]]),
        "timeline": [{"event": "telemetry_posted", "drone_id": drone_id, "latency_ms": post_result["latency_ms"]}],
        "observations": {
            "safety_state": target.get("safety_state"),
            "localization_confidence": target.get("localization_confidence"),
            "local_obstacle_count": target.get("local_obstacle_count"),
            "shared_obstacle_count": target.get("shared_obstacle_count"),
        },
    }


def emergency_landing_scenario(base_url: str, experiment_id: str) -> dict[str, Any]:
    drone_id = 7501
    payload = scenario_payload(
        drone_id,
        cluster_id="phase7-emergency",
        role="LEADER",
        mission_state="emergency_landing",
        source="real",
        localization_source="vision-inertial",
        localization_data_source="real",
        localization_state="lost",
        localization_confidence=0.18,
        tdoa_confidence=0.2,
        visible_anchor_count=1,
        peer_count=0,
        stale_peer_count=2,
        local_obstacle_count=4,
        shared_obstacle_count=5,
        safety_state="EMERGENCY_LAND",
        safety_summary="Emergency landing requested due to localization failure",
        security_state="LAND_IMMEDIATELY",
        remote_command_allowed=False,
        telemetry_uplink_allowed=True,
        health_flags=[f"experiment:{experiment_id}", "scenario:emergency", "failsafe:active"],
    )
    post_result = post_telemetry(base_url, payload)
    expect(post_result["status"] == 202, "emergency landing telemetry rejected")
    fleet = get_fleet(base_url)
    target = find_drone(fleet, drone_id)
    expect(target.get("safety_state") == "EMERGENCY_LAND", "emergency state not preserved")
    expect(target.get("security_state") == "LAND_IMMEDIATELY", "emergency security state mismatch")
    readiness = get_readiness(base_url)
    remote_gate_observed = target.get("remote_command_allowed")
    expect(
        remote_gate_observed in (False, None),
        f"unexpected remote_command_allowed echo in emergency scenario: {remote_gate_observed!r}",
    )
    return {
        "name": "emergency_landing_behavior",
        "pass": True,
        "latency_ms": summarize_latency([post_result["latency_ms"], fleet["_latency_ms"], readiness["_latency_ms"]]),
        "timeline": [{"event": "telemetry_posted", "drone_id": drone_id, "latency_ms": post_result["latency_ms"]}],
        "observations": {
            "safety_state": target.get("safety_state"),
            "security_state": target.get("security_state"),
            "remote_command_allowed": remote_gate_observed,
            "readiness_status": readiness.get("status"),
            "readiness_reason": readiness.get("reason"),
        },
    }


def summarize_latency(samples_ms: list[float]) -> dict[str, float]:
    return {
        "count": float(len(samples_ms)),
        "average_ms": statistics.fmean(samples_ms) if samples_ms else 0.0,
        "p50_ms": percentile(samples_ms, 0.50),
        "p95_ms": percentile(samples_ms, 0.95),
        "p99_ms": percentile(samples_ms, 0.99),
        "max_ms": max(samples_ms) if samples_ms else 0.0,
    }


def main() -> int:
    DOC_ROOT.mkdir(parents=True, exist_ok=True)
    RESULTS_ROOT.mkdir(parents=True, exist_ok=True)
    EXPERIMENT_ROOT.mkdir(parents=True, exist_ok=True)
    if RUN_LOG_PATH.exists():
        RUN_LOG_PATH.unlink()

    experiment_id = f"phase7-{datetime.now(UTC).strftime('%Y%m%dT%H%M%SZ')}"
    run_id = experiment_id
    backend_port = suite.allocate_free_port()
    backend_log = RESULTS_ROOT / "phase7_backend.log"
    scenario_fns = [
        formation_scenario,
        gps_denied_scenario,
        communication_loss_scenario,
        obstacle_degradation_scenario,
        emergency_landing_scenario,
    ]

    started_at = utc_now()
    process, startup_ms = suite.start_backend(
        backend_port,
        backend_log,
        mode="production",
        simulation_enabled=False,
        extra_env={"DRONE_BACKEND_STALE_SEC": "2"},
    )
    scenario_results: list[dict[str, Any]] = []
    failures: list[dict[str, Any]] = []
    try:
        base_url = f"http://127.0.0.1:{backend_port}"
        log(f"started backend on {base_url} pid={process.pid} startup_ms={startup_ms:.3f}")
        process_before = sample_metrics_process(process.pid)
        for fn in scenario_fns:
            scenario_started = time.perf_counter()
            scenario_name = fn.__name__.replace("_scenario", "")
            log(f"running scenario {scenario_name}")
            try:
                result = fn(base_url, experiment_id)
                result["started_at"] = utc_now()
                result["duration_ms"] = (time.perf_counter() - scenario_started) * 1000.0
                scenario_results.append(result)
                log(f"scenario {result['name']} PASS duration_ms={result['duration_ms']:.3f}")
            except Exception as exc:  # noqa: BLE001
                duration_ms = (time.perf_counter() - scenario_started) * 1000.0
                failure = {
                    "name": scenario_name,
                    "pass": False,
                    "duration_ms": duration_ms,
                    "error": str(exc),
                }
                scenario_results.append(failure)
                failures.append(failure)
                log(f"scenario {scenario_name} FAIL error={exc}")
        final_fleet = get_fleet(base_url)
        final_health = get_health(base_url)
        final_readiness = get_readiness(base_url)
        metrics_status = request_metrics(base_url)
        process_after = sample_metrics_process(process.pid)
    finally:
        suite.stop_backend(process)

    completed_at = utc_now()
    all_latencies = [
        result["latency_ms"]["average_ms"]
        for result in scenario_results
        if result.get("pass") and isinstance(result.get("latency_ms"), dict)
    ]
    report = {
        "generated_at": completed_at,
        "phase": "Phase 7",
        "experiment_id": experiment_id,
        "run_id": run_id,
        "started_at": started_at,
        "completed_at": completed_at,
        "backend": {
            "mode": "production",
            "simulation_enabled": False,
            "stale_after_seconds": 2,
            "startup_ms": startup_ms,
            "port": backend_port,
            "log_path": str(backend_log.relative_to(REPO_ROOT)),
            "process_before": process_before,
            "process_after": process_after,
        },
        "environment": {
            "platform": platform.platform(),
            "python_version": sys.version,
            "git_revision": git_revision(),
        },
        "config_snapshots": [
            hash_file(REPO_ROOT / "config" / "runtime.json"),
            hash_file(REPO_ROOT / "config" / "anchors.json"),
            hash_file(REPO_ROOT / "config" / "lidar.json"),
            hash_file(REPO_ROOT / "config" / "swarm_edge_protocol.json"),
        ],
        "scenario_count": len(scenario_results),
        "pass_count": sum(1 for result in scenario_results if result.get("pass")),
        "fail_count": len(failures),
        "scenarios": scenario_results,
        "latency_summary_ms": summarize_latency(all_latencies),
        "final_snapshot": {
            "fleet": {
                "leader_id": final_fleet.get("leader_id"),
                "avg_latency_ms": final_fleet.get("avg_latency_ms"),
                "avg_peer_latency_ms": final_fleet.get("avg_peer_latency_ms"),
                "avg_mesh_bandwidth_kbps": final_fleet.get("avg_mesh_bandwidth_kbps"),
                "packet_loss_pct": final_fleet.get("packet_loss_pct"),
                "critical_alerts": final_fleet.get("critical_alerts"),
                "real_drone_count": final_fleet.get("real_drone_count"),
                "stale_drone_count": final_fleet.get("stale_drone_count"),
                "cluster_count": len(final_fleet.get("clusters", [])),
                "fleet_latency_ms": final_fleet.get("_latency_ms"),
            },
            "health": final_health,
            "readiness": final_readiness,
            "metrics": metrics_status,
        },
        "limitations": [
            "This artifact validates software-level mission state propagation through the Go control-plane backend, not actuator-level flight behavior.",
            "No PX4, Gazebo, Ignition, or physical hardware interfaces were exercised in this run.",
            "Failure injection is represented by synthetic telemetry/state transitions rather than real sensor or radio impairment hardware.",
        ],
        "status": "PASS" if not failures else "FAIL",
    }
    OUTPUT_PATH.write_text(json.dumps(report, indent=2), encoding="utf-8")
    metadata = {
        "experiment_id": experiment_id,
        "run_id": run_id,
        "phase": "Phase 7",
        "started_at": started_at,
        "completed_at": completed_at,
        "artifact": str(OUTPUT_PATH.relative_to(REPO_ROOT)),
        "status": report["status"],
        "config_snapshots": report["config_snapshots"],
    }
    (EXPERIMENT_ROOT / f"{run_id}.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
