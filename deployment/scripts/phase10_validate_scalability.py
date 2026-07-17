#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
SCRIPTS_ROOT = REPO_ROOT / "scripts"
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

import phase6_performance_suite as phase6
from deployment.scripts.common import DOC_ROOT, RESULTS_ROOT, report_date, utc_now, write_json, write_markdown


OUTPUT_JSON = RESULTS_ROOT / "scalability_results.json"
OUTPUT_MD = DOC_ROOT / "SCALABILITY_REPORT.md"


def main() -> int:
    backend_port = phase6.allocate_free_port()
    backend_log = RESULTS_ROOT / "scalability_backend.log"
    process, startup_ms = phase6.start_backend(
        backend_port,
        backend_log,
        mode="simulation",
        simulation_enabled=True,
        extra_env={"DRONE_BACKEND_DEMO_FLEET_SIZE": "250"},
    )
    try:
        base_url = f"http://127.0.0.1:{backend_port}"
        stress_100 = phase6.run_stress_test_with_stats(base_url, process.pid, total_requests=200, workers=100)
        stress_500 = phase6.run_stress_test_with_stats(base_url, process.pid, total_requests=1000, workers=500)
        stress_1000 = phase6.run_stress_test_with_stats(base_url, process.pid, total_requests=2000, workers=1000)
        fleet_metric = phase6.benchmark_backend_reads(base_url, 20)
        status, fleet_snapshot = phase6.request_json("GET", f"{base_url}/api/v1/fleet")
        if status != 200:
            raise RuntimeError(f"fleet snapshot failed: {status}")
    finally:
        phase6.stop_backend(process)
    payload = {
        "generated_at": utc_now(),
        "backend_startup_ms": startup_ms,
        "stress": {
            "100_clients": stress_100,
            "500_clients": stress_500,
            "1000_clients": stress_1000,
        },
        "fleet_get_metric": {
            "mean_ms": fleet_metric.mean_ms,
            "p95_ms": fleet_metric.p95_ms,
            "max_ms": fleet_metric.max_ms,
        },
        "large_swarm_snapshot": {
            "total_drones": len(fleet_snapshot.get("drones", [])),
            "cluster_count": len(fleet_snapshot.get("clusters", [])),
            "real_drone_count": fleet_snapshot.get("real_drone_count"),
        },
        "status": "PASS"
        if stress_100["failures"] == []
        and stress_500["failures"] == []
        and stress_1000["failures"] == []
        and len(fleet_snapshot.get("drones", [])) >= 200
        else "FAIL",
    }
    write_json(OUTPUT_JSON, payload)
    write_markdown(
        OUTPUT_MD,
        [
            "# Phase 10 Scalability Report",
            "",
            f"Date: {report_date()}",
            "",
            "## Summary",
            "",
            f"- backend startup: `{startup_ms:.3f} ms`",
            f"- fleet snapshot drones: `{len(fleet_snapshot.get('drones', []))}`",
            f"- 100-client throughput: `{stress_100['throughput_hz']:.3f} req/s`",
            f"- 500-client throughput: `{stress_500['throughput_hz']:.3f} req/s`",
            f"- 1000-client throughput: `{stress_1000['throughput_hz']:.3f} req/s`",
            f"- fleet GET p95: `{fleet_metric.p95_ms:.3f} ms`",
            "",
            "## Evidence",
            "",
            "- concurrent client load executed at 100, 500, and 1000 worker levels",
            "- large swarm simulation executed with 250 seeded drones",
            "- CPU and memory deltas captured by stress helper sampling",
            "",
            "## Verdict",
            "",
            f"Status: {payload['status']}",
        ],
    )
    return 0 if payload["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
