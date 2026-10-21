#!/usr/bin/env python3
"""Linux CI control-plane performance smoke measurements.

The full Phase 6 suite is a Windows-specific hardware/workstation benchmark.
This job instead measures real HTTP requests against a locally started Go
control-plane on the GitHub Linux runner, using explicitly synthetic payloads.
It does not assert UAV flight performance or treat CI timings as stable SLAs.
"""

from __future__ import annotations

import json
import platform
import sys
from dataclasses import asdict
from datetime import UTC, datetime

from phase6_performance_suite import (
    DOC_ROOT,
    REPO_ROOT,
    allocate_free_port,
    benchmark_backend_reads,
    benchmark_config_load,
    benchmark_telemetry_posts,
    log,
    run_stress_test,
    start_backend,
    stop_backend,
    summarize,
)

REPORT = DOC_ROOT / "linux_ci_performance.json"
BACKEND_LOG = DOC_ROOT / "linux_ci_backend.log"


def validate_stress(result: dict) -> None:
    failures = result["failures"]
    if failures:
        raise RuntimeError(
            f"control-plane stress test returned {len(failures)} failed requests: "
            f"{failures[:3]}"
        )
    if result["requests"] != 100:
        raise RuntimeError("stress test did not cover the expected requests")
    if result["latency"]["iterations"] != 100:
        raise RuntimeError("stress test omitted request timing samples")


def main() -> int:
    DOC_ROOT.mkdir(parents=True, exist_ok=True)
    metrics = [
        benchmark_config_load(REPO_ROOT / "config" / name, 20)
        for name in ("runtime.json", "anchors.json", "lidar.json")
    ]
    port = allocate_free_port()
    log(f"Linux CI performance: starting local backend on ephemeral port {port}")
    process, startup_ms = start_backend(port, BACKEND_LOG)
    try:
        url = f"http://127.0.0.1:{port}"
        metrics.append(
            summarize("backend_startup", "process", [startup_ms], {"mode": "production"})
        )
        metrics.append(benchmark_backend_reads(url, 20))
        metrics.append(benchmark_telemetry_posts(url, 20))
        stress = run_stress_test(url, total_requests=100, workers=8)
        validate_stress(stress)
    finally:
        stop_backend(process)

    result = {
        "generated_at": datetime.now(UTC).isoformat().replace("+00:00", "Z"),
        "status": "passed",
        "source": "github-hosted-linux-runner",
        "platform": {
            "system": platform.system(),
            "release": platform.release(),
            "machine": platform.machine(),
            "python": platform.python_version(),
        },
        "scope": "Go HTTP control-plane and Python JSON config loading only",
        "synthetic_inputs": True,
        "not_measured": [
            "Windows native C++ benchmarks",
            "onboard UAV CPU latency",
            "radio loss and reordering",
            "hardware-in-the-loop or flight",
            "90-second resource soak or PowerShell process counters",
        ],
        "metrics": [asdict(m) for m in metrics],
        "stress_test": stress,
    }
    REPORT.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    log(f"Linux CI performance: wrote verified measurements to {REPORT}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"Linux performance validation failed: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
