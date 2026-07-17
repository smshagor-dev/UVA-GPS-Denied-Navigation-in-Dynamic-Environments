#!/usr/bin/env python3
"""Run Phase 6 stress scenarios and save raw results."""

from __future__ import annotations

import json
from datetime import UTC, datetime
from pathlib import Path

import phase6_performance_suite as suite


OUTPUT_PATH = suite.DOC_ROOT / "stress_results.json"


def main() -> int:
    suite.DOC_ROOT.mkdir(parents=True, exist_ok=True)
    scenarios = [
        {"name": "api_stress_100", "requests": 1000, "workers": 100},
        {"name": "api_stress_500", "requests": 2500, "workers": 500},
        {"name": "api_stress_1000", "requests": 5000, "workers": 1000},
        {"name": "telemetry_burst_multi_node", "requests": 4000, "workers": 200},
    ]

    results: list[dict[str, object]] = []
    for scenario in scenarios:
        port = suite.allocate_free_port()
        log_path = suite.DOC_ROOT / f"{scenario['name']}.log"
        process, startup_ms = suite.start_backend(port, log_path)
        try:
            result = suite.run_stress_test_with_stats(
                f"http://127.0.0.1:{port}",
                process.pid,
                total_requests=int(scenario["requests"]),
                workers=int(scenario["workers"]),
            )
            result["name"] = scenario["name"]
            result["startup_ms"] = startup_ms
            results.append(result)
        finally:
            suite.stop_backend(process)

    payload = {
        "generated_at": datetime.now(UTC).isoformat().replace("+00:00", "Z"),
        "scenarios": results,
        "notes": [
            "Stress scenarios were run against the local Windows control-plane backend in production mode.",
            "The 1000-client scenario is thread-limited by the local workstation and should be treated as a host-saturation check, not a production capacity claim.",
        ],
    }
    OUTPUT_PATH.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
