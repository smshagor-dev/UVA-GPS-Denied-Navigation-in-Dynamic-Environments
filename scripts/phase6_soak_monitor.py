#!/usr/bin/env python3
"""Run a configurable Phase 6 soak monitor and save raw results."""

from __future__ import annotations

import argparse
import json
from datetime import UTC, datetime

import phase6_performance_suite as suite


OUTPUT_PATH = suite.DOC_ROOT / "soak_results.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration-s", type=int, default=7200)
    parser.add_argument("--interval-s", type=float, default=30.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    suite.DOC_ROOT.mkdir(parents=True, exist_ok=True)

    port = suite.allocate_free_port()
    log_path = suite.DOC_ROOT / "backend_soak.log"
    process, startup_ms = suite.start_backend(port, log_path)
    try:
        result = suite.run_soak_test(
            f"http://127.0.0.1:{port}",
            process.pid,
            duration_s=args.duration_s,
            interval_s=args.interval_s,
        )
    finally:
        suite.stop_backend(process)

    payload = {
        "generated_at": datetime.now(UTC).isoformat().replace("+00:00", "Z"),
        "startup_ms": startup_ms,
        "result": result,
        "notes": [
            "This soak monitor captures live Get-Process memory and CPU samples on Windows.",
            "Default configuration now targets the Phase 6.5 minimum soak duration and sampling cadence.",
        ],
    }
    OUTPUT_PATH.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
