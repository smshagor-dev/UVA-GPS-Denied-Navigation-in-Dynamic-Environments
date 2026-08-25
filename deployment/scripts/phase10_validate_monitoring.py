#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
from pathlib import Path
from urllib.request import urlopen

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
SCRIPTS_ROOT = REPO_ROOT / "scripts"
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

import phase6_performance_suite as phase6
from deployment.scripts.common import DOC_ROOT, RESULTS_ROOT, report_date, utc_now, write_json, write_markdown


OUTPUT_JSON = RESULTS_ROOT / "monitoring_validation.json"
OUTPUT_MD = DOC_ROOT / "MONITORING_REPORT.md"


def get_json(url: str) -> dict[str, object]:
    with urlopen(url, timeout=10) as response:
        return json.loads(response.read().decode("utf-8"))


def get_text(url: str) -> str:
    with urlopen(url, timeout=10) as response:
        return response.read().decode("utf-8")


def main() -> int:
    backend_port = phase6.allocate_free_port()
    backend_log = RESULTS_ROOT / "monitoring_backend.log"
    process, startup_ms = phase6.start_backend(
        backend_port,
        backend_log,
        mode="simulation",
        simulation_enabled=True,
        extra_env={"DRONE_BACKEND_DEMO_FLEET_SIZE": "4"},
    )
    try:
        base_url = f"http://127.0.0.1:{backend_port}"
        health = get_json(f"{base_url}/api/v1/health")
        readiness = get_json(f"{base_url}/api/v1/ready")
        metrics = get_text(f"{base_url}/metrics")
    finally:
        phase6.stop_backend(process)
    metric_names = [line.split(" ")[0] for line in metrics.splitlines() if line and not line.startswith("#")]
    payload = {
        "generated_at": utc_now(),
        "backend_startup_ms": startup_ms,
        "health": health,
        "readiness": readiness,
        "metrics_count": len(metric_names),
        "metrics_names": metric_names,
        "prometheus_config": str((REPO_ROOT / "deployment/monitoring/prometheus.yml").relative_to(REPO_ROOT)),
        "grafana_dashboard": str((REPO_ROOT / "deployment/monitoring/grafana-control-plane-dashboard.json").relative_to(REPO_ROOT)),
        "status": "PASS" if len(metric_names) >= 8 and readiness.get("status") == "ready" else "FAIL",
    }
    write_json(OUTPUT_JSON, payload)
    write_markdown(
        OUTPUT_MD,
        [
            "# Phase 10 Monitoring Report",
            "",
            f"Date: {report_date()}",
            "",
            "## Summary",
            "",
            f"- backend startup: `{startup_ms:.3f} ms`",
            f"- readiness status: `{readiness.get('status')}`",
            f"- metrics exported: `{len(metric_names)}`",
            "",
            "## Delivered",
            "",
            "- Prometheus scrape configuration",
            "- Grafana dashboard template",
            "- live health endpoint verification",
            "- live readiness endpoint verification",
            "- metrics endpoint verification",
            "",
            "## Verdict",
            "",
            f"Status: {payload['status']}",
        ],
    )
    return 0 if payload["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
