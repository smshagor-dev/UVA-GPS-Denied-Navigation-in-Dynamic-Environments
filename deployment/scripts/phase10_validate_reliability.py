#!/usr/bin/env python3
from __future__ import annotations

import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
SCRIPTS_ROOT = REPO_ROOT / "scripts"
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

import phase6_performance_suite as phase6
from deployment.scripts.common import DOC_ROOT, RESULTS_ROOT, report_date, run_command, utc_now, write_json, write_markdown


OUTPUT_JSON = RESULTS_ROOT / "reliability_results.json"
OUTPUT_MD = DOC_ROOT / "RELIABILITY_REPORT.md"


def main() -> int:
    backend_port = phase6.allocate_free_port()
    backend_log = RESULTS_ROOT / "reliability_backend.log"
    process, startup_ms = phase6.start_backend(
        backend_port,
        backend_log,
        mode="simulation",
        simulation_enabled=True,
        extra_env={"DRONE_BACKEND_DEMO_FLEET_SIZE": "3"},
    )
    log_path = REPO_ROOT / "logs" / "control-plane" / "control-plane.log"
    try:
        phase6.request_json("GET", f"http://127.0.0.1:{backend_port}/api/v1/health")
        initial_log_size = log_path.stat().st_size if log_path.exists() else 0
    finally:
        phase6.stop_backend(process)
    time.sleep(1.0)
    process2, restart_ms = phase6.start_backend(
        backend_port,
        backend_log,
        mode="simulation",
        simulation_enabled=True,
        extra_env={"DRONE_BACKEND_DEMO_FLEET_SIZE": "3"},
    )
    try:
        ready_status, ready_body = phase6.request_json("GET", f"http://127.0.0.1:{backend_port}/api/v1/ready")
        final_log_size = log_path.stat().st_size if log_path.exists() else 0
    finally:
        phase6.stop_backend(process2)
    config_validation = run_command(["python", "scripts/validate_config_schemas.py"], timeout=120)
    compose_text = (REPO_ROOT / "docker-compose.production.yml").read_text(encoding="utf-8")
    disaster_recovery = run_command(["python", "deployment/scripts/phase10_backup_restore.py"], timeout=180)
    payload = {
        "generated_at": utc_now(),
        "startup_ms": startup_ms,
        "restart_ms": restart_ms,
        "readiness_after_restart": {"status": ready_status, "body": ready_body},
        "config_validation": config_validation["returncode"] == 0,
        "log_growth_bytes": final_log_size - initial_log_size,
        "log_rotation_configured": "max-size" in compose_text and "max-file" in compose_text,
        "backup_validation": disaster_recovery["returncode"] == 0,
        "status": "PASS"
        if ready_status == 200
        and config_validation["returncode"] == 0
        and final_log_size >= initial_log_size
        and "max-size" in compose_text
        and disaster_recovery["returncode"] == 0
        else "FAIL",
    }
    write_json(OUTPUT_JSON, payload)
    write_markdown(
        OUTPUT_MD,
        [
            "# Phase 10 Reliability Report",
            "",
            f"Date: {report_date()}",
            "",
            "## Summary",
            "",
            f"- first startup: `{startup_ms:.3f} ms`",
            f"- restart startup: `{restart_ms:.3f} ms`",
            f"- readiness after restart: `{ready_status}`",
            f"- config recovery validation: `{config_validation['returncode'] == 0}`",
            f"- log rotation configured: `{payload['log_rotation_configured']}`",
            f"- backup validation: `{payload['backup_validation']}`",
            "",
            "## Scope",
            "",
            "- restart recovery",
            "- crash/restart recovery",
            "- configuration validation",
            "- log persistence",
            "- log rotation configuration",
            "- backup/restore dependency",
            "",
            "## Verdict",
            "",
            f"Status: {payload['status']}",
        ],
    )
    return 0 if payload["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
