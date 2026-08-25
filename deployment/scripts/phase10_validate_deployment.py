#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from deployment.scripts.common import DOC_ROOT, RESULTS_ROOT, report_date, run_command, utc_now, write_json, write_markdown
from deployment.scripts.validate_environment import validate_env_template


OUTPUT_JSON = RESULTS_ROOT / "deployment_validation.json"
OUTPUT_MD = DOC_ROOT / "DEPLOYMENT_REPORT.md"


def main() -> int:
    DOC_ROOT.mkdir(parents=True, exist_ok=True)
    RESULTS_ROOT.mkdir(parents=True, exist_ok=True)
    files = {
        "root_dockerfile": REPO_ROOT / "Dockerfile",
        "production_dockerfile": REPO_ROOT / "deployment/docker/Dockerfile.production",
        "root_compose": REPO_ROOT / "docker-compose.yml",
        "root_production_compose": REPO_ROOT / "docker-compose.production.yml",
        "deployment_production_compose": REPO_ROOT / "deployment/compose/docker-compose.production.yml",
        "env_template": REPO_ROOT / "deployment/compose/.env.production.example",
        "secret_template": REPO_ROOT / "deployment/compose/control-plane.secrets.env.template",
    }
    presence = {name: path.exists() for name, path in files.items()}
    dockerfile_text = files["production_dockerfile"].read_text(encoding="utf-8")
    compose_default = run_command(["docker", "compose", "-f", "docker-compose.yml", "config"])
    compose_prod = run_command(["docker", "compose", "-f", "docker-compose.production.yml", "config"])
    env_validation = validate_env_template(files["env_template"])
    payload = {
        "generated_at": utc_now(),
        "presence": presence,
        "production_dockerfile": {
            "multi_stage": dockerfile_text.count("FROM ") >= 2,
            "uses_non_root": "USER drone" in dockerfile_text,
            "has_healthcheck": "HEALTHCHECK" in dockerfile_text,
            "has_graceful_stop": "STOPSIGNAL" in dockerfile_text,
        },
        "compose_validation": {
            "default": compose_default,
            "production": compose_prod,
        },
        "env_validation": env_validation,
        "status": "PASS" if all(presence.values()) and env_validation["valid"] and compose_default["returncode"] == 0 and compose_prod["returncode"] == 0 else "FAIL",
    }
    write_json(OUTPUT_JSON, payload)
    write_markdown(
        OUTPUT_MD,
        [
            "# Phase 10 Deployment Report",
            "",
            f"Date: {report_date()}",
            "",
            "## Summary",
            "",
            f"- production dockerfile present: `{presence['production_dockerfile']}`",
            f"- multi-stage build: `{payload['production_dockerfile']['multi_stage']}`",
            f"- non-root runtime: `{payload['production_dockerfile']['uses_non_root']}`",
            f"- compose config default: `{compose_default['returncode'] == 0}`",
            f"- compose config production: `{compose_prod['returncode'] == 0}`",
            f"- environment template valid: `{env_validation['valid']}`",
            "",
            "## Delivered",
            "",
            "- production Docker image definition",
            "- production Compose stack",
            "- secret template and environment template",
            "- health checks, restart policy, graceful stop, read-only filesystem, and no-new-privileges settings",
            "",
            "## Artifact",
            "",
            f"- `results/phase10/{OUTPUT_JSON.name}`",
            "",
            "## Verdict",
            "",
            f"Status: {payload['status']}",
        ],
    )
    return 0 if payload["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
