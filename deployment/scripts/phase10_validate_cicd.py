#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from deployment.scripts.common import DOC_ROOT, RESULTS_ROOT, report_date, utc_now, write_json, write_markdown


OUTPUT_JSON = RESULTS_ROOT / "cicd_validation.json"
OUTPUT_MD = DOC_ROOT / "CICD_REPORT.md"
WORKFLOW_PATH = REPO_ROOT / ".github/workflows/phase10-enterprise-validation.yml"


def main() -> int:
    text = WORKFLOW_PATH.read_text(encoding="utf-8")
    checks = {
        "lint": "lint:" in text,
        "build": "build:" in text,
        "unit_tests": "unit-tests:" in text,
        "integration_tests": "integration-tests:" in text,
        "security_scan": "security-scan:" in text,
        "performance_validation": "performance-validation:" in text,
        "packaging": "packaging:" in text,
        "artifact_upload": "upload-artifact" in text,
        "release_validation": "release-validation:" in text,
    }
    payload = {
        "generated_at": utc_now(),
        "workflow": str(WORKFLOW_PATH),
        "checks": checks,
        "status": "PASS" if all(checks.values()) else "FAIL",
    }
    write_json(OUTPUT_JSON, payload)
    write_markdown(
        OUTPUT_MD,
        [
            "# Phase 10 CI/CD Report",
            "",
            f"Date: {report_date()}",
            "",
            "## Summary",
            "",
            f"- workflow file: `{WORKFLOW_PATH}`",
            f"- required stages present: `{all(checks.values())}`",
            "",
            "## Included Stages",
            "",
            "- lint",
            "- build",
            "- unit tests",
            "- integration tests",
            "- security scan",
            "- performance validation",
            "- packaging",
            "- artifact upload",
            "- release validation",
            "",
            "## Verdict",
            "",
            f"Status: {payload['status']}",
        ],
    )
    return 0 if payload["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
