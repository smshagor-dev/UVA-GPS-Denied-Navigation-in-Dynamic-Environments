#!/usr/bin/env python3
from __future__ import annotations

import re
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
from deployment.scripts.common import DOC_ROOT, RESULTS_ROOT, report_date, run_command, utc_now, write_json, write_markdown


OUTPUT_JSON = DOC_ROOT / "security_report.json"
OUTPUT_MD = DOC_ROOT / "SECURITY_AUDIT.md"
SUSPECT_PATTERNS = [
    (re.compile(r"-----BEGIN (RSA |EC |OPENSSH )?PRIVATE KEY-----"), "private-key-block"),
    (re.compile(r"AKIA[0-9A-Z]{16}"), "aws-access-key"),
    (re.compile(r"(?i)ghp_[0-9a-z]{20,}"), "github-token"),
]


def scan_repo() -> list[dict[str, str]]:
    findings: list[dict[str, str]] = []
    for path in REPO_ROOT.rglob("*"):
        if not path.is_file():
            continue
        if (
            ".git" in path.parts
            or "__pycache__" in path.parts
            or "results" in path.parts
            or "build" in path.parts
            or "logs" in path.parts
            or path.suffix in {".pyc", ".pyd", ".exe", ".dll", ".lib", ".obj", ".zip"}
        ):
            continue
        try:
            text = path.read_text(encoding="utf-8")
        except Exception:
            continue
        for pattern, label in SUSPECT_PATTERNS:
            if pattern.search(text):
                findings.append({"path": str(path.relative_to(REPO_ROOT)), "type": label})
    return findings


def fetch_headers(base_url: str) -> dict[str, str]:
    with urlopen(f"{base_url}/api/v1/health", timeout=10) as response:
        return dict(response.headers.items())


def main() -> int:
    findings = scan_repo()
    static_checks = {
        "security_headers_middleware": "setSecurityHeaders" in (REPO_ROOT / "internal/controlplane/server.go").read_text(encoding="utf-8"),
        "tls_hardening": "tls must be enabled outside lab mode" in (REPO_ROOT / "cmd/control-plane/main.go").read_text(encoding="utf-8"),
        "signed_command_enforcement": "RequireSignedCommands" in (REPO_ROOT / "cmd/control-plane/main.go").read_text(encoding="utf-8"),
        "compose_least_privilege": "no-new-privileges:true" in (REPO_ROOT / "docker-compose.production.yml").read_text(encoding="utf-8"),
    }
    backend_port = phase6.allocate_free_port()
    backend_log = RESULTS_ROOT / "security_backend.log"
    process, _startup_ms = phase6.start_backend(
        backend_port,
        backend_log,
        mode="simulation",
        simulation_enabled=True,
        extra_env={"DRONE_BACKEND_DEMO_FLEET_SIZE": "3"},
    )
    try:
        headers = fetch_headers(f"http://127.0.0.1:{backend_port}")
    finally:
        phase6.stop_backend(process)
    workflow_text = (REPO_ROOT / ".github/workflows/security.yml").read_text(encoding="utf-8")
    workflow_checks = {
        "gitleaks_configured": "gitleaks" in workflow_text,
        "pip_audit_configured": "pip-audit" in workflow_text,
        "govulncheck_configured": "govulncheck" in workflow_text,
        "codeql_configured": "codeql" in workflow_text.lower(),
    }
    payload = {
        "generated_at": utc_now(),
        "secret_scan_findings": findings,
        "static_checks": static_checks,
        "workflow_checks": workflow_checks,
        "live_headers": {
            "X-Content-Type-Options": headers.get("X-Content-Type-Options", ""),
            "X-Frame-Options": headers.get("X-Frame-Options", ""),
            "Referrer-Policy": headers.get("Referrer-Policy", ""),
            "Content-Security-Policy": headers.get("Content-Security-Policy", ""),
            "Permissions-Policy": headers.get("Permissions-Policy", ""),
            "Cache-Control": headers.get("Cache-Control", ""),
        },
        "status": "PASS"
        if not findings
        and all(static_checks.values())
        and all(workflow_checks.values())
        and headers.get("X-Content-Type-Options") == "nosniff"
        else "FAIL",
    }
    write_json(OUTPUT_JSON, payload)
    write_markdown(
        OUTPUT_MD,
        [
            "# Phase 10 Security Audit",
            "",
            f"Date: {report_date()}",
            "",
            "## Summary",
            "",
            f"- secret findings: `{len(findings)}`",
            f"- security headers present: `{headers.get('X-Content-Type-Options') == 'nosniff'}`",
            f"- TLS hardening gate present: `{static_checks['tls_hardening']}`",
            f"- signed command enforcement present: `{static_checks['signed_command_enforcement']}`",
            f"- least privilege compose settings present: `{static_checks['compose_least_privilege']}`",
            f"- CI security scanners configured: `{all(workflow_checks.values())}`",
            "",
            "## Evidence",
            "",
            f"- machine-readable artifact: `docs/phase10/{OUTPUT_JSON.name}`",
            "- live header verification performed against a running local backend instance",
            "- repository secret scan performed with built-in pattern checks",
            "",
            "## Verdict",
            "",
            f"Status: {payload['status']}",
        ],
    )
    return 0 if payload["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
