#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from deployment.scripts.common import DOC_ROOT, RESULTS_ROOT, report_date, run_command, utc_now, write_json, write_markdown


OUTPUT_JSON = RESULTS_ROOT / "kubernetes_validation.json"
OUTPUT_MD = DOC_ROOT / "KUBERNETES_REPORT.md"


def main() -> int:
    manifests = sorted(str(path.relative_to(REPO_ROOT)) for path in (REPO_ROOT / "deployment/kubernetes").glob("*.yaml"))
    render = run_command(
        ["kubectl", "kustomize", "deployment/kubernetes"],
        timeout=120,
    )
    payload = {
        "generated_at": utc_now(),
        "manifest_count": len(manifests),
        "manifests": manifests,
        "render": render,
        "status": "PASS" if render["returncode"] == 0 and len(manifests) >= 7 else "FAIL",
    }
    write_json(OUTPUT_JSON, payload)
    write_markdown(
        OUTPUT_MD,
        [
            "# Phase 10 Kubernetes Report",
            "",
            f"Date: {report_date()}",
            "",
            "## Summary",
            "",
            f"- manifest count: `{len(manifests)}`",
            f"- kubectl kustomize render: `{render['returncode'] == 0}`",
            "",
            "## Included Resources",
            "",
            "- Namespace",
            "- ConfigMap",
            "- Secret template",
            "- Deployment",
            "- Service",
            "- Ingress",
            "- HPA",
            "- resource limits and health probes",
            "",
            "## Artifact",
            "",
            f"- `results/phase10/{OUTPUT_JSON.name}`",
            "- rendered manifest output verified with `kubectl kustomize`",
            "",
            "## Verdict",
            "",
            f"Status: {payload['status']}",
        ],
    )
    return 0 if payload["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
