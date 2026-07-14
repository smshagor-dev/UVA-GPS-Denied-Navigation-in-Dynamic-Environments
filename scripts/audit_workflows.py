#!/usr/bin/env python3
"""Audit repository workflows for unsafe action references and broad permissions."""

from __future__ import annotations

import re
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
WORKFLOWS_DIR = REPO_ROOT / ".github" / "workflows"
FLOATING_REFS = {"main", "master", "latest"}
OFFICIAL_PREFIXES = ("actions/", "github/codeql-action/")
USES_RE = re.compile(r"^\s*uses:\s*([^\s#]+)")


def main() -> int:
    failures: list[str] = []
    for workflow in sorted(WORKFLOWS_DIR.glob("*.yml")):
        text = workflow.read_text(encoding="utf-8")
        if "permissions:" not in text:
            failures.append(f"{workflow.name}: missing explicit permissions block")
        for line in text.splitlines():
            match = USES_RE.match(line)
            if not match:
                continue
            value = match.group(1)
            if value.startswith("./"):
                continue
            if "@" not in value:
                failures.append(
                    f"{workflow.name}: action reference without version: {value}"
                )
                continue
            action_name, ref = value.split("@", 1)
            if ref in FLOATING_REFS:
                failures.append(f"{workflow.name}: floating action ref {value}")
            if action_name.startswith(OFFICIAL_PREFIXES):
                continue
            if not re.fullmatch(r"[0-9a-fA-F]{40}", ref):
                failures.append(
                    f"{workflow.name}: third-party action should be pinned to a full commit SHA: {value}"
                )

    if failures:
        print("FAIL: workflow audit detected issues:")
        for failure in failures:
            print(f"  - {failure}")
        return 1

    print("PASS: workflow audit found no floating refs or missing permissions.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
