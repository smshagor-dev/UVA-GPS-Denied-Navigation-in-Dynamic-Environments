#!/usr/bin/env python3
"""Validate tracked JSON configuration files against Phase 5 schemas."""

from __future__ import annotations

import json
from pathlib import Path
import sys

from jsonschema import Draft202012Validator


REPO_ROOT = Path(__file__).resolve().parent.parent

SCHEMA_MAP = {
    "config/runtime.json": "config/schema/runtime.schema.json",
    "config/runtime.example.json": "config/schema/runtime.schema.json",
    "config/anchors.json": "config/schema/anchors.schema.json",
    "config/anchors.example.json": "config/schema/anchors.schema.json",
    "config/lidar.json": "config/schema/lidar.schema.json",
    "config/lidar.example.json": "config/schema/lidar.schema.json",
    "config/detector_labels.json": "config/schema/detector_labels.schema.json",
    "config/detector_labels.example.json": "config/schema/detector_labels.schema.json",
    "config/swarm_edge_protocol.json": "config/schema/swarm_edge_protocol.schema.json",
    "config/swarm_edge_protocol.example.json": "config/schema/swarm_edge_protocol.schema.json",
}


def load_json(path: Path) -> object:
    return json.loads(path.read_text(encoding="utf-8"))


def main() -> int:
    failures: list[str] = []
    for config_rel, schema_rel in SCHEMA_MAP.items():
        config_path = REPO_ROOT / config_rel
        schema_path = REPO_ROOT / schema_rel
        config_data = load_json(config_path)
        schema_data = load_json(schema_path)
        validator = Draft202012Validator(schema_data)
        errors = sorted(
            validator.iter_errors(config_data), key=lambda err: list(err.path)
        )
        if errors:
            failures.append(
                f"{config_rel} failed {schema_rel}: "
                + "; ".join(error.message for error in errors[:5])
            )
            continue
        print(f"PASS {config_rel} -> {schema_rel}")

    if failures:
        print("FAIL schema validation", file=sys.stderr)
        for failure in failures:
            print(failure, file=sys.stderr)
        return 1

    print("PASS all tracked configuration files validated against Phase 5 schemas")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
