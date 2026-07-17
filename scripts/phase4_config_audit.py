#!/usr/bin/env python3
"""Validate tracked JSON configuration files and emit a Phase 4 audit report."""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field
from datetime import UTC, datetime
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
OUTPUT_PATH = REPO_ROOT / "docs" / "phase4" / "config_audit.json"
ALLOWED_RUNTIME_MODES = {"simulation", "bench", "production", "edge_swarm"}
CONFIG_FILES = [
    "config/runtime.json",
    "config/runtime.example.json",
    "config/anchors.json",
    "config/anchors.example.json",
    "config/lidar.json",
    "config/lidar.example.json",
    "config/detector_labels.json",
    "config/detector_labels.example.json",
    "config/swarm_edge_protocol.json",
    "config/swarm_edge_protocol.example.json",
]


@dataclass
class ConfigResult:
    path: str
    ok: bool = True
    facts: dict[str, Any] = field(default_factory=dict)
    warnings: list[str] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def as_bool(value: Any) -> bool | None:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"true", "1", "yes", "on"}:
            return True
        if lowered in {"false", "0", "no", "off"}:
            return False
    return None


def is_finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(value)


def resolve_repo_path(raw: str) -> Path:
    return (REPO_ROOT / raw).resolve()


def validate_runtime_config(path: Path, data: dict[str, Any]) -> ConfigResult:
    result = ConfigResult(str(path.relative_to(REPO_ROOT)))
    global_cfg = data.get("global_configuration")
    if not isinstance(global_cfg, dict):
        result.ok = False
        result.errors.append("missing object: global_configuration")
        return result

    runtime_mode = str(global_cfg.get("runtime_mode", "")).strip().lower()
    if runtime_mode not in ALLOWED_RUNTIME_MODES:
        result.ok = False
        result.errors.append(f"invalid runtime_mode: {runtime_mode!r}")

    for key in ("anchor_config_path", "lidar_config_path", "detector_labels_path"):
        raw_value = str(global_cfg.get(key, "")).strip()
        if not raw_value:
            result.ok = False
            result.errors.append(f"missing path: {key}")
            continue
        resolved = resolve_repo_path(raw_value)
        result.facts[key] = raw_value
        if not resolved.exists():
            result.ok = False
            result.errors.append(f"{key} does not exist: {raw_value}")

    publish_rate = global_cfg.get("telemetry_publish_rate_hz")
    if publish_rate is not None and (
        not is_finite_number(publish_rate) or float(publish_rate) <= 0.0
    ):
        result.ok = False
        result.errors.append("telemetry_publish_rate_hz must be a positive number")

    supported = data.get("runtime_modes_supported", [])
    if not isinstance(supported, list) or not supported:
        result.ok = False
        result.errors.append("runtime_modes_supported must be a non-empty list")
    else:
        unsupported = sorted({str(item) for item in supported} - ALLOWED_RUNTIME_MODES)
        if unsupported:
            result.ok = False
            result.errors.append(
                f"runtime_modes_supported contains unsupported values: {unsupported}"
            )

    dataset_type = str(data.get("dataset_type", "")).strip().lower()
    if "example" in path.name and "example" not in dataset_type:
        result.warnings.append(
            "example config is not explicitly labeled with an example dataset_type"
        )

    result.facts["runtime_mode"] = runtime_mode
    result.facts["dataset_type"] = data.get("dataset_type")
    return result


def validate_anchor_config(path: Path, data: dict[str, Any]) -> ConfigResult:
    result = ConfigResult(str(path.relative_to(REPO_ROOT)))
    anchors = data.get("anchors")
    if not isinstance(anchors, list) or len(anchors) < 4:
        result.ok = False
        result.errors.append("anchors must be a list with at least 4 entries")
        return result

    seen_ids: set[str] = set()
    for index, anchor in enumerate(anchors):
        if not isinstance(anchor, dict):
            result.ok = False
            result.errors.append(f"anchor[{index}] is not an object")
            continue
        anchor_id = str(anchor.get("id", "")).strip()
        if not anchor_id:
            result.ok = False
            result.errors.append(f"anchor[{index}] missing id")
        elif anchor_id in seen_ids:
            result.ok = False
            result.errors.append(f"duplicate anchor id: {anchor_id}")
        else:
            seen_ids.add(anchor_id)
        for axis in ("x", "y", "z"):
            if not is_finite_number(anchor.get(axis)):
                result.ok = False
                result.errors.append(f"anchor[{index}] invalid {axis}")

    result.facts["anchor_count"] = len(anchors)
    result.facts["coordinate_frame"] = data.get("coordinate_frame")
    result.facts["units"] = data.get("units")
    return result


def extract_lidar_range(data: dict[str, Any]) -> tuple[Any, Any]:
    range_block = data.get("range")
    if isinstance(range_block, dict):
        return range_block.get("min_range_m"), range_block.get("max_range_m")
    return data.get("min_range_m"), data.get("max_range_m")


def validate_lidar_config(path: Path, data: dict[str, Any]) -> ConfigResult:
    result = ConfigResult(str(path.relative_to(REPO_ROOT)))
    host = str(data.get("host", "")).strip()
    model = str(data.get("model", "")).strip()
    frame_id = str(data.get("frame_id", "")).strip()
    port = data.get("port")
    min_range, max_range = extract_lidar_range(data)
    required = as_bool(data.get("required"))

    if not host:
        result.ok = False
        result.errors.append("missing host")
    if not is_finite_number(port) or not (1 <= int(port) <= 65535):
        result.ok = False
        result.errors.append("port must be between 1 and 65535")
    if not model:
        result.ok = False
        result.errors.append("missing model")
    if not frame_id:
        result.ok = False
        result.errors.append("missing frame_id")
    if not is_finite_number(min_range) or not is_finite_number(max_range):
        result.ok = False
        result.errors.append("min_range_m and max_range_m must be finite numbers")
    elif float(min_range) < 0.0 or float(max_range) <= float(min_range):
        result.ok = False
        result.errors.append("invalid LiDAR range ordering")
    if required is None:
        result.ok = False
        result.errors.append("required must be a boolean")

    result.facts["host"] = host
    result.facts["port"] = int(port) if is_finite_number(port) else port
    result.facts["required"] = required
    result.facts["runtime_mode"] = data.get("runtime_mode")
    return result


def validate_detector_labels(path: Path, data: dict[str, Any]) -> ConfigResult:
    result = ConfigResult(str(path.relative_to(REPO_ROOT)))
    labels = data.get("labels")
    if not isinstance(labels, dict) or not labels:
        result.ok = False
        result.errors.append("labels must be a non-empty object")
        return result

    non_string_values = [
        key for key, value in labels.items() if not isinstance(value, str)
    ]
    if non_string_values:
        result.ok = False
        result.errors.append(
            f"labels contains non-string values for keys: {non_string_values[:5]}"
        )

    result.facts["label_count"] = len(labels)
    result.facts["dataset_type"] = data.get("dataset_type")
    return result


def validate_swarm_protocol(path: Path, data: dict[str, Any]) -> ConfigResult:
    result = ConfigResult(str(path.relative_to(REPO_ROOT)))
    serialization = data.get("serialization")
    auth = data.get("auth")
    if not isinstance(serialization, dict):
        result.ok = False
        result.errors.append("missing object: serialization")
    else:
        mode = str(serialization.get("serialization_mode", "")).strip().lower()
        if mode not in {"json", "cbor", "protobuf_placeholder"}:
            result.ok = False
            result.errors.append(f"unsupported serialization_mode: {mode!r}")
        result.facts["serialization_mode"] = mode
    if not isinstance(auth, dict):
        result.ok = False
        result.errors.append("missing object: auth")
    else:
        auth_mode = str(auth.get("mode", "")).strip().lower()
        if auth_mode not in {"hmac_sha256", "none", "pqc_hybrid_placeholder"}:
            result.ok = False
            result.errors.append(f"unsupported auth.mode: {auth_mode!r}")
        shared_secret_env = str(auth.get("shared_secret_env", "")).strip()
        if not shared_secret_env:
            result.ok = False
            result.errors.append("auth.shared_secret_env is required")
        result.facts["auth_mode"] = auth_mode

    return result


def validate_file(path_text: str) -> ConfigResult:
    path = REPO_ROOT / path_text
    try:
        data = load_json(path)
    except Exception as exc:  # noqa: BLE001
        return ConfigResult(
            path_text, ok=False, errors=[f"failed to parse JSON: {exc}"]
        )

    if not isinstance(data, dict):
        return ConfigResult(
            path_text, ok=False, errors=["top-level JSON value must be an object"]
        )

    if path.name.startswith("runtime"):
        return validate_runtime_config(path, data)
    if path.name.startswith("anchors"):
        return validate_anchor_config(path, data)
    if path.name.startswith("lidar"):
        return validate_lidar_config(path, data)
    if path.name.startswith("detector_labels"):
        return validate_detector_labels(path, data)
    if path.name.startswith("swarm_edge_protocol"):
        return validate_swarm_protocol(path, data)
    return ConfigResult(path_text, ok=False, errors=["no validator registered"])


def main() -> int:
    results = [validate_file(path) for path in CONFIG_FILES]
    summary = {
        "generated_at": datetime.now(UTC).isoformat().replace("+00:00", "Z"),
        "ok": all(result.ok for result in results),
        "files_checked": len(results),
        "passed": sum(1 for result in results if result.ok),
        "failed": sum(1 for result in results if not result.ok),
        "results": [asdict(result) for result in results],
    }
    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_PATH.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"wrote {OUTPUT_PATH}")
    print(
        f"config audit status={'PASS' if summary['ok'] else 'FAIL'} "
        f"passed={summary['passed']} failed={summary['failed']}"
    )
    return 0 if summary["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
