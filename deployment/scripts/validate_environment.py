from __future__ import annotations

from pathlib import Path


REQUIRED_ENV_KEYS = [
    "DRONE_CONTROL_PLANE_PORT",
    "DRONE_BACKEND_MODE",
    "DRONE_BACKEND_SIMULATION_ENABLED",
    "DRONE_SECURITY_PROFILE",
    "DRONE_REQUIRE_SIGNED_COMMANDS",
    "DRONE_TLS_ENABLED",
    "DRONE_TLS_REQUIRE_CLIENT_CERT",
]


def parse_env_template(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        values[key.strip()] = value.strip()
    return values


def validate_env_template(path: Path) -> dict[str, object]:
    values = parse_env_template(path)
    missing = [key for key in REQUIRED_ENV_KEYS if key not in values]
    return {
        "path": str(path),
        "keys_present": sorted(values.keys()),
        "missing_keys": missing,
        "valid": not missing,
    }

