#!/usr/bin/env python
"""Bench acceptance preflight checks for GPS-denied UAV hardware."""

from __future__ import annotations

import argparse
import json
import os
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Any


def _env_bool(name: str, default: bool) -> bool:
    value = os.environ.get(name, "").strip().lower()
    if not value:
        return default
    return value in {"1", "true", "yes", "on"}


def _normalize_mode(value: str) -> str:
    normalized = str(value or "").strip().lower()
    if normalized in {"prod", "production"}:
        return "production"
    if normalized == "bench":
        return "bench"
    return "simulation"


@dataclass(slots=True)
class CheckResult:
    name: str
    passed: bool
    reason: str


class BenchChecker:
    def __init__(self, runtime_config_path: Path, backend_url: str, timeout_s: float) -> None:
        self.runtime_config_path = runtime_config_path
        self.backend_url = backend_url.rstrip("/")
        self.timeout_s = timeout_s
        self.runtime_config: dict[str, Any] = {}
        self.anchor_config_path: Path | None = None
        self.lidar_config_path: Path | None = None
        self.results: list[CheckResult] = []

    def run(self) -> int:
        self._check_runtime_config()
        self._check_runtime_mode()
        self._check_anchor_config()
        self._check_lidar_config()
        self._check_camera_open()
        self._check_imu_open()
        self._check_backend_reachable()
        self._check_dashboard_backend_mode_visible()
        return self._print_summary()

    def _record(self, name: str, passed: bool, reason: str) -> None:
        self.results.append(CheckResult(name=name, passed=passed, reason=reason))

    def _load_json_file(self, path: Path) -> dict[str, Any]:
        with path.open("r", encoding="utf-8") as handle:
            data = json.load(handle)
        if not isinstance(data, dict):
            raise ValueError(f"{path} must contain a JSON object")
        return data

    def _check_runtime_config(self) -> None:
        name = "runtime_config"
        if not self.runtime_config_path.exists():
            self._record(name, False, f"missing runtime config: {self.runtime_config_path}")
            return
        try:
            self.runtime_config = self._load_json_file(self.runtime_config_path)
        except Exception as exc:  # noqa: BLE001
            self._record(name, False, f"failed to load runtime config: {exc}")
            return
        anchor_value = str(self.runtime_config.get("anchor_config_path", "")).strip()
        lidar_value = str(self.runtime_config.get("lidar_config_path", "")).strip()
        self.anchor_config_path = self._resolve_repo_path(anchor_value) if anchor_value else None
        self.lidar_config_path = self._resolve_repo_path(lidar_value) if lidar_value else None
        self._record(name, True, f"loaded {self.runtime_config_path}")

    def _check_runtime_mode(self) -> None:
        mode = _normalize_mode(self.runtime_config.get("runtime_mode", "simulation"))
        if mode == "simulation":
            self._record("runtime_mode", False, "runtime_mode is simulation; bench acceptance requires bench or production")
            return
        self._record("runtime_mode", True, f"runtime_mode={mode}")

    def _check_anchor_config(self) -> None:
        name = "anchor_config"
        if self.anchor_config_path is None:
            self._record(name, False, "runtime config does not define anchor_config_path")
            return
        if not self.anchor_config_path.exists():
            self._record(name, False, f"missing anchor config: {self.anchor_config_path}")
            return
        try:
            data = self._load_json_file(self.anchor_config_path)
            anchors = data.get("anchors", [])
            if not isinstance(anchors, list) or len(anchors) < 4:
                raise ValueError("anchor config must define at least 4 anchors")
        except Exception as exc:  # noqa: BLE001
            self._record(name, False, f"invalid anchor config: {exc}")
            return
        self._record(name, True, f"anchors loaded from {self.anchor_config_path} ({len(anchors)} anchors)")

    def _check_lidar_config(self) -> None:
        name = "lidar_config"
        if self.lidar_config_path is None:
            self._record(name, False, "runtime config does not define lidar_config_path")
            return
        if not self.lidar_config_path.exists():
            self._record(name, False, f"missing LiDAR config: {self.lidar_config_path}")
            return
        try:
            data = self._load_json_file(self.lidar_config_path)
            host = str(data.get("host", "")).strip()
            port = int(data.get("port", 0))
            model = str(data.get("model", "")).strip()
            frame_id = str(data.get("frame_id", "")).strip()
            if not host or port <= 0 or not model or not frame_id:
                raise ValueError("LiDAR config must define host, port, model, and frame_id")
        except Exception as exc:  # noqa: BLE001
            self._record(name, False, f"invalid LiDAR config: {exc}")
            return
        self._record(name, True, f"LiDAR configured host={host} port={port} model={model}")

    def _check_camera_open(self) -> None:
        name = "camera_open"
        if not _env_bool("DRONE_ENABLE_CAMERA", True):
            self._record(name, False, "DRONE_ENABLE_CAMERA is false")
            return
        stream_url = os.environ.get("DRONE_CAMERA_STREAM_URL", "").strip()
        if not stream_url:
            esp32_ip = os.environ.get("DRONE_ESP32_IP", "192.168.4.1").strip()
            stream_url = f"rtsp://{esp32_ip}:554/stream"
        try:
            import cv2  # type: ignore
        except Exception as exc:  # noqa: BLE001
            self._record(name, False, f"OpenCV unavailable for camera probe: {exc}")
            return
        capture = cv2.VideoCapture(stream_url)
        try:
            if not capture.isOpened():
                self._record(name, False, f"failed to open camera stream: {stream_url}")
                return
            ok, _frame = capture.read()
            if not ok:
                self._record(name, False, f"camera stream opened but no frame received: {stream_url}")
                return
        finally:
            capture.release()
        self._record(name, True, f"camera stream responded: {stream_url}")

    def _check_imu_open(self) -> None:
        name = "imu_open"
        if not _env_bool("DRONE_ENABLE_IMU", True):
            self._record(name, False, "DRONE_ENABLE_IMU is false")
            return
        imu_device = Path(os.environ.get("DRONE_IMU_DEVICE", "/dev/i2c-1"))
        if not imu_device.exists():
            self._record(name, False, f"IMU device does not exist: {imu_device}")
            return
        try:
            with imu_device.open("rb"):
                pass
        except Exception as exc:  # noqa: BLE001
            self._record(name, False, f"failed to open IMU device {imu_device}: {exc}")
            return
        self._record(name, True, f"IMU device opened: {imu_device}")

    def _check_backend_reachable(self) -> None:
        name = "backend_reachable"
        if not self.backend_url:
            self._record(name, False, "DRONE_BACKEND_URL or --backend-url is not set")
            return
        try:
            self._fetch_backend_payload()
        except urllib.error.URLError as exc:
            self._record(name, False, f"backend unreachable at {self.backend_url}/api/v1/fleet: {exc}")
            return
        except Exception as exc:  # noqa: BLE001
            self._record(name, False, f"backend response invalid: {exc}")
            return
        self._record(name, True, f"backend reachable at {self.backend_url}/api/v1/fleet")

    def _check_dashboard_backend_mode_visible(self) -> None:
        name = "dashboard_backend_mode"
        if not self.backend_url:
            self._record(name, False, "backend URL unavailable, cannot verify dashboard-visible backend mode")
            return
        try:
            payload = self._fetch_backend_payload()
        except Exception as exc:  # noqa: BLE001
            self._record(name, False, f"failed to query backend mode visibility: {exc}")
            return
        backend_mode = _normalize_mode(payload.get("backend_mode", "simulation"))
        simulation_enabled = bool(payload.get("simulation_enabled", True))
        if "backend_mode" not in payload:
            self._record(name, False, "fleet response is missing backend_mode field")
            return
        if backend_mode == "simulation":
            self._record(name, False, "backend_mode is simulation; dashboard would not be in real-data mode")
            return
        if simulation_enabled:
            self._record(name, False, "simulation_enabled is true; dashboard would still show simulated backend")
            return
        self._record(name, True, f"backend_mode visible as {backend_mode} with simulation disabled")

    def _fetch_backend_payload(self) -> dict[str, Any]:
        fleet_url = f"{self.backend_url}/api/v1/fleet"
        with urllib.request.urlopen(fleet_url, timeout=self.timeout_s) as response:
            payload = json.loads(response.read().decode("utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("backend fleet response is not a JSON object")
        return payload

    def _resolve_repo_path(self, value: str) -> Path:
        path = Path(value)
        if path.is_absolute():
            return path
        return (self.runtime_config_path.parent.parent / path).resolve()

    def _print_summary(self) -> int:
        failures = [result for result in self.results if not result.passed]
        for result in self.results:
            status = "PASS" if result.passed else "FAIL"
            print(f"[{status}] {result.name}: {result.reason}")
        if failures:
            print()
            print("BENCH VERDICT: FAIL")
            print("Flight readiness must not be claimed until all bench checks pass.")
            return 1
        print()
        print("BENCH VERDICT: PASS")
        print("Bench gate passed. This is necessary before flight, but it is not by itself a flight-readiness claim.")
        return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Bench acceptance preflight checks")
    parser.add_argument(
        "--runtime-config",
        default=os.environ.get("DRONE_RUNTIME_CONFIG", "config/runtime.json"),
        help="Path to runtime JSON config",
    )
    parser.add_argument(
        "--backend-url",
        default=os.environ.get("DRONE_BACKEND_URL", ""),
        help="Control-plane backend URL",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=2.5,
        help="Network timeout in seconds",
    )
    args = parser.parse_args()

    checker = BenchChecker(
        runtime_config_path=Path(args.runtime_config).resolve(),
        backend_url=args.backend_url,
        timeout_s=max(args.timeout, 0.1),
    )
    return checker.run()


if __name__ == "__main__":
    raise SystemExit(main())
