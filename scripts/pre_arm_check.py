#!/usr/bin/env python
"""Strict tethered-flight pre-arm gate."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

from bench_check import BenchChecker, _normalize_mode


class PreArmChecker(BenchChecker):
    def __init__(
        self,
        runtime_config_path: Path,
        backend_url: str,
        timeout_s: float,
        localization_confidence_threshold: float,
    ) -> None:
        super().__init__(
            runtime_config_path=runtime_config_path,
            backend_url=backend_url,
            timeout_s=timeout_s,
        )
        self.localization_confidence_threshold = localization_confidence_threshold

    def run(self) -> int:
        self._check_runtime_config()
        self._check_runtime_mode()
        self._check_anchor_config()
        self._check_lidar_config()
        self._check_camera_open()
        self._check_imu_open()
        self._check_backend_reachable()
        self._check_dashboard_backend_mode_visible()
        self._check_real_backend_telemetry()
        self._check_no_synthetic_localization()
        self._check_localization_confidence()
        return self._print_summary()

    def _backend_payload_or_fail(self) -> dict[str, Any] | None:
        try:
            return self._fetch_backend_payload()
        except Exception as exc:  # noqa: BLE001
            self._record(
                "backend_payload", False, f"failed to read backend fleet payload: {exc}"
            )
            return None

    def _check_real_backend_telemetry(self) -> None:
        payload = self._backend_payload_or_fail()
        if payload is None:
            return
        real_drone_count = int(payload.get("real_drone_count", 0) or 0)
        stale_drone_count = int(payload.get("stale_drone_count", 0) or 0)
        drones = payload.get("drones", [])
        if real_drone_count <= 0:
            self._record(
                "real_backend_telemetry",
                False,
                "backend is not receiving any real drone telemetry",
            )
            return
        if stale_drone_count > 0:
            self._record(
                "real_backend_telemetry",
                False,
                f"backend reports stale telemetry for {stale_drone_count} drone(s)",
            )
            return
        if not isinstance(drones, list) or not drones:
            self._record(
                "real_backend_telemetry",
                False,
                "fleet payload contains no drone records",
            )
            return
        fake_sources = []
        for item in drones:
            if not isinstance(item, dict):
                continue
            source = str(item.get("source", "real")).strip().lower() or "real"
            if source != "real":
                fake_sources.append(source)
            if bool(item.get("stale", False)):
                self._record(
                    "real_backend_telemetry",
                    False,
                    "fleet payload contains stale drone records",
                )
                return
        if fake_sources:
            self._record(
                "real_backend_telemetry",
                False,
                f"fake or non-real drone sources present: {sorted(set(fake_sources))}",
            )
            return
        self._record(
            "real_backend_telemetry",
            True,
            f"backend receiving {real_drone_count} real drone(s) with no stale telemetry",
        )

    def _check_no_synthetic_localization(self) -> None:
        payload = self._backend_payload_or_fail()
        if payload is None:
            return
        drones = payload.get("drones", [])
        if not isinstance(drones, list) or not drones:
            self._record(
                "localization_data_source",
                False,
                "no drone records available to verify localization source",
            )
            return
        bad_sources: set[str] = set()
        for item in drones:
            if not isinstance(item, dict):
                continue
            source = (
                str(item.get("localization_data_source", "unavailable")).strip().lower()
                or "unavailable"
            )
            if source != "real":
                bad_sources.add(source)
        if bad_sources:
            self._record(
                "localization_data_source",
                False,
                f"non-real localization sources active: {sorted(bad_sources)}",
            )
            return
        self._record(
            "localization_data_source",
            True,
            "all reported localization data sources are real",
        )

    def _check_localization_confidence(self) -> None:
        payload = self._backend_payload_or_fail()
        if payload is None:
            return
        drones = payload.get("drones", [])
        if not isinstance(drones, list) or not drones:
            self._record(
                "localization_confidence",
                False,
                "no drone records available to verify localization confidence",
            )
            return
        low_confidence: list[str] = []
        for item in drones:
            if not isinstance(item, dict):
                continue
            drone_id = item.get("drone_id", "?")
            confidence = float(item.get("localization_confidence", 0.0) or 0.0)
            if confidence < self.localization_confidence_threshold:
                low_confidence.append(f"drone {drone_id}={confidence:.2f}")
        if low_confidence:
            self._record(
                "localization_confidence",
                False,
                f"localization confidence below threshold {self.localization_confidence_threshold:.2f}: {', '.join(low_confidence)}",
            )
            return
        self._record(
            "localization_confidence",
            True,
            f"all drones meet localization confidence threshold {self.localization_confidence_threshold:.2f}",
        )

    def _print_summary(self) -> int:
        failures = [result for result in self.results if not result.passed]
        for result in self.results:
            status = "PASS" if result.passed else "FAIL"
            print(f"[{status}] {result.name}: {result.reason}")
        if failures:
            print()
            print("PRE-ARM VERDICT: FAIL")
            print("Do not fly unless pre-arm check passes.")
            return 1
        print()
        print("PRE-ARM VERDICT: PASS")
        print(
            "Pre-arm gate passed for tethered test prerequisites. This is still not a free-flight readiness claim."
        )
        return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Strict tethered-flight pre-arm checks"
    )
    parser.add_argument(
        "--runtime-config",
        default="config/runtime.json",
        help="Path to runtime JSON config",
    )
    parser.add_argument(
        "--backend-url",
        default="",
        help="Control-plane backend URL",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=2.5,
        help="Network timeout in seconds",
    )
    parser.add_argument(
        "--loc-confidence-threshold",
        type=float,
        default=0.65,
        help="Minimum acceptable localization confidence",
    )
    args = parser.parse_args()

    checker = PreArmChecker(
        runtime_config_path=Path(args.runtime_config).resolve(),
        backend_url=args.backend_url,
        timeout_s=max(args.timeout, 0.1),
        localization_confidence_threshold=max(
            min(args.loc_confidence_threshold, 1.0), 0.0
        ),
    )
    return checker.run()


if __name__ == "__main__":
    raise SystemExit(main())
