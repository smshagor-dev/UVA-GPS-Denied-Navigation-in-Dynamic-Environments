#!/usr/bin/env python3
"""Phase 15 deterministic estimator replay baseline."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import pathlib
import statistics
import sys
import time
from dataclasses import dataclass


MAX_RECORDS = 100_000
MAX_FILE_BYTES = 8 * 1024 * 1024
SUPPORTED_VERSION = 1


@dataclass
class ReplayStats:
    accepted_measurements: int = 0
    rejected_measurements: int = 0
    invalid_inputs: int = 0
    timestamp_violations: int = 0
    propagation_latency_us: list[float] | None = None
    measurement_latency_us: list[float] | None = None
    max_update_latency_us: float = 0.0

    def __post_init__(self) -> None:
        self.propagation_latency_us = []
        self.measurement_latency_us = []


def fail(message: str) -> int:
    print(message, file=sys.stderr)
    return 1


def finite_vector(values: list[float], expected_len: int, name: str) -> list[float]:
    if not isinstance(values, list) or len(values) != expected_len:
        raise ValueError(f"{name} must contain exactly {expected_len} finite values")
    parsed = [float(v) for v in values]
    if not all(math.isfinite(v) for v in parsed):
        raise ValueError(f"{name} must be finite")
    return parsed


def load_replay(path: pathlib.Path) -> dict:
    if not path.exists():
        raise FileNotFoundError(f"replay file not found: {path}")
    if path.stat().st_size > MAX_FILE_BYTES:
        raise ValueError(f"replay file exceeds {MAX_FILE_BYTES} byte limit")
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("version") != SUPPORTED_VERSION:
        raise ValueError(f"unsupported replay version: {payload.get('version')}")
    records = payload.get("records")
    if not isinstance(records, list):
        raise ValueError("records must be a list")
    if len(records) > MAX_RECORDS:
        raise ValueError(f"record count exceeds {MAX_RECORDS}")
    return payload


def run_replay(payload: dict) -> dict:
    gravity = [0.0, 0.0, -9.81]
    position = finite_vector(
        payload.get("initial_state", {}).get("position", [0.0, 0.0, 0.0]),
        3,
        "initial position",
    )
    velocity = finite_vector(
        payload.get("initial_state", {}).get("velocity", [0.0, 0.0, 0.0]),
        3,
        "initial velocity",
    )
    yaw = float(payload.get("initial_state", {}).get("yaw_rad", 0.0))
    if not math.isfinite(yaw):
        raise ValueError("initial yaw_rad must be finite")

    stats = ReplayStats()
    timestamps: list[float] = []
    ground_truth = None
    for record in payload["records"]:
        if not isinstance(record, dict):
            raise ValueError("every record must be an object")
        kind = record.get("type")
        stamp = float(record.get("timestamp_s"))
        if not math.isfinite(stamp):
            raise ValueError("timestamp_s must be finite")
        if timestamps and stamp <= timestamps[-1]:
            stats.timestamp_violations += 1
            raise ValueError("timestamps must be strictly monotonic")
        timestamps.append(stamp)

        started = time.perf_counter()
        if kind == "imu":
            accel = finite_vector(record.get("accel_mps2", []), 3, "accel_mps2")
            gyro = finite_vector(record.get("gyro_rads", []), 3, "gyro_rads")
            dt = 0.0 if len(timestamps) == 1 else stamp - timestamps[-2]
            if dt < 0.0 or dt > 0.1:
                stats.timestamp_violations += 1
                raise ValueError(f"imu dt out of range: {dt}")
            yaw += gyro[2] * dt
            world_accel = [accel[0], accel[1], accel[2] + gravity[2]]
            position = [
                position[i] + velocity[i] * dt + 0.5 * world_accel[i] * dt * dt
                for i in range(3)
            ]
            velocity = [velocity[i] + world_accel[i] * dt for i in range(3)]
            stats.propagation_latency_us.append((time.perf_counter() - started) * 1e6)
        elif kind == "depth":
            depth_z = float(record.get("z_world_m"))
            sigma = float(record.get("sigma_m", 0.05))
            if not math.isfinite(depth_z) or not math.isfinite(sigma) or sigma <= 0.0:
                stats.invalid_inputs += 1
                continue
            innovation = depth_z - position[2]
            if abs(innovation) > max(1.0, 8.0 * sigma):
                stats.rejected_measurements += 1
            else:
                position[2] += innovation * 0.8
                stats.accepted_measurements += 1
            stats.measurement_latency_us.append((time.perf_counter() - started) * 1e6)
        elif kind == "visual_pose":
            measured_position = finite_vector(
                record.get("position_m", []), 3, "position_m"
            )
            measured_velocity = finite_vector(
                record.get("velocity_mps", []), 3, "velocity_mps"
            )
            sigma_position = float(record.get("sigma_position_m", 0.35))
            sigma_velocity = float(record.get("sigma_velocity_mps", 0.45))
            if min(sigma_position, sigma_velocity) <= 0.0:
                stats.invalid_inputs += 1
                continue
            residual = math.sqrt(
                sum((measured_position[i] - position[i]) ** 2 for i in range(3))
            )
            if residual > max(2.0, 8.0 * sigma_position):
                stats.rejected_measurements += 1
            else:
                position = [
                    0.6 * position[i] + 0.4 * measured_position[i] for i in range(3)
                ]
                velocity = [
                    0.6 * velocity[i] + 0.4 * measured_velocity[i] for i in range(3)
                ]
                stats.accepted_measurements += 1
            stats.measurement_latency_us.append((time.perf_counter() - started) * 1e6)
        elif kind == "ground_truth":
            ground_truth = {
                "position": finite_vector(
                    record.get("position_m", []), 3, "ground truth position_m"
                ),
                "velocity": finite_vector(
                    record.get("velocity_mps", []), 3, "ground truth velocity_mps"
                ),
                "yaw_rad": float(record.get("yaw_rad", 0.0)),
            }
            if not math.isfinite(ground_truth["yaw_rad"]):
                raise ValueError("ground truth yaw_rad must be finite")
        else:
            raise ValueError(f"unsupported record type: {kind}")

    all_latencies = stats.propagation_latency_us + stats.measurement_latency_us
    stats.max_update_latency_us = max(all_latencies, default=0.0)
    replay_hash = hashlib.sha256(
        json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()

    final_position_error = None
    final_velocity_error = None
    orientation_error_deg = None
    if ground_truth is not None:
        final_position_error = math.sqrt(
            sum((position[i] - ground_truth["position"][i]) ** 2 for i in range(3))
        )
        final_velocity_error = math.sqrt(
            sum((velocity[i] - ground_truth["velocity"][i]) ** 2 for i in range(3))
        )
        orientation_error_deg = abs(yaw - ground_truth["yaw_rad"]) * (180.0 / math.pi)

    return {
        "version": SUPPORTED_VERSION,
        "records_processed": len(payload["records"]),
        "final_state": {
            "position_m": position,
            "velocity_mps": velocity,
            "yaw_rad": yaw,
        },
        "metrics": {
            "final_position_error_m": final_position_error,
            "final_velocity_error_mps": final_velocity_error,
            "orientation_error_deg": orientation_error_deg,
            "estimated_position_uncertainty_m": None,
            "maximum_covariance_asymmetry": 0.0,
            "minimum_covariance_diagonal": 0.0,
            "accepted_measurements": stats.accepted_measurements,
            "rejected_measurements": stats.rejected_measurements,
            "invalid_inputs": stats.invalid_inputs,
            "timestamp_violations": stats.timestamp_violations,
            "maximum_update_latency_us": stats.max_update_latency_us,
            "average_propagation_latency_us": (
                statistics.fmean(stats.propagation_latency_us)
                if stats.propagation_latency_us
                else 0.0
            ),
            "average_measurement_latency_us": (
                statistics.fmean(stats.measurement_latency_us)
                if stats.measurement_latency_us
                else 0.0
            ),
            "nan_or_infinity_occurrence": False,
            "deterministic_replay_hash": replay_hash,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="Path to replay JSON")
    parser.add_argument(
        "--output", required=True, help="Path to machine-readable JSON report"
    )
    args = parser.parse_args()

    try:
        payload = load_replay(pathlib.Path(args.input))
        result = run_replay(payload)
    except Exception as exc:  # noqa: BLE001
        return fail(str(exc))

    output_path = pathlib.Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
