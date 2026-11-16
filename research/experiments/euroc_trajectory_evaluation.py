"""Auditable EuRoC trajectory evaluation; no generated or claimed real-world data.

Inputs: actual EuRoC `mav0/state_groundtruth_estimate0/data.csv` and an
independently produced estimator trajectory in timestamp_ns,x_m,y_m,z_m CSV.
This tool never runs the estimator. Aligns positions by rigid SE(3) (no scale)
and reports translation ATE plus world-frame relative displacement error.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class Position:
    timestamp_ns: int
    xyz: tuple[float, float, float]


def _fail(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def _timestamp(value: str) -> int:
    text = value.strip()
    _fail(bool(text) and text.isdecimal(), f"invalid integer nanosecond timestamp {value!r}")
    return int(text)


def _xyz(fields: list[str]) -> tuple[float, float, float]:
    _fail(len(fields) == 3, "position must have exactly 3 components")
    values = tuple(float(x) for x in fields)
    _fail(all(math.isfinite(x) for x in values), "non-finite position")
    return values


def _enforce_order(rows: list[Position], source: str) -> list[Position]:
    _fail(bool(rows), f"empty trajectory: {source}")
    _fail(all(a.timestamp_ns < b.timestamp_ns for a, b in zip(rows, rows[1:])),
          f"timestamps must be strictly increasing: {source}")
    return rows


def load_euroc_ground_truth(path: Path) -> list[Position]:
    """Read timestamp[ns], p_x,y,z, q_w,x,y,z, ... (ASL EuRoC GT CSV)."""
    rows: list[Position] = []
    with path.open(encoding="utf-8-sig", newline="") as stream:
        for line, columns in enumerate(csv.reader(stream), 1):
            if not columns or columns[0].strip().startswith("#"):
                continue
            _fail(len(columns) >= 8, f"EuRoC row {line} requires >= 8 columns")
            time_ns = _timestamp(columns[0])
            pos = _xyz(columns[1:4])
            quaternion = tuple(float(x) for x in columns[4:8])
            _fail(all(math.isfinite(x) for x in quaternion) and
                  abs(sum(x*x for x in quaternion) - 1.0) <= 0.1,
                  f"malformed EuRoC ground-truth orientation in row {line}")
            rows.append(Position(time_ns, pos))
    return _enforce_order(rows, str(path))


def load_estimator_trajectory(path: Path) -> list[Position]:
    rows: list[Position] = []
    required = ["timestamp_ns", "x_m", "y_m", "z_m"]
    with path.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        _fail(reader.fieldnames == required,
              f"estimator CSV columns must be {required}; no implicit frame conversion")
        for line, row in enumerate(reader, 2):
            _fail(None not in row, f"extra estimator columns in row {line}")
            rows.append(Position(_timestamp(row["timestamp_ns"]),
                                 _xyz([row["x_m"], row["y_m"], row["z_m"]])))
    return _enforce_order(rows, str(path))


def associate(ground_truth: list[Position], estimated: list[Position],
              tolerance_ns: int, time_offset_ns: int = 0) -> list[tuple[Position, Position]]:
    """One-to-one nearest-time association; do not interpolate or reuse ground truth."""
    _fail(tolerance_ns >= 0, "negative association tolerance")
    timestamps = [pose.timestamp_ns for pose in ground_truth]
    matches: list[tuple[Position, Position]] = []
    last_index = -1
    for pose in estimated:
        target_ns = pose.timestamp_ns + time_offset_ns
        index = bisect.bisect_left(timestamps, target_ns)
        candidates = [j for j in (index - 1, index) if last_index < j < len(timestamps)]
        if not candidates:
            continue
        nearest = min(candidates, key=lambda j: (abs(timestamps[j] - target_ns), j))
        if abs(timestamps[nearest] - target_ns) <= tolerance_ns:
            matches.append((ground_truth[nearest], pose))
            last_index = nearest
    return matches


def rigid_align(estimated: np.ndarray, ground_truth: np.ndarray
                ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Fit a rigid 3D alignment, estimated @ rotation + translation = GT.

    No similarity scale is optimized; translation ATE cannot hide scale drift.
    """
    _fail(estimated.shape == ground_truth.shape and estimated.ndim == 2 and
          estimated.shape[0] >= 4 and estimated.shape[1] == 3,
          "rigid alignment requires >= 4 paired 3D points")
    source_mean, target_mean = estimated.mean(axis=0), ground_truth.mean(axis=0)
    source_centered = estimated - source_mean
    target_centered = ground_truth - target_mean
    u, singular, vt = np.linalg.svd(source_centered.T @ target_centered)
    _fail(singular[1] > 1.0e-8, "degenerate geometry: no unique 3D alignment")
    rotation = u @ np.diag([1.0, 1.0, float(np.linalg.det(u @ vt) >= 0.0) * 2 - 1]) @ vt
    translation = target_mean - source_mean @ rotation
    aligned = estimated @ rotation + translation
    _fail(np.all(np.isfinite(aligned)), "nonfinite alignment")
    return aligned, rotation, translation


def metrics(values: np.ndarray) -> dict:
    _fail(values.ndim == 1 and values.size > 0 and np.all(np.isfinite(values)),
          "no valid finite error samples")
    return {"rmse_m": float(np.sqrt(np.mean(values**2))),
            "mean_m": float(np.mean(values)),
            "median_m": float(np.median(values)),
            "p95_m": float(np.percentile(values, 95)),
            "max_m": float(np.max(values)),
            "samples": int(values.size)}


def evaluate(ground_truth: list[Position], estimated: list[Position],
             *, tolerance_ns: int = 20_000_000, min_pairs: int = 10,
             min_overlap_s: float = 5.0, relative_lag_s: float = 1.0,
             time_offset_ns: int = 0) -> dict:
    _fail(relative_lag_s > 0 and math.isfinite(relative_lag_s) and
          min_overlap_s > 0 and math.isfinite(min_overlap_s),
          "invalid time/duration parameter")
    _fail(min_pairs >= 4, "minimum matched pairs must be at least four")
    paired = associate(ground_truth, estimated, tolerance_ns, time_offset_ns)
    _fail(len(paired) >= min_pairs,
          f"only {len(paired)} matched samples; minimum required {min_pairs}")
    t0, t1 = paired[0][0].timestamp_ns, paired[-1][0].timestamp_ns
    overlap_s = (t1 - t0) / 1e9
    _fail(overlap_s >= min_overlap_s, "insufficient time overlap")
    gt = np.asarray([p[0].xyz for p in paired], dtype=float)
    est = np.asarray([p[1].xyz for p in paired], dtype=float)
    aligned, rotation, translation = rigid_align(est, gt)
    errors = np.linalg.norm(aligned - gt, axis=1)
    stamps = [p[0].timestamp_ns for p in paired]
    relative_lag_ns = round(relative_lag_s * 1e9)
    relative_errors = []
    for index, start in enumerate(stamps):
        later = bisect.bisect_left(stamps, start + relative_lag_ns)
        options = [j for j in (later - 1, later) if j > index and j < len(stamps)]
        if not options:
            continue
        closest = min(options, key=lambda j: abs(stamps[j] - start - relative_lag_ns))
        if abs(stamps[closest] - start - relative_lag_ns) > tolerance_ns:
            continue
        relative_errors.append(float(np.linalg.norm(
            (aligned[closest] - aligned[index]) - (gt[closest] - gt[index]))))
    _fail(bool(relative_errors), "no fixed-lag relative-displacement pairs")
    return {
        "scope": "position_trajectory_comparison_only",
        "provenance_verified": False,
        "dataset_samples": len(ground_truth),
        "estimator_samples": len(estimated),
        "matched_samples": len(paired),
        "match_fraction_of_estimator": len(paired) / len(estimated),
        "overlap_duration_s": overlap_s,
        "association_tolerance_ms": tolerance_ns / 1e6,
        "time_offset_ns": time_offset_ns,
        "alignment": "SE3_rigid_no_scale",
        "ate_translation": metrics(errors),
        "world_frame_relative_displacement_error": {
            **metrics(np.asarray(relative_errors, dtype=float)),
            "lag_seconds": relative_lag_s,
            "note": "Aligned world-frame displacement; not full SE(3) relative pose error"
        },
        "alignment_rotation": rotation.tolist(),
        "alignment_translation_m": translation.tolist(),
        "limitations": [
            "requires_real_estimator_export_and_original_dataset_provenance",
            "trajectory_origin_must_match_IMU_body_or_be_calibrated_externally",
            "no_orientation_error_or_covariance_consistency_assessed",
            "offline_fit_uses_all_matched_pairs; not_online_navigation_accuracy",
            "no_physical_multi_UAV_or_RF_evidence"
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ground-truth", type=Path, required=True)
    parser.add_argument("--estimated", type=Path, required=True)
    parser.add_argument("--sequence", required=True, help="Source EuRoC sequence identifier")
    parser.add_argument("--frame-agreement", choices=("same-imu-body",), required=True,
                        help="Explicitly attest estimated and GT positions use the same body origin")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--tolerance-ms", type=float, default=20.0)
    parser.add_argument("--min-pairs", type=int, default=10)
    parser.add_argument("--min-overlap-s", type=float, default=5.0)
    parser.add_argument("--relative-lag-s", type=float, default=1.0)
    parser.add_argument("--time-offset-ns", type=int, default=0)
    args = parser.parse_args()
    _fail(args.sequence.strip() != "", "sequence identifier cannot be blank")
    _fail(math.isfinite(args.tolerance_ms) and args.tolerance_ms >= 0,
          "invalid timestamp tolerance")
    result = evaluate(
        load_euroc_ground_truth(args.ground_truth),
        load_estimator_trajectory(args.estimated),
        tolerance_ns=round(args.tolerance_ms * 1e6),
        min_pairs=args.min_pairs,
        min_overlap_s=args.min_overlap_s,
        relative_lag_s=args.relative_lag_s,
        time_offset_ns=args.time_offset_ns,
    )
    result["sequence"] = args.sequence
    result["frame_agreement_assertion"] = args.frame_agreement
    result["source_sha256"] = {
        "ground_truth": hashlib.sha256(args.ground_truth.read_bytes()).hexdigest(),
        "estimated": hashlib.sha256(args.estimated.read_bytes()).hexdigest(),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n",
                           encoding="utf-8")
    print(f"Evaluated {result['matched_samples']} paired positions: "
          f"ATE RMSE {result['ate_translation']['rmse_m']:.6f} m")


if __name__ == "__main__":
    main()
