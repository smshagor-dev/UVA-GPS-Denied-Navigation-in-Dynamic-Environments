"""Independent offline EuRoC 6-DoF pose evaluation, not estimator training.

Publisher ground truth is read ONLY by this evaluator. The source has no
connection to the native sensor-fusion replay. Both streams must express the
same IMU-body position and body-to-world orientation; no scale is fitted.
The test suite uses synthetic trajectories, not EuRoC benchmark results.
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

from research.experiments.euroc_trajectory_evaluation import (
    _fail,
    _timestamp,
    _xyz,
    associate,
    rigid_align,
)


GT_COLUMNS_MINIMUM = 8
ESTIMATOR_COLUMNS = [
    "timestamp_ns",
    "x_m",
    "y_m",
    "z_m",
    "qw",
    "qx",
    "qy",
    "qz",
]


@dataclass(frozen=True)
class Pose:
    timestamp_ns: int
    xyz: tuple[float, float, float]
    q_wxyz: tuple[float, float, float, float]


def quaternion(fields: list[str]) -> tuple[float, float, float, float]:
    _fail(len(fields) == 4, "quaternion requires four scalar-first components")
    q = tuple(float(x) for x in fields)
    _fail(all(math.isfinite(x) for x in q), "non-finite orientation quaternion")
    _fail(abs(sum(x * x for x in q) - 1.0) <= 1e-3, "non-unit orientation quaternion")
    return q


def rotation_from_quaternion(q: tuple[float, float, float, float]) -> np.ndarray:
    """Return body-to-world rotation from scalar-first quaternion."""
    w, x, y, z = quaternion([str(v) for v in q])
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def load_ground_truth(path: Path) -> list[Pose]:
    """Original EuRoC state_groundtruth_estimate0 CSV, first eight fields."""
    poses: list[Pose] = []
    with path.open("r", newline="", encoding="utf-8-sig") as stream:
        for line, columns in enumerate(csv.reader(stream), 1):
            if not columns or columns[0].strip().startswith("#"):
                continue
            _fail(
                len(columns) >= GT_COLUMNS_MINIMUM,
                f"EuRoC ground truth row {line} requires at least eight columns",
            )
            poses.append(
                Pose(
                    _timestamp(columns[0]),
                    _xyz(columns[1:4]),
                    quaternion(columns[4:8]),
                )
            )
    validate_order(poses, "EuRoC ground truth")
    return poses


def load_estimated(path: Path) -> list[Pose]:
    """Native phase-81 full-pose CSV; never infer quaternion order or frame."""
    poses: list[Pose] = []
    with path.open("r", newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        _fail(
            reader.fieldnames == ESTIMATOR_COLUMNS,
            "estimator must have exact eight-column scalar-first 6-DoF schema",
        )
        for line, fields in enumerate(reader, 2):
            _fail(None not in fields, f"extra estimator fields in row {line}")
            _fail(
                all(fields.get(key) is not None for key in ESTIMATOR_COLUMNS),
                f"missing estimator field in row {line}",
            )
            poses.append(
                Pose(
                    _timestamp(fields["timestamp_ns"]),
                    _xyz([fields[key] for key in ("x_m", "y_m", "z_m")]),
                    quaternion([fields[key] for key in ("qw", "qx", "qy", "qz")]),
                )
            )
    validate_order(poses, "estimated trajectory")
    return poses


def validate_order(poses: list[Pose], label: str) -> None:
    _fail(bool(poses), f"empty trajectory: {label}")
    _fail(
        all(a.timestamp_ns < b.timestamp_ns for a, b in zip(poses, poses[1:])),
        f"timestamps must be strictly increasing: {label}",
    )


def rotation_error_rad(left: np.ndarray, right: np.ndarray) -> float:
    """Geodesic SO(3) error; 180 degrees remains 180, not zero."""
    relative = left.T @ right
    cosine = float(np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0))
    return math.acos(cosine)


def summarize(values: list[float], unit: str) -> dict:
    samples = np.asarray(values, dtype=np.float64)
    _fail(samples.size > 0 and np.isfinite(samples).all(), "missing finite error samples")
    return {
        f"rmse_{unit}": float(np.sqrt(np.mean(samples * samples))),
        f"mean_{unit}": float(np.mean(samples)),
        f"median_{unit}": float(np.median(samples)),
        f"p95_{unit}": float(np.percentile(samples, 95)),
        f"max_{unit}": float(np.max(samples)),
        "samples": int(samples.size),
    }


def evaluate(
    ground_truth: list[Pose],
    estimated: list[Pose],
    *,
    tolerance_ns: int = 20_000_000,
    min_pairs: int = 10,
    min_overlap_s: float = 5.0,
    relative_lag_s: float = 1.0,
    time_offset_ns: int = 0,
) -> dict:
    validate_order(ground_truth, "ground truth")
    validate_order(estimated, "estimated trajectory")
    _fail(type(tolerance_ns) is int and tolerance_ns >= 0, "invalid timestamp tolerance")
    _fail(type(time_offset_ns) is int, "time offset must be integer nanoseconds")
    _fail(type(min_pairs) is int and min_pairs >= 4, "minimum matched pairs < 4")
    _fail(
        math.isfinite(min_overlap_s)
        and min_overlap_s > 0
        and math.isfinite(relative_lag_s)
        and relative_lag_s > 0,
        "invalid minimum overlap or relative lag",
    )
    pairs = associate(ground_truth, estimated, tolerance_ns, time_offset_ns)
    _fail(
        len(pairs) >= min_pairs,
        f"only {len(pairs)} matched 6-DoF poses; minimum required {min_pairs}",
    )
    stamps = [gt.timestamp_ns for gt, _ in pairs]
    overlap_s = (stamps[-1] - stamps[0]) / 1e9
    _fail(overlap_s >= min_overlap_s, "insufficient matching time overlap")

    gt_positions = np.asarray([gt.xyz for gt, _ in pairs], dtype=np.float64)
    est_positions = np.asarray([est.xyz for _, est in pairs], dtype=np.float64)
    aligned_positions, row_rotation, translation = rigid_align(est_positions, gt_positions)
    # Existing evaluator uses row-vector estimated @ R + t = GT.
    # Body-to-world quaternion matrices use column vectors: R_align = R_row.T.
    world_alignment = row_rotation.T
    gt_rotations = [rotation_from_quaternion(gt.q_wxyz) for gt, _ in pairs]
    est_rotations = [rotation_from_quaternion(est.q_wxyz) for _, est in pairs]

    ate = np.linalg.norm(aligned_positions - gt_positions, axis=1).tolist()
    orientation = [
        math.degrees(rotation_error_rad(g, world_alignment @ e))
        for g, e in zip(gt_rotations, est_rotations)
    ]

    relative_position: list[float] = []
    relative_orientation: list[float] = []
    lag_ns = round(relative_lag_s * 1e9)
    _fail(lag_ns > 0, "relative lag rounds to zero nanoseconds")
    for index, timestamp in enumerate(stamps):
        candidate = bisect.bisect_left(stamps, timestamp + lag_ns)
        candidates = [j for j in (candidate - 1, candidate) if index < j < len(stamps)]
        if not candidates:
            continue
        later = min(candidates, key=lambda j: abs(stamps[j] - timestamp - lag_ns))
        if abs(stamps[later] - timestamp - lag_ns) > tolerance_ns:
            continue
        # Relative poses are expressed in each first pose's body frame.
        gt_relative_p = gt_rotations[index].T @ (
            gt_positions[later] - gt_positions[index]
        )
        est_relative_p = est_rotations[index].T @ (
            est_positions[later] - est_positions[index]
        )
        relative_position.append(float(np.linalg.norm(est_relative_p - gt_relative_p)))
        gt_relative_r = gt_rotations[index].T @ gt_rotations[later]
        est_relative_r = est_rotations[index].T @ est_rotations[later]
        relative_orientation.append(
            math.degrees(rotation_error_rad(gt_relative_r, est_relative_r))
        )
    _fail(relative_position, "no matched full-SE3 relative-pose pairs at requested lag")

    return {
        "scope": "offline_independent_6dof_pose_evaluation",
        "ground_truth_used_for_estimator": False,
        "publisher_archive_provenance_verified": False,
        "matched_samples": len(pairs),
        "estimator_samples": len(estimated),
        "dataset_samples": len(ground_truth),
        "match_fraction_of_estimator": len(pairs) / len(estimated),
        "overlap_duration_s": overlap_s,
        "association_tolerance_ms": tolerance_ns / 1e6,
        "time_offset_ns": time_offset_ns,
        "alignment": "full_window_SE3_rigid_no_scale_evaluation_only",
        "alignment_rotation_est_world_to_gt_world": world_alignment.tolist(),
        "alignment_translation_m": translation.tolist(),
        "ate_translation": summarize(ate, "m"),
        "absolute_orientation_error": summarize(orientation, "deg"),
        "full_se3_relative_pose_error": {
            "lag_s": relative_lag_s,
            "translation_in_first_body_frame": summarize(relative_position, "m"),
            "orientation": summarize(relative_orientation, "deg"),
        },
        "limitations": [
            "source archive and publisher provenance require independent verification",
            "poses must refer to the same calibrated IMU-body origin and q_world_body",
            "alignment is offline full-window fitted; not online estimator performance",
            "timestamps use nearest unique association with explicit fixed offset",
            "no uncertainty consistency, credible SOTA or physical multi-UAV results",
        ],
    }


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ground-truth", type=Path, required=True)
    parser.add_argument("--estimated", type=Path, required=True)
    parser.add_argument("--sequence", required=True)
    parser.add_argument(
        "--frame-agreement", choices=("same-imu-body",), required=True
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--tolerance-ms", type=float, default=20.0)
    parser.add_argument("--min-pairs", type=int, default=10)
    parser.add_argument("--min-overlap-s", type=float, default=5.0)
    parser.add_argument("--relative-lag-s", type=float, default=1.0)
    parser.add_argument("--time-offset-ns", type=int, default=0)
    args = parser.parse_args()
    _fail(bool(args.sequence.strip()), "sequence name is required")
    _fail(
        math.isfinite(args.tolerance_ms) and args.tolerance_ms >= 0,
        "invalid tolerance",
    )
    _fail(
        args.output.resolve() not in (
            args.ground_truth.resolve(),
            args.estimated.resolve(),
        ),
        "evaluation output must not overwrite either input",
    )
    _fail(
        not args.output.exists() and not args.output.is_symlink(),
        "refusing to overwrite an existing evidence file",
    )
    result = evaluate(
        load_ground_truth(args.ground_truth),
        load_estimated(args.estimated),
        tolerance_ns=round(args.tolerance_ms * 1e6),
        min_pairs=args.min_pairs,
        min_overlap_s=args.min_overlap_s,
        relative_lag_s=args.relative_lag_s,
        time_offset_ns=args.time_offset_ns,
    )
    result["sequence"] = args.sequence
    result["frame_agreement_assertion"] = args.frame_agreement
    result["source_sha256"] = {
        "ground_truth": sha256(args.ground_truth),
        "estimated": sha256(args.estimated),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(
        f"{result['matched_samples']} poses: "
        f"ATE RMSE {result['ate_translation']['rmse_m']:.6f} m; "
        f"absolute orientation RMSE "
        f"{result['absolute_orientation_error']['rmse_deg']:.4f} degrees"
    )


if __name__ == "__main__":
    main()
