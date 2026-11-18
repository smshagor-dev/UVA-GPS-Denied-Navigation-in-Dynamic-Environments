"""Paired EuRoC IMU attitude-initialization ablation without publisher ground truth.

Both modes consume EXACTLY the same externally produced VO CSV and IMU CSV.
The output is an unverified estimator-initialization sensitivity study, not
an accuracy/robustness benchmark. This code never reads ground truth.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import subprocess
from pathlib import Path

from research.experiments.euroc_calibration_validation import require
from research.experiments.euroc_stereo_visual_odometry import VISUAL_COLUMNS
from research.experiments.euroc_vio_run import (
    NATIVE_COUNTS,
    TRAJECTORY_COLUMNS,
    checked_trajectory,
    digest,
)


MODES = (
    (
        "identity",
        "--assume-initial-imu-identity",
        "identity-assumed; gravity unverified",
    ),
    (
        "stationary_gravity",
        "--require-stationary-gravity-alignment",
        "stationary-gravity-aligned; yaw unobservable",
    ),
)


def visual_timestamps(path: Path) -> list[int]:
    stamps = []
    with path.open("r", newline="", encoding="utf-8-sig") as stream:
        rows = csv.DictReader(stream)
        require(
            rows.fieldnames == list(VISUAL_COLUMNS), "unsupported visual CSV schema"
        )
        for row in rows:
            require(
                None not in row and None not in row.values(), "invalid visual CSV row"
            )
            stamp = row["timestamp_ns"]
            require(
                stamp.isascii() and stamp.isdecimal(), "non-integer visual timestamp"
            )
            value = int(stamp)
            require(
                not stamps or value > stamps[-1], "visual timestamps not increasing"
            )
            require(
                all(math.isfinite(float(row[k])) for k in VISUAL_COLUMNS[1:]),
                "nonfinite visual measurement",
            )
            stamps.append(value)
            require(len(stamps) <= 500000, "visual input exceeds supported size")
    require(bool(stamps), "empty visual source")
    return stamps


def quaternion_product(a: list[float], b: list[float]) -> list[float]:
    """Scalar-first Hamilton product; matches native q_W_I orientation."""
    w, x, y, z = a
    v, i, j, k = b
    return [
        w * v - x * i - y * j - z * k,
        w * i + x * v + y * k - z * j,
        w * j - x * k + y * v + z * i,
        w * k + x * j - y * i + z * v,
    ]


def quaternion_conjugate(q: list[float]) -> list[float]:
    return [q[0], -q[1], -q[2], -q[3]]


def unit_quaternion(row: dict[str, str]) -> list[float]:
    q = [float(row[key]) for key in ("qw", "qx", "qy", "qz")]
    norm = math.sqrt(sum(x * x for x in q))
    require(math.isfinite(norm) and norm > 0, "invalid paired quaternion")
    return [x / norm for x in q]


def rotate_vector(q: list[float], point: list[float]) -> list[float]:
    return quaternion_product(
        quaternion_product(q, [0.0, *point]), quaternion_conjugate(q)
    )[1:]


def geodesic_angle(a: list[float], b: list[float]) -> float:
    # abs(dot) accounts for the quaternion q / -q gauge.
    dot = abs(sum(x * y for x, y in zip(a, b)))
    return 2.0 * math.acos(max(0.0, min(1.0, dot)))


def compare_native_streams(identity: Path, gravity: Path, expected: list[int]) -> dict:
    """Measure raw AND first-pose rigidly aligned paired differences, never accuracy.

    Each estimator has a distinct world-frame gauge. A single relative SE(3)
    transform, fixed using the FIRST pose only, removes that arbitrary frame
    difference without using any ground truth or optimizing later samples.
    Residual divergence after this alignment remains sensitivity, not error.
    """
    count = 0
    sums = {
        "raw_position": 0.0,
        "raw_orientation": 0.0,
        "aligned_position": 0.0,
        "aligned_orientation": 0.0,
    }
    peaks = {key: 0.0 for key in sums}
    alignment_rotation = None
    alignment_translation = None
    with identity.open(newline="", encoding="utf-8") as a, gravity.open(
        newline="", encoding="utf-8"
    ) as b:
        left, right = csv.DictReader(a), csv.DictReader(b)
        require(
            left.fieldnames == right.fieldnames == TRAJECTORY_COLUMNS,
            "inconsistent native pose schema",
        )
        for ra, rb in zip(left, right, strict=True):
            require(count < len(expected), "unexpected extra paired pose row")
            require(
                ra["timestamp_ns"] == rb["timestamp_ns"] == str(expected[count]),
                "paired native output timestamps differ",
            )
            xyz_a = [float(ra[k]) for k in ("x_m", "y_m", "z_m")]
            xyz_b = [float(rb[k]) for k in ("x_m", "y_m", "z_m")]
            qa, qb = unit_quaternion(ra), unit_quaternion(rb)
            if alignment_rotation is None:
                alignment_rotation = quaternion_product(
                    qa, quaternion_conjugate(qb)
                )
                mapped_first = rotate_vector(alignment_rotation, xyz_b)
                alignment_translation = [
                    x - y for x, y in zip(xyz_a, mapped_first)
                ]
            aligned_xyz_b = [
                x + y
                for x, y in zip(
                    rotate_vector(alignment_rotation, xyz_b),
                    alignment_translation,
                )
            ]
            aligned_qb = quaternion_product(alignment_rotation, qb)
            deltas = {
                "raw_position": math.dist(xyz_a, xyz_b),
                "raw_orientation": geodesic_angle(qa, qb),
                "aligned_position": math.dist(xyz_a, aligned_xyz_b),
                "aligned_orientation": geodesic_angle(qa, aligned_qb),
            }
            for key, delta in deltas.items():
                require(math.isfinite(delta), "nonfinite paired pose discrepancy")
                sums[key] += delta * delta
                peaks[key] = max(peaks[key], delta)
            count += 1
    require(count == len(expected) and count > 0, "paired native trajectory count mismatch")
    return {
        "sample_count": count,
        "position_difference_rms_m": math.sqrt(sums["raw_position"] / count),
        "position_difference_max_m": peaks["raw_position"],
        "orientation_difference_rms_rad": math.sqrt(sums["raw_orientation"] / count),
        "orientation_difference_max_rad": peaks["raw_orientation"],
        "first_pose_aligned_position_difference_rms_m": math.sqrt(
            sums["aligned_position"] / count
        ),
        "first_pose_aligned_position_difference_max_m": peaks["aligned_position"],
        "first_pose_aligned_orientation_difference_rms_rad": math.sqrt(
            sums["aligned_orientation"] / count
        ),
        "first_pose_aligned_orientation_difference_max_rad": peaks[
            "aligned_orientation"
        ],
        "alignment": "one rigid SE3 transform fixed from first paired poses, no GT",
        "is_accuracy_metric": False,
        "ground_truth_used": False,
    }

def run(
    sequence_root: Path,
    visual_csv: Path,
    native_binary: Path,
    output_dir: Path,
    *,
    timeout_s: int = 3600,
) -> dict:
    require(type(timeout_s) is int and timeout_s > 0, "invalid ablation timeout")
    source = sequence_root.resolve(strict=True)
    require(source.is_dir(), "EuRoC sequence must be a directory")
    imu = source / "mav0/imu0/data.csv"
    require(
        imu.is_file() and not imu.is_symlink() and imu.resolve().is_relative_to(source),
        "missing or symlinked original EuRoC imu0/data.csv",
    )
    visual = visual_csv.resolve(strict=True)
    require(
        visual.is_file() and not visual_csv.is_symlink(),
        "VO input must be a regular non-symlink CSV",
    )
    native = native_binary.resolve(strict=True)
    require(
        native.is_file() and os.access(native, os.X_OK),
        "missing or nonexecutable native fusion binary",
    )
    require(
        not visual.is_relative_to(source),
        "visual measurements must be generated outside the original source",
    )
    stamps = visual_timestamps(visual)
    output = output_dir.resolve()
    require(
        not output.is_relative_to(source)
        and not output.exists()
        and not output_dir.is_symlink(),
        "ablation output cannot overwrite data or reside inside source sequence",
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.mkdir(exist_ok=False)
    inputs = {
        "original_imu_sha256": digest(imu),
        "visual_csv_sha256": digest(visual),
        "native_binary_sha256": digest(native),
    }
    manifest = {
        "schema_version": 1,
        "status": "started",
        "scope": "paired_initialization_sensitivity_not_accuracy",
        "ground_truth_used": False,
        "dataset_provenance_verified": False,
        "publishable_accuracy_evidence": False,
        "input_sha256": inputs,
        "visual_sample_count": len(stamps),
        "mode_results": {},
        "limitations": [
            "both modes consume the same VO positions, velocities and quaternions",
            "stationary gravity mode is tilt-only and cannot estimate yaw",
            "no external publisher archive provenance or independent GT comparison",
            "VO correlations, fixed noise and drift remain unvalidated",
        ],
    }
    try:
        for name, flag, attestation in MODES:
            active = output / f"{name}_active.csv"
            shadow = output / f"{name}_shadow.csv"
            proc = subprocess.run(
                [
                    str(native),
                    "--imu-csv",
                    str(imu),
                    "--visual-csv",
                    str(visual),
                    "--active-output",
                    str(active),
                    "--shadow-output",
                    str(shadow),
                    flag,
                ],
                check=False,
                capture_output=True,
                text=True,
                timeout=timeout_s,
            )
            record = {
                "flag": flag,
                "exit_code": proc.returncode,
                "stderr_tail": proc.stderr[-3000:],
            }
            manifest["mode_results"][name] = record
            require(proc.returncode == 0, f"{name} native replay failed")
            require(
                f"Initial attitude prior: {attestation}" in proc.stdout,
                f"{name} native attitude attestation missing",
            )
            counter = NATIVE_COUNTS.search(proc.stdout)
            require(counter is not None, f"{name} native IMU counter missing")
            original, interpolated, updates = map(int, counter.groups())
            require(
                original >= 2 and updates == len(stamps),
                f"{name} visual update count mismatch",
            )
            record["native_counts"] = {
                "original_imu_rows": original,
                "interpolated_imu_rows": interpolated,
                "visual_updates": updates,
            }
            record["active"] = checked_trajectory(active, stamps)
            record["shadow"] = checked_trajectory(shadow, stamps)
            require(
                digest(imu) == inputs["original_imu_sha256"]
                and digest(visual) == inputs["visual_csv_sha256"]
                and digest(native) == inputs["native_binary_sha256"],
                "paired ablation input changed during experiment",
            )
        a = manifest["mode_results"]
        require(
            a["identity"]["native_counts"] == a["stationary_gravity"]["native_counts"],
            "native counters differ between paired initialization variants",
        )
        manifest["sensitivity"] = {
            "active": compare_native_streams(
                output / "identity_active.csv",
                output / "stationary_gravity_active.csv",
                stamps,
            ),
            "shadow": compare_native_streams(
                output / "identity_shadow.csv",
                output / "stationary_gravity_shadow.csv",
                stamps,
            ),
        }
        manifest["status"] = "completed_unverified_paired_software_replay"
    except Exception as error:
        manifest["status"] = "failed"
        manifest["failure_reason"] = str(error)
        raise
    finally:
        with (output / "ablation_manifest.json").open("x", encoding="utf-8") as writer:
            json.dump(manifest, writer, indent=2, sort_keys=True)
            writer.write("\n")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sequence-root", type=Path, required=True)
    parser.add_argument("--visual-csv", type=Path, required=True)
    parser.add_argument("--native-binary", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--timeout-seconds", type=int, default=3600)
    args = parser.parse_args()
    result = run(
        args.sequence_root,
        args.visual_csv,
        args.native_binary,
        args.output_dir,
        timeout_s=args.timeout_seconds,
    )
    print(
        "Paired initializer sensitivity only: "
        f"{result['visual_sample_count']} matched visual timestamps; "
        "not a EuRoC accuracy benchmark."
    )


if __name__ == "__main__":
    main()
