"""Fail-closed EuRoC VIO experiment runner with explicit provenance boundaries.

The independent EuRoC ground-truth files are never read. This produces
unverified experimental VIO outputs, NOT publishable accuracy measurements.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import re
import subprocess
from pathlib import Path

from research.experiments.euroc_calibration_validation import require
from research.experiments.euroc_stereo_imu_synchronization import (
    export as export_sync,
)
from research.experiments.euroc_stereo_visual_odometry import (
    export as export_visual,
)


TRAJECTORY_COLUMNS = ["timestamp_ns", "x_m", "y_m", "z_m", "qw", "qx", "qy", "qz"]
NATIVE_COUNTS = re.compile(
    r"IMU: (\d+) original rows; (\d+) interpolated camera-time rows; "
    r"(\d+) visual updates accepted by both filters\."
)


def digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as reader:
        for block in iter(lambda: reader.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def checked_trajectory(path: Path, expected_stamps: list[int]) -> dict:
    require(
        path.is_file() and not path.is_symlink(),
        f"missing or symlinked native trajectory: {path}",
    )
    timestamps = []
    with path.open("r", newline="", encoding="utf-8-sig") as reader:
        rows = csv.DictReader(reader)
        require(
            rows.fieldnames == TRAJECTORY_COLUMNS,
            "native trajectory columns must match independent evaluator",
        )
        for row in rows:
            require(None not in row, "extra native trajectory columns")
            stamp = row["timestamp_ns"]
            require(
                bool(stamp) and stamp.isascii() and stamp.isdecimal(),
                "non-integer native trajectory timestamp",
            )
            timestamps.append(int(stamp))
            require(
                all(math.isfinite(float(row[name])) for name in TRAJECTORY_COLUMNS[1:]),
                "nonfinite native pose",
            )
            qnorm = sum(float(row[k]) ** 2 for k in ("qw", "qx", "qy", "qz"))
            require(abs(qnorm - 1.0) < 0.02, "non-unit native fused attitude")
    require(
        timestamps == expected_stamps,
        "native trajectory lost, duplicated or reordered visual timestamps",
    )
    return {
        "sample_count": len(timestamps),
        "first_timestamp_ns": timestamps[0],
        "last_timestamp_ns": timestamps[-1],
        "sha256": digest(path),
        "relative_file": path.name,
    }


def export_evaluator_positions(pose_csv: Path, position_csv: Path) -> dict:
    """Lossless position columns for the independent legacy EuRoC ATE/RPE evaluator.

    Orientation remains in the full 6-DoF trajectory, never inserted into
    source/ground truth files, and is not evaluated by position-only ATE.
    """
    require(not position_csv.exists() and not position_csv.is_symlink(),
            "refusing to overwrite position-only evaluator projection")
    with pose_csv.open("r", newline="", encoding="utf-8") as source:
        reader = csv.DictReader(source)
        require(reader.fieldnames == TRAJECTORY_COLUMNS,
                "full pose schema must be validated before projection")
        with position_csv.open("x", newline="", encoding="utf-8") as target:
            writer = csv.DictWriter(
                target, fieldnames=["timestamp_ns", "x_m", "y_m", "z_m"]
            )
            writer.writeheader()
            count = 0
            for row in reader:
                writer.writerow({
                    key: row[key] for key in ("timestamp_ns", "x_m", "y_m", "z_m")
                })
                count += 1
    return {"relative_file": position_csv.name, "sample_count": count,
            "sha256": digest(position_csv)}


def run(
    sequence_root: Path,
    output_dir: Path,
    native_binary: Path,
    *,
    max_frames: int = 0,
    min_inliers: int = 12,
    timeout_s: int = 3600,
) -> dict:
    require(type(max_frames) is int and max_frames >= 0, "invalid frame limit")
    require(
        type(min_inliers) is int and 8 <= min_inliers <= 1000, "invalid PnP inlier gate"
    )
    require(
        type(timeout_s) is int and timeout_s > 0,
        "timeout must be positive integer seconds",
    )
    source = sequence_root.resolve(strict=True)
    require(source.is_dir(), "EuRoC source must be a directory")
    native = native_binary.resolve(strict=True)
    require(
        native.is_file() and os.access(native, os.X_OK),
        "native C++ fusion executable not found or not executable",
    )
    destination = output_dir.resolve()
    require(
        not destination.is_relative_to(source),
        "output directory must remain outside the original EuRoC source",
    )
    require(
        not destination.exists() and not output_dir.is_symlink(),
        "refusing to overwrite an existing experiment output directory",
    )
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.mkdir(exist_ok=False)

    manifest = {
        "schema_version": 1,
        "status": "started",
        "scope": "stereo_visual_inertial_software_replay_not_benchmark",
        "source_root": str(source),
        "native_executable_sha256": digest(native),
        "ground_truth_used": False,
        "dataset_provenance_verified": False,
        "publishable_accuracy_evidence": False,
        "full_6dof_orientation_fused": True,
        "visual_orientation_uncertainty_empirically_calibrated": False,
        "identity_initial_imu_orientation": True,
        "frame_limit": max_frames,
        "limitations": [
            "no publisher dataset archive provenance verified",
            "no independent ground-truth ATE/RPE or SOTA comparison",
            "sequential stereo PnP attitude is fused but is temporally correlated",
            "native EKF/ESKF initial attitude is unverified identity",
            "partial camera trajectory fails closed by default",
        ],
    }
    manifest_path = destination / "run_manifest.json"
    try:
        sync_path = destination / "synchronized_frames.jsonl"
        sync_manifest_path = destination / "synchronization_manifest.json"
        sync_info = export_sync(source, sync_path, sync_manifest_path, hash_images=True)
        manifest["synchronization"] = {
            "manifest_sha256": digest(sync_manifest_path),
            "events_sha256": digest(sync_path),
            "ready_intervals": sync_info["ready_intervals"],
            "rejected_frame_count": sync_info["rejected_frame_count"],
            "status_counts": sync_info["status_counts"],
        }
        visual_path = destination / "visual_odometry.csv"
        visual_manifest_path = destination / "visual_manifest.json"
        visual_info = export_visual(
            source,
            visual_path,
            visual_manifest_path,
            max_frames=max_frames,
            min_inliers=min_inliers,
        )
        require(
            sync_info["source_sensor_hashes"] == visual_info["camera_source_hashes"],
            "sensor source digests changed between synchronization and visual extraction",
        )
        require(
            sync_info["calibration_yaml_sha256"] == visual_info["calibration_sha256"],
            "calibration file digests changed between synchronization and visual extraction",
        )
        manifest["visual"] = {
            "manifest_sha256": digest(visual_manifest_path),
            "csv_sha256": digest(visual_path),
            "accepted_count": visual_info["visual_measurement_count"],
            "frame_limit_truncated": visual_info["frame_limit_truncated"],
            "tracking_failure_truncated": visual_info["tracking_failure_truncated"],
            "visual_status_counts": visual_info["visual_status_counts"],
            "frame_audit_count": visual_info["frame_audit_count"],
            "frame_audit_sha256": visual_info["frame_audit_sha256"],
        }
        with visual_path.open("r", newline="", encoding="utf-8") as reader:
            rows = list(csv.DictReader(reader))
        expected_stamps = [int(row["timestamp_ns"]) for row in rows]
        require(
            len(expected_stamps) == visual_info["visual_measurement_count"],
            "inconsistent visual rows and manifest",
        )
        ledger = visual_info["frame_audit"]
        require(
            len(ledger)
            == visual_info["frame_audit_count"]
            == visual_info["camera_timestamps_examined"],
            "frame audit count differs from examined camera timestamps",
        )
        require(
            not max_frames or len(ledger) <= max_frames,
            "examined frames exceed explicitly requested frame limit",
        )
        require(
            not visual_info["failed_closed_on_tracking_loss"],
            "refusing to classify failed stereo tracking as successful replay",
        )
        audit_stamps = [row["timestamp_ns"] for row in ledger]
        require(
            all(isinstance(value, int) for value in audit_stamps)
            and all(a < b for a, b in zip(audit_stamps, audit_stamps[1:])),
            "frame audit timestamps must be strictly increasing",
        )
        audit_visual_stamps = [
            row["timestamp_ns"] for row in ledger if row["status"] == "visual_pose"
        ]
        require(
            expected_stamps == audit_visual_stamps,
            "visual CSV timestamps do not match accepted frame audit",
        )
        require(
            all(
                row["status"]
                in (
                    "anchor",
                    "visual_pose",
                    "failed_stereo_frontend",
                    "failed_temporal_gap",
                    "failed_temporal_tracking",
                    "failed_speed_gate",
                    "failed_sync_discontinuity",
                )
                or row["status"].startswith("sync_")
                for row in ledger
            ),
            "unknown visual frame audit status",
        )
        require(
            hashlib.sha256(
                json.dumps(ledger, sort_keys=True, separators=(",", ":")).encode()
            ).hexdigest()
            == visual_info["frame_audit_sha256"],
            "visual frame audit hash mismatch",
        )

        active_path = destination / "active_estimated.csv"
        shadow_path = destination / "shadow_estimated.csv"
        process = subprocess.run(
            [
                str(native),
                "--imu-csv",
                str(source / "mav0/imu0/data.csv"),
                "--visual-csv",
                str(visual_path),
                "--active-output",
                str(active_path),
                "--shadow-output",
                str(shadow_path),
                "--assume-initial-imu-identity",
            ],
            capture_output=True,
            text=True,
            check=False,
            timeout=timeout_s,
        )
        manifest["native_exit_code"] = process.returncode
        manifest["native_stdout"] = process.stdout[-4000:]
        manifest["native_stderr"] = process.stderr[-4000:]
        require(
            process.returncode == 0,
            "native C++ estimator fusion failed; inspect run_manifest.json",
        )
        matched = NATIVE_COUNTS.search(process.stdout)
        require(matched is not None, "native fusion counter contract missing")
        originals, interpolated, updates = map(int, matched.groups())
        require(
            updates == len(expected_stamps) and originals >= 2,
            "native update count or original IMU count incorrect",
        )
        manifest["native_counts"] = {
            "original_imu_rows": originals,
            "interpolated_camera_time_imu_rows": interpolated,
            "visual_updates": updates,
        }
        manifest["native_active"] = checked_trajectory(active_path, expected_stamps)
        manifest["native_shadow"] = checked_trajectory(shadow_path, expected_stamps)
        manifest["active_evaluator_positions"] = export_evaluator_positions(
            active_path, destination / "active_position_only.csv"
        )
        manifest["shadow_evaluator_positions"] = export_evaluator_positions(
            shadow_path, destination / "shadow_position_only.csv"
        )
        manifest["status"] = (
            "completed_unverified_partial_software_replay"
            if visual_info["frame_limit_truncated"]
            else "completed_unverified_software_replay"
        )
    except Exception as error:
        manifest["status"] = "failed"
        manifest["failure_reason"] = str(error)
        raise
    finally:
        # Even a failed run has a permanent manifest; never claim success
        # based on a partially written active/shadow trajectory.
        with manifest_path.open("x", encoding="utf-8") as writer:
            json.dump(manifest, writer, indent=2, sort_keys=True)
            writer.write("\n")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sequence-root", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--native-binary", required=True, type=Path)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--min-inliers", type=int, default=12)
    parser.add_argument("--timeout-seconds", type=int, default=3600)
    args = parser.parse_args()
    result = run(
        args.sequence_root,
        args.output_dir,
        args.native_binary,
        max_frames=args.max_frames,
        min_inliers=args.min_inliers,
        timeout_s=args.timeout_seconds,
    )
    print(
        "Completed unverified software VIO replay with "
        f"{result['native_counts']['visual_updates']} visual updates; "
        "no ground truth and no benchmark accuracy validation."
    )


if __name__ == "__main__":
    main()
