"""Verify a saved EuRoC VIO software evidence bundle without publisher ground truth.

This is a consistency check against self-declared hashes, NOT proof of source
authenticity or benchmark accuracy. The original EuRoC data are never opened.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

from research.experiments.euroc_visual_coverage_audit import audit_coverage, require


POSE_COLUMNS = ("timestamp_ns", "x_m", "y_m", "z_m", "qw", "qx", "qy", "qz")
POSITION_COLUMNS = POSE_COLUMNS[:4]
VISUAL_COLUMNS = (
    "timestamp_ns",
    "x_m",
    "y_m",
    "z_m",
    "vx_mps",
    "vy_mps",
    "vz_mps",
    "sigma_position_m",
    "sigma_velocity_mps",
    "pnp_inliers",
    "pnp_reprojection_rmse_px",
    "qw",
    "qx",
    "qy",
    "qz",
    "sigma_orientation_rad",
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def safe_file(root: Path, name: str) -> Path:
    """Forbid symlinks and any manifest-driven path traversal."""
    require(name and Path(name).name == name, "unsafe bundle artifact name")
    path = root / name
    require(
        path.is_file() and not path.is_symlink(),
        f"missing or symlinked artifact: {name}",
    )
    return path


def checked_json(root: Path, name: str) -> dict:
    document = json.loads(safe_file(root, name).read_text(encoding="utf-8"))
    require(isinstance(document, dict), f"invalid JSON object: {name}")
    return document


def matched_hash(root: Path, name: str, expected: str) -> None:
    require(
        isinstance(expected, str)
        and len(expected) == 64
        and sha256(safe_file(root, name)) == expected,
        f"SHA-256 mismatch for {name}",
    )


def checked_csv(
    root: Path, name: str, columns: tuple[str, ...]
) -> list[dict[str, str]]:
    with safe_file(root, name).open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        require(reader.fieldnames == list(columns), f"unsupported CSV schema: {name}")
        rows = list(reader)
    require(bool(rows), f"empty CSV: {name}")
    stamps = []
    for row in rows:
        require(
            None not in row and all(value is not None for value in row.values()),
            f"malformed CSV row: {name}",
        )
        stamp = row["timestamp_ns"]
        require(
            bool(stamp) and stamp.isascii() and stamp.isdecimal(),
            f"invalid timestamp: {name}",
        )
        timestamp = int(stamp)
        require(
            not stamps or timestamp > stamps[-1],
            f"non-increasing timestamps: {name}",
        )
        stamps.append(timestamp)
        require(
            all(math.isfinite(float(row[k])) for k in columns[1:]),
            f"nonfinite numeric CSV value: {name}",
        )
        if "qw" in columns:
            qnorm = sum(float(row[k]) ** 2 for k in ("qw", "qx", "qy", "qz"))
            require(abs(qnorm - 1.0) < 0.02, f"non-unit quaternion: {name}")
    return rows


def check_pose_artifact(
    root: Path, declared: dict, name: str, rows: list[dict]
) -> None:
    require(declared["relative_file"] == name, f"trajectory name mismatch: {name}")
    matched_hash(root, name, declared["sha256"])
    require(declared["sample_count"] == len(rows), f"trajectory count mismatch: {name}")
    if name.endswith("estimated.csv"):
        require(
            declared["first_timestamp_ns"] == int(rows[0]["timestamp_ns"])
            and declared["last_timestamp_ns"] == int(rows[-1]["timestamp_ns"]),
            f"trajectory endpoint mismatch: {name}",
        )


def verify_bundle(root: Path, *, allow_partial: bool = False) -> dict:
    root = root.resolve(strict=True)
    require(root.is_dir(), "bundle root is not a directory")
    run = checked_json(root, "run_manifest.json")
    require(
        run.get("status")
        in (
            "completed_unverified_software_replay",
            "completed_unverified_partial_software_replay",
        ),
        "bundle is not a completed software replay",
    )
    require(
        run.get("ground_truth_used") is False
        and run.get("dataset_provenance_verified") is False
        and run.get("publishable_accuracy_evidence") is False,
        "unexpected ground-truth/provenance/accuracy declaration",
    )
    require(
        run.get("native_exit_code") == 0, "native estimator exit was not successful"
    )
    required_files = (
        ("synchronization", "manifest_sha256", "synchronization_manifest.json"),
        ("synchronization", "events_sha256", "synchronized_frames.jsonl"),
        ("visual", "manifest_sha256", "visual_manifest.json"),
        ("visual", "csv_sha256", "visual_odometry.csv"),
    )
    for section, key, name in required_files:
        matched_hash(root, name, run[section][key])

    visual = checked_json(root, "visual_manifest.json")
    coverage = audit_coverage(visual)
    declared = run["visual"]
    require(
        declared["accepted_count"] == coverage["visual_updates_accepted"]
        and declared["frame_audit_count"] == visual["frame_audit_count"]
        and declared["frame_audit_sha256"] == coverage["frame_audit_sha256"]
        and declared["visual_status_counts"] == visual["visual_status_counts"]
        and declared["frame_limit_truncated"] is coverage["frame_limit_truncated"]
        and declared["tracking_failure_truncated"]
        is coverage["tracking_failure_truncated"],
        "run/visual manifest coverage disagreement",
    )
    require(
        coverage["terminal_failure_count"] == 0,
        "failed tracking cannot produce completed evidence",
    )
    partial = coverage["frame_limit_truncated"]
    require(
        run["status"]
        == (
            "completed_unverified_partial_software_replay"
            if partial
            else "completed_unverified_software_replay"
        ),
        "run completion status inconsistent with coverage",
    )
    require(
        not partial or allow_partial, "partial camera sequence requires --allow-partial"
    )

    visual_rows = checked_csv(root, "visual_odometry.csv", VISUAL_COLUMNS)
    timestamps = [row["timestamp_ns"] for row in visual_rows]
    audit_timestamps = [
        str(row["timestamp_ns"])
        for row in visual["frame_audit"]
        if row["status"] == "visual_pose"
    ]
    require(
        timestamps == audit_timestamps,
        "visual CSV timestamps differ from accepted frame audit",
    )
    require(
        len(visual_rows) == declared["accepted_count"],
        "visual CSV count differs from run manifest",
    )

    projections = []
    for estimator in ("active", "shadow"):
        name = estimator + "_estimated.csv"
        position_name = estimator + "_position_only.csv"
        pose_rows = checked_csv(root, name, POSE_COLUMNS)
        position_rows = checked_csv(root, position_name, POSITION_COLUMNS)
        check_pose_artifact(root, run["native_" + estimator], name, pose_rows)
        check_pose_artifact(
            root, run[estimator + "_evaluator_positions"], position_name, position_rows
        )
        require(
            [row["timestamp_ns"] for row in pose_rows] == timestamps,
            f"{estimator} timestamps differ from visual observations",
        )
        require(
            [{key: row[key] for key in POSITION_COLUMNS} for row in pose_rows]
            == position_rows,
            f"{estimator} position projection differs from native pose",
        )
        projections.append(run["native_" + estimator]["sha256"])
    require(
        run["native_counts"]["visual_updates"] == len(timestamps)
        and run["native_counts"]["original_imu_rows"] >= 2,
        "native estimator counter mismatch",
    )
    return {
        "schema_version": 1,
        "scope": "offline_self_attested_vio_bundle_consistency_only",
        "status": "verified_internal_consistency_not_publisher_provenance",
        "run_manifest_sha256": sha256(safe_file(root, "run_manifest.json")),
        "visual_frame_audit_sha256": coverage["frame_audit_sha256"],
        "active_trajectory_sha256": projections[0],
        "shadow_trajectory_sha256": projections[1],
        "visual_updates": len(timestamps),
        "camera_frames_total": coverage["camera_frames_total"],
        "frame_limit_truncated": partial,
        "ground_truth_used": False,
        "publisher_archive_provenance_verified": False,
        "benchmark_accuracy_established": False,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle-dir", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--allow-partial", action="store_true")
    args = parser.parse_args()
    root = args.bundle_dir.resolve(strict=True)
    destination = args.output.resolve()
    require(
        not destination.is_relative_to(root),
        "write evidence verification outside the source bundle",
    )
    require(
        not args.output.exists() and not args.output.is_symlink(),
        "refusing to overwrite verification report",
    )
    result = verify_bundle(root, allow_partial=args.allow_partial)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as writer:
        json.dump(result, writer, indent=2, sort_keys=True)
        writer.write("\n")
    print("Internal evidence consistency verified; no publisher dataset or GT checked.")


if __name__ == "__main__":
    main()
