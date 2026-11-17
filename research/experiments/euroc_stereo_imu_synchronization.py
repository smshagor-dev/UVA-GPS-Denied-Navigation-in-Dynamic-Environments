"""Auditable EuRoC stereo/IMU synchronization, not pose estimation.

Consumes original, validated EuRoC camera/IMU CSV indexes and calibration.
Produces deterministic stereo event windows with original IMU samples and
explicit rejected/unpaired frame records. No ground truth is read.
"""

from __future__ import annotations

import argparse
import bisect
import hashlib
import json
from collections import Counter
from pathlib import Path

from research.experiments.euroc_calibration_validation import audit, require
from research.experiments.euroc_sensor_intake import collect


DEFAULT_MAX_GAP_NS = 25_000_000


def synchronize(
    root: Path,
    *,
    max_bracket_ns: int = DEFAULT_MAX_GAP_NS,
    max_imu_gap_ns: int = DEFAULT_MAX_GAP_NS,
    hash_images: bool = False,
) -> tuple[list[dict], dict]:
    """Build chronological, status-labeled raw sensor windows without interpolation.

    A synchronized frame is an exact-time cam0/cam1 pair supported by an
    original IMU sample on or before AND on or after its timestamp. An
    inter-frame IMU segment contains (previous accepted frame, current frame]
    samples, preserving every intervening native reading. Invalid frames are
    present in the output with an explicit status and empty replay segment.
    """
    require(type(max_bracket_ns) is int and max_bracket_ns > 0,
            "max_bracket_ns must be a positive integer")
    require(type(max_imu_gap_ns) is int and max_imu_gap_ns > 0,
            "max_imu_gap_ns must be a positive integer")
    sequence_root = root.resolve(strict=True)
    require(sequence_root.is_dir(), "sequence root must be a directory")
    measurements, intake = collect(sequence_root, hash_images=hash_images)
    calibration = audit(sequence_root)

    imu = [e for e in measurements if e["sensor"] == "imu0"]
    by_cam = {
        sensor: {
            e["timestamp_ns"]: e
            for e in measurements if e["sensor"] == sensor
        }
        for sensor in ("cam0", "cam1")
    }
    times = sorted(set(by_cam["cam0"]) | set(by_cam["cam1"]))
    imu_stamps = [e["timestamp_ns"] for e in imu]
    require(bool(times) and bool(imu_stamps), "missing stereo/IMU inputs")

    results = []
    previous_accepted_ns = None
    statuses = Counter()
    for stamp in times:
        cam0 = by_cam["cam0"].get(stamp)
        cam1 = by_cam["cam1"].get(stamp)
        frame = {
            "timestamp_ns": stamp,
            "cam0_relative_path": cam0["image_relative_path"] if cam0 else None,
            "cam1_relative_path": cam1["image_relative_path"] if cam1 else None,
            "imu_interframe": [],
            "previous_accepted_frame_ns": previous_accepted_ns,
            "status": "unavailable",
        }
        if hash_images:
            frame["source_image_sha256"] = {
                "cam0": cam0.get("image_sha256") if cam0 else None,
                "cam1": cam1.get("image_sha256") if cam1 else None,
            }
        if cam0 is None or cam1 is None:
            frame["status"] = "missing_cam0" if cam0 is None else "missing_cam1"
        else:
            before_index = bisect.bisect_right(imu_stamps, stamp) - 1
            after_index = bisect.bisect_left(imu_stamps, stamp)
            if before_index < 0:
                frame["status"] = "no_imu_before"
            elif after_index >= len(imu):
                frame["status"] = "no_imu_after"
            elif (
                stamp - imu_stamps[before_index] > max_bracket_ns
                or imu_stamps[after_index] - stamp > max_bracket_ns
            ):
                frame["status"] = "imu_bracket_gap"
            elif previous_accepted_ns is None:
                frame["status"] = "anchor"
                frame["imu_before_ns"] = imu_stamps[before_index]
                frame["imu_after_ns"] = imu_stamps[after_index]
                previous_accepted_ns = stamp
            else:
                start = bisect.bisect_right(imu_stamps, previous_accepted_ns)
                end = bisect.bisect_right(imu_stamps, stamp)
                segment = imu[start:end]
                # Require at least one IMU reading between accepted frames;
                # bound gaps from both camera frame times and between IMU samples.
                chronology = [previous_accepted_ns]
                chronology.extend(e["timestamp_ns"] for e in segment)
                chronology.append(stamp)
                if not segment:
                    frame["status"] = "no_imu_between_frames"
                elif any(
                    b - a > max_imu_gap_ns
                    for a, b in zip(chronology, chronology[1:])
                ):
                    frame["status"] = "imu_interframe_gap"
                else:
                    frame["status"] = "ready"
                    frame["imu_interframe"] = [
                        {
                            "timestamp_ns": e["timestamp_ns"],
                            "gyro_rad_s": e["gyro_rad_s"],
                            "accel_m_s2": e["accel_m_s2"],
                        }
                        for e in segment
                    ]
                    frame["imu_before_ns"] = imu_stamps[before_index]
                    frame["imu_after_ns"] = imu_stamps[after_index]
                    previous_accepted_ns = stamp

        statuses[frame["status"]] += 1
        results.append(frame)

    manifest = {
        "schema_version": 1,
        "scope": "stereo_imu_temporal_association_only_no_pose",
        "dataset_provenance_verified": False,
        "ground_truth_used": False,
        "world_pose_estimated": False,
        "camera_frames_considered": len(times),
        "status_counts": dict(sorted(statuses.items())),
        "ready_intervals": statuses["ready"],
        "initial_anchor_frames": statuses["anchor"],
        "rejected_frame_count": len(times) - statuses["ready"] - statuses["anchor"],
        "imu_sample_count": len(imu),
        "camera_sample_counts": {
            "cam0": len(by_cam["cam0"]),
            "cam1": len(by_cam["cam1"]),
        },
        "max_bracket_ns": max_bracket_ns,
        "max_imu_interframe_gap_ns": max_imu_gap_ns,
        "source_sensor_hashes": intake["source_hashes_and_counts"],
        "calibration_yaml_sha256": calibration["source_yaml_sha256"],
        "images_hashed": hash_images,
        "source_root": str(sequence_root),
        "limitations": [
            "raw sensor timestamps are aligned without interpolation",
            "calibration was validated but image undistortion/extrinsics were not applied",
            "no stereo feature matching or native estimator events were emitted",
            "dropped/unpaired frames are counted, never silently excluded",
            "no world pose, ground truth, physical UAV or accuracy evidence",
        ],
    }
    return results, manifest


def export(
    sequence_root: Path,
    events_path: Path,
    manifest_path: Path,
    *,
    max_bracket_ns: int = DEFAULT_MAX_GAP_NS,
    max_imu_gap_ns: int = DEFAULT_MAX_GAP_NS,
    hash_images: bool = False,
) -> dict:
    root = sequence_root.resolve(strict=True)
    for output in (events_path, manifest_path):
        require(not output.resolve().is_relative_to(root),
                "output must be outside the original sequence root")
        require(not output.exists() and not output.is_symlink(),
                "refusing to overwrite output")
    require(events_path.resolve() != manifest_path.resolve(),
            "event stream and manifest paths must differ")
    frames, manifest = synchronize(
        root,
        max_bracket_ns=max_bracket_ns,
        max_imu_gap_ns=max_imu_gap_ns,
        hash_images=hash_images,
    )
    payload = "".join(
        json.dumps(e, sort_keys=True, separators=(",", ":")) + "\n"
        for e in frames
    ).encode("utf-8")
    manifest["frame_stream_sha256"] = hashlib.sha256(payload).hexdigest()
    manifest["frame_stream_format"] = "jsonl_stereo_imu_windows_v1"
    events_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    with events_path.open("xb") as stream:
        stream.write(payload)
    try:
        with manifest_path.open("x", encoding="utf-8") as stream:
            json.dump(manifest, stream, indent=2, sort_keys=True)
            stream.write("\n")
    except Exception:
        events_path.unlink(missing_ok=True)
        raise
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sequence-root", type=Path, required=True)
    parser.add_argument("--events", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--max-bracket-ms", type=float, default=25.0)
    parser.add_argument("--max-imu-gap-ms", type=float, default=25.0)
    parser.add_argument("--hash-images", action="store_true")
    args = parser.parse_args()
    for value, name in (
        (args.max_bracket_ms, "max-bracket-ms"),
        (args.max_imu_gap_ms, "max-imu-gap-ms"),
    ):
        require(value > 0 and value < 10000, f"invalid {name}")
    result = export(
        args.sequence_root,
        args.events,
        args.manifest,
        max_bracket_ns=round(args.max_bracket_ms * 1e6),
        max_imu_gap_ns=round(args.max_imu_gap_ms * 1e6),
        hash_images=args.hash_images,
    )
    print(
        f"Stereo/IMU sync: {result['ready_intervals']} ready intervals, "
        f"{result['rejected_frame_count']} rejected frames. "
        "No estimator or EuRoC accuracy evaluation performed."
    )


if __name__ == "__main__":
    main()
