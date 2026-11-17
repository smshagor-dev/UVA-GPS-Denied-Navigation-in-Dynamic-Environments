"""Strict EuRoC stereo/IMU event intake, NOT a VIO estimator or EuRoC result.

Never reads ground-truth files or manufactures an estimated pose. Requires the
official mav0/{imu0,cam0,cam1} unpacked layout and records provenance hashes.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


STREAMS = ("imu0", "cam0", "cam1")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def checked_file(root: Path, path: Path) -> Path:
    require(not path.is_symlink(), f"symlink is forbidden: {path}")
    resolved = path.resolve(strict=True)
    require(resolved.is_relative_to(root), f"path escapes sequence root: {path}")
    require(resolved.is_file(), f"missing regular file: {path}")
    require(resolved.stat().st_size > 0, f"empty sensor file: {path}")
    return resolved


def timestamp_ns(value: str) -> int:
    token = value.strip()
    require(bool(token) and token.isascii() and token.isdecimal(),
            f"invalid nanosecond timestamp: {value!r}")
    return int(token)


def sensor_rows(path: Path, expected_count: int) -> list[list[str]]:
    with path.open("r", newline="", encoding="utf-8-sig") as stream:
        reader = csv.reader(stream)
        header = next(reader, [])
        require(len(header) == expected_count and
                header[0].strip().startswith("#timestamp"),
                f"unexpected EuRoC sensor CSV header: {path}")
        if expected_count == 2:
            require(header[1].strip().lower() == "filename",
                    f"expected camera filename column: {path}")
        else:
            names = ("w_RS_S_x", "w_RS_S_y", "w_RS_S_z",
                     "a_RS_S_x", "a_RS_S_y", "a_RS_S_z")
            require(all(column.strip().startswith(name)
                        for column, name in zip(header[1:], names)),
                    f"unexpected IMU column ordering: {path}")
        rows = []
        last = -1
        for line, row in enumerate(reader, 2):
            require(len(row) == expected_count, f"invalid CSV row {line}: {path}")
            ts = timestamp_ns(row[0])
            require(ts > last, f"non-increasing timestamps at row {line}: {path}")
            last = ts
            rows.append(row)
        require(bool(rows), f"empty sensor CSV: {path}")
        return rows


def collect(sequence_root: Path, *, hash_images: bool = False) -> tuple[list[dict], dict]:
    root = sequence_root.resolve(strict=True)
    require(root.is_dir(), "sequence root must be a directory")
    mav = root / "mav0"
    require(mav.is_dir() and not mav.is_symlink(), "expected mav0/ directory")
    events: list[dict] = []
    sources = {}
    camera_stamps = {}

    for sensor in STREAMS:
        folder = mav / sensor
        require(folder.is_dir() and not folder.is_symlink(),
                f"missing sensor directory: {folder}")
        csv_path = checked_file(root, folder / "data.csv")
        yaml_path = checked_file(root, folder / "sensor.yaml")
        sources[sensor] = {
            "csv_sha256": sha256_file(csv_path),
            "calibration_yaml_sha256": sha256_file(yaml_path),
        }
        rows = sensor_rows(csv_path, 7 if sensor == "imu0" else 2)
        sources[sensor]["samples"] = len(rows)

        if sensor == "imu0":
            for row in rows:
                values = [float(x) for x in row[1:]]
                require(all(math.isfinite(x) for x in values),
                        "non-finite IMU data")
                events.append({
                    "timestamp_ns": timestamp_ns(row[0]),
                    "sensor": "imu0",
                    "gyro_rad_s": values[:3],
                    "accel_m_s2": values[3:],
                })
        else:
            camera_stamps[sensor] = {timestamp_ns(row[0]) for row in rows}
            for row in rows:
                name = row[1].strip()
                require(bool(name) and name == Path(name).name and
                        not name.startswith(".") and name.lower().endswith(".png"),
                        f"unsafe camera filename: {name!r}")
                path = checked_file(root, folder / "data" / name)
                event = {
                    "timestamp_ns": timestamp_ns(row[0]),
                    "sensor": sensor,
                    "image_relative_path": path.relative_to(root).as_posix(),
                }
                if hash_images:
                    event["image_sha256"] = sha256_file(path)
                events.append(event)

    events.sort(key=lambda event: (
        event["timestamp_ns"], STREAMS.index(event["sensor"])
    ))
    cam0, cam1 = camera_stamps["cam0"], camera_stamps["cam1"]
    manifest = {
        "protocol_version": 1,
        "scope": "camera_imu_intake_only_no_pose_estimation",
        "dataset_provenance_verified": False,
        "images_hashed": hash_images,
        "source_root": str(root),
        "source_hashes_and_counts": sources,
        "stereo_exact_timestamp_pairs": len(cam0 & cam1),
        "cam0_unpaired": len(cam0 - cam1),
        "cam1_unpaired": len(cam1 - cam0),
        "total_events": len(events),
        "ground_truth_used": False,
        "limitations": [
            "calibration files are hashed but their transforms are not applied",
            "PNG files are checked for presence but not decoded",
            "no visual frontend, state estimator, pose or ATE is produced",
            "official archive provenance and licensing require independent verification",
        ],
    }
    return events, manifest


def export(sequence_root: Path, events_path: Path, manifest_path: Path,
           *, hash_images: bool = False) -> dict:
    events, manifest = collect(sequence_root, hash_images=hash_images)
    root = sequence_root.resolve(strict=True)
    for output in (events_path, manifest_path):
        require(not output.resolve().is_relative_to(root),
                "output must be outside source sequence root")
        require(not output.exists(), f"refusing to overwrite: {output}")
    require(events_path.resolve() != manifest_path.resolve(),
            "events and manifest outputs must be different paths")

    payload = "".join(
        json.dumps(event, sort_keys=True, separators=(",", ":")) + "\n"
        for event in events
    ).encode("utf-8")
    manifest["event_stream_sha256"] = hashlib.sha256(payload).hexdigest()
    manifest["event_stream_format"] = "jsonl_imu0_cam0_cam1_v1"
    events_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    with events_path.open("xb") as stream:
        stream.write(payload)
    with manifest_path.open("x", encoding="utf-8") as stream:
        json.dump(manifest, stream, indent=2, sort_keys=True)
        stream.write("\n")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sequence-root", type=Path, required=True)
    parser.add_argument("--events", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--hash-images", action="store_true",
                        help="SHA256 every referenced image (slower but auditable)")
    args = parser.parse_args()
    result = export(args.sequence_root, args.events, args.manifest,
                    hash_images=args.hash_images)
    print(f"Validated {result['total_events']} raw sensor events; "
          "no pose estimation or EuRoC accuracy measurement was performed.")


if __name__ == "__main__":
    main()
