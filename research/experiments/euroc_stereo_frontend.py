"""Calibrated EuRoC stereo correspondence + metric triangulation, NOT pose estimation.

Requires the independently audited EuRoC sensor.yaml calibration. Never loads
ground truth or fabricates a trajectory. Reports rectified stereo landmarks in
the IMU reference frame for a *single* camera timestamp.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np

from research.experiments.euroc_calibration_validation import audit, require


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def camera_at_timestamp(root: Path, sensor: str, timestamp_ns: int) -> Path:
    """Lookup one source camera filename in its original strictly ordered CSV."""
    require(sensor in ("cam0", "cam1"), "unsupported camera source")
    folder = root / "mav0" / sensor
    csv_path = folder / "data.csv"
    require(csv_path.is_file() and not csv_path.is_symlink(),
            f"{sensor}: missing regular data.csv")
    found = None
    previous = -1
    with csv_path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.reader(stream)
        header = next(reader, [])
        require(len(header) == 2 and header[0].strip().startswith("#timestamp")
                and header[1].strip().lower() == "filename",
                f"{sensor}: unexpected camera CSV header")
        for line_number, row in enumerate(reader, 2):
            require(len(row) == 2, f"{sensor}: malformed CSV row {line_number}")
            value = row[0].strip()
            require(value.isascii() and value.isdecimal(),
                    f"{sensor}: non-integer nanosecond timestamp")
            stamp = int(value)
            require(stamp > previous,
                    f"{sensor}: non-increasing timestamp row {line_number}")
            previous = stamp
            if stamp != timestamp_ns:
                continue
            name = row[1].strip()
            require(name == Path(name).name and
                    name == f"{stamp}.png",
                    f"{sensor}: unexpected or unsafe camera image name")
            found = folder / "data" / name
    require(found is not None, f"{sensor}: timestamp not present in source CSV")
    require(found.is_file() and not found.is_symlink(),
            f"{sensor}: missing regular camera image")
    resolved = found.resolve(strict=True)
    require(resolved.is_relative_to(root),
            f"{sensor}: camera path escaped dataset root")
    return resolved


def camera_matrix(model: dict) -> np.ndarray:
    fx, fy, cx, cy = model["intrinsics_fx_fy_cx_cy"]
    return np.asarray([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
                      dtype=np.float64)


def rectification(calibration: dict) -> dict:
    cam0 = calibration["camera_models"]["cam0"]
    cam1 = calibration["camera_models"]["cam1"]
    size0 = tuple(cam0["resolution_width_height"])
    size1 = tuple(cam1["resolution_width_height"])
    require(size0 == size1, "different camera resolutions are not supported")
    K0, K1 = camera_matrix(cam0), camera_matrix(cam1)
    D0 = np.asarray(cam0["distortion_k1_k2_p1_p2"], dtype=np.float64)
    D1 = np.asarray(cam1["distortion_k1_k2_p1_p2"], dtype=np.float64)
    T10 = np.asarray(calibration["T_cam1_cam0"], dtype=np.float64)
    require(T10.shape == (4, 4), "missing camera-to-camera transform")
    # Pass contiguous 3x3/3x1 matrices; OpenCV 5 rejects some strided NumPy views.
    R = np.ascontiguousarray(T10[:3, :3], dtype=np.float64)
    t = np.ascontiguousarray(T10[:3, 3], dtype=np.float64).reshape(3, 1)
    require(np.all(np.isfinite(T10)) and np.linalg.norm(t) > 1e-5,
            "nonfinite/degenerate stereo transform")
    R0, R1, P0, P1, Q, _, _ = cv2.stereoRectify(
        K0, D0, K1, D1, size0, R, t,
        flags=cv2.CALIB_ZERO_DISPARITY, alpha=0,
    )
    return {
        "image_size": size0,
        "K0": K0,
        "K1": K1,
        "D0": D0,
        "D1": D1,
        "R0": R0,
        "R1": R1,
        "P0": P0,
        "P1": P1,
        "Q": Q,
    }


def triangulate_correspondences(
    rect: dict, points0: np.ndarray, points1: np.ndarray,
    *, vertical_tolerance_px: float = 1.5,
    min_depth_m: float = 0.1, max_depth_m: float = 60.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Return accepted point indices and rectified cam0 3-D points (meters).

    Only positive-depth points satisfying rectified stereo epipolar geometry.
    The P0/P1 projection matrices supply metric baseline; no GT is used.
    """
    a = np.asarray(points0, dtype=np.float64)
    b = np.asarray(points1, dtype=np.float64)
    require(a.ndim == 2 and a.shape[1] == 2 and
            a.shape == b.shape and a.shape[0] > 0,
            "expected matched Nx2 stereo points")
    require(np.all(np.isfinite(a)) and np.all(np.isfinite(b)),
            "nonfinite stereo coordinates")
    require(0 < vertical_tolerance_px <= 8.0 and
            0 < min_depth_m < max_depth_m,
            "invalid stereo geometry thresholds")
    candidate = np.flatnonzero(
        np.abs(a[:, 1] - b[:, 1]) <= vertical_tolerance_px
    )
    if candidate.size == 0:
        return candidate, np.empty((0, 3), dtype=np.float64)
    projected = cv2.triangulatePoints(
        rect["P0"], rect["P1"], a[candidate].T, b[candidate].T
    )
    w = projected[3]
    good_w = np.isfinite(w) & (np.abs(w) > 1e-9)
    points = np.full((candidate.size, 3), np.nan, dtype=np.float64)
    points[good_w] = (projected[:3, good_w] / w[good_w]).T
    finite = np.all(np.isfinite(points), axis=1)
    valid = finite & (points[:, 2] >= min_depth_m) & (
        points[:, 2] <= max_depth_m
    )
    # Require positive camera1 depth, not just the rectified camera0 depth.
    transformed = rect["P1"] @ np.vstack((
        points[valid].T, np.ones((int(valid.sum()),), dtype=np.float64)
    ))
    if transformed.shape[1]:
        positive_second_camera = np.isfinite(transformed[2]) & (
            transformed[2] > 0.0
        )
        valid[np.flatnonzero(valid)] = positive_second_camera
    return candidate[valid], points[valid]


def find_stereo_landmarks(
    image0: np.ndarray, image1: np.ndarray, rect: dict,
    *, max_features: int = 2200,
) -> dict:
    require(max_features >= 50 and max_features <= 10000,
            "feature budget outside supported range")
    expected_width, expected_height = rect["image_size"]
    for label, image in (("cam0", image0), ("cam1", image1)):
        require(image is not None and image.ndim == 2 and
                image.dtype == np.uint8 and
                image.shape == (expected_height, expected_width),
                f"{label}: camera PNG missing or wrong pixel dimensions/type")
    map0x, map0y = cv2.initUndistortRectifyMap(
        rect["K0"], rect["D0"], rect["R0"], rect["P0"],
        rect["image_size"], cv2.CV_16SC2
    )
    map1x, map1y = cv2.initUndistortRectifyMap(
        rect["K1"], rect["D1"], rect["R1"], rect["P1"],
        rect["image_size"], cv2.CV_16SC2
    )
    left = cv2.remap(image0, map0x, map0y, cv2.INTER_LINEAR)
    right = cv2.remap(image1, map1x, map1y, cv2.INTER_LINEAR)
    orb = cv2.ORB_create(nfeatures=max_features)
    k0, d0 = orb.detectAndCompute(left, None)
    k1, d1 = orb.detectAndCompute(right, None)
    require(d0 is not None and d1 is not None and
            len(k0) >= 8 and len(k1) >= 8,
            "insufficient stereo image texture")
    matches = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(d0, d1, k=2)
    pairs = []
    used_right = set()
    for options in sorted(matches, key=lambda pair: pair[0].distance
                          if len(pair) >= 2 else float("inf")):
        if len(options) != 2:
            continue
        first, second = options
        if (first.distance >= 0.75 * second.distance or
                first.trainIdx in used_right):
            continue
        used_right.add(first.trainIdx)
        pairs.append((k0[first.queryIdx].pt, k1[first.trainIdx].pt))
    require(len(pairs) >= 8, "insufficient ratio-tested stereo correspondences")
    points0 = np.asarray([pair[0] for pair in pairs], dtype=np.float64)
    points1 = np.asarray([pair[1] for pair in pairs], dtype=np.float64)
    selected, points_rectified = triangulate_correspondences(
        rect, points0, points1
    )
    require(selected.size >= 8,
            "fewer than eight geometrically valid stereo landmarks")
    depth = points_rectified[:, 2]
    return {
        "rectified_left_points": points0[selected],
        "rectified_right_points": points1[selected],
        "rectified_points_m": points_rectified,
        "ratio_tested_matches": len(pairs),
        "geometrically_valid_points": int(selected.size),
        "median_depth_rectified_cam0_m": float(np.median(depth)),
    }


def evaluate_pair(
    root: Path, timestamp_ns: int, *, max_features: int = 2200,
    include_landmarks: bool = False,
) -> dict:
    require(timestamp_ns >= 0, "negative timestamp")
    root = root.resolve(strict=True)
    calibration = audit(root)
    rect = rectification(calibration)
    path0 = camera_at_timestamp(root, "cam0", timestamp_ns)
    path1 = camera_at_timestamp(root, "cam1", timestamp_ns)
    # Each image hash ties evidence to source bytes; no source images are modified.
    images = {
        "cam0": sha256(path0),
        "cam1": sha256(path1),
    }
    im0 = cv2.imread(str(path0), cv2.IMREAD_GRAYSCALE)
    im1 = cv2.imread(str(path1), cv2.IMREAD_GRAYSCALE)
    landmarks = find_stereo_landmarks(
        im0, im1, rect, max_features=max_features
    )
    pts_c0 = landmarks["rectified_points_m"] @ rect["R0"]
    T_i_c0 = np.asarray(calibration["T_imu_cam0"], dtype=np.float64)
    pts_i = (pts_c0 @ T_i_c0[:3, :3].T) + T_i_c0[:3, 3]
    require(np.all(np.isfinite(pts_i)), "nonfinite IMU-frame landmarks")
    result = {
        "schema_version": 1,
        "scope": "single_stereo_timestamp_metric_triangulation_only",
        "timestamp_ns": timestamp_ns,
        "source_image_sha256": images,
        "calibration_yaml_sha256": calibration["source_yaml_sha256"],
        "dataset_provenance_verified": False,
        "ground_truth_used": False,
        "world_pose_estimated": False,
        "camera_metric_points": landmarks["geometrically_valid_points"],
        "ratio_tested_matches": landmarks["ratio_tested_matches"],
        "median_depth_rectified_cam0_m":
            landmarks["median_depth_rectified_cam0_m"],
        "point_frame": "IMU",
        "limitations": [
            "stereo landmarks at one timestamp are not a tracked map or a trajectory",
            "ORB matching and epipolar gating do not prove correct 3-D matches",
            "no temporal tracking, VIO fusion, loop closure or world-frame pose",
            "publisher archive provenance must be checked independently",
            "not multi-UAV flight or physical collision-safety evidence",
        ],
    }
    if include_landmarks:
        result["landmarks"] = [
            {
                "rectified_cam0_px": [float(v) for v in left],
                "rectified_cam1_px": [float(v) for v in right],
                "position_imu_m": [float(v) for v in p],
            }
            for left, right, p in zip(
                landmarks["rectified_left_points"],
                landmarks["rectified_right_points"],
                pts_i,
            )
        ]
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sequence-root", required=True, type=Path)
    parser.add_argument("--timestamp-ns", required=True, type=int)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--max-features", type=int, default=2200)
    parser.add_argument("--include-landmarks", action="store_true")
    args = parser.parse_args()
    root = args.sequence_root.resolve(strict=True)
    output = args.output
    require(not output.resolve().is_relative_to(root),
            "output cannot be inside original dataset")
    require(not output.exists() and not output.is_symlink(),
            "refusing to overwrite existing output")
    result = evaluate_pair(
        root, args.timestamp_ns, max_features=args.max_features,
        include_landmarks=args.include_landmarks,
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(f"Validated {result['camera_metric_points']} stereo correspondences "
          "at one timestamp; no world pose/trajectory was estimated.")


if __name__ == "__main__":
    main()
