"""Ground-truth-free stereo visual odometry measurements for native VIO replay.

Uses validated EuRoC calibration, exact stereo/IMU time windows, stereo
triangulation and temporal optical flow/PnP. These are fallible **visual
odometry measurements**, NOT independently verified navigation results.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

from research.experiments.euroc_calibration_validation import audit, require
from research.experiments.euroc_stereo_frontend import (
    find_stereo_landmarks,
    rectification,
)
from research.experiments.euroc_stereo_imu_synchronization import synchronize


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


def make_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    r = np.asarray(rotation, dtype=np.float64)
    t = np.asarray(translation, dtype=np.float64).reshape(-1)
    require(
        r.shape == (3, 3) and t.shape == (3,), "rigid transform dimensions incorrect"
    )
    require(
        np.all(np.isfinite(r)) and np.all(np.isfinite(t)), "nonfinite rigid transform"
    )
    require(
        np.linalg.norm(r.T @ r - np.eye(3)) < 1e-3
        and abs(float(np.linalg.det(r)) - 1.0) < 1e-3,
        "non-rigid or reflected camera transform",
    )
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = r
    result[:3, 3] = t
    return result


# Experimental conditioning gate for the generic EPNP solver, not a
# validated physical accuracy threshold or a substitute for original EuRoC.
MIN_PNP_3D_INLIER_SINGULAR_RATIO = 0.005


def metric_pnp_spatial_conditioning(points_m: np.ndarray) -> float:
    """Reject near-coplanar/collinear metric landmarks in unsupported EPNP."""
    xyz = np.asarray(points_m, dtype=np.float64)
    require(
        xyz.ndim == 2 and xyz.shape[1] == 3 and xyz.shape[0] >= 8,
        "insufficient 3D PnP landmarks",
    )
    require(np.all(np.isfinite(xyz)), "nonfinite PnP metric landmarks")
    require(
        np.all(xyz[:, 2] > 0.05), "PnP stereo landmarks require positive metric depth"
    )
    singular = np.linalg.svd(xyz - xyz.mean(axis=0), compute_uv=False)
    require(
        singular.shape == (3,) and singular[0] >= 0.05,
        "degenerate PnP 3D landmark spread",
    )
    ratio = float(singular[-1] / singular[0])
    require(
        np.isfinite(ratio) and ratio >= MIN_PNP_3D_INLIER_SINGULAR_RATIO,
        "degenerate PnP 3D landmark geometry; unsupported near-planar EPNP",
    )
    return ratio


def relative_camera_pnp(
    points_prev_rect_m: np.ndarray,
    pixels_current_rect: np.ndarray,
    rectified_K: np.ndarray,
    *,
    min_inliers: int = 12,
    max_rmse_px: float = 2.5,
) -> tuple[np.ndarray, dict]:
    """Get T_current_cam_previous_cam from 3D-to-2D RANSAC correspondences."""
    xyz = np.asarray(points_prev_rect_m, dtype=np.float64)
    uv = np.asarray(pixels_current_rect, dtype=np.float64)
    K = np.asarray(rectified_K, dtype=np.float64)
    require(
        xyz.ndim == 2
        and xyz.shape[1] == 3
        and uv.shape == (xyz.shape[0], 2)
        and K.shape == (3, 3),
        "PnP requires matched Nx3/Nx2 and 3x3 K",
    )
    require(
        xyz.shape[0] >= min_inliers and min_inliers >= 8,
        "insufficient PnP correspondences",
    )
    require(
        np.all(np.isfinite(xyz))
        and np.all(np.isfinite(uv))
        and np.all(np.isfinite(K))
        and K[0, 0] > 0
        and K[1, 1] > 0,
        "nonfinite or invalid PnP calibration/input",
    )
    require(max_rmse_px > 0, "invalid reprojection tolerance")
    require(
        np.linalg.matrix_rank(np.cov(uv.T), tol=1e-8) == 2,
        "degenerate PnP image feature distribution",
    )
    # Do not let a few spatial outliers rescue a degenerate inlier consensus.
    metric_pnp_spatial_conditioning(xyz)
    cv2.setRNGSeed(2026)
    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        np.ascontiguousarray(xyz),
        np.ascontiguousarray(uv),
        np.ascontiguousarray(K),
        np.zeros((4, 1), dtype=np.float64),
        iterationsCount=150,
        reprojectionError=2.0,
        confidence=0.999,
        flags=cv2.SOLVEPNP_EPNP,
    )
    require(
        ok and inliers is not None and len(inliers) >= min_inliers,
        "PnP failed inlier gate",
    )
    selected = inliers[:, 0]
    spatial_ratio = metric_pnp_spatial_conditioning(xyz[selected])
    projected, _ = cv2.projectPoints(xyz[selected], rvec, tvec, K, None)
    errors = np.linalg.norm(projected.reshape(-1, 2) - uv[selected], axis=1)
    rmse = float(np.sqrt(np.mean(errors * errors)))
    rotation, _ = cv2.Rodrigues(rvec)
    camera_pts = xyz[selected] @ rotation.T + tvec.reshape((1, 3))
    require(
        np.all(camera_pts[:, 2] > 0),
        "PnP triangulated features are behind current camera",
    )
    require(
        np.isfinite(rmse) and rmse <= max_rmse_px, "PnP reprojection RMSE exceeded gate"
    )
    return make_transform(rotation, tvec), {
        "inliers": len(selected),
        "matches": xyz.shape[0],
        "rmse_px": rmse,
        "pnp_inlier_3d_singular_ratio": spatial_ratio,
    }


def integrate_camera_motion(
    previous_world_imu: np.ndarray,
    current_from_previous_rect_camera: np.ndarray,
    imu_from_rect_camera: np.ndarray,
) -> np.ndarray:
    """Compose relative rectified camera motion into anchored IMU-world pose."""
    last = np.asarray(previous_world_imu, dtype=np.float64)
    delta = np.asarray(current_from_previous_rect_camera, dtype=np.float64)
    extrinsic = np.asarray(imu_from_rect_camera, dtype=np.float64)
    require(
        all(m.shape == (4, 4) for m in (last, delta, extrinsic)),
        "invalid relative pose/extrinsics",
    )
    # T_W_Cprev = T_W_Iprev @ T_I_Crect
    # T_W_Ccur  = T_W_Cprev @ inverse(T_Ccur_Cprev)
    # T_W_Icur  = T_W_Ccur @ inverse(T_I_Crect)
    result = last @ extrinsic @ np.linalg.inv(delta) @ np.linalg.inv(extrinsic)
    require(np.all(np.isfinite(result)), "nonfinite world-to-IMU motion")
    return result


def rotation_to_quaternion_wxyz(rotation: np.ndarray) -> np.ndarray:
    """Right-handed orientation q_W_I from the integrated metric stereo pose.

    The first accepted IMU orientation is an identity prior, NOT measured gravity.
    """
    r = np.asarray(rotation, dtype=np.float64)
    require(
        r.shape == (3, 3) and np.isfinite(r).all(),
        "visual orientation must be a finite 3x3 matrix",
    )
    require(
        np.linalg.norm(r.T @ r - np.eye(3)) < 1e-3
        and abs(float(np.linalg.det(r)) - 1.0) < 1e-3,
        "visual orientation must be a proper rotation",
    )
    vector, _ = cv2.Rodrigues(r)
    vector = vector.reshape(3)
    theta = float(np.linalg.norm(vector))
    if theta < 1e-12:
        q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    else:
        half = theta / 2.0
        q = np.r_[np.cos(half), np.sin(half) * vector / theta]
    require(
        abs(float(np.dot(q, q)) - 1.0) < 1e-8, "non-unit visual orientation quaternion"
    )
    return q


def rectified_left(
    image_path: Path, rect: dict, *, source_image: np.ndarray | None = None
) -> np.ndarray:
    # Reuse the already-validated source image to avoid a second disk read.
    raw = (
        cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
        if source_image is None
        else source_image
    )
    width, height = rect["image_size"]
    require(
        raw is not None and raw.dtype == np.uint8 and raw.shape == (height, width),
        "invalid camera grayscale PNG dimensions",
    )
    mx, my = cv2.initUndistortRectifyMap(
        rect["K0"],
        rect["D0"],
        rect["R0"],
        rect["P0"],
        rect["image_size"],
        cv2.CV_16SC2,
    )
    return cv2.remap(raw, mx, my, cv2.INTER_LINEAR)


def tracked_stereo_motion(
    previous_left: np.ndarray,
    current_left: np.ndarray,
    previous_landmarks: dict,
    rect: dict,
    *,
    min_inliers: int = 12,
) -> tuple[np.ndarray, dict]:
    pixels = np.asarray(previous_landmarks["rectified_left_points"], dtype=np.float32)
    xyz = np.asarray(previous_landmarks["rectified_points_m"], dtype=np.float64)
    require(
        pixels.shape == (xyz.shape[0], 2),
        "stereo pixel-to-metric-landmark association missing",
    )
    require(
        pixels.shape[0] >= min_inliers, "too few stereo features for temporal tracking"
    )
    next_pixels, tracked, _ = cv2.calcOpticalFlowPyrLK(
        previous_left,
        current_left,
        pixels.reshape(-1, 1, 2),
        None,
        winSize=(21, 21),
        maxLevel=3,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
    )
    require(
        next_pixels is not None and tracked is not None,
        "optical flow produced no tracks",
    )
    backward_pixels, backward_tracked, _ = cv2.calcOpticalFlowPyrLK(
        current_left,
        previous_left,
        next_pixels,
        None,
        winSize=(21, 21),
        maxLevel=3,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
    )
    require(
        backward_pixels is not None and backward_tracked is not None,
        "backward optical flow produced no tracks",
    )
    fb_error = np.linalg.norm(backward_pixels.reshape(-1, 2) - pixels, axis=1)
    current = next_pixels.reshape(-1, 2)
    width, height = rect["image_size"]
    valid = (
        (tracked.reshape(-1) == 1)
        & (backward_tracked.reshape(-1) == 1)
        & np.isfinite(fb_error)
        & (fb_error <= 1.5)
        & np.isfinite(current).all(axis=1)
        & (current[:, 0] >= 0)
        & (current[:, 0] < width)
        & (current[:, 1] >= 0)
        & (current[:, 1] < height)
    )
    require(
        int(valid.sum()) >= min_inliers,
        "too few surviving temporal optical-flow tracks",
    )
    transform, stats = relative_camera_pnp(
        xyz[valid],
        current[valid],
        rect["P0"][:3, :3],
        min_inliers=min_inliers,
    )
    stats["flow_tracks"] = int(valid.sum())
    return transform, stats


def visual_measurements(
    sequence_root: Path,
    *,
    max_frames: int = 0,
    min_inliers: int = 12,
) -> tuple[list[dict], dict]:
    """Extract VO observations on accepted sync frames; fail-stop at tracking loss."""
    require(
        type(max_frames) is int and max_frames >= 0,
        "max_frames must be a nonnegative integer",
    )
    require(
        type(min_inliers) is int and 8 <= min_inliers <= 1000,
        "unsupported PnP inlier threshold",
    )
    root = sequence_root.resolve(strict=True)
    frames, synchronization = synchronize(root, hash_images=True)
    calibration = audit(root)
    rect = rectification(calibration)
    T_imu_cam = np.asarray(calibration["T_imu_cam0"], dtype=np.float64)
    R_rect = rect["R0"]
    T_imu_cam_rect = T_imu_cam @ make_transform(R_rect.T, np.zeros(3))
    # Anchor is the initial IMU frame; never use publisher ground truth.
    world_imu = np.eye(4, dtype=np.float64)
    last_position = np.zeros(3, dtype=np.float64)
    previous = None
    previous_left = None
    previous_landmarks = None
    observations = []
    statuses = Counter()
    frame_audit = []
    considered = 0
    for frame in frames:
        if max_frames and considered >= max_frames:
            break
        considered += 1
        stamp = frame["timestamp_ns"]
        if frame["status"] not in ("anchor", "ready"):
            statuses["sync_" + frame["status"]] += 1
            if previous is not None:
                statuses["failed_sync_discontinuity"] += 1
                frame_audit.append(
                    {
                        "timestamp_ns": stamp,
                        "status": "failed_sync_discontinuity",
                        "sync_rejection": frame["status"],
                    }
                )
                break
            frame_audit.append(
                {
                    "timestamp_ns": stamp,
                    "status": "sync_" + frame["status"],
                }
            )
            continue
        path0 = (root / frame["cam0_relative_path"]).resolve(strict=True)
        path1 = (root / frame["cam1_relative_path"]).resolve(strict=True)
        require(
            path0.is_relative_to(root) and path1.is_relative_to(root),
            "camera frame source escapes original dataset",
        )
        image0 = cv2.imread(str(path0), cv2.IMREAD_GRAYSCALE)
        image1 = cv2.imread(str(path1), cv2.IMREAD_GRAYSCALE)
        try:
            features = find_stereo_landmarks(
                image0,
                image1,
                rect,
            )
            current_left = rectified_left(path0, rect, source_image=image0)
        except (ValueError, cv2.error):
            statuses["failed_stereo_frontend"] += 1
            frame_audit.append(
                {
                    "timestamp_ns": stamp,
                    "status": "failed_stereo_frontend",
                }
            )
            break
        if previous is None:
            previous = stamp
            previous_left = current_left
            previous_landmarks = features
            statuses["anchor"] += 1
            frame_audit.append({"timestamp_ns": stamp, "status": "anchor"})
            continue
        dt_s = (stamp - previous) * 1e-9
        if not 0 < dt_s <= 0.5:
            statuses["failed_temporal_gap"] += 1
            frame_audit.append(
                {
                    "timestamp_ns": stamp,
                    "status": "failed_temporal_gap",
                }
            )
            break
        try:
            delta, stats = tracked_stereo_motion(
                previous_left,
                current_left,
                previous_landmarks,
                rect,
                min_inliers=min_inliers,
            )
            candidate = integrate_camera_motion(world_imu, delta, T_imu_cam_rect)
        except (ValueError, cv2.error, np.linalg.LinAlgError):
            statuses["failed_temporal_tracking"] += 1
            frame_audit.append(
                {
                    "timestamp_ns": stamp,
                    "status": "failed_temporal_tracking",
                }
            )
            break
        position = candidate[:3, 3]
        velocity = (position - last_position) / dt_s
        speed = float(np.linalg.norm(velocity))
        if not np.all(np.isfinite(velocity)) or speed > 30.0:
            statuses["failed_speed_gate"] += 1
            frame_audit.append(
                {
                    "timestamp_ns": stamp,
                    "status": "failed_speed_gate",
                }
            )
            break
        # These sigmas are conservative fixed defaults, NOT calibrated
        # residual statistics or a benchmark uncertainty estimate.
        attitude = rotation_to_quaternion_wxyz(candidate[:3, :3])
        observations.append(
            {
                "timestamp_ns": stamp,
                "x_m": float(position[0]),
                "y_m": float(position[1]),
                "z_m": float(position[2]),
                "vx_mps": float(velocity[0]),
                "vy_mps": float(velocity[1]),
                "vz_mps": float(velocity[2]),
                "sigma_position_m": 0.5,
                "sigma_velocity_mps": 0.75,
                "pnp_inliers": stats["inliers"],
                "pnp_reprojection_rmse_px": stats["rmse_px"],
                "qw": float(attitude[0]),
                "qx": float(attitude[1]),
                "qy": float(attitude[2]),
                "qz": float(attitude[3]),
                "sigma_orientation_rad": 0.25,
            }
        )
        statuses["visual_pose"] += 1
        frame_audit.append(
            {
                "timestamp_ns": stamp,
                "status": "visual_pose",
                "pnp_inliers": stats["inliers"],
                "pnp_reprojection_rmse_px": stats["rmse_px"],
                "flow_tracks": stats["flow_tracks"],
                **(
                    {"pnp_inlier_3d_singular_ratio": stats["pnp_inlier_3d_singular_ratio"]}
                    if "pnp_inlier_3d_singular_ratio" in stats
                    else {}
                ),
            }
        )
        world_imu = candidate
        last_position = position
        previous = stamp
        previous_left = current_left
        previous_landmarks = features

    require(
        len(frame_audit) == considered,
        "every examined camera timestamp must have an audit record",
    )
    manifest = {
        "schema_version": 1,
        "scope": "stereo_visual_odometry_measurements_not_benchmark",
        "coordinate_frame": "world_equals_first_accepted_imu_frame",
        "dataset_provenance_verified": False,
        "ground_truth_used": False,
        "uses_visual_pose_position_velocity_only": True,
        "visual_orientation_available_for_native_filter": True,
        "visual_orientation_uncertainty_empirically_calibrated": False,
        "first_camera_anchor_is_not_a_pose_measurement": True,
        "camera_source_hashes": synchronization["source_sensor_hashes"],
        "calibration_sha256": calibration["source_yaml_sha256"],
        "sync_status_counts": synchronization["status_counts"],
        "visual_status_counts": dict(sorted(statuses.items())),
        "frame_audit": frame_audit,
        "frame_audit_count": len(frame_audit),
        "frame_audit_sha256": hashlib.sha256(
            json.dumps(frame_audit, sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest(),
        "camera_timestamps_examined": considered,
        "camera_timestamps_total": len(frames),
        "frame_limit_requested": max_frames,
        "frame_limit_truncated": bool(
            max_frames
            and considered < len(frames)
            and not any(key.startswith("failed_") for key in statuses)
        ),
        "tracking_failure_truncated": bool(
            considered < len(frames)
            and any(key.startswith("failed_") for key in statuses)
        ),
        "visual_measurement_count": len(observations),
        "failed_closed_on_tracking_loss": any(
            key.startswith("failed_") for key in statuses
        ),
        "limitations": [
            "visual scale is from stereo; initial IMU orientation and gravity not estimated",
            "visual pose uses sequential PnP, not loop closure or global map optimization",
            "visual attitude is derived from correlated sequential stereo PnP and may drift",
            "fixed observation uncertainty is not empirically calibrated",
            "no publisher EuRoC trajectory or ATE/RPE comparison was performed",
        ],
    }
    return observations, manifest


def export(
    sequence_root: Path,
    output_csv: Path,
    manifest_json: Path,
    *,
    max_frames: int = 0,
    min_inliers: int = 12,
    allow_partial: bool = False,
) -> dict:
    root = sequence_root.resolve(strict=True)
    for output in (output_csv, manifest_json):
        require(
            not output.resolve().is_relative_to(root),
            "output must remain outside original EuRoC sequence root",
        )
        require(
            not output.exists() and not output.is_symlink(),
            "refusing to overwrite source output",
        )
    require(
        output_csv.resolve() != manifest_json.resolve(),
        "visual CSV and manifest paths must differ",
    )
    observations, manifest = visual_measurements(
        root, max_frames=max_frames, min_inliers=min_inliers
    )
    require(bool(observations), "no accepted visual measurements to export")
    require(
        not manifest["failed_closed_on_tracking_loss"] or allow_partial,
        "tracking failed; refusing partial visual trajectory without --allow-partial",
    )
    manifest["partial_failure_export"] = bool(
        manifest["failed_closed_on_tracking_loss"] and allow_partial
    )
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    manifest_json.parent.mkdir(parents=True, exist_ok=True)
    try:
        with output_csv.open("x", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=VISUAL_COLUMNS)
            writer.writeheader()
            writer.writerows(observations)
        manifest["visual_csv_sha256"] = hashlib.sha256(
            output_csv.read_bytes()
        ).hexdigest()
        with manifest_json.open("x", encoding="utf-8") as stream:
            json.dump(manifest, stream, indent=2, sort_keys=True)
            stream.write("\n")
    except Exception:
        output_csv.unlink(missing_ok=True)
        raise
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sequence-root", type=Path, required=True)
    parser.add_argument("--output-visual-csv", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--min-inliers", type=int, default=12)
    parser.add_argument("--allow-partial", action="store_true")
    args = parser.parse_args()
    result = export(
        args.sequence_root,
        args.output_visual_csv,
        args.manifest,
        max_frames=args.max_frames,
        min_inliers=args.min_inliers,
        allow_partial=args.allow_partial,
    )
    print(
        f"Visual PnP measurements: {result['visual_measurement_count']}; "
        "ground-truth-free VO only; not an accuracy benchmark."
    )


if __name__ == "__main__":
    main()
