"""Synthetic-only stereo-PnP/IMU-frame integration regression tests.

No authentic EuRoC imagery or publisher ground truth is evaluated here.
"""

import tempfile
import unittest
from pathlib import Path
from unittest import mock

import cv2
import numpy as np

from research.experiments.euroc_stereo_visual_odometry import (
    VISUAL_COLUMNS,
    export,
    integrate_camera_motion,
    make_transform,
    relative_camera_pnp,
    visual_measurements,
)


class StereoVisualOdometryTests(unittest.TestCase):
    def setUp(self):
        self.K = np.asarray([[458.0, 0.0, 367.0], [0.0, 457.0, 248.0], [0.0, 0.0, 1.0]])
        rng = np.random.default_rng(81)
        self.landmarks = np.column_stack(
            (
                rng.uniform(-1, 1, 60),
                rng.uniform(-0.6, 0.6, 60),
                rng.uniform(3, 9, 60),
            )
        )
        self.rotation, _ = cv2.Rodrigues(
            np.array([0.008, -0.012, 0.015], dtype=np.float64)
        )
        self.translation = np.asarray([0.08, -0.02, 0.04])
        p = self.landmarks @ self.rotation.T + self.translation
        pixels = p @ self.K.T
        self.current_pixels = pixels[:, :2] / pixels[:, 2:]

    def test_pnp_recovers_synthetic_metric_relative_pose(self):
        relative, diagnostics = relative_camera_pnp(
            self.landmarks, self.current_pixels, self.K
        )
        self.assertEqual(diagnostics["matches"], 60)
        self.assertGreaterEqual(diagnostics["inliers"], 50)
        self.assertLess(diagnostics["rmse_px"], 0.01)
        self.assertTrue(np.allclose(relative[:3, :3], self.rotation, atol=0.005))
        self.assertTrue(np.allclose(relative[:3, 3], self.translation, atol=0.02))

    def test_initial_imu_world_pose_and_camera_lever_arm(self):
        extra = make_transform(np.eye(3), [0.16, 0.02, -0.04])
        relative = make_transform(np.eye(3), [0.10, 0.0, 0.0])
        fused = integrate_camera_motion(np.eye(4), relative, extra)
        self.assertTrue(np.allclose(fused[:3, 3], [-0.1, 0.0, 0.0], atol=1e-10))
        self.assertTrue(np.allclose(fused[:3, :3], np.eye(3)))

    def test_right_handed_rotation_gate(self):
        reflection = np.diag([-1.0, 1.0, 1.0])
        with self.assertRaisesRegex(ValueError, "reflected"):
            make_transform(reflection, [0.0, 0.0, 0.0])

    def test_nonfinite_pose_input_gate(self):
        with self.assertRaisesRegex(ValueError, "nonfinite"):
            make_transform(np.eye(3), [float("nan"), 0.0, 0.0])

    def test_insufficient_pnp_points_gate(self):
        with self.assertRaisesRegex(ValueError, "insufficient"):
            relative_camera_pnp(self.landmarks[:5], self.current_pixels[:5], self.K)

    def test_nonfinite_correspondence_gate(self):
        pixels = self.current_pixels.copy()
        pixels[0, 0] = float("inf")
        with self.assertRaisesRegex(ValueError, "nonfinite"):
            relative_camera_pnp(self.landmarks, pixels, self.K)

    def test_degenerate_correspondences_do_not_claim_pose(self):
        with self.assertRaisesRegex(ValueError, "degenerate|PnP failed|behind|RMSE"):
            relative_camera_pnp(
                self.landmarks,
                np.tile([100.0, 100.0], (self.landmarks.shape[0], 1)),
                self.K,
            )

    def test_csv_schema_matches_native_visual_pose_contract(self):
        self.assertEqual(len(VISUAL_COLUMNS), 11)
        self.assertEqual(VISUAL_COLUMNS[0], "timestamp_ns")
        self.assertEqual(VISUAL_COLUMNS[1:4], ("x_m", "y_m", "z_m"))
        self.assertEqual(
            VISUAL_COLUMNS[7:9], ("sigma_position_m", "sigma_velocity_mps")
        )

    def test_rejected_camera_frame_breaks_visual_trajectory(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "synthetic"
            root.mkdir()
            (root / "cam0.png").write_bytes(b"not-a-real-image")
            (root / "cam1.png").write_bytes(b"not-a-real-image")
            frames = [
                {
                    "timestamp_ns": 1000000000,
                    "status": "anchor",
                    "cam0_relative_path": "cam0.png",
                    "cam1_relative_path": "cam1.png",
                },
                {
                    "timestamp_ns": 1020000000,
                    "status": "missing_cam1",
                    "cam0_relative_path": "cam0.png",
                    "cam1_relative_path": None,
                },
                {
                    "timestamp_ns": 1040000000,
                    "status": "ready",
                    "cam0_relative_path": "cam0.png",
                    "cam1_relative_path": "cam1.png",
                },
            ]
            sync_manifest = {
                "source_sensor_hashes": {"cam0": {}, "cam1": {}, "imu0": {}},
                "status_counts": {"anchor": 1, "missing_cam1": 1, "ready": 1},
            }
            calibration = {
                "T_imu_cam0": np.eye(4).tolist(),
                "source_yaml_sha256": {"cam0": "synthetic"},
            }
            stereo_points = {
                "rectified_left_points": np.zeros((15, 2)),
                "rectified_points_m": np.ones((15, 3)),
            }
            image = np.zeros((100, 100), dtype=np.uint8)
            with (
                mock.patch(
                    "research.experiments.euroc_stereo_visual_odometry.synchronize",
                    return_value=(frames, sync_manifest),
                ),
                mock.patch(
                    "research.experiments.euroc_stereo_visual_odometry.audit",
                    return_value=calibration,
                ),
                mock.patch(
                    "research.experiments.euroc_stereo_visual_odometry.rectification",
                    return_value={"R0": np.eye(3)},
                ),
                mock.patch(
                    "research.experiments.euroc_stereo_visual_odometry.cv2.imread",
                    return_value=image,
                ),
                mock.patch(
                    "research.experiments.euroc_stereo_visual_odometry.rectified_left",
                    return_value=image,
                ),
                mock.patch(
                    "research.experiments.euroc_stereo_visual_odometry.find_stereo_landmarks",
                    return_value=stereo_points,
                ) as frontend,
            ):
                observations, manifest = visual_measurements(root)
            self.assertEqual(observations, [])
            self.assertTrue(manifest["failed_closed_on_tracking_loss"])
            self.assertTrue(manifest["tracking_failure_truncated"])
            self.assertFalse(manifest["frame_limit_truncated"])
            self.assertEqual(
                manifest["visual_status_counts"]["failed_sync_discontinuity"],
                1,
            )
            self.assertEqual(manifest["camera_timestamps_examined"], 2)
            self.assertEqual(manifest["frame_audit_count"], 2)
            self.assertEqual(
                [entry["status"] for entry in manifest["frame_audit"]],
                ["anchor", "failed_sync_discontinuity"],
            )
            self.assertEqual(
                manifest["frame_audit"][1]["sync_rejection"], "missing_cam1"
            )
            self.assertEqual(len(manifest["frame_audit_sha256"]), 64)
            frontend.assert_called_once()

    def test_accepted_visual_frames_match_frame_audit_and_frame_limit(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "synthetic"
            root.mkdir()
            for name in ("cam0.png", "cam1.png"):
                (root / name).write_bytes(b"synthetic-camera-placeholder")
            frames = [
                {
                    "timestamp_ns": timestamp,
                    "status": "anchor" if index == 0 else "ready",
                    "cam0_relative_path": "cam0.png",
                    "cam1_relative_path": "cam1.png",
                }
                for index, timestamp in enumerate(
                    (1_000_000_000, 1_100_000_000, 1_200_000_000)
                )
            ]
            sync = {
                "source_sensor_hashes": {},
                "status_counts": {"anchor": 1, "ready": 2},
            }
            calibration = {
                "T_imu_cam0": np.eye(4).tolist(),
                "source_yaml_sha256": {},
            }
            landmarks = {
                "rectified_left_points": np.zeros((24, 2)),
                "rectified_points_m": np.ones((24, 3)),
            }
            frame_image = np.zeros((100, 100), dtype=np.uint8)
            pnp_transform = make_transform(np.eye(3), [0.1, 0.0, 0.0])
            pnp_diagnostics = {
                "inliers": 24,
                "rmse_px": 0.4,
                "flow_tracks": 27,
            }
            with (
                mock.patch(
                    "research.experiments.euroc_stereo_visual_odometry.synchronize",
                    return_value=(frames, sync),
                ),
                mock.patch(
                    "research.experiments.euroc_stereo_visual_odometry.audit",
                    return_value=calibration,
                ),
                mock.patch(
                    "research.experiments.euroc_stereo_visual_odometry.rectification",
                    return_value={"R0": np.eye(3)},
                ),
                mock.patch(
                    "research.experiments.euroc_stereo_visual_odometry.cv2.imread",
                    return_value=frame_image,
                ),
                mock.patch(
                    "research.experiments.euroc_stereo_visual_odometry.rectified_left",
                    return_value=frame_image,
                ),
                mock.patch(
                    "research.experiments.euroc_stereo_visual_odometry.find_stereo_landmarks",
                    return_value=landmarks,
                ),
                mock.patch(
                    "research.experiments.euroc_stereo_visual_odometry.tracked_stereo_motion",
                    return_value=(pnp_transform, pnp_diagnostics),
                ),
            ):
                observations, manifest = visual_measurements(root, max_frames=2)
            self.assertEqual(len(observations), 1)
            self.assertEqual(observations[0]["timestamp_ns"], 1_100_000_000)
            self.assertAlmostEqual(observations[0]["x_m"], -0.1, places=6)
            self.assertAlmostEqual(observations[0]["vx_mps"], -1.0, places=6)
            self.assertEqual(manifest["frame_audit_count"], 2)
            self.assertEqual(
                [row["status"] for row in manifest["frame_audit"]],
                ["anchor", "visual_pose"],
            )
            self.assertEqual(manifest["frame_audit"][1]["pnp_inliers"], 24)
            self.assertEqual(manifest["frame_audit"][1]["flow_tracks"], 27)
            self.assertTrue(manifest["frame_limit_truncated"])
            self.assertFalse(manifest["tracking_failure_truncated"])
            self.assertFalse(manifest["failed_closed_on_tracking_loss"])

    def test_export_refuses_output_in_source_tree(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "synthetic"
            root.mkdir()
            with self.assertRaisesRegex(ValueError, "outside"):
                export(
                    root,
                    root / "visual.csv",
                    Path(directory) / "manifest.json",
                )


if __name__ == "__main__":
    unittest.main()
