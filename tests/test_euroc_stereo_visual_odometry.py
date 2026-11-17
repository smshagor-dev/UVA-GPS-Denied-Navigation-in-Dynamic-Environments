"""Synthetic-only stereo-PnP/IMU-frame integration regression tests.

No authentic EuRoC imagery or publisher ground truth is evaluated here.
"""

import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from research.experiments.euroc_stereo_visual_odometry import (
    VISUAL_COLUMNS,
    export,
    integrate_camera_motion,
    make_transform,
    relative_camera_pnp,
)


class StereoVisualOdometryTests(unittest.TestCase):
    def setUp(self):
        self.K = np.asarray(
            [[458.0, 0.0, 367.0], [0.0, 457.0, 248.0], [0.0, 0.0, 1.0]]
        )
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
        self.assertTrue(
            np.allclose(relative[:3, :3], self.rotation, atol=0.005)
        )
        self.assertTrue(
            np.allclose(relative[:3, 3], self.translation, atol=0.02)
        )

    def test_initial_imu_world_pose_and_camera_lever_arm(self):
        extra = make_transform(np.eye(3), [0.16, 0.02, -0.04])
        relative = make_transform(np.eye(3), [0.10, 0.0, 0.0])
        fused = integrate_camera_motion(np.eye(4), relative, extra)
        self.assertTrue(
            np.allclose(fused[:3, 3], [-0.1, 0.0, 0.0], atol=1e-10)
        )
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
            relative_camera_pnp(
                self.landmarks[:5], self.current_pixels[:5], self.K
            )

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
            VISUAL_COLUMNS[7:9],
            ("sigma_position_m", "sigma_velocity_mps"),
        )

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
