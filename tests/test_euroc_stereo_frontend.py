"""Synthetic-only regression tests for calibrated EuRoC stereo geometry."""

import csv
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np
import yaml

from research.experiments.euroc_calibration_validation import audit
from research.experiments.euroc_stereo_frontend import (
    camera_at_timestamp,
    evaluate_pair,
    rectification,
    triangulate_correspondences,
)
from tests.test_euroc_calibration_validation import calibrations


START_NS = 1403636579763555584


def stereo_fixture(root, *, blank=False):
    calibration = calibrations()
    for sensor, values in calibration.items():
        folder = root / "mav0" / sensor
        folder.mkdir(parents=True, exist_ok=True)
        (folder / "sensor.yaml").write_text(
            yaml.safe_dump(values), encoding="utf-8"
        )

    rng = np.random.default_rng(131)
    width, height = 752, 480
    if blank:
        left = np.zeros((height, width), dtype=np.uint8)
    else:
        left = rng.integers(0, 256, (height, width), dtype=np.uint8)
    # Synthetic baseline/disparity fixture; deliberately NOT EuRoC evidence.
    right = cv2.warpAffine(
        left,
        np.float32([[1, 0, -20], [0, 1, 0]]),
        (width, height),
    )

    for sensor, image in (("cam0", left), ("cam1", right)):
        folder = root / "mav0" / sensor
        (folder / "data").mkdir(exist_ok=True)
        filename = f"{START_NS}.png"
        self_image = folder / "data" / filename
        assert cv2.imwrite(str(self_image), image)
        with (folder / "data.csv").open(
            "w", newline="", encoding="utf-8"
        ) as stream:
            writer = csv.writer(stream)
            writer.writerow(["#timestamp [ns]", "filename"])
            writer.writerow([START_NS, filename])


class EurocStereoFrontendTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / "synthetic_sequence"
        self.root.mkdir()
        stereo_fixture(self.root)

    def test_rectification_has_metric_baseline(self):
        rect = rectification(audit(self.root))
        self.assertEqual(rect["image_size"], (752, 480))
        self.assertGreater(abs(float(rect["P1"][0, 3])), 1.0)

    def test_known_metric_points_are_recovered(self):
        rect = rectification(audit(self.root))
        xyz = np.asarray(
            [
                [0.0, 0.0, 3.0],
                [0.2, 0.1, 4.0],
                [-0.3, -0.15, 7.0],
                [0.4, 0.18, 8.0],
                [-0.1, 0.2, 5.0],
                [0.15, -0.1, 6.0],
                [0.3, 0.2, 11.0],
                [-0.15, -0.2, 12.0],
            ],
            dtype=np.float64,
        )
        homogeneous = np.column_stack((xyz, np.ones(xyz.shape[0])))
        projection0 = (rect["P0"] @ homogeneous.T).T
        projection1 = (rect["P1"] @ homogeneous.T).T
        pixels0 = projection0[:, :2] / projection0[:, 2:3]
        pixels1 = projection1[:, :2] / projection1[:, 2:3]
        chosen, triangulated = triangulate_correspondences(
            rect, pixels0, pixels1
        )
        self.assertEqual(len(chosen), len(xyz))
        self.assertTrue(np.allclose(triangulated, xyz, atol=1e-5))

    def test_genuine_image_pipeline_on_synthetic_texture(self):
        result = evaluate_pair(self.root, START_NS, include_landmarks=True)
        self.assertGreaterEqual(result["camera_metric_points"], 8)
        self.assertEqual(
            len(result["landmarks"]), result["camera_metric_points"]
        )
        self.assertEqual(result["point_frame"], "IMU")
        self.assertFalse(result["ground_truth_used"])
        self.assertFalse(result["world_pose_estimated"])
        self.assertFalse(result["dataset_provenance_verified"])
        self.assertEqual(len(result["source_image_sha256"]["cam0"]), 64)

    def test_landmarks_omitted_unless_requested(self):
        result = evaluate_pair(self.root, START_NS)
        self.assertNotIn("landmarks", result)
        self.assertGreaterEqual(result["camera_metric_points"], 8)

    def test_no_texture_fails_closed(self):
        stereo_fixture(self.root, blank=True)
        with self.assertRaisesRegex(ValueError, "insufficient stereo"):
            evaluate_pair(self.root, START_NS)

    def test_no_camera_sample_fails_closed(self):
        with self.assertRaisesRegex(ValueError, "timestamp not present"):
            camera_at_timestamp(self.root, "cam0", START_NS + 1)

    def test_missing_image_fails_closed(self):
        (self.root / "mav0" / "cam1" / "data" / f"{START_NS}.png").unlink()
        with self.assertRaisesRegex(ValueError, "missing regular"):
            evaluate_pair(self.root, START_NS)

    def test_wrong_png_dimensions_fails_closed(self):
        path = self.root / "mav0" / "cam0" / "data" / f"{START_NS}.png"
        assert cv2.imwrite(str(path), np.zeros((64, 64), dtype=np.uint8))
        with self.assertRaisesRegex(ValueError, "wrong pixel dimensions"):
            evaluate_pair(self.root, START_NS)

    def test_camera_csv_path_traversal_fails_closed(self):
        path = self.root / "mav0" / "cam0" / "data.csv"
        path.write_text(
            f"#timestamp [ns],filename\n{START_NS},../escape.png\n",
            encoding="utf-8",
        )
        with self.assertRaisesRegex(ValueError, "unsafe camera image"):
            evaluate_pair(self.root, START_NS)

    def test_bad_correspondences_fail_closed(self):
        rect = rectification(audit(self.root))
        with self.assertRaisesRegex(ValueError, "nonfinite"):
            triangulate_correspondences(
                rect,
                np.asarray([[float("nan"), 0.0]]),
                np.asarray([[0.0, 0.0]]),
            )
        with self.assertRaisesRegex(ValueError, "thresholds"):
            triangulate_correspondences(
                rect,
                np.asarray([[1.0, 2.0]]),
                np.asarray([[1.0, 2.0]]),
                vertical_tolerance_px=50.0,
            )

    def test_nonmatching_epipolar_pairs_are_rejected(self):
        rect = rectification(audit(self.root))
        chosen, coords = triangulate_correspondences(
            rect,
            np.asarray([[150.0, 80.0], [160.0, 120.0]]),
            np.asarray([[145.0, 180.0], [158.0, 220.0]]),
        )
        self.assertEqual(len(chosen), 0)
        self.assertEqual(coords.shape, (0, 3))


if __name__ == "__main__":
    unittest.main()
