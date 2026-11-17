"""Only synthetic calibration fixtures: NOT authentic EuRoC dataset results."""

import hashlib
import json
import tempfile
import unittest
from pathlib import Path

import yaml

from research.experiments.euroc_calibration_validation import audit, export


def transform(tx=0.0, ty=0.0, tz=0.0):
    return {
        "rows": 4,
        "cols": 4,
        "data": [
            1.0,
            0.0,
            0.0,
            tx,
            0.0,
            1.0,
            0.0,
            ty,
            0.0,
            0.0,
            1.0,
            tz,
            0.0,
            0.0,
            0.0,
            1.0,
        ],
    }


def calibrations():
    imu = {
        "sensor_type": "imu",
        "rate_hz": 200.0,
        "T_BS": transform(),
        "gyroscope_noise_density": 1e-4,
        "gyroscope_random_walk": 1e-6,
        "accelerometer_noise_density": 2e-3,
        "accelerometer_random_walk": 2e-5,
    }
    camera = {
        "sensor_type": "camera",
        "rate_hz": 20.0,
        "camera_model": "pinhole",
        "distortion_model": "radial-tangential",
        "resolution": [752, 480],
        "intrinsics": [458.0, 457.0, 367.0, 248.0],
        "distortion_coefficients": [-0.28, 0.07, 0.0, 0.0],
    }
    return {
        "imu0": imu,
        "cam0": {**camera, "T_BS": transform(0.0, 0.0, 0.07)},
        "cam1": {**camera, "T_BS": transform(0.12, 0.0, 0.07)},
    }


class EurocCalibrationValidationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / "MH_01_easy_synthetic"
        self.root.mkdir()
        self.blocks = calibrations()
        self.save_all()

    def save_all(self):
        for sensor, block in self.blocks.items():
            path = self.root / "mav0" / sensor / "sensor.yaml"
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(yaml.safe_dump(block), encoding="utf-8")

    def test_valid_frames_and_baseline(self):
        result = audit(self.root)
        self.assertEqual(result["schema_version"], 1)
        self.assertAlmostEqual(result["stereo_baseline_m"], 0.12)
        self.assertAlmostEqual(result["T_imu_cam0"][2][3], 0.07)
        self.assertAlmostEqual(result["T_imu_cam1"][0][3], 0.12)
        self.assertAlmostEqual(result["T_cam1_cam0"][0][3], -0.12)
        self.assertEqual(
            result["camera_models"]["cam0"]["resolution_width_height"],
            [752, 480],
        )
        self.assertFalse(result["dataset_provenance_verified"])
        self.assertFalse(result["ground_truth_used"])

    def test_ground_truth_file_is_not_read(self):
        gt = self.root / "mav0" / "state_groundtruth_estimate0"
        gt.mkdir()
        (gt / "data.csv").write_text("invalid GT\n", encoding="utf-8")
        self.assertTrue(audit(self.root)["calibration_validated"])

    def test_reflection_is_rejected(self):
        self.blocks["cam1"]["T_BS"]["data"][0] = -1.0
        self.save_all()
        with self.assertRaisesRegex(ValueError, "right handed"):
            audit(self.root)

    def test_nonorthogonal_transform_is_rejected(self):
        self.blocks["cam0"]["T_BS"]["data"][0] = 2.0
        self.save_all()
        with self.assertRaisesRegex(ValueError, "not orthogonal"):
            audit(self.root)

    def test_bad_homogeneous_row_is_rejected(self):
        self.blocks["cam1"]["T_BS"]["data"][15] = 2.0
        self.save_all()
        with self.assertRaisesRegex(ValueError, "homogeneous"):
            audit(self.root)

    def test_zero_stereo_baseline_is_rejected(self):
        self.blocks["cam1"]["T_BS"] = transform(0, 0, 0.07)
        self.save_all()
        with self.assertRaisesRegex(ValueError, "degenerate stereo"):
            audit(self.root)

    def test_missing_camera_intrinsics_is_rejected(self):
        self.blocks["cam0"].pop("intrinsics")
        self.save_all()
        with self.assertRaisesRegex(ValueError, "fx, fy"):
            audit(self.root)

    def test_nonfinite_distortion_is_rejected(self):
        self.blocks["cam0"]["distortion_coefficients"][1] = float("nan")
        self.save_all()
        with self.assertRaisesRegex(ValueError, "nonfinite"):
            audit(self.root)

    def test_unsupported_distortion_is_rejected(self):
        self.blocks["cam0"]["distortion_model"] = "equidistant"
        self.save_all()
        with self.assertRaisesRegex(ValueError, "unsupported distortion"):
            audit(self.root)

    def test_bad_yaml_tag_is_rejected(self):
        path = self.root / "mav0" / "cam0" / "sensor.yaml"
        path.write_text("!!python/object/apply:os.system ['echo unsafe']\n")
        with self.assertRaisesRegex(ValueError, "invalid safe YAML"):
            audit(self.root)

    def test_symlink_source_is_rejected(self):
        path = self.root / "mav0" / "cam0" / "sensor.yaml"
        path.rename(path.with_suffix(".txt"))
        try:
            path.symlink_to(path.with_suffix(".txt"))
        except (OSError, NotImplementedError):
            self.skipTest("symlinks unsupported")
        with self.assertRaisesRegex(ValueError, "regular sensor.yaml"):
            audit(self.root)

    def test_export_checksums_and_overwrite_guard(self):
        output = Path(self.temp.name) / "outputs" / "calibration.json"
        result = export(self.root, output)
        stored = json.loads(output.read_text())
        self.assertEqual(stored["stereo_baseline_m"], result["stereo_baseline_m"])
        source = self.root / "mav0" / "imu0" / "sensor.yaml"
        expected = hashlib.sha256(source.read_bytes()).hexdigest()
        self.assertEqual(result["source_yaml_sha256"]["imu0"], expected)
        with self.assertRaisesRegex(ValueError, "overwrite"):
            export(self.root, output)

    def test_output_inside_dataset_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "outside"):
            export(self.root, self.root / "calibration.json")


if __name__ == "__main__":
    unittest.main()
