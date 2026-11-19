"""Synthetic-only EuRoC 6-DoF evaluator regression (no real dataset results)."""

import csv
import json
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from research.experiments.euroc_pose_evaluation import (
    Pose,
    evaluate,
    load_estimated,
    load_ground_truth,
    quaternion,
    rotation_error_rad,
    rotation_from_quaternion,
)


START_NS = 1403636579763555584


def yaw_quaternion(angle):
    return (math.cos(angle / 2), 0.0, 0.0, math.sin(angle / 2))


def trajectories(count=12):
    ground_truth = []
    estimated = []
    for index in range(count):
        p = (float(index), float(index % 3), float((index * index) % 5))
        yaw = 0.07 * index
        timestamp = START_NS + index * 1_000_000_000
        ground_truth.append(Pose(timestamp, p, yaw_quaternion(yaw)))
        # Rotate the entire estimated world frame by -90 degrees, translate
        # it and shift its timestamps by one millisecond. No scale correction.
        estimated.append(
            Pose(
                timestamp + 1_000_000,
                (p[1] + 3.0, -p[0] + 4.0, p[2] - 5.0),
                yaw_quaternion(yaw - math.pi / 2),
            )
        )
    return ground_truth, estimated


def write_inputs(directory, gt, est):
    gt_path = directory / "groundtruth.csv"
    est_path = directory / "estimated.csv"
    with gt_path.open("w", encoding="utf-8", newline="") as out:
        writer = csv.writer(out)
        writer.writerow(["#timestamp", "p_x", "p_y", "p_z", "q_w", "q_x", "q_y", "q_z"])
        for pose in gt:
            writer.writerow((pose.timestamp_ns, *pose.xyz, *pose.q_wxyz))
    with est_path.open("w", encoding="utf-8", newline="") as out:
        writer = csv.writer(out)
        writer.writerow(["timestamp_ns", "x_m", "y_m", "z_m", "qw", "qx", "qy", "qz"])
        for pose in est:
            writer.writerow((pose.timestamp_ns, *pose.xyz, *pose.q_wxyz))
    return gt_path, est_path


class EuRoCPoseEvaluationTests(unittest.TestCase):
    def test_known_rigid_frame_difference_has_zero_6dof_error(self):
        gt, est = trajectories()
        result = evaluate(gt, est, tolerance_ns=2_000_000)
        self.assertEqual(result["matched_samples"], 12)
        self.assertLess(result["ate_translation"]["rmse_m"], 1e-10)
        self.assertLess(result["absolute_orientation_error"]["rmse_deg"], 2e-5)
        relative = result["full_se3_relative_pose_error"]
        self.assertLess(relative["translation_in_first_body_frame"]["rmse_m"], 1e-10)
        self.assertLess(relative["orientation"]["rmse_deg"], 2e-5)
        self.assertEqual(relative["lag_s"], 1.0)
        self.assertFalse(result["ground_truth_used_for_estimator"])
        self.assertFalse(result["publisher_archive_provenance_verified"])

    def test_cannot_fit_metric_scale_away(self):
        gt, _ = trajectories()
        est = [
            Pose(p.timestamp_ns, tuple(1.1 * v for v in p.xyz), p.q_wxyz) for p in gt
        ]
        result = evaluate(gt, est)
        self.assertGreater(result["ate_translation"]["rmse_m"], 0.05)
        self.assertGreater(
            result["full_se3_relative_pose_error"]["translation_in_first_body_frame"][
                "rmse_m"
            ],
            0.05,
        )

    def test_orientation_drift_is_visible_independently_of_translation(self):
        gt, est = trajectories()
        modified = [
            Pose(
                p.timestamp_ns,
                p.xyz,
                yaw_quaternion(0.07 * index - math.pi / 2 + 0.025 * index),
            )
            for index, p in enumerate(est)
        ]
        result = evaluate(gt, modified, tolerance_ns=2_000_000)
        self.assertLess(result["ate_translation"]["rmse_m"], 1e-10)
        self.assertGreater(result["absolute_orientation_error"]["rmse_deg"], 1)
        self.assertGreater(
            result["full_se3_relative_pose_error"]["orientation"]["rmse_deg"],
            1,
        )

    def test_sign_equivalent_quaternions_have_same_rotation(self):
        q = yaw_quaternion(0.7)
        negative_q = tuple(-v for v in q)
        self.assertLess(
            rotation_error_rad(
                rotation_from_quaternion(q),
                rotation_from_quaternion(negative_q),
            ),
            1e-7,
        )

    def test_opposite_rotation_is_180_degrees(self):
        self.assertAlmostEqual(
            math.degrees(
                rotation_error_rad(
                    rotation_from_quaternion((1.0, 0.0, 0.0, 0.0)),
                    rotation_from_quaternion((0.0, 1.0, 0.0, 0.0)),
                )
            ),
            180,
            places=7,
        )

    def test_untrusted_quaternion_must_be_unit_and_finite(self):
        with self.assertRaisesRegex(ValueError, "non-unit"):
            quaternion(["2", "0", "0", "0"])
        with self.assertRaisesRegex(ValueError, "non-finite"):
            quaternion(["nan", "0", "0", "0"])
        with self.assertRaisesRegex(ValueError, "four scalar"):
            quaternion(["1", "0", "0"])

    def test_fails_closed_on_short_overlap_and_no_fixed_lag(self):
        gt, est = trajectories()
        with self.assertRaisesRegex(ValueError, "only 0 matched"):
            evaluate(gt, est, tolerance_ns=0)
        with self.assertRaisesRegex(ValueError, "insufficient matching time overlap"):
            evaluate(
                gt,
                est,
                tolerance_ns=2_000_000,
                min_overlap_s=15.0,
            )
        with self.assertRaisesRegex(ValueError, "no matched full-SE3"):
            evaluate(
                gt,
                est,
                tolerance_ns=2_000_000,
                relative_lag_s=0.4,
            )

    def test_strict_schema_duplicate_timestamps_and_frame_fields(self):
        gt, est = trajectories()
        with tempfile.TemporaryDirectory() as temp:
            gt_path, est_path = write_inputs(Path(temp), gt, est)
            self.assertEqual(len(load_ground_truth(gt_path)), 12)
            self.assertEqual(len(load_estimated(est_path)), 12)
            with est_path.open("a", encoding="utf-8") as stream:
                stream.write(f"{est[-1].timestamp_ns},0,0,0,1,0,0,0\n")
            with self.assertRaisesRegex(ValueError, "strictly increasing"):
                load_estimated(est_path)
            est_path.write_text(
                f"timestamp_ns,x_m,y_m,z_m,qx,qy,qz,qw\n{START_NS},0,0,0,0,0,0,1\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "exact eight-column"):
                load_estimated(est_path)
            gt_path.write_text(f"#header\n{START_NS},0,0,0,0,0,0,0\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "non-unit"):
                load_ground_truth(gt_path)

    def test_cli_generates_hashed_json_without_overwriting_source(self):
        gt, est = trajectories()
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)
            gt_path, est_path = write_inputs(folder, gt, est)
            output = folder / "six_dof_evaluation.json"
            cmd = [
                sys.executable,
                "-m",
                "research.experiments.euroc_pose_evaluation",
                "--ground-truth",
                str(gt_path),
                "--estimated",
                str(est_path),
                "--sequence",
                "synthetic-test-only",
                "--frame-agreement",
                "same-imu-body",
                "--output",
                str(output),
                "--tolerance-ms",
                "2",
            ]
            result = subprocess.run(cmd, check=True, capture_output=True, text=True)
            self.assertIn("12 poses", result.stdout)
            data = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(
                data["source_sha256"].keys(), {"ground_truth", "estimated"}
            )
            self.assertEqual(data["frame_agreement_assertion"], "same-imu-body")
            self.assertFalse(data["publisher_archive_provenance_verified"])
            duplicate = subprocess.run(cmd, capture_output=True, text=True)
            self.assertNotEqual(duplicate.returncode, 0)
            self.assertIn("refusing to overwrite", duplicate.stderr)


if __name__ == "__main__":
    unittest.main()
