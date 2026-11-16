"""Synthetic-fixture tests ONLY: never interpret them as EuRoC dataset results."""

import csv
import tempfile
import unittest
from pathlib import Path

import numpy as np

from research.experiments.euroc_trajectory_evaluation import (
    Position, associate, evaluate, load_estimator_trajectory,
    load_euroc_ground_truth, rigid_align,
)


START_NS = 1403636579763555584


def trajectories(count=12):
    ground_truth, estimated = [], []
    for index in range(count):
        gt = (float(index), float(index % 3), float((index * index) % 5))
        # Same shape rotated 90 degrees and shifted. No scale change.
        est = (-gt[1] + 12.0, gt[0] - 8.0, gt[2] + 5.0)
        timestamp = START_NS + index * 1_000_000_000
        ground_truth.append(Position(timestamp, gt))
        estimated.append(Position(timestamp + 1_000_000, est))
    return ground_truth, estimated


class EurocTrajectoryEvaluationTests(unittest.TestCase):
    def test_known_rigid_offset_has_zero_translation_ate(self):
        gt, est = trajectories()
        result = evaluate(gt, est, tolerance_ns=2_000_000,
                          min_pairs=10, min_overlap_s=5.0)
        self.assertEqual(result["matched_samples"], 12)
        self.assertEqual(result["alignment"], "SE3_rigid_no_scale")
        self.assertLess(result["ate_translation"]["rmse_m"], 1e-10)
        self.assertLess(
            result["world_frame_relative_displacement_error"]["rmse_m"], 1e-10)
        self.assertFalse(result["provenance_verified"])

    def test_metric_scale_error_cannot_be_fitted_away(self):
        gt, _ = trajectories()
        est = [Position(p.timestamp_ns, tuple(1.1 * v for v in p.xyz)) for p in gt]
        result = evaluate(gt, est, min_pairs=10)
        self.assertGreater(result["ate_translation"]["rmse_m"], 0.05)

    def test_missing_overlap_is_rejected(self):
        gt, est = trajectories()
        with self.assertRaisesRegex(ValueError, "only 0 matched"):
            evaluate(gt, [Position(p.timestamp_ns + 99_000_000, p.xyz)
                          for p in est], tolerance_ns=2_000_000)

    def test_no_duplicate_ground_truth_association(self):
        gt, est = trajectories()
        repeated = [Position(est[0].timestamp_ns + offset, est[0].xyz)
                    for offset in (0, 1, 2)]
        self.assertEqual(len(associate(gt, repeated, 2_000_000)), 1)

    def test_collinear_alignment_is_rejected(self):
        linear = np.array([[float(i), 0.0, 0.0] for i in range(10)])
        with self.assertRaisesRegex(ValueError, "degenerate geometry"):
            rigid_align(linear, linear.copy())

    def test_ground_truth_euroc_ns_schema(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "data.csv"
            path.write_text(
                "#timestamp,p_x,p_y,p_z,q_w,q_x,q_y,q_z,other\n"
                + f"{START_NS},1.0,2.0,3.0,1,0,0,0,0\n"
                + f"{START_NS + 1_000_000_000},2.0,3.0,4.0,1,0,0,0,0\n",
                encoding="utf-8")
            rows = load_euroc_ground_truth(path)
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0].xyz, (1.0, 2.0, 3.0))
            self.assertEqual(rows[1].timestamp_ns, START_NS + 1_000_000_000)

    def test_estimator_csv_rejects_duplicate_timestamp(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "pose.csv"
            with path.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(["timestamp_ns", "x_m", "y_m", "z_m"])
                writer.writerow([START_NS, 1, 2, 3])
                writer.writerow([START_NS, 2, 3, 4])
            with self.assertRaisesRegex(ValueError, "strictly increasing"):
                load_estimator_trajectory(path)

    def test_estimator_rejects_nan_and_ambiguous_columns(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "pose.csv"
            path.write_text(f"timestamp_ns,x_m,y_m,z_m\n{START_NS},nan,2,3\n",
                            encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "non-finite position"):
                load_estimator_trajectory(path)
            path.write_text(f"timestamp_ns,tx,ty,tz\n{START_NS},1,2,3\n",
                            encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "estimator CSV columns"):
                load_estimator_trajectory(path)


if __name__ == "__main__":
    unittest.main()
