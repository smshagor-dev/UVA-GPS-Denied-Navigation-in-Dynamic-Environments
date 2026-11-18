"""Only synthetic fixtures; no original EuRoC data or accuracy results."""

import csv
import json
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from research.experiments.euroc_pose_evaluation import Pose
from research.experiments.euroc_pose_sensitivity import (
    compare_offsets,
    parse_offsets_ms,
)
from tests.test_euroc_pose_evaluation import trajectories


class EurocPoseSensitivityTests(unittest.TestCase):
    def test_rejects_posthoc_or_invalid_clock_grids(self):
        self.assertEqual(
            parse_offsets_ms("-20,-10,0,10,20"),
            [-20_000_000, -10_000_000, 0, 10_000_000, 20_000_000],
        )
        self.assertEqual(parse_offsets_ms("-0.000001,0,0.000001"), [-1, 0, 1])
        for bad in (
            "-10,0",
            "-10,0,0,10",
            "-10,-20,0,10",
            "-10,0,5,100",
            "-10,0,nan",
            "-10,0,",
            "-10,0,inf",
            "-0.0000001,0,10",
            "0,10,20",
            "-10,-5,0",
            "-10,0,0",
        ):
            with self.subTest(grid=bad), self.assertRaises(ValueError):
                parse_offsets_ms(bad)

    def test_identical_estimates_are_paired_without_best_offset_selection(self):
        gt, estimates = trajectories()
        grid = parse_offsets_ms("-20,-10,0,10,20")
        result = compare_offsets(
            gt,
            estimates,
            estimates,
            grid,
            tolerance_ns=25_000_000,
        )
        self.assertEqual(result["status"], "completed")
        self.assertTrue(result["zero_offset_evaluated"])
        self.assertFalse(result["best_offset_selected"])
        self.assertFalse(result["ground_truth_used_for_estimator"])
        self.assertFalse(result["independent_preregistration_verified"])
        self.assertEqual(len(result["records"]), 5)
        for record in result["records"]:
            self.assertEqual(record["status"], "evaluated")
            self.assertEqual(record["active"]["matched_samples"], 12)
            self.assertEqual(record["shadow"]["matched_samples"], 12)
            self.assertLess(abs(record["active_minus_shadow"]["ate_rmse_m"]), 1e-12)
            self.assertLess(
                abs(record["active_minus_shadow"]["relative_orientation_rmse_deg"]),
                1e-12,
            )

    def test_orientation_drift_comparison_is_not_hidden_by_position_alignment(self):
        gt, active = trajectories()
        shadow = [
            Pose(
                pose.timestamp_ns,
                pose.xyz,
                (
                    math.cos((-math.pi / 2 + 0.07 * index + 0.035 * index) / 2),
                    0.0,
                    0.0,
                    math.sin((-math.pi / 2 + 0.07 * index + 0.035 * index) / 2),
                ),
            )
            for index, pose in enumerate(active)
        ]
        result = compare_offsets(
            gt,
            active,
            shadow,
            parse_offsets_ms("-10,0,10"),
            tolerance_ns=20_000_000,
        )
        baseline = result["records"][1]
        self.assertEqual(baseline["offset_ns"], 0)
        self.assertLess(
            baseline["active"]["absolute_orientation_error"]["rmse_deg"], 1e-4
        )
        self.assertGreater(
            baseline["shadow"]["absolute_orientation_error"]["rmse_deg"], 1.0
        )
        self.assertLess(
            baseline["active_minus_shadow"]["absolute_orientation_rmse_deg"], -1.0
        )

    def test_refuses_unpaired_native_estimator_timestamps(self):
        gt, active = trajectories()
        shadow = list(active)
        last = shadow[-1]
        shadow[-1] = Pose(last.timestamp_ns + 1, last.xyz, last.q_wxyz)
        with self.assertRaisesRegex(ValueError, "timestamps differ"):
            compare_offsets(gt, active, shadow, parse_offsets_ms("-10,0,10"))

    def test_failed_offsets_are_not_dropped_or_cherry_picked(self):
        gt, estimates = trajectories()
        result = compare_offsets(
            gt,
            estimates,
            estimates,
            parse_offsets_ms("-20,0,20"),
            tolerance_ns=20_000_000,
        )
        self.assertEqual(result["status"], "incomplete")
        self.assertTrue(result["zero_offset_evaluated"])
        self.assertEqual(len(result["records"]), 3)
        self.assertEqual(result["records"][-1]["status"], "failed")
        self.assertIn("only 0 matched", result["records"][-1]["failure_reason"])

    def test_cli_preserves_original_tree_and_writes_hashes(self):
        gt, est = trajectories()
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            original = root / "sequence"
            gt_dir = original / "mav0" / "state_groundtruth_estimate0"
            gt_dir.mkdir(parents=True)
            gt_csv = gt_dir / "data.csv"
            with gt_csv.open("w", newline="", encoding="utf-8") as file:
                writer = csv.writer(file)
                writer.writerow(
                    ["#timestamp", "p_x", "p_y", "p_z", "q_w", "q_x", "q_y", "q_z"]
                )
                for p in gt:
                    writer.writerow((p.timestamp_ns, *p.xyz, *p.q_wxyz))
            csv_paths = []
            for name in ("active.csv", "shadow.csv"):
                path = root / name
                csv_paths.append(path)
                with path.open("w", newline="", encoding="utf-8") as file:
                    writer = csv.writer(file)
                    writer.writerow(
                        ["timestamp_ns", "x_m", "y_m", "z_m", "qw", "qx", "qy", "qz"]
                    )
                    for p in est:
                        writer.writerow((p.timestamp_ns, *p.xyz, *p.q_wxyz))
            output = root / "results" / "sensitivity.json"
            command = [
                sys.executable,
                "-m",
                "research.experiments.euroc_pose_sensitivity",
                "--sequence-root",
                str(original),
                "--sequence",
                "synthetic-fixture-only",
                "--active-estimated",
                str(csv_paths[0]),
                "--shadow-estimated",
                str(csv_paths[1]),
                "--frame-agreement",
                "same-imu-body",
                "--offsets-ms=-20,-10,0,10,20",
                "--tolerance-ms",
                "25",
                "--output",
                str(output),
            ]
            success = subprocess.run(
                command, check=True, capture_output=True, text=True
            )
            self.assertIn("5/5 prespecified offsets evaluated", success.stdout)
            data = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(data["status"], "completed")
            self.assertEqual(data["requested_offsets"], 5)
            self.assertEqual(len(data["source_sha256"]), 3)
            self.assertEqual(len(list(original.rglob("*.json"))), 0)
            repeat = subprocess.run(command, capture_output=True, text=True)
            self.assertNotEqual(repeat.returncode, 0)
            self.assertIn("refusing to overwrite", repeat.stderr)
            incomplete = subprocess.run(
                [
                    *command[:-4],
                    "--tolerance-ms",
                    "20",
                    "--output",
                    str(root / "incomplete.json"),
                ],
                capture_output=True,
                text=True,
            )
            # The grid must not silently report success if one offset fails.
            self.assertNotEqual(incomplete.returncode, 0)


if __name__ == "__main__":
    unittest.main()
