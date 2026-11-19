"""Synthetic-only paired initialization ablation evidence contract tests."""

import csv
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from research.experiments.euroc_initialization_ablation import run
from research.experiments.euroc_stereo_visual_odometry import VISUAL_COLUMNS


STAMP = 1403636579800000000


class PairedInitializationAblationTests(unittest.TestCase):
    def setUp(self):
        self.work = tempfile.TemporaryDirectory()
        self.addCleanup(self.work.cleanup)
        self.root = Path(self.work.name)
        self.source = self.root / "original_sequence"
        imu = self.source / "mav0/imu0/data.csv"
        imu.parent.mkdir(parents=True)
        imu.write_text(
            "#timestamp [ns],w_RS_S_x,w_RS_S_y,w_RS_S_z,a_RS_S_x,a_RS_S_y,a_RS_S_z\n"
            f"{STAMP - 1000000},0,0,0,0,0,9.81\n"
            f"{STAMP},0,0,0,0,0,9.81\n",
            encoding="utf-8",
        )
        self.visual = self.root / "visual.csv"
        with self.visual.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=VISUAL_COLUMNS)
            writer.writeheader()
            for stamp in (STAMP, STAMP + 20000000):
                row = {k: "0" for k in VISUAL_COLUMNS}
                row["timestamp_ns"] = str(stamp)
                row["sigma_position_m"] = "0.5"
                row["sigma_velocity_mps"] = "0.75"
                row["pnp_inliers"] = "25"
                row["pnp_reprojection_rmse_px"] = "0.4"
                row["qw"] = "1"
                row["sigma_orientation_rad"] = "0.25"
                writer.writerow(row)
        self.out = self.root / "ablation"

    def fake_native(self, argv, **kwargs):
        self.assertTrue(kwargs["capture_output"])
        self.assertFalse(kwargs["check"])
        active = Path(argv[argv.index("--active-output") + 1])
        shadow = Path(argv[argv.index("--shadow-output") + 1])
        gravity = argv[-1] == "--require-stationary-gravity-alignment"
        prior = (
            "stationary-gravity-aligned; yaw unobservable"
            if gravity
            else "identity-assumed; gravity unverified"
        )
        for file in (active, shadow):
            with file.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(
                    ["timestamp_ns", "x_m", "y_m", "z_m", "qw", "qx", "qy", "qz"]
                )
                for stamp in (STAMP, STAMP + 20000000):
                    writer.writerow([stamp, 0.1 if gravity else 0, 0, 0, 1, 0, 0, 0])
        return subprocess.CompletedProcess(
            argv,
            0,
            stdout=f"Initial attitude prior: {prior}\n"
            "IMU: 12 original rows; 1 interpolated camera-time rows; "
            "2 visual updates accepted by both filters.\n",
            stderr="",
        )

    def test_valid_pair_preserves_hashes_and_differences_not_accuracy(self):
        with mock.patch(
            "research.experiments.euroc_initialization_ablation.subprocess.run",
            side_effect=self.fake_native,
        ) as native:
            record = run(self.source, self.visual, Path(sys.executable), self.out)
        self.assertEqual(native.call_count, 2)
        self.assertEqual(
            record["status"], "completed_unverified_paired_software_replay"
        )
        self.assertFalse(record["ground_truth_used"])
        self.assertFalse(record["publishable_accuracy_evidence"])
        self.assertEqual(record["sensitivity"]["active"]["sample_count"], 2)
        self.assertAlmostEqual(
            record["sensitivity"]["active"]["position_difference_rms_m"], 0.1
        )
        self.assertFalse(record["sensitivity"]["active"]["is_accuracy_metric"])
        self.assertEqual(len(record["input_sha256"]["visual_csv_sha256"]), 64)
        self.assertEqual(
            json.loads((self.out / "ablation_manifest.json").read_text())["status"],
            "completed_unverified_paired_software_replay",
        )

    def test_first_pose_alignment_removes_only_rigid_frame_gauge(self):
        # The second pose follows the same trajectory in two different world
        # frames: +90-degree global yaw and arbitrary translation.
        def rigid_frame_replay(argv, **kwargs):
            proc = self.fake_native(argv, **kwargs)
            gravity = argv[-1] == "--require-stationary-gravity-alignment"
            for index, field in enumerate(("--active-output", "--shadow-output")):
                path = Path(argv[argv.index(field) + 1])
                with path.open("w", newline="", encoding="utf-8") as stream:
                    writer = csv.writer(stream)
                    writer.writerow(
                        ["timestamp_ns", "x_m", "y_m", "z_m", "qw", "qx", "qy", "qz"]
                    )
                    for i, stamp in enumerate((STAMP, STAMP + 20000000)):
                        if gravity:
                            writer.writerow(
                                [
                                    stamp,
                                    3,
                                    2 + 0.4 * i,
                                    0,
                                    0.7071067811865476,
                                    0,
                                    0,
                                    0.7071067811865476,
                                ]
                            )
                        else:
                            writer.writerow([stamp, 0.4 * i, 0, 0, 1, 0, 0, 0])
            return proc

        with mock.patch(
            "research.experiments.euroc_initialization_ablation.subprocess.run",
            side_effect=rigid_frame_replay,
        ):
            result = run(self.source, self.visual, Path(sys.executable), self.out)
        for key in ("active", "shadow"):
            metric = result["sensitivity"][key]
            self.assertGreater(metric["position_difference_rms_m"], 1.0)
            self.assertGreater(metric["orientation_difference_rms_rad"], 1.0)
            self.assertLess(
                metric["first_pose_aligned_position_difference_rms_m"], 1e-9
            )
            self.assertLess(
                metric["first_pose_aligned_orientation_difference_rms_rad"], 1e-7
            )
            self.assertFalse(metric["is_accuracy_metric"])

    def test_rigid_alignment_does_not_erase_true_relative_divergence(self):
        def drifted_mode(argv, **kwargs):
            proc = self.fake_native(argv, **kwargs)
            gravity = argv[-1] == "--require-stationary-gravity-alignment"
            if gravity:
                path = Path(argv[argv.index("--active-output") + 1])
                with path.open("w", newline="", encoding="utf-8") as stream:
                    writer = csv.writer(stream)
                    writer.writerow(
                        ["timestamp_ns", "x_m", "y_m", "z_m", "qw", "qx", "qy", "qz"]
                    )
                    writer.writerow([STAMP, 0.1, 0, 0, 1, 0, 0, 0])
                    writer.writerow([STAMP + 20000000, 0.5, 0, 0, 1, 0, 0, 0])
            return proc

        with mock.patch(
            "research.experiments.euroc_initialization_ablation.subprocess.run",
            side_effect=drifted_mode,
        ):
            result = run(self.source, self.visual, Path(sys.executable), self.out)
        self.assertAlmostEqual(
            result["sensitivity"]["active"][
                "first_pose_aligned_position_difference_rms_m"
            ],
            (0.4**2 / 2) ** 0.5,
            places=7,
        )
        self.assertLess(
            result["sensitivity"]["shadow"][
                "first_pose_aligned_position_difference_rms_m"
            ],
            1e-9,
        )

    def test_stationarity_rejection_marks_entire_pair_failed(self):
        def reject_motion(argv, **kwargs):
            if argv[-1] == "--require-stationary-gravity-alignment":
                return subprocess.CompletedProcess(
                    argv, 1, stdout="", stderr="stationarity rejected"
                )
            return self.fake_native(argv, **kwargs)

        with mock.patch(
            "research.experiments.euroc_initialization_ablation.subprocess.run",
            side_effect=reject_motion,
        ):
            with self.assertRaisesRegex(
                ValueError, "stationary_gravity native replay failed"
            ):
                run(self.source, self.visual, Path(sys.executable), self.out)
        result = json.loads((self.out / "ablation_manifest.json").read_text())
        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["mode_results"]["stationary_gravity"]["exit_code"], 1)
        self.assertFalse(result["publishable_accuracy_evidence"])

    def test_paired_timestamps_must_match(self):
        def shorten_shadow(argv, **kwargs):
            proc = self.fake_native(argv, **kwargs)
            if argv[-1] == "--require-stationary-gravity-alignment":
                shadow = Path(argv[argv.index("--shadow-output") + 1])
                shadow.write_text(
                    "timestamp_ns,x_m,y_m,z_m,qw,qx,qy,qz\n" f"{STAMP},0,0,0,1,0,0,0\n",
                    encoding="utf-8",
                )
            return proc

        with mock.patch(
            "research.experiments.euroc_initialization_ablation.subprocess.run",
            side_effect=shorten_shadow,
        ):
            with self.assertRaisesRegex(ValueError, "lost, duplicated or reordered"):
                run(self.source, self.visual, Path(sys.executable), self.out)
        self.assertEqual(
            json.loads((self.out / "ablation_manifest.json").read_text())["status"],
            "failed",
        )

    def test_refuses_existing_ablation_output(self):
        self.out.mkdir()
        original = self.out / "untouched"
        original.write_text("keep", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "cannot overwrite"):
            run(self.source, self.visual, Path(sys.executable), self.out)
        self.assertEqual(original.read_text(encoding="utf-8"), "keep")

    def test_refuses_visual_csv_from_original_source_tree(self):
        forbidden = self.source / "visual.csv"
        forbidden.write_bytes(self.visual.read_bytes())
        with self.assertRaisesRegex(ValueError, "outside the original source"):
            run(self.source, forbidden, Path(sys.executable), self.out)
        self.assertFalse(self.out.exists())

    def test_rejects_nonmonotonic_visual_csv_before_creating_output(self):
        rows = self.visual.read_text(encoding="utf-8").splitlines()
        rows[-1] = rows[-2]
        self.visual.write_text("\n".join(rows) + "\n", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "not increasing"):
            run(self.source, self.visual, Path(sys.executable), self.out)
        self.assertFalse(self.out.exists())


if __name__ == "__main__":
    unittest.main()
