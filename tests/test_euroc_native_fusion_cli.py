"""End-to-end, synthetic-file CLI verification for native EuRoC VO + IMU replay.

No real EuRoC data, camera image or ground truth is used in these tests.
"""

import argparse
import csv
import subprocess
import tempfile
import unittest
from pathlib import Path


START = 1403636579763555584
VISUAL_COLUMNS = [
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
]


class NativeStereoImuCliTests(unittest.TestCase):
    binary = None

    def setUp(self):
        self.workspace = tempfile.TemporaryDirectory()
        self.addCleanup(self.workspace.cleanup)
        root = Path(self.workspace.name)
        self.imu = root / "source_sequence" / "mav0" / "imu0" / "data.csv"
        self.imu.parent.mkdir(parents=True)
        self.visual = root / "visual.csv"
        self.active = root / "active.csv"
        self.shadow = root / "shadow.csv"
        self.write_imu()
        self.write_visual()

    def write_imu(self, *, count=16, gyro=(0, 0, 0), accel=(0, 0, 9.81)):
        with self.imu.open("w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(
                [
                    "#timestamp [ns]",
                    "w_RS_S_x [rad s^-1]",
                    "w_RS_S_y [rad s^-1]",
                    "w_RS_S_z [rad s^-1]",
                    "a_RS_S_x [m s^-2]",
                    "a_RS_S_y [m s^-2]",
                    "a_RS_S_z [m s^-2]",
                ]
            )
            for n in range(count):
                writer.writerow([START + n * 5_000_000, *gyro, *accel])

    def write_visual(self, *, timestamp_offsets=None, bad_sigma=False):
        offsets = timestamp_offsets or [22_000_000, 50_000_000]
        with self.visual.open("w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(VISUAL_COLUMNS)
            for offset in offsets:
                writer.writerow(
                    [
                        START + offset,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        -0.5 if bad_sigma else 0.5,
                        0.75,
                        32,
                        0.25,
                        1.0,
                        0.0,
                        0.0,
                        0.0,
                        0.25,
                    ]
                )

    def run_native(self, *, active=None, shadow=None, gravity_aligned=False):
        self.assertIsNotNone(self.binary)
        return subprocess.run(
            [
                self.binary,
                "--imu-csv",
                str(self.imu),
                "--visual-csv",
                str(self.visual),
                "--active-output",
                str(active or self.active),
                "--shadow-output",
                str(shadow or self.shadow),
                (
                    "--require-stationary-gravity-alignment"
                    if gravity_aligned
                    else "--assume-initial-imu-identity"
                ),
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=15,
        )

    def test_two_fused_output_trajectories_and_interpolation(self):
        result = self.run_native()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("1 interpolated camera-time rows", result.stdout)
        self.assertIn("2 visual updates accepted", result.stdout)
        for path in (self.active, self.shadow):
            with path.open(newline="", encoding="utf-8") as file:
                rows = list(csv.DictReader(file))
            self.assertEqual(len(rows), 2)
            self.assertEqual(
                [int(r["timestamp_ns"]) for r in rows],
                [START + 22_000_000, START + 50_000_000],
            )
            for row in rows:
                self.assertTrue(
                    all(abs(float(row[k])) < 100 for k in ("x_m", "y_m", "z_m"))
                )
                self.assertAlmostEqual(float(row["qw"]), 1.0, places=3)
                self.assertAlmostEqual(float(row["qx"]), 0.0, places=3)

    def test_stationary_tilt_only_gravity_prior_succeeds_without_ground_truth(self):
        # 200 Hz and >200 ms of stationary pre-roll before the first VO update.
        # +30 degree tilted body: accel_body=(0,4.905,8.4957) in m/s^2.
        self.write_imu(count=101, accel=(0, 4.905, 8.495709211))
        self.write_visual(timestamp_offsets=[300_000_000, 400_000_000])
        result = self.run_native(gravity_aligned=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("stationary-gravity-aligned; yaw unobservable", result.stdout)
        for path in (self.active, self.shadow):
            with path.open(newline="", encoding="utf-8") as file:
                poses = list(csv.DictReader(file))
            self.assertEqual(len(poses), 2)
            self.assertGreater(abs(float(poses[-1]["qx"])), 0.10)

    def test_gravity_prior_rejects_motion_without_generating_trajectories(self):
        self.write_imu(count=101, gyro=(0.5, 0, 0))
        self.write_visual(timestamp_offsets=[300_000_000, 400_000_000])
        result = self.run_native(gravity_aligned=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("stationary gravity prior rejected motion", result.stderr)
        self.assertFalse(self.active.exists())
        self.assertFalse(self.shadow.exists())

    def test_gravity_prior_rejects_short_imu_preroll(self):
        result = self.run_native(gravity_aligned=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("at least 200 ms IMU pre-roll", result.stderr)
        self.assertFalse(self.active.exists())
        self.assertFalse(self.shadow.exists())

    def test_visual_out_of_order_is_rejected_without_outputs(self):
        self.write_visual(timestamp_offsets=[50_000_000, 22_000_000])
        result = self.run_native()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("nonmonotonic visual timestamps", result.stderr)
        self.assertFalse(self.active.exists())
        self.assertFalse(self.shadow.exists())

    def test_bad_visual_covariance_is_rejected(self):
        self.write_visual(bad_sigma=True)
        result = self.run_native()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("unsupported visual quality/sigma values", result.stderr)
        self.assertFalse(self.active.exists())

    def test_negative_pnp_reprojection_error_is_rejected(self):
        source = self.visual.read_text(encoding="utf-8")
        self.visual.write_text(
            source.replace(",32,0.25", ",32,-0.1"),
            encoding="utf-8",
        )
        result = self.run_native()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("unsupported visual quality/sigma values", result.stderr)
        self.assertFalse(self.active.exists())
        self.assertFalse(self.shadow.exists())

    def test_nonunit_visual_quaternion_is_rejected(self):
        source = self.visual.read_text(encoding="utf-8")
        self.visual.write_text(
            source.replace(",1.0,0.0,0.0,0.0,0.25", ",0.0,0.0,0.0,0.0,0.25"),
            encoding="utf-8",
        )
        result = self.run_native()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("invalid visual orientation", result.stderr)
        self.assertFalse(self.active.exists())
        self.assertFalse(self.shadow.exists())

    def test_synthetic_visual_yaw_corrects_both_native_attitudes(self):
        source = self.visual.read_text(encoding="utf-8")
        rows = source.splitlines()
        rows[-1] = rows[-1].replace(
            ",1.0,0.0,0.0,0.0,0.25",
            ",0.9987502604,0.0,0.0,0.0499791693,0.25",
        )
        self.visual.write_text("\n".join(rows) + "\n", encoding="utf-8")
        result = self.run_native()
        self.assertEqual(result.returncode, 0, result.stderr)
        for path in (self.active, self.shadow):
            with path.open(newline="", encoding="utf-8") as file:
                records = list(csv.DictReader(file))
            self.assertEqual(len(records), 2)
            self.assertGreater(float(records[-1]["qz"]), 0.001)
            self.assertLess(float(records[-1]["qz"]), 0.10)

    def test_visual_beyond_original_imu_window_is_rejected(self):
        self.write_visual(timestamp_offsets=[22_000_000, 100_000_000])
        result = self.run_native()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("outside original IMU support", result.stderr)
        self.assertFalse(self.active.exists())

    def test_existing_output_is_not_overwritten(self):
        self.active.write_text("existing result\n", encoding="utf-8")
        result = self.run_native()
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.active.read_text(), "existing result\n")
        self.assertFalse(self.shadow.exists())

    def test_native_export_cannot_modify_original_sequence(self):
        source_root = self.imu.parent.parent.parent
        forbidden = source_root / "overwritten_estimator.csv"
        result = self.run_native(active=forbidden)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("outside source sequence", result.stderr)
        self.assertFalse(forbidden.exists())
        self.assertFalse(self.shadow.exists())

    def test_original_imu_source_with_nonfinite_value_is_rejected(self):
        text = self.imu.read_text(encoding="utf-8")
        text = text.replace(",9.81", ",nan", 1)
        self.imu.write_text(text, encoding="utf-8")
        result = self.run_native()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("nonfinite", result.stderr)
        self.assertFalse(self.active.exists())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", required=True, type=Path)
    args = parser.parse_args()
    if not args.binary.is_file():
        parser.error("native binary must exist")
    NativeStereoImuCliTests.binary = str(args.binary.resolve())
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(NativeStereoImuCliTests)
    report = unittest.TextTestRunner(verbosity=2).run(suite)
    raise SystemExit(0 if report.wasSuccessful() else 1)


if __name__ == "__main__":
    main()
