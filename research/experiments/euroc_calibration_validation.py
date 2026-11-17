"""Fail-closed EuRoC stereo/IMU calibration geometry and provenance audit.

Never opens ground truth or camera/IMU samples; records source YAML hashes and
explicit sensor-to-IMU transforms. Does NOT run a VIO estimator or measure ATE.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml


SENSORS = ("imu0", "cam0", "cam1")
MAX_CALIBRATION_BYTES = 1_048_576
TRANSFORM_TOLERANCE = 1.0e-3


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def finite_number(value: object, field: str) -> float:
    require(not isinstance(value, bool), f"{field}: bool is not numeric")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field}: expected finite number") from exc
    require(math.isfinite(number), f"{field}: nonfinite number")
    return number


def matrix_to_list(matrix: np.ndarray) -> list[list[float]]:
    return [[float(x) for x in row] for row in matrix.tolist()]


def sensor_transform(sensor: str, data: dict) -> np.ndarray:
    block = data.get("T_BS")
    require(isinstance(block, dict), f"{sensor}: missing T_BS")
    require(block.get("rows") == 4 and block.get("cols") == 4,
            f"{sensor}: T_BS must be 4x4")
    values = block.get("data")
    require(isinstance(values, list) and len(values) == 16,
            f"{sensor}: T_BS requires 16 entries")
    matrix = np.asarray(
        [finite_number(x, f"{sensor}: T_BS") for x in values], dtype=float
    ).reshape((4, 4))
    require(np.allclose(matrix[3], [0, 0, 0, 1], rtol=0, atol=1e-8),
            f"{sensor}: invalid homogeneous bottom row")
    rotation = matrix[:3, :3]
    require(np.linalg.norm(rotation.T @ rotation - np.eye(3)) <=
            TRANSFORM_TOLERANCE,
            f"{sensor}: T_BS rotation not orthogonal")
    require(abs(float(np.linalg.det(rotation)) - 1) <= TRANSFORM_TOLERANCE,
            f"{sensor}: T_BS rotation not right handed")
    return matrix


def calibrated_camera(sensor: str, data: dict) -> dict:
    require(data.get("sensor_type") == "camera",
            f"{sensor}: expected camera sensor")
    require(data.get("camera_model") == "pinhole",
            f"{sensor}: unsupported camera model")
    require(data.get("distortion_model") == "radial-tangential",
            f"{sensor}: unsupported distortion model")
    intrinsics = data.get("intrinsics")
    distortion = data.get("distortion_coefficients")
    resolution = data.get("resolution")
    require(isinstance(intrinsics, list) and len(intrinsics) == 4,
            f"{sensor}: expected fx, fy, cx, cy")
    require(isinstance(distortion, list) and len(distortion) == 4,
            f"{sensor}: expected k1, k2, p1, p2")
    require(isinstance(resolution, list) and len(resolution) == 2,
            f"{sensor}: expected width, height")
    fx, fy, cx, cy = [
        finite_number(value, f"{sensor}: intrinsics") for value in intrinsics
    ]
    coeff = [
        finite_number(value, f"{sensor}: distortion") for value in distortion
    ]
    require(all(type(value) is int and 1 <= value <= 16384 for value in resolution),
            f"{sensor}: invalid resolution")
    width, height = resolution
    require(fx > 0 and fy > 0 and 0 <= cx < width and 0 <= cy < height,
            f"{sensor}: invalid focal lengths or principal point")
    rate = finite_number(data.get("rate_hz"), f"{sensor}: rate_hz")
    require(rate > 0, f"{sensor}: camera rate must be positive")
    return {
        "model": "pinhole",
        "distortion_model": "radial-tangential",
        "intrinsics_fx_fy_cx_cy": [fx, fy, cx, cy],
        "distortion_k1_k2_p1_p2": coeff,
        "resolution_width_height": [width, height],
        "rate_hz": rate,
    }


def calibrated_imu(data: dict) -> dict:
    require(data.get("sensor_type") == "imu", "imu0: expected IMU sensor")
    rate = finite_number(data.get("rate_hz"), "imu0: rate_hz")
    require(rate > 0, "imu0: invalid IMU rate")
    names = (
        "gyroscope_noise_density",
        "gyroscope_random_walk",
        "accelerometer_noise_density",
        "accelerometer_random_walk",
    )
    values = {}
    for name in names:
        value = finite_number(data.get(name), f"imu0: {name}")
        require(value > 0, f"imu0: {name} must be positive")
        values[name] = value
    return {"rate_hz": rate, **values}


def read_sensor(root: Path, sensor: str) -> tuple[dict, str]:
    directory = root / "mav0" / sensor
    require(directory.is_dir() and not directory.is_symlink(),
            f"{sensor}: missing regular sensor directory")
    path = directory / "sensor.yaml"
    require(path.is_file() and not path.is_symlink(),
            f"{sensor}: missing regular sensor.yaml")
    resolved = path.resolve(strict=True)
    require(resolved.is_relative_to(root),
            f"{sensor}: sensor.yaml outside sequence root")
    require(0 < resolved.stat().st_size <= MAX_CALIBRATION_BYTES,
            f"{sensor}: empty or oversized sensor.yaml")
    payload = resolved.read_bytes()
    digest = hashlib.sha256(payload).hexdigest()
    try:
        parsed = yaml.safe_load(payload.decode("utf-8-sig"))
    except (UnicodeDecodeError, yaml.YAMLError) as exc:
        raise ValueError(f"{sensor}: invalid safe YAML") from exc
    require(isinstance(parsed, dict), f"{sensor}: YAML must be a mapping")
    return parsed, digest


def audit(sequence_root: Path) -> dict:
    root = sequence_root.resolve(strict=True)
    require(root.is_dir() and (root / "mav0").is_dir() and
            not (root / "mav0").is_symlink(),
            "expected unpacked EuRoC sequence root with mav0")
    blocks = {}
    hashes = {}
    transforms = {}
    for sensor in SENSORS:
        block, digest = read_sensor(root, sensor)
        blocks[sensor] = block
        hashes[sensor] = digest
        transforms[sensor] = sensor_transform(sensor, block)

    imu = calibrated_imu(blocks["imu0"])
    cams = {sensor: calibrated_camera(sensor, blocks[sensor])
            for sensor in ("cam0", "cam1")}

    # EuRoC sensor.yaml defines T_BS: sensor S -> body B.
    # Express camera C in IMU frame I: T_I_C = inv(T_B_I) @ T_B_C.
    imu_to_body = transforms["imu0"]
    cam0_to_imu = np.linalg.solve(imu_to_body, transforms["cam0"])
    cam1_to_imu = np.linalg.solve(imu_to_body, transforms["cam1"])
    cam0_to_cam1 = np.linalg.solve(transforms["cam1"], transforms["cam0"])
    baseline_m = float(np.linalg.norm(
        transforms["cam0"][:3, 3] - transforms["cam1"][:3, 3]
    ))
    require(baseline_m > 1.0e-5, "degenerate stereo baseline")

    return {
        "schema_version": 1,
        "scope": "calibration_geometry_only_no_pose_or_accuracy_evidence",
        "sensor_reference": "EuRoC sensor.yaml T_BS (sensor to body)",
        "source_root": str(root),
        "source_yaml_sha256": hashes,
        "calibration_validated": True,
        "dataset_provenance_verified": False,
        "ground_truth_used": False,
        "camera_models": cams,
        "imu_model": imu,
        "T_body_imu": matrix_to_list(imu_to_body),
        "T_imu_cam0": matrix_to_list(cam0_to_imu),
        "T_imu_cam1": matrix_to_list(cam1_to_imu),
        "T_cam1_cam0": matrix_to_list(cam0_to_cam1),
        "stereo_baseline_m": baseline_m,
        "runtime_versions": {
            "numpy": np.__version__,
            "pyyaml": yaml.__version__,
        },
        "limitations": [
            "YAML numeric schema checked; independently verify source archive hash",
            "camera images and IMU measurements were not decoded or replayed",
            "camera distortion correction, rectification and time sync not performed",
            "no estimator, relative pose, localization metric or flight claim",
        ],
    }


def export(sequence_root: Path, output: Path) -> dict:
    result = audit(sequence_root)
    root = sequence_root.resolve(strict=True)
    require(not output.resolve().is_relative_to(root),
            "output must remain outside original sequence root")
    require(not output.exists() and not output.is_symlink(),
            "refusing to overwrite output or symlink")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write("\n")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sequence-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = export(args.sequence_root, args.output)
    print(f"Validated EuRoC stereo calibration; "
          f"camera baseline {result['stereo_baseline_m']:.6f} m. "
          "No localization accuracy or sensor replay was measured.")


if __name__ == "__main__":
    main()
