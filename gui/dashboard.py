#!/usr/bin/env python3
# System Designer and Developer: Md Shahanur Islam Shagor
# Project: UVA GPS Denied Navigation in Dynamic Environments
# Technology: C++, Python, Go, CMake

"""Realtime monitoring and control dashboard for the drone swarm project."""

from __future__ import annotations

import argparse
import json
import logging
from logging.handlers import RotatingFileHandler
import math
import os
import queue
import random
import sqlite3
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any
from urllib import error, parse, request

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from PySide6.QtCore import QThread, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QPalette
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QStatusBar,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)


DARK_BG = "#05090F"
PANEL_BG = "#0B111E"
PANEL_ALT = "#121A2B"
BORDER = "#1E2A3B"
TEXT = "#F8FAFC"
TEXT_DIM = "#8A9CB3"
ACCENT = "#00D2FF"
ACCENT_ALT = "#FF6B00"
SUCCESS = "#10B981"
WARN = "#F59E0B"
DANGER = "#EF4444"
CYAN = "#06B6D4"
MAGENTA = "#D946EF" 


logger = logging.getLogger(__name__)
LOG_DIR = Path(__file__).resolve().parent.parent / "logs" / "dashboard"
DATA_DIR = Path(__file__).resolve().parent.parent / "data" / "dashboard"
STORE_PATH = DATA_DIR / "dashboard.sqlite3"


def load_env_file(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    if not path.exists():
        return values
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip().strip('"').strip("'")
        if key:
            values[key] = value
    return values


def bootstrap_env() -> None:
    root = Path(__file__).resolve().parent.parent
    for env_path in (root / ".env", root / ".env.local"):
        for key, value in load_env_file(env_path).items():
            os.environ.setdefault(key, value)


DRONE_ENV_FIELDS = [
    ("enable_imu", "DRONE_ENABLE_IMU", "true"),
    ("imu_device", "DRONE_IMU_DEVICE", "/dev/i2c-1"),
    ("imu_addr", "DRONE_IMU_ADDR", "104"),
    ("enable_camera", "DRONE_ENABLE_CAMERA", "true"),
    ("esp32_ip", "DRONE_ESP32_IP", "192.168.4.1"),
    ("camera_stream_url", "DRONE_CAMERA_STREAM_URL", ""),
    ("enable_lidar", "DRONE_ENABLE_LIDAR", "true"),
    ("lidar_endpoint", "DRONE_LIDAR_ENDPOINT", "192.168.1.201:2368"),
    ("enable_barometer", "DRONE_ENABLE_BAROMETER", "true"),
    ("enable_motor", "DRONE_ENABLE_MOTOR", "true"),
    ("enable_optical_flow", "DRONE_ENABLE_OPTICAL_FLOW", "true"),
    ("enable_rangefinder", "DRONE_ENABLE_RANGEFINDER", "true"),
    ("enable_tdoa_ingestor", "DRONE_ENABLE_TDOA_INGESTOR", "true"),
    ("tdoa_udp_port", "DRONE_TDOA_UDP_PORT", "0"),
    ("tdoa_csv", "DRONE_TDOA_CSV", ""),
    ("enable_uwb_serial", "DRONE_ENABLE_UWB_SERIAL", "true"),
    ("tdoa_serial", "DRONE_TDOA_SERIAL", "COM5"),
]


def drone_env_key(drone_id: int, suffix: str) -> str:
    return f"DRONE_{drone_id}_{suffix}"


class DroneEnvManager:
    def __init__(self, path: Path) -> None:
        self._path = path

    def _load_values(self) -> dict[str, str]:
        return load_env_file(self._path)

    def _save_values(self, values: dict[str, str]) -> None:
        lines = [f"{key}={value}" for key, value in values.items()]
        self._path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    def dashboard_ids(self) -> list[int]:
        values = self._load_values()
        return [
            safe_int(part.strip(), -1)
            for part in safe_text(values.get("DRONE_DASHBOARD_IDS", ""), "").split(",")
            if safe_int(part.strip(), -1) > 0
        ]

    def update_dashboard_ids(self, drone_ids: list[int]) -> None:
        values = self._load_values()
        values["DRONE_DASHBOARD_IDS"] = ",".join(str(drone_id) for drone_id in sorted(set(drone_ids)))
        self._save_values(values)

    def drone_config(self, drone_id: int) -> dict[str, str]:
        values = self._load_values()
        config: dict[str, str] = {}
        for field_name, shared_key, default in DRONE_ENV_FIELDS:
            key = drone_env_key(drone_id, shared_key.removeprefix("DRONE_"))
            config[field_name] = safe_text(values.get(key, values.get(shared_key, default)), default)
        config["drone_id"] = str(drone_id)
        return config

    def upsert_drone(self, drone_id: int, config: dict[str, str], update_dashboard_ids: bool = True) -> None:
        values = self._load_values()
        values["DRONE_NODE_ID"] = values.get("DRONE_NODE_ID", str(drone_id))
        for field_name, shared_key, default in DRONE_ENV_FIELDS:
            suffix = shared_key.removeprefix("DRONE_")
            values[drone_env_key(drone_id, suffix)] = safe_text(config.get(field_name, values.get(shared_key, default)), default)
        if update_dashboard_ids:
            ids = self.dashboard_ids()
            if drone_id not in ids:
                ids.append(drone_id)
            values["DRONE_DASHBOARD_IDS"] = ",".join(str(item) for item in sorted(set(ids)))
        self._save_values(values)

    def remove_drone(self, drone_id: int, update_dashboard_ids: bool = True) -> None:
        values = self._load_values()
        prefix = f"DRONE_{drone_id}_"
        for key in [key for key in values if key.startswith(prefix)]:
            values.pop(key, None)
        if update_dashboard_ids:
            ids = [item for item in self.dashboard_ids() if item != drone_id]
            values["DRONE_DASHBOARD_IDS"] = ",".join(str(item) for item in sorted(set(ids)))
        self._save_values(values)


class AddDroneDialog(QDialog):
    def __init__(self, env_manager: DroneEnvManager, suggested_id: int, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._env_manager = env_manager
        self._demo_mode = False
        self.setWindowTitle("Add Drone")
        self.resize(520, 640)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(12)

        intro = QLabel("Each drone keeps its own sensor connection profile. Fill the drone-wise sensor endpoints below.")
        intro.setWordWrap(True)
        intro.setStyleSheet(f"color:{TEXT_DIM};")
        layout.addWidget(intro)

        form = QGridLayout()
        form.setHorizontalSpacing(10)
        form.setVerticalSpacing(10)

        self._drone_id = QSpinBox()
        self._drone_id.setRange(1, 999)
        self._drone_id.setValue(max(1, suggested_id))
        self._drone_id.valueChanged.connect(self._load_drone_defaults)
        form.addWidget(QLabel("Drone ID"), 0, 0)
        form.addWidget(self._drone_id, 0, 1)

        self._widgets: dict[str, Any] = {}

        def add_line(row: int, label: str, key: str) -> None:
            widget = QLineEdit()
            widget.setStyleSheet(f"background:{PANEL_ALT}; color:{TEXT}; border:1px solid {BORDER}; padding:6px;")
            self._widgets[key] = widget
            form.addWidget(QLabel(label), row, 0)
            form.addWidget(widget, row, 1)

        def add_check(row: int, label: str, key: str) -> None:
            widget = QCheckBox(label)
            widget.setChecked(True)
            widget.setStyleSheet(f"color:{TEXT};")
            self._widgets[key] = widget
            form.addWidget(widget, row, 0, 1, 2)

        row = 1
        add_check(row, "Enable IMU", "enable_imu"); row += 1
        add_line(row, "IMU Device", "imu_device"); row += 1
        add_line(row, "IMU Address", "imu_addr"); row += 1
        add_check(row, "Enable Camera", "enable_camera"); row += 1
        add_line(row, "ESP32 IP", "esp32_ip"); row += 1
        add_line(row, "Camera Stream URL", "camera_stream_url"); row += 1
        add_check(row, "Enable LiDAR", "enable_lidar"); row += 1
        add_line(row, "LiDAR Endpoint", "lidar_endpoint"); row += 1
        add_check(row, "Enable Barometer", "enable_barometer"); row += 1
        add_check(row, "Enable Motor Health", "enable_motor"); row += 1
        add_check(row, "Enable Optical Flow", "enable_optical_flow"); row += 1
        add_check(row, "Enable Rangefinder", "enable_rangefinder"); row += 1
        add_check(row, "Enable TDOA Ingestor", "enable_tdoa_ingestor"); row += 1
        add_line(row, "TDOA UDP Port", "tdoa_udp_port"); row += 1
        add_line(row, "TDOA CSV Path", "tdoa_csv"); row += 1
        add_check(row, "Enable UWB Serial", "enable_uwb_serial"); row += 1
        add_line(row, "UWB / TDOA Serial", "tdoa_serial"); row += 1

        layout.addLayout(form)

        buttons = QDialogButtonBox()
        self._demo_button = buttons.addButton("Add Demo Drone", QDialogButtonBox.ActionRole)
        self._add_button = buttons.addButton("Add Drone", QDialogButtonBox.AcceptRole)
        self._cancel_button = buttons.addButton(QDialogButtonBox.Cancel)
        self._demo_button.clicked.connect(self._accept_demo)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

        self._load_drone_defaults(self._drone_id.value())

    def _load_drone_defaults(self, drone_id: int) -> None:
        config = self._env_manager.drone_config(drone_id)
        for field_name, _shared_key, default in DRONE_ENV_FIELDS:
            widget = self._widgets[field_name]
            value = config.get(field_name, default)
            if isinstance(widget, QCheckBox):
                widget.setChecked(safe_text(value, default).lower() in {"1", "true", "yes", "on"})
            else:
                widget.setText(safe_text(value, default))

    def _accept_demo(self) -> None:
        self._demo_mode = True
        self.accept()

    def payload(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "drone_id": int(self._drone_id.value()),
            "demo_drone": self._demo_mode,
        }
        for field_name, _shared_key, _default in DRONE_ENV_FIELDS:
            widget = self._widgets[field_name]
            if isinstance(widget, QCheckBox):
                payload[field_name] = "true" if widget.isChecked() else "false"
            else:
                payload[field_name] = widget.text().strip()
        return payload


def safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def safe_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def safe_text(value: Any, default: str = "") -> str:
    if value is None:
        return default
    try:
        text = str(value).strip()
    except Exception:
        return default
    return text or default


def safe_vector3(values: Any, default: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> tuple[float, float, float]:
    if not isinstance(values, (list, tuple)) or len(values) < 3:
        return default
    return (
        safe_float(values[0], default[0]),
        safe_float(values[1], default[1]),
        safe_float(values[2], default[2]),
    )


def vector_norm3(values: tuple[float, float, float]) -> float:
    return math.sqrt(values[0] * values[0] + values[1] * values[1] + values[2] * values[2])


def derive_attitude_thrust(
    position: tuple[float, float, float],
    velocity: tuple[float, float, float],
    role: str,
    elapsed: float = 0.0,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    lateral_speed = math.hypot(velocity[0], velocity[1])
    yaw = math.atan2(velocity[1], velocity[0]) if lateral_speed > 1e-6 else 0.0
    pitch = math.atan2(velocity[0], 9.81)
    roll = math.atan2(-velocity[1], 9.81)
    climb_bias = 0.12 * math.sin(elapsed * 0.8 + position[2]) if role == "LEADER" else 0.08 * math.cos(elapsed * 0.6 + position[0])
    thrust = (
        velocity[0] * 0.9,
        velocity[1] * 0.9,
        9.81 + velocity[2] * 1.4 + climb_bias,
    )
    return (roll, pitch, yaw), thrust


class DashboardStore:
    def __init__(self, path: Path) -> None:
        self._path = path
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._init_db()

    def _connect(self) -> sqlite3.Connection:
        conn = sqlite3.connect(self._path)
        conn.row_factory = sqlite3.Row
        return conn

    def _init_db(self) -> None:
        with self._connect() as conn:
            conn.executescript(
                """
                CREATE TABLE IF NOT EXISTS settings (
                    key TEXT PRIMARY KEY,
                    value TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS command_history (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    created_at REAL NOT NULL,
                    action TEXT NOT NULL,
                    payload_json TEXT NOT NULL,
                    result_text TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS latest_snapshot_meta (
                    singleton_id INTEGER PRIMARY KEY CHECK (singleton_id = 1),
                    backend_mode TEXT NOT NULL,
                    leader_id INTEGER,
                    avg_latency_ms REAL NOT NULL,
                    packet_loss_pct REAL NOT NULL,
                    cpu_temp_c REAL NOT NULL,
                    gpu_load_pct REAL NOT NULL,
                    election_state TEXT NOT NULL,
                    timestamp REAL NOT NULL
                );
                CREATE TABLE IF NOT EXISTS latest_snapshot_states (
                    drone_id INTEGER PRIMARY KEY,
                    cluster_id TEXT NOT NULL,
                    role TEXT NOT NULL,
                    mission_state TEXT NOT NULL,
                    pos_x REAL NOT NULL,
                    pos_y REAL NOT NULL,
                    pos_z REAL NOT NULL,
                    vel_x REAL NOT NULL,
                    vel_y REAL NOT NULL,
                    vel_z REAL NOT NULL,
                    attitude_roll REAL NOT NULL DEFAULT 0,
                    attitude_pitch REAL NOT NULL DEFAULT 0,
                    attitude_yaw REAL NOT NULL DEFAULT 0,
                    thrust_x REAL NOT NULL DEFAULT 0,
                    thrust_y REAL NOT NULL DEFAULT 0,
                    thrust_z REAL NOT NULL DEFAULT 9.81,
                    drift_m REAL NOT NULL,
                    battery_pct REAL NOT NULL,
                    connectivity TEXT NOT NULL,
                    reachable INTEGER NOT NULL,
                    rssi_dbm REAL NOT NULL,
                    cpu_temp_c REAL NOT NULL,
                    gpu_load_pct REAL NOT NULL,
                    localization_source TEXT NOT NULL,
                    localization_state TEXT NOT NULL,
                    localization_confidence REAL NOT NULL,
                    tdoa_confidence REAL NOT NULL,
                    confidence_trend REAL NOT NULL,
                    relocalization_count INTEGER NOT NULL,
                    visible_anchor_count INTEGER NOT NULL,
                    occupancy_ratio REAL NOT NULL,
                    sync_confidence REAL NOT NULL,
                    imu_camera_offset_ms REAL NOT NULL,
                    motor_health REAL NOT NULL,
                    leadership_score REAL NOT NULL,
                    election_ready INTEGER NOT NULL,
                    timestamp REAL NOT NULL
                );
                """
            )
            for statement in (
                "ALTER TABLE latest_snapshot_states ADD COLUMN attitude_roll REAL NOT NULL DEFAULT 0",
                "ALTER TABLE latest_snapshot_states ADD COLUMN attitude_pitch REAL NOT NULL DEFAULT 0",
                "ALTER TABLE latest_snapshot_states ADD COLUMN attitude_yaw REAL NOT NULL DEFAULT 0",
                "ALTER TABLE latest_snapshot_states ADD COLUMN thrust_x REAL NOT NULL DEFAULT 0",
                "ALTER TABLE latest_snapshot_states ADD COLUMN thrust_y REAL NOT NULL DEFAULT 0",
                "ALTER TABLE latest_snapshot_states ADD COLUMN thrust_z REAL NOT NULL DEFAULT 9.81",
            ):
                try:
                    conn.execute(statement)
                except sqlite3.OperationalError:
                    pass

    def load_settings(self) -> dict[str, str]:
        with self._connect() as conn:
            rows = conn.execute("SELECT key, value FROM settings").fetchall()
        return {str(row["key"]): str(row["value"]) for row in rows}

    def save_settings(self, values: dict[str, str]) -> None:
        with self._connect() as conn:
            conn.executemany(
                "INSERT INTO settings(key, value) VALUES(?, ?) "
                "ON CONFLICT(key) DO UPDATE SET value=excluded.value",
                [(key, value) for key, value in values.items()],
            )

    def save_snapshot(self, snapshot: "DashboardSnapshot") -> None:
        with self._connect() as conn:
            conn.execute("DELETE FROM latest_snapshot_states")
            conn.execute(
                """
                INSERT INTO latest_snapshot_meta(
                    singleton_id, backend_mode, leader_id, avg_latency_ms, packet_loss_pct,
                    cpu_temp_c, gpu_load_pct, election_state, timestamp
                ) VALUES(1, ?, ?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(singleton_id) DO UPDATE SET
                    backend_mode=excluded.backend_mode,
                    leader_id=excluded.leader_id,
                    avg_latency_ms=excluded.avg_latency_ms,
                    packet_loss_pct=excluded.packet_loss_pct,
                    cpu_temp_c=excluded.cpu_temp_c,
                    gpu_load_pct=excluded.gpu_load_pct,
                    election_state=excluded.election_state,
                    timestamp=excluded.timestamp
                """,
                (
                    snapshot.backend_mode,
                    snapshot.leader_id,
                    snapshot.avg_latency_ms,
                    snapshot.packet_loss_pct,
                    snapshot.cpu_temp_c,
                    snapshot.gpu_load_pct,
                    snapshot.election_state,
                    snapshot.timestamp,
                ),
            )
            conn.executemany(
                """
                INSERT INTO latest_snapshot_states(
                    drone_id, cluster_id, role, mission_state,
                    pos_x, pos_y, pos_z, vel_x, vel_y, vel_z,
                    attitude_roll, attitude_pitch, attitude_yaw,
                    thrust_x, thrust_y, thrust_z,
                    drift_m, battery_pct, connectivity, reachable, rssi_dbm,
                    cpu_temp_c, gpu_load_pct, localization_source, localization_state,
                    localization_confidence, tdoa_confidence, confidence_trend,
                    relocalization_count, visible_anchor_count, occupancy_ratio,
                    sync_confidence, imu_camera_offset_ms, motor_health,
                    leadership_score, election_ready, timestamp
                ) VALUES(?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                [
                    (
                        state.drone_id,
                        state.cluster_id,
                        state.role,
                        state.mission_state,
                        state.position[0],
                        state.position[1],
                        state.position[2],
                        state.velocity[0],
                        state.velocity[1],
                        state.velocity[2],
                        state.attitude_rpy[0],
                        state.attitude_rpy[1],
                        state.attitude_rpy[2],
                        state.thrust_vector[0],
                        state.thrust_vector[1],
                        state.thrust_vector[2],
                        state.drift_m,
                        state.battery_pct,
                        state.connectivity,
                        1 if state.reachable else 0,
                        state.rssi_dbm,
                        state.cpu_temp_c,
                        state.gpu_load_pct,
                        state.localization_source,
                        state.localization_state,
                        state.localization_confidence,
                        state.tdoa_confidence,
                        state.confidence_trend,
                        state.relocalization_count,
                        state.visible_anchor_count,
                        state.occupancy_ratio,
                        state.sync_confidence,
                        state.imu_camera_offset_ms,
                        state.motor_health,
                        state.leadership_score,
                        1 if state.election_ready else 0,
                        state.timestamp,
                    )
                    for state in snapshot.states
                ],
            )

    def load_snapshot(self) -> "DashboardSnapshot | None":
        with self._connect() as conn:
            meta = conn.execute("SELECT * FROM latest_snapshot_meta WHERE singleton_id = 1").fetchone()
            rows = conn.execute("SELECT * FROM latest_snapshot_states ORDER BY drone_id").fetchall()
        if meta is None or not rows:
            return None
        states = [
            DroneState(
                drone_id=safe_int(row["drone_id"]),
                cluster_id=safe_text(row["cluster_id"]),
                role=safe_text(row["role"]),
                mission_state=safe_text(row["mission_state"]),
                position=(safe_float(row["pos_x"]), safe_float(row["pos_y"]), safe_float(row["pos_z"])),
                velocity=(safe_float(row["vel_x"]), safe_float(row["vel_y"]), safe_float(row["vel_z"])),
                attitude_rpy=(
                    safe_float(row["attitude_roll"]),
                    safe_float(row["attitude_pitch"]),
                    safe_float(row["attitude_yaw"]),
                ),
                thrust_vector=(
                    safe_float(row["thrust_x"]),
                    safe_float(row["thrust_y"]),
                    safe_float(row["thrust_z"], 9.81),
                ),
                drift_m=safe_float(row["drift_m"]),
                battery_pct=safe_float(row["battery_pct"]),
                connectivity=safe_text(row["connectivity"]),
                reachable=bool(row["reachable"]),
                rssi_dbm=safe_float(row["rssi_dbm"]),
                cpu_temp_c=safe_float(row["cpu_temp_c"]),
                gpu_load_pct=safe_float(row["gpu_load_pct"]),
                localization_source=safe_text(row["localization_source"], "vision-inertial"),
                localization_state=safe_text(row["localization_state"], "nominal"),
                localization_confidence=safe_float(row["localization_confidence"], 1.0),
                tdoa_confidence=safe_float(row["tdoa_confidence"]),
                confidence_trend=safe_float(row["confidence_trend"]),
                relocalization_count=safe_int(row["relocalization_count"]),
                visible_anchor_count=safe_int(row["visible_anchor_count"]),
                occupancy_ratio=safe_float(row["occupancy_ratio"]),
                sync_confidence=safe_float(row["sync_confidence"], 1.0),
                imu_camera_offset_ms=safe_float(row["imu_camera_offset_ms"]),
                motor_health=safe_float(row["motor_health"], 1.0),
                leadership_score=safe_float(row["leadership_score"]),
                election_ready=bool(row["election_ready"]),
                timestamp=safe_float(row["timestamp"], time.time()),
            )
            for row in rows
        ]
        return DashboardSnapshot(
            states=states,
            backend_mode=safe_text(meta["backend_mode"], "simulation"),
            leader_id=safe_int(meta["leader_id"], 0) or None,
            avg_latency_ms=safe_float(meta["avg_latency_ms"]),
            packet_loss_pct=safe_float(meta["packet_loss_pct"]),
            cpu_temp_c=safe_float(meta["cpu_temp_c"]),
            gpu_load_pct=safe_float(meta["gpu_load_pct"]),
            election_state=safe_text(meta["election_state"], "stable"),
            timestamp=safe_float(meta["timestamp"], time.time()),
        )

    def save_command(self, request_obj: "CommandRequest", result_text: str) -> None:
        with self._connect() as conn:
            conn.execute(
                "INSERT INTO command_history(created_at, action, payload_json, result_text) VALUES(?, ?, ?, ?)",
                (time.time(), request_obj.action, json.dumps(request_obj.payload, sort_keys=True), result_text),
            )
            conn.execute(
                "DELETE FROM command_history WHERE id NOT IN (SELECT id FROM command_history ORDER BY id DESC LIMIT 200)"
            )

    def load_recent_commands(self, limit: int = 20) -> list[str]:
        with self._connect() as conn:
            rows = conn.execute(
                "SELECT created_at, action, result_text FROM command_history ORDER BY id DESC LIMIT ?",
                (limit,),
            ).fetchall()
        lines = []
        for row in reversed(rows):
            timestamp = time.strftime("%H:%M:%S", time.localtime(safe_float(row["created_at"], time.time())))
            lines.append(f"[{timestamp}] {safe_text(row['action'])}: {safe_text(row['result_text'])}")
        return lines


def _add_windows_dll_dirs(root: Path) -> None:
    if os.name != "nt" or not hasattr(os, "add_dll_directory"):
        return
    candidates = [
        Path(os.environ.get("VCPKG_ROOT", "")) / "installed" / "x64-windows" / "bin",
        Path("C:/tools/vcpkg-full/installed/x64-windows/bin"),
        Path(sys.base_prefix),
        Path(sys.base_prefix) / "DLLs",
        root / "build-runtime-check" / "Release",
        root / "build-complete-check" / "Release",
    ]
    pythoncore = Path(os.environ.get("LOCALAPPDATA", "")) / "Python"
    if pythoncore.exists():
        candidates.extend(path for path in pythoncore.glob("pythoncore-*") if path.is_dir())
    for candidate in candidates:
        try:
            if candidate.exists():
                os.add_dll_directory(str(candidate))
        except Exception:
            continue


def discover_bridge():
    root = Path(__file__).resolve().parent.parent
    _add_windows_dll_dirs(root)
    candidates = [
        root / "build",
        root / "build" / "Release",
        root / "build-dashboard",
        root / "build-dashboard" / "Debug",
        root / "build-dashboard" / "Release",
        root / "build-full",
        root / "build-full" / "Release",
        root / "build-check",
        root / "build-runtime-check",
        root / "build-runtime-check" / "Release",
        root / "build-complete-check",
        root / "build-complete-check" / "Release",
    ]
    for candidate in candidates:
        if candidate.exists():
            sys.path.insert(0, str(candidate))
    try:
        import drone_bridge as bridge  # type: ignore

        return bridge, "pybind11"
    except Exception:
        return None, "simulation"


BRIDGE, BRIDGE_MODE = discover_bridge()


def enum_name(value: Any) -> str:
    return getattr(value, "name", str(value).split(".")[-1])


@dataclass(slots=True)
class DroneState:
    drone_id: int
    cluster_id: str
    role: str
    mission_state: str
    position: tuple[float, float, float]
    velocity: tuple[float, float, float]
    drift_m: float
    battery_pct: float
    connectivity: str
    reachable: bool
    rssi_dbm: float
    cpu_temp_c: float
    gpu_load_pct: float
    attitude_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    thrust_vector: tuple[float, float, float] = (0.0, 0.0, 9.81)
    localization_source: str = "vision-inertial"
    localization_state: str = "nominal"
    localization_confidence: float = 1.0
    tdoa_confidence: float = 0.0
    confidence_trend: float = 0.0
    relocalization_count: int = 0
    visible_anchor_count: int = 0
    occupancy_ratio: float = 0.0
    sync_confidence: float = 1.0
    imu_camera_offset_ms: float = 0.0
    motor_health: float = 1.0
    leadership_score: float = 0.0
    election_ready: bool = True
    timestamp: float = field(default_factory=time.time)


@dataclass(slots=True)
class DashboardSnapshot:
    states: list[DroneState]
    backend_mode: str
    leader_id: int | None
    avg_latency_ms: float
    packet_loss_pct: float
    cpu_temp_c: float
    gpu_load_pct: float
    election_state: str = "stable"
    timestamp: float = field(default_factory=time.time)


@dataclass(slots=True)
class CommandRequest:
    action: str
    payload: dict[str, Any] = field(default_factory=dict)


class GoControlPlaneClient:
    def __init__(self, base_url: str) -> None:
        self._base_url = base_url.rstrip("/")

    def fetch_snapshot(self) -> DashboardSnapshot:
        logger.debug("GoControlPlaneClient.fetch_snapshot url=%s", self._base_url)
        payload = self._get_json("/api/v1/fleet")
        drones = payload.get("drones", [])
        states = [
            DroneState(
                drone_id=safe_int(item.get("drone_id", 0), 0),
                cluster_id=safe_text(item.get("cluster_id", "cluster-01"), "cluster-01"),
                role=safe_text(item.get("role", "FOLLOWER"), "FOLLOWER"),
                mission_state=safe_text(item.get("mission_state", "standby"), "standby"),
                position=safe_vector3(item.get("position", [0.0, 0.0, 0.0])),
                velocity=safe_vector3(item.get("velocity", [0.0, 0.0, 0.0])),
                attitude_rpy=safe_vector3(item.get("attitude_rpy", [0.0, 0.0, 0.0])),
                thrust_vector=safe_vector3(item.get("thrust_vector", [0.0, 0.0, 9.81]), (0.0, 0.0, 9.81)),
                drift_m=safe_float(item.get("drift_m", 0.0)),
                battery_pct=safe_float(item.get("battery_pct", 0.0)),
                connectivity=safe_text(item.get("connectivity", "Unknown"), "Unknown"),
                reachable=bool(item.get("reachable", False)),
                rssi_dbm=safe_float(item.get("rssi_dbm", -100.0), -100.0),
                cpu_temp_c=safe_float(item.get("cpu_temp_c", 0.0)),
                gpu_load_pct=safe_float(item.get("gpu_load_pct", 0.0)),
                localization_source=safe_text(item.get("localization_source", "vision-inertial"), "vision-inertial"),
                localization_state=safe_text(item.get("localization_state", "nominal"), "nominal"),
                localization_confidence=safe_float(item.get("localization_confidence", 1.0), 1.0),
                tdoa_confidence=safe_float(item.get("tdoa_confidence", 0.0), 0.0),
                confidence_trend=safe_float(item.get("confidence_trend", 0.0), 0.0),
                relocalization_count=safe_int(item.get("relocalization_count", 0), 0),
                visible_anchor_count=safe_int(item.get("visible_anchor_count", 0), 0),
                occupancy_ratio=safe_float(item.get("occupancy_ratio", 0.0), 0.0),
                sync_confidence=safe_float(item.get("sync_confidence", 1.0), 1.0),
                imu_camera_offset_ms=safe_float(item.get("imu_camera_offset_ms", 0.0), 0.0),
                motor_health=safe_float(item.get("motor_health", 1.0), 1.0),
                leadership_score=safe_float(item.get("leadership_score", 0.0)),
                election_ready=bool(item.get("election_ready", True)),
                timestamp=safe_float(item.get("timestamp", time.time()), time.time()),
            )
            for item in drones
            if isinstance(item, dict)
        ]
        return DashboardSnapshot(
            states=states,
            backend_mode="go-control-plane",
            leader_id=safe_int(payload.get("leader_id"), 0) or None,
            avg_latency_ms=safe_float(payload.get("avg_latency_ms", 0.0)),
            packet_loss_pct=safe_float(payload.get("packet_loss_pct", 0.0)),
            cpu_temp_c=safe_float(payload.get("cpu_temp_c", 0.0)),
            gpu_load_pct=safe_float(payload.get("gpu_load_pct", 0.0)),
            election_state=safe_text(payload.get("election_state", "stable"), "stable"),
            timestamp=safe_float(payload.get("timestamp", time.time()), time.time()),
        )

    def send_command(self, request_obj: CommandRequest) -> str:
        logger.info("GoControlPlaneClient.send_command action=%s", request_obj.action)
        body = json.dumps(
            {
                "action": request_obj.action,
                "payload": request_obj.payload,
            }
        ).encode("utf-8")
        req = request.Request(
            self._base_url + "/api/v1/commands",
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with request.urlopen(req, timeout=2.5) as response:
            payload = json.loads(response.read().decode("utf-8"))
        return str(payload.get("message", "command accepted"))

    def _get_json(self, path: str) -> dict[str, Any]:
        with request.urlopen(self._base_url + path, timeout=2.5) as response:
            payload = json.loads(response.read().decode("utf-8"))
        if not isinstance(payload, dict):
            raise ValueError(f"unexpected payload type for {path}")
        return payload


class SwarmBackend:
    """Small facade over pybind11 bridge with simulation fallback."""

    def __init__(
        self,
        drone_ids: list[int],
        poll_hz: int,
        backend_url: str | None = None,
        initial_mission_overrides: dict[int, str] | None = None,
    ) -> None:
        self._ids = drone_ids
        self._poll_hz = max(poll_hz, 1)
        self._bridge = BRIDGE
        self._mode = BRIDGE_MODE
        self._pipelines: dict[int, Any] = {}
        self._network = None
        self._control_ready = False
        self._started_at = time.time()
        self._client = GoControlPlaneClient(backend_url) if backend_url else None
        self._mission_overrides: dict[int, str] = dict(initial_mission_overrides or {})
        logger.info("SwarmBackend init ids=%s poll_hz=%s backend_url=%s", drone_ids, poll_hz, backend_url)

        if self._client is not None:
            self._mode = "go-control-plane"
            self._control_ready = True
            return

        if self._bridge is not None:
            for drone_id in self._ids:
                try:
                    self._pipelines[drone_id] = self._bridge.VIOPipeline()
                except Exception:
                    self._pipelines.clear()
                    self._mode = "simulation"
                    break

        if (
            self._bridge is not None
            and getattr(self._bridge, "BUILD_SWARM_V2X", False)
            and hasattr(self._bridge, "V2XMeshNetwork")
        ):
            try:
                self._network = self._bridge.V2XMeshNetwork(9000, "239.255.0.1", 7400)
                self._control_ready = bool(self._network.start())
                if self._control_ready:
                    self._mode = "hybrid"
            except Exception:
                self._network = None
                self._control_ready = False

    @property
    def mode(self) -> str:
        return self._mode

    @property
    def mission_overrides(self) -> dict[int, str]:
        return dict(self._mission_overrides)

    def close(self) -> None:
        logger.info("SwarmBackend close mode=%s", self._mode)
        if self._network is not None:
            try:
                self._network.stop()
            except Exception:
                pass

    def _next_drone_id(self) -> int:
        return (max(self._ids) + 1) if self._ids else 1

    def _add_local_drone(self, requested_id: int | None = None) -> int:
        drone_id = requested_id or self._next_drone_id()
        if drone_id in self._ids:
            logger.info("SwarmBackend add drone skipped existing_id=%s", drone_id)
            return drone_id
        self._ids.append(drone_id)
        self._ids.sort()
        if self._bridge is not None:
            try:
                self._pipelines[drone_id] = self._bridge.VIOPipeline()
            except Exception:
                pass
        self._mission_overrides.setdefault(drone_id, "standby")
        logger.info("SwarmBackend added local drone id=%s total=%s", drone_id, len(self._ids))
        return drone_id

    def _remove_local_drone(self, drone_id: int) -> bool:
        if drone_id not in self._ids:
            logger.info("SwarmBackend remove drone skipped missing_id=%s", drone_id)
            return False
        self._ids.remove(drone_id)
        self._pipelines.pop(drone_id, None)
        self._mission_overrides.pop(drone_id, None)
        logger.info("SwarmBackend removed local drone id=%s total=%s", drone_id, len(self._ids))
        return True

    def _target_ids_from_request(self, request: CommandRequest) -> list[int]:
        target_ids = [int(v) for v in request.payload.get("target_ids", [])]
        if target_ids:
            return target_ids
        cluster_id = str(request.payload.get("cluster_id", "")).strip()
        if cluster_id:
            return [
                drone_id for drone_id in self._ids
                if f"cluster-{((drone_id - 1) // 20) + 1:02d}" == cluster_id
            ]
        return list(self._ids)

    def _apply_local_command_effect(self, request: CommandRequest) -> str:
        logger.info("SwarmBackend apply local command action=%s payload=%s", request.action, request.payload)
        target_ids = self._target_ids_from_request(request)
        if not target_ids:
            return "no target drones resolved"

        if request.action == "formation":
            shape = str(request.payload.get("shape", "DIAMOND")).lower()
            for drone_id in target_ids:
                self._mission_overrides[drone_id] = f"formation-{shape}"
            return f"{shape.upper()} formation assigned to {len(target_ids)} drone(s)"

        if request.action == "hold_position":
            for drone_id in target_ids:
                self._mission_overrides[drone_id] = "hold-position"
            return f"hold position applied to {len(target_ids)} drone(s)"

        if request.action == "return_home":
            for drone_id in target_ids:
                self._mission_overrides[drone_id] = "return-home"
            return f"return home applied to {len(target_ids)} drone(s)"

        if request.action == "emergency_land":
            for drone_id in target_ids:
                self._mission_overrides[drone_id] = "emergency-land"
            return f"emergency land applied to {len(target_ids)} drone(s)"

        if request.action == "fly":
            for drone_id in target_ids:
                self._mission_overrides[drone_id] = "fly"
            return f"takeoff applied to {len(target_ids)} drone(s)"

        if request.action == "land":
            for drone_id in target_ids:
                self._mission_overrides[drone_id] = "land"
            return f"target landing applied to {len(target_ids)} drone(s)"

        if request.action == "election":
            for drone_id in self._ids:
                self._mission_overrides.setdefault(drone_id, "formation-hold")
            return "leader election broadcast sent"

        return f"{request.action} accepted for {len(target_ids)} drone(s)"

    def poll(self) -> DashboardSnapshot:
        if self._client is not None:
            try:
                return self._client.fetch_snapshot()
            except (error.URLError, TimeoutError, ValueError, KeyError) as exc:
                logger.warning("go-control-plane snapshot failed, falling back to simulation: %s", exc)
                return self._simulate_snapshot()
        if self._bridge is not None and self._pipelines:
            snapshot = self._poll_bridge()
            if snapshot is not None:
                return snapshot
        return self._simulate_snapshot()

    def _poll_bridge(self) -> DashboardSnapshot | None:
        try:
            stats = self._bridge.read_system_stats()
            peer_map = self._peer_index()
            states: list[DroneState] = []
            now = time.time()

            for drone_id in self._ids:
                pose = self._pipelines[drone_id].current_pose()
                runtime = self._pipelines[drone_id].runtime_telemetry()
                peer = peer_map.get(drone_id)
                position = tuple(float(v) for v in pose.position)
                velocity = tuple(float(v) for v in pose.velocity)
                role = enum_name(peer.role) if peer is not None else ("LEADER" if drone_id == 1 else "FOLLOWER")
                battery = float(peer.battery_pct) if peer is not None else float(stats.battery_pct)
                rssi = float(peer.rssi_dbm) if peer is not None else float(stats.wifi_rssi_dbm)
                reachable = bool(peer.reachable) if peer is not None else True
                attitude_rpy = safe_vector3(getattr(runtime, "attitude_rpy", None))
                thrust_vector = safe_vector3(getattr(runtime, "thrust_vector", None), (0.0, 0.0, 9.81))
                if vector_norm3(thrust_vector) <= 1e-6:
                    attitude_rpy, thrust_vector = derive_attitude_thrust(position, velocity, role, now - self._started_at)

                states.append(
                    DroneState(
                        drone_id=drone_id,
                        cluster_id=str(peer.cluster_id) if peer is not None and hasattr(peer, "cluster_id") else f"cluster-{((drone_id - 1) // 20) + 1:02d}",
                        role=role,
                        mission_state=self._mission_overrides.get(
                            drone_id,
                            "tracking" if role == "LEADER" else "formation-hold",
                        ),
                        position=position,
                        velocity=velocity,
                        attitude_rpy=attitude_rpy,
                        thrust_vector=thrust_vector,
                        drift_m=float(self._pipelines[drone_id].drift_m()),
                        battery_pct=battery,
                        connectivity="Mesh" if reachable else "Lost",
                        reachable=reachable,
                        rssi_dbm=rssi,
                        cpu_temp_c=float(stats.cpu_temp_c),
                        gpu_load_pct=float(stats.gpu_pct),
                        localization_source=safe_text(getattr(pose, "localization_source", "vision-inertial"), "vision-inertial"),
                        localization_state=safe_text(getattr(runtime, "localization_state", "nominal"), "nominal"),
                        localization_confidence=safe_float(getattr(pose, "localization_confidence", 1.0), 1.0),
                        tdoa_confidence=safe_float(getattr(runtime, "tdoa_confidence", 0.0), 0.0),
                        confidence_trend=safe_float(getattr(runtime, "localization_confidence_trend", 0.0), 0.0),
                        relocalization_count=safe_int(getattr(runtime, "relocalization_count", 0), 0),
                        visible_anchor_count=safe_int(getattr(runtime, "visible_anchor_count", 0), 0),
                        occupancy_ratio=safe_float(getattr(runtime, "occupancy_ratio", 0.0), 0.0),
                        sync_confidence=safe_float(getattr(runtime, "sync_confidence", 1.0), 1.0),
                        imu_camera_offset_ms=safe_float(getattr(runtime, "imu_camera_offset_ms", 0.0), 0.0),
                        motor_health=max(0.45, min(1.0, 1.0 - (drone_id - 1) * 0.03)),
                        leadership_score=max(0.1, min(0.98, (battery / 100.0) * 0.35 + 0.55)),
                        election_ready=reachable and battery > 18.0,
                        timestamp=now,
                    )
                )

            if self._should_synthesize_fleet(states, peer_map):
                states = self._synthesize_bridge_fleet(states[0], float(stats.cpu_temp_c), float(stats.gpu_pct), now)

            return DashboardSnapshot(
                states=states,
                backend_mode=self._mode,
                leader_id=int(self._network.leader_id()) if self._network is not None else 1,
                avg_latency_ms=float(self._network.avg_latency_ms()) if self._network is not None else 0.0,
                packet_loss_pct=float(self._network.packet_loss_pct()) if self._network is not None else 0.0,
                cpu_temp_c=float(stats.cpu_temp_c),
                gpu_load_pct=float(stats.gpu_pct),
                election_state="election-ready" if any(state.election_ready for state in states) else "degraded",
                timestamp=now,
            )
        except Exception:
            logger.exception("SwarmBackend bridge polling failed")
            return None

    def _should_synthesize_fleet(self, states: list[DroneState], peer_map: dict[int, Any]) -> bool:
        if len(states) <= 1 or peer_map:
            return False
        first = states[0].position
        return all(np.linalg.norm(np.array(state.position) - np.array(first)) < 1e-5 for state in states[1:])

    def _synthesize_bridge_fleet(
        self,
        anchor: DroneState,
        cpu_temp_c: float,
        gpu_load_pct: float,
        now: float,
    ) -> list[DroneState]:
        elapsed = now - self._started_at
        synthesized: list[DroneState] = []
        for idx, drone_id in enumerate(self._ids):
            if drone_id == anchor.drone_id:
                synthesized.append(anchor)
                continue
            ring = idx + 1
            angle = elapsed * (0.18 + idx * 0.01) + idx * 0.9
            offset = np.array(
                [
                    math.cos(angle) * (1.8 + 0.7 * ring),
                    math.sin(angle) * (1.4 + 0.5 * ring),
                    0.15 * math.sin(angle * 0.6 + idx),
                ]
            )
            base = np.array(anchor.position)
            velocity = np.array(anchor.velocity) + np.array([-math.sin(angle) * 0.35, math.cos(angle) * 0.35, 0.0])
            synth_position = tuple((base + offset).tolist())
            synth_velocity = tuple(velocity.tolist())
            attitude_rpy, thrust_vector = derive_attitude_thrust(synth_position, synth_velocity, "FOLLOWER", elapsed)
            localization_confidence = max(0.22, anchor.localization_confidence - idx * 0.05)
            localization_state = "lost" if localization_confidence < 0.28 else "degraded" if localization_confidence < 0.58 else "nominal"
            synthesized.append(
                DroneState(
                    drone_id=drone_id,
                    cluster_id=f"cluster-{((drone_id - 1) // 20) + 1:02d}",
                    role="LEADER" if idx == 0 else "FOLLOWER",
                    mission_state=self._mission_overrides.get(drone_id, "formation-hold"),
                    position=synth_position,
                    velocity=synth_velocity,
                    attitude_rpy=attitude_rpy,
                    thrust_vector=thrust_vector,
                    drift_m=anchor.drift_m + 0.015 * idx,
                    battery_pct=max(20.0, anchor.battery_pct - idx * 2.5),
                    connectivity="Mesh",
                    reachable=True,
                    rssi_dbm=anchor.rssi_dbm - idx * 1.8,
                    cpu_temp_c=cpu_temp_c,
                    gpu_load_pct=gpu_load_pct,
                    localization_source=anchor.localization_source,
                    localization_state=localization_state,
                    localization_confidence=localization_confidence,
                    tdoa_confidence=max(0.0, anchor.tdoa_confidence - idx * 0.04),
                    confidence_trend=anchor.confidence_trend * 0.9,
                    relocalization_count=anchor.relocalization_count + idx // 2,
                    visible_anchor_count=max(0, anchor.visible_anchor_count - idx),
                    occupancy_ratio=min(1.0, anchor.occupancy_ratio + idx * 0.01),
                    sync_confidence=max(0.2, anchor.sync_confidence - idx * 0.04),
                    imu_camera_offset_ms=anchor.imu_camera_offset_ms + idx * 0.7,
                    motor_health=max(0.55, anchor.motor_health - idx * 0.04),
                    leadership_score=max(0.12, anchor.leadership_score - idx * 0.06),
                    election_ready=(anchor.battery_pct - idx * 2.5) > 18.0,
                    timestamp=now,
                )
            )
        return synthesized

    def _peer_index(self) -> dict[int, Any]:
        if self._network is None:
            return {}
        try:
            return {int(peer.id): peer for peer in self._network.active_peers()}
        except Exception:
            return {}

    def _simulate_snapshot(self) -> DashboardSnapshot:
        logger.debug("SwarmBackend simulate snapshot ids=%s", self._ids)
        elapsed = time.time() - self._started_at
        states: list[DroneState] = []
        for idx, drone_id in enumerate(self._ids):
            radius = 2.4 + idx * 1.3
            angle = elapsed * (0.22 + idx * 0.015) + idx * 0.85
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            z = 2.0 + 0.8 * idx + 0.45 * math.sin(elapsed * 0.5 + idx)
            vx = -radius * math.sin(angle) * 0.22
            vy = radius * math.cos(angle) * 0.22
            battery = max(18.0, 100.0 - elapsed * (0.22 + idx * 0.01))
            rssi = -48.0 - idx * 3.2 + 1.6 * math.sin(elapsed + idx)
            cpu_temp = 53.0 + 6.0 * math.sin(elapsed * 0.35)
            gpu_load = 35.0 + 18.0 * math.sin(elapsed * 0.28 + 1.4)
            motor_health = max(0.35, 0.94 - idx * 0.04 - 0.03 * abs(math.sin(elapsed * 0.23 + idx)))
            drift = 0.04 + 0.03 * idx + abs(math.sin(elapsed * 0.2 + idx)) * 0.12
            reachable = not (idx == len(self._ids) - 1 and int(elapsed) % 40 > 33)
            leadership_score = max(
                0.0,
                min(
                    1.0,
                    (battery / 100.0) * 0.32
                    + motor_health * 0.28
                    + max(0.0, min(1.0, (rssi + 90.0) / 45.0)) * 0.18
                    + max(0.0, min(1.0, 1.0 - cpu_temp / 100.0)) * 0.12
                    + max(0.0, min(1.0, 1.0 - gpu_load / 100.0)) * 0.10,
                ),
            )
            localization_confidence = max(0.18, 0.95 - idx * 0.07 - 0.18 * abs(math.sin(elapsed * 0.18 + idx)))
            localization_state = "lost" if localization_confidence < 0.28 else "degraded" if localization_confidence < 0.58 else "nominal"
            localization_source = (
                "tdoa-recovery" if localization_state == "lost"
                else "vision-depth-fused" if localization_confidence > 0.78
                else "vision-inertial"
            )
            tdoa_conf = max(0.0, min(1.0, 0.72 - idx * 0.08 + 0.06 * math.sin(elapsed * 0.31 + idx)))
            confidence_trend = 0.04 * math.sin(elapsed * 0.18 + idx)
            sync_confidence = max(0.25, 0.94 - idx * 0.06 - 0.08 * abs(math.sin(elapsed * 0.27 + idx)))
            anchor_visibility = max(0, 5 - idx)
            occupancy_ratio = min(1.0, 0.12 + idx * 0.04 + 0.03 * abs(math.sin(elapsed * 0.22 + idx)))
            attitude_rpy, thrust_vector = derive_attitude_thrust((x, y, z), (vx, vy, 0.0), "LEADER" if idx == 0 else "FOLLOWER", elapsed)

            states.append(
                DroneState(
                    drone_id=drone_id,
                    cluster_id=f"cluster-{((drone_id - 1) // 20) + 1:02d}",
                    role="LEADER" if idx == 0 else "FOLLOWER",
                    mission_state=self._mission_overrides.get(
                        drone_id,
                        "patrol" if idx == 0 else "formation-hold",
                    ),
                    position=(x, y, z),
                    velocity=(vx, vy, 0.0),
                    attitude_rpy=attitude_rpy,
                    thrust_vector=thrust_vector,
                    drift_m=drift,
                    battery_pct=battery,
                    connectivity="Mesh" if reachable else "Degraded",
                    reachable=reachable,
                    rssi_dbm=rssi,
                    cpu_temp_c=cpu_temp,
                    gpu_load_pct=max(0.0, gpu_load),
                    localization_source=localization_source,
                    localization_state=localization_state,
                    localization_confidence=localization_confidence,
                    tdoa_confidence=tdoa_conf,
                    confidence_trend=confidence_trend,
                    relocalization_count=max(0, idx - 1),
                    visible_anchor_count=anchor_visibility,
                    occupancy_ratio=occupancy_ratio,
                    sync_confidence=sync_confidence,
                    imu_camera_offset_ms=(1.5 + idx * 0.9) * math.sin(elapsed * 0.14 + idx),
                    motor_health=motor_health,
                    leadership_score=leadership_score,
                    election_ready=reachable and battery > 18.0 and motor_health > 0.45,
                )
            )

        return DashboardSnapshot(
            states=states,
            backend_mode=self._mode,
            leader_id=1,
            avg_latency_ms=4.0 + 2.0 * abs(math.sin(elapsed * 0.4)),
            packet_loss_pct=max(0.0, 1.4 * math.sin(elapsed * 0.2 + 0.5)),
            cpu_temp_c=states[0].cpu_temp_c,
            gpu_load_pct=states[0].gpu_load_pct,
            election_state="stable" if states[0].election_ready else "re-election",
        )

    def execute(self, request: CommandRequest) -> str:
        logger.info("SwarmBackend execute action=%s payload=%s mode=%s", request.action, request.payload, self._mode)
        if self._client is not None:
            try:
                return self._client.send_command(request)
            except (error.URLError, TimeoutError, ValueError) as exc:
                return f"{request.action} failed via go backend: {exc}"
        if request.action == "add_drone":
            drone_id = self._add_local_drone(int(request.payload.get("drone_id", 0)) or None)
            return f"drone {drone_id} added to local swarm"
        if request.action == "remove_drone":
            drone_id = int(request.payload.get("drone_id", 0) or 0)
            return f"drone {drone_id} removed from local swarm" if self._remove_local_drone(drone_id) else f"drone {drone_id} not found"
        if not self._control_ready or self._network is None or self._bridge is None:
            if request.action == "add_drone":
                drone_id = self._add_local_drone(int(request.payload.get("drone_id", 0)) or None)
                return f"drone {drone_id} added in simulation mode"
            if request.action == "remove_drone":
                drone_id = int(request.payload.get("drone_id", 0) or 0)
                return f"drone {drone_id} removed in simulation mode" if self._remove_local_drone(drone_id) else f"drone {drone_id} not found"
            if request.action in {"election", "formation", "hold_position", "return_home", "emergency_land", "fly", "land"}:
                return self._apply_local_command_effect(request)
            return f"{request.action} queued in simulation mode"

        try:
            if request.action == "election":
                self._network.trigger_election()
                return self._apply_local_command_effect(request)

            if request.action == "formation":
                self._ensure_leader()
                cmd = self._bridge.FormationCommand()
                shape_name = request.payload.get("shape", "DIAMOND")
                cmd.shape = getattr(self._bridge.FormationShape, shape_name)
                cmd.spacing_m = float(request.payload.get("spacing_m", 2.5))
                cmd.altitude_m = float(request.payload.get("altitude_m", 8.0))
                cmd.velocity_mps = float(request.payload.get("velocity_mps", 3.0))
                cmd.leader_target = np.array(request.payload.get("leader_target", [0.0, 0.0, cmd.altitude_m]))
                ok = bool(self._network.send_formation(cmd))
                if ok:
                    return self._apply_local_command_effect(request)
                return f"{shape_name} formation rejected"

            if request.action == "emergency_land":
                msg_type = self._bridge.SwarmMessageType.EMERGENCY_STOP
                ok = bool(self._network.broadcast(msg_type, []))
                if ok:
                    return self._apply_local_command_effect(request)
                return "emergency land broadcast failed"

            if request.action in {"hold_position", "return_home", "fly", "land"}:
                return self._apply_local_command_effect(request)

            return f"unknown command: {request.action}"
        except Exception as exc:
            return f"{request.action} failed: {exc}"

    def _ensure_leader(self) -> None:
        if enum_name(self._network.local_role()) != "LEADER":
            self._network.trigger_election()
            time.sleep(0.6)


class TelemetryWorker(QThread):
    snapshot_ready = Signal(object)
    log_ready = Signal(str)

    def __init__(self, backend: SwarmBackend, poll_hz: int) -> None:
        super().__init__()
        self._backend = backend
        self._poll_hz = max(poll_hz, 1)
        self._running = True

    def run(self) -> None:
        period = 1.0 / self._poll_hz
        self.log_ready.emit(f"telemetry worker started in {self._backend.mode} mode at {self._poll_hz} Hz")
        while self._running:
            started = time.perf_counter()
            try:
                snapshot = self._backend.poll()
                self.snapshot_ready.emit(snapshot)
            except Exception as exc:
                logger.exception("telemetry polling failed")
                self.log_ready.emit(f"telemetry polling error: {exc}")
            elapsed = time.perf_counter() - started
            time.sleep(max(0.0, period - elapsed))

    def stop(self) -> None:
        self._running = False


class CommandWorker(QThread):
    log_ready = Signal(str)
    command_completed = Signal(object, str)

    def __init__(self, backend: SwarmBackend) -> None:
        super().__init__()
        self._backend = backend
        self._running = True
        self._queue: queue.Queue[CommandRequest] = queue.Queue()

    def submit(self, request: CommandRequest) -> None:
        self._queue.put(request)

    def run(self) -> None:
        self.log_ready.emit("command worker ready")
        while self._running:
            try:
                request = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                message = self._backend.execute(request)
            except Exception as exc:
                logger.exception("command execution failed")
                message = f"{request.action} failed: {exc}"
            self.log_ready.emit(message)
            self.command_completed.emit(request, message)

    def stop(self) -> None:
        self._running = False


class MetricCard(QFrame):
    def __init__(self, title: str, accent: str) -> None:
        super().__init__()
        self._accent = accent
        self._value = QLabel("--")
        self._subtitle = QLabel("waiting")
        self._title = QLabel(title.upper())

        self.setObjectName("metricCard")
        # Ensure distinct colored top border or left border using stylesheet overrides in the main theme
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 16, 20, 16)
        layout.setSpacing(8)

        self._title.setStyleSheet(f"color: {TEXT_DIM}; font-size: 10px; font-weight: 700; letter-spacing: 1.5px;")
        self._value.setStyleSheet(f"color: {accent}; font-size: 28px; font-weight: 900; letter-spacing: -0.5px;")
        self._subtitle.setStyleSheet(f"color: {TEXT_DIM}; font-size: 12px; font-weight: 500;")

        layout.addWidget(self._title)
        layout.addWidget(self._value)
        layout.addWidget(self._subtitle)

    def set_data(self, value: str, subtitle: str, accent: str | None = None) -> None:
        if accent is None:
            accent = self._accent
        self._value.setText(value)
        self._value.setStyleSheet(f"color: {accent}; font-size: 28px; font-weight: 900; letter-spacing: -0.5px;")
        self._subtitle.setText(subtitle)


class Map3DView(gl.GLViewWidget):
    def __init__(self) -> None:
        super().__init__()
        self.setBackgroundColor(pg.mkColor(DARK_BG))
        self.opts['center'] = pg.Vector(0.0, 0.0, 5.0)
        self.setCameraPosition(distance=35, elevation=35, azimuth=45)

        grid = gl.GLGridItem()
        grid.setSize(40, 40)
        grid.setSpacing(2, 2)
        self.addItem(grid)

        axis = gl.GLAxisItem()
        axis.setSize(3.0, 3.0, 3.0)
        self.addItem(axis)

        self._scatter = gl.GLScatterPlotItem(pos=np.zeros((1, 3)), size=10)
        self.addItem(self._scatter)
        self._histories: dict[int, deque[tuple[float, float, float]]] = {}
        self._trails: dict[int, gl.GLLinePlotItem] = {}
        self._max_points = 240

    def ingest(self, states: list[DroneState]) -> None:
        if not states:
            return

        positions = np.array([state.position for state in states], dtype=float)
        colors = []
        for state in states:
            if not state.reachable:
                colors.append((0.94, 0.27, 0.27, 1.0))
            elif state.role == "LEADER":
                colors.append((0.98, 0.55, 0.12, 1.0))
            else:
                colors.append((0.15, 0.78, 0.85, 1.0))
        self._scatter.setData(pos=positions, color=np.array(colors), size=11)

        for state in states:
            if state.drone_id not in self._histories:
                self._histories[state.drone_id] = deque(maxlen=self._max_points)
                line = gl.GLLinePlotItem(width=2.0, antialias=True)
                self._trails[state.drone_id] = line
                self.addItem(line)
            self._histories[state.drone_id].append(state.position)
            if len(self._histories[state.drone_id]) >= 2:
                trail = np.array(self._histories[state.drone_id], dtype=float)
                color = np.tile(np.array([[0.22, 0.74, 0.97, 0.45]]), (trail.shape[0], 1))
                self._trails[state.drone_id].setData(pos=trail, color=color)


class DriftGraph(pg.PlotWidget):
    def __init__(self, drone_ids: list[int], poll_hz: int) -> None:
        super().__init__()
        self.setBackground(PANEL_ALT)
        self.showGrid(x=True, y=True, alpha=0.2)
        self.setLabel("left", "EKF Error (m)", color=TEXT_DIM)
        self.setLabel("bottom", "Time (s)", color=TEXT_DIM)
        self.addLegend(offset=(12, 12))
        self._window = 360
        self._poll_hz = max(poll_hz, 1)
        self._t0 = time.time()
        self._history: dict[int, deque[float]] = {drone_id: deque(maxlen=self._window) for drone_id in drone_ids}
        self._palette = [CYAN, MAGENTA, SUCCESS, WARN, ACCENT_ALT, "#A78BFA"]
        self._curves = {
            drone_id: self.plot(pen=pg.mkPen(self._palette[idx % len(self._palette)], width=2), name=f"Drone {drone_id}")
            for idx, drone_id in enumerate(drone_ids)
        }

    def ingest(self, states: list[DroneState]) -> None:
        now = time.time() - self._t0
        for state in states:
            if state.drone_id not in self._history:
                self._history[state.drone_id] = deque(maxlen=self._window)
                color = self._palette[len(self._curves) % len(self._palette)]
                self._curves[state.drone_id] = self.plot(
                    pen=pg.mkPen(color, width=2),
                    name=f"Drone {state.drone_id}",
                )
            self._history[state.drone_id].append(state.drift_m)
            series = np.array(self._history[state.drone_id], dtype=float)
            x = np.linspace(max(0.0, now - len(series) / self._poll_hz), now, len(series))
            self._curves[state.drone_id].setData(x, series)



class MCSSScoreGraph(pg.PlotWidget):
    def __init__(self, drone_ids: list[int], poll_hz: int) -> None:
        super().__init__()
        self.setBackground(PANEL_ALT)
        self.showGrid(x=True, y=True, alpha=0.2)
        self.setLabel("left", "Leader Score", color=TEXT_DIM)
        self.setLabel("bottom", "Time (s)", color=TEXT_DIM)
        self.addLegend(offset=(12, 12))
        self._window = 360
        self._poll_hz = max(poll_hz, 1)
        self._t0 = time.time()
        self._history: dict[int, deque[float]] = {drone_id: deque(maxlen=self._window) for drone_id in drone_ids}
        self._palette = [CYAN, MAGENTA, SUCCESS, WARN, ACCENT_ALT, "#A78BFA"]
        self._curves = {
            drone_id: self.plot(pen=pg.mkPen(self._palette[idx % len(self._palette)], width=2), name=f"Drone {drone_id}")
            for idx, drone_id in enumerate(drone_ids)
        }

    def ingest(self, states: list[DroneState]) -> None:
        now = time.time() - self._t0
        for state in states:
            if state.drone_id not in self._history:
                self._history[state.drone_id] = deque(maxlen=self._window)
                color = self._palette[len(self._curves) % len(self._palette)]
                self._curves[state.drone_id] = self.plot(pen=pg.mkPen(color, width=2), name=f"Drone {state.drone_id}")
            self._history[state.drone_id].append(state.leadership_score)
            series = np.array(self._history[state.drone_id], dtype=float)
            x = np.linspace(max(0.0, now - len(series) / self._poll_hz), now, len(series))
            self._curves[state.drone_id].setData(x, series)

class NetworkHealthGraph(pg.PlotWidget):
    def __init__(self, poll_hz: int) -> None:
        super().__init__()
        self.setBackground(PANEL_ALT)
        self.showGrid(x=True, y=True, alpha=0.2)
        self.setLabel("left", "Network Latency & Loss", color=TEXT_DIM)
        self.setLabel("bottom", "Time (s)", color=TEXT_DIM)
        self.addLegend(offset=(12, 12))
        self._window = 360
        self._poll_hz = max(poll_hz, 1)
        self._t0 = time.time()
        self._lat_history = deque(maxlen=self._window)
        self._loss_history = deque(maxlen=self._window)
        
        self._lat_curve = self.plot(pen=pg.mkPen(CYAN, width=2), name="Avg Latency (ms)")
        self._loss_curve = self.plot(pen=pg.mkPen(DANGER, width=2), name="Packet Loss (%)")

    def ingest(self, avg_latency: float, packet_loss: float) -> None:
        now = time.time() - self._t0
        self._lat_history.append(avg_latency)
        self._loss_history.append(packet_loss)
        
        arr_lat = np.array(self._lat_history, dtype=float)
        arr_loss = np.array(self._loss_history, dtype=float)
        x_lat = np.linspace(max(0.0, now - len(arr_lat) / self._poll_hz), now, len(arr_lat))
        x_loss = np.linspace(max(0.0, now - len(arr_loss) / self._poll_hz), now, len(arr_loss))
        
        self._lat_curve.setData(x_lat, arr_lat)
        self._loss_curve.setData(x_loss, arr_loss)

class ComputeLoadGraph(pg.PlotWidget):
    def __init__(self, poll_hz: int) -> None:
        super().__init__()
        self.setBackground(PANEL_ALT)
        self.showGrid(x=True, y=True, alpha=0.2)
        self.setLabel("left", "Compute (Temp & Load)", color=TEXT_DIM)
        self.setLabel("bottom", "Time (s)", color=TEXT_DIM)
        self.addLegend(offset=(12, 12))
        self._window = 360
        self._poll_hz = max(poll_hz, 1)
        self._t0 = time.time()
        self._cpu_history = deque(maxlen=self._window)
        self._gpu_history = deque(maxlen=self._window)
        
        self._cpu_curve = self.plot(pen=pg.mkPen(WARN, width=2), name="CPU Temp (C)")
        self._gpu_curve = self.plot(pen=pg.mkPen(ACCENT, width=2), name="GPU Load (%)")

    def ingest(self, cpu_temp: float, gpu_load: float) -> None:
        now = time.time() - self._t0
        self._cpu_history.append(cpu_temp)
        self._gpu_history.append(gpu_load)
        
        arr_cpu = np.array(self._cpu_history, dtype=float)
        arr_gpu = np.array(self._gpu_history, dtype=float)
        x_cpu = np.linspace(max(0.0, now - len(arr_cpu) / self._poll_hz), now, len(arr_cpu))
        x_gpu = np.linspace(max(0.0, now - len(arr_gpu) / self._poll_hz), now, len(arr_gpu))
        
        self._cpu_curve.setData(x_cpu, arr_cpu)
        self._gpu_curve.setData(x_gpu, arr_gpu)

class SwarmTable(QTableWidget):
    HEADERS = ["ID", "Cluster", "Role", "Mission", "Connectivity", "Battery", "RSSI", "Drift", "Position"]

    def __init__(self) -> None:
        super().__init__(0, len(self.HEADERS))
        self.setHorizontalHeaderLabels(self.HEADERS)
        self.horizontalHeader().setStretchLastSection(True)
        self.verticalHeader().setVisible(False)
        self.setAlternatingRowColors(True)
        self.setEditTriggers(QTableWidget.NoEditTriggers)
        self.setSelectionBehavior(QTableWidget.SelectRows)
        self.setShowGrid(False)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def ingest(self, states: list[DroneState]) -> None:
        self.setRowCount(len(states))
        for row, state in enumerate(states):
            values = [
                str(state.drone_id),
                state.cluster_id,
                state.role,
                state.mission_state,
                state.connectivity,
                f"{state.battery_pct:.0f}%",
                f"{state.rssi_dbm:.0f} dBm",
                f"{state.drift_m:.3f} m",
                f"({state.position[0]:.1f}, {state.position[1]:.1f}, {state.position[2]:.1f})",
            ]
            for col, value in enumerate(values):
                item = QTableWidgetItem(value)
                item.setTextAlignment(Qt.AlignCenter)
                if col == 2 and state.role == "LEADER":
                    item.setForeground(QColor(ACCENT_ALT))
                elif col == 4:
                    item.setForeground(QColor(SUCCESS if state.reachable else DANGER))
                elif col == 5 and state.battery_pct < 25.0:
                    item.setForeground(QColor(WARN if state.battery_pct >= 15.0 else DANGER))
                self.setItem(row, col, item)


class AdvancedMetricsTable(QTableWidget):
    HEADERS = ["ID", "VIO Drift (m)", "Loc Conf", "MCSS Score", "V2X Latency", "V2X Loss (%)", "CPU Temp (C)", "GPU Load (%)"]

    def __init__(self) -> None:
        super().__init__(0, len(self.HEADERS))
        self.setHorizontalHeaderLabels(self.HEADERS)
        self.horizontalHeader().setStretchLastSection(True)
        self.verticalHeader().setVisible(False)
        self.setAlternatingRowColors(True)
        self.setEditTriggers(QTableWidget.NoEditTriggers)
        self.setSelectionBehavior(QTableWidget.SelectRows)
        self.setShowGrid(False)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def ingest(self, snapshot: DashboardSnapshot) -> None:
        if not snapshot.states:
            return
        self.setRowCount(len(snapshot.states))
        for row, state in enumerate(snapshot.states):
            values = [
                str(state.drone_id),
                f"{state.drift_m:.3f}",
                f"{state.localization_confidence:.2f}",
                f"{state.leadership_score:.3f}",
                f"{snapshot.avg_latency_ms:.1f} ms",
                f"{snapshot.packet_loss_pct:.1f}",
                f"{state.cpu_temp_c:.1f}",
                f"{state.gpu_load_pct:.0f}"
            ]
            for col, value in enumerate(values):
                item = QTableWidgetItem(value)
                item.setTextAlignment(Qt.AlignCenter)
                
                # Coloring logic
                if col == 1 and state.drift_m > 0.5:
                    item.setForeground(QColor(WARN if state.drift_m < 1.0 else DANGER))
                elif col == 3 and state.leadership_score > 0.5:
                    item.setForeground(QColor(SUCCESS))
                elif col == 4 and snapshot.avg_latency_ms > 20:
                    item.setForeground(QColor(DANGER))
                elif col == 6 and state.cpu_temp_c > 75:
                    item.setForeground(QColor(WARN))
                    
                self.setItem(row, col, item)

class DroneDetailPanel(QGroupBox):
    def __init__(self) -> None:
        super().__init__("Selected Drone Matrix")
        self.setMinimumHeight(280)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(10, 16, 10, 10)
        outer.setSpacing(0)
        
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setStyleSheet(f"QScrollArea {{ background: transparent; border: none; }}")
        
        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setContentsMargins(4, 4, 12, 4)
        layout.setSpacing(16)
        
        self._labels: dict[str, QLabel] = {}
        self._bars: dict[str, QProgressBar] = {}

        # Helpers
        def make_section_header(title: str) -> QLabel:
            lbl = QLabel(title)
            lbl.setStyleSheet(f"color:{ACCENT}; font-size:11px; font-weight:800; letter-spacing:1.5px; border-bottom:1px solid rgba(0, 210, 255, 0.2); padding-bottom:4px;")
            return lbl

        def make_row(label: str, key: str, is_bar=False) -> QHBoxLayout:
            row = QHBoxLayout()
            lbl = QLabel(label)
            lbl.setStyleSheet(f"color:{TEXT_DIM}; font-size:12px; font-weight:600;")
            row.addWidget(lbl)
            
            if is_bar:
                bar = QProgressBar()
                bar.setFixedHeight(8)
                bar.setTextVisible(False)
                bar.setValue(0)
                self._bars[key] = bar
                row.addWidget(bar, 1)
                
                val = QLabel("--")
                val.setFixedWidth(40)
                val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                val.setStyleSheet(f"color:{TEXT}; font-size:12px; font-weight:700;")
                self._labels[key] = val
                row.addWidget(val)
            else:
                val = QLabel("--")
                val.setWordWrap(True)
                val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                val.setTextInteractionFlags(Qt.TextSelectableByMouse)
                val.setStyleSheet(f"color:{TEXT}; font-size:12px; font-weight:600;")
                self._labels[key] = val
                row.addWidget(val, 1)
            return row

        # Swarm identity
        layout.addWidget(make_section_header("IDENTITY & MISSION"))
        layout.addLayout(make_row("Drone ID / Cluster", "id_cluster"))
        layout.addLayout(make_row("Role / Mission", "role_mission"))
        layout.addLayout(make_row("MCSS Score / Election", "score_election"))
        
        # Hardware Health
        layout.addWidget(make_section_header("HARDWARE & TELEMETRY"))
        layout.addLayout(make_row("Battery Level", "battery", True))
        layout.addLayout(make_row("Motor Health", "motor", True))
        layout.addLayout(make_row("CPU Temp", "cpu", True))
        layout.addLayout(make_row("GPU Load", "gpu", True))
        layout.addLayout(make_row("Mesh RSSI", "rssi"))
        
        # Localization
        layout.addWidget(make_section_header("NAVIGATION & STATE"))
        layout.addLayout(make_row("State / Source", "loc_source"))
        layout.addLayout(make_row("VIO Confidence", "vio_conf", True))
        layout.addLayout(make_row("TDOA Confidence", "tdoa_conf", True))
        layout.addLayout(make_row("Avg Drift (m)", "drift"))
        layout.addLayout(make_row("Position / Velocity", "pos_vel"))
        layout.addLayout(make_row("Attitude RPY", "attitude"))
        layout.addLayout(make_row("Thrust Vector", "thrust"))
        layout.addLayout(make_row("Net Thrust", "thrust_mag", True))
        layout.addLayout(make_row("Relocal / Trend", "reloc_trend"))
        layout.addLayout(make_row("Anchors / Sync", "anchors_sync"))
        layout.addLayout(make_row("Occupancy", "occupancy", True))
        
        layout.addStretch(1)
        scroll.setWidget(content)
        outer.addWidget(scroll)

    def _set_bar(self, key: str, value: float, fmt: str, threshold_warn: float, threshold_danger: float, invert: bool = False):
        self._bars[key].setValue(int(max(0, min(100, value))))
        self._labels[key].setText(fmt.format(value))
        
        if invert:
            if value > threshold_danger: color = DANGER
            elif value > threshold_warn: color = WARN
            else: color = SUCCESS
        else:
            if value < threshold_danger: color = DANGER
            elif value < threshold_warn: color = WARN
            else: color = SUCCESS
            
        self._bars[key].setStyleSheet(f"QProgressBar::chunk {{ background-color: {color}; border-radius: 3px; }}")

    def ingest(self, state: DroneState | None) -> None:
        if state is None:
            for lbl in self._labels.values(): lbl.setText("--")
            for bar in self._bars.values(): bar.setValue(0)
            return
            
        self._labels["id_cluster"].setText(f"ID: {state.drone_id}   |   {state.cluster_id}")
        
        role_color = ACCENT_ALT if state.role == "LEADER" else CYAN
        self._labels["role_mission"].setText(f"[{state.role}]   {state.mission_state.upper()}")
        self._labels["role_mission"].setStyleSheet(f"color:{role_color}; font-size:12px; font-weight:800;")
        
        elec = "READY" if state.election_ready else "BLOCKED"
        self._labels["score_election"].setText(f"Score: {state.leadership_score:.3f}   |   {elec}")
        
        self._set_bar("battery", state.battery_pct, "{:.0f}%", 25.0, 15.0)
        self._set_bar("motor", state.motor_health * 100.0, "{:.0f}%", 75.0, 50.0)
        self._set_bar("cpu", state.cpu_temp_c, "{:.0f}°C", 70.0, 80.0, invert=True)
        self._set_bar("gpu", state.gpu_load_pct, "{:.0f}%", 80.0, 90.0, invert=True)
        
        self._labels["rssi"].setText(f"{state.rssi_dbm:.0f} dBm  |  {state.connectivity}")
        self._labels["loc_source"].setText(f"{state.localization_state.upper()}   |   {state.localization_source}")
        
        self._set_bar("vio_conf", state.localization_confidence * 100.0, "{:.0f}%", 60.0, 30.0)
        self._set_bar("tdoa_conf", state.tdoa_confidence * 100.0, "{:.0f}%", 50.0, 20.0)
        self._labels["drift"].setText(f"{state.drift_m:.3f} m")
        speed = vector_norm3(state.velocity)
        self._labels["pos_vel"].setText(
            f"Pos ({state.position[0]:.1f}, {state.position[1]:.1f}, {state.position[2]:.1f})  |  {speed:.2f} m/s"
        )
        self._labels["attitude"].setText(
            f"R {math.degrees(state.attitude_rpy[0]):+.1f}°  P {math.degrees(state.attitude_rpy[1]):+.1f}°  Y {math.degrees(state.attitude_rpy[2]):+.1f}°"
        )
        self._labels["thrust"].setText(
            f"({state.thrust_vector[0]:+.2f}, {state.thrust_vector[1]:+.2f}, {state.thrust_vector[2]:+.2f}) m/s²"
        )
        self._set_bar("thrust_mag", min(100.0, vector_norm3(state.thrust_vector) * 6.0), "{:.1f}", 45.0, 25.0)

        trend_color = SUCCESS if state.confidence_trend >= 0 else WARN
        self._labels["reloc_trend"].setText(f"Count: {state.relocalization_count}  |  Trend: {state.confidence_trend:+.2f}")
        
        self._labels["anchors_sync"].setText(f"Vis: {state.visible_anchor_count}  |  Sync: {state.sync_confidence*100.0:.0f}%  ({state.imu_camera_offset_ms:.1f}ms)")
        self._set_bar("occupancy", state.occupancy_ratio * 100.0, "{:.1f}%", 60.0, 85.0, invert=True)



class CommandConsole(QGroupBox):
    command_requested = Signal(object)

    def __init__(self) -> None:
        super().__init__("Command Console")
        self.setMinimumHeight(250)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 12, 10, 10)
        layout.setSpacing(12)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setStyleSheet(f"QScrollArea {{ background: transparent; border: none; }}")
        content = QWidget()
        content_layout = QVBoxLayout(content)
        content_layout.setContentsMargins(4, 4, 4, 4)
        content_layout.setSpacing(12)

        button_layout = QGridLayout()
        button_layout.setHorizontalSpacing(10)
        button_layout.setVerticalSpacing(10)

        self._scope = QComboBox()
        self._scope.addItems(["Fleet", "Selected Cluster", "Selected Drone"])
        self._scope.setStyleSheet(f"background:{PANEL_ALT}; color:{TEXT}; border:1px solid {BORDER}; padding:6px;")
        self._spacing = QDoubleSpinBox()
        self._spacing.setRange(1.0, 20.0)
        self._spacing.setValue(2.5)
        self._spacing.setSuffix(" m")
        self._spacing.setStyleSheet(f"background:{PANEL_ALT}; color:{TEXT}; border:1px solid {BORDER}; padding:6px;")
        self._altitude = QDoubleSpinBox()
        self._altitude.setRange(1.0, 120.0)
        self._altitude.setValue(8.0)
        self._altitude.setSuffix(" m")
        self._altitude.setStyleSheet(f"background:{PANEL_ALT}; color:{TEXT}; border:1px solid {BORDER}; padding:6px;")
        self._speed = QDoubleSpinBox()
        self._speed.setRange(0.5, 20.0)
        self._speed.setValue(3.0)
        self._speed.setSuffix(" m/s")
        self._speed.setStyleSheet(f"background:{PANEL_ALT}; color:{TEXT}; border:1px solid {BORDER}; padding:6px;")

        tuning_layout = QGridLayout()
        tuning_layout.setColumnStretch(0, 0)
        tuning_layout.setColumnStretch(1, 1)
        tuning_layout.addWidget(QLabel("Scope"), 0, 0)
        tuning_layout.addWidget(self._scope, 0, 1)
        tuning_layout.addWidget(QLabel("Spacing"), 1, 0)
        tuning_layout.addWidget(self._spacing, 1, 1)
        tuning_layout.addWidget(QLabel("Altitude"), 2, 0)
        tuning_layout.addWidget(self._altitude, 2, 1)
        tuning_layout.addWidget(QLabel("Speed"), 3, 0)
        tuning_layout.addWidget(self._speed, 3, 1)

        controls = [
            ("Add Drone", "#A3E635", "add_drone"),
            ("Remove Drone", DANGER, "remove_drone"),
            ("Election", SUCCESS, "election"),
            ("VEE", ACCENT, "VEE"),
            ("LINE", CYAN, "LINE"),
            ("DIAMOND", MAGENTA, "DIAMOND"),
            ("Takeoff (Fly)", SUCCESS, "fly"),
            ("Target Land", WARN, "land"),
            ("Hold", "#94A3B8", "hold_position"),
            ("Return Home", "#60A5FA", "return_home"),
            ("Emergency Land", DANGER, "emergency_land"),
        ]

        for idx, (label, color, action) in enumerate(controls):
            button = QPushButton(label)
            button.setCursor(Qt.PointingHandCursor)
            button.setMinimumHeight(38)
            button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            button.setStyleSheet(
                f"QPushButton {{ background:{color}; color:#041019; border:none; border-radius:10px; "
                f"font-weight:700; padding:10px 14px; }}"
                "QPushButton:pressed { padding-top: 12px; padding-bottom: 8px; }"
            )
            button.clicked.connect(lambda _checked=False, action_name=action: self.command_requested.emit(self._build_request(action_name)))
            button_layout.addWidget(button, idx // 2, idx % 2)
        button_layout.setColumnStretch(0, 1)
        button_layout.setColumnStretch(1, 1)

        self._log = QPlainTextEdit()
        self._log.setReadOnly(True)
        self._log.setMaximumBlockCount(200)
        self._log.setMinimumHeight(88)
        self._log.setPlaceholderText("Command activity stream...")
        self._log.setStyleSheet(
            f"QPlainTextEdit {{ background:{PANEL_ALT}; color:{TEXT}; border:1px solid {BORDER}; "
            "border-radius:10px; padding:8px; font-family: Consolas, 'Courier New', monospace; }}"
        )

        content_layout.addLayout(tuning_layout)
        content_layout.addLayout(button_layout)
        content_layout.addWidget(self._log, 1)
        scroll.setWidget(content)
        layout.addWidget(scroll)

    def append_log(self, message: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        self._log.appendPlainText(f"[{timestamp}] {message}")
        logger.info("console: %s", message)

    def _build_request(self, action: str) -> CommandRequest:
        payload = {
            "scope": self._scope.currentText(),
            "spacing_m": float(self._spacing.value()),
            "altitude_m": float(self._altitude.value()),
            "velocity_mps": float(self._speed.value()),
        }
        if action == "add_drone":
            return CommandRequest("add_drone", payload)
        if action in {"VEE", "LINE", "DIAMOND"}:
            return CommandRequest("formation", {**payload, "shape": action})
        return CommandRequest(action, payload)


class DashboardWindow(QMainWindow):
    def __init__(
        self,
        drone_ids: list[int],
        poll_hz: int,
        backend_url: str | None = None,
        store: DashboardStore | None = None,
    ) -> None:
        super().__init__()
        self._store = store or DashboardStore(STORE_PATH)
        self._env_manager = DroneEnvManager(Path(__file__).resolve().parent.parent / ".env")
        saved_settings = self._store.load_settings()
        if not backend_url:
            backend_url = safe_text(saved_settings.get("backend_url", ""), "")
        saved_ids = [
            safe_int(part.strip(), -1)
            for part in safe_text(saved_settings.get("drone_ids", ""), "").split(",")
            if part.strip()
        ]
        saved_ids = [drone_id for drone_id in saved_ids if drone_id > 0]
        if saved_ids:
            drone_ids = saved_ids
        poll_hz = safe_int(saved_settings.get("poll_hz"), poll_hz) or poll_hz
        self._ids = drone_ids
        self._poll_hz = poll_hz
        self._frame_counter = 0
        self._last_snapshot: DashboardSnapshot | None = None
        self._last_snapshot_persist_s = 0.0
        self._selected_drone_id = safe_int(saved_settings.get("selected_drone_id"), drone_ids[0] if drone_ids else 0) or (drone_ids[0] if drone_ids else None)
        self._last_backend_url = backend_url or ""

        initial_missions = {
            safe_int(key.removeprefix("mission_override_")): value
            for key, value in saved_settings.items()
            if key.startswith("mission_override_")
        }
        self._backend = SwarmBackend(drone_ids, poll_hz, backend_url=backend_url, initial_mission_overrides=initial_missions)
        self._telemetry = TelemetryWorker(self._backend, poll_hz)
        self._commands = CommandWorker(self._backend)

        self.setWindowTitle("UVA GPS-Denied Swarm Control Center (Lab Mode)")
        self.resize(1680, 980)
        self._apply_theme()
        self._build_ui()
        self._wire_threads()
        self._restore_persisted_state()

    def _apply_theme(self) -> None:
        app = QApplication.instance()
        assert app is not None
        app.setStyle("Fusion")

        palette = QPalette()
        palette.setColor(QPalette.Window, QColor(DARK_BG))
        palette.setColor(QPalette.WindowText, QColor(TEXT))
        palette.setColor(QPalette.Base, QColor(PANEL_ALT))
        palette.setColor(QPalette.AlternateBase, QColor(PANEL_BG))
        palette.setColor(QPalette.Text, QColor(TEXT))
        palette.setColor(QPalette.Button, QColor(PANEL_BG))
        palette.setColor(QPalette.ButtonText, QColor(TEXT))
        palette.setColor(QPalette.Highlight, QColor(ACCENT))
        palette.setColor(QPalette.HighlightedText, QColor("#041019"))
        app.setPalette(palette)

        self.setStyleSheet(
            f"""
            QMainWindow {{
                background: {DARK_BG};
            }}
            QGroupBox {{
                background: rgba(18, 26, 43, 0.6);
                border: 1px solid rgba(30, 42, 59, 0.8);
                border-radius: 8px;
                margin-top: 24px;
                color: {TEXT};
                font-size: 13px;
                font-weight: 700;
                letter-spacing: 1px;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 16px;
                top: -6px;
                padding: 4px 8px;
                background: {PANEL_BG};
                color: {ACCENT};
                border: 1px solid {BORDER};
                border-radius: 4px;
            }}
            QTableWidget {{
                background: transparent;
                color: {TEXT};
                alternate-background-color: rgba(11, 17, 30, 0.5);
                border: none;
                font-size: 12px;
                gridline-color: rgba(30, 42, 59, 0.4);
            }}
            QTableWidget::item {{
                padding: 6px;
                border-bottom: 1px solid rgba(30, 42, 59, 0.3);
            }}
            QTableWidget::item:selected {{
                background: rgba(0, 210, 255, 0.15);
                color: {TEXT};
                border-left: 3px solid {ACCENT};
            }}
            QHeaderView::section {{
                background: {PANEL_BG};
                color: {TEXT_DIM};
                border: none;
                border-bottom: 2px solid rgba(0, 210, 255, 0.3);
                padding: 10px;
                font-weight: 700;
                font-size: 11px;
                letter-spacing: 1px;
            }}
            QLabel#heroTitle {{
                color: {TEXT};
                font-size: 26px;
                font-weight: 900;
                letter-spacing: -0.5px;
            }}
            QLabel#heroSub {{
                color: {TEXT_DIM};
                font-size: 13px;
                letter-spacing: 0.5px;
            }}
            QFrame#metricCard {{
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 {PANEL_BG}, stop:1 {PANEL_ALT});
                border: 1px solid rgba(30, 42, 59, 0.8);
                border-radius: 8px;
            }}
            QProgressBar {{
                background-color: {PANEL_BG};
                border: 1px solid {BORDER};
                border-radius: 4px;
                text-align: center;
                color: transparent;
            }}
            QProgressBar::chunk {{
                background-color: {ACCENT};
                border-radius: 3px;
            }}
            QScrollBar:vertical {{
                border: none;
                background: {DARK_BG};
                width: 8px;
                margin: 0px 0px 0px 0px;
            }}
            QScrollBar::handle:vertical {{
                background: rgba(139, 155, 180, 0.3);
                min-height: 20px;
                border-radius: 4px;
            }}
            QScrollBar::handle:vertical:hover {{
                background: rgba(139, 155, 180, 0.5);
            }}
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
                height: 0px;
            }}
            """
        )

    def _build_ui(self) -> None:
        central = QWidget()
        outer = QVBoxLayout(central)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setStyleSheet(f"QScrollArea {{ background: {DARK_BG}; border: none; }}")
        content = QWidget()
        root = QVBoxLayout(content)
        root.setContentsMargins(16, 14, 16, 14)
        root.setSpacing(14)
        scroll.setWidget(content)
        outer.addWidget(scroll)
        self.setCentralWidget(central)

        header = QHBoxLayout()
        title_block = QVBoxLayout()
        title = QLabel("GPS-Denied Drone Swarm")
        title.setObjectName("heroTitle")
        subtitle = QLabel("Realtime monitoring, command orchestration, and EKF/V2X observability")
        subtitle.setObjectName("heroSub")
        title_block.addWidget(title)
        title_block.addWidget(subtitle)

        self._mode_label = QLabel(f"Backend: {self._backend.mode.upper()}")
        self._mode_label.setStyleSheet(f"color:{ACCENT}; font-size:12px; font-weight:700;")
        self._clock_label = QLabel()
        self._clock_label.setStyleSheet(f"color:{TEXT_DIM}; font-size:12px;")

        right_header = QVBoxLayout()
        right_header.addWidget(self._mode_label, alignment=Qt.AlignRight)
        right_header.addWidget(self._clock_label, alignment=Qt.AlignRight)

        header.addLayout(title_block)
        header.addStretch(1)
        header.addLayout(right_header)
        root.addLayout(header)



        cards = QGridLayout()
        cards.setHorizontalSpacing(12)
        cards.setVerticalSpacing(12)
        self._leader_card = MetricCard("LEADER", ACCENT_ALT)
        self._cpu_card = MetricCard("CPU TEMP", WARN)
        self._gpu_card = MetricCard("GPU LOAD", ACCENT)
        self._battery_card = MetricCard("SWARM BATTERY", SUCCESS)
        self._link_card = MetricCard("MESH HEALTH", CYAN)
        self._drift_card = MetricCard("AVG EKF DRIFT", MAGENTA)
        for idx, card in enumerate(
            [
                self._leader_card,
                self._cpu_card,
                self._gpu_card,
                self._battery_card,
                self._link_card,
                self._drift_card,
            ]
        ):
            cards.addWidget(card, 0, idx)
        root.addLayout(cards)

        body = QGridLayout()
        body.setHorizontalSpacing(14)
        body.setVerticalSpacing(14)

        map_box = QGroupBox("3D Map View")
        map_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        map_layout = QVBoxLayout(map_box)
        self._map = Map3DView()
        self._map.setMinimumHeight(360)
        map_layout.addWidget(self._map)
        body.addWidget(map_box, 0, 0, 2, 2)

        self._tabs = QTabWidget()
        self._tabs.setStyleSheet(f"QTabWidget::pane {{ border: 1px solid {BORDER}; border-radius: 4px; }} QTabBar::tab {{ background: {PANEL_BG}; color: {TEXT_DIM}; padding: 8px 16px; border: 1px solid {BORDER}; }} QTabBar::tab:selected {{ background: {PANEL_ALT}; color: {TEXT}; border-bottom-color: {ACCENT}; font-weight: bold; }}")
        self._tabs.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self._tabs.setMinimumHeight(200)

        self._drift = DriftGraph(self._ids, self._poll_hz)
        self._mcss_graph = MCSSScoreGraph(self._ids, self._poll_hz)
        self._net_graph = NetworkHealthGraph(self._poll_hz)
        self._comp_graph = ComputeLoadGraph(self._poll_hz)

        self._tabs.addTab(self._drift, "VIO Drift")
        self._tabs.addTab(self._mcss_graph, "MCSS Consensus")
        self._tabs.addTab(self._net_graph, "V2X Network")
        self._tabs.addTab(self._comp_graph, "Core Logic")

        body.addWidget(self._tabs, 0, 2, 1, 1)

        self._console = CommandConsole()
        body.addWidget(self._console, 1, 2, 1, 1)

        table_container = QWidget()
        table_container_layout = QVBoxLayout(table_container)
        table_container_layout.setContentsMargins(0, 0, 0, 0)
        table_container_layout.setSpacing(10)
        
        table_box = QGroupBox("Swarm Status Table")
        table_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        table_layout = QVBoxLayout(table_box)
        self._table = SwarmTable()
        self._table.setMinimumHeight(130)
        table_layout.addWidget(self._table)
        table_container_layout.addWidget(table_box)

        metrics_box = QGroupBox("Advanced Metrics (VIO, MCSS, V2X, Core Logic)")
        metrics_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        metrics_layout = QVBoxLayout(metrics_box)
        self._metrics_table = AdvancedMetricsTable()
        self._metrics_table.setMinimumHeight(130)
        metrics_layout.addWidget(self._metrics_table)
        table_container_layout.addWidget(metrics_box)

        body.addWidget(table_container, 2, 0, 1, 2)

        resource_box = QGroupBox("Jetson Health Cards")
        resource_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        resource_layout = QGridLayout(resource_box)
        self._cpu_health = MetricCard("CPU TEMP", WARN)
        self._gpu_health = MetricCard("GPU LOAD", ACCENT)
        self._loss_health = MetricCard("PACKET LOSS", DANGER)
        self._latency_health = MetricCard("AVG LATENCY", CYAN)
        self._motor_health = MetricCard("MOTOR HEALTH", SUCCESS)
        self._election_health = MetricCard("MCSS ELECTION", MAGENTA)
        resource_layout.addWidget(self._cpu_health, 0, 0)
        resource_layout.addWidget(self._gpu_health, 0, 1)
        resource_layout.addWidget(self._loss_health, 1, 0)
        resource_layout.addWidget(self._latency_health, 1, 1)
        resource_layout.addWidget(self._motor_health, 2, 0)
        resource_layout.addWidget(self._election_health, 2, 1)
        self._detail = DroneDetailPanel()
        side_stack = QVBoxLayout()
        side_widget = QWidget()
        side_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        side_widget.setLayout(side_stack)
        side_stack.setContentsMargins(0, 0, 0, 0)
        side_stack.setSpacing(14)
        side_stack.addWidget(resource_box)
        side_stack.addWidget(self._detail)
        side_stack.setStretch(0, 1)
        side_stack.setStretch(1, 1)
        body.addWidget(side_widget, 2, 2, 1, 1)

        body.setColumnStretch(0, 3)
        body.setColumnStretch(1, 2)
        body.setColumnStretch(2, 2)
        body.setRowStretch(0, 3)
        body.setRowStretch(1, 3)
        body.setRowStretch(2, 2)
        root.addLayout(body, 1)

        status = QStatusBar()
        status.setStyleSheet(f"QStatusBar {{ background:{PANEL_BG}; color:{TEXT_DIM}; }}")
        self._fps_label = QLabel("UI FPS: --")
        status.addPermanentWidget(self._fps_label)
        self.setStatusBar(status)

    def _wire_threads(self) -> None:
        self._telemetry.snapshot_ready.connect(self._on_snapshot)
        self._telemetry.log_ready.connect(self._console.append_log)
        self._commands.log_ready.connect(self._console.append_log)
        self._commands.command_completed.connect(self._on_command_completed)
        self._console.command_requested.connect(self._dispatch_command)
        self._table.itemSelectionChanged.connect(self._sync_selected_drone_from_table)

        self._clock = QTimer(self)
        self._clock.timeout.connect(self._update_clock)
        self._clock.start(500)

        self._fps_timer = QTimer(self)
        self._fps_timer.timeout.connect(self._flush_fps)
        self._fps_timer.start(1000)

        self._telemetry.start()
        self._commands.start()

    def _restore_persisted_state(self) -> None:
        for line in self._store.load_recent_commands(12):
            self._console.append_log(line)
        snapshot = self._store.load_snapshot()
        if snapshot is not None:
            self._on_snapshot(snapshot)

    def _update_clock(self) -> None:
        self._clock_label.setText(time.strftime("%Y-%m-%d  %H:%M:%S"))

    def _flush_fps(self) -> None:
        self._fps_label.setText(f"UI FPS: {self._frame_counter}")
        self._frame_counter = 0

    def _on_snapshot(self, snapshot: DashboardSnapshot) -> None:
        try:
            logger.debug("DashboardWindow on_snapshot states=%s backend=%s", len(snapshot.states), snapshot.backend_mode)
            self._last_snapshot = snapshot
            if (snapshot.timestamp - self._last_snapshot_persist_s) >= 1.0:
                self._store.save_snapshot(snapshot)
                self._last_snapshot_persist_s = snapshot.timestamp
            self._frame_counter += 1
            self._mode_label.setText(f"Backend: {snapshot.backend_mode.upper()}")

            self._map.ingest(snapshot.states)
            self._drift.ingest(snapshot.states)
            self._mcss_graph.ingest(snapshot.states)
            self._net_graph.ingest(snapshot.avg_latency_ms, snapshot.packet_loss_pct)
            self._comp_graph.ingest(snapshot.cpu_temp_c, snapshot.gpu_load_pct)
            self._table.ingest(snapshot.states)
            self._metrics_table.ingest(snapshot)
            self._select_row_for_drone()
            self._detail.ingest(self._selected_state(snapshot.states))

            leader_id = snapshot.leader_id if snapshot.leader_id else "?"
            avg_battery = sum(state.battery_pct for state in snapshot.states) / max(len(snapshot.states), 1)
            avg_drift = sum(state.drift_m for state in snapshot.states) / max(len(snapshot.states), 1)
            avg_motor = sum(state.motor_health for state in snapshot.states) / max(len(snapshot.states), 1)
            avg_localization = sum(state.localization_confidence for state in snapshot.states) / max(len(snapshot.states), 1)
            ready_count = sum(1 for state in snapshot.states if state.election_ready)
            online = sum(1 for state in snapshot.states if state.reachable)
            degraded_loc = sum(1 for state in snapshot.states if state.localization_state != "nominal")
            leader_state = next((state for state in snapshot.states if state.drone_id == snapshot.leader_id), None)
            leader_score = leader_state.leadership_score if leader_state is not None else 0.0

            self._leader_card.set_data(
                f"Drone {leader_id}",
                f"score {leader_score:.3f} | {online}/{len(snapshot.states)} nodes connected",
                accent=ACCENT_ALT if leader_score >= 0.55 else WARN,
            )
            self._cpu_card.set_data(
                f"{snapshot.cpu_temp_c:.1f} C",
                "Jetson CPU package",
                accent=WARN if snapshot.cpu_temp_c < 70 else DANGER,
            )
            self._gpu_card.set_data(
                f"{snapshot.gpu_load_pct:.0f}%",
                "CUDA / graphics load",
                accent=ACCENT if snapshot.gpu_load_pct < 85 else WARN,
            )
            self._battery_card.set_data(
                f"{avg_battery:.0f}%",
                "average swarm battery",
                accent=SUCCESS if avg_battery > 35 else WARN,
            )
            self._link_card.set_data(
                f"{online}/{len(snapshot.states)}",
                f"loss {snapshot.packet_loss_pct:.1f}% | latency {snapshot.avg_latency_ms:.1f} ms",
                accent=CYAN if snapshot.packet_loss_pct < 5 else WARN,
            )
            self._drift_card.set_data(
                f"{avg_localization:.2f}",
                f"localization confidence | drift {avg_drift:.3f} m",
                accent=MAGENTA if avg_localization >= 0.70 else WARN,
            )

            self._cpu_health.set_data(
                f"{snapshot.cpu_temp_c:.1f} C",
                "thermal headroom",
                accent=SUCCESS if snapshot.cpu_temp_c < 68 else WARN if snapshot.cpu_temp_c < 78 else DANGER,
            )
            self._gpu_health.set_data(
                f"{snapshot.gpu_load_pct:.0f}%",
                "GPU load",
                accent=ACCENT if snapshot.gpu_load_pct < 90 else WARN,
            )
            self._loss_health.set_data(
                f"{snapshot.packet_loss_pct:.1f}%",
                "mesh reliability",
                accent=SUCCESS if snapshot.packet_loss_pct < 2 else WARN if snapshot.packet_loss_pct < 5 else DANGER,
            )
            self._latency_health.set_data(
                f"{snapshot.avg_latency_ms:.1f} ms",
                "inter-drone latency",
                accent=CYAN if snapshot.avg_latency_ms < 10 else WARN,
            )
            self._motor_health.set_data(
                f"{avg_motor * 100.0:.0f}%",
                "average propulsion health",
                accent=SUCCESS if avg_motor >= 0.75 else WARN if avg_motor >= 0.55 else DANGER,
            )
            self._election_health.set_data(
                f"{ready_count}/{len(snapshot.states)}",
                snapshot.election_state.replace("-", " "),
                accent=MAGENTA if ready_count else DANGER,
            )

            self.statusBar().showMessage(
                f"Leader: {leader_id} | Nodes online: {online}/{len(snapshot.states)} | "
                f"Avg drift: {avg_drift:.3f} m | Loc degraded: {degraded_loc}/{len(snapshot.states)} | "
                f"Motor: {avg_motor * 100.0:.0f}% | Election-ready: {ready_count}/{len(snapshot.states)}"
            )
        except Exception as exc:
            logger.exception("snapshot render failed")
            self._console.append_log(f"snapshot render error: {exc}")
            self.statusBar().showMessage("Dashboard render degraded; check command log")

    def _selected_state(self, states: list[DroneState]) -> DroneState | None:
        if not states:
            return None
        if self._selected_drone_id is None:
            self._selected_drone_id = states[0].drone_id
        return next((state for state in states if state.drone_id == self._selected_drone_id), states[0])

    def _sync_selected_drone_from_table(self) -> None:
        selected = self._table.selectedItems()
        if not selected:
            return
        row = selected[0].row()
        item = self._table.item(row, 0)
        if item is None:
            return
        try:
            self._selected_drone_id = int(item.text())
        except ValueError:
            return
        logger.info("Dashboard selected drone id=%s", self._selected_drone_id)
        if self._last_snapshot is not None:
            self._detail.ingest(self._selected_state(self._last_snapshot.states))
        self._persist_settings()

    def _dispatch_command(self, request: CommandRequest) -> None:
        try:
            logger.info("Dashboard dispatch requested action=%s payload=%s", request.action, request.payload)
            request = CommandRequest(request.action, dict(request.payload))
            if request.action == "add_drone":
                suggested_id = (max(self._ids) + 1) if self._ids else 1
                dialog = AddDroneDialog(self._env_manager, suggested_id, self)
                if dialog.exec() != QDialog.Accepted:
                    self._console.append_log("add drone cancelled")
                    return
                request.payload.update(dialog.payload())
                if not bool(request.payload.get("demo_drone")):
                    drone_id = safe_int(request.payload.get("drone_id"), suggested_id)
                    self._env_manager.upsert_drone(
                        drone_id,
                        {key: safe_text(value) for key, value in request.payload.items()},
                    )
                    self._console.append_log(f"drone {drone_id} sensor profile saved to .env")
                else:
                    self._console.append_log("demo drone selected; .env left unchanged")
            if request.action == "remove_drone":
                selected = self._selected_state(self._last_snapshot.states if self._last_snapshot else [])
                if selected is None:
                    self._console.append_log("remove rejected: no drone selected")
                    return
                answer = QMessageBox.question(
                    self,
                    "Remove Drone",
                    f"Remove drone {selected.drone_id} and delete its drone-wise sensor connection from .env?",
                    QMessageBox.Yes | QMessageBox.No,
                    QMessageBox.No,
                )
                if answer != QMessageBox.Yes:
                    self._console.append_log("remove drone cancelled")
                    return
                request.payload["drone_id"] = selected.drone_id
                request.payload["target_ids"] = [selected.drone_id]
                request.payload["cluster_id"] = selected.cluster_id
                self._env_manager.remove_drone(selected.drone_id)
                self._console.append_log(f"drone {selected.drone_id} removed from .env")
                self._commands.submit(request)
                return
            if request.action == "add_drone":
                self._commands.submit(request)
                return
            scope = request.payload.get("scope", "Fleet")
            selected = self._selected_state(self._last_snapshot.states if self._last_snapshot else [])
            if scope == "Selected Drone":
                if selected is None:
                    self._console.append_log("command rejected: no drone selected")
                    return
                request.payload["target_ids"] = [selected.drone_id]
                request.payload["cluster_id"] = selected.cluster_id
            elif scope == "Selected Cluster":
                if selected is None:
                    self._console.append_log("command rejected: no cluster source available")
                    return
                request.payload["cluster_id"] = selected.cluster_id
            elif self._last_snapshot is not None:
                request.payload["target_ids"] = [state.drone_id for state in self._last_snapshot.states]
            self._commands.submit(request)
        except Exception as exc:
            logger.exception("command dispatch failed")
            self._console.append_log(f"command dispatch error: {exc}")

    def _on_command_completed(self, request_obj: CommandRequest, message: str) -> None:
        drone_id = safe_int(request_obj.payload.get("drone_id"), 0)
        if request_obj.action == "add_drone" and drone_id > 0 and drone_id not in self._ids:
            self._ids.append(drone_id)
            self._ids.sort()
            self._selected_drone_id = drone_id
        if request_obj.action == "remove_drone" and drone_id in self._ids:
            self._ids.remove(drone_id)
            if self._selected_drone_id == drone_id:
                self._selected_drone_id = self._ids[0] if self._ids else None
        self._store.save_command(request_obj, message)
        self._persist_settings()

    def _persist_settings(self) -> None:
        values = {
            "drone_ids": ",".join(str(drone_id) for drone_id in self._ids),
            "poll_hz": str(self._poll_hz),
            "backend_url": self._last_backend_url,
            "selected_drone_id": str(self._selected_drone_id or 0),
        }
        for drone_id, mission in self._backend.mission_overrides.items():
            values[f"mission_override_{drone_id}"] = mission
        self._store.save_settings(values)

    def _select_row_for_drone(self) -> None:
        if self._selected_drone_id is None:
            return
        for row in range(self._table.rowCount()):
            item = self._table.item(row, 0)
            if item is not None and item.text() == str(self._selected_drone_id):
                self._table.blockSignals(True)
                self._table.selectRow(row)
                self._table.blockSignals(False)
                return

    def closeEvent(self, event) -> None:  # noqa: N802
        try:
            self._persist_settings()
            self._telemetry.stop()
            self._commands.stop()
            self._telemetry.wait(1500)
            self._commands.wait(1500)
            self._backend.close()
        except Exception:
            logger.exception("dashboard shutdown failed")
        super().closeEvent(event)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="PySide6 realtime swarm dashboard")
    parser.add_argument("--ids", default=os.environ.get("DRONE_DASHBOARD_IDS", ""), help="comma-separated drone ids")
    parser.add_argument("--poll-hz", type=int, default=int(os.environ.get("DRONE_DASHBOARD_POLL_HZ", "0")), help="telemetry polling rate")
    parser.add_argument("--backend-url", default=os.environ.get("DRONE_BACKEND_URL", ""), help="Go control-plane base URL for fleet mode")
    return parser.parse_args()


def configure_logging() -> None:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    root = logging.getLogger()
    root.setLevel(logging.DEBUG)
    root.handlers.clear()

    formatter = logging.Formatter("%(asctime)s %(levelname)s %(name)s: %(message)s")

    console = logging.StreamHandler(sys.stdout)
    console.setLevel(logging.INFO)
    console.setFormatter(formatter)

    file_handler = RotatingFileHandler(
        LOG_DIR / "dashboard.log",
        maxBytes=5 * 1024 * 1024,
        backupCount=5,
        encoding="utf-8",
    )
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(formatter)

    root.addHandler(console)
    root.addHandler(file_handler)
    logger.info("Dashboard logging initialized file=%s", LOG_DIR / "dashboard.log")


def main() -> int:
    bootstrap_env()
    configure_logging()
    try:
        args = parse_args()
        store = DashboardStore(STORE_PATH)
        drone_ids = [safe_int(part.strip(), -1) for part in args.ids.split(",") if part.strip()]
        drone_ids = [drone_id for drone_id in drone_ids if drone_id > 0]
        if not drone_ids:
            drone_ids = [1, 2, 3, 4, 5]
        poll_hz = args.poll_hz if args.poll_hz > 0 else 20

        pg.setConfigOptions(antialias=False, useOpenGL=True)
        app = QApplication(sys.argv)
        app.setApplicationName("DroneSwarmDashboard")

        backend_url = args.backend_url.strip() or None
        window = DashboardWindow(drone_ids, poll_hz, backend_url=backend_url, store=store)
        window.show()
        return app.exec()
    except Exception as exc:
        logger.exception("dashboard startup failed")
        print(f"dashboard startup failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
