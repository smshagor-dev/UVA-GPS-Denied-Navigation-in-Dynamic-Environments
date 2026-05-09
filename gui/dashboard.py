#!/usr/bin/env python3
# System Designer and Developer: Md Shahanur Islam Shagor
# Project: UVA GPS Denied Navigation in Dynamic Environments
# Technology: C++, Python, Go, CMake

"""Realtime monitoring and control dashboard for the drone swarm project."""

from __future__ import annotations

import argparse
from datetime import datetime, timedelta, timezone
import hashlib
import hmac
import json
import logging
from logging.handlers import RotatingFileHandler
import math
import os
import queue
import random
import secrets
import sqlite3
import ssl
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
from PySide6.QtGui import QColor, QImage, QImageReader, QPainter, QPalette
from PySide6.QtSvg import QSvgRenderer
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
    QHeaderView,
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
ASSET_DIR = Path(__file__).resolve().parent.parent / "assets"


def _rgba(hex_color: str, alpha: float) -> tuple[int, int, int, int]:
    color = QColor(hex_color)
    return (color.red(), color.green(), color.blue(), max(0, min(255, int(alpha * 255))))


def _qimage_to_gl_image(image: QImage) -> np.ndarray:
    converted = image.convertToFormat(QImage.Format.Format_RGBA8888)
    width = converted.width()
    height = converted.height()
    buffer = converted.bits().tobytes()
    array = np.frombuffer(buffer, dtype=np.uint8).reshape((height, width, 4))
    return np.ascontiguousarray(np.flipud(array))


def _load_raster_gl_image(path: Path, size: tuple[int, int]) -> np.ndarray:
    reader = QImageReader(str(path))
    image = reader.read()
    if image.isNull():
        raise ValueError(f"failed to load image: {path}")
    scaled = image.scaled(
        size[0],
        size[1],
        Qt.AspectRatioMode.KeepAspectRatioByExpanding,
        Qt.TransformationMode.SmoothTransformation,
    )
    return _qimage_to_gl_image(scaled)


def _load_svg_gl_image(path: Path, size: tuple[int, int]) -> np.ndarray:
    renderer = QSvgRenderer(str(path))
    if not renderer.isValid():
        raise ValueError(f"failed to load svg: {path}")
    image = QImage(size[0], size[1], QImage.Format.Format_RGBA8888)
    image.fill(Qt.GlobalColor.transparent)
    painter = QPainter(image)
    try:
        renderer.render(painter)
    finally:
        painter.end()
    return _qimage_to_gl_image(image)


def _colorize_gl_image(image: np.ndarray, rgba: tuple[int, int, int, int]) -> np.ndarray:
    colored = image.copy()
    alpha_mask = colored[:, :, 3] > 0
    colored[:, :, 0][alpha_mask] = rgba[0]
    colored[:, :, 1][alpha_mask] = rgba[1]
    colored[:, :, 2][alpha_mask] = rgba[2]
    colored[:, :, 3][alpha_mask] = np.clip((colored[:, :, 3][alpha_mask].astype(np.float32) * (rgba[3] / 255.0)), 0, 255).astype(np.uint8)
    return colored


def style_plot_widget(widget: pg.PlotWidget, left_label: str, bottom_label: str = "Time (s)") -> pg.LegendItem:
    widget.setBackground(PANEL_ALT)
    widget.showGrid(x=True, y=True, alpha=0.18)
    widget.setMouseEnabled(x=False, y=False)
    widget.setMenuEnabled(False)
    widget.setAntialiasing(True)
    widget.getPlotItem().setClipToView(True)
    widget.getPlotItem().setDownsampling(mode="peak")
    widget.getPlotItem().layout.setContentsMargins(12, 10, 12, 12)
    widget.getPlotItem().vb.setDefaultPadding(0.04)
    widget.setLabel("left", left_label, color=TEXT_DIM)
    widget.setLabel("bottom", bottom_label, color=TEXT_DIM)
    widget.getAxis("left").setPen(pg.mkPen(color=_rgba(BORDER, 0.9), width=1))
    widget.getAxis("bottom").setPen(pg.mkPen(color=_rgba(BORDER, 0.9), width=1))
    widget.getAxis("left").setTextPen(pg.mkPen(TEXT_DIM))
    widget.getAxis("bottom").setTextPen(pg.mkPen(TEXT_DIM))
    legend = widget.addLegend(offset=(-14, 14))
    legend.anchor((1, 0), (1, 0))
    legend.setBrush(pg.mkBrush(_rgba(PANEL_BG, 0.88)))
    legend.setPen(pg.mkPen(_rgba(BORDER, 0.9)))
    return legend


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
    commanded_altitude_m: float = 0.0
    commanded_speed_mps: float = 0.0
    manual_target_position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    manual_control_active: bool = False
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
    peer_clock_offset_ms: float = 0.0
    anchor_visibility_ratio: float = 0.0
    tdoa_weight: float = 0.0
    planned_waypoint_count: int = 0
    last_relocalized_keyframe: int = 0
    security_state: str = "TRUSTED"
    security_summary: str = "All trust signals nominal"
    security_transition_reason: str = "initial-trust"
    remote_command_allowed: bool = True
    telemetry_uplink_allowed: bool = True
    link_integrity_score: float = 1.0
    trust_epoch: int = 1
    last_auth_failure_at_s: float = 0.0
    tamper_score: float = 0.0
    firmware_measurement: str = "lab-local-build"
    firmware_version: str = "0.0.0"
    secure_boot_state: str = "LAB_BOOT"
    boot_trust_summary: str = "Lab boot trust bypassed"
    rollback_counter: int = 0
    maintenance_mode: bool = False
    update_channel_state: str = "idle"
    last_remote_command_status: str = "no remote command"
    health_flags: list[str] = field(default_factory=list)
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
    clusters: list[dict[str, Any]] = field(default_factory=list)
    critical_alerts: int = 0
    health: dict[str, Any] = field(default_factory=dict)
    missions: list[dict[str, Any]] = field(default_factory=list)
    events: list[dict[str, Any]] = field(default_factory=list)
    services: list[str] = field(default_factory=list)
    timestamp: float = field(default_factory=time.time)


@dataclass(slots=True)
class CommandRequest:
    action: str
    payload: dict[str, Any] = field(default_factory=dict)


class GoControlPlaneClient:
    def __init__(self, base_url: str) -> None:
        self._base_url = base_url.rstrip("/")
        self._security_profile = normalize_security_profile(os.environ.get("DRONE_SECURITY_PROFILE", "lab"))
        self._require_signed_commands = parse_env_bool("DRONE_REQUIRE_SIGNED_COMMANDS", False)
        self._operator_id = os.environ.get("DRONE_OPERATOR_ID", "").strip()
        self._operator_role = normalize_operator_role(os.environ.get("DRONE_OPERATOR_ROLE", "operator"))
        self._operator_secret = os.environ.get("DRONE_OPERATOR_SECRET", "").strip()
        self._command_ttl_sec = max(safe_int(os.environ.get("DRONE_COMMAND_TTL_SEC", "90"), 90), 5)
        self._cache: dict[str, tuple[float, dict[str, Any]]] = {}
        validate_backend_security_profile(self._base_url, self._security_profile)
        self._ssl_context = build_backend_ssl_context(self._base_url)

    def fetch_snapshot(self) -> DashboardSnapshot:
        logger.debug("GoControlPlaneClient.fetch_snapshot url=%s", self._base_url)
        payload = self._get_json("/api/v1/fleet")
        health_payload = self._get_json_cached("/api/v1/health", ttl_sec=2.0)
        missions_payload = self._get_json_cached("/api/v1/missions", ttl_sec=3.0)
        events_payload = self._get_json_cached("/api/v1/events", ttl_sec=2.0)
        discovery_payload = self._get_json_cached("/api/v1/discovery", ttl_sec=5.0)
        drones = payload.get("drones", [])
        states = [
            DroneState(
                drone_id=safe_int(item.get("drone_id", 0), 0),
                cluster_id=safe_text(item.get("cluster_id", "cluster-01"), "cluster-01"),
                role=safe_text(item.get("role", "FOLLOWER"), "FOLLOWER"),
                mission_state=safe_text(item.get("mission_state", "standby"), "standby"),
                position=safe_vector3(item.get("position", [0.0, 0.0, 0.0])),
                velocity=safe_vector3(item.get("velocity", [0.0, 0.0, 0.0])),
                commanded_altitude_m=safe_float(item.get("commanded_altitude_m", 0.0)),
                commanded_speed_mps=safe_float(item.get("commanded_speed_mps", 0.0)),
                manual_target_position=safe_vector3(item.get("manual_target_position", [0.0, 0.0, 0.0])),
                manual_control_active=bool(item.get("manual_control_active", False)),
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
                peer_clock_offset_ms=safe_float(item.get("peer_clock_offset_ms", 0.0), 0.0),
                anchor_visibility_ratio=safe_float(item.get("anchor_visibility_ratio", 0.0), 0.0),
                tdoa_weight=safe_float(item.get("tdoa_weight", 0.0), 0.0),
                planned_waypoint_count=safe_int(item.get("planned_waypoint_count", 0), 0),
                last_relocalized_keyframe=safe_int(item.get("last_relocalized_keyframe", 0), 0),
                security_state=safe_text(item.get("security_state", "TRUSTED"), "TRUSTED"),
                security_summary=safe_text(item.get("security_summary", "All trust signals nominal"), "All trust signals nominal"),
                security_transition_reason=safe_text(item.get("security_transition_reason", "initial-trust"), "initial-trust"),
                remote_command_allowed=bool(item.get("remote_command_allowed", True)),
                telemetry_uplink_allowed=bool(item.get("telemetry_uplink_allowed", True)),
                link_integrity_score=safe_float(item.get("link_integrity_score", 1.0), 1.0),
                trust_epoch=safe_int(item.get("trust_epoch", 1), 1),
                last_auth_failure_at_s=safe_float(item.get("last_auth_failure_at_s", 0.0), 0.0),
                tamper_score=safe_float(item.get("tamper_score", 0.0), 0.0),
                firmware_measurement=safe_text(item.get("firmware_measurement", "lab-local-build"), "lab-local-build"),
                firmware_version=safe_text(item.get("firmware_version", "0.0.0"), "0.0.0"),
                secure_boot_state=safe_text(item.get("secure_boot_state", "LAB_BOOT"), "LAB_BOOT"),
                boot_trust_summary=safe_text(item.get("boot_trust_summary", "Lab boot trust bypassed"), "Lab boot trust bypassed"),
                rollback_counter=safe_int(item.get("rollback_counter", 0), 0),
                maintenance_mode=bool(item.get("maintenance_mode", False)),
                update_channel_state=safe_text(item.get("update_channel_state", "idle"), "idle"),
                last_remote_command_status=safe_text(item.get("last_remote_command_status", "no remote command"), "no remote command"),
                health_flags=[safe_text(v) for v in item.get("health_flags", []) if safe_text(v)],
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
            clusters=[
                {
                    "cluster_id": safe_text(item.get("cluster_id"), "cluster-01"),
                    "leader_id": safe_int(item.get("leader_id"), 0),
                    "drone_count": safe_int(item.get("drone_count"), 0),
                    "formation": safe_text(item.get("formation"), "unknown"),
                    "mission_state": safe_text(item.get("mission_state"), "unknown"),
                    "avg_battery": safe_float(item.get("avg_battery", 0.0)),
                }
                for item in payload.get("clusters", [])
                if isinstance(item, dict)
            ],
            critical_alerts=safe_int(payload.get("critical_alerts", health_payload.get("critical_alerts", 0)), 0),
            health=health_payload if isinstance(health_payload, dict) else {},
            missions=[
                item for item in missions_payload.get("missions", [])
                if isinstance(item, dict)
            ][:12],
            events=[
                item for item in events_payload.get("events", [])
                if isinstance(item, dict)
            ][-16:],
            services=[safe_text(item) for item in discovery_payload.get("services", []) if safe_text(item)],
            timestamp=safe_float(payload.get("timestamp", time.time()), time.time()),
        )

    def fetch_pending_approvals(self) -> dict[str, str]:
        logger.debug("GoControlPlaneClient.fetch_pending_approvals url=%s", self._base_url)
        payload = self._get_json("/api/v1/approvals")
        approvals = payload.get("approvals", [])
        out: dict[str, str] = {}
        for item in approvals:
            if not isinstance(item, dict):
                continue
            action = safe_text(item.get("action"), "").lower()
            approval_id = safe_text(item.get("approval_id"), "")
            if action and approval_id:
                out[action] = approval_id
        return out

    def send_command(self, request_obj: CommandRequest) -> str:
        logger.info("GoControlPlaneClient.send_command action=%s", request_obj.action)
        body = json.dumps(self._build_command_body(request_obj), separators=(",", ":"), ensure_ascii=False).encode("utf-8")
        req = request.Request(
            self._base_url + "/api/v1/commands",
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with request.urlopen(req, timeout=2.5, context=self._ssl_context) as response:
            payload = json.loads(response.read().decode("utf-8"))
        if isinstance(payload, dict):
            if bool(payload.get("approval_required")):
                request_obj.payload["_approval_required"] = True
                request_obj.payload["_approval_id"] = safe_text(payload.get("approval_id"), "")
            else:
                request_obj.payload.pop("_approval_required", None)
                request_obj.payload.pop("_approval_id", None)
        return str(payload.get("message", "command accepted"))

    def _build_command_body(self, request_obj: CommandRequest) -> dict[str, Any]:
        if not self._require_signed_commands and not security_profile_requires_signed(self._security_profile):
            return {
                "action": request_obj.action,
                "payload": request_obj.payload,
            }
        payload_json = json.dumps(request_obj.payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
        if not self._operator_id or not self._operator_secret:
            if security_profile_requires_signed(self._security_profile):
                raise ValueError("signed command security is required but DRONE_OPERATOR_ID/DRONE_OPERATOR_SECRET is missing")
            return {
                "action": request_obj.action,
                "payload": request_obj.payload,
            }

        issued_at = datetime.now(timezone.utc)
        expires_at = issued_at + timedelta(seconds=self._command_ttl_sec)
        issued_at_text = issued_at.isoformat(timespec="microseconds").replace("+00:00", "Z")
        expires_at_text = expires_at.isoformat(timespec="microseconds").replace("+00:00", "Z")
        nonce = secrets.token_hex(16)
        signature = sign_command_envelope(
            self._operator_secret,
            request_obj.action,
            payload_json,
            self._operator_id,
            self._operator_role,
            issued_at_text,
            expires_at_text,
            nonce,
        )
        return {
            "action": request_obj.action,
            "payload_json": payload_json,
            "auth": {
                "operator_id": self._operator_id,
                "operator_role": self._operator_role,
                "issued_at": issued_at_text,
                "expires_at": expires_at_text,
                "nonce": nonce,
                "signature": signature,
            },
        }

    def _get_json(self, path: str) -> dict[str, Any]:
        with request.urlopen(self._base_url + path, timeout=2.5, context=self._ssl_context) as response:
            payload = json.loads(response.read().decode("utf-8"))
        if not isinstance(payload, dict):
            raise ValueError(f"unexpected payload type for {path}")
        return payload

    def _get_json_cached(self, path: str, ttl_sec: float) -> dict[str, Any]:
        now = time.monotonic()
        cached = self._cache.get(path)
        if cached is not None and (now - cached[0]) < ttl_sec:
            return cached[1]
        try:
            payload = self._get_json(path)
        except Exception as exc:
            logger.debug("GoControlPlaneClient optional fetch failed path=%s error=%s", path, exc)
            return cached[1] if cached is not None else {}
        self._cache[path] = (now, payload)
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
        self._security_profile = normalize_security_profile(os.environ.get("DRONE_SECURITY_PROFILE", "lab"))
        self._pipelines: dict[int, Any] = {}
        self._network = None
        self._control_ready = False
        self._started_at = time.time()
        self._client = GoControlPlaneClient(backend_url) if backend_url else None
        self._mission_overrides: dict[int, str] = dict(initial_mission_overrides or {})
        self._firmware_overrides: dict[int, dict[str, Any]] = {}
        self._local_demo_ids: set[int] = set()
        self._live_positions: dict[int, tuple[float, float, float]] = {}
        self._manual_targets: dict[int, tuple[float, float, float]] = {}
        self._manual_speeds: dict[int, float] = {}
        self._last_manual_tick = time.monotonic()
        logger.info("SwarmBackend init ids=%s poll_hz=%s backend_url=%s", drone_ids, poll_hz, backend_url)

        if self._client is not None:
            self._mode = "go-control-plane"
            self._control_ready = True

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
                self._configure_network_security()
                network_ready = bool(self._network.start())
                self._control_ready = self._control_ready or network_ready
                if network_ready and self._client is None:
                    self._mode = "hybrid"
                elif network_ready:
                    logger.info("SwarmBackend local mesh sidecar ready while using go-control-plane mode")
            except Exception:
                self._network = None
                if self._client is None:
                    self._control_ready = False

    @property
    def mode(self) -> str:
        return self._mode

    @property
    def mission_overrides(self) -> dict[int, str]:
        return dict(self._mission_overrides)

    def _configure_network_security(self) -> None:
        if self._network is None or self._bridge is None:
            return
        if not hasattr(self._bridge, "SwarmSecurityConfig") or not hasattr(self._network, "configure_security"):
            logger.info("SwarmBackend bridge lacks swarm security bindings; mesh commands stay unsecured")
            return
        secret = os.environ.get("DRONE_SWARM_SECRET", "").strip() or "drone-swarm-dev-secret-change-me"
        if secret == "drone-swarm-dev-secret-change-me":
            logger.warning("SwarmBackend mesh sidecar using development swarm secret fallback")
        try:
            security_cfg = self._bridge.SwarmSecurityConfig()
            security_cfg.enabled = True
            security_cfg.swarm_secret = secret
            self._network.configure_security(security_cfg)
            enabled = bool(self._network.security_enabled()) if hasattr(self._network, "security_enabled") else True
            last_error = self._network.security_last_error() if hasattr(self._network, "security_last_error") else ""
            logger.info("SwarmBackend secure mesh sidecar configured enabled=%s error=%s", enabled, last_error)
        except Exception:
            logger.exception("SwarmBackend failed to configure mesh sidecar security")

    def _apply_firmware_overrides(self, state: DroneState) -> DroneState:
        override = self._firmware_overrides.get(state.drone_id)
        if not override:
            return state
        state.firmware_measurement = safe_text(override.get("firmware_measurement", state.firmware_measurement), state.firmware_measurement)
        state.firmware_version = safe_text(override.get("firmware_version", state.firmware_version), state.firmware_version)
        state.secure_boot_state = safe_text(override.get("secure_boot_state", state.secure_boot_state), state.secure_boot_state)
        state.boot_trust_summary = safe_text(override.get("boot_trust_summary", state.boot_trust_summary), state.boot_trust_summary)
        state.rollback_counter = safe_int(override.get("rollback_counter", state.rollback_counter), state.rollback_counter)
        state.maintenance_mode = bool(override.get("maintenance_mode", state.maintenance_mode))
        state.update_channel_state = safe_text(override.get("update_channel_state", state.update_channel_state), state.update_channel_state)
        return state

    def pending_approvals(self) -> dict[str, str]:
        if self._client is None:
            return {}
        try:
            return self._client.fetch_pending_approvals()
        except (error.URLError, TimeoutError, ValueError, KeyError) as exc:
            logger.warning("go-control-plane approvals fetch failed: %s", exc)
            return {}

    def _mirror_backend_command(self, request: CommandRequest) -> None:
        if self._network is None or self._bridge is None:
            return
        try:
            if request.action == "emergency_land":
                msg_type = self._bridge.SwarmMessageType.EMERGENCY_STOP
                ok = bool(self._network.broadcast(msg_type, []))
                logger.info("SwarmBackend mirrored emergency_land to local mesh ok=%s", ok)
                return
            if request.action == "election":
                self._network.trigger_election()
                logger.info("SwarmBackend mirrored election to local mesh")
                return
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
                logger.info("SwarmBackend mirrored formation to local mesh ok=%s shape=%s", ok, shape_name)
        except Exception:
            logger.exception("SwarmBackend failed to mirror backend command action=%s", request.action)

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
        self._live_positions.pop(drone_id, None)
        self._manual_targets.pop(drone_id, None)
        self._manual_speeds.pop(drone_id, None)
        logger.info("SwarmBackend removed local drone id=%s total=%s", drone_id, len(self._ids))
        return True

    def _clear_manual_targets(self, drone_ids: list[int]) -> None:
        for drone_id in drone_ids:
            self._manual_targets.pop(drone_id, None)
            self._manual_speeds.pop(drone_id, None)

    @staticmethod
    def _critical_alert_count(states: list[DroneState]) -> int:
        return sum(
            1
            for state in states
            if (not state.reachable)
            or state.battery_pct < 15.0
            or state.cpu_temp_c > 82.0
            or state.localization_state == "lost"
            or state.sync_confidence < 0.35
            or state.security_state not in {"", "TRUSTED", "DEGRADED_LINK"}
        )

    @staticmethod
    def _cluster_snapshot(states: list[DroneState]) -> list[dict[str, Any]]:
        clusters: dict[str, dict[str, Any]] = {}
        for state in states:
            cluster = clusters.setdefault(
                state.cluster_id,
                {
                    "cluster_id": state.cluster_id,
                    "leader_id": 0,
                    "drone_count": 0,
                    "formation": "dynamic",
                    "mission_state": state.mission_state,
                    "avg_battery": 0.0,
                },
            )
            cluster["drone_count"] += 1
            cluster["avg_battery"] += state.battery_pct
            if state.role == "LEADER" and not cluster["leader_id"]:
                cluster["leader_id"] = state.drone_id
        out = list(clusters.values())
        for cluster in out:
            count = max(1, safe_int(cluster["drone_count"], 1))
            cluster["avg_battery"] = safe_float(cluster["avg_battery"], 0.0) / count
        out.sort(key=lambda item: safe_text(item.get("cluster_id"), ""))
        return out

    @classmethod
    def _health_snapshot(cls, states: list[DroneState], cpu_temp_c: float) -> dict[str, Any]:
        avg_battery = sum(state.battery_pct for state in states) / max(len(states), 1)
        return {
            "online_drones": sum(1 for state in states if state.reachable),
            "total_drones": len(states),
            "critical_alerts": cls._critical_alert_count(states),
            "avg_battery_pct": avg_battery,
            "max_cpu_temp_c": max([cpu_temp_c] + [state.cpu_temp_c for state in states]) if states else cpu_temp_c,
            "updated_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        }

    def _set_manual_target(self, drone_id: int, target: tuple[float, float, float], speed: float) -> None:
        self._manual_targets[drone_id] = target
        self._manual_speeds[drone_id] = max(0.5, speed)
        self._mission_overrides[drone_id] = "remote-nav"

    def _apply_manual_step(self, request: CommandRequest, target_ids: list[int]) -> str:
        step = max(0.2, safe_float(request.payload.get("step_m"), 1.5))
        speed = max(0.5, safe_float(request.payload.get("velocity_mps"), 3.0))
        deltas = {
            "move_left": (-step, 0.0, 0.0),
            "move_right": (step, 0.0, 0.0),
            "move_up": (0.0, step, 0.0),
            "move_down": (0.0, -step, 0.0),
        }
        dx, dy, dz = deltas[request.action]
        for drone_id in target_ids:
            base = self._manual_targets.get(drone_id, self._live_positions.get(drone_id, (0.0, 0.0, max(4.0, safe_float(request.payload.get("altitude_m"), 8.0)))))
            target = (
                base[0] + dx,
                base[1] + dy,
                max(0.4, base[2] + dz),
            )
            self._set_manual_target(drone_id, target, speed)
        direction = request.action.removeprefix("move_")
        return f"{direction} move applied to {len(target_ids)} drone(s)"

    def _apply_manual_navigation(self, snapshot: DashboardSnapshot) -> DashboardSnapshot:
        now = time.monotonic()
        dt = max(1.0 / self._poll_hz, min(0.25, now - self._last_manual_tick))
        self._last_manual_tick = now
        active_ids = {state.drone_id for state in snapshot.states}
        for drone_id in list(self._manual_targets):
            if drone_id not in active_ids:
                self._manual_targets.pop(drone_id, None)
                self._manual_speeds.pop(drone_id, None)
        if not self._manual_targets:
            for state in snapshot.states:
                self._live_positions[state.drone_id] = state.position
            return snapshot

        for state in snapshot.states:
            target = self._manual_targets.get(state.drone_id)
            if target is None:
                self._live_positions[state.drone_id] = state.position
                continue
            current = np.array(self._live_positions.get(state.drone_id, state.position), dtype=float)
            desired = np.array(target, dtype=float)
            delta = desired - current
            distance = float(np.linalg.norm(delta))
            if distance <= 0.04:
                next_pos = desired
                velocity = np.zeros(3, dtype=float)
                self._mission_overrides[state.drone_id] = "hold-position"
            else:
                speed = self._manual_speeds.get(state.drone_id, 3.0)
                step = min(distance, speed * dt)
                direction = delta / max(distance, 1e-6)
                next_pos = current + direction * step
                velocity = direction * (step / max(dt, 1e-6))
                self._mission_overrides[state.drone_id] = "remote-nav"
            state.position = (float(next_pos[0]), float(next_pos[1]), float(next_pos[2]))
            state.velocity = (float(velocity[0]), float(velocity[1]), float(velocity[2]))
            state.commanded_altitude_m = float(desired[2])
            state.commanded_speed_mps = float(self._manual_speeds.get(state.drone_id, 3.0))
            state.manual_target_position = (float(desired[0]), float(desired[1]), float(desired[2]))
            state.manual_control_active = True
            state.attitude_rpy, state.thrust_vector = derive_attitude_thrust(state.position, state.velocity, state.role, time.time() - self._started_at)
            state.mission_state = self._mission_overrides.get(state.drone_id, state.mission_state)
            self._live_positions[state.drone_id] = state.position
        return snapshot

    def _local_demo_state(self, drone_id: int, index: int, snapshot: DashboardSnapshot | None) -> DroneState:
        now = time.time()
        elapsed = now - self._started_at
        radius = 2.4 + index * 1.3
        angle = elapsed * (0.22 + index * 0.015) + index * 0.85
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        z = 2.0 + 0.8 * index + 0.45 * math.sin(elapsed * 0.5 + index)
        vx = -radius * math.sin(angle) * 0.22
        vy = radius * math.cos(angle) * 0.22
        cluster_id = f"cluster-{((drone_id - 1) // 20) + 1:02d}"
        current_states = snapshot.states if snapshot is not None else []
        has_cluster_leader = any(
            state.cluster_id == cluster_id and state.role == "LEADER"
            for state in current_states
        )
        role = "FOLLOWER" if has_cluster_leader else "LEADER"
        attitude_rpy, thrust_vector = derive_attitude_thrust((x, y, z), (vx, vy, 0.0), role, elapsed)
        mission_state = self._mission_overrides.get(
            drone_id,
            "patrol" if role == "LEADER" else "formation-hold",
        )
        cpu_temp_c = snapshot.cpu_temp_c if snapshot is not None else 54.0
        gpu_load_pct = snapshot.gpu_load_pct if snapshot is not None else 38.0
        return self._apply_firmware_overrides(DroneState(
            drone_id=drone_id,
            cluster_id=cluster_id,
            role=role,
            mission_state=mission_state,
            position=(x, y, z),
            velocity=(vx, vy, 0.0),
            commanded_altitude_m=z,
            commanded_speed_mps=2.8,
            manual_target_position=(x + 0.5, y + 0.5, z),
            manual_control_active=False,
            attitude_rpy=attitude_rpy,
            thrust_vector=thrust_vector,
            drift_m=0.04 + 0.03 * index + abs(math.sin(elapsed * 0.2 + index)) * 0.12,
            battery_pct=max(18.0, 100.0 - elapsed * (0.22 + index * 0.01)),
            connectivity="Mesh",
            reachable=True,
            rssi_dbm=-48.0 - index * 3.2 + 1.6 * math.sin(elapsed + index),
            cpu_temp_c=cpu_temp_c,
            gpu_load_pct=max(0.0, gpu_load_pct),
            localization_source="demo-visualization",
            localization_state="nominal",
            localization_confidence=max(0.18, 0.95 - index * 0.07 - 0.18 * abs(math.sin(elapsed * 0.18 + index))),
            tdoa_confidence=max(0.0, min(1.0, 0.72 - index * 0.08 + 0.06 * math.sin(elapsed * 0.31 + index))),
            confidence_trend=0.04 * math.sin(elapsed * 0.18 + index),
            relocalization_count=max(0, index - 1),
            visible_anchor_count=max(0, 5 - index),
            occupancy_ratio=min(1.0, 0.12 + index * 0.04 + 0.03 * abs(math.sin(elapsed * 0.22 + index))),
            sync_confidence=max(0.25, 0.94 - index * 0.06 - 0.08 * abs(math.sin(elapsed * 0.27 + index))),
            imu_camera_offset_ms=(1.5 + index * 0.9) * math.sin(elapsed * 0.14 + index),
            peer_clock_offset_ms=1.2 * math.sin(elapsed * 0.16 + index),
            anchor_visibility_ratio=max(0.0, min(1.0, (5 - index) / 6.0)),
            tdoa_weight=0.58,
            planned_waypoint_count=max(1, 6 - index),
            last_relocalized_keyframe=64 - index,
            security_state="TRUSTED",
            security_summary="Demo drone rendered from local dashboard state",
            security_transition_reason="dashboard-demo-state",
            remote_command_allowed=True,
            telemetry_uplink_allowed=True,
            link_integrity_score=0.90,
            trust_epoch=2 + index,
            last_auth_failure_at_s=0.0,
            tamper_score=0.02 * index,
            firmware_measurement=f"fw-demo-{drone_id}",
            firmware_version="2.0.0",
            secure_boot_state="SECURE_BOOT_TRUSTED",
            boot_trust_summary="Demo drone trust profile active",
            rollback_counter=1,
            maintenance_mode=False,
            update_channel_state="idle",
            last_remote_command_status="dashboard demo add",
            health_flags=[],
            motor_health=max(0.35, 0.94 - index * 0.04 - 0.03 * abs(math.sin(elapsed * 0.23 + index))),
            leadership_score=max(0.1, min(0.98, 0.58 - index * 0.03 + (0.16 if role == "LEADER" else 0.0))),
            election_ready=True,
            timestamp=now,
        ))

    def _merge_local_demo_snapshot(self, snapshot: DashboardSnapshot) -> DashboardSnapshot:
        missing_ids = [drone_id for drone_id in sorted(self._local_demo_ids) if all(state.drone_id != drone_id for state in snapshot.states)]
        if not missing_ids:
            return snapshot
        states = list(snapshot.states)
        base_index = len(states)
        for offset, drone_id in enumerate(missing_ids):
            states.append(self._local_demo_state(drone_id, base_index + offset, snapshot))
        states.sort(key=lambda state: state.drone_id)
        leader_id = snapshot.leader_id
        if leader_id is None:
            leader = next((state for state in states if state.role == "LEADER"), None)
            leader_id = leader.drone_id if leader is not None else (states[0].drone_id if states else None)
        return DashboardSnapshot(
            states=states,
            backend_mode=snapshot.backend_mode,
            leader_id=leader_id,
            avg_latency_ms=snapshot.avg_latency_ms,
            packet_loss_pct=snapshot.packet_loss_pct,
            cpu_temp_c=snapshot.cpu_temp_c,
            gpu_load_pct=snapshot.gpu_load_pct,
            election_state=snapshot.election_state,
            clusters=self._cluster_snapshot(states),
            critical_alerts=self._critical_alert_count(states),
            health=self._health_snapshot(states, snapshot.cpu_temp_c),
            missions=snapshot.missions,
            events=snapshot.events,
            services=snapshot.services,
            timestamp=snapshot.timestamp,
        )

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
            self._clear_manual_targets(target_ids)
            shape = str(request.payload.get("shape", "DIAMOND")).lower()
            for drone_id in target_ids:
                self._mission_overrides[drone_id] = f"formation-{shape}"
            return f"{shape.upper()} formation assigned to {len(target_ids)} drone(s)"

        if request.action == "hold_position":
            for drone_id in target_ids:
                self._set_manual_target(drone_id, self._live_positions.get(drone_id, (0.0, 0.0, safe_float(request.payload.get("altitude_m"), 8.0))), safe_float(request.payload.get("velocity_mps"), 1.5))
                self._mission_overrides[drone_id] = "hold-position"
            return f"hold position applied to {len(target_ids)} drone(s)"

        if request.action in {"move_left", "move_right", "move_up", "move_down"}:
            return self._apply_manual_step(request, target_ids)

        if request.action == "return_home":
            self._clear_manual_targets(target_ids)
            for drone_id in target_ids:
                self._mission_overrides[drone_id] = "return-home"
            return f"return home applied to {len(target_ids)} drone(s)"

        if request.action == "emergency_land":
            self._clear_manual_targets(target_ids)
            for drone_id in target_ids:
                self._mission_overrides[drone_id] = "emergency-land"
            return f"emergency land applied to {len(target_ids)} drone(s)"

        if request.action == "fly":
            self._clear_manual_targets(target_ids)
            for drone_id in target_ids:
                self._mission_overrides[drone_id] = "fly"
            return f"takeoff applied to {len(target_ids)} drone(s)"

        if request.action == "land":
            self._clear_manual_targets(target_ids)
            for drone_id in target_ids:
                self._mission_overrides[drone_id] = "land"
            return f"target landing applied to {len(target_ids)} drone(s)"

        if request.action == "election":
            for drone_id in self._ids:
                self._mission_overrides.setdefault(drone_id, "formation-hold")
            return "leader election broadcast sent"

        if request.action == "maintenance_mode":
            token_id = safe_text(request.payload.get("maintenance_token_id"), "maintenance-window-1")
            for drone_id in target_ids:
                self._mission_overrides[drone_id] = "maintenance-window"
                self._firmware_overrides[drone_id] = {
                    **self._firmware_overrides.get(drone_id, {}),
                    "maintenance_mode": True,
                    "update_channel_state": f"maintenance-window-open:{token_id}",
                    "boot_trust_summary": "Maintenance window opened from dashboard console",
                }
            return f"maintenance window opened for {len(target_ids)} drone(s)"

        if request.action == "firmware_update":
            version = safe_text(request.payload.get("firmware_version"), "2.0.0")
            measurement = safe_text(request.payload.get("firmware_measurement"), "fw-secure-local")
            rollback_counter = safe_int(request.payload.get("rollback_counter"), 1)
            for drone_id in target_ids:
                self._mission_overrides[drone_id] = "firmware-update"
                self._firmware_overrides[drone_id] = {
                    **self._firmware_overrides.get(drone_id, {}),
                    "firmware_version": version,
                    "firmware_measurement": measurement,
                    "secure_boot_state": safe_text(request.payload.get("secure_boot_state"), "SECURE_BOOT_TRUSTED"),
                    "boot_trust_summary": safe_text(request.payload.get("boot_trust_summary"), "Firmware update applied from dashboard console"),
                    "rollback_counter": rollback_counter,
                    "maintenance_mode": True,
                    "update_channel_state": "firmware-updated",
                }
            return f"firmware update staged for {len(target_ids)} drone(s)"

        return f"{request.action} accepted for {len(target_ids)} drone(s)"

    def poll(self) -> DashboardSnapshot:
        if self._client is not None:
            try:
                return self._apply_manual_navigation(self._merge_local_demo_snapshot(self._client.fetch_snapshot()))
            except (error.URLError, TimeoutError, ValueError, KeyError) as exc:
                logger.warning("go-control-plane snapshot failed, falling back to simulation: %s", exc)
                return self._apply_manual_navigation(self._merge_local_demo_snapshot(self._simulate_snapshot()))
        if self._bridge is not None and self._pipelines:
            snapshot = self._poll_bridge()
            if snapshot is not None:
                return self._apply_manual_navigation(self._merge_local_demo_snapshot(snapshot))
        return self._apply_manual_navigation(self._merge_local_demo_snapshot(self._simulate_snapshot()))

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
                    self._apply_firmware_overrides(DroneState(
                        drone_id=drone_id,
                        cluster_id=str(peer.cluster_id) if peer is not None and hasattr(peer, "cluster_id") else f"cluster-{((drone_id - 1) // 20) + 1:02d}",
                        role=role,
                        mission_state=self._mission_overrides.get(
                            drone_id,
                            "tracking" if role == "LEADER" else "formation-hold",
                        ),
                        position=position,
                        velocity=velocity,
                        commanded_altitude_m=safe_float(getattr(runtime, "commanded_altitude_m", position[2]), position[2]),
                        commanded_speed_mps=safe_float(getattr(runtime, "commanded_speed_mps", vector_norm3(velocity)), vector_norm3(velocity)),
                        manual_target_position=safe_vector3(getattr(runtime, "manual_target_position", position), position),
                        manual_control_active=bool(getattr(runtime, "manual_control_active", False)),
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
                        peer_clock_offset_ms=safe_float(getattr(runtime, "peer_clock_offset_ms", 0.0), 0.0),
                        anchor_visibility_ratio=safe_float(getattr(runtime, "anchor_visibility_ratio", 0.0), 0.0),
                        tdoa_weight=safe_float(getattr(runtime, "tdoa_weight", 0.0), 0.0),
                        planned_waypoint_count=safe_int(getattr(runtime, "planned_waypoint_count", 0), 0),
                        last_relocalized_keyframe=safe_int(getattr(runtime, "last_relocalized_keyframe", 0), 0),
                        security_state=safe_text(getattr(runtime, "security_state", "TRUSTED"), "TRUSTED"),
                        security_summary=safe_text(getattr(runtime, "security_summary", "All trust signals nominal"), "All trust signals nominal"),
                        security_transition_reason=safe_text(getattr(runtime, "security_transition_reason", "initial-trust"), "initial-trust"),
                        remote_command_allowed=bool(getattr(runtime, "remote_command_allowed", True)),
                        telemetry_uplink_allowed=bool(getattr(runtime, "telemetry_uplink_allowed", True)),
                        link_integrity_score=safe_float(getattr(runtime, "link_integrity_score", 1.0), 1.0),
                        trust_epoch=safe_int(getattr(runtime, "trust_epoch", 1), 1),
                        last_auth_failure_at_s=safe_float(getattr(runtime, "last_auth_failure_at_s", 0.0), 0.0),
                        tamper_score=safe_float(getattr(runtime, "tamper_score", 0.0), 0.0),
                        firmware_measurement=safe_text(getattr(runtime, "firmware_measurement", "lab-local-build"), "lab-local-build"),
                        firmware_version=safe_text(getattr(runtime, "firmware_version", "0.0.0"), "0.0.0"),
                        secure_boot_state=safe_text(getattr(runtime, "secure_boot_state", "LAB_BOOT"), "LAB_BOOT"),
                        boot_trust_summary=safe_text(getattr(runtime, "boot_trust_summary", "Lab boot trust bypassed"), "Lab boot trust bypassed"),
                        rollback_counter=safe_int(getattr(runtime, "rollback_counter", 0), 0),
                        maintenance_mode=bool(getattr(runtime, "maintenance_mode", False)),
                        update_channel_state=safe_text(getattr(runtime, "update_channel_state", "idle"), "idle"),
                        last_remote_command_status=safe_text(getattr(runtime, "last_remote_command_status", "no remote command"), "no remote command"),
                        health_flags=[safe_text(v) for v in getattr(runtime, "health_flags", []) if safe_text(v)],
                        motor_health=max(0.45, min(1.0, 1.0 - (drone_id - 1) * 0.03)),
                        leadership_score=max(0.1, min(0.98, (battery / 100.0) * 0.35 + 0.55)),
                        election_ready=reachable and battery > 18.0,
                        timestamp=now,
                    ))
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
                clusters=self._cluster_snapshot(states),
                critical_alerts=self._critical_alert_count(states),
                health=self._health_snapshot(states, float(stats.cpu_temp_c)),
                events=[],
                missions=[],
                services=["telemetry-bridge", "mesh-sidecar"] if self._network is not None else ["telemetry-bridge"],
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
                self._apply_firmware_overrides(DroneState(
                    drone_id=drone_id,
                    cluster_id=f"cluster-{((drone_id - 1) // 20) + 1:02d}",
                    role="LEADER" if idx == 0 else "FOLLOWER",
                    mission_state=self._mission_overrides.get(drone_id, "formation-hold"),
                    position=synth_position,
                    velocity=synth_velocity,
                    commanded_altitude_m=anchor.commanded_altitude_m,
                    commanded_speed_mps=anchor.commanded_speed_mps,
                    manual_target_position=anchor.manual_target_position,
                    manual_control_active=anchor.manual_control_active,
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
                    peer_clock_offset_ms=anchor.peer_clock_offset_ms + idx * 0.4,
                    anchor_visibility_ratio=max(0.0, anchor.anchor_visibility_ratio - idx * 0.08),
                    tdoa_weight=max(0.0, anchor.tdoa_weight - idx * 0.05),
                    planned_waypoint_count=max(0, anchor.planned_waypoint_count - idx),
                    last_relocalized_keyframe=anchor.last_relocalized_keyframe,
                    security_state=anchor.security_state,
                    security_summary=anchor.security_summary,
                    security_transition_reason=anchor.security_transition_reason,
                    remote_command_allowed=anchor.remote_command_allowed,
                    telemetry_uplink_allowed=anchor.telemetry_uplink_allowed,
                    link_integrity_score=max(0.1, anchor.link_integrity_score - idx * 0.05),
                    trust_epoch=anchor.trust_epoch,
                    last_auth_failure_at_s=anchor.last_auth_failure_at_s,
                    tamper_score=anchor.tamper_score,
                    firmware_measurement=anchor.firmware_measurement,
                    firmware_version=anchor.firmware_version,
                    secure_boot_state=anchor.secure_boot_state,
                    boot_trust_summary=anchor.boot_trust_summary,
                    rollback_counter=anchor.rollback_counter,
                    maintenance_mode=anchor.maintenance_mode,
                    update_channel_state=anchor.update_channel_state,
                    last_remote_command_status=anchor.last_remote_command_status,
                    health_flags=list(anchor.health_flags),
                    motor_health=max(0.55, anchor.motor_health - idx * 0.04),
                    leadership_score=max(0.12, anchor.leadership_score - idx * 0.06),
                    election_ready=(anchor.battery_pct - idx * 2.5) > 18.0,
                    timestamp=now,
                ))
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
            link_integrity_score = max(0.08, min(1.0, (sync_confidence * 0.55) + max(0.0, min(1.0, (rssi + 90.0) / 45.0)) * 0.45))
            security_state = "SAFE_RETURN" if localization_state == "lost" and link_integrity_score < 0.3 else "DEGRADED_LINK" if link_integrity_score < 0.45 else "TRUSTED"
            security_summary = (
                "Navigation trust degraded with unstable link, returning via safe autonomy"
                if security_state == "SAFE_RETURN"
                else "Link integrity degraded, operating cautiously with reduced trust"
                if security_state == "DEGRADED_LINK"
                else "All trust signals nominal"
            )
            attitude_rpy, thrust_vector = derive_attitude_thrust((x, y, z), (vx, vy, 0.0), "LEADER" if idx == 0 else "FOLLOWER", elapsed)

            states.append(
                self._apply_firmware_overrides(DroneState(
                    drone_id=drone_id,
                    cluster_id=f"cluster-{((drone_id - 1) // 20) + 1:02d}",
                    role="LEADER" if idx == 0 else "FOLLOWER",
                    mission_state=self._mission_overrides.get(
                        drone_id,
                        "patrol" if idx == 0 else "formation-hold",
                    ),
                    position=(x, y, z),
                    velocity=(vx, vy, 0.0),
                    commanded_altitude_m=8.0 + idx * 0.4,
                    commanded_speed_mps=3.0,
                    manual_target_position=(x + 1.2, y + 0.8, z),
                    manual_control_active=False,
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
                    peer_clock_offset_ms=2.5 * math.sin(elapsed * 0.19 + idx),
                    anchor_visibility_ratio=max(0.0, min(1.0, anchor_visibility / 6.0)),
                    tdoa_weight=max(0.0, min(1.0, tdoa_conf * 0.9)),
                    planned_waypoint_count=max(1, 5 - idx),
                    last_relocalized_keyframe=max(0, 42 - idx * 3),
                    security_state=security_state,
                    security_summary=security_summary,
                    security_transition_reason="safety-consistency-failed" if security_state == "SAFE_RETURN" else "link-integrity-low" if security_state == "DEGRADED_LINK" else "nominal-trust",
                    remote_command_allowed=security_state == "TRUSTED",
                    telemetry_uplink_allowed=True,
                    link_integrity_score=link_integrity_score,
                    trust_epoch=3 + idx,
                    last_auth_failure_at_s=18.5 if security_state != "TRUSTED" else 0.0,
                    tamper_score=max(0.0, 0.42 - link_integrity_score * 0.3),
                    firmware_measurement=f"fw-sim-2026-04-{18 + idx:02d}",
                    firmware_version=f"2.0.{idx}",
                    secure_boot_state="SECURE_BOOT_TRUSTED" if security_state == "TRUSTED" else "SECURE_BOOT_DEGRADED",
                    boot_trust_summary="Trusted boot chain validated" if security_state == "TRUSTED" else "Boot chain held under degraded trust policy",
                    rollback_counter=1 + idx,
                    maintenance_mode=idx == 0 and int(elapsed) % 50 > 42,
                    update_channel_state="idle" if idx else "tracking",
                    last_remote_command_status="simulation: no remote command",
                    health_flags=[] if security_state == "TRUSTED" else [security_state.lower()],
                    motor_health=motor_health,
                    leadership_score=leadership_score,
                    election_ready=reachable and battery > 18.0 and motor_health > 0.45,
                ))
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
            clusters=self._cluster_snapshot(states),
            critical_alerts=self._critical_alert_count(states),
            health=self._health_snapshot(states, states[0].cpu_temp_c),
            missions=[
                {
                    "mission_id": "sim-patrol-1",
                    "name": "Sim Patrol",
                    "formation": "DIAMOND",
                    "cluster_id": states[0].cluster_id if states else "cluster-01",
                    "target": [6.0, 4.0, 8.0],
                    "status": "running",
                    "created_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                }
            ],
            events=[
                {
                    "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                    "type": "simulation",
                    "message": "Local simulation snapshot refreshed",
                }
            ],
            services=["sim-telemetry", "local-map", "command-console"],
        )

    def execute(self, request: CommandRequest) -> str:
        logger.info("SwarmBackend execute action=%s payload=%s mode=%s", request.action, request.payload, self._mode)
        if request.action == "add_drone" and bool(request.payload.get("demo_drone")):
            drone_id = self._add_local_drone(int(request.payload.get("drone_id", 0)) or None)
            self._local_demo_ids.add(drone_id)
            return f"demo drone {drone_id} added to local dashboard swarm"
        if request.action == "remove_drone":
            drone_id = int(request.payload.get("drone_id", 0) or 0)
            if drone_id in self._local_demo_ids:
                self._local_demo_ids.discard(drone_id)
                self._mission_overrides.pop(drone_id, None)
                return f"demo drone {drone_id} removed from local dashboard swarm" if self._remove_local_drone(drone_id) else f"drone {drone_id} not found"
        if self._client is not None:
            try:
                message = self._client.send_command(request)
                self._mirror_backend_command(request)
                return message
            except error.HTTPError as exc:
                if (
                    self._security_profile == "lab"
                    and exc.code in {403, 409}
                    and request.action in {"election", "maintenance_mode", "firmware_update", "emergency_land"}
                ):
                    logger.warning("Go backend rejected %s in lab mode with %s, applying local fallback", request.action, exc.code)
                    return self._apply_local_command_effect(request)
                return f"{request.action} failed via go backend: HTTP Error {exc.code}: {exc.reason}"
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
            if request.action in {"election", "formation", "hold_position", "return_home", "emergency_land", "fly", "land", "move_left", "move_right", "move_up", "move_down", "maintenance_mode", "firmware_update"}:
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

            if request.action in {"hold_position", "return_home", "fly", "land", "move_left", "move_right", "move_up", "move_down", "maintenance_mode", "firmware_update"}:
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
        self.opts["center"] = pg.Vector(0.0, 0.0, 6.0)
        self.setCameraPosition(distance=26, elevation=24, azimuth=38)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        grid = gl.GLGridItem()
        grid.setSize(56, 56)
        grid.setSpacing(2, 2)
        grid.translate(0, 0, -0.02)
        self.addItem(grid)

        axis = gl.GLAxisItem()
        axis.setSize(4.0, 4.0, 4.0)
        self.addItem(axis)

        self._drone_icon_base = _load_svg_gl_image(ASSET_DIR / "drone.svg", (56, 56))
        self._leader_icon = _colorize_gl_image(self._drone_icon_base, (255, 196, 64, 220))
        self._follower_icon = _colorize_gl_image(self._drone_icon_base, (84, 235, 255, 205))
        self._offline_icon = _colorize_gl_image(self._drone_icon_base, (255, 96, 96, 210))
        self._histories: dict[int, deque[tuple[float, float, float]]] = {}
        self._trails: dict[int, gl.GLLinePlotItem] = {}
        self._icons: dict[int, gl.GLImageItem] = {}
        self._max_points = 120
        self._camera_center = np.array([0.0, 0.0, 6.0], dtype=float)
        self._camera_distance = 26.0
        self._auto_camera_resume_at = 0.0

    def ingest(self, states: list[DroneState]) -> None:
        if not states:
            return

        positions = np.array([state.position for state in states], dtype=float)
        self._update_camera_frame(positions)

        for state in states:
            if state.drone_id not in self._histories:
                self._histories[state.drone_id] = deque(maxlen=self._max_points)
                line = gl.GLLinePlotItem(width=1.5, antialias=False)
                self._trails[state.drone_id] = line
                self.addItem(line)
            if state.drone_id not in self._icons:
                icon = gl.GLImageItem(self._follower_icon)
                icon.setGLOptions("translucent")
                self._icons[state.drone_id] = icon
                self.addItem(icon)
            self._histories[state.drone_id].append(state.position)
            icon = self._icons[state.drone_id]
            icon.resetTransform()
            if not state.reachable:
                icon.setData(self._offline_icon)
                trail_rgba = np.array([[1.0, 0.36, 0.36, 0.55]])
            elif state.role == "LEADER":
                icon.setData(self._leader_icon)
                trail_rgba = np.array([[1.0, 0.80, 0.28, 0.62]])
            else:
                icon.setData(self._follower_icon)
                trail_rgba = np.array([[0.34, 0.92, 1.0, 0.5]])
            icon.scale(0.028, 0.028, 0.028)
            icon.translate(state.position[0] - 0.78, state.position[1] - 0.78, max(0.22, state.position[2]) + 0.22)
            if len(self._histories[state.drone_id]) >= 2:
                trail = np.array(self._histories[state.drone_id], dtype=float)
                color = np.tile(trail_rgba, (trail.shape[0], 1))
                self._trails[state.drone_id].setData(pos=trail, color=color)

        active_ids = {state.drone_id for state in states}
        for drone_id in list(self._trails):
            if drone_id in active_ids:
                continue
            icon = self._icons.pop(drone_id, None)
            if icon is not None:
                self.removeItem(icon)
            self._histories.pop(drone_id, None)
            trail = self._trails.pop(drone_id, None)
            if trail is not None:
                self.removeItem(trail)

    def _update_camera_frame(self, positions: np.ndarray) -> None:
        if time.monotonic() < self._auto_camera_resume_at:
            return
        mins = positions.min(axis=0)
        maxs = positions.max(axis=0)
        center = (mins + maxs) / 2.0
        span = np.maximum(maxs - mins, np.array([10.0, 10.0, 6.0]))
        target_center = np.array([center[0], center[1], max(4.0, center[2] * 0.7 + 3.0)], dtype=float)
        target_distance = float(max(24.0, np.linalg.norm(span[:2]) * 1.1 + span[2] * 1.8))

        self._camera_center = 0.85 * self._camera_center + 0.15 * target_center
        self._camera_distance = 0.85 * self._camera_distance + 0.15 * target_distance
        self.opts["center"] = pg.Vector(*self._camera_center)
        self.setCameraPosition(
            distance=self._camera_distance,
            elevation=22,
            azimuth=38,
        )

    def _pause_auto_camera(self, seconds: float = 3.0) -> None:
        self._auto_camera_resume_at = max(self._auto_camera_resume_at, time.monotonic() + seconds)
        self._camera_distance = float(self.opts.get("distance", self._camera_distance))
        center = self.opts.get("center")
        if center is not None:
            self._camera_center = np.array([center.x(), center.y(), center.z()], dtype=float)

    def wheelEvent(self, event) -> None:  # noqa: N802
        self._pause_auto_camera(4.0)
        super().wheelEvent(event)
        self._pause_auto_camera(4.0)

    def mousePressEvent(self, event) -> None:  # noqa: N802
        self._pause_auto_camera()
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:  # noqa: N802
        self._pause_auto_camera()
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:  # noqa: N802
        self._pause_auto_camera()
        super().mouseReleaseEvent(event)


class DriftGraph(pg.PlotWidget):
    def __init__(self, drone_ids: list[int], poll_hz: int) -> None:
        super().__init__()
        self._legend = style_plot_widget(self, "EKF Error (m)")
        self._window = 360
        self._poll_hz = max(poll_hz, 1)
        self._t0 = time.time()
        self._history: dict[int, deque[float]] = {drone_id: deque(maxlen=self._window) for drone_id in drone_ids}
        self._palette = [CYAN, MAGENTA, SUCCESS, WARN, ACCENT_ALT, "#A78BFA", ACCENT, "#F97316"]
        self._curves = {
            drone_id: self.plot(pen=pg.mkPen(self._palette[idx % len(self._palette)], width=2), name=f"D{drone_id}")
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
                    name=f"D{state.drone_id}",
                )
            self._history[state.drone_id].append(state.drift_m)
            series = np.array(self._history[state.drone_id], dtype=float)
            x = np.linspace(max(0.0, now - len(series) / self._poll_hz), now, len(series))
            self._curves[state.drone_id].setData(x, series)



class MCSSScoreGraph(pg.PlotWidget):
    def __init__(self, drone_ids: list[int], poll_hz: int) -> None:
        super().__init__()
        self._legend = style_plot_widget(self, "Leader Score")
        self._window = 360
        self._poll_hz = max(poll_hz, 1)
        self._t0 = time.time()
        self._history: dict[int, deque[float]] = {drone_id: deque(maxlen=self._window) for drone_id in drone_ids}
        self._palette = [CYAN, MAGENTA, SUCCESS, WARN, ACCENT_ALT, "#A78BFA", ACCENT, "#F97316"]
        self._curves = {
            drone_id: self.plot(pen=pg.mkPen(self._palette[idx % len(self._palette)], width=2), name=f"D{drone_id}")
            for idx, drone_id in enumerate(drone_ids)
        }

    def ingest(self, states: list[DroneState]) -> None:
        now = time.time() - self._t0
        for state in states:
            if state.drone_id not in self._history:
                self._history[state.drone_id] = deque(maxlen=self._window)
                color = self._palette[len(self._curves) % len(self._palette)]
                self._curves[state.drone_id] = self.plot(pen=pg.mkPen(color, width=2), name=f"D{state.drone_id}")
            self._history[state.drone_id].append(state.leadership_score)
            series = np.array(self._history[state.drone_id], dtype=float)
            x = np.linspace(max(0.0, now - len(series) / self._poll_hz), now, len(series))
            self._curves[state.drone_id].setData(x, series)

class NetworkHealthGraph(pg.PlotWidget):
    def __init__(self, poll_hz: int) -> None:
        super().__init__()
        self._legend = style_plot_widget(self, "Network Latency & Loss")
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
        self._legend = style_plot_widget(self, "Compute (Temp & Load)")
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
        header = self.horizontalHeader()
        header.setStretchLastSection(True)
        header.setSectionResizeMode(QHeaderView.Stretch)
        header.setMinimumSectionSize(84)
        header.setDefaultAlignment(Qt.AlignCenter)
        self.verticalHeader().setVisible(False)
        self.setAlternatingRowColors(True)
        self.setEditTriggers(QTableWidget.NoEditTriggers)
        self.setSelectionBehavior(QTableWidget.SelectRows)
        self.setShowGrid(False)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setWordWrap(False)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def ingest(self, snapshot: DashboardSnapshot) -> None:
        if not snapshot.states:
            self.setRowCount(0)
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
        self.resizeRowsToContents()


class AutonomySecurityTable(QTableWidget):
    HEADERS = [
        "ID",
        "Keyframe-based SLAM",
        "Zero-Trust Security",
        "Health Flags",
    ]

    def __init__(self) -> None:
        super().__init__(0, len(self.HEADERS))
        self.setHorizontalHeaderLabels(self.HEADERS)
        header = self.horizontalHeader()
        header.setStretchLastSection(True)
        header.setMinimumSectionSize(120)
        header.setDefaultAlignment(Qt.AlignCenter)
        for col in range(len(self.HEADERS)):
            header.setSectionResizeMode(col, QHeaderView.Stretch)
        self.verticalHeader().setVisible(False)
        self.setAlternatingRowColors(True)
        self.setEditTriggers(QTableWidget.NoEditTriggers)
        self.setSelectionBehavior(QTableWidget.SelectRows)
        self.setShowGrid(False)
        self.setWordWrap(True)
        self.setTextElideMode(Qt.ElideNone)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def ingest(self, snapshot: DashboardSnapshot) -> None:
        self.setRowCount(len(snapshot.states))
        for row, state in enumerate(snapshot.states):
            health_flags = ", ".join(state.health_flags) if state.health_flags else "Nominal"
            values = [
                str(state.drone_id),
                (
                    f"anchors {state.visible_anchor_count} | occupancy {state.occupancy_ratio*100.0:.1f}% | "
                    f"relocal {state.relocalization_count} | trend {state.confidence_trend:+.2f}"
                ),
                (
                    f"{state.security_state} | integrity {state.link_integrity_score:.2f} | "
                    f"epoch {state.trust_epoch} | tamper {state.tamper_score:.2f}"
                ),
                health_flags,
            ]
            for col, value in enumerate(values):
                item = QTableWidgetItem(value)
                item.setTextAlignment(Qt.AlignCenter)
                if col == 2:
                    if state.security_state == "TRUSTED":
                        item.setForeground(QColor(SUCCESS))
                    elif state.security_state == "DEGRADED_LINK":
                        item.setForeground(QColor(WARN))
                    else:
                        item.setForeground(QColor(DANGER))
                elif col == 3 and state.health_flags:
                    item.setForeground(QColor(WARN))
                self.setItem(row, col, item)
        self.resizeRowsToContents()


class AutonomyHighlightsTable(QTableWidget):
    HEADERS = [
        "ID",
        "Visual-Inertial Odometry (VIO)",
        "Remote Command Trust",
        "Health Flags",
    ]

    def __init__(self) -> None:
        super().__init__(0, len(self.HEADERS))
        self.setHorizontalHeaderLabels(self.HEADERS)
        header = self.horizontalHeader()
        header.setStretchLastSection(True)
        header.setMinimumSectionSize(130)
        header.setDefaultAlignment(Qt.AlignCenter)
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        for col in range(1, len(self.HEADERS)):
            header.setSectionResizeMode(col, QHeaderView.Stretch)
        self.verticalHeader().setVisible(False)
        self.setAlternatingRowColors(True)
        self.setEditTriggers(QTableWidget.NoEditTriggers)
        self.setSelectionBehavior(QTableWidget.SelectRows)
        self.setShowGrid(False)
        self.setWordWrap(True)
        self.setTextElideMode(Qt.ElideNone)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def ingest(self, snapshot: DashboardSnapshot) -> None:
        self.setRowCount(len(snapshot.states))
        for row, state in enumerate(snapshot.states):
            health_flags = ", ".join(state.health_flags) if state.health_flags else "Nominal"
            values = [
                str(state.drone_id),
                (
                    f"{state.localization_source} | {state.localization_state.upper()} | "
                    f"drift {state.drift_m:.3f} m | conf {state.localization_confidence:.2f}"
                ),
                (
                    f"remote {'ALLOW' if state.remote_command_allowed else 'BLOCK'} | "
                    f"uplink {'ON' if state.telemetry_uplink_allowed else 'OFF'} | "
                    f"{state.last_remote_command_status} | fw {state.firmware_version} | {state.secure_boot_state}"
                ),
                health_flags,
            ]
            for col, value in enumerate(values):
                item = QTableWidgetItem(value)
                item.setTextAlignment(Qt.AlignCenter)
                if col == 2 and not state.remote_command_allowed:
                    item.setForeground(QColor(DANGER))
                elif col == 3 and state.health_flags:
                    item.setForeground(QColor(WARN))
                self.setItem(row, col, item)
        self.resizeRowsToContents()


class SensorConnectionStatusTable(QWidget):
    HEADERS = [
        "Sensor",
        "Connection",
        "Connection Time",
        "Working Ability",
        "Error",
    ]

    SENSOR_DEFINITIONS = [
        ("IMU", "enable_imu", lambda cfg: f"{cfg.get('imu_device', '/dev/i2c-1')} @ addr {cfg.get('imu_addr', '104')}"),
        ("Camera", "enable_camera", lambda cfg: cfg.get("camera_stream_url", "").strip() or cfg.get("esp32_ip", "").strip() or "Not configured"),
        ("LiDAR", "enable_lidar", lambda cfg: cfg.get("lidar_endpoint", "").strip() or "Not configured"),
        ("Barometer", "enable_barometer", lambda _cfg: "Onboard telemetry bus"),
        ("Motor Health", "enable_motor", lambda _cfg: "ESC telemetry / controller bus"),
        ("Optical Flow", "enable_optical_flow", lambda _cfg: "Downward optical-flow module"),
        ("Rangefinder", "enable_rangefinder", lambda _cfg: "Altitude / proximity rangefinder"),
        ("TDOA Ingestor", "enable_tdoa_ingestor", lambda cfg: f"UDP {cfg.get('tdoa_udp_port', '0')} | {cfg.get('tdoa_csv', '').strip() or 'live feed'}"),
        ("UWB Serial", "enable_uwb_serial", lambda cfg: cfg.get("tdoa_serial", "").strip() or "Not configured"),
    ]

    def __init__(self) -> None:
        super().__init__()
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._tables_by_drone_id: dict[int, QTableWidget] = {}
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)
        self._tabs = QTabWidget()
        self._tabs.setDocumentMode(True)
        self._tabs.setStyleSheet(
            f"QTabWidget::pane {{ border: 1px solid {BORDER}; border-radius: 6px; background: {PANEL_ALT}; top: -1px; }}"
            f" QTabBar::tab {{ background: {PANEL_BG}; color: {TEXT_DIM}; padding: 8px 14px; border: 1px solid {BORDER};"
            f" border-bottom: none; min-width: 90px; }}"
            f" QTabBar::tab:selected {{ background: {PANEL_ALT}; color: {TEXT}; border-top: 1px solid {ACCENT}; font-weight: 700; }}"
            f" QTabBar::tab:!selected:hover {{ color: {TEXT}; }}"
        )
        layout.addWidget(self._tabs)

    def _create_table(self) -> QTableWidget:
        table = QTableWidget(0, len(self.HEADERS))
        table.setHorizontalHeaderLabels(self.HEADERS)
        header = table.horizontalHeader()
        header.setStretchLastSection(True)
        header.setMinimumSectionSize(120)
        header.setDefaultAlignment(Qt.AlignCenter)
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        for col in range(1, len(self.HEADERS)):
            header.setSectionResizeMode(col, QHeaderView.Stretch)
        table.verticalHeader().setVisible(False)
        table.setAlternatingRowColors(True)
        table.setEditTriggers(QTableWidget.NoEditTriggers)
        table.setSelectionBehavior(QTableWidget.SelectRows)
        table.setShowGrid(False)
        table.setWordWrap(True)
        table.setTextElideMode(Qt.ElideNone)
        table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        table.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        return table

    @staticmethod
    def _set_table_item(table: QTableWidget, row: int, col: int, value: str, color: str | None = None) -> None:
        item = table.item(row, col)
        if item is None:
            item = QTableWidgetItem(value)
            item.setTextAlignment(Qt.AlignCenter)
            table.setItem(row, col, item)
        elif item.text() != value:
            item.setText(value)
        if color is not None:
            item.setForeground(QColor(color))

    @staticmethod
    def _enabled(config: dict[str, str], key: str) -> bool:
        return safe_text(config.get(key, "false"), "false").lower() in {"1", "true", "yes", "on"}

    @staticmethod
    def _connection_time_text(timestamp: float) -> str:
        if timestamp <= 0:
            return "Unknown"
        return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(timestamp))

    @staticmethod
    def _sensor_error_text(sensor_name: str, enabled: bool, state: DroneState) -> str:
        if not enabled:
            return "Disabled in sensor profile"
        if not state.reachable:
            return "Drone unreachable"

        sensor_key = sensor_name.lower().replace(" ", "_")
        matching_flags = [flag for flag in state.health_flags if sensor_key in flag.lower()]
        if matching_flags:
            return ", ".join(matching_flags)

        if sensor_name == "Camera" and state.localization_source.lower() == "vision-inertial" and state.localization_state != "nominal":
            return f"Localization {state.localization_state}"
        if sensor_name == "IMU" and abs(state.imu_camera_offset_ms) > 8.0:
            return f"Sync offset {state.imu_camera_offset_ms:.1f} ms"
        if sensor_name == "LiDAR" and state.occupancy_ratio <= 0.02:
            return "Low obstacle returns"
        if sensor_name == "Motor Health" and state.motor_health < 0.65:
            return f"Motor health {state.motor_health:.2f}"
        if sensor_name == "TDOA Ingestor" and state.visible_anchor_count == 0:
            return "No anchors visible"
        if sensor_name == "UWB Serial" and state.sync_confidence < 0.5:
            return f"Sync confidence {state.sync_confidence:.2f}"
        if sensor_name == "Rangefinder" and state.position[2] < 0.2:
            return "Altitude estimate near floor"
        if sensor_name == "Barometer" and state.cpu_temp_c >= 78.0:
            return "High compute temperature may skew readings"
        if sensor_name == "Optical Flow" and state.drift_m > 1.5:
            return f"High drift {state.drift_m:.2f} m"
        return "None"

    @staticmethod
    def _working_ability_text(sensor_name: str, enabled: bool, state: DroneState) -> tuple[str, str]:
        if not enabled:
            return "Disabled", TEXT_DIM
        if not state.reachable:
            return "Offline", DANGER
        if sensor_name == "Motor Health" and state.motor_health < 0.65:
            return "Degraded", WARN
        if sensor_name == "Camera" and state.localization_state != "nominal":
            return "Degraded", WARN
        if sensor_name == "IMU" and abs(state.imu_camera_offset_ms) > 8.0:
            return "Degraded", WARN
        if sensor_name == "UWB Serial" and state.sync_confidence < 0.5:
            return "Degraded", WARN
        if sensor_name == "TDOA Ingestor" and state.visible_anchor_count == 0:
            return "Degraded", WARN
        return "Working", SUCCESS

    def ingest(self, snapshot: DashboardSnapshot, env_manager: DroneEnvManager) -> None:
        current_drone_id = self._tabs.currentWidget().property("drone_id") if self._tabs.currentWidget() is not None else None
        incoming_ids = [state.drone_id for state in snapshot.states]

        for drone_id in [drone_id for drone_id in list(self._tables_by_drone_id) if drone_id not in incoming_ids]:
            table = self._tables_by_drone_id.pop(drone_id)
            index = self._tabs.indexOf(table)
            if index >= 0:
                self._tabs.removeTab(index)
            table.deleteLater()

        for state in snapshot.states:
            table = self._tables_by_drone_id.get(state.drone_id)
            is_new_table = table is None
            if table is None:
                table = self._create_table()
                table.setProperty("drone_id", state.drone_id)
                table.setRowCount(len(self.SENSOR_DEFINITIONS))
                self._tables_by_drone_id[state.drone_id] = table
                self._tabs.addTab(table, f"Drone {state.drone_id}")

            config = env_manager.drone_config(state.drone_id)
            connection_time = self._connection_time_text(state.timestamp)

            for row, (sensor_name, enabled_key, describe_connection) in enumerate(self.SENSOR_DEFINITIONS):
                enabled = self._enabled(config, enabled_key)
                ability_text, ability_color = self._working_ability_text(sensor_name, enabled, state)
                error_text = self._sensor_error_text(sensor_name, enabled, state)
                error_color = None
                if error_text != "None":
                    error_color = WARN if error_text != "Drone unreachable" else DANGER
                self._set_table_item(table, row, 0, sensor_name)
                self._set_table_item(table, row, 1, describe_connection(config))
                self._set_table_item(table, row, 2, connection_time)
                self._set_table_item(table, row, 3, ability_text, ability_color)
                self._set_table_item(table, row, 4, error_text, error_color or TEXT)

            if is_new_table:
                table.resizeRowsToContents()

        if current_drone_id is not None and current_drone_id in self._tables_by_drone_id:
            self._tabs.setCurrentWidget(self._tables_by_drone_id[current_drone_id])
        elif self._tabs.count() and self._tabs.currentIndex() < 0:
            self._tabs.setCurrentIndex(0)


class FleetOverviewPanel(QGroupBox):
    def __init__(self) -> None:
        super().__init__("Fleet Operations Overview")
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 16, 10, 10)
        layout.setSpacing(10)

        self._summary = QLabel("--")
        self._summary.setWordWrap(True)
        self._summary.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self._summary.setStyleSheet(f"color:{TEXT}; font-size:12px; font-weight:600;")
        layout.addWidget(self._summary)

        self._tabs = QTabWidget()
        self._tabs.setDocumentMode(True)
        self._tabs.setStyleSheet(
            f"QTabWidget::pane {{ border: 1px solid {BORDER}; border-radius: 6px; background: {PANEL_ALT}; top: -1px; }}"
            f" QTabBar::tab {{ background: {PANEL_BG}; color: {TEXT_DIM}; padding: 8px 14px; border: 1px solid {BORDER};"
            f" border-bottom: none; min-width: 90px; }}"
            f" QTabBar::tab:selected {{ background: {PANEL_ALT}; color: {TEXT}; border-top: 1px solid {ACCENT}; font-weight: 700; }}"
        )
        layout.addWidget(self._tabs)

        self._clusters = self._make_table(["Cluster", "Leader", "Drones", "Formation", "Mission", "Avg Battery"])
        self._missions = self._make_table(["Mission", "Formation", "Cluster", "Target", "Status", "Created"])
        self._events = self._make_table(["Time", "Type", "Message"])
        self._tabs.addTab(self._clusters, "Clusters")
        self._tabs.addTab(self._missions, "Missions")
        self._tabs.addTab(self._events, "Events")

    def _make_table(self, headers: list[str]) -> QTableWidget:
        table = QTableWidget(0, len(headers))
        table.setHorizontalHeaderLabels(headers)
        header = table.horizontalHeader()
        header.setStretchLastSection(True)
        header.setDefaultAlignment(Qt.AlignCenter)
        for col in range(len(headers)):
            header.setSectionResizeMode(col, QHeaderView.Stretch)
        table.verticalHeader().setVisible(False)
        table.setAlternatingRowColors(True)
        table.setEditTriggers(QTableWidget.NoEditTriggers)
        table.setSelectionBehavior(QTableWidget.SelectRows)
        table.setShowGrid(False)
        table.setWordWrap(True)
        table.setTextElideMode(Qt.ElideNone)
        table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        return table

    @staticmethod
    def _set_item(table: QTableWidget, row: int, col: int, value: str, color: str | None = None) -> None:
        item = table.item(row, col)
        if item is None:
            item = QTableWidgetItem(value)
            item.setTextAlignment(Qt.AlignCenter)
            table.setItem(row, col, item)
        else:
            item.setText(value)
        item.setForeground(QColor(color or TEXT))

    def ingest(self, snapshot: DashboardSnapshot) -> None:
        health = snapshot.health
        online = safe_int(health.get("online_drones"), sum(1 for state in snapshot.states if state.reachable))
        total = safe_int(health.get("total_drones"), len(snapshot.states))
        avg_battery = safe_float(health.get("avg_battery_pct"), 0.0)
        max_cpu = safe_float(health.get("max_cpu_temp_c"), snapshot.cpu_temp_c)
        services = ", ".join(snapshot.services[:6]) if snapshot.services else "fleet snapshot only"
        self._summary.setText(
            f"Critical alerts: {snapshot.critical_alerts}  |  Online: {online}/{total}  |  "
            f"Avg battery: {avg_battery:.1f}%  |  Max CPU: {max_cpu:.1f} C  |  Services: {services}"
        )

        self._clusters.setRowCount(len(snapshot.clusters))
        for row, cluster in enumerate(snapshot.clusters):
            values = [
                safe_text(cluster.get("cluster_id"), "cluster-01"),
                str(safe_int(cluster.get("leader_id"), 0) or "-"),
                str(safe_int(cluster.get("drone_count"), 0)),
                safe_text(cluster.get("formation"), "unknown"),
                safe_text(cluster.get("mission_state"), "unknown"),
                f"{safe_float(cluster.get('avg_battery', 0.0)):.1f}%",
            ]
            for col, value in enumerate(values):
                color = WARN if col == 5 and safe_float(cluster.get("avg_battery", 0.0)) < 25.0 else None
                self._set_item(self._clusters, row, col, value, color)

        missions = snapshot.missions
        self._missions.setRowCount(len(missions))
        for row, mission in enumerate(missions):
            target = safe_vector3(mission.get("target", [0.0, 0.0, 0.0]))
            values = [
                safe_text(mission.get("name"), safe_text(mission.get("mission_id"), "mission")),
                safe_text(mission.get("formation"), "unknown"),
                safe_text(mission.get("cluster_id"), "all"),
                f"({target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f})",
                safe_text(mission.get("status"), "unknown"),
                safe_text(mission.get("created_at"), "")[:19].replace("T", " "),
            ]
            for col, value in enumerate(values):
                color = SUCCESS if col == 4 and value.lower() in {"running", "scheduled"} else None
                self._set_item(self._missions, row, col, value, color)

        events = snapshot.events[-12:]
        self._events.setRowCount(len(events))
        for row, event in enumerate(events):
            values = [
                safe_text(event.get("timestamp"), "")[:19].replace("T", " "),
                safe_text(event.get("type"), "event"),
                safe_text(event.get("message"), ""),
            ]
            for col, value in enumerate(values):
                color = WARN if col == 1 and "approval" in value.lower() else DANGER if col == 2 and "rejected" in value.lower() else None
                self._set_item(self._events, row, col, value, color)

        self._clusters.resizeRowsToContents()
        self._missions.resizeRowsToContents()
        self._events.resizeRowsToContents()


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
        layout.addLayout(make_row("Bridge Diagnostics", "bridge_diag"))
        layout.addLayout(make_row("Planner / Relocal KF", "planner_diag"))
        layout.addLayout(make_row("Manual Control", "manual_control"))
        layout.addLayout(make_row("Manual Target", "manual_target"))
        layout.addLayout(make_row("Trust / Tamper", "trust_tamper"))
        layout.addLayout(make_row("Security Summary", "security_summary"))
        layout.addLayout(make_row("Security Reason", "security_reason"))
        layout.addLayout(make_row("Auth / Firmware", "auth_firmware"))
        layout.addLayout(make_row("Boot / Maintenance", "boot_maint"))
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
        self._labels["bridge_diag"].setText(
            f"Peer clk {state.peer_clock_offset_ms:+.1f} ms  |  Anchor vis {state.anchor_visibility_ratio*100.0:.0f}%  |  TDOA weight {state.tdoa_weight:.2f}"
        )
        self._labels["planner_diag"].setText(
            f"Waypoints: {state.planned_waypoint_count}  |  last KF: {state.last_relocalized_keyframe}  |  source {state.localization_source}"
        )
        self._labels["manual_control"].setText(
            f"{'ACTIVE' if state.manual_control_active else 'IDLE'}  |  cmd alt {state.commanded_altitude_m:.1f} m  |  cmd speed {state.commanded_speed_mps:.1f} m/s"
        )
        self._labels["manual_target"].setText(
            f"Target ({state.manual_target_position[0]:.1f}, {state.manual_target_position[1]:.1f}, {state.manual_target_position[2]:.1f})"
        )
        self._labels["trust_tamper"].setText(
            f"Epoch: {state.trust_epoch}  |  Tamper: {state.tamper_score:.2f}  |  "
            f"{state.secure_boot_state}  |  rc {state.rollback_counter}  |  {state.update_channel_state}"
        )
        self._labels["security_summary"].setText(state.security_summary)
        self._labels["security_reason"].setText(state.security_transition_reason.replace("-", " "))
        self._labels["auth_firmware"].setText(
            f"Last auth fail: {state.last_auth_failure_at_s:.1f}s  |  fw {state.firmware_version}  |  {state.firmware_measurement}"
        )
        self._labels["boot_maint"].setText(
            f"{'maintenance ON' if state.maintenance_mode else 'maintenance OFF'}  |  {state.boot_trust_summary}"
        )
        self._set_bar("occupancy", state.occupancy_ratio * 100.0, "{:.1f}%", 60.0, 85.0, invert=True)



class CommandConsole(QGroupBox):
    command_requested = Signal(object)

    def __init__(self) -> None:
        super().__init__("Command Console")
        self._operator_role = normalize_operator_role(os.environ.get("DRONE_OPERATOR_ROLE", "operator"))
        self._security_profile = normalize_security_profile(os.environ.get("DRONE_SECURITY_PROFILE", "lab"))
        self.setMinimumHeight(250)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 12, 10, 10)
        layout.setSpacing(12)

        self._approval_banner = QLabel("No pending critical approvals")
        self._approval_banner.setWordWrap(True)
        self._approval_banner.setStyleSheet(
            f"background:{PANEL_ALT}; color:{TEXT_DIM}; border:1px solid {BORDER}; "
            "border-radius:10px; padding:8px 10px; font-size:12px; font-weight:600;"
        )
        layout.addWidget(self._approval_banner)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setStyleSheet(f"QScrollArea {{ background: transparent; border: none; }}")
        content = QWidget()
        content_layout = QVBoxLayout(content)
        content_layout.setContentsMargins(4, 4, 4, 4)
        content_layout.setSpacing(12)

        def make_action_button(
            label: str,
            color: str,
            action: str,
            *,
            min_height: int = 38,
            radius: int = 10,
            font_size: int = 12,
            auto_repeat: bool = False,
        ) -> QPushButton:
            button = QPushButton(label)
            button.setCursor(Qt.PointingHandCursor)
            button.setMinimumHeight(min_height)
            button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            button.setAutoRepeat(auto_repeat)
            if auto_repeat:
                button.setAutoRepeatDelay(180)
                button.setAutoRepeatInterval(120)
            button.setStyleSheet(
                f"QPushButton {{ background:{color}; color:#041019; border:none; border-radius:{radius}px; "
                f"font-weight:700; font-size:{font_size}px; padding:10px 14px; }}"
                "QPushButton:pressed { padding-top: 12px; padding-bottom: 8px; }"
            )
            button.clicked.connect(lambda _checked=False, action_name=action: self.command_requested.emit(self._build_request(action_name)))
            return button

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
        self._drone_id = QSpinBox()
        self._drone_id.setRange(0, 999)
        self._drone_id.setSpecialValueText("Selected")
        self._drone_id.setValue(0)
        self._drone_id.setStyleSheet(f"background:{PANEL_ALT}; color:{TEXT}; border:1px solid {BORDER}; padding:6px;")
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
        self._step = QDoubleSpinBox()
        self._step.setRange(0.2, 25.0)
        self._step.setValue(1.5)
        self._step.setSuffix(" m")
        self._step.setStyleSheet(f"background:{PANEL_ALT}; color:{TEXT}; border:1px solid {BORDER}; padding:6px;")

        tuning_layout = QGridLayout()
        tuning_layout.setColumnStretch(0, 0)
        tuning_layout.setColumnStretch(1, 1)
        tuning_layout.addWidget(QLabel("Scope"), 0, 0)
        tuning_layout.addWidget(self._scope, 0, 1)
        tuning_layout.addWidget(QLabel("Drone ID"), 1, 0)
        tuning_layout.addWidget(self._drone_id, 1, 1)
        tuning_layout.addWidget(QLabel("Spacing"), 2, 0)
        tuning_layout.addWidget(self._spacing, 2, 1)
        tuning_layout.addWidget(QLabel("Altitude"), 3, 0)
        tuning_layout.addWidget(self._altitude, 3, 1)
        tuning_layout.addWidget(QLabel("Speed"), 4, 0)
        tuning_layout.addWidget(self._speed, 4, 1)
        tuning_layout.addWidget(QLabel("Move Step"), 5, 0)
        tuning_layout.addWidget(self._step, 5, 1)

        movement_box = QFrame()
        movement_box.setStyleSheet(f"background:{PANEL_ALT}; border:1px solid {BORDER}; border-radius:16px;")
        movement_layout = QGridLayout(movement_box)
        movement_layout.setContentsMargins(16, 16, 16, 16)
        movement_layout.setHorizontalSpacing(10)
        movement_layout.setVerticalSpacing(10)

        controls = [
            ("Add Drone", "#A3E635", "add_drone"),
            ("Remove Drone", DANGER, "remove_drone"),
            ("Election", SUCCESS, "election"),
            ("Maint Window", WARN, "maintenance_mode"),
            ("Firmware Update", "#F97316", "firmware_update"),
            ("VEE", ACCENT, "VEE"),
            ("LINE", CYAN, "LINE"),
            ("DIAMOND", MAGENTA, "DIAMOND"),
            ("Takeoff (Fly)", SUCCESS, "fly"),
            ("Target Land", WARN, "land"),
            ("Return Home", "#60A5FA", "return_home"),
            ("Emergency Land", DANGER, "emergency_land"),
        ]
        self._buttons: dict[str, QPushButton] = {}

        for action, label, color, row, col in [
            ("move_up", "↑", "#FDE68A", 0, 1),
            ("move_left", "←", "#7DD3FC", 1, 0),
            ("hold_position", "◎", "#94A3B8", 1, 1),
            ("move_right", "→", "#38BDF8", 1, 2),
            ("move_down", "↓", "#FDBA74", 2, 1),
        ]:
            button = make_action_button(
                label,
                color,
                action,
                min_height=54,
                radius=18,
                font_size=24,
                auto_repeat=action != "hold_position",
            )
            button.setToolTip(action.replace("_", " ").title())
            movement_layout.addWidget(button, row, col)
            self._buttons[action] = button
        movement_layout.setColumnStretch(0, 1)
        movement_layout.setColumnStretch(1, 1)
        movement_layout.setColumnStretch(2, 1)

        for idx, (label, color, action) in enumerate(controls):
            button = make_action_button(label, color, action)
            button_layout.addWidget(button, idx // 2, idx % 2)
            self._buttons[action] = button
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
        content_layout.addWidget(movement_box)
        content_layout.addLayout(button_layout)
        content_layout.addWidget(self._log, 1)
        scroll.setWidget(content)
        layout.addWidget(scroll)
        self.set_operator_role(self._operator_role)
        self.set_pending_approvals({})

    def append_log(self, message: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        self._log.appendPlainText(f"[{timestamp}] {message}")
        logger.info("console: %s", message)

    def set_operator_role(self, role: str) -> None:
        self._operator_role = normalize_operator_role(role)
        election_button = self._buttons.get("election")
        if election_button is not None:
            allowed = self._operator_role == "commander" or self._security_profile == "lab"
            election_button.setEnabled(True)
            election_button.setToolTip(
                "Commander role required outside lab mode" if not allowed else "Trigger leader election"
            )
        for action in ("maintenance_mode", "firmware_update"):
            button = self._buttons.get(action)
            if button is None:
                continue
            allowed = self._operator_role == "maintenance" or self._security_profile == "lab"
            button.setEnabled(True)
            button.setToolTip(
                "Maintenance role required outside lab mode" if not allowed else action.replace("_", " ").title()
            )

    def set_pending_approvals(self, approvals: dict[str, str]) -> None:
        if not approvals:
            self._approval_banner.setText("No pending critical approvals")
            self._approval_banner.setStyleSheet(
                f"background:{PANEL_ALT}; color:{TEXT_DIM}; border:1px solid {BORDER}; "
                "border-radius:10px; padding:8px 10px; font-size:12px; font-weight:600;"
            )
            return
        summary = " | ".join(
            f"{action.upper()} waiting for confirm ({approval_id})"
            for action, approval_id in sorted(approvals.items())
        )
        self._approval_banner.setText(summary)
        self._approval_banner.setStyleSheet(
            f"background:rgba(245, 158, 11, 0.14); color:{WARN}; border:1px solid rgba(245, 158, 11, 0.45); "
            "border-radius:10px; padding:8px 10px; font-size:12px; font-weight:700;"
        )

    def _build_request(self, action: str) -> CommandRequest:
        payload = {
            "scope": self._scope.currentText(),
            "drone_id": int(self._drone_id.value()),
            "spacing_m": float(self._spacing.value()),
            "altitude_m": float(self._altitude.value()),
            "velocity_mps": float(self._speed.value()),
            "step_m": float(self._step.value()),
        }
        if action == "add_drone":
            return CommandRequest("add_drone", payload)
        if action in {"VEE", "LINE", "DIAMOND"}:
            return CommandRequest("formation", {**payload, "shape": action})
        if action == "maintenance_mode":
            return CommandRequest("maintenance_mode", {
                **payload,
                "maintenance_token_id": safe_text(os.environ.get("DRONE_MAINTENANCE_APPROVAL_TOKEN", "maintenance-window-1"), "maintenance-window-1"),
            })
        if action == "firmware_update":
            rollback_counter = safe_int(os.environ.get("DRONE_FIRMWARE_ROLLBACK_COUNTER", "1"), 1) + 1
            return CommandRequest("firmware_update", {
                **payload,
                "maintenance_token_id": safe_text(os.environ.get("DRONE_MAINTENANCE_APPROVAL_TOKEN", "maintenance-window-1"), "maintenance-window-1"),
                "maintenance_window": True,
                "firmware_version": safe_text(os.environ.get("DRONE_FIRMWARE_VERSION", "2.0.0"), "2.0.0"),
                "firmware_measurement": safe_text(os.environ.get("DRONE_FIRMWARE_MEASUREMENT", "fw-secure-local"), "fw-secure-local"),
                "firmware_signer": safe_text(os.environ.get("DRONE_FIRMWARE_SIGNER", "release-ca"), "release-ca"),
                "firmware_signature": safe_text(os.environ.get("DRONE_FIRMWARE_SIGNATURE", "pending-signature"), "pending-signature"),
                "rollback_counter": rollback_counter,
                "secure_boot_state": "SECURE_BOOT_TRUSTED",
                "boot_trust_summary": "Firmware update requested from maintenance console",
            })
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
        self._compact_layout = False
        self._selected_drone_id = safe_int(saved_settings.get("selected_drone_id"), drone_ids[0] if drone_ids else 0) or (drone_ids[0] if drone_ids else None)
        self._last_backend_url = backend_url or ""
        self._operator_role = normalize_operator_role(os.environ.get("DRONE_OPERATOR_ROLE", "operator"))
        self._security_profile = normalize_security_profile(os.environ.get("DRONE_SECURITY_PROFILE", "lab"))
        self._pending_approvals: dict[str, str] = {}

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
        root = QVBoxLayout(central)
        root.setContentsMargins(16, 14, 16, 14)
        root.setSpacing(14)
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
        role_accent = ACCENT_ALT if self._operator_role == "commander" else WARN if self._operator_role == "maintenance" else CYAN
        self._role_label = QLabel(f"Role: {self._operator_role.upper()}")
        self._role_label.setStyleSheet(f"color:{role_accent}; font-size:12px; font-weight:700;")
        self._clock_label = QLabel()
        self._clock_label.setStyleSheet(f"color:{TEXT_DIM}; font-size:12px;")

        right_header = QVBoxLayout()
        right_header.addWidget(self._mode_label, alignment=Qt.AlignRight)
        right_header.addWidget(self._role_label, alignment=Qt.AlignRight)
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
        self._alerts_card = MetricCard("CRITICAL ALERTS", DANGER)
        for idx, card in enumerate(
            [
                self._leader_card,
                self._cpu_card,
                self._gpu_card,
                self._battery_card,
                self._link_card,
                self._drift_card,
                self._alerts_card,
            ]
        ):
            cards.addWidget(card, 0, idx)
        root.addLayout(cards)

        self._body_frame = QWidget()
        self._body_shell = QHBoxLayout(self._body_frame)
        self._body_shell.setContentsMargins(0, 0, 0, 0)
        self._body_shell.setSpacing(14)

        self._left_panel = QWidget()
        self._left_layout = QVBoxLayout(self._left_panel)
        self._left_layout.setContentsMargins(0, 0, 0, 0)
        self._left_layout.setSpacing(14)

        self._right_panel = QWidget()
        self._right_layout = QVBoxLayout(self._right_panel)
        self._right_layout.setContentsMargins(0, 0, 0, 0)
        self._right_layout.setSpacing(14)

        self._stack_panel = QWidget()
        self._stack_layout = QVBoxLayout(self._stack_panel)
        self._stack_layout.setContentsMargins(0, 0, 0, 0)
        self._stack_layout.setSpacing(14)

        self._left_scroll = QScrollArea()
        self._left_scroll.setWidgetResizable(True)
        self._left_scroll.setFrameShape(QFrame.NoFrame)
        self._left_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self._left_scroll.setStyleSheet(f"QScrollArea {{ background: transparent; border: none; }}")
        self._left_scroll.setWidget(self._left_panel)

        self._right_scroll = QScrollArea()
        self._right_scroll.setWidgetResizable(True)
        self._right_scroll.setFrameShape(QFrame.NoFrame)
        self._right_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self._right_scroll.setStyleSheet(f"QScrollArea {{ background: transparent; border: none; }}")
        self._right_scroll.setWidget(self._right_panel)

        self._stack_scroll = QScrollArea()
        self._stack_scroll.setWidgetResizable(True)
        self._stack_scroll.setFrameShape(QFrame.NoFrame)
        self._stack_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self._stack_scroll.setStyleSheet(f"QScrollArea {{ background: transparent; border: none; }}")
        self._stack_scroll.setWidget(self._stack_panel)

        self._map_box = QGroupBox("3D Map View")
        self._map_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        map_layout = QVBoxLayout(self._map_box)
        map_layout.setContentsMargins(10, 16, 10, 10)
        self._map = Map3DView()
        self._map.setMinimumHeight(430)
        self._map.setMaximumHeight(540)
        map_layout.addWidget(self._map)

        self._tabs = QTabWidget()
        self._tabs.setStyleSheet(
            f"QTabWidget::pane {{ border: 1px solid {BORDER}; border-radius: 6px; background: {PANEL_ALT}; top: -1px; }}"
            f" QTabBar::tab {{ background: {PANEL_BG}; color: {TEXT_DIM}; padding: 10px 16px; border: 1px solid {BORDER};"
            f" border-bottom: none; min-width: 110px; }}"
            f" QTabBar::tab:selected {{ background: {PANEL_ALT}; color: {TEXT}; border-top: 1px solid {ACCENT}; font-weight: 700; }}"
            f" QTabBar::tab:!selected:hover {{ color: {TEXT}; }}"
        )
        self._tabs.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._tabs.setMinimumHeight(250)
        self._tabs.setMaximumHeight(320)

        self._drift = DriftGraph(self._ids, self._poll_hz)
        self._mcss_graph = MCSSScoreGraph(self._ids, self._poll_hz)
        self._net_graph = NetworkHealthGraph(self._poll_hz)
        self._comp_graph = ComputeLoadGraph(self._poll_hz)

        self._tabs.addTab(self._drift, "VIO Drift")
        self._tabs.addTab(self._mcss_graph, "MCSS Consensus")
        self._tabs.addTab(self._net_graph, "V2X Network")
        self._tabs.addTab(self._comp_graph, "Core Logic")
        self._graph_box = QGroupBox("Telemetry Graphs")
        self._graph_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        graph_layout = QVBoxLayout(self._graph_box)
        graph_layout.setContentsMargins(10, 16, 10, 10)
        graph_layout.addWidget(self._tabs)

        self._console = CommandConsole()
        self._console.set_operator_role(self._operator_role)

        self._table_container = QWidget()
        table_container_layout = QVBoxLayout(self._table_container)
        table_container_layout.setContentsMargins(0, 0, 0, 0)
        table_container_layout.setSpacing(10)
        
        table_box = QGroupBox("Swarm Status Table")
        table_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        table_layout = QVBoxLayout(table_box)
        self._table = SwarmTable()
        self._table.setMinimumHeight(150)
        table_layout.addWidget(self._table)
        table_container_layout.addWidget(table_box)

        metrics_box = QGroupBox("Advanced Metrics (VIO, MCSS, V2X)")
        metrics_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        metrics_layout = QVBoxLayout(metrics_box)
        self._metrics_table = AdvancedMetricsTable()
        self._metrics_table.setMinimumHeight(130)
        metrics_layout.addWidget(self._metrics_table)
        table_container_layout.addWidget(metrics_box)

        autonomy_box = QGroupBox("Autonomy & Zero-Trust Matrix")
        autonomy_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        autonomy_layout = QVBoxLayout(autonomy_box)
        self._autonomy_security_table = AutonomySecurityTable()
        self._autonomy_security_table.setMinimumHeight(170)
        autonomy_layout.addWidget(self._autonomy_security_table)
        table_container_layout.addWidget(autonomy_box)

        autonomy_highlights_box = QGroupBox("Autonomy Highlights Table")
        autonomy_highlights_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        autonomy_highlights_layout = QVBoxLayout(autonomy_highlights_box)
        self._autonomy_highlights_table = AutonomyHighlightsTable()
        self._autonomy_highlights_table.setMinimumHeight(130)
        autonomy_highlights_layout.addWidget(self._autonomy_highlights_table)
        table_container_layout.addWidget(autonomy_highlights_box)

        sensor_status_box = QGroupBox("Sensor Connection Status Table")
        sensor_status_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        sensor_status_layout = QVBoxLayout(sensor_status_box)
        self._sensor_status_table = SensorConnectionStatusTable()
        self._sensor_status_table.setMinimumHeight(220)
        sensor_status_layout.addWidget(self._sensor_status_table)
        table_container_layout.addWidget(sensor_status_box)

        self._fleet_overview = FleetOverviewPanel()
        self._fleet_overview.setMinimumHeight(280)
        table_container_layout.addWidget(self._fleet_overview)

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
        self._side_widget = side_widget

        root.addWidget(self._body_frame, 1)
        self._update_responsive_layout()

        status = QStatusBar()
        status.setStyleSheet(f"QStatusBar {{ background:{PANEL_BG}; color:{TEXT_DIM}; }}")
        self._fps_label = QLabel("UI FPS: --")
        status.addPermanentWidget(self._fps_label)
        self.setStatusBar(status)

    def _update_responsive_layout(self) -> None:
        compact = self.width() < 1280
        if compact == self._compact_layout and self._body_shell.count():
            return
        self._compact_layout = compact

        while self._body_shell.count():
            item = self._body_shell.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.setParent(None)

        for layout in (self._left_layout, self._right_layout, self._stack_layout):
            while layout.count():
                item = layout.takeAt(0)
                widget = item.widget()
                if widget is not None:
                    widget.setParent(None)

        if compact:
            self._map.setMinimumHeight(320)
            self._map.setMaximumHeight(400)
            self._tabs.setMinimumHeight(240)
            self._tabs.setMaximumHeight(300)
            self._stack_layout.addWidget(self._map_box)
            self._stack_layout.addWidget(self._graph_box)
            self._stack_layout.addWidget(self._console)
            self._stack_layout.addWidget(self._table_container)
            self._stack_layout.addWidget(self._side_widget)
            self._stack_layout.addStretch(1)
            self._body_shell.addWidget(self._stack_scroll, 1)
            return

        self._map.setMinimumHeight(430)
        self._map.setMaximumHeight(540)
        self._tabs.setMinimumHeight(250)
        self._tabs.setMaximumHeight(320)
        self._left_layout.addWidget(self._map_box)
        self._left_layout.addWidget(self._table_container)
        self._left_layout.setStretch(0, 5)
        self._left_layout.setStretch(1, 3)
        self._left_layout.addStretch(1)

        self._right_layout.addWidget(self._graph_box)
        self._right_layout.addWidget(self._console)
        self._right_layout.addWidget(self._side_widget)
        self._right_layout.addStretch(1)

        self._body_shell.addWidget(self._left_scroll, 3)
        self._body_shell.addWidget(self._right_scroll, 2)

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._update_responsive_layout()

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

        self._approval_timer = QTimer(self)
        self._approval_timer.timeout.connect(self._sync_pending_approvals_from_backend)
        self._approval_timer.start(2500)

        self._telemetry.start()
        self._commands.start()

    def _restore_persisted_state(self) -> None:
        for line in self._store.load_recent_commands(12):
            self._console.append_log(line)
        self._sync_pending_approvals_from_backend()
        self._refresh_approval_status()
        snapshot = self._store.load_snapshot()
        if snapshot is not None:
            self._on_snapshot(snapshot)

    def _update_clock(self) -> None:
        self._clock_label.setText(time.strftime("%Y-%m-%d  %H:%M:%S"))

    def _flush_fps(self) -> None:
        self._fps_label.setText(f"UI FPS: {self._frame_counter}")
        self._frame_counter = 0

    def _refresh_approval_status(self) -> None:
        self._console.set_pending_approvals(self._pending_approvals)
        if self._pending_approvals:
            pending_summary = ", ".join(action.upper() for action in sorted(self._pending_approvals))
            self.statusBar().showMessage(f"Pending approval required: {pending_summary}")

    def _sync_pending_approvals_from_backend(self) -> None:
        approvals = self._backend.pending_approvals()
        if approvals != self._pending_approvals:
            self._pending_approvals = approvals
            self._refresh_approval_status()

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
            self._autonomy_security_table.ingest(snapshot)
            self._autonomy_highlights_table.ingest(snapshot)
            self._sensor_status_table.ingest(snapshot, self._env_manager)
            self._fleet_overview.ingest(snapshot)
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
            self._alerts_card.set_data(
                str(snapshot.critical_alerts),
                f"{len(snapshot.clusters)} clusters | {len(snapshot.missions)} missions tracked",
                accent=SUCCESS if snapshot.critical_alerts == 0 else WARN if snapshot.critical_alerts < 3 else DANGER,
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
                f"Motor: {avg_motor * 100.0:.0f}% | Election-ready: {ready_count}/{len(snapshot.states)} | "
                f"Critical alerts: {snapshot.critical_alerts}"
            )
        except Exception as exc:
            logger.exception("snapshot render failed")
            self._console.append_log(f"snapshot render error: {exc}")
            self.statusBar().showMessage("Dashboard render degraded; check command log")

    def _placeholder_state(self, drone_id: int) -> DroneState:
        existing_states = list(self._last_snapshot.states) if self._last_snapshot is not None else []
        index = len(existing_states)
        now = time.time()
        radius = 2.4 + index * 1.3
        angle = now * (0.22 + index * 0.015) + index * 0.85
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        z = 2.0 + 0.8 * index + 0.35 * math.sin(now * 0.5 + index)
        vx = -radius * math.sin(angle) * 0.22
        vy = radius * math.cos(angle) * 0.22
        cluster_id = f"cluster-{((drone_id - 1) // 20) + 1:02d}"
        has_cluster_leader = any(
            state.cluster_id == cluster_id and state.role == "LEADER"
            for state in existing_states
        )
        role = "FOLLOWER" if has_cluster_leader else "LEADER"
        attitude_rpy, thrust_vector = derive_attitude_thrust((x, y, z), (vx, vy, 0.0), role, now)
        return DroneState(
            drone_id=drone_id,
            cluster_id=cluster_id,
            role=role,
            mission_state="standby",
            position=(x, y, z),
            velocity=(vx, vy, 0.0),
            commanded_altitude_m=z,
            commanded_speed_mps=2.5,
            manual_target_position=(x, y, z),
            manual_control_active=False,
            attitude_rpy=attitude_rpy,
            thrust_vector=thrust_vector,
            drift_m=0.05,
            battery_pct=96.0,
            connectivity="Mesh",
            reachable=True,
            rssi_dbm=-50.0,
            cpu_temp_c=self._last_snapshot.cpu_temp_c if self._last_snapshot is not None else 54.0,
            gpu_load_pct=self._last_snapshot.gpu_load_pct if self._last_snapshot is not None else 38.0,
            localization_source="demo-visualization",
            localization_state="nominal",
            localization_confidence=0.92,
            tdoa_confidence=0.66,
            confidence_trend=0.03,
            relocalization_count=0,
            visible_anchor_count=5,
            occupancy_ratio=0.14,
            sync_confidence=0.93,
            imu_camera_offset_ms=2.1,
            peer_clock_offset_ms=0.8,
            anchor_visibility_ratio=0.82,
            tdoa_weight=0.64,
            planned_waypoint_count=4,
            last_relocalized_keyframe=51,
            security_state="TRUSTED",
            security_summary="Demo drone injected into local dashboard state",
            security_transition_reason="manual-add",
            remote_command_allowed=True,
            telemetry_uplink_allowed=True,
            link_integrity_score=0.90,
            trust_epoch=2,
            firmware_measurement="fw-demo-local",
            firmware_version="2.0.0",
            secure_boot_state="SECURE_BOOT_TRUSTED",
            boot_trust_summary="Dashboard demo trust profile",
            rollback_counter=1,
            last_remote_command_status="dashboard add_drone",
            health_flags=[],
            motor_health=0.94,
            leadership_score=0.72 if role == "LEADER" else 0.54,
            election_ready=True,
            timestamp=now,
        )

    def _inject_local_drone_visual(self, drone_id: int) -> None:
        if drone_id <= 0:
            return
        if self._last_snapshot is not None and any(state.drone_id == drone_id for state in self._last_snapshot.states):
            return
        states = list(self._last_snapshot.states) if self._last_snapshot is not None else []
        states.append(self._placeholder_state(drone_id))
        states.sort(key=lambda state: state.drone_id)
        snapshot = DashboardSnapshot(
            states=states,
            backend_mode=self._last_snapshot.backend_mode if self._last_snapshot is not None else self._backend.mode,
            leader_id=self._last_snapshot.leader_id if self._last_snapshot is not None else (drone_id if states and states[0].role == "LEADER" else None),
            avg_latency_ms=self._last_snapshot.avg_latency_ms if self._last_snapshot is not None else 0.0,
            packet_loss_pct=self._last_snapshot.packet_loss_pct if self._last_snapshot is not None else 0.0,
            cpu_temp_c=self._last_snapshot.cpu_temp_c if self._last_snapshot is not None else 54.0,
            gpu_load_pct=self._last_snapshot.gpu_load_pct if self._last_snapshot is not None else 38.0,
            election_state=self._last_snapshot.election_state if self._last_snapshot is not None else "stable",
            clusters=self._last_snapshot.clusters if self._last_snapshot is not None else self._backend._cluster_snapshot(states),
            critical_alerts=self._last_snapshot.critical_alerts if self._last_snapshot is not None else self._backend._critical_alert_count(states),
            health=self._last_snapshot.health if self._last_snapshot is not None else self._backend._health_snapshot(states, 54.0),
            missions=self._last_snapshot.missions if self._last_snapshot is not None else [],
            events=self._last_snapshot.events if self._last_snapshot is not None else [],
            services=self._last_snapshot.services if self._last_snapshot is not None else ["dashboard-console"],
            timestamp=time.time(),
        )
        self._on_snapshot(snapshot)

    def _remove_local_drone_visual(self, drone_id: int) -> None:
        if self._last_snapshot is None:
            return
        states = [state for state in self._last_snapshot.states if state.drone_id != drone_id]
        if len(states) == len(self._last_snapshot.states):
            return
        leader_id = self._last_snapshot.leader_id
        if leader_id == drone_id:
            leader_id = states[0].drone_id if states else None
        snapshot = DashboardSnapshot(
            states=states,
            backend_mode=self._last_snapshot.backend_mode,
            leader_id=leader_id,
            avg_latency_ms=self._last_snapshot.avg_latency_ms,
            packet_loss_pct=self._last_snapshot.packet_loss_pct,
            cpu_temp_c=self._last_snapshot.cpu_temp_c,
            gpu_load_pct=self._last_snapshot.gpu_load_pct,
            election_state=self._last_snapshot.election_state,
            clusters=self._backend._cluster_snapshot(states),
            critical_alerts=self._backend._critical_alert_count(states),
            health=self._backend._health_snapshot(states, self._last_snapshot.cpu_temp_c),
            missions=self._last_snapshot.missions,
            events=self._last_snapshot.events,
            services=self._last_snapshot.services,
            timestamp=time.time(),
        )
        self._on_snapshot(snapshot)

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
            requested_drone_id = safe_int(request.payload.get("drone_id"), 0)
            states = self._last_snapshot.states if self._last_snapshot else []
            selected = self._selected_state(states)
            target_state = next((state for state in states if state.drone_id == requested_drone_id), None) if requested_drone_id > 0 else selected
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
                if target_state is None:
                    self._console.append_log("remove rejected: no drone selected")
                    return
                answer = QMessageBox.question(
                    self,
                    "Remove Drone",
                    f"Remove drone {target_state.drone_id} and delete its drone-wise sensor connection from .env?",
                    QMessageBox.Yes | QMessageBox.No,
                    QMessageBox.No,
                )
                if answer != QMessageBox.Yes:
                    self._console.append_log("remove drone cancelled")
                    return
                request.payload["drone_id"] = target_state.drone_id
                request.payload["target_ids"] = [target_state.drone_id]
                request.payload["cluster_id"] = target_state.cluster_id
                self._env_manager.remove_drone(target_state.drone_id)
                self._console.append_log(f"drone {target_state.drone_id} removed from .env")
                self._commands.submit(request)
                return
            if request.action == "election" and self._operator_role != "commander" and self._security_profile != "lab":
                self._console.append_log(f"command rejected locally: election requires commander role, current role is {self._operator_role}")
                return
            if request.action in {"maintenance_mode", "firmware_update"} and self._operator_role != "maintenance" and self._security_profile != "lab":
                self._console.append_log(f"command rejected locally: {request.action} requires maintenance role, current role is {self._operator_role}")
                return
            approval_id = self._pending_approvals.get(request.action)
            if approval_id:
                request.payload["approval_id"] = approval_id
                self._console.append_log(f"approval confirmation submitted for {request.action} ({approval_id})")
                self._refresh_approval_status()
            if request.action == "add_drone":
                self._commands.submit(request)
                return
            scope = request.payload.get("scope", "Fleet")
            if target_state is not None and requested_drone_id > 0:
                request.payload["target_ids"] = [target_state.drone_id]
                request.payload["cluster_id"] = target_state.cluster_id
                self._commands.submit(request)
                return
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
        approval_required = bool(request_obj.payload.get("_approval_required"))
        approval_id = safe_text(request_obj.payload.get("_approval_id"), "")
        if approval_required and approval_id:
            self._pending_approvals[request_obj.action] = approval_id
        elif request_obj.action in self._pending_approvals and "approval" not in message.lower():
            self._pending_approvals.pop(request_obj.action, None)
        self._refresh_approval_status()
        normalized_message = message.strip().lower()
        add_succeeded = request_obj.action == "add_drone" and "added" in normalized_message and "failed" not in normalized_message
        remove_succeeded = request_obj.action == "remove_drone" and "removed" in normalized_message and "failed" not in normalized_message and "not found" not in normalized_message
        if add_succeeded and drone_id > 0 and drone_id not in self._ids:
            self._ids.append(drone_id)
            self._ids.sort()
            self._selected_drone_id = drone_id
        if remove_succeeded and drone_id in self._ids:
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


def normalize_security_profile(value: str) -> str:
    lowered = value.strip().lower()
    if lowered in {"field"}:
        return "field"
    if lowered in {"production", "prod"}:
        return "production"
    return "lab"


def normalize_operator_role(value: str) -> str:
    lowered = value.strip().lower()
    if lowered in {"commander", "mission-commander", "mission_commander"}:
        return "commander"
    if lowered in {"maintenance", "maintainer"}:
        return "maintenance"
    return "operator"


def security_profile_requires_signed(profile: str) -> bool:
    return normalize_security_profile(profile) in {"field", "production"}


def validate_backend_security_profile(base_url: str, profile: str) -> None:
    normalized_profile = normalize_security_profile(profile)
    if normalized_profile == "lab":
        return
    if not base_url.lower().startswith("https://"):
        raise ValueError("field/production mode requires an https control-plane backend URL")
    client_cert = os.environ.get("DRONE_TLS_CLIENT_CERT_FILE", "").strip()
    client_key = os.environ.get("DRONE_TLS_CLIENT_KEY_FILE", "").strip()
    ca_file = os.environ.get("DRONE_TLS_CA_FILE", "").strip()
    if not ca_file:
        raise ValueError("field/production mode requires DRONE_TLS_CA_FILE for backend trust")
    if not client_cert:
        raise ValueError("field/production mode requires DRONE_TLS_CLIENT_CERT_FILE for mTLS")
    if not client_key:
        raise ValueError("field/production mode requires DRONE_TLS_CLIENT_KEY_FILE for mTLS")


def build_backend_ssl_context(base_url: str) -> ssl.SSLContext | None:
    if not base_url.lower().startswith("https://"):
        return None

    skip_verify = parse_env_bool("DRONE_TLS_SKIP_VERIFY", False)
    ca_file = os.environ.get("DRONE_TLS_CA_FILE", "").strip()
    client_cert = os.environ.get("DRONE_TLS_CLIENT_CERT_FILE", "").strip()
    client_key = os.environ.get("DRONE_TLS_CLIENT_KEY_FILE", "").strip()

    if skip_verify:
        context = ssl._create_unverified_context()
    else:
        context = ssl.create_default_context(cafile=ca_file or None)

    if client_cert:
        context.load_cert_chain(client_cert, keyfile=client_key or None)
    return context


def parse_env_bool(key: str, fallback: bool) -> bool:
    value = os.environ.get(key, "").strip().lower()
    if value in {"1", "true", "yes", "on"}:
        return True
    if value in {"0", "false", "no", "off"}:
        return False
    return fallback


def sign_command_envelope(
    secret_key: str,
    action: str,
    payload_json: str,
    operator_id: str,
    operator_role: str,
    issued_at: str,
    expires_at: str,
    nonce: str,
) -> str:
    canonical = "\n".join(
        [
            action.strip().lower(),
            payload_json,
            operator_id,
            normalize_operator_role(operator_role),
            issued_at,
            expires_at,
            nonce,
        ]
    )
    return hmac.new(secret_key.encode("utf-8"), canonical.encode("utf-8"), hashlib.sha256).hexdigest()


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
