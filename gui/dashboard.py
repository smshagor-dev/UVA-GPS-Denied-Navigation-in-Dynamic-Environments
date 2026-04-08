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
import queue
import random
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
    QComboBox,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QStatusBar,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)


DARK_BG = "#070B14"
PANEL_BG = "#111827"
PANEL_ALT = "#0F172A"
BORDER = "#1F2937"
TEXT = "#E5EEF7"
TEXT_DIM = "#93A4B8"
ACCENT = "#26C6DA"
ACCENT_ALT = "#F97316"
SUCCESS = "#22C55E"
WARN = "#F59E0B"
DANGER = "#EF4444"
CYAN = "#38BDF8"
MAGENTA = "#F472B6"


logger = logging.getLogger(__name__)
LOG_DIR = Path(__file__).resolve().parent.parent / "logs" / "dashboard"


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


def discover_bridge():
    root = Path(__file__).resolve().parent.parent
    candidates = [
        root / "build",
        root / "build" / "Release",
        root / "build-dashboard",
        root / "build-dashboard" / "Debug",
        root / "build-dashboard" / "Release",
        root / "build-full",
        root / "build-full" / "Release",
        root / "build-check",
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
                drift_m=safe_float(item.get("drift_m", 0.0)),
                battery_pct=safe_float(item.get("battery_pct", 0.0)),
                connectivity=safe_text(item.get("connectivity", "Unknown"), "Unknown"),
                reachable=bool(item.get("reachable", False)),
                rssi_dbm=safe_float(item.get("rssi_dbm", -100.0), -100.0),
                cpu_temp_c=safe_float(item.get("cpu_temp_c", 0.0)),
                gpu_load_pct=safe_float(item.get("gpu_load_pct", 0.0)),
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

    def __init__(self, drone_ids: list[int], poll_hz: int, backend_url: str | None = None) -> None:
        self._ids = drone_ids
        self._poll_hz = max(poll_hz, 1)
        self._bridge = BRIDGE
        self._mode = BRIDGE_MODE
        self._pipelines: dict[int, Any] = {}
        self._network = None
        self._control_ready = False
        self._started_at = time.time()
        self._client = GoControlPlaneClient(backend_url) if backend_url else None
        self._mission_overrides: dict[int, str] = {}
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
            and getattr(self._bridge, "BUILD_FASTDDS", False)
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
                peer = peer_map.get(drone_id)
                position = tuple(float(v) for v in pose.position)
                velocity = tuple(float(v) for v in pose.velocity)
                role = enum_name(peer.role) if peer is not None else ("LEADER" if drone_id == 1 else "FOLLOWER")
                battery = float(peer.battery_pct) if peer is not None else float(stats.battery_pct)
                rssi = float(peer.rssi_dbm) if peer is not None else float(stats.wifi_rssi_dbm)
                reachable = bool(peer.reachable) if peer is not None else True

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
                        drift_m=float(self._pipelines[drone_id].drift_m()),
                        battery_pct=battery,
                        connectivity="Mesh" if reachable else "Lost",
                        reachable=reachable,
                        rssi_dbm=rssi,
                        cpu_temp_c=float(stats.cpu_temp_c),
                        gpu_load_pct=float(stats.gpu_pct),
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
            synthesized.append(
                DroneState(
                    drone_id=drone_id,
                    cluster_id=f"cluster-{((drone_id - 1) // 20) + 1:02d}",
                    role="LEADER" if idx == 0 else "FOLLOWER",
                    mission_state=self._mission_overrides.get(drone_id, "formation-hold"),
                    position=tuple((base + offset).tolist()),
                    velocity=tuple(velocity.tolist()),
                    drift_m=anchor.drift_m + 0.015 * idx,
                    battery_pct=max(20.0, anchor.battery_pct - idx * 2.5),
                    connectivity="Mesh",
                    reachable=True,
                    rssi_dbm=anchor.rssi_dbm - idx * 1.8,
                    cpu_temp_c=cpu_temp_c,
                    gpu_load_pct=gpu_load_pct,
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
                    drift_m=drift,
                    battery_pct=battery,
                    connectivity="Mesh" if reachable else "Degraded",
                    reachable=reachable,
                    rssi_dbm=rssi,
                    cpu_temp_c=cpu_temp,
                    gpu_load_pct=max(0.0, gpu_load),
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
        if not self._control_ready or self._network is None or self._bridge is None:
            if request.action == "add_drone":
                drone_id = self._add_local_drone(int(request.payload.get("drone_id", 0)) or None)
                return f"drone {drone_id} added in simulation mode"
            if request.action in {"election", "formation", "hold_position", "return_home", "emergency_land"}:
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

            if request.action in {"hold_position", "return_home"}:
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

    def stop(self) -> None:
        self._running = False


class MetricCard(QFrame):
    def __init__(self, title: str, accent: str) -> None:
        super().__init__()
        self._accent = accent
        self._value = QLabel("--")
        self._subtitle = QLabel("waiting")
        self._title = QLabel(title)

        self.setObjectName("metricCard")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 14, 16, 14)
        layout.setSpacing(6)

        self._title.setStyleSheet(f"color: {TEXT_DIM}; font-size: 11px; letter-spacing: 1px;")
        self._value.setStyleSheet(f"color: {accent}; font-size: 26px; font-weight: 700;")
        self._subtitle.setStyleSheet(f"color: {TEXT_DIM}; font-size: 11px;")

        layout.addWidget(self._title)
        layout.addWidget(self._value)
        layout.addWidget(self._subtitle)

    def set_data(self, value: str, subtitle: str, accent: str | None = None) -> None:
        if accent is None:
            accent = self._accent
        self._value.setText(value)
        self._value.setStyleSheet(f"color: {accent}; font-size: 26px; font-weight: 700;")
        self._subtitle.setText(subtitle)


class Map3DView(gl.GLViewWidget):
    def __init__(self) -> None:
        super().__init__()
        self.setBackgroundColor(pg.mkColor(DARK_BG))
        self.setCameraPosition(distance=28, elevation=24, azimuth=38)

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


class DroneDetailPanel(QGroupBox):
    def __init__(self) -> None:
        super().__init__("Selected Drone")
        self.setMinimumHeight(170)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(10, 12, 10, 10)
        outer.setSpacing(0)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setStyleSheet(f"QScrollArea {{ background: transparent; border: none; }}")
        content = QWidget()
        layout = QGridLayout(content)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setHorizontalSpacing(12)
        layout.setVerticalSpacing(8)
        layout.setColumnStretch(0, 0)
        layout.setColumnStretch(1, 1)
        self._labels: dict[str, QLabel] = {}
        fields = [
            ("Drone", "drone"),
            ("Cluster", "cluster"),
            ("Role", "role"),
            ("Mission", "mission"),
            ("Position", "position"),
            ("Velocity", "velocity"),
            ("Battery", "battery"),
            ("RSSI", "rssi"),
            ("Motor Health", "motor"),
            ("MCSS Score", "score"),
            ("Election", "election"),
        ]
        for row, (title, key) in enumerate(fields):
            title_label = QLabel(title)
            title_label.setStyleSheet(f"color:{TEXT_DIM}; font-size:12px; font-weight:600;")
            value_label = QLabel("--")
            value_label.setStyleSheet(f"color:{TEXT}; font-size:12px;")
            value_label.setWordWrap(True)
            value_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            layout.addWidget(title_label, row, 0)
            layout.addWidget(value_label, row, 1)
            self._labels[key] = value_label
        scroll.setWidget(content)
        outer.addWidget(scroll)

    def ingest(self, state: DroneState | None) -> None:
        if state is None:
            for label in self._labels.values():
                label.setText("--")
            return
        self._labels["drone"].setText(str(state.drone_id))
        self._labels["cluster"].setText(state.cluster_id)
        self._labels["role"].setText(state.role)
        self._labels["mission"].setText(state.mission_state)
        self._labels["position"].setText(f"{state.position[0]:.2f}, {state.position[1]:.2f}, {state.position[2]:.2f}")
        self._labels["velocity"].setText(f"{state.velocity[0]:.2f}, {state.velocity[1]:.2f}, {state.velocity[2]:.2f}")
        self._labels["battery"].setText(f"{state.battery_pct:.1f}%")
        self._labels["rssi"].setText(f"{state.rssi_dbm:.0f} dBm")
        self._labels["motor"].setText(f"{state.motor_health * 100.0:.0f}%")
        self._labels["score"].setText(f"{state.leadership_score:.3f}")
        self._labels["election"].setText("Ready" if state.election_ready else "Blocked")


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
            ("Election", SUCCESS, "election"),
            ("VEE", ACCENT, "VEE"),
            ("LINE", CYAN, "LINE"),
            ("DIAMOND", MAGENTA, "DIAMOND"),
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
    def __init__(self, drone_ids: list[int], poll_hz: int, backend_url: str | None = None) -> None:
        super().__init__()
        self._ids = drone_ids
        self._poll_hz = poll_hz
        self._frame_counter = 0
        self._last_snapshot: DashboardSnapshot | None = None
        self._selected_drone_id = drone_ids[0] if drone_ids else None

        self._backend = SwarmBackend(drone_ids, poll_hz, backend_url=backend_url)
        self._telemetry = TelemetryWorker(self._backend, poll_hz)
        self._commands = CommandWorker(self._backend)

        self.setWindowTitle("Drone Swarm Monitoring and Control Dashboard")
        self.resize(1680, 980)
        self._apply_theme()
        self._build_ui()
        self._wire_threads()

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
                background: {PANEL_BG};
                border: 1px solid {BORDER};
                border-radius: 14px;
                margin-top: 10px;
                color: {TEXT_DIM};
                font-size: 12px;
                font-weight: 600;
                padding-top: 10px;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 14px;
                padding: 0 4px;
            }}
            QTableWidget {{
                background: {PANEL_BG};
                color: {TEXT};
                alternate-background-color: {PANEL_ALT};
                border: none;
                font-size: 12px;
            }}
            QHeaderView::section {{
                background: {PANEL_ALT};
                color: {TEXT_DIM};
                border: none;
                padding: 8px;
                font-weight: 700;
            }}
            QLabel#heroTitle {{
                color: {TEXT};
                font-size: 24px;
                font-weight: 700;
            }}
            QLabel#heroSub {{
                color: {TEXT_DIM};
                font-size: 12px;
            }}
            QFrame#metricCard {{
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 {PANEL_BG}, stop:1 {PANEL_ALT});
                border: 1px solid {BORDER};
                border-radius: 14px;
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

        drift_box = QGroupBox("EKF Drift Graph")
        drift_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        drift_layout = QVBoxLayout(drift_box)
        self._drift = DriftGraph(self._ids, self._poll_hz)
        self._drift.setMinimumHeight(180)
        drift_layout.addWidget(self._drift)
        body.addWidget(drift_box, 0, 2, 1, 1)

        self._console = CommandConsole()
        body.addWidget(self._console, 1, 2, 1, 1)

        table_box = QGroupBox("Swarm Status Table")
        table_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        table_layout = QVBoxLayout(table_box)
        self._table = SwarmTable()
        self._table.setMinimumHeight(180)
        table_layout.addWidget(self._table)
        body.addWidget(table_box, 2, 0, 1, 2)

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

    def _update_clock(self) -> None:
        self._clock_label.setText(time.strftime("%Y-%m-%d  %H:%M:%S"))

    def _flush_fps(self) -> None:
        self._fps_label.setText(f"UI FPS: {self._frame_counter}")
        self._frame_counter = 0

    def _on_snapshot(self, snapshot: DashboardSnapshot) -> None:
        try:
            logger.debug("DashboardWindow on_snapshot states=%s backend=%s", len(snapshot.states), snapshot.backend_mode)
            self._last_snapshot = snapshot
            self._frame_counter += 1
            self._mode_label.setText(f"Backend: {snapshot.backend_mode.upper()}")

            self._map.ingest(snapshot.states)
            self._drift.ingest(snapshot.states)
            self._table.ingest(snapshot.states)
            self._select_row_for_drone()
            self._detail.ingest(self._selected_state(snapshot.states))

            leader_id = snapshot.leader_id if snapshot.leader_id else "?"
            avg_battery = sum(state.battery_pct for state in snapshot.states) / max(len(snapshot.states), 1)
            avg_drift = sum(state.drift_m for state in snapshot.states) / max(len(snapshot.states), 1)
            avg_motor = sum(state.motor_health for state in snapshot.states) / max(len(snapshot.states), 1)
            ready_count = sum(1 for state in snapshot.states if state.election_ready)
            online = sum(1 for state in snapshot.states if state.reachable)
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
                f"{avg_drift:.3f} m",
                "EKF drift envelope",
                accent=MAGENTA if avg_drift < 0.35 else WARN,
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
                f"Avg drift: {avg_drift:.3f} m | Motor: {avg_motor * 100.0:.0f}% | "
                f"Election-ready: {ready_count}/{len(snapshot.states)}"
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

    def _dispatch_command(self, request: CommandRequest) -> None:
        try:
            logger.info("Dashboard dispatch requested action=%s payload=%s", request.action, request.payload)
            request = CommandRequest(request.action, dict(request.payload))
            if request.action == "add_drone":
                if self._last_snapshot is not None and self._last_snapshot.states:
                    request.payload["drone_id"] = max(state.drone_id for state in self._last_snapshot.states) + 1
                elif self._ids:
                    request.payload["drone_id"] = max(self._ids) + 1
                else:
                    request.payload["drone_id"] = 1
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
    parser.add_argument("--ids", default="1,2,3,4,5", help="comma-separated drone ids")
    parser.add_argument("--poll-hz", type=int, default=20, help="telemetry polling rate")
    parser.add_argument("--backend-url", default="", help="Go control-plane base URL for fleet mode")
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
    configure_logging()
    try:
        args = parse_args()
        drone_ids = [safe_int(part.strip(), -1) for part in args.ids.split(",") if part.strip()]
        drone_ids = [drone_id for drone_id in drone_ids if drone_id > 0]
        if not drone_ids:
            drone_ids = [1, 2, 3, 4, 5]

        pg.setConfigOptions(antialias=False, useOpenGL=True)
        app = QApplication(sys.argv)
        app.setApplicationName("DroneSwarmDashboard")

        backend_url = args.backend_url.strip() or None
        window = DashboardWindow(drone_ids, args.poll_hz, backend_url=backend_url)
        window.show()
        return app.exec()
    except Exception as exc:
        logger.exception("dashboard startup failed")
        print(f"dashboard startup failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
