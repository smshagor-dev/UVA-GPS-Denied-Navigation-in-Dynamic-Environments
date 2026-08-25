from __future__ import annotations

import math
import time
from collections import deque
from pathlib import Path
from typing import Any

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QColor, QFont, QImage, QPainter, QPainterPath, QPen, QPixmap
from PySide6.QtWidgets import (
    QFrame,
    QGridLayout,
    QHeaderView,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QProgressBar,
    QSizePolicy,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from gui.dashboard_contract import DashboardSnapshot, DroneState


DARK_BG = "#0B1017"
PANEL_BG = "#141A22"
PANEL_ALT = "#191F28"
BORDER = "#2A313C"
TEXT = "#F2F4F7"
TEXT_DIM = "#8E98A8"
SUCCESS = "#63D02D"
WARN = "#F3C739"
DANGER = "#FF4A45"
CYAN = "#4FA3FF"
PURPLE = "#B566FF"
TEAL = "#38D1D1"
AMBER = "#F59E0B"
GREEN = "#77D63A"

SERIES_COLORS = [
    GREEN,
    CYAN,
    PURPLE,
    DANGER,
    AMBER,
    TEAL,
    "#D94CF2",
    "#E2D43B",
    "#A4E913",
    "#F97316",
]


def rgba(hex_color: str, alpha: float) -> QColor:
    color = QColor(hex_color)
    color.setAlphaF(max(0.0, min(1.0, alpha)))
    return color


def metric_color(value: str) -> str:
    lowered = value.lower()
    if "critical" in lowered or "offline" in lowered or "degraded" in lowered:
        return DANGER
    if "warning" in lowered or "stale" in lowered:
        return WARN
    return SUCCESS


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def human_hms(seconds: float) -> str:
    seconds = max(0, int(seconds))
    minutes, sec = divmod(seconds, 60)
    hours, minutes = divmod(minutes, 60)
    return f"{hours:02d}:{minutes:02d}:{sec:02d}"


def estimate_transfer_rates(
    snapshot: DashboardSnapshot, state: DroneState | None
) -> tuple[float, float]:
    reachable = [item for item in snapshot.states if item.reachable]
    if not reachable:
        return 0.0, 0.0
    average_mesh = snapshot.avg_mesh_bandwidth_kbps or (
        sum(item.mesh_bandwidth_kbps for item in reachable) / len(reachable)
    )
    selected = state or reachable[0]
    selected_mesh = max(selected.mesh_bandwidth_kbps, average_mesh * 0.72)
    uplink_ratio = sum(1 for item in reachable if item.telemetry_uplink_allowed) / max(
        1, len(reachable)
    )
    peer_factor = clamp(
        selected.peer_count / max(1, len(snapshot.states) - 1),
        0.28,
        1.0,
    )
    integrity_factor = clamp(
        sum(item.link_integrity_score for item in reachable) / len(reachable),
        0.35,
        1.0,
    )
    loss_penalty = clamp(1.0 - (snapshot.packet_loss_pct / 100.0), 0.4, 1.0)
    uplink = average_mesh * (0.78 + 0.36 * uplink_ratio) * integrity_factor
    downlink = selected_mesh * (0.48 + 0.34 * peer_factor) * loss_penalty
    return max(0.0, downlink), max(0.0, uplink)


def signal_bars(rssi_dbm: float, reachable: bool) -> str:
    if not reachable:
        return "○○○○○"
    strength_pct = clamp((rssi_dbm + 100.0) * 2.0, 0.0, 100.0)
    filled = max(1, min(5, int(math.ceil(strength_pct / 20.0))))
    return "●" * filled + "○" * (5 - filled)


def selected_or_first(
    snapshot: DashboardSnapshot, selected_id: int | None
) -> DroneState | None:
    if not snapshot.states:
        return None
    if selected_id is not None:
        for state in snapshot.states:
            if state.drone_id == selected_id:
                return state
    return snapshot.states[0]


def style_panel(frame: QFrame) -> None:
    frame.setStyleSheet(
        f"QFrame {{ background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 {PANEL_BG}, stop:1 {PANEL_ALT});"
        f" border: 1px solid {BORDER}; border-radius: 10px; }}"
    )


def panel_title(title: str) -> QLabel:
    label = QLabel(title)
    label.setStyleSheet(
        f"color:{TEXT}; font-size:15px; font-weight:700; letter-spacing:0.4px;"
    )
    return label


class SidebarButton(QFrame):
    def __init__(self, icon_text: str, label_text: str, active: bool = False) -> None:
        super().__init__()
        self.setObjectName("sidebarButton")
        self.setStyleSheet(
            f"QFrame#sidebarButton {{ background: {'rgba(255,255,255,0.03)' if active else 'transparent'}; "
            f"border: none; border-radius: 12px; }}"
        )
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(8)
        icon = QLabel(icon_text)
        icon.setAlignment(Qt.AlignCenter)
        icon.setStyleSheet(
            f"color:{TEXT if not active else '#E8D8FF'}; font-size:26px; font-weight:700;"
        )
        text = QLabel(label_text.upper())
        text.setAlignment(Qt.AlignCenter)
        text.setWordWrap(True)
        text.setStyleSheet(
            f"color:{TEXT}; font-size:11px; font-weight:600; letter-spacing:0.8px;"
        )
        layout.addWidget(icon)
        layout.addWidget(text)


class SidebarNavFrame(QFrame):
    def __init__(self) -> None:
        super().__init__()
        self.setFixedWidth(86)
        self.setStyleSheet(
            f"QFrame {{ background: #12171F; border-right: 1px solid {BORDER}; }}"
        )
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 12, 10, 12)
        layout.setSpacing(10)

        logo_box = QFrame()
        logo_box.setFixedHeight(60)
        logo_box.setStyleSheet(
            "QFrame { background: rgba(255,255,255,0.02); border-radius: 14px; border: 1px solid rgba(255,255,255,0.08); }"
        )
        logo_layout = QVBoxLayout(logo_box)
        logo_layout.setContentsMargins(0, 0, 0, 0)
        logo = QLabel("◈")
        logo.setAlignment(Qt.AlignCenter)
        logo.setStyleSheet(
            "color:#DCC7FF; font-size:26px; font-weight:800; background:#5C3AAE; border-radius:12px; margin:8px;"
        )
        logo_layout.addWidget(logo)
        layout.addWidget(logo_box)

        layout.addSpacing(6)
        for icon, text, active in [
            ("◔", "Dashboard", True),
            ("⌖", "Map", False),
            ("⚓", "Plan", False),
            ("✈", "Vehicles", False),
            ("▣", "Analysis", False),
            ("⚙", "Settings", False),
        ]:
            layout.addWidget(SidebarButton(icon, text, active))
        layout.addStretch(1)
        layout.addWidget(SidebarButton("?", "Help", False))


class OperationalSidebar(QFrame):
    menu_requested = Signal(str)

    def __init__(self) -> None:
        super().__init__()
        self._buttons: dict[str, QPushButton] = {}
        self.setFixedWidth(86)
        self.setStyleSheet(
            f"QFrame {{ background: #12171F; border-right: 1px solid {BORDER}; }}"
        )
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 10, 8, 10)
        layout.setSpacing(10)

        brand = QFrame()
        brand.setFixedHeight(52)
        brand.setStyleSheet("QFrame { background: transparent; border: none; }")
        brand_layout = QVBoxLayout(brand)
        brand_layout.setContentsMargins(10, 4, 10, 4)
        title = QLabel("✦")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(
            "color:#F5F5F5; font-size:20px; font-weight:800; background:#5C3AAE; border:1px solid rgba(220,199,255,0.5); border-radius:12px;"
        )
        brand_layout.addWidget(title)
        layout.addWidget(brand)

        for menu_name, button_label in [
            ("Dashboard", "▣  Dashboard"),
            ("Map", "⌖  Map"),
            ("Plan", "⚑  Plan"),
            ("Vehicles", "✈  Vehicles"),
            ("Analysis", "▣  Analysis"),
            ("Settings", "⚙  Settings"),
            ("Help", "?  Help"),
        ]:
            button = QPushButton(button_label)
            button.setCursor(Qt.PointingHandCursor)
            button.setCheckable(True)
            button.setMinimumHeight(36)
            button.setStyleSheet(
                f"QPushButton {{ text-align:left; color:{TEXT}; background: transparent; border:none; border-radius:10px; padding:7px 10px; font-size:12px; font-weight:600; }}"
                f"QPushButton:hover {{ background: rgba(255,255,255,0.05); color:{CYAN}; }}"
                f"QPushButton:checked {{ background: rgba(79,163,255,0.14); color:{CYAN}; border:1px solid rgba(79,163,255,0.35); }}"
            )
            button.clicked.connect(
                lambda _checked=False, name=menu_name: self._emit_menu(name)
            )
            self._buttons[menu_name] = button
            layout.addWidget(button)
        layout.addStretch(1)
        self.set_active("Dashboard")

    def _emit_menu(self, name: str) -> None:
        self.set_active(name)
        self.menu_requested.emit(name)

    def set_active(self, name: str) -> None:
        for label_text, button in self._buttons.items():
            button.setChecked(label_text == name)


class ReferenceSidebar(QFrame):
    menu_requested = Signal(str)

    def __init__(self) -> None:
        super().__init__()
        self._buttons: dict[str, QPushButton] = {}
        self.setFixedWidth(116)
        self.setStyleSheet(
            f"QFrame {{ background: #12171F; border-right: 1px solid {BORDER}; }}"
        )
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 12, 10, 12)
        layout.setSpacing(8)

        brand = QLabel("\u2736")
        brand.setAlignment(Qt.AlignCenter)
        brand.setFixedHeight(52)
        brand.setStyleSheet(
            "color:#F5F5F5; font-size:20px; font-weight:800; background:#203247; border:1px solid rgba(255,255,255,0.16); border-radius:12px;"
        )
        layout.addWidget(brand)

        for menu_name, icon_text, label_text in [
            ("Dashboard", "\u2302", "DASHBOARD"),
            ("Map", "\u25A6", "MAP"),
            ("Plan", "\u2691", "PLAN"),
            ("Vehicles", "\u25C9", "VEHICLES"),
            ("Analysis", "\u25A3", "ANALYSIS"),
            ("Settings", "\u2699", "SETTINGS"),
        ]:
            button = SidebarMenuButton(icon_text, label_text)
            button.clicked.connect(
                lambda _checked=False, name=menu_name: self._emit_menu(name)
            )
            self._buttons[menu_name] = button
            layout.addWidget(button)

        layout.addStretch(1)

        help_button = SidebarMenuButton("?", "HELP")
        help_button.setMinimumHeight(76)
        help_button.clicked.connect(lambda _checked=False: self._emit_menu("Help"))
        self._buttons["Help"] = help_button
        layout.addWidget(help_button)

        self.set_active("Dashboard")

    def _emit_menu(self, name: str) -> None:
        self.set_active(name)
        self.menu_requested.emit(name)

    def set_active(self, name: str) -> None:
        for label_text, button in self._buttons.items():
            button.setChecked(label_text == name)


class SidebarMenuButton(QPushButton):
    def __init__(self, icon_text: str, label_text: str) -> None:
        super().__init__()
        self.setCursor(Qt.PointingHandCursor)
        self.setCheckable(True)
        self.setMinimumHeight(98)
        self.setText("")
        self.setStyleSheet(
            f"QPushButton {{ text-align:center; color:{TEXT}; background: transparent; border:none; border-radius:12px; padding:8px 4px; }}"
            f"QPushButton:hover {{ background: rgba(255,255,255,0.035); color:{TEXT}; }}"
            f"QPushButton:checked {{ background: rgba(255,255,255,0.05); color:#F4F7FB; border:1px solid rgba(255,255,255,0.06); }}"
        )
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 8, 4, 8)
        layout.setSpacing(6)
        layout.addStretch(1)
        self._icon = QLabel(icon_text)
        self._icon.setAlignment(Qt.AlignCenter)
        self._icon.setFixedHeight(34)
        self._icon.setAttribute(Qt.WA_TransparentForMouseEvents, True)
        self._icon.setStyleSheet(
            f"color:{TEXT}; font-size:29px; font-weight:700; background: transparent; border:none;"
        )
        self._label = QLabel(label_text)
        self._label.setAlignment(Qt.AlignCenter)
        self._label.setWordWrap(True)
        self._label.setAttribute(Qt.WA_TransparentForMouseEvents, True)
        self._label.setStyleSheet(
            f"color:{TEXT}; font-size:11px; font-weight:700; letter-spacing:0.5px; background: transparent; border:none;"
        )
        layout.addWidget(self._icon)
        layout.addWidget(self._label)
        layout.addStretch(1)


class HeaderCounter(QFrame):
    def __init__(self, title: str, color: str) -> None:
        super().__init__()
        self._value = QLabel("0")
        self._value.setAlignment(Qt.AlignCenter)
        self._value.setStyleSheet(f"color:{color}; font-size:18px; font-weight:800;")
        title_label = QLabel(title)
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet(f"color:{TEXT_DIM}; font-size:11px; font-weight:500;")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)
        layout.addWidget(self._value)
        layout.addWidget(title_label)

    def set_value(self, value: int) -> None:
        self._value.setText(str(value))


class HeaderStatus(QFrame):
    def __init__(self, label: str, color: str) -> None:
        super().__init__()
        icon = QLabel("●")
        icon.setStyleSheet(f"color:{color}; font-size:14px;")
        self._value = QLabel("100%")
        self._value.setStyleSheet(f"color:{color}; font-size:16px; font-weight:700;")
        title = QLabel(label)
        title.setStyleSheet(f"color:{TEXT}; font-size:12px;")
        block = QVBoxLayout()
        block.setContentsMargins(0, 0, 0, 0)
        block.setSpacing(0)
        block.addWidget(title)
        block.addWidget(self._value)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)
        layout.addWidget(icon, 0, Qt.AlignTop)
        layout.addLayout(block)

    def set_value(self, text: str, color: str | None = None) -> None:
        self._value.setText(text)
        if color:
            self._value.setStyleSheet(
                f"color:{color}; font-size:16px; font-weight:700;"
            )


class OperatorTopBar(QFrame):
    def __init__(self) -> None:
        super().__init__()
        self.setFixedHeight(68)
        self.setStyleSheet(
            f"QFrame {{ background: #12171F; border-bottom: 1px solid {BORDER}; }}"
        )
        layout = QHBoxLayout(self)
        layout.setContentsMargins(16, 8, 18, 8)
        layout.setSpacing(18)

        title = QLabel("Fleet View")
        title.setStyleSheet(f"color:{TEXT}; font-size:17px; font-weight:600;")
        caret = QLabel("▾")
        caret.setStyleSheet(f"color:{TEXT_DIM}; font-size:12px;")
        title_row = QHBoxLayout()
        title_row.setSpacing(8)
        title_row.addWidget(title)
        title_row.addWidget(caret)
        title_row.addStretch(1)
        layout.addLayout(title_row, 0)

        self.active = HeaderCounter("Active", SUCCESS)
        self.warning = HeaderCounter("Warning", WARN)
        self.critical = HeaderCounter("Critical", DANGER)
        self.total = HeaderCounter("Total", TEXT)
        for item in (self.active, self.warning, self.critical, self.total):
            layout.addWidget(item)

        layout.addStretch(1)
        self.telemetry = HeaderStatus("Telemetry", SUCCESS)
        self.link = HeaderStatus("RC Link", SUCCESS)
        self.backend = HeaderStatus("Backend", SUCCESS)
        layout.addWidget(self.telemetry)
        layout.addWidget(self.link)
        layout.addWidget(self.backend)

        for attr_name, label in [
            ("commands_button", "COMMANDS"),
            ("operations_button", "OPS"),
            ("sensors_button", "SENSORS"),
            ("analytics_button", "ANALYTICS"),
        ]:
            button = QPushButton(label)
            button.setCursor(Qt.PointingHandCursor)
            button.setFixedHeight(34)
            button.setStyleSheet(
                f"QPushButton {{ background: rgba(255,255,255,0.03); color:{TEXT}; border:1px solid {BORDER};"
                " border-radius:8px; padding:0 12px; font-size:11px; font-weight:700; letter-spacing:0.8px; }"
                f"QPushButton:hover {{ border-color:{CYAN}; color:{CYAN}; background: rgba(79,163,255,0.08); }}"
            )
            setattr(self, attr_name, button)
            layout.addWidget(button)

        self.clock = QLabel("--:--:--\nUTC")
        self.clock.setAlignment(Qt.AlignCenter)
        self.clock.setStyleSheet(f"color:{TEXT}; font-size:13px; font-weight:500;")
        layout.addWidget(self.clock)

        gear = QLabel("⚙")
        gear.setAlignment(Qt.AlignCenter)
        gear.setFixedWidth(34)
        gear.setStyleSheet(f"color:{TEXT}; font-size:26px;")
        layout.addWidget(gear)

    def set_counts(self, active: int, warning: int, critical: int, total: int) -> None:
        self.active.set_value(active)
        self.warning.set_value(warning)
        self.critical.set_value(critical)
        self.total.set_value(total)

    def set_clock(self, text: str) -> None:
        clock_value = text.split()[-1] if text else "--:--:--"
        self.clock.setText(f"{clock_value}\nUTC")


class OperatorTable(QTableWidget):
    HEADERS = ["ID", "NAME", "STATUS", "BATTERY", "LINK", "ALTITUDE"]

    def __init__(self) -> None:
        super().__init__(0, len(self.HEADERS))
        self.setHorizontalHeaderLabels(self.HEADERS)
        self.verticalHeader().setVisible(False)
        self.setShowGrid(False)
        self.setAlternatingRowColors(False)
        self.setEditTriggers(QTableWidget.NoEditTriggers)
        self.setSelectionBehavior(QTableWidget.SelectRows)
        self.setSelectionMode(QTableWidget.SingleSelection)
        self.setFrameShape(QFrame.NoFrame)
        self.horizontalHeader().setStretchLastSection(True)
        for col in range(len(self.HEADERS)):
            self.horizontalHeader().setSectionResizeMode(
                col, QHeaderView.ResizeMode.Stretch
            )
        self.verticalHeader().setDefaultSectionSize(34)
        self.setStyleSheet(
            f"QTableWidget {{ background: transparent; color:{TEXT}; border:none; font-size:12px; }}"
            f"QHeaderView::section {{ background: transparent; color:{TEXT_DIM}; border:none; border-bottom:1px solid {BORDER};"
            " padding:7px 6px; font-size:10px; font-weight:700; }}"
            f"QTableWidget::item {{ border-bottom:1px solid rgba(255,255,255,0.05); padding:5px 6px; }}"
            f"QTableWidget::item:selected {{ background: rgba(255,255,255,0.04); }}"
        )

    def ingest(self, states: list[DroneState]) -> None:
        self.setRowCount(len(states))
        ordered = sorted(states, key=lambda state: state.drone_id)
        for row, state in enumerate(ordered):
            battery_color = (
                SUCCESS
                if state.battery_pct >= 55
                else WARN if state.battery_pct >= 30 else DANGER
            )
            status_text = (
                "Active"
                if state.reachable and not state.stale
                else "Warning" if state.reachable else "Critical"
            )
            status_color = (
                SUCCESS
                if status_text == "Active"
                else WARN if status_text == "Warning" else DANGER
            )
            link_bars = signal_bars(state.rssi_dbm, state.reachable)
            values = [
                str(state.drone_id),
                f"Node-{state.drone_id}",
                status_text,
                f"{state.battery_pct:.0f}%",
                link_bars,
                f"{state.position[2]:.0f} m",
            ]
            colors = [SUCCESS, TEXT, status_color, battery_color, status_color, TEXT]
            for col, value in enumerate(values):
                item = self.item(row, col)
                if item is None:
                    item = QTableWidgetItem()
                    item.setTextAlignment(Qt.AlignCenter)
                    self.setItem(row, col, item)
                item.setText(value)
                item.setForeground(QColor(colors[col]))


class HistoryPlot(pg.PlotWidget):
    def __init__(self, series_mode: str = "multi", area_fill: bool = False) -> None:
        super().__init__()
        self._series_mode = series_mode
        self._area_fill = area_fill
        self._window = 120
        self._history: dict[int, deque[float]] = {}
        self._curves: dict[int, Any] = {}
        self._single_history = deque(maxlen=self._window)
        self._single_curve = None
        self._base_reference = None
        self._fill_between = None
        self._t0 = time.time()
        self._single_line_color = CYAN

        self.setBackground(PANEL_BG)
        self.showGrid(x=False, y=True, alpha=0.14)
        self.setMenuEnabled(False)
        self.setMouseEnabled(x=False, y=False)
        self.hideButtons()
        self.getAxis("left").setPen(pg.mkPen(BORDER))
        self.getAxis("bottom").setPen(pg.mkPen(BORDER))
        self.getAxis("left").setTextPen(pg.mkPen(TEXT_DIM))
        self.getAxis("bottom").setTextPen(pg.mkPen(TEXT_DIM))
        self.getAxis("left").setStyle(tickTextOffset=8)
        self.getAxis("bottom").setStyle(tickTextOffset=6)
        self.getAxis("bottom").setTicks([])
        self.setContentsMargins(0, 0, 0, 0)
        self.plotItem.setContentsMargins(0, 4, 0, 0)
        self.plotItem.hideAxis("bottom")
        if self._series_mode == "single":
            self.getAxis("left").setWidth(34)

    def set_y_range(self, min_y: float, max_y: float) -> None:
        self.setYRange(min_y, max_y)

    def set_reference_single_style(self, color: str = CYAN) -> None:
        self._single_line_color = color
        self.showGrid(x=False, y=True, alpha=0.12)
        self.getAxis("left").setStyle(showValues=False, tickLength=0)
        self.getAxis("left").setPen(pg.mkPen((0, 0, 0, 0)))
        self.plotItem.getViewBox().setBorder(None)

    def update_multi(self, series_map: dict[int, float]) -> None:
        now = time.time() - self._t0
        for drone_id, value in sorted(series_map.items()):
            if drone_id not in self._history:
                self._history[drone_id] = deque(maxlen=self._window)
                color = SERIES_COLORS[(len(self._history) - 1) % len(SERIES_COLORS)]
                curve = self.plot(pen=pg.mkPen(color, width=2))
                self._curves[drone_id] = curve
            self._history[drone_id].append(value)
            arr = np.array(self._history[drone_id], dtype=float)
            xs = np.linspace(max(0.0, now - len(arr)), now, len(arr))
            self._curves[drone_id].setData(xs, arr)

    def update_single(self, value: float, color: str = CYAN) -> None:
        now = time.time() - self._t0
        self._single_history.append(value)
        arr = np.array(self._single_history, dtype=float)
        xs = np.linspace(max(0.0, now - len(arr)), now, len(arr))
        line_color = self._single_line_color or color
        if self._single_curve is None:
            pen = pg.mkPen(line_color, width=2.4)
            self._single_curve = self.plot(pen=pen)
            if self._area_fill:
                self._base_reference = self.plot(
                    xs,
                    np.zeros_like(arr),
                    pen=pg.mkPen((0, 0, 0, 0)),
                )
                fill = pg.FillBetweenItem(
                    self._single_curve,
                    self._base_reference,
                    brush=pg.mkBrush(rgba(line_color, 0.22)),
                )
                self.addItem(fill)
                self._fill_between = fill
        self._single_curve.setData(xs, arr)
        if self._base_reference is not None:
            self._base_reference.setData(xs, np.zeros_like(arr))


class PanelFrame(QFrame):
    def __init__(self, title: str) -> None:
        super().__init__()
        style_panel(self)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 10, 14, 12)
        layout.setSpacing(8)
        layout.addWidget(panel_title(title))
        self.content_layout = layout


class MapPanel(PanelFrame):
    def __init__(self, screenshot_path: Path) -> None:
        super().__init__("MAP")
        self._snapshot: DashboardSnapshot | None = None
        self._bg = self._load_crop(screenshot_path, QRectF(84, 75, 478, 349))
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.content_layout.addStretch(1)

    def _load_crop(self, path: Path, rect: QRectF) -> QPixmap | None:
        if not path.exists():
            return None
        image = QImage(str(path))
        if image.isNull():
            return None
        crop = image.copy(
            int(rect.x()), int(rect.y()), int(rect.width()), int(rect.height())
        )
        return QPixmap.fromImage(crop)

    def ingest(self, snapshot: DashboardSnapshot) -> None:
        self._snapshot = snapshot
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        content_rect = self.rect().adjusted(10, 38, -10, -10)
        if self._bg is not None:
            painter.drawPixmap(content_rect, self._bg)
        else:
            painter.fillRect(content_rect, rgba("#1A2A1A", 1.0))
        painter.fillRect(content_rect, rgba("#0B1114", 0.16))

        button_y = 8
        right = self.rect().right() - 12
        for idx, text in enumerate(["3D", "2D"]):
            w = 42
            x = right - (idx + 1) * (w + 6)
            btn = QRectF(x, button_y, w, 28)
            painter.setPen(QPen(rgba("#808A98", 0.5), 1))
            painter.setBrush(rgba("#252B34", 0.84))
            painter.drawRoundedRect(btn, 4, 4)
            painter.setPen(QColor(TEXT))
            painter.drawText(btn, Qt.AlignCenter, text)

        if self._snapshot is None or not self._snapshot.states:
            return

        states = sorted(self._snapshot.states, key=lambda state: state.drone_id)
        xs = [state.position[0] for state in states]
        ys = [state.position[1] for state in states]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        span_x = max(8.0, max_x - min_x)
        span_y = max(8.0, max_y - min_y)

        def to_point(state: DroneState) -> QPointF:
            px = (state.position[0] - min_x) / span_x
            py = (state.position[1] - min_y) / span_y
            x = content_rect.left() + 34 + px * (content_rect.width() - 68)
            y = content_rect.bottom() - 34 - py * (content_rect.height() - 68)
            return QPointF(x, y)

        leader = next((state for state in states if state.role == "LEADER"), states[0])
        leader_point = to_point(leader)
        for state in states:
            point = to_point(state)
            pen_color = SUCCESS if state.reachable else DANGER
            if state.drone_id != leader.drone_id:
                painter.setPen(QPen(rgba(pen_color, 0.7), 2, Qt.DashLine))
                painter.drawLine(leader_point, point)
            label_offset = QPointF(-10, -14)
            painter.setPen(QColor(TEXT))
            painter.setFont(QFont("Segoe UI", 10, QFont.Bold))
            painter.drawText(
                QRectF(point.x() - 12, point.y() - 28, 24, 18),
                Qt.AlignCenter,
                str(state.drone_id),
            )
            color = (
                SUCCESS
                if state.reachable and state.battery_pct >= 30
                else WARN if state.reachable else DANGER
            )
            painter.setPen(QPen(rgba("#0E141B", 1.0), 2))
            painter.setBrush(QColor(color))
            painter.drawEllipse(point, 11, 15)
            painter.drawEllipse(QPointF(point.x(), point.y() + 16), 2.5, 2.5)

        painter.setPen(QPen(QColor(TEXT), 2))
        painter.drawLine(
            content_rect.left() + 18,
            content_rect.bottom() - 16,
            content_rect.left() + 78,
            content_rect.bottom() - 16,
        )
        painter.drawLine(
            content_rect.left() + 18,
            content_rect.bottom() - 20,
            content_rect.left() + 18,
            content_rect.bottom() - 12,
        )
        painter.drawLine(
            content_rect.left() + 78,
            content_rect.bottom() - 20,
            content_rect.left() + 78,
            content_rect.bottom() - 12,
        )
        painter.setPen(QColor(TEXT))
        painter.drawText(
            QRectF(content_rect.left() + 18, content_rect.bottom() - 34, 70, 14),
            "500 m",
        )


class AttitudeIndicator(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self._roll = 0.0
        self._pitch = 0.0
        self.setMinimumSize(150, 110)

    def set_attitude(self, roll_deg: float, pitch_deg: float) -> None:
        self._roll = roll_deg
        self._pitch = pitch_deg
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        rect = self.rect().adjusted(8, 8, -8, -8)
        size = min(rect.width(), rect.height())
        circle = QRectF(rect.left(), rect.top(), size, size)
        center = circle.center()
        radius = circle.width() / 2

        painter.save()
        painter.setClipPath(QPainterPath())
        clip = QPainterPath()
        clip.addEllipse(circle)
        painter.setClipPath(clip)
        painter.translate(center)
        painter.rotate(-self._roll)
        horizon_shift = self._pitch * 1.8
        painter.fillRect(
            QRectF(-radius, -radius * 2 + horizon_shift, radius * 2, radius * 2),
            QColor("#507BC7"),
        )
        painter.fillRect(
            QRectF(-radius, horizon_shift, radius * 2, radius * 2), QColor("#886735")
        )
        painter.setPen(QPen(QColor("#EAD96F"), 3))
        painter.drawLine(
            QPointF(-radius, horizon_shift), QPointF(radius, horizon_shift)
        )
        painter.setPen(QPen(rgba(TEXT, 0.55), 1))
        for value in range(-30, 40, 10):
            y = horizon_shift - value * 2.2
            width = 44 if value % 20 == 0 else 24
            painter.drawLine(QPointF(-width / 2, y), QPointF(width / 2, y))
            if value != 0:
                painter.setPen(QColor(TEXT))
                painter.setFont(QFont("Segoe UI", 7, QFont.Medium))
                painter.drawText(
                    QRectF(-width / 2 - 20, y - 8, 16, 14),
                    Qt.AlignRight | Qt.AlignVCenter,
                    f"{abs(value)}",
                )
                painter.drawText(
                    QRectF(width / 2 + 4, y - 8, 16, 14),
                    Qt.AlignLeft | Qt.AlignVCenter,
                    f"{abs(value)}",
                )
                painter.setPen(QPen(rgba(TEXT, 0.55), 1))
        painter.restore()

        painter.setPen(QPen(rgba(TEXT, 0.75), 2))
        painter.drawEllipse(circle)
        painter.setPen(QPen(rgba(TEXT, 0.7), 1))
        for tick in range(-4, 5):
            if tick == 0:
                continue
            y = center.y() + tick * 14
            left_len = 14 if tick % 2 == 0 else 8
            painter.drawLine(
                QPointF(circle.left() - 2, y), QPointF(circle.left() + left_len, y)
            )
            painter.drawLine(
                QPointF(circle.right() - left_len, y), QPointF(circle.right() + 2, y)
            )
        painter.setPen(QPen(QColor(TEXT), 3))
        painter.drawLine(
            QPointF(center.x() - 40, center.y()), QPointF(center.x() - 8, center.y())
        )
        painter.drawLine(
            QPointF(center.x() + 8, center.y()), QPointF(center.x() + 40, center.y())
        )
        painter.drawLine(
            QPointF(center.x() - 8, center.y()), QPointF(center.x(), center.y() + 9)
        )
        painter.drawLine(
            QPointF(center.x() + 8, center.y()), QPointF(center.x(), center.y() + 9)
        )
        painter.setPen(QPen(QColor("#EAD96F"), 2))
        painter.drawLine(
            QPointF(center.x() - 12, center.y() + 10),
            QPointF(center.x(), center.y() + 22),
        )
        painter.drawLine(
            QPointF(center.x() + 12, center.y() + 10),
            QPointF(center.x(), center.y() + 22),
        )


class AttitudePanel(PanelFrame):
    def __init__(self) -> None:
        super().__init__("ATTITUDE - NODE 1")
        self._title_label = self.findChildren(QLabel)[0]
        row = QHBoxLayout()
        row.setSpacing(10)
        self._indicator = AttitudeIndicator()
        row.addWidget(self._indicator, 1)
        metrics = QVBoxLayout()
        metrics.setSpacing(10)
        self._roll = QLabel("--")
        self._pitch = QLabel("--")
        self._yaw = QLabel("--")
        self._alt = QLabel("--")
        for title, widget in [
            ("Roll", self._roll),
            ("Pitch", self._pitch),
            ("Yaw", self._yaw),
            ("Alt (AGL)", self._alt),
        ]:
            line = QHBoxLayout()
            label = QLabel(title)
            label.setStyleSheet(f"color:{TEXT}; font-size:12px;")
            widget.setStyleSheet(f"color:{TEXT}; font-size:15px; font-weight:500;")
            widget.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            line.addWidget(label)
            line.addStretch(1)
            line.addWidget(widget)
            metrics.addLayout(line)
        metrics.addStretch(1)
        row.addLayout(metrics, 1)
        self.content_layout.addLayout(row)

    def ingest(self, state: DroneState | None) -> None:
        if state is None:
            return
        self._title_label.setText(f"ATTITUDE - NODE {state.drone_id}")
        roll = math.degrees(state.attitude_rpy[0])
        pitch = math.degrees(state.attitude_rpy[1])
        yaw = math.degrees(state.attitude_rpy[2])
        self._indicator.set_attitude(roll, pitch)
        self._roll.setText(f"{roll:.1f}°")
        self._pitch.setText(f"{pitch:.1f}°")
        self._yaw.setText(f"{yaw:.1f}°")
        self._alt.setText(f"{state.position[2]:.1f} m")


class LineChartPanel(PanelFrame):
    def __init__(
        self, title: str, multi_series: bool = True, area_fill: bool = False
    ) -> None:
        super().__init__(title)
        self._chart = HistoryPlot(
            "multi" if multi_series else "single", area_fill=area_fill
        )
        self._chart.setMinimumHeight(150)
        self._legend = QLabel("")
        self._legend.setWordWrap(True)
        self._legend.setStyleSheet(f"color:{TEXT}; font-size:11px;")
        self.content_layout.addWidget(self._chart, 1)
        self.content_layout.addWidget(self._legend)

    def set_y_range(self, minimum: float, maximum: float) -> None:
        self._chart.set_y_range(minimum, maximum)

    def set_reference_single_style(self, color: str = CYAN) -> None:
        self._chart.set_reference_single_style(color)

    def update_multi(self, values: dict[int, float]) -> None:
        self._chart.update_multi(values)
        legend = []
        for idx, drone_id in enumerate(sorted(values)):
            color = SERIES_COLORS[idx % len(SERIES_COLORS)]
            legend.append(f"<span style='color:{color}'>■</span> Node-{drone_id}")
        self._legend.setText("   ".join(legend))

    def update_single(self, value: float, color: str = CYAN) -> None:
        self._chart.update_single(value, color=color)
        self._legend.setText("")


class DataRatePanel(PanelFrame):
    def __init__(self) -> None:
        super().__init__("DATA RATE (kbps)")
        self.content_layout.takeAt(0)

        header_row = QHBoxLayout()
        header_row.setContentsMargins(0, 0, 0, 0)
        header_row.setSpacing(12)
        title = panel_title("DATA RATE (kbps)")
        header_row.addWidget(title)
        header_row.addStretch(1)
        self._download = QLabel("\u2193 0 kbps")
        self._upload = QLabel("\u2191 0 kbps")
        self._download.setStyleSheet(f"color:{CYAN}; font-size:12px; font-weight:700;")
        self._upload.setStyleSheet(f"color:{PURPLE}; font-size:12px; font-weight:700;")
        header_row.addWidget(self._download)
        header_row.addWidget(self._upload)
        self.content_layout.addLayout(header_row)

        self._chart = HistoryPlot("single", area_fill=True)
        self._chart.setMinimumHeight(150)
        self._chart.set_y_range(0, 1250)
        self._chart.set_reference_single_style(CYAN)
        self.content_layout.addWidget(self._chart, 1)

    def ingest(self, snapshot: DashboardSnapshot, state: DroneState | None) -> None:
        downlink, uplink = estimate_transfer_rates(snapshot, state)
        self._download.setText(f"\u2193 {downlink:.0f} kbps")
        self._upload.setText(f"\u2191 {uplink:.0f} kbps")
        self._chart.update_single(snapshot.avg_mesh_bandwidth_kbps, color=CYAN)


class SystemMessagesPanel(PanelFrame):
    def __init__(self) -> None:
        super().__init__("SYSTEM MESSAGES")
        self._rows: list[tuple[QLabel, QLabel, QLabel]] = []
        for _ in range(5):
            row = QHBoxLayout()
            row.setContentsMargins(0, 1, 0, 1)
            row.setSpacing(8)
            time_label = QLabel("--:--:--")
            icon_label = QLabel("i")
            message_label = QLabel("--")
            time_label.setFixedWidth(62)
            time_label.setStyleSheet(
                f"color:{TEXT_DIM}; font-size:11px; font-weight:600;"
            )
            icon_label.setFixedWidth(20)
            icon_label.setFixedHeight(20)
            icon_label.setAlignment(Qt.AlignCenter)
            icon_label.setStyleSheet(
                f"color:{SUCCESS}; font-size:11px; font-weight:800; background:rgba(81, 219, 161, 0.12); border-radius:10px;"
            )
            message_label.setStyleSheet(f"color:{TEXT}; font-size:12px;")
            message_label.setWordWrap(False)
            row.addWidget(time_label)
            row.addWidget(icon_label)
            row.addWidget(message_label, 1)
            self.content_layout.addLayout(row)
            self._rows.append((time_label, icon_label, message_label))
        dots = QLabel("...")
        dots.setAlignment(Qt.AlignCenter)
        dots.setStyleSheet(f"color:{TEXT_DIM}; font-size:18px; letter-spacing:4px;")
        self.content_layout.addWidget(dots)

    def ingest(self, snapshot: DashboardSnapshot) -> None:
        messages = list(snapshot.events[-5:])
        while len(messages) < 5:
            messages.insert(0, {})
        for row, event in zip(self._rows, messages):
            time_label, icon_label, message_label = row
            msg = (
                str(event.get("message", "No recent telemetry event"))
                if event
                else "No recent telemetry event"
            )
            timestamp = str(event.get("timestamp", "")) if event else ""
            event_type = str(event.get("type", "info")).lower() if event else "info"
            lowered = msg.lower()
            time_label.setText(timestamp[11:19] if len(timestamp) >= 19 else "--:--:--")
            if (
                event_type in {"error", "critical"}
                or "reject" in lowered
                or "critical" in lowered
            ):
                icon_label.setText("x")
                icon_label.setStyleSheet(
                    f"color:{DANGER}; font-size:11px; font-weight:800; background:rgba(237, 98, 98, 0.12); border-radius:10px;"
                )
                message_label.setStyleSheet(f"color:{DANGER}; font-size:12px;")
            elif (
                event_type in {"warning", "warn"}
                or "approval" in lowered
                or "warning" in lowered
            ):
                icon_label.setText("!")
                icon_label.setStyleSheet(
                    f"color:{WARN}; font-size:11px; font-weight:800; background:rgba(246, 195, 67, 0.14); border-radius:10px;"
                )
                message_label.setStyleSheet(f"color:{WARN}; font-size:12px;")
            else:
                icon_label.setText("i")
                icon_label.setStyleSheet(
                    f"color:{SUCCESS}; font-size:11px; font-weight:800; background:rgba(81, 219, 161, 0.12); border-radius:10px;"
                )
                message_label.setStyleSheet(f"color:{TEXT}; font-size:12px;")
            message_label.setText(msg)


class CpuTemperaturePanel(PanelFrame):
    def __init__(self) -> None:
        super().__init__("CPU & TEMPERATURE")
        row = QHBoxLayout()
        row.setSpacing(10)
        left_box = QVBoxLayout()
        left_box.setSpacing(6)
        left_title = QLabel("CPU Usage")
        left_title.setStyleSheet(f"color:{TEXT}; font-size:12px;")
        self._cpu_plot = HistoryPlot("single", area_fill=False)
        self._cpu_plot.setYRange(0, 100)
        left_box.addWidget(left_title)
        left_box.addWidget(self._cpu_plot, 1)
        right_box = QVBoxLayout()
        right_box.setSpacing(6)
        right_title = QLabel("Temperature")
        right_title.setStyleSheet(f"color:{TEXT}; font-size:12px;")
        self._temp_plot = HistoryPlot("single", area_fill=False)
        self._temp_plot.setYRange(0, 100)
        right_box.addWidget(right_title)
        right_box.addWidget(self._temp_plot, 1)
        row.addLayout(left_box, 1)
        row.addLayout(right_box, 1)
        self.content_layout.addLayout(row, 1)

    def ingest(self, snapshot: DashboardSnapshot) -> None:
        self._cpu_plot.update_single(snapshot.gpu_load_pct, color=CYAN)
        self._temp_plot.update_single(snapshot.cpu_temp_c, color=DANGER)


class MissionProgressPanel(PanelFrame):
    def __init__(self, screenshot_path: Path) -> None:
        super().__init__("MISSION PROGRESS")
        self._bg = self._load_crop(screenshot_path, QRectF(1270, 750, 248, 259))
        self._progress = QLabel("0%")
        self._progress.setStyleSheet(f"color:{TEXT}; font-size:17px; font-weight:700;")
        self._mission_name = QLabel("No mission")
        self._mission_name.setStyleSheet(
            f"color:{TEXT}; font-size:12px; font-weight:600;"
        )
        self._bar = QProgressBar()
        self._bar.setRange(0, 100)
        self._bar.setTextVisible(False)
        self._bar.setFixedHeight(12)
        self._bar.setStyleSheet(
            f"QProgressBar {{ background:{BORDER}; border:none; border-radius:3px; }}"
            f"QProgressBar::chunk {{ background:{SUCCESS}; border-radius:3px; }}"
        )
        self._rows: dict[str, QLabel] = {}
        self._first_position: dict[int, tuple[float, float, float]] = {}

        top = QHBoxLayout()
        top.addStretch(1)
        top.addWidget(self._progress)
        self.content_layout.addLayout(top)

        body = QHBoxLayout()
        body.setSpacing(12)
        left = QVBoxLayout()
        left.setSpacing(8)
        left.addWidget(self._mission_name)
        left.addWidget(self._bar)
        for title in [
            "Distance Covered",
            "Est. Time Remaining",
            "Waypoints",
            "Current Waypoint",
        ]:
            row = QHBoxLayout()
            label = QLabel(title)
            value = QLabel("--")
            label.setStyleSheet(f"color:{TEXT_DIM}; font-size:11px;")
            value.setStyleSheet(f"color:{TEXT}; font-size:12px;")
            row.addWidget(label)
            row.addStretch(1)
            row.addWidget(value)
            left.addLayout(row)
            self._rows[title] = value
        left.addStretch(1)
        body.addLayout(left, 1)
        self._mini_map = QLabel()
        self._mini_map.setMinimumSize(160, 160)
        self._mini_map.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._mini_map.setStyleSheet(
            f"background:{PANEL_ALT}; border:1px solid rgba(255,255,255,0.12); border-radius:8px;"
        )
        body.addWidget(self._mini_map, 1)
        self.content_layout.addLayout(body, 1)

    def _load_crop(self, path: Path, rect: QRectF) -> QPixmap | None:
        if not path.exists():
            return None
        image = QImage(str(path))
        if image.isNull():
            return None
        crop = image.copy(
            int(rect.x()), int(rect.y()), int(rect.width()), int(rect.height())
        )
        return QPixmap.fromImage(crop)

    def ingest(self, snapshot: DashboardSnapshot, state: DroneState | None) -> None:
        if state is None:
            return
        self._first_position.setdefault(state.drone_id, state.position)
        mission = snapshot.missions[0] if snapshot.missions else {}
        mission_name = str(mission.get("name", "Survey Area"))
        target = mission.get(
            "target", [state.position[0], state.position[1], state.position[2]]
        )
        target_pos = (
            float(target[0]),
            float(target[1]),
            float(target[2]),
        )
        origin = self._first_position[state.drone_id]
        total_distance = max(0.1, math.dist(origin, target_pos))
        remaining = math.dist(state.position, target_pos)
        covered = max(0.0, total_distance - remaining)
        ratio = clamp(covered / total_distance, 0.0, 1.0)
        speed = max(0.1, math.sqrt(sum(v * v for v in state.velocity)))
        eta = remaining / speed
        waypoints = max(1, state.planned_waypoint_count or 1)
        current_waypoint = max(1, min(waypoints, int(round(ratio * waypoints))))

        self._mission_name.setText(mission_name)
        self._progress.setText(f"{int(ratio * 100):d}%")
        self._bar.setValue(int(ratio * 100))
        self._rows["Distance Covered"].setText(
            f"{covered:.1f} km" if covered > 1 else f"{covered * 1000:.0f} m"
        )
        self._rows["Est. Time Remaining"].setText(human_hms(eta))
        self._rows["Waypoints"].setText(f"{current_waypoint} / {waypoints}")
        self._rows["Current Waypoint"].setText(str(current_waypoint))

        self._mini_map.setPixmap(self._compose_minimap(state, target_pos))

    def _compose_minimap(
        self, state: DroneState, target_pos: tuple[float, float, float]
    ) -> QPixmap:
        if self._bg is None:
            pix = QPixmap(256, 256)
            pix.fill(rgba("#1A1F27", 1.0))
        else:
            pix = self._bg.scaled(
                256, 256, Qt.IgnoreAspectRatio, Qt.SmoothTransformation
            )
        painter = QPainter(pix)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(pix.rect(), rgba("#09120D", 0.08))
        path = QPainterPath()
        path.moveTo(22, 194)
        path.lineTo(22, 34)
        path.lineTo(228, 34)
        path.lineTo(228, 222)
        path.lineTo(58, 222)
        painter.setPen(QPen(QColor(SUCCESS), 2))
        painter.drawPath(path)
        painter.setBrush(QColor(SUCCESS))
        painter.drawEllipse(QPointF(26, 194), 8, 8)
        painter.setBrush(QColor(TEXT))
        painter.drawPolygon(
            [
                QPointF(74, 210),
                QPointF(88, 210),
                QPointF(80, 226),
            ]
        )
        painter.end()
        return pix


class OperatorConsoleView(QWidget):
    def __init__(self, docs_asset_dir: Path) -> None:
        super().__init__()
        screenshot = docs_asset_dir / "dashboard_operator_console.png"

        self.topbar = OperatorTopBar()
        self.map_panel = MapPanel(screenshot)
        self.vehicle_panel = PanelFrame("VEHICLE STATUS")
        self.vehicle_table = OperatorTable()
        self.vehicle_panel.content_layout.addWidget(self.vehicle_table, 1)
        self.commands_button = self.topbar.commands_button
        self.operations_button = self.topbar.operations_button
        self.sensors_button = self.topbar.sensors_button
        self.analytics_button = self.topbar.analytics_button
        for button in (
            self.commands_button,
            self.operations_button,
            self.sensors_button,
            self.analytics_button,
        ):
            button.setVisible(False)

        self.battery_panel = LineChartPanel("BATTERY STATUS", multi_series=True)
        self.battery_panel.set_y_range(0, 100)
        self.attitude_panel = AttitudePanel()
        self.signal_panel = LineChartPanel("SIGNAL STRENGTH", multi_series=True)
        self.signal_panel.set_y_range(0, 100)
        self.rate_panel = DataRatePanel()
        self.messages_panel = SystemMessagesPanel()
        self.cpu_temp_panel = CpuTemperaturePanel()
        self.mission_panel = MissionProgressPanel(screenshot)

        root = QHBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)
        self.sidebar = ReferenceSidebar()
        self.sidebar_menu_requested = self.sidebar.menu_requested
        root.addWidget(self.sidebar)

        content = QWidget()
        content_layout = QVBoxLayout(content)
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.setSpacing(10)
        content_layout.addWidget(self.topbar)

        grid = QGridLayout()
        grid.setContentsMargins(10, 0, 10, 10)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(10)
        grid.addWidget(self.map_panel, 0, 0)
        grid.addWidget(self.vehicle_panel, 0, 1)
        grid.addWidget(self.battery_panel, 0, 2)
        grid.addWidget(self.attitude_panel, 1, 0)
        grid.addWidget(self.signal_panel, 1, 1)
        grid.addWidget(self.rate_panel, 1, 2)
        grid.addWidget(self.messages_panel, 2, 0)
        grid.addWidget(self.cpu_temp_panel, 2, 1)
        grid.addWidget(self.mission_panel, 2, 2)
        grid.setColumnStretch(0, 36)
        grid.setColumnStretch(1, 34)
        grid.setColumnStretch(2, 38)
        grid.setRowStretch(0, 41)
        grid.setRowStretch(1, 32)
        grid.setRowStretch(2, 31)
        content_layout.addLayout(grid, 1)
        root.addWidget(content, 1)

    def set_clock(self, text: str) -> None:
        self.topbar.set_clock(text)

    def set_sidebar_active(self, name: str) -> None:
        self.sidebar.set_active(name)

    def ingest(
        self, snapshot: DashboardSnapshot, selected_id: int | None, operator_role: str
    ) -> None:
        active = sum(
            1 for state in snapshot.states if state.reachable and not state.stale
        )
        warning = sum(
            1 for state in snapshot.states if state.reachable and state.battery_pct < 55
        )
        critical = sum(1 for state in snapshot.states if not state.reachable)
        self.topbar.set_counts(active, warning, critical, len(snapshot.states))
        telemetry_pct = 100 - min(100, int(snapshot.packet_loss_pct * 6))
        self.topbar.telemetry.set_value(
            f"{telemetry_pct}%", SUCCESS if telemetry_pct >= 80 else WARN
        )
        self.topbar.link.set_value(
            f"{int(clamp((100 - snapshot.avg_latency_ms * 2.5), 0, 100))}%",
            (
                SUCCESS
                if snapshot.avg_latency_ms < 12
                else WARN if snapshot.avg_latency_ms < 25 else DANGER
            ),
        )
        backend_ok = snapshot.backend_mode.lower() not in {"unavailable", "offline"}
        self.topbar.backend.set_value(
            "100%" if backend_ok else "0%", SUCCESS if backend_ok else DANGER
        )

        self.map_panel.ingest(snapshot)
        self.vehicle_table.ingest(snapshot.states)
        self.battery_panel.update_multi(
            {state.drone_id: state.battery_pct for state in snapshot.states[:9]}
        )
        signal_values = {
            state.drone_id: clamp((state.rssi_dbm + 100.0) * 2.0, 0.0, 100.0)
            for state in snapshot.states[:9]
        }
        self.signal_panel.update_multi(signal_values)
        current_state = selected_or_first(snapshot, selected_id)
        self.rate_panel.ingest(snapshot, current_state)
        self.messages_panel.ingest(snapshot)
        self.cpu_temp_panel.ingest(snapshot)
        self.attitude_panel.ingest(current_state)
        self.mission_panel.ingest(snapshot, current_state)
