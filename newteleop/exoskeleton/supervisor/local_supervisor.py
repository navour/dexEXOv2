#!/usr/bin/env python3
"""dexEXO 树莓派本地 PyQt5 上位机。

界面与 DualHandController 处于同一进程，确保 Dynamixel
TTL 总线仍只有一个所有者。所有慢速硬件命令都在工作
线程中执行，Qt 主线程只负责显示缓存快照和安全门控。
"""

from __future__ import annotations

from collections import deque
import html
import math
import os
from pathlib import Path
import signal
import sys
import threading
import time
from typing import Callable

from PyQt5.QtCore import QObject, QSize, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QCloseEvent, QFont, QKeySequence
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QApplication,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMainWindow,
    QMessageBox,
    QProgressBar,
    QPushButton,
    QScrollArea,
    QShortcut,
    QSizePolicy,
    QSplitter,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
import pyqtgraph as pg


SUPERVISOR_DIR = Path(__file__).resolve().parent
EXOSKELETON_DIR = SUPERVISOR_DIR.parent
FORCE_CONTROL_DIR = EXOSKELETON_DIR / "force_control"
sys.path.insert(0, str(FORCE_CONTROL_DIR))
sys.path.insert(0, str(SUPERVISOR_DIR))

from dual_hand_force_test import DualHandController, parse_args  # noqa: E402
from supervisor_common import (  # noqa: E402
    BleBrokerAlreadyRunningError,
    BleBrokerManager,
    FINGER_NAMES,
    SIDES,
    SIDE_NAMES,
    display_value,
    format_number,
    format_status,
)


COLORS = {
    "bg": "#06101A",
    "panel": "#0D1A27",
    "panel_2": "#102131",
    "panel_3": "#13283A",
    "border": "#294255",
    "border_soft": "#1A3244",
    "text": "#E6F0F6",
    "muted": "#8299A9",
    "cyan": "#36CBE1",
    "green": "#44D591",
    "amber": "#F5B541",
    "red": "#F25C5C",
    "blue": "#5E8BFF",
}

FINGER_COLORS = ["#36CBE1", "#5E8BFF", "#A879FF", "#F5B541", "#44D591"]
STATE_COLORS = {
    "FREE": COLORS["green"],
    "FORCE_ENTRY": COLORS["amber"],
    "LOCKED": COLORS["cyan"],
    "RELEASE": "#A879FF",
    "RETURN_SETTLE": COLORS["blue"],
    "FAULT": COLORS["red"],
    "STOP": COLORS["muted"],
}


def translucent(hex_color: str, alpha: int) -> str:
    """Return a Qt5-compatible rgba() color from #RRGGBB."""
    value = hex_color.lstrip("#")
    red, green, blue = (int(value[index:index + 2], 16)
                        for index in (0, 2, 4))
    return f"rgba({red}, {green}, {blue}, {alpha})"


APP_QSS = f"""
* {{
    font-family: "Noto Sans CJK SC", "Noto Sans SC", "Microsoft YaHei", sans-serif;
}}
QMainWindow, QWidget#appRoot, QWidget#workspace {{
    background: {COLORS['bg']}; color: {COLORS['text']};
}}
QWidget {{ color: {COLORS['text']}; font-size: 13px; }}
QScrollArea, QScrollArea > QWidget > QWidget {{
    background: {COLORS['bg']}; border: none;
}}
QFrame#topBar, QFrame#panel, QFrame#metricCard, QFrame#handPanel,
QFrame#fingerCard, QFrame#commandRail {{
    background: {COLORS['panel']};
    border: 1px solid {COLORS['border_soft']};
    border-radius: 12px;
}}
QFrame#commandRail {{ background: #091622; }}
QLabel#appTitle {{ font-size: 22px; font-weight: 700; letter-spacing: 1px; }}
QLabel#sectionTitle {{ font-size: 14px; font-weight: 700; color: {COLORS['text']}; }}
QLabel#eyebrow {{ font-size: 10px; font-weight: 700; color: {COLORS['cyan']}; letter-spacing: 2px; }}
QLabel#muted {{ color: {COLORS['muted']}; font-size: 11px; }}
QLabel#metricValue {{ font-size: 19px; font-weight: 700; }}
QLabel#forceValue {{ font-size: 20px; font-weight: 700; }}
QGroupBox {{
    color: {COLORS['muted']}; font-weight: 700; border: 1px solid {COLORS['border_soft']};
    border-radius: 10px; margin-top: 12px; padding-top: 12px; background: transparent;
}}
QGroupBox::title {{ subcontrol-origin: margin; left: 12px; padding: 0 6px; }}
QPushButton {{
    min-height: 38px; padding: 0 13px; border-radius: 8px;
    background: {COLORS['panel_3']}; border: 1px solid {COLORS['border']};
    color: {COLORS['text']}; font-weight: 600;
}}
QPushButton:hover {{ border-color: {COLORS['cyan']}; background: #173248; }}
QPushButton:pressed {{ background: #0B2435; }}
QPushButton:disabled {{ color: #536A79; background: #0B1721; border-color: #172B39; }}
QPushButton[kind="primary"] {{ background: #12374A; border-color: {COLORS['cyan']}; color: #DFFBFF; }}
QPushButton[kind="success"] {{ background: #123B2D; border-color: {COLORS['green']}; color: #E4FFF3; }}
QPushButton[kind="danger"] {{ background: #521F25; border: 1px solid {COLORS['red']}; color: #FFF0F1; font-size: 15px; font-weight: 800; }}
QPushButton[kind="danger"]:hover {{ background: #6B252D; }}
QPushButton[kind="quiet"] {{ background: transparent; }}
QPushButton[kind="primary"]:disabled,
QPushButton[kind="success"]:disabled,
QPushButton[kind="danger"]:disabled {{
    color: #536A79; background: #0B1721; border-color: #172B39;
}}
QProgressBar {{
    border: 1px solid {COLORS['border_soft']}; border-radius: 5px;
    background: #07131D; height: 9px; text-align: center;
}}
QProgressBar::chunk {{ border-radius: 4px; background: {COLORS['cyan']}; }}
QTabWidget {{ background: {COLORS['panel']}; border-radius: 10px; }}
QTabWidget::pane {{ border: 1px solid {COLORS['border_soft']}; border-radius: 10px; background: {COLORS['panel']}; top: -1px; }}
QTabBar {{ background: {COLORS['panel']}; }}
QTabBar::tab {{ background: #091622; color: {COLORS['muted']}; padding: 10px 18px; border: 1px solid {COLORS['border_soft']}; }}
QTabBar::tab:selected {{ color: {COLORS['cyan']}; background: {COLORS['panel']}; border-bottom-color: {COLORS['panel']}; }}
QTableWidget {{
    background: #091622; alternate-background-color: #0C1B28; color: {COLORS['text']};
    border: none; gridline-color: #1A3040; selection-background-color: #17445C;
}}
QHeaderView::section {{
    background: #102333; color: #AFC3D0; padding: 8px; border: none;
    border-right: 1px solid #20394A; font-weight: 700;
}}
QTextEdit {{
    background: #07131D; color: #BCD0DC; border: 1px solid {COLORS['border_soft']};
    border-radius: 8px; padding: 7px; font-family: "Noto Sans Mono CJK SC", monospace;
}}
QScrollBar:vertical {{ width: 10px; background: #07131D; }}
QScrollBar::handle:vertical {{ background: #29485C; border-radius: 5px; min-height: 28px; }}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0; }}
QSplitter::handle {{ background: {COLORS['bg']}; width: 8px; }}
"""


class UiSignals(QObject):
    log = pyqtSignal(str)
    warning = pyqtSignal(str, str)
    serial_open = pyqtSignal(bool)
    busy = pyqtSignal(bool)
    close_ready = pyqtSignal()


class StatusPill(QLabel):
    def __init__(self, text: str = "未知", parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._visual_state: tuple[str, str] | None = None
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumHeight(28)
        self.setContentsMargins(12, 0, 12, 0)
        self.set_state(text, "muted")

    def set_state(self, text: str, level: str) -> None:
        visual_state = (text, level)
        if visual_state == self._visual_state:
            return
        self._visual_state = visual_state
        color = {
            "ok": COLORS["green"],
            "warning": COLORS["amber"],
            "error": COLORS["red"],
            "info": COLORS["cyan"],
            "muted": COLORS["muted"],
        }.get(level, COLORS["muted"])
        self.setText(f"●  {text}")
        self.setStyleSheet(
            f"QLabel {{ color: {color}; background: {translucent(color, 24)}; "
            f"border: 1px solid {translucent(color, 102)}; "
            "border-radius: 14px; font-weight: 700; }"
        )


class MetricCard(QFrame):
    def __init__(self, title: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("metricCard")
        self._visual_state: tuple[str, str, str] | None = None
        self.setMinimumHeight(84)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 11, 14, 11)
        layout.setSpacing(3)
        title_label = QLabel(title.upper())
        title_label.setObjectName("eyebrow")
        self.value = QLabel("—")
        self.value.setObjectName("metricValue")
        self.detail = QLabel("等待数据")
        self.detail.setObjectName("muted")
        layout.addWidget(title_label)
        layout.addWidget(self.value)
        layout.addWidget(self.detail)

    def update_value(self, value: str, detail: str, level: str = "muted") -> None:
        visual_state = (value, detail, level)
        if visual_state == self._visual_state:
            return
        self._visual_state = visual_state
        color = {
            "ok": COLORS["green"], "warning": COLORS["amber"],
            "error": COLORS["red"], "info": COLORS["cyan"],
            "muted": COLORS["text"],
        }.get(level, COLORS["text"])
        self.value.setText(value)
        self.value.setStyleSheet(f"color: {color};")
        self.detail.setText(detail)


class ReadinessRow(QWidget):
    def __init__(self, text: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._visual_state: tuple[bool, str, bool] | None = None
        layout = QHBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        self.dot = QLabel("●")
        self.dot.setFixedWidth(16)
        self.label = QLabel(text)
        self.detail = QLabel("等待")
        self.detail.setObjectName("muted")
        self.detail.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        layout.addWidget(self.dot)
        layout.addWidget(self.label)
        layout.addStretch(1)
        layout.addWidget(self.detail)
        self.set_state(False, "等待")

    def set_state(self, ready: bool, detail: str, *, optional: bool = False) -> None:
        visual_state = (ready, detail, optional)
        if visual_state == self._visual_state:
            return
        self._visual_state = visual_state
        color = COLORS["muted"] if optional else (
            COLORS["green"] if ready else COLORS["red"])
        self.dot.setStyleSheet(f"color: {color};")
        self.detail.setText(detail)
        self.detail.setStyleSheet(f"color: {color}; font-size: 11px;")


class FingerGauge(QFrame):
    def __init__(self, name: str, color: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._last_state: str | None = None
        self.setObjectName("fingerCard")
        self.setMinimumWidth(92)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 9, 10, 9)
        layout.setSpacing(4)

        title_row = QHBoxLayout()
        name_label = QLabel(name)
        name_label.setStyleSheet("font-weight: 700;")
        self.state = QLabel("STOP")
        self.state.setObjectName("muted")
        self.state.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.state.setToolTip("树莓派本地力控状态，不代表G1已经执行覆盖指令")
        title_row.addWidget(name_label)
        title_row.addStretch(1)
        title_row.addWidget(self.state)

        self.force = QLabel("— N")
        self.force.setObjectName("forceValue")
        self.force.setStyleSheet(f"color: {color};")
        self.detail = QLabel("原始 —  ·  预载 —")
        self.detail.setObjectName("muted")
        self.inspire = QLabel("INSPIRE — N")
        self.inspire.setObjectName("muted")
        self.bar = QProgressBar()
        self.bar.setRange(0, 6000)
        self.bar.setTextVisible(False)
        self.bar.setStyleSheet(
            f"QProgressBar::chunk {{ background: {color}; border-radius: 4px; }}")

        layout.addLayout(title_row)
        layout.addWidget(self.force)
        layout.addWidget(self.bar)
        layout.addWidget(self.detail)
        layout.addWidget(self.inspire)

    def update_data(self, finger: dict) -> None:
        fsr = finger.get("fsr", {})
        inspire = finger.get("inspire", {})
        raw = fsr.get("raw_n")
        rest = fsr.get("rest_n")
        excess = fsr.get("excess_n")
        state = str(finger.get("state", "—"))
        force_text = f"{format_number(excess, 2)} N"
        detail_text = (
            f"原始 {format_number(raw, 2)}  ·  预载 {format_number(rest, 2)}")
        inspire_text = f"INSPIRE {format_number(inspire.get('force_n'), 2)} N"
        if self.force.text() != force_text:
            self.force.setText(force_text)
        if self.detail.text() != detail_text:
            self.detail.setText(detail_text)
        if self.inspire.text() != inspire_text:
            self.inspire.setText(inspire_text)
        value = 0.0 if excess is None else max(0.0, min(6.0, float(excess)))
        bar_value = round(value * 1000)
        if self.bar.value() != bar_value:
            self.bar.setValue(bar_value)
        if state != self._last_state:
            self._last_state = state
            self.state.setText(state)
            self.state.setStyleSheet(
                f"color: {STATE_COLORS.get(state, COLORS['muted'])}; "
                "font-size: 10px; font-weight: 700;")


class HandPanel(QFrame):
    def __init__(self, side: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.side = side
        self.setObjectName("handPanel")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(13, 11, 13, 12)
        layout.setSpacing(8)

        head = QHBoxLayout()
        title = QLabel(f"{SIDE_NAMES[side]} / {'ID 1–5' if side == 'right' else 'ID 6–10'}")
        title.setObjectName("sectionTitle")
        self.mode = StatusPill("STOP")
        head.addWidget(title)
        head.addStretch(1)
        head.addWidget(self.mode)
        layout.addLayout(head)

        links = QHBoxLayout()
        self.ble = StatusPill("BLE/FSR 未连接")
        self.inspire = StatusPill("INSPIRE 未连接")
        self.override = StatusPill("覆盖未启用")
        links.addWidget(self.ble)
        links.addWidget(self.inspire)
        links.addWidget(self.override)
        links.addStretch(1)
        layout.addLayout(links)

        gauges = QHBoxLayout()
        gauges.setSpacing(7)
        self.gauges = []
        for index, name in enumerate(FINGER_NAMES):
            gauge = FingerGauge(name, FINGER_COLORS[index])
            gauges.addWidget(gauge, 1)
            self.gauges.append(gauge)
        layout.addLayout(gauges)

    def update_data(self, hand: dict) -> None:
        self.mode.set_state("ARMED" if hand.get("armed") else "STOP",
                            "ok" if hand.get("armed") else "muted")
        ble = hand.get("ble_fsr", {})
        inspire = hand.get("inspire_feedback", {})
        override = hand.get("haptic_override", {})
        self.ble.set_state(
            f"BLE/FSR {format_number(ble.get('rate_hz'))} Hz"
            if ble.get("connected") else "BLE/FSR 断开",
            "ok" if ble.get("connected") else "error",
        )
        self.inspire.set_state(
            f"INSPIRE {format_number(inspire.get('rate_hz'))} Hz"
            if inspire.get("connected") else "INSPIRE 断开",
            "ok" if inspire.get("connected") else "error",
        )
        if not override.get("enabled"):
            self.override.set_state("覆盖未启用", "muted")
        else:
            self.override.set_state(
                "覆盖TCP在线" if override.get("connected") else "覆盖TCP断开",
                "ok" if override.get("connected") else "error",
            )
        for gauge, finger in zip(self.gauges, hand.get("fingers", [])):
            gauge.update_data(finger)


class SupervisorWindow(QMainWindow):
    # 主界面数值以10 Hz刷新；曲线/表格是树莓派上的主要绘制开销，
    # 单独降频且只在对应页签可见时重绘。这些频率不影响力控线程。
    REFRESH_MS = 100
    HISTORY_SAMPLE_MS = 200
    HEAVY_REFRESH_MS = 500
    HISTORY_POINTS = 150

    def __init__(self, args) -> None:
        super().__init__()
        self.args = args
        self.controller = DualHandController(args)
        self.signals = UiSignals()
        self.signals.log.connect(self.append_log)
        self.signals.warning.connect(
            lambda title, message: QMessageBox.warning(self, title, message))
        self.signals.serial_open.connect(self._set_serial_open)
        self.signals.busy.connect(self._set_busy)
        self.signals.close_ready.connect(self._finish_close)
        self.broker = BleBrokerManager(self.signals.log.emit)
        self.serial_is_open = False
        self.command_busy = False
        self.closing = False
        self.allow_close = False
        self.terminal_shutdown = False
        self.last_snapshot: dict = {}
        self.servo_rows: dict[tuple[str, int], int] = {}
        self.history = {
            (kind, side, index): deque([math.nan] * self.HISTORY_POINTS,
                                       maxlen=self.HISTORY_POINTS)
            for kind in ("fsr", "current")
            for side in SIDES
            for index in range(5)
        }
        self.curves: dict[tuple[str, str, int], object] = {}
        self._history_x = [
            (index - self.HISTORY_POINTS + 1) * self.HISTORY_SAMPLE_MS / 1000.0
            for index in range(self.HISTORY_POINTS)
        ]
        self._last_history_sample = 0.0
        self._last_table_render = 0.0
        self._last_plot_render = 0.0
        self._arm_visual: tuple[bool, str] | None = None

        self._build_window()
        self._build_ui()
        self._install_shortcuts()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(self.REFRESH_MS)
        self.clock_timer = QTimer(self)
        self.clock_timer.timeout.connect(self._update_clock)
        self.clock_timer.start(1000)
        self._update_clock()
        self.refresh()
        self.append_log("上位机已启动。G1链路客户端已自动启动；请连接舵机与双FSR。")
        if args.enable_mhandpro:
            self.append_log(
                "G1 INSPIRE力控覆盖已启用：9301/9302可在未启动mHandPro"
                "手套采集时仍直接改变INSPIRE手指位置。")
        if not args.enable_write:
            self.append_log("当前为只读启动；ARM 将保持锁定。")
        self.start_g1_links()

    def _build_window(self) -> None:
        self.setWindowTitle("dexEXO Force Control Studio")
        self.resize(1600, 980)
        self.setMinimumSize(1240, 760)
        self.setStyleSheet(APP_QSS)

    def _build_ui(self) -> None:
        root = QWidget()
        root.setObjectName("appRoot")
        outer = QVBoxLayout(root)
        outer.setContentsMargins(14, 12, 14, 12)
        outer.setSpacing(10)
        outer.addWidget(self._build_top_bar())

        splitter = QSplitter(Qt.Horizontal)
        splitter.setChildrenCollapsible(False)
        splitter.addWidget(self._build_command_rail())
        splitter.addWidget(self._build_workspace())
        splitter.setSizes([310, 1250])
        outer.addWidget(splitter, 1)
        self.setCentralWidget(root)

    def _build_top_bar(self) -> QWidget:
        bar = QFrame()
        bar.setObjectName("topBar")
        bar.setFixedHeight(72)
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(18, 10, 16, 10)

        accent = QFrame()
        accent.setFixedWidth(4)
        accent.setStyleSheet(
            f"background: {COLORS['cyan']}; border-radius: 2px;")
        brand = QVBoxLayout()
        brand.setSpacing(1)
        title = QLabel("dexEXO FORCE CONTROL STUDIO")
        title.setObjectName("appTitle")
        subtitle = QLabel("双手 FSR · G1 INSPIRE · XL330 ID 1–10 · 本地安全控制台")
        subtitle.setObjectName("muted")
        brand.addWidget(title)
        brand.addWidget(subtitle)
        layout.addWidget(accent)
        layout.addSpacing(10)
        layout.addLayout(brand)
        layout.addStretch(1)

        self.mode_pill = StatusPill("只读" if not self.args.enable_write else "写入已授权")
        self.mode_pill.set_state(
            "只读模式" if not self.args.enable_write else "写入已授权",
            "muted" if not self.args.enable_write else "warning")
        self.system_pill = StatusPill("STOP")
        self.clock_label = QLabel("—")
        self.clock_label.setObjectName("muted")
        self.clock_label.setMinimumWidth(150)
        self.clock_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        layout.addWidget(self.mode_pill)
        layout.addSpacing(8)
        layout.addWidget(self.system_pill)
        layout.addSpacing(14)
        layout.addWidget(self.clock_label)
        return bar

    def _build_command_rail(self) -> QWidget:
        rail = QFrame()
        rail.setObjectName("commandRail")
        rail.setMinimumWidth(290)
        rail.setMaximumWidth(340)
        layout = QVBoxLayout(rail)
        layout.setContentsMargins(13, 13, 13, 13)
        layout.setSpacing(9)

        section = QLabel("CONNECTION SEQUENCE")
        section.setObjectName("eyebrow")
        layout.addWidget(section)
        self.servo_button = self._button("1  连接十舵机", "primary", self.connect_servos)
        self.ble_button = self._button("2  连接双 FSR 蓝牙", "primary", self.toggle_ble)
        layout.addWidget(self.servo_button)
        layout.addWidget(self.ble_button)

        readiness_box = QGroupBox("ARM 就绪清单")
        readiness_layout = QVBoxLayout(readiness_box)
        readiness_layout.setContentsMargins(9, 14, 9, 9)
        readiness_layout.setSpacing(3)
        self.readiness = {
            "serial": ReadinessRow("TTL 串口 / 十舵机"),
            "ble": ReadinessRow("双手 BLE / FSR"),
            "inspire": ReadinessRow("双 INSPIRE 触觉"),
            "override": ReadinessRow("G1 INSPIRE 力控覆盖"),
            "init": ReadinessRow("十指 INIT / FSR 预载"),
            "fault": ReadinessRow("安全联锁"),
        }
        for row in self.readiness.values():
            readiness_layout.addWidget(row)
        layout.addWidget(readiness_box)

        section = QLabel("CONTROL")
        section.setObjectName("eyebrow")
        layout.addWidget(section)
        commands = QGridLayout()
        commands.setSpacing(7)
        self.status_button = self._button("STATUS", "quiet", self.on_status)
        self.init_button = self._button("INIT", "quiet", self.on_init)
        self.arm_button = self._button("ARM", "success", self.on_arm)
        self.quit_button = self._button("QUIT", "quiet", self.close)
        commands.addWidget(self.status_button, 0, 0)
        commands.addWidget(self.init_button, 0, 1)
        commands.addWidget(self.arm_button, 1, 0)
        commands.addWidget(self.quit_button, 1, 1)
        layout.addLayout(commands)
        self.stop_button = self._button("STOP  /  EMERGENCY RELEASE", "danger", self.on_stop)
        self.stop_button.setMinimumHeight(54)
        layout.addWidget(self.stop_button)

        self.arm_reason = QLabel("ARM 锁定：请先连接十舵机")
        self.arm_reason.setWordWrap(True)
        self.arm_reason.setStyleSheet(
            f"color: {COLORS['amber']}; "
            f"background: {translucent(COLORS['amber'], 18)}; "
            f"border: 1px solid {translucent(COLORS['amber'], 68)}; "
            "border-radius: 8px; padding: 8px;")
        layout.addWidget(self.arm_reason)

        recent_title = QLabel("最近活动")
        recent_title.setObjectName("sectionTitle")
        layout.addWidget(recent_title)
        self.side_log = QTextEdit()
        self.side_log.setReadOnly(True)
        self.side_log.document().setMaximumBlockCount(120)
        self.side_log.setMinimumHeight(130)
        layout.addWidget(self.side_log, 1)

        hint = QLabel("Esc: STOP   F5: STATUS   Ctrl+Q: QUIT")
        hint.setObjectName("muted")
        hint.setAlignment(Qt.AlignCenter)
        layout.addWidget(hint)
        return rail

    def _build_workspace(self) -> QWidget:
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        content = QWidget()
        content.setObjectName("workspace")
        layout = QVBoxLayout(content)
        layout.setContentsMargins(2, 0, 2, 0)
        layout.setSpacing(10)

        metrics = QHBoxLayout()
        metrics.setSpacing(9)
        self.metric_serial = MetricCard("Dynamixel TTL")
        self.metric_ble = MetricCard("BLE / FSR")
        self.metric_g1 = MetricCard("G1 INSPIRE")
        self.metric_override = MetricCard("G1 Force Override")
        for card in (self.metric_serial, self.metric_ble,
                     self.metric_g1, self.metric_override):
            metrics.addWidget(card, 1)
        layout.addLayout(metrics)

        hands = QHBoxLayout()
        hands.setSpacing(10)
        self.hand_panels = {side: HandPanel(side) for side in SIDES}
        for side in SIDES:
            hands.addWidget(self.hand_panels[side], 1)
        layout.addLayout(hands)

        self.tabs = QTabWidget()
        self.tabs.setMinimumHeight(380)
        self.tabs.addTab(self._build_servo_table(), "舵机总览")
        self.tabs.addTab(self._build_trends(), "30 秒实时曲线")
        self.tabs.addTab(self._build_activity(), "完整日志")
        self.tabs.currentChanged.connect(self._on_tab_changed)
        layout.addWidget(self.tabs, 1)
        scroll.setWidget(content)
        return scroll

    def _build_servo_table(self) -> QWidget:
        wrapper = QWidget()
        layout = QVBoxLayout(wrapper)
        layout.setContentsMargins(8, 8, 8, 8)
        columns = [
            "手别", "手指", "ID", "状态", "位置", "INIT位", "目标电流",
            "实际电流", "电压/V", "温度/℃", "模式", "扭矩", "HW Err",
            "健康年龄/ms", "通信错误",
        ]
        self.table = QTableWidget(0, len(columns))
        self.table.setHorizontalHeaderLabels(columns)
        self.table.setAlternatingRowColors(True)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.table.verticalHeader().setVisible(False)
        header = self.table.horizontalHeader()
        # ResizeToContents会在每次单元格变化时重算整表列宽，树莓派上开销很大。
        header.setSectionResizeMode(QHeaderView.Interactive)
        header.setStretchLastSection(True)
        widths = [58, 64, 42, 92, 78, 78, 82, 82, 68, 68, 52, 54, 68, 102]
        for column, width in enumerate(widths):
            self.table.setColumnWidth(column, width)
        layout.addWidget(self.table)
        return wrapper

    def _build_trends(self) -> QWidget:
        # 20条抗锯齿曲线会明显占用树莓派CPU，实时监视无需开启。
        pg.setConfigOptions(antialias=False, background=COLORS["panel"],
                            foreground=COLORS["muted"])
        wrapper = QWidget()
        grid = QGridLayout(wrapper)
        grid.setContentsMargins(8, 8, 8, 8)
        grid.setSpacing(8)
        for row, kind in enumerate(("fsr", "current")):
            for column, side in enumerate(SIDES):
                unit = "N" if kind == "fsr" else "raw"
                title = f"{SIDE_NAMES[side]} {'FSR 新增力' if kind == 'fsr' else '舵机实际电流'}"
                plot = pg.PlotWidget(title=title)
                plot.setLabel("left", unit)
                plot.setLabel("bottom", "时间", units="s")
                plot.showGrid(x=True, y=True, alpha=0.16)
                plot.setXRange(-30, 0, padding=0)
                plot.setMenuEnabled(False)
                plot.hideButtons()
                plot.addLegend(offset=(8, 8))
                for index, finger_name in enumerate(FINGER_NAMES):
                    pen = pg.mkPen(FINGER_COLORS[index], width=1.8)
                    self.curves[(kind, side, index)] = plot.plot(
                        [], [], pen=pen, name=finger_name,
                        clipToView=True, autoDownsample=True)
                grid.addWidget(plot, row, column)
        return wrapper

    def _build_activity(self) -> QWidget:
        wrapper = QWidget()
        layout = QVBoxLayout(wrapper)
        layout.setContentsMargins(8, 8, 8, 8)
        self.full_log = QTextEdit()
        self.full_log.setReadOnly(True)
        self.full_log.document().setMaximumBlockCount(800)
        layout.addWidget(self.full_log)
        return wrapper

    @staticmethod
    def _button(text: str, kind: str, callback: Callable[[], None]) -> QPushButton:
        button = QPushButton(text)
        button.setProperty("kind", kind)
        button.clicked.connect(callback)
        button.setCursor(Qt.PointingHandCursor)
        return button

    def _install_shortcuts(self) -> None:
        self.shortcuts = []
        for sequence, callback in (
                (QKeySequence(Qt.Key_Escape), self.on_stop),
                (QKeySequence("F5"), self.on_status),
                (QKeySequence("Ctrl+Q"), self.close)):
            shortcut = QShortcut(sequence, self)
            shortcut.activated.connect(callback)
            self.shortcuts.append(shortcut)

    def _update_clock(self) -> None:
        self.clock_label.setText(time.strftime("%Y-%m-%d  %H:%M:%S"))

    def append_log(self, message: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        text = html.escape(f"[{stamp}] {message}").replace("\n", "<br>")
        self.side_log.append(text)
        self.full_log.append(text)

    def _set_serial_open(self, opened: bool) -> None:
        self.serial_is_open = opened

    def _set_busy(self, busy: bool) -> None:
        self.command_busy = busy

    def run_command(self, label: str, function: Callable[[], object], *,
                    exclusive: bool = True) -> None:
        if exclusive and self.command_busy:
            self.append_log(f"{label}被拒绝：上一操作尚未完成。")
            return
        if exclusive:
            self.command_busy = True

        def worker() -> None:
            self.signals.log.emit(f"开始 {label}")
            try:
                result = function()
                suffix = "" if result is None else f"，结果={result}"
                self.signals.log.emit(f"{label} 完成{suffix}")
            except Exception as exc:
                self.signals.log.emit(f"{label} 失败: {exc}")
                if isinstance(exc, BleBrokerAlreadyRunningError):
                    self.signals.warning.emit("BLE Broker 已存在", str(exc))
            finally:
                if exclusive:
                    self.signals.busy.emit(False)

        threading.Thread(target=worker, name=f"qt-{label}", daemon=True).start()

    def connect_servos(self) -> None:
        if self.serial_is_open:
            return

        def connect() -> bool:
            try:
                self.controller.open()
                self.controller.start_control()
            except Exception:
                self.controller.port.closePort()
                raise
            self.signals.serial_open.emit(True)
            return True

        self.run_command("连接十舵机", connect)

    def toggle_ble(self) -> None:
        if self.broker.is_running():
            self.run_command("断开上位机拥有的双BLE", self.broker.stop)
            return
        occupied = self.broker.occupied_ports()
        if occupied:
            error = BleBrokerAlreadyRunningError(occupied)
            self.append_log(f"启动双BLE Broker被拒绝: {error}")
            QMessageBox.warning(self, "BLE Broker 已存在", str(error))
            return
        self.controller.start_fsr_inputs()
        self.run_command("启动双BLE Broker", self.broker.start)

    def start_g1_links(self) -> None:
        """自动启动树莓派侧G1 TCP客户端；G1服务端仍由操作者启动。"""
        started = self.controller.start_g1_inputs()
        if started:
            ports = "9201/9202"
            if self.args.enable_mhandpro:
                ports += " + 9301/9302"
            self.append_log(f"G1链路自动监视：{ports}；等待G1服务端或自动重连。")

    def on_status(self) -> None:
        try:
            self.append_log("STATUS\n" + format_status(
                self.controller.telemetry_snapshot()))
        except Exception as exc:
            self.append_log(f"STATUS 失败: {exc}")

    def on_init(self) -> None:
        if not self.serial_is_open:
            self.append_log("INIT被拒绝：十舵机尚未连接。")
            return
        self.append_log("INIT：开始采集十指返回点与FSR预载；请保持机构静止。")
        self.run_command("INIT", self.controller.init_all)

    def on_arm(self) -> None:
        allowed, reason = self.arm_gate(self.last_snapshot)
        if not allowed:
            self.append_log(f"ARM被拒绝：{reason}。")
            return
        self.append_log("ARM：全部安全门控已通过，正在使能十指力反馈。")
        self.run_command("ARM", self.controller.arm_all)

    def on_stop(self) -> None:
        if not self.serial_is_open:
            self.append_log("STOP：串口未打开，无需操作。")
            return
        self.run_command(
            "STOP", lambda: self.controller.stop_all("上位机人工STOP"),
            exclusive=False)

    def arm_gate(self, snapshot: dict) -> tuple[bool, str]:
        if not self.serial_is_open:
            return False, "请先连接十舵机"
        if not self.args.enable_write:
            return False, "只读启动，需要--enable-write"
        finger_count = 0
        for side in SIDES:
            hand = snapshot.get("hands", {}).get(side, {})
            if not hand.get("ble_fsr", {}).get("connected"):
                return False, f"{SIDE_NAMES[side]} BLE/FSR未就绪"
            if not hand.get("inspire_feedback", {}).get("connected"):
                return False, f"{SIDE_NAMES[side]} INSPIRE数据未就绪"
            override = hand.get("haptic_override", {})
            if override.get("enabled") and not override.get("connected"):
                return False, f"{SIDE_NAMES[side]} G1力控覆盖未就绪"
            for finger in hand.get("fingers", []):
                finger_count += 1
                if finger.get("state") == "FAULT":
                    return False, f"{SIDE_NAMES[side]}{finger.get('name')} FAULT"
                servo = finger.get("servo", {})
                if (servo.get("init_position") is None
                        or finger.get("fsr", {}).get("rest_n") is None):
                    return False, f"{SIDE_NAMES[side]}{finger.get('name')}尚未INIT"
        if finger_count != 10:
            return False, f"舵机遥测不完整（{finger_count}/10）"
        return True, "全部前置就绪"

    def refresh(self) -> None:
        if self.closing:
            return
        try:
            snapshot = self.controller.telemetry_snapshot()
            self.last_snapshot = snapshot
            self.render_snapshot(snapshot)
        except Exception as exc:
            self.system_pill.set_state(f"遥测错误: {exc}", "error")

    def render_snapshot(self, snapshot: dict) -> None:
        now = time.monotonic()
        system = snapshot.get("system", {})
        hands = snapshot.get("hands", {})
        state = str(system.get("state", "STOP"))
        self.system_pill.set_state(
            state, "error" if state == "FAULT" else
            "ok" if state == "ARMED" else "warning" if state == "PARTIAL" else "muted")

        ble_ok = all(hands.get(side, {}).get("ble_fsr", {}).get("connected")
                     for side in SIDES)
        inspire_ok = all(
            hands.get(side, {}).get("inspire_feedback", {}).get("connected")
            for side in SIDES)
        override_enabled = any(
            hands.get(side, {}).get("haptic_override", {}).get("enabled")
            for side in SIDES)
        override_ok = override_enabled and all(
            hands.get(side, {}).get("haptic_override", {}).get("connected")
            for side in SIDES)

        self.metric_serial.update_value(
            "SERIAL OPEN" if self.serial_is_open else "DISCONNECTED",
            f"{self.args.device} @ {self.args.baudrate}",
            "ok" if self.serial_is_open else "error")
        ble_rate = min(
            (float(hands.get(side, {}).get("ble_fsr", {}).get("rate_hz") or 0)
             for side in SIDES), default=0.0)
        self.metric_ble.update_value(
            "BOTH ONLINE" if ble_ok else "WAITING",
            f"最低接收率 {ble_rate:.1f} Hz",
            "ok" if ble_ok else "error")
        inspire_rate = min(
            (float(hands.get(side, {}).get("inspire_feedback", {}).get("rate_hz") or 0)
             for side in SIDES), default=0.0)
        self.metric_g1.update_value(
            "BOTH ONLINE" if inspire_ok else "WAITING",
            f"{self.args.force_host} · {inspire_rate:.1f} Hz",
            "ok" if inspire_ok else "error")
        if not override_enabled:
            self.metric_override.update_value("DISABLED", "未传 --enable-mhandpro", "muted")
        else:
            self.metric_override.update_value(
                "TCP CONNECTED" if override_ok else "WAITING",
                "9301/9302 · 当前协议无执行回执",
                "ok" if override_ok else "error")

        for side in SIDES:
            self.hand_panels[side].update_data(hands.get(side, {}))
        self.update_readiness(snapshot, ble_ok, inspire_ok,
                              override_enabled, override_ok)
        if now - self._last_history_sample >= self.HISTORY_SAMPLE_MS / 1000.0:
            self.sample_histories(hands)
            self._last_history_sample = now
        if (self.tabs.currentIndex() == 0
                and now - self._last_table_render >= self.HEAVY_REFRESH_MS / 1000.0):
            self.update_table(hands)
            self._last_table_render = now
        elif (self.tabs.currentIndex() == 1
              and now - self._last_plot_render >= self.HEAVY_REFRESH_MS / 1000.0):
            self.render_histories()
            self._last_plot_render = now
        self.update_buttons(snapshot)

    def _on_tab_changed(self, index: int) -> None:
        """页签切换时立即画一帧，其余时间不在后台重绘隐藏组件。"""
        if not self.last_snapshot:
            return
        hands = self.last_snapshot.get("hands", {})
        now = time.monotonic()
        if index == 0:
            self.update_table(hands)
            self._last_table_render = now
        elif index == 1:
            self.render_histories()
            self._last_plot_render = now

    def update_readiness(self, snapshot: dict, ble_ok: bool, inspire_ok: bool,
                         override_enabled: bool, override_ok: bool) -> None:
        hands = snapshot.get("hands", {})
        fingers = [
            finger
            for side in SIDES
            for finger in hands.get(side, {}).get("fingers", [])
        ]
        initialized = len(fingers) == 10 and all(
            finger.get("servo", {}).get("init_position") is not None
            and finger.get("fsr", {}).get("rest_n") is not None
            for finger in fingers)
        has_fault = any(
            finger.get("state") == "FAULT"
            for side in SIDES
            for finger in hands.get(side, {}).get("fingers", []))
        self.readiness["serial"].set_state(
            self.serial_is_open, "已打开" if self.serial_is_open else "未连接")
        self.readiness["ble"].set_state(ble_ok, "双手正常" if ble_ok else "等待数据")
        self.readiness["inspire"].set_state(
            inspire_ok, "双手正常" if inspire_ok else "等待数据")
        self.readiness["override"].set_state(
            override_ok if override_enabled else True,
            "双手TCP在线" if override_ok else
            "未启用" if not override_enabled else "等待连接",
            optional=not override_enabled)
        self.readiness["init"].set_state(
            initialized, "十指完成" if initialized else "未完成")
        self.readiness["fault"].set_state(
            not has_fault, "无故障" if not has_fault else "FAULT")

    def update_buttons(self, snapshot: dict) -> None:
        allowed, reason = self.arm_gate(snapshot)
        arm_visual = (allowed, reason)
        if arm_visual != self._arm_visual:
            self._arm_visual = arm_visual
            self.arm_reason.setText(
                f"ARM {'就绪' if allowed else '锁定'}：{reason}")
            color = COLORS["green"] if allowed else COLORS["amber"]
            self.arm_reason.setStyleSheet(
                f"color: {color}; background: {color}12; "
                f"border: 1px solid {color}44; "
                "border-radius: 8px; padding: 8px;")
        self.servo_button.setEnabled(not self.serial_is_open and not self.command_busy)
        self.ble_button.setEnabled(not self.command_busy)
        self.ble_button.setText(
            "2  断开双 FSR 蓝牙" if self.broker.is_running()
            else "2  连接双 FSR 蓝牙")
        self.init_button.setEnabled(self.serial_is_open and not self.command_busy)
        self.arm_button.setEnabled(allowed and not self.command_busy)
        self.stop_button.setEnabled(self.serial_is_open)

    def update_table(self, hands: dict) -> None:
        self.table.setUpdatesEnabled(False)
        try:
            for side in SIDES:
                for finger in hands.get(side, {}).get("fingers", []):
                    servo = finger.get("servo", {})
                    servo_id = int(finger.get("servo_id", -1))
                    key = (side, servo_id)
                    row = self.servo_rows.get(key)
                    if row is None:
                        row = self.table.rowCount()
                        self.table.insertRow(row)
                        self.servo_rows[key] = row
                    torque = servo.get("torque_enabled")
                    values = [
                        SIDE_NAMES[side], display_value(finger.get("name")), servo_id,
                        display_value(finger.get("state")),
                        display_value(servo.get("position")),
                        display_value(servo.get("init_position")),
                        display_value(servo.get("goal_current")),
                        display_value(servo.get("present_current")),
                        format_number(servo.get("voltage_v")),
                        display_value(servo.get("temperature_c")),
                        display_value(servo.get("operating_mode")),
                        "—" if torque is None else "ON" if torque else "OFF",
                        display_value(servo.get("hardware_error")),
                        display_value(servo.get("health_age_ms")),
                        display_value(servo.get("communication_errors")),
                    ]
                    state = str(finger.get("state", "STOP"))
                    foreground = QColor(STATE_COLORS.get(state, COLORS["text"]))
                    for column, value in enumerate(values):
                        item = self.table.item(row, column)
                        if item is None:
                            item = QTableWidgetItem()
                            item.setTextAlignment(Qt.AlignCenter)
                            self.table.setItem(row, column, item)
                        text = str(value)
                        if item.text() != text:
                            item.setText(text)
                        if column == 3 and item.foreground().color() != foreground:
                            item.setForeground(foreground)
        finally:
            self.table.setUpdatesEnabled(True)

    def sample_histories(self, hands: dict) -> None:
        """采样始终进行，以便打开曲线页时仍能看到过去30秒。"""
        for side in SIDES:
            for index, finger in enumerate(hands.get(side, {}).get("fingers", [])):
                fsr = finger.get("fsr", {}).get("excess_n")
                current = finger.get("servo", {}).get("present_current")
                self.history[("fsr", side, index)].append(
                    math.nan if fsr is None else float(fsr))
                self.history[("current", side, index)].append(
                    math.nan if current is None else float(current))

    def render_histories(self) -> None:
        """只在曲线页可见时把缓存数据交给pyqtgraph。"""
        for side in SIDES:
            for index in range(5):
                self.curves[("fsr", side, index)].setData(
                    self._history_x,
                    list(self.history[("fsr", side, index)]))
                self.curves[("current", side, index)].setData(
                    self._history_x,
                    list(self.history[("current", side, index)]))

    def closeEvent(self, event: QCloseEvent) -> None:  # noqa: N802 - Qt API
        if self.allow_close:
            event.accept()
            return
        if self.closing:
            event.ignore()
            return
        armed = any(
            finger.get("armed")
            for hand in self.last_snapshot.get("hands", {}).values()
            for finger in hand.get("fingers", []))
        if armed and not self.terminal_shutdown:
            answer = QMessageBox.question(
                self, "确认退出", "当前有舵机已 ARM。是否 STOP 并退出？",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if answer != QMessageBox.Yes:
                event.ignore()
                return
        event.ignore()
        self.closing = True
        self.timer.stop()
        self.append_log("QUIT：正在 STOP、关闭串口和 BLE Broker…")

        def close_all() -> None:
            try:
                self.controller.close()
            except Exception as exc:
                self.signals.log.emit(f"关闭控制器失败: {exc}")
            try:
                self.broker.stop()
            except Exception as exc:
                self.signals.log.emit(f"关闭BLE Broker失败: {exc}")
            self.signals.close_ready.emit()

        threading.Thread(target=close_all, name="qt-close", daemon=True).start()

    def _finish_close(self) -> None:
        self.allow_close = True
        self.close()

    def terminal_interrupt(self) -> None:
        """把终端Ctrl+C转换成GUI的STOP与完整关闭流程。"""
        if self.closing:
            return
        self.terminal_shutdown = True
        self.append_log("收到终端中断：正在STOP并安全退出。")
        self.close()


def main() -> int:
    args = parse_args()
    if (not os.environ.get("DISPLAY")
            and not os.environ.get("WAYLAND_DISPLAY")
            and os.environ.get("QT_QPA_PLATFORM") not in ("offscreen", "minimal")):
        print("无法启动PyQt上位机：当前会话没有DISPLAY/WAYLAND_DISPLAY。",
              file=sys.stderr)
        print("请在树莓派本地桌面或VNC中运行。", file=sys.stderr)
        return 2
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    app = QApplication([sys.argv[0]])
    app.setApplicationName("dexEXO Force Control Studio")
    app.setOrganizationName("dexEXO")
    window = SupervisorWindow(args)
    window.show()
    signal.signal(
        signal.SIGINT,
        lambda *_: QTimer.singleShot(0, window.terminal_interrupt),
    )
    signal.signal(
        signal.SIGTERM,
        lambda *_: QTimer.singleShot(0, window.terminal_interrupt),
    )
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
