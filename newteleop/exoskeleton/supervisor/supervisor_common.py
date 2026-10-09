#!/usr/bin/env python3
"""PyQt上位机可独立测试的非图形公共逻辑。"""

from __future__ import annotations

from pathlib import Path
import errno
import socket
import subprocess
import sys
import threading
from typing import Callable


SUPERVISOR_DIR = Path(__file__).resolve().parent
EXOSKELETON_DIR = SUPERVISOR_DIR.parent
BLE_BROKER_PATH = EXOSKELETON_DIR / "FSR" / "ble_broker.py"
SIDES = ("right", "left")
SIDE_NAMES = {"right": "右手", "left": "左手"}
FINGER_NAMES = ["拇指", "食指", "中指", "无名指", "小指"]
BLE_BROKER_PORTS = (9001, 9002)


class BleBrokerAlreadyRunningError(RuntimeError):
    """A broker (or another listener) already owns an FSR TCP port."""

    def __init__(self, ports: tuple[int, ...]) -> None:
        self.ports = ports
        joined = "/".join(str(port) for port in ports)
        super().__init__(
            f"端口 {joined} 已被占用；已有 BLE broker 正在运行，"
            "或端口被其他程序占用。请先关闭现有进程。"
        )


def format_number(value: object, digits: int = 1) -> str:
    if value is None:
        return "—"
    try:
        return f"{float(value):.{digits}f}"
    except (TypeError, ValueError):
        return str(value)


def display_value(value: object) -> object:
    return "—" if value is None else value


def format_status(snapshot: dict) -> str:
    """将缓存快照格式化为STATUS日志，不访问硬件。"""
    system = snapshot.get("system", {})
    serial = system.get("serial", {})
    lines = [
        f"系统={system.get('state', '—')}  "
        f"串口={'就绪' if serial.get('connected') else '未就绪'}  "
        f"写入={'允许' if system.get('write_enabled') else '禁止'}",
    ]
    for side in SIDES:
        hand = snapshot.get("hands", {}).get(side, {})
        ble = hand.get("ble_fsr", {})
        inspire = hand.get("inspire_feedback", {})
        override = hand.get("haptic_override", {})
        override_text = (
            "TCP在线(无执行回执)" if override.get("connected") else "断开"
        ) if override.get("enabled") else "未启用"
        lines.append(
            f"{SIDE_NAMES[side]}: BLE/FSR="
            f"{'正常' if ble.get('connected') else '断开'}  "
            f"INSPIRE={'正常' if inspire.get('connected') else '断开'}  "
            f"覆盖={override_text}"
        )
        if ble.get("last_error"):
            lines.append(f"  BLE/FSR最近错误: {ble['last_error']}")
        if inspire.get("last_error"):
            lines.append(f"  INSPIRE最近错误: {inspire['last_error']}")
        for finger in hand.get("fingers", []):
            fsr = finger.get("fsr", {})
            inspire_finger = finger.get("inspire", {})
            servo = finger.get("servo", {})
            lines.append(
                f"  {finger.get('name', '—')} ID{finger.get('servo_id', '—')}: "
                f"{finger.get('state', '—')}  "
                f"FSR={format_number(fsr.get('raw_n'), 3)}N  "
                f"INSPIRE={format_number(inspire_finger.get('force_n'), 3)}N  "
                f"pos={display_value(servo.get('position'))}  "
                f"I={display_value(servo.get('present_current'))}"
            )
    return "\n".join(lines)


class BleBrokerManager:
    """只管理上位机自己创建的双BLE broker子进程。"""

    def __init__(self, on_line: Callable[[str], None]) -> None:
        self.on_line = on_line
        self.process: subprocess.Popen[str] | None = None
        self.lock = threading.Lock()
        self.stopping = False

    def is_running(self) -> bool:
        with self.lock:
            return self.process is not None and self.process.poll() is None

    @staticmethod
    def occupied_ports(host: str = "127.0.0.1",
                       ports: tuple[int, ...] = BLE_BROKER_PORTS) -> tuple[int, ...]:
        """无连接副作用地检查FSR broker监听端口。"""
        occupied = []
        for port in ports:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
                probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                try:
                    probe.bind((host, port))
                except OSError as exc:
                    if exc.errno == errno.EADDRINUSE:
                        occupied.append(port)
                    else:
                        raise
        return tuple(occupied)

    def start(self) -> bool:
        with self.lock:
            if self.process is not None and self.process.poll() is None:
                return False
            occupied = self.occupied_ports()
            if occupied:
                raise BleBrokerAlreadyRunningError(occupied)
            if not BLE_BROKER_PATH.exists():
                raise FileNotFoundError(BLE_BROKER_PATH)
            self.stopping = False
            process = subprocess.Popen(
                [sys.executable, "-u", str(BLE_BROKER_PATH), "--hand", "both"],
                cwd=str(EXOSKELETON_DIR.parent),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            self.process = process
        threading.Thread(
            target=self._read_output,
            args=(process,),
            name="ble-broker-output",
            daemon=True,
        ).start()
        return True

    def _read_output(self, process: subprocess.Popen[str]) -> None:
        if process.stdout is not None:
            for line in process.stdout:
                self.on_line(f"[BLE] {line.rstrip()}")
        return_code = process.wait()
        with self.lock:
            was_stopping = self.stopping
            if self.process is process:
                self.process = None
            self.stopping = False
        if not was_stopping:
            self.on_line(f"[BLE] broker已退出，code={return_code}")

    def stop(self) -> bool:
        with self.lock:
            process = self.process
            if process is None or process.poll() is not None:
                self.process = None
                return False
            self.stopping = True
        process.terminate()
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=1.0)
        return True
