#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Multi-ESP32 IMU discovery, connection and control core APIs."""

from __future__ import annotations

import http.client
import select as _select
import socket
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

DISCOVERY_PORT = 4212
UDP_DATA_PORT = 4211
DEFAULT_TCP_PORT = 4210
OTA_HTTP_PORT = 8080
PKT_ROTATION = 0x01
PKT_TEXT = 0x02
ROTATION_FMT = "<BBH4h"
ROTATION_SIZE = struct.calcsize(ROTATION_FMT)
ROTATION_EXT_FMT = "<BBH4hI3h"
ROTATION_EXT_SIZE = struct.calcsize(ROTATION_EXT_FMT)
Q15_SCALE = 1.0 / 32767.0
GYRO_DPS10_SCALE = 0.1


def decode_rotation_packet(data: bytes):
    """解析12字节旧包或22字节扩展包，格式无效时返回None。"""
    if len(data) < ROTATION_SIZE or data[0] != PKT_ROTATION:
        return None
    values = struct.unpack(ROTATION_FMT, data[:ROTATION_SIZE])
    result = {
        "flags": values[1],
        "seq": values[2],
        "quat": [v * Q15_SCALE for v in values[3:7]],
        "sensor_timestamp_us": None,
        "gyro_dps": None,
    }
    if len(data) >= ROTATION_EXT_SIZE:
        ext = struct.unpack(ROTATION_EXT_FMT, data[:ROTATION_EXT_SIZE])
        result["sensor_timestamp_us"] = ext[7]
        result["gyro_dps"] = [
            ext[8] * GYRO_DPS10_SCALE,
            ext[9] * GYRO_DPS10_SCALE,
            ext[10] * GYRO_DPS10_SCALE,
        ]
    return result


@dataclass
class DeviceState:
    node_id: str
    ip: str
    tcp_port: int = DEFAULT_TCP_PORT
    udp_port: int = UDP_DATA_PORT
    device_id: str = ""
    last_seen: float = field(default_factory=time.time)
    connected: bool = False
    quat: List[float] = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])
    rest: bool = False
    seq: int = -1
    sensor_timestamp_us: Optional[int] = None
    receive_monotonic: float = 0.0
    gyro_dps: Optional[List[float]] = None
    battery_voltage: Optional[float] = None
    battery_percent: Optional[int] = None
    battery_remain_min: int = -1  # -1 = unknown/charging, >=0 = estimated minutes
    firmware_version: str = ""
    pkt_rate_hz: float = 0.0
    key_voltage: Optional[float] = None
    key_pressed: bool = False
    power_source: str = "UNKNOWN"
    charging: bool = False
    cal_state: str = "idle"
    cal_message: str = ""
    cal_raw_samples: List[List[float]] = field(default_factory=list)
    cal_progress_pct: int = 0
    cal_progress_count: int = 0
    cal_hard_iron: Optional[List[float]] = None
    cal_soft_iron: Optional[List[List[float]]] = None
    cal_field_norm: Optional[float] = None


class MultiImuService:
    """Reusable API for multi-device IMU management over LAN."""

    def __init__(self) -> None:
        self._devices: Dict[str, DeviceState] = {}
        self._ip_to_node: Dict[str, str] = {}
        self._tcp_socks: Dict[str, socket.socket] = {}
        self._selected_node: Optional[str] = None

        self._running = False
        self._lock = threading.RLock()
        self._threads: List[threading.Thread] = []
        self._pkt_times: Dict[str, List[float]] = {}  # node_id -> recent packet timestamps

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._threads = [
            threading.Thread(target=self._discovery_loop, daemon=True),
            threading.Thread(target=self._udp_data_loop, daemon=True),
            threading.Thread(target=self._tcp_maintain_loop, daemon=True),
        ]
        for t in self._threads:
            t.start()

    def stop(self) -> None:
        self._running = False
        with self._lock:
            for s in self._tcp_socks.values():
                try:
                    s.close()
                except OSError:
                    pass
            self._tcp_socks.clear()

    def list_devices(self) -> List[DeviceState]:
        with self._lock:
            items = list(self._devices.values())
            items.sort(key=lambda d: d.node_id)
            return [self._copy_device(d) for d in items]

    def select_device(self, node_id: str) -> bool:
        with self._lock:
            if node_id not in self._devices:
                return False
            self._selected_node = node_id
            return True

    def get_selected(self) -> Optional[DeviceState]:
        with self._lock:
            if not self._selected_node:
                return None
            d = self._devices.get(self._selected_node)
            return self._copy_device(d) if d else None

    def send_cal_start(self, node_id: str) -> bool:
        return self._send_cmd(node_id, "$CMD,CAL_START\n")

    def send_cal_stop(self, node_id: str) -> bool:
        return self._send_cmd(node_id, "$CMD,CAL_STOP\n")

    def send_cal_erase(self, node_id: str) -> bool:
        return self._send_cmd(node_id, "$CMD,CAL_ERASE\n")

    def send_shutdown(self, node_id: str) -> bool:
        """发送关机命令并等待 ACK 确认, 带重试.

        流程: 发送 $CMD,OFF -> 等待 TCP 回复 $CMD,OFF_ACK
        - 收到 ACK: 确认关机
        - 连接失败: 设备可能已关机
        - 无 ACK: 重试 (最多 5 次)
        """
        import http.client
        MAX_RETRIES = 5
        ACK_MARKER = b"$CMD,OFF_ACK"

        with self._lock:
            d = self._devices.get(node_id)
            if not d:
                return False
            ip = d.ip

        for attempt in range(1, MAX_RETRIES + 1):
            with self._lock:
                s = self._tcp_socks.get(node_id)

            if s is None:
                # No TCP connection, try direct connect
                try:
                    ns = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    ns.settimeout(3.0)
                    ns.connect((ip, self._tcp_socks.get(node_id) is None and 4210 or d.tcp_port))
                    ns.sendall(b"$CMD,OFF\n")
                    # Read ACK
                    buf = b""
                    try:
                        while True:
                            chunk = ns.recv(256)
                            if not chunk:
                                break
                            buf += chunk
                            if ACK_MARKER in buf:
                                ns.close()
                                return True
                    except socket.timeout:
                        pass
                    ns.close()
                except (socket.timeout, ConnectionRefusedError, OSError):
                    # Can't connect - device might be off
                    return True
                continue

            # Use existing TCP connection
            try:
                s.sendall(b"$CMD,OFF\n")
                buf = b""
                deadline = time.time() + 2.0
                while time.time() < deadline:
                    s.settimeout(1.0)
                    try:
                        chunk = s.recv(256)
                        if not chunk:
                            break
                        buf += chunk
                        if ACK_MARKER in buf:
                            return True
                    except socket.timeout:
                        break
            except OSError:
                pass

            time.sleep(1.0)

        return False

    @staticmethod
    def _copy_device(d: Optional[DeviceState]) -> Optional[DeviceState]:
        if d is None:
            return None
        return DeviceState(
            node_id=d.node_id,
            ip=d.ip,
            tcp_port=d.tcp_port,
            udp_port=d.udp_port,
            device_id=d.device_id,
            last_seen=d.last_seen,
            connected=d.connected,
            quat=list(d.quat),
            rest=d.rest,
            seq=d.seq,
            sensor_timestamp_us=d.sensor_timestamp_us,
            receive_monotonic=d.receive_monotonic,
            gyro_dps=list(d.gyro_dps) if d.gyro_dps else None,
            battery_voltage=d.battery_voltage,
            battery_percent=d.battery_percent,
            battery_remain_min=d.battery_remain_min,
            firmware_version=d.firmware_version,
            pkt_rate_hz=d.pkt_rate_hz,
            key_voltage=d.key_voltage,
            key_pressed=d.key_pressed,
            power_source=d.power_source,
            charging=d.charging,
            cal_state=d.cal_state,
            cal_message=d.cal_message,
            cal_raw_samples=[list(s) for s in d.cal_raw_samples],
            cal_progress_pct=d.cal_progress_pct,
            cal_progress_count=d.cal_progress_count,
            cal_hard_iron=list(d.cal_hard_iron) if d.cal_hard_iron else None,
            cal_soft_iron=[list(r) for r in d.cal_soft_iron] if d.cal_soft_iron else None,
            cal_field_norm=d.cal_field_norm,
        )

    def _upsert_device(self, node_id: str, ip: str, tcp_port: int, udp_port: int,
                       device_id: str = "") -> None:
        with self._lock:
            d = self._devices.get(node_id)
            if d is None:
                d = DeviceState(node_id=node_id, ip=ip, tcp_port=tcp_port, udp_port=udp_port)
                self._devices[node_id] = d
                if self._selected_node is None:
                    self._selected_node = node_id
            d.ip = ip
            d.tcp_port = tcp_port
            d.udp_port = udp_port
            if device_id:
                d.device_id = device_id
            d.last_seen = time.time()
            self._ip_to_node[ip] = node_id

    def _discovery_loop(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", DISCOVERY_PORT))
        sock.settimeout(1.0)
        while self._running:
            try:
                data, addr = sock.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                break

            text = data.decode("utf-8", errors="ignore").strip()
            if not text.startswith("VQF_DISC,"):
                continue
            fields = {}
            for kv in text.split(",")[1:]:
                if "=" not in kv:
                    continue
                k, v = kv.split("=", 1)
                fields[k] = v
            node = fields.get("node")
            ip = fields.get("ip", addr[0])
            try:
                tcp_port = int(fields.get("tcp", DEFAULT_TCP_PORT))
                udp_port = int(fields.get("udp", UDP_DATA_PORT))
            except ValueError:
                continue
            if node:
                device_id = fields.get("id", "")
                self._upsert_device(node, ip, tcp_port, udp_port, device_id)
                fw_ver = fields.get("ver", "")
                if fw_ver:
                    with self._lock:
                        d = self._devices.get(node)
                        if d:
                            d.firmware_version = fw_ver

        sock.close()

    def _udp_data_loop(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", UDP_DATA_PORT))
        sock.settimeout(1.0)

        while self._running:
            try:
                data, addr = sock.recvfrom(512)
            except socket.timeout:
                continue
            except OSError:
                break

            ip = addr[0]
            with self._lock:
                node = self._ip_to_node.get(ip)
                if not node:
                    continue
                d = self._devices.get(node)
                if not d:
                    continue
                d.last_seen = time.time()

            if len(data) < 4:
                continue
            ptype = data[0]

            if ptype == PKT_ROTATION and len(data) >= ROTATION_SIZE:
                rotation = decode_rotation_packet(data)
                if rotation is None:
                    continue
                receive_monotonic = time.monotonic()
                with self._lock:
                    d = self._devices.get(node)
                    if not d:
                        continue
                    d.seq = rotation["seq"]
                    d.rest = (rotation["flags"] & 0x01) != 0
                    d.quat = rotation["quat"]
                    d.sensor_timestamp_us = rotation[
                        "sensor_timestamp_us"]
                    d.receive_monotonic = receive_monotonic
                    d.gyro_dps = rotation["gyro_dps"]
                    # Update packet rate tracking
                    now_t = time.time()
                    pts = self._pkt_times.setdefault(node, [])
                    pts.append(now_t)
                    # Keep only last 2 seconds
                    cutoff = now_t - 2.0
                    while pts and pts[0] < cutoff:
                        pts.pop(0)
                    # Rate = packets in last 1 second
                    d.pkt_rate_hz = float(sum(1 for t in pts if t >= now_t - 1.0))
            elif ptype == PKT_TEXT:
                txt = data[4:].decode("utf-8", errors="ignore").strip("\x00\r\n ")
                self._handle_text(node, txt)

        sock.close()

    def _handle_text(self, node: str, txt: str) -> None:
        with self._lock:
            d = self._devices.get(node)
            if not d:
                return

            if txt.startswith("$PWR,"):
                parts = txt[5:].split(",")
                if not parts:
                    return
                kind = parts[0]
                if kind == "BAT" and len(parts) >= 8:
                    try:
                        d.battery_voltage = float(parts[1])
                        d.battery_percent = int(parts[2])
                        d.key_voltage = float(parts[4])
                        d.key_pressed = int(parts[5]) != 0
                        d.power_source = parts[7]
                        d.charging = (d.power_source == "USB")
                        # Parse remaining time: ...,REM,<minutes>
                        if len(parts) >= 10 and parts[8] == "REM":
                            d.battery_remain_min = int(parts[9])
                        else:
                            d.battery_remain_min = -1
                    except ValueError:
                        return
                elif kind in ("OFF", "LOW_BATTERY", "LONG_PRESS", "SHUTDOWN_PENDING"):
                    d.cal_message = txt
            elif txt.startswith("$CAL,"):
                parts = txt[5:].split(",")
                if not parts:
                    return
                kind = parts[0].upper()
                if kind == "SAMPLE" and len(parts) >= 4:
                    try:
                        mx, my, mz = float(parts[1]), float(parts[2]), float(parts[3])
                        d.cal_raw_samples.append([mx, my, mz])
                    except ValueError:
                        pass
                    return
                elif kind == "PROGRESS" and len(parts) >= 3:
                    try:
                        d.cal_progress_pct = int(parts[1])
                        d.cal_progress_count = int(parts[2])
                    except ValueError:
                        pass
                elif kind == "START":
                    d.cal_raw_samples.clear()
                    d.cal_progress_pct = 0
                    d.cal_progress_count = 0
                    d.cal_hard_iron = None
                    d.cal_soft_iron = None
                    d.cal_field_norm = None
                elif kind == "PARAMS" and len(parts) >= 14:
                    try:
                        hi = [float(parts[1]), float(parts[2]), float(parts[3])]
                        si = [
                            [float(parts[4]), float(parts[5]), float(parts[6])],
                            [float(parts[7]), float(parts[8]), float(parts[9])],
                            [float(parts[10]), float(parts[11]), float(parts[12])],
                        ]
                        fn = float(parts[13])
                        d.cal_hard_iron = hi
                        d.cal_soft_iron = si
                        d.cal_field_norm = fn
                    except ValueError:
                        pass
                    return
                d.cal_state = kind.lower()
                d.cal_message = txt
            elif txt.startswith("$ID,"):
                parts = txt[4:].split(",", 1)
                if parts and parts[0] == "OK" and len(parts) > 1:
                    d.device_id = parts[1]
                elif parts and parts[0] not in ("FAIL", "OK"):
                    d.device_id = parts[0]

    def _tcp_maintain_loop(self) -> None:
        while self._running:
            now = time.time()
            with self._lock:
                nodes = list(self._devices.keys())

            for node in nodes:
                with self._lock:
                    d = self._devices.get(node)
                    if not d:
                        continue
                    stale = (now - d.last_seen) > 8.0
                    if stale:
                        d.connected = False
                        s = self._tcp_socks.pop(node, None)
                        if s:
                            try:
                                s.close()
                            except OSError:
                                pass
                        continue

                    s = self._tcp_socks.get(node)
                    if s is None:
                        try:
                            ns = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                            ns.settimeout(1.0)
                            # 启用 TCP keepalive，快速检测断开的连接
                            ns.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                            try:
                                ns.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 5)
                                ns.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 1)
                                ns.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)
                            except (OSError, AttributeError):
                                pass
                            ns.connect((d.ip, d.tcp_port))
                            ns.settimeout(0.2)
                            self._tcp_socks[node] = ns
                            d.connected = True
                        except OSError:
                            d.connected = False
                    else:
                        # Socket 存在，检测是否真正存活
                        try:
                            rlist, _, xlist = _select.select([s], [], [s], 0)
                            if xlist:
                                raise OSError("socket error detected")
                            if rlist:
                                try:
                                    peek = s.recv(1, socket.MSG_PEEK)
                                    if len(peek) == 0:
                                        raise OSError("connection closed by peer")
                                except OSError:
                                    raise
                            d.connected = True
                        except OSError:
                            d.connected = False
                            try:
                                s.close()
                            except OSError:
                                pass
                            self._tcp_socks.pop(node, None)

            time.sleep(0.5)

    def _send_cmd(self, node_id: str, cmd: str) -> bool:
        with self._lock:
            s = self._tcp_socks.get(node_id)
            if s is None:
                return False
            try:
                s.sendall(cmd.encode("utf-8"))
                return True
            except OSError:
                try:
                    s.close()
                except OSError:
                    pass
                self._tcp_socks.pop(node_id, None)
                d = self._devices.get(node_id)
                if d:
                    d.connected = False
                return False

    # ---- Device ID API ----

    def set_device_id(self, node_id: str, device_id: str) -> bool:
        """设置设备自定义 ID (NVS 持久化, 断电保持). device_id 最长 32 字符."""
        return self._send_cmd(node_id, f"$CMD,SET_ID,{device_id}\n")

    def get_device_id(self, node_id: str) -> bool:
        """请求设备返回当前 ID. 结果通过 DeviceState.device_id 更新."""
        return self._send_cmd(node_id, "$CMD,GET_ID\n")

    # ---- OTA API ----

    def ota_update(self, node_id: str, firmware_path: str,
                   timeout: float = 60.0) -> bool:
        """通过 WiFi OTA 推送固件到指定设备.

        Args:
            node_id: 目标设备 node_id
            firmware_path: 固件 .bin 文件路径
            timeout: 超时秒数 (默认 60s)

        Returns:
            True 上传成功 (设备将自动重启), False 失败
        """
        with self._lock:
            d = self._devices.get(node_id)
            if not d:
                return False
            ip = d.ip

        with open(firmware_path, "rb") as f:
            firmware_data = f.read()

        conn = http.client.HTTPConnection(ip, OTA_HTTP_PORT, timeout=timeout)
        try:
            conn.request(
                "POST", "/update",
                body=firmware_data,
                headers={
                    "Content-Type": "application/octet-stream",
                    "Content-Length": str(len(firmware_data)),
                },
            )
            resp = conn.getresponse()
            return resp.status == 200
        except (OSError, http.client.HTTPException):
            return False
        finally:
            conn.close()

    def get_ota_url(self, node_id: str) -> Optional[str]:
        """获取设备 OTA 上传 URL."""
        with self._lock:
            d = self._devices.get(node_id)
            if not d:
                return None
            return f"http://{d.ip}:{OTA_HTTP_PORT}/update"
