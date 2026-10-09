#!/usr/bin/env python3
"""板载机侧的因时 FTP 灵巧手驱动：闭合度 → Modbus TCP 寄存器。

**为什么这一层在机器人端而不是 PC 端。** 因时手用网线挂在机器人内网
(``192.168.123.x``) 上，PC 走 WiFi 时根本够不着它。把驱动放在板载机上，
板载机到手是内网 0 跳，和手臂的架构也一致 —— PC 只发意图 (UHND 尾块里
的闭合度)，机器人端负责驱动硬件。PC 端的 ``inspire_hand_ctrl.py`` 是
另一条路 (PC 网线直连时用)，两边共用 ``hand_mapping`` 的换算，不是副本。

**为什么不用 pymodbus。** 机器人上不了外网 (默认路由 192.168.123.1)，
装不了任何第三方包，能拷过去的只有纯 Python 文件。而 Modbus TCP 的读写
保持寄存器就是一个 7 字节 MBAP 头加一小段 PDU，用 socket 手写比把
pymodbus 整棵依赖树搬过去干净得多，也顺带绕开了 3.9 改
``slave=``→``device_id=`` 那个坑 (见 mhandpro/pymodbus_compat.py)。

本模块**不导入** unitree_sdk2py，所以能在 PC 上单测 —— 与 waist_follow.py
同一个理由：这段决定真手会不会握死，必须能在 PC 上测。
"""

from __future__ import annotations

import socket
import struct
import threading
import time
import json

import hand_mapping


# ---------- 因时寄存器 (与 mhandpro/standalone_inspire_bridge.py 一致,
# 那份在左手上实测跑通过) ----------
REGISTER_ANGLE_SET = 1486
REGISTER_ANGLE_ACT = 1546
DEFAULT_MODBUS_PORT = 6000
DEFAULT_DEVICE_ID = 1
# 宇树官方文档给的默认 IP。板载机和手在同一个 ② 网段, 直接可达。
HAND_IP = {"left": "192.168.123.210", "right": "192.168.123.211"}

COUNT_OPEN = hand_mapping.COUNT_MAX      # 1000 = 完全张开
COUNT_CLOSED = hand_mapping.COUNT_MIN    # 0 = 完全握紧
_FULL_OPEN = (COUNT_OPEN,) * hand_mapping.CHANNEL_COUNT
_FULL_CLOSED = (COUNT_CLOSED,) * hand_mapping.CHANNEL_COUNT

# 每帧最大变化量。50Hz 下 60 counts/帧 ≈ 0.33 秒走完全程。
DEFAULT_MAX_STEP = 60
DEFAULT_RATE_HZ = 50.0
# 多久没收到新的闭合度就算断流。比手臂的 1s 短 —— 手指夹住东西比手臂
# 停在半空更急, 而且张手不像手臂回零那样有姿态风险。
DEFAULT_TIMEOUT_SEC = 0.5
# 写失败后的重连间隔。不要更短: 手断电时会变成一秒几十次的连接风暴。
RECONNECT_INTERVAL_SEC = 1.0

MODBUS_FC_READ_HOLDING = 0x03
MODBUS_FC_WRITE_MULTIPLE = 0x10
_MBAP_LEN = 7

TOP_TOUCH_CHANNELS = [
    ("拇指", 4498, True),
    ("食指", 4128, False),
    ("中指", 3758, False),
    ("无名指", 3388, False),
    ("小指", 3018, False),
]
TOP_TOUCH_COUNT = 96
FORCE_K = 0.00292650244415058227
FORCE_B = -0.6037947156125716
THUMB_FORCE_K = 0.004420145759358057
THUMB_FORCE_B = -1.0701492398616255
FINGER_ANGLE_SLOTS = [4, 3, 2, 1, 0]  # [拇,食,中,无,小] -> Inspire六通道
HAPTIC_PORTS = {
    "right": {"force": 9201, "override": 9301},
    "left": {"force": 9202, "override": 9302},
}


class ModbusError(IOError):
    """Modbus 层的任何失败: 连不上、超时、异常响应、帧不合法。"""


# ======================== 纯函数: 帧编解码 ========================

def build_read_request(transaction, device_id, address, count):
    """构造读保持寄存器请求 (FC 0x03)。全部大端。"""
    pdu = struct.pack("!BHH", MODBUS_FC_READ_HOLDING, address, count)
    # MBAP 的 length 字段算的是它后面所有字节: unit_id(1) + pdu。
    return struct.pack("!HHHB", transaction, 0, len(pdu) + 1, device_id) + pdu


def build_write_request(transaction, device_id, address, values):
    """构造写多个保持寄存器请求 (FC 0x10)。"""
    values = [int(v) & 0xFFFF for v in values]
    pdu = struct.pack("!BHHB", MODBUS_FC_WRITE_MULTIPLE, address,
                      len(values), len(values) * 2)
    pdu += struct.pack(f"!{len(values)}H", *values)
    return struct.pack("!HHHB", transaction, 0, len(pdu) + 1, device_id) + pdu


def parse_response(frame, expect_fc):
    """校验响应帧并返回 PDU 里功能码之后的部分。

    Modbus 的异常响应把功能码最高位置 1 —— 这是**正常的响应**而不是传输
    错误, 所以必须在这里识别出来, 不然会被当成数据解析。
    """
    if len(frame) < _MBAP_LEN + 1:
        raise ModbusError(f"响应帧太短: {len(frame)} 字节")
    _, protocol, _, _ = struct.unpack("!HHHB", frame[:_MBAP_LEN])
    if protocol != 0:
        raise ModbusError(f"protocol id 不是 0: {protocol}")
    fc = frame[_MBAP_LEN]
    if fc == (expect_fc | 0x80):
        code = frame[_MBAP_LEN + 1] if len(frame) > _MBAP_LEN + 1 else -1
        raise ModbusError(f"从站返回异常码 {code} (功能码 {expect_fc})")
    if fc != expect_fc:
        raise ModbusError(f"功能码不匹配: 期望 {expect_fc}, 收到 {fc}")
    return frame[_MBAP_LEN + 1:]


def decode_read_response(frame, count):
    """从 FC 0x03 的响应里取出寄存器值。"""
    body = parse_response(frame, MODBUS_FC_READ_HOLDING)
    if not body:
        raise ModbusError("读响应没有字节计数")
    byte_count = body[0]
    if byte_count != count * 2:
        raise ModbusError(f"字节计数不符: 期望 {count * 2}, 收到 {byte_count}")
    data = body[1:1 + byte_count]
    if len(data) != byte_count:
        raise ModbusError("读响应数据不完整")
    return list(struct.unpack(f"!{count}H", data))


def signed_words(registers):
    packed = struct.pack(">" + "H" * len(registers), *registers)
    return list(struct.unpack(">" + "h" * len(registers), packed))


def touch_raw_to_force_n(raw, is_thumb):
    force = ((THUMB_FORCE_K * raw + THUMB_FORCE_B) if is_thumb
             else (FORCE_K * raw + FORCE_B))
    return max(0.0, min(force, 10.0))


# ======================== 纯函数: 换算与限速 ========================

def closure_to_counts(closure, range_scale=None):
    """闭合度 → 6 个寄存器值。``closure`` 为 None 时返回完全张开。

    None 是"这一帧没有手部数据", 张开是安全的, 保持上一个握持姿态不是 ——
    整条链路每个环节都守这一条。

    量程上限由 ``hand_mapping.closure_to_counts`` 自己再夹一次, 调用方
    传 1.0 也绕不过真手的 30% 硬上限。
    """
    if closure is None:
        return list(_FULL_OPEN)
    if range_scale is None:
        range_scale = hand_mapping.REAL_HAND_MAX_RANGE_SCALE
    return list(hand_mapping.closure_to_counts(
        closure, _FULL_OPEN, _FULL_CLOSED, range_scale=range_scale))


def step_toward(current, target, max_step=DEFAULT_MAX_STEP):
    """把 ``current`` 朝 ``target`` 推进一帧, 逐通道限速。

    限速放在下游而不是靠上游平滑: 上游断流时下游必须仍然是渐变的, 否则
    "没数据→张开"会变成一次瞬间弹开。
    """
    if len(current) != hand_mapping.CHANNEL_COUNT:
        raise ValueError(f"current 必须是 {hand_mapping.CHANNEL_COUNT} 个通道")
    if len(target) != hand_mapping.CHANNEL_COUNT:
        raise ValueError(f"target 必须是 {hand_mapping.CHANNEL_COUNT} 个通道")
    max_step = max(1, int(max_step))

    stepped = []
    for now, goal in zip(current, target):
        now = int(now)
        goal = int(goal)
        diff = goal - now
        if abs(diff) <= max_step:
            stepped.append(goal)
        else:
            stepped.append(now + (max_step if diff > 0 else -max_step))
    return [max(COUNT_CLOSED, min(COUNT_OPEN, v)) for v in stepped]


# ======================== 传输 ========================

class ModbusTcpHand:
    """裸 socket 的 Modbus TCP 客户端, 只实现这只手用到的两个功能码。

    每次调用都是"发一帧、收一帧"的同步往返。因时手是单连接单事务的从站,
    这里也不做流水线 —— 50Hz 下一帧一个往返完全够, 而且出错时的语义简单。
    """

    def __init__(self, ip, port=DEFAULT_MODBUS_PORT,
                 device_id=DEFAULT_DEVICE_ID, timeout=0.5):
        self.ip = ip
        self.port = port
        self.device_id = device_id
        self.timeout = timeout
        self._sock = None
        self._transaction = 0

    def connect(self):
        """建链并**真读一次寄存器**确认对面确实是这只手。

        只 connect 成功不算数: 交换机后面挂个别的设备也能三次握手, 那种
        情况下第一次写才会暴露, 而那时候手已经在动了。
        """
        self.close()
        sock = socket.create_connection((self.ip, self.port), self.timeout)
        sock.settimeout(self.timeout)
        self._sock = sock
        try:
            self.read_angles()
        except Exception:
            self.close()
            raise

    @property
    def connected(self):
        return self._sock is not None

    def _next_transaction(self):
        self._transaction = (self._transaction + 1) & 0xFFFF
        return self._transaction

    def _recv_exactly(self, n):
        chunks = []
        got = 0
        while got < n:
            try:
                chunk = self._sock.recv(n - got)
            except socket.timeout as exc:
                raise ModbusError(f"读超时 ({self.timeout}s)") from exc
            except OSError as exc:
                raise ModbusError(f"读失败: {exc}") from exc
            if not chunk:
                raise ModbusError("对端关闭了连接")
            chunks.append(chunk)
            got += len(chunk)
        return b"".join(chunks)

    def _transact(self, request):
        if self._sock is None:
            raise ModbusError("未连接")
        try:
            self._sock.sendall(request)
        except OSError as exc:
            raise ModbusError(f"发送失败: {exc}") from exc
        header = self._recv_exactly(_MBAP_LEN)
        # MBAP 的 length 含 unit_id, 而 unit_id 已经在 header 里读掉了。
        (_, _, length) = struct.unpack("!HHH", header[:6])
        remaining = length - 1
        if remaining < 1 or remaining > 260:
            raise ModbusError(f"响应长度不合法: {length}")
        return header + self._recv_exactly(remaining)

    def read_angles(self):
        """读实际角度。只读, 不下发任何动作。"""
        count = hand_mapping.CHANNEL_COUNT
        frame = self._transact(build_read_request(
            self._next_transaction(), self.device_id,
            REGISTER_ANGLE_ACT, count))
        return decode_read_response(frame, count)

    def read_holding(self, address, count, signed=False):
        frame = self._transact(build_read_request(
            self._next_transaction(), self.device_id, address, count))
        values = decode_read_response(frame, count)
        return signed_words(values) if signed else values

    def write_angles(self, counts):
        """写目标角度。"""
        frame = self._transact(build_write_request(
            self._next_transaction(), self.device_id,
            REGISTER_ANGLE_SET, counts))
        parse_response(frame, MODBUS_FC_WRITE_MULTIPLE)

    def close(self):
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None


class TcpJsonPublisher:
    """将灵巧手触觉广播给树莓派。慢客户端会被丢弃，不阻塞Modbus控制。"""

    def __init__(self, host, port, log):
        self.host, self.port, self.log = host, port, log
        self.running = False
        self.server = None
        self.clients = []
        self.lock = threading.Lock()

    def start(self):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((self.host, self.port))
        self.server.listen(2)
        self.server.settimeout(0.5)
        self.running = True
        threading.Thread(target=self._accept_loop, daemon=True).start()

    def _accept_loop(self):
        while self.running and self.server is not None:
            try:
                conn, address = self.server.accept()
                conn.settimeout(0.02)
                with self.lock:
                    self.clients.append(conn)
                self.log(f"[+] 触觉客户端已连接 {address}")
            except socket.timeout:
                continue
            except OSError:
                break

    def broadcast(self, packet):
        data = (json.dumps(packet, ensure_ascii=False,
                           separators=(",", ":")) + "\n").encode()
        with self.lock:
            alive = []
            for conn in self.clients:
                try:
                    conn.sendall(data)
                    alive.append(conn)
                except OSError:
                    conn.close()
            self.clients = alive

    def close(self):
        self.running = False
        if self.server is not None:
            self.server.close()
            self.server = None
        with self.lock:
            for conn in self.clients:
                conn.close()
            self.clients = []


class HapticOverrideServer:
    """接收树莓派每指状态及PID位置增量；超时自动恢复手套跟随。"""

    _ALLOWED = {"GLOVE", "FREE", "FORCE_ENTRY", "LOCKED", "RELEASE",
                "RETURN_SETTLE", "STOP", "FAULT"}

    def __init__(self, host, port, timeout, lock_retreat, max_step, log):
        self.host, self.port = host, port
        self.timeout, self.lock_retreat = timeout, lock_retreat
        self.max_step, self.log = max_step, log
        self.running = False
        self.server = None
        self.lock = threading.Lock()
        self.updated = 0.0
        self.states = None
        self.steps = [0.0] * 5
        self.locked = [None] * 5

    def start(self):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((self.host, self.port))
        self.server.listen(1)
        self.server.settimeout(0.5)
        self.running = True
        threading.Thread(target=self._accept_loop, daemon=True).start()

    def _clear(self):
        with self.lock:
            self.updated = 0.0
            self.states = None
            self.steps = [0.0] * 5
            self.locked = [None] * 5

    def _accept_loop(self):
        while self.running and self.server is not None:
            try:
                conn, address = self.server.accept()
                self.log(f"[+] 力控覆盖客户端已连接 {address}")
                self._client_loop(conn)
            except socket.timeout:
                continue
            except OSError:
                break

    def _client_loop(self, conn):
        buffer = ""
        conn.settimeout(1.0)
        try:
            while self.running:
                chunk = conn.recv(4096)
                if not chunk:
                    break
                buffer += chunk.decode("utf-8", errors="strict")
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    if not line.strip():
                        continue
                    packet = json.loads(line)
                    if packet.get("type") != "haptic_override":
                        continue
                    states = [str(v).upper() for v in
                              packet.get("finger_states", [])]
                    steps = packet.get("finger_steps", [])
                    if len(states) != 5 or any(v not in self._ALLOWED for v in states):
                        raise ValueError("finger_states必须是5个有效状态")
                    if len(steps) != 5:
                        raise ValueError("finger_steps必须是5个数")
                    with self.lock:
                        old = self.states or ["GLOVE"] * 5
                        for fi, state in enumerate(states):
                            if state == "LOCKED" and old[fi] != "LOCKED":
                                self.locked[fi] = None
                            elif state in ("GLOVE", "FREE", "STOP", "FAULT"):
                                self.locked[fi] = None
                            self.steps[fi] += float(steps[fi])
                        self.states = states
                        self.updated = time.monotonic()
        except Exception as exc:
            if self.running:
                self.log(f"[!] 力控覆盖客户端错误: {exc}")
        finally:
            conn.close()
            self._clear()
            self.log("[!] 力控覆盖已断开，恢复mHandPro跟随")

    def apply(self, glove_counts, last_written):
        result = list(glove_counts)
        with self.lock:
            if not self.updated or time.monotonic() - self.updated > self.timeout:
                self.states = None
                self.steps = [0.0] * 5
                self.locked = [None] * 5
                return result
            for fi, state in enumerate(self.states):
                slot = FINGER_ANGLE_SLOTS[fi]
                if state == "LOCKED":
                    if self.locked[fi] is None:
                        base = (last_written or glove_counts)[slot]
                        self.locked[fi] = float(min(COUNT_OPEN,
                                                    base + self.lock_retreat))
                    self.locked[fi] = max(COUNT_CLOSED, min(COUNT_OPEN,
                        self.locked[fi] - self.steps[fi]))
                    self.steps[fi] = 0.0
                    result[slot] = round(self.locked[fi])
                elif state in ("RELEASE", "RETURN_SETTLE") and last_written is not None:
                    old = last_written[slot]
                    result[slot] = max(old - self.max_step,
                                       min(old + self.max_step, glove_counts[slot]))
                else:
                    self.locked[fi] = None
                    self.steps[fi] = 0.0
        return result

    def close(self):
        self.running = False
        if self.server is not None:
            self.server.close()
            self.server = None
        self._clear()


# ======================== 跟随线程 ========================

class HandFollower:
    """一只手的驱动线程。

    独立于 500Hz 控制线程跑, 因为 Modbus 是阻塞 socket 往返 —— 手断电时
    一次 ``recv`` 就要阻塞到超时, 绝不能让它卡住手臂的控制周期。两者之间
    只通过 ``update()`` 传一个闭合度快照。

    超时判定放在本线程内部而不是靠调用方: 发送端整个不发包时, ``update()``
    根本不会被调用, 只有自己看时间戳才能发现断流。
    """

    def __init__(self, side, ip=None, port=DEFAULT_MODBUS_PORT,
                 device_id=DEFAULT_DEVICE_ID, max_step=DEFAULT_MAX_STEP,
                 rate_hz=DEFAULT_RATE_HZ, timeout_sec=DEFAULT_TIMEOUT_SEC,
                 range_scale=None, transport=None, log=None,
                 haptic=False, haptic_host="0.0.0.0", touch_hz=5.0,
                 haptic_timeout=0.5, lock_retreat=80):
        if side not in HAND_IP:
            raise ValueError("side 只能是 'right' 或 'left'")
        self.side = side
        self.ip = ip or HAND_IP[side]
        self.port = port
        self.max_step = max_step
        self.rate_hz = max(1.0, float(rate_hz))
        self.timeout_sec = timeout_sec
        self.range_scale = range_scale
        self._log = log or (lambda msg: None)
        self._transport = transport or ModbusTcpHand(
            self.ip, port=port, device_id=device_id)

        self._lock = threading.Lock()
        self._closure = None
        self._stamp = 0.0
        # 起始姿态是张开, 不是零 —— 零在这个量程里是"握死"。
        self._current = list(_FULL_OPEN)
        self._connected = False
        self._last_error = None
        self._next_retry = 0.0
        self._running = False
        self._thread = None
        self._haptic_enabled = bool(haptic)
        self._touch_period = 1.0 / max(1.0, float(touch_hz))
        self._next_touch = 0.0
        self._touch_seq = 0
        ports = HAPTIC_PORTS[side]
        self._publisher = TcpJsonPublisher(
            haptic_host, ports["force"], self._log) if haptic else None
        self._override = HapticOverrideServer(
            haptic_host, ports["override"], haptic_timeout,
            lock_retreat, max_step, self._log) if haptic else None

    # ---------- 对外接口 ----------

    def update(self, closure, now=None):
        """收到新的一帧闭合度。``closure`` 为 None 表示这只手没有数据。"""
        with self._lock:
            self._closure = None if closure is None else tuple(closure)
            self._stamp = time.monotonic() if now is None else now

    def start(self):
        if self._thread is not None:
            return
        self._running = True
        if self._publisher is not None:
            self._publisher.start()
        if self._override is not None:
            self._override.start()
        self._thread = threading.Thread(
            target=self._loop, name=f"hand-{self.side}", daemon=True)
        self._thread.start()

    def stop(self, timeout=2.0):
        """停线程前把手张开再断开, 别让它攥着东西停在那儿。"""
        self._running = False
        thread = self._thread
        if thread is not None:
            thread.join(timeout=timeout)
            self._thread = None
        self._open_now()
        if self._publisher is not None:
            self._publisher.close()
        if self._override is not None:
            self._override.close()
        self._transport.close()
        self._connected = False

    def status(self):
        """给状态行用的一行摘要。"""
        with self._lock:
            current = list(self._current)
            connected = self._connected
            error = self._last_error
            fresh = (self._closure is not None
                     and time.monotonic() - self._stamp <= self.timeout_sec)
        if not connected:
            return f"{self.side}手:断开({error or '连接中'})"
        # 显示平均闭合百分比 —— 6 个通道全列出来状态行会爆。
        span = COUNT_OPEN - COUNT_CLOSED
        closed_pct = 100.0 * (1.0 - sum(current) / (len(current) * span))
        flag = "" if fresh else "(张开)"
        return f"{self.side}手:{closed_pct:3.0f}%{flag}"

    # ---------- 内部 ----------

    def _target_counts(self, now):
        with self._lock:
            closure = self._closure
            stamp = self._stamp
        if closure is None or (now - stamp) > self.timeout_sec:
            # 断流 → 张开。不是"保持上一帧"。
            return list(_FULL_OPEN)
        return closure_to_counts(closure, self.range_scale)

    def _ensure_connected(self, now):
        if self._transport.connected:
            return True
        if now < self._next_retry:
            return False
        self._next_retry = now + RECONNECT_INTERVAL_SEC
        try:
            self._transport.connect()
        except Exception as exc:
            with self._lock:
                self._connected = False
                self._last_error = str(exc)
            return False
        with self._lock:
            self._connected = True
            self._last_error = None
        self._log(f"[+] {self.side}手已连接 {self.ip}:{self.port}")
        return True

    def _loop(self):
        period = 1.0 / self.rate_hz
        while self._running:
            started = time.monotonic()
            self._step(started)
            slept = period - (time.monotonic() - started)
            if slept > 0:
                time.sleep(slept)

    def _step(self, now):
        target = self._target_counts(now)
        if self._override is not None:
            target = self._override.apply(target, self._current)
        stepped = step_toward(self._current, target, self.max_step)
        # 先推进内部状态再发。断链期间也照推 —— 这样重连后的第一帧发的是
        # 张开而不是断链前那个握持姿态。
        with self._lock:
            self._current = stepped
        if not self._ensure_connected(now):
            return
        try:
            self._transport.write_angles(stepped)
            self._publish_touch_if_due(now)
        except Exception as exc:
            self._transport.close()
            with self._lock:
                self._connected = False
                self._last_error = str(exc)
            self._next_retry = now + RECONNECT_INTERVAL_SEC
            self._log(f"[!] {self.side}手写入失败, 将重连: {exc}")

    def _publish_touch_if_due(self, now):
        if self._publisher is None or now < self._next_touch:
            return
        self._next_touch = now + self._touch_period
        maxima, means, forces = [], [], []
        try:
            for _name, address, is_thumb in TOP_TOUCH_CHANNELS:
                values = self._transport.read_holding(
                    address, TOP_TOUCH_COUNT, signed=True)
                maximum = max(values)
                maxima.append(maximum)
                means.append(sum(sorted(values, reverse=True)[:5]) / 5.0)
                forces.append(touch_raw_to_force_n(maximum, is_thumb))
            valid, error = True, None
        except Exception as exc:
            valid, error = False, str(exc)
            maxima, means, forces = [0] * 5, [0.0] * 5, [0.0] * 5
        self._touch_seq += 1
        packet = {
            "type": "inspire_feedback", "hand": self.side,
            "seq": self._touch_seq, "mono_ms": int(now * 1000),
            "top_touch_order": [item[0] for item in TOP_TOUCH_CHANNELS],
            "top_touch_raw_max": maxima,
            "top_touch_top5_mean": means,
            "top_touch_force_n": forces,
            "touch_valid": valid,
        }
        if error is not None:
            packet["error"] = error
        self._publisher.broadcast(packet)

    def _open_now(self):
        """尽力把手张开。链路已经断了就没办法, 但不能因此抛异常。"""
        if not self._transport.connected:
            return
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            self._current = step_toward(
                self._current, list(_FULL_OPEN), self.max_step)
            try:
                self._transport.write_angles(self._current)
            except Exception:
                return
            if self._current == list(_FULL_OPEN):
                return
            time.sleep(1.0 / self.rate_hz)
