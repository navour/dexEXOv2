#!/usr/bin/env python3
"""mHandPro 手套数据源：收 mhandpro_diagnostic 的 teleop 流，产出 6 通道闭合度。

链路是这样接的 —— ``mhandpro_diagnostic`` 的 ``teleop`` 命令本来就会**主动
连出去**一个 TCP，逐行发 JSON::

    {"type":"ctrl","angle_set":[6个0~1000],"force_set":[...],"mode":1}

它原本连的是树莓派本机的 Inspire 桥接（``127.0.0.1:9102``）。把 cfg 里的
``host``/``port`` 指到这里，同一份 C++ 一行不用改就成了手套数据源。所以这边
是 **TCP 服务端**，等它连进来。

``angle_set`` 是因时的角度寄存器值，端点由 cfg 的 ``open``/``closed`` 决定，
所以这里直接解析同一个 cfg 文件取端点 —— 两边读同一份配置就不会对不上。
换算复用 ``hand_mapping.glove_to_closure``，全仓库只有那一处做方向反转。

失效一律**张开**：断连、超时、坏行、通道数不对，``latest_closure()`` 都返回
None，上层据此发"没有数据"而不是一个握着的姿态。
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import socket
import threading
import time

import hand_mapping


DEFAULT_PORT = 9103
# 手套是 60Hz、teleop 默认 30Hz 下发；200ms 没有新数据就当它断了。
# 与 cfg 里的 stale_hold_ms 同量级，比 stale_abort_ms(500) 紧一档。
DEFAULT_STALE_SEC = 0.2
# 单行 JSON 也就一百多字节，给足余量；超过说明对面不是我们要的东西。
MAX_LINE_BYTES = 4096


def parse_cfg(cfg_path):
    """把 mhandpro 的 Inspire cfg 解析成 键→值 字典（值保持字符串）。

    读同一份文件是为了让 C++ 那边的换算和这边严格互逆 —— 端点抄错一位，
    手在仿真里会一直半握着，而且不会有任何报错。
    """
    values = {}
    for raw in Path(cfg_path).read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line or "=" not in line:
            continue
        key, value = (part.strip() for part in line.split("=", 1))
        values[key] = value
    return values


def load_endpoints(cfg_path):
    """从 cfg 里读 open/closed 端点。"""
    values = parse_cfg(cfg_path)
    endpoints = []
    for key in ("open", "closed"):
        if key not in values:
            raise ValueError(f"{cfg_path}: 缺少 {key}")
        counts = tuple(int(part) for part in values[key].split(","))
        if len(counts) != hand_mapping.CHANNEL_COUNT:
            raise ValueError(f"{cfg_path}: {key} 必须是 6 个整数")
        endpoints.append(counts)
    return tuple(endpoints)


def load_port(cfg_path, default=DEFAULT_PORT):
    """从 cfg 里读 teleop 要连的端口。

    C++ 那边 ``teleop`` 是按这个 cfg 里的 ``port`` 连出来的，所以监听端必须
    读同一个值 —— 各写各的迟早对不上，而且症状只是"连不上"，不好查。
    """
    value = parse_cfg(cfg_path).get("port")
    if value is None:
        return default
    return int(value)


def parse_ctrl(line):
    """解析一行 teleop JSON，返回 ``(6 个寄存器值, thumb_uv 或 None)``。

    坏行只丢这一帧，不断连接 —— 串口偶发一行乱码不该让整只手掉线。
    整行不可解析时返回 ``(None, None)``。

    ``thumb_uv`` 是拇指指腹重定向的归一化坐标，手套那端标定齐全才会带上。
    它坏了只让拇指退回 ``angle_set`` 里的线性投影值，不影响四指。
    """
    try:
        message = json.loads(line)
    except (json.JSONDecodeError, UnicodeDecodeError):
        return None, None
    if not isinstance(message, dict) or message.get("type") != "ctrl":
        return None, None
    counts = message.get("angle_set")
    if (not isinstance(counts, list)
            or len(counts) != hand_mapping.CHANNEL_COUNT):
        return None, None
    try:
        counts = tuple(int(value) for value in counts)
    except (TypeError, ValueError):
        return None, None

    thumb_uv = message.get("thumb_uv")
    if isinstance(thumb_uv, list) and len(thumb_uv) == 2:
        try:
            u, v = (float(thumb_uv[0]), float(thumb_uv[1]))
            thumb_uv = (u, v) if math.isfinite(u) and math.isfinite(v) else None
        except (TypeError, ValueError):
            thumb_uv = None
    else:
        thumb_uv = None
    return counts, thumb_uv


def parse_ctrl_line(line):
    """只要寄存器值的旧入口。"""
    return parse_ctrl(line)[0]


class GloveSource:
    """监听 teleop 的 TCP 流，随时取最新一帧闭合度。

    只保留最新帧，不排队 —— 和 UDP 接收端一样，控制回路要的是"现在"，
    不是补齐历史。
    """

    def __init__(self, cfg_path=None, port=None, host="127.0.0.1",
                 stale_sec=DEFAULT_STALE_SEC,
                 open_counts=None, closed_counts=None,
                 side="right", thumb_retarget=True):
        if cfg_path is not None:
            open_counts, closed_counts = load_endpoints(cfg_path)
            # port=None 表示"照 cfg 来"，这样调用方不必自己去读一遍 cfg；
            # port=0 是"随便挑一个空闲端口"（测试用），不能当成没给。
            if port is None:
                port = load_port(cfg_path)
        if port is None:
            port = DEFAULT_PORT
        if open_counts is None or closed_counts is None:
            raise ValueError("必须给出 cfg_path 或 open/closed 端点")
        self.open_counts = tuple(open_counts)
        self.closed_counts = tuple(closed_counts)
        self.stale_sec = float(stale_sec)

        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind((host, port))
        self._server.listen(1)
        self._server.settimeout(0.5)
        self.port = self._server.getsockname()[1]

        # 拇指指腹位置重定向。因时拇指只有 2 个自由度而人的拇指有 4 个以上,
        # 在旋转特征空间里这两个动作高度耦合(实测两轴夹角仅 31.5 度), 线性
        # 投影分解是病态的 —— 症状是"四指跟得挺好, 拇指位置不对"。改成在位置
        # 空间匹配指腹(同样两轴夹角 60.2 度), 条件数好一个量级。
        #
        # 运行时只要一个纯算术函数: 分解已经在手套端做完了, 这里只把归一化
        # 坐标落到通道上。懒加载, 起不来只关掉拇指重定向, 四指照跑。
        self.side = side
        self.thumb_map = None
        self.thumb_error = None
        self.thumb_frames = 0
        if thumb_retarget:
            try:
                from thumb_retarget import uv_to_closures
                self.thumb_map = uv_to_closures
            except Exception as exc:      # noqa: BLE001 - 降级不能挑异常类型
                self.thumb_error = str(exc)

        self._lock = threading.Lock()
        self._closure = None
        self._updated_at = 0.0
        self._running = True
        self._connected = False
        self.frames = 0
        self.bad_lines = 0
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    # ---------- 对外 ----------

    @property
    def connected(self):
        with self._lock:
            return self._connected

    def latest_closure(self):
        """最新闭合度；没连上、超时或还没收到数据时返回 None（=张开）。"""
        with self._lock:
            if self._closure is None:
                return None
            if time.monotonic() - self._updated_at > self.stale_sec:
                return None
            return self._closure

    def close(self):
        self._running = False
        try:
            self._server.close()
        except OSError:
            pass
        self._thread.join(timeout=1.0)

    # ---------- 内部 ----------

    def _serve(self):
        while self._running:
            try:
                connection, _ = self._server.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with self._lock:
                self._connected = True
            try:
                self._read_stream(connection)
            finally:
                connection.close()
                with self._lock:
                    self._connected = False
                    # 断连立刻作废，不留着上一帧让手僵在半握。
                    self._closure = None

    def _apply_thumb_retarget(self, closure, thumb_uv):
        """用指腹重定向覆盖拇指两个通道；任何一步不成立就原样返回。

        退回的是 ``angle_set`` 里那两个通道的线性投影值，**不是** 0 —— 送 0
        等于命令拇指张开，那是一个错误的抓握姿态，比稍微不准危险得多。

        四指通道在任何情况下都不碰。
        """
        if self.thumb_map is None or thumb_uv is None:
            return closure
        try:
            result = self.thumb_map(*thumb_uv)
        except Exception as exc:          # noqa: BLE001 - 降级不能挑异常类型
            self.thumb_error = str(exc)
            self.thumb_map = None         # 坏一次就别每帧再坏一次
            return closure
        if result is None:
            return closure
        flexion, opposition = result
        self.thumb_frames += 1
        # 通道序: 小指, 无名指, 中指, 食指, 拇指弯曲, 拇指对掌
        return tuple(closure[:4]) + (flexion, opposition)

    def _read_stream(self, connection):
        connection.settimeout(0.5)
        buffer = b""
        while self._running:
            try:
                chunk = connection.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                return
            if not chunk:
                return
            buffer += chunk
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                self._handle_line(line)
            if len(buffer) > MAX_LINE_BYTES:
                # 对面在灌没有换行的东西，丢掉重来而不是无限吃内存。
                self.bad_lines += 1
                buffer = b""

    def _handle_line(self, line):
        counts, thumb_uv = parse_ctrl(line.decode("utf-8", "replace").strip())
        if counts is None:
            if line.strip():
                self.bad_lines += 1
            return
        closure = hand_mapping.glove_to_closure(
            counts, self.open_counts, self.closed_counts)
        closure = self._apply_thumb_retarget(closure, thumb_uv)
        with self._lock:
            self._closure = closure
            self._updated_at = time.monotonic()
            self.frames += 1
