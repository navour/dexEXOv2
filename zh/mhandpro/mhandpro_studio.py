#!/usr/bin/env python3
"""mHandPro 图形上位机：标定、映射、诊断与 Inspire URDF 数字孪生。

界面通过伪终端驱动原有 ``mhandpro_diagnostic``，所有标定、TELEOP 和安全
判断仍只有 C++ 一份实现。实时姿态来自只读 ``snapshot`` 命令；手部输出沿用
原来的 9103/9104 TCP 链路，不从 PC 直连机器人内网的 Modbus 地址。
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import asdict, dataclass, field
import json
import math
import os
from pathlib import Path
import queue
import re
import signal
import subprocess
import sys
import threading
import time
from typing import Callable


PROJECT_DIR = Path(__file__).resolve().parent
REPO_DIR = PROJECT_DIR.parent
PC_DIR = REPO_DIR / "PC端"
WORKING_CODE_DIR = REPO_DIR / "最新直接能用的代码" / "mhandpro"
URDF_PATH = (REPO_DIR / "仿真/models/g1_29dof/"
             "g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf")
BINARY_PATH = PROJECT_DIR / "bin/mhandpro_diagnostic"
# 真机现场长期使用的 CLI 配置在“最新直接能用的代码”中。
# 上位机优先沿用它们，保证按钮执行的 teleop both 与旧 CLI
# 逐字一致；独立拷贝本项目时才回退到新上位机自带配置。
_CONFIG_DIR = (WORKING_CODE_DIR / "config"
               if (WORKING_CODE_DIR / "config").is_dir()
               else PROJECT_DIR / "config")
RIGHT_SIM_CONFIG = _CONFIG_DIR / "inspire_right_sim.cfg"
LEFT_SIM_CONFIG = _CONFIG_DIR / "inspire_left_sim.cfg"
LIBRARY_PATHS = {
    "x86_64": PROJECT_DIR / "sdk/lib/x64/libVDMocapSDK_mHandPro.so",
    "aarch64": PROJECT_DIR / "sdk/lib/arm64/libVDMocapSDK_mHandProArm64.so",
}
SNAPSHOT_PREFIX = "MHAND_SNAPSHOT "
CHANNEL_NAMES = ("小指", "无名指", "中指", "食指", "拇指弯曲", "拇指对掌")

sys.path.insert(0, str(PC_DIR))
import hand_mapping  # noqa: E402
from g1_urdf_renderer import G1UrdfRenderer  # noqa: E402
from thumb_retarget import retarget_hand_closures  # noqa: E402
from hand_telemetry import (  # noqa: E402
    EpisodeRecorder, HandFeedbackReceiver, RobotFeedback, load_episode,
)

GESTURE_NAMES = {
    0: "未识别", 1: "指向", 2: "剪刀手", 3: "OK", 5: "张掌",
    11: "比心", 14: "点赞", 15: "抓取", 16: "握拳", 22: "三",
}


def build_teleop_command(side: str) -> str:
    """生成与原 CLI 完全相同的 9103/9104 TELEOP 命令。"""
    if side == "both":
        return f"teleop both {RIGHT_SIM_CONFIG} {LEFT_SIM_CONFIG}"
    if side == "right":
        return f"teleop {RIGHT_SIM_CONFIG}"
    if side == "left":
        return f"teleop {LEFT_SIM_CONFIG}"
    raise ValueError("side 只能是 'right'、'left' 或 'both'")


def top_status_layout(width: int) -> dict[str, tuple[int, int, int, int]]:
    """顶部双手卡片与链路状态的响应式布局。

    链路卡始终放在左手卡右侧，不再占用下方 3D 模型区。
    """
    main_x, top, height, gap = 356, 82, 144, 14
    available = max(730, int(width) - main_x - 14)
    chain_min = 250
    card_w = min(330, max(210, (available - chain_min - gap * 2) // 2))
    right = (main_x, top, card_w, height)
    left = (main_x + card_w + gap, top, card_w, height)
    chain_x = left[0] + card_w + gap
    chain = (chain_x, top, max(120, int(width) - chain_x - 14), height)
    return {"right_card": right, "left_card": left, "chain": chain}


@dataclass
class HandState:
    valid: bool = False
    frame: int = -1
    frequency: int = -1
    power: float = 0.0
    age_ms: int = -1
    sensors_ok: bool = False
    sensor_counts: tuple[int, ...] = (0, 0, 0, 0, 0)
    calibrated: bool = False
    thumb_retarget: bool = False
    thumb_virtual: bool = False
    closure: tuple[float, ...] = (0.0,) * 6
    gesture: int = 0
    virtual_valid: bool = False
    pinch_mm: float = -1.0
    fingertips: tuple[tuple[float, ...], ...] = ()
    stability: float = 0.0


@dataclass
class Snapshot:
    selected: str = "right"
    disconnected: bool = False
    hands: dict[str, HandState] = field(default_factory=lambda: {
        "right": HandState(), "left": HandState(),
    })


def parse_snapshot_line(line: str) -> Snapshot | None:
    """解析 C++ 的机器快照；坏行直接忽略，绝不让显示线程退出。"""
    marker = line.find(SNAPSHOT_PREFIX)
    if marker < 0:
        return None
    try:
        payload = json.loads(line[marker + len(SNAPSHOT_PREFIX):].strip())
        hands = {}
        for side in ("right", "left"):
            raw = payload.get("hands", {}).get(side, {})
            closure = tuple(max(0.0, min(1.0, float(v)))
                            for v in raw.get("closure", (0.0,) * 6))
            if len(closure) != 6:
                closure = (0.0,) * 6
            thumb_uv = raw.get("thumb_uv")
            if isinstance(thumb_uv, list) and len(thumb_uv) == 2:
                closure = retarget_hand_closures(closure, *thumb_uv)
            sensor_counts = tuple(int(v) for v in raw.get(
                "sensor_counts", (0, 0, 0, 0, 0)))
            if len(sensor_counts) != 5:
                sensor_counts = (0, 0, 0, 0, 0)
            hands[side] = HandState(
                valid=bool(raw.get("valid", False)),
                frame=int(raw.get("frame", -1)),
                frequency=int(raw.get("frequency", -1)),
                power=float(raw.get("power", 0.0)),
                age_ms=int(raw.get("age_ms", -1)),
                sensors_ok=bool(raw.get("sensors_ok", False)),
                sensor_counts=sensor_counts,
                calibrated=bool(raw.get("calibrated", False)),
                thumb_retarget=bool(raw.get("thumb_retarget", False)),
                thumb_virtual=bool(raw.get("thumb_virtual", False)),
                closure=closure,
                gesture=int(raw.get("gesture", 0)),
                virtual_valid=bool(raw.get("virtual_valid", False)),
                pinch_mm=float(raw.get("pinch_mm", -1.0)),
                fingertips=tuple(tuple(float(x) for x in point)
                                   for point in (raw.get("fingertips") or ())),
            )
        return Snapshot(
            selected=str(payload.get("selected", "right")),
            disconnected=bool(payload.get("disconnected", False)),
            hands=hands,
        )
    except (TypeError, ValueError, json.JSONDecodeError):
        return None


def classify_hand_state(state: HandState, disconnected: bool = False):
    """把数据链路与节点质量分开；有新鲜帧就不能显示成“离线”。"""
    if disconnected or not state.valid or state.age_ms < 0 or state.age_ms >= 500:
        return "离线或数据超时", "error"
    if not state.sensors_ok:
        return "在线 · 节点需检查", "warning"
    return "在线 · 传感正常", "ok"


def parse_workflow_text(text: str) -> dict:
    """从原 CLI 文本中提取供全屏标定卡片显示的结构化状态。"""
    result = {"kind": "", "step": 0, "count": 0, "title": "",
              "instruction": "", "phase": ""}
    mapping = re.search(
        r"=+\s*mapcal\s+(\d+)/(\d+):\s*(.*?)\s*=+", text)
    if mapping:
        result.update(kind="mapping", step=int(mapping.group(1)),
                      count=int(mapping.group(2)), title=mapping.group(3).strip())
        tail = text[mapping.end():]
        lines = [line.strip() for line in re.split(r"[\r\n]+", tail)
                 if line.strip()]
        if lines:
            result["instruction"] = lines[0]
    elif "官方 P-pose 手掌标定" in text:
        result.update(kind="ppose", step=1, count=1,
                      title="官方 P-pose 手掌标定",
                      instruction="双臂向前平举，手掌朝下，四指并拢伸直，拇指张开 45~60°")
    elif "磁力校准" in text:
        result.update(kind="magcal", step=1, count=1,
                      title="官方磁力校准",
                      instruction="摘下手套，远离磁性物体，缓慢覆盖三个轴的全部朝向")
    countdown = re.findall(r"(\d+)\s*秒后(?:开始|采集)", text)
    remaining = re.findall(r"剩余\s*(\d+)s", text)
    progress = re.findall(r"进度=([0-9]+)%", text)
    if countdown:
        result["phase"] = f"倒计时 {countdown[-1]} 秒"
    elif remaining:
        result["phase"] = f"采集中 · 剩余 {remaining[-1]} 秒"
    elif progress:
        result["phase"] = f"官方标定进度 {progress[-1]}%"
    elif "开始采集" in text or "开始标定" in text:
        result["phase"] = "正在采集，请保持不动"
    return result


class DiagnosticBackend:
    """用 PTY 承载交互式 CLI，既保留原流程又避免 stdout 块缓冲。"""

    WAIT_PATTERNS = (
        "摆好姿势后按回车", "摆好后按回车开始采集",
        "准备好按回车开始", "请先保持张手。输入 ARM",
        "请双手都保持张手。输入 ARM",
    )

    def __init__(self, preferred: str = "right"):
        self.preferred = preferred
        self.process: subprocess.Popen | None = None
        self.master_fd: int | None = None
        self.events: queue.Queue[str] = queue.Queue()
        self.logs: deque[str] = deque(maxlen=400)
        self.snapshot = Snapshot(selected=preferred)
        self.ready = False
        self.awaiting_confirmation = False
        self.awaiting_kind = "continue"
        self.teleop_active = False
        self.last_poll = 0.0
        self._raw_tail = ""
        self.error = ""
        self.workflow = {"kind": "", "step": 0, "count": 0, "title": "",
                         "instruction": "", "phase": "", "done": False}
        self._closure_history = {
            "right": deque(maxlen=12), "left": deque(maxlen=12),
        }

    @property
    def running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def start(self) -> None:
        if self.running:
            return
        self.process = None
        self.master_fd = None
        self.error = ""
        self.ready = False
        self.awaiting_confirmation = False
        self.teleop_active = False
        self.snapshot = Snapshot(selected=self.preferred)
        machine = os.uname().machine
        library = LIBRARY_PATHS.get(machine)
        if library is None:
            self.error = f"不支持的平台: {machine}"
            return
        if not BINARY_PATH.exists():
            self.error = "诊断程序尚未编译，请先运行 bash build.sh"
            return
        master_fd, slave_fd = os.openpty()
        command = [str(BINARY_PATH), str(library), self.preferred]
        environment = os.environ.copy()
        # TELEOP 会占用 CLI 命令循环；只在图形上位机中要求 C++ 遥操循环主动
        # 回传只读快照，使 URDF 在输出期间仍实时跟手。普通 CLI 不受影响。
        environment["MHANDPRO_STUDIO_SNAPSHOTS"] = "1"
        try:
            self.process = subprocess.Popen(
                command, cwd=PROJECT_DIR, stdin=slave_fd, stdout=slave_fd,
                stderr=slave_fd, close_fds=True, start_new_session=True,
                env=environment,
            )
        except OSError as exc:
            os.close(master_fd)
            os.close(slave_fd)
            self.error = str(exc)
            return
        os.close(slave_fd)
        self.master_fd = master_fd
        os.set_blocking(master_fd, True)
        threading.Thread(target=self._reader, args=(master_fd,), daemon=True).start()
        self.logs.append("正在连接 mHandPro 接收器……")

    def _reader(self, master_fd: int) -> None:
        decoder_tail = b""
        while self.process is not None and self.process.poll() is None:
            try:
                data = os.read(master_fd, 8192)
            except OSError:
                break
            if not data:
                break
            data = decoder_tail + data
            try:
                text = data.decode("utf-8")
                decoder_tail = b""
            except UnicodeDecodeError as exc:
                text = data[:exc.start].decode("utf-8", errors="replace")
                decoder_tail = data[exc.start:]
            if text:
                self.events.put(text)

    def consume(self) -> None:
        chunks = []
        while True:
            try:
                chunks.append(self.events.get_nowait())
            except queue.Empty:
                break
        if not chunks:
            if self.process is not None and self.process.poll() is not None and not self.error:
                self.error = f"诊断程序已退出（代码 {self.process.returncode}）"
            return
        text = "".join(chunks).replace("\x1b[K", "")
        combined = self._raw_tail + text
        workflow_update = parse_workflow_text(combined)
        if workflow_update["kind"]:
            self.workflow.update(workflow_update)
            self.workflow["done"] = False
        elif workflow_update["phase"] and self.workflow["kind"]:
            self.workflow["phase"] = workflow_update["phase"]
        if "诊断>" in combined:
            self.ready = True
            self.awaiting_confirmation = False
            self.teleop_active = False
            if self.workflow["kind"]:
                self.workflow["done"] = True
        if any(pattern in combined for pattern in self.WAIT_PATTERNS):
            self.awaiting_confirmation = True
            self.awaiting_kind = "arm" if "输入 ARM" in combined else "continue"
            self.ready = False
        if "遥操已启动，输入 STOP" in combined:
            self.teleop_active = True
            self.awaiting_confirmation = False

        # CR 是倒计时/进度刷新，也作为日志行边界；只保留最新的可读状态。
        parts = re.split(r"[\r\n]+", combined)
        self._raw_tail = parts.pop()[-1200:]
        for line in parts:
            clean = re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", line).strip()
            if not clean:
                continue
            snap = parse_snapshot_line(clean)
            if snap is not None:
                for side, state in snap.hands.items():
                    history = self._closure_history[side]
                    if state.valid:
                        history.append(state.closure)
                    if len(history) >= 4:
                        spread = max(
                            max(values[i] for values in history)
                            - min(values[i] for values in history)
                            for i in range(6))
                        state.stability = max(0.0, min(1.0, 1.0 - spread / 0.08))
                self.snapshot = snap
                continue
            if clean in ("snapshot", "snapshot both") or "诊断> snapshot" in clean:
                continue
            self.logs.append(clean[-180:])

    def send(self, command: str, mark_busy: bool = True) -> bool:
        if not self.running or self.master_fd is None:
            self.error = "诊断程序未运行"
            return False
        try:
            os.write(self.master_fd, (command + "\n").encode("utf-8"))
            if mark_busy:
                self.ready = False
                self.awaiting_confirmation = False
                # 不让刚处理完的确认提示在下一块倒计时输出中被重复命中。
                self._raw_tail = ""
            return True
        except OSError as exc:
            self.error = str(exc)
            return False

    def run_action(self, command: str, side: str) -> bool:
        if not self.ready:
            return False
        if side in ("right", "left") and command not in (
                "calibrate", "quickpose", "magcal normal", "magcal"):
            return self.send(f"use {side}\n{command}")
        return self.send(command)

    def poll(self, now: float) -> None:
        if self.ready and now - self.last_poll >= 0.12:
            self.last_poll = now
            # 快照是只读短命令。保持 ready，允许用户操作排在它后面执行；真正的
            # 标定命令仍会立即把 ready 置为 False，防止重复点击。
            self.send("snapshot both", mark_busy=False)

    def close(self) -> None:
        process = self.process
        master_fd = self.master_fd
        if process is None:
            return
        if process.poll() is None:
            try:
                if master_fd is not None:
                    os.write(master_fd, b"n\nSTOP\nquit\n")
                process.wait(timeout=1.0)
            except (OSError, subprocess.TimeoutExpired):
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=1.0)
                except (OSError, subprocess.TimeoutExpired):
                    pass
        if master_fd is not None:
            try:
                os.close(master_fd)
            except OSError:
                pass
        self.process = None
        self.master_fd = None
        self.ready = False
        self.awaiting_confirmation = False
        self.teleop_active = False
        self.snapshot = Snapshot(selected=self.preferred)
        self.logs.append("数据手套已断开。")


class DemoBackend:
    """无硬件界面演示；只生成姿态，不模拟任何设备写入。"""

    def __init__(self):
        self.logs = deque([
            "演示模式：未连接手套，不会执行标定或遥操命令。",
            "拖动模型旋转视角，滚轮缩放；实际运行请去掉 --demo。",
        ], maxlen=400)
        self.snapshot = Snapshot()
        self.ready = True
        self.awaiting_confirmation = False
        self.awaiting_kind = "continue"
        self.teleop_active = False
        self.error = ""
        self.running = True
        self.workflow = {"kind": "", "step": 0, "count": 0, "title": "",
                         "instruction": "", "phase": "", "done": False}

    def start(self) -> None:
        pass

    def consume(self) -> None:
        t = time.monotonic()
        for offset, side in enumerate(("right", "left")):
            phase = t * 0.65 + offset * 1.4
            closure = tuple(0.5 + 0.46 * math.sin(phase + i * 0.42)
                            for i in range(6))
            self.snapshot.hands[side] = HandState(
                valid=True, frame=int(t * 60), frequency=60, power=4.18,
                age_ms=8, sensors_ok=True, calibrated=True,
                thumb_retarget=True, closure=closure,
                gesture=3 if math.sin(phase) > 0.72 else 16,
                virtual_valid=True,
                pinch_mm=3.6 + abs(math.sin(phase)) * 18.0,
                stability=0.92,
            )

    def poll(self, now: float) -> None:
        del now

    def send(self, command: str) -> bool:
        self.logs.append(f"演示模式未执行：{command or '确认'}")
        return True

    def run_action(self, command: str, side: str) -> bool:
        return self.send(f"[{side}] {command}")

    def close(self) -> None:
        pass


class ReplayBackend(DemoBackend):
    """只驱动桌面 URDF 的安全回放；类中没有网络发送或真机 ARM 接口。"""

    def __init__(self, path):
        super().__init__()
        self.frames = [row for row in load_episode(path)
                       if row.get("kind") == "frame" and "glove" in row]
        if not self.frames:
            raise ValueError("记录文件中没有可回放的手套帧")
        self.logs.clear()
        self.logs.append(f"安全回放：{path}（{len(self.frames)} 帧，仅 URDF）")
        self._index = 0
        self._last_advance = time.monotonic()
        self.snapshot = self._snapshot_from_record(self.frames[0]["glove"])

    @staticmethod
    def _snapshot_from_record(raw):
        hands = {}
        for side in ("right", "left"):
            item = (raw.get("hands", {}) or {}).get(side, {})
            allowed = {key: item[key] for key in HandState.__dataclass_fields__
                       if key in item}
            for key in ("closure", "sensor_counts", "fingertips"):
                if key in allowed:
                    allowed[key] = tuple(
                        tuple(v) if isinstance(v, list) else v
                        for v in allowed[key])
            hands[side] = HandState(**allowed)
        return Snapshot(selected=str(raw.get("selected", "right")),
                        disconnected=bool(raw.get("disconnected", False)),
                        hands=hands)

    def consume(self):
        now = time.monotonic()
        if now - self._last_advance < 1.0 / 30.0:
            return
        self._last_advance = now
        self._index = (self._index + 1) % len(self.frames)
        self.snapshot = self._snapshot_from_record(
            self.frames[self._index]["glove"])

    def send(self, command: str) -> bool:
        self.logs.append(f"回放模式禁止执行命令：{command or '确认'}")
        return False


@dataclass
class Button:
    rect: tuple[int, int, int, int]
    label: str
    action: Callable[[], None]
    kind: str = "normal"
    enabled: bool = True

    def hit(self, pos: tuple[int, int]) -> bool:
        x, y, w, h = self.rect
        return self.enabled and x <= pos[0] < x + w and y <= pos[1] < y + h


class MHandStudio:
    BG = (4, 10, 17)
    PANEL = (10, 22, 33, 232)
    PANEL_2 = (13, 29, 43, 245)
    BORDER = (42, 77, 99)
    TEXT = (218, 232, 240)
    MUTED = (120, 148, 166)
    CYAN = (54, 203, 225)
    GREEN = (68, 213, 145)
    AMBER = (245, 181, 65)
    RED = (242, 92, 92)

    def __init__(self, backend, width: int = 1440, height: int = 900):
        import pygame
        from OpenGL.GL import (
            GL_BLEND, GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT, GL_DEPTH_TEST,
            GL_LEQUAL, GL_ONE_MINUS_SRC_ALPHA, GL_SRC_ALPHA, glBlendFunc,
            glClear, glClearColor, glDepthFunc, glEnable,
        )
        self.pg = pygame
        pygame.init()
        pygame.display.gl_set_attribute(pygame.GL_DEPTH_SIZE, 24)
        flags = pygame.OPENGL | pygame.DOUBLEBUF | pygame.RESIZABLE
        self.screen = pygame.display.set_mode((width, height), flags)
        pygame.display.set_caption("mHandPro Motion Studio")
        glClearColor(*(v / 255.0 for v in self.BG), 1.0)
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LEQUAL)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        self.width, self.height = width, height
        self.font = pygame.font.SysFont(
            "notosanscjksc,notosanscjk,wenquanyimicrohei,simhei,monospace", 16)
        self.font_sm = pygame.font.SysFont(
            "notosanscjksc,notosanscjk,wenquanyimicrohei,simhei,monospace", 13)
        self.font_lg = pygame.font.SysFont(
            "notosanscjksc,notosanscjk,wenquanyimicrohei,simhei,monospace", 26, bold=True)
        self.font_md = pygame.font.SysFont(
            "notosanscjksc,notosanscjk,wenquanyimicrohei,simhei,monospace", 18, bold=True)
        self.backend = backend
        self.selected = "both"
        self.command_text = ""
        self.command_focus = False
        self.rotation_x = -13.0
        self.rotation_y = 0.0
        self.zoom = 0.82
        self.dragging = False
        self.last_mouse = (0, 0)
        self.buttons: list[Button] = []
        self._texture = None
        self.renderer = G1UrdfRenderer(URDF_PATH, link_filter=self._is_hand_link)
        self.renderer.load_meshes()
        self._root_transform = self._make_root_transform()
        self.modal: dict | None = None
        self.feedback_receiver = None
        self.feedback_error = ""
        if not isinstance(backend, (DemoBackend, ReplayBackend)):
            try:
                self.feedback_receiver = HandFeedbackReceiver()
            except OSError as exc:
                self.feedback_error = f"真手反馈端口不可用: {exc}"
        self._demo_feedback = RobotFeedback()
        self.recorder = EpisodeRecorder()
        self.last_record_path = None
        self._last_record_sample = 0.0

    @staticmethod
    def _is_hand_link(link: str) -> bool:
        if not link.startswith(("right_", "left_")):
            return False
        return any(part in link for part in (
            "base_link", "palm", "thumb", "index", "middle", "ring",
            "little", "rubber_hand",
        ))

    def _make_root_transform(self):
        import numpy as np
        transforms = self.renderer.model.link_transforms()
        points = [transforms[name][:3, 3] for name in
                  ("right_base_link", "left_base_link")]
        center = (points[0] + points[1]) * 0.5
        # URDF: x前/y左/z上；显示: x横/y上/z朝观察者。
        rotation = np.array([[0., -1., 0.], [0., 0., 1.], [-1., 0., 0.]])
        scale = 2.35
        root = np.eye(4)
        root[:3, :3] = rotation * scale
        root[:3, 3] = -(rotation * scale) @ center
        return root

    def _robot_feedback(self) -> RobotFeedback:
        return (self.feedback_receiver.latest()
                if self.feedback_receiver is not None else self._demo_feedback)

    def _joint_values(self, actual: bool = False) -> dict[str, float]:
        result = {}
        feedback = self._robot_feedback()
        feedback_fresh = feedback.valid and 0.0 <= feedback.age_ms() < 700.0
        for side in ("right", "left"):
            state = self.backend.snapshot.hands[side]
            closure = state.closure if state.valid else hand_mapping.open_closure()
            if actual and feedback_fresh:
                robot = feedback.hands[side]
                if robot.enabled and robot.connected:
                    closure = robot.closure
            angles = hand_mapping.closure_to_angles(closure, side=side)
            result.update(hand_mapping.expand_mimic(angles, side=side))
        return result

    def _draw_3d(self) -> None:
        import numpy as np
        from OpenGL.GL import (
            GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT, GL_DEPTH_TEST, GL_LINES,
            glBegin, glClear, glColor4f, glDisable, glEnable, glEnd,
            glLineWidth, glLoadIdentity, glMatrixMode, glRotatef, glTranslatef,
            glVertex3f, glViewport, GL_MODELVIEW, GL_PROJECTION,
        )
        from OpenGL.GLU import gluPerspective
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        left, top, bottom = 346, 76, 238
        viewport_h = max(1, self.height - top - bottom)
        glViewport(left, bottom, max(1, self.width - left), viewport_h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(42.0, max(1, self.width - left) / viewport_h, 0.02, 10.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glTranslatef(0.0, 0.0, -self.zoom)
        glRotatef(self.rotation_x, 1.0, 0.0, 0.0)
        glRotatef(self.rotation_y, 0.0, 1.0, 0.0)

        glDisable(GL_DEPTH_TEST)
        glLineWidth(1.0)
        glColor4f(0.10, 0.25, 0.32, 0.50)
        glBegin(GL_LINES)
        for i in range(-8, 9):
            v = i * 0.06
            glVertex3f(-0.48, -0.26, v)
            glVertex3f(0.48, -0.26, v)
            glVertex3f(v, -0.26, -0.48)
            glVertex3f(v, -0.26, 0.48)
        glEnd()
        glEnable(GL_DEPTH_TEST)
        active = "right" if self.selected == "both" else self.selected
        feedback = self._robot_feedback()
        has_actual = feedback.valid and 0.0 <= feedback.age_ms() < 700.0 and any(
            hand.enabled and hand.connected for hand in feedback.hands.values())
        if has_actual:
            # 青色半透明 = 手套/映射目标；实体银绿色 = 真手实际角度。
            self.renderer.render(
                np.asarray(self._root_transform), None, None,
                active_side=active, following=True,
                hand_joints=self._joint_values(actual=False),
                tint=(0.12, 0.82, 0.95), alpha=0.24,
            )
            self.renderer.render(
                np.asarray(self._root_transform), None, None,
                active_side=active, following=True,
                hand_joints=self._joint_values(actual=True),
                tint=(0.55, 0.82, 0.68), alpha=1.0,
            )
        else:
            self.renderer.render(
                np.asarray(self._root_transform), None, None,
                active_side=active, following=True,
                hand_joints=self._joint_values(),
            )

    def _text(self, surface, text, pos, color=None, font=None) -> None:
        surface.blit((font or self.font).render(text, True, color or self.TEXT), pos)

    def _panel(self, surface, rect, color=None, radius=10) -> None:
        self.pg.draw.rect(surface, color or self.PANEL, rect, border_radius=radius)
        self.pg.draw.rect(surface, self.BORDER, rect, 1, border_radius=radius)

    def _add_button(self, surface, rect, label, action, kind="normal", enabled=True):
        button = Button(rect, label, action, kind, enabled)
        self.buttons.append(button)
        palette = {
            "normal": ((18, 40, 55), self.CYAN),
            "primary": ((18, 64, 73), self.CYAN),
            "safe": ((19, 57, 47), self.GREEN),
            "warn": ((65, 48, 20), self.AMBER),
            "danger": ((62, 29, 34), self.RED),
        }
        fill, border = palette[kind]
        if not enabled:
            fill, border = (23, 30, 36), (65, 75, 81)
        self.pg.draw.rect(surface, fill, rect, border_radius=7)
        self.pg.draw.rect(surface, border, rect, 1, border_radius=7)
        rendered = self.font.render(label, True, self.TEXT if enabled else self.MUTED)
        surface.blit(rendered, rendered.get_rect(center=self.pg.Rect(rect).center))

    def _run_action(self, command: str) -> None:
        original_command = command
        if self.selected == "both" and command in ("zero", "load", "save", "status"):
            command = (f"use right\n{command}\nuse left\n{command}"
                       "\nuse right")
        if self.backend.run_action(command, self.selected):
            if original_command.startswith(("calibrate", "quickpose", "mapcal", "magcal")):
                self.backend.workflow = {
                    "kind": "starting", "step": 0, "count": 0,
                    "title": "正在准备标定流程",
                    "instruction": "请查看即将出现的姿势说明",
                    "phase": "等待诊断程序响应", "done": False,
                }
                self.modal = {"kind": "workflow", "log_start": len(self.backend.logs)}
            elif original_command.startswith(("status", "linktest")):
                title = ("手套状态检查" if original_command.startswith("status")
                         else "链路质量检查")
                self.modal = {"kind": "diagnostic", "title": title,
                              "log_start": len(self.backend.logs)}
            elif original_command.startswith("teleop"):
                target = ("右手 9103 + 左手 9104" if self.selected == "both"
                          else "右手 9103" if self.selected == "right"
                          else "左手 9104")
                self.backend.workflow = {
                    "kind": "teleop", "step": 0, "count": 0,
                    "title": "启动真机灵巧手映射（100%）",
                    "instruction": (
                        f"输出目标：{target}。请确认 dual_arm_viz/原下游接收端"
                        "已经启动，再在下方输入 ARM。"),
                    "phase": "等待诊断程序给出 ARM 提示", "done": False,
                }
                self.modal = {"kind": "workflow", "log_start": len(self.backend.logs)}
        else:
            self.backend.logs.append("当前流程尚未结束，请先完成或取消当前步骤。")

    def _connect_glove(self) -> None:
        self.backend.start()
        self.modal = {"kind": "diagnostic", "title": "连接数据手套",
                      "log_start": max(0, len(self.backend.logs) - 1)}

    def _disconnect_glove(self) -> None:
        self.backend.close()
        self.modal = None

    def _toggle_recording(self) -> None:
        if self.recorder.recording:
            self.last_record_path = self.recorder.stop()
            self.backend.logs.append(
                f"记录完成：{self.last_record_path}（{self.recorder.frames} 帧，"
                f"丢弃 {self.recorder.dropped}）")
        else:
            path = self.recorder.start({
                "format": "mhandpro_episode_v1",
                "selected": self.selected,
                "mapping": "V5 virtual fingertips（兼容旧 V4）+ OneEuro",
                "safety": "回放文件不包含任何真机发送接口",
            })
            self.backend.logs.append(f"开始记录闭环数据：{path}")

    def _record_snapshot(self, now: float) -> None:
        if not self.recorder.recording or now - self._last_record_sample < 0.03:
            return
        self._last_record_sample = now
        feedback = self._robot_feedback()
        self.recorder.record({
            "kind": "frame",
            "glove": asdict(self.backend.snapshot),
            "robot": asdict(feedback),
            "robot_age_ms": feedback.age_ms(now),
        })

    def _request_teleop(self) -> None:
        if isinstance(self.backend, DemoBackend):
            self.backend.logs.append("演示模式禁止启动 TELEOP 输出。")
            return
        sides = (("right", "left") if self.selected == "both"
                 else (self.selected,))
        missing = [side for side in sides
                   if not (self.backend.snapshot.hands[side].valid
                           and self.backend.snapshot.hands[side].calibrated)]
        if missing:
            names = "、".join("右手" if side == "right" else "左手"
                             for side in missing)
            self.modal = {
                "kind": "message", "title": "不能启动 TELEOP",
                "message": (f"{names}尚未在线并完成六维映射。请先连接数据手套，"
                            "完成 P-pose 和五姿势映射。"),
            }
            return
        configs = (RIGHT_SIM_CONFIG, LEFT_SIM_CONFIG) if self.selected == "both" else (
            RIGHT_SIM_CONFIG if self.selected == "right" else LEFT_SIM_CONFIG,)
        missing_configs = [str(path) for path in configs if not path.is_file()]
        if missing_configs:
            self.modal = {
                "kind": "message", "title": "TELEOP 配置不存在",
                "message": "找不到配置文件：" + "、".join(missing_configs),
            }
            return
        self._run_action(build_teleop_command(self.selected))

    def _draw_header(self, surface) -> None:
        self.pg.draw.rect(surface, (5, 14, 23, 248), (0, 0, self.width, 66))
        self.pg.draw.line(surface, self.BORDER, (0, 65), (self.width, 65), 1)
        self.pg.draw.rect(surface, self.CYAN, (20, 16, 5, 34), border_radius=2)
        self._text(surface, "mHandPro", (38, 10), self.TEXT, self.font_lg)
        self._text(surface, "CALIBRATION · MAPPING · URDF TWIN",
                   (39, 39), self.MUTED, self.font_sm)
        record_label = (f"● 记录中 {self.recorder.frames}"
                        if self.recorder.recording else "一键记录闭环数据")
        self._add_button(surface, (self.width - 540, 15, 194, 36),
                         record_label, self._toggle_recording,
                         "danger" if self.recorder.recording else "normal")
        if isinstance(self.backend, ReplayBackend):
            state, color = "安全回放 · 仅 URDF", self.CYAN
        elif isinstance(self.backend, DemoBackend):
            state, color = "演示模式", self.AMBER
        elif not self.backend.running:
            state, color = "数据手套未连接", self.RED
            self._add_button(surface, (self.width - 330, 15, 166, 36),
                             "连接数据手套", self._connect_glove, "primary")
        else:
            state = "数据手套已连接" if self.backend.ready else "正在连接 / 执行"
            color = self.GREEN if self.backend.ready else self.AMBER
            self._add_button(surface, (self.width - 330, 15, 166, 36),
                             "断开数据手套", self._disconnect_glove, "danger")
        self.pg.draw.circle(surface, color, (self.width - 146, 33), 5)
        self._text(surface, state, (self.width - 134, 22), color, self.font_md)

    def _draw_sidebar(self, surface) -> None:
        self._panel(surface, (12, 78, 318, self.height - 90))
        self._text(surface, "操作对象", (28, 94), self.MUTED, self.font_sm)
        labels = (("right", "右手"), ("left", "左手"), ("both", "双手"))
        for index, (side, label) in enumerate(labels):
            rect = (28 + index * 91, 118, 82, 34)
            kind = "primary" if self.selected == side else "normal"
            self._add_button(surface, rect, label,
                             lambda value=side: setattr(self, "selected", value), kind)

        self._text(surface, "标定与映射", (28, 174), self.MUTED, self.font_sm)
        actions = [
            ("官方 P-pose（在线全部）", "calibrate", "primary"),
            ("快速 P-pose（在线全部）", "quickpose", "normal"),
            ("五姿势映射", "mapcal both" if self.selected == "both" else "mapcal", "primary"),
            ("更新张手零点", "zero", "safe"),
            ("读取标定文件", "load", "normal"),
            ("保存标定文件", "save", "normal"),
        ]
        y = 198
        for label, command, kind in actions:
            self._add_button(surface, (28, y, 286, 34), label,
                             lambda value=command: self._run_action(value), kind,
                             enabled=self.backend.ready)
            y += 38

        self._text(surface, "真机输出", (28, y + 8), self.MUTED, self.font_sm)
        y += 32
        if self.backend.teleop_active:
            self._add_button(surface, (28, y, 286, 36),
                             "STOP · 停止真机映射并张手",
                             lambda: self.backend.send("STOP"), "danger")
        else:
            side_name = {"both": "双手", "right": "右手", "left": "左手"}[
                self.selected]
            self._add_button(surface, (28, y, 286, 36),
                             f"启动{side_name}真机映射（100%）",
                             self._request_teleop, "warn",
                             enabled=self.backend.ready)
        y += 42

        self._text(surface, "诊断与维护", (28, y + 8), self.MUTED, self.font_sm)
        y += 32
        diagnostics = [
            ("状态检查", "status", "normal"),
            ("20 秒链路检测", "linktest 20", "normal"),
        ]
        for label, command, kind in diagnostics:
            self._add_button(surface, (28, y, 286, 34), label,
                             lambda value=command: self._run_action(value), kind,
                             enabled=self.backend.ready)
            y += 38
        for label, command in (
                ("普通磁校准 30s（全部）", "magcal normal"),
                ("深度磁校准 75s（全部）", "magcal")):
            self._add_button(surface, (28, y, 286, 34), label,
                             lambda value=command: self._run_action(value), "warn",
                             enabled=self.backend.ready)
            y += 38

        if self.backend.awaiting_confirmation:
            arm = self.backend.awaiting_kind == "arm"
            self._add_button(surface, (28, self.height - 112, 182, 42),
                             "ARM 并启动遥操" if arm else "确认姿势 / 继续",
                             lambda: self.backend.send("ARM" if arm else ""),
                             "warn" if arm else "safe")
            self._add_button(surface, (218, self.height - 112, 96, 42),
                             "取消", lambda: self.backend.send("n"), "danger")

    def _draw_hand_card(self, surface, side: str,
                        rect: tuple[int, int, int, int]) -> None:
        state = self.backend.snapshot.hands[side]
        x, y, w, h = rect
        self._panel(surface, rect, self.PANEL_2)
        title = "RIGHT / 右手" if side == "right" else "LEFT / 左手"
        status, severity = classify_hand_state(
            state, disconnected=self.backend.snapshot.disconnected)
        color = {"ok": self.GREEN, "warning": self.AMBER,
                 "error": self.RED}[severity]
        self.pg.draw.circle(surface, color, (x + 18, 101), 5)
        self._text(surface, title, (x + 30, 89), self.TEXT, self.font_md)
        self._text(surface, status, (x + 18, 121), color, self.font_sm)
        self._text(surface, f"{state.frequency} Hz  ·  {state.age_ms} ms  ·  电量 {state.power:.2f}",
                   (x + 18, 142), self.MUTED, self.font_sm)
        if state.thumb_virtual:
            mapping = "V5 虚拟指尖映射"
        elif state.thumb_retarget:
            mapping = "V4 指腹映射"
        else:
            mapping = "六维映射就绪" if state.calibrated else "尚未完成映射"
        self._text(surface, mapping, (x + 18, 164),
                   self.CYAN if state.calibrated else self.AMBER, self.font_sm)
        gesture = GESTURE_NAMES.get(state.gesture, f"手势 {state.gesture}")
        pinch = f" · 指尖 {state.pinch_mm:.1f} mm" if state.pinch_mm >= 0 else ""
        self._text(surface,
                   f"{gesture}{pinch} · 稳定 {state.stability:.0%}",
                   (x + 18, 184), self.MUTED, self.font_sm)
        feedback = self._robot_feedback()
        robot = feedback.hands[side]
        if feedback.valid and feedback.age_ms() < 700 and robot.enabled:
            robot_color = (self.GREEN if robot.connected and robot.state != "FAULT"
                           else self.RED)
            robot_text = (f"真手 {robot.state} · 力 {robot.max_force} · "
                          f"温度 {robot.max_temperature}°C")
        else:
            robot_color = self.MUTED
            robot_text = "真手反馈：等待机器人端"
        self._text(surface, robot_text, (x + 18, 204), robot_color, self.font_sm)

    def _draw_chain_status(self, surface) -> None:
        x, y, w, h = top_status_layout(self.width)["chain"]
        self._panel(surface, (x, y, w, h), (7, 20, 30, 244), 9)
        sides = ("right", "left") if self.selected == "both" else (self.selected,)
        glove_ok = all(self.backend.snapshot.hands[s].valid
                       and self.backend.snapshot.hands[s].age_ms < 500 for s in sides)
        mapping_ok = all(self.backend.snapshot.hands[s].calibrated for s in sides)
        feedback = self._robot_feedback()
        robot_ok = feedback.valid and feedback.age_ms() < 700
        modbus_ok = robot_ok and all(
            feedback.hands[s].enabled and feedback.hands[s].connected for s in sides)
        actual_ok = modbus_ok and all(
            feedback.hands[s].feedback_age_ms >= 0
            and feedback.hands[s].feedback_age_ms < 700
            and feedback.hands[s].state != "FAULT" for s in sides)
        stages = (
            ("手套传感", glove_ok), ("映射就绪", mapping_ok),
            ("TELEOP 输出", self.backend.teleop_active),
            ("机器人接收", robot_ok), ("真手跟随", actual_ok),
        )
        columns = 2 if w >= 360 else 1
        rows = (len(stages) + columns - 1) // columns
        gap_x, gap_y = 8, 5
        chip_w = max(90, (w - 24 - gap_x * (columns - 1)) // columns)
        chip_h = max(20, (h - 18 - gap_y * (rows - 1)) // rows)
        for index, (label, ok) in enumerate(stages):
            row, column = divmod(index, columns)
            cx = x + 12 + column * (chip_w + gap_x)
            cy = y + 9 + row * (chip_h + gap_y)
            color = self.GREEN if ok else (71, 91, 103)
            self.pg.draw.rect(surface, (13, 35, 43), (cx, cy, chip_w, chip_h),
                              border_radius=max(8, chip_h // 2))
            self.pg.draw.circle(surface, color, (cx + 13, cy + chip_h // 2), 4)
            self._text(surface, label, (cx + 24, cy + max(1, (chip_h - 17) // 2)),
                       self.TEXT if ok else self.MUTED, self.font_sm)

    def _draw_channels(self, surface) -> None:
        x = 356
        y = self.height - 222
        w = self.width - x - 14
        self._panel(surface, (x, y, w, 210))
        self._text(surface, "六维闭环 · 目标青色 / 真手绿色 / 接触橙色",
                   (x + 18, y + 13), self.TEXT, self.font_md)
        card_w = (w - 52) // 2
        feedback = self._robot_feedback()
        feedback_fresh = feedback.valid and feedback.age_ms() < 700
        for side_index, side in enumerate(("right", "left")):
            base_x = x + 18 + side_index * (card_w + 16)
            state = self.backend.snapshot.hands[side]
            robot = feedback.hands[side]
            actual = (robot.closure if feedback_fresh and robot.connected
                      else (0.0,) * 6)
            self._text(surface, "右手" if side == "right" else "左手",
                       (base_x, y + 44), self.CYAN, self.font_sm)
            for i, (name, value) in enumerate(zip(CHANNEL_NAMES, state.closure)):
                row_y = y + 66 + i * 21
                self._text(surface, name, (base_x, row_y - 2), self.MUTED, self.font_sm)
                bar_x = base_x + 72
                bar_w = max(55, card_w - 118)
                self.pg.draw.rect(surface, (20, 38, 49), (bar_x, row_y, bar_w, 10), border_radius=5)
                self.pg.draw.rect(surface, self.CYAN,
                                  (bar_x, row_y, int(bar_w * value), 4), border_radius=2)
                actual_color = self.AMBER if robot.contact[i] else self.GREEN
                self.pg.draw.rect(surface, actual_color,
                                  (bar_x, row_y + 6, int(bar_w * actual[i]), 4),
                                  border_radius=2)
                value_text = (f"{value:3.0%}/{actual[i]:3.0%}"
                              if feedback_fresh and robot.connected
                              else f"{value:4.0%}/ --")
                self._text(surface, value_text,
                           (bar_x + bar_w + 7, row_y - 5), self.TEXT, self.font_sm)

    def _draw_console(self, surface) -> None:
        # 日志放在 3D 视图下沿上方，保留最新几行；高级命令输入覆盖所有 CLI 功能。
        x, y = 356, self.height - 342
        w = self.width - x - 14
        self._panel(surface, (x, y, w, 108), (7, 18, 28, 232))
        lines = list(self.backend.logs)[-3:]
        for i, line in enumerate(lines):
            color = self.RED if any(word in line for word in ("失败", "错误", "异常")) else self.MUTED
            self._text(surface, line[:110], (x + 15, y + 11 + i * 20), color, self.font_sm)
        input_rect = (x + 12, y + 73, w - 24, 25)
        self.pg.draw.rect(surface, (3, 11, 18), input_rect, border_radius=4)
        self.pg.draw.rect(surface, self.CYAN if self.command_focus else self.BORDER,
                          input_rect, 1, border_radius=4)
        prompt = self.command_text or "高级命令（例如 teleop config/inspire_left_sim.cfg）"
        self._text(surface, "> " + prompt, (x + 20, y + 76),
                   self.TEXT if self.command_text else self.MUTED, self.font_sm)
        self.command_rect = input_rect

    def _wrapped_lines(self, text: str, font, max_width: int) -> list[str]:
        lines = []
        for paragraph in str(text).splitlines() or [""]:
            current = ""
            for char in paragraph:
                candidate = current + char
                if current and font.size(candidate)[0] > max_width:
                    lines.append(current)
                    current = char
                else:
                    current = candidate
            if current:
                lines.append(current)
        return lines

    def _draw_modal_logs(self, surface, rect, start: int, limit: int = 12) -> None:
        x, y, w, h = rect
        self.pg.draw.rect(surface, (4, 13, 22), rect, border_radius=8)
        self.pg.draw.rect(surface, self.BORDER, rect, 1, border_radius=8)
        lines = list(self.backend.logs)[start:]
        visible = lines[-limit:]
        line_h = 21
        for index, line in enumerate(visible):
            if index * line_h > h - 26:
                break
            color = (self.RED if any(word in line for word in ("失败", "错误", "异常"))
                     else self.GREEN if any(word in line for word in ("成功", "合格", "正常"))
                     else self.MUTED)
            self._text(surface, line[:118], (x + 14, y + 9 + index * line_h),
                       color, self.font_sm)

    def _draw_modal(self, surface) -> None:
        if self.modal is None:
            return
        # 截断底层所有点击；后加入的卡片按钮在事件反向遍历时优先命中。
        self.buttons.append(Button((0, 0, self.width, self.height), "", lambda: None))
        self.pg.draw.rect(surface, (1, 5, 9, 220), (0, 66, self.width, self.height - 66))
        kind = self.modal.get("kind")
        box_w = min(920, self.width - 70)
        box_h = min(590, self.height - 105)
        box_x = (self.width - box_w) // 2
        box_y = 78 + max(0, (self.height - 78 - box_h) // 2)
        self._panel(surface, (box_x, box_y, box_w, box_h), (8, 21, 34, 252), 18)
        self.pg.draw.rect(surface, self.CYAN, (box_x, box_y, 6, box_h),
                          border_radius=3)

        if kind == "workflow":
            flow = self.backend.workflow
            self._text(surface, "CALIBRATION WORKFLOW",
                       (box_x + 32, box_y + 24), self.CYAN, self.font_sm)
            step, count = int(flow.get("step", 0)), int(flow.get("count", 0))
            step_text = (f"步骤 {step} / {count}" if count else "正在建立标定流程")
            self._text(surface, step_text,
                       (box_x + box_w - 32 - self.font_sm.size(step_text)[0], box_y + 24),
                       self.MUTED, self.font_sm)
            if count:
                dot_y = box_y + 66
                span = box_w - 80
                spacing = span / max(1, count - 1)
                for i in range(count):
                    cx = int(box_x + 40 + i * spacing)
                    color = self.GREEN if i + 1 < step else self.CYAN if i + 1 == step else (61, 78, 94)
                    if i < count - 1:
                        self.pg.draw.line(surface, color, (cx + 7, dot_y),
                                          (int(box_x + 40 + (i + 1) * spacing) - 7, dot_y), 2)
                    self.pg.draw.circle(surface, color, (cx, dot_y), 7)
            self._text(surface, flow.get("title") or "请稍候",
                       (box_x + 32, box_y + 92), self.TEXT, self.font_lg)
            instruction = flow.get("instruction") or "正在读取姿势说明……"
            for i, line in enumerate(self._wrapped_lines(instruction, self.font, box_w - 64)[:3]):
                self._text(surface, line, (box_x + 32, box_y + 140 + i * 27),
                           self.TEXT, self.font)
            phase = flow.get("phase") or (
                "姿势准备好后点击下方按钮" if self.backend.awaiting_confirmation
                else "正在执行，请保持当前姿势")
            phase_color = self.GREEN if flow.get("done") else self.AMBER
            self._panel(surface, (box_x + 32, box_y + 222, box_w - 64, 52),
                        (18, 39, 48, 245), 8)
            self._text(surface, "流程已完成，可查看下面结果" if flow.get("done") else phase,
                       (box_x + 50, box_y + 237), phase_color, self.font_md)
            watch_side = "right" if self.selected == "both" else self.selected
            watch = self.backend.snapshot.hands[watch_side]
            gesture_text = GESTURE_NAMES.get(watch.gesture,
                                             f"手势 {watch.gesture}")
            quality_text = f"识别 {gesture_text} · 稳定度 {watch.stability:.0%}"
            self._text(surface, quality_text,
                       (box_x + box_w - 300, box_y + 241),
                       self.GREEN if watch.stability >= 0.75 else self.AMBER,
                       self.font_sm)
            stable_w = int((box_w - 64) * max(0.0, min(1.0, watch.stability)))
            self.pg.draw.rect(surface, (20, 39, 49),
                              (box_x + 32, box_y + 280, box_w - 64, 5),
                              border_radius=2)
            self.pg.draw.rect(surface, self.GREEN if watch.stability >= 0.75 else self.AMBER,
                              (box_x + 32, box_y + 280, stable_w, 5),
                              border_radius=2)
            self._draw_modal_logs(
                surface, (box_x + 32, box_y + 292, box_w - 64, box_h - 376),
                self.modal.get("log_start", 0), limit=9)
            button_y = box_y + box_h - 64
            if self.backend.teleop_active and flow.get("kind") == "teleop":
                self._add_button(surface, (box_x + 32, button_y, box_w - 330, 42),
                                 "返回主界面查看实时双手",
                                 lambda: setattr(self, "modal", None), "primary")
                self._add_button(surface, (box_x + box_w - 282, button_y, 250, 42),
                                 "STOP · 停止并张手",
                                 lambda: self.backend.send("STOP"), "danger")
            elif self.backend.awaiting_confirmation:
                arm = self.backend.awaiting_kind == "arm"
                self._add_button(surface, (box_x + 32, button_y, box_w - 184, 42),
                                 "ARM 并启动" if arm else "姿势已准备好 · 开始采集",
                                 lambda: self.backend.send("ARM" if arm else ""),
                                 "warn" if arm else "safe")
                self._add_button(surface, (box_x + box_w - 136, button_y, 104, 42),
                                 "取消流程", lambda: self.backend.send("n"), "danger")
            elif flow.get("done"):
                self._add_button(surface, (box_x + box_w - 180, button_y, 148, 42),
                                 "完成并关闭", lambda: setattr(self, "modal", None), "primary")
            else:
                self._text(surface, "采集中请勿移动 · 等待下一步提示",
                           (box_x + 32, button_y + 10), self.MUTED, self.font)

        elif kind == "diagnostic":
            title = self.modal.get("title", "诊断信息")
            self._text(surface, "DIAGNOSTIC REPORT", (box_x + 32, box_y + 24),
                       self.CYAN, self.font_sm)
            self._text(surface, title, (box_x + 32, box_y + 58), self.TEXT, self.font_lg)
            self._text(surface, "完整结果会保留在此面板，检查过程中也可随时关闭。",
                       (box_x + 32, box_y + 102), self.MUTED, self.font)
            self._draw_modal_logs(
                surface, (box_x + 32, box_y + 140, box_w - 64, box_h - 222),
                self.modal.get("log_start", 0), limit=16)
            self._add_button(surface, (box_x + box_w - 180, box_y + box_h - 64, 148, 42),
                             "关闭检查面板", lambda: setattr(self, "modal", None), "primary")

        else:
            self._text(surface, self.modal.get("title", "提示"),
                       (box_x + 32, box_y + 38), self.TEXT, self.font_lg)
            for i, line in enumerate(self._wrapped_lines(
                    self.modal.get("message", ""), self.font, box_w - 64)[:8]):
                self._text(surface, line, (box_x + 32, box_y + 106 + i * 28),
                           self.MUTED, self.font)
            self._add_button(surface, (box_x + box_w - 160, box_y + box_h - 64, 128, 42),
                             "知道了", lambda: setattr(self, "modal", None), "primary")

    def _draw_overlay(self):
        surface = self.pg.Surface((self.width, self.height), self.pg.SRCALPHA)
        self.buttons = []
        self._draw_header(surface)
        self._draw_sidebar(surface)
        layout = top_status_layout(self.width)
        self._draw_hand_card(surface, "right", layout["right_card"])
        self._draw_hand_card(surface, "left", layout["left_card"])
        self._draw_chain_status(surface)
        self._draw_console(surface)
        self._draw_channels(surface)
        if self.backend.error:
            self._panel(surface, (self.width - 500, 14, 350, 40), (62, 25, 30, 245), 7)
            self._text(surface, self.backend.error[:42], (self.width - 484, 23), self.RED, self.font_sm)
        elif self.feedback_error:
            self._panel(surface, (self.width - 500, 64, 350, 34),
                        (62, 25, 30, 245), 7)
            self._text(surface, self.feedback_error[:42],
                       (self.width - 484, 72), self.RED, self.font_sm)
        self._draw_modal(surface)
        self._blit_surface(surface)

    def _blit_surface(self, surface) -> None:
        from OpenGL.GL import (
            GL_BLEND, GL_DEPTH_TEST, GL_LINEAR, GL_MODELVIEW, GL_ONE_MINUS_SRC_ALPHA,
            GL_PROJECTION, GL_QUADS, GL_RGBA, GL_SRC_ALPHA, GL_TEXTURE_2D,
            GL_TEXTURE_MAG_FILTER, GL_TEXTURE_MIN_FILTER, GL_UNSIGNED_BYTE,
            glBegin, glBindTexture, glBlendFunc, glColor4f, glDisable, glEnable,
            glEnd, glGenTextures, glLoadIdentity, glMatrixMode, glOrtho, glTexCoord2f,
            glTexImage2D, glTexParameteri, glVertex2f, glViewport,
        )
        if self._texture is None:
            self._texture = glGenTextures(1)
        data = self.pg.image.tostring(surface, "RGBA", True)
        glViewport(0, 0, self.width, self.height)
        glDisable(GL_DEPTH_TEST)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glEnable(GL_TEXTURE_2D)
        glBindTexture(GL_TEXTURE_2D, self._texture)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, self.width, self.height, 0,
                     GL_RGBA, GL_UNSIGNED_BYTE, data)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(0, self.width, 0, self.height, -1, 1)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glColor4f(1, 1, 1, 1)
        glBegin(GL_QUADS)
        glTexCoord2f(0, 0); glVertex2f(0, 0)
        glTexCoord2f(1, 0); glVertex2f(self.width, 0)
        glTexCoord2f(1, 1); glVertex2f(self.width, self.height)
        glTexCoord2f(0, 1); glVertex2f(0, self.height)
        glEnd()
        glDisable(GL_TEXTURE_2D)
        glEnable(GL_DEPTH_TEST)

    def _submit_command(self) -> None:
        command = self.command_text.strip()
        if command:
            self.backend.send(command)
            self.command_text = ""

    def _handle_event(self, event) -> bool:
        pg = self.pg
        if event.type == pg.QUIT:
            return False
        if event.type == pg.VIDEORESIZE:
            self.width, self.height = max(1100, event.w), max(820, event.h)
        elif event.type == pg.MOUSEBUTTONDOWN:
            if event.button == 1:
                self.command_focus = hasattr(self, "command_rect") and pg.Rect(self.command_rect).collidepoint(event.pos)
                for button in reversed(self.buttons):
                    if button.hit(event.pos):
                        button.action()
                        return True
                if event.pos[0] > 346 and 205 < event.pos[1] < self.height - 350:
                    self.dragging = True
                    self.last_mouse = event.pos
            elif event.button == 4:
                self.zoom = max(0.45, self.zoom - 0.06)
            elif event.button == 5:
                self.zoom = min(1.6, self.zoom + 0.06)
        elif event.type == pg.MOUSEBUTTONUP and event.button == 1:
            self.dragging = False
        elif event.type == pg.MOUSEMOTION and self.dragging:
            dx = event.pos[0] - self.last_mouse[0]
            dy = event.pos[1] - self.last_mouse[1]
            self.rotation_y += dx * 0.45
            self.rotation_x = max(-80, min(80, self.rotation_x + dy * 0.35))
            self.last_mouse = event.pos
        elif event.type == pg.KEYDOWN:
            if event.key == pg.K_ESCAPE:
                if self.modal is not None and self.backend.awaiting_confirmation:
                    self.backend.send("n")
                elif self.modal is not None:
                    self.modal = None
                else:
                    return False
            elif event.key == pg.K_RETURN:
                if (self.modal is not None and self.backend.awaiting_confirmation
                        and not self.command_text):
                    reply = "ARM" if self.backend.awaiting_kind == "arm" else ""
                    self.backend.send(reply)
                elif self.modal is not None:
                    return True
                else:
                    self._submit_command()
            elif event.key == pg.K_BACKSPACE:
                self.command_text = self.command_text[:-1]
            elif event.unicode and event.unicode.isprintable():
                self.command_focus = True
                self.command_text += event.unicode
        return True

    def run(self, max_frames: int = 0) -> None:
        clock = self.pg.time.Clock()
        running = True
        frames = 0
        try:
            while running:
                for event in self.pg.event.get():
                    running = self._handle_event(event) and running
                self.backend.consume()
                self.backend.poll(time.monotonic())
                self._record_snapshot(time.monotonic())
                self._draw_3d()
                self._draw_overlay()
                self.pg.display.flip()
                frames += 1
                if max_frames and frames >= max_frames:
                    break
                clock.tick(60)
        finally:
            if self.recorder.recording:
                self.last_record_path = self.recorder.stop()
            if self.feedback_receiver is not None:
                self.feedback_receiver.close()
            self.backend.close()
            self.pg.quit()


def self_check() -> None:
    sample = (SNAPSHOT_PREFIX
              + '{"selected":"right","disconnected":false,"hands":{'
                '"right":{"valid":true,"frame":1,"frequency":60,"power":4.2,'
                '"age_ms":8,"sensors_ok":true,"calibrated":true,'
                '"thumb_retarget":true,"closure":[0,0.2,0.4,0.6,0.8,1]},'
                '"left":{"valid":false,"closure":[0,0,0,0,0,0]}}}')
    snapshot = parse_snapshot_line(sample)
    if snapshot is None or snapshot.hands["right"].closure[-1] != 1.0:
        raise RuntimeError("snapshot 解析自检失败")
    renderer = G1UrdfRenderer(URDF_PATH, link_filter=MHandStudio._is_hand_link)
    hand_visuals = [v for v in renderer.model.visuals if MHandStudio._is_hand_link(v.link)]
    if not hand_visuals:
        raise RuntimeError("URDF 中没有找到 Inspire 手部网格")
    joints = {}
    for side in ("right", "left"):
        joints.update(hand_mapping.expand_mimic(
            hand_mapping.closure_to_angles((0.5,) * 6, side=side), side=side))
    transforms = renderer.model.link_transforms(joints)
    if not all(name in transforms for name in ("right_thumb_4", "left_thumb_4")):
        raise RuntimeError("URDF 手部 FK 自检失败")
    print(f"自检通过：{len(hand_visuals)} 个手部 visual，{len(joints)} 个活动关节")


def main() -> None:
    parser = argparse.ArgumentParser(description="mHandPro 图形标定与映射上位机")
    parser.add_argument("--side", choices=("right", "left"), default="right",
                        help="两只手套在线时的默认操作侧")
    parser.add_argument("--demo", action="store_true",
                        help="不连接硬件，仅演示双手 URDF 和界面")
    parser.add_argument("--replay", metavar="EPISODE.jsonl",
                        help="安全回放闭环记录，仅驱动桌面 URDF，不发送真机命令")
    parser.add_argument("--check", action="store_true",
                        help="只检查快照协议、映射和 URDF 后退出")
    parser.add_argument("--frames", type=int, default=0, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.check:
        self_check()
        return
    if args.replay:
        backend = ReplayBackend(args.replay)
    else:
        backend = DemoBackend() if args.demo else DiagnosticBackend(args.side)
    MHandStudio(backend).run(max_frames=max(0, args.frames))


if __name__ == "__main__":
    main()
