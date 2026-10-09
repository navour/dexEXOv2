#!/usr/bin/env python3
"""Unitree G1 双臂 MuJoCo 运动学仿真接收器。

直接加载宇树官方 ``g1_dual_arm.urdf``，监听与 PC/树莓派发送端
相同的 UARM UDP 数据包，将左右双臂关节角映射到 MuJoCo。

第一阶段只验证运动学：固定躯干，直接更新 qpos，不计算电机力矩。
安全行为与真机接收端保持一致：关节限位、每臂独立模式、速度限制、
UDP 超时后缓慢回零。
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import math
from pathlib import Path
import re
import socket
import struct
import sys
import time
from typing import Any


PACKET_HEADER_V2 = b"UA2M"
PACKET_FMT_V2 = "!4sBIQ20f"
PACKET_SIZE_V2 = struct.calcsize(PACKET_FMT_V2)

WAIST_HEADER = b"UAWS"
WAIST_FMT = "!4sB3f3f"
WAIST_SIZE = struct.calcsize(WAIST_FMT)

# 灵巧手尾块，跟在腰部尾块后面。载荷是 6 通道闭合度（0=张开，1=握紧），
# 通道顺序见 hand_mapping。只有带手的模型才用得上，无手模型自动跳过。
HAND_HEADER = b"UHND"
HAND_FMT = "!4sB12f"
HAND_SIZE = struct.calcsize(HAND_FMT)
HAND_CHANNEL_COUNT = 6
HAND_FLAG_RIGHT = 0x1
HAND_FLAG_LEFT = 0x2

PACKET_HEADER = b"UARM"
PACKET_FMT_SINGLE = "!4sBffff"
PACKET_FMT_DUAL = "!4sBffffffff"
PACKET_FMT_DUAL_WR = "!4sBffffffffff"
PACKET_SIZE_SINGLE = struct.calcsize(PACKET_FMT_SINGLE)
PACKET_SIZE_DUAL = struct.calcsize(PACKET_FMT_DUAL)
PACKET_SIZE_DUAL_WR = struct.calcsize(PACKET_FMT_DUAL_WR)

DEFAULT_PORT = 9527
DEFAULT_CONTROL_HZ = 250.0
DEFAULT_TIMEOUT_SEC = 1.0
DEFAULT_FOLLOW_SPEED = 5.0
DEFAULT_RETURN_SPEED = 0.3

RIGHT_JOINTS = (
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
)
LEFT_JOINTS = (
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
)
LOCKED_WRIST_JOINTS = (
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)
CONTROLLED_JOINTS = RIGHT_JOINTS + LEFT_JOINTS
ALL_ARM_JOINTS = CONTROLLED_JOINTS + LOCKED_WRIST_JOINTS

# 腰：只驱动 yaw，roll/pitch 恒定回中，与真机接收端一致。上半身模型没有
# 这个关节，此时整段腰部逻辑自动跳过。
WAIST_YAW_JOINT = "waist_yaw_joint"
WAIST_CENTRED_JOINTS = ("waist_roll_joint", "waist_pitch_joint")
DEFAULT_WAIST_SPEED = 1.5

# 全身模型的固定站姿。这仍是运动学回放，腿不参与任何计算，只是摆个造型；
# 取值与 HumDex_IMU融合实验 的 DEFAULT_JOINTS 一致（微屈膝的站立姿态）。
STANDING_LEG_POSE = {
    "left_hip_pitch_joint": -0.2,
    "left_knee_joint": 0.4,
    "left_ankle_pitch_joint": -0.2,
    "right_hip_pitch_joint": -0.2,
    "right_knee_joint": 0.4,
    "right_ankle_pitch_joint": -0.2,
}
# 浮动基座高度，取自 MJCF 里 pelvis 的初始 pos。
FLOATING_BASE_HEIGHT = 0.793

MODE_NAMES = {
    0: "双臂回零",
    1: "右臂跟随",
    2: "左臂跟随",
    3: "双臂跟随",
}


@dataclass(frozen=True)
class ArmCommand:
    """双臂数据包解析结果，关节角单位为 rad。"""

    mode: int
    right: tuple[float, float, float, float, float]
    left: tuple[float, float, float, float, float]
    version: int = 1
    seq: int | None = None
    sender_timestamp_us: int | None = None
    velocities: tuple[float, ...] = (0.0,) * 10
    # 腰部 yaw/roll/pitch，发送端未附带尾块或未启用时为 None。
    waist: tuple[float, float, float] | None = None
    # 灵巧手 6 通道闭合度；{"right": 元组或 None, "left": ...}，无尾块时为 None。
    hands: dict[str, tuple[float, ...] | None] | None = None


def unpack_arm_command(data: bytes) -> ArmCommand | None:
    """解析新版 UA2M 和历史 UARM 数据包。

    返回 None 表示包头、模式或数值非法。
    """
    if len(data) >= PACKET_SIZE_V2 and data[:4] == PACKET_HEADER_V2:
        values = struct.unpack(PACKET_FMT_V2, data[:PACKET_SIZE_V2])
        header, mode, seq, sender_timestamp_us, *payload = values
        positions = tuple(payload[:10])
        velocities = tuple(payload[10:20])
        right = positions[:5]
        left = positions[5:10]
        version = 2
        waist = unpack_waist_block(data[PACKET_SIZE_V2:])
        hands = unpack_hand_block(data[PACKET_SIZE_V2:])
    elif len(data) >= PACKET_SIZE_DUAL_WR:
        values = struct.unpack(PACKET_FMT_DUAL_WR, data[:PACKET_SIZE_DUAL_WR])
        header, mode, *angles = values
        right = tuple(angles[:5])
        left = tuple(angles[5:10])
        version = 1
        seq = None
        sender_timestamp_us = None
        velocities = (0.0,) * 10
        waist = None
        hands = None
    elif len(data) >= PACKET_SIZE_DUAL:
        values = struct.unpack(PACKET_FMT_DUAL, data[:PACKET_SIZE_DUAL])
        header, mode, *angles = values
        right = tuple(angles[:4]) + (0.0,)
        left = tuple(angles[4:8]) + (0.0,)
        version = 1
        seq = None
        sender_timestamp_us = None
        velocities = (0.0,) * 10
        waist = None
        hands = None
    elif len(data) >= PACKET_SIZE_SINGLE:
        values = struct.unpack(PACKET_FMT_SINGLE, data[:PACKET_SIZE_SINGLE])
        header, mode, *angles = values
        right = tuple(angles[:4]) + (0.0,)
        left = (0.0, 0.0, 0.0, 0.0, 0.0)
        version = 1
        seq = None
        sender_timestamp_us = None
        velocities = (0.0,) * 10
        waist = None
        hands = None
    else:
        return None

    expected_header = PACKET_HEADER_V2 if version == 2 else PACKET_HEADER
    if header != expected_header or mode not in MODE_NAMES:
        return None
    if not all(math.isfinite(value) for value in right + left):
        return None
    if not all(math.isfinite(value) and abs(value) <= 100.0
               for value in velocities):
        return None
    return ArmCommand(
        mode=mode,
        right=right,
        left=left,
        version=version,
        seq=seq,
        sender_timestamp_us=sender_timestamp_us,
        velocities=velocities,
        waist=waist,
        hands=hands,
    )


def unpack_waist_block(tail: bytes) -> tuple[float, float, float] | None:
    """解析 UAWS 腰部尾块；缺失、包头不符、未启用或数值离谱时返回 None。

    尾块坏掉只丢腰、不丢整包 —— 丢整包等于双臂也停。
    """
    block = find_tail_block(tail, WAIST_HEADER, WAIST_SIZE)
    if block is None:
        return None
    try:
        values = struct.unpack(WAIST_FMT, block[:WAIST_SIZE])
    except struct.error:
        return None
    if not int(values[1]):
        return None
    waist = tuple(float(v) for v in values[2:5])
    if not all(math.isfinite(v) and abs(v) <= 3.2 for v in waist):
        return None
    return waist


def find_tail_block(tail: bytes, header: bytes, size: int) -> bytes | None:
    """在尾块区里按块走，找到指定包头的那一块。

    尾块自带包头、定长、可选，所以按块前进而不是固定偏移 —— 发送端将来
    调整尾块顺序时接收端不会读错。走到不认识的包头就停、不做搜索：宁可
    当作没有，也不能把随机字节当成手部指令。
    """
    known = ((WAIST_HEADER, WAIST_SIZE), (HAND_HEADER, HAND_SIZE))
    offset = 0
    while offset + 4 <= len(tail):
        magic = tail[offset:offset + 4]
        for known_header, known_size in known:
            if magic != known_header:
                continue
            if offset + known_size > len(tail):
                return None
            if magic == header:
                return tail[offset:offset + size]
            offset += known_size
            break
        else:
            return None
    return None


def unpack_hand_block(
    tail: bytes,
) -> dict[str, tuple[float, ...] | None] | None:
    """解析 UHND 灵巧手尾块；缺失或全部非法时返回 None。

    某只手标志位没置位、通道越界时这只手是 None，消费端应把它当"没有
    数据"张开处理，而不是沿用上一帧 —— 手上没数据时张开是安全的，握着
    不放不是。坏掉的手不连累另一只手，更不连累双臂。
    """
    block = find_tail_block(tail, HAND_HEADER, HAND_SIZE)
    if block is None:
        return None
    try:
        values = struct.unpack(HAND_FMT, block[:HAND_SIZE])
    except struct.error:
        return None

    flags = int(values[1])
    channels = values[2:2 + 2 * HAND_CHANNEL_COUNT]

    def side(flag: int, start: int) -> tuple[float, ...] | None:
        if not flags & flag:
            return None
        data = tuple(float(v)
                     for v in channels[start:start + HAND_CHANNEL_COUNT])
        if not all(math.isfinite(v) and 0.0 <= v <= 1.0 for v in data):
            return None
        return data

    right = side(HAND_FLAG_RIGHT, 0)
    left = side(HAND_FLAG_LEFT, HAND_CHANNEL_COUNT)
    if right is None and left is None:
        return None
    return {"right": right, "left": left}


class UdpCommandReceiver:
    """非阻塞 UDP 接收器，每个控制周期只使用最新的有效包。"""

    def __init__(self, bind_host: str, port: int) -> None:
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((bind_host, port))
        self.socket.setblocking(False)
        self.valid_packets = 0
        self.invalid_packets = 0
        self.last_source: tuple[str, int] | None = None
        self.last_version: int | None = None
        self.last_seq: int | None = None
        self.lost_packets = 0
        self.duplicate_packets = 0
        self.out_of_order_packets = 0
        self.last_transport_age_ms: float | None = None
        self._recent_receive_times: deque[float] = deque()

    def close(self) -> None:
        self.socket.close()

    @property
    def receive_hz(self) -> float:
        now = time.monotonic()
        while (
            self._recent_receive_times
            and now - self._recent_receive_times[0] > 2.0
        ):
            self._recent_receive_times.popleft()
        if len(self._recent_receive_times) < 2:
            return 0.0
        duration = (
            self._recent_receive_times[-1] - self._recent_receive_times[0]
        )
        if duration <= 0.0:
            return 0.0
        return (len(self._recent_receive_times) - 1) / duration

    def _update_v2_stats(self, command: ArmCommand, received_at: float) -> None:
        if command.seq is not None:
            if self.last_seq is not None:
                delta = (command.seq - self.last_seq) & 0xFFFFFFFF
                if delta == 0:
                    self.duplicate_packets += 1
                elif delta < 0x80000000:
                    self.lost_packets += max(0, delta - 1)
                    self.last_seq = command.seq
                else:
                    self.out_of_order_packets += 1
            else:
                self.last_seq = command.seq

        if command.sender_timestamp_us is not None:
            age_ms = (
                received_at * 1_000_000 - command.sender_timestamp_us
            ) / 1000.0
            # monotonic 时钟只在发送端与仿真器位于同一台主机时可比较。
            self.last_transport_age_ms = age_ms if abs(age_ms) < 60_000 else None

    def poll_latest(self) -> ArmCommand | None:
        latest = None
        while True:
            try:
                data, source = self.socket.recvfrom(2048)
            except BlockingIOError:
                break
            except InterruptedError:
                continue

            command = unpack_arm_command(data)
            if command is None:
                self.invalid_packets += 1
                continue
            received_at = time.monotonic()
            latest = command
            self.valid_packets += 1
            if source != self.last_source:
                self.last_seq = None
            self.last_source = source
            self.last_version = command.version
            self._recent_receive_times.append(received_at)
            while (
                self._recent_receive_times
                and received_at - self._recent_receive_times[0] > 2.0
            ):
                self._recent_receive_times.popleft()
            if command.version == 2:
                self._update_v2_stats(command, received_at)
        return latest


def step_toward(current: float, target: float, max_step: float) -> float:
    """以有限步长从 current 移向 target。"""
    delta = target - current
    if delta > max_step:
        return current + max_step
    if delta < -max_step:
        return current - max_step
    return target


def make_demo_command(elapsed: float) -> ArmCommand:
    """生成幅度较小的双臂正弦演示指令。"""
    phase = 2.0 * math.pi * 0.18 * elapsed
    right = (
        0.35 * math.sin(phase),
        -0.25 + 0.18 * math.sin(phase * 0.7),
        0.20 * math.sin(phase * 0.5),
        0.55 + 0.35 * math.sin(phase * 0.8),
        0.45 * math.sin(phase),
    )
    left = (
        0.35 * math.sin(phase + math.pi),
        0.25 - 0.18 * math.sin(phase * 0.7),
        -0.20 * math.sin(phase * 0.5),
        0.55 - 0.35 * math.sin(phase * 0.8),
        -0.45 * math.sin(phase),
    )
    return ArmCommand(mode=3, right=right, left=left)


def import_mujoco() -> Any:
    try:
        import mujoco
    except ImportError as exc:
        raise SystemExit(
            "未安装 MuJoCo。请先运行：\n"
            "  cd 仿真 && python3 -m venv .venv\n"
            "  source .venv/bin/activate\n"
            "  pip install -r requirements.txt"
        ) from exc
    return mujoco


def load_model(model_path: Path, mujoco: Any) -> Any:
    """按扩展名加载模型：MJCF 直接交给 MuJoCo，URDF 走下面的兼容路径。

    全身 29-DOF 模型是 MJCF（与 HumDex 全身仿真同一份文件），MuJoCo 原生
    支持，不需要 URDF 那套 meshdir 变通。旧的双臂 URDF 仍可用 --model 指定。
    """
    if model_path.suffix.lower() == ".xml":
        return mujoco.MjModel.from_xml_path(str(model_path))
    return load_urdf_model(model_path, mujoco)


def load_urdf_model(model_path: Path, mujoco: Any) -> Any:
    """从官方 URDF 编译 MuJoCo 模型，不修改原文件。

    官方 G1 URDF 同时使用 ``meshdir="meshes"`` 和
    ``filename="meshes/xxx.STL"``。MuJoCo 3.10 会将二者叠加为
    ``meshes/meshes/xxx.STL``。这里只在内存副本中移除 meshdir，
    并将 URDF 引用的网格作为 assets 传给 MuJoCo。
    """
    xml = model_path.read_text(encoding="utf-8")
    xml, substitutions = re.subn(
        r'(<compiler\b[^>]*?)\s+meshdir="[^"]*"',
        r"\1",
        xml,
        count=1,
    )
    if substitutions == 0:
        print("[!] URDF 未声明 meshdir，按原始网格路径加载")

    mesh_names = sorted(set(re.findall(r'mesh filename="([^"]+)"', xml)))
    assets: dict[str, bytes] = {}
    model_dir = model_path.parent.resolve()
    for name in mesh_names:
        mesh_path = (model_dir / name).resolve()
        if not mesh_path.is_relative_to(model_dir):
            raise ValueError(f"URDF 网格路径越界: {name}")
        if not mesh_path.is_file():
            raise FileNotFoundError(f"URDF 缺少网格: {mesh_path}")
        assets[name] = mesh_path.read_bytes()

    return mujoco.MjModel.from_xml_string(xml, assets=assets)


def resolve_joint_addresses(model: Any, mujoco: Any) -> dict[str, int]:
    """通过关节名查找 qpos 地址，避免依赖 URDF 文件顺序。"""
    addresses: dict[str, int] = {}
    missing: list[str] = []
    for name in ALL_ARM_JOINTS:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            missing.append(name)
            continue
        addresses[name] = int(model.jnt_qposadr[joint_id])
    if missing:
        raise RuntimeError(f"URDF 缺少关节: {', '.join(missing)}")
    return addresses


def resolve_joint_ranges(model: Any, mujoco: Any) -> dict[str, tuple[float, float]]:
    ranges: dict[str, tuple[float, float]] = {}
    for name in CONTROLLED_JOINTS:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        lower, upper = model.jnt_range[joint_id]
        ranges[name] = (float(lower), float(upper))
    return ranges


def resolve_waist(model: Any, mujoco: Any) -> tuple[int | None, tuple[float, float]]:
    """waist_yaw 的 qpos 地址与限位；上半身模型没有该关节时返回 (None, ...)。"""
    joint_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, WAIST_YAW_JOINT)
    if joint_id < 0:
        return None, (0.0, 0.0)
    lower, upper = model.jnt_range[joint_id]
    return int(model.jnt_qposadr[joint_id]), (float(lower), float(upper))


def resolve_centred_waist(model: Any, mujoco: Any) -> tuple[int, ...]:
    """waist_roll/pitch 的 qpos 地址；这两轴恒定保持 0，与真机一致。"""
    addresses = []
    for name in WAIST_CENTRED_JOINTS:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id >= 0:
            addresses.append(int(model.jnt_qposadr[joint_id]))
    return tuple(addresses)


def load_hand_mapping() -> Any:
    """按路径加载 ``PC端/hand_mapping.py``；不存在时返回 None。

    通道↔关节对应表、mimic 倍率和行程上限是模型事实，不是部署差异，所以
    全仓库只留一份（PC端 那份有单测钉着）。这里按路径引用而不是再复制一
    份 —— 复制出来的表迟早会和模型漂开，而且不会报错，只会让仿真里的手
    和真手对不上。
    """
    import importlib.util

    path = (Path(__file__).resolve().parent.parent
            / "PC端" / "hand_mapping.py")
    if not path.is_file():
        return None
    spec = importlib.util.spec_from_file_location("hand_mapping", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def resolve_hand_addresses(
    model: Any, mujoco: Any, hand_mapping: Any, side: str,
) -> dict[str, int]:
    """该侧全部 12 个手指关节的 qpos 地址；无手模型返回空表。

    MuJoCo 不认 URDF 的 ``<mimic>``，那 6 个联动关节在它眼里就是普通独立
    关节 —— 所以驱动端要用 ``expand_mimic()`` 把 6 个通道展开成 12 个角度
    逐个写进去，联动才会出现。
    """
    if hand_mapping is None:
        return {}
    addresses: dict[str, int] = {}
    for driver in hand_mapping.driver_joints(side):
        for name in (driver,) + tuple(
                passive for passive, _ in
                hand_mapping.mimic_chains(side)[driver]):
            joint_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id >= 0:
                addresses[name] = int(model.jnt_qposadr[joint_id])
    return addresses


def resolve_standing_pose(model: Any, mujoco: Any) -> dict[int, float]:
    """全身模型下非手臂关节的固定站姿，返回 qpos 地址 → 角度。

    这仍是运动学回放：不算力矩、不算平衡，腿只是摆成一个站立造型让画面
    对得上真机。取 HumDex 的默认站姿（微屈膝），与 9627 那套全身仿真一致；
    上半身模型没有腿部关节，此时返回空表。
    """
    pose: dict[int, float] = {}
    for name, angle in STANDING_LEG_POSE.items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id >= 0:
            pose[int(model.jnt_qposadr[joint_id])] = angle
    return pose


def apply_standing_pose(model: Any, data: Any, mujoco: Any,
                        pose: dict[int, float]) -> None:
    """初始化 qpos：浮动基座摆正，腿摆成站姿。

    全身 MJCF 的根是 freejoint，qpos 前 7 位是位置 + 四元数。默认全零会
    得到非法的零四元数，必须显式写成单位四元数。
    """
    data.qpos[:] = 0.0
    for joint_id in range(model.njnt):
        if model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_FREE:
            continue
        address = int(model.jnt_qposadr[joint_id])
        data.qpos[address + 2] = FLOATING_BASE_HEIGHT
        data.qpos[address + 3] = 1.0        # w，其余 x/y/z 保持 0
    for address, angle in pose.items():
        data.qpos[address] = angle


def clamp(value: float, limits: tuple[float, float]) -> float:
    return min(max(value, limits[0]), limits[1])


def parse_args() -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    default_model = script_dir / "models" / "g1_29dof" / "g1_29dof.xml"

    parser = argparse.ArgumentParser(
        description="用 MuJoCo 显示现有 UARM UDP 指令驱动的 G1 双臂",
    )
    parser.add_argument("--model", type=Path, default=default_model,
                        help="G1 模型路径，MJCF(.xml) 或 URDF(.urdf)。"
                             "默认全身 29-DOF；旧的上半身模型是 "
                             "models/g1_description/g1_dual_arm.urdf")
    parser.add_argument("--bind", default="127.0.0.1",
                        help="UDP 监听地址；跨设备接收时使用 0.0.0.0")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="UDP 监听端口")
    parser.add_argument("--control-hz", type=float, default=DEFAULT_CONTROL_HZ,
                        help="仿真关节更新频率")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_SEC,
                        help="UDP 超时回零时间（秒）")
    parser.add_argument("--follow-speed", type=float, default=DEFAULT_FOLLOW_SPEED,
                        help="跟随最大关节速度（rad/s）")
    parser.add_argument("--return-speed", type=float, default=DEFAULT_RETURN_SPEED,
                        help="回零最大关节速度（rad/s）")
    parser.add_argument("--waist-speed", type=float, default=DEFAULT_WAIST_SPEED,
                        help="腰部跟随最大速度（rad/s），与真机接收端一致")
    parser.add_argument("--demo", action="store_true",
                        help="不监听 UDP，运行内置双臂正弦动作")
    parser.add_argument("--headless", action="store_true", help="不打开可视化窗口")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="运行秒数，0 表示一直运行")
    parser.add_argument("--quiet", action="store_true", help="不输出周期状态")
    args = parser.parse_args()

    if not 1 <= args.port <= 65535:
        parser.error("--port 必须在 1..65535 之间")
    for option in ("control_hz", "timeout", "follow_speed", "return_speed",
                   "waist_speed"):
        if getattr(args, option) <= 0:
            parser.error(f"--{option.replace('_', '-')} 必须大于 0")
    if args.duration < 0:
        parser.error("--duration 不能小于 0")
    return args


def run(args: argparse.Namespace) -> None:
    mujoco = import_mujoco()
    model_path = args.model.expanduser().resolve()
    if not model_path.is_file():
        raise SystemExit(f"未找到模型: {model_path}")

    print(f"[*] 加载模型: {model_path}")
    model = load_model(model_path, mujoco)
    data = mujoco.MjData(model)
    addresses = resolve_joint_addresses(model, mujoco)
    joint_ranges = resolve_joint_ranges(model, mujoco)
    current = {name: 0.0 for name in CONTROLLED_JOINTS}
    standing_pose = resolve_standing_pose(model, mujoco)
    apply_standing_pose(model, data, mujoco, standing_pose)

    waist_address, waist_range = resolve_waist(model, mujoco)
    centred_waist_addresses = resolve_centred_waist(model, mujoco)
    waist_current = 0.0
    if waist_address is None:
        print("[*] 模型没有腰部关节，腰部尾块将被忽略")

    # 灵巧手：只有带手的模型（因时 FTP 版 URDF）才有这些关节，无手模型
    # 下 hand_addresses 为空，整段手部逻辑自动跳过，行为与加手之前一致。
    hand_mapping = load_hand_mapping()
    hand_addresses = {
        side: resolve_hand_addresses(model, mujoco, hand_mapping, side)
        for side in ("right", "left")
    }
    hand_closure = {side: (0.0,) * HAND_CHANNEL_COUNT
                    for side in ("right", "left")}
    has_hands = any(hand_addresses.values())
    if has_hands:
        print("[*] 模型带因时 FTP 灵巧手："
              + "，".join(f"{side} {len(addr)} 关节"
                          for side, addr in hand_addresses.items() if addr))
    elif hand_mapping is None:
        print("[*] 找不到 PC端/hand_mapping.py，灵巧手尾块将被忽略")
    else:
        print("[*] 模型没有手指关节，灵巧手尾块将被忽略")

    receiver = None
    if args.demo:
        print("[*] 演示模式：使用内置双臂轨迹")
    else:
        receiver = UdpCommandReceiver(args.bind, args.port)
        print(f"[*] 等待 UA2M/UARM UDP: {args.bind}:{args.port}")

    viewer = None
    if not args.headless:
        from mujoco import viewer as mujoco_viewer

        viewer = mujoco_viewer.launch_passive(
            model,
            data,
            show_left_ui=False,
            show_right_ui=False,
        )
        viewer.cam.lookat[:] = (0.05, 0.0, 0.18)
        viewer.cam.distance = 0.95
        viewer.cam.azimuth = 145.0
        viewer.cam.elevation = -15.0

    control_dt = 1.0 / args.control_hz
    started_at = time.monotonic()
    next_tick = started_at
    last_received_at = -math.inf
    last_command = ArmCommand(0, (0.0,) * 5, (0.0,) * 5)
    last_status_at = -math.inf

    try:
        while viewer is None or viewer.is_running():
            now = time.monotonic()
            elapsed = now - started_at
            if args.duration and elapsed >= args.duration:
                break

            if args.demo:
                command = make_demo_command(elapsed)
                timed_out = False
            else:
                latest = receiver.poll_latest()
                if latest is not None:
                    last_command = latest
                    last_received_at = now
                timed_out = now - last_received_at > args.timeout
                command = last_command

            effective_mode = 0 if timed_out else command.mode
            right_follow = effective_mode in (1, 3)
            left_follow = effective_mode in (2, 3)

            for name, packet_value in zip(RIGHT_JOINTS, command.right):
                target = clamp(packet_value, joint_ranges[name]) if right_follow else 0.0
                speed = args.follow_speed if right_follow else args.return_speed
                current[name] = step_toward(current[name], target, speed * control_dt)
            for name, packet_value in zip(LEFT_JOINTS, command.left):
                target = clamp(packet_value, joint_ranges[name]) if left_follow else 0.0
                speed = args.follow_speed if left_follow else args.return_speed
                current[name] = step_toward(current[name], target, speed * control_dt)

            # 腰：跟随条件与真机接收端一致 —— 有手臂在跟随且本帧有有效尾块，
            # 否则回中。上半身模型没有这个关节时 waist_address 为 None。
            if waist_address is not None:
                waist_following = (
                    (right_follow or left_follow) and command.waist is not None)
                if waist_following:
                    waist_target = clamp(command.waist[0], waist_range)
                    waist_speed = args.waist_speed
                else:
                    waist_target = 0.0
                    waist_speed = args.return_speed
                waist_current = step_toward(
                    waist_current, waist_target, waist_speed * control_dt)

            # 手：**不跟手臂的模式联动**，这一点和腰相反。腰要等手臂进入
            # 跟随才动，是因为扭腰改变上半身重心、直接扰动平衡；手指没有
            # 这个耦合。跟着 mode 走的话，不接 IMU 就永远测不了手。
            # 判据只剩一条：本帧有没有这只手的有效数据。没有就按回零速度
            # 张开 —— 松手、超时、尾块丢失都回到张开，没数据时张手是安全的。
            if has_hands:
                for side in ("right", "left"):
                    if not hand_addresses[side]:
                        continue
                    target = None
                    if command.hands is not None:
                        target = command.hands.get(side)
                    if target is None:
                        target = (0.0,) * HAND_CHANNEL_COUNT
                        speed = args.return_speed
                    else:
                        speed = args.follow_speed
                    hand_closure[side] = tuple(
                        step_toward(now_value, goal, speed * control_dt)
                        for now_value, goal
                        in zip(hand_closure[side], target))

            lock = viewer.lock() if viewer is not None else _NullContext()
            with lock:
                for name, value in current.items():
                    data.qpos[addresses[name]] = value
                for name in LOCKED_WRIST_JOINTS:
                    data.qpos[addresses[name]] = 0.0
                if waist_address is not None:
                    data.qpos[waist_address] = waist_current
                for address in centred_waist_addresses:
                    data.qpos[address] = 0.0
                if has_hands:
                    for side, addr in hand_addresses.items():
                        if not addr:
                            continue
                        # 6 通道 → 6 驱动关节角 → 展开成 12 个（MuJoCo 不
                        # 认 mimic，联动关节要自己写进去）。
                        angles = hand_mapping.expand_mimic(
                            hand_mapping.closure_to_angles(
                                hand_closure[side], side=side),
                            side=side)
                        for name, angle in angles.items():
                            if name in addr:
                                data.qpos[addr[name]] = angle
                for address, angle in standing_pose.items():
                    data.qpos[address] = angle
                data.qvel[:] = 0.0
                data.time = elapsed
                mujoco.mj_forward(model, data)

            if viewer is not None:
                viewer.sync()

            if not args.quiet and now - last_status_at >= 0.5:
                last_status_at = now
                if args.demo:
                    source_text = "demo"
                elif receiver.last_source is None:
                    source_text = "未收到数据"
                else:
                    source_text = f"{receiver.last_source[0]}:{receiver.last_source[1]}"
                timeout_text = " / UDP超时" if timed_out and not args.demo else ""
                packets = receiver.valid_packets if receiver is not None else 0
                transport_text = ""
                if receiver is not None and receiver.last_version is not None:
                    transport_text = (
                        f" 协议={'UA2M' if receiver.last_version == 2 else 'UARM'}"
                        f" 接收={receiver.receive_hz:.1f}Hz"
                    )
                    if receiver.last_seq is not None:
                        transport_text += (
                            f" seq={receiver.last_seq}"
                            f" 丢包={receiver.lost_packets}"
                        )
                    if receiver.last_transport_age_ms is not None:
                        transport_text += (
                            f" 传输={receiver.last_transport_age_ms:.1f}ms"
                        )
                hand_text = ""
                if has_hands:
                    parts = []
                    for side, label in (("right", "右手"), ("left", "左手")):
                        if not hand_addresses[side]:
                            continue
                        # 显示平均闭合度：0% 张开、100% 是当前行程比例的到底。
                        # 发送端没开手时这里恒为 0%，一眼能看出是"没数据"
                        # 而不是"手不动"。
                        average = sum(hand_closure[side]) / HAND_CHANNEL_COUNT
                        parts.append(f"{label}:{average * 100:.0f}%")
                    hand_text = " " + " ".join(parts)
                print(
                    f"\r[{MODE_NAMES[effective_mode]}{timeout_text}] "
                    f"来源={source_text} 有效包={packets}"
                    f"{transport_text}{hand_text}",
                    end="",
                    flush=True,
                )

            next_tick += control_dt
            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_tick = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        if not args.quiet:
            print()
        if viewer is not None:
            viewer.close()
        if receiver is not None:
            receiver.close()
            print(
                f"[*] UDP 统计: 有效={receiver.valid_packets}, "
                f"无效={receiver.invalid_packets}, "
                f"丢包={receiver.lost_packets}, "
                f"重复={receiver.duplicate_packets}, "
                f"乱序={receiver.out_of_order_packets}"
            )


class _NullContext:
    def __enter__(self) -> None:
        return None

    def __exit__(self, exc_type: Any, exc_value: Any, traceback: Any) -> bool:
        return False


def main() -> int:
    try:
        run(parse_args())
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"[!] 仿真启动失败: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
