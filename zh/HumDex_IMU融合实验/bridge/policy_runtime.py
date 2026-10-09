#!/usr/bin/env python3
"""TWIST2 策略运行时：观测构建、ONNX 推理和关节目标生成。

仿真后端 (twist2_dynamic_sim) 和真机后端 (twist2_real) 共用这一份实现，
保证喂给策略的观测由同一行代码算出，不会随时间漂移。

本模块是纯计算，不做任何 IO：不读 MuJoCo，不碰 DDS，不开 socket。
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from pathlib import Path
import time
from typing import Optional

import numpy as np

from .full_body_reference import (
    ArmToWholeBodyReference,
    DEFAULT_JOINTS,
    ReferenceStatus,
)
from .ua2m_protocol import ArmCommand


NUM_ACTIONS = 29
MIMIC_SIZE = 35
PROPRIO_SIZE = 92
SINGLE_OBS_SIZE = 127
HISTORY_LENGTH = 10
# 127 维当前观测 + 10×127 维历史 + 35 维未来参考。
# HumDex/TWIST2 上游注释误写为 1402，实际算术及仓库 ONNX 权重均为 1432。
POLICY_INPUT_SIZE = 1432

ACTION_SCALE = np.full(NUM_ACTIONS, 0.5, dtype=np.float32)
ANKLE_INDICES = np.array([4, 5, 10, 11])

ANG_VEL_SCALE = 0.25
DOF_VEL_SCALE = 0.05
DOF_POS_SCALE = 1.0

ACTION_CLIP = 10.0


def quaternion_to_roll_pitch(quaternion_wxyz: np.ndarray) -> tuple[float, float]:
    """wxyz 四元数转 roll/pitch。

    MuJoCo 的 qpos[3:7] 和 G1 LowState 的 imu_state.quaternion 都是 wxyz。
    """
    w, x, y, z = (float(v) for v in quaternion_wxyz)
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sin_roll, cos_roll)
    sin_pitch = np.clip(2.0 * (w * y - z * x), -1.0, 1.0)
    pitch = np.arcsin(sin_pitch)
    return float(roll), float(pitch)


def demo_arm_command(now: float, seq_hz: int = 50) -> ArmCommand:
    """无 IMU 时的内置双臂慢动作，仿真和真机吊装自检共用。"""
    phase = now * 1.1
    right = (
        0.15 * np.sin(phase),
        -0.4 + 0.12 * np.sin(phase * 0.7),
        0.15 * np.sin(phase * 0.6),
        1.2 + 0.25 * np.sin(phase),
        0.2 * np.sin(phase * 1.3),
    )
    left = (
        0.15 * np.sin(phase + np.pi),
        0.4 + 0.12 * np.sin(phase * 0.7 + np.pi),
        0.15 * np.sin(phase * 0.6 + np.pi),
        1.2 + 0.25 * np.sin(phase + np.pi),
        0.2 * np.sin(phase * 1.3 + np.pi),
    )
    return ArmCommand(
        mode=3,
        seq=int(now * seq_hz),
        sender_timestamp_us=None,
        positions=tuple(right + left),
        velocities=(0.0,) * 10,
        protocol="DEMO",
        received_monotonic=now,
    )


@dataclass(frozen=True)
class RobotObservation:
    """一帧本体感知，仿真与真机用同一个结构填。

    dof_pos / dof_vel : 29 关节位置和速度，顺序为 HumDex 29DOF
                        (左腿6 右腿6 腰3 左臂7 右臂7)，与 G1 SDK 电机索引一致。
    ang_vel           : 骨盆局部系角速度 rad/s。
                        MuJoCo 自由关节的 qvel[3:6] 即为局部角速度；
                        真机取 LowState.imu_state.gyroscope (imu_type=pelvis)。
    roll / pitch      : 骨盆姿态角 rad。
    """

    dof_pos: np.ndarray
    dof_vel: np.ndarray
    ang_vel: np.ndarray
    roll: float
    pitch: float


@dataclass(frozen=True)
class PolicyOutput:
    target_dof_pos: np.ndarray
    reference: np.ndarray
    status: ReferenceStatus
    inference_ms: float
    raw_action: np.ndarray


class OnnxPolicy:
    def __init__(self, path: Path) -> None:
        try:
            import onnxruntime as ort
        except ImportError as exc:
            raise RuntimeError(
                "缺少 onnxruntime，请先执行：.venv/bin/pip install -r requirements.txt"
            ) from exc

        self.session = ort.InferenceSession(
            str(path), providers=["CPUExecutionProvider"]
        )
        inputs = self.session.get_inputs()
        outputs = self.session.get_outputs()
        if len(inputs) != 1:
            raise RuntimeError(f"策略输入数量异常: {len(inputs)}")
        self.input_name = inputs[0].name
        self.output_name = outputs[0].name
        shape = inputs[0].shape
        if shape[-1] not in (POLICY_INPUT_SIZE, None, "None"):
            raise RuntimeError(f"策略输入维数应为 {POLICY_INPUT_SIZE}，实际为 {shape}")

    def infer(self, observation: np.ndarray) -> np.ndarray:
        observation = np.asarray(observation, dtype=np.float32).reshape(1, -1)
        result = self.session.run(
            [self.output_name], {self.input_name: observation}
        )[0]
        action = np.asarray(result, dtype=np.float32).reshape(-1)
        if action.shape != (NUM_ACTIONS,):
            raise RuntimeError(f"策略输出维数应为 29，实际为 {action.shape}")
        return action


class PolicyRuntime:
    """双臂参考 + 本体感知 → 29 关节位置目标。

    与上游 HumDex deploy_real 的观测拼装严格一致：
        proprio = [ang_vel*0.25, roll, pitch, dof_pos-default,
                   dof_vel*0.05 (踝置零), last_action]
        obs     = [reference, proprio] + 10 帧历史 + reference
    last_action 存的是 **clip 之前** 的原始策略输出，与上游
    server_low_level_g1_real.py 和 server_low_level_g1_sim.py 一致。
    """

    def __init__(
        self,
        policy_path: Path,
        *,
        joint_lower: np.ndarray,
        joint_upper: np.ndarray,
        timeout_seconds: float = 0.35,
        max_arm_speed: float = 5.0,
        prediction_seconds: float = 0.0,
        policy: Optional[OnnxPolicy] = None,
        allow_waist: bool = True,
    ) -> None:
        self.policy = OnnxPolicy(policy_path) if policy is None else policy
        self.joint_lower = np.asarray(joint_lower, dtype=np.float64)
        self.joint_upper = np.asarray(joint_upper, dtype=np.float64)
        if self.joint_lower.shape != (NUM_ACTIONS,) or self.joint_upper.shape != (
            NUM_ACTIONS,
        ):
            raise ValueError("关节限位必须是 29 维")

        self.reference_builder = ArmToWholeBodyReference(
            timeout_seconds=timeout_seconds,
            max_arm_speed=max_arm_speed,
            prediction_seconds=prediction_seconds,
            allow_waist=allow_waist,
        )
        self.history = deque(
            [
                np.zeros(SINGLE_OBS_SIZE, dtype=np.float32)
                for _ in range(HISTORY_LENGTH)
            ],
            maxlen=HISTORY_LENGTH,
        )
        self.last_action = np.zeros(NUM_ACTIONS, dtype=np.float32)
        self.last_reference = np.concatenate(
            (
                np.array([0, 0, 0.8, 0, 0, 0], dtype=np.float32),
                DEFAULT_JOINTS,
            )
        )
        self.last_status = ReferenceStatus(0, True, None, None, None)
        self.last_inference_ms = 0.0

    def consume(self, command: ArmCommand) -> None:
        self.reference_builder.consume(command)

    def build_observation(
        self, observation: RobotObservation, reference: np.ndarray
    ) -> np.ndarray:
        """拼出 1432 维策略输入。会把当前帧写入历史缓冲。"""
        dof_pos = np.asarray(observation.dof_pos, dtype=np.float32)
        dof_vel = np.asarray(observation.dof_vel, dtype=np.float32)
        ang_vel = np.asarray(observation.ang_vel, dtype=np.float32)
        if dof_pos.shape != (NUM_ACTIONS,) or dof_vel.shape != (NUM_ACTIONS,):
            raise ValueError("dof_pos/dof_vel 必须是 29 维")
        if ang_vel.shape != (3,):
            raise ValueError("ang_vel 必须是 3 维")

        observation_velocities = dof_vel.copy()
        observation_velocities[ANKLE_INDICES] = 0.0
        proprio = np.concatenate(
            (
                ang_vel * ANG_VEL_SCALE,
                np.array([observation.roll, observation.pitch]),
                (dof_pos - DEFAULT_JOINTS) * DOF_POS_SCALE,
                observation_velocities * DOF_VEL_SCALE,
                self.last_action,
            )
        ).astype(np.float32)
        if proprio.shape != (PROPRIO_SIZE,):
            raise RuntimeError(f"proprio 维数错误: {proprio.shape}")

        current = np.concatenate((reference, proprio)).astype(np.float32)
        history = np.concatenate(tuple(self.history)).astype(np.float32)
        self.history.append(current)
        policy_input = np.concatenate((current, history, reference)).astype(
            np.float32
        )
        if policy_input.shape != (POLICY_INPUT_SIZE,):
            raise RuntimeError(f"内部观测维数错误: {policy_input.shape}")
        return policy_input

    def step(
        self,
        observation: RobotObservation,
        *,
        now: Optional[float] = None,
        dt: float = 0.02,
    ) -> PolicyOutput:
        now = time.monotonic() if now is None else now
        reference, status = self.reference_builder.step(now=now, dt=dt)
        policy_input = self.build_observation(observation, reference)

        inference_start = time.perf_counter()
        action = self.policy.infer(policy_input)
        inference_ms = (time.perf_counter() - inference_start) * 1000.0

        # 与上游一致：历史里存 clip 之前的原始输出。
        self.last_action = action.copy()
        clipped = np.clip(action, -ACTION_CLIP, ACTION_CLIP)
        target = (DEFAULT_JOINTS + clipped * ACTION_SCALE).astype(np.float64)
        target = np.clip(target, self.joint_lower, self.joint_upper)

        self.last_reference = reference
        self.last_status = status
        self.last_inference_ms = inference_ms
        return PolicyOutput(
            target_dof_pos=target,
            reference=reference,
            status=status,
            inference_ms=inference_ms,
            raw_action=action,
        )
