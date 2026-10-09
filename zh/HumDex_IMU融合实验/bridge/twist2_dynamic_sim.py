#!/usr/bin/env python3
"""现有 UA2M 双臂输入 + HumDex TWIST2 全身动力学平衡仿真。"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import signal
import sys
import time
from typing import Optional

import mujoco
import numpy as np

from .full_body_reference import DEFAULT_JOINTS, ReferenceStatus
from .policy_runtime import (
    ACTION_SCALE,
    ANKLE_INDICES,
    HISTORY_LENGTH,
    MIMIC_SIZE,
    NUM_ACTIONS,
    POLICY_INPUT_SIZE,
    PROPRIO_SIZE,
    SINGLE_OBS_SIZE,
    OnnxPolicy,
    PolicyRuntime,
    RobotObservation,
    demo_arm_command,
    quaternion_to_roll_pitch,
)
from .ua2m_protocol import ArmCommand, LatestArmReceiver


EXPERIMENT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = (
    EXPERIMENT_ROOT
    / "upstream/HumDex/assets/g1/g1_sim2sim_29dof.xml"
)
DEFAULT_POLICY = (
    EXPERIMENT_ROOT
    / "upstream/HumDex/assets/ckpts/twist2_1017_25k.onnx"
)

STIFFNESS = np.array(
    [
        100, 100, 100, 150, 40, 40,
        100, 100, 100, 150, 40, 40,
        150, 150, 150,
        40, 40, 40, 40, 4, 4, 4,
        40, 40, 40, 40, 4, 4, 4,
    ],
    dtype=np.float64,
)
DAMPING = np.array(
    [
        2, 2, 2, 4, 2, 2,
        2, 2, 2, 4, 2, 2,
        4, 4, 4,
        5, 5, 5, 5, 0.2, 0.2, 0.2,
        5, 5, 5, 5, 0.2, 0.2, 0.2,
    ],
    dtype=np.float64,
)
TORQUE_LIMITS = STIFFNESS.copy()


class CsvRunLogger:
    def __init__(self, path: Optional[Path]) -> None:
        self.file = None
        self.writer = None
        self.path = path
        if path is None:
            return
        path.parent.mkdir(parents=True, exist_ok=True)
        self.file = path.open("w", newline="", encoding="utf-8")
        fields = [
            "wall_time",
            "sim_time",
            "seq",
            "mode",
            "timed_out",
            "packet_age_ms",
            "pelvis_z",
            "roll_deg",
            "pitch_deg",
            "inference_ms",
        ]
        fields += [f"reference_{i}" for i in range(NUM_ACTIONS)]
        fields += [f"joint_{i}" for i in range(NUM_ACTIONS)]
        fields += [f"target_{i}" for i in range(NUM_ACTIONS)]
        fields += [f"torque_{i}" for i in range(NUM_ACTIONS)]
        self.writer = csv.DictWriter(self.file, fieldnames=fields)
        self.writer.writeheader()

    def write(
        self,
        *,
        sim_time: float,
        status: ReferenceStatus,
        pelvis_z: float,
        roll: float,
        pitch: float,
        inference_ms: float,
        reference: np.ndarray,
        joints: np.ndarray,
        target: np.ndarray,
        torque: np.ndarray,
    ) -> None:
        if self.writer is None:
            return
        row: dict[str, object] = {
            "wall_time": time.time(),
            "sim_time": sim_time,
            "seq": "" if status.seq is None else status.seq,
            "mode": status.mode,
            "timed_out": int(status.timed_out),
            "packet_age_ms": (
                "" if status.packet_age_ms is None else status.packet_age_ms
            ),
            "pelvis_z": pelvis_z,
            "roll_deg": np.degrees(roll),
            "pitch_deg": np.degrees(pitch),
            "inference_ms": inference_ms,
        }
        for prefix, values in (
            ("reference", reference),
            ("joint", joints),
            ("target", target),
            ("torque", torque),
        ):
            row.update({f"{prefix}_{i}": float(v) for i, v in enumerate(values)})
        self.writer.writerow(row)
        self.file.flush()

    def close(self) -> None:
        if self.file is not None:
            self.file.close()


class Twist2DynamicSimulation:
    def __init__(self, args: argparse.Namespace) -> None:
        if args.physics_hz % args.policy_hz != 0:
            raise ValueError("physics-hz 必须能被 policy-hz 整除")
        self.args = args
        self.model = mujoco.MjModel.from_xml_path(str(args.model))
        if (self.model.nq, self.model.nv, self.model.nu) != (36, 35, 29):
            raise RuntimeError(
                "必须使用 HumDex 的 29DOF 浮动基座模型，"
                f"实际 nq/nv/nu={self.model.nq}/{self.model.nv}/{self.model.nu}"
            )
        self.model.opt.timestep = 1.0 / args.physics_hz
        self.data = mujoco.MjData(self.model)
        self.receiver = None if args.demo else LatestArmReceiver(args.bind, args.port)

        # 策略输出仍需受物理模型关节范围保护；全部 29 个关节都是 limited。
        joint_ranges = self.model.jnt_range[1:]
        self.runtime = PolicyRuntime(
            args.policy,
            joint_lower=joint_ranges[:, 0],
            joint_upper=joint_ranges[:, 1],
            timeout_seconds=args.timeout,
            max_arm_speed=args.max_arm_speed,
            prediction_seconds=args.prediction_ms / 1000.0,
        )
        self.policy = self.runtime.policy
        self.policy_period = 1.0 / args.policy_hz
        self.decimation = args.physics_hz // args.policy_hz
        self.pd_target = DEFAULT_JOINTS.astype(np.float64).copy()
        self.last_torque = np.zeros(NUM_ACTIONS, dtype=np.float64)
        self.running = True
        self.last_print = 0.0
        self.last_inference_ms = 0.0
        self.last_status = ReferenceStatus(0, True, None, None, None)
        self.last_reference = np.concatenate(
            (
                np.array([0, 0, 0.8, 0, 0, 0], dtype=np.float32),
                DEFAULT_JOINTS,
            )
        )
        self.logger = CsvRunLogger(args.log)
        self._reset()

    def _reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:7] = np.array([0, 0, 0.793, 1, 0, 0, 0])
        self.data.qpos[7:] = DEFAULT_JOINTS
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def _demo_command(self, now: float) -> ArmCommand:
        return demo_arm_command(now, self.args.policy_hz)

    def _policy_step(self, now: float) -> None:
        if self.receiver is not None:
            command = self.receiver.poll_latest()
            if command is not None:
                self.runtime.consume(command)
        else:
            self.runtime.consume(self._demo_command(now))

        roll, pitch = quaternion_to_roll_pitch(self.data.qpos[3:7])
        observation = RobotObservation(
            dof_pos=self.data.qpos[7:].copy(),
            dof_vel=self.data.qvel[6:].copy(),
            # 自由关节的 qvel[3:6] 就是骨盆局部角速度，对应真机 IMU 陀螺。
            ang_vel=self.data.qvel[3:6].copy(),
            roll=roll,
            pitch=pitch,
        )
        output = self.runtime.step(observation, now=now, dt=self.policy_period)

        self.pd_target = output.target_dof_pos
        self.last_inference_ms = output.inference_ms
        self.last_action = self.runtime.last_action
        self.last_status = output.status
        self.last_reference = output.reference
        self._log_and_check_fall(roll, pitch)

    def _log_and_check_fall(self, roll: float, pitch: float) -> None:
        pelvis_z = float(self.data.qpos[2])
        self.logger.write(
            sim_time=float(self.data.time),
            status=self.last_status,
            pelvis_z=pelvis_z,
            roll=roll,
            pitch=pitch,
            inference_ms=self.last_inference_ms,
            reference=self.last_reference[6:],
            joints=self.data.qpos[7:],
            target=self.pd_target,
            torque=self.last_torque,
        )
        if self.data.time > self.args.fall_grace_seconds and (
            pelvis_z < self.args.min_pelvis_height
            or abs(roll) > np.radians(self.args.max_tilt_deg)
            or abs(pitch) > np.radians(self.args.max_tilt_deg)
        ):
            print(
                "\n[安全停止] 检测到跌倒："
                f"z={pelvis_z:.3f}m roll={np.degrees(roll):+.1f}° "
                f"pitch={np.degrees(pitch):+.1f}°"
            )
            self.running = False

    def _physics_step(self) -> None:
        joints = self.data.qpos[7:]
        velocities = self.data.qvel[6:]
        torque = (
            (self.pd_target - joints) * STIFFNESS - velocities * DAMPING
        )
        self.last_torque = np.clip(
            torque, -TORQUE_LIMITS, TORQUE_LIMITS
        )
        self.data.ctrl[:] = self.last_torque
        mujoco.mj_step(self.model, self.data)

    def _print_status(self, now: float) -> None:
        if now - self.last_print < 1.0:
            return
        self.last_print = now
        roll, pitch = quaternion_to_roll_pitch(self.data.qpos[3:7])
        age = self.last_status.packet_age_ms
        age_text = "--" if age is None else f"{age:.1f}"
        packet_text = ""
        if self.receiver is not None:
            packet_text = (
                f" 包={self.receiver.packet_count}"
                f" 丢={self.receiver.lost_count}"
                f" 坏={self.receiver.invalid_count}"
            )
        print(
            f"\r仿真={self.data.time:7.1f}s 模式={self.last_status.mode} "
            f"包龄={age_text}ms z={self.data.qpos[2]:.3f}m "
            f"RP={np.degrees(roll):+.1f}/{np.degrees(pitch):+.1f}° "
            f"推理={self.last_inference_ms:.2f}ms{packet_text}",
            end="",
            flush=True,
        )

    def run(self) -> None:
        viewer = None
        if not self.args.headless:
            import mujoco.viewer

            viewer = mujoco.viewer.launch_passive(
                self.model,
                self.data,
                show_left_ui=False,
                show_right_ui=False,
            )
            viewer.cam.distance = 2.7
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -15

        print("[*] HumDex/TWIST2 双臂融合动力学仿真")
        print(f"[*] 模型: {self.args.model}")
        print(f"[*] 策略: {self.args.policy}")
        if self.receiver is None:
            print("[*] 输入: 内置双臂慢动作演示")
        else:
            print(f"[*] 等待 UA2M/UARM UDP: {self.args.bind}:{self.args.port}")
        print("[*] 腿不跟踪 IMU，由 TWIST2 策略自动维持平衡；"
              "腰仅在发送端带 UAWS 尾块时跟随 waist_yaw")
        if self.args.log is not None:
            print(f"[*] 日志: {self.args.log}")

        start_wall = time.monotonic()
        next_wall = start_wall
        physics_step = 0
        try:
            while self.running and (viewer is None or viewer.is_running()):
                now = time.monotonic()
                if (
                    self.args.duration > 0
                    and now - start_wall >= self.args.duration
                ):
                    break
                if physics_step % self.decimation == 0:
                    self._policy_step(now)
                    if viewer is not None:
                        viewer.cam.lookat[:] = self.data.qpos[:3]
                        viewer.sync()
                self._physics_step()
                physics_step += 1
                self._print_status(now)

                if self.args.realtime:
                    next_wall += self.model.opt.timestep
                    delay = next_wall - time.monotonic()
                    if delay > 0:
                        time.sleep(delay)
                    elif delay < -0.25:
                        next_wall = time.monotonic()
        except KeyboardInterrupt:
            pass
        finally:
            print()
            if viewer is not None:
                viewer.close()
            if self.receiver is not None:
                self.receiver.close()
            self.logger.close()
            print("[*] 仿真结束，未向真实机器人发送任何命令")


def default_log_path() -> Path:
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    return EXPERIMENT_ROOT / "logs" / f"twist2_dual_arm_{timestamp}.csv"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="UA2M 双臂输入驱动 HumDex/TWIST2 全身平衡动力学仿真"
    )
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9627)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--policy", type=Path, default=DEFAULT_POLICY)
    parser.add_argument("--physics-hz", type=int, default=1000)
    parser.add_argument("--policy-hz", type=int, default=50)
    parser.add_argument("--timeout", type=float, default=0.35)
    parser.add_argument("--max-arm-speed", type=float, default=5.0)
    parser.add_argument("--prediction-ms", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--demo", action="store_true")
    parser.add_argument("--no-realtime", dest="realtime", action="store_false")
    parser.set_defaults(realtime=True)
    parser.add_argument("--no-log", action="store_true")
    parser.add_argument("--log", type=Path)
    parser.add_argument("--fall-grace-seconds", type=float, default=2.0)
    parser.add_argument("--min-pelvis-height", type=float, default=0.45)
    parser.add_argument("--max-tilt-deg", type=float, default=50.0)
    args = parser.parse_args()
    if not args.model.is_file():
        parser.error(f"模型不存在: {args.model}")
    if not args.policy.is_file():
        parser.error(f"策略不存在: {args.policy}")
    if args.policy_hz <= 0 or args.physics_hz <= 0:
        parser.error("频率必须大于 0")
    if args.no_log:
        args.log = None
    elif args.log is None:
        args.log = default_log_path()
    return args


def main() -> int:
    args = parse_args()
    simulation = Twist2DynamicSimulation(args)

    def stop(_signum: int, _frame: object) -> None:
        simulation.running = False

    signal.signal(signal.SIGTERM, stop)
    simulation.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
