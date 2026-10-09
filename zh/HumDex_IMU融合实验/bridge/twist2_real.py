#!/usr/bin/env python3
"""TWIST2 全身策略真机运行时（宇树 G1）。

  UA2M 双臂 UDP ──> PolicyRuntime(50Hz) ──> 29 关节位置目标
                                              │
                    rt/lowstate ──────────────┤
                                              ↓
                          发送线程 500Hz ──> rt/lowcmd (位置模式，驱动板 PD)

分三个阶段，靠 --stage 门控：
  shadow  只读状态跑策略、写日志，一帧 lowcmd 都不发（默认）
  hold    只做纯 PD 位置保持，不跑策略；用来单独调 --kp-scale，必须吊装
  hang    下发完整 29 关节并交给策略，必须吊装
  ground  落地站立，额外要求 --i-know-this-can-fall

重要：UA2M 超时（350ms）只让双臂参考限速回安全姿态，**不停策略**。
腿仍由策略维持平衡，IMU 掉线就停策略等于直接摔。这与
机器人端/robot_arm_receiver.py 的"超时回零"语义完全不同。

用法见 真机部署说明.md。
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import signal
import sys
import threading
import time
from typing import Optional

import numpy as np

from . import real_config as cfg
from .full_body_reference import DEFAULT_JOINTS
from .policy_runtime import (
    NUM_ACTIONS,
    PolicyRuntime,
    RobotObservation,
    demo_arm_command,
    quaternion_to_roll_pitch,
)
from .real_robot_io import G1RealIO, LowStateSnapshot
from .ua2m_protocol import LatestArmReceiver


EXPERIMENT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_POLICY = EXPERIMENT_ROOT / "upstream/HumDex/assets/ckpts/twist2_1017_25k.onnx"

STAGES = ("shadow", "hold", "hang", "ground")

# 安全阈值
STATE_STALE_SEC = 0.10        # lowstate 断流
POLICY_STALE_SEC = 0.10       # 策略线程停摆
TAKEOVER_MOVE_SEC = 2.0       # 插值到 default_angles 的时长
KP_RAMP_SEC = 3.0             # 接管后 kp/kd 由 KP_RAMP_START 渐增到 1.0
KP_RAMP_START = 0.2
DAMPING_HOLD_SEC = 1.5        # 阻尼态持续下发时长，之后才停发
TAU_OVER_RATIO = 0.95         # tau_est 超过该比例的力矩上限
TAU_OVER_SEC = 0.20           # 且持续这么久 → 中止


def _uniform_rate_limit(
    current: np.ndarray, target: np.ndarray, max_step: float
) -> np.ndarray:
    """整体等比缩放的限速，保持目标姿态方向不被扭曲。

    与上游 safety_rate_limit 同思路：任一关节增量超限时，整个增量向量
    按同一系数收缩，而不是逐关节独立截断。
    """
    delta = target - current
    max_abs = float(np.max(np.abs(delta))) if delta.size else 0.0
    if max_abs <= max_step or max_abs == 0.0:
        return target.copy()
    return current + delta * (max_step / max_abs)


class RealRunLogger:
    def __init__(self, path: Optional[Path]) -> None:
        self.file = None
        self.writer = None
        self.path = path
        self.rows = 0
        if path is None:
            return
        path.parent.mkdir(parents=True, exist_ok=True)
        self.file = path.open("w", newline="", encoding="utf-8")
        fields = [
            "wall_time", "stage", "seq", "mode", "timed_out", "packet_age_ms",
            "roll_deg", "pitch_deg", "inference_ms", "kp_scale",
        ]
        fields += [f"reference_{i}" for i in range(NUM_ACTIONS)]
        fields += [f"joint_{i}" for i in range(NUM_ACTIONS)]
        fields += [f"target_{i}" for i in range(NUM_ACTIONS)]
        fields += [f"tau_est_{i}" for i in range(NUM_ACTIONS)]
        self.writer = csv.DictWriter(self.file, fieldnames=fields)
        self.writer.writeheader()

    def write(self, **kwargs) -> None:
        if self.writer is None:
            return
        row = {
            "wall_time": time.time(),
            "stage": kwargs["stage"],
            "seq": "" if kwargs["status"].seq is None else kwargs["status"].seq,
            "mode": kwargs["status"].mode,
            "timed_out": int(kwargs["status"].timed_out),
            "packet_age_ms": (
                ""
                if kwargs["status"].packet_age_ms is None
                else kwargs["status"].packet_age_ms
            ),
            "roll_deg": np.degrees(kwargs["roll"]),
            "pitch_deg": np.degrees(kwargs["pitch"]),
            "inference_ms": kwargs["inference_ms"],
            "kp_scale": kwargs["kp_scale"],
        }
        for prefix, values in (
            ("reference", kwargs["reference"]),
            ("joint", kwargs["joints"]),
            ("target", kwargs["target"]),
            ("tau_est", kwargs["tau_est"]),
        ):
            row.update({f"{prefix}_{i}": float(v) for i, v in enumerate(values)})
        self.writer.writerow(row)
        self.rows += 1
        if self.rows % 50 == 0:
            self.file.flush()

    def close(self) -> None:
        if self.file is not None:
            self.file.close()


class Twist2RealController:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.stage = args.stage
        self.io = G1RealIO(net=args.net)
        self.runtime = PolicyRuntime(
            args.policy,
            joint_lower=cfg.JOINT_LOWER,
            joint_upper=cfg.JOINT_UPPER,
            timeout_seconds=args.timeout,
            max_arm_speed=args.max_arm_speed,
            prediction_seconds=args.prediction_ms / 1000.0,
            allow_waist=args.waist,
        )
        self.receiver = (
            None if args.demo else LatestArmReceiver(args.bind, args.port)
        )
        self.logger = RealRunLogger(args.log)

        self.policy_period = 1.0 / args.policy_hz
        self.send_period = 1.0 / args.send_hz
        self.max_step_per_send = args.max_dof_delta_per_step * (
            args.policy_hz / args.send_hz
        )
        self.max_tilt_rad = np.radians(args.max_tilt_deg)

        self._lock = threading.Lock()
        self._target = DEFAULT_JOINTS.astype(np.float64).copy()
        self._target_time = 0.0
        self._commanded = DEFAULT_JOINTS.astype(np.float64).copy()
        self._takeover_time: Optional[float] = None
        self._kp_scale = KP_RAMP_START
        self._policy_active = False
        self._pd_log_at = 0.0

        self._abort_reason: Optional[str] = None
        self._running = True
        self._tau_over_since: Optional[float] = None

    # ---------- 中止 ----------

    def abort(self, reason: str) -> None:
        with self._lock:
            if self._abort_reason is None:
                self._abort_reason = reason
            self._running = False

    @property
    def aborted(self) -> bool:
        with self._lock:
            return self._abort_reason is not None

    # ---------- 安全检查 ----------

    def _check_safety(self, state: Optional[LowStateSnapshot], now: float) -> None:
        if state is None:
            self.abort("从未收到 rt/lowstate")
            return
        if now - state.received_monotonic > STATE_STALE_SEC:
            self.abort(
                f"rt/lowstate 断流 {(now - state.received_monotonic) * 1000:.0f}ms"
            )
            return

        roll, pitch = quaternion_to_roll_pitch(state.quaternion)
        if abs(roll) > self.max_tilt_rad or abs(pitch) > self.max_tilt_rad:
            self.abort(
                f"躯干倾角超限 roll={np.degrees(roll):+.1f}° "
                f"pitch={np.degrees(pitch):+.1f}°"
            )
            return

        over = np.abs(state.tau_est) > cfg.TORQUE_LIMITS * TAU_OVER_RATIO
        if np.any(over):
            if self._tau_over_since is None:
                self._tau_over_since = now
            elif now - self._tau_over_since > TAU_OVER_SEC:
                joints = ", ".join(
                    cfg.JOINT_NAMES[i] for i in np.flatnonzero(over)[:4]
                )
                self.abort(f"力矩持续接近上限: {joints}")
                return
        else:
            self._tau_over_since = None

        if self.args.max_motor_temp > 0:
            hot = state.temperature > self.args.max_motor_temp
            if np.any(hot):
                joints = ", ".join(cfg.JOINT_NAMES[i] for i in np.flatnonzero(hot)[:4])
                self.abort(f"电机超温: {joints}")
                return

        if self.io.key_pressed("select"):
            self.abort("遥控器 SELECT")
            return

        with self._lock:
            policy_active = self._policy_active
            target_time = self._target_time
        if policy_active and now - target_time > POLICY_STALE_SEC:
            self.abort(f"策略线程停摆 {(now - target_time) * 1000:.0f}ms")

    # ---------- 观测 ----------

    @staticmethod
    def _observation(state: LowStateSnapshot) -> RobotObservation:
        roll, pitch = quaternion_to_roll_pitch(state.quaternion)
        return RobotObservation(
            dof_pos=state.dof_pos.astype(np.float32),
            dof_vel=state.dof_vel.astype(np.float32),
            ang_vel=state.gyroscope.astype(np.float32),
            roll=roll,
            pitch=pitch,
        )

    # ---------- 策略线程 ----------

    def _policy_loop(self) -> None:
        next_tick = time.monotonic()
        while self._running:
            now = time.monotonic()
            state = self.io.read_state()
            if state is None:
                time.sleep(self.policy_period)
                continue

            if self.receiver is not None:
                command = self.receiver.poll_latest()
                if command is not None:
                    self.runtime.consume(command)
            else:
                self.runtime.consume(demo_arm_command(now, self.args.policy_hz))

            try:
                output = self.runtime.step(
                    self._observation(state), now=now, dt=self.policy_period
                )
            except Exception as exc:  # 推理异常必须立刻转阻尼，不能带病继续
                self.abort(f"策略推理异常: {exc}")
                return

            if not np.all(np.isfinite(output.target_dof_pos)):
                self.abort("策略输出含 NaN/Inf")
                return

            with self._lock:
                self._target = output.target_dof_pos
                self._target_time = now
                self._policy_active = True
                kp_scale = self._kp_scale

            self.logger.write(
                stage=self.stage,
                status=output.status,
                roll=self._observation(state).roll,
                pitch=self._observation(state).pitch,
                inference_ms=output.inference_ms,
                kp_scale=kp_scale,
                reference=output.reference[6:],
                joints=state.dof_pos,
                target=output.target_dof_pos,
                tau_est=state.tau_est,
            )

            next_tick += self.policy_period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()

    # ---------- 发送线程 ----------

    def _send_loop(self) -> None:
        next_tick = time.monotonic()
        while self._running:
            now = time.monotonic()
            state = self.io.read_state()
            self._check_safety(state, now)
            if not self._running:
                break

            with self._lock:
                target = self._target.copy()
                takeover_time = self._takeover_time

            if takeover_time is not None:
                ramp = min(1.0, (now - takeover_time) / KP_RAMP_SEC)
                kp_scale = KP_RAMP_START + (1.0 - KP_RAMP_START) * ramp
            else:
                kp_scale = KP_RAMP_START

            self._commanded = _uniform_rate_limit(
                self._commanded, target, self.max_step_per_send
            )
            commanded = np.clip(self._commanded, cfg.JOINT_LOWER, cfg.JOINT_UPPER)

            with self._lock:
                self._kp_scale = kp_scale
            self.io.send_targets(commanded, kp_scale=kp_scale, kd_scale=kp_scale)

            next_tick += self.send_period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()

    # ---------- 阶段流程 ----------

    def _wait_key(self, name: str, prompt: str) -> None:
        print(prompt, flush=True)
        while self._running and not self.io.key_pressed(name):
            time.sleep(0.02)

    def _move_to_default(self) -> None:
        """2s 插值：当前实际姿态 → default_angles，kp/kd 同步渐入。"""
        state = self.io.wait_for_state()
        start = state.dof_pos.copy()
        target = DEFAULT_JOINTS.astype(np.float64)
        steps = int(TAKEOVER_MOVE_SEC / self.send_period)
        ceiling = self.args.kp_scale
        print(f"[*] {TAKEOVER_MOVE_SEC:.0f}s 插值到 HumDex 默认站姿"
              f"（kp 上限 ×{ceiling:.2f}）...")
        for i in range(steps):
            if not self._running:
                return
            alpha = (i + 1) / steps
            interp = start * (1.0 - alpha) + target * alpha
            kp_scale = (KP_RAMP_START + (1.0 - KP_RAMP_START) * alpha) * ceiling
            self.io.send_targets(interp, kp_scale=kp_scale, kd_scale=kp_scale)
            with self._lock:
                self._kp_scale = kp_scale
            self._log_pd_frame(interp, kp_scale)

            now = time.monotonic()
            self._check_safety(self.io.read_state(), now)
            time.sleep(self.send_period)
        self._commanded = target.copy()
        with self._lock:
            self._target = target.copy()

    def _log_pd_frame(self, target, kp_scale):
        """记录纯 PD 阶段的一帧。

        原来只有策略线程写日志, 而策略要按 [A] 之后才启动 —— 从 [START] 到
        [A] 之间的纯 PD 保持完全没有记录。真机上恰恰是这一段先抖起来的,
        没有数据就只能靠猜。这里按 50Hz 抽样, 不拖慢 500Hz 发送线程。
        """
        now = time.monotonic()
        if now - self._pd_log_at < 0.02:
            return
        self._pd_log_at = now
        state = self.io.read_state()
        if state is None:
            return
        roll, pitch = quaternion_to_roll_pitch(state.quaternion)
        self.logger.write(
            stage=f"{self.stage}-pd",
            status=self.runtime.last_status,
            roll=roll,
            pitch=pitch,
            inference_ms=0.0,
            kp_scale=kp_scale,
            reference=DEFAULT_JOINTS,
            joints=state.dof_pos,
            target=target,
            tau_est=state.tau_est,
        )

    def _hold_default(self) -> None:
        target = DEFAULT_JOINTS.astype(np.float64)
        ceiling = self.args.kp_scale
        self._wait_key("A", "[*] 已在默认站姿保持。确认无误后按遥控器 [A] 启动策略...")
        while self._running and not self.io.key_pressed("A"):
            self.io.send_targets(target, kp_scale=ceiling, kd_scale=ceiling)
            self._log_pd_frame(target, ceiling)
            time.sleep(self.send_period)

    def _run_hold(self) -> None:
        """只做"插值到默认站姿并保持"，不跑策略、不等 [A]。

        专门用来在真机上单独验证 PD 增益：从 [START] 到 [A] 之间那一段本来
        没有独立入口，出了问题只能连同策略一起复现。配 --kp-scale 从低刚度
        往上试，找到不抖的值。
        """
        print(f"[*] hold 阶段：只做纯 PD 位置保持，不跑策略")
        print(f"[!] 机器人必须已吊装悬空。kp 上限 ×{self.args.kp_scale:.2f}")
        self._wait_key("start", "\n[*] 准备就绪后按遥控器 [START] 开始接管...")
        self._move_to_default()
        if not self._running:
            return
        print("[+] 保持中。[SELECT] 或 Ctrl+C 退出")

        target = DEFAULT_JOINTS.astype(np.float64)
        ceiling = self.args.kp_scale
        last_status = 0.0
        try:
            while self._running:
                self.io.send_targets(
                    target, kp_scale=ceiling, kd_scale=ceiling)
                self._log_pd_frame(target, ceiling)
                now = time.monotonic()
                self._check_safety(self.io.read_state(), now)
                if now - last_status >= 0.5:
                    last_status = now
                    self._print_status()
                time.sleep(self.send_period)
        except KeyboardInterrupt:
            self.abort("Ctrl+C")
        self._release_damping()

    def _release_damping(self) -> None:
        """转阻尼态并持续下发 DAMPING_HOLD_SEC，之后才停发。"""
        print("\n[*] 转入阻尼释放态...")
        deadline = time.monotonic() + DAMPING_HOLD_SEC
        while time.monotonic() < deadline:
            try:
                self.io.send_damping()
            except Exception as exc:
                print(f"[!] 阻尼帧发送失败: {exc}")
                break
            time.sleep(self.send_period)
        print("[+] 阻尼态结束，已停止下发")

    def _run_shadow(self) -> None:
        print("[*] shadow 阶段：只读状态、跑策略、写日志，不发送任何 lowcmd")
        print("    Ctrl+C 退出")
        policy = threading.Thread(target=self._policy_loop, daemon=True)
        policy.start()
        try:
            while self._running:
                self._print_status()
                # shadow 不发 lowcmd，因此只做断流和倾角监视，不触发阻尼。
                state = self.io.read_state()
                if state is not None and (
                    time.monotonic() - state.received_monotonic > 1.0
                ):
                    print("\n[!] rt/lowstate 断流超过 1s")
                time.sleep(0.5)
        except KeyboardInterrupt:
            self._running = False
        policy.join(timeout=1.0)

    def _run_active(self) -> None:
        print(f"[*] {self.stage} 阶段：将下发完整 29 关节 lowcmd")
        if self.stage == "hang":
            print("[!] 机器人必须已吊装悬空，脚不着地")
        else:
            print("[!] 落地站立模式。确认场地清空、有人可随时按物理急停")
        print("[!] 确认宇树官方运控已关闭，否则两个控制器会抢 rt/lowcmd")
        if self.runtime.reference_builder.allow_waist:
            print("[!] 腰部跟随已开启 —— 扭腰会直接扰动平衡策略，"
                  "该组合尚未在真机上验证过，请从小幅度开始")
        else:
            print("[*] 腰部跟随关闭 (waist_yaw 保持默认参考)，"
                  "需要时加 --waist")

        self._wait_key("start", "\n[*] 准备就绪后按遥控器 [START] 开始接管...")
        self._move_to_default()
        if not self._running:
            return
        self._hold_default()
        if not self._running:
            return

        with self._lock:
            self._takeover_time = time.monotonic()
        print("[+] 策略接管中。[SELECT] 或 Ctrl+C 退出")

        sender = threading.Thread(target=self._send_loop, daemon=True)
        policy = threading.Thread(target=self._policy_loop, daemon=True)
        sender.start()
        policy.start()
        try:
            while self._running:
                self._print_status()
                time.sleep(0.5)
        except KeyboardInterrupt:
            self.abort("Ctrl+C")
        sender.join(timeout=1.0)
        policy.join(timeout=1.0)
        self._release_damping()

    def _print_status(self) -> None:
        state = self.io.read_state()
        if state is None:
            print("\r[*] 等待 rt/lowstate...", end="", flush=True)
            return
        roll, pitch = quaternion_to_roll_pitch(state.quaternion)
        status = self.runtime.last_status
        age = status.packet_age_ms
        age_text = "--" if age is None else f"{age:.0f}"
        packets = ""
        if self.receiver is not None:
            packets = (
                f" 包={self.receiver.packet_count}"
                f" 丢={self.receiver.lost_count}"
                f" 坏={self.receiver.invalid_count}"
            )
        with self._lock:
            kp_scale = self._kp_scale
        print(
            f"\r[{self.stage}] 模式={status.mode} 包龄={age_text}ms "
            f"RP={np.degrees(roll):+.1f}/{np.degrees(pitch):+.1f}° "
            f"推理={self.runtime.last_inference_ms:.2f}ms "
            f"kp×{kp_scale:.2f} 状态帧={self.io.state_count}{packets}    ",
            end="",
            flush=True,
        )

    # ---------- 入口 ----------

    def run(self) -> int:
        print("=" * 64)
        print(f"  TWIST2 全身策略真机运行时  阶段={self.stage}")
        print(f"  策略: {self.args.policy}")
        print(f"  网卡: {self.args.net or '默认(机载本机)'}")
        if self.receiver is None:
            print("  输入: 内置双臂慢动作演示")
        else:
            print(f"  输入: UA2M/UARM UDP {self.args.bind}:{self.args.port}")
        if self.args.log is not None:
            print(f"  日志: {self.args.log}")
        print("=" * 64)

        try:
            self.io.connect()
            state = self.io.wait_for_state()
            print(f"[+] 已连接 (mode_machine={state.mode_machine})")
            if state.dof_pos.shape != (cfg.NUM_MOTORS,):
                self.abort("电机数量不是 29")
            elif not np.all(np.isfinite(state.dof_pos)):
                self.abort("初始关节角含 NaN/Inf")

            if self.aborted:
                pass
            elif self.stage == "shadow":
                self._run_shadow()
            elif self.stage == "hold":
                self._run_hold()
            else:
                self._run_active()
        except RuntimeError as exc:
            # 连接/握手类错误只报原因，不甩 traceback。
            self.abort(str(exc))
        finally:
            self._running = False
            if self.receiver is not None:
                self.receiver.close()
            self.logger.close()
            self.io.close()
            print()

        with self._lock:
            reason = self._abort_reason
        if reason is not None:
            print(f"[!] 中止原因: {reason}")
            return 1
        print("[+] 正常结束")
        return 0


def default_log_path() -> Path:
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    return EXPERIMENT_ROOT / "logs" / f"twist2_real_{timestamp}.csv"


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="TWIST2 全身策略真机运行时（宇树 G1）"
    )
    parser.add_argument(
        "--stage", choices=STAGES, default="shadow",
        help="shadow=只读不发(默认) hold=纯PD保持(调增益) hang=吊装下发 ground=落地站立",
    )
    parser.add_argument(
        "--i-know-this-can-fall", action="store_true",
        help="--stage ground 的必需确认标志",
    )
    parser.add_argument("--net", default=None, help="DDS 网卡名；机载运行时留空")
    parser.add_argument("--policy", type=Path, default=DEFAULT_POLICY)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=9627)
    parser.add_argument("--policy-hz", type=int, default=50)
    parser.add_argument("--send-hz", type=int, default=500)
    parser.add_argument("--timeout", type=float, default=0.35)
    parser.add_argument("--max-arm-speed", type=float, default=5.0)
    parser.add_argument("--prediction-ms", type=float, default=0.0)
    parser.add_argument(
        "--waist", action="store_true",
        help="允许发送端的 UAWS 尾块驱动 waist_yaw。默认关闭 —— 扭腰对"
             "平衡策略是真实扰动，先在 twist2_dynamic_sim 里验证过再打开")
    parser.add_argument(
        "--max-dof-delta-per-step", type=float, default=0.05,
        help="策略周期内允许的最大关节增量 (rad)，按发送频率等比换算",
    )
    parser.add_argument(
        "--kp-scale", type=float, default=1.0,
        help="kp/kd 的总体上限系数 (0<x<=1)。真机吊装时整机刚度过高会自激"
             "振荡, 用它从 0.3 往上试。默认 1.0 即配置文件里的原值")
    parser.add_argument("--max-tilt-deg", type=float, default=30.0)
    parser.add_argument("--max-motor-temp", type=float, default=80.0)
    parser.add_argument("--demo", action="store_true", help="不收 UDP，用内置慢动作")
    parser.add_argument("--no-log", action="store_true")
    parser.add_argument("--log", type=Path)
    args = parser.parse_args(argv)

    if not args.policy.is_file():
        parser.error(f"策略不存在: {args.policy}")
    if args.policy_hz <= 0 or args.send_hz <= 0:
        parser.error("频率必须大于 0")
    if args.send_hz < args.policy_hz:
        parser.error("send-hz 不能低于 policy-hz")
    if not 0.0 < args.kp_scale <= 1.0:
        parser.error("--kp-scale 必须在 (0, 1] 之间; 高于 1 会超出配置的增益")
    if args.stage == "ground" and not args.i_know_this_can_fall:
        parser.error(
            "--stage ground 会让机器人靠策略自己站立平衡，失稳即摔机。\n"
            "确认场地清空、有人守着物理急停后，加上 --i-know-this-can-fall 重试。"
        )
    if args.no_log:
        args.log = None
    elif args.log is None:
        args.log = default_log_path()
    return args


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    controller = Twist2RealController(args)

    def stop(_signum: int, _frame: object) -> None:
        controller.abort("收到终止信号")

    signal.signal(signal.SIGTERM, stop)
    return controller.run()


if __name__ == "__main__":
    sys.exit(main())
