#!/usr/bin/env python3
"""宇树 G1 官方遥操作思路的三 IMU 硬件适配入口。

宇树 ``xr_teleoperate`` 的 G1_23 方案使用手部 6D 位姿作为目标，并以
Pinocchio/CasADi 求解：

    50 * position + 0.5 * rotation + 0.02 * posture + 0.1 * smooth

求解结果再经过 [0.4, 0.3, 0.2, 0.1] 加权移动滤波。本项目的硬件没有
XR 手柄，输入是胸部、右大臂、右小臂三颗 IMU，因此本入口进行如下适配：

* 复用现有三 IMU 角色绑定、标定文件和相对姿态零漂抑制；
* 用大臂/小臂方向和 G1 官方 URDF 连杆长度构造腕部任务；
* 独立构造五关节手腕位姿 IK，不再调用原来的方向 IK；
* 使用官方任务空间、自然姿态、上一帧平滑和 URDF 限位目标结构；
* 增加宇树官方四帧加权移动滤波；
* 复用原上位机的 50 Hz UDP、输出限速、暂停和标定质量安全门控。

这不是把官方 XR 输入假装成 IMU 输入，而是保留官方控制结构、替换硬件
目标生成层。现有 ``dual_arm_viz.py`` 不会被这个独立入口覆盖。

官方参考：
https://github.com/unitreerobotics/xr_teleoperate
https://github.com/unitreerobotics/xr_teleoperate/blob/main/teleop/robot_control/robot_arm_ik.py
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, Sequence

import numpy as np
from scipy.optimize import minimize as scipy_minimize

from arm_solver import (
    clamp_joint_angles,
    direction_to_joint_angles,
    evaluate_direction_mapping_deg,
    forward_kinematics_left_arm_full,
    forward_kinematics_right_arm_full,
    rot_x,
)
from dual_arm_viz import DualArmViz
from robot_config import JointAngles, get_joint_limits


@dataclass(frozen=True)
class UnitreeG123ReferenceProfile:
    """宇树官方 G1_23 ``robot_arm_ik.py`` 的参考参数。"""

    translation_weight: float = 50.0
    rotation_weight: float = 0.5
    posture_weight: float = 0.02
    smooth_weight: float = 0.1
    filter_weights: tuple[float, ...] = (0.4, 0.3, 0.2, 0.1)


OFFICIAL_G1_23_PROFILE = UnitreeG123ReferenceProfile()


class OfficialWeightedMovingFilter:
    """按宇树官方新帧优先权重平滑关节角，并正确处理角度跨 ±pi。"""

    def __init__(self, weights: Sequence[float]):
        values = np.asarray(tuple(weights), dtype=float)
        if values.ndim != 1 or values.size == 0:
            raise ValueError("滤波权重必须是一维非空序列")
        if not np.all(np.isfinite(values)) or np.any(values < 0.0):
            raise ValueError("滤波权重必须是有限非负数")
        if float(np.sum(values)) <= 0.0:
            raise ValueError("滤波权重之和必须大于零")
        self._weights = values
        self._history: Dict[str, Deque[np.ndarray]] = {
            "right": deque(maxlen=values.size),
            "left": deque(maxlen=values.size),
        }

    def reset(self, side: str | None = None) -> None:
        if side is None:
            for history in self._history.values():
                history.clear()
            return
        self._history[side].clear()

    @staticmethod
    def _align_to_reference(values: np.ndarray,
                            reference: np.ndarray) -> np.ndarray:
        delta = (values - reference + np.pi) % (2.0 * np.pi) - np.pi
        return reference + delta

    def apply(self, angles: JointAngles) -> JointAngles:
        side = angles.side
        if side not in self._history:
            raise ValueError(f"不支持的手臂侧: {side}")

        sample = np.asarray(angles.as_array(), dtype=float)
        history = self._history[side]
        history.appendleft(sample)
        reference = history[0]
        aligned = np.stack([
            self._align_to_reference(item, reference) for item in history
        ])
        weights = self._weights[:len(history)]
        weights = weights / float(np.sum(weights))
        filtered = np.sum(aligned * weights[:, None], axis=0)
        filtered = (filtered + np.pi) % (2.0 * np.pi) - np.pi
        return JointAngles(
            shoulder_pitch=float(filtered[0]),
            shoulder_roll=float(filtered[1]),
            shoulder_yaw=float(filtered[2]),
            elbow=float(filtered[3]),
            wrist_roll=float(filtered[4]),
            side=side,
        )


class UnitreeOfficialImuIkAdapter:
    """以三 IMU 重建手腕位姿，独立求解官方风格的五关节 IK。"""

    def __init__(self, *, enable_filter: bool = True,
                 profile: UnitreeG123ReferenceProfile =
                 OFFICIAL_G1_23_PROFILE):
        self.profile = profile
        self.enable_filter = bool(enable_filter)
        self.filter = OfficialWeightedMovingFilter(profile.filter_weights)
        self._last_output: Dict[str, JointAngles] = {}

    @staticmethod
    def _normalize(vector: np.ndarray) -> np.ndarray:
        value = np.asarray(vector, dtype=float)
        norm = float(np.linalg.norm(value))
        if norm < 1e-8:
            raise ValueError("IMU 目标方向长度过小")
        return value / norm

    @staticmethod
    def _chest_to_urdf(vector: np.ndarray) -> np.ndarray:
        value = UnitreeOfficialImuIkAdapter._normalize(vector)
        return np.array([value[0], -value[1], value[2]], dtype=float)

    @staticmethod
    def _forward_kinematics(side: str, angles: JointAngles):
        if side == "left":
            return forward_kinematics_left_arm_full(angles)
        return forward_kinematics_right_arm_full(angles)

    @classmethod
    def _robot_palm_direction(cls, side: str, angles: JointAngles,
                              fk=None) -> np.ndarray:
        if fk is None:
            fk = cls._forward_kinematics(side, angles)
        palm_local = np.array(
            [0.0, -1.0 if side == "left" else 1.0, 0.0])
        palm = (fk["R_forearm"] @ rot_x(angles.wrist_roll)
                @ palm_local)
        return cls._normalize(palm)

    @classmethod
    def _link_lengths(cls, side: str) -> tuple[float, float]:
        zero = JointAngles(0.0, 0.0, 0.0, 0.0, 0.0, side=side)
        fk = cls._forward_kinematics(side, zero)
        upper = float(np.linalg.norm(
            fk["elbow"] - fk["shoulder_pitch"]))
        forearm = float(np.linalg.norm(fk["wrist"] - fk["elbow"]))
        return upper, forearm

    def reset(self, side: str | None = None) -> None:
        self.filter.reset(side)
        if side is None:
            self._last_output.clear()
        else:
            self._last_output.pop(side, None)

    @staticmethod
    def _max_joint_distance(first: JointAngles,
                            second: JointAngles) -> float:
        delta = (first.as_array() - second.as_array()
                 + np.pi) % (2.0 * np.pi) - np.pi
        return float(np.max(np.abs(delta)))

    def solve_pose(
        self,
        upper_dir_chest: np.ndarray,
        forearm_dir_chest: np.ndarray,
        palm_dir_chest: np.ndarray,
        side: str = "right",
        seed: JointAngles | None = None,
    ) -> tuple[JointAngles, Dict[str, float | bool]]:
        """求解三 IMU 构造的腕部位置和朝向任务。

        官方 XR 输入直接提供手腕 SE(3)。三 IMU 没有平移传感器，因此腕部
        位置由两段单位方向与 G1 URDF 连杆长度构造；小臂方向和掌心法向
        共同构成腕部旋转任务。五个关节在同一个目标函数中联合优化。
        """
        if side not in ("right", "left"):
            raise ValueError(f"不支持的手臂侧: {side}")
        if seed is None:
            seed = JointAngles(0.0, 0.0, 0.0, 0.0, 0.0, side=side)
        elif seed.side != side:
            seed = seed.with_side(side)

        # 标定重置或手动切换姿态造成种子突变时，清掉旧历史，避免旧数据
        # 被带入新的控制会话。正常连续动作由官方四帧滤波处理。
        previous = self._last_output.get(side)
        if (previous is not None
                and self._max_joint_distance(previous, seed)
                > np.radians(45.0)):
            self.reset(side)
            previous = None

        target_upper = self._chest_to_urdf(upper_dir_chest)
        target_forearm = self._chest_to_urdf(forearm_dir_chest)
        target_palm = self._chest_to_urdf(palm_dir_chest)
        upper_length, forearm_length = self._link_lengths(side)
        target_elbow = target_upper * upper_length
        target_wrist = (target_elbow
                        + target_forearm * forearm_length)

        limits = get_joint_limits(side)
        joint_names = (
            "shoulder_pitch", "shoulder_roll", "shoulder_yaw",
            "elbow", "wrist_roll",
        )
        bounds = [limits[f"{side}_{name}"] for name in joint_names]
        continuity_reference = np.asarray(seed.as_array(), dtype=float)
        palm_local = np.array(
            [0.0, -1.0 if side == "left" else 1.0, 0.0])
        straight_alignment = float(np.clip(
            np.dot(target_upper, target_forearm), 0.0, 1.0))
        redundancy_factor = straight_alignment ** 8

        def objective(values):
            candidate = JointAngles(
                shoulder_pitch=float(values[0]),
                shoulder_roll=float(values[1]),
                shoulder_yaw=float(values[2]),
                elbow=float(values[3]),
                wrist_roll=float(values[4]),
                side=side,
            )
            fk = self._forward_kinematics(side, candidate)
            solved_upper = self._normalize(
                fk["elbow"] - fk["shoulder_pitch"])
            solved_forearm = self._normalize(
                fk["wrist"] - fk["elbow"])
            solved_palm = self._normalize(
                fk["R_forearm"] @ rot_x(candidate.wrist_roll)
                @ palm_local)

            # 官方只有腕部平移任务。这里增加较弱的肘部位置项，保存大臂
            # IMU 提供的信息，否则单个腕点无法唯一决定人体肘部姿态。
            wrist_delta = ((fk["wrist"] - fk["shoulder_pitch"])
                           - target_wrist)
            elbow_delta = ((fk["elbow"] - fk["shoulder_pitch"])
                           - target_elbow)
            translation_cost = (
                float(np.dot(wrist_delta, wrist_delta))
                + 0.35 * float(np.dot(elbow_delta, elbow_delta)))

            # 小臂轴向与掌心法向共同表示手腕旋转；上臂方向是三 IMU
            # 相对官方 XR 输入多出的可观测约束。
            upper_delta = solved_upper - target_upper
            forearm_delta = solved_forearm - target_forearm
            palm_delta = solved_palm - target_palm
            rotation_cost = (
                0.5 * float(np.dot(upper_delta, upper_delta))
                + float(np.dot(forearm_delta, forearm_delta))
                + float(np.dot(palm_delta, palm_delta)))
            posture_cost = float(np.dot(values, values))
            # 两段手臂接近共线时，手腕朝向可以由 shoulder_yaw 与
            # wrist_roll 多种组合实现。对 shoulder_yaw 增加 G1 自然姿态
            # 偏置，让翻掌主要由腕关节承担，避免自然下垂时肩部外翻。
            posture_cost += (
                5.0 * redundancy_factor * float(values[2] ** 2))
            smooth_delta = np.asarray(values) - continuity_reference
            smooth_cost = float(np.dot(smooth_delta, smooth_delta))
            return (
                self.profile.translation_weight * translation_cost
                + self.profile.rotation_weight * rotation_cost
                + self.profile.posture_weight * posture_cost
                + self.profile.smooth_weight * smooth_cost)

        analytic = direction_to_joint_angles(
            upper_dir_chest, forearm_dir_chest,
            wrist_roll=seed.wrist_roll, side=side, seed=seed)
        neutral = np.zeros(5, dtype=float)
        # 官方求解器正常运行时只从上一帧热启动。首次进入或滤波历史被
        # 重置时再加入解析解与零位，既避免错误分支，又能满足实时频率。
        starts = [continuity_reference]
        if previous is None:
            starts.extend((analytic.as_array(), neutral))
        candidates = [
            scipy_minimize(
                objective, np.asarray(start, dtype=float),
                method="L-BFGS-B", bounds=bounds,
                options={"maxiter": 45, "ftol": 1e-10})
            for start in starts
        ]
        finite_candidates = [
            result for result in candidates
            if np.all(np.isfinite(result.x)) and np.isfinite(result.fun)
        ]
        if finite_candidates:
            best = min(finite_candidates, key=lambda result: result.fun)
            solved = clamp_joint_angles(JointAngles(
                float(best.x[0]), float(best.x[1]), float(best.x[2]),
                float(best.x[3]), float(best.x[4]), side=side))
            converged = bool(best.success)
            cost = float(best.fun)
        else:
            solved = clamp_joint_angles(seed)
            converged = False
            cost = float("inf")

        if self.enable_filter and np.all(np.isfinite(solved.as_array())):
            solved = clamp_joint_angles(self.filter.apply(solved))

        self._last_output[side] = solved
        diagnostics = evaluate_direction_mapping_deg(
            upper_dir_chest, forearm_dir_chest, solved)
        solved_fk = self._forward_kinematics(side, solved)
        solved_palm = self._robot_palm_direction(side, solved, solved_fk)
        palm_error_deg = float(np.degrees(np.arccos(np.clip(
            np.dot(solved_palm, target_palm), -1.0, 1.0))))
        wrist_error = float(np.linalg.norm(
            (solved_fk["wrist"] - solved_fk["shoulder_pitch"])
            - target_wrist))
        diagnostics.update({
            "official_adapter": True,
            "official_6d_pose": True,
            "official_output_filter": self.enable_filter,
            "converged": converged,
            "optimized": converged,
            "cost": cost,
            "palm_error_deg": palm_error_deg,
            "wrist_position_error_m": wrist_error,
            "posture_assisted": bool(redundancy_factor > 0.50),
        })
        return solved, diagnostics

    def solve(
        self,
        upper_dir_chest: np.ndarray,
        forearm_dir_chest: np.ndarray,
        wrist_roll: float = 0.0,
        side: str = "right",
        seed: JointAngles | None = None,
    ) -> tuple[JointAngles, Dict[str, float | bool]]:
        """兼容旧测试/调用；从种子 FK 推导掌心后进入完整位姿求解。"""
        if seed is None:
            seed = JointAngles(
                0.0, 0.0, 0.0, 0.0, wrist_roll, side=side)
        seed_fk = self._forward_kinematics(side, seed)
        palm_urdf = self._robot_palm_direction(side, seed, seed_fk)
        palm_chest = np.array(
            [palm_urdf[0], -palm_urdf[1], palm_urdf[2]])
        return self.solve_pose(
            upper_dir_chest, forearm_dir_chest, palm_chest,
            side=side, seed=seed)


class UnitreeOfficialTeleopViz(DualArmViz):
    """只在官方入口启用完整手腕位姿 IK，原上位机行为保持不变。"""

    def __init__(self, *args, ik_adapter: UnitreeOfficialImuIkAdapter,
                 **kwargs):
        self.official_ik_adapter = ik_adapter
        super().__init__(*args, **kwargs)

    def _solve_arm_mapping(self, arm):
        return self.official_ik_adapter.solve_pose(
            arm.mapping_arm_dir_chest,
            arm.mapping_forearm_dir_chest,
            arm.palm_dir_chest,
            side=arm.side,
            seed=arm.angles,
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="宇树 G1 官方遥操作结构 / 胸部+右大臂+右小臂 IMU 适配")
    parser.add_argument(
        "--udp-target", default=None,
        help="仿真用 127.0.0.1:9527；真机可填写机器人 IP:9527，省略则自动发现")
    parser.add_argument(
        "--fullscreen", action="store_true",
        help="以桌面原生分辨率全屏启动；运行中可按 F11 切换")
    parser.add_argument(
        "--disable-official-filter", action="store_true",
        help="关闭宇树官方四帧输出滤波，仅用于比较延迟和抖动")
    return parser


def main(argv: Sequence[str] | None = None) -> None:
    args = build_parser().parse_args(argv)
    adapter = UnitreeOfficialImuIkAdapter(
        enable_filter=not args.disable_official_filter)

    print("宇树 G1 官方遥操作结构：三 IMU 硬件适配模式")
    print("输入设备：胸部 IMU + 右大臂 IMU + 右小臂 IMU")
    print("标定：自动复用 imu_motor_calibration_right.json")
    print("输出滤波：" + ("[0.4, 0.3, 0.2, 0.1]"
                         if adapter.enable_filter else "已关闭"))
    if args.udp_target:
        print(f"UDP 目标：{args.udp_target}")
    else:
        print("UDP 目标：自动发现（按 SPACE 前不会启用跟随）")

    app = UnitreeOfficialTeleopViz(
        udp_target=args.udp_target,
        fullscreen=args.fullscreen,
        ik_adapter=adapter,
    )
    app.run()


if __name__ == "__main__":
    main()
