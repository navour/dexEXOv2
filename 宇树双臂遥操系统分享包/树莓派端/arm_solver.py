"""统一校准与解算模块 — arm solver for Unitree G1.

提供:
  - ArmDirectionCalibrator: IMU 方向校准 (3姿态 Procrustes)
  - TwistCalibration: 小臂扭转/掌心朝向校准
  - direction_to_joint_angles: 方向向量 → 关节角度 (IK)
  - compute_wrist_roll_from_palm: 掌心朝向 → wrist_roll 角度
  - forward_kinematics_*: 正运动学 (FK)
  - 校准姿态常量 CALIB_POSES

掌心零位定义: 侧平举时掌心朝下 = wrist_roll=0
"""

from __future__ import annotations

import math as _math
from dataclasses import dataclass
from typing import Dict

import numpy as np

from scipy.optimize import minimize as _scipy_minimize

from robot_config import (
    HUMAN_ARM_DEFAULT_LENGTH,
    RIGHT_ARM_JOINT_LIMITS,
    RIGHT_ARM_LINK_OFFSETS,
    LEFT_ARM_JOINT_LIMITS,
    LEFT_ARM_LINK_OFFSETS,
    SHOULDER_PITCH_ORIGIN_RPY_X,
    SHOULDER_ROLL_ORIGIN_RPY_X,
    LEFT_SHOULDER_PITCH_ORIGIN_RPY_X,
    LEFT_SHOULDER_ROLL_ORIGIN_RPY_X,
    JointAngles,
    get_joint_limits,
)


def normalize_quaternion(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if norm == 0:
        raise ValueError("Quaternion norm cannot be zero")
    return q / norm


def quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=float,
    )


def quat_conj(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([w, -x, -y, -z], dtype=float)


def quat_inv(q: np.ndarray) -> np.ndarray:
    q = normalize_quaternion(q)
    return quat_conj(q)


def quat_from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis_norm = np.linalg.norm(axis)
    if axis_norm == 0:
        raise ValueError("Axis norm cannot be zero")
    axis = axis / axis_norm
    half = angle * 0.5
    return np.array(
        [
            np.cos(half),
            axis[0] * np.sin(half),
            axis[1] * np.sin(half),
            axis[2] * np.sin(half),
        ],
        dtype=float,
    )


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    w, x, y, z = normalize_quaternion(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def matrix_to_quat(r: np.ndarray) -> np.ndarray:
    r = np.asarray(r, dtype=float)
    tr = np.trace(r)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    return normalize_quaternion(np.array([w, x, y, z], dtype=float))


def rot_x(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def rot_y(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rot_z(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def decompose_yxz(r: np.ndarray) -> tuple[float, float, float]:
    sr = -np.clip(r[1, 2], -1.0, 1.0)
    x_angle = np.arcsin(sr)
    cx = np.cos(x_angle)

    if abs(cx) > 1e-8:
        z_angle = np.arctan2(r[1, 0], r[1, 1])
        y_angle = np.arctan2(r[0, 2], r[2, 2])
    else:
        z_angle = 0.0
        y_angle = np.arctan2(-r[2, 0], r[0, 0])

    return y_angle, x_angle, z_angle


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min(value, max_value), min_value)


def clamp_joint_angles(angles: JointAngles) -> JointAngles:
    limits = get_joint_limits(angles.side)
    prefix = f"{angles.side}_"
    return JointAngles(
        shoulder_pitch=clamp(angles.shoulder_pitch, *limits[f"{prefix}shoulder_pitch"]),
        shoulder_roll=clamp(angles.shoulder_roll, *limits[f"{prefix}shoulder_roll"]),
        shoulder_yaw=clamp(angles.shoulder_yaw, *limits[f"{prefix}shoulder_yaw"]),
        elbow=clamp(angles.elbow, *limits[f"{prefix}elbow"]),
        wrist_roll=clamp(angles.wrist_roll, *limits[f"{prefix}wrist_roll"]),
        side=angles.side,
    )


def extract_twist_angle_about_y(q: np.ndarray) -> float:
    q = normalize_quaternion(q)
    w, _x, y, _z = q
    twist = normalize_quaternion(np.array([w, 0.0, y, 0.0], dtype=float))
    return 2.0 * np.arctan2(twist[2], twist[0])


def extract_wrist_roll(q_rel: np.ndarray, elbow_angle: float) -> float:
    """从 q_rel = inv(q_upper) * q_forearm 中提取腕部 Roll 角度.

    机器人 FK 模型: R_forearm = R_shoulder @ rot_y(elbow) @ rot_x(wrist_roll)
    因此 R_rel = rot_y(elbow) @ rot_x(wrist_roll).
    先减去 elbow (绕 Y 的旋转), 再提取绕 X 的扭转即为 wrist_roll.
    """
    q_rel = normalize_quaternion(q_rel)
    R_rel = quat_to_matrix(q_rel)
    # 减去 elbow: R_rem = rot_y(-elbow) @ R_rel = rot_x(wrist_roll)
    R_rem = rot_y(-elbow_angle) @ R_rel
    # 提取绕 X 轴的旋转角: atan2(R_rem[2,1], R_rem[1,1])
    wrist_roll = float(np.arctan2(R_rem[2, 1], R_rem[1, 1]))
    return wrist_roll


def quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """用四元数旋转向量: v' = q * v * q^-1"""
    q = normalize_quaternion(q)
    qv = np.array([0.0, v[0], v[1], v[2]], dtype=float)
    return quat_mul(quat_mul(q, qv), quat_conj(q))[1:]


@dataclass
class CalibrationOffsets:
    """IMU 校准参数。

    校准公式 (相似变换):
        q_urdf = q_transform * q_rel * q_mount

    其中:
        q_transform: 胸部体坐标系 → URDF 基坐标系的变换
        q_mount: IMU 安装偏移
        q_rel: 上臂/小臂相对于胸部的旋转 (inv(q_chest) * q_imu)
    """
    q_heading: np.ndarray       # 已弃用, 保持向后兼容
    upper_mount: np.ndarray     # 大臂 IMU 安装偏移
    forearm_mount: np.ndarray   # 小臂 IMU 安装偏移
    upper_transform: np.ndarray = None
    forearm_transform: np.ndarray = None
    use_similarity_transform: bool = False

    def __post_init__(self):
        if self.upper_transform is None:
            self.upper_transform = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        if self.forearm_transform is None:
            self.forearm_transform = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

    @property
    def upper(self) -> np.ndarray:
        return quat_mul(self.q_heading, self.upper_mount)

    @property
    def forearm(self) -> np.ndarray:
        return quat_mul(self.q_heading, self.forearm_mount)


@dataclass
class SolveDiagnostics:
    limit_triggered: bool
    limit_violations: Dict[str, float]
    unconstrained_angles: JointAngles
    clamped_error_deg: Dict[str, float]
    best_fit_error_deg: Dict[str, float]


class RightArmIMUSolver:
    def __init__(self) -> None:
        self.offsets = CalibrationOffsets(
            q_heading=np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
            upper_mount=np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
            forearm_mount=np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
        )

    def calibrate(
        self,
        q_raw_upper_cal: np.ndarray,
        q_raw_forearm_cal: np.ndarray,
        q_desired_upper: np.ndarray | None = None,
        q_desired_forearm: np.ndarray | None = None,
        upper_body_dir: np.ndarray | None = None,
        forearm_body_dir: np.ndarray | None = None,
    ) -> CalibrationOffsets:
        if q_desired_upper is None:
            q_desired_upper = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        if q_desired_forearm is None:
            q_desired_forearm = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

        q_raw_upper_cal = normalize_quaternion(q_raw_upper_cal)
        q_raw_forearm_cal = normalize_quaternion(q_raw_forearm_cal)
        q_desired_upper = normalize_quaternion(q_desired_upper)
        q_desired_forearm = normalize_quaternion(q_desired_forearm)

        if upper_body_dir is None:
            upper_body_dir = np.array([0.0, -1.0, 0.0])

        # 计算 ENU→URDF 航向角
        # 手臂在 ENU 中的方向
        arm_dir_enu = quat_rotate(q_raw_upper_cal, upper_body_dir)
        # 手臂在 URDF 中的方向 (从 FK 骨骼方向)
        arm_dir_urdf = quat_to_matrix(q_desired_upper) @ _UPPER_BONE_DIR_LOCAL
        # 航向角 = URDF XY 角度 - ENU XY 角度
        heading_angle = (
            np.arctan2(arm_dir_urdf[1], arm_dir_urdf[0])
            - np.arctan2(arm_dir_enu[1], arm_dir_enu[0])
        )
        q_heading = quat_from_axis_angle(
            np.array([0.0, 0.0, 1.0]), heading_angle
        )

        # 安装四元数: q_mount = inv(q_raw_cal) * inv(q_heading) * q_desired
        q_inv_heading = quat_inv(q_heading)
        upper_mount = quat_mul(
            quat_inv(q_raw_upper_cal),
            quat_mul(q_inv_heading, q_desired_upper),
        )
        forearm_mount = quat_mul(
            quat_inv(q_raw_forearm_cal),
            quat_mul(q_inv_heading, q_desired_forearm),
        )

        self.offsets = CalibrationOffsets(
            q_heading=normalize_quaternion(q_heading),
            upper_mount=normalize_quaternion(upper_mount),
            forearm_mount=normalize_quaternion(forearm_mount),
        )
        return self.offsets

    def apply_calibration_upper(self, q_raw: np.ndarray) -> np.ndarray:
        """将原始大臂四元数转换为 URDF 连杆四元数"""
        return normalize_quaternion(
            quat_mul(quat_mul(self.offsets.q_heading, q_raw), self.offsets.upper_mount)
        )

    def apply_calibration_forearm(self, q_raw: np.ndarray) -> np.ndarray:
        """将原始小臂四元数转换为 URDF 连杆四元数"""
        return normalize_quaternion(
            quat_mul(quat_mul(self.offsets.q_heading, q_raw), self.offsets.forearm_mount)
        )

    def solve(self, q_raw_upper: np.ndarray, q_raw_forearm: np.ndarray) -> JointAngles:
        solved, _diag = self.solve_with_diagnostics(q_raw_upper, q_raw_forearm)
        return solved

    def solve_with_diagnostics(
        self,
        q_raw_upper: np.ndarray,
        q_raw_forearm: np.ndarray,
    ) -> tuple[JointAngles, SolveDiagnostics]:
        q_raw_upper = normalize_quaternion(q_raw_upper)
        q_raw_forearm = normalize_quaternion(q_raw_forearm)

        q_upper = self.apply_calibration_upper(q_raw_upper)
        q_forearm = self.apply_calibration_forearm(q_raw_forearm)

        r_upper = quat_to_matrix(q_upper)

        r_adjusted = rot_x(-SHOULDER_PITCH_ORIGIN_RPY_X) @ r_upper
        shoulder_pitch, shoulder_roll_with_mount, shoulder_yaw = decompose_yxz(r_adjusted)
        shoulder_roll = shoulder_roll_with_mount - SHOULDER_ROLL_ORIGIN_RPY_X

        q_rel = normalize_quaternion(quat_mul(quat_inv(q_upper), q_forearm))
        elbow = extract_twist_angle_about_y(q_rel)
        wrist_roll = extract_wrist_roll(q_rel, elbow)

        angles = JointAngles(
            shoulder_pitch=shoulder_pitch,
            shoulder_roll=shoulder_roll,
            shoulder_yaw=shoulder_yaw,
            elbow=elbow,
            wrist_roll=wrist_roll,
        )
        clamped = clamp_joint_angles(angles)
        violations = compute_limit_violations(angles)
        limit_triggered = any(abs(v) > 1e-10 for v in violations.values())

        clamped_error = evaluate_mapping_consistency_deg(q_upper, q_forearm, clamped)
        if limit_triggered:
            best_fit = self._find_best_fit_within_limits(q_upper, q_forearm, clamped)
        else:
            best_fit = clamped
        best_fit_error = evaluate_mapping_consistency_deg(q_upper, q_forearm, best_fit)

        diagnostics = SolveDiagnostics(
            limit_triggered=limit_triggered,
            limit_violations=violations,
            unconstrained_angles=angles,
            clamped_error_deg=clamped_error,
            best_fit_error_deg=best_fit_error,
        )
        return best_fit, diagnostics

    def _find_best_fit_within_limits(
        self,
        q_upper_target: np.ndarray,
        q_forearm_target: np.ndarray,
        seed: JointAngles,
    ) -> JointAngles:
        current = clamp_joint_angles(seed)
        current_cost = self._mapping_cost(q_upper_target, q_forearm_target, current)

        step_schedule_deg = [18.0, 10.0, 5.0, 2.0, 1.0]
        joint_names = [
            "right_shoulder_pitch",
            "right_shoulder_roll",
            "right_shoulder_yaw",
            "right_elbow",
        ]

        for step_deg in step_schedule_deg:
            step = float(np.radians(step_deg))
            improved = True
            while improved:
                improved = False
                for name in joint_names:
                    for direction in (-1.0, 1.0):
                        candidate = JointAngles(
                            shoulder_pitch=current.shoulder_pitch,
                            shoulder_roll=current.shoulder_roll,
                            shoulder_yaw=current.shoulder_yaw,
                            elbow=current.elbow,
                            wrist_roll=current.wrist_roll,
                        )
                        if name == "right_shoulder_pitch":
                            candidate.shoulder_pitch += direction * step
                        elif name == "right_shoulder_roll":
                            candidate.shoulder_roll += direction * step
                        elif name == "right_shoulder_yaw":
                            candidate.shoulder_yaw += direction * step
                        else:
                            candidate.elbow += direction * step

                        candidate = clamp_joint_angles(candidate)
                        candidate_cost = self._mapping_cost(
                            q_upper_target,
                            q_forearm_target,
                            candidate,
                        )
                        if candidate_cost + 1e-9 < current_cost:
                            current = candidate
                            current_cost = candidate_cost
                            improved = True
        return current

    @staticmethod
    def _mapping_cost(
        q_upper_target: np.ndarray,
        q_forearm_target: np.ndarray,
        candidate: JointAngles,
    ) -> float:
        err = evaluate_mapping_consistency_deg(q_upper_target, q_forearm_target, candidate)
        return (err["upper_error_deg"] ** 2) + (err["forearm_error_deg"] ** 2)


def compute_limit_violations(angles: JointAngles) -> Dict[str, float]:
    limits = get_joint_limits(angles.side)
    violations: Dict[str, float] = {}
    for key, value in angles.as_dict().items():
        lower, upper = limits[key]
        if value < lower:
            violations[key] = value - lower
        elif value > upper:
            violations[key] = value - upper
        else:
            violations[key] = 0.0
    return violations


def compose_shoulder_rotation(angles: JointAngles) -> np.ndarray:
    """合成肩部旋转矩阵，根据 arm side 使用正确的 origin RPY."""
    if angles.side == "left":
        pitch_origin = LEFT_SHOULDER_PITCH_ORIGIN_RPY_X
        roll_origin = LEFT_SHOULDER_ROLL_ORIGIN_RPY_X
    else:
        pitch_origin = SHOULDER_PITCH_ORIGIN_RPY_X
        roll_origin = SHOULDER_ROLL_ORIGIN_RPY_X
    return (
        rot_x(pitch_origin)
        @ rot_y(angles.shoulder_pitch)
        @ rot_x(roll_origin)
        @ rot_x(angles.shoulder_roll)
        @ rot_z(angles.shoulder_yaw)
    )


def synthesize_imu_from_joint_angles(angles: JointAngles) -> tuple[np.ndarray, np.ndarray]:
    r_upper = compose_shoulder_rotation(angles)
    r_forearm = r_upper @ rot_y(angles.elbow) @ rot_x(angles.wrist_roll)
    return matrix_to_quat(r_upper), matrix_to_quat(r_forearm)


# 骨骼方向向量: 延迟到 forward_kinematics_right_arm_full 定义后初始化
_UPPER_BONE_DIR_LOCAL: np.ndarray
_FOREARM_BONE_DIR_LOCAL: np.ndarray


def quaternion_angular_distance_deg(q1: np.ndarray, q2: np.ndarray) -> float:
    q1 = normalize_quaternion(q1)
    q2 = normalize_quaternion(q2)
    dot = abs(float(np.dot(q1, q2)))
    dot = np.clip(dot, -1.0, 1.0)
    return float(np.degrees(2.0 * np.arccos(dot)))


def evaluate_mapping_consistency_deg(
    q_upper_target: np.ndarray,
    q_forearm_target: np.ndarray,
    solved: JointAngles,
) -> Dict[str, float]:
    q_upper_recon, q_forearm_recon = synthesize_imu_from_joint_angles(solved)
    upper_err = quaternion_angular_distance_deg(q_upper_target, q_upper_recon)
    forearm_err = quaternion_angular_distance_deg(q_forearm_target, q_forearm_recon)
    return {
        "upper_error_deg": upper_err,
        "forearm_error_deg": forearm_err,
        "max_error_deg": max(upper_err, forearm_err),
    }


def forward_kinematics_right_arm(angles: JointAngles) -> Dict[str, np.ndarray]:
    shoulder = np.zeros(3, dtype=float)

    r = rot_x(SHOULDER_PITCH_ORIGIN_RPY_X) @ rot_y(angles.shoulder_pitch)
    p = shoulder.copy()

    p = p + r @ RIGHT_ARM_LINK_OFFSETS["shoulder_roll_origin_xyz"]
    r = r @ rot_x(SHOULDER_ROLL_ORIGIN_RPY_X) @ rot_x(angles.shoulder_roll)

    p = p + r @ RIGHT_ARM_LINK_OFFSETS["shoulder_yaw_origin_xyz"]
    r = r @ rot_z(angles.shoulder_yaw)

    elbow = p + r @ RIGHT_ARM_LINK_OFFSETS["elbow_origin_xyz"]

    r = r @ rot_y(angles.elbow)
    wrist = elbow + r @ RIGHT_ARM_LINK_OFFSETS["wrist_roll_origin_xyz"]

    return {"shoulder": shoulder, "elbow": elbow, "wrist": wrist}


def forward_kinematics_human_from_imus(
    q_upper: np.ndarray,
    q_forearm: np.ndarray,
    upper_length: float = 0.30,
    forearm_length: float = 0.26,
) -> Dict[str, np.ndarray]:
    shoulder = np.zeros(3, dtype=float)
    r_upper = quat_to_matrix(normalize_quaternion(q_upper))
    r_forearm = quat_to_matrix(normalize_quaternion(q_forearm))

    elbow = shoulder + r_upper @ (_UPPER_BONE_DIR_LOCAL * upper_length)
    wrist = elbow + r_forearm @ (_FOREARM_BONE_DIR_LOCAL * forearm_length)

    return {"shoulder": shoulder, "elbow": elbow, "wrist": wrist}


def preliminary_zero_pose_validation() -> Dict[str, float | bool]:
    zero = JointAngles(0.0, 0.0, 0.0, 0.0)
    points = forward_kinematics_right_arm(zero)

    upper_vec = points["elbow"] - points["shoulder"]
    forearm_vec = points["wrist"] - points["elbow"]

    upper_dir = upper_vec / np.linalg.norm(upper_vec)
    forearm_dir = forearm_vec / np.linalg.norm(forearm_vec)

    dot = np.clip(np.dot(upper_dir, forearm_dir), -1.0, 1.0)
    included_angle_deg = float(np.degrees(np.arccos(dot)))

    is_upper_natural_down = bool(upper_dir[2] < -0.45)
    is_elbow_lift_like_90 = bool(60.0 <= included_angle_deg <= 120.0)

    return {
        "upper_direction_z": float(upper_dir[2]),
        "upper_forearm_included_angle_deg": included_angle_deg,
        "is_upper_natural_down": is_upper_natural_down,
        "is_elbow_lift_like_90": is_elbow_lift_like_90,
        "preliminary_pass": bool(is_upper_natural_down and is_elbow_lift_like_90),
    }


# ============================================================
#  扩展: 完整FK (含旋转矩阵)、用户位姿、逆运动学
# ============================================================

# IK 模式常量
IK_MODE_ORIENTATION = 'balanced'      # 保留兼容, 均指向 balanced
IK_MODE_POSITION    = 'balanced'      # 保留兼容, 均指向 balanced
IK_MODE_BALANCED    = 'balanced'      # 均衡

# 工作空间缩放
_ROBOT_APPROX_REACH = 0.32
_HUMAN_APPROX_REACH = HUMAN_ARM_DEFAULT_LENGTH["upper"] + HUMAN_ARM_DEFAULT_LENGTH["forearm"]
_REACH_SCALE = _ROBOT_APPROX_REACH / _HUMAN_APPROX_REACH

# 各模式权重 (w_upper_rot, w_forearm_dir, w_pos)
# cost = w_upper * rot_err_upper + w_forearm * forearm_dir_err + w_pos * ||wrist_pos_err||^2
# 小臂使用方向匹配 (忽略绕长轴自转), 权重相对旋转匹配翻倍以补偿量程差异
_MODE_WEIGHTS = {
    IK_MODE_BALANCED:    (2.0,  4.0, 0.0),    # 均衡: 大臂旋转 + 小臂方向
}

# 预计算的 IK 常量 (避免每帧重复计算)
_ROT_X_NEG_SP = rot_x(-SHOULDER_PITCH_ORIGIN_RPY_X)
_CA_SP = _math.cos(SHOULDER_PITCH_ORIGIN_RPY_X)
_SA_SP = _math.sin(SHOULDER_PITCH_ORIGIN_RPY_X)
_SR_ORIGIN = SHOULDER_ROLL_ORIGIN_RPY_X
_JOINT_BOUNDS = [
    RIGHT_ARM_JOINT_LIMITS["right_shoulder_pitch"],
    RIGHT_ARM_JOINT_LIMITS["right_shoulder_roll"],
    RIGHT_ARM_JOINT_LIMITS["right_shoulder_yaw"],
    RIGHT_ARM_JOINT_LIMITS["right_elbow"],
]


def forward_kinematics_right_arm_full(angles: JointAngles) -> Dict[str, np.ndarray]:
    """正运动学: 返回所有关节位置、旋转矩阵。

    机器人右肩是三个电机串联的连杆系统:
        shoulder_pitch → shoulder_roll → shoulder_yaw → elbow → wrist

    Returns
    -------
    dict with keys:
        shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist : np.ndarray (3,)
        shoulder : 同 shoulder_pitch (兼容旧接口)
        R_shoulder_pitch, R_shoulder_roll, R_shoulder_yaw : 3×3
        R_upper  : 3×3 肘前旋转矩阵 (同 R_shoulder_yaw)
        R_forearm: 3×3 腕前旋转矩阵
    """
    p_pitch = np.zeros(3, dtype=float)

    R = rot_x(SHOULDER_PITCH_ORIGIN_RPY_X) @ rot_y(angles.shoulder_pitch)
    R_pitch = R.copy()

    p_roll = p_pitch + R @ RIGHT_ARM_LINK_OFFSETS["shoulder_roll_origin_xyz"]
    R = R @ rot_x(SHOULDER_ROLL_ORIGIN_RPY_X) @ rot_x(angles.shoulder_roll)
    R_roll = R.copy()

    p_yaw = p_roll + R @ RIGHT_ARM_LINK_OFFSETS["shoulder_yaw_origin_xyz"]
    R = R @ rot_z(angles.shoulder_yaw)
    R_yaw = R.copy()

    elbow = p_yaw + R @ RIGHT_ARM_LINK_OFFSETS["elbow_origin_xyz"]
    R_upper = R.copy()

    R = R @ rot_y(angles.elbow)

    R = R @ rot_x(angles.wrist_roll)
    R_forearm = R.copy()

    wrist = elbow + R @ RIGHT_ARM_LINK_OFFSETS["wrist_roll_origin_xyz"]

    return {
        "shoulder_pitch": p_pitch,
        "shoulder_roll": p_roll,
        "shoulder_yaw": p_yaw,
        "elbow": elbow,
        "wrist": wrist,
        "shoulder": p_pitch,  # 兼容
        "R_shoulder_pitch": R_pitch,
        "R_shoulder_roll": R_roll,
        "R_shoulder_yaw": R_yaw,
        "R_upper": R_upper,
        "R_forearm": R_forearm,
    }


# 计算骨骼方向向量 (局部坐标系, 从零位 FK 反推)
def _compute_bone_dirs():
    """计算大臂/小臂骨骼在关节旋转坐标系中的方向向量和长度。

    URDF 中各连杆偏移向量不沿 X 轴, 真正的骨骼方向必须从 FK 零位反推。
    同时返回骨骼长度供位置误差计算使用。
    """
    zero = JointAngles(0.0, 0.0, 0.0, 0.0)
    fk = forward_kinematics_right_arm_full(zero)
    R_upper = compose_shoulder_rotation(zero)
    R_forearm = R_upper  # rot_y(0) = I

    upper_vec = fk["elbow"] - fk["shoulder_pitch"]
    upper_len = float(np.linalg.norm(upper_vec))
    upper_local = R_upper.T @ upper_vec
    upper_local /= upper_len

    forearm_vec = fk["wrist"] - fk["elbow"]
    forearm_len = float(np.linalg.norm(forearm_vec))
    forearm_local = R_forearm.T @ forearm_vec
    forearm_local /= forearm_len

    return upper_local, forearm_local, upper_len, forearm_len


# 初始化骨骼方向 + 长度常量
_UPPER_BONE_DIR_LOCAL, _FOREARM_BONE_DIR_LOCAL, _ROBOT_UPPER_LEN, _ROBOT_FOREARM_LEN = _compute_bone_dirs()

# 预计算位置误差常用向量 (骨骼方向 * 长度)
_UPPER_BONE_VEC = _UPPER_BONE_DIR_LOCAL * _ROBOT_UPPER_LEN
_FOREARM_BONE_VEC = _FOREARM_BONE_DIR_LOCAL * _ROBOT_FOREARM_LEN

# 人体骨骼向量 (用于坐标距离误差: 人体手腕 vs 机器人手腕)
_HUMAN_UPPER_LEN = HUMAN_ARM_DEFAULT_LENGTH["upper"]
_HUMAN_FOREARM_LEN = HUMAN_ARM_DEFAULT_LENGTH["forearm"]
_HUMAN_UPPER_BONE_VEC = _UPPER_BONE_DIR_LOCAL * _HUMAN_UPPER_LEN
_HUMAN_FOREARM_BONE_VEC = _FOREARM_BONE_DIR_LOCAL * _HUMAN_FOREARM_LEN

# Delta position mapping: 位置增量映射偏移
# 校准姿态 (R=I) 下: robot_wrist_neutral + (human_wrist - human_wrist_neutral) = human_wrist + offset
_ROBOT_WRIST_NEUTRAL = _UPPER_BONE_VEC + _FOREARM_BONE_VEC
_HUMAN_WRIST_NEUTRAL = _HUMAN_UPPER_BONE_VEC + _HUMAN_FOREARM_BONE_VEC
_POS_DELTA_OFFSET = _ROBOT_WRIST_NEUTRAL - _HUMAN_WRIST_NEUTRAL


def compute_human_forearm_end_pose(
    q_upper: np.ndarray,
    q_forearm: np.ndarray,
    upper_length: float | None = None,
    forearm_length: float | None = None,
) -> Dict[str, np.ndarray]:
    """从校准后的 IMU 四元数计算用户小臂末端位姿。

    骨骼方向使用 _UPPER_BONE_DIR_LOCAL / _FOREARM_BONE_DIR_LOCAL,
    这些是从 URDF FK 零位反推的真实连杆方向 (非简单 X 轴)。

    Returns
    -------
    dict with keys:
        shoulder, elbow, wrist : np.ndarray (3,)
        R_upper  : 3×3 大臂旋转矩阵
        R_forearm: 3×3 小臂旋转矩阵
    """
    if upper_length is None:
        upper_length = HUMAN_ARM_DEFAULT_LENGTH["upper"]
    if forearm_length is None:
        forearm_length = HUMAN_ARM_DEFAULT_LENGTH["forearm"]

    shoulder = np.zeros(3, dtype=float)
    R_upper = quat_to_matrix(normalize_quaternion(q_upper))
    R_forearm = quat_to_matrix(normalize_quaternion(q_forearm))

    elbow = shoulder + R_upper @ (_UPPER_BONE_DIR_LOCAL * upper_length)
    wrist = elbow + R_forearm @ (_FOREARM_BONE_DIR_LOCAL * forearm_length)

    return {
        "shoulder": shoulder,
        "elbow": elbow,
        "wrist": wrist,
        "R_upper": R_upper,
        "R_forearm": R_forearm,
    }


def rotation_matrix_error_deg(R1: np.ndarray, R2: np.ndarray) -> float:
    """两个旋转矩阵之间的角度误差 (度)。"""
    R_err = R1.T @ R2
    tr = np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(tr)))


def inverse_kinematics_solve(
    q_upper_target: np.ndarray,
    q_forearm_target: np.ndarray,
    seed: JointAngles | None = None,
    mode: str = IK_MODE_BALANCED,
) -> tuple[JointAngles, Dict]:
    """逆运动学解算: 给定目标 IMU 四元数, 求机器人关节角度。

    优化策略:
      1. 先尝试解析解 (从四元数直接分解关节角度)
      2. 总是走 L-BFGS-B 优化器 (balanced 模式)
      3. seed 连续性保护: 只有当其他起点 cost 显著低于 seed 时才切换

    Parameters
    ----------
    q_upper_target  : 用户大臂校准后四元数 [w,x,y,z]
    q_forearm_target: 用户小臂校准后四元数 [w,x,y,z]
    seed            : 优化初始值 (用于热启动)
    mode            : IK_MODE_BALANCED (仅保留均衡模式)

    Returns
    -------
    (solved_angles, diagnostics)
    """
    mode = IK_MODE_BALANCED  # 强制均衡模式

    q_upper_target = normalize_quaternion(q_upper_target)
    q_forearm_target = normalize_quaternion(q_forearm_target)

    # 预计算目标旋转矩阵
    _tRu = quat_to_matrix(q_upper_target)      # target R_upper
    _tRf = quat_to_matrix(q_forearm_target)    # target R_forearm

    # === 1. 解析解: 从目标四元数直接分解关节角度 ===
    _R_adj = _ROT_X_NEG_SP @ _tRu
    sp_a, sr_m_a, sy_a = decompose_yxz(_R_adj)
    sr_a = sr_m_a - _SR_ORIGIN
    _v_el = _tRu.T @ (_tRf @ _FOREARM_BONE_DIR_LOCAL)
    _d_el = _FOREARM_BONE_DIR_LOCAL
    el_a = float(np.arctan2(
        _v_el[0] * _d_el[2] - _v_el[2] * _d_el[0],
        _v_el[0] * _d_el[0] + _v_el[2] * _d_el[2],
    ))
    _q_rel_ik = normalize_quaternion(quat_mul(quat_inv(q_upper_target), q_forearm_target))
    wr_a = extract_wrist_roll(_q_rel_ik, el_a)

    ideal_angles = JointAngles(sp_a, sr_a, sy_a, el_a, wr_a)
    ideal_clamped = clamp_joint_angles(ideal_angles)

    # === 2. 总是走优化器 (balanced 需要释放 shoulder_yaw 优化小臂方向) ===
    if True:
        # === 优化求解 ===
        w_upper, w_forearm, w_pos = _MODE_WEIGHTS[IK_MODE_BALANCED]

        # 内联标量代价函数 — 用 math.sin/cos 替代 numpy 避免数组分配开销
        tu00, tu01, tu02 = float(_tRu[0, 0]), float(_tRu[0, 1]), float(_tRu[0, 2])
        tu10, tu11, tu12 = float(_tRu[1, 0]), float(_tRu[1, 1]), float(_tRu[1, 2])
        tu20, tu21, tu22 = float(_tRu[2, 0]), float(_tRu[2, 1]), float(_tRu[2, 2])
        tf00, tf01, tf02 = float(_tRf[0, 0]), float(_tRf[0, 1]), float(_tRf[0, 2])
        tf10, tf11, tf12 = float(_tRf[1, 0]), float(_tRf[1, 1]), float(_tRf[1, 2])
        tf20, tf21, tf22 = float(_tRf[2, 0]), float(_tRf[2, 1]), float(_tRf[2, 2])

        # 小臂骨骼方向 (局部坐标系, 归一化) — 用于方向匹配 (忽略轴向旋转)
        _fd0 = float(_FOREARM_BONE_DIR_LOCAL[0])
        _fd1 = float(_FOREARM_BONE_DIR_LOCAL[1])
        _fd2 = float(_FOREARM_BONE_DIR_LOCAL[2])
        # 目标小臂方向 (世界坐标系)
        _tdx = tf00*_fd0 + tf01*_fd1 + tf02*_fd2
        _tdy = tf10*_fd0 + tf11*_fd1 + tf12*_fd2
        _tdz = tf20*_fd0 + tf21*_fd1 + tf22*_fd2

        # 大臂骨骼方向 (局部坐标系) — BALANCED 模式用方向匹配释放 shoulder_yaw
        _ud0 = float(_UPPER_BONE_DIR_LOCAL[0])
        _ud1 = float(_UPPER_BONE_DIR_LOCAL[1])
        _ud2 = float(_UPPER_BONE_DIR_LOCAL[2])
        # 目标大臂方向 (世界坐标系)
        _tudx = tu00*_ud0 + tu01*_ud1 + tu02*_ud2
        _tudy = tu10*_ud0 + tu11*_ud1 + tu12*_ud2
        _tudz = tu20*_ud0 + tu21*_ud1 + tu22*_ud2
        _upper_dir_only = True  # 均衡模式: 大臂用方向匹配

        _cos = _math.cos
        _sin = _math.sin
        _ca = _CA_SP
        _sa = _SA_SP
        _b = _SR_ORIGIN

        # 预计算骨骼向量: 均衡模式统一使用机器人臂长
        _bu0, _bu1, _bu2 = float(_UPPER_BONE_VEC[0]), float(_UPPER_BONE_VEC[1]), float(_UPPER_BONE_VEC[2])
        _bf0, _bf1, _bf2 = float(_FOREARM_BONE_VEC[0]), float(_FOREARM_BONE_VEC[1]), float(_FOREARM_BONE_VEC[2])
        _tbu0, _tbu1, _tbu2 = _bu0, _bu1, _bu2
        _tbf0, _tbf1, _tbf2 = _bf0, _bf1, _bf2
        # target_elbow = tRu @ bone_upper_target; target_wrist = target_elbow + tRf @ bone_forearm_target
        _tex = tu00*_tbu0 + tu01*_tbu1 + tu02*_tbu2
        _tey = tu10*_tbu0 + tu11*_tbu1 + tu12*_tbu2
        _tez = tu20*_tbu0 + tu21*_tbu1 + tu22*_tbu2
        _twx = _tex + tf00*_tbf0 + tf01*_tbf1 + tf02*_tbf2
        _twy = _tey + tf10*_tbf0 + tf11*_tbf1 + tf12*_tbf2
        _twz = _tez + tf20*_tbf0 + tf21*_tbf1 + tf22*_tbf2

        def cost(x: np.ndarray) -> float:
            sp, sr, sy, el = x[0], x[1], x[2], x[3]
            csp = _cos(sp); ssp = _sin(sp)
            phi = _b + sr
            cphi = _cos(phi); sphi = _sin(phi)
            csy = _cos(sy); ssy = _sin(sy)

            # R_upper = Rx(a) @ Ry(sp) @ Rx(phi) @ Rz(sy)  —— 展开为标量运算
            # Row 0
            b01 = ssp * sphi
            b02 = ssp * cphi
            r00 = csp * csy + b01 * ssy
            r01 = -csp * ssy + b01 * csy
            r02 = b02

            # Row 1
            sa_ssp = _sa * ssp
            sa_csp = _sa * csp
            b11 = _ca * cphi - sa_csp * sphi
            b12 = -_ca * sphi - sa_csp * cphi
            r10 = sa_ssp * csy + b11 * ssy
            r11 = -sa_ssp * ssy + b11 * csy
            r12 = b12

            # Row 2
            ca_ssp = _ca * ssp
            ca_csp = _ca * csp
            b21 = _sa * cphi + ca_csp * sphi
            b22 = -_sa * sphi + ca_csp * cphi
            r20 = -ca_ssp * csy + b21 * ssy
            r21 = ca_ssp * ssy + b21 * csy
            r22 = b22

            # Upper Frobenius inner product
            fro_u = (tu00*r00 + tu01*r01 + tu02*r02 +
                     tu10*r10 + tu11*r11 + tu12*r12 +
                     tu20*r20 + tu21*r21 + tu22*r22)

            # R_forearm = R_upper @ Ry(el)
            cel = _cos(el); sel = _sin(el)
            rf00 = r00*cel - r02*sel;  rf02 = r00*sel + r02*cel
            rf10 = r10*cel - r12*sel;  rf12 = r10*sel + r12*cel
            rf20 = r20*cel - r22*sel;  rf22 = r20*sel + r22*cel

            # 小臂方向匹配 (忽略绕骨骼长轴的自转, 只匹配骨骼朝向)
            sdx = rf00*_fd0 + r01*_fd1 + rf02*_fd2
            sdy = rf10*_fd0 + r11*_fd1 + rf12*_fd2
            sdz = rf20*_fd0 + r21*_fd1 + rf22*_fd2
            dir_dot = sdx*_tdx + sdy*_tdy + sdz*_tdz

            # 大臂方向匹配 (2 DOF, 释放 shoulder_yaw)
            sudx = r00*_ud0 + r01*_ud1 + r02*_ud2
            sudy = r10*_ud0 + r11*_ud1 + r12*_ud2
            sudz = r20*_ud0 + r21*_ud1 + r22*_ud2
            udir_dot = sudx*_tudx + sudy*_tudy + sudz*_tudz
            total = w_upper * (1.0 - udir_dot) + w_forearm * (1.0 - dir_dot)

            return total

        # 用解析解 (clamped) 做主种子 — 最接近精确解的合法点
        x0 = ideal_clamped.as_array()
        best = _scipy_minimize(
            cost, x0, method="L-BFGS-B", bounds=_JOINT_BOUNDS,
            options={"maxiter": 80, "ftol": 1e-12},
        )

        # 用户种子 (帧间连续性) — 优先保持连续, 只有显著更好时才切换
        _SEED_HYSTERESIS = 1.3  # 容忍 seed 方案 cost 高 30%
        if seed is not None:
            r_seed = _scipy_minimize(
                cost, seed.as_array(), method="L-BFGS-B", bounds=_JOINT_BOUNDS,
                options={"maxiter": 80, "ftol": 1e-12},
            )
            if r_seed.fun < best.fun * _SEED_HYSTERESIS:
                best = r_seed

        # 若误差偏大, 从零位再试
        if best.fun > 0.01:
            r3 = _scipy_minimize(
                cost, np.zeros(4), method="L-BFGS-B", bounds=_JOINT_BOUNDS,
                options={"maxiter": 150, "ftol": 1e-12},
            )
            if r3.fun < best.fun:
                best = r3

        # 若仍偏大, 多起点搜索
        if best.fun > 0.1:
            for sp in [-1.0, 0.0, 1.0]:
                for sr in [-1.2, 0.0]:
                    for el in [0.0, 1.0]:
                        s = np.array([sp, sr, 0.0, el])
                        for i in range(4):
                            s[i] = clamp(s[i], _JOINT_BOUNDS[i][0], _JOINT_BOUNDS[i][1])
                        r = _scipy_minimize(
                            cost, s, method="L-BFGS-B", bounds=_JOINT_BOUNDS,
                            options={"maxiter": 200, "ftol": 1e-14},
                        )
                        if r.fun < best.fun:
                            best = r

        solved_base = JointAngles(best.x[0], best.x[1], best.x[2], best.x[3])
        _q_rel_opt = normalize_quaternion(quat_mul(quat_inv(q_upper_target), q_forearm_target))
        _wr_opt = extract_wrist_roll(_q_rel_opt, solved_base.elbow)
        solved = JointAngles(
            solved_base.shoulder_pitch, solved_base.shoulder_roll,
            solved_base.shoulder_yaw, solved_base.elbow, _wr_opt,
        )
        _converged = best.success
        _cost_val = float(best.fun)

    # ---- 诊断信息 ----
    robot_fk = forward_kinematics_right_arm_full(solved)
    human_pose = compute_human_forearm_end_pose(q_upper_target, q_forearm_target)

    # 旋转矩阵误差 (度)
    orient_err = evaluate_mapping_consistency_deg(q_upper_target, q_forearm_target, solved)

    # 小臂方向误差 (度) — 骨骼朝向 (忽略轴向旋转)
    target_fore_dir = human_pose["R_forearm"] @ _FOREARM_BONE_DIR_LOCAL
    robot_fore_dir = robot_fk["R_forearm"] @ _FOREARM_BONE_DIR_LOCAL
    dir_cos = float(np.clip(np.dot(robot_fore_dir, target_fore_dir), -1.0, 1.0))
    forearm_dir_error_deg = float(np.degrees(np.arccos(dir_cos)))

    # 坐标距离误差: 人体手腕 (人体臂长) vs 机器人手腕 (机器人臂长)
    R_upper_solved = robot_fk["R_upper"]
    R_forearm_solved = robot_fk["R_forearm"]

    # 人体手腕坐标 (使用人体臂长)
    target_elbow_pos = _tRu @ _HUMAN_UPPER_BONE_VEC
    target_wrist_pos = target_elbow_pos + _tRf @ _HUMAN_FOREARM_BONE_VEC
    # 机器人手腕坐标 (使用机器人臂长)
    solved_elbow_pos = R_upper_solved @ _UPPER_BONE_VEC
    solved_wrist_pos = solved_elbow_pos + R_forearm_solved @ _FOREARM_BONE_VEC

    pos_error_vec = target_wrist_pos - solved_wrist_pos  # XYZ 各轴误差
    pos_error_m = float(np.linalg.norm(pos_error_vec))

    t_norm = float(np.linalg.norm(target_wrist_pos))
    s_norm = float(np.linalg.norm(solved_wrist_pos))
    if t_norm > 1e-8 and s_norm > 1e-8:
        pos_dir_cos = float(np.clip(np.dot(target_wrist_pos / t_norm, solved_wrist_pos / s_norm), -1.0, 1.0))
        pos_direction_error_deg = float(np.degrees(np.arccos(pos_dir_cos)))
    else:
        pos_direction_error_deg = 0.0

    ideal_wrist = target_wrist_pos
    ideal_elbow = target_elbow_pos

    diagnostics = {
        "forearm_dir_error_deg": forearm_dir_error_deg,
        "pos_error_m": pos_error_m,
        "pos_error_xyz_m": [float(pos_error_vec[0]), float(pos_error_vec[1]), float(pos_error_vec[2])],
        "pos_direction_error_deg": pos_direction_error_deg,
        "upper_rot_error_deg": orient_err["upper_error_deg"],
        "forearm_rot_error_deg": orient_err["forearm_error_deg"],
        "max_orient_error_deg": orient_err["max_error_deg"],
        "mode": mode,
        "converged": _converged,
        "cost": _cost_val,
        "human_wrist": human_pose["wrist"],
        "robot_wrist": robot_fk["wrist"],
        "ideal_wrist": ideal_wrist,
        "human_elbow": human_pose["elbow"],
        "robot_elbow": robot_fk["elbow"],
        "ideal_elbow": ideal_elbow,
        "human_R_forearm": human_pose["R_forearm"],
        "robot_R_forearm": robot_fk["R_forearm"],
    }

    return solved, diagnostics


# ============================================================
#  左臂支持
# ============================================================

def _quat_left_matrix(q):
    """四元数左乘矩阵: QL(q) @ v = quat_mul(q, v)"""
    w, x, y, z = q
    return np.array([
        [w, -x, -y, -z],
        [x,  w, -z,  y],
        [y,  z,  w, -x],
        [z, -y,  x,  w],
    ])


def _quat_right_matrix(q):
    """四元数右乘矩阵: QR(q) @ v = quat_mul(v, q)"""
    w, x, y, z = q
    return np.array([
        [w, -x, -y, -z],
        [x,  w,  z, -y],
        [y, -z,  w,  x],
        [z,  y, -x,  w],
    ])


def _solve_similarity_params(q_rel1, q_urdf1, q_rel2, q_urdf2):
    """求解变换参数 A, B 使得 q_urdf = A * q_rel * B"""
    C = quat_mul(q_rel1, quat_inv(q_rel2))
    D = quat_mul(q_urdf1, quat_inv(q_urdf2))
    M = _quat_left_matrix(C) - _quat_right_matrix(D)
    _, S, Vt = np.linalg.svd(M)

    best_A = None
    best_err = float('inf')
    for i in [-1, -2]:
        candidate = normalize_quaternion(Vt[i])
        if candidate[0] < 0:
            candidate = -candidate
        B_cand = normalize_quaternion(
            quat_mul(quat_inv(quat_mul(candidate, q_rel1)), q_urdf1)
        )
        q_v1 = normalize_quaternion(quat_mul(quat_mul(candidate, q_rel1), B_cand))
        q_v2 = normalize_quaternion(quat_mul(quat_mul(candidate, q_rel2), B_cand))
        err1 = 1.0 - abs(float(np.dot(q_v1, q_urdf1)))
        err2 = 1.0 - abs(float(np.dot(q_v2, q_urdf2)))
        err = err1 + err2
        if err < best_err:
            best_err = err
            best_A = candidate

    def _cost(params):
        a = normalize_quaternion(np.array(params))
        b = normalize_quaternion(quat_mul(quat_inv(quat_mul(a, q_rel1)), q_urdf1))
        q_v1 = normalize_quaternion(quat_mul(quat_mul(a, q_rel1), b))
        q_v2 = normalize_quaternion(quat_mul(quat_mul(a, q_rel2), b))
        e1 = 1.0 - abs(float(np.dot(q_v1, q_urdf1)))
        e2 = 1.0 - abs(float(np.dot(q_v2, q_urdf2)))
        return e1 + e2

    result = _scipy_minimize(_cost, best_A, method='Nelder-Mead',
                             options={'xatol': 1e-12, 'fatol': 1e-15, 'maxiter': 5000})
    A = normalize_quaternion(np.array(result.x))
    if A[0] < 0:
        A = -A
    B = normalize_quaternion(quat_mul(quat_inv(quat_mul(A, q_rel1)), q_urdf1))
    return A, B


def forward_kinematics_left_arm(angles: JointAngles) -> Dict[str, np.ndarray]:
    """左臂简化 FK: 返回肩、肘、腕三点位置。"""
    if angles.side != "left":
        angles = angles.with_side("left")
    shoulder = np.zeros(3, dtype=float)
    link_offsets = LEFT_ARM_LINK_OFFSETS

    R = rot_x(LEFT_SHOULDER_PITCH_ORIGIN_RPY_X) @ rot_y(angles.shoulder_pitch)
    p = shoulder.copy()

    p = p + R @ link_offsets["shoulder_roll_origin_xyz"]
    R = R @ rot_x(LEFT_SHOULDER_ROLL_ORIGIN_RPY_X) @ rot_x(angles.shoulder_roll)

    p = p + R @ link_offsets["shoulder_yaw_origin_xyz"]
    R = R @ rot_z(angles.shoulder_yaw)

    elbow = p + R @ link_offsets["elbow_origin_xyz"]

    R = R @ rot_y(angles.elbow)
    wrist = elbow + R @ link_offsets["wrist_roll_origin_xyz"]

    return {"shoulder": shoulder, "elbow": elbow, "wrist": wrist}


def forward_kinematics_left_arm_full(angles: JointAngles) -> Dict[str, np.ndarray]:
    """左臂完整 FK: 返回所有关节位置和旋转矩阵。"""
    if angles.side != "left":
        angles = angles.with_side("left")
    link_offsets = LEFT_ARM_LINK_OFFSETS

    p_pitch = np.zeros(3, dtype=float)
    R = rot_x(LEFT_SHOULDER_PITCH_ORIGIN_RPY_X) @ rot_y(angles.shoulder_pitch)
    R_pitch = R.copy()

    p_roll = p_pitch + R @ link_offsets["shoulder_roll_origin_xyz"]
    R = R @ rot_x(LEFT_SHOULDER_ROLL_ORIGIN_RPY_X) @ rot_x(angles.shoulder_roll)
    R_roll = R.copy()

    p_yaw = p_roll + R @ link_offsets["shoulder_yaw_origin_xyz"]
    R = R @ rot_z(angles.shoulder_yaw)
    R_yaw = R.copy()

    elbow = p_yaw + R @ link_offsets["elbow_origin_xyz"]
    R_upper = R.copy()

    R = R @ rot_y(angles.elbow)

    R = R @ rot_x(angles.wrist_roll)
    R_forearm = R.copy()

    wrist = elbow + R @ link_offsets["wrist_roll_origin_xyz"]

    return {
        "shoulder_pitch": p_pitch, "shoulder_roll": p_roll,
        "shoulder_yaw": p_yaw, "elbow": elbow, "wrist": wrist,
        "shoulder": p_pitch,
        "R_shoulder_pitch": R_pitch, "R_shoulder_roll": R_roll,
        "R_shoulder_yaw": R_yaw, "R_upper": R_upper, "R_forearm": R_forearm,
    }


# 左臂骨骼方向向量
_LEFT_UPPER_BONE_DIR_LOCAL: np.ndarray
_LEFT_FOREARM_BONE_DIR_LOCAL: np.ndarray
_LEFT_UPPER_BONE_VEC: np.ndarray
_LEFT_FOREARM_BONE_VEC: np.ndarray


def _compute_left_bone_dirs():
    """计算左臂骨骼方向向量 (从 FK 零位反推)"""
    zero = JointAngles(0.0, 0.0, 0.0, 0.0, side="left")
    fk = forward_kinematics_left_arm_full(zero)
    R_upper = compose_shoulder_rotation(zero)

    upper_vec = fk["elbow"] - fk["shoulder_pitch"]
    upper_len = float(np.linalg.norm(upper_vec))
    upper_local = R_upper.T @ upper_vec / upper_len

    forearm_vec = fk["wrist"] - fk["elbow"]
    forearm_len = float(np.linalg.norm(forearm_vec))
    forearm_local = (R_upper.T @ forearm_vec) / forearm_len

    return upper_local, forearm_local, upper_len, forearm_len


_LEFT_UPPER_BONE_DIR_LOCAL, _LEFT_FOREARM_BONE_DIR_LOCAL, _L_ULEN, _L_FLEN = _compute_left_bone_dirs()
_LEFT_UPPER_BONE_VEC = _LEFT_UPPER_BONE_DIR_LOCAL * _L_ULEN
_LEFT_FOREARM_BONE_VEC = _LEFT_FOREARM_BONE_DIR_LOCAL * _L_FLEN


class LeftArmIMUSolver:
    """左臂 IMU 解算器 (使用左臂限位和偏移)"""

    def __init__(self) -> None:
        self.offsets = CalibrationOffsets(
            q_heading=np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
            upper_mount=np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
            forearm_mount=np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
        )

    def calibrate(
        self,
        q_raw_upper_cal: np.ndarray,
        q_raw_forearm_cal: np.ndarray,
        q_desired_upper: np.ndarray | None = None,
        q_desired_forearm: np.ndarray | None = None,
        upper_body_dir: np.ndarray | None = None,
        forearm_body_dir: np.ndarray | None = None,
    ) -> CalibrationOffsets:
        """侧平举单姿态校准 (与右臂方法相同, 使用航向+安装偏移)"""
        if q_desired_upper is None:
            q_desired_upper = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        if q_desired_forearm is None:
            q_desired_forearm = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

        q_raw_upper_cal = normalize_quaternion(q_raw_upper_cal)
        q_raw_forearm_cal = normalize_quaternion(q_raw_forearm_cal)
        q_desired_upper = normalize_quaternion(q_desired_upper)
        q_desired_forearm = normalize_quaternion(q_desired_forearm)

        if upper_body_dir is None:
            upper_body_dir = np.array([0.0, -1.0, 0.0])

        arm_dir_enu = quat_rotate(q_raw_upper_cal, upper_body_dir)
        arm_dir_urdf = quat_to_matrix(q_desired_upper) @ _LEFT_UPPER_BONE_DIR_LOCAL
        heading_angle = (
            np.arctan2(arm_dir_urdf[1], arm_dir_urdf[0])
            - np.arctan2(arm_dir_enu[1], arm_dir_enu[0])
        )
        q_heading = quat_from_axis_angle(np.array([0.0, 0.0, 1.0]), heading_angle)

        q_inv_heading = quat_inv(q_heading)
        upper_mount = quat_mul(
            quat_inv(q_raw_upper_cal),
            quat_mul(q_inv_heading, q_desired_upper),
        )
        forearm_mount = quat_mul(
            quat_inv(q_raw_forearm_cal),
            quat_mul(q_inv_heading, q_desired_forearm),
        )

        self.offsets = CalibrationOffsets(
            q_heading=normalize_quaternion(q_heading),
            upper_mount=normalize_quaternion(upper_mount),
            forearm_mount=normalize_quaternion(forearm_mount),
        )
        return self.offsets

    def apply_calibration_upper(self, q_raw: np.ndarray) -> np.ndarray:
        if self.offsets.use_similarity_transform:
            return normalize_quaternion(
                quat_mul(quat_mul(self.offsets.upper_transform, q_raw), self.offsets.upper_mount)
            )
        return normalize_quaternion(
            quat_mul(quat_mul(self.offsets.q_heading, q_raw), self.offsets.upper_mount)
        )

    def apply_calibration_forearm(self, q_raw: np.ndarray) -> np.ndarray:
        if self.offsets.use_similarity_transform:
            return normalize_quaternion(
                quat_mul(quat_mul(self.offsets.forearm_transform, q_raw), self.offsets.forearm_mount)
            )
        return normalize_quaternion(
            quat_mul(quat_mul(self.offsets.q_heading, q_raw), self.offsets.forearm_mount)
        )

    def solve(self, q_raw_upper: np.ndarray, q_raw_forearm: np.ndarray) -> JointAngles:
        solved, _diag = self.solve_with_diagnostics(q_raw_upper, q_raw_forearm)
        return solved

    def solve_with_diagnostics(
        self,
        q_raw_upper: np.ndarray,
        q_raw_forearm: np.ndarray,
    ) -> tuple[JointAngles, SolveDiagnostics]:
        q_raw_upper = normalize_quaternion(q_raw_upper)
        q_raw_forearm = normalize_quaternion(q_raw_forearm)

        q_upper = self.apply_calibration_upper(q_raw_upper)
        q_forearm = self.apply_calibration_forearm(q_raw_forearm)

        r_upper = quat_to_matrix(q_upper)
        r_adjusted = rot_x(-LEFT_SHOULDER_PITCH_ORIGIN_RPY_X) @ r_upper
        shoulder_pitch, shoulder_roll_with_mount, shoulder_yaw = decompose_yxz(r_adjusted)
        shoulder_roll = shoulder_roll_with_mount - LEFT_SHOULDER_ROLL_ORIGIN_RPY_X

        q_rel = normalize_quaternion(quat_mul(quat_inv(q_upper), q_forearm))
        elbow = extract_twist_angle_about_y(q_rel)
        wrist_roll = extract_wrist_roll(q_rel, elbow)

        angles = JointAngles(
            shoulder_pitch=shoulder_pitch,
            shoulder_roll=shoulder_roll,
            shoulder_yaw=shoulder_yaw,
            elbow=elbow,
            wrist_roll=wrist_roll,
            side="left",
        )
        clamped = clamp_joint_angles(angles)
        violations = compute_limit_violations(angles)
        limit_triggered = any(abs(v) > 1e-10 for v in violations.values())

        clamped_error = evaluate_mapping_consistency_deg(q_upper, q_forearm, clamped)
        if limit_triggered:
            best_fit = self._find_best_fit_within_limits(q_upper, q_forearm, clamped)
        else:
            best_fit = clamped
        best_fit_error = evaluate_mapping_consistency_deg(q_upper, q_forearm, best_fit)

        diagnostics = SolveDiagnostics(
            limit_triggered=limit_triggered,
            limit_violations=violations,
            unconstrained_angles=angles,
            clamped_error_deg=clamped_error,
            best_fit_error_deg=best_fit_error,
        )
        return best_fit, diagnostics

    def _find_best_fit_within_limits(
        self, q_upper_target, q_forearm_target, seed
    ) -> JointAngles:
        current = clamp_joint_angles(seed)
        current_cost = self._mapping_cost(q_upper_target, q_forearm_target, current)
        step_schedule_deg = [18.0, 10.0, 5.0, 2.0, 1.0]
        joint_keys = ["left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow"]

        for step_deg in step_schedule_deg:
            step = float(np.radians(step_deg))
            improved = True
            while improved:
                improved = False
                for key in joint_keys:
                    for direction in (-1.0, 1.0):
                        candidate = JointAngles(
                            shoulder_pitch=current.shoulder_pitch,
                            shoulder_roll=current.shoulder_roll,
                            shoulder_yaw=current.shoulder_yaw,
                            elbow=current.elbow,
                            wrist_roll=current.wrist_roll,
                            side="left",
                        )
                        if key == "left_shoulder_pitch":
                            candidate.shoulder_pitch += direction * step
                        elif key == "left_shoulder_roll":
                            candidate.shoulder_roll += direction * step
                        elif key == "left_shoulder_yaw":
                            candidate.shoulder_yaw += direction * step
                        else:
                            candidate.elbow += direction * step

                        candidate = clamp_joint_angles(candidate)
                        candidate_cost = self._mapping_cost(q_upper_target, q_forearm_target, candidate)
                        if candidate_cost + 1e-9 < current_cost:
                            current = candidate
                            current_cost = candidate_cost
                            improved = True
        return current

    @staticmethod
    def _mapping_cost(q_upper_target, q_forearm_target, candidate) -> float:
        err = evaluate_mapping_consistency_deg(q_upper_target, q_forearm_target, candidate)
        return err["upper_error_deg"] ** 2 + err["forearm_error_deg"] ** 2


# ========================================================
#  方向→关节角度 (配合 arm_calibration.py 使用)
# ========================================================

def _rodrigues_rotation(v: np.ndarray, w: np.ndarray) -> np.ndarray:
    """最小旋转矩阵将 v 映射到 w (Rodrigues)."""
    v = v / (np.linalg.norm(v) + 1e-12)
    w = w / (np.linalg.norm(w) + 1e-12)
    dot_vw = float(np.dot(v, w))
    if dot_vw > 1.0 - 1e-8:
        return np.eye(3)
    if dot_vw < -1.0 + 1e-8:
        perp = np.array([1, 0, 0]) if abs(v[0]) < 0.9 else np.array([0, 1, 0])
        axis = np.cross(v, perp)
        axis = axis / np.linalg.norm(axis)
        return 2.0 * np.outer(axis, axis) - np.eye(3)
    axis = np.cross(v, w)
    axis = axis / np.linalg.norm(axis)
    angle = np.arccos(np.clip(dot_vw, -1, 1))
    K = np.array([[0, -axis[2], axis[1]],
                   [axis[2], 0, -axis[0]],
                   [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)


def direction_to_joint_angles(
    upper_dir_chest: np.ndarray,
    forearm_dir_chest: np.ndarray,
    wrist_roll: float = 0.0,
    side: str = "right",
    seed: JointAngles | None = None,
) -> JointAngles:
    """从胸部坐标系方向向量计算关节角度 (新分解逻辑).

    新逻辑:
      upper_dir (2 DOF) → SP, SR (肩部pitch+roll, 无yaw)
      forearm_dir (2 DOF) → SY, EL (肩部yaw + 肘)
      forearm_twist (1 DOF) → WR (腕)

    Args:
        upper_dir_chest: 上臂方向 (胸部坐标系)
        forearm_dir_chest: 前臂方向 (胸部坐标系)
        wrist_roll: 前臂扭转角 (弧度, 来自 twist 校准)
        side: "right" 或 "left"
        seed: 上一帧角度 (用于连续性, 可选)
    """
    upper_dir = upper_dir_chest / (np.linalg.norm(upper_dir_chest) + 1e-12)
    forearm_dir = forearm_dir_chest / (np.linalg.norm(forearm_dir_chest) + 1e-12)

    # Chest → URDF: 翻转 Y (right → left)
    upper_dir = np.array([upper_dir[0], -upper_dir[1], upper_dir[2]])
    forearm_dir = np.array([forearm_dir[0], -forearm_dir[1], forearm_dir[2]])

    # 根据 side 选择正确的 origin RPY
    if side == "left":
        pitch_origin = LEFT_SHOULDER_PITCH_ORIGIN_RPY_X
        roll_origin = LEFT_SHOULDER_ROLL_ORIGIN_RPY_X
    else:
        pitch_origin = SHOULDER_PITCH_ORIGIN_RPY_X
        roll_origin = SHOULDER_ROLL_ORIGIN_RPY_X

    # ========== Step 1: SP, SR from upper_dir (解析法, 2 DOF) ==========
    # 实际 FK: rot_x(ORIGIN_SP) @ rot_y(sp) @ rot_x(ORIGIN_SR + sr) @ bone = upper_dir_urdf
    # 变形:    rot_y(sp) @ rot_x(sr_total) @ bone = rot_x(-ORIGIN_SP) @ upper_dir_urdf
    # 其中 sr_total = ORIGIN_SR + sr
    if side == "left":
        bx, by, bz = _LEFT_UPPER_BONE_DIR_LOCAL
    else:
        bx, by, bz = _UPPER_BONE_DIR_LOCAL

    # 先用 origin 补偿 upper_dir
    target_upper = rot_x(-pitch_origin) @ upper_dir
    tx, ty, tz = float(target_upper[0]), float(target_upper[1]), float(target_upper[2])

    # 从 Y 分量求 sr_total:
    #   ty = by*cos(sr_total) - bz*sin(sr_total)
    #   = R*cos(sr_total - δ), 其中 R=sqrt(by²+bz²), δ=atan2(-bz, by)
    #   sr_total = δ ± acos(ty/R) (两个分支)
    r_yz = float(np.sqrt(by * by + bz * bz))
    if r_yz < 1e-6:
        sr_total = 0.0
    else:
        delta = float(np.arctan2(-bz, by))  # atan2(B, A) for A*cos + B*sin
        acos_arg = float(np.clip(ty / r_yz, -1.0, 1.0))
        acos_val = float(np.arccos(acos_arg))

        # 两个候选解
        sr_total_1 = delta + acos_val
        sr_total_2 = delta - acos_val

        # 选择更合理的解（在关节限位内，或更接近零位）
        sr_1 = sr_total_1 - roll_origin
        sr_2 = sr_total_2 - roll_origin

        limits = get_joint_limits(side)
        sr_min, sr_max = limits[f"{side}_shoulder_roll"]

        # 优先选择在限位内的解
        in_limits_1 = sr_min <= sr_1 <= sr_max
        in_limits_2 = sr_min <= sr_2 <= sr_max

        if in_limits_1 and in_limits_2:
            # 都在限位内，选接近零位的
            sr_total = sr_total_1 if abs(sr_1) < abs(sr_2) else sr_total_2
        elif in_limits_1:
            sr_total = sr_total_1
        elif in_limits_2:
            sr_total = sr_total_2
        else:
            # 都超出限位，选偏离较小的
            sr_total = sr_total_1 if abs(sr_1 - (sr_min + sr_max) / 2) < abs(sr_2 - (sr_min + sr_max) / 2) else sr_total_2

    # 还原关节角 sr
    sr = sr_total - roll_origin

    # 检查 SR 是否超出限制，若超出则 clamp 并重新计算 sp
    sr_clamped = clamp(sr, sr_min, sr_max)

    # 如果 SR 被 clamp，重新计算 sr_total 和 sp
    if abs(sr_clamped - sr) > 1e-6:
        sr_total = sr_clamped + roll_origin

    # 从 X,Z 分量求 SP:
    #   rot_x(sr_total) @ bone 的 XZ 分量
    vx = bx
    vz = by * np.sin(sr_total) + bz * np.cos(sr_total)
    sp = float(np.arctan2(tx, tz) - np.arctan2(vx, vz))

    # Wrap SP 到 [-π, π] (atan2 减法可能超出范围)
    sp = float((sp + np.pi) % (2 * np.pi) - np.pi)

    sr = sr_clamped

    # ========== Step 2: SY, EL from forearm_dir ==========
    # 构造 R_base = rot(SP, SR, SY=0) 的肩部旋转矩阵
    R_base = rot_x(pitch_origin) @ rot_y(sp) @ \
             rot_x(roll_origin + sr)

    # target = forearm 在 upper 局部坐标系中的方向 (假设 SY=0)
    target = R_base.T @ forearm_dir  # 这是一个单位向量

    # 骨骼方向分量
    if side == "left":
        bx, by, bz = _LEFT_FOREARM_BONE_DIR_LOCAL
    else:
        bx, by, bz = _FOREARM_BONE_DIR_LOCAL

    # 从 target_z 求 EL:
    #   target_z = -bx*sin(EL) + bz*cos(EL)
    # 这可以写成: target_z = r * cos(EL - phase), 其中
    #   r = sqrt(bx² + bz²), phase = atan2(bx, bz)
    r = float(np.sqrt(bx * bx + bz * bz))
    if r < 1e-6:
        # 骨骼几乎沿 Y 轴, 从 z 分量无法求解 EL
        # 用上下臂夹角近似
        el = float(np.arccos(np.clip(np.dot(upper_dir, forearm_dir), -1.0, 1.0)))
        el -= 1.39626  # ~零位夹角 (79.5° - 一些修正)
        # SY 从 XY 平面投影求解
        sy = float(np.arctan2(target[1], target[0]) - np.arctan2(by, bx))
    else:
        phase = float(np.arctan2(bx, bz))  # 注意: atan2(bx, bz) 不是 atan2(y, x)
        tz_clamped = float(np.clip(target[2] / r, -1.0, 1.0))
        el_base = float(np.arccos(tz_clamped))

        # EL - delta = ±arccos(target_z/r), 其中 delta = atan2(-bx, bz) = -phase
        # 所以 EL = -phase ± arccos(target_z/r)
        el_c1 = el_base - phase
        el_c2 = -el_base - phase

        # 选择更合理的解 (肘关节范围 [-60°, 120°])
        # 用 seed 或默认约束选择
        if seed is not None:
            el = el_c1 if abs(el_c1 - seed.elbow) < abs(el_c2 - seed.elbow) else el_c2
        else:
            # 选择在范围内的解
            if -1.05 <= el_c1 <= 2.10:
                el = el_c1
            elif -1.05 <= el_c2 <= 2.10:
                el = el_c2
            else:
                el = el_c1  # 默认

        # 计算 rot_y(EL) @ bone 的 XY 分量
        vx = bx * np.cos(el) + bz * np.sin(el)
        vy = by

        # 从 2D 旋转求 SY:
        #   (target_x, target_y) = rot_z(SY) @ (vx, vy)
        #   SY = atan2(target_y, target_x) - atan2(vy, vx)
        sy = float(np.arctan2(target[1], target[0]) - np.arctan2(vy, vx))

    # ========== Step 3: WR = forearm_twist ==========
    wr = wrist_roll

    # 连续性处理: 使输出接近 seed，避免 ±2π 跳变
    if seed is not None:
        d_sp = sp - seed.shoulder_pitch
        d_sr = sr - seed.shoulder_roll
        d_sy = sy - seed.shoulder_yaw
        # 将差值 wrap 到 [-π, π]，检测是否需要 ±2π 补偿
        sp -= round(d_sp / (2 * np.pi)) * 2 * np.pi
        sy -= round(d_sy / (2 * np.pi)) * 2 * np.pi

    angles = JointAngles(sp, sr, sy, el, wr, side=side)
    return clamp_joint_angles(angles)


def compute_wrist_roll_from_palm(
    palm_dir_chest: np.ndarray,
    sp: float,
    sr: float,
    sy: float,
    el: float,
    side: str = "right",
    prev_wr: float | None = None,
) -> float:
    """从掌心朝向计算 wrist_roll (掌心匹配法).

    将人体掌心朝向转换到机器人腕关节局部坐标系,
    从中提取绕 X 轴的旋转角度 WR, 使机器人掌心与人体掌心一致。

    Args:
        palm_dir_chest: 掌心方向 (胸部坐标系)
        sp, sr, sy, el: 已计算的关节角度
        side: "right" 或 "left"
        prev_wr: 上一次 WR 值 (用于连续性, 可选)

    Returns:
        wr (rad)
    """
    # 根据 side 选择正确的 origin RPY
    if side == "left":
        pitch_origin = LEFT_SHOULDER_PITCH_ORIGIN_RPY_X
        roll_origin = LEFT_SHOULDER_ROLL_ORIGIN_RPY_X
    else:
        pitch_origin = SHOULDER_PITCH_ORIGIN_RPY_X
        roll_origin = SHOULDER_ROLL_ORIGIN_RPY_X

    # Chest → URDF: 翻转 Y (right→left)
    palm_urdf = np.array([palm_dir_chest[0], -palm_dir_chest[1], palm_dir_chest[2]])
    palm_urdf = palm_urdf / (np.linalg.norm(palm_urdf) + 1e-12)

    # 腕关节之前的旋转链
    R_before_wrist = (
        rot_x(pitch_origin)
        @ rot_y(sp)
        @ rot_x(roll_origin + sr)
        @ rot_z(sy)
        @ rot_y(el)
    )

    # 掌心转换到腕关节局部帧
    palm_local = R_before_wrist.T @ palm_urdf

    # rot_x(WR) 绕 X 轴旋转, 从零位 [0,1,0] 转到当前 palm_local
    # 校准定义: 侧平举掌心朝下 = WR=0, 此时 palm_local ≈ [0, 1, 0]
    # rot_x(WR) @ [0, 1, 0] = [0, cos(WR), sin(WR)]
    # WR = atan2(palm_local[2], palm_local[1])
    wr = float(np.arctan2(palm_local[2], palm_local[1]))

    # 连续性处理
    if prev_wr is not None:
        d_wr = wr - prev_wr
        wr -= round(d_wr / (2 * np.pi)) * 2 * np.pi

    return wr
