"""统一校准与解算模块 — 从 upper_arm_3pose.py 提取.

提供:
  - 四元数工具函数
  - ArmDirectionCalibrator (3姿态迭代Procrustes + Roll精修)
  - fit_hinge_axis (肘屈伸功能轴拟合 + 质量判定)
  - TwistCalibration (小臂扭转/掌心朝向)
  - IncrementalTracker (增量q_rel跟踪, 消除IMU漂移)
  - 校准姿态常量 CALIB_POSES
"""

import numpy as np
from dataclasses import dataclass


# ========================================================
#  四元数 / 向量工具
# ========================================================

def quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_rotate(q, v):
    qv = np.array([0.0, v[0], v[1], v[2]])
    return quat_mul(quat_mul(q, qv), quat_conj(q))[1:]


def normalize_quat(q, eps=1e-10):
    n = float(np.linalg.norm(q))
    return q / n if n > eps else np.array([1.0, 0.0, 0.0, 0.0])


def normalize_vec(v, eps=1e-10):
    n = float(np.linalg.norm(v))
    return v / n if n > eps else np.array([1.0, 0.0, 0.0])


def average_quaternions(quats):
    if not quats:
        return np.array([1.0, 0.0, 0.0, 0.0])
    q0 = quats[0]
    aligned = [q0.copy()]
    for q in quats[1:]:
        aligned.append(-q if np.dot(q, q0) < 0 else q.copy())
    q_avg = np.mean(aligned, axis=0)
    norm = np.linalg.norm(q_avg)
    return q_avg / norm if norm > 1e-10 else np.array([1.0, 0.0, 0.0, 0.0])


def quaternion_angle_deg(q1, q2):
    """两个姿态之间的最短夹角（度），忽略四元数正负号。"""
    a = normalize_quat(np.asarray(q1, dtype=float))
    b = normalize_quat(np.asarray(q2, dtype=float))
    dot = float(np.clip(abs(np.dot(a, b)), 0.0, 1.0))
    return float(np.degrees(2.0 * np.arccos(dot)))


def quaternion_dispersion_deg(quats):
    """四元数样本相对内禀均值的 RMS 角离散度（度）。"""
    if not quats:
        return float("inf")
    q_avg = average_quaternions(quats)
    errors = [quaternion_angle_deg(q_avg, q) for q in quats]
    return float(np.sqrt(np.mean(np.square(errors))))


def average_relative_quaternions(reference_quats, link_quats):
    """逐帧计算相对姿态后再平均，避免分别平均造成同步误差。"""
    n = min(len(reference_quats), len(link_quats))
    if n == 0:
        return np.array([1.0, 0.0, 0.0, 0.0])
    relatives = [normalize_quat(quat_mul(quat_conj(reference_quats[i]),
                                         link_quats[i]))
                 for i in range(n)]
    return average_quaternions(relatives)


def _quat_to_rotvec(q):
    """单位四元数转最短旋转向量（弧度）。"""
    q = normalize_quat(q)
    if q[0] < 0.0:
        q = -q
    sin_half = float(np.linalg.norm(q[1:4]))
    if sin_half < 1e-10:
        return np.zeros(3)
    angle = 2.0 * np.arctan2(sin_half, float(np.clip(q[0], -1.0, 1.0)))
    return q[1:4] * (angle / sin_half)


@dataclass
class HingeAxisCalibration:
    """肘屈伸功能标定结果，轴分别表示在大臂/小臂 IMU 坐标系中。"""

    upper_axis: np.ndarray
    forearm_axis: np.ndarray
    motion_range_deg: float
    axis_dispersion_deg: float
    upper_motion_deg: float
    sample_count: int
    valid: bool
    reason: str = ""

    def save_dict(self):
        return {
            "upper_axis": self.upper_axis.tolist(),
            "forearm_axis": self.forearm_axis.tolist(),
            "motion_range_deg": self.motion_range_deg,
            "axis_dispersion_deg": self.axis_dispersion_deg,
            "upper_motion_deg": self.upper_motion_deg,
            "sample_count": self.sample_count,
            "valid": self.valid,
            "reason": self.reason,
        }


def _principal_axis(rotvecs, weights):
    matrix = np.zeros((3, 3))
    for rv, weight in zip(rotvecs, weights):
        axis = normalize_vec(rv)
        matrix += float(weight) * np.outer(axis, axis)
    values, vectors = np.linalg.eigh(matrix)
    return normalize_vec(vectors[:, int(np.argmax(values))])


def fit_hinge_axis(q_chest_samples, q_upper_samples, q_forearm_samples,
                   min_increment_deg=0.6, max_increment_deg=30.0,
                   min_motion_range_deg=30.0,
                   max_axis_dispersion_deg=18.0,
                   max_upper_motion_deg=22.0):
    """从肘屈伸序列鲁棒拟合铰链轴，并给出自动验收指标。

    无线 IMU 的相邻帧转角很小时，量化噪声会让瞬时旋转轴严重发散。
    这里使用自适应跨帧增量、按转角加权以及离群轴剔除，避免正常的
    缓慢屈伸在换向点被误判为“轴离散”。
    """
    n = min(len(q_chest_samples), len(q_upper_samples),
            len(q_forearm_samples))
    if n < 12:
        return HingeAxisCalibration(
            np.array([1., 0., 0.]), np.array([1., 0., 0.]),
            0.0, float("inf"), float("inf"), n, False, "有效样本不足")

    q_rel = [normalize_quat(quat_mul(quat_conj(q_upper_samples[i]),
                                     q_forearm_samples[i]))
             for i in range(n)]
    upper_rotvecs = []
    forearm_rotvecs = []
    weights = []
    # 约 1/80 采集长度的窗口通常对应 50~100 ms，既能压低噪声，
    # 又不会把一次正常屈伸平均掉。短序列仍自动退化为相邻帧。
    increment_window = max(1, min(8, n // 80))
    for index in range(increment_window, n):
        prev = q_rel[index - increment_window]
        curr = q_rel[index]
        # 左增量的轴位于大臂 IMU 系；右增量的轴位于小臂 IMU 系。
        rv_upper = _quat_to_rotvec(quat_mul(curr, quat_conj(prev)))
        rv_forearm = _quat_to_rotvec(quat_mul(quat_conj(prev), curr))
        angle_deg = float(np.degrees(np.linalg.norm(rv_upper)))
        if min_increment_deg <= angle_deg <= max_increment_deg:
            upper_rotvecs.append(rv_upper)
            forearm_rotvecs.append(rv_forearm)
            # 较大的有效转角比换向附近的小转角有更高的轴信噪比。
            weights.append(angle_deg * angle_deg)

    if len(upper_rotvecs) < 10:
        return HingeAxisCalibration(
            np.array([1., 0., 0.]), np.array([1., 0., 0.]),
            0.0, float("inf"), float("inf"), len(upper_rotvecs), False,
            "肘关节运动太小或太快")

    weights = np.asarray(weights, dtype=float)
    upper_axis = _principal_axis(upper_rotvecs, weights)

    # 第一次估计后剔除明显非铰链运动；阈值用 MAD 自适应，且保留
    # 至少 12° 容差，以适应消费级 IMU 的动态噪声。
    initial_deviations = np.asarray([
        np.degrees(np.arccos(float(np.clip(
            abs(np.dot(normalize_vec(rv), upper_axis)), 0.0, 1.0))))
        for rv in upper_rotvecs
    ])
    median_deviation = float(np.median(initial_deviations))
    mad = float(np.median(np.abs(initial_deviations - median_deviation)))
    outlier_limit = min(45.0, max(12.0, median_deviation + 3.5 * max(mad, 1.0)))
    keep = initial_deviations <= outlier_limit
    if np.count_nonzero(keep) >= 10:
        upper_rotvecs = [rv for rv, accepted in zip(upper_rotvecs, keep) if accepted]
        forearm_rotvecs = [rv for rv, accepted in zip(forearm_rotvecs, keep) if accepted]
        weights = weights[keep]

    upper_axis = _principal_axis(upper_rotvecs, weights)
    forearm_axis = _principal_axis(forearm_rotvecs, weights)

    # 相对首帧提取绕拟合轴的 twist 并展开，直接得到真实关节活动范围；
    # 不再对重叠的跨帧增量累计，以免把往返次数算进动作幅度。
    q_rel_ref = q_rel[0]
    hinge_angles = []
    for q in q_rel:
        delta = normalize_quat(quat_mul(q, quat_conj(q_rel_ref)))
        hinge_angles.append(swing_twist_angle(delta, upper_axis))
    hinge_angles = np.unwrap(np.asarray(hinge_angles, dtype=float))
    motion_range_deg = float(np.degrees(np.ptp(hinge_angles)))

    deviations = np.asarray([
        np.degrees(np.arccos(float(np.clip(
            abs(np.dot(normalize_vec(rv), upper_axis)), 0.0, 1.0))))
        for rv in upper_rotvecs
    ])
    axis_dispersion_deg = float(np.sqrt(
        np.average(np.square(deviations), weights=weights)))

    # 检查大臂是否相对胸部保持基本不动，防止肩部代偿污染肘轴。
    q_cu = [normalize_quat(quat_mul(quat_conj(q_chest_samples[i]),
                                    q_upper_samples[i]))
            for i in range(n)]
    q_cu_ref = average_quaternions(q_cu[:min(10, n)])
    upper_deviations = [quaternion_angle_deg(q_cu_ref, q) for q in q_cu]
    # 单个无线异常包不应让整段动作失败；持续的肩部代偿仍会体现在 P95。
    upper_motion_deg = float(np.percentile(upper_deviations, 95.0))

    reasons = []
    if motion_range_deg < min_motion_range_deg:
        reasons.append(
            f"有效屈伸范围仅{motion_range_deg:.1f}°，请从接近伸直到约90°")
    if axis_dispersion_deg > max_axis_dispersion_deg:
        reasons.append(
            f"肘轴一致性不足{axis_dispersion_deg:.1f}°，请勿旋转小臂")
    if upper_motion_deg > max_upper_motion_deg:
        reasons.append(
            f"大臂相对胸部移动{upper_motion_deg:.1f}°，保持自然下垂即可")
    return HingeAxisCalibration(
        upper_axis, forearm_axis, motion_range_deg, axis_dispersion_deg,
        upper_motion_deg, len(upper_rotvecs), not reasons, "；".join(reasons))


def swing_twist_angle(q, axis):
    q = normalize_quat(q)
    if q[0] < 0:
        q = -q
    d = np.dot(q[1:4], axis)
    return 2.0 * np.arctan2(d, q[0])


def extract_roll_axis(q_rel_samples, initial_arm_body_dir,
                      min_angle_deg=3.0, max_angle_deg=60.0):
    if len(q_rel_samples) < 10:
        return None
    n_ref = min(5, len(q_rel_samples))
    q_ref = average_quaternions(q_rel_samples[:n_ref])
    axes = []
    weights = []
    for q in q_rel_samples:
        q_delta = normalize_quat(quat_mul(quat_conj(q_ref), q))
        if q_delta[0] < 0:
            q_delta = -q_delta
        vec = q_delta[1:4]
        norm = np.linalg.norm(vec)
        if norm < 1e-10:
            continue
        angle_deg = np.degrees(2.0 * np.arctan2(norm, q_delta[0]))
        if angle_deg < min_angle_deg or angle_deg > max_angle_deg:
            continue
        axis = vec / norm
        if np.dot(axis, initial_arm_body_dir) < 0:
            axis = -axis
        axes.append(axis)
        weights.append(angle_deg)
    if len(axes) < 3:
        return None
    avg = np.average(np.array(axes), axis=0, weights=np.array(weights))
    return normalize_vec(avg)


# ========================================================
#  ArmDirectionCalibrator
# ========================================================

class ArmDirectionCalibrator:
    """方向校准: Roll轴提取 + Procrustes 求解 IMU→胸体 对齐."""

    def __init__(self, label=""):
        self.label = label
        self.R_align = np.eye(3)
        self.arm_body_dir = np.array([-1.0, 0.0, 0.0])
        self._poses = []
        self.calibrated = False

    def collect_pose(self, q_chest, q_link, known_dir):
        q_rel = quat_mul(quat_conj(q_chest), q_link)
        self._poses.append((normalize_quat(q_rel), np.array(known_dir, dtype=float)))

    def calibrate(self):
        if len(self._poses) != 3:
            raise ValueError(f"[{self.label}] 需要3个姿态, 当前{len(self._poses)}个")
        arm_dir = np.array([-1.0, 0.0, 0.0])
        for _ in range(50):
            u_vecs = [quat_rotate(q_rel, arm_dir) for q_rel, _ in self._poses]
            v_vecs = [kd for _, kd in self._poses]
            U = np.column_stack(u_vecs)
            V = np.column_stack(v_vecs)
            M = V @ U.T
            W, _, Vt = np.linalg.svd(M)
            det_sign = np.linalg.det(W @ Vt)
            R = W @ np.diag([1.0, 1.0, det_sign]) @ Vt
            estimates = []
            for q_rel, known_dir in self._poses:
                est = quat_rotate(quat_conj(q_rel), R.T @ known_dir)
                estimates.append(est)
            arm_dir_new = normalize_vec(np.mean(estimates, axis=0))
            if np.allclose(arm_dir, arm_dir_new, atol=1e-6):
                arm_dir = arm_dir_new
                break
            arm_dir = arm_dir_new
        self.R_align = R
        self.arm_body_dir = arm_dir
        self.calibrated = True
        self._verify()
        return R, arm_dir

    def _verify(self):
        max_err = 0.0
        for i, (q_rel, known_dir) in enumerate(self._poses):
            pred = self.R_align @ quat_rotate(q_rel, self.arm_body_dir)
            cos_a = np.clip(np.dot(pred, known_dir), -1.0, 1.0)
            err = np.degrees(np.arccos(cos_a))
            max_err = max(max_err, err)
            ok = "OK" if err < 2.0 else "BAD"
            print(f"  [{self.label}] 姿态 {i}: 误差={err:.4f}deg {ok}")
        print(f"  [{self.label}] arm_body_dir={self.arm_body_dir.round(6)}, "
              f"最大误差={max_err:.4f}deg")
        return max_err

    def pose_errors_deg(self):
        """返回各训练姿态的方向误差，用于保存前质量报告。"""
        errors = []
        for q_rel, known_dir in self._poses:
            pred = normalize_vec(self.reconstruct(q_rel))
            target = normalize_vec(known_dir)
            errors.append(float(np.degrees(np.arccos(
                np.clip(np.dot(pred, target), -1.0, 1.0)))))
        return errors

    def refine_with_hinge_axis(self, hinge_axis, max_correction_deg=18.0):
        """利用骨段纵轴应垂直于肘轴的约束，修正静态姿势带来的偏差。"""
        axis = normalize_vec(np.asarray(hinge_axis, dtype=float))
        old_dir = normalize_vec(self.arm_body_dir)
        projected = old_dir - np.dot(old_dir, axis) * axis
        if np.linalg.norm(projected) < 1e-6:
            return False, 90.0
        projected = normalize_vec(projected)
        if np.dot(projected, old_dir) < 0.0:
            projected = -projected
        correction = float(np.degrees(np.arccos(
            np.clip(np.dot(old_dir, projected), -1.0, 1.0))))
        if correction > max_correction_deg:
            print(f"  [{self.label}] 肘轴修正需求 {correction:.2f}° 过大，拒绝修正")
            return False, correction
        self.arm_body_dir = projected
        self._recompute_r_align()
        print(f"  [{self.label}] 肘轴正交修正 {correction:.2f}°")
        return True, correction

    def refine_with_roll(self, q_rel_samples):
        if not self.calibrated or len(q_rel_samples) < 10:
            print(f"  [{self.label}] Roll数据不足, 跳过精校")
            return False
        refined = extract_roll_axis(q_rel_samples, self.arm_body_dir)
        if refined is None:
            print(f"  [{self.label}] 无法提取Roll轴, 跳过精校")
            return False
        old_dir = self.arm_body_dir.copy()
        angle_change = np.degrees(
            np.arccos(np.clip(np.dot(old_dir, refined), -1, 1)))
        self.arm_body_dir = refined
        self._recompute_r_align()
        print(f"  [{self.label}] Roll轴精校: 方向修正 {angle_change:.2f}deg")
        print(f"    旧: {old_dir.round(6)}")
        print(f"    新: {refined.round(6)}")
        self._verify()
        return True

    def _recompute_r_align(self):
        u_vecs = [quat_rotate(q_rel, self.arm_body_dir)
                  for q_rel, _ in self._poses]
        v_vecs = [kd for _, kd in self._poses]
        U = np.column_stack(u_vecs)
        V = np.column_stack(v_vecs)
        M = V @ U.T
        W, _, Vt = np.linalg.svd(M)
        det_sign = np.linalg.det(W @ Vt)
        self.R_align = W @ np.diag([1.0, 1.0, det_sign]) @ Vt

    def calibrate_with_roll(self, q_rel_samples, roll_weight=1.0):
        if len(self._poses) < 3:
            raise ValueError(
                f"[{self.label}] 需要>=3个姿态, 当前{len(self._poses)}个")
        # Step 1: 纯3姿态迭代Procrustes
        arm_dir = np.array([-1.0, 0.0, 0.0])
        for _ in range(50):
            u_vecs = [quat_rotate(q_rel, arm_dir) for q_rel, _ in self._poses]
            v_vecs = [kd for _, kd in self._poses]
            M = np.column_stack(v_vecs) @ np.column_stack(u_vecs).T
            W, _, Vt = np.linalg.svd(M)
            det_sign = np.linalg.det(W @ Vt)
            R = W @ np.diag([1.0, 1.0, det_sign]) @ Vt
            estimates = []
            for q_rel, known_dir in self._poses:
                est = quat_rotate(quat_conj(q_rel), R.T @ known_dir)
                estimates.append(est)
            arm_dir_new = normalize_vec(np.mean(estimates, axis=0))
            if np.allclose(arm_dir, arm_dir_new, atol=1e-6):
                arm_dir = arm_dir_new
                break
            arm_dir = arm_dir_new
        procrustes_dir = arm_dir.copy()
        print(f"  [{self.label}] 3姿态Procrustes: arm_body_dir="
              f"{procrustes_dir.round(6)}")
        # Step 2: Roll精修
        roll_dir = extract_roll_axis(q_rel_samples, procrustes_dir)
        if roll_dir is None:
            print(f"  [{self.label}] Roll数据不足, 使用纯3姿态结果")
            self.R_align = R
            self.arm_body_dir = procrustes_dir
            self.calibrated = True
            self._verify()
            return R, procrustes_dir
        roll_angle = np.degrees(
            np.arccos(np.clip(np.dot(roll_dir, procrustes_dir), -1, 1)))
        print(f"  [{self.label}] Roll精修方向: {roll_dir.round(6)} "
              f"(与Procrustes差{roll_angle:.2f}deg)")
        # Step 3: Roll正则化迭代Procrustes
        arm_dir = procrustes_dir
        n_poses = len(self._poses)
        n_roll_copies = max(1, round(roll_weight * n_poses))
        for _ in range(50):
            u_vecs = [quat_rotate(q_rel, arm_dir) for q_rel, _ in self._poses]
            v_vecs = [kd for _, kd in self._poses]
            M = np.column_stack(v_vecs) @ np.column_stack(u_vecs).T
            W, _, Vt = np.linalg.svd(M)
            det_sign = np.linalg.det(W @ Vt)
            R = W @ np.diag([1.0, 1.0, det_sign]) @ Vt
            estimates = []
            for q_rel, known_dir in self._poses:
                est = quat_rotate(quat_conj(q_rel), R.T @ known_dir)
                estimates.append(est)
            for _ in range(n_roll_copies):
                estimates.append(roll_dir)
            arm_dir_new = normalize_vec(np.mean(estimates, axis=0))
            if np.allclose(arm_dir, arm_dir_new, atol=1e-6):
                arm_dir = arm_dir_new
                break
            arm_dir = arm_dir_new
        self.R_align = R
        self.arm_body_dir = arm_dir
        self.calibrated = True
        final_roll_shift = np.degrees(
            np.arccos(np.clip(np.dot(arm_dir, procrustes_dir), -1, 1)))
        print(f"  [{self.label}] 最终: arm_body_dir={arm_dir.round(6)} "
              f"(Roll修正{final_roll_shift:.2f}deg)")
        self._verify()
        return R, arm_dir

    def reconstruct(self, q_rel):
        """从 q_rel (conj(q_chest)*q_link) 重建胸部坐标系方向向量."""
        return self.R_align @ quat_rotate(q_rel, self.arm_body_dir)

    def save_dict(self):
        return {"R_align": self.R_align.tolist(),
                "arm_body_dir": self.arm_body_dir.tolist()}

    def load_dict(self, d):
        self.R_align = np.array(d["R_align"])
        self.arm_body_dir = np.array(d["arm_body_dir"])
        self.calibrated = True


# ========================================================
#  TwistCalibration
# ========================================================

def palm_axis_from_pose(q_rel_forearm, target_chest, forearm_calibrator):
    """把胸部坐标系的目标掌心方向反算到小臂 IMU 坐标系。"""
    R_align = np.asarray(forearm_calibrator.R_align, dtype=float)
    q_rel = normalize_quat(np.asarray(q_rel_forearm, dtype=float))
    target_raw = R_align.T @ np.asarray(target_chest, dtype=float)
    return normalize_vec(quat_rotate(quat_conj(q_rel), target_raw))


def palm_pose_consistency_deg(q_rel_palm_up, q_rel_palm_down,
                              forearm_calibrator):
    """朝上/朝下两个端点各自反算出的掌心轴之间的夹角。

    最终判定与采集过程中的实时预览必须用同一份计算，否则预览显示
    合格而保存时仍被拒绝。
    """
    palm_from_up = palm_axis_from_pose(
        q_rel_palm_up, [0.0, 0.0, 1.0], forearm_calibrator)
    palm_from_down = palm_axis_from_pose(
        q_rel_palm_down, [0.0, 0.0, -1.0], forearm_calibrator)
    return float(np.degrees(np.arccos(np.clip(
        np.dot(palm_from_up, palm_from_down), -1.0, 1.0))))


@dataclass
class TwistCalibration:
    q_rel_twist_ref: np.ndarray
    twist_axis: np.ndarray
    palm_in_forearm_imu: np.ndarray
    palm_pose_consistency_deg: float = 0.0
    palm_up_error_deg: float = 0.0
    palm_down_error_deg: float = 0.0

    @staticmethod
    def from_palm_poses(q_rel_palm_up, q_rel_palm_down,
                        forearm_calibrator,
                        max_consistency_deg=30.0):
        """用掌心朝上/朝下两个静态端点确定掌心轴与腕部零位。

        两个端点都使用“小臂向前”的姿势。分别将胸部坐标系中的
        +Z（掌心朝上）和 -Z（掌心朝下）反算到小臂 IMU 坐标系，
        再取球面平均。两次反算应得到同一个掌心轴。
        """
        q_up = normalize_quat(np.asarray(q_rel_palm_up, dtype=float))
        q_down = normalize_quat(np.asarray(q_rel_palm_down, dtype=float))

        palm_from_up = palm_axis_from_pose(
            q_up, [0.0, 0.0, 1.0], forearm_calibrator)
        palm_from_down = palm_axis_from_pose(
            q_down, [0.0, 0.0, -1.0], forearm_calibrator)
        consistency_deg = palm_pose_consistency_deg(
            q_up, q_down, forearm_calibrator)
        if consistency_deg > max_consistency_deg:
            raise ValueError(
                "掌心朝上/朝下结果不一致（%.1f° > %.1f°）"
                % (consistency_deg, max_consistency_deg))

        palm_sum = palm_from_up + palm_from_down
        if np.linalg.norm(palm_sum) < 1e-8:
            raise ValueError("掌心朝上/朝下结果方向相反，请检查佩戴和姿势")
        palm_in_imu = normalize_vec(palm_sum)

        # q_delta = conj(q_ref) * q_current 位于参考小臂坐标系中，
        # 因此扭转轴必须使用 IMU 局部的骨段纵轴。
        twist_axis = normalize_vec(forearm_calibrator.arm_body_dir)
        calibration = TwistCalibration(
            q_rel_twist_ref=q_down,
            twist_axis=twist_axis,
            palm_in_forearm_imu=palm_in_imu,
            palm_pose_consistency_deg=consistency_deg,
        )

        def endpoint_error(q_rel, target):
            predicted = normalize_vec(
                calibration.compute_palm_direction(
                    q_rel, forearm_calibrator))
            return float(np.degrees(np.arccos(np.clip(
                np.dot(predicted, normalize_vec(np.asarray(target))),
                -1.0, 1.0))))

        calibration.palm_up_error_deg = endpoint_error(
            q_up, [0.0, 0.0, 1.0])
        calibration.palm_down_error_deg = endpoint_error(
            q_down, [0.0, 0.0, -1.0])
        return calibration

    def compute_forearm_twist(self, q_rel_forearm_inc):
        q_delta = quat_mul(quat_conj(self.q_rel_twist_ref),
                           q_rel_forearm_inc)
        return swing_twist_angle(q_delta, self.twist_axis)

    def compute_palm_direction(self, q_rel_forearm_inc, forearm_calibrator):
        palm_chest = (forearm_calibrator.R_align
                      @ quat_rotate(q_rel_forearm_inc,
                                    self.palm_in_forearm_imu))
        return palm_chest

    def save_dict(self):
        return {
            "q_rel_twist_ref": self.q_rel_twist_ref.tolist(),
            "twist_axis": self.twist_axis.tolist(),
            "palm_in_forearm_imu": self.palm_in_forearm_imu.tolist(),
            "palm_pose_consistency_deg": self.palm_pose_consistency_deg,
            "palm_up_error_deg": self.palm_up_error_deg,
            "palm_down_error_deg": self.palm_down_error_deg,
        }

    @staticmethod
    def load_dict(d, forearm_calibrator=None):
        q_ref = np.array(d["q_rel_twist_ref"])
        axis = np.array(d["twist_axis"])
        palm = None
        if "palm_in_forearm_imu" in d and d["palm_in_forearm_imu"] is not None:
            palm = np.array(d["palm_in_forearm_imu"])
        elif forearm_calibrator is not None:
            palm_raw_0 = forearm_calibrator.R_align.T @ np.array([0., 0., -1.])
            palm = normalize_vec(quat_rotate(quat_conj(q_ref), palm_raw_0))
        if palm is None:
            palm = np.array([0., 0., -1.])
        return TwistCalibration(
            q_ref, axis, palm,
            float(d.get("palm_pose_consistency_deg", 0.0)),
            float(d.get("palm_up_error_deg", 0.0)),
            float(d.get("palm_down_error_deg", 0.0)))


# ========================================================
#  IncrementalTracker
# ========================================================

class IncrementalTracker:
    """增量 q_rel 跟踪，并在传感器静止时抑制相对零位漂移。

    ``rest`` 由 IMU 端 VQF 根据陀螺仪和加速度计判定。胸部与骨段
    传感器都连续静止若干帧后，不再积分两颗 IMU 之间缓慢且不一致的
    航向修正；仍持续更新原始姿态基准，因此解除静止后不会发生跳变。
    """

    def __init__(self, drift_threshold=0.5, rest_confirm_frames=8):
        self.q_rel_inc = None
        self.q_prev = None
        self.q_chest_prev = None
        self.drift_threshold = drift_threshold
        self.rest_confirm_frames = max(1, int(rest_confirm_frames))
        self.rest_frames = 0
        self.drift_locked = False

    def init(self, q_chest, q_link):
        self.q_rel_inc = normalize_quat(quat_mul(quat_conj(q_chest), q_link))
        self.q_prev = q_link.copy()
        self.q_chest_prev = q_chest.copy()

    def update(self, q_chest, q_link, chest_rest=False, link_rest=False):
        if self.q_rel_inc is None:
            self.init(q_chest, q_link)
            return
        if self.q_chest_prev is None or self.q_prev is None:
            self.q_rel_inc = normalize_quat(quat_mul(quat_conj(q_chest), q_link))
            self.q_prev = q_link.copy()
            self.q_chest_prev = q_chest.copy()
            return

        dq_c = normalize_quat(quat_mul(quat_conj(self.q_chest_prev), q_chest))
        ac = 2 * np.arccos(np.clip(abs(dq_c[0]), 0, 1))
        dq_l = normalize_quat(quat_mul(quat_conj(self.q_prev), q_link))
        al = 2 * np.arccos(np.clip(abs(dq_l[0]), 0, 1))

        if chest_rest and link_rest:
            self.rest_frames += 1
        else:
            self.rest_frames = 0
        self.drift_locked = self.rest_frames >= self.rest_confirm_frames

        if self.drift_locked:
            # 只推进输入基准，不推进输出。这样能滤掉 VQF 在静止期进行
            # 陀螺零偏/磁航向修正时造成的多 IMU 相对零位缓慢移动。
            pass
        elif ac < self.drift_threshold and al < self.drift_threshold:
            self.q_rel_inc = normalize_quat(
                quat_mul(quat_conj(dq_c),
                         quat_mul(self.q_rel_inc, dq_l)))
        else:
            self.q_rel_inc = normalize_quat(
                quat_mul(quat_conj(q_chest), q_link))

        self.q_prev = q_link.copy()
        self.q_chest_prev = q_chest.copy()

    def reset(self):
        self.q_rel_inc = None
        self.q_prev = None
        self.q_chest_prev = None
        self.rest_frames = 0
        self.drift_locked = False


# ========================================================
#  校准姿态常量
# ========================================================

CALIB_POSES = [
    {"name": "右臂自然下垂",          "dir": [0, 0, -1],  "side": "right"},
    {"name": "右臂前平举",            "dir": [1, 0, 0],   "side": "right"},
    {"name": "右臂侧平举(掌心朝下)",  "dir": [0, 1, 0],   "side": "right"},
    {"name": "右臂Roll轴校准",        "dir": None,         "side": "right"},
    {"name": "左臂自然下垂",          "dir": [0, 0, -1],  "side": "left"},
    {"name": "左臂前平举",            "dir": [1, 0, 0],   "side": "left"},
    {"name": "左臂侧平举(掌心朝下)",  "dir": [0, -1, 0],  "side": "left"},
    {"name": "左臂Roll轴校准",        "dir": None,         "side": "left"},
]

T_CHEST2DISP = np.array([[0, 1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)
