"""统一校准与解算模块 — 从 upper_arm_3pose.py 提取.

提供:
  - 四元数工具函数
  - ArmDirectionCalibrator (3姿态迭代Procrustes + Roll精修)
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

@dataclass
class TwistCalibration:
    q_rel_twist_ref: np.ndarray
    twist_axis: np.ndarray
    palm_in_forearm_imu: np.ndarray

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
        return TwistCalibration(q_ref, axis, palm)


# ========================================================
#  IncrementalTracker
# ========================================================

class IncrementalTracker:
    """增量 q_rel 跟踪, 消除 IMU 绝对朝向漂移."""

    def __init__(self, drift_threshold=0.5):
        self.q_rel_inc = None
        self.q_prev = None
        self.q_chest_prev = None
        self.drift_threshold = drift_threshold

    def init(self, q_chest, q_link):
        self.q_rel_inc = normalize_quat(quat_mul(quat_conj(q_chest), q_link))
        self.q_prev = q_link.copy()
        self.q_chest_prev = q_chest.copy()

    def update(self, q_chest, q_link):
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

        if ac < self.drift_threshold and al < self.drift_threshold:
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
