"""校验 g1_29dof 的 URDF 与 MJCF 描述的是同一台机器人。

同一目录下放着两份模型：MJCF 给 MuJoCo（本仿真和 HumDex 全身仿真跑的就是
它），URDF 给 PC 端的 OpenGL 渲染器。两者来自不同上游 —— MJCF 出自
HumDex 的 assets，URDF 出自宇树 unitree_ros —— 所以不能假定它们一致。

不一致不会报错，只会让上位机画的姿态和仿真里跑的姿态悄悄对不上。因此这里
直接逐 link 对拍正运动学，而不是只检查文件能否解析。
"""

import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import numpy as np


MODELS = Path(__file__).resolve().parent / "models" / "g1_29dof"
URDF = MODELS / "g1_29dof.urdf"
MJCF = MODELS / "g1_29dof.xml"

# 遥操实际驱动的关节：每臂 5 轴 + 腰 yaw。
DRIVEN_JOINTS = (
    "waist_yaw_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
)


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def axis_angle(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    skew = np.array([
        [0.0, -axis[2], axis[1]],
        [axis[2], 0.0, -axis[0]],
        [-axis[1], axis[0], 0.0],
    ])
    matrix = np.eye(4)
    matrix[:3, :3] = (np.eye(3) * cos_a + sin_a * skew
                      + (1.0 - cos_a) * np.outer(axis, axis))
    return matrix


class UrdfFk:
    """与 PC端/g1_urdf_renderer.G1UrdfModel 同算法，但不依赖 OpenGL。"""

    def __init__(self, path):
        root = ET.parse(path).getroot()
        self.links = {link.get("name") for link in root.findall("link")}
        self.joints = []
        for joint in root.findall("joint"):
            origin = joint.find("origin")
            axis = joint.find("axis")
            transform = np.eye(4)
            if origin is not None:
                transform[:3, :3] = rpy_matrix(
                    *[float(v) for v in (origin.get("rpy") or "0 0 0").split()])
                transform[:3, 3] = [
                    float(v) for v in (origin.get("xyz") or "0 0 0").split()]
            self.joints.append({
                "name": joint.get("name"),
                "type": joint.get("type"),
                "parent": joint.find("parent").get("link"),
                "child": joint.find("child").get("link"),
                "origin": transform,
                "axis": ([float(v) for v in axis.get("xyz").split()]
                         if axis is not None else [1.0, 0.0, 0.0]),
                "limit": joint.find("limit"),
            })
        roots = self.links - {joint["child"] for joint in self.joints}
        assert len(roots) == 1, f"根 link 数量异常: {roots}"
        self.root = roots.pop()

    def limits(self):
        result = {}
        for joint in self.joints:
            if joint["type"] == "revolute" and joint["limit"] is not None:
                result[joint["name"]] = (
                    float(joint["limit"].get("lower")),
                    float(joint["limit"].get("upper")))
        return result

    def transforms(self, values):
        by_parent = {}
        for joint in self.joints:
            by_parent.setdefault(joint["parent"], []).append(joint)
        result = {self.root: np.eye(4)}
        pending = [self.root]
        while pending:
            parent = pending.pop()
            for joint in by_parent.get(parent, []):
                motion = np.eye(4)
                if joint["type"] in ("revolute", "continuous"):
                    motion = axis_angle(
                        joint["axis"], values.get(joint["name"], 0.0))
                result[joint["child"]] = (
                    result[parent] @ joint["origin"] @ motion)
                pending.append(joint["child"])
        return result


class ModelConsistencyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fk = UrdfFk(URDF)

    def test_both_models_are_present(self):
        self.assertTrue(URDF.is_file())
        self.assertTrue(MJCF.is_file())

    def test_urdf_root_is_the_pelvis(self):
        self.assertEqual(self.fk.root, "pelvis")

    def test_urdf_has_29_revolute_joints(self):
        self.assertEqual(len(self.fk.limits()), 29)

    def test_every_mesh_file_is_present(self):
        root = ET.parse(URDF).getroot()
        for mesh in root.iter("mesh"):
            path = URDF.parent / mesh.get("filename")
            self.assertTrue(path.is_file(), f"缺少 mesh: {path}")

    def test_driven_joints_exist_in_both(self):
        import mujoco

        model = mujoco.MjModel.from_xml_path(str(MJCF))
        urdf_limits = self.fk.limits()
        for name in DRIVEN_JOINTS:
            self.assertIn(name, urdf_limits, f"URDF 缺少 {name}")
            self.assertGreaterEqual(
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name), 0,
                f"MJCF 缺少 {name}")

    def test_joint_limits_agree(self):
        """限位不一致会让上位机和仿真在同一个指令下夹到不同角度。"""
        import mujoco

        model = mujoco.MjModel.from_xml_path(str(MJCF))
        for name, (lower, upper) in self.fk.limits().items():
            joint_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id < 0:
                continue
            mj_lower, mj_upper = model.jnt_range[joint_id]
            self.assertAlmostEqual(lower, float(mj_lower), places=3, msg=name)
            self.assertAlmostEqual(upper, float(mj_upper), places=3, msg=name)

    def test_forward_kinematics_agrees(self):
        """两份模型必须描述同一台机器人，否则画面与仿真会悄悄错位。"""
        import mujoco

        model = mujoco.MjModel.from_xml_path(str(MJCF))
        data = mujoco.MjData(model)
        hinge_names = [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
            for i in range(model.njnt)
            if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE
        ]
        rng = np.random.default_rng(0)
        pelvis_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")

        worst, worst_link = 0.0, None
        for _ in range(25):
            values = {name: float(rng.uniform(-0.6, 0.6))
                      for name in hinge_names}
            data.qpos[:] = 0.0
            data.qpos[3] = 1.0          # freejoint: 原点 + 单位四元数
            for name, value in values.items():
                joint_id = mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_JOINT, name)
                data.qpos[model.jnt_qposadr[joint_id]] = value
            mujoco.mj_forward(model, data)

            transforms = self.fk.transforms(values)
            pelvis = data.xpos[pelvis_id]
            for name, transform in transforms.items():
                body_id = mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_BODY, name)
                if body_id < 0:
                    continue        # URDF 里多出的固定 frame，MJCF 没有对应 body
                error = float(np.linalg.norm(
                    (data.xpos[body_id] - pelvis) - transform[:3, 3]))
                if error > worst:
                    worst, worst_link = error, name
        self.assertLess(
            worst, 1e-6, f"URDF 与 MJCF 的 FK 偏差 {worst:.3e} m @ {worst_link}")


if __name__ == "__main__":
    unittest.main()
