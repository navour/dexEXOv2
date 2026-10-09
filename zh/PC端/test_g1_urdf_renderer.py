"""G1 URDF 渲染器的无窗口资源与 FK 测试。"""

import os
import unittest

import numpy as np

from g1_urdf_renderer import G1UrdfModel, G1UrdfRenderer


HERE = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(
    os.path.dirname(HERE), "仿真", "models", "g1_description",
    "g1_dual_arm.urdf")
FULL_URDF_PATH = os.path.join(
    os.path.dirname(HERE), "仿真", "models", "g1_29dof", "g1_29dof.urdf")


class G1UrdfRendererTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.model = G1UrdfModel(URDF_PATH)

    def test_official_model_resources_are_complete(self):
        self.assertEqual(self.model.root_link, "waist_yaw_link")
        self.assertIn("right_rubber_hand", self.model.links)
        self.assertIn("left_rubber_hand", self.model.links)
        self.assertGreaterEqual(len(self.model.visuals), 20)
        self.assertTrue(all(os.path.isfile(v.mesh_path)
                            for v in self.model.visuals))

    def test_joint_angle_changes_hand_transform(self):
        neutral = self.model.link_transforms()
        bent = self.model.link_transforms({"right_elbow_joint": 1.0})
        neutral_hand = neutral["right_rubber_hand"][:3, 3]
        bent_hand = bent["right_rubber_hand"][:3, 3]
        self.assertGreater(float(np.linalg.norm(neutral_hand - bent_hand)), 0.05)

    def test_all_stl_meshes_can_be_loaded(self):
        renderer = G1UrdfRenderer(URDF_PATH)
        renderer.load_meshes()
        self.assertEqual(len(renderer.meshes), len(self.model.visuals))
        self.assertGreater(renderer.triangle_count, 100_000)


class G1FullBodyModelTest(unittest.TestCase):
    """全身 29-DOF 模型是默认显示模型, 单独覆盖。"""

    @classmethod
    def setUpClass(cls):
        cls.model = G1UrdfModel(FULL_URDF_PATH)

    def test_resources_are_complete(self):
        self.assertEqual(self.model.root_link, "pelvis")
        for link in ("left_rubber_hand", "right_rubber_hand",
                     "left_ankle_roll_link", "right_ankle_roll_link",
                     "waist_yaw_link", "torso_link"):
            self.assertIn(link, self.model.links)
        self.assertTrue(all(os.path.isfile(v.mesh_path)
                            for v in self.model.visuals))

    def test_standing_pose_puts_the_feet_below_the_pelvis(self):
        """腿只是摆造型, 但造型得是站着的。"""
        transforms = self.model.link_transforms(
            G1UrdfRenderer._joint_values(None, None))
        for side in ("left", "right"):
            ankle = transforms[f"{side}_ankle_roll_link"][:3, 3]
            self.assertAlmostEqual(float(ankle[2]), -0.7442, places=3)

    def test_waist_yaw_rotates_the_arms_but_not_the_legs(self):
        straight = self.model.link_transforms(
            G1UrdfRenderer._joint_values(None, None, waist_yaw=0.0))
        twisted = self.model.link_transforms(
            G1UrdfRenderer._joint_values(None, None, waist_yaw=0.5))

        shoulder_shift = np.linalg.norm(
            straight["right_shoulder_pitch_link"][:3, 3]
            - twisted["right_shoulder_pitch_link"][:3, 3])
        self.assertGreater(float(shoulder_shift), 0.02)

        # 腰在腿之上, 扭腰不该动脚。
        for side in ("left", "right"):
            foot_shift = np.linalg.norm(
                straight[f"{side}_ankle_roll_link"][:3, 3]
                - twisted[f"{side}_ankle_roll_link"][:3, 3])
            self.assertAlmostEqual(float(foot_shift), 0.0, places=12)

    def test_waist_roll_and_pitch_stay_centred(self):
        """与真机接收端一致: 只驱动 yaw。"""
        values = G1UrdfRenderer._joint_values(None, None, waist_yaw=0.7)
        self.assertEqual(values["waist_roll_joint"], 0.0)
        self.assertEqual(values["waist_pitch_joint"], 0.0)

    def test_leg_links_are_not_highlighted_as_the_active_arm(self):
        """全身模型里 left_/right_ 前缀腿也有, 高亮只能落在手臂上。"""
        self.assertTrue(G1UrdfRenderer._is_arm_link("right_elbow_link"))
        self.assertTrue(G1UrdfRenderer._is_arm_link("right_rubber_hand"))
        self.assertFalse(G1UrdfRenderer._is_arm_link("right_knee_link"))
        self.assertFalse(G1UrdfRenderer._is_arm_link("right_hip_yaw_link"))


if __name__ == "__main__":
    unittest.main()
