"""3D 场景里 G1 与人体模型的摆位测试。

这两件事都不会在出错时报异常，只会让画面变得难看或容易误读，所以用测试
钉住：
  * 两个模型等高 —— 刻意不按真实身高比 (1.32m vs 1.75m)，对照双臂姿态时
    等高更好读；
  * 两个模型到默认镜头等距 —— 否则透视会额外放大/缩小一边，等高也白搭。
"""

import math
import unittest

import numpy as np

import dual_arm_viz as viz
from g1_urdf_renderer import G1UrdfRenderer
from human_gltf_renderer import HumanGltfModel, HumanGltfRenderer


# 与 dual_arm_viz._render_scene 里的默认镜头一致。
CAM_DIST, CAM_ROT_X, CAM_ROT_Y = 2.0, 20.0, -30.0


def default_camera():
    rx, ry = math.radians(CAM_ROT_X), math.radians(CAM_ROT_Y)
    return np.array([
        CAM_DIST * math.sin(ry) * math.cos(rx),
        CAM_DIST * math.cos(ry) * math.cos(rx),
        CAM_DIST * math.sin(rx) + 0.38,
    ])


class SceneLayoutTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        human = HumanGltfModel(viz.HUMAN_GLB_PATH)
        root = HumanGltfRenderer.root_transform(viz.HUMAN_MODEL_OFFSET)
        vertices = np.vstack([a[0] for a in human.skinned_arrays(human.pose())])
        cls.human = (np.c_[vertices, np.ones(len(vertices))] @ root.T)[:, :3]

        renderer = G1UrdfRenderer(viz.G1_URDF_PATH)
        renderer.load_meshes()
        transforms = renderer.model.link_transforms(
            G1UrdfRenderer._joint_values(None, None))
        root = np.eye(4)
        root[:3, :3] = viz.T_URDF2DISP * viz.G1_MODEL_SCALE
        root[:3, 3] = viz.G1_MODEL_OFFSET
        points = []
        for visual in renderer.model.visuals:
            if visual.link not in transforms:
                continue
            matrix = root @ transforms[visual.link] @ visual.origin
            mesh = renderer.meshes[visual.mesh_path].vertices.reshape(-1, 3)
            points.append(
                (np.c_[mesh, np.ones(len(mesh))] @ matrix.T)[:, :3])
        cls.g1 = np.vstack(points)

    @staticmethod
    def height(points):
        return float(points[:, 2].max() - points[:, 2].min())

    def test_the_two_models_are_about_the_same_height(self):
        ratio = self.height(self.g1) / self.height(self.human)
        self.assertAlmostEqual(ratio, 1.0, delta=0.05)

    def test_both_models_stand_on_the_ground_grid(self):
        """地面网格在 z=0, 谁陷进去或者浮着都不对。"""
        for name, points in (("G1", self.g1), ("人体", self.human)):
            with self.subTest(model=name):
                self.assertAlmostEqual(
                    float(points[:, 2].min()), 0.0, delta=0.02)

    def test_both_models_are_equidistant_from_the_default_camera(self):
        """沿显示系 X 摆会让一边远 24%, 透视再缩小 0.81 倍。"""
        camera = default_camera()
        to_g1 = np.linalg.norm(camera - self.g1.mean(axis=0))
        to_human = np.linalg.norm(camera - self.human.mean(axis=0))
        self.assertAlmostEqual(to_g1 / to_human, 1.0, delta=0.05)

    def test_apparent_on_screen_height_ratio_is_close_to_one(self):
        """等高 + 等距的合并结果, 也是用户实际看到的那个比例。"""
        camera = default_camera()
        to_g1 = np.linalg.norm(camera - self.g1.mean(axis=0))
        to_human = np.linalg.norm(camera - self.human.mean(axis=0))
        apparent = (self.height(self.g1) / self.height(self.human)
                    * to_human / to_g1)
        self.assertAlmostEqual(apparent, 1.0, delta=0.08)

    def test_models_do_not_overlap(self):
        """两个模型左右分开, 包围盒不能相交。"""
        self.assertGreater(
            float(self.g1[:, 0].min()), float(self.human[:, 0].max()))

    def test_labels_sit_just_above_each_head(self):
        g1_label = viz.G1_MODEL_OFFSET[2] + viz.G1_LABEL_HEIGHT
        human_label = viz.HUMAN_MODEL_OFFSET[2] + 0.84
        for label, points, name in ((g1_label, self.g1, "G1"),
                                    (human_label, self.human, "人体")):
            with self.subTest(model=name):
                head = float(points[:, 2].max())
                self.assertGreater(label, head)
                self.assertLess(label - head, 0.15)


if __name__ == "__main__":
    unittest.main()
