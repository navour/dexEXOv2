"""带因时 FTP 灵巧手的 URDF 在上位机渲染器里的无窗口测试。

渲染器不解析 URDF 的 ``<mimic>``，所以联动关节必须由调用方用
``hand_mapping.expand_mimic()`` 算好一起传进来。只传 6 个驱动关节的话手指
会只弯一半 —— 不报错、不崩溃，就是画得不对。这里把这件事钉住。
"""

import os
import unittest

import numpy as np

import hand_mapping as hm
from g1_urdf_renderer import G1UrdfModel, G1UrdfRenderer


HERE = os.path.dirname(os.path.abspath(__file__))
HAND_URDF_PATH = os.path.join(
    os.path.dirname(HERE), "仿真", "models", "g1_29dof",
    "g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf")


@unittest.skipUnless(os.path.isfile(HAND_URDF_PATH), "缺少因时 FTP 模型")
class HandModelTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.model = G1UrdfModel(HAND_URDF_PATH)

    def test_model_parses_and_meshes_exist(self):
        missing = [v.mesh_path for v in self.model.visuals
                   if not os.path.isfile(v.mesh_path)]
        self.assertEqual(missing, [], "URDF 引用了不存在的网格")

    def test_every_mapped_joint_is_in_the_urdf(self):
        """hand_mapping 的表和 URDF 必须一个不差。

        少一个不会报错，只会那根手指在画面里不动，所以逐个核。
        """
        names = {joint.name for joint in self.model.joints}
        for side in ("right", "left"):
            for driver in hm.driver_joints(side):
                self.assertIn(driver, names)
                for passive, _ in hm.mimic_chains(side)[driver]:
                    self.assertIn(passive, names)

    def test_hand_joints_actually_move_the_links(self):
        open_tf = self.model.link_transforms(
            G1UrdfRenderer._joint_values(None, None, 0.0, None))
        fist = hm.expand_mimic(hm.closure_to_angles((1.0,) * 6))
        closed_tf = self.model.link_transforms(
            G1UrdfRenderer._joint_values(None, None, 0.0, fist))

        moved = [name for name in open_tf
                 if name in closed_tf
                 and not np.allclose(open_tf[name], closed_tf[name])]
        self.assertTrue(moved, "传了手部关节角，却没有任何连杆动")
        # 只应该动右手；左手没给数据，不能跟着动。
        self.assertTrue(all("left" not in name for name in moved),
                        f"左手不该动: {[n for n in moved if 'left' in n]}")

    def test_passive_joints_are_not_driven_by_the_urdf_itself(self):
        """确认渲染器确实不解析 <mimic> —— 这正是要在调用方展开的原因。

        只喂 6 个驱动关节，被动关节应当保持在 0；如果哪天渲染器支持了
        mimic，这个测试会失败，那时 expand_mimic 的双重驱动就得撤掉。
        """
        drivers_only = dict(zip(
            hm.driver_joints("right"), hm.closure_to_angles((1.0,) * 6)))
        partial = self.model.link_transforms(
            G1UrdfRenderer._joint_values(None, None, 0.0, drivers_only))
        full = self.model.link_transforms(
            G1UrdfRenderer._joint_values(
                None, None, 0.0,
                hm.expand_mimic(hm.closure_to_angles((1.0,) * 6))))

        passive_link = "right_little_2"
        if passive_link not in partial:
            self.skipTest(f"URDF 里没有 {passive_link} 连杆")
        self.assertFalse(
            np.allclose(partial[passive_link], full[passive_link]),
            "被动关节在只喂驱动关节时也动了 —— 渲染器可能已支持 mimic")

    def test_open_hand_is_the_zero_pose(self):
        zero = self.model.link_transforms(
            G1UrdfRenderer._joint_values(None, None, 0.0, None))
        opened = self.model.link_transforms(
            G1UrdfRenderer._joint_values(
                None, None, 0.0,
                hm.expand_mimic(hm.closure_to_angles(hm.open_closure()))))
        for name, transform in zero.items():
            self.assertTrue(np.allclose(transform, opened[name]),
                            f"{name}: 张开状态应当等于零位")


if __name__ == "__main__":
    unittest.main()
