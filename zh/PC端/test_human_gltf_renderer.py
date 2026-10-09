import os
import unittest

import numpy as np

from human_gltf_renderer import HumanGltfModel, HumanGltfRenderer


ASSET_PATH = os.path.join(
    os.path.dirname(__file__), "assets", "human", "animated_base_character.glb")


class HumanGltfModelTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.model = HumanGltfModel(ASSET_PATH)

    def test_model_contains_expected_mesh_and_bones(self):
        self.assertEqual(len(self.model.primitives), 2)
        self.assertEqual(self.model.triangle_count, 13744)
        for name in (
                "DEF-spine.003", "DEF-upper_arm.R",
                "DEF-forearm.R", "DEF-hand.R"):
            self.assertIn(name, self.model.node_by_name)

    def test_pose_aims_right_arm_at_requested_directions(self):
        upper_target = np.array([0.35, -0.25, -0.90])
        forearm_target = np.array([0.80, 0.10, -0.25])
        upper_target /= np.linalg.norm(upper_target)
        forearm_target /= np.linalg.norm(forearm_target)
        globals_ = self.model.pose({
            "right": (upper_target, forearm_target, 0.2),
        })
        by_name = self.model.node_by_name
        upper_direction = (
            globals_[by_name["DEF-forearm.R"]][:3, 3]
            - globals_[by_name["DEF-upper_arm.R"]][:3, 3])
        forearm_direction = (
            globals_[by_name["DEF-hand.R"]][:3, 3]
            - globals_[by_name["DEF-forearm.R"]][:3, 3])
        upper_direction /= np.linalg.norm(upper_direction)
        forearm_direction /= np.linalg.norm(forearm_direction)
        np.testing.assert_allclose(upper_direction, upper_target, atol=1e-6)
        np.testing.assert_allclose(forearm_direction, forearm_target, atol=1e-6)

    def test_cpu_skinning_returns_finite_vertex_arrays(self):
        globals_ = self.model.pose()
        arrays = self.model.skinned_arrays(globals_)
        self.assertEqual(len(arrays), 2)
        for primitive, (positions, normals) in zip(self.model.primitives, arrays):
            self.assertEqual(positions.shape, primitive.positions.shape)
            self.assertEqual(normals.shape, primitive.normals.shape)
            self.assertTrue(np.isfinite(positions).all())
            self.assertTrue(np.isfinite(normals).all())
            np.testing.assert_allclose(
                np.linalg.norm(normals, axis=1), 1.0, atol=1e-5)

    def test_display_direction_conversion_is_a_rotation(self):
        direction = np.array([0.2, 0.7, -0.4])
        converted = HumanGltfRenderer.direction_to_model(direction)
        self.assertAlmostEqual(
            float(np.linalg.norm(direction)), float(np.linalg.norm(converted)))


if __name__ == "__main__":
    unittest.main()
