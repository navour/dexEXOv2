import unittest
import json
import tempfile
from pathlib import Path

import mhandpro_studio as studio
from thumb_retarget import FingerKinematics, ThumbKinematics, retarget_hand_closures
import numpy as np


class SnapshotProtocolTest(unittest.TestCase):
    def test_parses_and_clamps_snapshot(self):
        line = (
            'noise MHAND_SNAPSHOT {"selected":"left","disconnected":false,'
            '"hands":{"right":{"valid":true,"frame":42,"frequency":60,'
            '"power":4.18,"age_ms":7,"sensors_ok":true,"calibrated":true,'
            '"thumb_retarget":true,"closure":[-1,0.2,0.4,0.6,0.8,2]},'
            '"left":{"valid":false,"closure":[0,0,0,0,0,0]}}}')
        result = studio.parse_snapshot_line(line)
        self.assertIsNotNone(result)
        self.assertEqual(result.selected, "left")
        self.assertEqual(result.hands["right"].frame, 42)
        self.assertEqual(result.hands["right"].closure, (0.0, 0.2, 0.4, 0.6, 0.8, 1.0))

    def test_rejects_bad_or_wrong_length_payload(self):
        self.assertIsNone(studio.parse_snapshot_line("not a snapshot"))
        result = studio.parse_snapshot_line(
            'MHAND_SNAPSHOT {"hands":{"right":{"closure":[1,2]},'
            '"left":{"closure":[0,0,0,0,0,0]}}}')
        self.assertEqual(result.hands["right"].closure, (0.0,) * 6)

    def test_thumb_uv_overrides_legacy_thumb_projection_for_ok_pose(self):
        line = (
            'MHAND_SNAPSHOT {"selected":"right","hands":{"right":{'
            '"valid":true,"closure":[0,0,0,0,0.08,0.27],'
            '"thumb_uv":[1.0,0.5]},"left":{"closure":[0,0,0,0,0,0]}}}')
        result = studio.parse_snapshot_line(line)
        self.assertEqual(result.hands["right"].closure[4:], (0.5, 1.0))
        self.assertAlmostEqual(result.hands["right"].closure[3], 0.58)


class UiStateTest(unittest.TestCase):
    def test_chain_status_sits_right_of_left_card_and_frees_model_row(self):
        layout = studio.top_status_layout(1688)
        left_x, _, left_w, left_h = layout["left_card"]
        chain_x, chain_y, _, chain_h = layout["chain"]
        self.assertGreaterEqual(chain_x, left_x + left_w + 12)
        self.assertEqual(chain_y, layout["left_card"][1])
        self.assertLessEqual(chain_y + chain_h,
                             layout["left_card"][1] + left_h)

    def test_fresh_frame_is_online_even_when_a_sensor_needs_attention(self):
        state = studio.HandState(
            valid=True, frequency=60, age_ms=14, sensors_ok=False)
        label, severity = studio.classify_hand_state(state, disconnected=False)
        self.assertEqual(label, "在线 · 节点需检查")
        self.assertEqual(severity, "warning")

    def test_extracts_mapping_step_for_full_screen_workflow(self):
        event = studio.parse_workflow_text(
            "========== mapcal 5/5: OK 手势(双手) ==========\n"
            "拇指指腹和食指指腹捏在一起成一个圈。\n"
            "摆好后按回车开始采集；输入 n 回车取消整次 mapcal: ")
        self.assertEqual(event["step"], 5)
        self.assertEqual(event["count"], 5)
        self.assertEqual(event["title"], "OK 手势(双手)")
        self.assertIn("拇指指腹", event["instruction"])


class OkGestureGeometryTest(unittest.TestCase):
    def test_ok_anchor_brings_index_and_thumb_pads_together(self):
        closure = retarget_hand_closures(
            (0.0, 0.0, 0.0, 0.80, 0.08, 0.27), 1.0, 0.5)
        self.assertAlmostEqual(closure[3], 0.58)
        for side in ("right", "left"):
            thumb = ThumbKinematics(side)
            index = FingerKinematics("index", side)
            thumb_pad = thumb.pad_position(
                thumb.beta_max * closure[5], thumb.theta_max * closure[4])
            gap_mm = float(np.linalg.norm(
                thumb_pad - index.pad_position(closure[3])) * 1000.0)
            self.assertLess(gap_mm, 5.0)

    def test_palm_anchor_does_not_change_index(self):
        closure = retarget_hand_closures(
            (0.0, 0.0, 0.0, 0.73, 0.0, 0.0), 1.0, 0.0)
        self.assertAlmostEqual(closure[3], 0.73)


class TeleopTopologyTest(unittest.TestCase):
    def test_both_uses_existing_sim_tcp_pipeline_not_direct_modbus(self):
        command = studio.build_teleop_command("both")
        self.assertEqual(
            command,
            "teleop both "
            f"{studio.RIGHT_SIM_CONFIG} {studio.LEFT_SIM_CONFIG}",
        )
        self.assertNotIn("192.168.123", command)

    def test_single_side_uses_matching_sim_config(self):
        self.assertEqual(
            studio.build_teleop_command("right"),
            f"teleop {studio.RIGHT_SIM_CONFIG}",
        )
        self.assertEqual(
            studio.build_teleop_command("left"),
            f"teleop {studio.LEFT_SIM_CONFIG}",
        )

    def test_output_button_dispatches_original_both_command(self):
        class FakeBackend:
            def __init__(self):
                self.snapshot = studio.Snapshot()
                for state in self.snapshot.hands.values():
                    state.valid = True
                    state.calibrated = True
                self.logs = []
                self.workflow = {}
                self.command = None

            def run_action(self, command, side):
                self.command = (command, side)
                return True

        ui = studio.MHandStudio.__new__(studio.MHandStudio)
        ui.backend = FakeBackend()
        ui.selected = "both"
        ui.modal = None
        ui._request_teleop()
        self.assertEqual(
            ui.backend.command,
            (studio.build_teleop_command("both"), "both"),
        )
        self.assertEqual(ui.modal["kind"], "workflow")


class HandUrdfTest(unittest.TestCase):
    def test_filter_contains_both_hands_and_fk(self):
        renderer = studio.G1UrdfRenderer(
            studio.URDF_PATH, link_filter=studio.MHandStudio._is_hand_link)
        links = {visual.link for visual in renderer.model.visuals
                 if studio.MHandStudio._is_hand_link(visual.link)}
        self.assertIn("right_thumb_4", links)
        self.assertIn("left_thumb_4", links)
        transforms = renderer.model.link_transforms()
        self.assertIn("right_index_2", transforms)
        self.assertIn("left_index_2", transforms)


class SafeReplayTest(unittest.TestCase):
    def test_replay_backend_has_no_true_hand_command_path(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "episode.jsonl"
            row = {
                "kind": "frame",
                "glove": {
                    "selected": "right", "disconnected": False,
                    "hands": {
                        "right": {"valid": True, "closure": [0.5] * 6},
                        "left": {"valid": True, "closure": [0.0] * 6},
                    },
                },
            }
            path.write_text(json.dumps(row) + "\n", encoding="utf-8")
            backend = studio.ReplayBackend(path)
            self.assertAlmostEqual(backend.snapshot.hands["right"].closure[0], 0.5)
            self.assertFalse(backend.send("teleop both anything"))


if __name__ == "__main__":
    unittest.main()
