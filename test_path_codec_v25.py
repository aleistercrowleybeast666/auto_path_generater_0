# -*- coding: utf-8 -*-
import unittest
from dataclasses import replace

from path_codec_cli import (
    ACTION_FLAG_HOLD_PATH,
    ACTION_FLAG_LOCKED,
    ACTION_FLAG_REQUIRED_AT_END,
    PATH_FLAG_END,
    PATH_FLAG_WAIT_ACTION_V2,
    PATH_POINT_ARRIVE_SCAN,
    PATH_POINT_PASS,
    PROJECT_FORMAT_V2,
    LegacyPathPoint,
    MechanicalAction,
    PathCodec,
    PathPoint,
    load_project_dict,
)


class PathCodecV25Test(unittest.TestCase):
    def setUp(self):
        self.points = [
            PathPoint(x_mm=0, y_mm=0, point_id=0),
            PathPoint(x_mm=500, y_mm=0, point_id=1, type=PATH_POINT_ARRIVE_SCAN, gate_id=0),
            PathPoint(x_mm=800, y_mm=100, point_id=2),
            PathPoint(x_mm=1200, y_mm=100, point_id=3, type=PATH_POINT_ARRIVE_SCAN, gate_id=1),
            PathPoint(
                x_mm=1500,
                y_mm=300,
                point_id=4,
                type=PATH_POINT_ARRIVE_SCAN,
                gate_id=2,
                flags=PATH_FLAG_END,
            ),
        ]
        locked_hold = ACTION_FLAG_LOCKED | ACTION_FLAG_HOLD_PATH
        self.actions = [
            MechanicalAction(action_seq=0, action=0x11),
            MechanicalAction(action_seq=1, action=0x20, unlock_gate_id=0, flags=locked_hold),
            MechanicalAction(action_seq=2, action=0x31),
            MechanicalAction(action_seq=3, action=0x13),
            MechanicalAction(action_seq=4, action=0x20, unlock_gate_id=1, flags=locked_hold),
            MechanicalAction(action_seq=5, action=0x32),
            MechanicalAction(
                action_seq=6,
                action=0x43,
                unlock_gate_id=2,
                flags=locked_hold | ACTION_FLAG_REQUIRED_AT_END,
            ),
        ]

    def assert_build_rejected(self, points=None, actions=None, message=""):
        with self.assertRaisesRegex(ValueError, message):
            PathCodec.build_bin(0, points or self.points, actions or self.actions)

    def test_v25_build_parse_round_trip(self):
        data = PathCodec.build_bin(0, self.points, self.actions)
        traj_id, points, actions = PathCodec.parse_bin(data, expected_traj_id=0)
        self.assertEqual(traj_id, 0)
        self.assertEqual(points, self.points)
        self.assertEqual(actions, self.actions)

    def test_crc_damage_is_rejected(self):
        data = bytearray(PathCodec.build_bin(0, self.points, self.actions))
        data[-1] ^= 0x01
        with self.assertRaisesRegex(ValueError, "CRC32"):
            PathCodec.parse_bin(bytes(data), expected_traj_id=0)

    def test_wrong_file_size_is_rejected(self):
        data = PathCodec.build_bin(0, self.points, self.actions) + b"\x00"
        with self.assertRaisesRegex(ValueError, "文件大小"):
            PathCodec.parse_bin(data, expected_traj_id=0)

    def test_gate_non_contiguous_and_duplicate_are_rejected(self):
        non_contiguous = [replace(point) for point in self.points]
        non_contiguous[3].gate_id = 2
        non_contiguous[4].gate_id = 3
        self.assert_build_rejected(non_contiguous, self.actions, "Gate 必须按路径顺序连续")

        duplicate = [replace(point) for point in self.points]
        duplicate[3].gate_id = 0
        self.assert_build_rejected(duplicate, self.actions, "重复")

    def test_action_seq_must_be_contiguous(self):
        actions = [replace(action) for action in self.actions]
        actions[3].action_seq = 8
        self.assert_build_rejected(self.points, actions, "action_seq=8")

    def test_locked_and_unlock_gate_must_match(self):
        actions = [replace(action) for action in self.actions]
        actions[0].unlock_gate_id = 0
        self.assert_build_rejected(self.points, actions, "LOCKED=0")

        actions = [replace(action) for action in self.actions]
        actions[1].unlock_gate_id = 0xFF
        self.assert_build_rejected(self.points, actions, "LOCKED=1")

    def test_pick_drop_must_be_locked_and_use_arrive_gate(self):
        actions = [replace(action) for action in self.actions]
        actions[1].flags = 0
        actions[1].unlock_gate_id = 0xFF
        self.assert_build_rejected(self.points, actions, r"LOCKED\|HOLD_PATH")

        points = [replace(point) for point in self.points]
        points[1].type = PATH_POINT_PASS
        self.assert_build_rejected(points, self.actions, "PASS 点")

    def test_locked_gate_order_must_not_go_backwards(self):
        actions = [replace(action) for action in self.actions]
        actions[1].unlock_gate_id = 1
        actions[4].unlock_gate_id = 0
        self.assert_build_rejected(self.points, actions, "FIFO 死锁风险")

    def test_v2_json_is_explicitly_converted(self):
        legacy_json = {
            "format": PROJECT_FORMAT_V2,
            "traj_id": 0,
            "points": [
                {
                    "x_mm": 0,
                    "y_mm": 0,
                    "yaw_ddeg": 0,
                    "point_id": 0,
                    "type": PATH_POINT_PASS,
                    "action": 0x11,
                    "marker_id": 0xFF,
                    "flags": 0,
                },
                {
                    "x_mm": 500,
                    "y_mm": 0,
                    "yaw_ddeg": 0,
                    "point_id": 1,
                    "type": PATH_POINT_ARRIVE_SCAN,
                    "action": 0x20,
                    "marker_id": 5,
                    "flags": PATH_FLAG_WAIT_ACTION_V2 | PATH_FLAG_END,
                },
            ],
        }
        project = load_project_dict(legacy_json)
        self.assertTrue(project.migrated_from_v2)
        self.assertEqual([point.gate_id for point in project.points], [0xFF, 0])
        self.assertEqual([action.action for action in project.actions], [0x11, 0x20])
        self.assertEqual(project.actions[1].flags, ACTION_FLAG_LOCKED | ACTION_FLAG_HOLD_PATH)
        self.assertEqual(project.points[1].flags, PATH_FLAG_END)

    def test_v2_bin_is_explicitly_converted(self):
        legacy_points = [
            LegacyPathPoint(0, 0, 0, 0, PATH_POINT_PASS, 0x11, 0xFF, 0),
            LegacyPathPoint(
                500,
                0,
                0,
                1,
                PATH_POINT_ARRIVE_SCAN,
                0x41,
                5,
                PATH_FLAG_WAIT_ACTION_V2 | PATH_FLAG_END,
            ),
        ]
        data = PathCodec.build_v2_bin(0, legacy_points)
        project = PathCodec.load_bin(data, expected_traj_id=0)
        self.assertTrue(project.migrated_from_v2)
        self.assertEqual(project.points[1].gate_id, 0)
        self.assertEqual(project.actions[1].unlock_gate_id, 0)
        self.assertEqual(
            project.actions[1].flags,
            ACTION_FLAG_LOCKED | ACTION_FLAG_HOLD_PATH | ACTION_FLAG_REQUIRED_AT_END,
        )


if __name__ == "__main__":
    unittest.main()
