# -*- coding: utf-8 -*-
import struct
import unittest
import zlib
from dataclasses import replace
from pathlib import Path

from path_codec_cli import (
    ACTION_FMT,
    ACTION_SIZE,
    CRC32_OFFSET,
    FILE_FLAG_ARRIVAL_YAW_ANCHORS,
    HEADER_FMT,
    HEADER_SIZE,
    NODE_FMT,
    NODE_SIZE,
    PathCodec,
    bin_path_traj_id,
    load_project_dict,
)
from path_models import (
    ACTION_FLAG_HOLD_PATH,
    ACTION_FLAG_LOCKED,
    ACTION_GATE_ACCEL,
    ACTION_GATE_UNCONDITIONAL,
    ACTIONS,
    MechanicalAction,
    PATH_ACT_DROP_12,
    PATH_ACT_DROP_23,
    PATH_ACT_PICK,
    PATH_ACT_PREP_STORE_2,
    PATH_ACT_STORE,
    PROJECT_FORMAT,
    POINT_TYPE_ARRIVAL,
)
from trajectory_planner import plan_project
from v33_test_utils import make_straight_project


class PathCodecV33Test(unittest.TestCase):
    def _project_with_gate(self):
        project = make_straight_project(2400)
        project.points.insert(
            1,
            replace(
                project.points[-1],
                point_id=1,
                x_mm=900,
                stop_required=True,
                gate_id=0,
                is_end=False,
            ),
        )
        project.points[-1].point_id = 2
        project.actions = [
            MechanicalAction(
                action_seq=0,
                action=PATH_ACT_PICK,
                unlock_gate_id=0,
                flags=ACTION_FLAG_LOCKED | ACTION_FLAG_HOLD_PATH,
            )
        ]
        return project

    @staticmethod
    def _repack_header(data: bytes, index: int, value: int) -> bytes:
        values = list(struct.unpack(HEADER_FMT, data[:HEADER_SIZE]))
        values[index] = value
        values[14] = 0
        updated = bytearray(struct.pack(HEADER_FMT, *values) + data[HEADER_SIZE:])
        crc = zlib.crc32(updated) & 0xFFFFFFFF
        updated[CRC32_OFFSET : CRC32_OFFSET + 4] = crc.to_bytes(4, "little")
        return bytes(updated)

    def test_struct_sizes(self):
        self.assertEqual(struct.calcsize(HEADER_FMT), 64)
        self.assertEqual(struct.calcsize(NODE_FMT), 16)
        self.assertEqual(struct.calcsize(ACTION_FMT), 20)
        self.assertEqual((HEADER_SIZE, NODE_SIZE, ACTION_SIZE), (64, 16, 20))
        self.assertEqual(zlib.crc32(b"123456789") & 0xFFFFFFFF, 0xCBF43926)

    def test_json_plan_bin_parse_round_trip(self):
        project = self._project_with_gate()
        loaded = load_project_dict(project.to_dict())
        plan = plan_project(loaded)
        data = PathCodec.build_bin(loaded, plan)
        parsed = PathCodec.parse_bin(data, expected_traj_id=0)
        self.assertEqual(parsed.header.node_count, len(plan.nodes))
        self.assertEqual(parsed.header.action_count, 1)
        self.assertEqual(parsed.header.gate_count, 1)
        self.assertEqual(parsed.actions[0].action, PATH_ACT_PICK)
        self.assertEqual(parsed.nodes[0].speed_mmps, loaded.cut_in.target_speed_mmps)

    def test_crc_damage_is_rejected(self):
        project = self._project_with_gate()
        data = bytearray(PathCodec.build_bin(project, plan_project(project)))
        data[-1] ^= 0x01
        with self.assertRaisesRegex(ValueError, "CRC32"):
            PathCodec.parse_bin(bytes(data), 0)

    def test_version_size_offset_and_reserved_are_rejected(self):
        project = self._project_with_gate()
        data = PathCodec.build_bin(project, plan_project(project))
        with self.assertRaisesRegex(ValueError, "version"):
            PathCodec.parse_bin(self._repack_header(data, 1, 31), 0)
        with self.assertRaisesRegex(ValueError, "header_size"):
            PathCodec.parse_bin(self._repack_header(data, 2, 63), 0)
        with self.assertRaisesRegex(ValueError, "node_offset"):
            PathCodec.parse_bin(self._repack_header(data, 15, 68), 0)
        with self.assertRaisesRegex(ValueError, "reserved"):
            PathCodec.parse_bin(self._repack_header(data, 13, 1), 0)
        with self.assertRaisesRegex(ValueError, "planned_time_ms"):
            PathCodec.parse_bin(self._repack_header(data, 18, 1), 0)
        header = struct.unpack(HEADER_FMT, data[:HEADER_SIZE])
        without_yaw_anchor_flag = header[6] & ~FILE_FLAG_ARRIVAL_YAW_ANCHORS
        with self.assertRaisesRegex(ValueError, "必需标志"):
            PathCodec.parse_bin(
                self._repack_header(data, 6, without_yaw_anchor_flag),
                0,
            )

    def test_file_name_matches_traj_id(self):
        self.assertEqual(bin_path_traj_id(Path("P0359.BIN")), 359)
        with self.assertRaises(ValueError):
            bin_path_traj_id(Path("path.bin"))
        with self.assertRaises(ValueError):
            bin_path_traj_id(Path("P0360.BIN"))

    def test_old_json_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "不兼容"):
            load_project_dict({"format": "HJMB_PATH_EDITOR_JSON_V32"})
        self.assertEqual(load_project_dict({"format": PROJECT_FORMAT}).traj_id, 0)

    def test_removed_yaw_mode_is_rejected(self):
        project = make_straight_project()
        data = project.to_dict()
        data["points"][0]["yaw_mode"] = "FIXED"
        with self.assertRaisesRegex(ValueError, "yaw_mode"):
            load_project_dict(data)

    def test_json_export_contains_configuration_only(self):
        project = self._project_with_gate()
        plan_project(project)
        exported = project.to_config_dict()
        self.assertEqual(exported["format"], PROJECT_FORMAT)
        self.assertEqual(len(exported["points"]), len(project.points))
        self.assertEqual(len(exported["actions"]), len(project.actions))
        for forbidden in ("nodes", "trajectory", "plan", "summary", "planned_nodes"):
            self.assertNotIn(forbidden, exported)

    def test_accel_gate_drop_12_round_trip(self):
        project = make_straight_project(7000)
        project.mechanism_profile.action_duration_ms["DROP_12"] = 100
        project.mechanism_profile.drop_safety_margin_ms = 50
        project.actions = [
            MechanicalAction(
                action_seq=0,
                action=PATH_ACT_DROP_12,
                unlock_gate_id=ACTION_GATE_ACCEL,
                flags=ACTION_FLAG_LOCKED,
                accel_limit_mmps2=100,
                beta_limit_ddegps2=100,
                speed_limit_mmps=2100,
                stable_time_ms=50,
            ),
        ]
        plan = plan_project(project)
        self.assertEqual(project.actions[0].arm_s_mm, 0)
        self.assertEqual(project.actions[0].disarm_s_mm, 0xFFFF)
        self.assertGreater(plan.actions[0].disarm_s_mm, plan.actions[0].arm_s_mm)
        parsed = PathCodec.parse_bin(PathCodec.build_bin(project, plan), 0)
        self.assertEqual(parsed.actions[0].action, PATH_ACT_DROP_12)
        self.assertEqual(parsed.actions[0].unlock_gate_id, ACTION_GATE_ACCEL)

    def test_prep_store_then_store_is_valid(self):
        project = make_straight_project(3000)
        project.actions = [
            MechanicalAction(action_seq=0, action=PATH_ACT_PREP_STORE_2),
            MechanicalAction(
                action_seq=1,
                action=PATH_ACT_STORE,
                unlock_gate_id=ACTION_GATE_UNCONDITIONAL,
            )
        ]
        plan = plan_project(project)
        self.assertEqual(
            [action.action for action in plan.actions],
            [PATH_ACT_PREP_STORE_2, PATH_ACT_STORE],
        )

    def test_store_without_prep_store_is_rejected(self):
        project = make_straight_project(3000)
        project.actions = [MechanicalAction(action_seq=0, action=PATH_ACT_STORE)]
        with self.assertRaisesRegex(ValueError, "PREP_STORE"):
            plan_project(project)

    def test_drop_combinations_are_exactly_1_2_3_12_23(self):
        self.assertEqual(
            {
                code: name
                for code, name in ACTIONS.items()
                if name.startswith("DROP_")
            },
            {
                0x31: "DROP_1",
                0x32: "DROP_2",
                0x33: "DROP_3",
                0x34: "DROP_12",
                0x35: "DROP_23",
            },
        )
        self.assertEqual(ACTIONS[PATH_ACT_DROP_23], "DROP_23")
        project = make_straight_project(3000)
        project.actions = [MechanicalAction(action_seq=0, action=0x36)]
        with self.assertRaisesRegex(ValueError, "非法"):
            plan_project(project)

    def test_pick_requires_arrival_gate_stop(self):
        project = self._project_with_gate()
        project.points[1].type = POINT_TYPE_ARRIVAL
        project.points[1].stop_required = False
        with self.assertRaisesRegex(ValueError, "ARRIVAL\\|GATE\\|STOP"):
            plan_project(project)


if __name__ == "__main__":
    unittest.main()
