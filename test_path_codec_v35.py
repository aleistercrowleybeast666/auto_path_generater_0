# -*- coding: utf-8 -*-
import struct
import unittest
import zlib
from pathlib import Path

from path_codec_cli import (
    ACTION_FMT,
    ACTION_SIZE,
    CRC32_OFFSET,
    HEADER_FMT,
    HEADER_SIZE,
    NODE_FMT,
    NODE_SIZE,
    VERSION,
    PathCodec,
    bin_path_traj_id,
    load_project_dict,
)
from path_models import (
    ACTION_MODE_ASYNC,
    ACTION_MODE_KINEMATIC,
    ACTION_MODE_STOP_AND_WAIT,
    MechanicalAction,
    PATH_ACT_DROP_1,
    PATH_ACT_PICK,
    PATH_MODE_FIXED_8,
    PROJECT_FORMAT,
    POINT_TYPE_ARRIVAL,
    POINT_TYPE_START,
    FIXED_SITE_KEYS,
    YAW_UNSPECIFIED_DDEG,
    resolve_edit_points,
)
from trajectory_planner import plan_project
from v35_test_utils import add_stop_action, make_straight_project


class PathCodecV35Test(unittest.TestCase):
    @staticmethod
    def _repack_header(data: bytes, index: int, value: int) -> bytes:
        values = list(struct.unpack(HEADER_FMT, data[:HEADER_SIZE]))
        values[index] = value
        values[14] = 0
        updated = bytearray(struct.pack(HEADER_FMT, *values) + data[HEADER_SIZE:])
        updated[CRC32_OFFSET : CRC32_OFFSET + 4] = b"\x00\x00\x00\x00"
        crc = zlib.crc32(updated) & 0xFFFFFFFF
        updated[CRC32_OFFSET : CRC32_OFFSET + 4] = crc.to_bytes(4, "little")
        return bytes(updated)

    def test_struct_sizes_and_crc32(self):
        self.assertEqual(struct.calcsize(HEADER_FMT), 64)
        self.assertEqual(struct.calcsize(NODE_FMT), 16)
        self.assertEqual(struct.calcsize(ACTION_FMT), 22)
        self.assertEqual((HEADER_SIZE, NODE_SIZE, ACTION_SIZE), (64, 16, 22))
        self.assertEqual(VERSION, 35)
        self.assertEqual(zlib.crc32(b"123456789") & 0xFFFFFFFF, 0xCBF43926)

    def test_json_plan_bin_parse_round_trip(self):
        project = make_straight_project(3000)
        add_stop_action(project)
        loaded = load_project_dict(project.to_dict())
        plan = plan_project(loaded)
        data = PathCodec.build_bin(loaded, plan)
        parsed = PathCodec.parse_bin(data, expected_traj_id=0)
        self.assertEqual(parsed.header.node_count, len(plan.nodes))
        self.assertEqual(parsed.header.arrival_count, 1)
        self.assertEqual(parsed.header.action_count, 1)
        self.assertEqual(parsed.actions[0].mode, ACTION_MODE_STOP_AND_WAIT)
        self.assertEqual(parsed.actions[0].arrival_id, 0)
        self.assertEqual(parsed.nodes[0].speed_mmps, 0.0)

    def test_crc_version_size_offset_and_reserved_are_rejected(self):
        project = make_straight_project()
        data = PathCodec.build_bin(project, plan_project(project))
        damaged = bytearray(data)
        damaged[-1] ^= 0x01
        with self.assertRaisesRegex(ValueError, "CRC32"):
            PathCodec.parse_bin(bytes(damaged), 0)
        with self.assertRaisesRegex(ValueError, "version"):
            PathCodec.parse_bin(self._repack_header(data, 1, 33), 0)
        with self.assertRaisesRegex(ValueError, "header_size"):
            PathCodec.parse_bin(self._repack_header(data, 2, 63), 0)
        with self.assertRaisesRegex(ValueError, "node_offset"):
            PathCodec.parse_bin(self._repack_header(data, 15, 68), 0)
        with self.assertRaisesRegex(ValueError, "reserved"):
            PathCodec.parse_bin(self._repack_header(data, 13, 1), 0)
        with self.assertRaisesRegex(ValueError, "planned_motion_time_ms"):
            PathCodec.parse_bin(self._repack_header(data, 18, 1), 0)

    def test_file_name_matches_traj_id(self):
        self.assertEqual(bin_path_traj_id(Path("P0359.BIN")), 359)
        with self.assertRaises(ValueError):
            bin_path_traj_id(Path("path.bin"))
        with self.assertRaises(ValueError):
            bin_path_traj_id(Path("P0360.BIN"))

    def test_v34_json_and_removed_fields_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "V3.5"):
            load_project_dict({"format": "HJMB_PATH_EDITOR_JSON_V33"})
        with self.assertRaisesRegex(ValueError, "V3.5"):
            load_project_dict({"format": "HJMB_PATH_EDITOR_JSON_V34"})
        with self.assertRaisesRegex(ValueError, "cut_in"):
            load_project_dict({"format": PROJECT_FORMAT, "cut_in": {}})
        data = make_straight_project().to_dict()
        data["points"][0]["stop_required"] = True
        with self.assertRaisesRegex(ValueError, "stop_required"):
            load_project_dict(data)
        data = make_straight_project().to_dict()
        data["actions"] = [{"action_seq": 0, "action": "PICK", "unlock_gate_id": 0}]
        with self.assertRaisesRegex(ValueError, "unlock_gate_id"):
            load_project_dict(data)
        data = make_straight_project().to_dict()
        data["actions"] = [{"action_seq": 0, "action": "PICK", "min_wait_ms": 1}]
        with self.assertRaisesRegex(ValueError, "min_wait_ms"):
            load_project_dict(data)

    def test_fixed_8_resolves_sites_and_sets_file_flag(self):
        project = make_straight_project()
        self.assertEqual(
            FIXED_SITE_KEYS[:5],
            ("P_START", "P_PICK_1", "P_PICK_2L", "P_PICK_2R", "P_PICK_3"),
        )
        project.path_mode = PATH_MODE_FIXED_8
        project.fixed_sites[0].x_mm = 10
        project.fixed_sites[0].y_mm = 20
        project.fixed_sites[0].yaw_ddeg = 100
        project.fixed_sites[1].x_mm = 1000
        project.fixed_sites[1].y_mm = 20
        project.points[0].site_id = 0
        project.points[1].site_id = 1
        resolved = resolve_edit_points(project)
        self.assertEqual((resolved[0].x_mm, resolved[0].y_mm, resolved[0].yaw_ddeg), (10, 20, 100))
        parsed = PathCodec.parse_bin(PathCodec.build_bin(project, plan_project(project)), 0)
        self.assertTrue(parsed.header.flags & 0x0020)

    def test_drop_fixed_site_yaw_ff_uses_arrival_override(self):
        project = make_straight_project()
        project.path_mode = PATH_MODE_FIXED_8
        project.fixed_sites[0].x_mm = 0
        project.fixed_sites[0].y_mm = 0
        project.fixed_sites[5].x_mm = 1200
        project.fixed_sites[5].y_mm = 400
        project.fixed_sites[5].yaw_ddeg = YAW_UNSPECIFIED_DDEG
        project.points[0].site_id = 0
        project.points[1].site_id = 5
        project.points[1].yaw_ddeg = 450
        resolved = resolve_edit_points(project)
        self.assertEqual((resolved[1].x_mm, resolved[1].y_mm, resolved[1].yaw_ddeg), (1200, 400, 450))
        exported = project.to_config_dict()
        self.assertEqual(exported["points"][1]["yaw_ddeg"], 450)

    def test_pick_fixed_site_yaw_ff_is_rejected(self):
        project = make_straight_project()
        project.path_mode = PATH_MODE_FIXED_8
        project.fixed_sites[1].yaw_ddeg = YAW_UNSPECIFIED_DDEG
        project.points[0].site_id = 0
        project.points[1].site_id = 1
        with self.assertRaisesRegex(ValueError, "0xFF"):
            resolve_edit_points(project)

    def test_async_and_kinematic_actions_round_trip(self):
        project = make_straight_project(4000)
        project.actions = [
            MechanicalAction(
                action_seq=0,
                action=PATH_ACT_PICK,
                mode=ACTION_MODE_ASYNC,
                timeout_ms=2000,
            ),
            MechanicalAction(
                action_seq=1,
                action=PATH_ACT_DROP_1,
                mode=ACTION_MODE_KINEMATIC,
                accel_limit_mmps2=300,
                beta_limit_ddegps2=500,
                wz_limit_ddegps=300,
                speed_limit_mmps=800,
                stable_time_ms=100,
                timeout_ms=2000,
                post_wait_ms=200,
            ),
        ]
        parsed = PathCodec.parse_bin(PathCodec.build_bin(project, plan_project(project)), 0)
        self.assertEqual([action.mode for action in parsed.actions], [ACTION_MODE_ASYNC, ACTION_MODE_KINEMATIC])
        self.assertEqual(parsed.actions[0].check_start_s_mm, 0xFFFF)
        self.assertNotEqual(parsed.actions[1].check_start_s_mm, 0xFFFF)
        self.assertEqual(parsed.actions[1].wz_limit_ddegps, 300)
        self.assertEqual(parsed.actions[1].post_wait_ms, 200)

    def test_json_export_contains_editable_config_only(self):
        project = make_straight_project()
        exported = project.to_config_dict()
        self.assertEqual(exported["format"], PROJECT_FORMAT)
        for forbidden in ("nodes", "trajectory", "plan", "summary", "planned_nodes"):
            self.assertNotIn(forbidden, exported)
        self.assertEqual(exported["points"][1]["type"], POINT_TYPE_ARRIVAL)
        self.assertEqual(exported["points"][0]["type"], POINT_TYPE_START)
        self.assertNotEqual(exported["points"][1].get("yaw_ddeg"), YAW_UNSPECIFIED_DDEG)


if __name__ == "__main__":
    unittest.main()
