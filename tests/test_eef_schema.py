"""Pose mathematics, layout selection, and production recorder error paths."""

import ast
from collections import deque
import copy
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np

from eef_schema import (
    EEF_NAMES, EefSyncError, capture_schema, interpolate_poses, parse_eef_pose,
    select_indices, synchronized_pose, validate_layout,
)
from robot_schema import build_robot_schema


def payload(rotation=None):
    return {f"{side}_eef_pose": {"position": [0.4, y, 0.8], "rotation": list(rotation or [0., 0., 0., 1.])}
            for side, y in (("left", .2), ("right", -.2))}


def sample(stamp, pose=None, received=None):
    return SimpleNamespace(value=parse_eef_pose(payload()) if pose is None else pose,
                           stamp_sec=stamp, received_sec=stamp if received is None else received)


class EefSchemaTest(unittest.TestCase):
    def test_complete_finite_payload_and_quaternion_normalization(self):
        pose = parse_eef_pose(payload([0, 0, 0, -2]))
        self.assertEqual(pose.shape, (14,))
        self.assertEqual(pose.dtype, np.float32)
        np.testing.assert_allclose(pose[[6, 13]], 1)
        for broken in (None, {}, {"pos_left_in_robot": [0, 0, 0]}, payload([0, 0, 0, 0]),
                       payload([0, 0, float("nan"), 1]), payload([0, 0, True, 1]), payload([0, 0, 1])):
            self.assertIsNone(parse_eef_pose(broken))
        broken = payload()
        broken["right_eef_pose"]["position"][1] = float("inf")
        self.assertIsNone(parse_eef_pose(broken))

    def test_slerp_rotation_not_component_lerp(self):
        before = parse_eef_pose(payload())
        after = parse_eef_pose(payload([0, 0, 1, 0]))
        after[0] += 2
        halfway = interpolate_poses(before, after, .5)
        np.testing.assert_allclose(halfway[3:7], [0, 0, 2 ** -.5, 2 ** -.5], atol=1e-6)
        self.assertAlmostEqual(float(halfway[0]), 1.4, places=5)
        after[3:7] = -before[3:7]
        np.testing.assert_allclose(interpolate_poses(before, after, .5)[3:7], before[3:7])

    def test_synchronized_pose_wait_vs_invalid(self):
        samples = deque([sample(10), sample(10.02)])
        pose, deltas, ages = synchronized_pose(samples, 10.01, 10.06, max_gap=.05, max_age=.15)
        self.assertEqual(len(pose), 14)
        self.assertAlmostEqual(deltas["eef_state_before"], -10)
        self.assertAlmostEqual(ages["eef_state_after"], 40)
        scenarios = [(deque(), 10.01, 10.06, "before_missing", False),
                     (deque([sample(10)]), 10.01, 10.06, "after_not_ready", True),
                     (samples, 10.01, 11, "stale", False),
                     (deque([sample(10), sample(10.1)]), 10.05, 10.12, "gap", False)]
        for buffer, anchor, now, reason, wait in scenarios:
            with self.subTest(reason=reason), self.assertRaises(EefSyncError) as caught:
                synchronized_pose(buffer, anchor, now, max_gap=.05, max_age=.15)
            self.assertIn(reason, str(caught.exception))
            self.assertEqual(caught.exception.wait, wait)

    def test_all_feature_combinations_and_stable_joint_prefix(self):
        for head, waist, upper in ((False, False, False), (True, False, True), (True, True, False)):
            joints = build_robot_schema(head, waist, upper).names
            for state in (False, True):
                for action in (False, True):
                    layout = capture_schema(joints, with_state=state, with_action=action)
                    info = {"features": {key: dict(dtype="float32", shape=[len(entry["names"])], names=entry["names"])
                                         for key, entry in layout["features"].items()}}
                    validate_layout(info, layout)
                    for key, enabled in (("observation.state", state), ("action", action)):
                        entry = layout["features"][key]
                        self.assertEqual(entry["names"][:len(joints)], list(joints))
                        self.assertEqual(len(entry["names"]), len(joints) + 14 * enabled)
                        self.assertEqual(entry["groups"]["gripper"], [14, 15])
                        if enabled:
                            self.assertEqual([entry["names"][i] for i in select_indices(entry, "eef")], list(EEF_NAMES))
                            self.assertEqual(len(select_indices(entry, "eef+gripper")), 16)
                        else:
                            with self.assertRaisesRegex(ValueError, "not recorded"):
                                select_indices(entry, "eef")

    def test_ambiguous_layout_rejected(self):
        layout = capture_schema(build_robot_schema().names, with_state=True, with_action=True)
        info = {"features": {key: dict(dtype="float32", shape=[30], names=entry["names"])
                             for key, entry in layout["features"].items()}}
        for field, value in (("eef", [1]), ("joint", [1, 1]), ("gripper", [0])):
            broken = copy.deepcopy(layout)
            broken["features"]["action"]["groups"][field] = value
            with self.assertRaises(ValueError):
                validate_layout(info, broken)


class RecorderEefTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = Path(__file__).resolve().parents[1] / "record_lerobot_official.py"
        tree = ast.parse(path.read_text())
        methods = [method for node in tree.body if isinstance(node, ast.ClassDef)
                   for method in node.body if isinstance(method, ast.FunctionDef) and method.name == "_append_eef"]
        namespace = dict(np=np, synchronized_pose=synchronized_pose, EefSyncError=EefSyncError)
        exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), "exec"), namespace)
        cls.append_eef = staticmethod(namespace["_append_eef"])

    def node(self, state=True, action=True):
        node = Mock()
        node.args = SimpleNamespace(with_eef_state=state, with_eef_action=action,
                                    max_state_interpolation_gap_sec=.05, max_state_age_sec=.15)
        node.eef_state_buffer = deque([sample(10), sample(10.02)])
        node._held_signal.return_value = sample(10)
        return node

    def test_independent_flags_and_no_extra_sampling_when_disabled(self):
        for state in (False, True):
            for action in (False, True):
                node = self.node(state, action)
                frame = {"observation.state": np.zeros(21, np.float32), "action": np.ones(21, np.float32)}
                self.assertTrue(self.append_eef(node, frame, 10.01, 10.06, {}, {}))
                self.assertEqual(frame["observation.state"].size, 21 + 14 * state)
                self.assertEqual(frame["action"].size, 21 + 14 * action)
                self.assertEqual(node._held_signal.call_count, int(action))
                np.testing.assert_allclose(frame["action"][:21], 1)

    def test_missing_action_invalidates_no_state_fallback(self):
        node = self.node()
        node._held_signal.return_value = None
        self.assertFalse(self.append_eef(node, {"observation.state": np.zeros(16), "action": np.zeros(16)}, 10.01, 10.06, {}, {}))
        self.assertEqual(node._invalidate_episode.call_args.args[0], "missing_stale_or_noncausal_eef_action")

    def test_wait_does_not_invalidate_or_consume_image(self):
        node = self.node()
        node.eef_state_buffer.pop()
        self.assertFalse(self.append_eef(node, {"observation.state": np.zeros(16), "action": np.zeros(16)}, 10.01, 10.06, {}, {}))
        node._wait_for_reference.assert_called_once()
        node._invalidate_episode.assert_not_called()
        node._held_signal.assert_not_called()


if __name__ == "__main__":
    unittest.main()
