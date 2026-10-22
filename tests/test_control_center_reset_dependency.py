"""Audit the new reset-pose import without ROS or any motor interface."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from dagger import dependencies as deps

ROOT = Path(__file__).resolve().parents[1]
LOADER = ROOT / 'teleop_snapshot/control_center_reset_pose/reset_pose_loader.py'
spec = importlib.util.spec_from_file_location('reviewed_reset_pose_loader', LOADER)
loader = importlib.util.module_from_spec(spec)
spec.loader.exec_module(loader)


class ResetDependencyTest(unittest.TestCase):
    def test_new_controller_requires_exact_loader_old_controller_does_not(self):
        relative = 'openarmx_teleop_vr_306_v4/openarmx_teleop_vr_306_v4/controller_node.py'
        old, new = hashlib.sha256(b'old').hexdigest(), hashlib.sha256(b'new').hexdigest()
        self.assertEqual(hashlib.sha256(LOADER.read_bytes()).hexdigest(), deps.RESET_POSE_LOADER_HASH)
        with tempfile.TemporaryDirectory() as directory, \
                patch.object(deps, 'HASHES', {relative: old}), \
                patch.object(deps, 'REVIEWED_ALTERNATES', {relative: {new}}), \
                patch.object(deps, 'CONTROL_CENTER_RESET_CONTROLLER', new):
            root = Path(directory)
            controller = root / relative
            controller.parent.mkdir(parents=True)
            controller.write_bytes(b'old')
            deps.validate(root)
            controller.write_bytes(b'new')
            with self.assertRaisesRegex(RuntimeError, 'reset_pose_loader.py'):
                deps.validate(root)
            target = root / deps.RESET_POSE_LOADER
            target.write_bytes(LOADER.read_bytes())
            deps.validate(root)
            target.write_bytes(LOADER.read_bytes() + b'\n# unreviewed change\n')
            with self.assertRaises(RuntimeError):deps.validate(root)
            controller.write_bytes(b'unreviewed')
            with self.assertRaises(RuntimeError):deps.validate(root)

    def test_custom_pose_matches_robot_and_preserves_all_21_targets(self):
        joints = {name: float(i) for i, name in enumerate(loader.CUSTOM_POSE_JOINTS)}
        state = {'version': 1, 'robot_id': '300', 'mode': 'custom', 'custom': {'joints_deg': joints}}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'pose.json'
            path.write_text(json.dumps(state))
            targets = loader.load_quick_reset_targets('300', path)
            self.assertEqual(set(targets), {'leg_waist', 'left_arm', 'right_arm', 'neck'})
            self.assertEqual(targets['leg_waist'], [joints[n] for n in loader.LEG_JOINTS])
            self.assertEqual(targets['left_arm'], [joints[n] for n in loader.LEFT_ARM_JOINTS])
            self.assertEqual(targets['right_arm'], [joints[n] for n in loader.RIGHT_ARM_JOINTS])
            self.assertEqual(targets['neck'], [joints[n] for n in loader.NECK_JOINTS])
            with self.assertRaisesRegex(ValueError, 'different robot'):
                loader.load_quick_reset_targets('347', path)
            for bad in (True, float('nan'), '5'):
                joints[loader.LEFT_ARM_JOINTS[0]] = bad
                path.write_text(json.dumps(state))
                with self.assertRaises(ValueError):loader.load_quick_reset_targets('300', path)
            path.unlink()
            with self.assertRaises(OSError):loader.load_quick_reset_targets('300', path)

    def test_original_pose_extracts_body_arm_and_neck_offsets(self):
        config = {'LEFT_ARM_MOTOR_HOME_ANGLE': {'standing': list(range(11))},
                  'RIGHT_ARM_MOTOR_HOME_ANGLE': {'standing': list(range(20,31))},
                  'NECK_MOTOR_HOME_ANGLE': {'zero': list(range(40,47))}}
        state = {'version': 1, 'robot_id': '300', 'mode': 'original',
                 'original': {'configs': {'robot.json': config}}}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'pose.json'
            path.write_text(json.dumps(state))
            targets = loader.load_quick_reset_targets('300', path)
            self.assertEqual(targets, {'leg_waist': list(range(4)), 'left_arm': list(range(4,11)), 'right_arm': list(range(24,31)), 'neck': list(range(44,47))})


if __name__ == '__main__':
    unittest.main()
