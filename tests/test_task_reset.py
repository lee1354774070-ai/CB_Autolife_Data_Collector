import json
import importlib.util
import os
import subprocess
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from dagger.task_reset import task_pose, pose_choices, reset_grippers
from teleop_snapshot.control_center_reset_pose.reset_pose_loader import CUSTOM_POSE_JOINTS


class TaskResetTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.joints = dict.fromkeys(CUSTOM_POSE_JOINTS, 1.)
        self.library = self.root / 'poses.yaml'
        self.library.write_text(json.dumps({'poses': {'table': {'joints_deg': self.joints}}}))
        self.plan = self.root / 'plan.json'
        self.plan.write_text(json.dumps({'poses_file': 'poses.yaml', 'task_poses': {'4': 'table'}}))
        self.env = {'ROBOT_ID': '300', 'DAGGER_RESET_PLAN': str(self.plan),
                    'TASK_TEXT': 'Pick the laundry bag from the table.'}

    def test_exact_task_and_independent_selection(self):
        self.assertEqual(task_pose(self.env)['pose_name'], 'table')
        self.assertEqual(pose_choices(self.env), ['table'])
        self.env.update(TASK_TEXT='custom task', DAGGER_RESET_POSE='table')
        self.assertEqual(task_pose(self.env)['custom']['joints_deg'], self.joints)

    def test_unknown_prompt_does_not_guess(self):
        self.env['TASK_TEXT'] = 'Pick the laundry bag.'
        with self.assertRaisesRegex(ValueError, '没有复位姿态'):
            task_pose(self.env)

    def test_task_grippers_are_frozen_and_laundry_targets_open(self):
        self.joints.update(Joint_Left_Gripper=340., Joint_Right_Gripper=180.)
        self.library.write_text(json.dumps({'poses': {name: {'joints_deg': self.joints}
            for name in ('table', 'laundry_down', 'laundry_down_safe', 'laundry_down_1')}}))
        self.assertEqual(task_pose(self.env)['gripper_targets_deg'], [340., 180.])
        for name in ('laundry_down', 'laundry_down_safe', 'laundry_down_1'):
            self.assertEqual(task_pose({**self.env, 'DAGGER_RESET_POSE': name})['gripper_targets_deg'], [10., 10.])
        for bad in ([0., 10.], [361., 10.], [True, 10.], [float('nan'), 10.], [10.]):
            with self.assertRaises(ValueError): reset_grippers({'gripper_targets_deg': bad})

    def test_other_robot_and_unknown_pose_rejected(self):
        with self.assertRaisesRegex(ValueError, '不属于'):
            task_pose({**self.env, 'ROBOT_ID': '347'})
        with self.assertRaisesRegex(ValueError, '不存在'):
            task_pose({**self.env, 'DAGGER_RESET_POSE': 'absent'})

    def test_missing_or_nonfinite_joint_rejected(self):
        for value in (None, True, float('nan')):
            self.joints['Joint_Knee'] = value
            self.library.write_text(json.dumps({'poses': {'table': {'joints_deg': self.joints}}}))
            with self.assertRaisesRegex(ValueError, '有效关节'):
                task_pose(self.env)

    def test_runtime_loader_uses_frozen_task_not_global_pose(self):
        root = Path(__file__).resolve().parents[1]
        loader = self.root / 'openarmx_teleop_vr_306_v4/reset_pose_loader.py'
        loader.parent.mkdir()
        loader.write_bytes((root / 'teleop_snapshot/control_center_reset_pose/reset_pose_loader.py').read_bytes())
        subprocess.run(['patch', '--batch', '--fuzz=0', '-p1', '-i', str(root / 'dagger/task_reset_loader.patch')],
                       cwd=self.root, check=True, capture_output=True)
        spec = importlib.util.spec_from_file_location('task_loader_test', loader)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        state = self.root / 'snapshot.json'
        state.write_text(json.dumps(task_pose(self.env)))
        with patch.dict(os.environ, {'DAGGER_RESET_STATE_PATH': str(state)}, clear=True):
            self.assertEqual(module.load_quick_reset_targets('300')['left_arm'], [1.] * 7)
        with patch.dict(os.environ, {}, clear=True):
            with self.assertRaisesRegex(ValueError, 'snapshot is missing'):
                module.load_quick_reset_targets('300')


if __name__ == '__main__':
    unittest.main()
