"""Execute the real height methods without ROS, IK, publishers or hardware."""
import ast
import json
import os
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import threading
import time

import numpy as np
from openarmx_teleop_vr_306_v4.body_height import (
    MAX_BODY_LOWERING_M, MAX_REVERSE_BODY_LOWERING_M, BODY_HEIGHT_JOINT_NAMES, BODY_HEIGHT_MAXIMUM_DEG,
    estimate_body_lowering, coordinated_body_height_targets, measured_reverse_squat,
)

SOURCE = Path(os.environ.get('HEIGHT_CONTROLLER_SOURCE',
    str(Path(__file__).parents[1] / 'openarmx_teleop_vr_306_v4/controller_node.py')))
tree = ast.parse(SOURCE.read_text())
controller = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                  and n.name == 'IndependentArmController')
names = {'_body_height_output_target_locked', '_on_body_height_command',
         '_deactivate_body_height_locked'}
namespace = dict(globals())
exec(compile(ast.Module(body=[n for n in controller.body
    if isinstance(n, ast.FunctionDef) and n.name in names], type_ignores=[]),
    str(SOURCE), 'exec'), namespace)

class Harness:
    def __init__(self):
        self._lock = threading.RLock()
        self.params = dict(body_height_command_timeout_sec=.25,
            body_height_hold_command_lead_deg=2., body_height_motion_command_lead_deg=6.,
            body_height_command_lead_slew_deg_sec=60., body_height_lowering_rate_m_sec=.15,
            feedback_timeout_sec=.35)
        self.get_parameter = lambda k: SimpleNamespace(value=self.params[k])
        self.measured = np.array([10.99, 22.44, 11.30, 0.])
        self._feedback = SimpleNamespace(as_dict=lambda: {'leg_waist': self.measured.copy()})
        self._feedback_time = 10.
        self._hardware_enabled = True
        self._body_height_control_enabled = True
        self._reset_active = self._enable_pending = self._estop_latched = False
        self._body_height_active = True
        self._body_height_watchdog_stopped = False
        self._body_height_joint_targets = np.array([12.34, 25.17, 13.15])
        self._body_height_command_time = 9.5
        self._body_height_update_time = 9.5
        self._body_height_command_progress = .162
        self._body_height_lowering_m = .104
        self._body_height_last_direction = 1.
        self._body_height_command_lead_deg = 6.
        self._waist_follow_enabled = True
        self._waist_follow_profile = 'forward_pitch_only'
        self._waist_follow_neutral_pitch_deg = 8.38
        self._waist_follow_pitch_target_deg = 8.38
        self._target_groups = {'leg_waist': np.array([12.34, 25.17, 13.15, 0.])}
        self._kinematics = SimpleNamespace(upper=np.deg2rad([76.,155.,81.]), lower=np.deg2rad([-83.,-153.,-78.]),
            joint_q=dict(zip(BODY_HEIGHT_JOINT_NAMES, range(3))))
        self._configuration_guard_error = lambda *a, **k: ('', None)
        self._target_velocity_estimator = SimpleNamespace(
            reset_indices=lambda *a: None, velocity=np.zeros(21))

for name in names:
    setattr(Harness, name, namespace[name])

def test_half_second_gap_holds_all_three_not_old_waist():
    c = Harness()
    held = c._body_height_output_target_locked(10., c._feedback)
    np.testing.assert_allclose(held, [10.99,22.44,11.30])
    assert c._waist_follow_neutral_pitch_deg == 11.30
    assert c._body_height_watchdog_stopped
    assert c._body_height_last_direction == 0

def test_timeout_hold_is_latched_not_chasing_each_new_feedback():
    c = Harness()
    held = c._body_height_output_target_locked(10., c._feedback)
    c.measured += .3
    np.testing.assert_allclose(c._body_height_output_target_locked(10.1, c._feedback), held)

def test_new_height_command_resumes_from_stopped_progress():
    c = Harness()
    held = c._body_height_output_target_locked(10., c._feedback)
    with patch.object(time, 'monotonic', return_value=10.02):
        c._on_body_height_command(SimpleNamespace(data=json.dumps({'enabled':True, 'direction':1})))
    assert not c._body_height_watchdog_stopped
    assert abs(c._body_height_lowering_m - (estimate_body_lowering((*held,0))+.003)) < 1e-8
    assert c._waist_follow_neutral_pitch_deg == c._body_height_joint_targets[2]

def test_fresh_height_overrides_stale_ik_shared_target():
    c = Harness()
    c._body_height_command_time = 10.
    c._target_groups['leg_waist'][:3] = 0.
    np.testing.assert_allclose(c._body_height_output_target_locked(10.01,c._feedback),
                               c._body_height_joint_targets)

def test_fresh_height_keeps_forward_assist_separate():
    c = Harness()
    c._body_height_command_time = 10.
    c._waist_follow_pitch_target_deg += 3.
    expected = c._body_height_joint_targets.copy()
    expected[2] += 3.
    np.testing.assert_allclose(c._body_height_output_target_locked(10.01,c._feedback), expected)

def test_release_key_holds_actual_pose_and_updates_baseline():
    c = Harness()
    with patch.object(time,'monotonic',return_value=10.):
        c._on_body_height_command(SimpleNamespace(data='{"enabled":true,"direction":0}'))
    np.testing.assert_allclose(c._body_height_joint_targets,c.measured[:3])
    assert c._waist_follow_neutral_pitch_deg == c.measured[2]

def test_release_takeover_does_not_chase_old_waist_target():
    c = Harness()
    c._on_body_height_command(SimpleNamespace(data='{"enabled":false,"direction":0}'))
    assert not c._body_height_active
    np.testing.assert_allclose(c._target_groups['leg_waist'][:3],c.measured[:3])
    assert c._waist_follow_neutral_pitch_deg == c.measured[2]

def test_reset_keeps_priority_over_height():
    c = Harness()
    c._reset_active = True
    assert c._body_height_output_target_locked(10.,c._feedback) is None

def test_unavailable_feedback_rejects_new_height():
    c = Harness()
    before = c._body_height_joint_targets.copy()
    c._feedback_time = 0.
    with patch.object(time,'monotonic',return_value=10.):
        c._on_body_height_command(SimpleNamespace(data='{"enabled":true,"direction":1}'))
    np.testing.assert_allclose(c._body_height_joint_targets,before)
    assert 'stale' in c._reason
