"""Safety/receipt regressions for local execution; no ROS or motor I/O."""
import importlib.util
import ast
import json
import os
from pathlib import Path
import unittest
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np

from dagger.local_policy_execution import (PolicyMotion, binding, identity,
                                         matches, project_configuration)


def limiter_type():
    source = Path(os.environ.get('DAGGER_V5_SOURCE_ROOT', '/home/ubuntu/openarmx_teleop_vr_v5'))
    p = source / 'openarmx_teleop_vr_306_v5/trajectory_limiter.py'
    spec = importlib.util.spec_from_file_location('local_test_trajectory_limiter', p)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.TrajectoryLimiter


class LocalPolicyTests(unittest.TestCase):
    def packet(self):
        return dict(collector_session_id='s', authority_epoch=4, proposal_id='p:2',
                    chunk_step=0, bridge_generation=3, action=[0.] * 21)

    def test_known_shoulder_overshoot_is_bounded_without_mutating_request(self):
        raw = np.deg2rad([10., -20.])
        saved = raw.copy()
        result, delta = project_configuration(raw, np.deg2rad([-178., -148.]),
                                              np.deg2rad([9., -2.]))
        np.testing.assert_array_equal(raw, saved)
        np.testing.assert_allclose(np.rad2deg(result), [9., -20.])
        np.testing.assert_allclose(delta, [1., 0.])

    def test_large_invalid_goal_is_rejected_instead_of_saturated(self):
        with self.assertRaisesRegex(ValueError, 'tolerance'):
            project_configuration(np.deg2rad([15.]), np.deg2rad([-178.]), np.deg2rad([9.]))

    def test_bound_receipt_cannot_ack_different_action_epoch_or_session(self):
        p = self.packet()
        b = binding(p)
        r = dict(b, accepted=True, receipt_scope='controller_submission', physical_execution_confirmed=False)
        self.assertTrue(matches(r, b))
        for key, value in [('authority_epoch', 3), ('collector_session_id', 'other'),
                           ('chunk_step', 1), ('action_digest', 'different')]:
            self.assertFalse(matches(dict(r, **{key: value}), b))
        self.assertFalse(matches(dict(r, physical_execution_confirmed=True), b))
        changed = dict(p, action=[1.] + [0.] * 20)
        self.assertEqual(identity(binding(changed)), identity(b))
        self.assertNotEqual(binding(changed)['action_digest'], b['action_digest'])

    def test_new_chunk_jump_moves_continuously_without_waiting_for_unsent_goal(self):
        motion = PolicyMotion(limiter_type(), np.full(21, -150.), np.full(21, 150.))
        q = np.zeros(21)
        goal = q.copy()
        goal[14] = 24.  # More than the previous bridge's 18-degree gate.
        previous = q.copy()
        previous_velocity = np.zeros(21)
        for tick in range(200):
            command = motion.step(goal, q, .01, 1, tick * .01)
            velocity = (command - previous) / .01
            self.assertLessEqual(abs(velocity[14]), 80. + 1e-5)
            self.assertLessEqual(abs(velocity[14] - previous_velocity[14]) / .01, 300. + 1e-5)
            self.assertLessEqual(abs(command[14] - q[14]), 12. + 1e-5)
            q = command.copy()
            previous, previous_velocity = command.copy(), velocity
        self.assertAlmostEqual(q[14], 24., places=3)

    def test_frozen_actuator_still_faults(self):
        motion = PolicyMotion(limiter_type(), np.full(21, -150.), np.full(21, 150.))
        q = np.zeros(21)
        goal = q.copy()
        goal[14] = 60.
        with self.assertRaisesRegex(RuntimeError, 'stalled'):
            for tick in range(500):
                motion.step(goal, q, .01, 1, tick * .01)

    def test_epoch_change_discards_previous_motion_history(self):
        motion = PolicyMotion(limiter_type(), np.full(21, -150.), np.full(21, 150.))
        q = np.zeros(21)
        goal = np.full(21, 20.)
        for tick in range(10):
            q = motion.step(goal, q, .01, 1, tick * .01)
        measured = np.full(21, -10.)
        command = motion.step(measured, measured, .01, 2, 1.)
        np.testing.assert_allclose(command, measured)


def bridge_for_test():
    # Execute the production methods without constructing ROS or a worker.
    tree = ast.parse((Path(__file__).resolve().parents[1] / 'dagger/thor_bridge.py').read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    methods = {'_on_control_state', '_on_forward_ack', '_await_local_receipt', '_query_local_receipt',
               '_wait_forward_ack', '_recover_local_receipt', '_close_session',
               '_resolve_proposal', '_policy_payload'}
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in methods]
    cls.bases = [ast.Name(id='Base', ctx=ast.Load())]
    class Base:
        def _on_control_state(self, message):
            pass
        def _close_session(self):
            self.closed = True
        def _policy_payload(self, action, proposal, step, generation, wall, state, **kw):
            return dict(action=action, proposal_id=proposal['proposal_id'],
                        chunk_step=step, bridge_generation=generation)
    ns = dict(__package__='dagger', Base=Base, json=json, time=time,
              String=lambda **kw: SimpleNamespace(**kw),
              array_digest_float32=__import__('dagger.control.groot_bridge_core', fromlist=['array_digest_float32']).array_digest_float32)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])), '<production_bridge>', 'exec'), ns)
    b = ns[cls.name]()
    b._local_execution = True
    b._condition = threading.Condition()
    b._local_answers = {}
    b._collector_session_id, b._authority_epoch, b._generation = 's', 4, 3
    b._generation_valid = Mock(return_value=True)
    b._proposal_authority = (3, 's', 4)
    b._local_capability = dict(version=1, timestamp_ns=time.time_ns())
    b._local_pending = None
    b._v5_policy_pacing = True
    b._wait_policy_tracking = Mock()
    b.get_logger = Mock(return_value=Mock())
    b._proposal = {'proposal_id': 'p:2', 'context_digest': 'context', 'executable_chunk_digest': 'chunk'}
    b._session_id = 'remote'
    b._executed = []
    b._post = Mock()
    b._policy_payload([0.] * 21, b._proposal, 0, 3, 0, [0.] * 21, shadow_only=False)
    return b


class LocalReceiptTests(unittest.TestCase):
    def test_local_receipt_path_also_checks_tracking(self):
        b = bridge_for_test()
        b._wait_policy_tracking.assert_called_once_with([0.] * 21, 3)

    def test_controller_capability_is_read_from_the_actual_compatibility_envelope(self):
        b = bridge_for_test()
        capability = dict(version=1, timestamp_ns=time.time_ns())
        b._on_control_state(SimpleNamespace(data=json.dumps(dict(session_id='s',
            authority_epoch=4, collector_compat=dict(local_policy_execution=capability)))))
        self.assertEqual(b._local_capability, capability)

    def receipt(self, b, **kw):
        return dict(b._local_pending['binding'], accepted=True,
                    receipt_scope='controller_submission', physical_execution_confirmed=False, **kw)

    def test_forward_only_ack_does_not_count_as_controller_acceptance(self):
        b = bridge_for_test()
        r = self.receipt(b)
        r.pop('action_digest')
        b._on_forward_ack(SimpleNamespace(data=json.dumps(r)))
        self.assertEqual(b._local_answers, {})
        r = self.receipt(b)
        b._on_forward_ack(SimpleNamespace(data=json.dumps(r)))
        self.assertTrue(b._wait_forward_ack('p:2', 0, 3))

    def test_negative_receipt_reports_reason_without_accepting_prefix(self):
        b = bridge_for_test()
        r = self.receipt(b)
        r.update(accepted=False, reason='right shoulder invalid')
        b._on_forward_ack(SimpleNamespace(data=json.dumps(r)))
        with self.assertRaisesRegex(RuntimeError, 'right shoulder invalid'):
            b._wait_forward_ack('p:2', 0, 3)
        self.assertEqual(b._executed, [])

    def test_lost_reply_is_queried_without_republishing_motion(self):
        b = bridge_for_test()
        r = self.receipt(b)
        b._await_local_receipt = Mock(side_effect=[None, r])
        b._local_query_pub = Mock()
        self.assertTrue(b._wait_forward_ack('p:2', 0, 3))
        query = json.loads(b._local_query_pub.publish.call_args.args[0].data)
        self.assertTrue(query['query_only'])
        self.assertEqual(query['action'], [0.] * 21)

    def test_unknown_outcome_preserves_proposal_and_remote_session(self):
        b = bridge_for_test()
        b._await_local_receipt = Mock(return_value=None)
        b._local_query_pub = Mock()
        with self.assertRaisesRegex(RuntimeError, 'outcome unknown'):
            b._wait_forward_ack('p:2', 0, 3)
        with self.assertRaisesRegex(RuntimeError, 'proposal not retired'):
            b._resolve_proposal(cancel=True)
        b._close_session()
        self.assertFalse(getattr(b, 'closed', False))
        b._post.assert_not_called()

    def test_recovered_acceptance_retires_only_exact_requested_prefix(self):
        b = bridge_for_test()
        b._query_local_receipt = Mock(return_value=self.receipt(b))
        b._resolve_proposal(cancel=True)
        self.assertIsNone(b._proposal)
        body = b._post.call_args.args[1]
        self.assertEqual(body['submitted_prefix'], [[0.] * 21])

    def test_missing_controller_capability_fails_before_any_submission(self):
        b = bridge_for_test()
        b._local_pending = None
        b._local_capability = None
        with self.assertRaisesRegex(RuntimeError, 'capability'):
            b._policy_payload([0.] * 21, b._proposal, 0, 3, 0, [], shadow_only=False)
        self.assertIsNone(b._local_pending)


if __name__ == '__main__':
    unittest.main()
