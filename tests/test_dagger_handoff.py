"""No-motion checks for the real V4 handoff hook and mapper wrappers."""

import ast
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np

from dagger.controller_handoff import apply_authority

ROOT = Path(__file__).resolve().parents[1] / 'dagger'


class ControllerHandoffTest(unittest.TestCase):
    def setUp(self):
        self.node = SimpleNamespace(
            _collector_authority_epoch=5, _collector_session_id='trial',
            _collector_revoked_epoch=-1, _follow_authority_mode='POLICY_ACTIVE',
            _follow_authority_time=time.monotonic(), _latest_target_mailbox=Mock(),
            _feedback=SimpleNamespace(as_dict=lambda: {'left_arm': np.zeros(7)},
                                      left_gripper=[30.], right_gripper=[40.]),
            _feedback_time=time.monotonic(), _reset_active=False,
            _enable_pending=False, _estop_latched=False,
            get_parameter=lambda name: SimpleNamespace(value=.5),
            _begin_fresh_teleop_session_locked=Mock())
        self.packet = {'session_id': 'trial', 'authority_epoch': 6}

    def test_replaces_policy_goals_without_io_or_hold_ack(self):
        self.assertTrue(apply_authority(self.node, self.packet, 'EXPERT_ACTIVE'))
        self.node._latest_target_mailbox.reset.assert_called_once()
        self.node._begin_fresh_teleop_session_locked.assert_called_once()
        self.assertEqual(self.node._collector_revoked_epoch, 5)
        self.assertEqual(self.node._collector_authority_epoch, 6)
        self.assertEqual(self.node._gripper_targets, {'left': 30., 'right': 40.})
        self.assertEqual(self.node._collector_body_origin_epoch, -1)
        # Ordinary heartbeat must not reset the human's moving target.
        self.node._follow_authority_mode = 'EXPERT_ACTIVE'
        self.assertTrue(apply_authority(self.node, self.packet, 'EXPERT_ACTIVE'))
        self.node._begin_fresh_teleop_session_locked.assert_called_once()

    def test_stale_state_never_restores_policy_authority(self):
        self.assertFalse(apply_authority(self.node, {**self.packet, 'authority_epoch': 4}, 'POLICY_ACTIVE'))
        self.assertEqual(self.node._collector_authority_epoch, 5)
        self.node._begin_fresh_teleop_session_locked.assert_not_called()

    def test_stop_state_itself_clears_targets_without_cross_topic_release_order(self):
        self.assertTrue(apply_authority(self.node, self.packet, 'DISARMED'))
        self.node._begin_fresh_teleop_session_locked.assert_called_once()

    def test_disarmed_does_not_rebase_an_active_reset(self):
        self.node._reset_active = True
        self.assertTrue(apply_authority(self.node, self.packet, 'DISARMED'))
        self.node._begin_fresh_teleop_session_locked.assert_not_called()

    def test_stale_feedback_remains_fenced_until_fresh_rebase(self):
        self.node._feedback_time -= 10
        for _ in range(2):
            self.assertFalse(apply_authority(self.node, self.packet, 'EXPERT_ACTIVE'))
            self.assertEqual(self.node._follow_authority_mode, '')
        self.node._begin_fresh_teleop_session_locked.assert_not_called()
        self.node._feedback_time = time.monotonic()
        self.assertTrue(apply_authority(self.node, self.packet, 'EXPERT_ACTIVE'))
        self.node._begin_fresh_teleop_session_locked.assert_called_once()

    def test_reset_and_estop_do_not_get_overridden(self):
        for flag in ('_reset_active', '_enable_pending', '_estop_latched'):
            setattr(self.node, flag, True)
            self.assertFalse(apply_authority(self.node, self.packet, 'EXPERT_ACTIVE'))
            self.node._begin_fresh_teleop_session_locked.assert_not_called()
            setattr(self.node, flag, False)


class MapperHandoffTest(unittest.TestCase):
    def setUp(self):
        # Execute the actual wrapper classes with a minimal vendor/ROS facade.
        # This isolates ownership logic, not IK or robot dynamics.
        class VendorMapper:
            def __init__(self):
                self._lock = threading.RLock()
                self._eef_target_pub = Mock()
                self._gripper_pub = Mock()
                self._release_pub = Mock()
                self._release_all = Mock()
                self.create_subscription = Mock()
                self.ticks = 0

            def _control_tick(self):
                self.ticks += 1

        path = ROOT / 'mapper.py'
        classes = [item for item in ast.parse(path.read_text()).body if isinstance(item, ast.ClassDef)]
        ns = dict(json=json, time=time, IndependentVrMapper=VendorMapper,
                  String=lambda **kw: SimpleNamespace(**kw),
                  QoSProfile=lambda **kw: kw, ReliabilityPolicy=SimpleNamespace(RELIABLE=1))
        exec(compile(ast.Module(body=classes, type_ignores=[]), str(path), 'exec'), ns)
        self.node = ns['CollectorVrMapper']()

    def authority(self, epoch, mode):
        self.node._on_authority(SimpleNamespace(data=json.dumps(
            {'session_id': 'trial', 'authority_epoch': epoch, 'mode': mode})))

    def test_policy_does_not_latch_expert_anchor(self):
        self.authority(2, 'POLICY_ACTIVE')
        self.node._control_tick()
        self.assertEqual(self.node.ticks, 0)


    def test_immediate_reanchor_and_epoch_tagged_packets(self):
        self.authority(2, 'POLICY_ACTIVE')
        self.authority(3, 'EXPERT_ACTIVE')
        self.node._release_all.assert_called_with(require_release=False, notify_controller=False)
        self.node._control_tick()
        self.assertEqual(self.node.ticks, 1)
        publisher = self.node._eef_target_pub
        publisher.publish(SimpleNamespace(data='{"pos_left_in_robot": [0,0,0]}'))
        packet = json.loads(publisher.publisher.publish.call_args.args[0].data)
        self.assertEqual(packet['authority_epoch'], 3)
        self.assertEqual(packet['collector_session_id'], 'trial')
        calls = self.node._release_all.call_count
        self.authority(3, 'EXPERT_ACTIVE')
        self.assertEqual(self.node._release_all.call_count, calls)
        self.authority(2, 'POLICY_ACTIVE')
        self.assertEqual(self.node._collector_authority[1], 3)

    def test_stale_authority_cannot_send_expert_targets(self):
        self.authority(3, 'EXPERT_ACTIVE')
        self.node._collector_authority_time -= 1
        for name in ('_eef_target_pub', '_gripper_pub', '_release_pub'):
            publisher = getattr(self.node, name)
            publisher.publish(SimpleNamespace(data='{}'))
            publisher.publisher.publish.assert_not_called()
        self.node._control_tick()
        self.assertEqual(self.node.ticks, 0)


class LateInferenceTest(unittest.TestCase):
    def test_completed_old_chunk_is_discarded_without_publishing(self):
        path = ROOT / 'thor_bridge.py'
        tree = ast.parse(path.read_text())
        names = {'_on_control_state', '_valid', '_submit'}
        methods = [method for cls in tree.body if isinstance(cls, ast.ClassDef)
                   for method in cls.body if isinstance(method, ast.FunctionDef) and method.name in names]
        cls = ast.ClassDef(name='Bridge', bases=[], keywords=[], body=methods, decorator_list=[])
        ns = dict(json=json, time=time, np=np, ProtocolError=ValueError,
                  array_digest=lambda _: 'digest', String=lambda **kw: SimpleNamespace(**kw))
        exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])), str(path), 'exec'), ns)
        node = ns['Bridge']()
        node._condition = threading.Condition()
        node._stopped = False
        node._identity = ('trial', 1, 'POLICY_ACTIVE')
        node._generation = 1
        node._state_received = time.monotonic()
        node._action_pub = Mock()
        node.get_parameter = lambda name: SimpleNamespace(value=30.)
        node._on_control_state(SimpleNamespace(data=json.dumps(
            dict(session_id='trial', authority_epoch=2, mode='EXPERT_ACTIVE'))))
        proposal = dict(executable_actions=np.zeros((3, 21)).tolist(), executable_chunk_digest='digest')
        contract = SimpleNamespace(action_dim=21, chunk_size=3, n_action_steps=3)
        prefix = node._submit(proposal, contract, 1, np.zeros(21), time.time_ns())
        self.assertEqual(prefix, [])
        node._action_pub.publish.assert_not_called()
