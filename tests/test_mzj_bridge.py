"""Execute the pinned MZJ bridge plus adapter without ROS/HTTP/motor objects."""
import ast
import __future__
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from dagger.mzj_base.groot_bridge_core import (
    INFERENCE_MODES, array_digest_float32, bridge_generation_boundary,
    policy_state_from_q23, validated_actions)

ROOT = Path(__file__).resolve().parents[1] / 'dagger'


def bridge_type():
    names = {'_on_control_state', '_generation_valid', '_execute', '_policy_payload',
             '_on_forward_ack', '_close_session', '_track_response_proposal',
             '_check_inference_latency', '_next_response'}
    tree = ast.parse((ROOT / 'mzj_base/groot_policy_bridge.py').read_text())
    cls = next(c for c in tree.body if isinstance(c, ast.ClassDef) and c.name == 'GrootPolicyBridge')
    cls.bases = []
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    adapter = next(c for c in ast.parse((ROOT / 'thor_bridge.py').read_text()).body if isinstance(c, ast.ClassDef))
    adapter.body = [n for n in adapter.body if isinstance(n, ast.FunctionDef) and n.name != '__init__']
    ns = dict(json=json, time=time, INFERENCE_MODES=INFERENCE_MODES,
              bridge_generation_boundary=bridge_generation_boundary,
              validated_actions=validated_actions, array_digest_float32=array_digest_float32,
              policy_state_from_q23=policy_state_from_q23,
              String=lambda **kw: SimpleNamespace(**kw), compact=json.dumps)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls, adapter], type_ignores=[])),
                 'mzj_bridge_callbacks', 'exec', flags=__future__.annotations.compiler_flag), ns)
    return ns['CollectorThorBridge']


class MzjBridgeTest(unittest.TestCase):
    def setUp(self):
        self.node = node = bridge_type()()
        node._condition = threading.Condition()
        node._shutdown = False
        node._mode, node._authority_epoch, node._generation = 'POLICY_ACTIVE', 1, 1
        node._collector_session_id, node._control_received = 'trial', time.monotonic()
        node._proposal_authority = (1, 'trial', 1)
        node._session_id, node._policy_sequence = 'remote', 0
        node._contract = SimpleNamespace(n_action_steps=3)
        node._policy_pub = Mock()
        node._fresh_q23 = lambda: [0.] * 18 + [10., 10.] + [0.] * 3
        node._post = Mock(return_value={})
        node._set_phase, node.get_logger = Mock(), Mock()
        node._wait_forward_ack = Mock(return_value=True)
        node.get_parameter = lambda key: SimpleNamespace(value={'action_rate_hz': 100.,
            'task': 'Pick the laundry bag.', 'max_inference_latency_sec': 2.}[key])
        node._forward_acks = set()
        node._executed = []
        self.actions = [[float(i)] * 14 + [360., 360.] + [0.] * 5 for i in range(3)]
        self.proposal = dict(proposal_id='p1', context_digest='context', executable_actions=self.actions,
                            executable_chunk_digest=array_digest_float32(self.actions))
        node._proposal = self.proposal

    def state(self, mode, epoch=2, session='trial'):
        self.node._on_control_state(SimpleNamespace(data=json.dumps(
            dict(session_id=session, authority_epoch=epoch, mode=mode))))

    def execute(self):
        return self.node._execute(self.proposal, 1, time.time_ns(), [0.] * 21)

    def test_late_inference_is_discarded_after_takeover_without_publish(self):
        self.state('EXPERT_ACTIVE')
        self.assertFalse(self.execute())
        self.node._policy_pub.publish.assert_not_called()
        self.node._post.assert_called_once_with('/discard', {'session_id': 'remote', 'proposal_id': 'p1'})

    def test_360_prefix_and_digest_remain_identical(self):
        self.assertTrue(self.execute())
        endpoint, receipt = self.node._post.call_args.args
        self.assertEqual(endpoint, '/controller_ack')
        self.assertEqual(receipt['submitted_prefix'], self.actions)
        self.assertEqual(receipt['submitted_prefix_digest'], self.proposal['executable_chunk_digest'])
        self.assertEqual(receipt['receipt_scope'], 'controller_submission')
        for index, call in enumerate(self.node._policy_pub.publish.call_args_list):
            packet = json.loads(call.args[0].data)
            self.assertEqual(packet['action'], self.actions[index])
            self.assertEqual(packet['collector_session_id'], 'trial')
            self.assertEqual(packet['authority_epoch'], 1)

    def test_partial_forward_receipt_only_counts_accepted_prefix(self):
        def acknowledge(*args):
            self.state('EXPERT_ACTIVE')
            return True  # First step was forwarded before the grip edge.
        self.node._wait_forward_ack.side_effect = acknowledge
        self.assertFalse(self.execute())
        endpoint, receipt = self.node._post.call_args.args
        self.assertEqual(endpoint, '/controller_ack')
        self.assertEqual(receipt['submitted_prefix'], self.actions[:1])
        self.assertEqual(receipt['submitted_prefix_digest'], array_digest_float32(self.actions[:1]))
        self.assertEqual(self.node._policy_pub.publish.call_count, 1)

    def test_stale_heartbeat_and_new_trial_revoke_old_chunk(self):
        self.node._control_received -= 1
        self.assertFalse(self.node._generation_valid(1))
        self.state('POLICY_ACTIVE', session='new-trial')
        self.assertFalse(self.node._generation_valid(1))
        self.assertEqual(self.node._proposal_authority, (1, 'trial', 1))
        self.assertFalse(self.execute())
        self.node._policy_pub.publish.assert_not_called()

    def test_rejected_ack_does_not_claim_submission(self):
        packet = dict(proposal_id='p1', chunk_step=0, bridge_generation=1, accepted=False)
        self.node._on_forward_ack(SimpleNamespace(data=json.dumps(packet)))
        self.assertEqual(self.node._forward_acks, set())
        packet.pop('accepted')  # MZJ positive-ACK format stays compatible.
        self.node._on_forward_ack(SimpleNamespace(data=json.dumps(packet)))
        self.assertEqual(self.node._forward_acks, {('p1', 0, 1)})

    def test_failed_receipt_keeps_proposal_for_retry(self):
        self.node._post.side_effect = RuntimeError('HTTP timeout')
        with self.assertRaises(RuntimeError):
            self.node._resolve_proposal(cancel=True)
        self.assertIs(self.node._proposal, self.proposal)
        self.node._post.side_effect = None
        self.node._resolve_proposal(cancel=True)
        self.node._close_session()
        self.assertIsNone(self.node._proposal)
        self.assertEqual(self.node._session_id, '')

    def test_slow_start_tracks_proposal_before_latency_rejection(self):
        self.node._session_id = ''
        self.node._proposal = None
        self.node._post.return_value = dict(session_id='remote', decision='execute',
                                           proposal=self.proposal, bridge_round_trip_ms=3000.)
        with self.assertRaisesRegex(RuntimeError, 'latency'):
            self.node._next_response({})
        self.assertIs(self.node._proposal, self.proposal)
        self.node._resolve_proposal(cancel=True)
        self.assertEqual(self.node._post.call_args.args[0], '/discard')

    def test_bad_digest_does_not_publish(self):
        self.proposal['executable_chunk_digest'] = 'bad'
        with self.assertRaises(RuntimeError):
            self.execute()
        self.node._policy_pub.publish.assert_not_called()
