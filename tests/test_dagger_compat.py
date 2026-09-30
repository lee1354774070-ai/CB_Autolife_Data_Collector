"""Attachment must not create a competing robot owner or replay a command."""

import ast
import json
from pathlib import Path
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import uuid

from dagger.compat import CommandGate, CompatibilityPublisher, FEATURES, SERVICE, check_host
from dagger.desktop_copy import render


class CompatibilityTest(unittest.TestCase):
    def setUp(self):
        self.gate = CommandGate()
        self.raw = dict(instance_id=self.gate.instance_id, request_id=uuid.uuid4().hex,
                        command='start', issued_ns=time.time_ns())
        self.execute = Mock(return_value=(True, 'queued'))

    def dispatch(self, **changes):
        return self.gate.dispatch(json.dumps({**self.raw, **changes}), self.execute)

    def test_duplicate_is_not_executed_twice(self):
        self.assertEqual(self.dispatch(), (True, 'queued'))
        self.assertEqual(self.dispatch(), (True, 'queued'))
        self.execute.assert_called_once_with('start')
        self.assertFalse(self.dispatch(command='reset')[0])
        self.execute.assert_called_once()

    def test_restarted_host_rejects_inflight_command(self):
        self.assertFalse(self.dispatch(instance_id=uuid.uuid4().hex)[0])
        self.execute.assert_not_called()

    def test_old_future_or_invalid_commands_never_run(self):
        for changes in ({'issued_ns': 0}, {'issued_ns': time.time_ns() + 10**12},
                        {'command': 'quit'}, {'request_id': 'invalid'}, {'issued_ns': True}):
            self.assertFalse(self.dispatch(**changes)[0])
        self.execute.assert_not_called()

    def test_uncertain_callback_does_not_replay(self):
        self.execute.side_effect = RuntimeError('after command submission')
        self.assertFalse(self.dispatch()[0])
        self.assertFalse(self.dispatch()[0])
        self.execute.assert_called_once()

    def test_cache_does_not_evict_still_replayable_requests(self):
        for _ in range(256):
            self.assertTrue(self.dispatch(request_id=uuid.uuid4().hex)[0])
        self.assertFalse(self.dispatch()[0])
        self.assertEqual(self.execute.call_count, 256)

    def test_legacy_wrong_task_or_missing_capabilities_rejected(self):
        contract = dict(version=1, instance_id=self.gate.instance_id, robot_id='300', domain_id='0',
                        command_service=SERVICE, features=FEATURES, task_text='task', with_depth=False,
                        hardware_publish=False)
        state = dict(collector_compat=contract, collector_fifo='/tmp/task/.official_recording_control')
        fifo = state['collector_fifo']
        self.assertEqual(check_host(state, fifo=fifo), contract)
        for options in ({'fifo': '/tmp/other'}, {'fifo': fifo, 'task_text': 'other'},
                        {'fifo': fifo, 'publish': True}, {'fifo': fifo, 'depth': True},
                        {'fifo': fifo, 'domain': '211'}):
            with self.assertRaises(ValueError):
                check_host(state, **options)
        for invalid in ({}, {'collector_compat': {}},
                        {**state, 'collector_compat': {**contract, 'features': []}}):
            with self.assertRaises(ValueError):
                check_host(invalid, fifo=fifo)

    def test_capability_is_added_without_changing_authority(self):
        output = Mock()
        publisher = CompatibilityPublisher(output, {'version': 1})
        publisher.publish(SimpleNamespace(data='{"mode":"EXPERT_ACTIVE","authority_epoch":9}'))
        packet = json.loads(output.publish.call_args.args[0].data)
        self.assertEqual(packet['authority_epoch'], 9)
        self.assertEqual(packet['collector_compat'], {'version': 1})

    def test_attach_branch_does_not_fall_through_to_owned_startup(self):
        source = Path(__file__).resolve().parents[1] / 'dagger/run.py'
        main = next(node for node in ast.parse(source.read_text()).body
                    if isinstance(node, ast.FunctionDef) and node.name == 'main')
        branch = next(node for node in main.body if isinstance(node, ast.If)
                      and ast.unparse(node.test) == "backend == 'attach'")
        self.assertIsInstance(branch.body[-1], ast.Raise)
        self.assertNotIn('prepare_copy', ast.unparse(branch))
        self.assertNotIn('Popen', ast.unparse(branch))
        self.assertIn('dagger/attach.py', ast.unparse(branch))


class DesktopCopyTest(unittest.TestCase):
    def test_unreviewed_source_never_executes(self):
        with self.assertRaises(ValueError):
            render(b'raise RuntimeError("must never run")', Path('/tools'))

    def test_ast_rewrites_start_and_dataset_filter_in_copy_only(self):
        source = b"""
DATA_ROOT = Path('/legacy')
class Launcher:
 def start(self):
  self.process = subprocess.Popen(['bash', str(BASE/'scripts/quickstart_dagger.sh'), task, instruction], env={})
 def on_state(self):
  expected = DATA_ROOT / self.task / ('hg_dagger_' + self.variant) / '.official_recording_control'
 def command(self, action):
  pass
"""
        import hashlib
        with patch('dagger.desktop_copy.REVIEWED', {hashlib.sha256(source).hexdigest()}):
            output = render(source, Path('/tools'))
        compile(output, '<test-copy>', 'exec')
        self.assertIn("'--serve'", output)
        self.assertIn('/tools/lerobot_data_collector/dagger/run.py', output)
        self.assertNotIn('hg_dagger_', output)
        self.assertIn('return self.stop()', output)
        self.assertIn("'DAGGER_BACKEND': 'owned'", output)
