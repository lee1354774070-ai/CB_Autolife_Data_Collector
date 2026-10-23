"""Saved-pose reset orchestration; no hardware clients are created."""
import ast
from concurrent.futures import Future
import os
from pathlib import Path
import threading
import time
from types import SimpleNamespace, ModuleType
import unittest
from unittest.mock import Mock, patch

from dagger.run import environment

ROOT = Path(__file__).resolve().parents[1]
tree = ast.parse((ROOT / 'dagger/supervisor.py').read_text())
method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
              and n.name == '_mechanical_reset_worker')


class SavedResetTest(unittest.TestCase):
    def setUp(self):
        future = Future()
        future.set_result(SimpleNamespace(success=True, message='accepted'))
        self.node = SimpleNamespace(_lock=threading.RLock(), _reset_pending=False,
            _set_notice_locked=Mock(), _publish_state=Mock(),
            _forward_enable=Mock(return_value=(True, 'ready')),
            _wait_for_reset_heartbeat=Mock(),
            _wait_for_reset_enable=Mock(return_value=(True, 'ready')),
            _saved_reset=Mock(), _wait_for_quick_reset=Mock(return_value=True),
            _open_reset_grippers=Mock(return_value=(True, 'open')),
            _controller_status={'reason': 'target not reached'},
            get_parameter=lambda _:SimpleNamespace(value=.1))
        self.node._saved_reset.call_async.return_value = future
        self.loader = ModuleType('openarmx_teleop_vr_306_v4.reset_pose_loader')
        self.loader.load_quick_reset_targets = Mock()
        self.scope = {'__package__':'dagger', 'Path':Path, 'os':os, 'time':time,
                      'threading':threading, 'Trigger':SimpleNamespace(Request=lambda:None),
                      'resolve_robot':lambda _:SimpleNamespace(robot_id='300')}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(ROOT/'dagger/supervisor.py'), 'exec'), self.scope)

    def run_reset(self, validation_error=None):
        with patch.dict(os.environ, DAGGER_DEPENDENCY_ROOT='/reviewed', DAGGER_RESET_STATE_PATH='/run/reset.json'), \
                patch.dict('sys.modules', {self.loader.__name__:self.loader}), \
                patch('dagger.dependencies.validate_saved_reset', side_effect=validation_error):
            self.scope['_mechanical_reset_worker'](self.node, 'X')

    def test_complete_reset_waits_for_body_and_grippers_then_releases(self):
        self.run_reset()
        self.assertEqual([c.args[0] for c in self.node._forward_enable.call_args_list], [True, False])
        self.loader.load_quick_reset_targets.assert_called_once_with('300', '/run/reset.json')
        self.node._wait_for_quick_reset.assert_called_once()
        self.node._open_reset_grippers.assert_called_once()
        self.assertEqual(self.node._set_notice_locked.call_args.args[1], 'success')
        self.assertFalse(self.node._reset_pending)

    def test_missing_pose_rejected_before_claiming_control(self):
        self.run_reset(RuntimeError('unreviewed controller'))
        self.node._forward_enable.assert_not_called()
        self.node._saved_reset.call_async.assert_not_called()
        self.assertEqual(self.node._set_notice_locked.call_args.args[1], 'error')

    def test_unreached_target_never_reports_success(self):
        self.node._wait_for_quick_reset.return_value = False
        self.run_reset()
        self.node._open_reset_grippers.assert_not_called()
        self.assertIn('target not reached', self.node._set_notice_locked.call_args.args[0])
        self.assertEqual(self.node._forward_enable.call_args.args, (False,))

    def test_unknown_enable_still_releases_but_never_starts_trajectory(self):
        self.node._forward_enable.side_effect = [(False, 'unknown'), (True, 'disabled')]
        self.run_reset()
        self.node._saved_reset.call_async.assert_not_called()
        self.assertEqual(self.node._forward_enable.call_count, 2)

    def test_missing_controller_heartbeat_never_enables_hardware(self):
        self.node._wait_for_reset_heartbeat.side_effect = RuntimeError('heartbeat not ready')
        self.run_reset()
        self.node._forward_enable.assert_not_called()
        self.node._saved_reset.call_async.assert_not_called()
        self.assertIn('heartbeat not ready', self.node._set_notice_locked.call_args.args[0])

    def test_reset_environment_does_not_require_model(self):
        env = environment('reset', '', {'ROBOT_ID':'300'}, reset_only=True)
        self.assertNotIn('DAGGER_SERVER_URL', env)
        self.assertEqual(env['DAGGER_RESET_ONLY'], '1')
        with self.assertRaisesRegex(ValueError, 'DAGGER_SERVER_URL'):
            environment('record', '', {'ROBOT_ID':'300'})

    def test_launcher_reset_without_service_starts_only_reset_worker(self):
        source = ROOT/'gui/dagger_launcher.py'
        tree = ast.parse(source.read_text())
        command = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name=='command')
        scope = {}
        exec(compile(ast.Module(body=[command], type_ignores=[]), str(source), 'exec'), scope)
        node = SimpleNamespace(process=None, start=Mock(), stop=Mock())
        scope['command'](node, 'reset')
        node.start.assert_called_once_with(reset_only=True)


class ResetHeartbeatTest(unittest.TestCase):
    def run_wait(self, *, ready=True, cancel=False, stale=False):
        now = [100.]
        node = SimpleNamespace(_lock=threading.RLock(), _closing=cancel, _cancel_start=False,
            _controller_status={'collector_heartbeat_ready': False}, _controller_status_received_ns=0)
        def sleep(_):
            now[0] += 1
            node._controller_status = {'collector_heartbeat_ready': ready}
            node._controller_status_received_ns = 0 if stale else int(now[0]*1e9)
        method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
                      and n.name == '_wait_for_reset_heartbeat')
        scope = {'time':SimpleNamespace(monotonic=lambda:now[0],
                    monotonic_ns=lambda:int(now[0]*1e9), sleep=sleep)}
        exec(compile(ast.Module(body=[method], type_ignores=[]), '<heartbeat>', 'exec'), scope)
        scope['_wait_for_reset_heartbeat'](node)
        return now[0]

    def test_waits_until_controller_confirms_new_heartbeat(self):
        self.assertEqual(self.run_wait(), 101.)

    def test_stale_missing_and_cancelled_never_pass(self):
        for kwargs in ({'stale': True}, {'ready': False}, {'cancel': True}):
            with self.assertRaises(RuntimeError):self.run_wait(**kwargs)


if __name__ == '__main__':
    unittest.main()
