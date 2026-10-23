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
    def test_reset_accepts_fresh_completion_without_observing_motion(self):
        source = ROOT / 'dagger/control/supervisor_node.py'
        wait = next(n for n in ast.walk(ast.parse(source.read_text()))
                    if isinstance(n, ast.FunctionDef) and n.name == '_wait_for_quick_reset')
        for state, ready, progress, expected in [
                ('ARMED', True, 1., True), ('HOLDING', True, 1., True),
                ('ARMED', True, .5, False), ('ARMED', False, 1., False),
                ('FAULT', True, 1., False), ('ESTOP', True, 1., False)]:
            with self.subTest(state=state, ready=ready, progress=progress):
                clock = SimpleNamespace(value=10.)
                node = SimpleNamespace(_lock=threading.RLock(),
                    get_parameter=lambda _: SimpleNamespace(value=.2),
                    _controller_status_received_ns=9_990_000_000,
                    _controller_status=dict(state='ARMED', hardware_ready=True,
                                           quick_reset_progress=1., quick_reset_active=False))
                def sleep(seconds):
                    clock.value += seconds
                    node._controller_status_received_ns = round(clock.value * 1e9)
                    node._controller_status.update(state=state, hardware_ready=ready,
                                                   quick_reset_progress=progress)
                scope = {'time': SimpleNamespace(monotonic=lambda:clock.value,
                         monotonic_ns=lambda:round(clock.value * 1e9), sleep=sleep)}
                exec(compile(ast.Module(body=[wait], type_ignores=[]), str(source), 'exec'), scope)
                self.assertEqual(scope['_wait_for_quick_reset'](node), expected)
                self.assertGreater(clock.value, 10.)  # Cached 100% must not complete this request.

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
    def run_wait(self, *, ready=True, cancel=False, stale=False, guard='', quiet_until=101.):
        now = [100.]
        node = SimpleNamespace(_lock=threading.RLock(), _closing=cancel, _cancel_start=False,
            _controller_status={'collector_heartbeat_ready': False}, _controller_status_received_ns=0)
        def sleep(_):
            now[0] += 1
            node._controller_status = {'collector_heartbeat_ready': ready,
                'collector_startup_guard_error': 'startup silence' if now[0] < quiet_until else guard}
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

    def test_heartbeat_alone_does_not_bypass_quiet_window(self):
        self.assertEqual(self.run_wait(quiet_until=104.), 104.)

    def test_unready_graph_remains_blocked_without_retrying_enable(self):
        with self.assertRaisesRegex(RuntimeError, 'DDS not ready'):
            self.run_wait(guard='DDS not ready')

    def test_real_controller_quiet_guard_is_reported_before_enable(self):
        source = ROOT / 'teleop_snapshot/openarmx_teleop_vr_306_v4/openarmx_teleop_vr_306_v4/controller_node.py'
        parsed = ast.parse(source.read_text())
        guard = next(n for n in ast.walk(parsed) if isinstance(n, ast.FunctionDef)
                     and n.name == '_active_command_guard_error')
        scope = {'np': SimpleNamespace(isfinite=lambda x: x == x and abs(x) != float('inf'))}
        exec(compile(ast.Module(body=[guard], type_ignores=[]), str(source), 'exec'), scope)
        node = SimpleNamespace(_command_monitor_started=100., _last_external_command_time=0.,
            _last_external_command_topic='', _own_command_echo_timeout=lambda:1.,
            get_parameter=lambda name:SimpleNamespace(value={
                'enable_active_command_arbitration':True, 'external_command_quiet_sec':1.,
                'require_sync_session_service':True, 'feedback_timeout_sec':.35}[name]),
            _hardware_output_guard_error=lambda **_: '', _sync_session_client=Mock(), _feedback=object())
        node._active_command_guard_error=lambda now:scope['_active_command_guard_error'](node,now)
        expression = next(value for n in ast.walk(parsed) if isinstance(n, ast.Dict)
                          for key,value in zip(n.keys,n.values)
                          if isinstance(key,ast.Constant) and key.value=='collector_startup_guard_error')
        evaluate=compile(ast.Expression(expression),str(source),'eval')
        # The readiness status uses exactly the controller's actual guard.
        self.assertIn('0.16s',eval(evaluate,{'self':node,'now':100.16,'feedback_age':0.}))
        self.assertIn('1.00s',node._active_command_guard_error(100.16))
        self.assertEqual(node._active_command_guard_error(101.01),'')
        self.assertEqual(eval(evaluate,{'self':node,'now':101.01,'feedback_age':0.}),'')
        self.assertIn('feedback',eval(evaluate,{'self':node,'now':101.01,'feedback_age':1.}))
        node._last_external_command_time=101.
        self.assertIn('external command on',node._active_command_guard_error(101.16))

    def test_stale_missing_and_cancelled_never_pass(self):
        for kwargs in ({'stale': True}, {'ready': False}, {'cancel': True}):
            with self.assertRaises(RuntimeError):self.run_wait(**kwargs)


if __name__ == '__main__':
    unittest.main()
