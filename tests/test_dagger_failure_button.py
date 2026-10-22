"""Run the production hold detector without ROS, transport or robot calls."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from dagger.control.core import finite_vector

SOURCE = Path(__file__).resolve().parents[1] / 'dagger/supervisor.py'
method = next(n for c in ast.parse(SOURCE.read_text()).body if isinstance(c, ast.ClassDef)
              for n in c.body if isinstance(n, ast.FunctionDef) and n.name == '_update_vr_failure_button_locked')
namespace = dict(finite_vector=finite_vector, Trigger=SimpleNamespace(Response=SimpleNamespace))
exec(compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), 'exec'), namespace)


class FailureButtonTest(unittest.TestCase):
    def setUp(self):
        self.now = 1_000_000_000
        self.node = SimpleNamespace(_intervention_id='trial', _closing=False,
            _button_worker_active=False, _session_start_pending=False, _reset_pending=False,
            _launcher_control_active=False, _launcher_wait_for_release=False,
            _pending_collector_result=None, _failure_button_start_ns=0,
            _failure_button_last_ns=0, _failure_button_released=False,
            _failure_button_trial='', _dispatch_operator=Mock())
        self.node._on_face_button_click_locked = Mock()

    def feed(self, y=False, *, tracked=True, faces=None, step=.1, stick=False):
        self.now += round(step * 1e9)
        packet = {key: dict(position=dict(x=0., y=0., z=1.) if tracked else None,
                            thumbstick=dict(x=0., y=0., pressed=int(stick)))
                  for key in ('leftController', 'rightController')}
        packet['leftController']['yButton'] = int(y)
        for side, fields in (faces or {}).items():
            packet[side].update(fields)
        namespace['_update_vr_failure_button_locked'](self.node, packet, self.now)

    def hold(self, **kwargs):
        for _ in range(11):
            self.feed(y=True, **kwargs)

    def test_single_hold_saves_once_and_requires_release(self):
        self.feed()
        self.hold()
        self.node._dispatch_operator.assert_called_once()
        self.assertEqual(self.node._dispatch_operator.call_args.args[0], 'fail')
        self.assertFalse(self.node._dispatch_operator.call_args.kwargs['launcher'])
        self.hold()
        self.assertEqual(self.node._dispatch_operator.call_count, 1)
        self.feed()
        self.assertEqual(self.node._dispatch_operator.call_count, 1, 'Release after fail discarded')
        self.hold()
        self.assertEqual(self.node._dispatch_operator.call_count, 2)

    def test_short_y_discards_only_on_release(self):
        self.feed()
        for _ in range(9):
            self.feed(y=True)
        self.node._dispatch_operator.assert_not_called()
        self.feed()
        self.node._dispatch_operator.assert_called_once()
        self.assertEqual(self.node._dispatch_operator.call_args.args[0], 'discard')

    def test_threshold_crossed_on_release_saves_failure(self):
        self.feed()
        for _ in range(10):
            self.feed(y=True)
        self.node._dispatch_operator.assert_not_called()
        self.feed(step=.1)
        self.node._dispatch_operator.assert_called_once()
        self.assertEqual(self.node._dispatch_operator.call_args.args[0], 'fail')

    def test_thumbstick_no_longer_saves_and_events_do_not_discard(self):
        self.feed()
        for _ in range(15):
            self.feed(stick=True)
        for pressed in (True, False):
            namespace['_update_vr_failure_button_locked'](self.node,
                dict(type='button_press' if pressed else 'button_release', button='Y', pressed=pressed), self.now)
        self.node._dispatch_operator.assert_not_called()

    def test_tracking_loss_and_transport_gap_require_fresh_release(self):
        self.feed()
        for _ in range(8):
            self.feed(y=True)
        self.feed(y=True, tracked=False)
        self.hold()
        self.node._dispatch_operator.assert_not_called()
        self.feed()
        for _ in range(8):
            self.feed(y=True)
        self.feed(y=True, step=.4)
        self.hold()
        self.node._dispatch_operator.assert_not_called()
        self.feed()
        self.hold()
        self.node._dispatch_operator.assert_called_once()

    def test_held_button_cannot_carry_into_a_new_trial(self):
        self.feed()
        for _ in range(8):
            self.feed(y=True)
        self.node._intervention_id = 'next-trial'
        self.hold()
        self.node._dispatch_operator.assert_not_called()
        self.feed()
        self.hold()
        self.node._dispatch_operator.assert_called_once()

    def test_busy_or_ambiguous_storage_cancels_hold(self):
        self.feed()
        for attr in ('_button_worker_active', '_session_start_pending', '_reset_pending',
                     '_launcher_control_active', '_launcher_wait_for_release', '_closing',
                     '_pending_collector_result'):
            with self.subTest(attr=attr):
                self.feed()
                setattr(self.node, attr, True)
                self.hold()
                setattr(self.node, attr, None if attr == '_pending_collector_result' else False)
                self.hold()
                self.node._dispatch_operator.assert_not_called()

    def test_other_face_buttons_cancel_y_until_release(self):
        self.feed()
        for _ in range(8):
            self.feed(y=True)
        self.feed(y=True, faces={'rightController': {'bButton': True}})
        self.hold()
        self.node._dispatch_operator.assert_not_called()

    def test_cancelled_release_and_keyboard_discard_are_separate(self):
        self.feed()
        self.feed(y=True)
        namespace['_update_vr_failure_button_locked'](self.node,
            dict(type='button_release', button='Y', cancelled=True), self.now)
        self.feed()
        self.node._dispatch_operator.assert_not_called()
        namespace['_update_vr_failure_button_locked'](self.node,
            dict(source='keyboard', type='button_press', button='Y', pressed=True), self.now)
        self.node._on_face_button_click_locked.assert_called_once_with('Y', self.now)
        self.node._dispatch_operator.assert_not_called()

    def test_launcher_displays_pending_saved_and_error_notices(self):
        source = SOURCE.parents[1] / 'gui/dagger_launcher.py'
        method = next(n for n in ast.parse(source.read_text()).body
                      if isinstance(n, ast.FunctionDef) and n.name == 'describe_state')
        scope = {}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), scope)
        for message, level in [('正在保存：任务失败', 'info'), ('本条已保存：任务失败', 'success'),
                               ('本条未保存，请检查录制错误', 'error')]:
            text, color = scope['describe_state'](dict(mode='DISARMED', notice=dict(text=message, level=level)))
            self.assertIn(message, text)
            self.assertEqual(color, '#ffb09a' if level == 'error' else '#8bd5ae')


if __name__ == '__main__':
    unittest.main()
