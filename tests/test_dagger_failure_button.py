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

    def feed(self, right=False, left=False, *, tracked=True, faces=None, step=.1, push=False):
        self.now += round(step * 1e9)
        packet = {key: dict(position=dict(x=0., y=0., z=1.) if tracked else None,
                            thumbstick=dict(x=1. if push else 0., y=0., pressed=int(pressed)))
                  for key, pressed in (('leftController', left), ('rightController', right))}
        for side, fields in (faces or {}).items():
            packet[side].update(fields)
        namespace['_update_vr_failure_button_locked'](self.node, packet, self.now)

    def hold(self, **kwargs):
        for _ in range(11):
            self.feed(right=True, **kwargs)

    def test_single_hold_saves_once_and_requires_release(self):
        self.feed()
        self.hold()
        self.node._dispatch_operator.assert_called_once()
        self.assertEqual(self.node._dispatch_operator.call_args.args[0], 'fail')
        self.assertFalse(self.node._dispatch_operator.call_args.kwargs['launcher'])
        self.hold()
        self.assertEqual(self.node._dispatch_operator.call_count, 1)
        self.feed()
        self.hold()
        self.assertEqual(self.node._dispatch_operator.call_count, 2)

    def test_short_click_or_axis_motion_does_not_save(self):
        self.feed()
        for _ in range(9):
            self.feed(right=True)
        self.feed()
        for _ in range(15):
            self.feed(push=True)
        self.node._dispatch_operator.assert_not_called()

    def test_two_stick_chord_cancels_until_both_release(self):
        self.feed()
        for _ in range(8):
            self.feed(right=True)
        self.hold(left=True)
        self.hold()
        self.node._dispatch_operator.assert_not_called()
        self.feed()
        self.hold()
        self.node._dispatch_operator.assert_called_once()

    def test_tracking_loss_and_transport_gap_require_fresh_release(self):
        self.feed()
        for _ in range(8):
            self.feed(right=True)
        self.feed(right=True, tracked=False)
        self.hold()
        self.node._dispatch_operator.assert_not_called()
        self.feed()
        for _ in range(8):
            self.feed(right=True)
        self.feed(right=True, step=.4)
        self.hold()
        self.node._dispatch_operator.assert_not_called()
        self.feed()
        self.hold()
        self.node._dispatch_operator.assert_called_once()

    def test_held_button_cannot_carry_into_a_new_trial(self):
        self.feed()
        for _ in range(8):
            self.feed(right=True)
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

    def test_face_buttons_take_precedence_without_changing_abxy(self):
        self.feed()
        for _ in range(8):
            self.feed(right=True)
        self.feed(right=True, faces={'rightController': {'bButton': True}})
        self.hold()
        self.node._dispatch_operator.assert_not_called()


if __name__ == '__main__':
    unittest.main()
