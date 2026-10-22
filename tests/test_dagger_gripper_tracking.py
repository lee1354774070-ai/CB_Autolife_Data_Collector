"""Exercise the real tracking guard without ROS or motor interfaces."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from dagger.control.core import Mode

SOURCE = Path(__file__).resolve().parents[1] / 'dagger/control/supervisor_node.py'
tree = ast.parse(SOURCE.read_text())
method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
              and n.name == '_check_gripper_tracking_locked')
scope = {'Mode': Mode}
exec(compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), 'exec'), scope)


class GripperTrackingTest(unittest.TestCase):
    def setUp(self):
        self.node = SimpleNamespace(
            _machine=SimpleNamespace(mode=Mode.POLICY_ACTIVE, authority_epoch=3),
            _policy_gripper_epoch=-1, _gripper_tracking_warning='',
            _measured_grippers=[181., 181.], _gripper_measurement_ns=0,
            _controller_status={'hardware_ready': True}, _latest_grippers=[0., 0.],
            _gripper_tracking_side=-1, _gripper_tracking_since_ns=0,
            _gripper_tracking_start_measurement=0., _grips=[False, False],
            _event=Mock(), _begin_failure_hold=Mock())

    def tick(self, seconds):
        now = int(seconds * 1e9)
        self.node._gripper_measurement_ns = now
        scope['_check_gripper_tracking_locked'](self.node, now)

    def test_startup_and_previous_epoch_are_not_commands(self):
        for epoch in (-1, 2):
            self.node._policy_gripper_epoch = epoch
            self.tick(1)
            self.tick(4)
            self.node._event.assert_not_called()
            self.node._begin_failure_hold.assert_not_called()

    def test_real_command_stall_still_holds_after_two_seconds(self):
        self.node._policy_gripper_epoch = 3
        self.node._latest_grippers = [360., 360.]
        self.tick(1)
        self.tick(2)
        self.node._begin_failure_hold.assert_not_called()
        self.tick(3.1)
        self.node._begin_failure_hold.assert_called_once()

    def test_moving_gripper_restarts_stall_clock(self):
        self.node._policy_gripper_epoch = 3
        self.node._latest_grippers = [360., 360.]
        self.tick(1)
        self.node._measured_grippers = [220., 220.]
        self.tick(2.9)
        self.tick(3.1)
        self.node._begin_failure_hold.assert_not_called()


if __name__ == '__main__':
    unittest.main()
