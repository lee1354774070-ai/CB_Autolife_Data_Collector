"""Production callback wiring without ROS, devices or a dataset."""
import ast
from collections import deque
import inspect
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
from typing import Any

from time_sync import telemetry_timestamps

SOURCE = Path(__file__).resolve().parents[1] / 'record_lerobot_official.py'
tree = ast.parse(SOURCE.read_text())
names = {'_state_ros_cb', '_state_cb', '_state_eef_ros_cb', '_action_eef_ros_cb'}
methods = [method for cls in tree.body if isinstance(cls, ast.ClassDef)
           for method in cls.body if isinstance(method, ast.FunctionDef) and method.name in names]
scope = dict(Any=Any, String=object, telemetry_timestamps=telemetry_timestamps,
             time=SimpleNamespace(time=lambda: 1_750_000_000.08),
             parse_json_payload=lambda text: {'timestamp': 1_749_999_989.0},
             StateSample=SimpleNamespace)
exec(compile(ast.Module(body=methods, type_ignores=[]), str(SOURCE), 'exec'), scope)


class RecorderTimingTest(unittest.TestCase):
    def test_dds_metadata_reaches_atomic_state_and_action_packet(self):
        node = Mock()
        node.args.max_state_age_sec = .15
        node.state_buffer = deque()
        state, target = object(), object()
        node.schema.parse_state.return_value = state
        node.schema.parse_status_target.return_value = target
        node._state_cb = lambda *a: scope['_state_cb'](node, *a)
        scope['_state_ros_cb'](node, SimpleNamespace(data='state'),
                              {'source_timestamp': 1_750_000_000_000_000_000})
        sample, = node.state_buffer
        self.assertEqual(sample.stamp_sec, 1_750_000_000.0)
        self.assertEqual(sample.received_sec, 1_750_000_000.0)
        self.assertIs(sample.state, state)
        self.assertIs(sample.status_target_action, target)

    def test_ros_wrappers_require_message_info_for_rclpy_dispatch(self):
        for name in ('_state_ros_cb', '_state_eef_ros_cb', '_action_eef_ros_cb'):
            with self.subTest(name=name):
                signature = inspect.signature(scope[name])
                with self.assertRaises(TypeError):
                    signature.bind(object(), object())  # self + message is insufficient
                signature.bind(object(), object(), {})

    def test_eef_wrappers_forward_metadata_without_touching_dagger_authority(self):
        node = Mock()
        info, msg = {}, object()
        scope['_state_eef_ros_cb'](node, msg, info)
        node._store_eef.assert_called_once_with(msg, node.eef_state_buffer, node.args.max_state_age_sec, info)
        node._store_eef.reset_mock()
        scope['_action_eef_ros_cb'](node, msg, info)
        node._store_eef.assert_called_once_with(msg, node.action_eef_buffer, node.args.max_action_hold_sec, info)


if __name__ == '__main__':
    unittest.main()
