"""Exercise real start/reset method bodies without importing ROS or LeRobot.

Only the surrounding hardware/dataset dependencies are replaced. AST extraction
keeps these tests tied to production methods rather than copies of their logic.
"""

import ast
import time
import unittest
from collections import deque
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock


SOURCE = Path(__file__).resolve().parents[1] / 'record_lerobot_official.py'
tree = ast.parse(SOURCE.read_text())
methods = [item for cls in tree.body if isinstance(cls, ast.ClassDef)
           for item in cls.body if isinstance(item, ast.FunctionDef)
           and item.name in {'start_episode', '_reset_episode_state'}]
namespace = {'time': time}
exec(compile(ast.Module(body=methods, type_ignores=[]), str(SOURCE), 'exec'), namespace)


class RecorderControlsTest(unittest.TestCase):
    def recorder(self):
        node = Mock()
        node.dataset = object()
        node.is_recording = False
        node.episode_invalid = False
        node.current_episode_frames = 0
        node.saved_episodes = 3
        node.session_frames_written = 42
        node.reported_image_overflows = {'left'}
        node.has_pending_episode.return_value = False
        node._acquire_motion_lock.return_value = True
        node.sync_reference_camera = 'left'
        node.image_buffers = {'left': deque([SimpleNamespace(stamp_sec=10)]), 'right': deque([object()])}
        node._reset_episode_state = lambda: namespace['_reset_episode_state'](node)
        return node

    def test_start_clears_warmup_fifo_keeps_time_lower_bound_and_acks(self):
        node = self.recorder()
        self.assertTrue(namespace['start_episode'](node, 'test', 'request-1'))
        self.assertTrue(node.is_recording)
        self.assertTrue(all(not b for b in node.image_buffers.values()))
        self.assertEqual(node.last_reference_stamp_sec, 10)
        self.assertIsNone(node.last_episode_anchor_stamp_sec)
        self.assertFalse(node.reported_image_overflows)
        node._write_command_status.assert_called_once_with(
            'start', True, 'request-1', episode_index=3, message='episode started')

    def test_each_rejected_start_is_acknowledged_without_clearing_frames(self):
        for field, value in [('dataset', None), ('is_recording', True), ('episode_invalid', True), ('current_episode_frames', 5)]:
            with self.subTest(field=field):
                node = self.recorder()
                setattr(node, field, value)
                self.assertFalse(namespace['start_episode'](node, 'test', 'rejected'))
                self.assertEqual(len(node.image_buffers['left']), 1)
                self.assertEqual(node._write_command_status.call_args.args, ('start', False, 'rejected'))

    def test_motion_lock_rejection_is_acknowledged(self):
        node = self.recorder()
        node._acquire_motion_lock.return_value = False
        self.assertFalse(namespace['start_episode'](node, 'test', 'busy'))
        node._write_command_status.assert_called_once_with('start', False, 'busy', message='robot motion lock is busy')


if __name__ == '__main__':
    unittest.main()
