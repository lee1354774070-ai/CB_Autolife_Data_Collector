"""Failure diagnostics must preserve missing-camera and ROS-source handling."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

SOURCE = Path(__file__).resolve().parents[1] / 'record_lerobot_official.py'
tree = ast.parse(SOURCE.read_text())
method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
              and n.name == '_capture_diagnostics')


class RecorderTimingTest(unittest.TestCase):
    def setUp(self):
        self.read = Mock(return_value=(1234,))
        self.scope = {'read_shm_metadata': self.read,
                      'CAMERA_SPECS': {'hand_left': SimpleNamespace(meta_path='left')}}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), 'exec'), self.scope)
        self.node = SimpleNamespace(args=SimpleNamespace(image_source='shm'),
            active_cameras=['hand_left'], last_shm_timestamps={'left': 1200},
            _last_shm_poll_sec=9.5, _last_shm_poll_gap_ms=400,
            _last_record_tick_ms=2., _recent_gc_pause={'duration_ms': 300., 'wall_time': 9.9})

    def diagnose(self):
        return self.scope['_capture_diagnostics'](self.node, 10.)

    def test_distinguishes_producer_progress_from_callback_stall(self):
        result = self.diagnose()
        self.assertEqual(result['last_shm_poll_age_ms'], 500.)
        self.assertEqual(result['cameras']['hand_left'], {'last_copied_ns': 1200, 'producer_ns': 1234})
        self.assertEqual(result['recent_gc_pause']['duration_ms'], 300.)

    def test_missing_metadata_does_not_mask_original_recording_error(self):
        self.read.return_value = None
        self.assertIsNone(self.diagnose()['cameras']['hand_left']['producer_ns'])
        self.read.side_effect = OSError('camera missing')
        self.assertIn('camera missing', self.diagnose()['cameras']['hand_left']['diagnostic_error'])

    def test_ros_images_do_not_read_shm(self):
        self.node.args.image_source = 'ros'
        self.assertNotIn('cameras', self.diagnose())
        self.read.assert_not_called()


if __name__ == '__main__':
    unittest.main()
