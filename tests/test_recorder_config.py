"""Configuration and help are testable without ROS, NumPy or LeRobot."""

import contextlib
import io
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from recorder_config import parse_args
from launcher_help import parameters
from time_sync import payload_timestamp_sec

ROOT = Path(__file__).resolve().parents[1]


class RecorderConfigTest(unittest.TestCase):
    def test_preserves_default_schema_and_sync_limits(self):
        args = parse_args(['--output-dir', '/unused'])
        self.assertEqual((args.fps, args.sync_image_buffer_size, args.max_sync_delta_sec), (30, 16, .03))
        self.assertEqual(list(args.cameras), ['rgbd_head_color', 'hand_left', 'hand_right'])
        self.assertEqual(args.action_mode, 'status_target')
        self.assertFalse(args.with_head or args.with_waist or args.with_upper_waist or args.with_depth)

    def test_invalid_settings_fail_before_runtime_imports(self):
        for options in (
            ['--min-cameras', '4'], ['--encoder-threads', '0'], ['--video-crf', 'nan'],
            ['--video-gop', '-1'], ['--data-files-size-in-mb', '0'],
            ['--sync-reference-camera', 'unknown'], ['--camera', 'hand_left', '--camera', 'hand_left'],
            ['--image-source', 'ros', '--camera', '../outside=/image'],
            ['--image-source', 'ros', '--camera', 'ok=relative'],
            ['--camera', 'rgbd_head_depth=/camera/depth'],
            ['--depth-camera', 'head_left=/camera/head'],
        ):
            with self.subTest(options=options), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                parse_args(['--output-dir', '/unused', *options])

    def test_all_help_entries_resolve_and_single_help_is_scoped(self):
        self.assertGreater(len(parameters()), 75)
        for args in (['MAX_SYNC_DELTA_SEC', '--help'], ['--help', 'MAX_SYNC_DELTA_SEC']):
            result = subprocess.run(['bash', str(ROOT / 'start_lerobot_official_collect.sh'), *args],
                                    text=True, capture_output=True, timeout=5)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('0.03', result.stdout)
            self.assertNotIn('DEPTH_SHIFT', result.stdout)

    def test_recorder_help_works_in_standard_library_only_interpreter(self):
        for args in (['--help'], ['--with-depth', '--help']):
            result = subprocess.run([sys.executable, '-S', str(ROOT / 'record_lerobot_official.py'), *args],
                                    text=True, capture_output=True, timeout=5)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('--with-depth', result.stdout)

    def test_bad_shell_switches_and_paths_create_no_output(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'unused'
            for task, extra in [('test', {'WITH_HEAD': 'true'}), ('test', {'WITH_DEPTH': '2'}),
                                ('../outside', {}), ('--unknown', {})]:
                env = dict(os.environ, OUTPUT_BASE_DIR=str(output), COLLECTOR_MODE='keyboard',
                           VR_CONTROL='0', SUBTASKS_JSON='[]', **extra)
                result = subprocess.run(['bash', str(ROOT / 'start_lerobot_official_collect.sh'), task],
                                        env=env, text=True, capture_output=True, timeout=5)
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertFalse(output.exists())

    def test_huge_timestamp_uses_receive_time_without_crashing_callback(self):
        for value in (10 ** 1000, {'sec': 10 ** 1000, 'nanosec': 0}):
            self.assertEqual(payload_timestamp_sec({'timestamp': value}, 1784527495), 1784527495)


if __name__ == '__main__':
    unittest.main()
