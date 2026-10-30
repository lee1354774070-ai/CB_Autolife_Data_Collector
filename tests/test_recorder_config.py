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
        self.assertEqual(args.jpeg_decoder, 'auto')
        self.assertEqual(args.jpeg_device, 'cuda:0')
        self.assertFalse(args.with_head or args.with_waist or args.with_upper_waist or args.with_depth)
        self.assertEqual(args.video_accelerator, 'cpu')
        self.assertEqual((args.encoder_threads, args.depth_encoder_threads), (2, 2))

    def test_hardware_encoding_is_opt_in_and_preserves_depth_settings(self):
        args = parse_args(['--output-dir', '/unused', '--video-accelerator', 'nvenc',
                           '--with-depth', '--video-hw-qp', '20', '--depth-encoder-threads', '3'])
        self.assertEqual(args.video_accelerator, 'nvenc')
        self.assertEqual(args.video_hw_qp, 20)
        self.assertEqual(args.depth_encoder_threads, 3)
        self.assertEqual(args.depth_vcodec, 'hevc')

    def test_invalid_settings_fail_before_runtime_imports(self):
        for options in (
            ['--min-cameras', '4'], ['--encoder-threads', '0'], ['--video-crf', 'nan'],
            ['--video-gop', '-1'], ['--data-files-size-in-mb', '0'],
            ['--sync-reference-camera', 'unknown'], ['--camera', 'hand_left', '--camera', 'hand_left'],
            ['--image-source', 'ros', '--camera', '../outside=/image'],
            ['--image-source', 'ros', '--camera', 'ok=relative'],
            ['--camera', 'rgbd_head_depth=/camera/depth'],
            ['--depth-camera', 'head_left=/camera/head'],
            ['--video-hw-qp', '-1'], ['--video-hw-qp', '52'],
            ['--depth-encoder-threads', '0'], ['--depth-encoder-threads', '65'],
            ['--video-encoding-timeout-sec', 'nan'], ['--video-encoding-timeout-sec', '0'],
            ['--video-accelerator', 'nvenc', '--vcodec', 'av1'],
            ['--video-accelerator', 'auto', '--video-crf', '20'],
            ['--video-accelerator', 'vaapi', '--video-preset', 'fast'],
            ['--video-encoding-timeout-sec', 'nan'], ['--video-encoding-timeout-sec', '0'],
            ['--video-accelerator', 'unknown'],
            ['--video-accelerator', 'vaapi', '--vcodec', 'hevc'],
            ['--video-accelerator', 'auto', '--video-crf', '23'],
            ['--video-accelerator', 'nvenc', '--video-preset', 'fast'],
            ['--jpeg-decoder', 'unknown'], ['--jpeg-device', 'cpu'],
            ['--jpeg-device', 'cuda:-1'], ['--image-source', 'ros', '--jpeg-decoder', 'nvjpeg'],
        ):
            with self.subTest(options=options), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                parse_args(['--output-dir', '/unused', *options])

    def test_all_help_entries_resolve_and_single_help_is_scoped(self):
        self.assertGreater(len(parameters()), 75)
        self.assertEqual(parameters()['JPEG_DECODER'][0], 'auto')
        self.assertIn('nvjpeg', parameters()['JPEG_DECODER'][1])
        self.assertEqual(parse_args(['--output-dir', '/unused', '--jpeg-decoder', 'nvjpeg']).jpeg_decoder, 'nvjpeg')
        for args in (['MAX_SYNC_DELTA_SEC', '--help'], ['--help', 'MAX_SYNC_DELTA_SEC']):
            result = subprocess.run(['bash', str(ROOT / 'start_lerobot_official_collect.sh'), *args],
                                    text=True, capture_output=True, timeout=5)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('0.03', result.stdout)
            self.assertNotIn('DEPTH_SHIFT', result.stdout)

    def test_ros_default_auto_remains_compatible_and_explicit_cpu_is_available(self):
        self.assertEqual(parse_args(['--output-dir', '/unused', '--image-source', 'ros']).jpeg_decoder, 'auto')
        self.assertEqual(parse_args(['--output-dir', '/unused', '--jpeg-decoder', 'opencv']).jpeg_decoder, 'opencv')

    def test_launcher_decoder_default_and_startup_timeout_match_help(self):
        source = (ROOT / 'start_lerobot_official_collect.sh').read_text()
        self.assertIn('--jpeg-decoder "${JPEG_DECODER:-auto}"', source)
        start = source.index('camera_ready_timeout=15')
        end = source.index('\npython3 ', start)
        for image_source, decoder, encoder, expected in (
            ('shm', None, 'cpu', 45), ('shm', 'nvjpeg', 'cpu', 45),
            ('shm', 'opencv', 'cpu', 15), ('ros', None, 'cpu', 15),
            ('ros', 'opencv', 'nvenc', 45), ('shm', 'opencv', 'auto', 45),
        ):
            env = dict(os.environ, IMAGE_SOURCE=image_source, VIDEO_ACCELERATOR=encoder)
            env.pop('JPEG_DECODER', None)
            if decoder is not None:
                env['JPEG_DECODER'] = decoder
            result = subprocess.run(['bash', '-c', source[start:end] + '\nprintf "%s" "$camera_ready_timeout"'],
                                    env=env, text=True, capture_output=True, check=True, timeout=5)
            self.assertEqual(int(result.stdout), expected)
        self.assertIn('${CAMERA_DETECTION_TIMEOUT_SEC:-${camera_ready_timeout}}', source)

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
