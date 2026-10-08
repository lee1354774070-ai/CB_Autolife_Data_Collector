"""MZJ capture regressions: lossless pixels, bounded work, and process isolation."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

import numpy as np
from PIL import Image
try:
    from dagger import capture_image_writer as module
except ImportError:
    module = None

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipIf(module is None, 'requires the deployed LeRobot image-writer API')
class CaptureTest(unittest.TestCase):
    def test_rgb_chw_depth_and_older_opencv_are_lossless(self):
        rng = np.random.default_rng(17)
        rgb = rng.integers(0, 256, (24, 32, 3), dtype=np.uint8)
        depth = rng.integers(0, 65536, (24, 32, 1), dtype=np.uint16)
        with tempfile.TemporaryDirectory() as directory:
            for source, expected, suffix in ((rgb, rgb, '.png'),
                    (rgb.transpose(2, 0, 1), rgb, '.png'), (depth, depth[..., 0], '.tiff')):
                path = Path(directory) / ('frame' + suffix)
                module.write_capture_image(source, path)
                with Image.open(path) as img:
                    actual = np.asarray(img)
                self.assertEqual(actual.dtype, expected.dtype)
                np.testing.assert_array_equal(actual, expected)
            # Old OpenCV installations use the original PIL fallback.
            with patch.object(module, 'cv2', object()):
                path = Path(directory) / 'fallback.png'
                module.write_capture_image(rgb, path)
                with Image.open(path) as img:
                    np.testing.assert_array_equal(np.asarray(img), rgb)

    def test_full_queue_and_disk_failure_are_visible(self):
        entered, release = threading.Event(), threading.Event()
        def block(*args):
            entered.set()
            if not release.wait(3):
                raise TimeoutError('test writer not released')
        with patch.object(module, 'write_capture_image', block):
            writer = module.CaptureImageWriter(1, max_pending=2)
            try:
                writer.save_image(None, 'one.png')
                self.assertTrue(entered.wait(2))
                writer.save_image(None, 'two.png')
                with self.assertRaisesRegex(RuntimeError, 'queue full'):
                    writer.save_image(None, 'three.png')
            finally:
                release.set()
                writer.stop()
            self.assertEqual(writer.pending_count, 0)
            with self.assertRaisesRegex(RuntimeError, 'stopped'):
                writer.save_image(None, 'four.png')
        with patch.object(module, 'write_capture_image', side_effect=OSError('disk full')):
            writer = module.CaptureImageWriter(1)
            writer.save_image(None, 'one.png')
            with self.assertRaisesRegex(OSError, 'disk full'):
                writer.wait_until_done()
            writer.stop()


@unittest.skipUnless(hasattr(os, 'sched_getaffinity') and shutil.which('taskset'), 'Linux affinity required')
class IsolationTest(unittest.TestCase):
    def test_only_child_and_descendants_inherit_resource_limits(self):
        before = os.sched_getaffinity(0)
        nice = os.getpriority(os.PRIO_PROCESS, 0)
        chosen = min(before)
        probe = ('import os,json;print(json.dumps(dict(cpus=sorted(os.sched_getaffinity(0)), '
                 'nice=os.getpriority(os.PRIO_PROCESS,0), threads=os.environ["OPENBLAS_NUM_THREADS"])))')
        child = 'import subprocess,sys;exec(' + repr(probe) + ');subprocess.run([sys.executable,"-c",' + repr(probe) + '],check=True)'
        result = subprocess.run(['bash', str(ROOT / 'dagger/run_recorder_isolated.sh'), sys.executable, '-c', child],
            env={**os.environ, 'HG_DAGGER_RECORDER_CPUS': str(chosen), 'OPENBLAS_NUM_THREADS': '8'},
            text=True, capture_output=True, check=True)
        rows = [json.loads(line) for line in result.stdout.splitlines() if line.startswith('{')]
        self.assertEqual(len(rows), 2)
        for row in rows:
            self.assertEqual(row, dict(cpus=[chosen], nice=min(19, nice + 5), threads='1'))
        self.assertEqual(os.sched_getaffinity(0), before)
        self.assertEqual(os.getpriority(os.PRIO_PROCESS, 0), nice)
        result = subprocess.run(['bash', str(ROOT / 'dagger/run_recorder_isolated.sh'), sys.executable, '-c', 'print("UNSAFE_STARTED")'],
            env={**os.environ, 'HG_DAGGER_RECORDER_CPUS': 'invalid'}, text=True, capture_output=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn('UNSAFE_STARTED', result.stdout)


if __name__ == '__main__':
    unittest.main()
