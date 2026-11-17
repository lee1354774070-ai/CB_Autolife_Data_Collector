import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
import cv2
import numpy as np
from jpeg_decoding import StrictCpuJpegDecoder, save_jpeg_failure


@unittest.skipUnless(Path(__file__).resolve().parents[1].joinpath('libstrictjpeg.so').exists(), 'private native decoder not built')
class StrictJpegTest(unittest.TestCase):
    def setUp(self):
        self.decoder = StrictCpuJpegDecoder()
        rng = np.random.default_rng(17)
        self.source = rng.integers(0, 256, (64, 80, 3), dtype=np.uint8)
        _, encoded = cv2.imencode('.jpg', self.source)
        self.jpeg = encoded.tobytes()

    def test_real_pixels_order_padding_progressive_and_independent_storage(self):
        _, progressive = cv2.imencode('.jpg', self.source, [cv2.IMWRITE_JPEG_PROGRESSIVE, 1])
        frames = [(self.jpeg, 80, 64), (progressive.tobytes() + bytes(1024), 80, 64)]
        images = self.decoder.decode(frames)
        for frame, image in zip(frames, images):
            reference = cv2.imdecode(np.frombuffer(frame[0], np.uint8), cv2.IMREAD_COLOR)[..., ::-1]
            np.testing.assert_array_equal(image, reference)
            self.assertTrue(image.flags.c_contiguous)
            self.assertTrue(image.flags.owndata)
        self.assertFalse(np.shares_memory(*images))
        original = images[0].copy()
        self.decoder.decode(list(reversed(frames)))
        np.testing.assert_array_equal(images[0], original)

    def test_corruption_is_fatal_even_when_opencv_returns_pixels(self):
        corrupt = self.jpeg[:len(self.jpeg)//2] + b'\xff\xd9'
        self.assertIsNotNone(cv2.imdecode(np.frombuffer(corrupt, np.uint8), cv2.IMREAD_COLOR))
        for data in (b'not jpeg', self.jpeg[:-2], corrupt):
            with self.subTest(size=len(data)), self.assertRaises(ValueError):
                self.decoder.decode([(self.jpeg, 80, 64), (data, 80, 64)])
        with self.assertRaisesRegex(ValueError, 'dimensions'):
            self.decoder.decode([(self.jpeg, 81, 64)])
        self.assertEqual(self.decoder.decode([]), [])
        self.assertEqual(len(self.decoder.decode([(self.jpeg, 80, 64)])), 1)


class DiagnosticTest(unittest.TestCase):
    def test_exact_bytes_timestamps_digest_and_event_bound(self):
        with tempfile.TemporaryDirectory() as directory:
            frame = SimpleNamespace(data=b'bad original jpeg', timestamp_ns=1234567,
                                    width=640, height=480, pixel_format=4)
            for _ in range(10):
                save_jpeg_failure(Path(directory)/'dataset', [('hand_left', None, frame, 0)], 'bad stream')
            events = list(Path(directory).glob('jpeg_failures/event-*'))
            self.assertEqual(len(events), 8)
            info = json.loads((events[0]/'failure.json').read_text())['frames'][0]
            self.assertEqual(info['timestamp_ns'], frame.timestamp_ns)
            self.assertEqual((events[0]/info['file']).read_bytes(), frame.data)
            self.assertEqual(info['sha256'], hashlib.sha256(frame.data).hexdigest())
