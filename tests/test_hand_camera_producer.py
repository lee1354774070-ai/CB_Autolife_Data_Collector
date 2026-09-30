"""JPEG forwarding contract, using a fake V4L2 camera and temporary SHM files."""

import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import cv2
import numpy as np

from camera_config import HandCameraSpec
from hand_camera_producer import HandCameraProducer
from shm_camera import PIXEL_FORMAT_MJPEG, frame_to_hwc, read_shm_frame


class HandCameraProducerTest(unittest.TestCase):
    def test_forwards_original_jpeg_without_per_frame_decode_or_encode(self):
        source = np.full((8, 10, 3), (20, 80, 160), dtype=np.uint8)
        ok, encoded = cv2.imencode('.jpg', source)
        self.assertTrue(ok)
        with tempfile.TemporaryDirectory() as directory:
            spec = HandCameraSpec('hand_left', str(Path(directory) / 'meta'),
                                  str(Path(directory) / 'buffer'), '/camera/hand_left/image_raw',
                                  'hand_left', '/dev/test')
            capture = Mock()
            capture.read.return_value = (True, encoded.reshape(1, -1))
            producer = HandCameraProducer(spec.name, spec)
            with patch('hand_camera_producer.cv2.VideoCapture', return_value=capture):
                self.assertTrue(producer.open())
            capture.set.assert_any_call(cv2.CAP_PROP_CONVERT_RGB, 0)
            with patch('hand_camera_producer.cv2.imdecode', side_effect=AssertionError('extra decode')), \
                 patch('hand_camera_producer.cv2.imencode', side_effect=AssertionError('re-encode')):
                self.assertTrue(producer.produce_once())
            frame = read_shm_frame(spec)
            self.assertEqual(frame.pixel_format, PIXEL_FORMAT_MJPEG)
            self.assertEqual((frame.width, frame.height), (10, 8))
            self.assertEqual(frame.data, encoded.tobytes())
            self.assertEqual(frame_to_hwc(frame, False).shape, source.shape)
            producer.close()
            capture.release.assert_called_once()

    def test_rejects_bgr_pixels_or_non_jpeg_packet(self):
        capture = Mock()
        for frame in (np.zeros((8, 10, 3), np.uint8), np.zeros((1, 100), np.uint8), None):
            capture.read.return_value = (True, frame)
            self.assertIsNone(HandCameraProducer._read_one(capture))

    def test_unsupported_raw_capture_fails_without_starting_producer(self):
        spec = HandCameraSpec('hand_left', '/unused', '/unused', '/unused', 'hand_left', '/dev/test')
        capture = Mock()
        capture.set.return_value = False
        producer = HandCameraProducer(spec.name, spec)
        with patch('hand_camera_producer.cv2.VideoCapture', return_value=capture):
            self.assertFalse(producer.open())
        self.assertIsNone(producer.capture)
        capture.release.assert_called_once()
        capture.read.assert_not_called()
