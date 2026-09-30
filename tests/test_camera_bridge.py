"""Run the bridge's publish callback without a ROS graph or camera devices."""
import ast
from pathlib import Path
from types import SimpleNamespace
import time
import unittest
from unittest.mock import Mock

import cv2
import numpy as np

from camera_config import CAMERA_SPECS
from shm_camera import PIXEL_FORMAT_MJPEG, ShmFrame, frame_to_hwc


class CameraBridgeTest(unittest.TestCase):
    def test_jpeg_is_published_as_decoded_bgr_and_depth_is_unchanged(self):
        source = Path(__file__).resolve().parents[1] / 'shm_camera_topic_bridge.py'
        tree = ast.parse(source.read_text())
        callback = next(method for cls in tree.body if isinstance(cls, ast.ClassDef)
                        for method in cls.body if isinstance(method, ast.FunctionDef)
                        and method.name == '_publish_once')
        bgr = np.full((8, 10, 3), (20, 80, 160), np.uint8)
        ok, encoded = cv2.imencode('.jpg', bgr)
        self.assertTrue(ok)
        cases = (
            ('hand_left', ShmFrame(1, 10, 8, 3, PIXEL_FORMAT_MJPEG, encoded.size, encoded.tobytes())),
            ('rgbd_head_depth', ShmFrame(1, 1, 1, 1, 2, 2, b'\x34\x12')),
        )
        for name, frame in cases:
            with self.subTest(name=name):
                node = Mock()
                node.cameras = [CAMERA_SPECS[name]]
                node.last_source_stamp = {}
                node.published_counts = {name: 0}
                node.image_publishers = {name: Mock()}
                node.last_status_time = time.time()
                read = Mock(return_value=frame)
                namespace = dict(time=time, read_shm_metadata=lambda _: (1,), read_shm_frame=read,
                                 frame_to_hwc=frame_to_hwc, stamp_from_ns=lambda stamp, _: stamp,
                                 Image=lambda: SimpleNamespace(header=SimpleNamespace()))
                exec(compile(ast.Module(body=[callback], type_ignores=[]), str(source), 'exec'), namespace)
                namespace['_publish_once'](node)
                msg = node.image_publishers[name].publish.call_args.args[0]
                depth = name.endswith('depth')
                self.assertEqual(msg.encoding, '16UC1' if depth else 'bgr8')
                self.assertEqual(msg.step, frame.width * (2 if depth else 3))
                self.assertEqual(msg.data, frame_to_hwc(frame, depth).tobytes())
                namespace['_publish_once'](node)
                read.assert_called_once()
                node.image_publishers[name].publish.assert_called_once()
