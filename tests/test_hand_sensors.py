import unittest
from collections import deque
from threading import Lock
from types import SimpleNamespace

import numpy as np

from hand_sensors import HandSensorCapture, decode_packet, tactile_points


class NumericFrame:
    def __init__(self, **fields):
        self.values = fields
        self.fields = {
            (0, key): SimpleNamespace(validity=b'\x01' * len(array))
            for key, array in fields.items()
        }

    def numpy(self, sensor_id, name):
        return self.values[name]


class HandSensorsTest(unittest.TestCase):
    def test_wrench_order_and_stale_frame_rejection(self):
        stamp = 1_795_000_000.0
        packet = NumericFrame(
            force=np.array([[1, 2, 3]]), torque=np.array([[4, 5, 6]]),
            host_receipt_epoch_s=np.array([stamp]),
        )
        sample = decode_packet(packet, "force", "left")
        stream = "observation.force.left"
        capture = HandSensorCapture.__new__(HandSensorCapture)
        capture.samples = {stream: deque((sample,))}
        capture.errors = {}
        capture.lock = Lock()
        frame, offsets = capture.match(stamp + 0.01, stamp + 0.03, 0.03, 0.15)
        np.testing.assert_array_equal(frame[stream], [1, 2, 3, 4, 5, 6])
        self.assertAlmostEqual(offsets[stream], -10.0, places=2)
        self.assertTrue(capture.problems(stamp + 1, 0.15))
        with self.assertRaisesRegex(ValueError, "no fresh sample"):
            capture.match(stamp + 1, stamp + 1.05, 0.03, 0.15)
        packet.fields[(0, "torque")].validity = b"\0"
        with self.assertRaisesRegex(ValueError, "invalid torque"):
            decode_packet(packet, "force", "left")

    def test_tactile_sorts_and_preserves_all_points_and_status(self):
        stamp = 1_795_000_000.0
        pairs = list(reversed(tactile_points("right")))
        samples = np.arange(8)[::-1]
        packet = NumericFrame(
            logical_sensor_id=np.array([pair[0] for pair in pairs]),
            point_index=np.array([pair[1] for pair in pairs]),
            contact_force_xyz=samples[:, None] + np.array([[0, 10, 20]]),
            normal_force=samples + 30,
            tangential_force=samples + 40,
            tangential_direction=samples + 50,
            status=samples + 1,
            host_receipt_epoch_s=stamp + samples / 1000,
        )
        sample = decode_packet(packet, "tactile", "right")
        result = sample.values.reshape(8, 6)
        np.testing.assert_array_equal(result, np.arange(8)[:, None] + np.arange(6)[None, :] * 10)
        np.testing.assert_array_equal(sample.status, np.arange(1, 9))
        self.assertEqual(sample.stamps[0], stamp)
        self.assertGreater(sample.stamps[-1], stamp)
        with self.assertRaisesRegex(ValueError, "Agile left"):
            decode_packet(packet, "tactile", "left")
