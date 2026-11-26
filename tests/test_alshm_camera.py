import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock

from alshm_camera import AlshmCameraReader
from shm_camera import PIXEL_FORMAT_BGR, PIXEL_FORMAT_MJPEG


EPOCH = 1_795_676_558_900_000_000


def packet(source, capture, ms, fmt, data, size=(2, 2)):
    stamp = EPOCH + ms * 1_000_000
    plane = NS(sensor_id=0, source_frame_id=source, capture_id=capture, publish_epoch_ns=stamp,
               width=size[0], height=size[1], pixel_format=fmt, data=data)
    return NS(status="frame", frame=NS(planes=(plane,), meta=NS(publish_epoch_ns=stamp)))


class WristRecoveryTest(unittest.TestCase):
    def make_reader(self, candidate, next_id=5490, next_capture=5492):
        reader = AlshmCameraReader.__new__(AlshmCameraReader)
        reader.shm_error = OSError
        reader.publisher_lost = ()
        reader.formats = {"BGR": (PIXEL_FORMAT_BGR, 3, 1), "MJPG": (PIXEL_FORMAT_MJPEG, 3, 0)}
        reader.logger = Mock()
        reader.last_wrists = {}
        reader.decoded_wrists = {"hand_right": NS(spec=NS(slots=4))}
        self.decoded = Mock()
        self.decoded.read.side_effect = [candidate, NS(status="no_data", frame=None)]
        reader.open_camera_channel = Mock(return_value=self.decoded)
        jpeg = Mock()
        jpeg.read.side_effect = [packet(5488, 5490, 0, "MJPG", b"a"),
                                 packet(next_id, next_capture, 68, "MJPG", b"b")]
        reader.readers = [("right_jpeg", jpeg, {0: "hand_right"})]
        return reader

    def test_gap_recovers_exact_original_exposure_before_newest_jpeg(self):
        pixels = bytes(range(12))
        reader = self.make_reader(packet(5489, 5491, 34, "BGR", pixels))
        self.assertEqual(len(reader.read_frames()), 1)
        frames = [row[2] for row in reader.read_frames()]
        self.assertEqual([(f.timestamp_ns, f.pixel_format, f.data) for f in frames],
                         [(EPOCH + 34_000_000, PIXEL_FORMAT_BGR, pixels),
                          (EPOCH + 68_000_000, PIXEL_FORMAT_MJPEG, b"b")])
        reader.open_camera_channel.assert_called_once_with(reader.decoded_wrists["hand_right"], start="oldest")
        self.decoded.close.assert_called_once()
        reader.logger.info.assert_called_once()

    def test_rejects_nonmatching_capture_id_or_stale_timestamp(self):
        for candidate in (packet(5489, 1, 34, "BGR", bytes(12)),
                          packet(5489, 5491, -30, "BGR", bytes(12)),
                          packet(5489, 5491, 34, "BGR", bytes(24), size=(2, 4))):
            with self.subTest(candidate=candidate):
                reader = self.make_reader(candidate)
                reader.read_frames()
                self.assertEqual([row[2].data for row in reader.read_frames()], [b"b"])
                self.decoded.close.assert_called_once()
                reader.logger.info.assert_not_called()

    def test_continuous_jpeg_does_not_open_decoded_reader(self):
        reader = self.make_reader(None, next_id=5489, next_capture=5491)
        reader.read_frames()
        self.assertEqual(len(reader.read_frames()), 1)
        reader.open_camera_channel.assert_not_called()
