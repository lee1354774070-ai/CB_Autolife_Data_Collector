import struct
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
import cv2

from camera_config import SHM_METADATA_FORMAT, CameraSpec
from shm_camera import (
    SHM2_HEADER_FORMAT,
    PIXEL_FORMAT_MJPEG,
    ShmFrame,
    frame_to_hwc,
    read_shm_frame,
    read_shm_metadata,
    shm_timestamp_sec,
)


class ShmCameraTest(unittest.TestCase):
    def test_corrupt_header_or_changed_generation_never_yields_a_frame(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            spec = CameraSpec('test', str(root / 'meta'), str(root / 'buffer'), '/test', 'test')
            Path(spec.buffer_path).write_bytes(b'abcdef')
            stable = (1700000000000000000, 2, 1, 3, 1, 6)
            Path(spec.meta_path).write_bytes(struct.pack(SHM_METADATA_FORMAT, *stable))
            for bad in (stable[:3], (stable[0], 10**10, 10**10, 3, 1, 10**22),
                        (*stable, -1, 1, 1), (*stable, 6, 1, 1)):
                self.assertIsNone(read_shm_frame(spec, bad))
            with patch('shm_camera.read_shm_metadata', return_value=(stable[0] + 1, *stable[1:])):
                self.assertIsNone(read_shm_frame(spec, stable))

    def test_shm2_cannot_read_past_its_published_slot(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            spec = CameraSpec('test', str(root / 'meta'), str(root / 'buffer'), '/test', 'test')
            raw = struct.pack(SHM2_HEADER_FORMAT, b'SHM2', 2, 2, 0, 2, 1,
                              2, 1, 3, 1, 7, struct.unpack('<I', b'BGR\0')[0], 0, 6, 0, 0)
            Path(spec.meta_path).write_bytes(raw)
            self.assertIsNone(read_shm_metadata(spec))

    def test_reader_requires_a_stable_metadata_pair(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            spec = CameraSpec(
                name="test",
                meta_path=str(root / "meta"),
                buffer_path=str(root / "buffer"),
                topic="/test",
                frame_id="test",
            )
            metadata = struct.pack(SHM_METADATA_FORMAT, 1_700_000_000_000_000_000, 2, 1, 3, 1, 6)
            Path(spec.buffer_path).write_bytes(b"\x01\x02\x03\x04\x05\x06")
            Path(spec.meta_path).write_bytes(metadata)

            parsed = read_shm_metadata(spec)
            frame = read_shm_frame(spec, parsed)

            self.assertEqual(parsed, (1_700_000_000_000_000_000, 2, 1, 3, 1, 6))
            self.assertIsNotNone(frame)
            self.assertEqual(frame.data, b"\x01\x02\x03\x04\x05\x06")

    def test_non_epoch_timestamp_falls_back_to_receive_time(self):
        self.assertEqual(shm_timestamp_sec(123, 42.5), 42.5)

    def test_shm2_reader_selects_the_active_image_slot(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            spec = CameraSpec(
                name="test",
                meta_path=str(root / "meta"),
                buffer_path=str(root / "buffer"),
                topic="/test",
                frame_id="test",
            )
            timestamp_ns = 1_700_000_000_000_000_000
            header = struct.pack(
                SHM2_HEADER_FORMAT,
                b"SHM2",
                2,
                2,
                1,
                84,
                42,
                2,
                1,
                3,
                1,
                6,
                struct.unpack("<I", b"BGR\0")[0],
                7,
                6,
                123,
                timestamp_ns,
            )
            Path(spec.meta_path).write_bytes(header.ljust(256, b"\0"))
            Path(spec.buffer_path).write_bytes(b"oldold" + b"newnew")

            parsed = read_shm_metadata(spec)
            frame = read_shm_frame(spec, parsed)

            self.assertIsNotNone(parsed)
            self.assertEqual(parsed[:6], (timestamp_ns, 2, 1, 3, 1, 6))
            self.assertEqual(parsed[6], 6)
            self.assertIsNotNone(frame)
            self.assertEqual(frame.data, b"newnew")

    def test_frame_decoder_handles_bgr_and_depth(self):
        rgb_frame = ShmFrame(1, 1, 1, 3, 1, 3, b"\x01\x02\x03")
        np.testing.assert_array_equal(frame_to_hwc(rgb_frame, False), np.array([[[1, 2, 3]]], dtype=np.uint8))
        np.testing.assert_array_equal(frame_to_hwc(rgb_frame, False, rgb=True), np.array([[[3, 2, 1]]], dtype=np.uint8))

        depth_frame = ShmFrame(1, 1, 1, 1, 2, 2, b"\x34\x12")
        np.testing.assert_array_equal(frame_to_hwc(depth_frame, True), np.array([[[0x1234]]], dtype=np.uint16))

    def test_shm2_reader_and_decoder_handle_mjpeg(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            spec = CameraSpec(
                name="test",
                meta_path=str(root / "meta"),
                buffer_path=str(root / "buffer"),
                topic="/test",
                frame_id="test",
            )
            source = np.full((8, 10, 3), (20, 80, 160), dtype=np.uint8)
            ok, encoded = cv2.imencode(".jpg", source)
            self.assertTrue(ok)
            jpeg = encoded.tobytes()
            timestamp_ns = 1_700_000_000_000_000_000
            slot_stride = 2048
            header = struct.pack(
                SHM2_HEADER_FORMAT,
                b"SHM2", 2, 2, 1, 10, 5, 10, 8, 3, 1, len(jpeg),
                struct.unpack("<I", b"MJPG")[0], 7, slot_stride, 123, timestamp_ns,
            )
            Path(spec.meta_path).write_bytes(header.ljust(256, b"\0"))
            Path(spec.buffer_path).write_bytes(bytes(slot_stride) + jpeg)

            metadata = read_shm_metadata(spec)
            frame = read_shm_frame(spec, metadata)

            self.assertEqual(metadata[4], PIXEL_FORMAT_MJPEG)
            self.assertIsNotNone(frame)
            decoded = frame_to_hwc(frame, False)
            self.assertEqual(decoded.shape, source.shape)
            self.assertLess(np.abs(decoded.astype(int) - source.astype(int)).mean(), 3.0)
            rgb = frame_to_hwc(frame, False, rgb=True)
            np.testing.assert_array_equal(rgb, decoded[..., ::-1])
            self.assertTrue(rgb.flags.c_contiguous)


if __name__ == "__main__":
    unittest.main()
