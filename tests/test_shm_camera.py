import os
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
    ShmCameraReader,
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

    def test_native_color_conversion_preserves_rgb_and_all_pixels(self):
        pixels = np.arange(3 * 17 * 23, dtype=np.uint8).reshape(17, 23, 3)
        for pixel_format, source_rgb in ((1, False), (3, True)):
            frame = ShmFrame(1, 23, 17, 3, pixel_format, pixels.size, pixels.tobytes())
            for rgb in (False, True):
                actual = frame_to_hwc(frame, False, rgb=rgb)
                expected = pixels if rgb == source_rgb else pixels[..., ::-1]
                np.testing.assert_array_equal(actual, expected)
                self.assertTrue(actual.flags.c_contiguous)

    def test_invalid_jpeg_is_still_rejected(self):
        data = b'not a JPEG'
        frame = ShmFrame(1, 10, 8, 3, PIXEL_FORMAT_MJPEG, len(data), data)
        with self.assertRaisesRegex(ValueError, 'could not decode'):
            frame_to_hwc(frame, False, rgb=True)

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


class PersistentShmReaderTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        self.spec = CameraSpec('test', str(root / 'meta'), str(root / 'buffer'), '/test', 'test')
        self.header = (1_700_000_000_000_000_000, 2, 1, 3, 1, 6)
        self.publish(self.header, b'abcdef')
        self.reader = ShmCameraReader(self.spec)
        self.addCleanup(self.reader.close)

    def publish(self, header, data, *, replace=False):
        for path, payload in ((self.spec.buffer_path, data),
                              (self.spec.meta_path, struct.pack(SHM_METADATA_FORMAT, *header))):
            target = Path(path + '.new') if replace else Path(path)
            target.write_bytes(payload)
            if replace:
                target.replace(path)

    def test_descriptor_reuse_with_fresh_metadata_and_no_image_cache(self):
        with patch('shm_camera.os.open', wraps=os.open) as opened:
            for sequence in range(5):
                header = (self.header[0] + sequence, *self.header[1:])
                data = bytes([sequence]) * 6
                self.publish(header, data)
                metadata = self.reader.read_metadata()
                frame = self.reader.read_frame(metadata)
                self.assertEqual(frame.timestamp_ns, header[0])
                self.assertEqual(frame.data, data)
            self.assertEqual(opened.call_count, 2)

    def test_atomic_replacement_reconnects_both_descriptors(self):
        with patch('shm_camera.os.open', wraps=os.open) as opened:
            self.assertEqual(self.reader.read_frame().data, b'abcdef')
            next_header = (self.header[0] + 1, *self.header[1:])
            self.publish(next_header, b'newnew', replace=True)
            frame = self.reader.read_frame()
            self.assertEqual(frame.timestamp_ns, next_header[0])
            self.assertEqual(frame.data, b'newnew')
            self.assertEqual(opened.call_count, 4)

    def test_restart_with_missing_files_then_recreation_recovers(self):
        self.assertIsNotNone(self.reader.read_frame())
        Path(self.spec.meta_path).unlink()
        Path(self.spec.buffer_path).unlink()
        self.assertIsNone(self.reader.read_metadata())
        self.assertIsNone(self.reader.read_frame(self.header))
        self.publish(self.header, b'newnew')
        self.assertEqual(self.reader.read_frame().data, b'newnew')

    def test_truncated_buffer_is_rejected_then_recovers_without_mmap(self):
        self.assertIsNotNone(self.reader.read_frame())
        Path(self.spec.buffer_path).write_bytes(b'a')
        self.assertIsNone(self.reader.read_frame())
        Path(self.spec.buffer_path).write_bytes(b'uvwxyz')
        self.assertEqual(self.reader.read_frame().data, b'uvwxyz')

    def test_rejects_generation_change_while_copying(self):
        pread = os.pread
        def changing_read(fd, count, offset):
            data = pread(fd, count, offset)
            if fd == self.reader._buffer.fd:
                new_header = (self.header[0] + 1, *self.header[1:])
                Path(self.spec.meta_path).write_bytes(struct.pack(SHM_METADATA_FORMAT, *new_header))
            return data
        with patch('shm_camera.os.pread', side_effect=changing_read):
            self.assertIsNone(self.reader.read_frame())

    def test_rejects_metadata_inode_change_even_with_same_timestamp(self):
        pread = os.pread
        def replacing_read(fd, count, offset):
            data = pread(fd, count, offset)
            if fd == self.reader._buffer.fd:
                path = Path(self.spec.meta_path + '.new')
                path.write_bytes(struct.pack(SHM_METADATA_FORMAT, *self.header))
                path.replace(self.spec.meta_path)
            return data
        with patch('shm_camera.os.pread', side_effect=replacing_read):
            self.assertIsNone(self.reader.read_frame())

    def test_rejects_buffer_inode_change_during_copy(self):
        pread = os.pread
        def replacing_read(fd, count, offset):
            data = pread(fd, count, offset)
            if fd == self.reader._buffer.fd:
                path = Path(self.spec.buffer_path + '.new')
                path.write_bytes(b'newnew')
                path.replace(self.spec.buffer_path)
            return data
        with patch('shm_camera.os.pread', side_effect=replacing_read):
            self.assertIsNone(self.reader.read_frame())
        self.assertEqual(self.reader.read_frame().data, b'newnew')

    def test_short_read_during_truncation_is_rejected(self):
        pread = os.pread
        def short_read(fd, count, offset):
            if fd == self.reader._buffer.fd:
                return b'a'
            return pread(fd, count, offset)
        with patch('shm_camera.os.pread', side_effect=short_read):
            self.assertIsNone(self.reader.read_frame())

    def test_oversize_metadata_cannot_allocate_an_oversize_buffer(self):
        malformed = (self.header[0], 10**9, 10**9, 3, 1, 2**31 - 1)
        Path(self.spec.meta_path).write_bytes(struct.pack(SHM_METADATA_FORMAT, *malformed))
        with patch('shm_camera.os.pread', wraps=os.pread) as read:
            self.assertIsNone(self.reader.read_frame())
            self.assertTrue(all(call.args[1] <= 128 for call in read.call_args_list))

    def test_context_manager_closes_exactly_two_fds_and_does_not_reopen(self):
        with patch('shm_camera.os.close', wraps=os.close) as closed:
            with self.reader as reader:
                self.assertIsNotNone(reader.read_frame())
            self.assertEqual(closed.call_count, 2)
            reader.close()
            self.assertIsNone(reader.read_metadata())
            self.assertIsNone(reader.read_frame())
            self.assertEqual(closed.call_count, 2)

    def test_non_regular_file_does_not_block(self):
        Path(self.spec.meta_path).unlink()
        os.mkfifo(self.spec.meta_path)
        self.assertIsNone(self.reader.read_metadata())

    def test_shm2_slot_and_generation_validation_matches_stateless_api(self):
        def write_header(slot, sequence):
            Path(self.spec.meta_path).write_bytes(struct.pack(
                SHM2_HEADER_FORMAT, b'SHM2', 2, 2, slot, sequence, sequence // 2,
                2, 1, 3, 1, 6, struct.unpack('<I', b'BGR\0')[0], 0, 6, 0, self.header[0],
            ))
        Path(self.spec.buffer_path).write_bytes(b'oldoldnewnew')
        write_header(1, 42)
        metadata = self.reader.read_metadata()
        self.assertEqual(self.reader.read_frame(metadata), read_shm_frame(self.spec, metadata))
        self.assertEqual(self.reader.read_frame(metadata).data, b'newnew')
        write_header(0, 44)
        self.assertIsNone(self.reader.read_frame(metadata))
        self.assertEqual(self.reader.read_frame().data, b'oldold')


if __name__ == "__main__":
    unittest.main()
