#!/usr/bin/env python3
"""Read Autolife camera frames directly from the shared-memory files.

The robot camera service and ``hand_camera_producer.py`` expose one latest
frame through two files in ``/dev/shm``.  This module contains the small,
ROS-free reader shared by the recorder and the optional ROS bridge.  Keeping
the reader here makes direct-SHM recording and topic recording use exactly the
same validation rules.
"""

from __future__ import annotations

import struct
import os
import stat
import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np

from camera_config import SHM_METADATA_FORMAT, CameraSpec

METADATA_SIZE = struct.calcsize(SHM_METADATA_FORMAT)
SHM2_MAGIC = b"SHM2"
SHM2_HEADER_FORMAT = "<4sIIIQQIIIIQIIQQQ"
SHM2_HEADER_SIZE = struct.calcsize(SHM2_HEADER_FORMAT)
SHM2_METADATA_READ_SIZE = 128
EPOCH_NS_MIN = 946684800 * 1_000_000_000
EPOCH_NS_MAX = 4102444800 * 1_000_000_000
PIXEL_FORMAT_BGR = 1
PIXEL_FORMAT_DEPTH16 = 2
PIXEL_FORMAT_RGB = 3
PIXEL_FORMAT_MJPEG = 4


@dataclass(frozen=True)
class ShmFrame:
    """One complete frame copied out of a camera's shared-memory files."""

    timestamp_ns: int
    width: int
    height: int
    channels: int
    pixel_format: int
    byte_count: int
    data: bytes


def _decode_shm2_metadata(raw: bytes) -> tuple[int, ...] | None:
    """Convert the SDK's SHM2 header to the collector's common tuple.

    SHM2 stores two or more image slots in one data region. The returned
    extension fields carry the byte offset and generation identifiers needed
    to read the active slot and detect a producer update during the copy.
    """

    if len(raw) < SHM2_HEADER_SIZE:
        return None
    (
        magic,
        version,
        slot_count,
        active_slot,
        publish_sequence,
        frame_id,
        width,
        height,
        channels,
        depth_bytes,
        max_frame_size,
        fourcc,
        _flags,
        slot_stride,
        pts_ns,
        publish_epoch_ns,
    ) = struct.unpack(SHM2_HEADER_FORMAT, raw[:SHM2_HEADER_SIZE])
    if (
        magic != SHM2_MAGIC
        or version != 2
        or slot_count < 2
        or active_slot >= slot_count
        or slot_stride <= 0
        or max_frame_size <= 0
        or max_frame_size > slot_stride
    ):
        return None

    fourcc_text = struct.pack("<I", fourcc).rstrip(b"\0")
    if channels == 1 and depth_bytes == 2 and fourcc_text in {b"Z16", b"D16"}:
        pixel_format = PIXEL_FORMAT_DEPTH16
    elif channels == 3 and depth_bytes == 1 and fourcc_text == b"BGR":
        pixel_format = PIXEL_FORMAT_BGR
    elif channels == 3 and depth_bytes == 1 and fourcc_text == b"RGB":
        pixel_format = PIXEL_FORMAT_RGB
    elif fourcc_text in {b"MJPG", b"JPEG"}:
        pixel_format = PIXEL_FORMAT_MJPEG
    else:
        return None

    timestamp_ns = publish_epoch_ns or pts_ns or frame_id
    data_offset = active_slot * slot_stride
    return (
        timestamp_ns,
        width,
        height,
        channels,
        pixel_format,
        max_frame_size,
        data_offset,
        publish_sequence,
        frame_id,
    )


def _decode_metadata(raw: bytes) -> tuple[int, ...] | None:
    if raw.startswith(SHM2_MAGIC):
        return _decode_shm2_metadata(raw)
    if len(raw) < METADATA_SIZE:
        return None
    return struct.unpack(SHM_METADATA_FORMAT, raw[:METADATA_SIZE])


def read_shm_metadata(spec: CameraSpec) -> tuple[int, ...] | None:
    try:
        with open(spec.meta_path, "rb") as stream:
            raw = stream.read(SHM2_METADATA_READ_SIZE)
        return _decode_metadata(raw)
    except (FileNotFoundError, OSError, struct.error):
        return None


def _frame_extent(metadata: tuple[int, ...] | None) -> tuple[int, int] | None:
    if metadata is None or len(metadata) not in (6, 9):
        return None
    _, width, height, channels, pixel_format, buffer_size = metadata[:6]
    data_offset = metadata[6] if len(metadata) > 6 else 0
    if width <= 0 or height <= 0 or channels <= 0 or buffer_size <= 0 or data_offset < 0:
        return None
    if pixel_format == PIXEL_FORMAT_MJPEG:
        expected_size = buffer_size
    else:
        bytes_per_pixel = 2 if pixel_format == PIXEL_FORMAT_DEPTH16 else 1
        expected_size = width * height * channels * bytes_per_pixel
    if expected_size <= 0 or buffer_size < expected_size:
        return None
    return data_offset, expected_size


def read_shm_frame(
    spec: CameraSpec,
    metadata: tuple[int, ...] | None = None,
) -> ShmFrame | None:
    """Read one internally consistent metadata/image pair.

    Metadata and image bytes are separate files, so a producer can update the
    pair between our reads.  Reading metadata again after the image catches
    that race and prevents a frame from being built from mixed generations.
    """

    first = metadata if metadata is not None else read_shm_metadata(spec)
    extent = _frame_extent(first)
    if extent is None:
        return None
    data_offset, expected_size = extent
    try:
        with open(spec.buffer_path, "rb") as stream:
            # Validate before allocating: a torn/corrupt header must not ask
            # Python for a multi-gigabyte buffer or read outside the SHM file.
            if expected_size > os.fstat(stream.fileno()).st_size - data_offset:
                return None
            stream.seek(data_offset)
            data = stream.read(expected_size)
    except (FileNotFoundError, OSError):
        return None
    if len(data) != expected_size:
        return None

    second = read_shm_metadata(spec)
    if second != first:
        return None
    timestamp_ns, width, height, channels, pixel_format = first[:5]
    return ShmFrame(timestamp_ns, width, height, channels, pixel_format, expected_size, data)


class _ReadOnlyRegion:
    """One descriptor, rebound when a producer replaces its SHM inode."""

    def __init__(self, path: str) -> None:
        self.path = path
        self.fd: int | None = None
        self.identity: tuple[int, int] | None = None

    def current(self) -> os.stat_result:
        current = os.stat(self.path)
        identity = (current.st_dev, current.st_ino)
        if self.fd is None or self.identity != identity:
            self.close()
            # O_NONBLOCK also makes an accidentally configured FIFO fail our
            # regular-file check without hanging the capture callback.
            self.fd = os.open(self.path, os.O_RDONLY | os.O_CLOEXEC | os.O_NONBLOCK)
            opened = os.fstat(self.fd)
            if not stat.S_ISREG(opened.st_mode):
                self.close()
                raise OSError("camera SHM must be a regular file")
            self.identity = (opened.st_dev, opened.st_ino)
            if self.identity != identity:
                # Replacement between stat and open: retry next poll rather
                # than combine this inode with metadata from another producer.
                self.close()
                raise OSError("camera SHM replaced while opening")
            current = opened
        return current

    def is_current(self) -> bool:
        current = os.stat(self.path)
        return self.identity == (current.st_dev, current.st_ino)

    def close(self) -> None:
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
        self.identity = None


class ShmCameraReader:
    """Reusable read-only SHM reader with at most two open descriptors.

    Keep one reader per camera and call ``close`` when its capture worker exits
    (or use a context manager). Metadata is freshly read on *every* poll; no
    image or decoded-frame cache is kept. File replacement is checked on each
    read, and both metadata generation and inode identity are checked after
    copying an image. Missing/replaced/truncated files are retried on a later
    call, without restarting a producer or returning stale cached pixels.

    ``pread`` is intentional: unlike an mmap access, it cannot SIGBUS the
    process if a producer truncates its file during restart. Like the legacy
    functions, this depends on the producer's publication protocol: metadata
    equality alone cannot detect bytes overwritten in an active slot *before*
    the producer publishes its next generation. It is not JPEG repair and does
    not hide decoder warnings or change synchronization tolerances.
    """

    def __init__(self, spec: CameraSpec) -> None:
        self.spec = spec
        self._metadata = _ReadOnlyRegion(spec.meta_path)
        self._buffer = _ReadOnlyRegion(spec.buffer_path)
        self._lock = threading.RLock()
        self._closed = False
        self._last_metadata: tuple[int, ...] | None = None
        self._last_metadata_identity: tuple[int, int] | None = None

    def read_metadata(self) -> tuple[int, ...] | None:
        with self._lock:
            if self._closed:
                return None
            try:
                self._metadata.current()
                raw = os.pread(self._metadata.fd, SHM2_METADATA_READ_SIZE, 0)
                result = _decode_metadata(raw)
                if not self._metadata.is_current():
                    result = None
            except (OSError, struct.error):
                self._metadata.close()
                result = None
            self._last_metadata = result
            self._last_metadata_identity = self._metadata.identity if result is not None else None
            return result

    def read_frame(self, metadata: tuple[int, ...] | None = None) -> ShmFrame | None:
        with self._lock:
            if self._closed:
                return None
            first = metadata if metadata is not None else self.read_metadata()
            extent = _frame_extent(first)
            if extent is None:
                return None
            # Callers normally pass the preceding read_metadata() result. Also
            # support external metadata without accepting an unverified inode.
            if first != self._last_metadata and self.read_metadata() != first:
                return None
            metadata_identity = self._last_metadata_identity
            data_offset, expected_size = extent
            try:
                current = self._buffer.current()
                if expected_size > current.st_size - data_offset:
                    return None
                data = os.pread(self._buffer.fd, expected_size, data_offset)
                if len(data) != expected_size:
                    return None
                second = self.read_metadata()
                if (second != first
                        or self._last_metadata_identity != metadata_identity
                        or not self._buffer.is_current()):
                    return None
            except OSError:
                self._buffer.close()
                return None
            timestamp_ns, width, height, channels, pixel_format = first[:5]
            return ShmFrame(timestamp_ns, width, height, channels, pixel_format, expected_size, data)

    def close(self) -> None:
        with self._lock:
            self._metadata.close()
            self._buffer.close()
            self._last_metadata = None
            self._last_metadata_identity = None
            self._closed = True

    def __enter__(self) -> "ShmCameraReader":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()


def frame_to_hwc(frame: ShmFrame, is_depth: bool, *, rgb: bool = False) -> np.ndarray:
    """Decode one validated SHM frame into a contiguous HWC array.

    RGB producers expose BGR bytes. Set ``rgb=True`` for LeRobot dataset input;
    leave it false for consumers that perform their own BGR-to-RGB conversion.
    Depth is always little-endian uint16 millimetres with one channel.
    """

    if is_depth:
        if frame.pixel_format != PIXEL_FORMAT_DEPTH16 or frame.channels != 1:
            raise ValueError(
                f"depth camera requires depth16/channels=1, got {frame.pixel_format}/{frame.channels}"
            )
        return np.frombuffer(frame.data, dtype="<u2").reshape(frame.height, frame.width, 1).copy()

    if frame.pixel_format == PIXEL_FORMAT_MJPEG:
        image = cv2.imdecode(np.frombuffer(frame.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("OpenCV could not decode the MJPEG frame")
        if image.shape[:2] != (frame.height, frame.width):
            raise ValueError(
                f"MJPEG dimensions are {image.shape[1]}x{image.shape[0]}, "
                f"metadata declares {frame.width}x{frame.height}"
            )
        # OpenCV's native color conversion avoids a slow NumPy negative-stride
        # copy of every pixel; output values and contiguous RGB layout match.
        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB) if rgb else image

    if frame.pixel_format not in (PIXEL_FORMAT_BGR, PIXEL_FORMAT_RGB) or frame.channels != 3:
        raise ValueError(
            f"color camera requires BGR, RGB, or MJPEG, "
            f"got {frame.pixel_format}/{frame.channels}"
        )
    image = np.frombuffer(frame.data, dtype=np.uint8).reshape(frame.height, frame.width, 3)
    source_is_rgb = frame.pixel_format == PIXEL_FORMAT_RGB
    needs_swap = source_is_rgb != rgb
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB) if needs_swap else image


def shm_timestamp_sec(timestamp_ns: int, received_sec: float | None = None) -> float:
    """Return SHM epoch seconds, falling back for monotonic/device clocks."""

    if EPOCH_NS_MIN <= timestamp_ns <= EPOCH_NS_MAX:
        return timestamp_ns * 1e-9
    return time.time() if received_sec is None else received_sec
