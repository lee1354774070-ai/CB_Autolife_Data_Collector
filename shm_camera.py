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


def read_shm_metadata(spec: CameraSpec) -> tuple[int, ...] | None:
    try:
        with open(spec.meta_path, "rb") as stream:
            raw = stream.read(SHM2_METADATA_READ_SIZE)
        if raw.startswith(SHM2_MAGIC):
            return _decode_shm2_metadata(raw)
        if len(raw) < METADATA_SIZE:
            return None
        return struct.unpack(SHM_METADATA_FORMAT, raw[:METADATA_SIZE])
    except (FileNotFoundError, OSError, struct.error):
        return None


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
    if first is None:
        return None
    timestamp_ns, width, height, channels, pixel_format, buffer_size = first[:6]
    data_offset = first[6] if len(first) > 6 else 0
    if width <= 0 or height <= 0 or channels <= 0 or buffer_size <= 0:
        return None

    if pixel_format == PIXEL_FORMAT_MJPEG:
        expected_size = buffer_size
    else:
        bytes_per_pixel = 2 if pixel_format == PIXEL_FORMAT_DEPTH16 else 1
        expected_size = width * height * channels * bytes_per_pixel
    if expected_size <= 0 or buffer_size < expected_size:
        return None
    try:
        with open(spec.buffer_path, "rb") as stream:
            stream.seek(data_offset)
            data = stream.read(expected_size)
    except (FileNotFoundError, OSError):
        return None
    if len(data) != expected_size:
        return None

    second = read_shm_metadata(spec)
    if second != first:
        return None
    return ShmFrame(timestamp_ns, width, height, channels, pixel_format, expected_size, data)


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
        return np.ascontiguousarray(image[..., ::-1] if rgb else image)

    if frame.pixel_format not in (PIXEL_FORMAT_BGR, PIXEL_FORMAT_RGB) or frame.channels != 3:
        raise ValueError(
            f"color camera requires BGR, RGB, or MJPEG, "
            f"got {frame.pixel_format}/{frame.channels}"
        )
    image = np.frombuffer(frame.data, dtype=np.uint8).reshape(frame.height, frame.width, 3)
    source_is_rgb = frame.pixel_format == PIXEL_FORMAT_RGB
    needs_swap = source_is_rgb != rgb
    return np.ascontiguousarray(image[..., ::-1] if needs_swap else image)


def shm_timestamp_sec(timestamp_ns: int, received_sec: float | None = None) -> float:
    """Return SHM epoch seconds, falling back for monotonic/device clocks."""

    if EPOCH_NS_MIN <= timestamp_ns <= EPOCH_NS_MAX:
        return timestamp_ns * 1e-9
    return time.time() if received_sec is None else received_sec
