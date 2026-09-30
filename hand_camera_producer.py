#!/usr/bin/env python3
"""Capture the two DICOTA hand cameras into Autolife shared memory.

The robot vision service cannot reliably decode these cameras' MJPEG streams.
This optional process forwards the camera's original JPEG bytes to *_jpeg SHM.
It never publishes decoded BGR or re-encodes images. Do not run it while the
vision service owns the same devices; the launcher checks device ownership.
"""

from __future__ import annotations

import os
import signal
import struct
import time
from types import FrameType

import cv2
import numpy as np

from camera_config import HAND_CAMERA_SPECS, SHM_METADATA_FORMAT, HandCameraSpec
from shm_camera import PIXEL_FORMAT_MJPEG

# SDK-compatible little-endian metadata layout: timestamp_ns (int64), then
# width, height, channels, pixel format, and image byte count (five int32s).
# Pixel format 4 identifies a compressed JPEG payload, not width*height*3 bytes.
RUNNING = True


def signal_handler(_signal_number: int, _frame: FrameType | None) -> None:
    """Request a clean exit so every V4L2 device is released in ``finally``."""

    global RUNNING
    print("\nShutdown requested.")
    RUNNING = False


class HandCameraProducer:
    """Own one V4L2 camera and publish its latest compressed frame."""

    def __init__(self, name: str, config: HandCameraSpec):
        self.name = name
        self.config = config
        self.capture: cv2.VideoCapture | None = None
        self.frame_interval = 1.0 / config.fps
        self.width, self.height = config.width, config.height

    def open(self) -> bool:
        """Open and validate the camera, returning false when it is unavailable."""

        capture = cv2.VideoCapture(self.config.device, cv2.CAP_V4L2)
        if not capture.isOpened():
            print(f"[{self.name}] ERROR: cannot open {self.config.device}")
            return False

        # MJPEG is required for 1280x720 at 30 FPS on these DICOTA cameras;
        # their uncompressed YUYV mode is limited to approximately 5 FPS.
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
        capture.set(cv2.CAP_PROP_FPS, self.config.fps)

        # Require raw capture: silently accepting decoded pixels here would
        # either corrupt the JPEG SHM contract or require expensive re-encoding.
        if not capture.set(cv2.CAP_PROP_CONVERT_RGB, 0):
            capture.release()
            print(f"[{self.name}] ERROR: V4L2 backend does not support raw MJPEG capture")
            return False
        jpeg = self._read_one(capture)
        image = None if jpeg is None else cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            capture.release()
            print(f"[{self.name}] ERROR: initial JPEG validation failed")
            return False

        self.capture = capture
        self.height, self.width = image.shape[:2]
        print(f"[{self.name}] Started JPEG {self.width}x{self.height} on {self.config.device}")
        return True

    @staticmethod
    def _read_one(capture: cv2.VideoCapture) -> bytes | None:
        """Read a raw 1xN V4L2 packet without decoding or changing its bytes."""

        ok, frame = capture.read()
        if not ok or frame is None or frame.dtype != np.uint8 or frame.ndim != 2 or frame.shape[0] != 1:
            return None
        jpeg = frame.tobytes()
        return jpeg if jpeg.startswith(b"\xff\xd8") else None

    def produce_once(self) -> bool:
        """Capture and publish one frame, returning whether publication succeeded."""

        if self.capture is None:
            return False
        image_bytes = self._read_one(self.capture)
        if image_bytes is None:
            print(f"[{self.name}] WARNING: frame read failed")
            return False

        # Publish the image first and metadata second.  Consumers use the
        # metadata timestamp to identify a new, already complete frame.
        self._atomic_write(self.config.buffer_path, image_bytes)
        metadata = struct.pack(
            SHM_METADATA_FORMAT,
            time.time_ns(),
            self.width,
            self.height,
            3,
            PIXEL_FORMAT_MJPEG,
            len(image_bytes),
        )
        self._atomic_write(self.config.meta_path, metadata)
        return True

    @staticmethod
    def _atomic_write(path: str, data: bytes) -> None:
        """Replace a SHM file atomically so readers never observe partial data."""

        temporary_path = f"{path}.tmp"
        with open(temporary_path, "wb") as stream:
            stream.write(data)
        os.replace(temporary_path, path)

    def close(self) -> None:
        """Release the camera device if it was opened successfully."""

        if self.capture is not None:
            self.capture.release()
            self.capture = None
        print(f"[{self.name}] Closed")


def main() -> None:
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print("=" * 50)
    print("  Hand Camera SHM Producer")
    print("=" * 50)

    producers: list[HandCameraProducer] = []
    for name, config in HAND_CAMERA_SPECS.items():
        producer = HandCameraProducer(name, config)
        if producer.open():
            producers.append(producer)
        else:
            print(f"[{name}] Skipped because the device is unavailable")

    if not producers:
        print("ERROR: no hand cameras are available.")
        raise SystemExit(1)

    print(f"Publishing {len(producers)} camera(s). Press Ctrl+C to stop.\n")
    last_capture = {producer.name: 0.0 for producer in producers}
    frame_counts = {producer.name: 0 for producer in producers}
    status_started = time.monotonic()

    try:
        while RUNNING:
            now = time.monotonic()
            for producer in producers:
                if now - last_capture[producer.name] < producer.frame_interval:
                    continue
                if producer.produce_once():
                    frame_counts[producer.name] += 1
                last_capture[producer.name] = now

            if now - status_started >= 8.0:
                elapsed = now - status_started
                for name, count in frame_counts.items():
                    print(f"[{name}] {count} frames, {count / elapsed:.1f} FPS")
                    frame_counts[name] = 0
                status_started = now
            time.sleep(0.001)
    finally:
        for producer in producers:
            producer.close()
        print("All hand cameras closed.")


if __name__ == "__main__":
    main()
