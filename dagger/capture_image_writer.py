"""Bounded, lossless temporary-image writer shared by all recording modes.

All recorder modes use uncompressed PNG to reduce capture CPU. The optional
compressed writer remains available to callers. Final video encoding runs
through LeRobot at save time. Every
worker failure propagates to the recorder. No ROS publishers or hardware APIs.
"""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import cv2
import numpy as np
from PIL import Image
from lerobot.datasets.image_writer import image_array_to_pil_image, save_kwargs_for_path


def write_compressed_image(image, fpath):
    # LeRobot's convenience write_image logs and swallows exceptions in some
    # releases. Do not use it: the recorder must see every failed write.
    img = image if isinstance(image, Image.Image) else image_array_to_pil_image(image)
    img.save(fpath, **save_kwargs_for_path(Path(fpath), compress_level=1))


def write_capture_image(image, fpath):
    path = Path(fpath)
    # Normal capture is already uint8 RGB. Avoid the former NumPy -> PIL ->
    # NumPy round trip (two full-frame copies) before OpenCV writes lossless PNG.
    # Match LeRobot's CHW precedence and keep its conversion for float/depth.
    rgb = None
    if (isinstance(image, np.ndarray) and image.dtype == np.uint8
            and image.ndim == 3 and image.shape[0] != 1 and image.shape[-1] != 1):
        if image.shape[0] == 3:
            rgb = image.transpose(1, 2, 0)
        elif image.shape[-1] == 3:
            rgb = image
    elif isinstance(image, Image.Image) and image.mode == "RGB":
        rgb = np.asarray(image)
    if (rgb is not None and path.suffix.lower() == ".png"
            and hasattr(cv2, "IMWRITE_PNG_FILTER")):
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if not cv2.imwrite(str(path), bgr, [cv2.IMWRITE_PNG_COMPRESSION, 0,
                          cv2.IMWRITE_PNG_FILTER, cv2.IMWRITE_PNG_FILTER_NONE]):
            raise OSError(f"temporary image write failed: {path}")
        return
    img = image if isinstance(image, Image.Image) else image_array_to_pil_image(image)
    if path.suffix.lower() == ".png" and img.mode == "RGB" and hasattr(cv2, "IMWRITE_PNG_FILTER"):
        # LeRobot stores RGB; OpenCV's file writer takes BGR. Filter NONE and
        # compression 0 preserve every pixel while avoiding per-row heuristics.
        bgr = cv2.cvtColor(np.asarray(img), cv2.COLOR_RGB2BGR)
        if not cv2.imwrite(str(path), bgr, [cv2.IMWRITE_PNG_COMPRESSION, 0,
                          cv2.IMWRITE_PNG_FILTER, cv2.IMWRITE_PNG_FILTER_NONE]):
            raise OSError(f"temporary image write failed: {path}")
    else:
        # Older OpenCV remains supported; uint16 depth keeps raw TIFF storage.
        img.save(path, **save_kwargs_for_path(path, compress_level=0))


class CaptureImageWriter:
    """LeRobot image-writer interface; bounded pending work and visible errors."""
    def __init__(self, num_threads=2, max_pending=64, max_pending_mb=256, low_cpu=True):
        if num_threads < 0 or max_pending < 1 or max_pending_mb <= 0:
            raise ValueError("image-writer threads must be nonnegative and queue capacity positive")
        self._pool = (ThreadPoolExecutor(max_workers=num_threads, thread_name_prefix="collector-image")
                      if num_threads else None)
        self._pending = deque()
        self._max_pending = max_pending
        self._max_bytes = int(max_pending_mb * 1024 ** 2)
        self._bytes = 0
        self._error = None
        self._write = write_capture_image if low_cpu else write_compressed_image
        self._stopped = False

    @property
    def pending_count(self):
        return len(self._pending)

    def save_image(self, image, fpath, compress_level=1):
        if self._stopped:
            raise RuntimeError("image writer is stopped")
        self.check()
        if len(self._pending) >= self._max_pending:
            raise RuntimeError(f"temporary image queue full ({self._max_pending}); storage cannot keep up")
        if hasattr(image, "cpu"):
            image = image.cpu().numpy()
        size = (image.width * image.height * len(image.getbands()) * 4
                if isinstance(image, Image.Image) else int(getattr(image, 'nbytes', 0)))
        if self._bytes + size > self._max_bytes:
            raise RuntimeError('temporary image queue memory limit exceeded')
        if self._pool is None:
            self._write(image, fpath)
        else:
            self._pending.append((self._pool.submit(self._write, image, fpath), size))
            self._bytes += size

    def check(self):
        """Observe failures even behind a slow first write; never block a tick."""
        remaining = deque()
        while self._pending:
            future, size = self._pending.popleft()
            if not future.done():
                remaining.append((future, size))
                continue
            self._bytes -= size
            try:
                future.result()
            except Exception as exc:
                self._error = self._error or exc
        self._pending = remaining
        if self._error is not None:
            raise self._error

    def wait_until_done(self):
        error = self._error
        while self._pending:
            future, size = self._pending.popleft()
            try:
                future.result()
            except Exception as exc:
                if error is None:
                    error = exc
            finally:
                self._bytes -= size
        self._error = error
        if error is not None:
            raise error

    def stop(self):
        if self._stopped:
            return
        self._stopped = True
        if self._pool is not None:
            self._pool.shutdown(wait=True)
        self.wait_until_done()
