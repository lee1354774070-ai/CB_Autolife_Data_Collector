"""Bounded, lossless temporary-image writer used only by the MZJ recorder.

Capture spends CPU on copying pixels, not PNG compression. Final video encoding
still runs through LeRobot at save time. No ROS publishers or hardware APIs.
"""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import cv2
import numpy as np
from PIL import Image
from lerobot.datasets.image_writer import image_array_to_pil_image, save_kwargs_for_path


def write_capture_image(image, fpath):
    path = Path(fpath)
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
    def __init__(self, num_threads=2, max_pending=64):
        if num_threads < 0 or max_pending < 1:
            raise ValueError("image-writer threads must be nonnegative and queue capacity positive")
        self._pool = (ThreadPoolExecutor(max_workers=num_threads, thread_name_prefix="dagger_image")
                      if num_threads else None)
        self._pending = deque()
        self._max_pending = max_pending
        self._stopped = False

    @property
    def pending_count(self):
        return len(self._pending)

    def save_image(self, image, fpath, compress_level=1):
        if self._stopped:
            raise RuntimeError("image writer is stopped")
        while self._pending and self._pending[0].done():
            self._pending.popleft().result()
        if len(self._pending) >= self._max_pending:
            raise RuntimeError(f"temporary image queue full ({self._max_pending}); storage cannot keep up")
        if hasattr(image, "cpu"):
            image = image.cpu().numpy()
        if self._pool is None:
            write_capture_image(image, fpath)
        else:
            self._pending.append(self._pool.submit(write_capture_image, image, fpath))

    def wait_until_done(self):
        error = None
        while self._pending:
            try:
                self._pending.popleft().result()
            except Exception as exc:
                if error is None:
                    error = exc
        if error is not None:
            raise error

    def stop(self):
        if self._stopped:
            return
        self._stopped = True
        if self._pool is not None:
            self._pool.shutdown(wait=True)
        self.wait_until_done()
