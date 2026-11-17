"""GPU-preferred JPEG decode for the bounded images copied in one SHM poll.

The recorder still consumes CPU HWC uint8 RGB. Include the device-to-host copy
when measuring this backend; GPU-only timing is not capture latency. OpenCV
is the startup fallback. strict_cpu uses warning-fatal libjpeg without CUDA.
Importing this module does not initialize CUDA.
"""

import re
import ctypes
import hashlib
import json
import time
from pathlib import Path

import cv2
import numpy as np


class StrictCpuJpegDecoder:
    """Decode immutable camera bytes; every corrupt-data warning is fatal."""
    backend = 'strict_cpu'
    device = 'cpu'

    def __init__(self):
        self._library = ctypes.CDLL(str(Path(__file__).with_name('libstrictjpeg.so')))
        self._decode = self._library.strict_jpeg_decode
        self._decode.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_uint,
                                 ctypes.c_uint, ctypes.c_void_p, ctypes.c_size_t,
                                 ctypes.c_void_p, ctypes.c_size_t]
        self._decode.restype = ctypes.c_int
        ok, jpeg = cv2.imencode('.jpg', np.full((32, 32, 3), 120, np.uint8))
        if not ok or not np.all(self.decode([(jpeg.tobytes(), 32, 32)])[0] == 120):
            raise RuntimeError('Strict CPU JPEG startup probe failed')

    def decode(self, frames):
        images = []
        for data, width, height in frames:
            if not (0 < width <= 8192 and 0 < height <= 8192 and 0 < len(data) <= 32 * 1024**2):
                raise ValueError('JPEG input exceeds bounds')
            image = np.empty((height, width, 3), np.uint8)
            error = ctypes.create_string_buffer(256)
            source = ctypes.c_char_p(data)
            if self._decode(source, len(data), width, height, image.ctypes.data,
                            image.nbytes, error, len(error)):
                raise ValueError('Strict JPEG decode failed: ' + error.value.decode('utf-8', errors='replace'))
            images.append(image)
        return images


def save_jpeg_failure(output_dir, copied_frames, error):
    """Bounded immutable evidence, outside the dataset; never replace frames."""
    root = Path(output_dir).parent / 'jpeg_failures'
    root.mkdir(parents=True, exist_ok=True)
    if len(list(root.glob('event-*'))) >= 8:
        return {'diagnostic': 'event_limit_reached'}
    event = root / f'event-{time.time_ns()}'
    event.mkdir()
    records, total = [], 0
    for name, _, frame, _ in copied_frames:
        if frame.pixel_format != 4:
            continue
        total += len(frame.data)
        if total > 8 * 1024**2:
            break
        filename = f'{len(records):02d}.jpg'
        (event / filename).write_bytes(frame.data)
        records.append(dict(camera=name, file=filename, timestamp_ns=frame.timestamp_ns,
                            width=frame.width, height=frame.height, byte_count=len(frame.data),
                            sha256=hashlib.sha256(frame.data).hexdigest()))
    (event / 'failure.json').write_text(json.dumps(dict(error=str(error), frames=records), indent=2))
    return {'diagnostic': str(event)}


class NvJpegDecoder:
    def __init__(self, device='cuda:0'):
        import torch
        version = tuple(int(v) for v in (torch.version.cuda or '0.0').split('.')[:2])
        if version < (11, 6):
            raise RuntimeError('nvJPEG requires CUDA >= 11.6 to avoid older decoder memory leaks')
        if not torch.cuda.is_available():
            raise RuntimeError('CUDA is not available')
        if not re.fullmatch(r'cuda(?::\d+)?', device):
            raise ValueError('JPEG device must be cuda or cuda:<index>')
        self.device = torch.device(device)
        if self.device.index is None:
            self.device = torch.device('cuda', torch.cuda.current_device())
        if self.device.index >= torch.cuda.device_count():
            raise ValueError(f'JPEG device {self.device} is not visible to CUDA')
        from torchvision.io import decode_jpeg

        self.torch, self._decode_jpeg = torch, decode_jpeg
        # Initialize CUDA and exercise the actual codec before polling cameras.
        # Model inference is not involved. Actual camera dimensions are checked
        # on every decode, including the normal camera warmup period.
        ok, jpeg = cv2.imencode('.jpg', np.full((32, 32, 3), 120, np.uint8))
        if not ok:
            raise RuntimeError('Could not build JPEG startup probe')
        image = self.decode([(jpeg.tobytes(), 32, 32)])[0]
        if not np.all(image == 120):
            raise RuntimeError('nvJPEG startup probe returned incorrect pixels')

    def decode(self, frames):
        """Decode (JPEG bytes, width, height) entries in order; own every output.

        No accumulation between polls, stale-frame reuse, resizing or runtime
        CPU fallback. torchvision batches nvJPEG calls; only frames already
        copied by this poll participate, so another camera is never awaited.
        """
        if not frames:
            return []
        torch = self.torch
        with torch.inference_mode():
            # Small host bitstream copies avoid writable-buffer warnings and
            # keep the byte storage alive until nvJPEG finishes consuming it.
            inputs = [torch.from_numpy(np.frombuffer(data, dtype=np.uint8).copy())
                      for data, _, _ in frames]
            images = self._decode_jpeg(inputs, mode='RGB', device=self.device)
            if len(images) != len(frames):
                raise RuntimeError('nvJPEG returned an incomplete batch')
            result = []
            for image, (_, width, height) in zip(images, frames, strict=True):
                if image.dtype != torch.uint8 or tuple(image.shape) != (3, height, width):
                    raise ValueError(f'JPEG dimensions/type differ from SHM: {tuple(image.shape)} != (3, {height}, {width})')
                if image.device != self.device:
                    raise RuntimeError(f'nvJPEG returned {image.device}, expected {self.device}')
                # Synchronous copy is intentional: FIFO entries must be ready
                # for CPU writers, with independent storage for future polls.
                result.append(image.permute(1, 2, 0).contiguous().cpu().numpy())
        return result


def select_jpeg_decoder(backend, device, logger):
    if backend == 'strict_cpu':
        decoder = StrictCpuJpegDecoder()
        logger.info('JPEG decoder: strict_cpu; corrupt-data warnings are fatal, original timestamps preserved')
        return decoder
    if backend == 'opencv':
        logger.info('JPEG decoder: opencv (CPU, explicitly selected)')
        return None
    if backend not in ('nvjpeg', 'auto'):
        raise ValueError(f'Unknown JPEG decoder: {backend}')
    try:
        decoder = NvJpegDecoder(device)
    except Exception as exc:
        if backend == 'nvjpeg':
            raise RuntimeError(f'nvJPEG initialization failed: {exc}') from exc
        logger.warning(f'JPEG decoder: opencv (CPU startup fallback); nvJPEG unavailable: {exc}')
        return None
    logger.info(f'JPEG decoder: nvjpeg on {decoder.device}; batched decode with CPU RGB output')
    return decoder
