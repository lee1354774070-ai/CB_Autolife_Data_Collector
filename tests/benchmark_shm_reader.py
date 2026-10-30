#!/usr/bin/env python3
"""Synthetic SHM-reader/color-copy microbenchmark; never opens a real camera.

Run from the repository root with ``python tests/benchmark_shm_reader.py``.
Fixtures live in an isolated TemporaryDirectory and are removed afterwards.
Results measure cached file reads and color conversion, not end-to-end camera
delivery, hardware JPEG decode, robot control latency, or synchronization.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import statistics
import struct
import sys
import tempfile
import time

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from camera_config import CameraSpec, SHM_METADATA_FORMAT
from shm_camera import ShmCameraReader, read_shm_frame, read_shm_metadata


def benchmark(function, iterations, rounds):
    function()
    wall, cpu = [], []
    for _ in range(rounds):
        start_wall = time.perf_counter_ns()
        start_cpu = time.process_time_ns()
        for _ in range(iterations):
            function()
        cpu.append((time.process_time_ns() - start_cpu) / iterations / 1000)
        wall.append((time.perf_counter_ns() - start_wall) / iterations / 1000)
    return {'median_wall_us': statistics.median(wall),
            'median_process_cpu_us': statistics.median(cpu)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--iterations', type=int, default=1500)
    parser.add_argument('--rounds', type=int, default=5)
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--temp-root', default=None,
                        help='Optional fixture parent such as /dev/shm; never a camera file')
    args = parser.parse_args()
    if min(args.iterations, args.rounds, args.width, args.height) < 1:
        parser.error('dimensions and repetition counts must be positive')
    cv2.setNumThreads(1)
    pixels = np.random.default_rng(17).integers(
        0, 256, size=(args.height, args.width, 3), dtype=np.uint8)
    np.testing.assert_array_equal(cv2.cvtColor(pixels, cv2.COLOR_BGR2RGB), pixels[..., ::-1])
    with tempfile.TemporaryDirectory(prefix='collector-shm-synthetic-', dir=args.temp_root) as directory:
        root = Path(directory)
        spec = CameraSpec('synthetic', str(root / 'meta'), str(root / 'buffer'), '/unused', 'synthetic')
        Path(spec.buffer_path).write_bytes(pixels.tobytes())
        Path(spec.meta_path).write_bytes(struct.pack(
            SHM_METADATA_FORMAT, 1_700_000_000_000_000_000,
            args.width, args.height, 3, 1, pixels.size))
        with ShmCameraReader(spec) as reader:
            assert reader.read_frame() == read_shm_frame(spec)
            results = {
                'kind': 'synthetic_cached_file_and_cpu_color_conversion_only',
                'shape': list(pixels.shape), 'opencv_threads': cv2.getNumThreads(),
                'iterations': args.iterations, 'rounds': args.rounds,
                'metadata_stateless': benchmark(lambda: read_shm_metadata(spec), args.iterations, args.rounds),
                'metadata_persistent': benchmark(reader.read_metadata, args.iterations, args.rounds),
                'frame_stateless': benchmark(lambda: read_shm_frame(spec), args.iterations, args.rounds),
                'frame_persistent': benchmark(reader.read_frame, args.iterations, args.rounds),
                'rgb_numpy_stride_copy': benchmark(lambda: np.ascontiguousarray(pixels[..., ::-1]),
                                                   args.iterations, args.rounds),
                'rgb_native_conversion': benchmark(lambda: cv2.cvtColor(pixels, cv2.COLOR_BGR2RGB),
                                                   args.iterations, args.rounds),
            }
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
