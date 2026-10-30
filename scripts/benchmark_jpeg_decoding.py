#!/usr/bin/env python3
"""Offline OpenCV/nvJPEG comparison including RGB layout and GPU-to-CPU transfer.

Use --jpeg with representative camera JPEG files for deployment decisions.
Synthetic inputs exercise the implementation but do not establish robot load.
"""
import argparse
import json
from pathlib import Path
import platform
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import cv2
import numpy as np

from jpeg_decoding import NvJpegDecoder


def cpu_decode(frames):
    return [np.ascontiguousarray(cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)[..., ::-1])
            for data, _, _ in frames]


def measure(decode, frames, iterations, hz):
    latency = []
    cpu_start, wall_start = time.process_time(), time.perf_counter()
    next_tick = wall_start
    late = 0
    for _ in range(iterations):
        if hz:
            time.sleep(max(0, next_tick - time.perf_counter()))
        started = time.perf_counter()
        images = decode(frames)
        elapsed = time.perf_counter() - started
        latency.append(elapsed * 1000)
        del images
        if hz:
            next_tick += 1 / hz
            late += time.perf_counter() > next_tick
    wall = time.perf_counter() - wall_start
    cpu_ms = (time.process_time() - cpu_start) * 1000 / iterations
    return dict(iterations=iterations, cameras=len(frames), wall_seconds=wall,
                cpu_ms_per_batch=cpu_ms, estimated_one_core_percent_at_30hz=cpu_ms * 3,
                mean_ms=float(np.mean(latency)), p50_ms=float(np.percentile(latency, 50)),
                p95_ms=float(np.percentile(latency, 95)), p99_ms=float(np.percentile(latency, 99)),
                max_ms=max(latency), missed_deadlines=late if hz else None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--jpeg', type=Path, nargs='+', help='Read-only local camera JPEG samples; one file per camera')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--cameras', type=int, default=2)
    parser.add_argument('--iterations', type=int, default=300)
    parser.add_argument('--runs', type=int, default=3)
    parser.add_argument('--hz', type=float, default=0, help='Optional paced batch rate; 0 runs at full throughput')
    parser.add_argument('--device', default='cuda:0')
    parser.add_argument('--output', type=Path, help='New JSON report path; existing reports are never overwritten')
    args = parser.parse_args()
    if min(args.width, args.height, args.cameras, args.iterations, args.runs) <= 0 or not np.isfinite(args.hz) or args.hz < 0:
        parser.error('Dimensions/counts must be positive; hz must be finite and nonnegative')
    if args.output and args.output.exists():
        parser.error('Output already exists')
    import torch
    import torchvision
    cv2.setNumThreads(1)
    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    if args.jpeg:
        bitstreams = [p.read_bytes() for p in args.jpeg]
    else:
        rng = np.random.default_rng(20261029)
        bitstreams = []
        for index in range(args.cameras):
            # Textured, colored input instead of a trivially cheap solid image.
            image = rng.integers(0, 256, (args.height, args.width, 3), dtype=np.uint8)
            image = cv2.GaussianBlur(image, (5, 5), 1)
            cv2.rectangle(image, (20, 20), (args.width // 2, args.height // 2), (20, 50, 230), 8)
            cv2.putText(image, f'SYNTHETIC CAMERA {index}', (20, args.height // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, .7, (240, 180, 30), 2)
            ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 90])
            if not ok:
                raise RuntimeError('Synthetic JPEG encode failed')
            bitstreams.append(encoded.tobytes())
    frames = []
    for data in bitstreams:
        image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError('Invalid input JPEG')
        frames.append((data, image.shape[1], image.shape[0]))
    gpu = NvJpegDecoder(args.device)
    quality = []
    for cpu, cuda in zip(cpu_decode(frames), gpu.decode(frames), strict=True):
        diff = np.abs(cpu.astype(np.int16) - cuda.astype(np.int16))
        quality.append(dict(mean_absolute_error=float(diff.mean()),
                            p99_absolute_error=float(np.percentile(diff, 99)),
                            max_absolute_error=int(diff.max())))
    for _ in range(20):
        gpu.decode(frames)
        cpu_decode(frames)
    report = dict(source='camera_samples' if args.jpeg else 'synthetic_not_robot_validation',
                  platform=platform.platform(), torch=torch.__version__, torchvision=torchvision.__version__,
                  gpu=torch.cuda.get_device_name(args.device), cuda=torch.version.cuda, hz=args.hz,
                  dimensions=[dict(width=w, height=h, jpeg_bytes=len(data)) for data, w, h in frames],
                  quality_vs_opencv=quality, runs=[])
    for run in range(args.runs):
        result = {'run': run}
        methods = [('opencv', cpu_decode), ('nvjpeg', gpu.decode)]
        for name, method in methods if run % 2 == 0 else reversed(methods):
            result[name] = measure(method, frames, args.iterations, args.hz)
        report['runs'].append(result)
        print(json.dumps(result), flush=True)
    report['cuda_allocated_bytes'] = torch.cuda.memory_allocated(args.device)
    report['cuda_reserved_bytes'] = torch.cuda.memory_reserved(args.device)
    report['cuda_note'] = 'PyTorch counters exclude CUDA context and nvJPEG internal allocations.'
    if args.output:
        with args.output.open('x') as f:
            json.dump(report, f, indent=2, allow_nan=False)
            f.write('\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'runs'}, indent=2))


if __name__ == '__main__':
    main()
