#!/usr/bin/env python3
"""Synthetic save-time RGB benchmark; never opens cameras, ROS, or robot IPC.

CPU baseline is FFmpeg/libx264 veryfast, NOT LeRobot's complete save pipeline.
The same strict frame/color checks run for every backend. QP values across
different codecs/backends do not guarantee equivalent quality. No GPU inference
or training is run; nvenc/vaapi uses the requested device's video encoder only.
"""

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import resource
import shutil
import statistics
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from video_encoding import HardwareVideoEncoder, probe_encoder


def cpu_seconds():
    return sum(usage.ru_utime + usage.ru_stime for usage in
               (resource.getrusage(resource.RUSAGE_SELF), resource.getrusage(resource.RUSAGE_CHILDREN)))


def frame_pattern(width, height, index, camera):
    import numpy as np
    y, x = np.indices((height, width), dtype=np.int32)
    return np.stack(((x + index * 3 + camera * 35) % 256,
                     (y + index * 2 + camera * 55) % 256,
                     ((x // 16 + y // 16 + index // 4) % 2) * 120 + 45), axis=-1).astype(np.uint8)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="New directory; refuses overwrite. Synthetic only.")
    parser.add_argument("--backend", choices=("cpu", "nvenc", "vaapi"), required=True)
    parser.add_argument("--compare-cpu", action="store_true", help="Also measure FFmpeg/libx264 veryfast, not native LeRobot.")
    parser.add_argument("--ffmpeg", default="ffmpeg")
    parser.add_argument("--device", default="/dev/dri/renderD128", help="VAAPI render device; NVENC uses the configured CUDA visibility.")
    parser.add_argument("--frames", type=int, default=90)
    parser.add_argument("--cameras", type=int, default=3)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--threads", type=int, default=1)
    parser.add_argument("--qp", type=int, default=23)
    parser.add_argument("--gop", type=int, default=2)
    parser.add_argument("--timeout", type=float, default=120, help="Encode + full decode budget per video in seconds.")
    parser.add_argument("--max-input-gib", type=float, default=1, help="Reject larger synthetic PNG input estimates before allocation.")
    args = parser.parse_args()
    if min(args.frames, args.cameras, args.fps, args.width, args.height, args.repeats, args.threads, args.gop) <= 0:
        parser.error("counts, dimensions, FPS and threads must be positive")
    if args.width < 24 or args.height < 4 or args.width % 2 or args.height % 2 or not 0 <= args.qp <= 51:
        parser.error("dimensions must be even, at least 24x4; QP must be in [0,51]")
    if not math.isfinite(args.timeout) or args.timeout <= 0 or not math.isfinite(args.max_input_gib) or args.max_input_gib <= 0:
        parser.error("timeout and max-input-gib must be finite and positive")
    raw_bytes = args.width * args.height * 3 * args.frames * args.cameras
    if raw_bytes * 1.1 > args.max_input_gib * 1024 ** 3:
        parser.error("synthetic input exceeds --max-input-gib; reduce size/count or explicitly raise the limit")
    ffmpeg = shutil.which(args.ffmpeg)
    if not ffmpeg:
        parser.error("FFmpeg executable not found; pass --ffmpeg /absolute/path/to/ffmpeg")
    if args.output.exists():
        parser.error("output directory already exists; choose a new path")
    backends = [args.backend]
    if args.compare_cpu and args.backend != "cpu":
        backends.insert(0, "cpu")
    encoders = {backend: HardwareVideoEncoder(backend, ffmpeg, device=args.device, qp=args.qp,
                                              gop=args.gop, threads=args.threads, timeout=args.timeout)
                for backend in backends}
    # No auto fallback: benchmark requested hardware or fail explicitly.
    for encoder in encoders.values():
        probe_encoder(encoder, {(args.width, args.height)}, args.fps)
    from PIL import Image
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "NOT_TRAINING.txt").write_text(
        "Synthetic encoder benchmark only. No robot recordings; do not use for training.\n")
    inputs = []
    start = time.monotonic()
    for camera in range(args.cameras):
        images = args.output / f"input_{camera}"
        images.mkdir()
        for index in range(args.frames):
            Image.fromarray(frame_pattern(args.width, args.height, index, camera)).save(
                images / f"frame-{index:06d}.png", compress_level=0)
        inputs.append(images)
    report = {
        "scope": "synthetic sequential RGB save encoding only; NOT collection, VR latency or training",
        "limitations": ["CPU baseline: FFmpeg libx264 veryfast, not native LeRobot save",
                        "PNG decode, RGB conversion, full readback and depth remain CPU work",
                        "No live SHM JPEG decode, disk durability, depth or Parquet measured",
                        "QP across backends is not a quality-equivalence claim",
                        "Hardware device utilization is not sampled; no peak VR-load test"],
        "args": {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()},
        "encoders": {key: asdict(value) for key, value in encoders.items()},
        "prepare_seconds": time.monotonic() - start,
        "runs": [],
    }
    for repeat in range(args.repeats):
        for backend in (backends if repeat % 2 == 0 else list(reversed(backends))):
            started, usage = time.monotonic(), cpu_seconds()
            streams = [encoders[backend].encode(images, args.output / f"{backend}_{repeat}_{camera}.mp4",
                                                args.fps, args.frames)
                       for camera, images in enumerate(inputs)]
            run = {"backend": backend, "repeat": repeat, "streams": streams,
                   "wall_seconds": time.monotonic() - started, "cpu_seconds": cpu_seconds() - usage}
            report["runs"].append(run)
            print(json.dumps(run), flush=True)
    report["summary"] = {}
    for backend in backends:
        runs = [run for run in report["runs"] if run["backend"] == backend]
        wall = statistics.median(run["wall_seconds"] for run in runs)
        cpu = statistics.median(run["cpu_seconds"] for run in runs)
        report["summary"][backend] = {
            "wall_seconds_median_including_validation": wall,
            "cpu_seconds_median_including_validation": cpu,
            "effective_cpu_cores": cpu / wall,
            "aggregate_frames_per_second_including_validation": args.frames * args.cameras / wall,
            "encode_seconds_median": statistics.median(sum(s["encode_seconds"] for s in run["streams"]) for run in runs),
            "validate_seconds_median": statistics.median(sum(s["validate_seconds"] for s in run["streams"]) for run in runs),
            "bytes_median": statistics.median(sum(s["bytes"] for s in run["streams"]) for run in runs),
        }
    (args.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report["summary"], indent=2))


if __name__ == "__main__":
    main()
