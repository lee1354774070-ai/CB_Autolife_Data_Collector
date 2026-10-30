"""Optional save-time RGB acceleration, isolated from capture and robot control.

CPU mode keeps LeRobot unchanged. NVENC/VAAPI are probed before collecting and
never silently fall back during a save. Every staged frame is encoded and the
result is fully decoded in a bounded child process before publication. Depth
retains LeRobot's native 12-bit-capable path; ordinary NVENC cannot replace it.
"""

from contextlib import contextmanager
from dataclasses import asdict, dataclass, replace
from fractions import Fraction
import inspect
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time


def _positive_int(value, name):
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")


def _remaining(deadline, operation):
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise RuntimeError(f"{operation} exceeded its deadline")
    return remaining


def _run_checked(command, timeout, operation):
    """No shell, inherited input, unbounded in-memory logs, or orphan on timeout."""
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("subprocess timeout must be finite and positive")
    with tempfile.TemporaryFile() as errors:
        try:
            result = subprocess.run(
                command, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                stderr=errors, timeout=timeout, check=False,
                env={**os.environ, "OMP_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1",
                     "MKL_NUM_THREADS": "1", "NUMEXPR_NUM_THREADS": "1"},
            )
        except subprocess.TimeoutExpired as exc:
            # subprocess.run kills and waits for this direct child on timeout.
            raise RuntimeError(f"{operation} exceeded {timeout:g}s") from exc
        if result.returncode:
            errors.seek(0, 2)
            errors.seek(max(0, errors.tell() - 4096))
            detail = errors.read(4096).decode("utf-8", errors="replace").strip()
            raise RuntimeError(f"{operation} failed ({result.returncode}): {detail}")


@dataclass(frozen=True)
class HardwareVideoEncoder:
    backend: str
    ffmpeg: str
    device: str = "/dev/dri/renderD128"
    qp: int = 23
    gop: int = 2
    threads: int = 1
    timeout: float = 600.0

    def __post_init__(self):
        # cpu is available only to the standalone synthetic benchmark. The
        # production select_encoder(cpu) returns None (native LeRobot).
        if self.backend not in {"cpu", "nvenc", "vaapi"}:
            raise ValueError(f"unsupported video backend: {self.backend}")
        if not self.ffmpeg:
            raise ValueError("FFmpeg executable is required")
        if isinstance(self.qp, bool) or not isinstance(self.qp, int) or not 0 <= self.qp <= 51:
            raise ValueError("H.264 QP must be an integer in [0,51]")
        _positive_int(self.gop, "GOP")
        _positive_int(self.threads, "encoder threads")
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError("encoding timeout must be finite and positive")

    def command(self, images: Path, output: Path, fps: int, frames: int) -> list[str]:
        _positive_int(fps, "FPS")
        _positive_int(frames, "frames")
        command = [self.ffmpeg, "-hide_banner", "-loglevel", "error", "-nostdin", "-xerror"]
        if self.backend == "vaapi":
            command += ["-vaapi_device", self.device]
        command += ["-threads", str(self.threads), "-filter_threads", "1", "-filter_complex_threads", "1",
                    "-framerate", str(fps), "-start_number", "0", "-i", str(images / "frame-%06d.png"),
                    "-frames:v", str(frames), "-an", "-fps_mode", "passthrough"]
        # The inputs are RGB PNGs, never OpenCV BGR. Make the RGB->YUV matrix,
        # limited range and metadata explicit rather than relying on resolution
        # dependent defaults. Startup readback tests catch channel/range errors.
        # LeRobot/PyAV's established RGB readback uses BT.601. In particular,
        # some installed PyAV builds ignore a BT.709 tag during to_ndarray;
        # changing the matrix would silently change training colors.
        conversion = "scale=out_color_matrix=bt601:out_range=tv"
        if self.backend == "vaapi":
            command += ["-vf", conversion + ",format=nv12,hwupload", "-c:v", "h264_vaapi", "-qp", str(self.qp)]
        elif self.backend == "nvenc":
            command += ["-vf", conversion + ",format=yuv420p", "-c:v", "h264_nvenc",
                        "-preset", "p4", "-rc", "constqp", "-qp", str(self.qp)]
        else:
            command += ["-vf", conversion + ",format=yuv420p", "-c:v", "libx264",
                        "-preset", "veryfast", "-qp", str(self.qp)]
        command += ["-threads", str(self.threads), "-g", str(self.gop), "-bf", "0",
                    "-color_range", "tv", "-colorspace", "smpte170m", "-color_primaries", "smpte170m",
                    "-color_trc", "smpte170m", "-movflags", "+faststart", "-n", str(output)]
        return command

    def encode(self, images: Path, output: Path, fps: int, frames: int, *, color_probe=False) -> dict:
        """Publish only fully checked output; input files always survive errors.

        Encoder and readback share one wall-clock budget. Staging and final
        publication use the destination filesystem; atomic hard-link refuses an
        existing file even if another writer creates it during encoding.
        """
        from PIL import Image

        _positive_int(fps, "FPS")
        _positive_int(frames, "frames")
        images, output = Path(images), Path(output)
        if output.exists() or output.is_symlink():
            raise FileExistsError(output)
        if images.is_symlink() or not images.is_dir():
            raise RuntimeError("RGB staging directory is missing or unsafe")
        inputs = sorted(images.glob("frame-*.png"))
        if len(inputs) != frames or any(
            path.name != f"frame-{index:06d}.png" or not path.is_file() or path.is_symlink()
            for index, path in enumerate(inputs)
        ):
            raise RuntimeError("RGB frames missing, non-contiguous or unsafe; refusing a shortened video")
        start = time.monotonic()
        deadline = start + self.timeout
        width = height = 0
        for path in inputs:
            _remaining(deadline, "RGB input inspection")
            with Image.open(path) as frame:
                if frame.mode != "RGB":
                    raise ValueError("RGB encoding accepts RGB only; depth must use native LeRobot")
                if not width:
                    width, height = frame.size
                elif frame.size != (width, height):
                    raise ValueError("RGB dimensions changed within the episode; resizing is forbidden")
        if width % 2 or height % 2:
            raise ValueError("H.264 yuv420p requires even dimensions; resizing is forbidden")
        output.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix=".collector-encode-", dir=output.parent) as directory:
            private_output = Path(directory) / "video.mp4"
            _run_checked(self.command(images, private_output, fps, frames),
                         _remaining(deadline, "RGB encoding"), f"{self.backend} encoding")
            encoded = time.monotonic()
            validate_video(private_output, frames=frames, fps=fps, width=width, height=height,
                           timeout=_remaining(deadline, "RGB validation"), color_probe=color_probe)
            validated = time.monotonic()
            if private_output.is_symlink() or not private_output.is_file():
                raise RuntimeError("encoder did not produce a regular video file")
            size = private_output.stat().st_size
            if size <= 0:
                raise RuntimeError("encoder produced an empty video")
            os.link(private_output, output)
        return {"frames": frames, "width": width, "height": height, "bytes": size,
                "encode_seconds": encoded - start, "validate_seconds": validated - encoded,
                "color_matrix": "bt601", "color_range": "limited", "pixel_format": "yuv420p"}


def validate_video(path: Path, *, frames: int, fps: int, width: int, height: int,
                   timeout: float = 60.0, color_probe: bool = False) -> None:
    """Full CPU readback with a hard process timeout, not a metadata-only check."""
    for value, name in ((frames, "frames"), (fps, "FPS"), (width, "width"), (height, "height")):
        _positive_int(value, name)
    command = [sys.executable, "-B", str(Path(__file__).resolve()), "--validate", str(Path(path).resolve()),
               str(frames), str(fps), str(width), str(height)]
    if color_probe:
        command.append("--color-probe")
    _run_checked(command, timeout, "RGB full readback")


_PROBE_COLORS = ((230, 20, 20), (20, 230, 20), (20, 20, 230), (80, 80, 80), (235, 235, 235), (20, 20, 20))


def _validate_in_process(path, frames, fps, width, height, color_probe=False):
    """Worker entrypoint. No ROS/camera/LeRobot imports or child processes."""
    import av

    with av.open(str(path)) as container:
        if len(container.streams.video) != 1 or len(container.streams) != 1:
            raise RuntimeError("encoded RGB must contain exactly one video stream")
        stream = container.streams.video[0]
        stream.thread_count = 1
        if (stream.width, stream.height) != (width, height) or stream.codec_context.name != "h264":
            raise RuntimeError("encoded RGB dimensions/codec changed")
        if stream.average_rate != fps or stream.pix_fmt != "yuv420p":
            raise RuntimeError("encoded RGB FPS/pixel format changed")
        count = 0
        previous_timestamp = None
        for frame in container.decode(stream):
            if frame.pts is None or frame.time_base is None:
                raise RuntimeError("encoded RGB frame has no timestamp")
            timestamp = frame.pts * frame.time_base
            if (previous_timestamp is not None and timestamp <= previous_timestamp
                    or abs(timestamp - Fraction(count, fps)) > frame.time_base / 2):
                raise RuntimeError("encoded RGB timestamps are not contiguous")
            previous_timestamp = timestamp
            if frame.width != width or frame.height != height or frame.is_corrupt:
                raise RuntimeError("encoded RGB frame is corrupt or dimensions changed")
            if color_probe:
                # Compare well inside uniform patches; YUV420 is deliberately
                # lossy at color edges. This detects RGB/BGR swaps and gross
                # range/matrix errors, not exact photometric calibration.
                rgb = frame.to_ndarray(format="rgb24")
                for index, expected in enumerate(_PROBE_COLORS):
                    x = min(width - 1, (2 * index + 1) * width // (2 * len(_PROBE_COLORS)))
                    actual = rgb[height // 2, x].astype(float)
                    if max(abs(actual - expected)) > 24:
                        raise RuntimeError(f"RGB color probe mismatch: {actual.tolist()} vs {expected}")
            count += 1
            if count > frames:
                raise RuntimeError("encoded RGB contains extra frames")
        if count != frames:
            raise RuntimeError(f"encoded RGB frame count {count} != expected {frames}")


def probe_encoder(encoder: HardwareVideoEncoder, sizes: set[tuple[int, int]], fps: int) -> None:
    """Six synthetic color frames per actual RGB size; no camera/GPU inference."""
    from PIL import Image, ImageDraw

    if not sizes:
        raise ValueError("hardware encoding needs at least one RGB camera")
    deadline = time.monotonic() + min(encoder.timeout, 15.0)
    with tempfile.TemporaryDirectory(prefix="collector-video-probe-") as directory:
        root = Path(directory)
        for width, height in sorted(sizes):
            _positive_int(width, "width")
            _positive_int(height, "height")
            if width < 24 or height < 4 or width % 2 or height % 2:
                raise ValueError("hardware RGB probe requires even dimensions of at least 24x4")
            images = root / f"{width}x{height}"
            images.mkdir()
            frame = Image.new("RGB", (width, height))
            draw = ImageDraw.Draw(frame)
            for index, color in enumerate(_PROBE_COLORS):
                draw.rectangle((index * width // 6, 0, (index + 1) * width // 6 - 1, height - 1), fill=color)
            for index in range(6):
                frame.save(images / f"frame-{index:06d}.png", compress_level=0)
            replace(encoder, timeout=_remaining(deadline, "hardware startup probe")).encode(
                images, root / f"{width}x{height}.mp4", fps, 6, color_probe=True)


def select_encoder(args, sizes: set[tuple[int, int]], logger) -> HardwareVideoEncoder | None:
    """CPU never probes; auto fallback is allowed only before collection starts."""
    requested = args.video_accelerator
    if requested == "cpu":
        return None
    if requested not in {"auto", "nvenc", "vaapi"}:
        raise ValueError(f"unknown video accelerator: {requested}")
    binary = shutil.which(args.video_ffmpeg_bin)
    failures = []
    if binary is None:
        failures.append(f"FFmpeg executable not found: {args.video_ffmpeg_bin}")
    else:
        candidates = ("nvenc", "vaapi") if requested == "auto" else (requested,)
        for backend in candidates:
            encoder = HardwareVideoEncoder(backend, str(Path(binary).absolute()), device=args.video_hw_device,
                                           qp=args.video_hw_qp, gop=args.video_gop or 2,
                                           threads=args.encoder_threads or 1,
                                           timeout=args.video_encoding_timeout_sec)
            try:
                probe_encoder(encoder, sizes, int(args.fps))
            except Exception as exc:
                failures.append(f"{backend}: {exc}")
            else:
                logger.info(f"RGB save acceleration: {backend}, H.264 QP={encoder.qp}, GOP={encoder.gop}; "
                            "full decode/color probe passed. JPEG capture and depth unchanged.")
                return encoder
    detail = "; ".join(failures)
    if requested != "auto":
        raise RuntimeError(f"Requested RGB hardware encoder unavailable: {detail}")
    logger.warning(f"RGB hardware unavailable; native CPU encoder selected before capture: {detail}")
    return None


def validate_resume_encoding(info: dict) -> None:
    """Do not append H.264/420 clips under incompatible dataset metadata."""
    if not info.get("total_episodes", 0):
        return
    for key, feature in info.get("features", {}).items():
        video = feature.get("info", {})
        if feature.get("dtype") != "video" or video.get("is_depth_map") or video.get("video.is_depth_map"):
            continue
        if (video.get("video.codec"), video.get("video.pix_fmt")) != ("h264", "yuv420p"):
            raise RuntimeError(f"Hardware RGB resume requires h264/yuv420p metadata for {key}; "
                               "use the original encoder or a new dataset root")


@contextmanager
def accelerated_rgb_encoding(dataset, encoder: HardwareVideoEncoder | None):
    """Override one sequential writer during one save, restoring it on failure.

    LeRobot owns paths, metadata, depth and Parquet. Its per-video info probe
    reads actual H.264 metadata rather than claiming native CPU CRF settings.
    Return the receipt to the caller for its transactional episode sidecar.
    """
    if encoder is None:
        yield None
        return
    from lerobot.datasets.utils import DEFAULT_IMAGE_PATH

    writer = dataset.writer
    original = writer._encode_temporary_episode_video
    if (set(inspect.signature(original).parameters) != {"video_key", "episode_index"}
            or writer._batch_encoding_size != 1 or writer._streaming_encoder is not None):
        raise RuntimeError("hardware encoding requires the LeRobot 0.6-compatible sequential writer")
    if getattr(writer, "_collector_hardware_encoding", False):
        raise RuntimeError("nested hardware save is forbidden")
    frames = int(writer.episode_buffer["size"])
    _positive_int(frames, "episode frames")
    original_config = writer._rgb_encoder
    had_override = "_encode_temporary_episode_video" in vars(writer)
    receipt = {"version": 1, "encoder": asdict(encoder), "streams": {}}
    created = []

    def encode(video_key, episode_index):
        if video_key in writer._meta.depth_keys:
            return original(video_key, episode_index)
        relative = DEFAULT_IMAGE_PATH.format(image_key=video_key, episode_index=episode_index, frame_index=0)
        images = (writer._root / relative).parent
        # Never let a feature name or upstream path change delete outside the
        # writer's private transaction root.
        if images.is_symlink() or not images.resolve().is_relative_to(writer._root.resolve()):
            raise RuntimeError("RGB image path escapes the transaction root")
        temporary = Path(tempfile.mkdtemp(prefix="collector-rgb-", dir=writer._root))
        created.append(temporary)
        output = temporary / "video.mp4"
        receipt["streams"][video_key] = encoder.encode(images, output, writer._meta.fps, frames)
        # LeRobot normally consumes PNGs after encoding. Keep the same lifetime
        # only after successful full readback; failed encode leaves inputs.
        shutil.rmtree(images)
        return output

    try:
        writer._collector_hardware_encoding = True
        writer._encode_temporary_episode_video = encode
        writer._rgb_encoder = None
        yield receipt
    finally:
        writer._rgb_encoder = original_config
        if had_override:
            writer._encode_temporary_episode_video = original
        else:
            del writer._encode_temporary_episode_video
        del writer._collector_hardware_encoding
        for temporary in created:
            shutil.rmtree(temporary, ignore_errors=True)


if __name__ == "__main__":
    if len(sys.argv) not in {7, 8} or sys.argv[1] != "--validate":
        raise SystemExit("Internal readback worker; use scripts/benchmark_video_encoding.py")
    _validate_in_process(sys.argv[2], *(int(value) for value in sys.argv[3:7]),
                         color_probe=len(sys.argv) == 8 and sys.argv[7] == "--color-probe")
