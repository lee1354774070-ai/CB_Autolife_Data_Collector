"""Save-time accelerator contracts; all real media tests use CPU, never GPU."""
from contextlib import contextmanager
from fractions import Fraction
import importlib.util
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from video_encoding import (
    HardwareVideoEncoder, _run_checked, _validate_in_process, accelerated_rgb_encoding,
    probe_encoder, select_encoder, validate_resume_encoding, validate_video,
)


def args(**overrides):
    values = dict(video_accelerator="cpu", video_ffmpeg_bin="ffmpeg",
                  video_hw_device="/dev/dri/renderD128", video_hw_qp=23,
                  video_gop=None, encoder_threads=None, video_encoding_timeout_sec=30, fps=30)
    return SimpleNamespace(**(values | overrides))


@contextmanager
def fake_lerobot():
    module = ModuleType("lerobot.datasets.utils")
    module.DEFAULT_IMAGE_PATH = "images/{image_key}/episode-{episode_index:06d}/frame-{frame_index:06d}.png"
    with patch.dict(sys.modules, {"lerobot.datasets.utils": module}):
        yield module.DEFAULT_IMAGE_PATH


class VideoEncodingTests(unittest.TestCase):
    def make_images(self, root, size=(32, 32)):
        from PIL import Image
        images = root / "images"
        images.mkdir()
        for index in range(3):
            Image.new("RGB", size, (100, index * 50, 20)).save(images / f"frame-{index:06d}.png")
        return images

    def test_default_does_not_probe_or_import_lerobot(self):
        with patch("video_encoding.probe_encoder") as probe, patch("video_encoding.shutil.which") as which:
            self.assertIsNone(select_encoder(args(), {(640, 480)}, Mock()))
            probe.assert_not_called()
            which.assert_not_called()
        with accelerated_rgb_encoding(object(), None) as receipt:
            self.assertIsNone(receipt)

    def test_auto_nvenc_first_then_vaapi_and_explicit_fail_closed(self):
        with patch("video_encoding.shutil.which", return_value="/usr/bin/ffmpeg"), \
             patch("video_encoding.probe_encoder", side_effect=[RuntimeError("no NVENC"), None]) as probe:
            selected = select_encoder(args(video_accelerator="auto"), {(640, 480)}, Mock())
            self.assertEqual(selected.backend, "vaapi")
            self.assertEqual([call.args[0].backend for call in probe.call_args_list], ["nvenc", "vaapi"])
        with patch("video_encoding.shutil.which", return_value="/usr/bin/ffmpeg"), \
             patch("video_encoding.probe_encoder", side_effect=RuntimeError("driver unavailable")) as probe:
            with self.assertRaisesRegex(RuntimeError, "driver unavailable"):
                select_encoder(args(video_accelerator="nvenc"), {(640, 480)}, Mock())
            self.assertEqual(probe.call_count, 1)

    def test_auto_warns_at_startup_without_ffmpeg(self):
        with patch("video_encoding.shutil.which", return_value=None):
            logger = Mock()
            self.assertIsNone(select_encoder(args(video_accelerator="auto"), {(640, 480)}, logger))
            logger.warning.assert_called_once()
            with self.assertRaisesRegex(RuntimeError, "not found"):
                select_encoder(args(video_accelerator="nvenc"), {(640, 480)}, Mock())

    def test_invalid_options_rejected(self):
        for invalid in (dict(backend="unknown"), dict(qp=-1), dict(qp=52), dict(qp=True),
                        dict(threads=0), dict(gop=0), dict(timeout=0), dict(timeout=float("inf"))):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                HardwareVideoEncoder(**(dict(backend="nvenc", ffmpeg="ffmpeg") | invalid))

    def test_argument_list_and_frame_color_thread_contract(self):
        for backend in ("vaapi", "nvenc", "cpu"):
            with self.subTest(backend=backend):
                command = HardwareVideoEncoder(backend, "/bin/custom ffmpeg").command(
                    Path("/input with spaces"), Path("/output with spaces.mp4"), 30, 90)
                self.assertEqual(command[0], "/bin/custom ffmpeg")
                self.assertIn("/input with spaces/frame-%06d.png", command)
                for option, value in (("-bf", "0"), ("-g", "2"), ("-frames:v", "90"),
                                      ("-fps_mode", "passthrough"), ("-filter_threads", "1"),
                                      ("-colorspace", "smpte170m"), ("-color_range", "tv")):
                    self.assertEqual(command[command.index(option) + 1], value)
                self.assertEqual(command.count("-threads"), 2)
                self.assertIn("-n", command)
                self.assertNotIn("-y", command)
                self.assertNotIn("-r", command)
                self.assertTrue(command[command.index("-vf") + 1].startswith("scale=out_color_matrix=bt601"))

    def test_missing_extra_symlink_or_changed_image_rejected(self):
        from PIL import Image
        for failure in ("missing", "extra", "symlink", "size", "depth", "odd"):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                images = self.make_images(root, size=(31, 32) if failure == "odd" else (32, 32))
                path = images / "frame-000001.png"
                if failure == "missing":
                    path.unlink()
                elif failure == "extra":
                    shutil.copyfile(path, images / "frame-000003.png")
                elif failure == "symlink":
                    path.unlink()
                    path.symlink_to(images / "frame-000000.png")
                elif failure == "size":
                    Image.new("RGB", (64, 32)).save(path)
                elif failure == "depth":
                    Image.new("I;16", (32, 32)).save(path)
                with patch("video_encoding._run_checked") as run, self.assertRaises((RuntimeError, ValueError)):
                    HardwareVideoEncoder("nvenc", "ffmpeg").encode(images, root / "out.mp4", 30, 3)
                run.assert_not_called()

    def test_encode_success_is_private_then_atomic_and_keeps_inputs(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            images = self.make_images(root)
            output = root / "out.mp4"
            def run(command, timeout, operation):
                self.assertFalse(output.exists())
                self.assertLessEqual(timeout, 12)
                private = Path(command[-1])
                self.assertNotEqual(private, output)
                private.write_bytes(b"synthetic validated output")
            with patch("video_encoding._run_checked", side_effect=run), \
                 patch("video_encoding.validate_video") as validate:
                receipt = HardwareVideoEncoder("nvenc", "ffmpeg", timeout=12).encode(images, output, 30, 3)
            self.assertEqual(output.read_bytes(), b"synthetic validated output")
            self.assertEqual(receipt["frames"], 3)
            self.assertEqual(receipt["color_matrix"], "bt601")
            self.assertLess(validate.call_args.kwargs["timeout"], 12)
            self.assertEqual(len(list(images.glob("*.png"))), 3)
            self.assertFalse(list(root.glob(".collector-encode-*")))

    def test_encode_error_validation_error_never_fallback_or_delete_inputs(self):
        for failure in ("encoder", "validation"):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                images = self.make_images(root)
                output = root / "failed.mp4"
                def run(command, timeout, operation):
                    Path(command[-1]).write_bytes(b"partial")
                    if failure == "encoder":
                        raise RuntimeError("GPU unavailable")
                with patch("video_encoding._run_checked", side_effect=run) as encode, \
                     patch("video_encoding.validate_video", side_effect=RuntimeError("corrupt output")):
                    with self.assertRaises(RuntimeError):
                        HardwareVideoEncoder("nvenc", "ffmpeg").encode(images, output, 30, 3)
                    self.assertEqual(encode.call_count, 1)
                self.assertFalse(output.exists())
                self.assertEqual(len(list(images.glob("*.png"))), 3)
                self.assertFalse(list(root.glob(".collector-encode-*")))

    def test_output_race_never_overwrites_or_deletes_existing_data(self):
        for race in (False, True):
            with self.subTest(race=race), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                images = self.make_images(root)
                output = root / "out.mp4"
                if not race:
                    output.write_bytes(b"existing")
                def run(command, timeout, operation):
                    Path(command[-1]).write_bytes(b"new")
                    output.write_bytes(b"existing")
                with patch("video_encoding._run_checked", side_effect=run), patch("video_encoding.validate_video"):
                    with self.assertRaises(FileExistsError):
                        HardwareVideoEncoder("nvenc", "ffmpeg").encode(images, output, 30, 3)
                self.assertEqual(output.read_bytes(), b"existing")

    def test_bounded_subprocess_exit_and_timeout_messages(self):
        with patch("video_encoding.subprocess.run", side_effect=subprocess.TimeoutExpired(["ffmpeg"], 1)):
            with self.assertRaisesRegex(RuntimeError, "exceeded"):
                _run_checked(["ffmpeg"], 1, "encode")
        def run(command, **kwargs):
            self.assertIs(kwargs["stdin"], subprocess.DEVNULL)
            self.assertEqual(kwargs["env"]["OMP_NUM_THREADS"], "1")
            kwargs["stderr"].write(b"x" * 6000 + b"tail-marker")
            return subprocess.CompletedProcess(command, 1)
        with patch("video_encoding.subprocess.run", side_effect=run), self.assertRaises(RuntimeError) as error:
            _run_checked(["ffmpeg"], 1, "encode")
        self.assertLess(len(str(error.exception)), 4200)
        self.assertTrue(str(error.exception).endswith("tail-marker"))

    def test_readback_is_isolated_with_timeout(self):
        with patch("video_encoding._run_checked") as run:
            validate_video(Path("/unused.mp4"), frames=3, fps=30, width=32, height=32, timeout=2)
        self.assertEqual(run.call_args.args[0][0], sys.executable)
        self.assertIn("--validate", run.call_args.args[0])
        self.assertEqual(run.call_args.args[1], 2)

    def test_readback_rejects_bad_counts_timestamps_formats(self):
        base_stream = dict(width=32, height=32, average_rate=30, pix_fmt="yuv420p",
                           codec_context=SimpleNamespace(name="h264"))
        for failure in (None, "short", "extra", "pts", "duplicate", "corrupt", "fps", "format", "size"):
            with self.subTest(failure=failure):
                stream = SimpleNamespace(**base_stream)
                if failure == "fps": stream.average_rate = 15
                if failure == "format": stream.pix_fmt = "yuv444p"
                frames = [SimpleNamespace(pts=i, time_base=Fraction(1, 30), width=32, height=32, is_corrupt=False)
                          for i in range(4 if failure == "extra" else 2 if failure == "short" else 3)]
                if failure == "pts": frames[1].pts = 5
                if failure == "duplicate": frames[1].pts = 0
                if failure == "corrupt": frames[1].is_corrupt = True
                if failure == "size": frames[1].width = 64
                class Streams(list):
                    video = [stream]
                container = Mock()
                container.streams = Streams([stream])
                container.decode.return_value = iter(frames)
                context = Mock()
                context.__enter__ = Mock(return_value=container)
                context.__exit__ = Mock(return_value=False)
                with patch.dict(sys.modules, {"av": SimpleNamespace(open=Mock(return_value=context))}):
                    if failure is None:
                        _validate_in_process("unused", 3, 30, 32, 32)
                    else:
                        with self.assertRaises(RuntimeError):
                            _validate_in_process("unused", 3, 30, 32, 32)

    def test_resume_rgb_contract_and_depth_aliases(self):
        validate_resume_encoding({"total_episodes": 0})
        for codec, fmt in (("h264", "yuv420p"), ("av1", "yuv420p"), ("h264", "yuv444p"), (None, None)):
            info = {"total_episodes": 1, "features": {
                "rgb": {"dtype": "video", "info": {"video.codec": codec, "video.pix_fmt": fmt}},
                "depth": {"dtype": "video", "info": {"is_depth_map": True}},
                "depth2": {"dtype": "video", "info": {"video.is_depth_map": True}},
            }}
            if (codec, fmt) == ("h264", "yuv420p"):
                validate_resume_encoding(info)
            else:
                with self.assertRaisesRegex(RuntimeError, "original encoder or a new dataset"):
                    validate_resume_encoding(info)

    def test_writer_restore_depth_native_and_no_global_patch(self):
        class Writer:
            _batch_encoding_size = 1
            _streaming_encoder = None
            _rgb_encoder = object()
            episode_buffer = {"size": 3}
            _meta = SimpleNamespace(depth_keys=["depth"], fps=30)
            def _encode_temporary_episode_video(self, video_key, episode_index):
                return "native-" + video_key
        with fake_lerobot() as pattern, tempfile.TemporaryDirectory() as directory:
            writer = Writer()
            writer._root = Path(directory)
            images = (writer._root / pattern.format(image_key="rgb", episode_index=0, frame_index=0)).parent
            images.mkdir(parents=True)
            config = writer._rgb_encoder
            encoder = HardwareVideoEncoder("nvenc", "ffmpeg")
            with patch.object(HardwareVideoEncoder, "encode", side_effect=RuntimeError("GPU lost")):
                with self.assertRaisesRegex(RuntimeError, "GPU lost"):
                    with accelerated_rgb_encoding(SimpleNamespace(writer=writer), encoder):
                        self.assertIsNone(writer._rgb_encoder)
                        self.assertEqual(writer._encode_temporary_episode_video("depth", 0), "native-depth")
                        self.assertEqual(Writer()._encode_temporary_episode_video("rgb", 0), "native-rgb")
                        with self.assertRaisesRegex(RuntimeError, "nested"):
                            with accelerated_rgb_encoding(SimpleNamespace(writer=writer), encoder):
                                pass
                        writer._encode_temporary_episode_video("rgb", 0)
            self.assertIs(writer._rgb_encoder, config)
            self.assertNotIn("_encode_temporary_episode_video", vars(writer))
            self.assertNotIn("_collector_hardware_encoding", vars(writer))
            self.assertTrue(images.exists())
            self.assertFalse(list(writer._root.glob("collector-rgb-*")))

    def test_real_cpu_color_probe_optional_dependencies_only(self):
        binary = shutil.which("ffmpeg") or str(Path(sys.prefix) / "bin/ffmpeg")
        if not Path(binary).is_file() or importlib.util.find_spec("av") is None:
            self.skipTest("requires FFmpeg and PyAV; never probes GPU")
        probe_encoder(HardwareVideoEncoder("cpu", binary, timeout=15), {(64, 48)}, 30)

    def test_actual_lerobot_cpu_adapter_optional_dependencies(self):
        binary = shutil.which("ffmpeg") or str(Path(sys.prefix) / "bin/ffmpeg")
        if not Path(binary).is_file() or any(importlib.util.find_spec(name) is None
                                             for name in ("av", "lerobot", "numpy")):
            self.skipTest("requires LeRobot, PyAV and FFmpeg; never probes GPU")
        import av
        import numpy as np
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        with tempfile.TemporaryDirectory(prefix="collector-cpu-adapter-smoke-") as directory:
            root = Path(directory) / "NOT_TRAINING"
            dataset = LeRobotDataset.create(
                "local/encoder-smoke", 30, root=root, robot_type="synthetic",
                features={"observation.state": {"dtype": "float32", "shape": (1,), "names": ["joint"]},
                          "action": {"dtype": "float32", "shape": (1,), "names": ["joint"]},
                          "observation.images.test": {"dtype": "video", "shape": (32, 32, 3),
                                                      "names": ["height", "width", "channel"]}},
                use_videos=True, image_writer_threads=1, encoder_threads=1,
            )
            try:
                for index in range(3):
                    dataset.add_frame({"observation.state": np.array([index], dtype=np.float32),
                                       "action": np.array([index], dtype=np.float32),
                                       "observation.images.test": np.full((32, 32, 3), 30 + index * 40, dtype=np.uint8),
                                       "task": "synthetic NOT_TRAINING"})
                with accelerated_rgb_encoding(dataset, HardwareVideoEncoder("cpu", binary, timeout=20)) as receipt:
                    dataset.save_episode()
            finally:
                dataset.finalize()
            self.assertEqual(receipt["streams"]["observation.images.test"]["frames"], 3)
            self.assertEqual(dataset.meta.total_episodes, 1)
            videos = list(root.rglob("*.mp4"))
            self.assertEqual(len(videos), 1)
            with av.open(str(videos[0])) as container:
                self.assertEqual(sum(1 for _ in container.decode(video=0)), 3)
            self.assertFalse(list(root.glob("collector-rgb-*")))
            self.assertFalse(getattr(dataset.writer, "_collector_hardware_encoding", False))


if __name__ == "__main__":
    unittest.main()
