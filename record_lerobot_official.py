#!/usr/bin/env python3
"""Record Autolife ROS2 topics into an official LeRobotDataset v3 dataset.

The default robot schema is 16 dimensions (two 7-DoF arms and two grippers).
Head (3), upper-waist (2), or waist/leg (4) groups can be appended. Joint-based
action modes use the exact same names and ordering as ``observation.state`` so
LeRobot's index-wise relative-action transform remains physically meaningful.

RGB frames are stored as ordinary LeRobot video features.  With LeRobot 0.6 or
newer, the optional uint16 depth stream is stored through the native depth-video
pipeline, preserving physical depth instead of converting it to an 8-bit image.

Batch collection behavior:
* start   -> begin accumulating a new episode buffer
* save    -> save current episode buffer into the same dataset root
* discard -> drop current episode buffer and keep the dataset untouched
* quit    -> save pending frames (if any) and stop the whole recording session

Each dataset frame is anchored to a new reference-camera timestamp.  Other
cameras, state, and action are selected from short buffers by nearest timestamp
and rejected when their difference exceeds the configured synchronization
tolerance.
"""

from __future__ import annotations

import argparse
import fcntl
import inspect
import json
import os
import re
import shutil
import signal
import socket
import threading
import time
from collections import Counter, deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path
from queue import Empty, SimpleQueue
from typing import Any

import cv2
import numpy as np
import rclpy
try:
    from lerobot.datasets import LeRobotDataset
except ImportError:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
try:
    from lerobot.configs import RGBEncoderConfig
except ImportError:
    RGBEncoderConfig = None
try:
    from lerobot.configs import DepthEncoderConfig
except ImportError:
    DepthEncoderConfig = None
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from camera_config import (
    CAMERA_SPECS,
    DEFAULT_DEPTH_CAMERA_TOPICS,
    DEFAULT_IMAGE_POLL_FPS,
    DEFAULT_RGB_CAMERA_TOPICS,
    DEFAULT_SYNC_IMAGE_BUFFER_SIZE,
    DEFAULT_SYNC_SIGNAL_BUFFER_SIZE,
)
from robot_schema import RobotSchema, build_robot_schema, parse_gripper_command
from dagger_labels import (
    DaggerLabels, FEATURES as DAGGER_FEATURES, timestamp_ns as dagger_timestamp_ns,
    validate_features as validate_dagger_features,
)
from cli_help import show_requested_parameter_help
from subtask_annotations import (
    SUBTASK_FEATURE, SubtaskAnnotations, check_pending_annotations, parse_subtasks, prepare_annotation,
)
from shm_camera import frame_to_hwc, read_shm_frame, read_shm_metadata, shm_timestamp_sec
from time_sync import (
    frame_interval_error_ratio,
    latest_at_or_before,
    linear_interpolation_alpha,
    nearest_sample,
    oldest_ready_sample,
    payload_timestamp_sec,
    surrounding_samples,
)


def default_robot_id() -> str:
    value = os.getenv("ROBOT_ID")
    if value:
        return value
    match = re.search(r"-(\d+)$", socket.gethostname())
    return match.group(1) if match else "283"


def default_topic_id() -> str:
    return f"{os.getenv('ROS_DOMAIN_ID', '0')}_{default_robot_id()}"


TOPIC_NODE_ID = default_topic_id()
STATE_TOPIC = f"/topic_arm_whole_body_and_gripper_current_joints_status_{TOPIC_NODE_ID}"
ACTION_ARM_TOPIC = f"/topic_arm_whole_body_target_joints_position_{TOPIC_NODE_ID}"
ACTION_GRIPPER_TOPIC = f"/topic_arm_gripper_target_joints_position_{TOPIC_NODE_ID}"
ACTION_EEF_TOPIC = f"/topic_arm_target_robot_eef_pose_{TOPIC_NODE_ID}"
ACTION_HEIGHT_TOPIC = f"/topic_arm_target_robot_height_z_{TOPIC_NODE_ID}"

EEF_ACTION_NAMES = [
    "left_eef_x", "left_eef_y", "left_eef_z", "left_eef_qx", "left_eef_qy", "left_eef_qz", "left_eef_qw",
    "right_eef_x", "right_eef_y", "right_eef_z", "right_eef_qx", "right_eef_qy", "right_eef_qz", "right_eef_qw",
    "height_z",
]

@dataclass
class ImageSample:
    image_hwc: np.ndarray
    stamp_sec: float
    received_sec: float
    width: int
    height: int
    is_depth_map: bool


@dataclass(frozen=True)
class TimedSample:
    """Parsed telemetry value with source/aligned and local receive times."""

    value: Any
    stamp_sec: float
    received_sec: float


@dataclass(frozen=True)
class StateSample:
    """State and status-target action parsed from one atomic status packet."""

    state: np.ndarray
    status_target_action: np.ndarray | None
    stamp_sec: float
    received_sec: float


def write_json_atomic(path: Path, payload: dict[str, Any]) -> None:
    """Replace a small JSON IPC file without exposing a partial document."""

    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary_path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
    os.replace(temporary_path, path)


def parse_json_payload(text: str) -> Any | None:
    if not text:
        return None
    brace = text.find("{")
    bracket = text.find("[")
    if brace >= 0 and bracket >= 0:
        start = min(brace, bracket)
    elif brace >= 0:
        start = brace
    elif bracket >= 0:
        start = bracket
    else:
        return None
    try:
        return json.loads(text[start:])
    except json.JSONDecodeError:
        return None


def numeric_list(value: Any) -> list[float] | None:
    if isinstance(value, (int, float, bool)):
        return [float(value)]
    if isinstance(value, list) and all(isinstance(item, (int, float, bool)) for item in value):
        return [float(item) for item in value]
    return None


def parse_eef_action(obj: Any) -> list[float] | None:
    if not isinstance(obj, dict):
        return None

    def pose_values(key: str) -> list[float] | None:
        pose = obj.get(key)
        if not isinstance(pose, dict):
            return None
        position = numeric_list(pose.get("position"))
        rotation = numeric_list(pose.get("rotation"))
        if position is None or rotation is None or len(position) < 3 or len(rotation) < 4:
            return None
        return [*position[:3], *rotation[:4]]

    left = pose_values("left_eef_pose")
    right = pose_values("right_eef_pose")
    if left is None or right is None:
        return None
    return [*left, *right]


def parse_height_action(obj: Any) -> float | None:
    if not isinstance(obj, dict):
        return None
    value = obj.get("height_z")
    if isinstance(value, (int, float, bool)):
        return float(value)
    return None


def parse_gripper_action(obj: Any) -> tuple[float | None, float | None] | None:
    """Keep a gripper command only when it controls at least one gripper."""

    values = parse_gripper_command(obj)
    return values if any(value is not None for value in values) else None


def ros_stamp_to_sec(msg: Image) -> float:
    stamp = msg.header.stamp
    if stamp.sec == 0 and stamp.nanosec == 0:
        return time.time()
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def image_msg_to_hwc(msg: Image, is_depth_map: bool) -> np.ndarray:
    """Convert a ROS Image to LeRobot's HWC representation.

    ROS rows may contain padding, so conversion honors ``msg.step`` instead of
    blindly reshaping the complete byte buffer.  Depth remains uint16 in sensor
    units (millimetres on robot 283); LeRobot 0.6 performs its own quantization.
    """

    required_bytes = msg.height * msg.step
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if raw.size < required_bytes:
        raise ValueError(f"image buffer has {raw.size} bytes, expected at least {required_bytes}")
    rows = raw[:required_bytes].reshape(msg.height, msg.step)

    if is_depth_map:
        if msg.encoding not in ("16UC1", "mono16"):
            raise ValueError(f"depth camera requires 16UC1/mono16, got {msg.encoding}")
        row_bytes = msg.width * 2
        packed = np.ascontiguousarray(rows[:, :row_bytes])
        byte_order = ">u2" if msg.is_bigendian else "<u2"
        return packed.view(byte_order).reshape(msg.height, msg.width, 1).astype(np.uint16, copy=False)

    if msg.encoding not in ("rgb8", "bgr8"):
        raise ValueError(f"RGB camera requires rgb8/bgr8, got {msg.encoding}")
    row_bytes = msg.width * 3
    image = rows[:, :row_bytes].reshape(msg.height, msg.width, 3)
    if msg.encoding == "bgr8":
        image = image[..., ::-1]
    return np.ascontiguousarray(image)


def dataset_features(
    active_images: dict[str, ImageSample],
    state_names: tuple[str, ...],
    action_names: tuple[str, ...] | list[str],
    with_subtasks: bool = False,
    with_dagger: bool = False,
) -> dict[str, dict[str, Any]]:
    """Create metadata directly from the resolved schema and live cameras."""

    features: dict[str, dict[str, Any]] = {
        "observation.state": {"dtype": "float32", "shape": (len(state_names),), "names": list(state_names)},
        "action": {"dtype": "float32", "shape": (len(action_names),), "names": list(action_names)},
    }
    if with_subtasks:
        features["subtask_index"] = dict(SUBTASK_FEATURE)
    if with_dagger:
        features.update(DAGGER_FEATURES)
    for camera_name, latest in active_images.items():
        feature = {
            "dtype": "video",
            "shape": (latest.height, latest.width, 1 if latest.is_depth_map else 3),
            "names": ["height", "width", "channel"],
        }
        if latest.is_depth_map:
            feature["info"] = {"is_depth_map": True, "video.output_unit": "mm"}
        features[f"observation.images.{camera_name}"] = feature
    return features


def call_lerobot_dataset(method: Any, **kwargs: Any) -> LeRobotDataset:
    supported = inspect.signature(method).parameters
    return method(**{key: value for key, value in kwargs.items() if key in supported})


def make_rgb_encoder(args: argparse.Namespace) -> Any | None:
    if RGBEncoderConfig is None:
        return None
    kwargs: dict[str, Any] = {"vcodec": args.vcodec}
    if args.video_crf is not None:
        kwargs["crf"] = args.video_crf
    if args.video_preset is not None:
        kwargs["preset"] = args.video_preset
    if args.video_gop is not None:
        kwargs["g"] = args.video_gop
    return RGBEncoderConfig(**kwargs)


def make_depth_encoder(args: argparse.Namespace) -> Any | None:
    """Build LeRobot 0.6's native depth encoder only when depth is requested."""

    if not args.with_depth:
        return None
    if DepthEncoderConfig is None:
        raise RuntimeError(
            "Depth recording requires LeRobot >= 0.6 with DepthEncoderConfig. "
            "RGB-only recording remains compatible with older LeRobot versions."
        )
    return DepthEncoderConfig(
        vcodec=args.depth_vcodec,
        depth_min=args.depth_min,
        depth_max=args.depth_max,
        shift=args.depth_shift,
        use_log=args.depth_use_log,
    )


class OfficialLeRobotRecorder(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("official_lerobot_recorder")
        self.args = args
        self.subtasks = SubtaskAnnotations(args.subtasks)
        self.dagger = DaggerLabels(args.sync_signal_buffer_size) if getattr(args, "dagger", False) else None
        self.dagger_progress_future = None
        # A failed save can already have mutated LeRobot's buffer/metadata.
        # Never retry it implicitly during shutdown or accept a new episode.
        self.save_failed = False
        # Mid-episode acknowledgements may live on NAS. Keep their file I/O off
        # the frame/ROS thread; one outstanding receipt also bounds the queue.
        self.mark_status_writer = ThreadPoolExecutor(max_workers=1, thread_name_prefix="subtask_receipt")
        self.mark_status_future = None
        self.schema: RobotSchema = build_robot_schema(
            with_head=args.with_head,
            with_waist=args.with_waist,
            with_upper_waist=args.with_upper_waist,
        )
        self.dataset: LeRobotDataset | None = None
        self.active_cameras: list[str] = []
        self.channel_first_cameras: set[str] = set()
        self.stop_requested = False
        self.is_recording = False
        # ``frames_written`` includes all episodes already present when a
        # dataset is resumed.  Keep a separate session counter for live FPS;
        # otherwise a large historical dataset makes the reported rate huge.
        self.frames_written = 0
        self.session_frames_written = 0
        self.record_ticks_waited = 0
        self.episodes_invalidated = 0
        self.image_buffer_overflows = 0
        self.current_episode_frames = 0
        self.saved_episodes = 0
        self.discarded_episodes = 0
        self.task_episode_counts: Counter[str] = Counter()
        self.session_saved_by_task: Counter[str] = Counter()
        self.session_discarded_by_task: Counter[str] = Counter()
        self.last_progress = time.time()
        self.last_progress_frame_count = 0
        self.started_sec = time.time()
        self.sync_log = None
        self.control_queue: SimpleQueue[str] = SimpleQueue()
        self.motion_lock_path: Path | None = (
            Path(args.motion_lock_file).expanduser().resolve()
            if args.motion_lock_file
            else None
        )
        self.motion_lock_handle = None
        if self.motion_lock_path is not None:
            self.motion_lock_path.parent.mkdir(parents=True, exist_ok=True)
            self.motion_lock_handle = self.motion_lock_path.open("a+", encoding="utf-8")
            self.get_logger().info(f"motion lock file: {self.motion_lock_path}")

        self.latest_images: dict[str, ImageSample] = {}
        # Images are large, so their buffers are deliberately short. Telemetry
        # buffers are inexpensive and longer to tolerate different ROS rates.
        self.image_buffers: dict[str, deque[ImageSample]] = {
            name: deque(maxlen=args.sync_image_buffer_size) for name in args.cameras
        }
        self.state_buffer: deque[StateSample] = deque(maxlen=args.sync_signal_buffer_size)
        self.action_body_buffer: deque[TimedSample] = deque(maxlen=args.sync_signal_buffer_size)
        self.action_gripper_buffer: deque[TimedSample] = deque(maxlen=args.sync_signal_buffer_size)
        self.action_eef_buffer: deque[TimedSample] = deque(maxlen=args.sync_signal_buffer_size)
        self.action_height_buffer: deque[TimedSample] = deque(maxlen=args.sync_signal_buffer_size)
        self.sync_reference_camera: str | None = None
        self.last_reference_stamp_sec: float | None = None
        self.last_episode_anchor_stamp_sec: float | None = None
        self.episode_invalid = False
        self.episode_invalid_reason: str | None = None
        self.state_ready_written = False
        self.control_thread: threading.Thread | None = None
        self.last_shm_timestamps: dict[str, int] = {}
        self.reported_image_overflows: set[str] = set()

        image_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        if args.image_source == "ros":
            for name, topic in args.cameras.items():
                is_depth_map = name in args.depth_cameras
                self.create_subscription(Image, topic, self._make_image_cb(name, is_depth_map), image_qos)
                camera_type = "depth" if is_depth_map else "RGB"
                self.get_logger().info(f"{camera_type} camera {name}: ROS topic {topic}")
        else:
            self.create_timer(1.0 / float(args.image_poll_fps), self._poll_shm_images)
            for name in args.cameras:
                self.get_logger().info(
                    f"{'depth' if name in args.depth_cameras else 'RGB'} camera {name}: "
                    f"direct SHM ({CAMERA_SPECS[name].buffer_path})"
                )

        self.create_subscription(String, args.state_topic, self._state_cb, 20)
        self.create_subscription(String, args.action_arm_topic, self._action_arm_cb, 20)
        self.create_subscription(String, args.action_gripper_topic, self._action_gripper_cb, 20)
        self.create_subscription(String, args.action_eef_topic, self._action_eef_cb, 20)
        self.create_subscription(String, args.action_height_topic, self._action_height_cb, 20)
        self.get_logger().info(f"state topic: {args.state_topic}")
        self.get_logger().info(f"state/action joints ({self.schema.size}): {', '.join(self.schema.names)}")
        self.get_logger().info(f"action mode: {args.action_mode}")
        self.get_logger().info(f"action arm topic: {args.action_arm_topic}")
        self.get_logger().info(f"action gripper topic: {args.action_gripper_topic}")
        self.get_logger().info(f"action eef topic: {args.action_eef_topic}")
        self.get_logger().info(f"action height topic: {args.action_height_topic}")

    def _make_image_cb(self, camera_name: str, is_depth_map: bool):
        def cb(msg: Image) -> None:
            received_sec = time.time()
            try:
                image = image_msg_to_hwc(msg, is_depth_map)
            except Exception as exc:
                self.get_logger().warn(f"drop {camera_name}: {exc}")
                return
            self._store_image(
                camera_name=camera_name,
                image=image,
                stamp_sec=ros_stamp_to_sec(msg),
                received_sec=received_sec,
                width=msg.width,
                height=msg.height,
                is_depth_map=is_depth_map,
            )
        return cb

    def _store_image(
        self,
        *,
        camera_name: str,
        image: np.ndarray,
        stamp_sec: float,
        received_sec: float,
        width: int,
        height: int,
        is_depth_map: bool,
    ) -> None:
        """Store one complete image and detect bounded-FIFO overwrites."""

        if self.dataset is not None and camera_name not in self.active_cameras:
            return  # Late cameras cannot change the saved dataset schema.

        sample = ImageSample(
            image_hwc=image,
            stamp_sec=stamp_sec,
            received_sec=received_sec,
            width=width,
            height=height,
            is_depth_map=is_depth_map,
        )
        buffer = self.image_buffers[camera_name]
        # Before the operator starts an episode, a full FIFO is expected: it
        # simply retains a short rolling warmup window. Only an overwrite
        # during active recording can affect saved data and must invalidate the
        # entire episode.
        if self.is_recording and buffer.maxlen is not None and len(buffer) >= buffer.maxlen:
            self.image_buffer_overflows += 1
            if camera_name not in self.reported_image_overflows:
                self.reported_image_overflows.add(camera_name)
                self._log_sync(
                    {
                        "event": "image_buffer_overflow",
                        "camera": camera_name,
                        "capacity": buffer.maxlen,
                        "wall_time": received_sec,
                    }
                )
                self._invalidate_episode(
                    f"image_buffer_overflow:{camera_name}",
                    received_sec,
                    capacity=buffer.maxlen,
                )
        self.latest_images[camera_name] = sample
        buffer.append(sample)

    def _poll_shm_images(self) -> None:
        """Batch-snapshot SHM metadata, then append new images in FIFO order.

        The SHM ABI exposes the latest complete frame rather than a kernel
        queue. First read all camera metadata in one tight pass. This makes
        the four-camera snapshot as close in time as the file interface allows;
        only then copy image bytes for metadata entries whose source timestamp
        is new. Each completed image is appended to its own FIFO, and
        ``_record_tick`` later chooses the nearest timestamp across those FIFOs.

        If a producer updates metadata while its bytes are being copied,
        ``read_shm_frame`` rejects that generation and the next poll retries it.
        """

        metadata_by_camera: dict[str, tuple[Any, tuple[int, ...]]] = {}
        cameras = self.active_cameras if self.dataset is not None else self.args.cameras
        for camera_name in cameras:
            # Exactly one SHM source per logical camera. Hands read *_jpeg;
            # no second representation can enter the FIFO for the same frame.
            spec = CAMERA_SPECS[camera_name]
            metadata = read_shm_metadata(spec)
            if (
                metadata is None
                or metadata[0] <= 0
                or self.last_shm_timestamps.get(spec.meta_path) == metadata[0]
            ):
                continue
            metadata_by_camera[camera_name] = (spec, metadata)

        # Copy every camera before decoding any JPEG. Decoding one hand can
        # let the depth producer replace the generation captured above.
        copied_frames = []
        for camera_name, (spec, metadata) in metadata_by_camera.items():
            frame = read_shm_frame(spec, metadata)
            if frame is None:
                # One bounded retry on a fresh generation, still before decode.
                # read_shm_frame validates the metadata pair on both attempts.
                metadata = read_shm_metadata(spec)
                if (metadata is None or metadata[0] <= 0
                        or self.last_shm_timestamps.get(spec.meta_path) == metadata[0]):
                    continue
                frame = read_shm_frame(spec, metadata)
            if frame is None or frame.timestamp_ns != metadata[0]:
                continue
            copied_frames.append((camera_name, spec, frame, time.time()))
        for camera_name, spec, frame, received_sec in copied_frames:
            try:
                image = frame_to_hwc(frame, camera_name in self.args.depth_cameras, rgb=True)
            except ValueError as exc:
                self.get_logger().warn(f"drop {camera_name}: {exc}")
                continue
            self._store_image(
                camera_name=camera_name,
                image=image,
                stamp_sec=shm_timestamp_sec(frame.timestamp_ns, received_sec),
                received_sec=received_sec,
                width=frame.width,
                height=frame.height,
                is_depth_map=camera_name in self.args.depth_cameras,
            )
            self.last_shm_timestamps[spec.meta_path] = frame.timestamp_ns

    def _state_cb(self, msg: String) -> None:
        obj = parse_json_payload(msg.data)
        received_sec = time.time()
        state = self.schema.parse_state(obj) if obj is not None else None
        if state is not None:
            self.state_buffer.append(
                StateSample(
                    state=state,
                    status_target_action=self.schema.parse_status_target(obj),
                    # A few controller builds publish a regularly updating
                    # timestamp from a clock several seconds behind wall time.
                    # Such a stamp cannot share a synchronization domain with
                    # camera SHM, so fall back to callback arrival time once the
                    # offset exceeds the configured state freshness window.
                    stamp_sec=payload_timestamp_sec(
                        obj,
                        received_sec,
                        max_clock_skew_sec=self.args.max_state_age_sec,
                    ),
                    received_sec=received_sec,
                )
            )
            self._write_state_ready_file()

    def _write_state_ready_file(self) -> None:
        """Atomically tell the launcher that joint buffering has started."""

        if self.state_ready_written or not self.args.state_ready_file:
            return
        ready_path = Path(self.args.state_ready_file)
        write_json_atomic(
            ready_path,
            {
                "pid": os.getpid(),
                "state_samples": len(self.state_buffer),
                "wall_time": time.time(),
            },
        )
        self.state_ready_written = True
        self.get_logger().info("first complete state packet received; camera startup may proceed")

    def all_requested_cameras_seen(self) -> bool:
        """Return whether every configured image topic has produced a frame."""

        return set(self.args.cameras).issubset(self.latest_images)

    def _store_action(self, msg: String, parser: Any, buffer: deque[TimedSample]) -> None:
        obj = parse_json_payload(msg.data)
        if obj is None:
            return
        value = parser(obj)
        if value is not None:
            received_sec = time.time()
            if getattr(self, "dagger", None) is not None:
                try:
                    stamp = dagger_timestamp_ns(obj.get("timestamp_ns")) / 1e9
                    if abs(stamp - received_sec) > self.args.max_action_hold_sec:
                        return
                    self.dagger.ingest(obj, gripper=buffer is self.action_gripper_buffer)
                except (TypeError, ValueError):
                    return
            else:
                stamp = payload_timestamp_sec(obj, received_sec, max_clock_skew_sec=self.args.max_action_hold_sec)
            buffer.append(
                TimedSample(
                    value,
                    stamp,
                    received_sec,
                )
            )

    def _action_arm_cb(self, msg: String) -> None:
        self._store_action(msg, self.schema.parse_body_command, self.action_body_buffer)

    def _action_gripper_cb(self, msg: String) -> None:
        self._store_action(msg, parse_gripper_action, self.action_gripper_buffer)

    def _action_eef_cb(self, msg: String) -> None:
        self._store_action(msg, parse_eef_action, self.action_eef_buffer)

    def _action_height_cb(self, msg: String) -> None:
        self._store_action(msg, parse_height_action, self.action_height_buffer)

    def _control_fifo_loop(self) -> None:
        if not self.args.control_fifo:
            return
        fifo_path = Path(self.args.control_fifo)
        while not self.stop_requested:
            try:
                with fifo_path.open("r", encoding="utf-8") as fifo:
                    for line in fifo:
                        command = line.strip().lower()
                        if command:
                            self.control_queue.put(command)
                        if self.stop_requested:
                            break
            except FileNotFoundError:
                return
            except Exception as exc:
                self.get_logger().warn(f"control fifo error: {exc}")
                time.sleep(0.2)

    def start_control_listener(self) -> None:
        if not self.args.control_fifo:
            return
        self.control_thread = threading.Thread(target=self._control_fifo_loop, daemon=True)
        self.control_thread.start()

    def _load_existing_task_counts(self) -> None:
        """Recover per-task episode totals from LeRobot episode metadata.

        Both LeRobot 0.5.1 and 0.6 expose episode rows through
        ``dataset.meta.episodes``.  Each row contains a ``tasks`` list.  Counting
        unique values per row makes this robust to a future episode containing
        more than one language annotation while still counting each episode only
        once for each task category.
        """

        if self.dataset is None or self.saved_episodes == 0:
            return
        try:
            episodes = getattr(self.dataset.meta, "episodes", None)
            if episodes is None and hasattr(self.dataset.meta, "ensure_readable"):
                self.dataset.meta.ensure_readable()
                episodes = getattr(self.dataset.meta, "episodes", None)
            if episodes is None:
                raise RuntimeError("episode metadata is unavailable")
            for episode in episodes:
                tasks = episode.get("tasks", [])
                if isinstance(tasks, str):
                    tasks = [tasks]
                for task in set(tasks):
                    if isinstance(task, str) and task:
                        self.task_episode_counts[task] += 1
        except Exception as exc:
            # Recording can continue even if an unusual legacy metadata layout
            # prevents historical per-task reconstruction. New session counts
            # will still be exact and the warning makes the limitation visible.
            self.get_logger().warn(f"could not load historical per-task episode counts: {exc}")

    def _write_command_status(
        self,
        event: str,
        success: bool,
        request_id: str | None,
        *,
        episode_index: int | None = None,
        frames: int = 0,
        message: str,
        subtask_status: dict | None = None,
    ) -> None:
        """Publish receipts; mid-episode marks use compact asynchronous I/O."""

        if not self.args.status_file:
            return
        all_tasks = [] if event == "mark_subtask" else sorted(
            set(self.task_episode_counts)
            | set(self.session_saved_by_task)
            | set(self.session_discarded_by_task)
            | {self.args.task_name},
            key=str.casefold,
        )
        payload = {
            "event": event,
            "success": success,
            "recording": self.is_recording,
            "episode_invalid": self.episode_invalid,
            "request_id": request_id,
            "task": self.args.task_name,
            "episode_index": episode_index,
            "frames": frames,
            "expert_frames": (
                (self.dagger.last_expert_frames if event in ("save", "discard") and success
                 else self.dagger.expert_frames) if getattr(self, "dagger", None) else 0
            ),
            "message": message,
            "subtasks": subtask_status if subtask_status is not None else self.subtasks.progress(),
            "total_saved_episodes": self.saved_episodes,
            "session_saved_episodes": sum(self.session_saved_by_task.values()),
            "session_discarded_episodes": sum(self.session_discarded_by_task.values()),
            "task_counts": [
                {
                    "task": task,
                    "total": self.task_episode_counts[task],
                    "session_added": self.session_saved_by_task[task],
                    "session_discarded": self.session_discarded_by_task[task],
                }
                for task in all_tasks
            ],
            "wall_time": time.time(),
        }
        status_path = Path(self.args.status_file)
        if event == "mark_subtask":
            self.mark_status_future = self.mark_status_writer.submit(write_json_atomic, status_path, payload)
        else:
            write_json_atomic(status_path, payload)

    def _validate_resume_schema(self, info: dict[str, Any]) -> None:
        """Reject changes that LeRobot cannot append to an existing dataset.

        Feature shapes and camera keys are fixed when a dataset is created.  A
        clear startup error is much safer than failing after an episode has been
        collected, or writing vectors with a different physical interpretation.
        """

        features = info.get("features", {})
        validate_dagger_features(features, getattr(self, "dagger", None) is not None)
        subtask_feature = features.get("subtask_index")
        if bool(subtask_feature) != self.subtasks.enabled or (
            subtask_feature and (
                subtask_feature.get("dtype") != "int64" or list(subtask_feature.get("shape", [])) != [1]
            )
        ):
            raise RuntimeError(
                "Subtask annotation schema differs from the existing dataset. "
                "Use a new dataset root when enabling/disabling SUBTASKS_JSON."
            )
        existing_fps = info.get("fps")
        try:
            fps_matches = float(existing_fps) == float(self.args.fps)
        except (TypeError, ValueError):
            fps_matches = False
        if not fps_matches:
            raise RuntimeError(
                f"Existing dataset FPS is {existing_fps!r}; this run requests {self.args.fps}. "
                "Resume with the original COLLECT_FPS or use a new dataset root."
            )
        state_shape = features.get("observation.state", {}).get("shape", [])
        action_shape = features.get("action", {}).get("shape", [])
        expected_action_size = len(EEF_ACTION_NAMES) if self.args.action_mode == "eef" else self.schema.size
        if not state_shape or int(state_shape[0]) != self.schema.size:
            raise RuntimeError(
                f"Existing dataset state size is {state_shape}; this run requires {self.schema.size}. "
                "Use a new dataset root when changing WITH_HEAD, WITH_UPPER_WAIST, or WITH_WAIST."
            )
        if not action_shape or int(action_shape[0]) != expected_action_size:
            raise RuntimeError(
                f"Existing dataset action size is {action_shape}; this run requires {expected_action_size}. "
                "Use the original action mode or a new dataset root."
            )

        existing_camera_features = {
            key.removeprefix("observation.images."): value
            for key, value in features.items()
            if key.startswith("observation.images.")
        }
        existing_cameras = set(existing_camera_features)
        live_requested_cameras = set(self.latest_images)
        if existing_cameras != live_requested_cameras:
            missing = sorted(existing_cameras - live_requested_cameras)
            added = sorted(live_requested_cameras - existing_cameras)
            details = []
            if missing:
                details.append("required but not live: " + ", ".join(missing))
            if added:
                details.append("live but absent from dataset: " + ", ".join(added))
            raise RuntimeError(
                "Requested cameras do not match the existing dataset (" + "; ".join(details) + "). "
                "Resume with the original camera set or use a new dataset root."
            )
        existing_depth = {
            name
            for name, feature in existing_camera_features.items()
            if feature.get("info", {}).get("is_depth_map")
            or feature.get("info", {}).get("video.is_depth_map")
            or name in DEFAULT_DEPTH_CAMERA_TOPICS
        }
        if bool(existing_depth) != self.args.with_depth:
            raise RuntimeError(
                "WITH_DEPTH must match the setting used when this dataset root was created. "
                "LeRobot cannot add or remove a video feature while resuming."
            )

        existing_names = features.get("observation.state", {}).get("names")
        if existing_names and list(existing_names) != list(self.schema.names):
            self.get_logger().warn(
                "resuming a legacy dataset whose state/action labels predate canonical names; "
                "vector order is compatible, but metadata names remain unchanged"
            )

    def create_dataset(self) -> None:
        active = {name: self.latest_images[name] for name in self.args.cameras if name in self.latest_images}
        if len(active) < self.args.min_cameras:
            missing = sorted(set(self.args.cameras).difference(active))
            raise RuntimeError(
                f"Only {len(active)} active camera(s) after warmup, need at least {self.args.min_cameras}. "
                f"Active: {', '.join(sorted(active)) or 'none'}. Missing: {', '.join(missing) or 'none'}."
            )
        missing_depth = sorted(set(self.args.depth_cameras).difference(active))
        if missing_depth:
            raise RuntimeError("Requested depth camera(s) are not publishing: " + ", ".join(missing_depth))
        if not self.state_buffer:
            raise RuntimeError(
                "No complete state packet received for the selected joint schema. "
                "Check the state topic and WITH_HEAD/WITH_UPPER_WAIST/WITH_WAIST settings."
            )
        if self.args.action_mode == "status_target" and not any(
            sample.status_target_action is not None for sample in self.state_buffer
        ):
            raise RuntimeError("State packets do not contain all status-target fields required by this schema.")

        action_names = tuple(EEF_ACTION_NAMES) if self.args.action_mode == "eef" else self.schema.names
        # Construct encoders before touching the output directory so unsupported
        # depth configurations fail without leaving a partial dataset shell.
        rgb_encoder = make_rgb_encoder(self.args)
        depth_encoder = make_depth_encoder(self.args)
        output_dir = Path(self.args.output_dir)
        check_pending_annotations(output_dir)
        info_path = output_dir / "meta" / "info.json"
        should_recreate_empty_shell = False
        existing_info = None

        if info_path.exists():
            existing_info = json.loads(info_path.read_text(encoding="utf-8"))
            total_episodes = int(existing_info.get("total_episodes", 0))
            total_frames = int(existing_info.get("total_frames", 0))
            has_data_files = any((output_dir / "data").rglob("*.parquet")) if (output_dir / "data").exists() else False
            has_video_files = any((output_dir / "videos").rglob("*.mp4")) if (output_dir / "videos").exists() else False
            should_recreate_empty_shell = (
                total_episodes == 0
                and total_frames == 0
                and not has_data_files
                and not has_video_files
            )

        if info_path.exists() and not should_recreate_empty_shell:
            self._validate_resume_schema(existing_info)
            # Preserve feature insertion order. Sorting here could silently
            # change the default reference camera between create and resume.
            self.active_cameras = [
                key.replace("observation.images.", "")
                for key in existing_info.get("features", {})
                if key.startswith("observation.images.")
            ]
            # Datasets created by early collector versions declared video as
            # CHW.  New datasets follow LeRobot's HWC convention.  Preserve the
            # old layout only while appending to one of those legacy roots.
            self.channel_first_cameras = {
                name
                for name in self.active_cameras
                if (shape := existing_info["features"][f"observation.images.{name}"].get("shape", []))
                and shape[0] in (1, 3)
                and shape[-1] not in (1, 3)
            }
            self.dataset = call_lerobot_dataset(
                LeRobotDataset.resume,
                repo_id=self.args.repo_id,
                root=output_dir,
                image_writer_threads=0 if self.dagger is not None else self.args.image_writer_threads,
                vcodec=self.args.vcodec,
                rgb_encoder=rgb_encoder,
                depth_encoder=depth_encoder,
                batch_encoding_size=self.args.batch_encoding_size,
                streaming_encoding=self.args.streaming_encoding,
                encoder_queue_maxsize=self.args.encoder_queue_maxsize,
                encoder_threads=self.args.encoder_threads,
            )
            self.saved_episodes = int(existing_info.get("total_episodes", 0))
            self.frames_written = int(existing_info.get("total_frames", 0))
            self.get_logger().info(f"resuming dataset with {self.saved_episodes} saved episode(s)")
        else:
            if output_dir.exists():
                if should_recreate_empty_shell:
                    shutil.rmtree(output_dir)
                elif any(output_dir.iterdir()):
                    raise RuntimeError(
                        f"Dataset root exists but is not a valid LeRobot dataset: {output_dir}. "
                        "If this is a broken partial run, move it away or delete it first."
                    )
                else:
                    output_dir.rmdir()

            self.active_cameras = list(active)
            self.dataset = call_lerobot_dataset(
                LeRobotDataset.create,
                repo_id=self.args.repo_id,
                root=output_dir,
                fps=int(self.args.fps),
                features=dataset_features(active, self.schema.names, action_names, self.subtasks.enabled,
                                          self.dagger is not None),
                robot_type=self.args.robot_type,
                use_videos=True,
                image_writer_threads=0 if self.dagger is not None else self.args.image_writer_threads,
                vcodec=self.args.vcodec,
                rgb_encoder=rgb_encoder,
                depth_encoder=depth_encoder,
                batch_encoding_size=self.args.batch_encoding_size,
                streaming_encoding=self.args.streaming_encoding,
                encoder_queue_maxsize=self.args.encoder_queue_maxsize,
                encoder_threads=self.args.encoder_threads,
                video_files_size_in_mb=self.args.video_files_size_in_mb,
                data_files_size_in_mb=self.args.data_files_size_in_mb,
            )

        if self.dagger is not None:
            # Reuse MZJ's bounded, lossless staging writer on create AND resume.
            # Compression runs after authority is revoked, when saving video.
            from dagger.capture_image_writer import CaptureImageWriter
            self.dataset.writer.image_writer = CaptureImageWriter(
                num_threads=self.args.image_writer_threads)
            self.get_logger().info(
                "MZJ capture: lossless uncompressed PNG, raw uint16 depth, "
                f"{self.args.image_writer_threads} writer threads, queue limit 64")

        # Resolve the anchor only after resume/create establishes the exact
        # camera feature set that will actually be written to this dataset.
        if not self.active_cameras:
            raise RuntimeError("At least one recorded camera is required for timestamp synchronization.")
        requested_reference = self.args.sync_reference_camera
        if requested_reference is not None and requested_reference not in self.active_cameras:
            raise RuntimeError(
                f"Synchronization reference camera '{requested_reference}' is not recorded by this dataset. "
                f"Recorded cameras: {', '.join(self.active_cameras)}"
            )
        # On robot 283 the hand-left stream has a more stable receive cadence
        # than the native head-color stream. Keep explicit overrides intact,
        # prefer hand_left for the normal full-camera set, and fall back to the
        # first active camera for camera-only or custom configurations.
        self.sync_reference_camera = requested_reference or (
            "hand_left" if "hand_left" in self.active_cameras else self.active_cameras[0]
        )

        self._load_existing_task_counts()
        self.sync_log = open(Path(self.args.output_dir) / "sync_log.jsonl", "a", encoding="utf-8")
        self.started_sec = time.time()
        self.create_timer(1.0 / float(self.args.fps), self._record_tick)
        self.get_logger().info(f"active cameras: {', '.join(self.active_cameras)}")
        self.get_logger().info(
            f"sync reference: {self.sync_reference_camera}, "
            f"max delta: {self.args.max_sync_delta_sec * 1000:.1f} ms"
        )
        self.get_logger().info(f"dataset root: {self.args.output_dir}")
        self.get_logger().info("waiting for 'start' command to begin episode recording")
        if self.dagger is not None:
            self._write_command_status("ready", True, None, message="DAgger recorder initialized; no episode started")

    def _held_signal(
        self,
        samples: deque[TimedSample],
        anchor_sec: float,
        now: float,
    ) -> TimedSample | None:
        """Return the latest causal command that is still valid at the anchor."""

        sample = latest_at_or_before(samples, anchor_sec)
        if sample is None:
            return None
        if anchor_sec - sample.stamp_sec > self.args.max_action_hold_sec:
            return None
        if now - sample.received_sec > self.args.max_action_hold_sec + self.args.max_sync_delta_sec:
            return None
        return sample

    def _build_action(
        self,
        anchor_sec: float,
        interpolated_state: np.ndarray,
        now: float,
    ) -> tuple[np.ndarray, dict[str, float], dict[str, float]] | None:
        """Build an action whose source packet is synchronized to the anchor."""

        if self.args.action_mode == "eef":
            eef_sample = self._held_signal(self.action_eef_buffer, anchor_sec, now)
            height_sample = self._held_signal(self.action_height_buffer, anchor_sec, now)
            if eef_sample is None or height_sample is None:
                return None
            action = np.array([*eef_sample.value, height_sample.value], dtype=np.float32)
            return (
                action,
                {
                    "action_eef": (eef_sample.stamp_sec - anchor_sec) * 1000.0,
                    "action_height": (height_sample.stamp_sec - anchor_sec) * 1000.0,
                },
                {
                    "action_eef": (now - eef_sample.received_sec) * 1000.0,
                    "action_height": (now - height_sample.received_sec) * 1000.0,
                },
            )

        if self.args.action_mode == "status_target":
            status_sample = latest_at_or_before(
                (sample for sample in self.state_buffer if sample.status_target_action is not None),
                anchor_sec,
            )
            if status_sample is None or status_sample.status_target_action is None:
                return None
            if anchor_sec - status_sample.stamp_sec > self.args.max_action_hold_sec:
                return None
            return (
                status_sample.status_target_action.copy(),
                {"action_status_target": (status_sample.stamp_sec - anchor_sec) * 1000.0},
                {"action_status_target": (now - status_sample.received_sec) * 1000.0},
            )

        body_sample = self._held_signal(self.action_body_buffer, anchor_sec, now)
        if body_sample is None:
            if self.args.fallback_action_to_state:
                return (
                    interpolated_state.copy(),
                    {"action_state_fallback": 0.0},
                    {"action_state_fallback": 0.0},
                )
            return None
        gripper_sample = self._held_signal(self.action_gripper_buffer, anchor_sec, now)
        if gripper_sample is None:
            return None
        action = self.schema.compose_command_action(body_sample.value, gripper_sample.value)
        if action is None:
            return None
        return (
            action,
            {
                "action_body": (body_sample.stamp_sec - anchor_sec) * 1000.0,
                "action_gripper": (gripper_sample.stamp_sec - anchor_sec) * 1000.0,
            },
            {
                "action_body": (now - body_sample.received_sec) * 1000.0,
                "action_gripper": (now - gripper_sample.received_sec) * 1000.0,
            },
        )

    def _wait_for_reference(self, reason: str, now: float, **details: Any) -> None:
        """Record a timer tick that is waiting for a usable reference frame.

        No source frame is discarded here. The timer simply returns and tries
        again after more camera and state samples enter their FIFOs.
        """

        self.record_ticks_waited += 1
        self._log_sync({"event": "wait", "reason": reason, "wall_time": now, **details})

    def _invalidate_episode(self, reason: str, now: float, **details: Any) -> None:
        """Freeze the current buffer so it can never be saved as valid data."""

        if self.episode_invalid:
            return
        self.episodes_invalidated += 1
        self.episode_invalid = True
        self.episode_invalid_reason = reason
        self.is_recording = False
        self._release_motion_lock()
        self._log_sync(
            {
                "event": "episode_invalidated",
                "reason": reason,
                "buffered_frames": self.current_episode_frames,
                "wall_time": now,
                **details,
            }
        )
        if self.args.episode_event_file:
            event_path = Path(self.args.episode_event_file)
            write_json_atomic(
                event_path,
                {
                    "event": "episode_invalidated",
                    "reason": reason,
                    "buffered_frames": self.current_episode_frames,
                    "wall_time": now,
                },
            )
        self.get_logger().error(
            f"episode invalidated by synchronization failure: {reason}; "
            "press S or D to discard it before starting again"
        )

    def _consume_image_sample(self, camera_name: str, selected: ImageSample) -> None:
        """Remove one matched frame and all older frames from a camera FIFO."""

        buffer = self.image_buffers[camera_name]
        while buffer:
            if buffer.popleft() is selected:
                return

    def _wait_for_camera_buffers(self, now: float) -> bool:
        """Wait for every active camera to contribute its next FIFO sample.

        Starting an episode clears the reference warmup FIFO. Cameras are independent,
        so one stream can publish its first post-start frame before another.
        Treating that short phase offset as a missing camera would invalidate an
        otherwise healthy episode before its first dataset frame is written.

        A stream is only considered failed when its latest received frame has
        exceeded ``max_image_age_sec`` (or no frame has ever been received).
        """

        waiting_for: list[str] = []
        for camera_name in self.active_cameras:
            if self.image_buffers[camera_name]:
                continue

            latest = self.latest_images.get(camera_name)
            is_reference = camera_name == self.sync_reference_camera
            if latest is None:
                reason = "missing_reference_camera" if is_reference else f"missing_camera:{camera_name}"
                self._invalidate_episode(reason, now)
                return True

            receive_age = now - latest.received_sec
            if receive_age > self.args.max_image_age_sec:
                reason = "reference_camera_stalled" if is_reference else f"camera_stalled:{camera_name}"
                self._invalidate_episode(reason, now, receive_age_ms=receive_age * 1000.0)
                return True
            waiting_for.append(camera_name)

        if not waiting_for:
            return False
        self._wait_for_reference("camera_frames_not_ready", now, cameras=waiting_for)
        return True

    def _record_tick(self) -> None:
        if self.stop_requested or self.dataset is None or not self.is_recording:
            return
        now = time.time()
        dagger = getattr(self, "dagger", None)
        if dagger is not None and not dagger.ready:
            # Starting the recorder precedes hardware enable. No frames have
            # been accepted yet: establish a fresh, explicit action/label start
            # barrier instead of making up expert labels or skipping mid-run.
            if now - self.started_sec > self.args.dagger_start_timeout_sec:
                self._invalidate_episode("dagger_start_barrier_timeout", now)
                return
            ready = (dagger.labels and dagger.labels[-1]["timestamp_ns"] >= dagger.started_ns
                     and self.action_body_buffer and self.action_gripper_buffer
                     and all(now - samples[-1].stamp_sec <= self.args.max_action_hold_sec
                             for samples in (self.action_body_buffer, self.action_gripper_buffer)))
            self._clear_warmup_images()
            self.last_reference_stamp_sec = None
            if ready:
                dagger.ready = True
                self._log_sync({"event": "dagger_start_barrier_ready", "wall_time": now})
            return
        if self.args.duration is not None and now - self.started_sec >= self.args.duration:
            self.stop_requested = True
            return

        reference_name = self.sync_reference_camera
        if reference_name is None:
            self._invalidate_episode("missing_reference_camera", now)
            return
        if self._wait_for_camera_buffers(now):
            return
        reference = oldest_ready_sample(
            self.image_buffers[reference_name],
            now_sec=now,
            wait_sec=max(self.args.max_sync_delta_sec, self.args.max_state_interpolation_gap_sec),
            after_stamp_sec=self.last_reference_stamp_sec,
        )
        if reference is None:
            latest_reference = self.image_buffers[reference_name][-1]
            if now - latest_reference.received_sec > self.args.max_image_age_sec:
                self._invalidate_episode(
                    "reference_camera_stalled",
                    now,
                    receive_age_ms=(now - latest_reference.received_sec) * 1000.0,
                )
                return
            self._wait_for_reference("no_reference_frame_ready", now)
            return
        reference_age = now - reference.received_sec
        if reference_age > self.args.max_image_age_sec:
            self._invalidate_episode(
                "stale_reference_camera",
                now,
                receive_age_ms=reference_age * 1000.0,
            )
            return
        anchor_sec = reference.stamp_sec

        frame_interval_ms: float | None = None
        if self.last_episode_anchor_stamp_sec is not None:
            frame_interval = anchor_sec - self.last_episode_anchor_stamp_sec
            expected_interval = 1.0 / float(self.args.fps)
            frame_interval_ms = frame_interval * 1000.0
            interval_error_ratio = frame_interval_error_ratio(
                self.last_episode_anchor_stamp_sec,
                anchor_sec,
                float(self.args.fps),
            )
            if interval_error_ratio > self.args.max_frame_interval_error_ratio:
                self._invalidate_episode(
                    "reference_frame_interval",
                    now,
                    interval_ms=frame_interval_ms,
                    expected_interval_ms=expected_interval * 1000.0,
                    interval_error_ratio=interval_error_ratio,
                )
                return

        state_before, state_after = surrounding_samples(self.state_buffer, anchor_sec)
        state_stamps = [sample.stamp_sec for sample in self.state_buffer]
        state_oldest_sec = min(state_stamps, default=None)
        state_latest_sec = max(state_stamps, default=None)

        if state_before is None:
            # State samples are chronological. Once the oldest retained sample
            # is newer than this image, a "before" sample can never arrive.
            self._invalidate_episode(
                "state_interpolation_before_missing",
                now,
                anchor_sec=anchor_sec,
                state_oldest_sec=state_oldest_sec,
                state_latest_sec=state_latest_sec,
                state_samples=len(self.state_buffer),
            )
            return
        if state_after is None:
            latest_receive_age = now - state_before.received_sec
            if latest_receive_age > self.args.max_state_age_sec:
                self._invalidate_episode(
                    "state_stream_stalled",
                    now,
                    anchor_sec=anchor_sec,
                    state_latest_sec=state_latest_sec,
                    receive_age_ms=latest_receive_age * 1000.0,
                    state_samples=len(self.state_buffer),
                )
                return
            # The reference image remains in its FIFO. Retry the same image
            # after the next state callback supplies the interpolation endpoint.
            self._wait_for_reference(
                "state_interpolation_after_not_ready",
                now,
                anchor_sec=anchor_sec,
                state_oldest_sec=state_oldest_sec,
                state_latest_sec=state_latest_sec,
                anchor_ahead_ms=(anchor_sec - state_before.stamp_sec) * 1000.0,
                state_samples=len(self.state_buffer),
            )
            return
        before_delta = anchor_sec - state_before.stamp_sec
        after_delta = state_after.stamp_sec - anchor_sec
        state_span = state_after.stamp_sec - state_before.stamp_sec
        if state_span > self.args.max_state_interpolation_gap_sec:
            self._invalidate_episode(
                "state_interpolation_gap",
                now,
                before_delta_ms=before_delta * 1000.0,
                after_delta_ms=after_delta * 1000.0,
                span_ms=state_span * 1000.0,
            )
            return
        before_age = now - state_before.received_sec
        after_age = now - state_after.received_sec
        if before_age > self.args.max_state_age_sec or after_age > self.args.max_state_age_sec:
            self._invalidate_episode(
                "state_interpolation_stale",
                now,
                before_age_ms=before_age * 1000.0,
                after_age_ms=after_age * 1000.0,
            )
            return

        interpolation_alpha = linear_interpolation_alpha(
            state_before.stamp_sec,
            state_after.stamp_sec,
            anchor_sec,
        )
        if interpolation_alpha is None:
            self._invalidate_episode("state_interpolation_order", now)
            return
        interpolated_state = (
            state_before.state.copy()
            if state_span <= 0
            else state_before.state + (state_after.state - state_before.state) * interpolation_alpha
        ).astype(np.float32, copy=False)

        frame: dict[str, Any] = {
            "task": self.args.task_name,
            "observation.state": interpolated_state,
        }
        if self.subtasks.enabled:
            # Do not label a prospective subtask until the operator confirms it.
            # Confirmed spans are filled into this column immediately before save.
            frame["subtask_index"] = np.array([-1], dtype=np.int64)
        sync_deltas_ms = {
            "state_before": (state_before.stamp_sec - anchor_sec) * 1000.0,
            "state_after": (state_after.stamp_sec - anchor_sec) * 1000.0,
        }
        receive_ages_ms = {
            "state_before": before_age * 1000.0,
            "state_after": after_age * 1000.0,
        }
        matched_images: dict[str, ImageSample] = {}
        for camera_name in self.active_cameras:
            image_match = (
                (reference, 0.0)
                if camera_name == reference_name
                else nearest_sample(self.image_buffers[camera_name], anchor_sec)
            )
            if image_match is None:
                self._invalidate_episode(f"missing_camera:{camera_name}", now)
                return
            image_sample, image_delta = image_match
            if image_delta > self.args.max_sync_delta_sec:
                self._invalidate_episode(
                    f"unsynchronized_camera:{camera_name}",
                    now,
                    sync_delta_ms=image_delta * 1000.0,
                )
                return
            image_age = now - image_sample.received_sec
            if image_age > self.args.max_image_age_sec:
                self._invalidate_episode(
                    f"stale_camera:{camera_name}",
                    now,
                    receive_age_ms=image_age * 1000.0,
                )
                return
            frame[f"observation.images.{camera_name}"] = image_sample.image_hwc
            if camera_name in self.channel_first_cameras:
                frame[f"observation.images.{camera_name}"] = np.transpose(image_sample.image_hwc, (2, 0, 1))
            sync_deltas_ms[f"image:{camera_name}"] = (image_sample.stamp_sec - anchor_sec) * 1000.0
            receive_ages_ms[f"image:{camera_name}"] = image_age * 1000.0
            matched_images[camera_name] = image_sample

        action_result = self._build_action(anchor_sec, interpolated_state, now)
        if action_result is None:
            self._invalidate_episode("missing_stale_or_noncausal_action", now)
            return
        action, action_deltas_ms, action_ages_ms = action_result
        frame["action"] = action
        sync_deltas_ms.update(action_deltas_ms)
        receive_ages_ms.update(action_ages_ms)

        if dagger is not None:
            try:
                labels = dagger.frame(anchor_sec, action_deltas_ms, self.args.max_action_hold_sec)
            except ValueError as exc:
                self._invalidate_episode("missing_dagger_provenance", now, detail=str(exc))
                return
            frame.update({key: np.array([value], dtype=np.int64) for key, value in labels.items()})
        self.dataset.add_frame(frame)
        if dagger is not None:
            dagger.expert_frames += labels["dagger.train_mask"]
            if (self.current_episode_frames % 15 == 0 and
                    (self.dagger_progress_future is None or self.dagger_progress_future.done())):
                if self.dagger_progress_future is not None:
                    self.dagger_progress_future.result()
                self.dagger_progress_future = self.mark_status_writer.submit(
                    write_json_atomic, Path(self.args.output_dir).parent / ".dagger_progress.json",
                    {"trial_id": dagger.trial_id, "frames": self.current_episode_frames + 1,
                     "expert_frames": dagger.expert_frames, "wall_time": now})
        for camera_name, image_sample in matched_images.items():
            self._consume_image_sample(camera_name, image_sample)
        self.last_reference_stamp_sec = anchor_sec
        self.last_episode_anchor_stamp_sec = anchor_sec
        self.frames_written += 1
        self.session_frames_written += 1
        self.current_episode_frames += 1
        self._log_sync({
            "event": "frame",
            "frame": self.frames_written,
            "current_episode_frames": self.current_episode_frames,
            "wall_time": now,
            "anchor_camera": reference_name,
            "anchor_stamp_sec": anchor_sec,
            "frame_interval_ms": frame_interval_ms,
            "state_interpolation_alpha": interpolation_alpha,
            "sync_deltas_ms": sync_deltas_ms,
            "receive_ages_ms": receive_ages_ms,
        })
        if now - self.last_progress >= 5.0:
            elapsed = now - self.last_progress
            window_frames = self.session_frames_written - self.last_progress_frame_count
            fps = window_frames / elapsed if elapsed > 0 else 0.0
            self.get_logger().info(
                f"frames={self.frames_written}, current_episode_frames={self.current_episode_frames}, "
                f"saved_episodes={self.saved_episodes}, waiting_ticks={self.record_ticks_waited}, "
                f"invalidated={self.episodes_invalidated}, recording={self.is_recording}, effective_fps={fps:.2f}"
            )
            self.last_progress = now
            self.last_progress_frame_count = self.session_frames_written

    def _log_sync(self, event: dict[str, Any]) -> None:
        if self.sync_log is None:
            return
        self.sync_log.write(json.dumps(event, ensure_ascii=False) + "\n")

    def has_pending_episode(self) -> bool:
        if self.dataset is None:
            return False
        return self.dataset.has_pending_frames()

    def _reset_episode_state(self) -> None:
        """Return all in-memory episode bookkeeping to the paused state."""

        self.current_episode_frames = 0
        self.is_recording = False
        self.episode_invalid = False
        self.episode_invalid_reason = None
        self.last_episode_anchor_stamp_sec = None
        self.reported_image_overflows.clear()
        self.subtasks.reset()
        if getattr(self, "dagger", None) is not None:
            self.dagger.reset()

    def _acquire_motion_lock(self) -> bool:
        """Hold the robot motion lock only while an episode is recording."""

        if self.motion_lock_handle is None:
            return True
        try:
            fcntl.flock(self.motion_lock_handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self.get_logger().warn("episode start refused: motion lock is held by a reset command")
            return False
        return True

    def _release_motion_lock(self) -> None:
        if self.motion_lock_handle is not None:
            fcntl.flock(self.motion_lock_handle.fileno(), fcntl.LOCK_UN)

    def _clear_warmup_images(self) -> None:
        """Drop the anchor backlog, retaining one cross-camera boundary neighbor.

        A camera may lead the first new anchor by just a few milliseconds.
        Clearing that neighbor manufactures a gap when the next frame is more
        than MAX_SYNC_DELTA_SEC away. This is matching context, not a saved
        pre-start row: normal timestamp/age checks still apply to every row.
        """
        for name, buffer in self.image_buffers.items():
            neighbor = self.latest_images.get(name) if name != self.sync_reference_camera else None
            buffer.clear()
            if neighbor is not None:
                buffer.append(neighbor)

    def start_episode(self, reason: str, request_id: str | None = None) -> bool:
        # Both keyboard and VR clients need an explicit outcome, including a
        # rejected start. Sending a command alone does not mean recording began.
        rejection = None
        if self.save_failed:
            rejection = "previous save failed; inspect dataset and restart the collector"
        elif self.dataset is None:
            rejection = "dataset is not ready yet"
        elif self.is_recording:
            rejection = "episode is already recording"
        elif self.episode_invalid:
            rejection = "invalid episode must be discarded before starting another episode"
        elif self.has_pending_episode() or self.current_episode_frames > 0:
            rejection = "pending episode frames still exist"
        elif not self._acquire_motion_lock():
            rejection = "robot motion lock is busy"
        if rejection:
            self.get_logger().warn(rejection)
            self._write_command_status("start", False, request_id, message=rejection)
            return False
        episode_index = self.saved_episodes
        # Warmup frames are useful for dataset creation but belong before the
        # operator's start command. Clear them so a full warmup FIFO is not
        # mistaken for an overflow in the new episode.
        reference_baseline = (
            self.image_buffers[self.sync_reference_camera][-1].stamp_sec
            if self.sync_reference_camera and self.image_buffers[self.sync_reference_camera]
            else None
        )
        self._clear_warmup_images()
        self._reset_episode_state()
        self.is_recording = True
        # Do not use an image captured before the operator pressed Enter. The
        # next timer tick waits for a strictly newer reference-camera frame.
        self.last_reference_stamp_sec = reference_baseline
        # Start a fresh reporting window so time spent paused between episodes
        # does not artificially lower the displayed effective FPS.
        self.started_sec = time.time()
        if getattr(self, "dagger", None) is not None:
            self.dagger.started_ns = time.time_ns()
        self.last_progress = self.started_sec
        self.last_progress_frame_count = self.session_frames_written
        self._log_sync({
            "event": "episode_started",
            "episode_index": episode_index,
            "reason": reason,
            "wall_time": time.time(),
        })
        self.get_logger().info(f"episode {episode_index} started ({reason})")
        self._write_command_status(
            "start", True, request_id, episode_index=episode_index, message="episode started"
        )
        return True

    def mark_subtask(self, request_id: str | None = None) -> bool:
        """Close a nonempty accepted-frame span, saving on the final mark.

        Commands and add_frame run on the same recorder thread, so the boundary
        cannot race a frame write. Frames still waiting in camera FIFOs are not
        included: this is an operator annotation, not hardware time alignment.
        """
        if not self.is_recording or self.episode_invalid or self.save_failed:
            self._write_command_status("mark_subtask", False, request_id,
                                       message="a valid recording episode is required")
            return False
        try:
            complete = self.subtasks.mark(self.current_episode_frames)
        except ValueError as exc:
            self._write_command_status("mark_subtask", False, request_id, message=str(exc))
            return False
        # Do not write/flush a sidecar or relabel all frames here. The small
        # receipt is asynchronous; spans/labels are materialized only on save.
        if complete:
            return self.save_current_episode("subtasks_complete", request_id)
        self._write_command_status("mark_subtask", True, request_id, episode_index=self.saved_episodes,
                                   frames=self.current_episode_frames, message="subtask confirmed")
        return True

    def save_current_episode(self, reason: str, request_id: str | None = None) -> bool:
        if self.save_failed:
            self._write_command_status("save", False, request_id, message="save failed: previous result uncertain; no retry")
            return False
        if self.episode_invalid:
            invalid_reason = self.episode_invalid_reason or "unknown synchronization failure"
            self.get_logger().warn(
                f"save rejected because episode is invalid ({invalid_reason}); discarding entire episode"
            )
            return self.discard_current_episode(f"invalid:{invalid_reason}", request_id)
        if (getattr(self, "dagger", None) is not None and self.is_recording
                and self.dataset is not None and self.current_episode_frames == 0):
            return self.discard_current_episode("empty_dagger_episode", request_id)
        if self.dataset is None or not self.has_pending_episode() or self.current_episode_frames <= 0:
            self.is_recording = False
            self._release_motion_lock()
            self.get_logger().warn(f"no pending episode to save ({reason})")
            self._write_command_status(
                "save",
                False,
                request_id,
                message="no pending episode to save",
            )
            return False
        episode_index = self.saved_episodes
        episode_frames = self.current_episode_frames
        subtask_status = self.subtasks.progress()
        self.get_logger().info(
            f"saving episode {episode_index} with {episode_frames} frame(s) ({reason})"
        )
        try:
            annotation_paths = None
            if self.subtasks.enabled:
                manifest = self.subtasks.manifest(
                    episode_index, self.args.task_name, episode_frames, self.args.fps, reason,
                )
                self.subtasks.apply_to_buffer(self.dataset, episode_frames)
                annotation_paths = prepare_annotation(Path(self.args.output_dir), manifest)
                subtask_status.update(complete=manifest["annotation_complete"],
                                      needs_review=manifest["needs_review"],
                                      annotation_file=str(annotation_paths[1]))
            self.dataset.save_episode()
            if annotation_paths is not None:
                os.replace(*annotation_paths)
        except Exception as exc:
            self.save_failed = True
            self.stop_requested = True
            self.is_recording = False
            self._release_motion_lock()
            self.get_logger().error(f"failed to save episode {episode_index}: {exc}")
            self._write_command_status(
                "save",
                False,
                request_id,
                episode_index=episode_index,
                frames=episode_frames,
                message=f"save failed: {exc}",
            )
            return False
        self.saved_episodes += 1
        self.task_episode_counts[self.args.task_name] += 1
        self.session_saved_by_task[self.args.task_name] += 1
        self._log_sync(
            {
                "event": "episode_saved",
                "episode_index": episode_index,
                "frames": episode_frames,
                "reason": reason,
                "total_saved_episodes": self.saved_episodes,
                "wall_time": time.time(),
            }
        )
        self._reset_episode_state()
        self._release_motion_lock()
        self._write_command_status(
            "save",
            True,
            request_id,
            episode_index=episode_index,
            frames=episode_frames,
            message="episode saved",
            subtask_status=subtask_status,
        )
        return True

    def discard_current_episode(self, reason: str, request_id: str | None = None) -> bool:
        if self.save_failed:
            self._write_command_status("discard", False, request_id,
                                       message="discard failed: previous save result uncertain; inspect dataset")
            return False
        invalid_reason = self.episode_invalid_reason
        has_buffered_frames = self.has_pending_episode() and self.current_episode_frames > 0
        if self.dataset is None or (not has_buffered_frames and not self.episode_invalid
                                    and not (getattr(self, "dagger", None) and self.is_recording)):
            self.is_recording = False
            self._release_motion_lock()
            self.get_logger().warn(f"no pending episode to discard ({reason})")
            self._write_command_status(
                "discard",
                False,
                request_id,
                message="no pending episode to discard",
            )
            return False
        discarded_frames = self.current_episode_frames
        self.get_logger().warn(
            f"discarding current episode buffer with {discarded_frames} frame(s) ({reason})"
        )
        if has_buffered_frames:
            try:
                # LeRobot 0.6.0 clears image features but leaves PNGs used to
                # encode video features. Resolve only this unsaved episode's
                # temporary directories before the buffer is reset. The public
                # clear call joins image writers before we remove leftovers.
                writer = getattr(self.dataset, "writer", self.dataset)
                image_dir = getattr(writer, "_get_image_file_dir", None)
                video_dirs = []
                if callable(image_dir):
                    images_root = (Path(self.args.output_dir) / "images").resolve()
                    episode_name = f"episode-{self.saved_episodes:06d}"
                    for key in self.dataset.meta.video_keys:
                        directory = Path(image_dir(self.saved_episodes, key)).resolve()
                        if directory.relative_to(images_root).parts != (key, episode_name):
                            raise RuntimeError(f"Unexpected temporary video directory: {directory}")
                        video_dirs.append(directory)
                self.dataset.clear_episode_buffer(delete_images=True)
                for directory in video_dirs:
                    if directory.is_dir():
                        shutil.rmtree(directory)
            except Exception as exc:
                self.is_recording = False
                self._release_motion_lock()
                self.get_logger().error(f"failed to discard current episode: {exc}")
                self._write_command_status(
                    "discard",
                    False,
                    request_id,
                    frames=discarded_frames,
                    message=f"discard failed: {exc}",
                )
                return False
        self.frames_written -= discarded_frames
        self.session_frames_written -= discarded_frames
        self.discarded_episodes += 1
        self.session_discarded_by_task[self.args.task_name] += 1
        self._log_sync(
            {
                "event": "episode_discarded",
                "reason": reason,
                "invalid_reason": invalid_reason,
                "discarded_frames": discarded_frames,
                "total_discarded_episodes": self.discarded_episodes,
                "wall_time": time.time(),
            }
        )
        self._reset_episode_state()
        self._release_motion_lock()
        self._write_command_status(
            "discard",
            True,
            request_id,
            frames=discarded_frames,
            message=(f"invalid episode discarded: {invalid_reason}" if invalid_reason else "episode discarded"),
        )
        return True

    def request_quit(self, reason: str) -> None:
        self._log_sync({"event": "quit_requested", "reason": reason, "wall_time": time.time()})
        self.stop_requested = True

    def process_control_commands(self) -> None:
        while True:
            if self.mark_status_future is not None:
                if not self.mark_status_future.done():
                    return  # Recording timers continue; only commands wait.
                try:
                    self.mark_status_future.result()
                except Exception as exc:
                    self.get_logger().error(f"subtask acknowledgement failed: {exc}; stopping session")
                    self.stop_requested = True
                    return
                self.mark_status_future = None
            try:
                command_line = self.control_queue.get_nowait()
            except Empty:
                break

            parts = command_line.split(maxsplit=1)
            command = parts[0]
            request_id = parts[1] if len(parts) == 2 else None
            if command in ("start", "enter", "resume"):
                self.start_episode(f"command:{command}", request_id)
            elif command == "save":
                self.save_current_episode("command:save", request_id)
            elif command == "mark_subtask":
                self.mark_subtask(request_id)
            elif command == "discard":
                self.discard_current_episode("command:discard", request_id)
            elif command == "quit":
                self.request_quit("command:quit")
            else:
                self.get_logger().warn(f"unknown control command: {command}")

    def finish(self) -> None:
        self.stop_requested = True
        self.mark_status_writer.shutdown(wait=True)
        if self.dataset is not None and not self.save_failed:
            if self.episode_invalid:
                self.discard_current_episode("shutdown:invalid_episode")
            elif getattr(self, "dagger", None) is not None and self.is_recording:
                self.discard_current_episode("shutdown:unconfirmed_dagger_trial")
            elif self.has_pending_episode() and self.current_episode_frames > 0:
                self.save_current_episode("shutdown")
        if self.dataset is not None:
            self.dataset.finalize()
        if self.sync_log is not None:
            self.sync_log.flush()
            self.sync_log.close()
            self.sync_log = None
        self._release_motion_lock()
        if self.motion_lock_handle is not None:
            self.motion_lock_handle.close()
            self.motion_lock_handle = None


def parse_camera_args(items: list[str], defaults: dict[str, str], camera_type: str) -> dict[str, str]:
    """Resolve built-in camera names and ``name=/custom/topic`` entries."""

    cameras: dict[str, str] = {}
    for item in items:
        if "=" in item:
            name, topic = item.split("=", 1)
            name = name.strip()
            topic = topic.strip()
            if not name or not topic:
                raise SystemExit(f"Invalid {camera_type} camera mapping: {item!r}")
            cameras[name] = topic
        else:
            if item not in defaults:
                raise SystemExit(f"Unknown {camera_type} camera '{item}'. Use name=/topic for custom cameras.")
            cameras[item] = defaults[item]
    return cameras


def safe_repo_id(value: str) -> str:
    value = value.strip().replace(" ", "_")
    if "/" in value:
        return value
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", value).strip("_")
    return f"local/{cleaned or 'autolife_dataset'}"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        epilog=(
            "Examples:\n"
            "  record_lerobot_official.py --output-dir /tmp/dataset --task-name 'pick up the bottle'\n"
            "  record_lerobot_official.py --output-dir /tmp/dataset --with-head --with-waist --with-depth\n"
            "  record_lerobot_official.py --max-sync-delta-sec --help"
        ),
    )
    parser.add_argument(
        "--output-dir", type=Path, required=True,
        help="Root directory of the LeRobotDataset. Existing metadata and episodes are resumed.",
    )
    parser.add_argument(
        "--repo-id", default=None,
        help="Dataset repository identifier stored in metadata. If omitted, derive local/<output-name>.",
    )
    parser.add_argument(
        "--motion-lock-file",
        type=Path,
        default=None,
        help="Advisory flock held only while an episode is actively recording.",
    )
    parser.add_argument(
        "--task-name", default="mango_pick",
        help="Natural-language task instruction written into every frame, for example 'pick up the bottle'.",
    )
    parser.add_argument(
        "--subtasks-json", default="[]", type=parse_subtasks,
        help='Ordered subtask texts as a JSON array. Empty [] disables annotation. Example: \'["pick", "place"]\'.',
    )
    parser.add_argument(
        "--robot-type", default="autolife_s1",
        help="Robot type label stored in LeRobot metadata; it does not change ROS topics.",
    )
    parser.add_argument(
        "--fps", type=int, default=30,
        help="Target dataset frame rate. The reference camera supplies one frame anchor per tick.",
    )
    parser.add_argument(
        "--image-source",
        choices=("shm", "ros"),
        default="shm",
        help="Image transport: 'shm' reads camera shared memory directly; 'ros' subscribes to bridge topics.",
    )
    parser.add_argument(
        "--image-poll-fps",
        type=float,
        default=DEFAULT_IMAGE_POLL_FPS,
        help="Direct-SHM metadata polling frequency. This is the camera polling rate, not dataset FPS; new source timestamps are buffered once.",
    )
    parser.add_argument(
        "--duration", type=float, default=None,
        help="Optional automatic recording duration in seconds. Leave unset for keyboard/FIFO-controlled recording.",
    )
    parser.add_argument(
        "--camera",
        dest="camera_items",
        action="append",
        default=None,
        help="RGB camera to record. Use a built-in name or name=/topic for ROS mode; repeat this option for multiple cameras.",
    )
    parser.add_argument(
        "--with-depth",
        action="store_true",
        help="Enable the default uint16 depth camera and native depth-video encoding from LeRobot 0.6 or newer.",
    )
    parser.add_argument(
        "--depth-camera",
        dest="depth_camera_items",
        action="append",
        default=None,
        help="Depth camera to record. Use a built-in name or name=/topic; specifying it automatically enables depth recording.",
    )
    parser.add_argument(
        "--with-head", action="store_true",
        help="Append neck_roll, neck_pitch, and neck_yaw to state/action. Dataset state/action becomes 19-D without waist or 23-D with waist.",
    )
    waist_group = parser.add_mutually_exclusive_group()
    waist_group.add_argument(
        "--with-waist", action="store_true",
        help="Append leg_ankle, leg_knee, waist_pitch, and waist_yaw to state/action. Dataset state/action becomes 20-D without head or 23-D with head.",
    )
    waist_group.add_argument(
        "--with-upper-waist", action="store_true",
        help="Append only waist_pitch and waist_yaw to state/action; leg_ankle and leg_knee are excluded. Dataset state/action becomes 18-D without head or 21-D with head. Mutually exclusive with --with-waist.",
    )
    parser.add_argument(
        "--camera-warmup-sec", type=float, default=3.0,
        help="Maximum time to wait for all requested cameras to expose valid frames before initialization.",
    )
    parser.add_argument(
        "--state-warmup-sec",
        type=float,
        default=10.0,
        help="Maximum time to wait for the first complete state packet before camera polling begins.",
    )
    parser.add_argument(
        "--state-ready-file",
        default=None,
        help="Optional path for an atomic readiness file written after the first complete state packet.",
    )
    parser.add_argument(
        "--min-cameras", type=int, default=1,
        help="Minimum number of cameras that must be valid before recording starts; missing optional cameras invalidate ticks.",
    )
    parser.add_argument(
        "--sync-reference-camera",
        default=None,
        help="Camera whose new frames anchor each dataset tick. It must be active; defaults to hand_left, otherwise the first active camera.",
    )
    parser.add_argument(
        "--max-sync-delta-sec",
        type=float,
        default=0.03,
        help="Maximum absolute timestamp difference from the reference image for other images, state, and action. A failed match invalidates the episode.",
    )
    parser.add_argument(
        "--max-image-age-sec", type=float, default=0.15,
        help="Reject an image older than this many seconds at the recording tick.",
    )
    parser.add_argument(
        "--max-state-age-sec", type=float, default=0.15,
        help="Reject a joint-state sample older than this many seconds at the recording tick.",
    )
    parser.add_argument(
        "--max-state-interpolation-gap-sec",
        type=float,
        default=0.05,
        help="Maximum time span between the two joint-state samples used for linear interpolation at an image timestamp.",
    )
    parser.add_argument(
        "--max-action-hold-sec",
        type=float,
        default=0.5,
        help="Maximum age of the latest causal action target held until an image timestamp; older targets invalidate the tick.",
    )
    parser.add_argument(
        "--max-frame-interval-error-ratio",
        type=float,
        default=0.45,
        help="Maximum relative error of the reference-camera frame interval compared with the requested 1/FPS period.",
    )
    parser.add_argument(
        "--sync-image-buffer-size", type=int, default=DEFAULT_SYNC_IMAGE_BUFFER_SIZE,
        help="Maximum number of recent frames kept per camera in the image FIFO. Overflow invalidates the active episode.",
    )
    parser.add_argument(
        "--sync-signal-buffer-size", type=int, default=DEFAULT_SYNC_SIGNAL_BUFFER_SIZE,
        help="Maximum number of recent joint/action samples kept for timestamp matching.",
    )
    parser.add_argument(
        "--fallback-action-to-state", action="store_true",
        help="Use the measured state as action when no action topic is available; intended only for camera/link tests.",
    )
    parser.add_argument(
        "--state-topic", default=STATE_TOPIC,
        help="ROS2 JSON-string topic carrying the complete physical joint status packet.",
    )
    parser.add_argument(
        "--action-mode",
        choices=("joint", "eef", "status_target"),
        default="status_target",
        help=(
            "Action source: 'status_target' reads schema-aligned targets from the status topic "
            "(recommended for PI0.5 collection), 'joint' reads explicit joint/gripper command topics, "
            "and 'eef' records a 15-d EEF target for debugging. EEF names intentionally differ from state."
        ),
    )
    parser.add_argument("--action-arm-topic", default=ACTION_ARM_TOPIC, help="ROS2 JSON topic for arm and whole-body joint target commands.")
    parser.add_argument("--action-gripper-topic", default=ACTION_GRIPPER_TOPIC, help="ROS2 JSON topic for left/right gripper target commands.")
    parser.add_argument("--action-eef-topic", default=ACTION_EEF_TOPIC, help="ROS2 JSON topic for end-effector pose targets when --action-mode=eef.")
    parser.add_argument("--action-height-topic", default=ACTION_HEIGHT_TOPIC, help="ROS2 JSON topic for robot height targets when --action-mode=eef.")
    parser.add_argument("--image-writer-threads", type=int, default=4, help="Number of background image-writer threads used by LeRobot.")
    parser.add_argument("--vcodec", default="h264", help="RGB video codec passed to LeRobot; h264 favors speed, libsvtav1 favors smaller files.")
    parser.add_argument("--video-crf", type=float, default=None, help="Optional RGB video CRF/quality value.")
    parser.add_argument("--video-preset", default=None, help="Optional RGB video encoder preset.")
    parser.add_argument("--video-gop", type=int, default=None, help="Optional RGB video GOP/keyframe interval.")
    parser.add_argument("--depth-vcodec", default="hevc", help="Depth video codec used by LeRobot 0.6; hevc is the default 12-bit-capable choice.")
    parser.add_argument("--depth-min", type=float, default=0.05, help="Minimum physical depth in metres passed to the depth quantizer.")
    parser.add_argument("--depth-max", type=float, default=10.0, help="Maximum physical depth in metres passed to the depth quantizer.")
    parser.add_argument("--depth-shift", type=float, default=3.5, help="Logarithmic depth quantizer shift used by LeRobot; affects depth precision distribution.")
    parser.add_argument(
        "--depth-use-log",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable logarithmic depth quantization; use --no-depth-use-log for linear depth quantization.",
    )
    parser.add_argument("--batch-encoding-size", type=int, default=1, help="Number of frames submitted together to the video encoder.")
    parser.add_argument("--streaming-encoding", action="store_true", help="Encode videos while recording when supported; reduces finalization work but increases runtime load.")
    parser.add_argument("--encoder-queue-maxsize", type=int, default=30, help="Maximum pending frame batches in the asynchronous encoder queue.")
    parser.add_argument("--encoder-threads", type=int, default=None, help="Optional number of encoder worker threads; unset lets LeRobot choose.")
    parser.add_argument("--video-files-size-in-mb", type=int, default=None, help="Optional maximum size of each video file before LeRobot starts a new segment.")
    parser.add_argument("--data-files-size-in-mb", type=int, default=None, help="Optional maximum size of each Parquet data file before LeRobot starts a new segment.")
    parser.add_argument("--control-fifo", default=None, help="Optional named pipe receiving start/mark_subtask/save/discard/quit commands.")
    parser.add_argument(
        "--status-file",
        default=None,
        help="Optional JSON status file atomically updated after save/discard completes.",
    )
    parser.add_argument(
        "--episode-event-file",
        default=None,
        help="Optional JSON event file used to notify the launcher about saved, discarded, or invalid episodes.",
    )
    show_requested_parameter_help(parser)
    parser.add_argument("--dagger", action="store_true", help="Record causal HG-DAgger action provenance; requires joint proxy topics.")
    parser.add_argument("--dagger-start-timeout-sec", type=float, default=30.0,
                        help="Maximum wait for the first DAgger action/label barrier, before accepting any frame.")
    args = parser.parse_args()
    args.subtasks = args.subtasks_json
    if args.dagger and (args.action_mode != "joint" or args.fallback_action_to_state or args.subtasks):
        parser.error("DAgger requires --action-mode joint, no state fallback, and no subtask plan")
    if not np.isfinite(args.dagger_start_timeout_sec) or args.dagger_start_timeout_sec <= 0:
        parser.error("--dagger-start-timeout-sec must be finite and positive")
    if args.depth_min < 0 or args.depth_max <= args.depth_min:
        parser.error("depth range must satisfy 0 <= --depth-min < --depth-max")
    if args.fps <= 0 or args.min_cameras < 1 or args.image_poll_fps <= 0:
        parser.error("--fps and --min-cameras must be positive")
    if args.state_warmup_sec <= 0 or args.camera_warmup_sec <= 0:
        parser.error("state and camera warmup timeouts must be positive")
    if (
        args.max_sync_delta_sec <= 0
        or args.max_image_age_sec <= 0
        or args.max_state_age_sec <= 0
        or args.max_state_interpolation_gap_sec <= 0
    ):
        parser.error("synchronization delta and freshness limits must be positive")
    if args.max_action_hold_sec <= 0:
        parser.error("--max-action-hold-sec must be positive")
    if not 0 < args.max_frame_interval_error_ratio < 1:
        parser.error("--max-frame-interval-error-ratio must be between 0 and 1")
    if args.sync_image_buffer_size < 2 or args.sync_signal_buffer_size < 2:
        parser.error("synchronization buffer sizes must be at least 2")

    camera_items = args.camera_items or list(DEFAULT_RGB_CAMERA_TOPICS)
    rgb_cameras = parse_camera_args(camera_items, DEFAULT_RGB_CAMERA_TOPICS, "RGB")
    args.with_depth = args.with_depth or bool(args.depth_camera_items)
    depth_items = args.depth_camera_items or (list(DEFAULT_DEPTH_CAMERA_TOPICS) if args.with_depth else [])
    args.depth_cameras = parse_camera_args(depth_items, DEFAULT_DEPTH_CAMERA_TOPICS, "depth")
    duplicate_names = set(rgb_cameras).intersection(args.depth_cameras)
    if duplicate_names:
        parser.error("camera names cannot be both RGB and depth: " + ", ".join(sorted(duplicate_names)))
    args.cameras = {**rgb_cameras, **args.depth_cameras}
    if args.image_source == "shm":
        unsupported = sorted(set(args.cameras).difference(CAMERA_SPECS))
        if unsupported:
            parser.error(
                "direct SHM mode only supports built-in camera names: "
                + ", ".join(unsupported)
                + "; use --image-source ros for custom topic mappings"
            )
    args.repo_id = safe_repo_id(args.repo_id or args.output_dir.name)
    return args


def main() -> None:
    args = parse_args()
    # The ROS recorder is single-threaded; avoid a second OpenCV worker pool
    # competing with LeRobot image writers and the robot's control processes.
    cv2.setNumThreads(1)
    if args.dagger:
        import torch
        import pyarrow as pa
        torch.set_num_threads(1)
        torch.set_num_interop_threads(1)
        pa.set_cpu_count(1)
        pa.set_io_thread_count(1)
        print(f"Recorder resources: cpus={sorted(os.sched_getaffinity(0))}, "
              f"nice={os.getpriority(os.PRIO_PROCESS, 0)}, torch_threads=1, arrow_threads=1",
              flush=True)
    rclpy.init()
    node = OfficialLeRobotRecorder(args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def on_signal(signum, frame):
        if signum == signal.SIGUSR1:
            node.control_queue.put("save")
            return
        if signum == signal.SIGUSR2:
            node.control_queue.put("discard")
            return
        node.control_queue.put("quit")

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGUSR1, on_signal)
    signal.signal(signal.SIGUSR2, on_signal)
    try:
        # Start FIFO handling before warmup so launcher cleanup can stop a
        # recorder that is still waiting for state or camera readiness.
        node.start_control_listener()
        # Phase 1 deliberately runs before camera processes are launched. The
        # state callback writes ``state_ready_file``; the launcher observes it
        # and only then starts producing and publishing images.
        state_deadline = time.time() + args.state_warmup_sec
        while rclpy.ok() and time.time() < state_deadline and not node.stop_requested and not node.state_buffer:
            executor.spin_once(timeout_sec=0.05)
            node.process_control_commands()
        if node.stop_requested:
            return
        if not node.state_buffer:
            raise RuntimeError(
                f"No complete state packet received within {args.state_warmup_sec:g}s. "
                "Check the state topic and selected joint schema."
            )

        # Phase 2 starts after the launcher sees the state-ready marker. Wait
        # for every requested camera when possible, then let create_dataset()
        # apply MIN_CAMERAS and required-depth checks at the deadline.
        camera_deadline = time.time() + args.camera_warmup_sec
        while (
            rclpy.ok()
            and time.time() < camera_deadline
            and not node.stop_requested
            and not node.all_requested_cameras_seen()
        ):
            executor.spin_once(timeout_sec=0.05)
            node.process_control_commands()
        if node.stop_requested:
            return
        node.create_dataset()
        while rclpy.ok() and not node.stop_requested:
            executor.spin_once(timeout_sec=0.05)
            node.process_control_commands()
    finally:
        node.finish()
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
    print(f"frames_written={node.frames_written}")
    print(f"record_ticks_waited={node.record_ticks_waited}")
    print(f"episodes_invalidated={node.episodes_invalidated}")
    print(f"image_buffer_overflows={node.image_buffer_overflows}")
    print(f"episodes_saved={node.saved_episodes}")
    print(f"episodes_discarded={node.discarded_episodes}")
    print(f"dataset={args.output_dir}")


if __name__ == "__main__":
    main()
