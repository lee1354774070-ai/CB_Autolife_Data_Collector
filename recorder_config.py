#!/usr/bin/env python3
"""Recorder configuration, independent of ROS, NumPy and LeRobot.

Validate options before creating a dataset or opening robot interfaces. The
launcher help uses this parser too, so option descriptions have one owner.
"""

from __future__ import annotations

import argparse
import math
import os
import re
import socket
from pathlib import Path

from camera_config import (
    CAMERA_SPECS, DEFAULT_DEPTH_CAMERA_TOPICS, DEFAULT_RGB_CAMERA_TOPICS,
    DEFAULT_IMAGE_POLL_FPS, DEFAULT_SYNC_IMAGE_BUFFER_SIZE, DEFAULT_SYNC_SIGNAL_BUFFER_SIZE,
)
from cli_help import show_requested_parameter_help
from subtask_annotations import parse_subtasks


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
STATE_EEF_TOPIC = f"/topic_arm_current_robot_eef_pose_{TOPIC_NODE_ID}"


def parse_camera_args(items: list[str], defaults: dict[str, str], camera_type: str) -> dict[str, str]:
    """Resolve built-in camera names and ``name=/custom/topic`` entries."""

    cameras: dict[str, str] = {}
    for item in items:
        if "=" in item:
            name, topic = item.split("=", 1)
            name = name.strip()
            topic = topic.strip()
            if not name or not topic.startswith('/'):
                raise SystemExit(f"Invalid {camera_type} camera mapping: {item!r}")
        else:
            if item not in defaults:
                raise SystemExit(f"Unknown {camera_type} camera '{item}'. Use name=/topic for custom cameras.")
            name, topic = item, defaults[item]
        # Logical camera names become feature keys and directories in LeRobot.
        if not re.fullmatch(r"[A-Za-z0-9_][A-Za-z0-9_.-]*", name) or name in cameras:
            raise SystemExit(f"Invalid or repeated {camera_type} camera name: {name!r}")
        cameras[name] = topic
    return cameras


def safe_repo_id(value: str) -> str:
    value = value.strip().replace(" ", "_")
    if "/" in value:
        return value
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", value).strip("_")
    return f"local/{cleaned or 'autolife_dataset'}"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Record synchronized AutoLife observations/actions as LeRobot v3 episodes.",
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
        "--robot-type", default=os.environ.get("ROBOT_TYPE", "autolife_s1"),
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
        help="Maximum wait for the first complete state packet before camera initialization and optional producer startup.",
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
        choices=("status_target", "joint"),
        default="status_target",
        help=(
            "Use 'status_target' for regular keyboard/VR/subtask collection: controller targets "
            "from the status topic, with measured gripper positions. DAgger requires 'joint': "
            "explicit body/gripper target commands from its verified controller proxies. "
            "EEF poses are optional additions via --with-eef-state/--with-eef-action."
        ),
    )
    parser.add_argument("--action-arm-topic", default=ACTION_ARM_TOPIC, help="ROS2 JSON topic for arm and whole-body joint target commands.")
    parser.add_argument("--action-gripper-topic", default=ACTION_GRIPPER_TOPIC, help="ROS2 JSON topic for left/right gripper target commands.")
    parser.add_argument("--with-eef-state", action="store_true", help="Append 14-D measured dual-arm EEF poses to joint state (robot frame, metres, quaternion XYZW).")
    parser.add_argument("--with-eef-action", action="store_true", help="Append 14-D reported dual-arm EEF targets to joint action; never substitutes measured state.")
    parser.add_argument("--state-eef-topic", default=STATE_EEF_TOPIC, help="Vendor current EEF pose report, required with --with-eef-state.")
    parser.add_argument("--action-eef-topic", default=ACTION_EEF_TOPIC, help="Vendor target EEF pose report, required with --with-eef-action.")
    parser.add_argument("--image-writer-threads", type=int, default=4, help="Number of background image-writer threads used by LeRobot.")
    parser.add_argument('--image-writer-max-pending', type=int, default=64,
                        help='Maximum unfinished image jobs shared by all cameras; overflow invalidates the episode.')
    parser.add_argument('--image-writer-max-mb', type=float, default=256,
                        help='Maximum raw image MiB in unfinished write jobs; overflow invalidates the episode.')
    parser.add_argument('--min-free-disk-gb', type=float, default=2,
                        help='Minimum free GiB, in addition to one raw episode estimate reserved for saving.')
    parser.add_argument('--max-episode-temp-gb', type=float, default=8,
                        help='Maximum raw-image-plus-overhead GiB admitted per episode; reaching it invalidates, never drops frames.')
    parser.add_argument('--max-episode-seconds', type=float, default=600,
                        help='Maximum wall duration after start, including waits. Exceeding it invalidates the episode.')
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
    parser.add_argument("--batch-encoding-size", type=int, default=1, help="Episodes per encoding batch; only 1 is supported by transactional saving.")
    parser.add_argument("--streaming-encoding", action="store_true", help="Legacy option: rejected because transactional episode saving requires non-streaming encoding.")
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
    parser.add_argument("--dagger", action="store_true", help="Record causal HG-DAgger action provenance; requires joint proxy topics.")
    parser.add_argument("--dagger-start-timeout-sec", type=float, default=30.0,
                        help="Maximum wait for the first DAgger action/label barrier, before accepting any frame.")
    return parser


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = build_parser()
    show_requested_parameter_help(parser, argv)
    args = parser.parse_args(argv)
    if args.with_eef_action and args.fallback_action_to_state:
        parser.error("EEF targets require genuine joint actions; disable --fallback-action-to-state")
    if args.streaming_encoding or args.batch_encoding_size != 1:
        parser.error('transactional saving requires --batch-encoding-size=1 and no --streaming-encoding')
    if args.image_writer_threads < 0 or args.image_writer_max_pending < 1:
        parser.error('image writer threads must be nonnegative and queue capacity positive')
    if any(not math.isfinite(v) or v <= 0 for v in (
            args.image_writer_max_mb, args.min_free_disk_gb, args.max_episode_temp_gb, args.max_episode_seconds)):
        parser.error('storage/memory/duration limits must be finite and positive')
    args.subtasks = args.subtasks_json
    if args.dagger and (args.action_mode != "joint" or args.fallback_action_to_state or args.subtasks):
        parser.error("DAgger requires --action-mode joint, no state fallback, and no subtask plan")
    if not math.isfinite(args.dagger_start_timeout_sec) or args.dagger_start_timeout_sec <= 0:
        parser.error("--dagger-start-timeout-sec must be finite and positive")
    timing_limits = (args.image_poll_fps, args.state_warmup_sec, args.camera_warmup_sec,
                     args.max_sync_delta_sec, args.max_image_age_sec, args.max_state_age_sec,
                     args.max_state_interpolation_gap_sec, args.max_action_hold_sec)
    if any(not math.isfinite(value) or value <= 0 for value in timing_limits):
        parser.error('camera rates, warmup and synchronization limits must be finite and positive')
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
        parser.error('--duration must be finite and positive')
    if any(not math.isfinite(value) for value in (args.depth_min, args.depth_max, args.depth_shift)):
        parser.error('depth range and shift must be finite')
    if args.depth_min < 0 or args.depth_max <= args.depth_min:
        parser.error("depth range must satisfy 0 <= --depth-min < --depth-max")
    if args.fps <= 0 or args.min_cameras < 1:
        parser.error("--fps and --min-cameras must be positive")
    for name in ('encoder_queue_maxsize', 'encoder_threads', 'video_gop',
                 'video_files_size_in_mb', 'data_files_size_in_mb'):
        value = getattr(args, name)
        if value is not None and value <= 0:
            parser.error(f"--{name.replace('_', '-')} must be positive")
    if args.video_crf is not None and (not math.isfinite(args.video_crf) or args.video_crf < 0):
        parser.error('--video-crf must be finite and nonnegative')
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
    if args.min_cameras > len(args.cameras):
        parser.error('--min-cameras exceeds the number of requested cameras')
    if args.sync_reference_camera is not None and args.sync_reference_camera not in args.cameras:
        parser.error('--sync-reference-camera must name a requested camera')
    if args.image_source == "shm":
        unsupported = sorted(set(args.cameras).difference(CAMERA_SPECS))
        if unsupported:
            parser.error(
                "direct SHM mode only supports built-in camera names: "
                + ", ".join(unsupported)
                + "; use --image-source ros for custom topic mappings"
            )
        if 'rgbd_head_depth' in rgb_cameras or set(args.depth_cameras).difference(DEFAULT_DEPTH_CAMERA_TOPICS):
            parser.error('direct SHM RGB/depth selections must match the physical stream type')
    args.repo_id = safe_repo_id(args.repo_id or args.output_dir.name)
    return args
