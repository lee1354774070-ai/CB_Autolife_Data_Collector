#!/usr/bin/env python3
"""Publish Autolife SHM camera frames as ROS2 sensor_msgs/Image topics."""

from __future__ import annotations

import argparse
import time

import rclpy
from builtin_interfaces.msg import Time
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image

from camera_config import CAMERA_SPECS, CameraSpec
from shm_camera import read_shm_frame, read_shm_metadata
EPOCH_NS_MIN = 946684800 * 1_000_000_000
EPOCH_NS_MAX = 4102444800 * 1_000_000_000


def stamp_from_ns(timestamp_ns: int, node: Node) -> Time:
    if EPOCH_NS_MIN <= timestamp_ns <= EPOCH_NS_MAX:
        stamp = Time()
        stamp.sec = int(timestamp_ns // 1_000_000_000)
        stamp.nanosec = int(timestamp_ns % 1_000_000_000)
        return stamp
    return node.get_clock().now().to_msg()


class ShmCameraTopicBridge(Node):
    def __init__(self, cameras: list[CameraSpec], fps: float):
        super().__init__("shm_camera_topic_bridge")
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.cameras = cameras
        self.image_publishers = {cam.name: self.create_publisher(Image, cam.topic, qos) for cam in cameras}
        self.last_source_stamp: dict[str, int] = {}
        self.published_counts = {cam.name: 0 for cam in cameras}
        self.last_status_time = time.time()
        self.create_timer(1.0 / fps, self._publish_once)

        self.get_logger().info("Publishing SHM cameras:")
        for cam in cameras:
            self.get_logger().info(f"  {cam.name}: {cam.topic}")

    def _publish_once(self) -> None:
        now = time.time()

        # The SHM producer exposes only its newest frame. Snapshot metadata for
        # every camera first, then copy image bytes only for a new generation.
        # This prevents a high bridge poll rate from repeatedly copying the
        # same multi-megabyte image before the camera publishes its next frame.
        metadata_by_camera = {}
        for cam in self.cameras:
            metadata = read_shm_metadata(cam)
            if metadata is None or self.last_source_stamp.get(cam.name) == metadata[0]:
                continue
            metadata_by_camera[cam.name] = metadata

        for cam in self.cameras:
            metadata = metadata_by_camera.get(cam.name)
            if metadata is None:
                continue
            frame = read_shm_frame(cam, metadata)
            if frame is None or frame.timestamp_ns != metadata[0]:
                continue

            if frame.pixel_format == 1 and frame.channels == 3:
                encoding = "bgr8"
                step = frame.width * 3
            elif frame.pixel_format == 2 and frame.channels == 1:
                encoding = "16UC1"
                step = frame.width * 2
            else:
                self.get_logger().warn(
                    f"Unsupported SHM format for {cam.name}: "
                    f"channels={frame.channels}, fmt={frame.pixel_format}"
                )
                continue

            msg = Image()
            msg.header.stamp = stamp_from_ns(frame.timestamp_ns, self)
            msg.header.frame_id = cam.frame_id
            msg.height = frame.height
            msg.width = frame.width
            msg.encoding = encoding
            msg.is_bigendian = 0
            msg.step = step
            msg.data = frame.data[: frame.byte_count]
            self.image_publishers[cam.name].publish(msg)

            self.last_source_stamp[cam.name] = frame.timestamp_ns
            self.published_counts[cam.name] += 1

        if now - self.last_status_time >= 5.0:
            status = ", ".join(f"{name}:{count}" for name, count in sorted(self.published_counts.items()))
            self.get_logger().info(f"Published frames in last window: {status}")
            self.published_counts = {cam.name: 0 for cam in self.cameras}
            self.last_status_time = now


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--cameras",
        nargs="+",
        default=["rgbd_head_color", "hand_left", "hand_right"],
        help=f"Camera names. Available: {', '.join(sorted(CAMERA_SPECS))}",
    )
    parser.add_argument("--fps", type=float, default=30.0, help="Maximum publish rate.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.fps <= 0:
        raise SystemExit("--fps must be greater than zero")
    unknown = [name for name in args.cameras if name not in CAMERA_SPECS]
    if unknown:
        raise SystemExit(f"Unknown camera name(s): {', '.join(unknown)}")

    rclpy.init()
    node = ShmCameraTopicBridge([CAMERA_SPECS[name] for name in args.cameras], args.fps)
    try:
        rclpy.spin(node)
    # ROS Jazzy may raise RCLError instead of ExternalShutdownException when
    # its default SIGTERM handler invalidates the context during spin().
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
