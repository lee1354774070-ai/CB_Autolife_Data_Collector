"""Read paired RGB-D and wrist JPEG frames through the vendor SDK 3.x."""

from __future__ import annotations

import time

from camera_config import CAMERA_SPECS
from shm_camera import (
    EPOCH_NS_MAX, EPOCH_NS_MIN, PIXEL_FORMAT_BGR, PIXEL_FORMAT_DEPTH16,
    PIXEL_FORMAT_MJPEG, PIXEL_FORMAT_RGB, ShmFrame,
)


CHANNELS = {
    "alshm_camera_rgbd_head_rgbd": {"color": "rgbd_head_color", "depth": "rgbd_head_depth"},
    "alshm_camera_hand_left_jpeg": {"JPEG": "hand_left"},
    "alshm_camera_hand_right_jpeg": {"JPEG": "hand_right"},
}


class AlshmCameraReader:
    def __init__(self, camera_names):
        try:
            from autolife_robot_sdk import GLOBAL_VARS
            from autolife_robot_sdk.shm import ReadStatus, ShmError
            from autolife_robot_sdk.shm.camera import PixelFormat
            from autolife_robot_sdk.shm.catalog import open_camera_channel, robot_camera_channels
        except ImportError as exc:
            raise RuntimeError("ALSHM requires the vendor SDK 3.x in the recorder's Python environment") from exc
        self.readers = []
        self.shm_error = ShmError
        self.publisher_lost = (ReadStatus.CLOSED, ReadStatus.OWNER_LOST)
        self.formats = {
            PixelFormat.BGR: (PIXEL_FORMAT_BGR, 3, 1),
            PixelFormat.RGB: (PIXEL_FORMAT_RGB, 3, 1),
            PixelFormat.Z16: (PIXEL_FORMAT_DEPTH16, 1, 2),
            PixelFormat.MJPG: (PIXEL_FORMAT_MJPEG, 3, 0),
        }
        requested = set(camera_names)
        found = set()
        try:
            for channel in robot_camera_channels(
                robot_model=GLOBAL_VARS.ACTIVE_ROBOT_MODEL, robot_version=GLOBAL_VARS.ACTIVE_ROBOT_VERSION
            ):
                names = CHANNELS.get(channel.spec.name.lstrip("/"), {})
                sensors = {
                    plane.sensor_id: names[plane.output]
                    for plane in channel.spec.planes if names.get(plane.output) in requested
                }
                if sensors:
                    self.readers.append((channel.spec.name, open_camera_channel(channel, start="latest"), sensors))
                    found.update(sensors.values())
            if requested != found:
                raise RuntimeError(f"Required SDK camera streams missing: {', '.join(sorted(requested - found))}")
        except Exception:
            self.close()
            raise

    def read_frames(self):
        frames = []
        for channel, reader, sensors in self.readers:
            try:
                # A brief recorder stall must not skip an available camera frame.
                result = reader.read(mode="sequential", timeout=0)
            except self.shm_error as exc:
                raise RuntimeError(f"{channel} camera read failed: {exc}") from exc
            if result.status in self.publisher_lost:
                raise RuntimeError(f"{channel} camera publisher {result.status.value}. Restart collection after it is ready.")
            if result.frame is None:
                continue
            received_sec = time.time()
            planes = {plane.sensor_id: plane for plane in result.frame.planes}
            if not sensors.keys() <= planes.keys():
                raise ValueError(f"{channel} is missing a required image plane")
            if len(sensors) > 1 and len({(planes[i].width, planes[i].height) for i in sensors}) != 1:
                raise ValueError(f"{channel} RGB and depth have different native resolutions")

            # The shared frame metadata has an optional epoch stamp. The
            # SDK supplies per-plane publication times on current S3 builds.
            timestamp = result.frame.meta.publish_epoch_ns or max(
                planes[sensor_id].publish_epoch_ns or 0 for sensor_id in sensors
            )
            if not EPOCH_NS_MIN <= timestamp <= EPOCH_NS_MAX:
                raise ValueError(f"{channel} has no valid publication timestamp")

            for sensor_id, name in sensors.items():
                plane = planes[sensor_id]
                if plane.pixel_format not in self.formats:
                    raise ValueError(f"{name} has unsupported SDK pixel format {plane.pixel_format}")
                pixel_format, channels, bytes_per_channel = self.formats[plane.pixel_format]
                data = bytes(plane.data)
                if bytes_per_channel and len(data) != plane.width * plane.height * channels * bytes_per_channel:
                    data = plane.numpy().tobytes()
                frame = ShmFrame(timestamp, plane.width, plane.height, channels, pixel_format, len(data), data)
                frames.append((name, CAMERA_SPECS[name], frame, received_sec))
        return frames

    def close(self):
        for _, reader, _ in self.readers:
            reader.close()
        self.readers.clear()

