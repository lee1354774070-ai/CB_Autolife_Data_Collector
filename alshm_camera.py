"""Read paired RGB-D and wrist JPEG frames through the vendor SDK 3.x."""

from __future__ import annotations

from numbers import Integral
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
    def __init__(self, camera_names, logger=None):
        try:
            from autolife_robot_sdk import GLOBAL_VARS
            from autolife_robot_sdk.shm import ReadStatus, ShmError
            from autolife_robot_sdk.shm.camera import PixelFormat
            from autolife_robot_sdk.shm.catalog import open_camera_channel, robot_camera_channels
        except ImportError as exc:
            raise RuntimeError("ALSHM requires the vendor SDK 3.x in the recorder's Python environment") from exc
        self.readers = []
        self.logger = logger
        self.open_camera_channel = open_camera_channel
        self.decoded_wrists = {}
        self.last_wrists = {}
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
            channels = {channel.spec.name.lstrip("/"): channel for channel in robot_camera_channels(
                robot_model=GLOBAL_VARS.ACTIVE_ROBOT_MODEL, robot_version=GLOBAL_VARS.ACTIVE_ROBOT_VERSION
            )}
            for channel_name, channel in channels.items():
                names = CHANNELS.get(channel_name, {})
                sensors = {
                    plane.sensor_id: names[plane.output]
                    for plane in channel.spec.planes if names.get(plane.output) in requested
                }
                if sensors:
                    self.readers.append((channel.spec.name, open_camera_channel(channel, start="latest"), sensors))
                    found.update(sensors.values())
                    if channel_name.endswith("_jpeg"):
                        decoded = channels.get(channel_name.removesuffix("_jpeg") + "_decoded")
                        owner = getattr(channel, "module_name", None)
                        if owner and getattr(decoded, "module_name", None) == owner:
                            for name in sensors.values():
                                self.decoded_wrists[name] = decoded
            if requested != found:
                raise RuntimeError(f"Required SDK camera streams missing: {', '.join(sorted(requested - found))}")
        except Exception:
            self.close()
            raise

    def _frame_from_plane(self, plane, timestamp):
        pixel_format, channels, bytes_per_channel = self.formats[plane.pixel_format]
        data = bytes(plane.data)
        if bytes_per_channel and len(data) != plane.width * plane.height * channels * bytes_per_channel:
            data = plane.numpy().tobytes()
        return ShmFrame(timestamp, plane.width, plane.height, channels, pixel_format, len(data), data)

    def _recover_wrist_gap(self, name, plane, timestamp):
        # Older SDKs can expose a perfectly valid JPEG without exposure IDs.
        # Never bridge across an unidentified packet or rewind the last good
        # boundary on a late delivery.
        source_id, capture_id = getattr(plane, "source_frame_id", None), getattr(plane, "capture_id", None)
        previous = self.last_wrists.get(name)
        if previous is not None and timestamp <= previous[2]:
            return []
        if not all(isinstance(i, Integral) and not isinstance(i, bool) and i >= 0
                   for i in (source_id, capture_id)):
            self.last_wrists.pop(name, None)
            return []
        self.last_wrists[name] = (source_id, capture_id, timestamp)
        if previous is None:
            return []
        previous_source, previous_capture, previous_timestamp = previous
        channel = self.decoded_wrists[name]
        gap = source_id - previous_source
        slots = getattr(channel.spec, "slots", 0)
        if (gap <= 1 or capture_id - previous_capture != gap
                or not isinstance(slots, Integral) or gap > min(8, slots)):
            return []

        # The decoded ring holds the very same camera exposures. Recover only a
        # complete gap, with both identities and publication order verified.
        recovered = {}
        try:
            reader = self.open_camera_channel(channel, start="oldest")
            try:
                for _ in range(min(8, slots)):
                    packet = reader.read(mode="sequential", timeout=0)
                    if packet.status in self.publisher_lost or packet.frame is None:
                        break
                    for candidate in packet.frame.planes:
                        candidate_id = getattr(candidate, "source_frame_id", None)
                        candidate_time = (getattr(candidate, "publish_epoch_ns", None)
                                          or getattr(packet.frame.meta, "publish_epoch_ns", None))
                        candidate_format = self.formats.get(candidate.pixel_format)
                        if (
                            candidate.sensor_id == plane.sensor_id
                            and candidate_id is not None and previous_source < candidate_id < source_id
                            and getattr(candidate, "capture_id", None) == previous_capture + candidate_id - previous_source
                            and candidate_time is not None and previous_timestamp < candidate_time < timestamp
                            and (candidate.width, candidate.height) == (plane.width, plane.height)
                            and candidate_format is not None
                            and candidate_format[0] in (PIXEL_FORMAT_BGR, PIXEL_FORMAT_RGB)
                        ):
                            frame = self._frame_from_plane(candidate, candidate_time)
                            if frame.byte_count == plane.width * plane.height * 3:
                                recovered[candidate_id] = frame
                    if len(recovered) == gap - 1:
                        break
            finally:
                reader.close()
        except (OSError, ValueError, AttributeError, TypeError, BufferError, self.shm_error):
            # This is a best-effort auxiliary channel only. Keep the genuine
            # JPEG; the normal gap/age checks still decide if the episode is valid.
            return []

        if len(recovered) != gap - 1:
            return []
        ordered = [recovered[i] for i in range(previous_source + 1, source_id)]
        if any(a.timestamp_ns >= b.timestamp_ns for a, b in zip(ordered, ordered[1:])):
            return []
        if self.logger is not None:
            self.logger.info(
                f"Recovered original {name} frame(s) {previous_source + 1}..{source_id - 1} "
                f"from the SDK decoded stream (capture {previous_capture + 1}..{capture_id - 1})"
            )
        return ordered

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
                if name in self.decoded_wrists:
                    frames.extend((name, CAMERA_SPECS[name], frame, received_sec)
                                  for frame in self._recover_wrist_gap(name, plane, timestamp))
                frame = self._frame_from_plane(plane, timestamp)
                frames.append((name, CAMERA_SPECS[name], frame, received_sec))
        return frames

    def close(self):
        for _, reader, _ in self.readers:
            reader.close()
        self.readers.clear()

