#!/usr/bin/env python3
"""Camera names, SHM paths, and ROS topics shared by collector processes.

The hand-camera producer, SHM bridge, and LeRobot recorder must agree on these
identifiers.  Keeping them here prevents a camera rename from updating one
process while silently leaving another process subscribed to the old path.
This module deliberately has no ROS, OpenCV, or LeRobot dependency.
"""

from __future__ import annotations

from dataclasses import dataclass


# Legacy Autolife camera producers use this little-endian metadata layout:
# timestamp_ns (int64), followed by width, height, channels, pixel format, and
# byte count (five int32 values). ``shm_camera`` also detects and parses the
# newer SHM2 ring/double-buffer protocol used by current robot SDK releases.
SHM_METADATA_FORMAT = "<qiiiii"

# Runtime defaults shared by the recorder and its documentation. The shell
# launcher repeats these values because it must pass them to the LeRobot process.
DEFAULT_IMAGE_POLL_FPS = 120.0
DEFAULT_SYNC_IMAGE_BUFFER_SIZE = 16
DEFAULT_SYNC_SIGNAL_BUFFER_SIZE = 64


@dataclass(frozen=True)
class CameraSpec:
    """Shared-memory and ROS identifiers for one camera stream."""

    name: str
    meta_path: str
    buffer_path: str
    topic: str
    frame_id: str


@dataclass(frozen=True)
class HandCameraSpec(CameraSpec):
    """Add V4L2 capture settings to a regular camera specification."""

    device: str
    width: int = 1280
    height: int = 720
    fps: float = 30.0


def _camera_fields(name: str, shm_name: str | None = None) -> dict[str, str]:
    """Build identifiers that follow the robot SDK's camera naming rule."""

    shm_name = shm_name or name
    return {
        "name": name,
        "meta_path": f"/dev/shm/camera_metadata_struct_{shm_name}",
        "buffer_path": f"/dev/shm/camera_image_buffer_{shm_name}",
        "topic": f"/camera/{name}/image_raw",
        "frame_id": name,
    }


HAND_CAMERA_SPECS = {
    "hand_left": HandCameraSpec(**_camera_fields("hand_left", "hand_left_jpeg"), device="/dev/video12"),
    "hand_right": HandCameraSpec(**_camera_fields("hand_right", "hand_right_jpeg"), device="/dev/video14"),
}

CAMERA_SPECS: dict[str, CameraSpec] = {
    "rgbd_head_color": CameraSpec(**_camera_fields("rgbd_head_color")),
    "rgbd_head_depth": CameraSpec(**_camera_fields("rgbd_head_depth")),
    **HAND_CAMERA_SPECS,
    "head_left": CameraSpec(**_camera_fields("head_left")),
    "head_right": CameraSpec(**_camera_fields("head_right")),
}


def camera_shm_candidates(name: str) -> tuple[CameraSpec, ...]:
    """Return the single source; keep the reader API shared with deployment.

    Hands use JPEG exclusively: never probe the unreliable SDK decoded-BGR
    outputs, even when JPEG is missing. Logical names and ROS topics stay fixed.
    Head color and depth retain their native uncompressed sources.
    """

    return (CAMERA_SPECS[name],)

# These are the streams selected by the main launcher.  Stereo head cameras
# remain available to the standalone bridge, but are not recorded by default.
DEFAULT_RGB_CAMERA_NAMES = ("rgbd_head_color", "hand_left", "hand_right")
DEFAULT_DEPTH_CAMERA_NAMES = ("rgbd_head_depth",)
DEFAULT_RGB_CAMERA_TOPICS = {name: CAMERA_SPECS[name].topic for name in DEFAULT_RGB_CAMERA_NAMES}
DEFAULT_DEPTH_CAMERA_TOPICS = {name: CAMERA_SPECS[name].topic for name in DEFAULT_DEPTH_CAMERA_NAMES}
