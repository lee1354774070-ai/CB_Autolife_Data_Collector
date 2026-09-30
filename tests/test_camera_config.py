#!/usr/bin/env python3

from __future__ import annotations

import unittest

from camera_config import (
    CAMERA_SPECS,
    DEFAULT_DEPTH_CAMERA_TOPICS,
    DEFAULT_IMAGE_POLL_FPS,
    DEFAULT_RGB_CAMERA_TOPICS,
    DEFAULT_SYNC_IMAGE_BUFFER_SIZE,
    DEFAULT_SYNC_SIGNAL_BUFFER_SIZE,
    HAND_CAMERA_SPECS,
    camera_shm_candidates,
)


class CameraConfigTest(unittest.TestCase):
    def test_paths_and_topics_follow_one_name(self) -> None:
        spec = CAMERA_SPECS["hand_left"]
        self.assertEqual(spec.meta_path, "/dev/shm/camera_metadata_struct_hand_left_jpeg")
        self.assertEqual(spec.buffer_path, "/dev/shm/camera_image_buffer_hand_left_jpeg")
        self.assertEqual(spec.topic, "/camera/hand_left/image_raw")
        self.assertEqual(spec.frame_id, "hand_left")

    def test_default_recording_sets_are_explicit(self) -> None:
        self.assertEqual(
            tuple(DEFAULT_RGB_CAMERA_TOPICS),
            ("rgbd_head_color", "hand_left", "hand_right"),
        )
        self.assertEqual(tuple(DEFAULT_DEPTH_CAMERA_TOPICS), ("rgbd_head_depth",))
        self.assertEqual(DEFAULT_IMAGE_POLL_FPS, 120.0)
        self.assertEqual(DEFAULT_SYNC_IMAGE_BUFFER_SIZE, 16)
        self.assertEqual(DEFAULT_SYNC_SIGNAL_BUFFER_SIZE, 64)

    def test_hand_capture_settings_share_bridge_identifiers(self) -> None:
        for name, hand_spec in HAND_CAMERA_SPECS.items():
            self.assertIs(CAMERA_SPECS[name], hand_spec)
            self.assertGreater(hand_spec.fps, 0)
            self.assertTrue(hand_spec.device.startswith("/dev/video"))

    def test_hand_cameras_use_only_jpeg_and_keep_logical_names(self) -> None:
        for name in HAND_CAMERA_SPECS:
            (spec,) = camera_shm_candidates(name)
            self.assertIs(spec, CAMERA_SPECS[name])
            self.assertEqual(spec.name, name)
            self.assertTrue(spec.meta_path.endswith(f"{name}_jpeg"))
            self.assertTrue(spec.buffer_path.endswith(f"{name}_jpeg"))
        for name in ("rgbd_head_color", "rgbd_head_depth", "head_left", "head_right"):
            (spec,) = camera_shm_candidates(name)
            self.assertTrue(spec.meta_path.endswith(name))
            self.assertTrue(spec.buffer_path.endswith(name))


if __name__ == "__main__":
    unittest.main()
