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
        self.assertEqual(spec.meta_path, "/dev/shm/camera_metadata_struct_hand_left")
        self.assertEqual(spec.buffer_path, "/dev/shm/camera_image_buffer_hand_left")
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

    def test_hand_cameras_offer_jpeg_shm_fallback(self) -> None:
        primary, fallback = camera_shm_candidates("hand_left")
        self.assertIs(primary, CAMERA_SPECS["hand_left"])
        self.assertEqual(fallback.name, "hand_left")
        self.assertTrue(fallback.meta_path.endswith("hand_left_jpeg"))

        self.assertEqual(camera_shm_candidates("rgbd_head_color"), (CAMERA_SPECS["rgbd_head_color"],))


if __name__ == "__main__":
    unittest.main()
