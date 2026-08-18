#!/usr/bin/env python3
"""Unit tests for control/status helpers without requiring ROS or LeRobot."""

from __future__ import annotations

import json
import os
import tempfile
import unittest
from pathlib import Path

from collector_control import (
    SessionPaths,
    consume_episode_event,
    format_dataset_summary,
    format_status,
    format_sync_report,
    read_active_cameras,
    run_command,
    wait_for_ready_file,
)


class CollectorControlTest(unittest.TestCase):
    def test_save_status_includes_per_task_and_session_counts(self) -> None:
        output = format_status(
            {
                "event": "save",
                "success": True,
                "task": "pick up the bottle",
                "episode_index": 7,
                "frames": 42,
                "task_counts": [
                    {
                        "task": "pick up the bottle",
                        "total": 5,
                        "session_added": 2,
                        "session_discarded": 1,
                    }
                ],
                "session_saved_episodes": 2,
                "session_discarded_episodes": 1,
            }
        )
        self.assertIn("[SAVED] Episode 7 | 42 frames | Task: pick up the bottle", output)
        self.assertIn("total 5, added this session 2, discarded this session 1", output)
        self.assertIn("Session totals: saved 2, discarded 1", output)

    def test_dataset_summary_lists_trainable_features(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_dir:
            root = Path(temporary_dir)
            (root / "meta").mkdir()
            (root / "meta" / "info.json").write_text(
                json.dumps(
                    {
                        "total_frames": 120,
                        "total_episodes": 3,
                        "fps": 15,
                        "features": {
                            "observation.state": {"shape": [16]},
                            "action": {"shape": [16]},
                            "timestamp": {"shape": [1]},
                        },
                    }
                ),
                encoding="utf-8",
            )
            summary = format_dataset_summary(root)
            self.assertIn("Frames: 120", summary)
            self.assertIn("Episodes: 3", summary)
            self.assertIn("observation.state: [16]", summary)
            self.assertNotIn("timestamp", summary)

    def test_command_round_trip_uses_fifo_protocol(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_dir:
            base_dir = Path(temporary_dir)
            paths = SessionPaths(base_dir)
            paths.pidfile.write_text(f"recorder {os.getpid()} recorder.log\n", encoding="utf-8")
            os.mkfifo(paths.control_fifo)
            reader_fd = os.open(paths.control_fifo, os.O_RDWR | os.O_NONBLOCK)
            try:
                run_command(base_dir, "start", wait=False, timeout_sec=1.0)
                self.assertEqual(os.read(reader_fd, 1024).decode("utf-8"), "start\n")
            finally:
                os.close(reader_fd)

    def test_active_cameras_are_read_from_dataset_metadata(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_dir:
            root = Path(temporary_dir)
            (root / "meta").mkdir()
            (root / "meta" / "info.json").write_text(
                json.dumps(
                    {
                        "features": {
                            "observation.images.hand_left": {},
                            "observation.images.rgbd_head_color": {},
                            "observation.state": {},
                        }
                    }
                ),
                encoding="utf-8",
            )
            self.assertEqual(
                read_active_cameras(root, root / "missing.log"),
                ["hand_left", "rgbd_head_color"],
            )

    def test_sync_report_summarizes_deltas_and_waits(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_dir:
            sync_log = Path(temporary_dir) / "sync_log.jsonl"
            events = [
                {"event": "frame", "sync_deltas_ms": {"state": -4.0, "image:hand_left": 8.0}},
                {"event": "frame", "sync_deltas_ms": {"state": 6.0, "image:hand_left": -12.0}},
                {"event": "wait", "reason": "no_reference_frame_ready"},
                {"event": "episode_invalidated", "reason": "reference_frame_interval"},
            ]
            sync_log.write_text("\n".join(json.dumps(event) for event in events), encoding="utf-8")
            report = format_sync_report(sync_log)
            self.assertIn("accepted frames : 2", report)
            self.assertIn("state: p95=6.00 ms, max=6.00 ms", report)
            self.assertIn("waiting ticks   : 1", report)
            self.assertIn("no_reference_frame_ready: 1", report)
            self.assertIn("invalid episodes: 1", report)
            self.assertIn("reference_frame_interval: 1", report)

    def test_existing_ready_file_returns_immediately(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_dir:
            ready_path = Path(temporary_dir) / "state.ready"
            ready_path.write_text("{}", encoding="utf-8")
            wait_for_ready_file(ready_path, os.getpid(), timeout_sec=0.1)

    def test_invalid_episode_event_is_reported_and_consumed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_dir:
            event_path = Path(temporary_dir) / "episode-event.json"
            event_path.write_text(
                json.dumps(
                    {
                        "event": "episode_invalidated",
                        "reason": "reference_frame_interval",
                        "buffered_frames": 12,
                    }
                ),
                encoding="utf-8",
            )
            output = consume_episode_event(event_path)
            self.assertIn("[EPISODE INVALID] 12 buffered frames", output)
            self.assertIn("reference_frame_interval", output)
            self.assertFalse(event_path.exists())


if __name__ == "__main__":
    unittest.main()
