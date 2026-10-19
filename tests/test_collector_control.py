#!/usr/bin/env python3
"""Unit tests for control/status helpers without requiring ROS or LeRobot."""

from __future__ import annotations

import json
import fcntl
import os
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import patch

from collector_control import (
    SessionPaths,
    consume_episode_event,
    format_dataset_summary,
    format_status,
    format_sync_report,
    run_command,
    read_recorder_pid,
    process_is_running,
    send_fifo_command,
    wait_for_status,
    wait_for_active_cameras,
    wait_for_ready_file,
)


class CollectorControlTest(unittest.TestCase):
    def test_invalid_pid_never_addresses_a_process_group(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'pids'
            for pid in ('0', '-1', 'oops'):
                path.write_text(f'recorder {pid} recorder.log\n')
                with self.assertRaisesRegex(RuntimeError, 'Invalid recorder PID'):
                    read_recorder_pid(path)
            with patch('collector_control.os.kill') as kill:
                self.assertFalse(process_is_running(0))
                self.assertFalse(process_is_running(-1))
                kill.assert_not_called()

    def test_control_path_must_be_a_real_fifo(self):
        with tempfile.TemporaryDirectory() as directory:
            regular = Path(directory) / 'file'
            regular.write_text('preserve me')
            link = Path(directory) / 'link'
            link.symlink_to(regular)
            for path in (regular, link):
                with self.assertRaises(RuntimeError):
                    send_fifo_command(path, 'save')
            self.assertEqual(regular.read_text(), 'preserve me')

    def test_fifo_rejects_injection_and_oversize_without_writing(self):
        with tempfile.TemporaryDirectory() as directory:
            fifo = Path(directory) / 'fifo'
            os.mkfifo(fifo)
            reader = os.open(fifo, os.O_RDWR | os.O_NONBLOCK)
            try:
                for command, request in [('start\nquit', None), ('start', 'id\nquit'),
                                         ('start', 'x' * 100000)]:
                    with self.assertRaises(RuntimeError):
                        send_fifo_command(fifo, command, request)
                with self.assertRaises(BlockingIOError):
                    os.read(reader, 1024)
            finally:
                os.close(reader)

    def test_bad_timeout_never_sends_a_command(self):
        with patch('collector_control.send_fifo_command') as send:
            for value in (0, -1, float('nan'), float('inf')):
                with self.assertRaisesRegex(RuntimeError, 'timeout'):
                    run_command(Path('/unused'), 'start', wait=True, timeout_sec=value)
            send.assert_not_called()

    def test_non_object_receipt_is_ignored_not_accepted(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'status.json'
            path.write_text('[]')
            with self.assertRaises(TimeoutError):
                wait_for_status(path, 'expected', os.getpid(), .01)

    def test_old_metadata_does_not_mark_current_recorder_ready(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'meta').mkdir()
            (root / 'meta' / 'info.json').write_text(json.dumps({'features': {'observation.images.hand_left': {}}}))
            with patch('collector_control.process_is_running', return_value=False):
                with self.assertRaisesRegex(RuntimeError, 'before camera initialization'):
                    wait_for_active_cameras(root, root / 'new.log', 1, 12345)

    def test_current_log_marks_camera_startup_ready(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            log = root / '1.recorder.log'
            log.write_text('active cameras: hand_left, hand_right\n')
            self.assertEqual(wait_for_active_cameras(root, log, 1, os.getpid()), ['hand_left', 'hand_right'])

    def test_command_lock_rejects_second_client_without_erasing_receipt(self):
        with tempfile.TemporaryDirectory() as temporary_dir:
            root = Path(temporary_dir)
            paths = SessionPaths(root)
            paths.pidfile.write_text(f"recorder {os.getpid()} recorder.log\n")
            paths.status_file.write_text('{"request_id": "first"}')
            with (root / '.official_control.lock').open('a') as lock:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                with self.assertRaisesRegex(RuntimeError, 'Another collector command'):
                    run_command(root, 'save', wait=True, timeout_sec=.1)
            self.assertEqual(json.loads(paths.status_file.read_text())['request_id'], 'first')

    def test_wait_can_be_cancelled_for_prompt_helper_shutdown(self):
        cancel = threading.Event()
        cancel.set()
        with self.assertRaisesRegex(RuntimeError, 'cancelled'):
            wait_for_status(Path('/unused'), 'id', os.getpid(), 300, cancel)

    def test_command_round_trip_matches_request_id(self):
        with tempfile.TemporaryDirectory() as temporary_dir:
            root = Path(temporary_dir)
            paths = SessionPaths(root)
            paths.pidfile.write_text(f"recorder {os.getpid()} recorder.log\n")
            def respond(fifo, command, request_id):
                paths.status_file.write_text(json.dumps(dict(request_id=request_id, event=command, success=True)))
            with patch('collector_control.send_fifo_command', side_effect=respond):
                status = run_command(root, 'start', wait=True, timeout_sec=1)
            self.assertEqual(status['event'], 'start')
            self.assertTrue(status['success'])

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

    def test_old_metadata_without_current_startup_marker_times_out(self) -> None:
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
                wait_for_active_cameras(root, root / "missing.log", 0.001, os.getpid()),
                [],
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
