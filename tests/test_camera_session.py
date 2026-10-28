import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from camera_session import CameraSession, KEEP_MODULES, four_stream_config

class CameraSessionTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.config = self.root / "vision.json"
        self.original = (json.dumps({"ENABLED_MODULES": [
            "mod_camera_head_left", *KEEP_MODULES, "mod_camera_head_rear", "mod_microphone_gadget"
        ], "video": {"fps": 30}, "other": {"untouched": True}}) + "\n").encode()
        self.config.write_bytes(self.original)
        self.settings = self.root / "settings.json"
        self.settings.write_text(json.dumps({"vision_config": str(self.config)}))
        self.state = self.root / "state"
        self.addCleanup(patch.stopall)
        patch("camera_session.recorder_pids", return_value=[]).start()
        patch("camera_session.stream_stamp", return_value=0).start()
        patch.object(CameraSession, "_service_active").start()
        self.restart = patch.object(CameraSession, "_restart").start()

    def session(self):
        return CameraSession(self.settings, self.state)

    def test_selection_preserves_audio_and_other_fields(self):
        selected = json.loads(four_stream_config(self.original))
        self.assertEqual(selected["ENABLED_MODULES"], [*KEEP_MODULES, "mod_microphone_gadget"])
        self.assertEqual(selected["video"], {"fps": 30})
        self.assertEqual(selected["other"], {"untouched": True})

    def test_exit_restores_exact_original(self):
        s = self.session()
        s.acquire()
        self.assertNotEqual(self.config.read_bytes(), self.original)
        s.release()
        self.assertEqual(self.config.read_bytes(), self.original)
        self.assertFalse(s.journal.exists())
        self.assertEqual(self.restart.call_count, 2)

    def test_failed_start_rolls_back(self):
        self.restart.side_effect = [RuntimeError("camera timeout"), None]
        with self.assertRaisesRegex(RuntimeError, "camera timeout"):
            self.session().acquire()
        self.assertEqual(self.config.read_bytes(), self.original)
        self.assertFalse((self.state / "pending.json").exists())

    def test_rollback_only_requires_previously_live_streams(self):
        import time
        from camera_session import STREAMS
        baseline = [name for name in STREAMS if name != "hand_right"]
        self.restart.side_effect = [RuntimeError("right camera missing"), None]
        with patch("camera_session.stream_stamp", side_effect=lambda name: time.time_ns() if name in baseline else 0):
            with self.assertRaisesRegex(RuntimeError, "right camera missing"):
                self.session().acquire()
        self.assertEqual(self.restart.call_args_list[0].args[0], STREAMS)
        self.assertEqual(self.restart.call_args_list[1].args[0], sorted(baseline))
        self.assertEqual(self.config.read_bytes(), self.original)

    def test_parallel_session_is_rejected(self):
        first = self.session()
        first.acquire()
        try:
            with self.assertRaisesRegex(RuntimeError, "其他采集"):
                self.session().acquire()
            self.assertNotEqual(self.config.read_bytes(), self.original)
        finally:
            first.release()

    def test_interrupted_session_recovers(self):
        first = self.session()
        first.acquire()
        first._unlock()
        self.session().recover()
        self.assertEqual(self.config.read_bytes(), self.original)

    def test_external_config_edit_is_not_overwritten(self):
        s = self.session()
        s.acquire()
        changed = json.loads(self.config.read_text())
        changed["video"]["fps"] = 15
        self.config.write_text(json.dumps(changed))
        with self.assertRaisesRegex(RuntimeError, "被其他人修改"):
            s.release()
        self.assertEqual(json.loads(self.config.read_text())["video"]["fps"], 15)
        self.assertTrue(s.journal.exists())

    def test_recording_blocks_restore(self):
        s = self.session()
        s.acquire()
        with patch("camera_session.recorder_pids", return_value=[123]):
            with self.assertRaisesRegex(RuntimeError, "录制进程仍存活"):
                s.release()
        self.assertTrue(s.journal.exists())

    def test_missing_camera_module_is_rejected_without_restarting(self):
        self.config.write_text(json.dumps({"ENABLED_MODULES": ["mod_camera_head_left"]}))
        with self.assertRaisesRegex(RuntimeError, "没有完整"):
            self.session().acquire()
        self.restart.assert_not_called()

    def test_child_error_restores_and_preserves_exit_code(self):
        import sys
        from camera_session import run_command
        with patch("camera_session.CameraSession", return_value=self.session()):
            result = run_command([sys.executable, "-c", "raise SystemExit(7)"])
        self.assertEqual(result, 7)
        self.assertEqual(self.config.read_bytes(), self.original)
        self.assertFalse((self.state / "pending.json").exists())

    def test_retained_batches_do_not_restart_and_explicit_recover_restores(self):
        import time
        self.settings.write_text(json.dumps({"vision_config": str(self.config), "retain_between_batches": True}))
        first = self.session()
        first.acquire()
        first.release()
        self.assertTrue(first.journal.exists())
        self.assertEqual(self.restart.call_count, 1)
        with patch("camera_session.stream_stamp", side_effect=lambda name: time.time_ns()):
            second = self.session()
            second.acquire()
            second.release()
        self.assertEqual(self.restart.call_count, 1)
        self.session().recover()
        self.assertEqual(self.restart.call_count, 2)
        self.assertEqual(self.config.read_bytes(), self.original)

    def test_retained_stale_stream_does_not_restart_or_erase_backup(self):
        self.settings.write_text(json.dumps({"vision_config": str(self.config), "retain_between_batches": True}))
        first = self.session()
        first.acquire()
        first.release()
        with self.assertRaisesRegex(RuntimeError, "持续出帧"):
            self.session().acquire()
        self.assertTrue(first.journal.exists())
        self.assertEqual(self.restart.call_count, 1)

    def test_unconfigured_host_is_unchanged(self):
        s = CameraSession(self.root / "missing", self.state)
        s.acquire()
        s.release()
        self.restart.assert_not_called()

if __name__ == "__main__":
    unittest.main()
