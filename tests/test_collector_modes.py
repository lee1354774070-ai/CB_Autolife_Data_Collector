"""Mode selection is fail-closed and requires neither ROS nor robot access."""

import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from collector_modes import resolve_mode


class CollectorModeTest(unittest.TestCase):
    launcher = Path(__file__).resolve().parents[1] / "start_lerobot_official_collect.sh"

    def test_all_regular_modes_enter_camera_session_before_recording(self):
        import json
        import shutil
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            launcher = root / self.launcher.name
            shutil.copy2(self.launcher, launcher)
            shutil.copy2(self.launcher.parent / "collector_modes.py", root / "collector_modes.py")
            shutil.copy2(self.launcher.parent / "subtask_annotations.py", root / "subtask_annotations.py")
            settings = root / ".config/autolife_collector/camera_session.json"
            settings.parent.mkdir(parents=True)
            settings.write_text("{}")
            (root / "camera_session.py").write_text(
                "import json, os, sys\n"
                "print('CAMERA_HOOK ' + json.dumps(sys.argv[1:]))\n"
            )
            for mode, plan in (("keyboard", "[]"), ("vr", "[]"),
                               ("subtask", '["pick","place"]')):
                with self.subTest(mode=mode):
                    env = dict(os.environ, HOME=str(root), COLLECTOR_MODE=mode,
                               VR_CONTROL="", SUBTASKS_JSON=plan,
                               COLLECTOR_CAMERA_SESSION_ACTIVE="",
                               ACTION_MODE="status_target", WITH_HEAD="0",
                               WITH_WAIST="0", WITH_UPPER_WAIST="0")
                    result = subprocess.run(["bash", str(launcher), "camera_test"],
                                            env=env, text=True, capture_output=True, timeout=5)
                    self.assertEqual(result.returncode, 0, result.stderr)
                    line = next(x for x in result.stdout.splitlines()
                                if x.startswith("CAMERA_HOOK "))
                    args = json.loads(line.removeprefix("CAMERA_HOOK "))
                    self.assertEqual(args, ["run", "--", "bash", str(launcher), "camera_test"])
                    self.assertNotIn("recorder PID", result.stdout)

    def test_legacy_commands_keep_their_existing_behavior(self):
        for mode, vr, plan, expected in [
            ("", "", (), ("keyboard", False)),
            ("", "0", (), ("keyboard", False)),
            ("", "1", (), ("vr", True)),
            ("", "", ("pick",), ("subtask", False)),
            ("", "1", ("pick",), ("subtask", True)),
        ]:
            self.assertEqual(resolve_mode(mode, vr, plan), expected)

    def test_explicit_modes_select_controls(self):
        self.assertEqual(resolve_mode("keyboard", "", ()), ("keyboard", False))
        self.assertEqual(resolve_mode("vr", "", ()), ("vr", True))
        self.assertEqual(resolve_mode("subtask", "", ("pick",)), ("subtask", True))
        self.assertEqual(resolve_mode("dagger", "", ()), ("dagger", True))

    def test_contradictory_or_invalid_settings_fail(self):
        for mode, vr, plan in [
            ("unknown", "", ()), ("", "true", ()),
            ("keyboard", "1", ()), ("vr", "0", ()), ("subtask", "0", ("pick",)),
            ("vr", "", ("pick",)), ("keyboard", "", ("pick",)),
            ("subtask", "", ()), ("dagger", "", ("pick",)),
        ]:
            with self.subTest(mode=mode, vr=vr, plan=plan), self.assertRaises(ValueError):
                resolve_mode(mode, vr, plan)

    def test_dagger_and_bad_settings_exit_before_creating_data_or_starting_ros(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "must_not_exist"
            for mode, plan in (("dagger", "[]"), ("subtask", "[]"), ("keyboard", "[1]")):
                env = dict(os.environ, COLLECTOR_MODE=mode, SUBTASKS_JSON=plan,
                           VR_CONTROL="", OUTPUT_BASE_DIR=str(output), DAGGER_SERVER_URL="",
                           ROBOT_ID="300", ROS_DOMAIN_ID="0")
                result = subprocess.run(["bash", str(self.launcher), "test"], env=env,
                                        text=True, capture_output=True, timeout=5)
                self.assertNotEqual(result.returncode, 0, result.stdout)
                self.assertFalse(output.exists())
                if mode == "dagger":
                    self.assertIn("Set DAGGER_SERVER_URL", result.stderr)

    def test_help_states_dagger_requirements_and_no_publish_default(self):
        result = subprocess.run(["bash", str(self.launcher), "COLLECTOR_MODE", "--help"],
                                text=True, capture_output=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("DAGGER_PUBLISH=0", result.stdout)
        self.assertIn("inspected V4/HG", result.stdout)
        self.assertIn("SUBTASKS_JSON", result.stdout)

    def test_action_mode_help_distinguishes_regular_collection_and_dagger(self):
        for args in (("--help",), ("ACTION_MODE", "--help")):
            with self.subTest(args=args):
                result = subprocess.run(["bash", str(self.launcher), *args],
                                        text=True, capture_output=True, timeout=5)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn("keyboard/vr/subtask", result.stdout)
                self.assertIn("status_target", result.stdout)
                self.assertIn("dagger", result.stdout)
                self.assertIn("joint", result.stdout)
                self.assertNotIn("status_target|joint|eef", result.stdout)
        self.assertIn("WITH_EEF_STATE/WITH_EEF_ACTION", result.stdout)

    def test_removed_or_unknown_action_mode_exits_before_startup(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "must_not_exist"
            for mode in ("keyboard", "vr", "subtask", "dagger"):
                for action, eef in (("eef", "0"), ("eef", "1"), ("unknown", "0")):
                    with self.subTest(mode=mode, action=action, eef=eef):
                        env = dict(os.environ, COLLECTOR_MODE=mode, ACTION_MODE=action,
                                   WITH_EEF_STATE=eef, WITH_EEF_ACTION=eef,
                                   OUTPUT_BASE_DIR=str(output))
                        result = subprocess.run(["bash", str(self.launcher), "test"], env=env,
                                                text=True, capture_output=True, timeout=5)
                        self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
                        self.assertIn("ACTION_MODE", result.stderr)
                        if action == "eef":
                            self.assertIn("was removed", result.stderr)
                            self.assertIn("WITH_EEF_STATE/WITH_EEF_ACTION", result.stderr)
                        self.assertNotIn("AutoLife Collector |", result.stdout)
                        self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
