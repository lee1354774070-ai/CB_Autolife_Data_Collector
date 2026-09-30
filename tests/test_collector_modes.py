"""Mode selection is fail-closed and requires neither ROS nor robot access."""

import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from collector_modes import resolve_mode


class CollectorModeTest(unittest.TestCase):
    launcher = Path(__file__).resolve().parents[1] / "start_lerobot_official_collect.sh"

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
                           VR_CONTROL="", OUTPUT_BASE_DIR=str(output), DAGGER_SERVER_URL="")
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


if __name__ == "__main__":
    unittest.main()
