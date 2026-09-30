"""Read-only launcher validation, before ROS, dataset or hardware side effects."""

from pathlib import Path
import hashlib
import signal
import subprocess
import tempfile
import unittest
from unittest.mock import Mock, patch

from dagger.dependencies import (COMMAND_PUBLISHERS, command_conflicts, conflicting_processes,
                                 interpreter_environment, validate)
from dagger.run import environment, stop_child


class DaggerLaunchTest(unittest.TestCase):
    def test_launch_owns_first_shutdown_signal(self):
        child = Mock(pid=12345)
        child.poll.return_value = None
        with patch('dagger.run.os.killpg') as kill:
            stop_child(child, launch_managed=True)
            child.send_signal.assert_called_once_with(signal.SIGINT)
            kill.assert_not_called()

    def test_stuck_launch_still_escalates_only_its_process_group(self):
        child = Mock(pid=12345)
        child.poll.return_value = None
        child.wait.side_effect = [subprocess.TimeoutExpired('test', 30), None]
        with patch('dagger.run.os.killpg') as kill:
            stop_child(child, launch_managed=True)
            kill.assert_called_once_with(child.pid, signal.SIGTERM)

    def test_recorder_shutdown_still_uses_its_process_group(self):
        child = Mock(pid=12345)
        child.poll.return_value = None
        with patch('dagger.run.os.killpg') as kill:
            stop_child(child)
            kill.assert_called_once_with(child.pid, signal.SIGINT)
            child.send_signal.assert_not_called()

    def env(self, **updates):
        return environment("towel_dagger", None, {"DAGGER_SERVER_URL": "http://thor:8777", **updates})

    def test_defaults_are_no_publish_new_controller_and_21d(self):
        env = self.env()
        self.assertEqual(env["DAGGER_PUBLISH"], "0")
        self.assertEqual(env["VR_CONTROL"], "0")
        self.assertEqual(env["START_HAND_PRODUCER"], "0")
        self.assertEqual(env["ACTION_MODE"], "joint")
        self.assertEqual(env["WITH_HEAD"], "1")
        self.assertEqual(env["WITH_UPPER_WAIST"], "1")
        self.assertEqual(env["WITH_WAIST"], "0")

    def test_unsupported_settings_are_rejected(self):
        for updates in ({"WITH_WAIST": "1"}, {"WITH_HEAD": "0"}, {"WITH_UPPER_WAIST": "0"},
                        {"ROBOT_ID": "283"}, {"ROS_DOMAIN_ID": "1"}, {"DAGGER_PUBLISH": "true"},
                        {"ACTION_MODE": "status_target"}, {"FALLBACK_ACTION_TO_STATE": "1"},
                        {"DAGGER_SERVER_URL": ""}):
            with self.subTest(updates=updates), self.assertRaises(ValueError):
                self.env(**updates)

    def test_process_checks_report_only_pids_not_command_secrets(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for pid, cmd in (("1", b"python\0-m\0openarmx_teleop_vr_306_v4.controller_node\0"),
                             ("2", b"ros2\0shell text mentioning hg_dagger_supervisor\0"),
                             ("3", b"/path/hg_dagger_supervisor\0secret-test-value\0")):
                (root / pid).mkdir()
                (root / pid / "cmdline").write_bytes(cmd)
            self.assertEqual(conflicting_processes(root), [1, 3])

    def test_missing_dependencies_fail_closed(self):
        with tempfile.TemporaryDirectory() as root:
            with self.assertRaisesRegex(RuntimeError, "changed or missing"):
                validate(Path(root))

    def test_idle_vendor_endpoints_are_not_active_commands(self):
        topic = "/topic_arm_whole_body_target_joints_position_0_300"
        conflicts = command_conflicts({topic: ["/node_arm_vr_control_service_0_300"]}, {})
        self.assertFalse(any(conflicts.values()))
        self.assertNotIn("/topic_arm_target_robot_eef_pose_0_300", COMMAND_PUBLISHERS)
        self.assertNotIn("/topic_arm_target_robot_height_z_0_300", COMMAND_PUBLISHERS)
        self.assertIn("/topic_arm_move_eef_pose_in_robot_frame_0_300", COMMAND_PUBLISHERS)

    def test_active_vendor_or_unknown_idle_controller_blocks_start(self):
        topic = "/topic_arm_whole_body_target_joints_position_0_300"
        self.assertEqual(command_conflicts({topic: ["/node_arm_vr_control_service_0_300"]},
                                          {topic: 1})["active_commands"], {topic: 1})
        for name in ("/whole_body_joint_bridge", "/other/node_arm_vr_control_service_0_300"):
            self.assertEqual(command_conflicts({topic: [name]}, {})["unknown_publishers"],
                             {topic: [name]})

    def test_only_reviewed_alternate_identity_is_accepted(self):
        original = hashlib.sha256(b"original").hexdigest()
        reviewed = hashlib.sha256(b"reviewed UI").hexdigest()
        with tempfile.TemporaryDirectory() as directory, \
                patch("dagger.dependencies.HASHES", {"app.js": original}), \
                patch("dagger.dependencies.REVIEWED_ALTERNATES", {"app.js": {reviewed}}):
            root = Path(directory)
            for content in (b"original", b"reviewed UI"):
                (root / "app.js").write_bytes(content)
                validate(root)
            (root / "app.js").write_bytes(b"unreviewed")
            with self.assertRaisesRegex(RuntimeError, "changed or missing"):
                validate(root)

    def test_conda_venv_uses_its_base_libraries_only(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            prefix = root / "conda"
            (prefix / "conda-meta").mkdir(parents=True)
            (prefix / "bin").mkdir()
            (prefix / "bin/python").touch()
            venv = root / "venv/bin"
            venv.mkdir(parents=True)
            (venv / "python").symlink_to(prefix / "bin/python")
            env = {"LD_LIBRARY_PATH": "/opt/ros/jazzy/lib"}
            self.assertEqual(interpreter_environment(str(venv / "python"), env),
                             {"LD_LIBRARY_PATH": str(prefix / "lib") + ":/opt/ros/jazzy/lib"})
            self.assertEqual(interpreter_environment("/usr/bin/python3", env), {})
