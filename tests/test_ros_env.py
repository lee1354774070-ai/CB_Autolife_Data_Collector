"""Environment selection without loading ROS or opening hardware endpoints."""
import os
from pathlib import Path
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]


class RosEnvironmentTest(unittest.TestCase):
    def run_shell(self, host, options=None, nounset=False):
        env = {k: v for k, v in os.environ.items()
               if k not in ("ROBOT_ID", "HG_DAGGER_ROBOT_ID")}
        env.update(options or {})
        script = r'''set -e
hostname() { printf '%s\n' "$TEST_HOST"; }
source() { [ "$1" = /opt/ros/jazzy/setup.bash ] || [ "$1" = /home/ubuntu/ros2_ws/install/setup.bash ]; }
[ "$TEST_NOUNSET" = 0 ] || set -u
builtin source "$TEST_SCRIPT"
printf 'RESULT:%s:%s:%s\n' "$ROBOT_ID" "$ROS_DOMAIN_ID" "$-"
'''
        env.update(TEST_HOST=host, TEST_NOUNSET=str(int(nounset)),
                   TEST_SCRIPT=str(ROOT / "scripts/source_ros_env.sh"))
        result = subprocess.run(["bash", "-c", script], env=env, text=True,
                                capture_output=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        return result.stdout.split("RESULT:", 1)[1].strip().split(":")

    def test_host_id_and_shell_flags(self):
        for robot in ("300", "345", "346", "347"):
            for nounset in (False, True):
                with self.subTest(robot=robot, nounset=nounset):
                    actual, domain, flags = self.run_shell("autolife-robot-" + robot, nounset=nounset)
                    self.assertEqual((actual, domain), (robot, "0"))
                    self.assertEqual("u" in flags, nounset)

    def test_explicit_ids_keep_precedence(self):
        self.assertEqual(self.run_shell("autolife-robot-346", {"ROBOT_ID": "347"})[0], "347")
        self.assertEqual(self.run_shell("autolife-robot-346", {
            "ROBOT_ID": "347", "HG_DAGGER_ROBOT_ID": "345"})[0], "345")
