"""Installer safety tests. Never invoke Conda, pip, CUDA, ROS or networking."""
import contextlib
import fcntl
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import setup_collector_env as setup


class EnvironmentSetupTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.base = Path(self.directory.name)
        self.ros = self.base / 'setup.bash'
        self.ros.touch()
        self.prefix = self.base / 'envs' / 'test_collector'
        self.marker = self.base / 'envs' / '.collector-setup-test_collector.json'
        self.args = ['--name', 'test_collector', '--backend', 'cpu', '--ros-setup', str(self.ros)]
        self.stack = contextlib.ExitStack()
        self.addCleanup(self.stack.close)
        self.stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
        self.stack.enter_context(patch.object(setup.platform, 'system', return_value='Linux'))
        self.stack.enter_context(patch.object(setup.platform, 'machine', return_value='x86_64'))
        self.stack.enter_context(patch.object(setup, 'conda_executable', return_value=self.base / 'bin/conda'))
        self.output = self.stack.enter_context(patch.object(setup.subprocess, 'check_output', return_value=str(self.base)))
        self.calls = []
        self.runner = self.stack.enter_context(patch.object(setup, 'run', side_effect=self.command))
        self.checker = self.stack.enter_context(patch.object(setup, 'check_environment'))

    def command(self, command, **kwargs):
        self.calls.append([str(value) for value in command])
        if 'create' in command:
            (self.prefix / 'bin').mkdir(parents=True)
            (self.prefix / 'bin/python').touch()

    def test_dry_run_does_not_create_or_install(self):
        setup.main(self.args + ['--dry-run'])
        self.assertFalse(self.prefix.parent.exists())
        self.runner.assert_not_called()
        self.checker.assert_not_called()

    def test_low_disk_space_stops_before_creating_environment(self):
        with patch.object(setup.shutil, 'disk_usage', return_value=Mock(free=1024)):
            with self.assertRaisesRegex(RuntimeError, 'GiB free'):
                setup.main(self.args)
        self.assertFalse(self.prefix.exists())
        self.assertFalse(self.marker.exists())
        self.runner.assert_not_called()

    def test_refuses_existing_unmanaged_environment(self):
        self.prefix.mkdir(parents=True)
        with self.assertRaisesRegex(RuntimeError, 'Target exists'):
            setup.main(self.args)
        self.runner.assert_not_called()
        self.assertFalse(self.marker.exists())

    def test_names_cannot_replace_existing_robot_environments(self):
        for name in ('base', 'lerobot', 'robot_env', 'cuda_env', '../other', '/tmp/test', 'name;exit'):
            with self.subTest(name=name), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                setup.parse_args(['--name', name])
        self.assertEqual(setup.parse_args(['--name', 'lerobot', '--check']).name, 'lerobot')

    def test_success_checks_before_publishing_ready(self):
        setup.main(self.args)
        self.checker.assert_called_once_with(self.prefix, 'cpu', self.ros)
        state = json.loads(self.marker.read_text())
        self.assertEqual(state['status'], 'ready')
        self.assertEqual(state['spec']['prefix'], str(self.prefix))
        self.assertIn(f'LEROBOT_PY={self.prefix}/bin/python', self.calls[-1])
        self.assertIn('torch==2.7.1+cpu', self.calls[1])
        with self.assertRaisesRegex(RuntimeError, 'incomplete install'):
            setup.main(self.args + ['--resume'])

    def test_failure_is_resumable_but_does_not_reclone(self):
        def fail(command, **kwargs):
            self.command(command)
            if 'install' in command:
                raise subprocess.CalledProcessError(1, command)
        self.runner.side_effect = fail
        with self.assertRaises(subprocess.CalledProcessError):
            setup.main(self.args)
        self.assertEqual(json.loads(self.marker.read_text())['status'], 'failed')
        self.checker.assert_not_called()
        self.calls.clear()
        self.runner.side_effect = self.command
        setup.main(self.args + ['--resume'])
        self.assertTrue(all('create' not in command for command in self.calls))
        self.assertEqual(json.loads(self.marker.read_text())['status'], 'ready')

    def test_resume_rejects_unowned_or_changed_configuration(self):
        with self.assertRaisesRegex(RuntimeError, 'incomplete install'):
            setup.main(self.args + ['--resume'])
        self.checker.side_effect = RuntimeError('probe failed')
        with self.assertRaisesRegex(RuntimeError, 'probe failed'):
            setup.main(self.args)
        self.calls.clear()
        with patch.object(setup, 'choose_backend', return_value='nvjpeg'):
            with self.assertRaisesRegex(RuntimeError, 'identical options'):
                setup.main(self.args + ['--resume'])
        self.assertEqual(self.calls, [])

    def test_check_does_not_create_marker_or_set_variables(self):
        (self.prefix / 'bin').mkdir(parents=True)
        (self.prefix / 'bin/python').touch()
        setup.main(self.args + ['--check'])
        self.runner.assert_not_called()
        self.checker.assert_called_once()
        self.assertFalse(self.marker.exists())

    def test_missing_ros_and_symlink_target_fail_before_commands(self):
        self.ros.unlink()
        with self.assertRaisesRegex(RuntimeError, 'ROS setup not found'):
            setup.main(self.args)
        self.ros.touch()
        self.prefix.parent.mkdir()
        self.prefix.symlink_to(self.base)
        with self.assertRaisesRegex(RuntimeError, 'symlink'):
            setup.main(self.args)
        self.runner.assert_not_called()

    def test_clone_source_is_read_only_and_isolated(self):
        source = self.base / 'envs/lerobot'
        (source / 'bin').mkdir(parents=True)
        (source / 'bin/python').touch()
        self.output.side_effect = [str(self.base), '3.12']
        setup.main(self.args + ['--clone', 'lerobot'])
        self.assertIn('--offline', self.calls[0])
        self.assertEqual(self.calls[0][-2:], [str(source), '--offline'])
        self.assertTrue((source / 'bin/python').exists())
        self.assertTrue(all(str(source / 'bin/python') not in command for command in self.calls))

    def test_wrong_python_clone_is_rejected_before_writes(self):
        source = self.base / 'envs/old_python'
        (source / 'bin').mkdir(parents=True)
        (source / 'bin/python').touch()
        self.output.side_effect = [str(self.base), '3.10']
        with self.assertRaisesRegex(RuntimeError, 'Python 3.12'):
            setup.main(self.args + ['--clone', 'old_python'])
        self.runner.assert_not_called()
        self.assertFalse(self.prefix.exists())
        self.assertFalse(self.marker.exists())

    def test_concurrent_install_is_rejected(self):
        self.marker.parent.mkdir()
        with self.marker.with_suffix('.lock').open('w') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with self.assertRaisesRegex(RuntimeError, 'Another installer'):
                setup.main(self.args)
        self.runner.assert_not_called()

    def test_gpu_detection_requires_successful_device_query(self):
        with patch.object(setup.subprocess, 'run', return_value=Mock(returncode=0, stdout='RTX GPU\n')):
            self.assertEqual(setup.choose_backend('auto'), 'nvjpeg')
        with patch.object(setup.subprocess, 'run', side_effect=FileNotFoundError):
            self.assertEqual(setup.choose_backend('auto'), 'cpu')
            with self.assertRaisesRegex(RuntimeError, 'not available'):
                setup.choose_backend('nvjpeg')
        with patch.object(setup.subprocess, 'run') as run:
            self.assertEqual(setup.choose_backend('cpu'), 'cpu')
            run.assert_not_called()

    def test_gpu_package_plan_pins_both_install_steps(self):
        commands = setup.install_commands(Path('/conda'), Path('/env'), 'nvjpeg', None)
        for command in commands[1:]:
            self.assertIn('torch==2.7.1+cu126', command)
            self.assertIn('torchvision==0.22.1+cu126', command)
        self.assertIn('https://download.pytorch.org/whl/cu126', commands[1])
        self.assertIn('python=3.12', commands[0])


if __name__ == '__main__':
    unittest.main()
