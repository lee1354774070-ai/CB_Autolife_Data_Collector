import fcntl
import io
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch
from dagger import power_session as power


class PowerLeaseTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.before = {'wifi': {'wlan0': 'on'}, 'desktop': {}, 'clocks': False}
        self.settings = Mock(root=self.root, journal=self.root / 'before.json')
        self.settings.capture.return_value = self.before

    def run_lease(self, eof=True, ready=None):
        read, write = os.pipe()
        if eof:
            os.close(write)
        else:
            self.addCleanup(os.close, write)
        with os.fdopen(read, 'rb') as stream:
            power.hold(stream, self.settings, timeout=.01, ready=ready)

    def test_eof_restores_and_removes_journal(self):
        self.run_lease()
        self.settings.restore.assert_called_once_with(self.before)
        self.assertFalse(self.settings.journal.exists())

    def test_missing_heartbeat_restores(self):
        self.run_lease(eof=False)
        self.settings.restore.assert_called_once_with(self.before)

    def test_partial_enable_failure_rolls_back(self):
        self.settings.enable.side_effect = RuntimeError('clock failure')
        with self.assertRaisesRegex(RuntimeError, 'clock failure'):
            self.run_lease()
        self.settings.restore.assert_called_once_with(self.before)

    def test_restore_failure_keeps_original_journal(self):
        self.settings.restore.side_effect = RuntimeError('restore failure')
        with self.assertRaisesRegex(RuntimeError, 'restore failure'):
            self.run_lease()
        self.assertEqual(json.loads(self.settings.journal.read_text()), self.before)

    def test_recovered_lease_preserves_original_baseline(self):
        self.settings.journal.write_text(json.dumps(self.before))
        self.run_lease()
        self.settings.capture.assert_not_called()
        self.settings.restore.assert_called_once_with(self.before)

    def test_second_lease_cannot_restore_first_owner(self):
        with (self.root / 'owner.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with self.assertRaises(BlockingIOError):
                self.run_lease()
        self.settings.enable.assert_not_called()
        self.settings.restore.assert_not_called()

    def test_restore_tries_wifi_even_when_clocks_fail(self):
        real = power.Settings(self.root)
        before = {'clocks': True, 'desktop': {'schema': {'key': 'old'}}, 'wifi': {'wifi0': 'on'}}
        with patch.object(power, 'command', side_effect=[RuntimeError('clock'), '', '']) as command:
            with self.assertRaisesRegex(RuntimeError, 'clock'):
                real.restore(before)
            self.assertEqual(command.call_count, 3)
            self.assertEqual(command.call_args.args[-1], 'on')

    def test_sleep_inhibitor_is_pipe_owned_and_no_desktop_settings_written(self):
        real = power.Settings(self.root)
        before = {'wifi': {}, 'desktop': {}, 'clocks': False}
        inhibitor = Mock(stdin=io.StringIO(), stdout=io.StringIO('ready\n'))
        with patch.object(power.subprocess, 'Popen', return_value=inhibitor) as spawn, \
             patch.object(power.select, 'select', return_value=([inhibitor.stdout], [], [])), \
             patch.object(power, 'command') as command:
            real.enable(before)
            real.restore(before)
        self.assertIn('systemd-inhibit', spawn.call_args.args[0])
        self.assertTrue(inhibitor.stdin.closed)
        command.assert_not_called()

    def test_disabled_and_dryrun_do_nothing(self):
        for env in ({}, {'DAGGER_SESSION_POWER': '1', 'DAGGER_PUBLISH': '0'}):
            with patch.object(power.subprocess, 'Popen') as spawn:
                session = power.PowerSession(env)
                session.start()
                session.close()
                spawn.assert_not_called()

    def test_s2_or_unknown_server_refused_before_any_change(self):
        for robot, url in [('300', 'http://192.168.8.224:8777'), ('340', 'http://elsewhere:8783')]:
            with patch.object(power.subprocess, 'Popen') as spawn:
                with self.assertRaises(ValueError):
                    power.PowerSession({'DAGGER_SESSION_POWER':'1', 'DAGGER_PUBLISH':'1',
                                        'ROBOT_ID':robot, 'DAGGER_SERVER_URL':url}).start()
                spawn.assert_not_called()

    def test_remote_start_failure_closes_local_lease(self):
        first = Mock(stdin=io.StringIO(), stdout=io.StringIO('{"ready":true}\n'))
        first.wait.return_value = 0
        session = power.PowerSession({'DAGGER_SESSION_POWER':'1', 'DAGGER_PUBLISH':'1',
                                     'ROBOT_ID':'340', 'DAGGER_SERVER_URL':'http://192.168.8.224:8783'})
        with patch.object(power.subprocess, 'Popen', side_effect=[first, OSError('SSH')]), \
             patch.object(power.select, 'select', return_value=([first.stdout], [], [])):
            with self.assertRaisesRegex(OSError, 'SSH'):
                session.start()
        self.assertTrue(first.stdin.closed)
        first.wait.assert_called_once()

    def test_forced_command_only_dispatches_exact_power_request(self):
        from dagger import s3_checkpoint_control as control
        with patch.dict(os.environ, SSH_ORIGINAL_COMMAND='dagger-power-session'), \
             patch.object(power, 'worker', return_value=0) as worker:
            with self.assertRaises(SystemExit) as result:
                control.main()
        self.assertEqual(result.exception.code, 0)
        worker.assert_called_once_with()


if __name__ == '__main__':
    unittest.main()
