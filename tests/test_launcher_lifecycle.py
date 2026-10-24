"""Exercise the launcher's real cleanup body with harmless child processes."""

import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class LauncherLifecycleTest(unittest.TestCase):
    def test_cleanup_retries_quit_when_startup_lost_first_request(self):
        source = (ROOT / 'start_lerobot_official_collect.sh').read_text()
        cleanup = source[source.index('cleanup_session() {'):source.index('on_interrupt() {')]
        with tempfile.TemporaryDirectory() as directory:
            script = '''
set -eu
ROOT=$1
CLEANUP_DONE=0
PIDFILE="$ROOT/pids"
CONTROL_FIFO="$ROOT/control"
STATUS_FILE="$ROOT/status"
STATE_READY_FILE="$ROOT/ready"
EPISODE_EVENT_FILE="$ROOT/event"
LOG_PREFIX="$ROOT/log"
CONTROL_SCRIPT=unused
DATASET_ROOT="$ROOT/dataset"
python3() { :; }
send_control() { if [ -f "$ROOT/first" ]; then touch "$ROOT/quit"; else touch "$ROOT/first"; fi; }
'''+cleanup+'''
(while [ ! -f "$ROOT/quit" ]; do sleep .05; done; touch "$ROOT/finalized") &
RECORDER_PID=$!
cleanup_session
test -f "$ROOT/finalized"
'''
            result = subprocess.run(['bash', '-c', script, 'retry-test', directory],
                                    capture_output=True, text=True, timeout=8)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_finalization_waits_for_writer_before_removing_session_files(self):
        source = (ROOT / 'start_lerobot_official_collect.sh').read_text()
        cleanup = source[source.index('cleanup_session() {'):source.index('on_interrupt() {')]
        with tempfile.TemporaryDirectory() as directory:
            # Stub only external reporting/IPC; run the production cleanup and
            # a real child that would lose its output if signalled prematurely.
            script = '''
set -eu
ROOT=$1
CLEANUP_DONE=0
PIDFILE="$ROOT/pids"
CONTROL_FIFO="$ROOT/control"
STATUS_FILE="$ROOT/status"
STATE_READY_FILE="$ROOT/ready"
EPISODE_EVENT_FILE="$ROOT/event"
LOG_PREFIX="$ROOT/log"
CONTROL_SCRIPT=unused
DATASET_ROOT="$ROOT/dataset"
python3() { :; }
send_control() { printf '%s\n' "$1" >> "$ROOT/commands"; }
'''
            script += cleanup
            script += '''
(sleep .15; printf done > "$ROOT/saved") &
RECORDER_PID=$!
printf 'recorder %s log\n' "$RECORDER_PID" > "$PIDFILE"
cleanup_session
test -f "$ROOT/saved"
test ! -f "$PIDFILE"
wait
'''
            result = subprocess.run(['bash', '-c', script, 'lifecycle-test', directory],
                                    capture_output=True, text=True, timeout=5)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertEqual((Path(directory) / 'commands').read_text().strip(), 'quit')

    def test_existing_launcher_lock_blocks_before_ros_or_fifo_changes(self):
        import fcntl
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            base = root / 'test'
            base.mkdir()
            sentinel = base / '.official_recording_status.json'
            sentinel.write_text('keep')
            with (base / '.official_session.lock').open('a') as lock:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                env = dict(os.environ, OUTPUT_BASE_DIR=directory, COLLECTOR_MODE='keyboard',
                           VR_CONTROL='0', SUBTASKS_JSON='[]')
                result = subprocess.run(['bash', str(ROOT / 'start_lerobot_official_collect.sh'), 'test'],
                                        env=env, capture_output=True, text=True, timeout=5)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('another launcher owns', result.stderr)
            self.assertEqual(sentinel.read_text(), 'keep')


if __name__ == '__main__':
    unittest.main()
