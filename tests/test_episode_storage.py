"""Fault injection on real files; no robot, ROS publishers or model requests."""
import errno
import json
from pathlib import Path
import tempfile
import time
import threading
import unittest
from unittest.mock import patch

import numpy as np
from episode_storage import AsyncTextLog, EpisodeStore, StorageBudget, StorageLimitError, durable_json


class SyncLogTest(unittest.TestCase):
    def test_slow_disk_does_not_block_capture_and_flush_keeps_order(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / 'sync.jsonl'
            log = AsyncTextLog(path)
            release, entered = threading.Event(), threading.Event()
            original = log.stream.write
            def slow_write(text):
                entered.set()
                release.wait(2)
                return original(text)
            try:
                with patch.object(log.stream, 'write', side_effect=slow_write):
                    log.write('first\n')
                    self.assertTrue(entered.wait(1))
                    log.write('second\n')
                    self.assertFalse(release.is_set())
                    release.set()
                    log.flush()
                self.assertEqual(path.read_text(), 'first\nsecond\n')
            finally:
                release.set()
                log.close()

    def test_background_error_is_reported_before_publication(self):
        with tempfile.TemporaryDirectory() as root:
            log = AsyncTextLog(Path(root) / 'sync.jsonl')
            with patch.object(log.stream, 'write', side_effect=OSError('disk full')):
                log.write('first\n')
                with self.assertRaisesRegex(OSError, 'disk full'):log.flush()
                with self.assertRaisesRegex(OSError, 'disk full'):log.write('second\n')
                with self.assertRaisesRegex(OSError, 'disk full'):log.close()

    def test_queue_is_bounded_without_dropping_logs(self):
        with tempfile.TemporaryDirectory() as root:
            log = AsyncTextLog(Path(root) / 'sync.jsonl', max_pending=1)
            release = threading.Event()
            try:
                with patch.object(log.stream, 'write', side_effect=lambda _:release.wait(2)):
                    log.write('first\n')
                    with self.assertRaisesRegex(OSError, 'queue full'):log.write('second\n')
            finally:
                release.set()
                with self.assertRaisesRegex(OSError, 'queue full'):log.close()


class BudgetTest(unittest.TestCase):
    def test_disk_limits_duration_and_stale_monitor(self):
        with tempfile.TemporaryDirectory() as root:
            budget = StorageBudget(root, min_free_gb=.01, max_temp_gb=.02, max_seconds=1)
            try:
                budget.start()
                budget.admit({'observation.images.left': np.zeros((10, 10, 3), dtype=np.uint8)})
                self.assertGreater(budget.bytes, 300)
                with self.assertRaisesRegex(StorageLimitError, 'temp_limit'):
                    budget.check(additional=budget.maximum)
                budget.started -= 2
                with self.assertRaisesRegex(StorageLimitError, 'duration_limit'):
                    budget.check()
                budget.started = None
                budget.close()
                budget._snapshot = (time.monotonic(), 1, 0, None)
                with self.assertRaisesRegex(StorageLimitError, 'reserve_low'):
                    budget.check()
                budget._snapshot = (time.monotonic() - 3, 10**12, 0, None)
                with self.assertRaisesRegex(StorageLimitError, 'stale'):
                    budget.check()
                budget._snapshot = (time.monotonic(), 0, 0, 'unavailable')
                with self.assertRaisesRegex(StorageLimitError, 'failed'):
                    budget.check()
            finally:
                budget.close()

    def test_reject_nonfinite_or_disabled_limits(self):
        for value in (0, -1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                StorageBudget('.', min_free_gb=value)


class PublicationTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name) / 'dataset'
        (self.root / 'meta').mkdir(parents=True)
        (self.root / 'meta/info.json').write_text(json.dumps({'total_episodes': 1}))
        (self.root / 'data').mkdir()
        (self.root / 'data/old.parquet').write_bytes(b'saved-data')
        self.before = {str(p.relative_to(self.root)): p.read_bytes() for p in self.root.rglob('*') if p.is_file()}
        self.store = EpisodeStore(self.root)
        self.work = self.store.prepare()
        (self.work / 'meta/info.json').write_text(json.dumps({'total_episodes': 2}))
        for name in ('data/new.parquet', 'videos/left/new.mp4', 'annotations/subtasks/episode_000001.json'):
            p = self.work / name
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_bytes(b'new')

    def tearDown(self):
        self.store.close()
        self.temp.cleanup()

    def assert_saved_unchanged(self):
        for name, value in self.before.items():
            self.assertEqual((self.root / name).read_bytes(), value)

    def test_success_and_no_overwrite(self):
        self.store.commit(1)
        self.assertEqual((self.root / 'data/old.parquet').read_bytes(), b'saved-data')
        self.assertEqual(json.loads((self.root / 'meta/info.json').read_text())['total_episodes'], 2)
        self.assertFalse(self.work.exists())
        self.assertFalse(self.store.journal.exists())

    def test_collision_never_overwrites(self):
        (self.work / 'data/old.parquet').write_bytes(b'bad')
        with self.assertRaisesRegex(RuntimeError, 'overwrite'):
            self.store.commit(1)
        self.assert_saved_unchanged()

    def test_commit_marker_written_before_sync_failure_is_not_a_failed_save(self):
        def injected(path, journal):
            durable_json(path, journal)
            if journal['phase'] == 'committed':
                raise OSError('injected final sync failure')
        with patch('episode_storage.durable_json', injected), self.assertLogs(level='ERROR'):
            self.store.commit(1)
        self.assertEqual(json.loads((self.root / 'meta/info.json').read_text())['total_episodes'], 2)
        self.assertEqual((self.root / 'data/old.parquet').read_bytes(), b'saved-data')

    def test_committed_cleanup_failure_is_recovered_without_replay(self):
        with patch.object(self.store, '_recover_publication', side_effect=OSError('cleanup failed')):
            self.store.commit(1)
        self.assertTrue(self.store.journal.exists())
        self.store.recover()
        self.assertEqual(json.loads((self.root / 'meta/info.json').read_text())['total_episodes'], 2)
        self.assertFalse(self.store.journal.exists())
        self.assertFalse(self.work.exists())

    def test_failure_at_each_publication_move_restores_previous_dataset(self):
        import os
        real_link, real_replace = os.link, os.replace
        for failure in ('data', 'meta'):
            with self.subTest(failure=failure):
                def link(src, dst):
                    if failure == 'data' and '/videos/' in str(dst):
                        raise OSError(errno.ENOSPC, 'injected full disk')
                    return real_link(src, dst)
                def replace(src, dst):
                    if failure == 'meta' and Path(src) == self.work / 'meta':
                        raise OSError(errno.ENOSPC, 'injected metadata failure')
                    return real_replace(src, dst)
                with patch('episode_storage.os.link', link), patch('episode_storage.os.replace', replace):
                    with self.assertRaises(OSError):
                        self.store.commit(1)
                self.assert_saved_unchanged()
                self.assertFalse((self.root / 'data/new.parquet').exists())
                self.assertFalse(self.store.journal.exists())

    def test_interrupted_capture_preserved_but_not_counted(self):
        retained = self.store.recover()
        self.assertTrue(retained.is_dir())
        self.assert_saved_unchanged()
        self.assertFalse(self.store.work.exists())

    def test_restart_rolls_back_partial_publication(self):
        import os
        durable_json(self.store.journal, {'version': 1, 'phase': 'prepared', 'files': ['data/new.parquet']})
        os.link(self.work / 'data/new.parquet', self.root / 'data/new.parquet')
        os.replace(self.root / 'meta', self.work / 'previous-meta')
        os.replace(self.work / 'meta', self.root / 'meta')
        self.store.recover()
        self.assert_saved_unchanged()
        self.assertFalse((self.root / 'data/new.parquet').exists())

    def test_lock_and_journal_path_are_fail_closed(self):
        with self.assertRaisesRegex(RuntimeError, 'owned'):
            EpisodeStore(self.root)
        durable_json(self.store.journal, {'version': 1, 'phase': 'prepared', 'files': ['../unrelated']})
        with self.assertRaisesRegex(RuntimeError, 'unsafe'):
            self.store.recover()
        self.assert_saved_unchanged()


if __name__ == '__main__':
    unittest.main()
