"""Bounded capture and publish-on-success storage, independent of ROS.

LeRobot writes only into ``.collector-work``. Historical data/video files are
never opened for append. A small publication journal makes the multi-file move
recoverable; the old metadata tree survives until the new one is committed.
"""

import fcntl
import json
import logging
import math
import os
from pathlib import Path
import shutil
import threading
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor

GIB = 1024 ** 3


class StorageLimitError(RuntimeError):
    pass


class AsyncTextLog:
    """Ordered, bounded log writes; flush/close are command-boundary barriers."""
    def __init__(self, path, max_pending=256):
        self.stream = open(path, 'a', encoding='utf-8')
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='collector-sync-log')
        self.pending = deque()
        self.max_pending = max_pending
        self.error = None

    def write(self, text):
        while self.pending and self.pending[0].done():
            try:
                self.pending.popleft().result()
            except OSError as exc:
                self.error = self.error or exc
        if self.error:
            raise self.error
        if len(self.pending) >= self.max_pending:
            self.error = OSError('sync log queue full; storage cannot keep up')
            raise self.error
        self.pending.append(self.pool.submit(self.stream.write, text))

    def flush(self):
        while self.pending:
            try:
                self.pending.popleft().result()
            except OSError as exc:
                self.error = self.error or exc
        if self.error:
            raise self.error
        self.stream.flush()

    def close(self):
        try:
            self.flush()
        finally:
            self.pool.shutdown(wait=True)
            self.stream.close()


def sync_dir(path):
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def durable_json(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    with temporary.open('w', encoding='utf-8') as stream:
        json.dump(value, stream)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)
    sync_dir(path.parent)


class StorageBudget:
    """Constant-time frame accounting; filesystem queries stay off ROS ticks.

    Bytes are conservative raw-image estimates, not a filesystem quota. The
    free-space snapshot is discounted by every byte admitted since the query.
    Other processes can still exhaust the disk; write errors must be handled.
    """
    def __init__(self, root, *, min_free_gb=2, max_temp_gb=8, max_seconds=600):
        if any(not math.isfinite(v) or v <= 0 for v in (min_free_gb, max_temp_gb, max_seconds)):
            raise ValueError('storage limits must be finite and positive')
        self.root = Path(root)
        self.minimum = int(min_free_gb * GIB)
        self.maximum = int(max_temp_gb * GIB)
        self.max_seconds = max_seconds
        self.bytes = 0
        self.started = None
        self._snapshot = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._monitor, name='collector-disk', daemon=True)
        self.refresh()
        self._thread.start()

    def refresh(self):
        # Read the byte counter BEFORE statvfs, so concurrent admissions are
        # conservatively counted even if the filesystem already sees them.
        admitted = self.bytes
        try:
            free = shutil.disk_usage(self.root).free
            self._snapshot = (time.monotonic(), free, admitted, None)
        except OSError as exc:
            self._snapshot = (time.monotonic(), 0, admitted, str(exc))

    def _monitor(self):
        while not self._stop.wait(.5):
            self.refresh()

    def check(self, *, additional=0, saving=False):
        now = time.monotonic()
        if not saving and self.started is not None and now - self.started >= self.max_seconds:
            raise StorageLimitError('episode_duration_limit')
        if self.bytes + additional > self.maximum:
            raise StorageLimitError('episode_temp_limit')
        snapshot = self._snapshot
        if snapshot is None or now - snapshot[0] > 2:
            raise StorageLimitError('disk_check_stale')
        checked, free, admitted, error = snapshot
        if error:
            raise StorageLimitError('disk_check_failed: ' + error)
        # Retain an extra raw episode's worth for video encoding/publication.
        reserve = self.bytes + additional
        available = free - max(0, self.bytes - admitted) - additional
        if available < self.minimum + reserve:
            raise StorageLimitError('disk_reserve_low')

    def start(self):
        self.bytes = 0
        self.started = None
        self.refresh()  # Only at command boundaries, not in the recording tick.
        self.check()
        self.started = time.monotonic()

    def admit(self, frame):
        # Include per-image headers plus numeric columns, independent of codec.
        estimate = sum(int(value.nbytes) + 65536 for key, value in frame.items()
                       if key.startswith('observation.images.')) + 4096
        self.check(additional=estimate)
        self.bytes += estimate

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1)


class EpisodeStore:
    """Single-writer, same-filesystem episode workspace and recovery journal."""
    def __init__(self, root):
        self.root = Path(root).resolve()
        self.root.parent.mkdir(parents=True, exist_ok=True)
        self._lock = (self.root.parent / ('.' + self.root.name + '.collector.lock')).open('a+')
        try:
            fcntl.flock(self._lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self._lock.close()
            raise RuntimeError('dataset already owned by another collector') from None
        except BaseException:
            self._lock.close()
            raise
        self.work = self.root / '.collector-work'
        self.journal = self.root / '.collector-commit.json'

    def recover(self):
        if self.journal.exists():
            self._recover_publication()
        if self.work.exists():
            # An interrupted capture may be useful for manual recovery. Never
            # silently delete it or count it as a successfully saved episode.
            recovered = self.root / f'.collector-interrupted-{time.time_ns()}'
            self.work.rename(recovered)
            return recovered
        return None

    def prepare(self):
        if self.journal.exists() or self.work.exists():
            raise RuntimeError('previous episode workspace has not been resolved')
        info = json.loads((self.root / 'meta/info.json').read_text())
        if info['total_episodes'] == 0:
            # LeRobot cannot resume an empty shell: tasks/episodes Parquet do
            # not exist yet. The caller creates its first writer here instead.
            return self.work
        self.work.mkdir()
        shutil.copytree(self.root / 'meta', self.work / 'meta')
        return self.work

    def discard(self):
        if self.journal.exists():
            raise RuntimeError('unresolved publication; restart for journal recovery')
        if self.work.exists():
            shutil.rmtree(self.work)

    @staticmethod
    def _files(root, groups):
        result = []
        for group in groups:
            if (root / group).is_symlink():
                raise RuntimeError(f'symlink not allowed in staged data: {root / group}')
            for path in sorted((root / group).rglob('*')):
                if path.is_symlink():
                    raise RuntimeError(f'symlink not allowed in staged data: {path}')
                if path.is_file():
                    result.append(path.relative_to(root).as_posix())
        return result

    def commit(self, expected_episode):
        """Publish ONLY finalized output. Metadata is the visibility boundary."""
        before = json.loads((self.root / 'meta/info.json').read_text())
        after = json.loads((self.work / 'meta/info.json').read_text())
        if before['total_episodes'] != expected_episode or after['total_episodes'] != expected_episode + 1:
            raise RuntimeError('episode index changed during publication')
        files = self._files(self.work, ('data', 'videos', 'annotations'))
        if not any(path.startswith('data/') for path in files):
            raise RuntimeError('finalized episode has no data file')
        if any((self.root / path).exists() for path in files):
            raise RuntimeError('publication would overwrite saved data; refusing')
        for relative in files:
            parent = (self.root / relative).parent
            while parent != self.root:
                if parent.is_symlink():
                    raise RuntimeError(f'symlink not allowed in destination: {parent}')
                parent = parent.parent
        for relative in files + self._files(self.work, ('meta',)):
            with (self.work / relative).open('rb') as stream:
                os.fsync(stream.fileno())
        # A file fsync does not persist its directory entry. Seal the staging
        # tree before recording the journal that may need it during recovery.
        for directory in sorted((p for p in self.work.rglob('*') if p.is_dir()),
                                key=lambda p: len(p.parts), reverse=True):
            sync_dir(directory)
        sync_dir(self.work)
        journal = {'version': 1, 'phase': 'prepared', 'files': files}
        durable_json(self.journal, journal)
        try:
            for relative in files:
                destination = self.root / relative
                destination.parent.mkdir(parents=True, exist_ok=True)
                # link() refuses collisions; source remains for crash recovery.
                os.link(self.work / relative, destination)
                parent = destination.parent
                while parent != self.root:
                    sync_dir(parent)
                    parent = parent.parent
            os.replace(self.root / 'meta', self.work / 'previous-meta')
            os.replace(self.work / 'meta', self.root / 'meta')
            sync_dir(self.work)
            sync_dir(self.root)
            durable_json(self.journal, {**journal, 'phase': 'committed'})
        except Exception:
            # The last rename may have succeeded before its directory fsync
            # failed. The visible committed marker is authoritative: never
            # report this saved episode as invalid or encourage a duplicate.
            if json.loads(self.journal.read_text()).get('phase') != 'committed':
                self._recover_publication()
                raise
            logging.exception('Episode committed, but final journal sync failed; inspect storage health')
        # Cleanup failure is not a failed save and must never replay the commit.
        try:
            self._recover_publication()
        except OSError:
            pass  # The committed journal can be cleaned on the next restart.

    def _recover_publication(self):
        journal = json.loads(self.journal.read_text())
        if journal.get('version') != 1 or journal.get('phase') not in ('prepared', 'committed'):
            raise RuntimeError('unrecognized publication journal; manual inspection required')
        files = journal['files']
        for name in files:
            path = Path(name)
            if path.is_absolute() or '..' in path.parts or not path.parts or path.parts[0] not in ('data', 'videos', 'annotations'):
                raise RuntimeError('unsafe publication journal path')
        if journal['phase'] == 'prepared':
            previous = self.work / 'previous-meta'
            if previous.exists():
                if (self.root / 'meta').exists():
                    # Move the new metadata back to the failed workspace.
                    os.replace(self.root / 'meta', self.work / 'failed-meta')
                os.replace(previous, self.root / 'meta')
                sync_dir(self.root)
            for name in files:
                path, staged = self.root / name, self.work / name
                if path.exists():
                    if not staged.exists() or not os.path.samefile(path, staged):
                        raise RuntimeError(f'publication file identity changed: {name}')
                    path.unlink()
                    sync_dir(path.parent)
        self.journal.unlink()
        sync_dir(self.root)
        if journal['phase'] == 'committed':
            self.discard()

    def close(self):
        self._lock.close()
