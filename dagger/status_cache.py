"""Read NAS-backed recorder status outside ROS control locks.

Only the worker accesses files. Readers exchange immutable snapshots under a
short lock. A stalled filesystem cannot stall takeover; it makes this cache
stale, which the supervisor treats as a reason to hold the robot.
"""

import threading
import time


class CachedCollector:
    def __init__(self, collector, interval=.05):
        self.collector = collector
        self.interval = interval
        self._lock = threading.Lock()
        self._selector = (False, "", 0)
        self._snapshot = (self._selector, {}, {}, 0.)
        self._stopped = threading.Event()
        self._worker = threading.Thread(target=self._run, name="dagger-status", daemon=True)
        self._worker.start()

    def __getattr__(self, name):
        return getattr(self.collector, name)

    def select(self, depth, trial, since_ns):
        with self._lock:
            self._selector = (depth, trial, since_ns)

    def _read(self):
        with self._lock:
            key, progress, invalid, completed = self._snapshot
            if key != self._selector:
                return {}, {}, float("inf")
            return progress, invalid, time.monotonic() - completed

    def progress(self, depth, trial):
        progress, _, _ = self._read()
        return progress if progress.get("trial_id") == trial else {}

    def invalid_episode_event(self, depth, since_wall_ns):
        _, invalid, _ = self._read()
        return invalid

    @property
    def age(self):
        return self._read()[2]

    def _run(self):
        while not self._stopped.is_set():
            with self._lock:
                selector = self._selector
            depth, trial, since = selector
            try:
                progress = self.collector.progress(depth, trial)
                invalid = self.collector.invalid_episode_event(depth, since)
            except Exception:
                # Read failures must not manufacture a healthy heartbeat.
                pass
            else:
                with self._lock:
                    self._snapshot = (selector, progress, invalid, time.monotonic())
            self._stopped.wait(self.interval)

    def close(self):
        self._stopped.set()
        self._worker.join(timeout=.2)
