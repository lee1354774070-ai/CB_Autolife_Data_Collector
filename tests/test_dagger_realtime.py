"""Bounded queues and cached storage reads; no ROS or robot required."""

import threading
import time
import unittest
from unittest.mock import Mock

from dagger.status_cache import CachedCollector
from dagger.trace import AsyncTrace


class RealtimeIsolationTest(unittest.TestCase):
    def test_slow_storage_never_runs_on_status_reader(self):
        blocked, release = threading.Event(), threading.Event()
        source = Mock()
        def slow(*args):
            blocked.set()
            release.wait(2)
            return {"trial_id": "test", "frames": 1}
        source.progress.side_effect = slow
        source.invalid_episode_event.return_value = {}
        cache = CachedCollector(source, interval=.005)
        try:
            self.assertTrue(blocked.wait(1))
            cache.select(False, "test", 1)
            before = time.monotonic()
            for _ in range(1000):
                self.assertEqual(cache.progress(False, "test"), {})
                self.assertEqual(cache.invalid_episode_event(False, 1), {})
            self.assertLess(time.monotonic() - before, .1)
            self.assertGreater(cache.age, 1)
        finally:
            release.set()
            cache.close()

    def test_trace_lifecycle_never_waits_inside_control_callback(self):
        blocked, release = threading.Event(), threading.Event()
        writer = Mock()
        def slow(*args):
            blocked.set()
            release.wait(2)
        writer.start.side_effect = slow
        trace = AsyncTrace(writer, capacity=4)
        try:
            trace.start("trial")
            self.assertTrue(blocked.wait(1))
            before = time.monotonic()
            for _ in range(100):
                trace.write("events", {})
            self.assertLess(time.monotonic() - before, .05)
            self.assertLessEqual(trace.queue.qsize(), 4)
            self.assertGreater(trace.dropped, 0)
            with self.assertRaises(RuntimeError):
                trace.start("must-not-start")
            self.assertEqual(writer.start.call_count, 1)
        finally:
            release.set()
            trace.close()
