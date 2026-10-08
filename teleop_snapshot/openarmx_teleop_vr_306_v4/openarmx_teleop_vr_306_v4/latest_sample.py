"""Single-slot mailbox for latency-sensitive teleoperation targets."""

import threading


class LatestSampleMailbox:
    """Keep exactly one pending sample and invalidate in-flight old work.

    ``put`` always replaces the pending sample.  A consumer may already be
    processing an older sequence; ``is_latest`` lets it discard that result
    instead of applying motion that has since been superseded.
    """

    def __init__(self):
        self._condition = threading.Condition()
        self._sequence = 0
        self._pending = None
        self._closed = False
        self.replaced_pending = 0
        self.discarded_inflight = 0

    def put(self, value):
        with self._condition:
            if self._closed:
                return None
            self._sequence += 1
            if self._pending is not None:
                self.replaced_pending += 1
            self._pending = (self._sequence, value)
            self._condition.notify()
            return self._sequence

    def take(self):
        with self._condition:
            while self._pending is None and not self._closed:
                self._condition.wait()
            if self._pending is None:
                return None
            sample = self._pending
            self._pending = None
            return sample

    def is_latest(self, sequence):
        with self._condition:
            return not self._closed and int(sequence) == self._sequence

    def discard_inflight(self):
        with self._condition:
            self.discarded_inflight += 1

    def reset(self):
        """Invalidate all work from the previous teleoperation session.

        Incrementing the sequence is important: an IK solve already running
        outside the controller lock must become stale even when no newer pose
        has arrived yet.  The worker thread remains alive and reusable.
        """
        with self._condition:
            if self._closed:
                return None
            self._sequence += 1
            self._pending = None
            self._condition.notify_all()
            return self._sequence

    def close(self):
        with self._condition:
            self._closed = True
            self._pending = None
            self._condition.notify_all()

    def stats(self):
        with self._condition:
            return {
                'latest_sequence': self._sequence,
                'pending': self._pending is not None,
                'replaced_pending': self.replaced_pending,
                'discarded_inflight': self.discarded_inflight,
            }
