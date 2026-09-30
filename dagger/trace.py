"""Single-owner diagnostic writer; control callbacks never wait for storage."""

from queue import Empty, Full, Queue
import threading


class AsyncTrace:
    def __init__(self, writer, capacity=2048):
        self.writer = writer
        self.queue = Queue(maxsize=capacity)
        self.dropped = 0
        self.error = ""
        self.stopped = threading.Event()
        self.worker = threading.Thread(target=self._run, name="dagger-trace", daemon=True)
        self.worker.start()

    def append_ring(self, record):
        self.writer.append_ring(record)  # Bounded in-memory history only.

    def _enqueue(self, method, *args, diagnostic=False):
        if self.stopped.is_set():
            if diagnostic:
                self.dropped += 1
                return
            raise RuntimeError("diagnostic writer already stopped")
        try:
            self.queue.put_nowait((method, args))
        except Full:
            if not diagnostic:
                raise RuntimeError("diagnostic lifecycle queue full; refusing a new trial")
            self.dropped += 1

    def write(self, stream, record):
        self._enqueue("write", stream, record, diagnostic=True)

    def prepare_root(self):
        self._enqueue("prepare_root")

    def start(self, *args):
        self._enqueue("start", *args)

    def finish(self, status, frame_count, reason, collector_result=None):
        result = dict(collector_result or {})
        result.update(diagnostic_trace_dropped=self.dropped, diagnostic_trace_error=self.error)
        self._enqueue("finish", status, frame_count, reason, result)

    def barrier(self, timeout=2.0):
        """Only the lifecycle worker may wait here, never a control callback."""
        done = threading.Event()
        self._enqueue("barrier", done)
        if not done.wait(timeout):
            raise TimeoutError("diagnostic storage is stalled; robot remains held")
        if self.error:
            raise RuntimeError(f"diagnostic storage failed: {self.error}")

    def _run(self):
        while not self.stopped.is_set() or not self.queue.empty():
            try:
                method, args = self.queue.get(timeout=.1)
            except Empty:
                continue
            try:
                if method == "barrier":
                    args[0].set()
                else:
                    getattr(self.writer, method)(*args)
            except Exception as exc:
                self.error = str(exc)
            finally:
                self.queue.task_done()

    def close(self):
        self.stopped.set()
        self.worker.join(timeout=2)
