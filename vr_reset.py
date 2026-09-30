"""Worker-only adapter to the inspected V4 guarded full-body reset service.

No direct joint targets or vendor reset topics are published. Service acceptance
is not completion: fresh controller feedback must show the enable/reset cycle
and then hardware readiness. Ambiguous RPCs are never retried automatically.
"""

import json
import fcntl
import threading
import time


class GuardedReset:
    def __init__(self, node, prefix, released, cancel, motion_lock=None, timeout=75):
        from std_msgs.msg import String
        from std_srvs.srv import SetBool, Trigger
        self.node, self.released, self.cancel = node, released, cancel
        self.timeout = timeout
        self.motion_lock = motion_lock
        self.motion_handle = None
        self.lock = threading.Lock()
        self.status, self.received = {}, 0.0
        self.active_seen = False
        self.watch_since = float("inf")
        self.enable = node.create_client(SetBool, prefix + "/set_hardware_enabled")
        self.reset = node.create_client(Trigger, prefix + "/full_body_reset")
        node.create_subscription(String, prefix + "/status", self.on_status, 10)

    def on_status(self, message):
        try:
            packet = json.loads(message.data)
            if not isinstance(packet, dict):
                return
        except (ValueError, TypeError):
            return
        with self.lock:
            self.status, self.received = packet, time.monotonic()
            if self.received >= self.watch_since and packet.get("hardware_enable_pending") is True:
                self.active_seen = True

    def rpc(self, client, request):
        if not client.wait_for_service(timeout_sec=1):
            raise RuntimeError("Guarded V4 reset service unavailable; no fallback reset is sent")
        if self.cancel.is_set():
            raise RuntimeError("Reset cancelled")
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        deadline = time.monotonic() + 5
        while not done.wait(.02):
            if self.cancel.is_set() or time.monotonic() >= deadline:
                raise TimeoutError("Reset RPC result unknown; do not retry")
        result = future.result()
        if result is None or not result.success:
            raise RuntimeError("Reset rejected: " + str(getattr(result, "message", "empty response")))

    def __call__(self):
        if not self.motion_lock:
            raise RuntimeError("Reset requires the same motion-lock file as the recorder")
        if self.motion_handle is not None:
            raise RuntimeError("Previous reset unresolved; inspect before restarting")
        handle = open(self.motion_lock, "a+")
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            handle.close()
            raise RuntimeError("Recording/motion is active; reset blocked") from None
        self.motion_handle = handle
        # Keep the lock latched on any uncertain outcome, including keyboard
        # start attempts, until the operator inspects and restarts this helper.
        result = self._reset_once()
        handle.close()
        self.motion_handle = None
        return result

    def _reset_once(self):
        from std_srvs.srv import SetBool, Trigger
        # The X chord requires GL+GR. Wait for release before allowing reset;
        # stale/disconnected VR is not interpreted as a deliberate release.
        deadline = time.monotonic() + 10
        while not self.released():
            if self.cancel.wait(.02) or time.monotonic() >= deadline:
                raise RuntimeError("Release both Grips with live VR input before reset")
        with self.lock:
            if time.monotonic() - self.received > 1:
                raise RuntimeError("No fresh V4 controller status; reset not sent")
        self.rpc(self.enable, SetBool.Request(data=False))
        with self.lock:
            self.watch_since, self.active_seen = time.monotonic(), False
        try:
            self.rpc(self.reset, Trigger.Request())
            deadline = time.monotonic() + self.timeout
            while not self.cancel.wait(.02):
                with self.lock:
                    status, received, active = dict(self.status), self.received, self.active_seen
                if time.monotonic() - received > 1:
                    raise RuntimeError("Controller feedback lost during reset")
                if status.get("emergency_stop_latched") or status.get("state") in ("FAULT", "ESTOP", "E_STOP"):
                    raise RuntimeError("Controller fault during reset")
                if (active and status.get("hardware_ready") is True
                        and status.get("hardware_enabled") is True
                        and status.get("hardware_enable_pending") is False):
                    self.rpc(self.enable, SetBool.Request(data=False))
                    return {"success": True, "event": "reset"}
                if time.monotonic() >= deadline:
                    raise TimeoutError("Reset completion unconfirmed; do not retry")
            raise RuntimeError("Reset cancelled")
        except Exception:
            # Best-effort disable is not a retry of the motion request.
            self.enable.call_async(SetBool.Request(data=False))
            raise
