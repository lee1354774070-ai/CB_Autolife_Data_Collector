"""Serve a small UI extension without editing the colleague's web files."""

import os
import asyncio
import json
import time
import threading
from pathlib import Path

from aiohttp import web
from openarmx_teleop_vr_306_v4 import vr_web_bridge as upstream
from vr_feedback import PATTERNS


def main():
    original = upstream.VrWebBridge
    source = Path(os.environ["DAGGER_DEPENDENCY_ROOT"]) / "autolife_hg_dagger_MZJ_300"

    class CollectorWebBridge(original):
        def __init__(self):
            super().__init__()
            from std_msgs.msg import String
            self._feedback_lock = threading.Lock()
            self._feedback_clients = set()
            mode = os.environ.get("COLLECTOR_WEB_MODE", "dagger")
            if mode not in ("dagger", "vr", "subtask"):
                raise ValueError("Invalid COLLECTOR_WEB_MODE")
            self._index_body = Path(__file__).with_name("web_index.html").read_text(encoding="utf-8")
            self._app_body = (source / "web/vr_app.js").read_text(encoding="utf-8")
            if mode == "dagger":
                self._app_body += "\n" + Path(__file__).with_name("ui_overrides.js").read_text(encoding="utf-8")
            self._app_body += "\nconst collectorWebMode = " + json.dumps(mode) + ";\n"
            self._app_body += Path(__file__).with_name("feedback.js").read_text(encoding="utf-8")
            self.create_subscription(String, "/collector/feedback", self._on_feedback, 10)

        def _on_feedback(self, message):
            try:
                event = json.loads(message.data)
                if (not isinstance(event, dict) or not isinstance(event.get("id"), str)
                        or event.get("event") not in PATTERNS):
                    return
            except (TypeError, ValueError):
                return
            with self._feedback_lock:
                clients = tuple(self._feedback_clients)
            for loop, queue in clients:
                try:
                    loop.call_soon_threadsafe(self._queue_feedback, queue, event)
                except RuntimeError:
                    pass  # HTTP loop closed during shutdown.

        @staticmethod
        def _queue_feedback(queue, event):
            if queue.full():
                queue.get_nowait()
            queue.put_nowait((time.monotonic(), event))

        async def _feedback_stream(self, request):
            """One bounded SSE connection per page; no idle 10 Hz HTTP polling."""
            client = (asyncio.get_running_loop(), asyncio.Queue(maxsize=32))
            with self._feedback_lock:
                if len(self._feedback_clients) >= 4:
                    raise web.HTTPServiceUnavailable(text="Too many feedback clients")
                self._feedback_clients.add(client)
            response = web.StreamResponse(headers={"Content-Type": "text/event-stream",
                                                   "Cache-Control": "no-store",
                                                   "X-Accel-Buffering": "no"})
            try:
                await response.prepare(request)
                await response.write(b": connected\n\n")
                while True:
                    try:
                        stamp, event = await asyncio.wait_for(client[1].get(), timeout=15)
                        if time.monotonic() - stamp >= 2:
                            continue
                        payload = ("data: " + json.dumps(event) + "\n\n").encode()
                    except asyncio.TimeoutError:
                        payload = b": keepalive\n\n"
                    await asyncio.wait_for(response.write(payload), timeout=2)
            except (ConnectionError, asyncio.TimeoutError):
                pass
            finally:
                with self._feedback_lock:
                    self._feedback_clients.discard(client)
            return response

        async def _index(self, request):
            return web.Response(
                text=self._index_body,
                content_type="text/html", headers={"Cache-Control": "no-store"})

        async def _asset(self, request):
            if request.match_info["asset"] == "collector_events":
                return await self._feedback_stream(request)
            if request.match_info["asset"] == "vr_app.js":
                return web.Response(text=self._app_body, content_type="application/javascript",
                                    headers={"Cache-Control": "no-store"})
            return await super()._asset(request)

    lookup = upstream.get_package_share_directory
    upstream.get_package_share_directory = lambda name: (
        str(source) if name == "openarmx_teleop_vr_306_v4" else lookup(name))
    upstream.VrWebBridge = CollectorWebBridge
    upstream.main()
