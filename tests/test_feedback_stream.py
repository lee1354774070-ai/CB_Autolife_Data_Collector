"""Actual SSE transport with an inert upstream stub; no ROS or robot services."""
import ast
import asyncio
import json
from pathlib import Path
import threading
import time
import unittest

try:
    from aiohttp import ClientSession, web
    from aiohttp.test_utils import TestServer
except ImportError:
    web = None

from vr_feedback import PATTERNS, feedback_packet

SOURCE = Path(__file__).resolve().parents[1] / 'dagger/web_bridge.py'
tree = ast.parse(SOURCE.read_text())
cls = next(item for function in tree.body if isinstance(function, ast.FunctionDef)
           for item in function.body if isinstance(item, ast.ClassDef))
namespace = dict(original=object, web=web, asyncio=asyncio, json=json, time=time,
                 PATTERNS=PATTERNS)
exec(compile(ast.Module(body=[cls], type_ignores=[]), str(SOURCE), 'exec'), namespace)


@unittest.skipIf(web is None, 'aiohttp is only required in the VR web environment')
class FeedbackStreamTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.node = object.__new__(namespace['CollectorWebBridge'])
        self.node._feedback_lock = threading.Lock()
        self.node._feedback_clients = set()
        self.node._index_body = '<html>test</html>'
        self.node._app_body = 'test'
        app = web.Application()
        app.router.add_get('/{asset:.*}', self.node._asset)
        self.server = TestServer(app, shutdown_timeout=.01)
        await self.server.start_server()
        self.client = ClientSession()

    async def asyncTearDown(self):
        await self.client.close()
        await self.server.close()

    async def connect(self):
        response = await self.client.get(self.server.make_url('/collector_events'))
        self.assertEqual(response.status, 200)
        self.assertEqual(await response.content.readline(), b': connected\n')
        await response.content.readline()
        return response

    async def test_cross_thread_event_delivery_and_bounded_queue(self):
        response = await self.connect()
        event = feedback_packet('save')
        thread = threading.Thread(target=self.node._on_feedback,
                                  args=(type('Message', (), {'data': json.dumps(event)})(),))
        thread.start()
        thread.join()
        line = await asyncio.wait_for(response.content.readline(), 1)
        self.assertEqual(json.loads(line.decode()[6:]), event)
        queue = asyncio.Queue(maxsize=32)
        for index in range(100):
            self.node._queue_feedback(queue, index)
        self.assertEqual(queue.qsize(), 32)
        self.assertEqual(queue.get_nowait()[1], 68)
        response.close()

    async def test_no_replay_on_connect_and_client_cap(self):
        self.node._on_feedback(type('Message', (), {'data': json.dumps(feedback_packet('save'))})())
        responses = [await self.connect() for _ in range(4)]
        self.assertTrue(all(queue.empty() for _, queue in self.node._feedback_clients))
        response = await self.client.get(self.server.make_url('/collector_events'))
        self.assertEqual(response.status, 503)
        for item in responses:
            item.close()

    async def test_expired_events_are_not_replayed(self):
        response = await self.connect()
        queue = next(iter(self.node._feedback_clients))[1]
        queue.put_nowait((time.monotonic() - 3, feedback_packet('discard')))
        event = feedback_packet('start')
        self.node._queue_feedback(queue, event)
        line = await asyncio.wait_for(response.content.readline(), 1)
        self.assertEqual(json.loads(line.decode()[6:])['event'], 'start')
        response.close()
