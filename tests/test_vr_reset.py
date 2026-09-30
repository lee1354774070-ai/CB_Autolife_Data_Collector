"""Guarded reset protocol without ROS, service calls, or hardware."""
import fcntl
import json
import sys
import tempfile
import threading
import types
import unittest
from concurrent.futures import Future
from pathlib import Path
from unittest.mock import patch

from vr_reset import GuardedReset


class ResetTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'motion.lock'
        service = types.SimpleNamespace(Request=lambda **kw: types.SimpleNamespace(**kw))
        modules = {'std_msgs.msg': types.SimpleNamespace(String=object),
                   'std_srvs.srv': types.SimpleNamespace(SetBool=service, Trigger=service)}
        self.patch = patch.dict(sys.modules, modules)
        self.patch.start()
        self.addCleanup(self.patch.stop)
        self.calls = []
        self.available = True
        self.pending_reset = False
        owner = self

        class Client:
            def __init__(self, name):
                self.name = name

            def wait_for_service(self, **kwargs):
                return owner.available

            def call_async(self, request):
                owner.calls.append((self.name, getattr(request, 'data', None)))
                if self.name.endswith('full_body_reset'):
                    owner.status(hardware_enable_pending=True)
                    owner.status(hardware_enable_pending=False, hardware_ready=True, hardware_enabled=True)
                future = Future()
                future.set_result(types.SimpleNamespace(success=True, message='ok'))
                return future

        node = types.SimpleNamespace(create_client=lambda kind, name: Client(name),
                                     create_subscription=lambda *args: None)
        self.reset = GuardedReset(node, '/test', lambda: True, threading.Event(), self.path)
        self.addCleanup(lambda: self.reset.motion_handle and self.reset.motion_handle.close())
        self.status(hardware_enabled=False)

    def status(self, **values):
        self.reset.on_status(types.SimpleNamespace(data=json.dumps(values)))

    def test_disable_reset_confirm_disable_order(self):
        self.assertTrue(self.reset()['success'])
        self.assertEqual(self.calls, [('/test/set_hardware_enabled', False),
                                     ('/test/full_body_reset', None),
                                     ('/test/set_hardware_enabled', False)])
        self.assertIsNone(self.reset.motion_handle)

    def test_recording_lock_blocks_all_services(self):
        with self.path.open('a+') as handle:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with self.assertRaisesRegex(RuntimeError, 'active'):
                self.reset()
        self.assertFalse(self.calls)

    def test_service_unavailable_never_falls_back_and_latches(self):
        self.available = False
        with self.assertRaisesRegex(RuntimeError, 'unavailable'):
            self.reset()
        with self.assertRaisesRegex(RuntimeError, 'unresolved'):
            self.reset()
        self.assertFalse(self.calls)

    def test_stale_status_blocks_reset(self):
        self.reset.received = 0
        with self.assertRaisesRegex(RuntimeError, 'fresh'):
            self.reset()
        self.assertFalse(self.calls)

    def test_cancel_before_grip_release_blocks_reset(self):
        self.reset.released = lambda: False
        self.reset.cancel.set()
        with self.assertRaisesRegex(RuntimeError, 'Release'):
            self.reset()
        self.assertFalse(self.calls)
