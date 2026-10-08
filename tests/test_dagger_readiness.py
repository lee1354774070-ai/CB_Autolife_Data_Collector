"""Exercise actual start/readiness bodies with no ROS or motor interfaces."""
import ast
import math
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
import uuid

SOURCE = Path(__file__).resolve().parents[1] / 'dagger/supervisor.py'
tree = ast.parse(SOURCE.read_text())
methods = [method for cls in tree.body if isinstance(cls, ast.ClassDef)
           for method in cls.body if isinstance(method, ast.FunctionDef)
           and method.name in {'_start_trial', '_wait_for_controller_ready'}]


class ReadinessTest(unittest.TestCase):
    def test_no_extra_target_to_measurement_cutoffs(self):
        defaults = {node.args[0].value: node.args[1].value for node in ast.walk(tree)
                    if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                    and node.func.attr == 'declare_parameter' and len(node.args) == 2
                    and all(isinstance(arg, ast.Constant) for arg in node.args)}
        self.assertNotIn('policy_max_body_step_deg', defaults)
        self.assertNotIn('policy_max_arm_step_deg', defaults)

    def setUp(self):
        self.now = 1.0
        self.on_sleep = lambda: None
        clock = SimpleNamespace(monotonic=lambda: self.now,
                                monotonic_ns=lambda: int(self.now * 1e9), sleep=self.sleep)
        self.namespace = {'time': clock, 'uuid': uuid}
        exec(compile(ast.Module(body=methods, type_ignores=[]), str(SOURCE), 'exec'), self.namespace)
        self.node = Mock()
        self.node._lock = threading.RLock()
        self.node._cancel_start = self.node._closing = False
        self.node._controller_status = {'state': 'ENABLING', 'hardware_enable_pending': True}
        self.node._controller_status_received_ns = 1_000_000_000
        self.node.get_parameter.return_value = SimpleNamespace(value=.15)

    def sleep(self, seconds):
        self.now += seconds
        self.on_sleep()

    def ready(self):
        self.node._controller_status = {'state': 'ARMED', 'hardware_enabled': True,
                                        'hardware_ready': True, 'hardware_enable_pending': False}
        self.node._controller_status_received_ns = int(self.now * 1e9)

    def wait(self):
        return self.namespace['_wait_for_controller_ready'](self.node, 990_000_000)

    def test_pending_ack_does_not_authorize_policy(self):
        self.assertFalse(self.wait()[0])

    def test_requires_new_fresh_ready_status(self):
        self.ready()
        self.node._controller_status_received_ns = 900_000_000
        self.on_sleep = self.ready
        self.assertTrue(self.wait()[0])
        self.assertGreater(self.now, 1.0)

    def test_fault_and_cancel_interrupt_wait(self):
        self.node._controller_status = {'state': 'FAULT', 'reason': 'drive failure'}
        self.assertEqual(self.wait(), (False, 'drive failure'))
        self.node._cancel_start = True
        self.assertIn('cancelled', self.wait()[1])

    def start(self, enabled):
        # Run the pinned DAgger start body: recording must precede policy authority.
        from dagger.control.core import Mode
        import time as real_time
        native = SOURCE.parent / 'control/supervisor_node.py'
        body = next(n for c in ast.parse(native.read_text()).body if isinstance(c, ast.ClassDef)
                    for n in c.body if isinstance(n, ast.FunctionDef) and n.name == '_on_set_session_enabled')
        body.returns = None
        for arg in body.args.args:
            arg.annotation = None
        namespace = dict(Mode=Mode, uuid=uuid, time=real_time)
        exec(compile(ast.Module(body=[body], type_ignores=[]), str(native), 'exec'), namespace)
        node = self.node
        node._collection_exited = node._session_start_pending = False
        node._intervention_id = ''
        node._machine.mode = Mode.DISARMED
        node._prepare_intervention_locked.return_value = ('trial', True)
        node._start_collector_for_trial.return_value = (True, 'recorder ready')
        def enable(value):
            node._start_collector_for_trial.assert_called_once()
            node._machine.enable.assert_not_called()
            return enabled, 'ready' if enabled else 'readiness timeout'
        node._forward_enable.side_effect = enable
        response = SimpleNamespace()
        namespace['_on_set_session_enabled'](node, SimpleNamespace(data=True), response)
        return response

    def test_failed_readiness_disables_hardware_without_policy_enable(self):
        self.assertFalse(self.start(False).success)
        self.assertEqual([c.args for c in self.node._forward_enable.call_args_list], [(True,), (False,)])
        self.node._machine.enable.assert_not_called()
        self.assertFalse(self.node._session_start_pending)

    def test_policy_enable_follows_physical_readiness(self):
        self.assertTrue(self.start(True).success)
        self.node._machine.enable.assert_called_once()


class ForwardEnableTest(unittest.TestCase):
    def test_pending_enable_is_not_readiness(self):
        class Base:
            def _forward_enable(self, enabled):
                self.calls.append(enabled)
                return True, 'accepted'
        method = next(n for c in tree.body if isinstance(c, ast.ClassDef)
                      for n in c.body if isinstance(n, ast.FunctionDef) and n.name == '_forward_enable')
        cls = ast.ClassDef(name='Adapter', bases=[ast.Name(id='Base', ctx=ast.Load())],
                           keywords=[], body=[method], decorator_list=[])
        ns = dict(Base=Base, time=SimpleNamespace(monotonic_ns=lambda: 123))
        exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])), str(SOURCE), 'exec'), ns)
        node = ns['Adapter']()
        node.calls = []
        node._closing = node._cancel_start = False
        node._wait_for_controller_ready = Mock(return_value=(False, 'timeout'))
        self.assertEqual(node._forward_enable(True), (False, 'timeout'))
        self.assertEqual(node.calls, [True, False])
        node._wait_for_controller_ready.assert_called_once_with(123)


class PolicyBoundaryTest(unittest.TestCase):
    """Run the real adapter callback; never construct ROS or hardware objects."""

    def setUp(self):
        callback = next(method for cls in tree.body if isinstance(cls, ast.ClassDef)
                        for method in cls.body if isinstance(method, ast.FunctionDef)
                        and method.name == '_on_policy_action')

        class Base:
            def _on_policy_action(self, message):
                self.forwarded.append(message)

        def finite_vector(values, size):
            if not isinstance(values, list) or len(values) != size:
                return None
            return values if all(math.isfinite(value) for value in values) else None

        adapter = ast.ClassDef(name='Adapter', bases=[ast.Name(id='Base', ctx=ast.Load())],
                               keywords=[], body=[callback], decorator_list=[])
        module = ast.fix_missing_locations(ast.Module(body=[adapter], type_ignores=[]))
        namespace = {'Base': Base, 'finite_vector': finite_vector}
        exec(compile(module, str(SOURCE), 'exec'), namespace)
        self.node = namespace['Adapter']()
        self.node._lock = threading.RLock()
        self.node._cancel_start = self.node._closing = False
        self.node._machine = SimpleNamespace(mode=SimpleNamespace(value='POLICY_ACTIVE'), authority_epoch=1)
        self.node._session_id = 'active'
        self.node._reject_policy = Mock()
        self.node._begin_failure_hold = Mock()
        self.node._publish_state = Mock()
        self.node.forwarded = []
        self.payload = dict(action=[20.] * 14 + [10., 330.] + [20.] * 5,
                            measured_policy_state=[0.] * 14 + [10., 330.] + [0.] * 5,
                            collector_session_id='active', authority_epoch=1)
        self.node._parse = Mock(return_value=self.payload)

    def test_tracking_error_is_delegated_to_guarded_controller(self):
        self.node._on_policy_action('message')
        self.assertEqual(self.node.forwarded, ['message'])
        self.node._begin_failure_hold.assert_not_called()
        self.node._reject_policy.assert_not_called()

    def test_thor_closed_360_is_forwarded_unchanged(self):
        self.payload['action'][14:16] = [360., 360.]
        self.node._on_policy_action('original-message')
        self.assertEqual(self.node.forwarded, ['original-message'])
        self.assertEqual(self.payload['action'][14:16], [360., 360.])
        self.node._reject_policy.assert_not_called()

    def test_invalid_vectors_are_not_forwarded(self):
        for key in ('action', 'measured_policy_state'):
            valid = self.payload[key]
            for invalid in (valid[:-1], [float('nan')] * 21, [float('inf')] * 21):
                with self.subTest(key=key, invalid=invalid):
                    self.payload[key] = invalid
                    self.node._on_policy_action('message')
                    self.assertEqual(self.node.forwarded, [])
                    self.node._reject_policy.assert_called_with(self.payload, 'invalid action/state')
            self.payload[key] = valid

    def test_out_of_range_grippers_still_hold(self):
        for index, value in ((14, 9.), (15, 361.)):
            valid = self.payload['action'][index]
            self.payload['action'][index] = value
            self.node._on_policy_action('message')
            self.assertEqual(self.node.forwarded, [])
            self.node._begin_failure_hold.assert_called()
            self.node._reject_policy.assert_called()
            self.payload['action'][index] = valid

    def test_old_session_and_epoch_remain_rejected(self):
        for key, invalid in (('collector_session_id', 'old'), ('authority_epoch', 0)):
            valid = self.payload[key]
            self.payload[key] = invalid
            self.node._on_policy_action('message')
            self.assertEqual(self.node.forwarded, [])
            self.node._reject_policy.assert_called_with(self.payload, 'authority revoked')
            self.payload[key] = valid

    def test_cancel_or_exit_never_forwards(self):
        for key in ('_cancel_start', '_closing'):
            setattr(self.node, key, True)
            self.node._on_policy_action('message')
            self.assertEqual(self.node.forwarded, [])
            setattr(self.node, key, False)

    def test_takeover_or_stop_never_forwards(self):
        for mode in ('FAILURE_HOLD', 'EXPERT_READY', 'EXPERT_ACTIVE', 'DISARMED', 'ESTOP'):
            self.node._machine.mode.value = mode
            self.node._on_policy_action('message')
            self.assertEqual(self.node.forwarded, [])
